"""
flash_ctrl.py — Calibration tab「Save to Flash」用的 host 邏輯（2026-07-31 新增）

跟 calib_ctrl.py 分開一支檔案：calib_ctrl.py 管的是「臨時寫入」（Set/
Set All，只寫 live 暫存器，reboot 就消失），這裡管的是「存入 flash」
（reload/開機後自動套用），兩者是使用者明確要求分開的兩層功能，底層
封包完全不同（T_SCALE_CFG/T_CALIB_WR vs T_FLASH_TARGET_SEL/T_FLASH_
WRITE_DATA），合併只會讓人搞不清楚「這個 Set 到底存不存得住」。

**設計對齊記錄（2026-07-31，跟使用者來回確認過三輪）**：
  - Live 跟 Flash 是兩個分開顯示的欄位，**不強求即時同步**——Flash
    欄位反映的是「上次成功存檔/開機當下」的內容，跟目前 Live 值可能
    不一樣，這是預期行為不是 bug。
  - 「Save to Flash」按鈕的語意：把**目前 Live 的值**（已經生效、
    使用者已經在畫面上看到的值）存進 flash，不是輸入框裡還沒送出的
    草稿值。呼叫端（app.py）負責先用 calib_ctrl.read_calib() 讀目前
    live 值，再傳進這裡的 save_*_to_flash()。
  - Flash 欄位資料來源：`fpga_flash_ctrl_0` 內部 `flash_startup_
    loader` 開機時已經把 flash 內容解析進 `init_scale_cfg`/`init_
    coef_0..31`，這次只是把這份既有資料再接出去給查詢用，沒有新增
    「存檔時同步更新快取」的邏輯——存檔跟這份快取更新是分開的動作，
    存檔後 Flash 欄位要靠重新查詢（read_flash_calib()）才會更新，
    不是自動即時反映。

封包格式跟遠端可寫性已經**上機驗證過**（2026-07-15，
`test_flash_target_sel_remote.py`/`test_amp_calib_flash.py`，這裡的
`pack18()`/`_flash_payload()` 直接沿用同一套組包邏輯，不是重新設計）：
  - T_FLASH_TARGET_SEL(0x15)：2-beat，beat1[1:0]=sector
    （0=identity/1=scale_cfg/2=amp_ctrl/3=calib_coef）
  - T_FLASH_WRITE_DATA(0x16)：33-beat（header+256 bytes payload），
    offset 0x00=FLASH_MAGIC，offset 0x04 起依序塞值

⚠️ **這個模組依賴的 `QT_FLASH_STATUS` 查詢是規格先行**：Linux 端
2026-07-31 先做完「只有 busy/done/err/loader_done 狀態」的簡單版，
使用者確認後再擴充成含 `scale_cfg`/`calib_coef` 實際內容的版本（見
NOTES.md「擴充：QT_FLASH_STATUS 加上 flash 實際內容」），這支檔案
是照擴充版規格先寫的，**在擴充版 build 傳回來、`awg_common.py` 的
`decode_qt_flash_status()` 補上 scale_cfg/calib_coef 欄位之前，
`read_flash_calib()`/`poll_flash_done()` 沒辦法真的在硬體上跑**（可以
語法檢查，不能上機驗證）。
"""
import sys
import os
import time
import struct

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from awg_common import (
    send_pkt, send_query, read_status_reply,
    T_FLASH_TARGET_SEL, T_FLASH_WRITE_DATA,
    QT_FLASH_STATUS, decode_qt_flash_status,
    TI_CMD, TI_BIT_FLUSH_STANDBY,
)

FLASH_MAGIC = 0x41574739
FLASH_RECORD_SZ = 256

TARGET_IDENTITY   = 0
TARGET_SCALE_CFG  = 1
TARGET_AMP_CTRL   = 2
TARGET_CALIB_COEF = 3


def _flush_standby(fp):
    fp.ActivateTriggerIn(TI_CMD, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)


def pack18(val):
    """flash_startup_loader.v unpack18 的反函數：3 bytes LE，第 3 byte
    只用 bits[1:0]。跟 test_amp_calib_flash.py 的 pack18() 完全一致。"""
    val &= 0x3FFFF
    return bytes([val & 0xFF, (val >> 8) & 0xFF, (val >> 16) & 0x3])


def _payload_to_beats(payload_bytes):
    """256 bytes -> 32 個 (lo, hi) tuple，給 send_pkt(extra_beats=...) 用。"""
    assert len(payload_bytes) == FLASH_RECORD_SZ
    beats = []
    for off in range(0, FLASH_RECORD_SZ, 8):
        lo, hi = struct.unpack_from('<II', payload_bytes, off)
        beats.append((lo, hi))
    return beats


def scale_cfg_payload(value):
    buf = bytearray(FLASH_RECORD_SZ)
    struct.pack_into('<I', buf, 0x00, FLASH_MAGIC)
    buf[0x04] = value & 0xFF
    return bytes(buf)


def calib_coef_payload(values):
    """values: 32 個 18-bit 值（跟 index 0-31 順序一致）。"""
    assert len(values) == 32
    buf = bytearray(FLASH_RECORD_SZ)
    struct.pack_into('<I', buf, 0x00, FLASH_MAGIC)
    off = 0x04
    for v in values:
        buf[off:off + 3] = pack18(v)
        off += 3
    return bytes(buf)


def set_flash_target_sel(fp, dest_id, target):
    """T_FLASH_TARGET_SEL(0x15)：接下來的 T_FLASH_WRITE_DATA 要存哪個
    sector。中間可能間隔好幾拍，這個暫存器是 sticky 的，不是 pulse。"""
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_FLASH_TARGET_SEL, beat1_lo=target & 0x3)


def write_flash_payload(fp, dest_id, payload_bytes):
    """T_FLASH_WRITE_DATA(0x16)：送 256 bytes payload（33-beat 封包，
    呼叫前要先 set_flash_target_sel() 選好 sector）。"""
    beats = _payload_to_beats(payload_bytes)
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_FLASH_WRITE_DATA,
              beat1_lo=beats[0][0], beat1_hi=beats[0][1],
              extra_beats=beats[1:])


def read_flash_calib(fp, dest_id, wait=0.05):
    """T_QUERY(QT_FLASH_STATUS)，回傳 {"busy","done","err","loader_done",
    "scale_cfg","calib_coef":[32]}（擴充版規格，見檔頭說明）。query_type
    不符時回傳 None。"""
    _flush_standby(fp)
    send_query(fp, dest_id, QT_FLASH_STATUS)
    time.sleep(wait)
    _, got_query_type, data_words = read_status_reply(fp)
    if got_query_type != QT_FLASH_STATUS:
        return None
    return decode_qt_flash_status(data_words)


def poll_flash_done(fp, dest_id, timeout=10.0, poll_interval=0.1):
    """輪詢 QT_FLASH_STATUS 到 done 或 err，逾時/出錯直接 raise（呼叫端
    的 Flask endpoint 用 try/except 轉成 JSON error response）。回傳最後
    一次讀到的完整狀態（含 scale_cfg/calib_coef，讓呼叫端可以直接拿來
    更新 Flash 欄位，不用另外再查一次）。"""
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        last = read_flash_calib(fp, dest_id)
        if last is not None:
            if last["err"]:
                raise RuntimeError(f"flash save error (dest_id={dest_id}, status={last})")
            if last["done"]:
                return last
        time.sleep(poll_interval)
    raise TimeoutError(f"flash save timed out after {timeout}s (dest_id={dest_id}, last_status={last})")


def save_scale_cfg_to_flash(fp, dest_id, live_value, timeout=10.0):
    """把目前 live 的 scale_cfg 值存進 flash sector 1。live_value 由
    呼叫端先用 calib_ctrl.read_calib() 讀出來傳進來（Save to Flash 存的
    是「目前已生效的值」，不是輸入框裡的草稿）。回傳存檔完成後的
    QT_FLASH_STATUS（含最新 flash 內容）。"""
    set_flash_target_sel(fp, dest_id, TARGET_SCALE_CFG)
    write_flash_payload(fp, dest_id, scale_cfg_payload(live_value))
    return poll_flash_done(fp, dest_id, timeout=timeout)


def save_calib_coef_to_flash(fp, dest_id, live_values, timeout=10.0):
    """把目前 live 的 32 筆 calib_coef 存進 flash sector 3。live_values
    由呼叫端先用 calib_ctrl.read_calib() 讀出來傳進來。回傳存檔完成後的
    QT_FLASH_STATUS（含最新 flash 內容）。"""
    set_flash_target_sel(fp, dest_id, TARGET_CALIB_COEF)
    write_flash_payload(fp, dest_id, calib_coef_payload(live_values))
    return poll_flash_done(fp, dest_id, timeout=timeout)
