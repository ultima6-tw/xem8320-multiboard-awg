"""
calib_ctrl.py — 校準面板（scale_cfg/amp_ctrl/calib_coef）用的 host 邏輯
（2026-07-31 新增）

跟 board_ctrl.py 分開一支檔案，因為關注點不同：board_ctrl.py 是「播放
控制」（波形/trigger/播放狀態），這裡是「類比輸出校準」（scale/gain/
offset），兩者底層封包/暫存器完全獨立，合併在一起只會讓兩份邏輯互相
干擾理解。一樣全部走 master 這條 USB + dest_id 定址，不開該板自己的
USB。

三種校準值（`amp_ctrl` 目前沒有接在 web UI 上，見下方說明，函式/
endpoint 保留）：
  - scale_cfg（`T_SCALE_CFG`=0x07）：8-bit，每個 bit 對應一個實體輸出
    通道（4 塊 Zmod x 2 channel）的量程檔位——**使用者確認**：bit=0
    是 ±1.25V，bit=1 是 ±5V。見 `rtl/awg_calib_regs.v`（`ZmodAWG
    Controller` 的 `sExtCh{1,2}Scale`，官方 IP 內部真正做二選一，這個
    模組只是把兩組係數都算好、都輸出，實際生效哪一組的判斷邏輯藏在
    官方 IP 黑盒子裡，這個 repo 看不到，只能上機實測，見下面
    calib_coef 說明裡的實測記錄）。
  - amp_ctrl（`T_AMP_CTRL`=0x09）：8 個 channel 各自的增益係數，18-bit
    定點格式（1.0 = 0x10000），channel index 跟 scale_cfg bit 同一種
    「zmod*2+ch」編號。**2026-07-31 使用者決定從 web UI 移除**（跟
    calib_coef 的 Mult 係數是相乘關係，兩個都能調的「增益」控制容易
    搞混），這裡的函式跟 `app.py` 的 endpoint 都保留，只是沒有 UI
    呼叫。
  - calib_coef（`T_CALIB_WR`=0x02/`T_CALIB_RST`=0x03）：32 筆更底層的
    Mult/Add 校準係數（18-bit），index 語意已對照 `rtl/awg_calib_
    regs.v` 逐行核對過（不是猜的）：
      bit4 = 0 Mult / 1 Add
      bit3 = 0 Hg / 1 Lg（**這是 RTL 自己的命名，直接 1:1 接線到官方
             IP 對應同名的 pin**：`create_bd.tcl:4437-4446`
             `z${inst}_ch${ch}_hg_mult` → `cExt{Ch1,Ch2}HgMultCoef`，
             中間沒有任何反相/邏輯，是硬體事實不是猜的）
      bit2 = 0 ch1 / 1 ch2
      bit[1:0] = zmod index（0-3）
    reset 預設值：Mult 全部 0x10000（=1.0），Add 全部 0x00000（見
    `awg_calib_regs.v:148` `coef[i] <= i[4] ? 18'h00000 : 18'h10000`）。
    `T_CALIB_RST` 只重置 calib_coef 這個陣列回預設值，**不影響**
    scale_cfg/amp_ctrl（那兩個是完全獨立的暫存器/mux）。

    **`calib_sel` bit3 跟 `scale_cfg` bit 的配對關係（2026-07-31 上機
    實測確認，之前猜錯過兩次）**：兩者是**相反**的，不是同一種 0/1
    編碼——`scale_cfg` bit=1（±5V）時，實際生效的是 `calib_sel
    bit3=0`（也就是 RTL/官方 IP 自己命名的「Hg」）這組係數，不是
    `bit3=1`。**實測方法**：`scale_cfg` 確認是 `0xFF`（全部 ±5V，
    示波器上量到 5V）的狀態下，把 `calib_coef[0x00]`（z0 ch1 的
    Hg Mult）設成 0，board 0 channel 0 的示波器輸出立刻變成 0V；
    再設成 `0x8000`（0.5）輸出振幅正確變成一半——確認 `bit3=0`
    （Hg）就是 `scale=1`（±5V）時真正在用的那組。**結論**：Hg=
    ±5V、Lg=±1.25V（跟「增益越高、範圍越大」的直覺一致，RTL 自己的
    Hg/Lg 命名本身沒有錯），錯的是我原本假設「`calib_sel` bit3 跟
    `scale_cfg` bit 用同一種 0/1 編碼」這件事——兩者其實是 XOR
    關係：`active = (calib_sel bit3) != (scale_cfg bit)`。

讀回統一用既有的 `T_QUERY(QT_CALIB_STATUS)`（`awg_common.py` 已有
`decode_qt_calib_status`，回傳 `scale_cfg`/`amp_ctrl`[8]/`calib_coef`[32]）。

⚠️ calib_coef 直接影響即時類比輸出校準，這個專案歷史上曾經因為測試
腳本寫壞 Add 係數導致待機非 0V（見 test_calib_wr_rst_remote.py 檔頭
背景說明）。跟 trig_delay 不同，這裡改壞了可以用 `T_CALIB_RST`
救回來（不是 sticky/不可逆），所以沒有做成 confirm() 彈窗，但 web UI
上要清楚標示風險＋顯眼放「Reset to defaults」按鈕。
"""
import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from awg_common import (
    send_pkt, send_query, read_status_reply,
    T_SCALE_CFG, T_AMP_CTRL, T_CALIB_WR, T_CALIB_RST,
    QT_CALIB_STATUS, decode_qt_calib_status,
    TI_CMD, TI_BIT_FLUSH_STANDBY,
)

AMP_CTRL_BROADCAST_SEL = 0x8  # bit3=1：T_AMP_CTRL 廣播全部 8 個 channel


def _flush_standby(fp):
    fp.ActivateTriggerIn(TI_CMD, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)


def read_calib(fp, dest_id, wait=0.05):
    """T_QUERY(QT_CALIB_STATUS)，回傳 {"scale_cfg":int, "amp_ctrl":[8 ints],
    "calib_coef":[32 ints]}。query_type 不符時回傳 None。"""
    _flush_standby(fp)
    send_query(fp, dest_id, QT_CALIB_STATUS)
    time.sleep(wait)
    _, got_query_type, data_words = read_status_reply(fp)
    if got_query_type != QT_CALIB_STATUS:
        return None
    return decode_qt_calib_status(data_words)


def channel_full_scale_v(scale_cfg, channel):
    """回傳某個實體 channel（0-7，「zmod*2+ch」編號，見上方模組說明）
    目前 scale_cfg 檔位對應的滿幅電壓（V）——bit=0 是 ±1.25V，bit=1 是
    ±5V（使用者確認，2026-07-31 上機實測記錄見上方模組說明）。2026-08-20
    新增，給 test1 面板的電壓→振幅比例換算用。"""
    return 5.0 if (scale_cfg >> channel) & 1 else 1.25


def set_scale_cfg(fp, dest_id, value):
    """T_SCALE_CFG(0x07)：8-bit，每個 bit 對應一個通道的量程檔位。"""
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_SCALE_CFG, beat1_lo=value & 0xFF)


def set_amp_ctrl(fp, dest_id, channel, value):
    """T_AMP_CTRL(0x09)：channel 0-7 設單一通道，channel=None 廣播全部
    8 個（bit3=1）。value 是 18-bit 定點格式（1.0=0x10000）。"""
    ch_sel = AMP_CTRL_BROADCAST_SEL if channel is None else (channel & 0x7)
    val = (value & 0x3FFFF) | ((ch_sel & 0xF) << 18)
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_AMP_CTRL, beat1_lo=val)


def set_calib_coef(fp, dest_id, index, value):
    """T_CALIB_WR(0x02)：index 0-31（見檔頭 bit 排列說明），value 18-bit。"""
    val = (index & 0x1F) | ((value & 0x3FFFF) << 5)
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_CALIB_WR, beat1_lo=val)


def reset_calib_coef(fp, dest_id):
    """T_CALIB_RST(0x03)：只重置 calib_coef 陣列回硬體預設值（Mult=
    0x10000/Add=0x00000），不影響 scale_cfg/amp_ctrl。"""
    _flush_standby(fp)
    send_pkt(fp, dest_id, T_CALIB_RST, beat1_lo=0)


def set_amp_ctrl_bulk(fp, dest_id, values):
    """values: {channel(int): value(int)}，依序對每個 channel 各送一次
    T_AMP_CTRL——協定本身沒有「一次寫多個 channel」的封包，這裡只是把
    迴圈搬到 host 端一次做完，讓 UI 只需要一個 Set 按鈕，不用每個
    channel 各按一次。"""
    for channel, value in values.items():
        set_amp_ctrl(fp, dest_id, channel, value)


def set_calib_coef_bulk(fp, dest_id, values):
    """values: {index(int): value(int)}，依序對每個 index 各送一次
    T_CALIB_WR，理由同 set_amp_ctrl_bulk。"""
    for index, value in values.items():
        set_calib_coef(fp, dest_id, index, value)
