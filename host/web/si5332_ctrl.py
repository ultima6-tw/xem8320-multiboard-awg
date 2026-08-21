"""
si5332_ctrl.py — Diagnostics tab 的 Si5332 狀態查詢/重新設定（2026-08-01 新增）

Si5332 EVB 是透過獨立 USB 直接接在這台 Windows host 上的裝置（不經過
Opal Kelly/Aurora，跟三片 AWG 板子的通訊路徑完全獨立、不需要 app.py 的
`_lock`/`_fp`），全域只有一台，不分板子。沿用既有 `check_si5332_
status.py`/`si5332_configure.py`（CBPro CLI wrapper），用跟
`init_flow.py` Phase 0 完全一樣的方式（獨立 subprocess 呼叫
`sys.executable <script>`）不重寫底層邏輯——`si5332_configure.py`
docstring 記錄過 CBPro 工具有「同一個 process/session 連續操作會
lockup」的已知 bug，修法就是每次都用全新獨立 process，這裡不能改成
直接 import 呼叫節省一層 subprocess。

這兩支腳本原本只在 `post_flash_checklist.py`/`init_flow.py` Phase 0
裡自動被呼叫過，這裡是第一次獨立暴露成使用者能單獨觸發的動作。

⚠️ **Reconfigure 會重寫 Si5332 的 RAM-based 時脈設定**——這是目前三片
板子共用的外部 DAC 時脈來源，如果板子正在運作中，重設當下時脈會短暫
消失/跳動。前端呼叫 `/api/si5332/reconfigure` 前必須先 `confirm()`
明確警告，不能無聲執行。

## 6 路輸出開關（2026-07-31，同一天延續，使用者釐清真正需求後新增）

使用者其實要的是「開關 Si5332 的 6 個輸出、設定共用頻率」，不是只有
ACTIVE/reconfigure。直接讀了一次運作中的裝置（`CBProDeviceRead --all`，
唯讀不影響硬體）確認：
  - 6 個輸出目前**已經共用同一條合成路徑**（`OMUX0~5_SEL` 全部是 0，
    `HSDIV0A_DIV=24`，對照 NOTES.md 2026-07-13「OUT0~OUT5 六路全部
    100MHz、LVDS、共用同一個除頻器」的紀錄一致）——跟使用者說的
    「頻率共通就好」天生吻合，不需要另外設計共用機制
  - 每路獨立的 output-enable bit 是 `OUT{0..5}_OE`（`0xB6`/`0xB7`），
    現況 `OUT0/1/2_OE=1`（對應目前 3 片接線中的板子）、
    `OUT3/4/5_OE=0`（備用，尚未接板子）——跟 NOTES.md 2026-07-13
    「OUT3/4/5 三路輸出版本」的紀錄完全一致
  - **頻率這次維持固定 100MHz，不開放任意輸入**：使用者確認只需要
    100MHz（跟現有 FPGA 端 Clocking Wizard 鎖定 100MHz 輸入的設計
    一致），這次只做開關，頻率設定留白（顯示為唯讀資訊，不做輸入框）

**寫入方式**：`CBProDeviceWrite --settings <file>`（`OUTx_OE,0/1` 一行），
這跟 `si5332_configure.py` docstring 記錄的 lockup bug **是不同性質
的操作**——lockup bug 描述的是「一次性把 Ready→Config→Active 三個
階段接續完成」這個特定轉換序列的問題，這裡只是修改一顆已經在 ACTIVE
狀態下運作的裝置的單一 bitfield，不涉及那個狀態轉換序列。CBPro CLI
本身也會在每次 settings/registers 寫入後自動讀回驗證（見
`CBProDeviceWrite --help` VALIDATION 章節），不用額外自己再讀回確認。
"""
import sys
import os
import re
import subprocess
import tempfile
import json

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from post_flash_checklist import SI5332_PROJECT, SI5332_CONFIGURE_PY
from si5332_configure import CBPRO_READ, CBPRO_WRITE

IS_LINUX = sys.platform.startswith("linux")
if IS_LINUX:
    # 2026-08-20：read_outputs()/set_output_enable() 移植成 Linux pyusb 版
    # （Windows 分支完全不動），用既有、已在 init_flow.py Phase 0 上機
    # 驗證過的 Si5332USB 直接讀寫暫存器，不用另外裝 CBPro CLI。
    from si5332_usb_linux import Si5332USB, Si5332USBError

CHECK_STATUS_PY = os.path.join(os.path.dirname(SI5332_CONFIGURE_PY), "check_si5332_status.py")

NUM_OUTPUTS = 6
OE_SETTING_NAMES = [f"OUT{i}_OE" for i in range(NUM_OUTPUTS)]
OUTPUT_FREQ_HZ = 100_000_000  # 固定值，這次不開放任意輸入，見上方模組說明

# OUTx_OE 的暫存器位址/bit 對應（0xB6/0xB7 兩個 byte 暫存器），來源：
# NOTES.md 2026-07-13「OUT0/1/2 三路輸出版本」+ 2026-07-31「暫存器研究」
# 兩次獨立量測互相印證（0xB6=0x0B/0xB7=0x00 對應 OUT0/1/2=on、OUT3/4/5=off，
# 跟這裡的 bit 拆解算出來的結果一致），另外 si5332_usb_linux.py 的
# DISCRIMINATING 字典也記錄了同一顆 0xB6。**這次沒有重新上機逐一核對**，
# 只是套用既有記錄，read_outputs() 讀出來的值務必先跟已知現況比對過再信任。
OE_BIT_MAP = {
    0: (0xB6, 0),
    1: (0xB6, 1),
    2: (0xB6, 3),
    3: (0xB6, 6),
    4: (0xB7, 1),
    5: (0xB7, 2),
}

# 2026-07-31：手動切過的 OUTx_OE 是 volatile 的（見 set_output_enable()
# docstring），reconfigure（si5332_configure.py，也是 init_flow.py Phase 0
# 每次「Initialize System」都會跑的那步）會把它蓋回 project 檔案裡寫死的
# 值——這個檔案讓 Initialize System 之後能重新套用「上次手動設定」，而不是
# 每次都被 project 預設值蓋掉。檔案不存在＝從沒手動調過，維持 project 預設。
OUTPUT_STATE_FILE = os.path.join(os.path.dirname(__file__), "si5332_output_state.json")


def read_status():
    """呼叫 check_si5332_status.py 讀 USYS_STAT，回傳
    {"ok", "active", "status", "value", "raw", "error"}。
    `ok`=True 只代表讀取本身成功（裝置有回應、輸出格式看得懂）；
    `active`=True 才代表真的是 ACTIVE、正常輸出時脈——兩者是分開的，
    `ok=True, active=False` 是合法狀態（讀到了，但顯示 READY/UNINIT）。
    """
    result = subprocess.run(
        [sys.executable, CHECK_STATUS_PY],
        capture_output=True, text=True)
    stdout = result.stdout.strip()
    m = re.search(r"USYS_STAT = 0x([0-9A-Fa-f]+) \((\w+)\)", stdout)
    if not m:
        return {"ok": False, "active": False, "status": None, "value": None,
                "raw": stdout, "error": result.stderr.strip() or "無法解析 check_si5332_status.py 輸出格式"}
    value = int(m.group(1), 16)
    name = m.group(2)
    return {"ok": True, "active": name == "ACTIVE", "status": name,
            "value": value, "raw": stdout, "error": None}


def reconfigure():
    """呼叫 si5332_configure.py --project SI5332_PROJECT（跟
    `init_flow.py` Phase 0 用同一組常數/呼叫方式），回傳
    {"ok", "raw", "error"}。"""
    result = subprocess.run(
        [sys.executable, SI5332_CONFIGURE_PY, "--project", SI5332_PROJECT],
        capture_output=True, text=True)
    return {
        "ok": result.returncode == 0,
        "raw": result.stdout.strip(),
        "error": None if result.returncode == 0 else result.stderr.strip(),
    }


def read_outputs():
    """讀 OUT0_OE..OUT5_OE，回傳 {"ok","channels":{0:bool,...,5:bool},
    "freq_hz","error"}。`channels` 用 int key（JSON 化後 Flask
    `jsonify` 會自動轉成字串 key，前端已知道要用字串索引）。

    2026-08-20：Linux 走 Si5332USB 直接讀 0xB6/0xB7 兩個暫存器、照
    OE_BIT_MAP 拆解成 6 個 bool，不再依賴 Windows-only 的 CBPRO_READ。"""
    if IS_LINUX:
        try:
            with Si5332USB() as si:
                reg_cache = {}
                channels = {}
                for ch, (addr, bit) in OE_BIT_MAP.items():
                    if addr not in reg_cache:
                        reg_cache[addr] = si.read_register(addr)
                    channels[ch] = bool((reg_cache[addr] >> bit) & 1)
        except Si5332USBError as e:
            return {"ok": False, "channels": None, "freq_hz": None,
                    "error": f"Si5332USB read failed: {e}"}
        return {"ok": True, "channels": channels, "freq_hz": OUTPUT_FREQ_HZ, "error": None}

    # 2026-08-18：CBPRO_READ 是 Windows-only 的 ClockBuilder Pro CLI，
    # 這台機器上不存在時 subprocess.run() 會直接丟 FileNotFoundError，
    # 沒有被接住——但呼叫端 apply_saved_output_state()/init_flow.py 的
    # 設計本來就是「這一步 non-fatal，失敗就跳過」（見 apply_saved_
    # output_state() 檔頭說明），所以這裡補上 try/except，讓它照原本
    # 設計優雅地回傳 ok=False，不要讓整個 Initialize System 流程中斷。
    try:
        result = subprocess.run(
            [CBPRO_READ, "--quiet", "--format", "csv", "--settings"] + OE_SETTING_NAMES,
            capture_output=True, text=True)
    except OSError as e:
        return {"ok": False, "channels": None, "freq_hz": None,
                "error": f"CBPRO_READ not available on this platform: {e}"}
    if result.returncode != 0:
        return {"ok": False, "channels": None, "freq_hz": None,
                "error": result.stderr.strip() or result.stdout.strip()}

    channels = {}
    for line in result.stdout.strip().splitlines():
        if line.startswith("Location") or not line.strip():
            continue
        parts = [p.strip() for p in line.split(",")]
        # Location,Type,SettingName,DecimalValue,HexValue
        m = re.match(r"OUT(\d)_OE$", parts[2])
        if m:
            channels[int(m.group(1))] = bool(int(parts[3]))

    if len(channels) != NUM_OUTPUTS:
        return {"ok": False, "channels": None, "freq_hz": None,
                "error": f"expected {NUM_OUTPUTS} OUTx_OE settings, parsed {len(channels)} — output format may have changed"}
    return {"ok": True, "channels": channels, "freq_hz": OUTPUT_FREQ_HZ, "error": None}


def set_output_enable(channel, enabled):
    """寫單一 OUTx_OE（volatile，跟 RAM 內其他設定無關，重開機/重新
    reconfigure 才會回到 project 檔案裡的預設值）。這是修改一顆已經在
    ACTIVE 狀態運作中裝置的單一 bitfield，不是 si5332_configure.py
    docstring 記錄的 Ready→Config→Active 那個特定轉換序列，不會觸發
    同一個 lockup bug（見模組開頭說明）。"""
    if not (0 <= channel < NUM_OUTPUTS):
        raise ValueError(f"channel must be 0-{NUM_OUTPUTS - 1}, got {channel}")

    if IS_LINUX:
        addr, bit = OE_BIT_MAP[channel]
        try:
            with Si5332USB() as si:
                before = si.read_register(addr)
                after = (before | (1 << bit)) if enabled else (before & ~(1 << bit) & 0xFF)
                si.write_register(addr, after)
        except Si5332USBError as e:
            return {"ok": False, "raw": None, "error": f"Si5332USB write failed: {e}"}
        return {"ok": True,
                "raw": f"OUT{channel}_OE={1 if enabled else 0}: 0x{addr:02X} 0x{before:02X} -> 0x{after:02X}",
                "error": None}

    fd, path = tempfile.mkstemp(suffix=".txt", prefix="si5332_oe_")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(f"OUT{channel}_OE,{1 if enabled else 0}\n")
        result = subprocess.run(
            [CBPRO_WRITE, "--settings", path],
            capture_output=True, text=True)
        return {
            "ok": result.returncode == 0,
            "raw": result.stdout.strip(),
            "error": None if result.returncode == 0 else result.stderr.strip(),
        }
    finally:
        os.unlink(path)


def save_output_state(channels):
    """把目前 6 路 OUTx_OE 狀態存進 OUTPUT_STATE_FILE，呼叫端在每次
    set_output_enable() 成功、重讀完整 6 路之後呼叫。`channels` 可以是
    int key 或 str key 的 dict（read_outputs() 回傳的是 int key，經過
    Flask jsonify 的則是 str key），這裡統一存成 str key 的 JSON。"""
    with open(OUTPUT_STATE_FILE, "w", encoding="utf-8") as f:
        json.dump({str(k): bool(v) for k, v in channels.items()}, f)


def apply_saved_output_state():
    """Initialize System 的 Phase 0（si5332_configure.py 重寫/略過）跑完
    之後呼叫：把 OUTPUT_STATE_FILE 記錄的上次手動設定重新套用回去，蓋掉
    reconfigure 剛寫入的 project 預設值。檔案不存在（從沒手動調過任何
    channel）就直接跳過，不動任何東西。

    ⚠️ 這裡跟網頁上的手動切換不同，不會有 confirm() 警告——如果存檔內容
    是某個 channel=OFF，這裡會直接靜默把它設回 OFF，即使目前有板子正靠
    這個輸出取得外部時脈。這是「記住上次設定」這個功能本身的效果，是
    刻意的設計，不是遺漏。"""
    if not os.path.exists(OUTPUT_STATE_FILE):
        return {"ok": True, "applied": None, "note": "no saved output state yet, kept project defaults"}

    try:
        with open(OUTPUT_STATE_FILE, "r", encoding="utf-8") as f:
            saved = {int(k): bool(v) for k, v in json.load(f).items()}
    except (json.JSONDecodeError, ValueError, OSError) as e:
        return {"ok": False, "applied": None, "error": f"failed to read {OUTPUT_STATE_FILE}: {e}"}

    current = read_outputs()
    if not current["ok"]:
        return {"ok": False, "applied": None, "error": f"could not read current output state: {current['error']}"}

    applied = []
    for ch, desired in saved.items():
        if not (0 <= ch < NUM_OUTPUTS):
            continue
        if current["channels"].get(ch) == desired:
            continue
        result = set_output_enable(ch, desired)
        if not result["ok"]:
            return {"ok": False, "applied": applied, "error": f"failed to set OUT{ch}_OE: {result['error']}"}
        applied.append({"channel": ch, "enabled": desired})

    return {"ok": True, "applied": applied}
