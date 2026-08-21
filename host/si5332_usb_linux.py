#!/usr/bin/env python3
"""
si5332_usb_linux.py -- Si5332-6EX-EVB 低階 USB 協定（pyusb 直接實作）

從 Windows 上用 Wireshark + USBPcap 側錄 ClockBuilder Pro CLI
（CBProDeviceWrite.exe/CBProDeviceRead.exe）實際送出的 USB bulk transfer
封包逆向出來的協定，完整記錄見 NOTES.md「Si5332-6EX-EVB USB 協定側錄 +
逆向」章節（2026-07-28）。原始封包檔案：同目錄
`si5332_usb_capture_2026-07-28.pcap`（用 tshark 分析，見下方 2026-08-08
補充）。裝置：VID_10C4 PID_8B63，一般 USB bulk transfer
（不是 HID report），端點 0x02(OUT, host->device) / 0x82(IN, device->host)。

已驗證（信心~100%，62 筆逐一比對 CBPro 匯出的 CSV 全部吻合）：
  寫入 (6 bytes): 10 6A 01 01 <addr> <data>  -> 回應 (2 bytes) 01 01
  讀取 step1 (5 bytes): 10 6A 00 01 <addr>   -> 回應 (2 bytes) 01 00
  讀取 step2 (4 bytes): 11 6A 00 01          -> 回應 (3 bytes) 01 00 <data>
  USYS_STAT 就是普通暫存器位址 0x07，不是特殊指令。

2026-08-08（Pi 5 上機測試，第一版完全連不上，回頭重新用 tshark 分析
`aa.pcap` 才發現漏掉的一段）：**每一次 bulk OUT 之前，實際上都固定先送
兩次 USB control transfer（vendor request，不是 bulk），第一版逆向完全
沒有把這段記錄進來**——當時只看了 bulk endpoint 的封包內容，這兩個
control transfer 因為在不同的 endpoint（EP0）上，被漏看了。統計整份
封包（660 次 bulk 寫入 x 2 = 1320 次，加上 6 個 process/session 各自
開頭第一次的變體，總共 1326 次，比例 100% 一致，不是巧合）：
  bmRequestType=0x41 (Vendor, Host->Device, Interface)
  bRequest=2, wIndex=0, wLength=0（沒有 data stage，純通知）
  wValue=0x0002：只有整個 session（`with Si5332USB()`）的第一次 control
                 transfer 用這個值
  wValue=0x0001：其餘所有情況（包含每次 bulk OUT 之前的第二次 control
                 transfer）都是這個值
見 `_ctrl_ping()`。這是造成第一版在真實硬體上第一個 handshake byte
（0x80）送出後完全沒回應、timeout 的根本原因——不是 0x80/0x81/0x55/
0x83/0x47 那串 bulk 序列寫錯，是序列前面漏了這兩個 control transfer。

2026-08-08（同一次上機測試，補上 `_ctrl_ping()` 後 handshake 10 步驟全部
跟側錄逐位元組吻合，但第一次 `read_register(0x07)` 收到異常回應
`fa ff`，證實這段是必要步驟，不能省略）：`_handshake()` 後面、真正碰
`0x6A`（Si5332 本體 I2C 位址）暫存器之前，還有一大段（85 次 bulk OUT，
`aa.pcap`/`si5332_usb_capture_2026-07-28.pcap` frame 164~843）對 I2C
位址 `0x21`/`0x22`/`0x4C` 的操作，加上最後 5 次對位址 `0x63` 的大量讀取
（內容像是重複的 telemetry/校準表）。當時語意沒有拆解，整段原樣照抄重放。

2026-08-10（**重要修正，這段照抄是錯的**）：重新用 tshark 逐 session 對齊
比對後確認，那 85 步**不是固定序列，是對板上兩顆 I2C GPIO expander 的
「讀→改→寫」迴圈**：
  - `0x21`/`0x22` 是 PCA9555 類的 16-bit I/O expander。reg `0x06`/`0x07`
    是方向暫存器（寫 0x00 = 全部設成輸出，冪等），reg `0x02`/`0x03` 是
    output port 0/1。
  - 指令格式：讀 = `12 <slave> 00 00 <reg>` -> `01 00 <value>`
              寫 = `10 <slave> 01 00 <reg> <value>` -> `01 01`
  - CBPro 的真實邏輯是「把某個 port 的某一個 bit 清 0 後寫回」，值一律
    來自**當下讀到的值**，不是常數。開機後第一個 session 讀到 `0xFF`，
    所以會走出 ef/cf/cd/c9/49 這串遞減 pattern；**第 2 個 session 之後
    讀到的已經是穩定值 `A=0x49 B=0x49 C=0x24 D=0xFF`，同一串邏輯算出來
    的寫入值就等於原值，整段變成 no-op**（側錄 session 1~5 實際就是這樣，
    每一筆寫入都是把剛讀到的值原封不動寫回去）。
  - 舊版把 session 0 的字面值在**每個 session** 都重放一次，等於在已經
    穩定的板子上把約 16 支 expander 腳位重新拉高再走回低——每開一個
    session（含 Config 階段、Active 階段、每次讀回驗證）都做一次。這是
    「62 筆寫入全部回報成功、讀回卻是空白晶片狀態」最可能的根因。
  - 現在 `_i2c_wake()` 改成真正的 read-modify-write（`_AUX_SEQ`）。已用
    模擬器拿 pcap 的實際回應餵進去驗證：**同一份邏輯在 6 個 session 全部
    逐位元組重現側錄**（session 0 走完整走位、session 1~5 全部 no-op），
    所以這不是「裁剪」，是把原本抄錯的部分改對。`_I2C_WAKE_STEPS_HEX`
    保留為「已知的開機走位完整序列」參考常數，不再被執行。

注意 `01 01` / `01 00` 這個 ACK 只代表 **USB bridge 完成了 I2C transaction
（slave 有回 ACK）**，不代表 Si5332 內部真的把值收下。整份側錄從頭到尾
沒出現過非 `01` 開頭的回應，所以我們手上沒有「失敗長什麼樣」的樣本，
`write_register()` 成功不能當成設定生效的證據。
"""
import time
import usb.core
import usb.util

VID = 0x10C4
PID = 0x8B63
EP_OUT = 0x02
EP_IN = 0x82
TIMEOUT_MS = 2000

USYS_STAT_ADDR = 0x07
USYS_STAT_ACTIVE = 0x02
USYS_STAT_READY = 0x01

# DEVICE_REV（唯讀）。側錄裡每個 session 在碰任何設定暫存器之前都會先讀它，
# 回應固定 0x11。拿來當「DUT 這條 I2C 真的通了」的 per-session sanity check。
DEVICE_REV_ADDR = 0x0E
DEVICE_REV_EXPECTED = 0x11


class Si5332USBError(RuntimeError):
    pass


class Si5332USB:
    """一個 instance = 一次獨立的 USB session（open -> handshake -> 操作 -> close）。

    Windows 版 CBPro CLI 每次呼叫都是獨立 process/USB session；si5332_configure.py
    docstring 記錄的已知 I2C lockup bug要求 Ready/Config/Active 三個階段必須是
    三次完全獨立的 session，同一個 session 裡連續做完全部步驟會讓裝置卡死、
    RAM 設定被清空。這裡刻意不提供「開一次連線、跨多個 stage 共用」的介面，
    每次 with Si5332USB() 都是一次全新的 open+handshake。
    """

    def __init__(self, verbose=False):
        self.dev = None
        self.verbose = verbose
        self._first_ctrl = True
        self.device_rev = None
        # 這個 session 開始時，兩顆 GPIO expander 四個 output port 的實際讀值。
        # 診斷用：第一次上機時先看這個值，就能確認板子是「剛開機(0xFF)」還是
        # 「已穩定(49/49/24/FF)」，不需要寫入任何東西。見 _i2c_wake()。
        self.aux_ports_initial = None

    def open(self, probe_only=False):
        """probe_only=True：只做到「讀出 aux expander 四個 port 的現值」就停，
        不跑位元走位、不碰 Si5332 任何暫存器。純唯讀診斷用，見 probe_aux_ports()。
        """
        dev = usb.core.find(idVendor=VID, idProduct=PID)
        if dev is None:
            raise Si5332USBError(
                f"找不到裝置 VID={VID:04x} PID={PID:04x}"
                "（USB 線接好了嗎？61-si5332-evb.rules 這條 udev 規則裝了嗎？）"
            )
        try:
            if dev.is_kernel_driver_active(0):
                dev.detach_kernel_driver(0)
        except (NotImplementedError, usb.core.USBError):
            pass
        # 2026-08-08：側錄裡 SET_CONFIGURATION 整份只在裝置列舉時出現
        # 一次，6 個 session 一次都沒有重複呼叫。這裡改成只在裝置還沒
        # 設定過 configuration 時才呼叫，避免 Linux usbfs 對已設定過的
        # 裝置重複下 SETCONFIGURATION，可能連帶重置 host 端 data toggle
        # 但裝置韌體沒有同步重置，導致 OUT 被當重傳靜默丟棄。
        try:
            dev.get_active_configuration()
        except usb.core.USBError:
            dev.set_configuration()
        self.dev = dev
        self._first_ctrl = True
        self._handshake()
        self._i2c_wake(probe_only=probe_only)
        if probe_only:
            return self
        # 2026-08-10：側錄裡每個 session 在碰任何 0x6A 設定暫存器之前，一定
        # 先讀一次 DEVICE_REV(0x0E)（-> 0x11）。唯讀、不改變任何狀態，補回來
        # 跟側錄一致，順便當成 DUT I2C 通路的 per-session sanity check。
        self.device_rev = self.read_register(DEVICE_REV_ADDR)
        if self.device_rev != DEVICE_REV_EXPECTED:
            print(f"[!] DEVICE_REV(0x{DEVICE_REV_ADDR:02X}) 讀到 "
                  f"0x{self.device_rev:02X}，側錄裡是 0x{DEVICE_REV_EXPECTED:02X}"
                  "（不同 silicon rev 也可能是正常的，這裡只提示不中止）")
        elif self.verbose:
            print(f"  device_rev 0x{self.device_rev:02X} OK")
        return self

    def close(self):
        if self.dev is not None:
            # 2026-08-08（Opus 重新分析 pcap 找到的根因，信心~85%）：
            # 逐 session 精確比對後發現，側錄裡每個 session 最後一次
            # bulk IN 之後，還有一個我們完全沒送的收尾 control transfer
            # （跟 _ctrl_ping() 同格式，wValue=0x0001，後面沒有接
            # bulk OUT）。1326 = 660*2 + 6 這個之前對不齊的數字，其實是
            # 「6 個 session 各自的收尾」，不是「6 個 session 各自開頭的
            # 變體」。不送這個收尾，裝置會停在「transaction 還開著」的
            # 閂鎖狀態，跟 AN1360 描述的 CTS/HWERR「只有斷電才能清除」
            # 完全吻合——這也解釋了為什麼調 sleep 時間、release_interface
            # 都沒用：這兩個都跟這個閂鎖狀態無關。
            try:
                self.dev.ctrl_transfer(0x41, 2, 0x0001, 0, None, timeout=TIMEOUT_MS)
            except usb.core.USBError:
                pass
            try:
                usb.util.release_interface(self.dev, 0)
            except usb.core.USBError:
                pass
            usb.util.dispose_resources(self.dev)
            self.dev = None
            # 2026-08-08：連續開兩個 session（例如 already_configured()
            # 緊接著 run_stage()）幾乎沒有間隔時，下一個 session 的第一個
            # handshake byte 會 timeout。回頭查 aa.pcap 量到 CBPro CLI
            # 每次獨立 process 呼叫之間，最後一筆封包到下一個 session 開頭
            # 實際間隔 ~3 秒（6 次 session 起點間隔量到 5.5~7.3 秒，
            # 扣掉每個 session 自己執行的時間，尾端到下一個開頭的純間隔
            # 量到 ~2.98 秒），原本猜的 1 秒明顯不夠，改成貼近實測值。
            time.sleep(3.5)

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()

    # -- low level --

    def _ctrl_ping(self):
        """每次 bulk OUT 之前裝置要求的 vendor control transfer（見上方
        模組說明 2026-08-08 段落，`aa.pcap` 逆向出來）。"""
        w_value = 0x0002 if self._first_ctrl else 0x0001
        self._first_ctrl = False
        self.dev.ctrl_transfer(0x41, 2, w_value, 0, None, timeout=TIMEOUT_MS)

    def _send(self, data: bytes):
        self._ctrl_ping()
        self._ctrl_ping()
        self.dev.write(EP_OUT, data, timeout=TIMEOUT_MS)

    def _recv(self, length=64) -> bytes:
        resp = bytes(self.dev.read(EP_IN, length, timeout=TIMEOUT_MS))
        # 2026-08-08：官方 AN1360（Serial Communications and API
        # Programming Guide）記載這系列晶片有 CTS（Clear To Send）流量
        # 控制——裝置還在處理上一個指令（CTS=0）時如果送下一個指令，會
        # 觸發 HWERR，且「這個錯誤只有斷電重開才能清除」，跟這次上機
        # 測試反覆卡住、只能靠拔插恢復的現象完全吻合。這裡的 USB 協定是
        # bridge 晶片包了一層，沒有文件說明 CTS bit 在回應裡確切位置，
        # 沒辦法照文件做真正的 CTS 輪詢，退而求其次用固定延遲讓裝置有
        # 時間處理完，用意跟官方文件的 CTS 輪詢一樣（避免指令送太快），
        # 只是做法比較保守、沒那麼精確。
        time.sleep(0.03)
        return resp

    def _handshake(self):
        steps = [
            bytes([0x80]),
            bytes([0x81]),
            bytes([0x81]),
            bytes([0x55, 0x00]),
            bytes([0x83]),
            bytes([0x47, 0x02, 0x02, 0x00]),
            bytes([0x47, 0x02, 0x05, 0x00]),
            bytes([0x47, 0x02, 0x06, 0x00]),
            bytes([0x47, 0x02, 0x07, 0x00]),
            bytes([0x47, 0x03, 0x00, 0x00]),
        ]
        for s in steps:
            self._send(s)
            resp = self._recv()
            if self.verbose:
                print(f"  handshake {s.hex()} -> {resp.hex()}")

    # ---- 板上輔助晶片（GPIO expander）喚醒/設定 ----
    #
    # 見上方模組說明 2026-08-10 段落。這一段原本是把側錄 session 0 的 85 步
    # 字面重放，已確認是錯的（那是 read-modify-write 迴圈，值取決於讀到什麼）。
    # 現在改成用下面的 _AUX_* 定義重建真實邏輯，`_I2C_WAKE_STEPS_HEX` 僅保留
    # 為「開機後第一個 session 的完整走位序列」參考，不再被執行。

    # 兩顆 expander 的四個 output port。代號 A/B/C/D 只是這支程式內部用的。
    _AUX_PORTS = {
        "A": (0x21, 0x02),   # expander @0x21, output port 0
        "B": (0x21, 0x03),   # expander @0x21, output port 1
        "C": (0x22, 0x02),   # expander @0x22, output port 0
        "D": (0x22, 0x03),   # expander @0x22, output port 1
    }

    # 走位跑完後的穩定終態。側錄 session 1~5 每次一進來讀到的就是這組值，
    # 這時候整段走位算出來的寫入值 == 原值，全部是 no-op。
    _AUX_SETTLED = {"A": 0x49, "B": 0x49, "C": 0x24, "D": 0xFF}

    # 方向暫存器：兩顆 expander 的 port0/port1 全部設成輸出。本來就冪等，
    # 側錄每個 session 都照送，維持原樣。
    _AUX_DIR_STEPS_HEX = [
        "102101000600", "102101000700", "102201000600", "102201000700",
    ]

    # 位元走位序列（對應側錄 prefix 的第 18~85 步，共 68 步）：
    #   ("W", port, bit)  -> 把該 port 目前值的 bit 清成 0 之後寫回
    #   ("W", port, None) -> 目前值原封不動寫回（不清任何位元）
    #   ("R", port)       -> 讀回該 port，並用讀到的值重新同步快取
    # 注意寫入值一律由「當下的值」算出來，不是常數——這正是舊版抄錯的地方。
    # 已用模擬器拿 pcap 的實際回應驗證過：這張表在側錄 6 個 session 全部
    # 逐位元組重現原始封包（session 0 走完整走位，session 1~5 全部 no-op）。
    _AUX_SEQ = [
        ("W", "A", 4),    ("W", "A", 4),    ("W", "D", None), ("R", "A"),
        ("W", "A", 5),    ("R", "A"),       ("R", "B"),       ("R", "C"), ("R", "D"),
        ("W", "A", 1),    ("W", "A", 1),    ("W", "D", None), ("R", "A"),
        ("W", "A", 2),    ("R", "A"),       ("R", "B"),       ("R", "C"), ("R", "D"),
        ("W", "A", 7),    ("W", "A", 7),    ("W", "D", None), ("R", "B"),
        ("W", "B", 7),    ("R", "A"),       ("R", "B"),       ("R", "C"), ("R", "D"),
        ("W", "B", 5),    ("W", "B", 5),    ("W", "D", None), ("R", "B"),
        ("W", "B", 4),    ("R", "A"),       ("R", "B"),       ("R", "C"), ("R", "D"),
        ("W", "B", 2),    ("W", "B", 2),    ("W", "D", None), ("R", "B"),
        ("W", "B", 1),    ("R", "A"),       ("R", "B"),       ("R", "C"), ("R", "D"),
        ("W", "C", 0),    ("W", "B", 1),    ("W", "D", None), ("R", "C"),
        ("W", "C", 1),    ("R", "A"),       ("R", "B"),       ("R", "C"), ("R", "D"),
        ("W", "C", 3),    ("W", "C", 3),    ("W", "D", None), ("R", "C"),
        ("W", "C", 4),    ("R", "A"),       ("R", "B"),       ("R", "C"), ("R", "D"),
        ("W", "C", 6),    ("W", "C", 6),    ("W", "D", None), ("R", "C"),
        ("W", "C", 7),
    ]

    # 走位之後的收尾：對 0x4C/0x4D 的三筆操作 + 5 次 0x63 telemetry 讀取。
    # 這幾筆在側錄 6 個 session 完全相同（回應內容會變，像是 ADC 讀值），
    # 沒有 read-modify-write 的性質，照抄即可。
    _AUX_TAIL_STEPS_HEX = [
        "104c000000", "104d000000", "104c000040",
        "63040332", "63040350", "63040350", "63040350", "63040350", "63040350",
    ]

    # 參考用：開機後第一個 session（讀到 0xFF 時）實際走出來的完整 85 步。
    # 逐 byte 從 `si5332_usb_capture_2026-07-28.pcap` frame 164~843 匯出。
    # 這份常數現在只是文件，執行路徑不會用到它。
    _I2C_WAKE_STEPS_HEX = [
        "102101000600", "102101000700", "102201000600", "102201000700",
        "1221000002", "1221000003", "1222000002", "1222000003",
        "1021010002ef", "1021010002ef", "1022010003ff", "1221000002",
        "1021010002cf", "1221000002", "1221000003", "1222000002", "1222000003",
        "1021010002cd", "1021010002cd", "1022010003ff", "1221000002",
        "1021010002c9", "1221000002", "1221000003", "1222000002", "1222000003",
        "102101000249", "102101000249", "1022010003ff", "1221000003",
        "10210100037f", "1221000002", "1221000003", "1222000002", "1222000003",
        "10210100035f", "10210100035f", "1022010003ff", "1221000003",
        "10210100034f", "1221000002", "1221000003", "1222000002", "1222000003",
        "10210100034b", "10210100034b", "1022010003ff", "1221000003",
        "102101000349", "1221000002", "1221000003", "1222000002", "1222000003",
        "1022010002fe", "102101000349", "1022010003ff", "1222000002",
        "1022010002fc", "1221000002", "1221000003", "1222000002", "1222000003",
        "1022010002f4", "1022010002f4", "1022010003ff", "1222000002",
        "1022010002e4", "1221000002", "1221000003", "1222000002", "1222000003",
        "1022010002a4", "1022010002a4", "1022010003ff", "1222000002",
        "102201000224", "104c000000", "104d000000", "104c000040",
        "63040332", "63040350", "63040350", "63040350", "63040350", "63040350",
    ]

    def _aux_read_port(self, port: str) -> int:
        """讀一個 expander output port（`12 <slave> 00 00 <reg>` -> `01 00 <val>`）"""
        slave, reg = self._AUX_PORTS[port]
        self._send(bytes([0x12, slave, 0x00, 0x00, reg]))
        resp = self._recv(length=256)
        if len(resp) < 3 or resp[0] != 0x01:
            raise Si5332USBError(
                f"aux 讀取 {port}(0x{slave:02X} reg 0x{reg:02X}) 回應異常 {resp.hex()}")
        return resp[2]

    def _aux_write_port(self, port: str, value: int):
        """寫一個 expander output port（`10 <slave> 01 00 <reg> <val>` -> `01 01`）"""
        slave, reg = self._AUX_PORTS[port]
        self._send(bytes([0x10, slave, 0x01, 0x00, reg, value]))
        ack = self._recv(length=256)
        if len(ack) < 2 or ack[0] != 0x01:
            raise Si5332USBError(
                f"aux 寫入 {port}(0x{slave:02X} reg 0x{reg:02X})=0x{value:02X} "
                f"ACK 異常 {ack.hex()}")

    def _i2c_wake(self, probe_only=False):
        """板上兩顆 GPIO expander 的設定序列（真正的 read-modify-write 版本）。

        見模組說明 2026-08-10 段落。重點：**寫入值一律由當下讀到的值算出來**，
        所以板子已經在穩定狀態時，這整段自然會退化成「把讀到的值原封不動寫
        回去」的 no-op，不會像舊版那樣把腳位重新拉高再走一次位。
        """
        # 1) 方向暫存器（冪等，照側錄原樣送）
        for hex_str in self._AUX_DIR_STEPS_HEX:
            s = bytes.fromhex(hex_str)
            self._send(s)
            resp = self._recv(length=256)
            if self.verbose:
                print(f"  aux_dir  {s.hex()} -> {resp.hex()}")

        # 2) 讀四個 output port 的目前值，決定接下來要不要真的走位
        state = {p: self._aux_read_port(p) for p in ("A", "B", "C", "D")}
        self.aux_ports_initial = dict(state)
        if self.verbose or state != self._AUX_SETTLED:
            desc = " ".join(f"{p}=0x{state[p]:02X}" for p in ("A", "B", "C", "D"))
            if state == self._AUX_SETTLED:
                print(f"  aux ports {desc}（已是穩定值，走位序列會全部是 no-op）")
            elif all(v == 0xFF for v in state.values()):
                print(f"  aux ports {desc}（剛上電未設定，會跑完整的位元走位）")
            else:
                print(f"  [!] aux ports {desc}（既不是 0xFF 也不是穩定值 "
                      f"49/49/24/FF，走位仍會把指定位元清成 0 收斂到穩定值）")

        if probe_only:
            return

        # 3) 位元走位。已在穩定狀態時每一筆 W 算出來的就是原值 -> no-op，
        #    跟側錄 session 1~5 逐位元組一致；剛上電(0xFF)時會走出跟側錄
        #    session 0 完全相同的 ef/cf/cd/c9/49 ... 序列。
        for op in self._AUX_SEQ:
            if op[0] == "R":
                port = op[1]
                state[port] = self._aux_read_port(port)
                if self.verbose:
                    print(f"  aux_rd   {port} -> 0x{state[port]:02X}")
            else:
                _, port, bit = op
                value = state[port] if bit is None else (state[port] & ~(1 << bit)) & 0xFF
                self._aux_write_port(port, value)
                noop = " (no-op)" if value == state[port] else ""
                state[port] = value
                if self.verbose:
                    bit_desc = "writeback" if bit is None else f"clr bit{bit}"
                    print(f"  aux_wr   {port} {bit_desc} -> 0x{value:02X}{noop}")

        if state != self._AUX_SETTLED:
            desc = " ".join(f"{p}=0x{state[p]:02X}" for p in ("A", "B", "C", "D"))
            print(f"[!] aux 走位結束後的狀態 {desc} 跟預期的穩定值 "
                  "A=0x49 B=0x49 C=0x24 D=0xFF 不符，請回頭查 pcap 比對")

        # 4) 收尾：0x4C/0x4D 三筆 + 5 次 0x63 telemetry 讀取（照抄）
        for hex_str in self._AUX_TAIL_STEPS_HEX:
            s = bytes.fromhex(hex_str)
            self._send(s)
            resp = self._recv(length=256)
            if self.verbose:
                print(f"  aux_tail {s.hex()} -> {resp.hex()}")

    # -- register access --

    def read_register(self, addr: int) -> int:
        self._send(bytes([0x10, 0x6A, 0x00, 0x01, addr]))
        ack = self._recv()
        if len(ack) < 2 or ack[0] != 0x01 or ack[1] != 0x00:
            raise Si5332USBError(f"reg 0x{addr:02X}: 讀取 step1 ACK 異常 {ack.hex()}")
        self._send(bytes([0x11, 0x6A, 0x00, 0x01]))
        resp = self._recv()
        if len(resp) < 3:
            raise Si5332USBError(f"reg 0x{addr:02X}: 讀取 step2 回應長度不足 {resp.hex()}")
        return resp[2]

    def write_register(self, addr: int, data: int):
        self._send(bytes([0x10, 0x6A, 0x01, 0x01, addr, data]))
        ack = self._recv()
        if len(ack) < 2 or ack[0] != 0x01 or ack[1] != 0x01:
            raise Si5332USBError(f"reg 0x{addr:02X}=0x{data:02X}: 寫入 ACK 異常 {ack.hex()}")

    def read_usys_stat(self) -> int:
        return self.read_register(USYS_STAT_ADDR)

    def read_back(self, addrs) -> dict:
        return {addr: self.read_register(addr) for addr in addrs}


# 真正會隨設定改變的暫存器（值取自 si5332_3out_registers.csv 的 config 段）。
# 這些位址在**空白/設定遺失的晶片上讀回是 0x00**，所以它們是唯一能證明
# 「設定真的生效了」的位址。
DISCRIMINATING = {
    0x2B: 0x18,  # HSDIV0A_DIV
    0x67: 0x30,  # IDPA_INTG (低位元組)
    0x7A: 0x04,  # OUT0_MODE
    0x7B: 0x01,  # OUT0_DIV
    0xB6: 0x0B,  # OUT0~OUT5 OE（六路輸出開關，量不到時脈時第一個要看的）
    0xBA: 0x7E,
    0xBE: 0x10,  # PLL_MODE
    0xC0: 0x30,  # XOSC_CTRIM_XA（空白值是 0x04）
}

# 2026-08-10：**這些位址不能拿來當驗證依據**，它們在空白晶片上就會讀到
# 「正確」的值，是舊版 self_test() 一路假陽性的來源：
#   0x17-0x1B DESIGN_ID —— si5332-rm.pdf Table 11.1 標示為 R（唯讀），來自
#     NVM。"EX_BL" 就是空白 -EX 版晶片的預設 design ID（CBPro 專案檔名
#     `Si5332-GM1-RevD-EX_BL-...` 本身就是這麼來的），寫它無效、永遠吻合。
#   0x24 / 0x75 / 0xB9 / 0xBF —— 目標值剛好等於空白件預設值。
#   其餘 33 筆 config 暫存器目標值本來就是 0x00，讀回 0x00 證明不了任何事。
NON_DISCRIMINATING = {0x17: 0x45, 0x18: 0x58, 0x19: 0x5F, 0x1A: 0x42, 0x1B: 0x4C,
                       0x24: 0x01, 0x75: 0x01, 0xB9: 0x02, 0xBF: 0x01}


def probe_aux_ports(verbose=False):
    """唯讀診斷：只讀出板上兩顆 GPIO expander 四個 output port 的現值就結束。

    **完全不寫入任何 Si5332 暫存器、不跑位元走位**（只送冪等的方向暫存器設定
    和四筆讀取）。上機驗證 2026-08-10 這次改動時建議先跑這支：
      - 讀到 A=0x49 B=0x49 C=0x24 D=0xFF -> 板子在穩定狀態，代表新版 _i2c_wake()
        接下來會全部走 no-op 路徑（也就是舊版每個 session 都在亂脈衝腳位）
      - 讀到全部 0xFF                     -> 板子剛上電，新版會走完整走位序列，
        而且送出的封包跟舊版字面序列完全相同
    """
    si = Si5332USB(verbose=verbose)
    try:
        si.open(probe_only=True)
        state = si.aux_ports_initial
    finally:
        si.close()
    print("aux expander output ports: "
          + " ".join(f"{p}(0x{Si5332USB._AUX_PORTS[p][0]:02X} reg 0x"
                     f"{Si5332USB._AUX_PORTS[p][1]:02X})=0x{state[p]:02X}"
                     for p in ("A", "B", "C", "D")))
    if state == Si5332USB._AUX_SETTLED:
        print("-> 穩定狀態（跟側錄 session 1~5 一致）")
    elif all(v == 0xFF for v in state.values()):
        print("-> 剛上電未設定（跟側錄 session 0 一致）")
    else:
        print("-> 第三種狀態，兩份側錄都沒看過，先回報不要繼續寫入")
    return state


def self_test(verbose=True):
    """確認讀取協定正常，並且**真的**核對設定是否生效。

    EXPECTED 值取自 `si5332_3out_registers.csv`；如果之後換了 Si5332 的
    設定內容，`DISCRIMINATING` 也要跟著更新，不然會誤判成 MISMATCH。
    """
    print("=== 開啟裝置 + 連線握手 ===")
    with Si5332USB(verbose=verbose) as si:
        aux = si.aux_ports_initial
        if aux:
            print("  session 開始時的 aux expander 狀態："
                  + " ".join(f"{p}=0x{aux[p]:02X}" for p in ("A", "B", "C", "D")))

        print("\n=== 有鑑別力的暫存器（空白晶片會讀到 0x00，這才算數）===")
        all_ok = True
        for addr, expect in DISCRIMINATING.items():
            val = si.read_register(addr)
            match = (val == expect)
            all_ok &= match
            print(f"  0x{addr:02X}: read=0x{val:02X} expect=0x{expect:02X}  "
                  f"{'OK' if match else '[X] MISMATCH'}")

        print("\n=== 僅供參考，不列入判定 ===")
        stat = si.read_register(USYS_STAT_ADDR)
        stat_name = {USYS_STAT_READY: "READY", USYS_STAT_ACTIVE: "ACTIVE"}.get(stat, "?")
        print(f"  0x07 USYS_STAT = 0x{stat:02X} ({stat_name})"
              " —— 只代表 Active 命令跑過，**不代表設定值正確**"
              "（NOTES.md 已記錄過兩次「設定歸零但 USYS_STAT=ACTIVE」的矛盾狀態）")
        for addr, expect in NON_DISCRIMINATING.items():
            val = si.read_register(addr)
            print(f"  0x{addr:02X}: read=0x{val:02X} (=0x{expect:02X} 也代表空白件，無鑑別力)")

    print()
    if all_ok:
        print("[OK] 有鑑別力的暫存器全部吻合，設定確實生效、讀取協定也正常")
    else:
        print("[X] 設定沒有生效（讀到的是空白/設定遺失的狀態）。"
              "先跑 si5332_configure_linux.py 寫入；"
              "若寫入回報成功卻仍是這個結果，見 NOTES.md 2026-08-10 的分析")
    return all_ok


if __name__ == "__main__":
    import argparse

    _p = argparse.ArgumentParser(description="Si5332-6EX-EVB USB 低階協定自我測試")
    _p.add_argument("--aux-only", action="store_true",
                     help="唯讀診斷：只讀 GPIO expander 四個 port 的現值就結束，"
                          "不走位、不碰 Si5332（上機驗證改動時先跑這個）")
    _p.add_argument("--quiet", action="store_true", help="不印每一步的封包內容")
    _a = _p.parse_args()

    if _a.aux_only:
        probe_aux_ports(verbose=not _a.quiet)
    else:
        self_test(verbose=not _a.quiet)
