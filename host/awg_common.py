"""
awg_common.py — awg-test-step-16 host 端共用模組（2026-07-27 新增）

背景：`host/SCRIPTS_AUDIT.md`（2026-07-21）盤點 71 支腳本後發現，本專案
一直沒有自己的 `awg_common.py`，部分腳本繼續 import 隔壁 `awg-test-
step-14/host/awg_common.py`——那是完全不同版本的暫存器配置（Step 11-14
共用），只有極少數常數剛好還沒撞號，是個容易被複製貼上帶出問題的地雷
（見 audit 報告 2.5 節）。這支模組是本專案（`awg-test-step-16`）自己的
權威來源，所有位址常數直接對照 `PORTS.md`「指令總表」（表 1-4）逐條抄
過來——**`PORTS.md` 改了，這裡也要跟著改**，兩邊要保持一致。

內容：
  - `open_board()` / `wi()` / `wo()` / `ti()` 基本 wrapper
  - WI(0x00-0x1F)/WO(0x20-0x3F)/TI bit(TI_CMD=0x40)/Aurora 封包 type(T_*)
    常數
  - `send_pkt()`：透過 `PI_WAVE_DATA`(0x81) BTPipeIn 送出 2-beat 封包
    （本專案「統一讀取/寫入架構」收斂後，寫入類封包幾乎都是這個形狀，
    dest_id=自己的 board_id 就是本機 loopback，跟送去其他板子完全同一
    套機制，一致性優先）
  - `send_beat0_pkt()`：1-beat 封包（`T_ENUM_START`/`T_TRIG_START`/
    `T_REINIT` 這幾個沒有 payload 的封包用）
  - 常見 WO 的 decode 函式（`decode_enum_status`/`decode_aurora_status`/
    `decode_rt_board_info`/`decode_hard_err`）
  - `read_status_reply()` + 四種 `QT_*` payload 的 decode 函式（配合
    `T_QUERY`(0x12)/`PO_STATUS_REPLY`(0xA1)，見 PORTS.md「統一讀取/
    寫入架構」章節的精確 bit 排列）
  - `BOARD_MAP`：目前 3 片板子（board A/board B/board C）的 serial/board_id/is_master
    名冊，跟 `post_flash_checklist.py` 保持同一份

不包含（刻意不做）：DDR4/waveform 播放**狀態**相關的 helper（`read_
port_status.py` 讀 `WO 0x30-0x33` 的 play_pos/mux_sel 等即時狀態，這次
統一讀取/寫入架構明確排除的範圍，見 PROJECT.md——`wave_len_for_freq`/
`pad_to_burst` 這種純數學 buffer 長度計算不算在排除範圍內，見下方
「DDR4 波形串流常數 + 產生 helper」小節），flash record pack/unpack
（`test_flash_ctrl.py`/`test_amp_calib_flash.py` 各自有專屬格式，沒有
共通到值得抽出來）。
"""
import sys
import struct
import time

import numpy as np

sys.path.insert(0, r"C:\Program Files\Opal Kelly\FrontPanelUSB\API\Python\3")
import ok


# ── FrontPanel connection ────────────────────────────────────────────────

def open_board(serial=None):
    """開啟一片板子，回傳 (dev, fp)。serial=None 時自動偵測目前 USB 上
    第一片裝置（不指定序號）——2026-08-10 新增，給「誰接 USB 就是誰」
    這種用法用（跟 FPGA 端 Phase 6「本機直送 T_BOARD_ID_ASSIGN」的既有
    邏輯一致，那邊本來就是看物理上接哪條 USB 決定 is_master，不是看
    序號）；多片板子同時接著 USB 時任意挑第一片。連不上/掃描不到裝置
    就直接 sys.exit。"""
    devs = ok.FrontPanelDevices()
    if serial is None:
        count = devs.GetCount()
        if count == 0:
            sys.exit("[X] no Opal Kelly device found on USB")
        serial = devs.GetSerial(0)
    dev = devs.Open(serial)
    if dev is None:
        sys.exit(f"[X] cannot open {serial}")
    dev.LoadDefaultPLLConfiguration()
    return dev, dev.GetFPGADataPortClassic()


def device_serial(dev):
    """回傳已開啟 dev 物件的序號（自動偵測連的是哪片板子時，給呼叫端
    log 用）。"""
    info = ok.okTDeviceInfo()
    dev.GetDeviceInfo(info)
    return info.serialNumber


# ── 基本 WI/WO/TI wrapper ─────────────────────────────────────────────────

TI_CMD = 0x40


def wi(fp, addr, val):
    fp.SetWireInValue(addr, val & 0xFFFFFFFF)
    fp.UpdateWireIns()


def wo(fp, addr):
    fp.UpdateWireOuts()
    return fp.GetWireOutValue(addr)


def ti(fp, bit):
    fp.ActivateTriggerIn(TI_CMD, bit)


# ── WI（WireIn，0x00-0x1F）── 對照 PORTS.md 表 3 ─────────────────────────
# 「已退役」的位址故意不定義常數（沒有腳本應該再寫它們），保留原始位址
# 數字在註解裡方便查閱歷史。

WI_DDR4_ADDR         = 0x00  # fp_ddr4_addr（Aurora/FP 共用 DDR4 write 位址）
WI_DDR4_W0           = 0x01
WI_DDR4_W1           = 0x02
WI_DDR4_W2           = 0x03
WI_DDR4_W3           = 0x04
WI_LIST_SEL          = 0x05  # fp_list_sel
WI_LIST_ADDR         = 0x06  # fp_list_addr
WI_LIST_LEN          = 0x07  # fp_list_len
WI_DEPTH             = 0x08  # fp_depth
WI_PLAY_EN           = 0x09  # fp_play_en
# 0x0a fp_scale_cfg          — 2026-07-27 已退役（統一讀取/寫入架構收斂）
# 0x0b calib_sel             — 2026-07-27 已退役
# 0x0c calib_data            — 2026-07-27 已退役
# 0x0d board_id/is_master    — 2026-07-27 已退役（收斂到 T_BOARD_ID_ASSIGN）
# 0x0e fp_trig_slot           — 2026-08-04 已退役（z0 合併進 T_TIMER_CTRL，
#                                T_TRIG_SLOT 整條路徑退役，見該常數註解）
# 0x0f fp_trig_intv           — 2026-08-04 已退役（同上）
WI_TIMER_CTRL_WORD   = 0x10  # port 0-3 timer run/loop/depth 打包（2026-08-04
                             # 起 port 0 也併入，之前只有 port 1-3）
WI_RESERVE_DEST_ID   = 0x11  # reserve_dest_id
WI_DDR4_RW1_ADDR     = 0x12  # fp_ddr4_rw_1/wi_addr（本機 DDR4 讀寫位址）
# 0x13 fp_amp_ctrl_data      — 2026-07-27 已退役
WI_DDR4_RW1_W0       = 0x14
WI_DDR4_RW1_W1       = 0x15
WI_DDR4_RW1_W2       = 0x16
WI_DDR4_RW1_W3       = 0x17
# 0x18 wi_ext_clk_sel        — 2026-07-27 已退役
WI_AMP_SEL           = 0x19  # amp_ctrl 選擇器：讀寫雙用途，寫入方向已退役
                              # （配 ti_amp_ctrl_wr），讀回方向（配 WO 0x36）仍在用
WI_FLASH_TARGET_SEL  = 0x1A  # flash_target_sel（bits[1:0]）
# 0x1b sine_wi_sel_slice     — 2026-07-27 已退役
# 0x1c sine 資料              — 2026-07-27 已退役
# 0x1d dac_mode_ramp 暫存區   — 2026-07-27 已退役
WI_RAW_RD_ADDR       = 0x1E  # ddr_writer_0/raw_rd_addr[6:0]
WI_PER_HOP_VALUE     = 0x1F  # fp_per_hop_value（手動覆寫用，配 TI bit10）


# ── WO（WireOut，0x20-0x3F）── 對照 PORTS.md 表 4 ────────────────────────

WO_DDR_BUSY          = 0x20  # concat_ddr_busy
WO_DDR4_RW1_R0       = 0x21
WO_DDR4_RW1_R1       = 0x22
WO_DDR4_RW1_R2       = 0x23
WO_DDR4_RW1_R3       = 0x24
WO_DDR4_RW1_STATUS   = 0x25  # fp_ddr4_rw_1/wo_status
WO_ENUM_STATUS       = 0x26  # bit0=init_ok bits[5:1]=total_boards bits[10:6]=board_index
WO_AURORA_STATUS     = 0x27  # bit0=channel_up_0 bit1=channel_up_1
WO_RESERVE_STATUS    = 0x28  # concat_reserve_status
WO_TRIG_DELAY        = 0x29  # concat_trig_delay（au_trig_delay 讀回）
WO_FLASH_STATUS      = 0x2A  # concat_flash_status（busy/done/err/loader_done）
WO_RAW_MAGIC         = 0x2B  # fpga_flash_ctrl_0/raw_magic（診斷用）
WO_HARD_ERR          = 0x2C  # [1:0]=hard_err sticky [31:2]=per_hop_value_eff[29:0]
WO_RESERVE_DIAG      = 0x2D  # concat_reserve_diag
WO_EXT_CLK_FREQ      = 0x2E  # ext_clk_freq_counter_0/freq_count_sync
WO_DIAG              = 0x2F  # concat_diag
WO_PORT_STATUS_0     = 0x30  # current_idx/next_idx/mux_sel/play_pos
WO_PORT_STATUS_1     = 0x31
WO_PORT_STATUS_2     = 0x32
WO_PORT_STATUS_3     = 0x33
WO_RT_BOARD_INFO     = 0x34  # board_id[15:0]/is_master[16]/ext_clk_sel[17]/dac_mode_ramp[29:18]
WO_SCALE_CFG_RO      = 0x35  # scale_cfg 即時值讀回
WO_AMP_CTRL_RO       = 0x36  # amp_ctrl 選中 channel 讀回（索引沿用 WI 0x19）
WO_COEF_RO           = 0x37  # calib_coef 讀回（2026-07-27 起索引沿用 au_sel，見 PORTS.md）
WO_RAW_LO            = 0x38  # ddr_writer_0/raw_rd_data_lo
WO_RAW_HI            = 0x39  # ddr_writer_0/raw_rd_data_hi
WO_DIAG_NODES        = 0x3A  # concat_diag_nodes
WO_FP_FIRST_LO       = 0x3B
WO_FP_FIRST_HI       = 0x3C
WO_DISP_FIRST_LO     = 0x3D
WO_LRH_FIRST_LO      = 0x3E
WO_LRH_FIRST_HI      = 0x3F

WO_PORT_STATUS_ALL = [WO_PORT_STATUS_0, WO_PORT_STATUS_1, WO_PORT_STATUS_2, WO_PORT_STATUS_3]


# ── TI bit（TI_CMD=0x40）── 對照 PORTS.md 表 2 ──────────────────────────
# 已退役的 bit（4/6/8/9/11/15/18，全部是 2026-07-27 統一讀取/寫入架構
# 收斂時拔掉的「本機直寫」觸發）故意不定義常數。

TI_BIT_DDR4_WR           = 0   # fp_ddr4_rw_1 內部解碼
TI_BIT_DDR4_RD           = 1
TI_BIT_LIST_WR           = 2   # playlist LIST_WRITE（本機）
TI_BIT_SW_TRIG           = 3   # 已於 2026-07-24「Trigger 統一化架構改版」移除
                                # （本機 TI global trigger 拿掉，所有播放觸發
                                # 統一由 native_trig_cdc_0 驅動）。保留這個
                                # 常數只是為了讓還在用它的舊腳本 import 不會
                                # 直接爆掉，觸發本身已經沒有效果——舊腳本要
                                # 真正觸發播放，需改用 test_aurora_trigger.py
                                # 送 T_TRIG_START。
TI_BIT_TRIG_LIST_WR      = 5   # port 0 trigger list 寫入觸發
TI_BIT_AU_FULL_RESET     = 7   # 整個 Aurora 子系統回到乾淨狀態
TI_BIT_PER_HOP_VALUE_WR  = 10  # 手動覆寫 per_hop_value（配 WI 0x1F）
TI_BIT_DEBUG_REPLY_TRIG  = 12  # 2026-07-29 新增，診斷專用：直接觸發 au_reply_wr/src(=0xDEAD)/
                                # query_type(=0x7F)/data，取代已刪除的 T_DEBUG_REPLY_INJECT(0x1F)
                                # 封包機制——不用組封包，ActivateTriggerIn 一行直接觸發
TI_BIT_FP_ALIGN          = 16  # fp_input pair-acc 對齊重置
TI_BIT_FLUSH_STANDBY     = 16  # 沿用既有腳本慣稱（跟上面同一個 bit）
TI_BIT_REINIT            = 17  # 「初始化」：清空 playlist/trigger list/DDR4 + 立即 0V
TI_BIT_FLASH_TARGET_SEL_WR = 19
TI_BIT_TIMER_WR_1        = 20
TI_BIT_TIMER_WR_2        = 21
TI_BIT_TIMER_WR_3        = 22
TI_BIT_FIFO_RST          = 28
TI_BIT_FLASH_CTRL_RST    = 29
TI_BIT_AU_CTRL_RST       = 30
TI_BIT_AU_RESERVE        = 31


# ── Aurora 封包 type（8-bit，beat0[39:32]）── 對照 PORTS.md 表 1 ────────

T_CALIB_WR              = 0x02
T_CALIB_RST             = 0x03
T_DDR4_WRITE            = 0x04
T_LIST_WRITE            = 0x05
T_PLAY_CTRL             = 0x06
T_SCALE_CFG             = 0x07
# 0x08 T_TRIG_SLOT           — 2026-08-04 已退役（z0 的 trig_timer 排程
#                              資料原本走這個獨立封包，從無 host wrapper，
#                              已合併進 T_TIMER_CTRL 跟 z1-z3 用同一套機制，
#                              見 rtl/aurora_ctrl_mux.v「Timer hold
#                              registers」章節完整推導）
T_AMP_CTRL              = 0x09
T_FLUSH_STANDBY         = 0x0A
T_TIMER_CTRL            = 0x0C  # 3-beat：beat1[3:0]=list_wr_slot(0-15) [8:4]=list_depth(0-16,5-bit，
                                 # 見 rtl/trig_timer.v) [9]=loop_en [10]=run [12:11]=port(0-3,
                                 # 對應 module z0-z3，非 trigger group) beat2[31:0]=list_wr_intv
                                 # （dac_clk cycles）。2026-08-04 新增，host 端首次接上這條既有的
                                 # local_reg_handler.v(0x0C) 解碼路徑，見 board_ctrl.set_trig_timer()。
                                 # 同一天稍後：slot 3→4-bit、depth 3→5-bit（16 slot 排程功能，見
                                 # PROJECT.md trigger group 排程功能三部曲「B」條目），beat1 版面
                                 # 隨之從 10-bit 改成 13-bit。
T_TRIG_MASK             = 0x0D  # 2026-07-27 起改稱 group cfg：beat1[5:0]={group_id[1:0],channel_mask[3:0]}
T_FLASH_ERASE           = 0x0F
T_STATUS_REPORT         = 0x11  # 2026-07-27 起可變長度（見 aurora_reply_tx.v payload 排列）
T_QUERY                 = 0x12  # 2026-07-27 起 2-beat：beat1[7:0]=query_type
T_WAVEFORM_STREAM       = 0x13
T_WAVEFORM_STREAM_LAST  = 0x14
T_FLASH_TARGET_SEL      = 0x15  # beat1[1:0]=目標 sector（0=身份/1=scale_cfg/2=amp_ctrl/3=calib_coef）
T_FLASH_WRITE_DATA      = 0x16
T_ENUM_START            = 0x1A  # 1-beat
T_TRIG_START            = 0x1B  # 2-beat：beat1[3:0]=group_select
T_RESERVE_START         = 0x1C
T_TRIG_DELAY_CFG        = 0x1D
T_BOARD_ID_ASSIGN       = 0x1E  # 2-beat：beat1[31:0]=per_hop_value(auto) [36:32]=total_boards(auto)
                                 # [37]=is_master（2026-07-27 新增，host 指定），board_id 恆等於
                                 # board_index（enum 算好的既有值，不是封包 payload 帶的）
T_DIAG_START            = 0x1F  # 2-beat：beat1[15:0]=查詢起點 board_id，2026-08-20 Phase 7 新增
                                 # （enum 失敗斷點定位，見 rtl/aurora_ctrl_channel.v TYPE_DIAG_REQ/ACK）。
                                 # 原本是 0x1F T_DEBUG_REPLY_INJECT——2026-07-29 新增又拔除，改成
                                 # TI_BIT_DEBUG_REPLY_TRIG（見上方），號碼空出後這次重新分配
T_REINIT                = 0x27  # 1-beat，廣播版「初始化」
T_EXT_CLK_SEL           = 0x28  # 2-beat：beat1[0]=值
T_SINE_CTRL             = 0x29  # 2-beat：beat1[31:0]=data beat1[37:32]=sel
T_DAC_MODE_RAMP         = 0x2A  # 已於 2026-08-05 退役，RTL 不再解碼，不要
                                 # 再用這個送封包——dac_mode_ramp 併入
                                 # sine_ctrl_regs.v，改用 T_SINE_CTRL
                                 # （PARAM_MODE=7/PARAM_RAMP_EN=6，見上方
                                 # 對應常數 + rtl/sine_ctrl_regs.v 檔頭）。
                                 # 常數保留只為了舊 log/註解對照，不要拿去
                                 # send_pkt()。
T_MANUAL_TOTAL_BOARDS   = 0x2B  # 2-beat：beat1[4:0]=值（0=停用）
T_GROUP_SCHED_CTRL      = 0x2C  # 2026-08-04 新增（trigger group 排程功能三部曲「C」）。
                                 # 3-beat，跟 T_TIMER_CTRL 同一套格式，差別是沒有 port
                                 # 欄位（group_trig_scheduler 每板只有一個實例，不像
                                 # trig_timer 要選 z0-z3），多一個 group_sel 欄位：
                                 # beat1[3:0]=slot(0-15) [8:4]=depth(0-16,5-bit)
                                 # [9]=loop_en [10]=run [14:11]=group_sel(0-15)
                                 # beat2[31:0]=interval_cycles（dac_clk cycles）。
                                 # 純 Aurora 路徑，沒有 FP WireIn 直寫（見
                                 # board_ctrl.set_group_sched()）。
T_SINE_LIST_CTRL        = 0x2D  # 2026-08 新增：Sine mode N-slot 排程機制（比照 DDR
                                 # waveform_controller.v 的 list 設計），寫入
                                 # sine_ctrl_regs.v 新增的 per-channel slot table
                                 # （N_SLOT=4）。2-beat：beat1[31:0]=data
                                 # beat1[39:32]=sel={slot_idx[1:0],param_sel[2:0],
                                 # ch_sel[2:0]}——sel 是獨立於 T_SINE_CTRL 的命名
                                 # 空間（那邊 6-bit，這邊 8-bit，多了 slot_idx）。
                                 # param_sel 0-5 跟 T_SINE_CTRL 共用同一組 PARAM_*
                                 # 常數（見下方），6=SINE_LIST_PARAM_COMMIT_DEPTH
                                 # （commit list_depth 並 arm，語意跟 T_SINE_CTRL 的
                                 # PARAM_RAMP_EN=6 無關，只是剛好同一個數字，兩個
                                 # 封包各自獨立解讀）。host 端寫入順序 hard
                                 # contract：一個 channel 的每個 slot 內容都要先寫
                                 # 完，最後才送 commit——board_ctrl.set_sine_slot()
                                 # 已經照這個順序做，見 rtl/sine_ctrl_regs.v 檔頭
                                 # 完整規格。
# 2026-08-18/19：T_NOTCH_SWEEP_CTRL(0x2E)（IFFT 梳狀波形固定多頻抵銷，
# FPGA 即時扣除版）已整批退役——上機驗證通過後確認實際使用情境不需要
# 真正即時切換，改成 host 端在 arm_multitone_pattern() 直接算好排除
# 頻率再上膛（見 board_ctrl.py 對應章節），RTL 端的解碼也已一併拆除、
# 重新 build+燒錄，opcode 0x2E 現在是空號，完整過程見 NOTES.md
# 2026-08-19「IFFT notch 改回 host 端算好上膛」章節。

QT_BOARD_INFO    = 0
QT_SINE_STATUS   = 1
QT_CALIB_STATUS  = 2
QT_TRIGGER_GROUP = 3
QT_DDR_STATUS    = 4  # 2026-07-30 新增
QT_FLASH_STATUS  = 5  # 2026-07-31 新增
QT_NAMES = {0: "QT_BOARD_INFO", 1: "QT_SINE_STATUS", 2: "QT_CALIB_STATUS", 3: "QT_TRIGGER_GROUP", 4: "QT_DDR_STATUS", 5: "QT_FLASH_STATUS"}


# ── BlockPipeIn / BlockPipeOut ───────────────────────────────────────────

PI_WAVE_DATA    = 0x81  # 統一指令閘道：所有寫入類封包 + T_QUERY 都走這裡
BTPIPE_BLOCK    = 1024
PO_DIAG         = 0xA0
PO_STATUS_REPLY = 0xA1  # 2026-07-27 新增，配 T_QUERY/T_STATUS_REPORT

DEST_BCAST = 0xFFFF


# ── 封包組裝/送出 ─────────────────────────────────────────────────────────

def _pad_to_block(pkt, block=BTPIPE_BLOCK):
    rem = len(pkt) % block
    if rem != 0:
        pkt += b'\x00' * (block - rem)
    return pkt


def send_pkt(fp, dest_id, pkt_type, beat1_lo=0, beat1_hi=0, extra_beats=None):
    """組一個「本機 loopback / 跨板廣播共用同一套機制」的封包送出去。

    dest_id：目標板子的 board_id（本機設定自己就填自己的 board_id，
    跟送去別片板子完全一樣，見 PROJECT.md「統一讀取/寫入架構」的
    loopback 設計）。dest_id=DEST_BCAST 是廣播。

    beat1_lo/beat1_hi：2-beat 封包（本專案絕大多數 T_* 都是）的 beat1
    低/高 32-bit。extra_beats：list of (lo, hi)，給超過 2-beat 的封包
    （如 T_DDR4_WRITE）用，會接在 beat1 後面依序送出，pkt_len 自動
    含入。
    """
    beats = [(beat1_lo, beat1_hi)] + list(extra_beats or [])
    pkt_len = 1 + len(beats)
    b0 = struct.pack('<II', (0 << 16) | (dest_id & 0xFFFF), (pkt_len << 8) | pkt_type)
    pkt = bytearray(b0)
    for lo, hi in beats:
        pkt += struct.pack('<II', lo & 0xFFFFFFFF, hi & 0xFFFFFFFF)
    pkt = _pad_to_block(bytearray(pkt))
    ret = fp.WriteToBlockPipeIn(PI_WAVE_DATA, BTPIPE_BLOCK, pkt)
    if ret != len(pkt):
        # 2026-08-20：原本這裡是 sys.exit()，寫給單次執行的 CLI 診斷腳本用
        # 沒問題（USB 寫入失敗直接結束腳本很合理），但這支函式後來被
        # host/web/app.py 這個長期執行的 Flask server 共用，sys.exit() 會
        # 直接把整個 web server process 砍掉，不會被任何 route 的
        # try/except Exception 接住——實測發現 Refresh Status 卡住 ~10 秒
        # 後整個 process 死掉、之後所有請求全部連不上，就是這裡造成的
        # （見 PROJECT.md「test1 面板」章節的除錯過程）。改成 raise，讓
        # 呼叫端（web server 的每個 route 都已經有 try/except Exception）
        # 正常接住、回傳錯誤訊息，不會拖垮整個 server；CLI 腳本沒接的話
        # 這個 exception 一樣會讓腳本印出 traceback 並結束，效果跟原本的
        # sys.exit() 差不多。
        raise RuntimeError(f"WriteToBlockPipeIn short: {ret} != {len(pkt)} (type=0x{pkt_type:02X})")


def send_beat0_pkt(fp, dest_id, pkt_type):
    """1-beat 封包（無 payload）：T_ENUM_START/T_TRIG_START(舊)/T_REINIT 這類。"""
    pkt_len = 1
    b0 = struct.pack('<II', (0 << 16) | (dest_id & 0xFFFF), (pkt_len << 8) | pkt_type)
    pkt = _pad_to_block(bytearray(b0))
    ret = fp.WriteToBlockPipeIn(PI_WAVE_DATA, BTPIPE_BLOCK, pkt)
    if ret != len(pkt):
        # 同上 send_pkt() 的說明——改成 raise，不用 sys.exit()。
        raise RuntimeError(f"WriteToBlockPipeIn short: {ret} != {len(pkt)} (type=0x{pkt_type:02X})")


# ── 常見 WO 的 decode ─────────────────────────────────────────────────────

def decode_enum_status(raw):
    return {
        "init_ok":      raw & 0x1,
        "total_boards": (raw >> 1) & 0x1F,
        "board_index":  (raw >> 6) & 0x1F,
    }


def decode_aurora_status(raw):
    return {"channel_up_0": raw & 0x1, "channel_up_1": (raw >> 1) & 0x1}


def decode_rt_board_info(raw):
    return {
        "board_id":      raw & 0xFFFF,
        "is_master":     (raw >> 16) & 0x1,
        "ext_clk_sel":   (raw >> 17) & 0x1,
        "dac_mode_ramp": (raw >> 18) & 0xFFF,
    }


def decode_hard_err(raw):
    return {"hard_err": raw & 0x3, "per_hop_value_eff": (raw >> 2) & 0x3FFFFFFF}


def decode_reserve_diag(raw):
    """WO 0x2D。[0:4]=reserve 診斷 sticky（見 check_reserve_diag.py），
    [5]=reply_arb_seen，[6]=reply_arb_granted（2026-07-29 新增，見
    aurora_tx1_arbiter.v）。[7:22]（16-bit）=au_reply_src、[23:30]
    （8-bit）=au_reply_query_type（2026-07-29 再新增，取代已刪除的
    T_DEBUG_REPLY_INJECT(0x1F)+PO_STATUS_REPLY 診斷組合，直接讀 WO 看
    這兩個暫存器「當下」的值，不經過 BTPipeOut/CDC）。"""
    return {
        "fwd_sent":          raw & 0x1,
        "timeout_hit":       (raw >> 1) & 0x1,
        "reply_hit":         (raw >> 2) & 0x1,
        "pulse_seen":        (raw >> 3) & 0x1,
        "au_start_seen":     (raw >> 4) & 0x1,
        "reply_arb_seen":    (raw >> 5) & 0x1,
        "reply_arb_granted": (raw >> 6) & 0x1,
        "au_reply_src":        (raw >> 7) & 0xFFFF,
        "au_reply_query_type": (raw >> 23) & 0xFF,
    }


# ── T_QUERY / PO_STATUS_REPLY ─────────────────────────────────────────────

def send_query(fp, dest_id, query_type):
    """送 T_QUERY(0x12)，beat1[7:0]=query_type。回覆要另外呼叫
    read_status_reply() 讀 PO_STATUS_REPLY（記得先等一下讓封包跑完
    一圈，本機 loopback 通常幾十 us 內就好，跨板視環路大小可能要多等）。
    """
    send_pkt(fp, dest_id, T_QUERY, beat1_lo=query_type & 0xFF)


def read_status_reply(fp):
    """讀 PO_STATUS_REPLY(0xA1)，回傳 (src_board_id, query_type, data_words)。
    data_words 是 60 個 32-bit word 的 tuple（word[i] = au_reply_data[i*32 +: 32]），
    依 query_type 呼叫對應的 decode_qt_*() 解讀。"""
    buf = bytearray(1024)
    fp.ReadFromPipeOut(PO_STATUS_REPLY, buf)
    w = struct.unpack_from('<256I', buf)
    src        = w[0] & 0xFFFF
    query_type = (w[0] >> 16) & 0xFF
    data_words = w[1:61]
    return src, query_type, data_words


def measure_ext_clk_freq_remote(fp, dest_id, interval=0.3, reply_wait=0.05):
    """跟 measure_ext_clk_freq.py（本機 USB 直讀 WO 0x2E）邏輯完全一樣的
    兩次讀取＋算 delta，但兩次都透過 T_QUERY(QT_BOARD_INFO) 遠端讀
    ext_clk_freq_count，給 master-only USB 的 web backend 用（2026-07-30
    新增，見 NOTES.md「QT_BOARD_INFO 擴充規格」）。fp 必須是 master
    板子自己的 FrontPanel handle，dest_id 是要量測的目標板子。
    回傳頻率（Hz，float）。"""
    send_query(fp, dest_id, QT_BOARD_INFO)
    time.sleep(reply_wait)
    _, _, data_words = read_status_reply(fp)
    cnt0 = decode_qt_board_info(data_words)["ext_clk_freq_count"]
    t0 = time.time()

    time.sleep(interval)

    send_query(fp, dest_id, QT_BOARD_INFO)
    time.sleep(reply_wait)
    _, _, data_words = read_status_reply(fp)
    cnt1 = decode_qt_board_info(data_words)["ext_clk_freq_count"]
    t1 = time.time()

    delta_cnt = (cnt1 - cnt0) & 0xFFFFFFFF
    delta_t = t1 - t0
    return delta_cnt / delta_t


def _bits_across_words(data_words, lo_bit, width):
    """從攤平的 data_words（每個 32-bit）取出 [lo_bit +: width] 的值，
    跨 word 邊界時自動組合（QT_SINE_STATUS/QT_CALIB_STATUS 的長欄位要用）。"""
    val = 0
    got = 0
    bit = lo_bit
    while got < width:
        w_idx = bit // 32
        w_off = bit % 32
        take = min(32 - w_off, width - got)
        chunk = (data_words[w_idx] >> w_off) & ((1 << take) - 1)
        val |= chunk << got
        got += take
        bit += take
    return val


def decode_qt_board_info(data_words):
    """QT_BOARD_INFO（word 數=2→2026-08-20 Phase 7 起 3，word0=bit0-63/
    word1=bit64-127/word2=bit128-191）。word1：trig_delay(16)+
    manual_delay_active(1) 之後接續 2026-07-30 新增的 dac_mode_ramp(12)/
    ext_clk_freq_count(32)+2026-08-20 新增 tx_timeout_seen(1)，剩
    2-bit padding。word2 是 2026-08-20 Phase 7 新開的（DIAG 5 個
    bit，其餘 59-bit padding），用 _bits_across_words 保證正確。"""
    return {
        "board_id":            _bits_across_words(data_words, 0, 16),
        "is_master":           _bits_across_words(data_words, 16, 1),
        "total_boards":        _bits_across_words(data_words, 17, 5),
        "board_index":         _bits_across_words(data_words, 22, 5),
        "channel_up_0":        _bits_across_words(data_words, 27, 1),
        "channel_up_1":        _bits_across_words(data_words, 28, 1),
        "init_ok":             _bits_across_words(data_words, 29, 1),
        "ext_clk_sel":         _bits_across_words(data_words, 30, 1),
        "per_hop_value":       _bits_across_words(data_words, 31, 32),
        "trig_delay":          _bits_across_words(data_words, 64, 16),
        "manual_delay_active": _bits_across_words(data_words, 80, 1),
        "dac_mode_ramp":       _bits_across_words(data_words, 81, 12),
        "ext_clk_freq_count":  _bits_across_words(data_words, 93, 32),
        # 2026-08-20 新增：dispatcher.v 的 tx_timeout sticky 旗標（見
        # rtl/dispatcher.v/aurora_reply_tx.v 檔頭說明）——這片板子自己
        # 往下一棒送封包時，link 有搭起來（channel_up_1=1）但曾經逾時
        # 過。跟 channel_up_1（即時值）互補，兩者不會同時反映同一種
        # 原因：channel_up_1=0 時這裡不會是 1。
        "tx_timeout_seen":     _bits_across_words(data_words, 125, 1),
        # 2026-08-20 Phase 7 新增：DIAG（enum 失敗斷點定位）查詢結果，見
        # rtl/aurora_ctrl_channel.v/aurora_reply_tx.v board_info_word2。
        "diag_ok":               _bits_across_words(data_words, 128, 1),
        "diag_busy":             _bits_across_words(data_words, 129, 1),
        "diag_r_channel_up_0":   _bits_across_words(data_words, 130, 1),
        "diag_r_channel_up_1":   _bits_across_words(data_words, 131, 1),
        "diag_r_relay_blocked":  _bits_across_words(data_words, 132, 1),
    }


PARAM_TUNING_WORD     = 0
PARAM_PHASE           = 1
PARAM_START_AMP       = 2
PARAM_STEP            = 3
PARAM_DURATION_CYCLES = 4
PARAM_LOOP_MODE       = 5
# 2026-08-05 新增：dac_mode_ramp 併入 sine_ctrl_regs.v（idle/active 雙
# 緩衝，取代退役的 T_DAC_MODE_RAMP=0x2A 整包覆寫），走同一個 T_SINE_CTRL
# 封包（sel={param_sel[2:0],ch_sel[2:0]}），value 只用 bit0。
# PARAM_RAMP_EN：per-channel（ch_sel 0-7 直接選 physical channel）。
# PARAM_MODE：per-module（ch_sel[2:1] 選 module 0-3，ch_sel[0] 忽略——
# 呼叫端固定傳 ch_sel=module<<1，見 rtl/sine_ctrl_regs.v 檔頭說明）。
PARAM_RAMP_EN         = 6
PARAM_MODE            = 7
PARAM_NAMES = ["tuning_word", "phase", "start_amp", "step", "duration_cycles", "loop_mode",
               "ramp_en", "mode"]

# 2026-08 新增：T_SINE_LIST_CTRL(0x2D) 專用的 param_sel=6（跟上面
# T_SINE_CTRL 的 PARAM_RAMP_EN=6 是完全不同的命名空間，只是數字剛好
# 一樣，見 T_SINE_LIST_CTRL 常數定義處說明）。
SINE_LIST_PARAM_COMMIT_DEPTH = 6
SINE_N_SLOT = 4  # rtl/sine_ctrl_regs.v 的 N_SLOT，slot table 深度上限


def decode_qt_sine_status(data_words):
    """QT_SINE_STATUS（word 數=29）。sine_stage_eff 是「依 mux_sel_sync
    已經選好 active side」的 8 channel x 6 param，channel-major/param-minor
    （channel ch 的 param p 在 bit[(ch*6+p)*32 +: 32]）。"""
    channels = []
    for ch in range(8):
        params = {}
        for p in range(6):
            bit = (ch * 6 + p) * 32
            params[PARAM_NAMES[p]] = _bits_across_words(data_words, bit, 32)
        channels.append(params)
    mux_sel_sync = _bits_across_words(data_words, 1536, 8)
    phase_acc = [_bits_across_words(data_words, 1544 + ch * 32, 32) for ch in range(8)]
    return {"channels": channels, "mux_sel_sync": mux_sel_sync, "phase_acc": phase_acc}


def decode_qt_calib_status(data_words):
    """QT_CALIB_STATUS（word 數=12）。"""
    scale_cfg = _bits_across_words(data_words, 0, 8)
    amp_ctrl = [_bits_across_words(data_words, 8 + ch * 18, 18) for ch in range(8)]
    calib_coef = [_bits_across_words(data_words, 152 + i * 18, 18) for i in range(32)]
    return {"scale_cfg": scale_cfg, "amp_ctrl": amp_ctrl, "calib_coef": calib_coef}


def decode_qt_trigger_group(data_words):
    """QT_TRIGGER_GROUP（word 數=1）。"""
    v = data_words[0]
    return {
        "group_id_a": v & 0x3,
        "group_id_b": (v >> 2) & 0x3,
        "group_id_c": (v >> 4) & 0x3,
        "group_id_d": (v >> 6) & 0x3,
    }


def decode_qt_ddr_status(data_words):
    """QT_DDR_STATUS（word 數=1，2026-07-30 新增）。每 channel 11-bit，
    跟既有 read_port_status.py（WO 0x30-33）同一種格式：
    bits[2:0]=current_idx, bits[5:3]=next_idx, bit[6]=mux_sel,
    bits[10:7]=play_pos（4-bit signed，-1=已上膛還沒被真正 trigger）。"""
    channels = []
    for ch in range(4):
        base = ch * 11
        current_idx = _bits_across_words(data_words, base, 3)
        next_idx    = _bits_across_words(data_words, base + 3, 3)
        mux_sel     = _bits_across_words(data_words, base + 6, 1)
        play_pos_raw = _bits_across_words(data_words, base + 7, 4)
        play_pos = play_pos_raw - 16 if play_pos_raw & 0x8 else play_pos_raw
        channels.append({
            "current_idx": current_idx,
            "next_idx":    next_idx,
            "mux_sel":     mux_sel,
            "play_pos":    play_pos,
        })
    return {"channels": channels}


def decode_qt_flash_status(data_words):
    """QT_FLASH_STATUS（word 數=10，2026-07-31 擴充：加入 flash 實際
    內容，不只狀態）。bit[3:0]=status，bit[11:4]=scale_cfg，
    bit[587:12]=calib_coef 32 筆（跟 aurora_reply_tx.v 組裝順序一致）。"""
    v0 = data_words[0]
    status = {
        "busy":        v0 & 1,
        "done":        (v0 >> 1) & 1,
        "err":         (v0 >> 2) & 1,
        "loader_done": (v0 >> 3) & 1,
    }
    scale_cfg = _bits_across_words(data_words, 4, 8)
    calib_coef = [_bits_across_words(data_words, 12 + i * 18, 18) for i in range(32)]
    return {**status, "scale_cfg": scale_cfg, "calib_coef": calib_coef}


QT_DECODERS = {
    QT_BOARD_INFO:    decode_qt_board_info,
    QT_SINE_STATUS:   decode_qt_sine_status,
    QT_CALIB_STATUS:  decode_qt_calib_status,
    QT_TRIGGER_GROUP: decode_qt_trigger_group,
    QT_DDR_STATUS:    decode_qt_ddr_status,
    QT_FLASH_STATUS:  decode_qt_flash_status,
}


# ── DDR4 波形串流常數 + 產生 helper ───────────────────────────────────────
# 移植自 awg-test-step-14/host/awg_common.py（2026-07-27，見檔頭說明 2.5
# 節）——這幾個是跟 DDR4 波形 buffer 大小/內容相關的純數學函式，沒有任何
# WI/WO/TI 位址依賴，多支腳本（`test_awg_arm_*_fpddr4.py` 等）需要用來
# 算「這個頻率要存幾個 sample 才能無縫循環」，跟本檔案「不含 DDR4/
# waveform 播放『狀態』」的排除範圍是兩件事（那個排除的是 play_pos/
# mux_sel 這種即時讀回，不是這種 host 端算 buffer 長度的數學）。

DAC_FS        = 100_000_000   # DAC sampling rate (Hz)
BURST_SAMPLES = 256           # samples per 64-beat DDR4 burst (64 x 4 samples/beat)


# 2026-08-21：容許的「浪費量」上限（sample 數）——N 不用剛好是
# BURST_SAMPLES 的倍數，只要離最近的倍數（往下）不超過這個值。放寬
# 這個限制的理由：`wave_len` 本來就可以是任意值，`ddr4_stream_reader.v`
# 會用 `fifo_we <= (samp_cnt < wave_len_r)` 擋掉超過 wave_len 的補值
# 樣本（不進播放 FIFO），這個能力這個專案第一次真正拿來用是 DC ramp
# 功能（見 board_ctrl._dc_ramp_cycle_length() 說明）。
#
# 但不能無限放寬到任意 N——完整的 arbitration 餘裕分析（直接讀
# `ddr4_stream_reader.v`/`dc_fifo_xpm`(`fifo_a/b_$ch`, DEPTH=1024,
# PROG_FULL_THRESH=768) 算出來，不是猜的）：
#   - 一次 AXI burst = 64 beat × 128-bit = 256 sample，狀態機每 beat
#     ~5 cycle（1 cycle 等 rvalid + 4 cycle 序列化拆 128-bit 寫 FIFO，
#     見 S_R/S_WRITE4），64 beat ≈320 cycle @300MHz ≈ **1.07µs**。
#   - DAC 100MHz 消耗，256 sample 播放耗時 **2.56µs** → 單一 reader
#     duty cycle ≈1.07/2.56≈42%（兩個獨立算法互相印證，見 NOTES.md
#     2026-07-26＋2026-08-14）。
#   - FIFO 從 `fifo_prog_full` 門檻(768)真的見底(0)的時間預算：
#     768×10ns=**7.68µs**；8 個 reader（4 module×A/B）最壞輪詢等待：
#     7×1.07µs（等其他 7 個）+1.07µs（自己這次）≈**8.56µs**，理論上
#     餘裕吃緊（8.56>7.68）。但這只是「同時 arm 多個 module」瞬間的
#     過渡情況——穩態時只有 4 個 active 側 reader 持續消耗+競爭
#     （idle 側沒在播放不搶 arbiter），4-way 最壞等待只要 ~4.28µs，
#     遠低於 7.68µs 預算；idle 側新波形只要在被真正選為 active 之前
#     於 7.49µs 內補滿即可，補滿後立刻降回 4-way 穩態，不會長期卡在
#     8-way 緊繃狀態。
# BURST_TOLERANCE=32（256 的 12.5%）讓 N 幾乎貼著 256 的倍數、浪費量
# 微小，不去打亂這整套既有餘裕結構，同時把搜尋空間從「只能是 256 的
# 倍數」大幅放寬，足以解掉像 10kHz+20kHz 這種在嚴格 256 倍數限制下
# 需要 N=160,000（超過舊預設 max_samples=100,000）才能兩個頻率都
# 精確表示的已知案例。
BURST_TOLERANCE = 32

# 每個 module 有獨立的 16MB 定址區段（board_ctrl.MODULE_ADDR_STRIDE，
# 2026-08-18 新增），這裡沿用同一個數值當預設搜尋上限（不 import
# board_ctrl，避免循環依賴，純數值巧合對齊）——比舊預設 100,000 寬裕
# 很多，`_check_slot_fits()` 仍是實際寫入前的最後一道防線。
DEFAULT_MAX_WAVE_SAMPLES = 4_194_304


def _search_burst_aligned(check_fn, max_samples, tol=BURST_TOLERANCE):
    """共用的搜尋迴圈：逐一嘗試 burst 數 j=1,2,3...，每個 j 只在
    [j*BURST_SAMPLES-tol+1, j*BURST_SAMPLES] 這個小窗口內找（從
    j*BURST_SAMPLES 開始往下試，優先選浪費量最小的 N），check_fn(n)
    回傳非 None 就直接回傳。找不到時回傳 None，呼叫端負責組錯誤訊息。"""
    j = 1
    while j * BURST_SAMPLES <= max_samples:
        L = j * BURST_SAMPLES
        lo = max(1, L - tol + 1)
        for n in range(L, lo - 1, -1):
            result = check_fn(n)
            if result is not None:
                return result
        j += 1
    return None


def wave_len_for_freq(freq_hz, max_error_ppm=100, max_samples=DEFAULT_MAX_WAVE_SAMPLES, dac_fs=DAC_FS):
    """Find smallest N (within BURST_TOLERANCE samples of a BURST_SAMPLES=256
    boundary, see BURST_TOLERANCE comment above) that represents freq_hz with
    at most max_error_ppm frequency error.

    Returns (N, K) where K is the number of complete cycles in N samples.
    actual_freq = K * dac_fs / N

    Use N/K as the waveform period (samples per cycle) so the buffer loops
    with zero phase discontinuity at the exact actual frequency. Caller must
    still pad the physically-written buffer to a BURST_SAMPLES multiple
    (pad_to_burst()), but must pass this true N (not the padded length) as
    wave_len so the RTL's tail-suppression drops only the small waste, not a
    full extra silent burst.
    """
    def check(n):
        k = round(n * freq_hz / dac_fs)
        if k == 0:
            return None
        actual = k * dac_fs / n
        if abs(actual - freq_hz) / freq_hz * 1e6 <= max_error_ppm:
            return n, k
        return None

    result = _search_burst_aligned(check, max_samples)
    if result is None:
        raise ValueError(
            f"Cannot represent {freq_hz/1e3:.3f} kHz within {max_error_ppm} ppm "
            f"in {max_samples} samples. Try increasing max_error_ppm."
        )
    return result


def wave_len_for_two_freqs(freq1_hz, freq2_hz, max_error_ppm=100, max_samples=DEFAULT_MAX_WAVE_SAMPLES, dac_fs=DAC_FS):
    """Same search as wave_len_for_freq(), but finds one N that represents
    BOTH freq1_hz and freq2_hz within max_error_ppm at once (2026-08-20, for
    independent ch1/ch2 frequencies packed into the same module buffer via
    _pack_ch1_ch2()) — each channel still gets its own integer cycle count
    (K1/K2) within the shared N, so both loop with zero phase discontinuity
    at their own actual frequency. See wave_len_for_freq() for the
    wave_len-vs-padded-length caveat.

    Returns (N, K1, K2). actual_freq1 = K1 * dac_fs / N, actual_freq2 likewise.
    """
    def check(n):
        k1 = round(n * freq1_hz / dac_fs)
        k2 = round(n * freq2_hz / dac_fs)
        if k1 == 0 or k2 == 0:
            return None
        actual1 = k1 * dac_fs / n
        actual2 = k2 * dac_fs / n
        if (abs(actual1 - freq1_hz) / freq1_hz * 1e6 <= max_error_ppm
                and abs(actual2 - freq2_hz) / freq2_hz * 1e6 <= max_error_ppm):
            return n, k1, k2
        return None

    result = _search_burst_aligned(check, max_samples)
    if result is None:
        raise ValueError(
            f"Cannot represent {freq1_hz/1e3:.3f} kHz and {freq2_hz/1e3:.3f} kHz "
            f"together within {max_error_ppm} ppm in {max_samples} samples. "
            "Try increasing max_error_ppm."
        )
    return result


# 2026-08-21：wave_len_for_freq_multiple_of()（「buffer 長度必須是 IFFT
# 週期 base_n 的整數倍」）已經被下面的 wave_len_for_comb_plus_freq() 取代
# ——上機前的單元測試直接抓到這個舊設計是數論死路：base_n 常帶有 11/71
# 這類「不友善」質因數（例如 500-5000Hz/step 500Hz 算出 base_n=199936=
# 2^8×11×71），要找到 j 讓 j*base_n 精確表示某些頻率（實測 1kHz/2kHz）
# 需要的 buffer 大到不現實（分析證明：在合理預算內搜尋的相對誤差恆定，
# 不會因為多試幾個 j 就變好，是結構性限制不是搜尋範圍問題）。完整推導
# 過程、Opus 深度分析、失敗案例數據見 NOTES.md 2026-08-21「wave_len_for_
# freq/wave_len_for_two_freqs 改用 BURST_TOLERANCE...」章節。


def wave_len_for_comb_plus_freq(step_hz, freq_hz, max_samples, burst_tol=BURST_TOLERANCE, dac_fs=DAC_FS):
    """IFFT 梳狀波 + Single 頻率混用時（同一 module 的兩個 physical
    channel 共用同一個 DDR buffer），找一個 buffer 長度 N，讓梳狀波的
    諧波間距（`bin_stride * bin_hz`）跟 Single 頻率（`k_single * bin_hz`）
    同時精確落在頻率格點 `bin_hz = dac_fs/N` 上——**不要求 N 是任何特定
    週期的整數倍**，這是取代 `wave_len_for_freq_multiple_of()` 的新設計
    （2026-08-21，Opus 深度分析後的建議方案）。

    核心原理：buffer 長度 N 本身就決定了唯一能無縫播放的頻率格點集合
    `{k*dac_fs/N : k 為整數}`。「IFFT 內容必須整數倍 tile」只是「梳狀波
    每根諧波都落在格點上」這個要求的一種特例（tile j 次時自動成立），
    不是必要條件——只要梳狀波跟 Single 頻率各自的整數週期數落在同一個
    N 上，兩者可以完全不相關的 N 也能同時精確表示。

    兩邊的精確度要求刻意不對稱：Single 頻率使用者會直接量測，用較嚴的
    容差；梳狀波「間距」本來就有先例可以有一定誤差（`precompute_full_
    sum()` 的 N=200192 版本，間距誤差達 960 ppm，頻譜儀比對確認分辨不
    出差異），用較鬆的容差——這個不對稱提供了搜尋的自由度。依序嘗試容差
    階梯 (1ppm,100ppm)→(10ppm,1000ppm)→(100ppm,1000ppm)→(100ppm,5000ppm)，
    每一階都找「離 256 倍數不超過 burst_tol 的最小 N」，第一階成功就用
    （越嚴格的階梯通常對應越小的 N）。全部階梯都失敗時**不 raise**，
    回傳「Single 頻率誤差最小」的候選並標記 `degraded=True`（唯一真的
    raise 的情況是 freq_hz 低於這個 max_samples 下的物理下限）。

    Returns dict：
      n：buffer 長度（含 pad_to_burst() 前的真實長度，當 wave_len 用）
      k_single：Single 頻率的整數週期數
      single_actual_hz / single_ppm：Single 頻率實際值/誤差
      bin_hz：頻率格點間距 dac_fs/n（render_comb_waveform() 的 exclude
        頻率轉 bin 編號要用這個，不能用 step_actual_hz——bin_stride>1
        時兩者不相等，這是最容易漏掉、不會報錯但會挖錯 notch 位置的陷阱）
      bin_stride：梳狀波每個諧波之間相隔幾個格點（compute_comb_bins()
        的新參數，取代舊版恆為 1 的隱含假設）
      step_actual_hz / step_ppm：梳狀波間距實際值/誤差
      degraded：True 代表沒有階梯真的達標，這是盡力而為的降級結果
    """
    if max_samples < BURST_SAMPLES:
        raise ValueError(f"max_samples={max_samples} too small (< BURST_SAMPLES={BURST_SAMPLES})")

    L_all = np.arange(BURST_SAMPLES, max_samples + 1, dtype=np.int64)
    waste = (-L_all) % BURST_SAMPLES
    mask = waste <= burst_tol
    L_cand = L_all[mask]

    k_single = np.round(L_cand * freq_hz / dac_fs)
    k_step = np.round(L_cand * step_hz / dac_fs)
    valid = (k_single > 0) & (k_step > 0)

    if not np.any(valid):
        raise ValueError(
            f"Cannot represent {freq_hz/1e3:.3f} kHz at all within {max_samples} samples "
            f"(DDR waveform path's physical floor here is ~{dac_fs/max_samples:.1f} Hz)."
        )

    actual_single = k_single * dac_fs / L_cand
    actual_step = k_step * dac_fs / L_cand
    single_ppm = np.abs(actual_single - freq_hz) / freq_hz * 1e6
    step_ppm = np.abs(actual_step - step_hz) / step_hz * 1e6

    def _result(idx, degraded):
        return {
            "n": int(L_cand[idx]),
            "k_single": int(k_single[idx]),
            "single_actual_hz": float(actual_single[idx]),
            "single_ppm": float(single_ppm[idx]),
            "bin_hz": dac_fs / float(L_cand[idx]),
            "bin_stride": int(k_step[idx]),
            "step_actual_hz": float(actual_step[idx]),
            "step_ppm": float(step_ppm[idx]),
            "degraded": degraded,
        }

    for max_error_ppm, step_error_ppm in ((1, 100), (10, 1000), (100, 1000), (100, 5000)):
        ok_mask = valid & (single_ppm <= max_error_ppm) & (step_ppm <= step_error_ppm)
        if np.any(ok_mask):
            idx = np.argmax(ok_mask)  # 第一個 True（L_cand 由小到大排序，優先選最小 N）
            return _result(idx, degraded=False)

    valid_idx = np.where(valid)[0]
    best = valid_idx[np.argmin(single_ppm[valid_idx])]
    return _result(best, degraded=True)


def pad_to_burst(samples, burst_samples=BURST_SAMPLES):
    """Pad sample list to next multiple of BURST_SAMPLES (256) with 0V.

    DDR reader issues 64-beat bursts = 256 samples each. A wave_len that is
    not a multiple of 256 triggers a shorter tail burst on the last
    iteration, reducing fill rate below drain rate and risking FIFO
    underflow. Padding with 0 is only correct for waveforms where trailing
    silence is acceptable -- for pure tones use wave_len_for_freq() instead.

    Returns a new list; original is not modified.
    """
    rem = len(samples) % burst_samples
    if rem == 0:
        return list(samples)
    return list(samples) + [0] * (burst_samples - rem)


# ── Board 名冊（跟 post_flash_checklist.py 保持同一份，board 名冊變動
#    時兩邊都要改）───────────────────────────────────────────────────────
# (serial, board_id, is_master) — board_id 是 BOARD_MAP 指定的固定值，
# 但跑過 post_flash_checklist.py 的 Phase 8b（T_BOARD_ID_ASSIGN 廣播）
# 之後，實際 board_id 會依環路位置（board_index）重新分配，不保證還跟
# 這裡一致（本專案目前環路方向下 board B/board C 會互換）——用 --dest 指定板子
# 的腳本，要用當下讀回的 board_id，不能假設這份表。
BOARD_MAP = [
    ("BOARD_A_SERIAL", 0x0000, True),
    ("BOARD_B_SERIAL", 0x0001, False),
    ("BOARD_C_SERIAL", 0x0002, False),
]
