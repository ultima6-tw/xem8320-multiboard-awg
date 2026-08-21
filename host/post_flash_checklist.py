"""
post_flash_checklist.py — 燒錄/reload 後標準初始化流程（2026-07-18 新增）

背景：2026-07-18 連續踩到兩次「肉眼看起來像新 bug，其實是舊 session/
舊測試留下的殘留狀態，沒有被系統性檢查出來」：
  1. `scale_cfg`（flash sector 201）三塊板子讀回互不相同（0xFF/0x3C/
     0x55），查出來是兩次不同的舊測試腳本寫入 flash 的測試殘留值，不是
     真正校準值
  2. `dac_output_mux`（WI 0x1D，DDR/sine 模式選擇）在完整重新燒錄之後，
     board A 播放電壓異常小，是這個 session 稍早測 sine_gen 功能留下的模式
     殘留（根因細節沒有 100% 查清楚，強制寫 0 後問題消失）

見 PROJECT.md 2026-07-18 章節、[[feedback_post_flash_state_checklist]]。
這支腳本把整套「燒錄/reload 後、開始功能測試前」該做的檢查/初始化
收斂成一次執行，未來每次燒錄完固定先跑這支，不要憑假設「應該是預設值」
就直接開始測試。

涵蓋範圍：
  1. Si5332 狀態確保 ACTIVE（不符合才重寫，si5332_configure.py 內建
     idempotent 檢查）
  2. 偵測連接的板子，比對已知 board 名冊（BOARD_MAP，board 名冊變動時
     改 awg_common.py）
  3. 每片板子：channel_up 檢查
  4. 每片板子：ext_clk 訊號頻率量測（確認 Si5332 訊號有送達、頻率正確）
  6. **2026-07-28 新增（原本 2026-07-27 移除，這次為了不同目的復活）**：
     本機直送 T_BOARD_ID_ASSIGN 給 master，搶先解開 is_master/enum
     死結，見下方「is_master/enum 雞生蛋死結」說明
  7. 從 master 觸發 Aurora enum，驗證 total_boards/board_index 正確
  8b. 廣播 T_BOARD_ID_ASSIGN——套用 board_id（依環路位置，board_index）
      + 觸發硬體除法器算出 per_hop_value 並分送全部板子 + 指定
      is_master。**2026-07-27 改版**：board_id/is_master 統一收斂到
      這個封包（見下方說明），這支腳本原本 Phase 6（本機 WI 0x0D+
      TI bit15 直寫）已整個移除。**⚠️ 副作用不變**：board_id 執行完後
      會變成依環路位置分配，不再是 BOARD_MAP 的固定值（本專案目前
      環路方向下 board B/board C 會互換）——之後任何用 --dest 指定板子的腳本，
      要用這支腳本印出的新 board_id，不能假設 BOARD_MAP 的值
  5. **2026-07-28 改到這裡（原本排第 5、在 enum 之前）**：每片板子
     dac_clk_mux 切外部時脈——見下方 Phase 5 函式前的完整根因說明，
     排在 enum/board_id_assign 之後才能通過 relayfifo 的 src_id
     合理性檢查
  9. 每片板子：dac_output_mux/amp_ctrl_mux 模式強制歸零（WI 0x1D <- 0，
     DDR 播放模式、關閉 amp ramp）——這是 2026-07-18 踩到的坑，不能只
     假設它會自動回預設值
  9. 每片板子：讀回 scale_cfg/amp_ctrl/calib_coef，跨板比對，**只報告
     差異，不自動覆寫**（校準值以後可能是真的有意義的，不是這支腳本
     該自作主張洗掉的東西）

**2026-07-27 追加修正**：Phase 5（ext_clk_sel 切換）跟 Phase 9
（dac_mode_ramp 歸零）原本用的本機 `WI 0x18`+TI bit8、`WI 0x1D`+TI bit9
這兩條路徑，**其實也已經隨統一讀取/寫入架構收斂拔除**（見 PORTS.md
表2 bit8/bit9 條目）——這兩個不是 PROJECT.md 記載「刻意保留本機 only」
的 per_hop_value（TI bit10）那一類，第一版改寫時看錯了，误把它們當
成沒被這次收斂納入。已改成廣播版 `T_EXT_CLK_SEL`(0x28)/`T_DAC_MODE_
RAMP`(0x2A)（從 master 送一次 broadcast，套用到全部板子，語意跟原本
「全部板子做同一件事」一致，不需要像 is_master 那樣拆成 unicast）。

**2026-07-27 統一讀取/寫入架構收斂，board_id/is_master 流程改版**：
原本 Phase 6 用本機 `WI 0x0D`(`fp_board_cfg_wr`) + TI bit15 直寫
board_id/is_master，這條路徑已從 `board_cfg_reg.v` 拔除（收斂到
`T_BOARD_ID_ASSIGN`(0x1E) 一條入口），Phase 6 整個移除。is_master
改成在 Phase 8b 的 `T_BOARD_ID_ASSIGN` 廣播裡一併決定——但這個欄位是
單一 broadcast 封包裡的一個 bit，所有收到的板子會拿到同一個值，沒辦法
一次廣播就讓 master=1、slave=0。做法：先廣播一次 is_master=0（幫全部
板子把 board_id 設成 board_index，is_master 暫時全部是 0），再對
master 那片板子送一個 **unicast**（dest_id=master 自己剛分配到的
board_id）is_master=1 的封包——`dispatcher.v` 只有「原生發起」（host
從 USB 送出）才會替換 beat1，這次 dest_id 不是 broadcast，只有目標
板子自己的 `local_reg_handler` 會處理（RT_LOCAL），不會被轉送出去
干擾其他板子的 is_master。詳見 `rtl/dispatcher.v` 2026-07-27 修正
註解（`PROJECT.md`「board_id / is_master 機制」章節有記錄一個相關
bug：這個替換邏輯原本會把 is_master 永遠清成 0，已修好）。

**2026-07-28 新發現：is_master/enum 雞生蛋死結，Phase 6 復活修好**：
`rtl/aurora_ctrl_channel.v` 的 enum/trigger/reserve/GO 觸發全部被
`init_start_pulse && is_master` 這類條件擋住（line 525/574/596/815），
但這次改版後 is_master 唯一的入口 `T_BOARD_ID_ASSIGN` 只能在 Phase 8b
（enum 之後）廣播設定——is_master 沒設就不給 enum 跑、enum 沒跑完
就設不了 is_master，死結。修法：enum 之前，新增 Phase 6，只對 master
這片板子走它自己的 USB **本機直送**（不是 broadcast）一次
`T_BOARD_ID_ASSIGN`，`dest_id` 用它目前的 `board_id`（enum 前的
reset 預設值 0xFFFE），is_master=1。`au_board_id_assign_value`（實際
寫入的 board_id）來源是 `aurora_ctrl_channel.v` 的 `board_index` reg
（不是封包內容），reset 預設值剛好是 0——master 該有的正確值，不會
設錯。`dispatcher.v` 只看 `dest_id==目前 board_id` 判斷 RT_LOCAL，
不會被送上 Aurora，天生只命中物理上接這條 USB 的那片板子，不需要
board_id 先分化。詳見 `NOTES.md` 2026-07-28「獨立發現：enum 完全無法
觸發」章節。

用法：
  python post_flash_checklist.py
"""
import sys, time, subprocess

sys.path.insert(0, r"C:\path\to\awg-test-step-16\host")
from awg_common import open_board, wo, ti, send_pkt, send_beat0_pkt, T_SINE_CTRL, T_TRIG_START, \
    PARAM_MODE, PARAM_RAMP_EN, \
    send_query, read_status_reply, \
    decode_rt_board_info, decode_aurora_status, decode_enum_status, decode_hard_err, \
    decode_qt_calib_status, BOARD_MAP, \
    WO_RT_BOARD_INFO, WO_AURORA_STATUS, WO_ENUM_STATUS, WO_EXT_CLK_FREQ, WO_HARD_ERR, \
    T_ENUM_START, T_BOARD_ID_ASSIGN, T_EXT_CLK_SEL, \
    DEST_BCAST, QT_CALIB_STATUS, TI_BIT_FLUSH_STANDBY
import ok

EXT_CLK_EXPECT_HZ   = 100_000_000
EXT_CLK_TOLERANCE   = 0.02   # +-2%

SI5332_PROJECT = r"C:\path\to\Si5332-GM1-RevD-EX_BL-3out-Project.slabtimeproj"
SI5332_CONFIGURE_PY = r"C:\path\to\awg-test-step-16\host\si5332_configure.py"


# ── Phase 0: Si5332 ──────────────────────────────────────────────────────

def phase0_si5332():
    print("=== [0] Si5332 狀態確保 ACTIVE ===")
    ret = subprocess.run(
        [sys.executable, SI5332_CONFIGURE_PY, "--project", SI5332_PROJECT],
        capture_output=True, text=True)
    print(ret.stdout.strip())
    if ret.returncode != 0:
        print(ret.stderr.strip())
        sys.exit("[X] Si5332 設定失敗，中止")
    print()


# ── Phase 1: 偵測連接的板子 ──────────────────────────────────────────────

def phase1_detect():
    print("=== [1] 偵測連接的板子 ===")
    devs = ok.FrontPanelDevices()
    n = devs.GetCount()
    connected = [devs.GetSerial(i) for i in range(n)]
    print(f"  連接到的裝置: {connected}")

    known_serials = {s for s, _, _ in BOARD_MAP}
    unknown = [s for s in connected if s not in known_serials]
    if unknown:
        print(f"  [!] 有連接但不在 BOARD_MAP 名冊裡的板子，會被忽略: {unknown}")

    active = [(s, bid, m) for s, bid, m in BOARD_MAP if s in connected]
    missing = [s for s, _, _ in BOARD_MAP if s not in connected]
    if missing:
        print(f"  [!] 名冊裡但沒偵測到的板子: {missing}")

    masters = [s for s, _, m in active if m]
    if len(masters) != 1:
        sys.exit(f"[X] 預期剛好 1 片 master，實際偵測到 {len(masters)} 片: {masters}，中止")

    print(f"  本次要跑的板子: {[s for s, _, _ in active]}（master={masters[0]}）")
    print()
    return active


# ── Phase 2: 開啟所有板子連線 ────────────────────────────────────────────

def phase2_open_all(active):
    print("=== [2] 開啟所有板子連線 ===")
    boards = {}
    for serial, board_id, is_master in active:
        dev, fp = open_board(serial)
        if fp is None:
            sys.exit(f"[X] 無法開啟 {serial}，中止")
        boards[serial] = {"dev": dev, "fp": fp, "board_id": board_id, "is_master": is_master}
        print(f"  {serial}: OK")
    print()
    return boards


# ── Phase 3: channel_up 檢查 ─────────────────────────────────────────────

def phase3_channel_up(boards):
    print("=== [3] channel_up 檢查 ===")
    all_ok = True
    for serial, b in boards.items():
        status = decode_aurora_status(wo(b["fp"], WO_AURORA_STATUS))
        ok_flag = status["channel_up_0"] == 1 and status["channel_up_1"] == 1
        all_ok &= ok_flag
        print(f"  {serial}: channel_up_0={status['channel_up_0']} channel_up_1={status['channel_up_1']}  {'OK' if ok_flag else '[X] FAIL'}")
    if not all_ok:
        sys.exit("[X] 有板子 channel_up 沒有全部為 1，中止（Aurora GT 鏈路沒有正常建立）")
    print()


# ── Phase 4: ext_clk 頻率量測 ────────────────────────────────────────────

def phase4_ext_clk_freq(boards, interval=0.3):
    print("=== [4] ext_clk 訊號頻率量測 ===")
    all_ok = True
    for serial, b in boards.items():
        fp = b["fp"]
        cnt0 = wo(fp, WO_EXT_CLK_FREQ)
        time.sleep(interval)
        cnt1 = wo(fp, WO_EXT_CLK_FREQ)
        delta = (cnt1 - cnt0) & 0xFFFFFFFF
        freq_hz = delta / interval
        err = abs(freq_hz - EXT_CLK_EXPECT_HZ) / EXT_CLK_EXPECT_HZ
        ok_flag = err <= EXT_CLK_TOLERANCE
        all_ok &= ok_flag
        print(f"  {serial}: ext_clk ~ {freq_hz/1e6:.4f} MHz  {'OK' if ok_flag else '[X] FAIL (超出 +-2%容許範圍)'}")
    if not all_ok:
        sys.exit("[X] 有板子 ext_clk 頻率異常，中止（檢查 Si5332 輸出/SMA 接線）")
    print()


# ── Phase 5: 切外部時脈 ──────────────────────────────────────────────────
# 2026-07-27 改版：本機 WI 0x18+TI bit8 直寫路徑已拔除，改用廣播版
# T_EXT_CLK_SEL(0x28)。
#
# 2026-07-28 改順序（根因找到）：這個 Phase 原本排在 enum(Phase 7)
# 之前，因為當時假設 board_id 分配前後跟這個廣播無關。但
# `aurora_data_channel_relayfifo.v` 2026-07-21 加的 src_id 合理性檢查
# （`beat0_src_id < effective_max_boards`，enum 前 `effective_max_
# boards` 保守設 4）會把 enum 前 `board_id` 還是開機預設值（`0xFFFE`）
# 時送出的廣播封包，在每一個中繼站都判定成「src_id 不合理」直接丟棄
# （本地送達+轉送兩條路都需要 `beat0_src_valid`，兩者一起失敗）——
# 這正是這次 Phase 5 卡住兩跳/一跳板子的真正根因，不是仲裁器或 CDC
# 問題。改成排在 Phase 6/7/8b（is_master+enum+board_id_assign）**之後**
# 執行，此時 `board_id`/`total_boards` 都已經是正常小數字，src_id
# 合理性檢查會正常通過。ext_clk 切換本身跟 enum 沒有硬性先後依賴
# （enum 走的是 Aurora GT 自己的 `aurora_clk`，不受 DAC `ext_clk_sel`
# 影響），純粹是排序問題，不需要改 RTL。完整診斷過程見 NOTES.md
# 2026-07-28「T_EXT_CLK_SEL 真正根因」章節。

def phase5_switch_ext_clk(boards, master_serial):
    print("=== [5] dac_clk_mux 切外部時脈（開機階段直接切，不等 LED）===")
    fp = boards[master_serial]["fp"]
    ti(fp, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)
    send_pkt(fp, DEST_BCAST, T_EXT_CLK_SEL, beat1_lo=1)
    time.sleep(0.05)

    all_ok = True
    for serial, b in boards.items():
        info = decode_rt_board_info(wo(b["fp"], WO_RT_BOARD_INFO))
        ok_flag = info["ext_clk_sel"] == 1
        all_ok &= ok_flag
        print(f"  {serial}: ext_clk_sel readback={info['ext_clk_sel']}  {'OK' if ok_flag else '[X] FAIL'}")
    if not all_ok:
        sys.exit("[X] ext_clk_sel 切換失敗，中止")
    print()


# ── Phase 6: 本機直送 T_BOARD_ID_ASSIGN 給 master，搶先解開 is_master/enum 死結 ──
# 2026-07-28 新增：見檔頭「is_master/enum 雞生蛋死結」說明。dest_id 用
# master 目前的 board_id（enum 前是 reset 預設值 0xFFFE），確保 dispatcher
# 判成 RT_LOCAL（本機直送，不上 Aurora），只有物理上接這條 USB 的 master
# 板子會處理。beat1 只設 is_master=1（bit37，beat1_hi bit5），per_hop_
# value/total_boards 兩個欄位留 0——這個階段 enum 還沒跑，這兩個值本來
# 就沒意義，Phase 8b 之後會被正確的廣播值覆蓋。

def phase6_set_master_local(boards, master_serial):
    print("=== [6] 本機直送 T_BOARD_ID_ASSIGN 給 master（搶先解開 is_master/enum 死結）===")
    fp = boards[master_serial]["fp"]
    info = decode_rt_board_info(wo(fp, WO_RT_BOARD_INFO))
    local_dest = info["board_id"]  # 目前的 board_id（enum 前是 reset 預設值），走本機 loopback
    ti(fp, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)
    send_pkt(fp, local_dest, T_BOARD_ID_ASSIGN, beat1_lo=0, beat1_hi=(1 << 5))
    time.sleep(0.05)

    info = decode_rt_board_info(wo(fp, WO_RT_BOARD_INFO))
    ok_flag = info["is_master"] == 1
    print(f"  {master_serial}: board_id=0x{info['board_id']:04X}  is_master={info['is_master']}  "
          f"{'OK' if ok_flag else '[X] FAIL'}")
    if not ok_flag:
        sys.exit("[X] master is_master 提前設定失敗，中止")
    boards[master_serial]["board_id"] = info["board_id"]
    print()


# ── Phase 7+8: enum ──────────────────────────────────────────────────────

def phase7_8_enum(boards, master_serial, n_boards):
    print("=== [7] Aurora enum ===")
    fp = boards[master_serial]["fp"]
    ti(fp, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)
    send_beat0_pkt(fp, DEST_BCAST, T_ENUM_START)
    time.sleep(0.1)

    status = decode_enum_status(wo(fp, WO_ENUM_STATUS))
    print(f"  master({master_serial}): init_ok={status['init_ok']} total_boards={status['total_boards']}")
    if not (status['init_ok'] == 1 and status['total_boards'] == n_boards):
        sys.exit(f"[X] enum 失敗（預期 total_boards={n_boards}），中止")

    print("=== [8] 逐板驗證 board_index ===")
    for serial, b in boards.items():
        status = decode_enum_status(wo(b["fp"], WO_ENUM_STATUS))
        print(f"  {serial}: board_index={status['board_index']}  (board_id=0x{b['board_id']:04X})")
    print()


# ── Phase 8b: T_BOARD_ID_ASSIGN 廣播（載入跨板延遲補償 + 指定 master）───

def phase8b_board_id_assign(boards, master_serial):
    print("=== [8b] T_BOARD_ID_ASSIGN 廣播（套用 board_index、載入 per_hop_value 跨板延遲補償、指定 is_master）===")
    # 2026-07-26 新增：這個封包除了把 board_id 覆寫成 board_index（環路
    # 位置），還會 piggyback 觸發硬體除法器算出 per_hop_value 並分送給
    # 全部板子——au_trig_delay 全自動補償機制的必經步驟。
    #
    # ⚠️ 副作用：board_id 會從上面 BOARD_MAP 指定的固定值，改成依環路
    # 位置（board_index）分配的值——不再保證跟 BOARD_MAP 一致（本專案
    # 目前的環路方向下，board B/board C 的 board_id 會互換）。下面已經重新讀回
    # 並更新 `boards[serial]["board_id"]`，之後任何用 --dest 指定板子
    # 的腳本，都要用這裡印出的新 board_id，不能再假設 BOARD_MAP 的值。
    #
    # 2026-07-27 新增：is_master 也收斂進這個封包（見檔頭說明），但這是
    # 單一 broadcast 裡的一個 bit，全部板子會拿到同一個值——先廣播一次
    # is_master=0（幫全部板子把 board_id 設成 board_index），再對 master
    # 那片板子單獨送一個 unicast is_master=1 的封包，不會影響其他板子。
    fp = boards[master_serial]["fp"]
    ti(fp, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)
    send_pkt(fp, DEST_BCAST, T_BOARD_ID_ASSIGN, beat1_lo=0)
    time.sleep(0.1)

    per_hop_values = set()
    for serial, b in boards.items():
        # 2026-07-30 修正：這裡原本用 fp 當迴圈變數，會覆寫掉上面
        # `fp = boards[master_serial]["fp"]`（master 自己的 handle），
        # 導致迴圈跑完後下面的 unicast 送出動作誤用了「boards 字典裡
        # 最後一個板子」的 fp（BOARD_MAP 順序下是 board C），不是 master
        # 自己——board C（slave）原生發起送到 board A 的封包會被 dispatcher.v
        # 判定要送上 Aurora，且用 board C 自己（slave，恆為 0）的
        # per_hop_value/total_boards 覆寫 beat1，污染 board A 收到的值。
        # 詳見 NOTES.md 2026-07-30「查出並用實測確認：master 查自己
        # per_hop_value 讀到 0 的根因」章節。改用獨立變數名避免覆寫。
        b_fp = b["fp"]
        info = decode_rt_board_info(wo(b_fp, WO_RT_BOARD_INFO))
        b["board_id"] = info["board_id"]
        per_hop_eff = decode_hard_err(wo(b_fp, WO_HARD_ERR))["per_hop_value_eff"]
        per_hop_values.add(per_hop_eff)
        print(f"  {serial}: board_id=0x{info['board_id']:04X} (依環路位置重新分配)  "
              f"per_hop_value_eff={per_hop_eff}")
    if len(per_hop_values) != 1:
        sys.exit(f"[X] per_hop_value_eff 三片板子不一致 {per_hop_values}，中止")
    if 0 in per_hop_values:
        sys.exit("[X] per_hop_value_eff 仍為 0，補償沒有生效，中止")
    print(f"  [OK] per_hop_value_eff 全部一致（{per_hop_values.pop()}），跨板延遲補償已載入")

    # is_master 的 unicast 步驟：dest_id = master 剛剛依 board_index
    # 重新分配到的 board_id（這裡就是 fp 自己，走 loopback）。beat1[37]
    # 拆成 beat1_hi 的 bit5（37-32=5，見 rtl/local_reg_handler.v 解碼）。
    master_new_id = boards[master_serial]["board_id"]
    send_pkt(fp, master_new_id, T_BOARD_ID_ASSIGN, beat1_lo=0, beat1_hi=(1 << 5))
    time.sleep(0.1)
    all_ok = True
    for serial, b in boards.items():
        info = decode_rt_board_info(wo(b["fp"], WO_RT_BOARD_INFO))
        expect_master = int(serial == master_serial)
        ok_flag = info["is_master"] == expect_master
        all_ok &= ok_flag
        print(f"  {serial}: is_master={info['is_master']} (expect {expect_master})  {'OK' if ok_flag else '[X] FAIL'}")
    if not all_ok:
        sys.exit("[X] is_master 指定失敗，中止")
    print()


# ── Phase 9: 模式選擇類暫存器強制歸零 ────────────────────────────────────
# 2026-08-05 改版：dac_mode_ramp 併入 sine_ctrl_regs.v（idle/active 雙
# 緩衝，per-module/per-channel 獨立定址），退役的 T_DAC_MODE_RAMP(0x2A)
# 改用廣播版 T_SINE_CTRL（PARAM_MODE=7 逐一歸零 4 個 module、
# PARAM_RAMP_EN=6 逐一歸零 8 個 channel），見 rtl/sine_ctrl_regs.v 檔頭
# + board_ctrl.py set_dac_mode()/set_ramp_en() 說明。這個階段所有板子
# 都已經有 board_id（Phase 8b 之後才跑），但這裡本來就是要「全部板子
# 做同一件事」，廣播比逐片 loopback 簡單且少一次 per-board 迴圈延遲，
# 沿用跟 Phase 5 一樣的設計。
#
# ⚠️ 雙緩衝設計下，寫入只會進到目前 idle 側，不會立刻反映在 active
# 側（下面讀回驗證讀的正是 active 側，見 create_bd.tcl 的 mode_active/
# ramp_en_active -> dac_mode_ramp_concat_0 -> WO_RT_BOARD_INFO 接線）。
# 光「寫0+觸發一次」不夠持久——寫的那個 0 只會待在剛變成 idle 的那一側，
# 另一側仍是舊的殘留值，下一次任何不相干的觸發（例如日後真正的播放
# trigger）都會把值切回那個舊殘留值（用 sim/tb_dac_mode_ramp.v 的 B3
# 段落驗證過這個現象，也用 iverilog 驗證過下面「寫→觸發→再寫」兩次
# 都寫 0 才能讓兩側都變 0、之後不管再觸發幾次都不會反彈，見該檔案
# C0-C4 段落 + NOTES.md 2026-08-05 對應章節）。這裡因此寫兩次、中間
# 廣播一次 T_TRIG_START（group_select=0xF，涵蓋全部 4 個 group，不
# 依賴各 module 目前被分派到哪個 group——group_id_a/b/c/d 預設也是
# 0，但這裡不假設預設值沒被改過）。這個時間點（enum/board_id 已確定，
# 還沒有任何波形被 arm）廣播 trigger 是否安全，已用 Opus 讀
# aurora_ctrl_channel.v 的 PAUSE/ACK/GO 狀態機 + 既有的
# tb_trig_negotiation_timing_10board.v／tb_full_aurora_roundtrip.v
# 模擬先例覆核確認安全（is_master/total_boards/board_id 等前置條件
# 在 Phase 8b 就已經settle，且有「enum 完成、什麼都還沒 arm」時觸發
# 成功完成的既有模擬先例）。

def _write_mode_ramp_zero(fp):
    for module in range(4):
        sel = (PARAM_MODE << 3) | (module << 1)
        send_pkt(fp, DEST_BCAST, T_SINE_CTRL, beat1_lo=0, beat1_hi=sel & 0x3F)
        time.sleep(0.01)
    for channel in range(8):
        sel = (PARAM_RAMP_EN << 3) | channel
        send_pkt(fp, DEST_BCAST, T_SINE_CTRL, beat1_lo=0, beat1_hi=sel & 0x3F)
        time.sleep(0.01)


def phase9_force_ddr_mode(boards, master_serial):
    print("=== [9] dac_output_mux/amp_ctrl_mux 模式強制歸零（T_SINE_CTRL PARAM_MODE/PARAM_RAMP_EN <- 0，寫→觸發→再寫，兩側都清零）===")
    fp = boards[master_serial]["fp"]
    ti(fp, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)
    _write_mode_ramp_zero(fp)
    send_pkt(fp, DEST_BCAST, T_TRIG_START, beat1_lo=0xF)
    time.sleep(0.05)
    _write_mode_ramp_zero(fp)
    time.sleep(0.05)

    all_ok = True
    for serial, b in boards.items():
        info = decode_rt_board_info(wo(b["fp"], WO_RT_BOARD_INFO))
        ok_flag = info["dac_mode_ramp"] == 0
        all_ok &= ok_flag
        print(f"  {serial}: dac_mode_ramp readback=0x{info['dac_mode_ramp']:03X}  {'OK' if ok_flag else '[X] FAIL'}")
    if not all_ok:
        sys.exit("[X] dac_output_mux/amp_ctrl_mux 模式歸零失敗，中止")
    print()


# ── Phase 10: 校準設定讀回比對（只報告，不覆寫）─────────────────────────
# 2026-07-27 改版：calib_coef 原本用 WI 0x0B（calib_sel）逐一選 index
# 讀 WO 0x37，但這條路徑已隨統一讀取/寫入架構收斂退役（WI 0x0B 已無法
# 指定任意 index，見 PORTS.md 0x0b 條目）。改用 T_QUERY(QT_CALIB_STATUS)
# + PO_STATUS_REPLY 一次拿回 scale_cfg/amp_ctrl/calib_coef 全部 32 筆，
# 這是官方文件記載的正式取代方案。

def read_calib(fp, board_id):
    ti(fp, TI_BIT_FLUSH_STANDBY)
    time.sleep(0.01)
    send_query(fp, board_id, QT_CALIB_STATUS)
    time.sleep(0.05)
    src, query_type, data_words = read_status_reply(fp)
    if query_type != QT_CALIB_STATUS:
        sys.exit(f"[X] QT_CALIB_STATUS 查詢回覆的 query_type 不符（收到 {query_type}），中止")
    return decode_qt_calib_status(data_words)


def phase10_calib_report(boards):
    print("=== [10] 校準設定讀回比對（scale_cfg/amp_ctrl/calib_coef，只報告差異，不覆寫）===")
    results = {serial: read_calib(b["fp"], b["board_id"]) for serial, b in boards.items()}
    serials = list(boards.keys())
    ref_serial = serials[0]
    ref = results[ref_serial]

    for serial in serials:
        r = results[serial]
        print(f"  {serial}: scale_cfg=0x{r['scale_cfg']:02X}")

    any_diff = False
    for serial in serials[1:]:
        r = results[serial]
        diffs = []
        if r["scale_cfg"] != ref["scale_cfg"]:
            diffs.append(f"scale_cfg: 0x{ref['scale_cfg']:02X} vs 0x{r['scale_cfg']:02X}")
        for i, (a, c) in enumerate(zip(ref["amp_ctrl"], r["amp_ctrl"])):
            if a != c:
                diffs.append(f"amp_ctrl[{i}]: 0x{a:05X} vs 0x{c:05X}")
        for i, (a, c) in enumerate(zip(ref["calib_coef"], r["calib_coef"])):
            if a != c:
                diffs.append(f"calib_coef[{i}]: 0x{a:05X} vs 0x{c:05X}")
        if diffs:
            any_diff = True
            print(f"  [!] {serial} 跟 {ref_serial} 不一致:")
            for d in diffs:
                print(f"       {d}")
    if not any_diff:
        print("  各板 scale_cfg/amp_ctrl/calib_coef 完全一致。")
    else:
        print("  [!] 上面列出的差異可能是真正的校準值，也可能是舊測試殘留——"
              "這支腳本不會自動覆寫，需要人工判斷後用對應的 flash 寫入腳本處理。")
    print()


def main():
    phase0_si5332()
    active = phase1_detect()
    boards = phase2_open_all(active)
    master_serial = next(s for s, b in boards.items() if b["is_master"])

    phase3_channel_up(boards)
    phase4_ext_clk_freq(boards)
    phase6_set_master_local(boards, master_serial)
    phase7_8_enum(boards, master_serial, len(boards))
    phase8b_board_id_assign(boards, master_serial)
    phase5_switch_ext_clk(boards, master_serial)
    phase9_force_ddr_mode(boards, master_serial)
    phase10_calib_report(boards)

    print("=== 初始化流程完成，板子已就緒可以開始功能測試 ===")


if __name__ == "__main__":
    main()
