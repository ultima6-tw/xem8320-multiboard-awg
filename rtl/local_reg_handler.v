`timescale 1ns/1ps
`default_nettype none

// local_reg_handler v1 — Step 14
//
// 讀取 Local reg async FIFO（aurora_clk → sys_clk），解析封包，產生控制信號。
// 取代 step-12 aurora_packet_rx.v 中 0x01-0x12 的本機處理邏輯。
//
// 優點：在 sys_clk domain 直接輸出 1-cycle pulse，無需 toggle-sync。
//
// ── 輸入 FIFO 格式（與 Dispatcher local 輸出相同）─────────────────────────────
//   beat0：[63:40]=pkt_len, [39:32]=type, [31:16]=src_id, [15:0]=dest_id
//   beat1..N：依 type 定義（格式與 aurora_packet_rx.v 相同）
//
// ── 不處理的 type ─────────────────────────────────────────────────────────────
//   0x13 T_WAVEFORM_STREAM → Dispatcher 已路由到 DDR path，不會進入此 FIFO
//
// 2026-07-27（統一讀取/寫入架構收斂，見 PROJECT.md「統一讀取/寫入架構 —
// 完整規格」小節）：
//   - T_BOARD_CFG(0x10) 整個刪除——確認從未被任何腳本用來設定過
//     is_master，純粹死路，功能跟擴充後的 T_BOARD_ID_ASSIGN 重複。
//   - T_BOARD_ID_ASSIGN(0x1E) 新增 beat1[37]=is_master 欄位，讓它同時
//     觸發 board_id（沿用 board_index）跟 is_master（host 明確指定）。
//   - T_QUERY(0x12) 從 1-beat 無 payload 改成 2-beat，beat1[7:0]=
//     query_type（QT_BOARD_INFO=0/QT_SINE_STATUS=1/QT_CALIB_STATUS=2/
//     QT_TRIGGER_GROUP=3），一次只問一種。
//   - T_STATUS_REPORT(0x11) 從固定 2-beat/32-bit flags 改成通用可變長度
//     累加器：beat1[7:0]=query_type（其餘 beat1 保留不用），beat2..N
//     每個 beat 64-bit 依序塞進 `au_reply_data`（依 query_type 由消費端
//     解讀內容，這裡不解析欄位語意）。最大支援 REPLY_MAX_WORDS 個
//     64-bit word，由 QT_SINE_STATUS（8 channel × 6 個 32-bit 參數 +
//     mux_sel + phase_acc ≈ 1800 bit）決定所需大小。
//
// 2026-07-29 新增（T_QUERY 上機失敗，拆解寫入端/讀出端診斷，見 NOTES.md
// 同日對應章節）：
//   - `ti_debug_reply_trig`（TI bit，診斷專用，取代原本規劃/實作過又
//     拔掉的 T_DEBUG_REPLY_INJECT(0x1F) 封包機制——使用者要求最直接的
//     觸發方式，不要經過封包/dispatcher/case 解碼這些中間步驟）：TI
//     bit 觸發時完全繞過 aurora_reply_tx.v/aurora_tx1_arbiter/實體
//     Aurora 環路，直接用寫死常數驅動 au_reply_wr/au_reply_src
//     (=0xDEAD)/au_reply_query_type(=0x7F)/au_reply_data（每個 word =
//     {32'hDEB1F000, word_index}）。host 可以用 PO_STATUS_REPLY（驗證
//     完整 60-word payload）或直接讀 WO（`au_reply_src`/`au_reply_
//     query_type` 這 24-bit 有另外接進 concat_reserve_diag，見
//     create_bd.tcl）驗證讀出端。

module local_reg_handler #(
    // 2026-07-27 新增：T_STATUS_REPORT 通用累加器容量（64-bit word 數），
    // 30 words = 1920 bit，足夠裝下最大的 QT_SINE_STATUS payload
    // （8 channel × 6 個 32-bit 參數 + mux_sel + phase_acc ≈ 1800 bit）
    parameter REPLY_MAX_WORDS = 30
) (
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF rx" *)
    input  wire        sys_clk,
    input  wire        sys_rst,
    input  wire        half_reset,    // 診斷 capture 同步 reset（與 fp_input 對齊）

    // 輸入（async FIFO 讀側，sys_clk）
    input  wire [63:0] rx_tdata,
    input  wire        rx_tvalid,
    output wire        rx_tready,   // combinatorial，永遠接受（無 backpressure）

    // ── 控制信號輸出（sys_clk，1-cycle pulse）─────────────────────────────────
    output reg         calib_wr_en,
    output reg [4:0]   calib_wr_addr,
    output reg [17:0]  calib_wr_data,
    output reg         calib_rst_out,

    output reg         au_ddr4_wr,
    output reg [31:0]  au_ddr4_addr,
    output reg [31:0]  au_ddr4_w0,
    output reg [31:0]  au_ddr4_w1,
    output reg [31:0]  au_ddr4_w2,
    output reg [31:0]  au_ddr4_w3,

    output reg         au_list_wr,
    output reg [31:0]  au_list_sel,
    output reg [31:0]  au_list_addr,
    output reg [31:0]  au_list_len,

    output reg         au_play_ctrl_wr,
    output reg [31:0]  au_play_ctrl,

    output reg         au_scale_cfg_wr,
    output reg [31:0]  au_scale_cfg,

    output reg         au_amp_ctrl_wr,
    output reg [3:0]   au_amp_ch_sel,
    output reg [17:0]  au_amp_val,

    // Step 12 新增
    output reg         au_flush_standby,
    output reg         au_timer_ctrl_wr,
    output reg [1:0]   au_timer_ctrl_port,
    output reg         au_timer_ctrl_run,
    output reg         au_timer_ctrl_loop,
    output reg [4:0]   au_timer_ctrl_depth,
    output reg [3:0]   au_timer_ctrl_slot,
    output reg [31:0]  au_timer_ctrl_intv,
    // T_GROUP_SCHED_CTRL(0x2C，2026-08-04 新增，trigger group 排程功能
    // 三部曲「C」)：group_trig_scheduler_0（每板一個）的控制路徑，
    // 3-beat 格式比照 T_TIMER_CTRL，多一個 group_sel 欄位（這個排程
    // 器沒有「port」概念，每板只有一個實例，不像 trig_timer 要選
    // z0-z3）。純 Aurora 路徑，沒有 FP WireIn 直寫（比照 2026-07-27
    // 統一讀取/寫入架構的既有慣例，這是全新功能不需要相容舊的本機
    // 直寫路徑）。2026-08-20 新增 arm_mode（beat1[15]，多板同步輪播
    // Architecture B，見 rtl/group_trig_scheduler.v 檔頭說明）——
    // beat1[14:0] 原本就用滿到 bit 14，bit 15 是唯一還空著的欄位。
    output reg         au_group_sched_wr,
    output reg         au_group_sched_run,
    output reg         au_group_sched_loop,
    output reg [4:0]   au_group_sched_depth,
    output reg [3:0]   au_group_sched_slot,
    output reg [3:0]   au_group_sched_group_sel,
    output reg [31:0]  au_group_sched_intv,
    output reg         au_group_sched_arm_mode,
    // Group-based Trigger 架構（2026-07-27 新增，取代原本的
    // au_trig_mask_wr/au_trig_mask_val 機制）：T_TRIG_MASK(0x0D) 重新
    // 定義成「設定某個 group 涵蓋哪些模組」，2-beat，
    // beat1[5:0]={group_id[1:0],channel_mask[3:0]}，channel_mask 是
    // [A,B,C,D]（bit3=A/bit2=B/bit1=C/bit0=D）。轉換成「每個模組屬於
    // 哪個 group」的邏輯在 board_cfg_reg.v 做，這裡只負責解碼原始值。
    output reg         au_group_cfg_wr,
    output reg [1:0]   au_group_cfg_group_id,
    output reg [3:0]   au_group_cfg_mask,
    output reg         au_flash_erase,

    // T_FLASH_TARGET_SEL (0x15，2-beat，beat1[1:0]=目標 sector，2026-07-15
    // 新增：Flash「分開儲存」——送 T_FLASH_WRITE_DATA/存檔前，先用這個封包
    // 指定這次要動哪個 64KB sector（0=身份/1=scale_cfg/2=amp_ctrl/
    // 3=calib_coef），master 才能遠端存另一片板子的單一欄位群組，不用碰到
    // 其他群組）
    output reg         au_flash_target_sel_wr,
    output reg [1:0]   au_flash_target_sel,

    // T_QUERY (0x12，2026-07-27 起 2-beat)：產生 req_reply（觸發
    // aurora_reply_tx 送 T_STATUS_REPORT），reply_dest_id 是查詢方
    // board_id（beat0 pkt_src_id，跟 req_reply 同一拍鎖存），
    // req_query_type 是 beat1[7:0] 帶的查詢種類
    output reg         req_reply,
    output reg [15:0]  reply_dest_id,
    output reg [7:0]   req_query_type,

    // 2026-07-07 (step 15b) 新增：host 觸發 Aurora 多板協定（enum/trigger/
    // reserve）原本是 TI bit 29/30/31 直接接線，改成跟其他控制命令一樣走
    // 封包（host→dispatcher→這裡解碼），拿掉 TI bit 直接接線這條旁路，
    // 統一 host 溝通入口都經過 dispatcher。
    // T_ENUM_START (0x1A，1-beat，無 payload)
    output reg         au_enum_start,
    // T_TRIG_START (0x1B，2026-07-27 起改成 2-beat，Group-based Trigger
    // 架構）：beat1[3:0]=group_select（這次要 fire 哪幾個 group，bit
    // 對應 group 0-3），跟 au_trig_start pulse 同一拍鎖存輸出
    output reg         au_trig_start,
    output reg [3:0]   au_trig_group_select,
    // T_REINIT (0x27，1-beat，無 payload，2026-07-24 新增)：「初始化」
    // 指令（清空 playlist/trigger list/DDR4 + 立即 0V）原本只有 TI bit17
    // （host 本機直連），slave 真實部署下沒有 USB 就完全沒有辦法讓它
    // 緊急歸零——安全層級考量，是 USB-only 功能盤點裡優先權最高的一項
    // （見 NOTES.md「USB-only 功能盤點」章節）。跟 ti_reinit/Dout 在
    // create_bd.tcl 用 OR gate 合併，兩條路徑都能觸發同一組
    // reinit_pulse_cdc_0/ddr_zero_writer_0 目的地。
    output reg         au_reinit,
    // T_EXT_CLK_SEL (0x28，2-beat，beat1[0]=值，2026-07-24 新增)：USB-only
    // 功能盤點第 2 項——`board_cfg_reg.v` 把 `ext_clk_sel` 改成 TriggerIn
    // 觸發式（跟 board_id 同一套慣例），這裡是 Aurora 端的觸發來源，見
    // `board_cfg_reg.v` 的 `au_ext_clk_sel_wr`/`au_ext_clk_sel` port 註解。
    output reg         au_ext_clk_sel_wr,
    output reg         au_ext_clk_sel,
    // T_SINE_CTRL (0x29，2-beat，beat1[31:0]=data、beat1[37:32]=sel，
    // 2026-07-24 新增)：USB-only 功能盤點第 3 項——sine_gen/amp_ramp_gen
    // 參數（tuning_word/phase/start_amp/step/duration_cycles/loop_mode ×
    // 8 個 channel）走的是 sine_ctrl_regs.v 既有的「selector+data+
    // strobe」匯流排介面（同 amp_ctrl 慣例），這裡直接把 sel/data
    // 原封不動送過去，優先權 mux 在 sine_ctrl_regs.v 內部做（比照
    // board_cfg_reg.v 慣例），不在這裡處理。
    output reg         au_sine_wr,
    output reg [5:0]   au_sine_sel,
    output reg [31:0]  au_sine_data,
    // T_SINE_LIST_CTRL (0x2D，2-beat，beat1[31:0]=data、beat1[39:32]=sel，
    // 2026-08 新增)：Sine mode N-slot 排程功能，寫入 sine_ctrl_regs.v 新增
    // 的 per-channel slot table。sel[7:0]={slot_idx[1:0],param_sel[2:0],
    // ch_sel[2:0]}——跟 T_SINE_CTRL 的 6-bit sel 是獨立的命名空間（這裡
    // param_sel 額外多一個值 6=commit list_depth 並 arm，語意跟
    // T_SINE_CTRL 的 param_sel 6/7=ramp_en/mode 無關，純粹是兩個不同封包
    // 各自的欄位）。原封不動交給 sine_ctrl_regs.v，優先權由該檔案內部
    // 的統一 dac_clk write-routing 處理，這裡不做仲裁。
    output reg         au_sine_list_wr,
    output reg [7:0]   au_sine_list_sel,
    output reg [31:0]  au_sine_list_data,
    // T_DAC_MODE_RAMP (0x2A) 已於 2026-08-05 退役——dac_mode_ramp 併入
    // rtl/sine_ctrl_regs.v，改用 T_SINE_CTRL（param_sel=6/7），見該檔案
    // 檔頭說明。這裡不再解碼 0x2A。
    // T_MANUAL_TOTAL_BOARDS (0x2B，2-beat，beat1[4:0]=值，2026-07-24
    // 新增)：單板 bench test 用——host 手動宣告「這個環路只有 N 片板
    // 子」，繞過 enum（不需要實體 SFP 連線/channel_up 就能用）。0=停用
    // （沿用 enum 算出的真正 total_boards），非 0=覆寫。純本機用途，
    // 沒有跟任何 host 來源競爭優先權，不需要額外的 wr/active 旗標，
    // 直接鎖存值即可（見 aurora_ctrl_channel.v total_boards_eff 註解）。
    output reg [4:0]   au_manual_total_boards = 5'd0,
    // T_RESERVE_START (0x1C，2-beat，beat1[15:0]=目的地 board_id，取代原本
    // WI 0x11 reserve_dest_id_slice 暫存器）
    output reg         au_reserve_start,
    output reg [15:0]  au_reserve_dest_id,

    // T_DIAG_START (0x1F，2-beat，beat1[15:0]=起點 board_id，2026-08-20
    // Phase 7 新增）：enum 失敗時的斷點定位診斷，host 觸發後由
    // aurora_ctrl_channel.v 沿環送出 TYPE_DIAG_REQ 查詢。0x1F 原本是
    // 已拔除的 T_DEBUG_REPLY_INJECT，號碼空出可用（見 awg_common.py
    // 該常數註解），跟 T_RESERVE_START(0x1C) 完全同一套 2-beat pattern，
    // 只是目的欄位改叫 dest_id 沿用既有命名習慣。
    output reg         au_diag_start,
    output reg [15:0]  au_diag_dest_id,

    // T_BOARD_ID_ASSIGN (0x1E，1-beat，無 payload，2026-07-14 step-16 新增）：
    // 觸發 board_cfg_reg.v 把 board_id 設成目前的 board_index（enum 結果）。
    // 不是 enum 完成就自動觸發，是獨立指令，讓使用者先確認 enum 結果正確
    // 再決定要不要套用。動機：未來部署只接 master 一條 USB，slave 完全沒有
    // USB 可以手動下 set_board_cfg_direct.py，board_id 要能透過 Aurora
    // 廣播遠端產生。
    output reg         au_board_id_assign,

    // T_TRIG_DELAY_CFG (0x1D，2-beat，beat1[15:0]=延遲量，2026-07-07 step
    // 15b 新增，每板獨立設定觸發前的延遲，手動補償 Aurora hop 數造成的
    // 觸發時間差）。2026-07-24 trigger 統一化架構改版：這個模組只單純
    // 鎖存原始 16-bit 值，不解讀單位——單位語意（改成 aurora_clk cycle
    // 數，不做頻率轉換）在消費端 aurora_ctrl_channel.v/manual_trig_
    // delay_in 決定，見該檔案 port 註解。
    output reg         au_trig_delay_wr,
    output reg [15:0]  au_trig_delay,

    // 2026-07-24 新增（trigger 統一化架構改版）：sticky flag，au_trig_delay
    // 曾經被 T_TRIG_DELAY_CFG 封包手動設過值之後永遠維持 1（直到
    // sys_rst）。BD 層 CDC 回 aurora_clk 給
    // aurora_ctrl_channel_0/manual_trig_delay_active_in，決定要不要覆寫
    // 自動算出的補償值（見該檔案 port 註解）。
    //
    // 2026-07-24 移除開機 flash 載入路徑（原本 flash_load_valid/
    // flash_trig_delay 兩個 port）：上機實測發現這條路徑每次開機都會
    // 自動觸發，永久鎖住這個 sticky flag，導致新做的自動計算補償永遠
    // 沒有機會生效（模擬 sim/tb_flash_boot_blocks_auto_trig_delay.v
    // 已確認）。使用者決定拿掉，開機永遠優先用自動計算，flash 裡的
    // trig_delay 舊值不再被讀取使用（PROJECT.md/NOTES.md 有完整記錄）。
    output reg         manual_delay_override_active,

    // 2026-07-23 再追加：au_trig_delay 全自動校準機制的 trigger 準確度
    // 改版——原本這裡算 au_trig_delay（×16÷25 頻率換算）+ 用
    // trig_delay_cnt 在 sys_clk 倒數，最後要跨到 dac_clk 才能用，實測
    // 發現跨兩次時脈域（aurora_clk->sys_clk 收封包、sys_clk->dac_clk
    // 出給 DAC，且 dac_clk 選外部 Si5332 時跟 sys_clk 是兩個無鎖相關係
    // 的獨立振盪器）殘留相位差每次觸發都不一樣。改成整個計算 + 倒數都
    // 搬到 aurora_ctrl_channel.v（aurora_clk domain）做，只跨一次域到
    // dac_clk。這裡只需要把 T_BOARD_ID_ASSIGN beat1 收到的原始值（不
    // 算 hops，不做頻率換算）交出去，讓 BD 層 CDC 回 aurora_clk。
    // au_trig_delay/au_trig_delay_wr/T_TRIG_DELAY_CFG(0x1D) 手動路徑
    // 維持不變（獨立於這個自動機制之外，供手動測試/覆寫用）。
    output reg  [31:0] bid_per_hop_value,
    output reg  [4:0]  bid_total_boards,
    // 2026-07-27 新增：T_BOARD_ID_ASSIGN 擴充帶入的 is_master（beat1[37]），
    // 跟 au_board_id_assign 同一拍鎖存，見 board_cfg_reg.v
    // au_board_id_assign_is_master port 註解
    output reg         bid_is_master,
    // 2026-07-30 新增：這次 T_BOARD_ID_ASSIGN 是不是 broadcast（beat0
    // dest_id==0xFFFF）。根因：broadcast 一次只能帶一個共用的 is_master
    // 值，enum 完成後套用 board_index 的那次 broadcast 會把全部板子
    // （含剛設好的 master）的 is_master 一起洗成同一個值——board_cfg_
    // reg.v 收到 broadcast 時應該只更新 board_id，不要動 is_master
    // （is_master 只由明確指定目標板子的 unicast 封包設定，見該檔案
    // au_board_id_assign_is_broadcast port 註解、NOTES.md 對應章節）。
    output reg         bid_is_broadcast,

    // 2026-07-30 新增：跨板讀取 QT_BOARD_INFO 用的 init_ok 等效訊號
    // ——aurora_ctrl_channel.v 原本的 init_ok/total_boards 只有 master
    // 執行 enum 才會被寫入，slave 這兩個 output register 永遠停在
    // reset 值 0，導致跨板查詢 slave 的 total_boards/init_ok 永遠讀回
    // 0（見 NOTES.md 2026-07-30「Windows 上機驗收：build12」章節根因
    // 分析）。total_boards 有現成的 bid_total_boards 可以借用（每片
    // 板子都收得到），但 init_ok 沒有天生對應的訊號，這裡新增一個
    // sticky register：語意明確是「這片板子有沒有收過 T_BOARD_ID_
    // ASSIGN」，跟 bid_total_boards 同一拍（收到 beat1）鎖存成 1，
    // sys_rst 才清回 0——不用 bid_total_boards!=0 這種組合邏輯代替，
    // 避免跟「total_boards 欄位本身的值」這個不同概念混淆（使用者
    // 2026-07-30 確認選這個方向）。
    output reg         bid_init_ok,

    // T_STATUS_REPORT (0x11，2026-07-27 起通用可變長度累加器，見檔頭說明)：
    // 收到查詢回覆時觸發，query_type 標籤 + 依 query_type 解讀的
    // payload（au_reply_data，最大 REPLY_MAX_WORDS 個 64-bit word）
    output reg         au_reply_wr,
    output reg [15:0]  au_reply_src,        // beat0 src_id = 回覆方 board_id
    output reg [7:0]   au_reply_query_type, // beat1[7:0]
    output wire [REPLY_MAX_WORDS*64-1:0] au_reply_data,

    // 2026-07-29 新增（取代已移除的 T_DEBUG_REPLY_INJECT(0x1F) 封包
    // 機制，見檔頭同日說明）：TI bit 直接觸發，完全獨立於上面的封包
    // 解碼 state machine 之外，不用組封包/不經過 dispatcher 路由/不經過
    // case 解碼，是最少步驟的診斷寫入路徑。跟封包版本一樣直接驅動
    // au_reply_wr/au_reply_src/au_reply_query_type/reply_payload_words，
    // 讓 host 可以用 PO_STATUS_REPLY（驗證完整 60-word payload）或直接
    // 讀 WO（驗證 au_reply_src/au_reply_query_type 這 24-bit，見
    // create_bd.tcl concat_reserve_diag 新增的接線）兩種方式驗證。
    input  wire        ti_debug_reply_trig,

    // 2026-08 新增：T_QUERY 回覆本機捷徑（修 PROJECT.md 記錄的已知設計
    // 缺口——查自己時原本一定要讓回覆封包真的送出 Aurora TX、繞完整個
    // 環路回到自己的 RX 才會被解碼寫入這幾個暫存器，SFP 沒接或環路沒
    // 通時永遠讀不到回覆）。aurora_reply_tx_0 發現 T_QUERY 的
    // reply_dest_id 就是自己的 board_id 時，直接把這批訊號駁接過來，
    // 完全比照 ti_debug_reply_trig 的寫入方式（同一個 always block、
    // 同樣的 au_reply_wr/au_reply_src/au_reply_query_type/
    // reply_payload_words 四個目標），不進 ST_HDR/QTYPE/PAYLOAD 那個
    // state machine，不碰實體 Aurora TX。兩個模組都在 sys_clk domain，
    // 不需要 CDC，見 aurora_reply_tx.v 對應章節。
    input  wire        local_reply_valid,
    input  wire [15:0] local_reply_src,
    input  wire [7:0]  local_reply_query_type,
    input  wire [REPLY_MAX_WORDS*64-1:0] local_reply_payload,

    // 2026-07-27：diag_lrh_bcfg_wr（原本是 au_board_cfg_wr 的 sticky
    // latch）已隨 T_BOARD_CFG(0x10) 一併刪除

    // 診斷：N4 節點 — rx_tvalid 曾觸發（= async_fifo_local 有資料到達）
    output reg         diag_lrh_rx_seen,
    output reg  [31:0] diag_lrh_first_lo,   // 第一個 rx_tdata[31:0]
    output reg  [31:0] diag_lrh_first_hi,   // 第一個 rx_tdata[63:32]

    // 診斷：rx beats 0-5 捕捉 + DDR4 write 輸出捕捉（reset on sys_rst || half_reset）
    // [i*64+:32] = rx beat i [31:0]（lo），[i*64+32+:32] = [63:32]（hi），i=0..5
    // [384+:32]=wr_addr, [416+:32]=wr_w0, [448+:32]=wr_w1, [480+:32]=wr_w2, [512+:32]=wr_w3
    output wire [543:0] diag_lrh_data
);

    // ── 狀態機 ────────────────────────────────────────────────────────────────
    localparam ST_IDLE = 1'b0;
    localparam ST_DATA = 1'b1;

    reg        state    = ST_IDLE;
    reg [23:0] remaining;   // 剩餘 beats（beat0 消費後設為 pkt_len-1，每 beat 遞減）
    reg [7:0]  pkt_type;
    reg [15:0] pkt_src_id;
    reg [15:0] pkt_dest_id;   // 2026-07-30 新增：T_BOARD_ID_ASSIGN 判斷 broadcast/unicast 用
    reg [7:0]  beat_idx;    // ST_DATA 中的 beat 編號（從 1 開始，對應 beat1..N）

    // 暫存（跨 beat 保留）
    reg [31:0] ddr4_addr_l;
    reg [31:0] ddr4_w0_l, ddr4_w1_l, ddr4_w2_l;
    reg [31:0] list_sel_l, list_addr_l;

    // 2026-07-27 新增：T_STATUS_REPORT 通用累加器（見檔頭說明），
    // beat2..N 的原始 64-bit 內容依序存入，跟 diag_lrh_data 一樣用
    // generate 攤平成單一 output wire
    reg [63:0] reply_payload_words [0:REPLY_MAX_WORDS-1];
    genvar rpi;
    generate
        for (rpi = 0; rpi < REPLY_MAX_WORDS; rpi = rpi + 1) begin : g_reply_payload
            assign au_reply_data[rpi*64 +: 64] = reply_payload_words[rpi];
        end
    endgenerate
    reg [3:0]  timer_ctrl_slot_l;
    reg [4:0]  timer_ctrl_depth_l;
    reg        timer_ctrl_run_l, timer_ctrl_loop_l;
    reg [1:0]  timer_ctrl_port_l;
    // T_GROUP_SCHED_CTRL(0x2C) latch，見上方 output port 宣告處說明
    reg [3:0]  group_sched_slot_l;
    reg [4:0]  group_sched_depth_l;
    reg        group_sched_run_l, group_sched_loop_l;
    reg [3:0]  group_sched_group_sel_l;
    reg        group_sched_arm_mode_l;   // 2026-08-20 新增，見上方 output port 宣告處說明
    integer    dbg_reply_i; // T_DEBUG_REPLY_INJECT(0x1F) 用，展開成固定迴圈
    integer    loc_reply_i; // local_reply_valid（本機捷徑）用，同樣展開成固定迴圈

    // 所有 pulse 輸出的 macro reset（避免冗長重複）
    task clear_pulses;
        begin
            calib_wr_en      <= 1'b0;
            calib_rst_out    <= 1'b0;
            au_ddr4_wr       <= 1'b0;
            au_list_wr       <= 1'b0;
            au_play_ctrl_wr  <= 1'b0;
            au_scale_cfg_wr  <= 1'b0;
            au_amp_ctrl_wr   <= 1'b0;
            au_flush_standby <= 1'b0;
            au_timer_ctrl_wr <= 1'b0;
            au_group_sched_wr <= 1'b0;
            au_group_cfg_wr  <= 1'b0;
            au_flash_erase   <= 1'b0;
            au_flash_target_sel_wr <= 1'b0;
            req_reply        <= 1'b0;
            au_reply_wr      <= 1'b0;
            au_enum_start    <= 1'b0;
            au_trig_start    <= 1'b0;
            au_reinit        <= 1'b0;
            au_ext_clk_sel_wr <= 1'b0;
            au_sine_wr        <= 1'b0;
            au_sine_list_wr   <= 1'b0;
            au_reserve_start <= 1'b0;
            au_trig_delay_wr <= 1'b0;
            au_board_id_assign <= 1'b0;
            au_diag_start    <= 1'b0;
        end
    endtask

    // 無 backpressure：永遠接受（async FIFO rd_en = rx_tvalid && rx_tready = rx_tvalid）
    assign rx_tready = 1'b1;

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            state <= ST_IDLE;
            clear_pulses;
            // 2026-07-30 新增：bid_total_boards/bid_per_hop_value/
            // bid_is_master/bid_is_broadcast 這 4 個 reg 原本沒有任何
            // reset 路徑，只在收到 T_BOARD_ID_ASSIGN 才第一次被賦值。
            // 真實硬體上 FPGA 暫存器一定有 bitstream INIT 值（預設 0），
            // 不受影響；但 iverilog 模擬預設是 'x'，而 bid_total_boards
            // 會經 CDC 餵進 aurora_rx_beat0_gate.v 的 total_boards_in，
            // X 會讓 effective_max_boards/src_id_valid/dest_id_valid
            // 全部變成 'x'，導致 enum 還沒跑完之前，連第一個 ENUM_COUNT
            // 封包本身都會被 gate 擋下（'x' 在 if 條件式視為 false）——
            // 這是純模擬環境的雞生蛋死結，不是真實硬體行為。補上明確
            // reset 讓 tb_ring_rtt.v/tb_group_based_trigger.v 這類直接
            // 把 total_boards_in 接到真實 bid_total_boards（不像其他
            // 測試台圖方便 tie 常數 0）的測試台不再卡住。
            bid_total_boards  <= 5'd0;
            bid_per_hop_value <= 32'd0;
            bid_is_master     <= 1'b0;
            bid_is_broadcast  <= 1'b0;
            bid_init_ok       <= 1'b0;
        end else begin
            // 每 cycle 預設清除所有 pulse 輸出
            clear_pulses;

            // 2026-07-29 新增：ti_debug_reply_trig，完全獨立於下面的封包
            // 解碼 state machine 之外，不管目前 state 是什麼都能觸發（跟
            // 封包來源互斥使用即可，不會真的同時發生）。
            if (ti_debug_reply_trig) begin
                au_reply_src        <= 16'hDEAD;
                au_reply_query_type <= 8'h7F;
                au_reply_wr         <= 1'b1;
                for (dbg_reply_i = 0; dbg_reply_i < REPLY_MAX_WORDS; dbg_reply_i = dbg_reply_i + 1)
                    reply_payload_words[dbg_reply_i] <= {32'hDEB1_F000, dbg_reply_i[31:0]};
            end

            // 2026-08 新增：T_QUERY 回覆本機捷徑，跟上面 ti_debug_reply_trig
            // 同一種獨立注入模式（不進封包解碼 state machine），見上方
            // port 宣告處說明。
            if (local_reply_valid) begin
                au_reply_src        <= local_reply_src;
                au_reply_query_type <= local_reply_query_type;
                au_reply_wr         <= 1'b1;
                for (loc_reply_i = 0; loc_reply_i < REPLY_MAX_WORDS; loc_reply_i = loc_reply_i + 1)
                    reply_payload_words[loc_reply_i] <= local_reply_payload[loc_reply_i*64 +: 64];
            end

            case (state)

                // ── ST_IDLE：消費 beat0 ────────────────────────────────────
                ST_IDLE: begin
                    if (rx_tvalid && (rx_tdata[63:40] != 24'd0)) begin
                        // pkt_len=0 guard：FIFO reset 後 BRAM 初值全 0 會造成 remaining 溢位（0-1=0xFFFFFF）
                        pkt_type    <= rx_tdata[39:32];
                        pkt_src_id  <= rx_tdata[31:16];
                        pkt_dest_id <= rx_tdata[15:0];

                        if (rx_tdata[63:40] == 24'd1) begin
                            // 單 beat 封包（T_ENUM_START 0x1A, T_REINIT
                            // 0x27 — 都無 payload，byte0 消費完就直接
                            // 觸發，不需要進 ST_DATA）。
                            // T_BOARD_ID_ASSIGN(0x1E) 2026-07-23 起改成
                            // 2-beat（見下方 ST_DATA case 8'h1E），beat1
                            // 由 dispatcher.v 自動填入 per_hop_value，
                            // 不會再走這個 1-beat 分支。
                            // T_TRIG_START(0x1B) 2026-07-27 起改成 2-beat
                            // （Group-based Trigger 架構，見下方 ST_DATA
                            // case 8'h1B），不會再走這個 1-beat 分支。
                            // T_QUERY(0x12) 2026-07-27 起改成 2-beat（見
                            // 下方 ST_DATA case 8'h12），不會再走這個
                            // 1-beat 分支。
                            if (rx_tdata[39:32] == 8'h1A)
                                au_enum_start <= 1'b1;
                            else if (rx_tdata[39:32] == 8'h27)
                                au_reinit <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            remaining <= rx_tdata[63:40] - 24'd1;
                            beat_idx  <= 8'd1;
                            state     <= ST_DATA;
                        end
                    end
                end

                // ── ST_DATA：消費 beat1..N ─────────────────────────────────
                ST_DATA: begin
                    if (rx_tvalid) begin
                        remaining <= remaining - 24'd1;

                        case (pkt_type)

                            // 2026-07-24 移除：8'h01（TRIGGER）branch——trigger
                            // 統一化架構改版，這個 type 已經沒有任何 host 腳本
                            // 會送（合成 TRIGGER 封包注入機制在 aurora_ctrl_
                            // channel.v 早就停用，見該檔案 2026-07-23 註解），
                            // 移除後 0x01 落到 default: ; 安全 no-op。

                            8'h02: begin // CALIB_WR（2-beat）
                                if (remaining == 24'd1) begin
                                    calib_wr_addr <= rx_tdata[4:0];
                                    calib_wr_data <= rx_tdata[22:5];
                                    calib_wr_en   <= 1'b1;
                                end
                            end

                            8'h03: // CALIB_RST（2-beat）
                                if (remaining == 24'd1) calib_rst_out <= 1'b1;

                            8'h04: begin // DDR4_WRITE（6-beat）
                                case (beat_idx)
                                    8'd1: ddr4_addr_l <= rx_tdata[31:0];
                                    8'd2: ddr4_w0_l   <= rx_tdata[31:0];
                                    8'd3: ddr4_w1_l   <= rx_tdata[31:0];
                                    8'd4: ddr4_w2_l   <= rx_tdata[31:0];
                                    8'd5: if (remaining == 24'd1) begin
                                        au_ddr4_addr <= ddr4_addr_l;
                                        au_ddr4_w0   <= ddr4_w0_l;
                                        au_ddr4_w1   <= ddr4_w1_l;
                                        au_ddr4_w2   <= ddr4_w2_l;
                                        au_ddr4_w3   <= rx_tdata[31:0];
                                        au_ddr4_wr   <= 1'b1;
                                    end
                                    default: ;
                                endcase
                            end

                            8'h05: begin // LIST_WRITE（4-beat）
                                case (beat_idx)
                                    8'd1: list_sel_l  <= rx_tdata[31:0];
                                    8'd2: list_addr_l <= rx_tdata[31:0];
                                    8'd3: if (remaining == 24'd1) begin
                                        au_list_sel  <= list_sel_l;
                                        au_list_addr <= list_addr_l;
                                        au_list_len  <= rx_tdata[31:0];
                                        au_list_wr   <= 1'b1;
                                    end
                                    default: ;
                                endcase
                            end

                            8'h06: begin // PLAY_CTRL（2-beat）
                                if (remaining == 24'd1) begin
                                    au_play_ctrl    <= rx_tdata[31:0];
                                    au_play_ctrl_wr <= 1'b1;
                                end
                            end

                            8'h07: begin // SCALE_CFG（2-beat）
                                if (remaining == 24'd1) begin
                                    au_scale_cfg    <= rx_tdata[31:0];
                                    au_scale_cfg_wr <= 1'b1;
                                end
                            end

                            // 2026-08-04 移除：8'h08（T_TRIG_SLOT）branch——
                            // 這個封包唯一的消費者是 trig_timer_0（module
                            // z0）的 slot/intv，這條路徑從無 host 端 wrapper
                            // （查證見 rtl/aurora_ctrl_mux.v「Timer hold
                            // registers」章節完整推導），z0 已合併進
                            // T_TIMER_CTRL(0x0C) 跟 z1-z3 用同一套機制，
                            // 這個封包整個退役，0x08 落到 default: ; 安全
                            // no-op（比照 2026-07-24 移除 8'h0B 的既有慣例）。

                            8'h09: begin // AMP_CTRL（2-beat）
                                if (remaining == 24'd1) begin
                                    au_amp_ch_sel  <= rx_tdata[21:18];
                                    au_amp_val     <= rx_tdata[17:0];
                                    au_amp_ctrl_wr <= 1'b1;
                                end
                            end

                            8'h0A: // FLUSH_STANDBY（2-beat）
                                if (remaining == 24'd1) au_flush_standby <= 1'b1;

                            // 2026-07-24 移除：8'h0B（T_TRIG_PORT）branch——
                            // trigger 統一化架構改版，這個機制完全沒有任何
                            // host 腳本在用，是死代碼，移除後 0x0B 落到
                            // default: ; 安全 no-op。

                            8'h0C: begin // TIMER_CTRL（3-beat）
                                // 2026-08-04 beat1 欄位重新排列（10-bit→
                                // 13-bit）：slot 3-bit→4-bit、depth
                                // 3-bit→5-bit（16 slot 排程功能，見
                                // PROJECT.md trigger group 排程功能
                                // 三部曲「B」條目），loop_en/run/port
                                // 往後平移。host 端 board_ctrl.py 的
                                // send_timer_ctrl() 要同步改新版面，
                                // 否則協定對不上。
                                case (beat_idx)
                                    8'd1: begin
                                        timer_ctrl_slot_l  <= rx_tdata[3:0];
                                        timer_ctrl_depth_l <= rx_tdata[8:4];
                                        timer_ctrl_loop_l  <= rx_tdata[9];
                                        timer_ctrl_run_l   <= rx_tdata[10];
                                        timer_ctrl_port_l  <= rx_tdata[12:11];
                                    end
                                    8'd2: if (remaining == 24'd1) begin
                                        au_timer_ctrl_slot  <= timer_ctrl_slot_l;
                                        au_timer_ctrl_depth <= timer_ctrl_depth_l;
                                        au_timer_ctrl_loop  <= timer_ctrl_loop_l;
                                        au_timer_ctrl_run   <= timer_ctrl_run_l;
                                        au_timer_ctrl_port  <= timer_ctrl_port_l;
                                        au_timer_ctrl_intv  <= rx_tdata[31:0];
                                        au_timer_ctrl_wr    <= 1'b1;
                                    end
                                    default: ;
                                endcase
                            end

                            8'h0D: begin // T_TRIG_MASK（2-beat，2026-07-27
                                // 重新定義，Group-based Trigger 架構：設定
                                // 某個 group 涵蓋哪些模組（取代原本的
                                // au_trig_mask_wr/au_trig_mask_val 機制，
                                // 完全不是新增，號碼原地重新賦予新語意）。
                                // beat1[5:0]={group_id[1:0],
                                // channel_mask[3:0]}，channel_mask=
                                // [A,B,C,D]（bit3=A/bit2=B/bit1=C/bit0=D）。
                                // 「mask→group_id」的轉換邏輯在
                                // board_cfg_reg.v 做，這裡只解碼原始值。
                                if (remaining == 24'd1) begin
                                    au_group_cfg_group_id <= rx_tdata[5:4];
                                    au_group_cfg_mask     <= rx_tdata[3:0];
                                    au_group_cfg_wr       <= 1'b1;
                                end
                            end

                            // 2026-07-10：8'h0E(T_FLASH_SAVE)decode 已移除
                            // ——2026-07-09 架構簡化後 host 不再送這個封包，
                            // au_flash_save 變成永遠不會 fire 的死路徑，誤導
                            // 了兩輪 ILA 診斷（見 PROJECT.md），連同 port 一併
                            // 清掉。0x0E 現在是未定義 type，會落到 default
                            // 分支不做任何事。

                            8'h0F: // FLASH_ERASE（2-beat）
                                if (remaining == 24'd1) au_flash_erase <= 1'b1;

                            8'h15: begin // T_FLASH_TARGET_SEL（2-beat，2026-07-15 新增）
                                if (remaining == 24'd1) begin
                                    au_flash_target_sel    <= rx_tdata[1:0];
                                    au_flash_target_sel_wr <= 1'b1;
                                end
                            end

                            // 2026-07-27 移除：8'h10（T_BOARD_CFG）——確認
                            // 從未被任何腳本用來設定過 is_master，純粹死
                            // 路，功能跟擴充後的 T_BOARD_ID_ASSIGN(0x1E)
                            // 重複，移除後 0x10 落到 default: ; 安全 no-op。

                            8'h11: begin // T_STATUS_REPORT（2026-07-27 起
                                // 通用可變長度累加器，見檔頭說明）：
                                // beat1[7:0]=query_type，beat2..N 每個
                                // beat 原封不動塞進 reply_payload_words，
                                // 最後一拍（remaining==1）才 fire
                                // au_reply_wr，資料語意由消費端依
                                // au_reply_query_type 解讀
                                if (beat_idx == 8'd1) begin
                                    au_reply_query_type <= rx_tdata[7:0];
                                end else begin
                                    reply_payload_words[beat_idx - 8'd2] <= rx_tdata;
                                end
                                if (remaining == 24'd1) begin
                                    au_reply_src <= pkt_src_id;
                                    au_reply_wr  <= 1'b1;
                                end
                            end

                            8'h1C: begin // T_RESERVE_START（2-beat，2026-07-07 step 15b 新增）
                                // beat1[15:0] = 目的地 board_id，取代原本
                                // WI 0x11 reserve_dest_id_slice 暫存器
                                if (remaining == 24'd1) begin
                                    au_reserve_dest_id <= rx_tdata[15:0];
                                    au_reserve_start   <= 1'b1;
                                end
                            end

                            8'h1F: begin // T_DIAG_START（2-beat，2026-08-20
                                // Phase 7 新增）：beat1[15:0] = 查詢起點
                                // board_id，跟 T_RESERVE_START(0x1C) 同一套
                                // pattern，只是 dest 欄位改成起點語意
                                if (remaining == 24'd1) begin
                                    au_diag_dest_id <= rx_tdata[15:0];
                                    au_diag_start   <= 1'b1;
                                end
                            end

                            8'h1B: begin // T_TRIG_START（2-beat，2026-07-27
                                // 起，Group-based Trigger 架構）：
                                // beat1[3:0]=group_select（這次要 fire
                                // 哪幾個 group），跟 au_trig_start pulse
                                // 同一拍鎖存輸出，見 aurora_ctrl_channel.v
                                // native_trig_group_r 說明
                                if (remaining == 24'd1) begin
                                    au_trig_group_select <= rx_tdata[3:0];
                                    au_trig_start         <= 1'b1;
                                end
                            end

                            8'h1E: begin // T_BOARD_ID_ASSIGN（2-beat，2026-07-23 改版，
                                // 2026-07-27 再擴充 is_master）
                                // beat1[31:0] = per_hop_value、
                                // beat1[36:32] = total_boards（dispatcher.v
                                // 自動填入，host 送的原始內容不重要，
                                // 格式見 dispatcher.v 的 data_in_used
                                // 註解）。au_board_id_assign 行為不變
                                // （觸發 board_cfg_reg.v 套用
                                // board_index）。2026-07-23 再追加：這裡
                                // 不再算 hops/au_trig_delay（那個計算 +
                                // 倒數整個搬到 aurora_ctrl_channel.v 做，
                                // 見該檔案 native_trig_out port 註解），
                                // 只單純把原始值鎖存出去給 BD 層 CDC 回
                                // aurora_clk。2026-07-27：beat1[37] =
                                // is_master（host 明確指定，不是自動算出
                                // 的欄位），讓這個封包同時決定 board_id
                                // 跟 is_master，成為 board 身份指定的唯一
                                // 入口（取代已刪除的 T_BOARD_CFG）。
                                if (remaining == 24'd1) begin
                                    au_board_id_assign <= 1'b1;
                                    bid_per_hop_value <= rx_tdata[31:0];
                                    bid_total_boards  <= rx_tdata[36:32];
                                    bid_is_master     <= rx_tdata[37];
                                    // 2026-07-30 新增：pkt_dest_id 在 beat0
                                    // 就已經鎖存好了（見 ST_IDLE），這裡直接
                                    // 判斷這次是不是 broadcast。
                                    bid_is_broadcast  <= (pkt_dest_id == 16'hFFFF);
                                    // 2026-07-30 新增：見上方 bid_init_ok
                                    // port 註解——這片板子收過 T_BOARD_ID_
                                    // ASSIGN 就代表「已完成身份指定」，跨板
                                    // 查詢 QT_BOARD_INFO 的 init_ok 欄位用
                                    // 這個 sticky 值，不受 slave 天生沒有
                                    // enum-completed 分支的限制。
                                    bid_init_ok <= 1'b1;
                                end
                            end

                            8'h12: begin // T_QUERY（2026-07-27 起 2-beat，
                                // 見檔頭說明）：beat1[7:0] = query_type，
                                // req_reply 跟 reply_dest_id（查詢方
                                // board_id，來自 pkt_src_id）同一拍鎖存，
                                // 讓 aurora_reply_tx.v 知道要組哪種
                                // T_STATUS_REPORT、送去哪裡
                                if (remaining == 24'd1) begin
                                    req_reply      <= 1'b1;
                                    reply_dest_id  <= pkt_src_id;
                                    req_query_type <= rx_tdata[7:0];
                                end
                            end

                            8'h1D: begin // T_TRIG_DELAY_CFG（2-beat，2026-07-07 step 15b 新增）
                                // beat1[15:0] = 這片板子的 trigger 延遲量
                                // （2026-07-24 起單位是 aurora_clk cycle，
                                // 這裡只單純鎖存原始值，不解讀單位，見
                                // port 宣告處註解），host 手動調整用
                                if (remaining == 24'd1) begin
                                    au_trig_delay    <= rx_tdata[15:0];
                                    au_trig_delay_wr <= 1'b1;
                                end
                            end

                            8'h28: begin // T_EXT_CLK_SEL（2-beat，2026-07-24 新增）
                                // beat1[0] = ext_clk_sel 值（0=內部/1=外部
                                // 時脈），board_cfg_reg.v 的 TriggerIn 觸發
                                // 式暫存器，跟 board_id 同一套慣例
                                if (remaining == 24'd1) begin
                                    au_ext_clk_sel    <= rx_tdata[0];
                                    au_ext_clk_sel_wr <= 1'b1;
                                end
                            end

                            8'h29: begin // T_SINE_CTRL（2-beat，2026-07-24 新增）
                                // beat1[31:0] = data、beat1[37:32] = sel
                                // （6-bit：{param_sel[2:0],ch_sel[2:0]}），
                                // 原封不動交給 sine_ctrl_regs.v 的既有
                                // selector+data+strobe 介面，優先權 mux
                                // 在該檔案內部做
                                if (remaining == 24'd1) begin
                                    au_sine_data <= rx_tdata[31:0];
                                    au_sine_sel  <= rx_tdata[37:32];
                                    au_sine_wr   <= 1'b1;
                                end
                            end

                            8'h2D: begin // T_SINE_LIST_CTRL（2-beat，2026-08 新增）
                                // beat1[31:0] = data、beat1[39:32] = sel
                                // （8-bit：{slot_idx[1:0],param_sel[2:0],
                                // ch_sel[2:0]}），原封不動交給
                                // sine_ctrl_regs.v 新增的 N-slot table 寫入
                                // 介面，見該檔案檔頭 2026-08 改版說明
                                if (remaining == 24'd1) begin
                                    au_sine_list_data <= rx_tdata[31:0];
                                    au_sine_list_sel  <= rx_tdata[39:32];
                                    au_sine_list_wr   <= 1'b1;
                                end
                            end

                            // 8'h2E（T_NOTCH_SWEEP_CTRL）已於 2026-08-19 退役
                            // （IFFT 梳狀波形排除頻率改成 host 端算好再上膛，
                            // 不需要 FPGA 即時扣除，見 PROJECT.md 對應章節），
                            // 不再解碼。

                            // 8'h2A（T_DAC_MODE_RAMP）已於 2026-08-05 退役，
                            // 不再解碼，見上方 port 宣告處說明。

                            8'h2B: begin // T_MANUAL_TOTAL_BOARDS（2026-07-24 新增）
                                // beat1[4:0] = 手動宣告的環路板數（0=停用，
                                // 沿用 enum 算出的值），單板 bench test 用，
                                // 繞過 enum/channel_up，見 aurora_ctrl_
                                // channel.v total_boards_eff 註解
                                if (remaining == 24'd1) begin
                                    au_manual_total_boards <= rx_tdata[4:0];
                                end
                            end

                            8'h2C: begin // T_GROUP_SCHED_CTRL（3-beat，2026-08-04
                                // 新增，trigger group 排程功能三部曲「C」）。
                                // beat1[3:0]=slot(0-15) [8:4]=depth(0-16,5-bit)
                                // [9]=loop_en [10]=run [14:11]=group_sel(0-15)
                                // [15]=arm_mode（2026-08-20 新增，多板同步
                                // 輪播 Architecture B，見 rtl/group_trig_
                                // scheduler.v 檔頭說明）beat2[31:0]=
                                // interval_cycles。跟 T_TIMER_CTRL 同一套
                                // 3-beat 格式，差別是沒有 port 欄位
                                // （group_trig_scheduler 每板只有一個實例），
                                // 多一個 group_sel 欄位。
                                case (beat_idx)
                                    8'd1: begin
                                        group_sched_slot_l      <= rx_tdata[3:0];
                                        group_sched_depth_l     <= rx_tdata[8:4];
                                        group_sched_loop_l      <= rx_tdata[9];
                                        group_sched_run_l       <= rx_tdata[10];
                                        group_sched_group_sel_l <= rx_tdata[14:11];
                                        group_sched_arm_mode_l  <= rx_tdata[15];
                                    end
                                    8'd2: if (remaining == 24'd1) begin
                                        au_group_sched_slot      <= group_sched_slot_l;
                                        au_group_sched_depth     <= group_sched_depth_l;
                                        au_group_sched_loop      <= group_sched_loop_l;
                                        au_group_sched_run       <= group_sched_run_l;
                                        au_group_sched_group_sel <= group_sched_group_sel_l;
                                        au_group_sched_arm_mode  <= group_sched_arm_mode_l;
                                        au_group_sched_intv      <= rx_tdata[31:0];
                                        au_group_sched_wr        <= 1'b1;
                                    end
                                    default: ;
                                endcase
                            end

                            default: ; // 忽略未知 type

                        endcase

                        beat_idx <= beat_idx + 8'd1;

                        if (remaining == 24'd1)
                            state <= ST_IDLE;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

    // 獨立 always block：latch manual_delay_override_active（2026-07-24
    // 新增，trigger 統一化架構改版）。比照下面 diag_lrh_bcfg_wr 的既有
    // sticky-latch 手法，不跟主狀態機 always block 共用同一個 reg 的驅動
    // 來源。au_trig_delay_wr 讀到的是上一拍的 registered 值（跟
    // diag_lrh_bcfg_wr 對 au_board_cfg_wr 的讀法一致，1-cycle 延遲對
    // sticky flag 語意沒有影響）。2026-07-24 移除 flash_load_valid 觸發
    // 這條路徑（見上方 port 宣告處註解）——只有 T_TRIG_DELAY_CFG 封包
    // 才會設這個 sticky flag。
    always @(posedge sys_clk) begin
        if (sys_rst)
            manual_delay_override_active <= 1'b0;
        else if (au_trig_delay_wr)
            manual_delay_override_active <= 1'b1;
    end

    // 2026-07-27：diag_lrh_bcfg_wr always block 已隨 T_BOARD_CFG(0x10)
    // 一併移除（見上方 port 宣告處說明）

    // N4 診斷：記錄第一次有效封包（pkt_len!=0）觸發時的資料
    always @(posedge sys_clk) begin
        if (sys_rst) begin
            diag_lrh_rx_seen  <= 1'b0;
            diag_lrh_first_lo <= 32'd0;
            diag_lrh_first_hi <= 32'd0;
        end else if (rx_tvalid && !diag_lrh_rx_seen && (rx_tdata[63:40] != 24'd0)) begin
            diag_lrh_rx_seen  <= 1'b1;
            diag_lrh_first_lo <= rx_tdata[31:0];
            diag_lrh_first_hi <= rx_tdata[63:32];
        end
    end

    // ── 診斷 capture：rx beats 0-5（sys_clk）────────────────────────────────
    reg [31:0] lrh_rx_cap_lo [0:5];
    reg [31:0] lrh_rx_cap_hi [0:5];
    reg [3:0]  lrh_rx_cap_cnt = 4'd0;

    integer rxi;
    always @(posedge sys_clk) begin
        if (sys_rst || half_reset) begin
            lrh_rx_cap_cnt <= 4'd0;
            for (rxi = 0; rxi < 6; rxi = rxi + 1) begin
                lrh_rx_cap_lo[rxi] <= 32'd0;
                lrh_rx_cap_hi[rxi] <= 32'd0;
            end
        end else if (rx_tvalid && lrh_rx_cap_cnt < 4'd6) begin
            lrh_rx_cap_lo[lrh_rx_cap_cnt] <= rx_tdata[31:0];
            lrh_rx_cap_hi[lrh_rx_cap_cnt] <= rx_tdata[63:32];
            lrh_rx_cap_cnt <= lrh_rx_cap_cnt + 4'd1;
        end
    end

    // ── 診斷 capture：DDR4 write 輸出（au_ddr4_wr 觸發後 1 cycle 捕捉）────
    reg [31:0] lrh_wr_addr_cap = 32'd0;
    reg [31:0] lrh_wr_w0_cap   = 32'd0;
    reg [31:0] lrh_wr_w1_cap   = 32'd0;
    reg [31:0] lrh_wr_w2_cap   = 32'd0;
    reg [31:0] lrh_wr_w3_cap   = 32'd0;

    always @(posedge sys_clk) begin
        if (sys_rst || half_reset) begin
            lrh_wr_addr_cap <= 32'd0;
            lrh_wr_w0_cap   <= 32'd0;
            lrh_wr_w1_cap   <= 32'd0;
            lrh_wr_w2_cap   <= 32'd0;
            lrh_wr_w3_cap   <= 32'd0;
        end else if (au_ddr4_wr) begin
            lrh_wr_addr_cap <= au_ddr4_addr;
            lrh_wr_w0_cap   <= au_ddr4_w0;
            lrh_wr_w1_cap   <= au_ddr4_w1;
            lrh_wr_w2_cap   <= au_ddr4_w2;
            lrh_wr_w3_cap   <= au_ddr4_w3;
        end
    end

    // ── diag_lrh_data 封裝輸出 ───────────────────────────────────────────────
    genvar lgi;
    generate
        for (lgi = 0; lgi < 6; lgi = lgi + 1) begin : g_lrh_rx
            assign diag_lrh_data[lgi*64     +: 32] = lrh_rx_cap_lo[lgi];
            assign diag_lrh_data[lgi*64 + 32 +: 32] = lrh_rx_cap_hi[lgi];
        end
    endgenerate
    assign diag_lrh_data[384 +: 32] = lrh_wr_addr_cap;
    assign diag_lrh_data[416 +: 32] = lrh_wr_w0_cap;
    assign diag_lrh_data[448 +: 32] = lrh_wr_w1_cap;
    assign diag_lrh_data[480 +: 32] = lrh_wr_w2_cap;
    assign diag_lrh_data[512 +: 32] = lrh_wr_w3_cap;

endmodule
`default_nettype wire
