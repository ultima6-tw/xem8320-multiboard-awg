`timescale 1ns/1ps
`default_nettype none

// sine_ctrl_regs.v -- Staging register file for 8 physical channels' sine_gen
// AND amp_ramp_gen parameters, each with an active/idle hardware pair.
//
// Background (2026-07-23 8-channel rewrite): each ZmodAWG module physically
// has 2 channels (ch1/ch2), each of the 8 physical channels gets TWO full
// sine_gen/amp_ramp_gen hardware instances (suffix _a/_b), one active
// (feeding the DAC) and one idle (being staged), true double-buffering.
// Host writes always target whichever hardware instance is currently idle,
// so the active instance is never disturbed by a parameter write. On
// trig_start, mux_sel toggles (active<->idle swap).
//
// ch_sel encoding (3-bit, 0-7): {inst[1:0], is_ch2}, module-major.
//
// 2026-08-05 改版：dac_mode_ramp（DDR/Sine mode + amp_ctrl_mux ramp_en）併入
// 本模組，走 T_SINE_CTRL 既有封包（param_sel 新增 6=ramp_en/7=mode）。
//
// ══════════════════════════════════════════════════════════════════════
// 2026-08 改版（Sine mode N-slot 排程功能，比照 DDR waveform_controller.v
// 的方式，使用者明確要求「用相同的方法比較好，因為 ddr modes 已經有了，
// 不用重新找問題」）：
//
// 1. 整條寫入路徑從 sys_clk 搬到 dac_clk domain：原本 au_sine_sel/data/wr
//    （sys_clk，靠 mux_sel_sync 這顆 2-flop CDC 判斷哪組是 idle）改名
//    dac_sine_sel/data/wr，直接是 dac_clk domain 的訊號（create_bd.tcl 端
//    要在 local_reg_handler_0 的 au_sine_sel/data/wr 輸出跟這裡之間插入
//    trigger_cdc（wr pulse）+ level_cdc（sel/data，準穩態值，host 送完
//    sel/data 兩個欄位後才送 wr pulse，settling time 遠大於 CDC 需要的時
//    間，安全性論證跟本專案既有 timer_wr_cdc_$port/timer_slot_cdc_$port
//    這組已驗證慣例完全一樣，見 create_bd.tcl 對應章節）。這樣一來，
//    write-routing 判斷 idle 側可以直接用 mux_sel（dac_clk 原生值），不用
//    再繞 mux_sel_sync。
//    這個決定的直接後果：tuning_word_stage_X_a/b 等暫存器現在只有 dac_clk
//    一個 always block 在寫（原本 sys_clk 那個 always block 整個搬過來、
//    跟下面的 N-slot 邏輯合併），避免 check_verilog_multidriver.py 要抓的
//    「同一個 reg 被兩個 always block 驅動」問題。
//
// 2. 新增 per-channel N-slot table（N_SLOT=4，見下方 localparam）：每個
//    slot 存 6 個既有參數（tuning_word/phase/start_amp/step/
//    duration_cycles/loop_mode）。這組表格本身不直接驅動 sine_gen/
//    amp_ramp_gen，只在特定時機被複製進正牌的 *_stage_a/_b 暫存器：
//      (a) dac_sine_list_wr 寫入 list_depth（list_param_sel==3'd6）時：
//          視為「commit/arm」——重置 current_idx=0/next_idx=1，同時把
//          slot[0] 直接複製進目前 active 那側、slot[1] 複製進 idle 那
//          側。這一步完全比照 waveform_controller.v 的 ST_CFG_INIT（一
//          次把 current_idx/next_idx 兩個位置都準備好，讓「未來的第一
//          次 trigger」能正確揭露 slot 0）。host 端寫入順序有硬性要求：
//          必須先把這個 channel 要用的每個 slot 內容都寫完，最後才寫
//          list_depth 來 arm/commit（如果順序反過來，arm 那一刻 slot
//          table 內容還沒填，preload 進去的會是舊值/0）。
//      (b) trig_start_$inst 觸發時：跟既有 mux_sel 翻轉同一拍，把
//          slot_table[next_idx] 的內容複製進「剛因為這次 swap 變成
//          idle」的那一側（DDR 對照：waveform_controller.v 用一個多拍
//          的 flush+reload FSM 做同一件事，因為 DDR reader 有真正的記
//          憶體 burst fetch 延遲；sine 這裡是純暫存器寫入，沒有 fetch
//          延遲，直接同一拍完成，不需要對應的 FSM/flush 狀態）。
//    Slot table 本身完全不會被「立即生效」的 T_SINE_CTRL 單值寫入路徑
//    觸碰，兩者是獨立的儲存區——這也是為什麼 T_SINE_CTRL 的既有行為
//    （Bulk Sine Setup 用的立即生效單值寫入）完全不受影響，只是實際寫
//    暫存器的動作現在跟 N-slot 邏輯共用同一個 dac_clk always block。
//
// 3. CDC 安全性論證（沿用專案既有慣例，非新發明）：dac_sine_wr/
//    dac_sine_list_wr 都是 host 送封包觸發的 pulse，host 端固定「先送
//    sel+data 兩個欄位、settling 一段時間、才送 wr pulse」的封包序列
//    （Aurora/USB 序列傳輸的固有延遲遠大於 2-flop 同步器需要的時間），
//    這跟 trig_timer/group_trig_scheduler 已驗證過的 level_cdc（sel/data
//    準穩態值）+ trigger_cdc（wr pulse）模式安全性論證完全相同。不適用
//    這個論證的是「FPGA 內部同一拍產生的瞬態多 bit 資料」（例如
//    2026-08-04 修過的 group_trig_scheduler group_select_out 那個案
//    例，需要改用 FIFO 而不是 level_cdc）——這裡沒有這種情況，因為 slot
//    table 寫入永遠是 host 主動發起、pulse 唯一來源。
//
// CDC notes（既有、未變）：mux_sel 本身仍在 dac_clk 產生，仍然透過標準
// 2-flop synchronizer 帶回 sys_clk 成 mux_sel_sync，純粹給
// aurora_reply_tx.v 的 QT_SINE_STATUS 讀回用，不再用於 write-routing。

module sine_ctrl_regs (
    input  wire        clk,       // sys_clk -- 只剩 mux_sel_sync 這顆 2-flop CDC 還在用
    input  wire        rst,

    input  wire        dac_clk,   // 2026-08 起是本模組唯一的 write-routing domain
    input  wire        dac_rst,

    // 2026-08 改版：au_sine_sel/data/wr 改名 dac_sine_sel/data/wr，從
    // sys_clk 搬到 dac_clk domain（create_bd.tcl 用 level_cdc+trigger_cdc
    // 從 local_reg_handler_0 的 au_sine_sel/data/wr 正規 CDC 過來）。
    // T_SINE_CTRL(0x29) 封包格式完全不變（param_sel 0-5=六個既有參數、
    // 6=ramp_en、7=mode），只是解碼後的訊號現在直接是 dac_clk 側。
    input  wire [5:0]  dac_sine_sel,
    input  wire [31:0] dac_sine_data,
    input  wire        dac_sine_wr,

    // 2026-08 新增：T_SINE_LIST_CTRL 封包（N-slot table 寫入），CDC 過的
    // dac_clk 側訊號。sel[7:0] = {slot_idx[1:0], param_sel[2:0], ch_sel[2:0]}；
    // param_sel 0-5 跟既有 6 個參數同一套編碼（獨立於上面 dac_sine_sel 的
    // param_sel 命名空間，兩者互不影響），6=commit list_depth 並 arm（見
    // 上方檔頭說明），7 未用。
    input  wire [7:0]  dac_sine_list_sel,
    input  wire [31:0] dac_sine_list_data,
    input  wire        dac_sine_list_wr,

    input  wire        trig_start_0,
    input  wire        trig_start_1,
    input  wire        trig_start_2,
    input  wire        trig_start_3,

    output reg [31:0] tuning_word_stage_0_a,
    output reg [31:0] tuning_word_stage_0_b,
    output reg [31:0] tuning_word_stage_1_a,
    output reg [31:0] tuning_word_stage_1_b,
    output reg [31:0] tuning_word_stage_2_a,
    output reg [31:0] tuning_word_stage_2_b,
    output reg [31:0] tuning_word_stage_3_a,
    output reg [31:0] tuning_word_stage_3_b,
    output reg [31:0] tuning_word_stage_4_a,
    output reg [31:0] tuning_word_stage_4_b,
    output reg [31:0] tuning_word_stage_5_a,
    output reg [31:0] tuning_word_stage_5_b,
    output reg [31:0] tuning_word_stage_6_a,
    output reg [31:0] tuning_word_stage_6_b,
    output reg [31:0] tuning_word_stage_7_a,
    output reg [31:0] tuning_word_stage_7_b,
    output reg [31:0] phase_stage_0_a,
    output reg [31:0] phase_stage_0_b,
    output reg [31:0] phase_stage_1_a,
    output reg [31:0] phase_stage_1_b,
    output reg [31:0] phase_stage_2_a,
    output reg [31:0] phase_stage_2_b,
    output reg [31:0] phase_stage_3_a,
    output reg [31:0] phase_stage_3_b,
    output reg [31:0] phase_stage_4_a,
    output reg [31:0] phase_stage_4_b,
    output reg [31:0] phase_stage_5_a,
    output reg [31:0] phase_stage_5_b,
    output reg [31:0] phase_stage_6_a,
    output reg [31:0] phase_stage_6_b,
    output reg [31:0] phase_stage_7_a,
    output reg [31:0] phase_stage_7_b,
    output reg [31:0] start_amp_stage_0_a,
    output reg [31:0] start_amp_stage_0_b,
    output reg [31:0] start_amp_stage_1_a,
    output reg [31:0] start_amp_stage_1_b,
    output reg [31:0] start_amp_stage_2_a,
    output reg [31:0] start_amp_stage_2_b,
    output reg [31:0] start_amp_stage_3_a,
    output reg [31:0] start_amp_stage_3_b,
    output reg [31:0] start_amp_stage_4_a,
    output reg [31:0] start_amp_stage_4_b,
    output reg [31:0] start_amp_stage_5_a,
    output reg [31:0] start_amp_stage_5_b,
    output reg [31:0] start_amp_stage_6_a,
    output reg [31:0] start_amp_stage_6_b,
    output reg [31:0] start_amp_stage_7_a,
    output reg [31:0] start_amp_stage_7_b,
    output reg [31:0] step_stage_0_a,
    output reg [31:0] step_stage_0_b,
    output reg [31:0] step_stage_1_a,
    output reg [31:0] step_stage_1_b,
    output reg [31:0] step_stage_2_a,
    output reg [31:0] step_stage_2_b,
    output reg [31:0] step_stage_3_a,
    output reg [31:0] step_stage_3_b,
    output reg [31:0] step_stage_4_a,
    output reg [31:0] step_stage_4_b,
    output reg [31:0] step_stage_5_a,
    output reg [31:0] step_stage_5_b,
    output reg [31:0] step_stage_6_a,
    output reg [31:0] step_stage_6_b,
    output reg [31:0] step_stage_7_a,
    output reg [31:0] step_stage_7_b,
    output reg [31:0] duration_cycles_stage_0_a,
    output reg [31:0] duration_cycles_stage_0_b,
    output reg [31:0] duration_cycles_stage_1_a,
    output reg [31:0] duration_cycles_stage_1_b,
    output reg [31:0] duration_cycles_stage_2_a,
    output reg [31:0] duration_cycles_stage_2_b,
    output reg [31:0] duration_cycles_stage_3_a,
    output reg [31:0] duration_cycles_stage_3_b,
    output reg [31:0] duration_cycles_stage_4_a,
    output reg [31:0] duration_cycles_stage_4_b,
    output reg [31:0] duration_cycles_stage_5_a,
    output reg [31:0] duration_cycles_stage_5_b,
    output reg [31:0] duration_cycles_stage_6_a,
    output reg [31:0] duration_cycles_stage_6_b,
    output reg [31:0] duration_cycles_stage_7_a,
    output reg [31:0] duration_cycles_stage_7_b,
    output reg [31:0] loop_mode_stage_0_a,
    output reg [31:0] loop_mode_stage_0_b,
    output reg [31:0] loop_mode_stage_1_a,
    output reg [31:0] loop_mode_stage_1_b,
    output reg [31:0] loop_mode_stage_2_a,
    output reg [31:0] loop_mode_stage_2_b,
    output reg [31:0] loop_mode_stage_3_a,
    output reg [31:0] loop_mode_stage_3_b,
    output reg [31:0] loop_mode_stage_4_a,
    output reg [31:0] loop_mode_stage_4_b,
    output reg [31:0] loop_mode_stage_5_a,
    output reg [31:0] loop_mode_stage_5_b,
    output reg [31:0] loop_mode_stage_6_a,
    output reg [31:0] loop_mode_stage_6_b,
    output reg [31:0] loop_mode_stage_7_a,
    output reg [31:0] loop_mode_stage_7_b,

    // mux_sel: dac_clk domain, one per physical channel (0=a active/b idle,
    // 1=b active/a idle) -- feeds dac_output_mux.v's active-instance select
    output reg         mux_sel_0,
    output reg         mux_sel_1,
    output reg         mux_sel_2,
    output reg         mux_sel_3,
    output reg         mux_sel_4,
    output reg         mux_sel_5,
    output reg         mux_sel_6,
    output reg         mux_sel_7,

    // 2026-07-27 新增（統一讀取/寫入架構，QT_SINE_STATUS 查詢用）：
    // mux_sel 的 sys_clk 版本（下面 CDC 段落本來就有算，只是原本沒有
    // 當 output 露出），讓 aurora_reply_tx.v 知道每個 channel 現在
    // active 的是 a 還是 b，才能選出正確的 stage 值回報
    output wire [7:0]  mux_sel_sync,

    // 2026-08-05 新增：併入 dac_mode_ramp（見上方檔頭說明），直接重用
    // 本模組既有的 mux_sel_$ch（dac_clk）判斷 active 側，不需要額外 CDC。
    // mode_active_$m：module m 的 DDR(0)/Sine(1)；ramp_en_active_$c：
    // physical channel c 的 amp ramp 開關。個別 1-bit port（不是 vector），
    // 比照本檔案既有 mux_sel_0..7/trig_start_0..3 的慣例，直接一對一接
    // dac_output_mux_$ch/mode、amp_ctrl_mux_$i/ramp_en，不需要 create_bd.tcl
    // 額外的 xlslice（取代原本從 board_cfg_reg_0/dac_mode_ramp 拉 xlslice
    // 的接法）。
    output wire        mode_active_0,
    output wire        mode_active_1,
    output wire        mode_active_2,
    output wire        mode_active_3,
    output wire        ramp_en_active_0,
    output wire        ramp_en_active_1,
    output wire        ramp_en_active_2,
    output wire        ramp_en_active_3,
    output wire        ramp_en_active_4,
    output wire        ramp_en_active_5,
    output wire        ramp_en_active_6,
    output wire        ramp_en_active_7
);

    // ══════════════════════════════════════════════════════════════════
    //  2026-08 新增：per-channel N-slot table（N_SLOT=4），dac_clk
    //  domain，跟 waveform_controller.v 的 list_addr[]/list_len[] 同構
    //  ——純粹是 host 端排程資料的儲存區，不直接驅動下游，只在 arm/
    //  trig_start 那兩個時機才把內容複製進正牌的 *_stage_a/_b（見檔頭
    //  說明、下方統一 write-routing always block）。
    // ══════════════════════════════════════════════════════════════════
    localparam N_SLOT = 4;
    reg [31:0] slot_tuning_word     [0:7][0:N_SLOT-1];
    reg [31:0] slot_phase           [0:7][0:N_SLOT-1];
    reg [31:0] slot_start_amp       [0:7][0:N_SLOT-1];
    reg [31:0] slot_step            [0:7][0:N_SLOT-1];
    reg [31:0] slot_duration_cycles [0:7][0:N_SLOT-1];
    reg [31:0] slot_loop_mode       [0:7][0:N_SLOT-1];

    reg [1:0] current_idx [0:7];  // 控制用指標，語意同 waveform_controller.v，
    reg [1:0] next_idx    [0:7];  // 不代表「現在正在播哪個 slot」
    reg [2:0] list_depth  [0:7];  // 1-4 為有效深度；0（尚未 arm）視同 4
                                   // （inc_idx 內 depth-1 underflow 成
                                   // 3'b111，截斷後跟 depth=4 行為相同，
                                   // 純粹是安全預設值，不影響已 arm 過的
                                   // channel）

    integer si, sj; // reset 用迴圈變數

    // ── Next-index helper（跟 waveform_controller.v 的 inc_idx 同構，只是
    //    寬度改成 2-bit（N_SLOT=4）） ──────────────────────────────────
    function [1:0] inc_idx;
        input [1:0] idx;
        input [2:0] depth;
        reg   [2:0] depth_m1;
        begin
            depth_m1 = depth - 3'd1;
            inc_idx  = (idx >= depth_m1[1:0]) ? 2'd0 : idx + 2'd1;
        end
    endfunction

    // 2026-08 起唯一寫入路徑：dac_sine_wr（既有單值）+ dac_sine_list_wr
    // （新 N-slot），都已經是 dac_clk domain，不需要優先權 mux。
    wire [2:0] ch_sel    = dac_sine_sel[2:0];
    wire [2:0] param_sel = dac_sine_sel[5:3];

    wire [2:0] list_ch_sel    = dac_sine_list_sel[2:0];
    wire [2:0] list_param_sel = dac_sine_list_sel[5:3];
    wire [1:0] list_slot_idx  = dac_sine_list_sel[7:6];

    reg mode_stage_0_a, mode_stage_0_b;
    reg mode_stage_1_a, mode_stage_1_b;
    reg mode_stage_2_a, mode_stage_2_b;
    reg mode_stage_3_a, mode_stage_3_b;
    reg ramp_en_stage_0_a, ramp_en_stage_0_b;
    reg ramp_en_stage_1_a, ramp_en_stage_1_b;
    reg ramp_en_stage_2_a, ramp_en_stage_2_b;
    reg ramp_en_stage_3_a, ramp_en_stage_3_b;
    reg ramp_en_stage_4_a, ramp_en_stage_4_b;
    reg ramp_en_stage_5_a, ramp_en_stage_5_b;
    reg ramp_en_stage_6_a, ramp_en_stage_6_b;
    reg ramp_en_stage_7_a, ramp_en_stage_7_b;

    // -- CDC: mux_sel (dac_clk) -> sys_clk, standard 2-flop synchronizer.
    //    2026-08 起只給 QT_SINE_STATUS 讀回用（write-routing 已經改用
    //    dac_clk 原生的 mux_sel，不再需要這顆 CDC 才能判斷 idle 側）----
    (* ASYNC_REG = "TRUE" *) reg [7:0] mux_sel_s1 = 8'd0, mux_sel_s2 = 8'd0;
    always @(posedge clk) begin
        mux_sel_s1 <= {mux_sel_7, mux_sel_6, mux_sel_5, mux_sel_4, mux_sel_3, mux_sel_2, mux_sel_1, mux_sel_0};
        mux_sel_s2 <= mux_sel_s1;
    end
    assign mux_sel_sync = mux_sel_s2;

    // -- mode_active/ramp_en_active：dac_clk domain 直接用 mux_sel_$ch
    //    （不是 mux_sel_sync）選 active 側，跟 mux_sel 本身同一個 domain，
    //    不需要額外 CDC（2026-08-05 新增，2026-08 N-slot 改版沒有變動這
    //    段邏輯）-----------------------------------------------------------
    assign mode_active_0 = mux_sel_0 ? mode_stage_0_b : mode_stage_0_a;
    assign mode_active_1 = mux_sel_2 ? mode_stage_1_b : mode_stage_1_a;
    assign mode_active_2 = mux_sel_4 ? mode_stage_2_b : mode_stage_2_a;
    assign mode_active_3 = mux_sel_6 ? mode_stage_3_b : mode_stage_3_a;

    assign ramp_en_active_0 = mux_sel_0 ? ramp_en_stage_0_b : ramp_en_stage_0_a;
    assign ramp_en_active_1 = mux_sel_1 ? ramp_en_stage_1_b : ramp_en_stage_1_a;
    assign ramp_en_active_2 = mux_sel_2 ? ramp_en_stage_2_b : ramp_en_stage_2_a;
    assign ramp_en_active_3 = mux_sel_3 ? ramp_en_stage_3_b : ramp_en_stage_3_a;
    assign ramp_en_active_4 = mux_sel_4 ? ramp_en_stage_4_b : ramp_en_stage_4_a;
    assign ramp_en_active_5 = mux_sel_5 ? ramp_en_stage_5_b : ramp_en_stage_5_a;
    assign ramp_en_active_6 = mux_sel_6 ? ramp_en_stage_6_b : ramp_en_stage_6_a;
    assign ramp_en_active_7 = mux_sel_7 ? ramp_en_stage_7_b : ramp_en_stage_7_a;

    // ══════════════════════════════════════════════════════════════════
    //  dac_clk domain：統一 write-routing（2026-08 改版，比照 DDR
    //  waveform_controller.v，取代原本分成 sys_clk write-routing +
    //  dac_clk mux_sel-only 兩個 always block 的舊架構）。三個獨立寫入
    //  來源共用同一個 always block（同一個 reg 只有這裡一個 writer）：
    //    (A) dac_sine_wr：既有單值寫入（T_SINE_CTRL，Bulk Sine Setup 用）
    //    (B) dac_sine_list_wr：N-slot table 寫入 + arm（T_SINE_LIST_CTRL）
    //    (C) trig_start_$inst：trigger 時 mux_sel 翻轉 + 從 slot_table
    //        自動 load 進剛變 idle 的那側
    // ══════════════════════════════════════════════════════════════════
    always @(posedge dac_clk) begin
        if (dac_rst) begin
            mux_sel_0 <= 1'b0; mux_sel_1 <= 1'b0; mux_sel_2 <= 1'b0; mux_sel_3 <= 1'b0;
            mux_sel_4 <= 1'b0; mux_sel_5 <= 1'b0; mux_sel_6 <= 1'b0; mux_sel_7 <= 1'b0;

            tuning_word_stage_0_a <= 32'd0;
            tuning_word_stage_0_b <= 32'd0;
            tuning_word_stage_1_a <= 32'd0;
            tuning_word_stage_1_b <= 32'd0;
            tuning_word_stage_2_a <= 32'd0;
            tuning_word_stage_2_b <= 32'd0;
            tuning_word_stage_3_a <= 32'd0;
            tuning_word_stage_3_b <= 32'd0;
            tuning_word_stage_4_a <= 32'd0;
            tuning_word_stage_4_b <= 32'd0;
            tuning_word_stage_5_a <= 32'd0;
            tuning_word_stage_5_b <= 32'd0;
            tuning_word_stage_6_a <= 32'd0;
            tuning_word_stage_6_b <= 32'd0;
            tuning_word_stage_7_a <= 32'd0;
            tuning_word_stage_7_b <= 32'd0;
            phase_stage_0_a <= 32'd0;
            phase_stage_0_b <= 32'd0;
            phase_stage_1_a <= 32'd0;
            phase_stage_1_b <= 32'd0;
            phase_stage_2_a <= 32'd0;
            phase_stage_2_b <= 32'd0;
            phase_stage_3_a <= 32'd0;
            phase_stage_3_b <= 32'd0;
            phase_stage_4_a <= 32'd0;
            phase_stage_4_b <= 32'd0;
            phase_stage_5_a <= 32'd0;
            phase_stage_5_b <= 32'd0;
            phase_stage_6_a <= 32'd0;
            phase_stage_6_b <= 32'd0;
            phase_stage_7_a <= 32'd0;
            phase_stage_7_b <= 32'd0;
            start_amp_stage_0_a <= 32'd0;
            start_amp_stage_0_b <= 32'd0;
            start_amp_stage_1_a <= 32'd0;
            start_amp_stage_1_b <= 32'd0;
            start_amp_stage_2_a <= 32'd0;
            start_amp_stage_2_b <= 32'd0;
            start_amp_stage_3_a <= 32'd0;
            start_amp_stage_3_b <= 32'd0;
            start_amp_stage_4_a <= 32'd0;
            start_amp_stage_4_b <= 32'd0;
            start_amp_stage_5_a <= 32'd0;
            start_amp_stage_5_b <= 32'd0;
            start_amp_stage_6_a <= 32'd0;
            start_amp_stage_6_b <= 32'd0;
            start_amp_stage_7_a <= 32'd0;
            start_amp_stage_7_b <= 32'd0;
            step_stage_0_a <= 32'd0;
            step_stage_0_b <= 32'd0;
            step_stage_1_a <= 32'd0;
            step_stage_1_b <= 32'd0;
            step_stage_2_a <= 32'd0;
            step_stage_2_b <= 32'd0;
            step_stage_3_a <= 32'd0;
            step_stage_3_b <= 32'd0;
            step_stage_4_a <= 32'd0;
            step_stage_4_b <= 32'd0;
            step_stage_5_a <= 32'd0;
            step_stage_5_b <= 32'd0;
            step_stage_6_a <= 32'd0;
            step_stage_6_b <= 32'd0;
            step_stage_7_a <= 32'd0;
            step_stage_7_b <= 32'd0;
            duration_cycles_stage_0_a <= 32'd0;
            duration_cycles_stage_0_b <= 32'd0;
            duration_cycles_stage_1_a <= 32'd0;
            duration_cycles_stage_1_b <= 32'd0;
            duration_cycles_stage_2_a <= 32'd0;
            duration_cycles_stage_2_b <= 32'd0;
            duration_cycles_stage_3_a <= 32'd0;
            duration_cycles_stage_3_b <= 32'd0;
            duration_cycles_stage_4_a <= 32'd0;
            duration_cycles_stage_4_b <= 32'd0;
            duration_cycles_stage_5_a <= 32'd0;
            duration_cycles_stage_5_b <= 32'd0;
            duration_cycles_stage_6_a <= 32'd0;
            duration_cycles_stage_6_b <= 32'd0;
            duration_cycles_stage_7_a <= 32'd0;
            duration_cycles_stage_7_b <= 32'd0;
            loop_mode_stage_0_a <= 32'd0;
            loop_mode_stage_0_b <= 32'd0;
            loop_mode_stage_1_a <= 32'd0;
            loop_mode_stage_1_b <= 32'd0;
            loop_mode_stage_2_a <= 32'd0;
            loop_mode_stage_2_b <= 32'd0;
            loop_mode_stage_3_a <= 32'd0;
            loop_mode_stage_3_b <= 32'd0;
            loop_mode_stage_4_a <= 32'd0;
            loop_mode_stage_4_b <= 32'd0;
            loop_mode_stage_5_a <= 32'd0;
            loop_mode_stage_5_b <= 32'd0;
            loop_mode_stage_6_a <= 32'd0;
            loop_mode_stage_6_b <= 32'd0;
            loop_mode_stage_7_a <= 32'd0;
            loop_mode_stage_7_b <= 32'd0;
            mode_stage_0_a <= 1'b0; mode_stage_0_b <= 1'b0;
            mode_stage_1_a <= 1'b0; mode_stage_1_b <= 1'b0;
            mode_stage_2_a <= 1'b0; mode_stage_2_b <= 1'b0;
            mode_stage_3_a <= 1'b0; mode_stage_3_b <= 1'b0;
            ramp_en_stage_0_a <= 1'b0; ramp_en_stage_0_b <= 1'b0;
            ramp_en_stage_1_a <= 1'b0; ramp_en_stage_1_b <= 1'b0;
            ramp_en_stage_2_a <= 1'b0; ramp_en_stage_2_b <= 1'b0;
            ramp_en_stage_3_a <= 1'b0; ramp_en_stage_3_b <= 1'b0;
            ramp_en_stage_4_a <= 1'b0; ramp_en_stage_4_b <= 1'b0;
            ramp_en_stage_5_a <= 1'b0; ramp_en_stage_5_b <= 1'b0;
            ramp_en_stage_6_a <= 1'b0; ramp_en_stage_6_b <= 1'b0;
            ramp_en_stage_7_a <= 1'b0; ramp_en_stage_7_b <= 1'b0;

// ---- GEN: reset for slot table / idx / depth ----
for (si = 0; si < 8; si = si + 1) begin
    current_idx[si] <= 2'd0;
    next_idx[si]    <= 2'd0;
    list_depth[si]  <= 3'd0;
    for (sj = 0; sj < N_SLOT; sj = sj + 1) begin
        slot_tuning_word[si][sj] <= 32'd0;
        slot_phase[si][sj] <= 32'd0;
        slot_start_amp[si][sj] <= 32'd0;
        slot_step[si][sj] <= 32'd0;
        slot_duration_cycles[si][sj] <= 32'd0;
        slot_loop_mode[si][sj] <= 32'd0;
    end
end

        end else begin
            // ── (A) 既有單值寫入路徑（T_SINE_CTRL），邏輯完全不變，只是
            //    domain 換成 dac_clk、idle 側判斷從 mux_sel_sync[N] 換成
            //    原生 mux_sel_N ──────────────────────────────────────────
            if (dac_sine_wr) begin
            case ({param_sel, ch_sel})
                6'b000_000: begin
                    if (mux_sel_0) tuning_word_stage_0_a <= dac_sine_data;  // b active -> write idle a
                    else                  tuning_word_stage_0_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b000_001: begin
                    if (mux_sel_1) tuning_word_stage_1_a <= dac_sine_data;  // b active -> write idle a
                    else                  tuning_word_stage_1_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b000_010: begin
                    if (mux_sel_2) tuning_word_stage_2_a <= dac_sine_data;  // b active -> write idle a
                    else                  tuning_word_stage_2_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b000_011: begin
                    if (mux_sel_3) tuning_word_stage_3_a <= dac_sine_data;  // b active -> write idle a
                    else                  tuning_word_stage_3_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b000_100: begin
                    if (mux_sel_4) tuning_word_stage_4_a <= dac_sine_data;  // b active -> write idle a
                    else                  tuning_word_stage_4_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b000_101: begin
                    if (mux_sel_5) tuning_word_stage_5_a <= dac_sine_data;  // b active -> write idle a
                    else                  tuning_word_stage_5_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b000_110: begin
                    if (mux_sel_6) tuning_word_stage_6_a <= dac_sine_data;  // b active -> write idle a
                    else                  tuning_word_stage_6_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b000_111: begin
                    if (mux_sel_7) tuning_word_stage_7_a <= dac_sine_data;  // b active -> write idle a
                    else                  tuning_word_stage_7_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b001_000: begin
                    if (mux_sel_0) phase_stage_0_a <= dac_sine_data;  // b active -> write idle a
                    else                  phase_stage_0_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b001_001: begin
                    if (mux_sel_1) phase_stage_1_a <= dac_sine_data;  // b active -> write idle a
                    else                  phase_stage_1_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b001_010: begin
                    if (mux_sel_2) phase_stage_2_a <= dac_sine_data;  // b active -> write idle a
                    else                  phase_stage_2_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b001_011: begin
                    if (mux_sel_3) phase_stage_3_a <= dac_sine_data;  // b active -> write idle a
                    else                  phase_stage_3_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b001_100: begin
                    if (mux_sel_4) phase_stage_4_a <= dac_sine_data;  // b active -> write idle a
                    else                  phase_stage_4_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b001_101: begin
                    if (mux_sel_5) phase_stage_5_a <= dac_sine_data;  // b active -> write idle a
                    else                  phase_stage_5_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b001_110: begin
                    if (mux_sel_6) phase_stage_6_a <= dac_sine_data;  // b active -> write idle a
                    else                  phase_stage_6_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b001_111: begin
                    if (mux_sel_7) phase_stage_7_a <= dac_sine_data;  // b active -> write idle a
                    else                  phase_stage_7_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b010_000: begin
                    if (mux_sel_0) start_amp_stage_0_a <= dac_sine_data;  // b active -> write idle a
                    else                  start_amp_stage_0_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b010_001: begin
                    if (mux_sel_1) start_amp_stage_1_a <= dac_sine_data;  // b active -> write idle a
                    else                  start_amp_stage_1_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b010_010: begin
                    if (mux_sel_2) start_amp_stage_2_a <= dac_sine_data;  // b active -> write idle a
                    else                  start_amp_stage_2_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b010_011: begin
                    if (mux_sel_3) start_amp_stage_3_a <= dac_sine_data;  // b active -> write idle a
                    else                  start_amp_stage_3_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b010_100: begin
                    if (mux_sel_4) start_amp_stage_4_a <= dac_sine_data;  // b active -> write idle a
                    else                  start_amp_stage_4_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b010_101: begin
                    if (mux_sel_5) start_amp_stage_5_a <= dac_sine_data;  // b active -> write idle a
                    else                  start_amp_stage_5_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b010_110: begin
                    if (mux_sel_6) start_amp_stage_6_a <= dac_sine_data;  // b active -> write idle a
                    else                  start_amp_stage_6_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b010_111: begin
                    if (mux_sel_7) start_amp_stage_7_a <= dac_sine_data;  // b active -> write idle a
                    else                  start_amp_stage_7_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b011_000: begin
                    if (mux_sel_0) step_stage_0_a <= dac_sine_data;  // b active -> write idle a
                    else                  step_stage_0_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b011_001: begin
                    if (mux_sel_1) step_stage_1_a <= dac_sine_data;  // b active -> write idle a
                    else                  step_stage_1_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b011_010: begin
                    if (mux_sel_2) step_stage_2_a <= dac_sine_data;  // b active -> write idle a
                    else                  step_stage_2_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b011_011: begin
                    if (mux_sel_3) step_stage_3_a <= dac_sine_data;  // b active -> write idle a
                    else                  step_stage_3_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b011_100: begin
                    if (mux_sel_4) step_stage_4_a <= dac_sine_data;  // b active -> write idle a
                    else                  step_stage_4_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b011_101: begin
                    if (mux_sel_5) step_stage_5_a <= dac_sine_data;  // b active -> write idle a
                    else                  step_stage_5_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b011_110: begin
                    if (mux_sel_6) step_stage_6_a <= dac_sine_data;  // b active -> write idle a
                    else                  step_stage_6_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b011_111: begin
                    if (mux_sel_7) step_stage_7_a <= dac_sine_data;  // b active -> write idle a
                    else                  step_stage_7_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b100_000: begin
                    if (mux_sel_0) duration_cycles_stage_0_a <= dac_sine_data;  // b active -> write idle a
                    else                  duration_cycles_stage_0_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b100_001: begin
                    if (mux_sel_1) duration_cycles_stage_1_a <= dac_sine_data;  // b active -> write idle a
                    else                  duration_cycles_stage_1_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b100_010: begin
                    if (mux_sel_2) duration_cycles_stage_2_a <= dac_sine_data;  // b active -> write idle a
                    else                  duration_cycles_stage_2_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b100_011: begin
                    if (mux_sel_3) duration_cycles_stage_3_a <= dac_sine_data;  // b active -> write idle a
                    else                  duration_cycles_stage_3_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b100_100: begin
                    if (mux_sel_4) duration_cycles_stage_4_a <= dac_sine_data;  // b active -> write idle a
                    else                  duration_cycles_stage_4_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b100_101: begin
                    if (mux_sel_5) duration_cycles_stage_5_a <= dac_sine_data;  // b active -> write idle a
                    else                  duration_cycles_stage_5_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b100_110: begin
                    if (mux_sel_6) duration_cycles_stage_6_a <= dac_sine_data;  // b active -> write idle a
                    else                  duration_cycles_stage_6_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b100_111: begin
                    if (mux_sel_7) duration_cycles_stage_7_a <= dac_sine_data;  // b active -> write idle a
                    else                  duration_cycles_stage_7_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b101_000: begin
                    if (mux_sel_0) loop_mode_stage_0_a <= dac_sine_data;  // b active -> write idle a
                    else                  loop_mode_stage_0_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b101_001: begin
                    if (mux_sel_1) loop_mode_stage_1_a <= dac_sine_data;  // b active -> write idle a
                    else                  loop_mode_stage_1_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b101_010: begin
                    if (mux_sel_2) loop_mode_stage_2_a <= dac_sine_data;  // b active -> write idle a
                    else                  loop_mode_stage_2_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b101_011: begin
                    if (mux_sel_3) loop_mode_stage_3_a <= dac_sine_data;  // b active -> write idle a
                    else                  loop_mode_stage_3_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b101_100: begin
                    if (mux_sel_4) loop_mode_stage_4_a <= dac_sine_data;  // b active -> write idle a
                    else                  loop_mode_stage_4_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b101_101: begin
                    if (mux_sel_5) loop_mode_stage_5_a <= dac_sine_data;  // b active -> write idle a
                    else                  loop_mode_stage_5_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b101_110: begin
                    if (mux_sel_6) loop_mode_stage_6_a <= dac_sine_data;  // b active -> write idle a
                    else                  loop_mode_stage_6_b <= dac_sine_data;  // a active -> write idle b
                end
                6'b101_111: begin
                    if (mux_sel_7) loop_mode_stage_7_a <= dac_sine_data;  // b active -> write idle a
                    else                  loop_mode_stage_7_b <= dac_sine_data;  // a active -> write idle b
                end
                // 2026-08-05 新增（併入 dac_mode_ramp，見檔頭說明）：
                // param_sel=6 -> ramp_en（per-channel，完整 ch_sel 0-7）
                6'b110_000: begin
                    if (mux_sel_0) ramp_en_stage_0_a <= dac_sine_data[0];  // b active -> write idle a
                    else                  ramp_en_stage_0_b <= dac_sine_data[0];  // a active -> write idle b
                end
                6'b110_001: begin
                    if (mux_sel_1) ramp_en_stage_1_a <= dac_sine_data[0];
                    else                  ramp_en_stage_1_b <= dac_sine_data[0];
                end
                6'b110_010: begin
                    if (mux_sel_2) ramp_en_stage_2_a <= dac_sine_data[0];
                    else                  ramp_en_stage_2_b <= dac_sine_data[0];
                end
                6'b110_011: begin
                    if (mux_sel_3) ramp_en_stage_3_a <= dac_sine_data[0];
                    else                  ramp_en_stage_3_b <= dac_sine_data[0];
                end
                6'b110_100: begin
                    if (mux_sel_4) ramp_en_stage_4_a <= dac_sine_data[0];
                    else                  ramp_en_stage_4_b <= dac_sine_data[0];
                end
                6'b110_101: begin
                    if (mux_sel_5) ramp_en_stage_5_a <= dac_sine_data[0];
                    else                  ramp_en_stage_5_b <= dac_sine_data[0];
                end
                6'b110_110: begin
                    if (mux_sel_6) ramp_en_stage_6_a <= dac_sine_data[0];
                    else                  ramp_en_stage_6_b <= dac_sine_data[0];
                end
                6'b110_111: begin
                    if (mux_sel_7) ramp_en_stage_7_a <= dac_sine_data[0];
                    else                  ramp_en_stage_7_b <= dac_sine_data[0];
                end
                // param_sel=7 -> mode（per-module，只看 ch_sel[2:1] 選
                // module，ch_sel[0] 忽略——host 端 set_dac_mode() 固定送
                // ch_sel[0]=0，見 board_ctrl.py；奇數 ch_sel 落到 default
                // no-op，不會發生）
                6'b111_000: begin
                    if (mux_sel_0) mode_stage_0_a <= dac_sine_data[0];
                    else                  mode_stage_0_b <= dac_sine_data[0];
                end
                6'b111_010: begin
                    if (mux_sel_2) mode_stage_1_a <= dac_sine_data[0];
                    else                  mode_stage_1_b <= dac_sine_data[0];
                end
                6'b111_100: begin
                    if (mux_sel_4) mode_stage_2_a <= dac_sine_data[0];
                    else                  mode_stage_2_b <= dac_sine_data[0];
                end
                6'b111_110: begin
                    if (mux_sel_6) mode_stage_3_a <= dac_sine_data[0];
                    else                  mode_stage_3_b <= dac_sine_data[0];
                end
                default: ; // param_sel 6/7 的奇數 ch_sel（mode 不使用）: no-op
            endcase
            end

            // ── (B) N-slot table 寫入 + arm（T_SINE_LIST_CTRL） ─────────
            if (dac_sine_list_wr) begin
                if (list_param_sel == 3'd6) begin
                    // commit list_depth + arm：重置指標、把 slot0/slot1
                    // 分別 preload 進 active/idle 側（比照
                    // waveform_controller.v 的 ST_CFG_INIT）。host 端必須
                    // 保證這個 channel 的 slot 內容已經全部寫完才送這個
                    // （見檔頭說明的寫入順序 contract）。
                    list_depth[list_ch_sel]  <= dac_sine_list_data[2:0];
                    current_idx[list_ch_sel] <= 2'd0;
                    next_idx[list_ch_sel]    <= inc_idx(2'd0, dac_sine_list_data[2:0]);
case (list_ch_sel)
    3'd0: if (mux_sel_0) begin
        // b active -> refresh active(b)<-slot0, preload idle(a)<-slot inc_idx(0,depth)
        tuning_word_stage_0_b <= slot_tuning_word[0][0];
        phase_stage_0_b <= slot_phase[0][0];
        start_amp_stage_0_b <= slot_start_amp[0][0];
        step_stage_0_b <= slot_step[0][0];
        duration_cycles_stage_0_b <= slot_duration_cycles[0][0];
        loop_mode_stage_0_b <= slot_loop_mode[0][0];
        tuning_word_stage_0_a <= slot_tuning_word[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_0_a <= slot_phase[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_0_a <= slot_start_amp[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_0_a <= slot_step[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_0_a <= slot_duration_cycles[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_0_a <= slot_loop_mode[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end else begin
        // a active -> refresh active(a)<-slot0, preload idle(b)<-slot inc_idx(0,depth)
        tuning_word_stage_0_a <= slot_tuning_word[0][0];
        phase_stage_0_a <= slot_phase[0][0];
        start_amp_stage_0_a <= slot_start_amp[0][0];
        step_stage_0_a <= slot_step[0][0];
        duration_cycles_stage_0_a <= slot_duration_cycles[0][0];
        loop_mode_stage_0_a <= slot_loop_mode[0][0];
        tuning_word_stage_0_b <= slot_tuning_word[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_0_b <= slot_phase[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_0_b <= slot_start_amp[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_0_b <= slot_step[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_0_b <= slot_duration_cycles[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_0_b <= slot_loop_mode[0][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end
    3'd1: if (mux_sel_1) begin
        // b active -> refresh active(b)<-slot0, preload idle(a)<-slot inc_idx(0,depth)
        tuning_word_stage_1_b <= slot_tuning_word[1][0];
        phase_stage_1_b <= slot_phase[1][0];
        start_amp_stage_1_b <= slot_start_amp[1][0];
        step_stage_1_b <= slot_step[1][0];
        duration_cycles_stage_1_b <= slot_duration_cycles[1][0];
        loop_mode_stage_1_b <= slot_loop_mode[1][0];
        tuning_word_stage_1_a <= slot_tuning_word[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_1_a <= slot_phase[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_1_a <= slot_start_amp[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_1_a <= slot_step[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_1_a <= slot_duration_cycles[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_1_a <= slot_loop_mode[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end else begin
        // a active -> refresh active(a)<-slot0, preload idle(b)<-slot inc_idx(0,depth)
        tuning_word_stage_1_a <= slot_tuning_word[1][0];
        phase_stage_1_a <= slot_phase[1][0];
        start_amp_stage_1_a <= slot_start_amp[1][0];
        step_stage_1_a <= slot_step[1][0];
        duration_cycles_stage_1_a <= slot_duration_cycles[1][0];
        loop_mode_stage_1_a <= slot_loop_mode[1][0];
        tuning_word_stage_1_b <= slot_tuning_word[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_1_b <= slot_phase[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_1_b <= slot_start_amp[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_1_b <= slot_step[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_1_b <= slot_duration_cycles[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_1_b <= slot_loop_mode[1][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end
    3'd2: if (mux_sel_2) begin
        // b active -> refresh active(b)<-slot0, preload idle(a)<-slot inc_idx(0,depth)
        tuning_word_stage_2_b <= slot_tuning_word[2][0];
        phase_stage_2_b <= slot_phase[2][0];
        start_amp_stage_2_b <= slot_start_amp[2][0];
        step_stage_2_b <= slot_step[2][0];
        duration_cycles_stage_2_b <= slot_duration_cycles[2][0];
        loop_mode_stage_2_b <= slot_loop_mode[2][0];
        tuning_word_stage_2_a <= slot_tuning_word[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_2_a <= slot_phase[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_2_a <= slot_start_amp[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_2_a <= slot_step[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_2_a <= slot_duration_cycles[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_2_a <= slot_loop_mode[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end else begin
        // a active -> refresh active(a)<-slot0, preload idle(b)<-slot inc_idx(0,depth)
        tuning_word_stage_2_a <= slot_tuning_word[2][0];
        phase_stage_2_a <= slot_phase[2][0];
        start_amp_stage_2_a <= slot_start_amp[2][0];
        step_stage_2_a <= slot_step[2][0];
        duration_cycles_stage_2_a <= slot_duration_cycles[2][0];
        loop_mode_stage_2_a <= slot_loop_mode[2][0];
        tuning_word_stage_2_b <= slot_tuning_word[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_2_b <= slot_phase[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_2_b <= slot_start_amp[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_2_b <= slot_step[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_2_b <= slot_duration_cycles[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_2_b <= slot_loop_mode[2][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end
    3'd3: if (mux_sel_3) begin
        // b active -> refresh active(b)<-slot0, preload idle(a)<-slot inc_idx(0,depth)
        tuning_word_stage_3_b <= slot_tuning_word[3][0];
        phase_stage_3_b <= slot_phase[3][0];
        start_amp_stage_3_b <= slot_start_amp[3][0];
        step_stage_3_b <= slot_step[3][0];
        duration_cycles_stage_3_b <= slot_duration_cycles[3][0];
        loop_mode_stage_3_b <= slot_loop_mode[3][0];
        tuning_word_stage_3_a <= slot_tuning_word[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_3_a <= slot_phase[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_3_a <= slot_start_amp[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_3_a <= slot_step[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_3_a <= slot_duration_cycles[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_3_a <= slot_loop_mode[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end else begin
        // a active -> refresh active(a)<-slot0, preload idle(b)<-slot inc_idx(0,depth)
        tuning_word_stage_3_a <= slot_tuning_word[3][0];
        phase_stage_3_a <= slot_phase[3][0];
        start_amp_stage_3_a <= slot_start_amp[3][0];
        step_stage_3_a <= slot_step[3][0];
        duration_cycles_stage_3_a <= slot_duration_cycles[3][0];
        loop_mode_stage_3_a <= slot_loop_mode[3][0];
        tuning_word_stage_3_b <= slot_tuning_word[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_3_b <= slot_phase[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_3_b <= slot_start_amp[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_3_b <= slot_step[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_3_b <= slot_duration_cycles[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_3_b <= slot_loop_mode[3][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end
    3'd4: if (mux_sel_4) begin
        // b active -> refresh active(b)<-slot0, preload idle(a)<-slot inc_idx(0,depth)
        tuning_word_stage_4_b <= slot_tuning_word[4][0];
        phase_stage_4_b <= slot_phase[4][0];
        start_amp_stage_4_b <= slot_start_amp[4][0];
        step_stage_4_b <= slot_step[4][0];
        duration_cycles_stage_4_b <= slot_duration_cycles[4][0];
        loop_mode_stage_4_b <= slot_loop_mode[4][0];
        tuning_word_stage_4_a <= slot_tuning_word[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_4_a <= slot_phase[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_4_a <= slot_start_amp[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_4_a <= slot_step[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_4_a <= slot_duration_cycles[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_4_a <= slot_loop_mode[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end else begin
        // a active -> refresh active(a)<-slot0, preload idle(b)<-slot inc_idx(0,depth)
        tuning_word_stage_4_a <= slot_tuning_word[4][0];
        phase_stage_4_a <= slot_phase[4][0];
        start_amp_stage_4_a <= slot_start_amp[4][0];
        step_stage_4_a <= slot_step[4][0];
        duration_cycles_stage_4_a <= slot_duration_cycles[4][0];
        loop_mode_stage_4_a <= slot_loop_mode[4][0];
        tuning_word_stage_4_b <= slot_tuning_word[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_4_b <= slot_phase[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_4_b <= slot_start_amp[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_4_b <= slot_step[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_4_b <= slot_duration_cycles[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_4_b <= slot_loop_mode[4][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end
    3'd5: if (mux_sel_5) begin
        // b active -> refresh active(b)<-slot0, preload idle(a)<-slot inc_idx(0,depth)
        tuning_word_stage_5_b <= slot_tuning_word[5][0];
        phase_stage_5_b <= slot_phase[5][0];
        start_amp_stage_5_b <= slot_start_amp[5][0];
        step_stage_5_b <= slot_step[5][0];
        duration_cycles_stage_5_b <= slot_duration_cycles[5][0];
        loop_mode_stage_5_b <= slot_loop_mode[5][0];
        tuning_word_stage_5_a <= slot_tuning_word[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_5_a <= slot_phase[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_5_a <= slot_start_amp[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_5_a <= slot_step[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_5_a <= slot_duration_cycles[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_5_a <= slot_loop_mode[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end else begin
        // a active -> refresh active(a)<-slot0, preload idle(b)<-slot inc_idx(0,depth)
        tuning_word_stage_5_a <= slot_tuning_word[5][0];
        phase_stage_5_a <= slot_phase[5][0];
        start_amp_stage_5_a <= slot_start_amp[5][0];
        step_stage_5_a <= slot_step[5][0];
        duration_cycles_stage_5_a <= slot_duration_cycles[5][0];
        loop_mode_stage_5_a <= slot_loop_mode[5][0];
        tuning_word_stage_5_b <= slot_tuning_word[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_5_b <= slot_phase[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_5_b <= slot_start_amp[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_5_b <= slot_step[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_5_b <= slot_duration_cycles[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_5_b <= slot_loop_mode[5][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end
    3'd6: if (mux_sel_6) begin
        // b active -> refresh active(b)<-slot0, preload idle(a)<-slot inc_idx(0,depth)
        tuning_word_stage_6_b <= slot_tuning_word[6][0];
        phase_stage_6_b <= slot_phase[6][0];
        start_amp_stage_6_b <= slot_start_amp[6][0];
        step_stage_6_b <= slot_step[6][0];
        duration_cycles_stage_6_b <= slot_duration_cycles[6][0];
        loop_mode_stage_6_b <= slot_loop_mode[6][0];
        tuning_word_stage_6_a <= slot_tuning_word[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_6_a <= slot_phase[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_6_a <= slot_start_amp[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_6_a <= slot_step[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_6_a <= slot_duration_cycles[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_6_a <= slot_loop_mode[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end else begin
        // a active -> refresh active(a)<-slot0, preload idle(b)<-slot inc_idx(0,depth)
        tuning_word_stage_6_a <= slot_tuning_word[6][0];
        phase_stage_6_a <= slot_phase[6][0];
        start_amp_stage_6_a <= slot_start_amp[6][0];
        step_stage_6_a <= slot_step[6][0];
        duration_cycles_stage_6_a <= slot_duration_cycles[6][0];
        loop_mode_stage_6_a <= slot_loop_mode[6][0];
        tuning_word_stage_6_b <= slot_tuning_word[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_6_b <= slot_phase[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_6_b <= slot_start_amp[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_6_b <= slot_step[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_6_b <= slot_duration_cycles[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_6_b <= slot_loop_mode[6][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end
    3'd7: if (mux_sel_7) begin
        // b active -> refresh active(b)<-slot0, preload idle(a)<-slot inc_idx(0,depth)
        tuning_word_stage_7_b <= slot_tuning_word[7][0];
        phase_stage_7_b <= slot_phase[7][0];
        start_amp_stage_7_b <= slot_start_amp[7][0];
        step_stage_7_b <= slot_step[7][0];
        duration_cycles_stage_7_b <= slot_duration_cycles[7][0];
        loop_mode_stage_7_b <= slot_loop_mode[7][0];
        tuning_word_stage_7_a <= slot_tuning_word[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_7_a <= slot_phase[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_7_a <= slot_start_amp[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_7_a <= slot_step[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_7_a <= slot_duration_cycles[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_7_a <= slot_loop_mode[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end else begin
        // a active -> refresh active(a)<-slot0, preload idle(b)<-slot inc_idx(0,depth)
        tuning_word_stage_7_a <= slot_tuning_word[7][0];
        phase_stage_7_a <= slot_phase[7][0];
        start_amp_stage_7_a <= slot_start_amp[7][0];
        step_stage_7_a <= slot_step[7][0];
        duration_cycles_stage_7_a <= slot_duration_cycles[7][0];
        loop_mode_stage_7_a <= slot_loop_mode[7][0];
        tuning_word_stage_7_b <= slot_tuning_word[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
        phase_stage_7_b <= slot_phase[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
        start_amp_stage_7_b <= slot_start_amp[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
        step_stage_7_b <= slot_step[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
        duration_cycles_stage_7_b <= slot_duration_cycles[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
        loop_mode_stage_7_b <= slot_loop_mode[7][inc_idx(2'd0, dac_sine_list_data[2:0])];
    end
    default: ;
endcase
                end else begin
                    // slot 內容寫入（param_sel 0-5 對應既有 6 個參數）：
                    // 直接寫進陣列，不需要像 (A) 那樣逐 channel 展開 case
                    // ——陣列可以直接用變數索引，不像 *_stage_X_a/b 是各自
                    // 獨立命名的 port。
                    case (list_param_sel)
                        3'd0: slot_tuning_word    [list_ch_sel][list_slot_idx] <= dac_sine_list_data;
                        3'd1: slot_phase          [list_ch_sel][list_slot_idx] <= dac_sine_list_data;
                        3'd2: slot_start_amp      [list_ch_sel][list_slot_idx] <= dac_sine_list_data;
                        3'd3: slot_step           [list_ch_sel][list_slot_idx] <= dac_sine_list_data;
                        3'd4: slot_duration_cycles[list_ch_sel][list_slot_idx] <= dac_sine_list_data;
                        3'd5: slot_loop_mode      [list_ch_sel][list_slot_idx] <= dac_sine_list_data;
                        default: ; // param_sel 7 未用
                    endcase
                end
            end

            // ── (C) trigger 時 mux_sel 翻轉 + 從 slot_table 自動 load 進
            //    剛變 idle 的那側（每個模組獨立，group_trig_select.v 已經
            //    過濾過哪些模組這次真的要動作，見 rtl 既有 2026-07-27
            //    per-module trigger 架構）────────────────────────────────
if (trig_start_0) begin
    mux_sel_0 <= ~mux_sel_0;
    current_idx[0] <= next_idx[0];
    next_idx[0]    <= inc_idx(next_idx[0], list_depth[0]);
    if (mux_sel_0) begin
        // b was active -> b becomes idle after swap -> preload b with
        // the slot needed at the FOLLOWING trigger (post-increment next_idx)
        tuning_word_stage_0_b <= slot_tuning_word[0][inc_idx(next_idx[0], list_depth[0])];
        phase_stage_0_b <= slot_phase[0][inc_idx(next_idx[0], list_depth[0])];
        start_amp_stage_0_b <= slot_start_amp[0][inc_idx(next_idx[0], list_depth[0])];
        step_stage_0_b <= slot_step[0][inc_idx(next_idx[0], list_depth[0])];
        duration_cycles_stage_0_b <= slot_duration_cycles[0][inc_idx(next_idx[0], list_depth[0])];
        loop_mode_stage_0_b <= slot_loop_mode[0][inc_idx(next_idx[0], list_depth[0])];
    end else begin
        // a was active -> a becomes idle after swap -> preload a likewise
        tuning_word_stage_0_a <= slot_tuning_word[0][inc_idx(next_idx[0], list_depth[0])];
        phase_stage_0_a <= slot_phase[0][inc_idx(next_idx[0], list_depth[0])];
        start_amp_stage_0_a <= slot_start_amp[0][inc_idx(next_idx[0], list_depth[0])];
        step_stage_0_a <= slot_step[0][inc_idx(next_idx[0], list_depth[0])];
        duration_cycles_stage_0_a <= slot_duration_cycles[0][inc_idx(next_idx[0], list_depth[0])];
        loop_mode_stage_0_a <= slot_loop_mode[0][inc_idx(next_idx[0], list_depth[0])];
    end

    mux_sel_1 <= ~mux_sel_1;
    current_idx[1] <= next_idx[1];
    next_idx[1]    <= inc_idx(next_idx[1], list_depth[1]);
    if (mux_sel_1) begin
        // b was active -> b becomes idle after swap -> preload b with
        // the slot needed at the FOLLOWING trigger (post-increment next_idx)
        tuning_word_stage_1_b <= slot_tuning_word[1][inc_idx(next_idx[1], list_depth[1])];
        phase_stage_1_b <= slot_phase[1][inc_idx(next_idx[1], list_depth[1])];
        start_amp_stage_1_b <= slot_start_amp[1][inc_idx(next_idx[1], list_depth[1])];
        step_stage_1_b <= slot_step[1][inc_idx(next_idx[1], list_depth[1])];
        duration_cycles_stage_1_b <= slot_duration_cycles[1][inc_idx(next_idx[1], list_depth[1])];
        loop_mode_stage_1_b <= slot_loop_mode[1][inc_idx(next_idx[1], list_depth[1])];
    end else begin
        // a was active -> a becomes idle after swap -> preload a likewise
        tuning_word_stage_1_a <= slot_tuning_word[1][inc_idx(next_idx[1], list_depth[1])];
        phase_stage_1_a <= slot_phase[1][inc_idx(next_idx[1], list_depth[1])];
        start_amp_stage_1_a <= slot_start_amp[1][inc_idx(next_idx[1], list_depth[1])];
        step_stage_1_a <= slot_step[1][inc_idx(next_idx[1], list_depth[1])];
        duration_cycles_stage_1_a <= slot_duration_cycles[1][inc_idx(next_idx[1], list_depth[1])];
        loop_mode_stage_1_a <= slot_loop_mode[1][inc_idx(next_idx[1], list_depth[1])];
    end

end
if (trig_start_1) begin
    mux_sel_2 <= ~mux_sel_2;
    current_idx[2] <= next_idx[2];
    next_idx[2]    <= inc_idx(next_idx[2], list_depth[2]);
    if (mux_sel_2) begin
        // b was active -> b becomes idle after swap -> preload b with
        // the slot needed at the FOLLOWING trigger (post-increment next_idx)
        tuning_word_stage_2_b <= slot_tuning_word[2][inc_idx(next_idx[2], list_depth[2])];
        phase_stage_2_b <= slot_phase[2][inc_idx(next_idx[2], list_depth[2])];
        start_amp_stage_2_b <= slot_start_amp[2][inc_idx(next_idx[2], list_depth[2])];
        step_stage_2_b <= slot_step[2][inc_idx(next_idx[2], list_depth[2])];
        duration_cycles_stage_2_b <= slot_duration_cycles[2][inc_idx(next_idx[2], list_depth[2])];
        loop_mode_stage_2_b <= slot_loop_mode[2][inc_idx(next_idx[2], list_depth[2])];
    end else begin
        // a was active -> a becomes idle after swap -> preload a likewise
        tuning_word_stage_2_a <= slot_tuning_word[2][inc_idx(next_idx[2], list_depth[2])];
        phase_stage_2_a <= slot_phase[2][inc_idx(next_idx[2], list_depth[2])];
        start_amp_stage_2_a <= slot_start_amp[2][inc_idx(next_idx[2], list_depth[2])];
        step_stage_2_a <= slot_step[2][inc_idx(next_idx[2], list_depth[2])];
        duration_cycles_stage_2_a <= slot_duration_cycles[2][inc_idx(next_idx[2], list_depth[2])];
        loop_mode_stage_2_a <= slot_loop_mode[2][inc_idx(next_idx[2], list_depth[2])];
    end

    mux_sel_3 <= ~mux_sel_3;
    current_idx[3] <= next_idx[3];
    next_idx[3]    <= inc_idx(next_idx[3], list_depth[3]);
    if (mux_sel_3) begin
        // b was active -> b becomes idle after swap -> preload b with
        // the slot needed at the FOLLOWING trigger (post-increment next_idx)
        tuning_word_stage_3_b <= slot_tuning_word[3][inc_idx(next_idx[3], list_depth[3])];
        phase_stage_3_b <= slot_phase[3][inc_idx(next_idx[3], list_depth[3])];
        start_amp_stage_3_b <= slot_start_amp[3][inc_idx(next_idx[3], list_depth[3])];
        step_stage_3_b <= slot_step[3][inc_idx(next_idx[3], list_depth[3])];
        duration_cycles_stage_3_b <= slot_duration_cycles[3][inc_idx(next_idx[3], list_depth[3])];
        loop_mode_stage_3_b <= slot_loop_mode[3][inc_idx(next_idx[3], list_depth[3])];
    end else begin
        // a was active -> a becomes idle after swap -> preload a likewise
        tuning_word_stage_3_a <= slot_tuning_word[3][inc_idx(next_idx[3], list_depth[3])];
        phase_stage_3_a <= slot_phase[3][inc_idx(next_idx[3], list_depth[3])];
        start_amp_stage_3_a <= slot_start_amp[3][inc_idx(next_idx[3], list_depth[3])];
        step_stage_3_a <= slot_step[3][inc_idx(next_idx[3], list_depth[3])];
        duration_cycles_stage_3_a <= slot_duration_cycles[3][inc_idx(next_idx[3], list_depth[3])];
        loop_mode_stage_3_a <= slot_loop_mode[3][inc_idx(next_idx[3], list_depth[3])];
    end

end
if (trig_start_2) begin
    mux_sel_4 <= ~mux_sel_4;
    current_idx[4] <= next_idx[4];
    next_idx[4]    <= inc_idx(next_idx[4], list_depth[4]);
    if (mux_sel_4) begin
        // b was active -> b becomes idle after swap -> preload b with
        // the slot needed at the FOLLOWING trigger (post-increment next_idx)
        tuning_word_stage_4_b <= slot_tuning_word[4][inc_idx(next_idx[4], list_depth[4])];
        phase_stage_4_b <= slot_phase[4][inc_idx(next_idx[4], list_depth[4])];
        start_amp_stage_4_b <= slot_start_amp[4][inc_idx(next_idx[4], list_depth[4])];
        step_stage_4_b <= slot_step[4][inc_idx(next_idx[4], list_depth[4])];
        duration_cycles_stage_4_b <= slot_duration_cycles[4][inc_idx(next_idx[4], list_depth[4])];
        loop_mode_stage_4_b <= slot_loop_mode[4][inc_idx(next_idx[4], list_depth[4])];
    end else begin
        // a was active -> a becomes idle after swap -> preload a likewise
        tuning_word_stage_4_a <= slot_tuning_word[4][inc_idx(next_idx[4], list_depth[4])];
        phase_stage_4_a <= slot_phase[4][inc_idx(next_idx[4], list_depth[4])];
        start_amp_stage_4_a <= slot_start_amp[4][inc_idx(next_idx[4], list_depth[4])];
        step_stage_4_a <= slot_step[4][inc_idx(next_idx[4], list_depth[4])];
        duration_cycles_stage_4_a <= slot_duration_cycles[4][inc_idx(next_idx[4], list_depth[4])];
        loop_mode_stage_4_a <= slot_loop_mode[4][inc_idx(next_idx[4], list_depth[4])];
    end

    mux_sel_5 <= ~mux_sel_5;
    current_idx[5] <= next_idx[5];
    next_idx[5]    <= inc_idx(next_idx[5], list_depth[5]);
    if (mux_sel_5) begin
        // b was active -> b becomes idle after swap -> preload b with
        // the slot needed at the FOLLOWING trigger (post-increment next_idx)
        tuning_word_stage_5_b <= slot_tuning_word[5][inc_idx(next_idx[5], list_depth[5])];
        phase_stage_5_b <= slot_phase[5][inc_idx(next_idx[5], list_depth[5])];
        start_amp_stage_5_b <= slot_start_amp[5][inc_idx(next_idx[5], list_depth[5])];
        step_stage_5_b <= slot_step[5][inc_idx(next_idx[5], list_depth[5])];
        duration_cycles_stage_5_b <= slot_duration_cycles[5][inc_idx(next_idx[5], list_depth[5])];
        loop_mode_stage_5_b <= slot_loop_mode[5][inc_idx(next_idx[5], list_depth[5])];
    end else begin
        // a was active -> a becomes idle after swap -> preload a likewise
        tuning_word_stage_5_a <= slot_tuning_word[5][inc_idx(next_idx[5], list_depth[5])];
        phase_stage_5_a <= slot_phase[5][inc_idx(next_idx[5], list_depth[5])];
        start_amp_stage_5_a <= slot_start_amp[5][inc_idx(next_idx[5], list_depth[5])];
        step_stage_5_a <= slot_step[5][inc_idx(next_idx[5], list_depth[5])];
        duration_cycles_stage_5_a <= slot_duration_cycles[5][inc_idx(next_idx[5], list_depth[5])];
        loop_mode_stage_5_a <= slot_loop_mode[5][inc_idx(next_idx[5], list_depth[5])];
    end

end
if (trig_start_3) begin
    mux_sel_6 <= ~mux_sel_6;
    current_idx[6] <= next_idx[6];
    next_idx[6]    <= inc_idx(next_idx[6], list_depth[6]);
    if (mux_sel_6) begin
        // b was active -> b becomes idle after swap -> preload b with
        // the slot needed at the FOLLOWING trigger (post-increment next_idx)
        tuning_word_stage_6_b <= slot_tuning_word[6][inc_idx(next_idx[6], list_depth[6])];
        phase_stage_6_b <= slot_phase[6][inc_idx(next_idx[6], list_depth[6])];
        start_amp_stage_6_b <= slot_start_amp[6][inc_idx(next_idx[6], list_depth[6])];
        step_stage_6_b <= slot_step[6][inc_idx(next_idx[6], list_depth[6])];
        duration_cycles_stage_6_b <= slot_duration_cycles[6][inc_idx(next_idx[6], list_depth[6])];
        loop_mode_stage_6_b <= slot_loop_mode[6][inc_idx(next_idx[6], list_depth[6])];
    end else begin
        // a was active -> a becomes idle after swap -> preload a likewise
        tuning_word_stage_6_a <= slot_tuning_word[6][inc_idx(next_idx[6], list_depth[6])];
        phase_stage_6_a <= slot_phase[6][inc_idx(next_idx[6], list_depth[6])];
        start_amp_stage_6_a <= slot_start_amp[6][inc_idx(next_idx[6], list_depth[6])];
        step_stage_6_a <= slot_step[6][inc_idx(next_idx[6], list_depth[6])];
        duration_cycles_stage_6_a <= slot_duration_cycles[6][inc_idx(next_idx[6], list_depth[6])];
        loop_mode_stage_6_a <= slot_loop_mode[6][inc_idx(next_idx[6], list_depth[6])];
    end

    mux_sel_7 <= ~mux_sel_7;
    current_idx[7] <= next_idx[7];
    next_idx[7]    <= inc_idx(next_idx[7], list_depth[7]);
    if (mux_sel_7) begin
        // b was active -> b becomes idle after swap -> preload b with
        // the slot needed at the FOLLOWING trigger (post-increment next_idx)
        tuning_word_stage_7_b <= slot_tuning_word[7][inc_idx(next_idx[7], list_depth[7])];
        phase_stage_7_b <= slot_phase[7][inc_idx(next_idx[7], list_depth[7])];
        start_amp_stage_7_b <= slot_start_amp[7][inc_idx(next_idx[7], list_depth[7])];
        step_stage_7_b <= slot_step[7][inc_idx(next_idx[7], list_depth[7])];
        duration_cycles_stage_7_b <= slot_duration_cycles[7][inc_idx(next_idx[7], list_depth[7])];
        loop_mode_stage_7_b <= slot_loop_mode[7][inc_idx(next_idx[7], list_depth[7])];
    end else begin
        // a was active -> a becomes idle after swap -> preload a likewise
        tuning_word_stage_7_a <= slot_tuning_word[7][inc_idx(next_idx[7], list_depth[7])];
        phase_stage_7_a <= slot_phase[7][inc_idx(next_idx[7], list_depth[7])];
        start_amp_stage_7_a <= slot_start_amp[7][inc_idx(next_idx[7], list_depth[7])];
        step_stage_7_a <= slot_step[7][inc_idx(next_idx[7], list_depth[7])];
        duration_cycles_stage_7_a <= slot_duration_cycles[7][inc_idx(next_idx[7], list_depth[7])];
        loop_mode_stage_7_a <= slot_loop_mode[7][inc_idx(next_idx[7], list_depth[7])];
    end

end
        end
    end

endmodule
`default_nettype wire
