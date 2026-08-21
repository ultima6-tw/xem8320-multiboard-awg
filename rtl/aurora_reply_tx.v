`timescale 1ns/1ps
`default_nettype none

// aurora_reply_tx.v -- 2026-07-27 新增，統一讀取/寫入架構的查詢回覆組
// 裝器（見 PROJECT.md「統一讀取/寫入架構 — 完整規格」小節）。
//
// 背景：`local_reg_handler.v` 收到 `T_QUERY`(0x12) 後產生
// `req_reply`/`reply_dest_id`/`req_query_type` 三個 sys_clk domain
// pulse/資料，這個模組負責依 `req_query_type` 組裝對應的
// `T_STATUS_REPORT`(0x11) 封包內容，送出一個標準 AXI-stream-like 介面
// （`reply_tdata`/`reply_tvalid`/`reply_tlast`/`reply_tready`），下游
// 由 create_bd.tcl 接一個 xpm_fifo_async（sys_clk -> aurora_clk，比照
// 既有 RX 方向 async FIFO 的做法）送進 aurora_tx1_arbiter.v 的第三個
// 輸入（見該檔案 2026-07-27 擴充說明）。
//
// query_type 對照（跟 host 端一致，見 PROJECT.md）：
//   0 = QT_BOARD_INFO    board_id/is_master/total_boards/board_index/
//                        per_hop_value/channel_up_0-1/init_ok/ext_clk_sel
//   1 = QT_SINE_STATUS   8-channel sine_gen/amp_ramp_gen 設定值 + active
//                        mux_sel + phase_acc（即時值）
//   2 = QT_CALIB_STATUS  scale_cfg + amp_ctrl x8 + calib_coef x32
//   3 = QT_TRIGGER_GROUP 各 channel 的 group_id 分組設定
//
// 封包格式：beat0=標準 header({pkt_len,type=0x11,src_id=own board_id,
// dest_id=reply_dest_id})，beat1[7:0]=query_type（其餘保留0），
// beat2..N=對應 query_type 的 payload，每 beat 64-bit，不足一個 word
// 邊界的部分在最高位補 0（見下方 payload_* 組裝，pkt_len =
// 2 + word_count，跟 local_reg_handler.v 收到 T_STATUS_REPORT 時的
// REPLY_MAX_WORDS=30 累加器對應，本模組最大用到 29 words，見
// QT_SINE_STATUS_WORDS）。
//
// CDC 設計（跟 PROJECT.md 討論記錄一致，這裡逐一列出）：
//   - phase_acc（QT_SINE_STATUS 用）：dac_clk domain 即時累加器，不是
//     準靜態訊號，用官方 xpm_cdc_handshake（WIDTH=256）安全跨到
//     sys_clk。設計成「dac_clk 端持續自由跑：凍結目前值→送出→等對方
//     收到→再凍結下一次」，sys_clk 端隨時讀 dest_out 即可拿到「最近一次
//     成功跨域」的快照，容許數個 dac_clk cycle 的新鮮度誤差（非即時
//     查詢，可接受，比另外設計 request/ack 反向路徑簡單且風險更低）。
//   - 其餘所有欄位（board_id/is_master/total_boards/board_index/
//     channel_up/init_ok/ext_clk_sel/per_hop_value/sine stage 值/
//     scale_cfg/amp_ctrl/calib_coef/group_id）都已經是 sys_clk domain
//     的準靜態值（board_cfg_reg.v/sine_ctrl_regs.v/awg_calib_regs.v/
//     既有 level_cdc 實例算出來的），直接接即可，不需要額外 CDC。
//
// ✅ 2026-07-28 起已完成 xsim 驗證（這行原本寫「還沒做」，已過時，
// 2026-07-29 訂正）：phase_acc handshake 時序見 sim/tb_aurora_reply_
// tx_phase_acc_cdc.v（真正 xpm_cdc_handshake）；4 種 query_type 端
// 對端見 sim/tb_query_status_report.v；含真實 async_fifo_reply_tx
// IP + aurora_tx1_arbiter.v、重現真實查詢順序見 sim/tb_reply_
// arbiter_query_sequence.v；完整 3 板環路見 sim/tb_query_ring_relay_
// full_arbiter.v。全部 PASS，本模組的 write 路徑邏輯已充分驗證，見
// NOTES.md 2026-07-29「`aurora_reply_tx.v` write 路徑其實已充分驗證」
// 章節。

module aurora_reply_tx #(
    parameter MAXW = 30   // 跟 local_reg_handler.v 的 REPLY_MAX_WORDS 一致
) (
    input  wire        sys_clk,
    input  wire        sys_rst,

    // ── 來自 local_reg_handler.v（T_QUERY 解碼）────────────────────────
    input  wire        req_reply,
    input  wire [15:0] reply_dest_id,
    input  wire [7:0]  req_query_type,

    // ── QT_BOARD_INFO 欄位（全部已是 sys_clk domain 準靜態值）─────────
    input  wire [15:0] bi_board_id,      // 也當作 own board_id 用於 beat0 src_id
    input  wire        bi_is_master,
    input  wire [4:0]  bi_total_boards,
    input  wire [4:0]  bi_board_index,
    input  wire [31:0] bi_per_hop_value,
    input  wire        bi_channel_up_0,
    input  wire        bi_channel_up_1,
    input  wire        bi_init_ok,
    input  wire        bi_ext_clk_sel,
    // 2026-07-30 新增：trigger delay 手動覆寫狀態，跟 aurora_reply_tx_0
    // 同一個 sys_clk domain（local_reg_handler_0 直接輸出），不需要 CDC。
    input  wire [15:0] bi_trig_delay,          // au_trig_delay（手動覆寫值本身）
    input  wire        bi_manual_delay_active, // manual_delay_override_active（是否處於覆寫狀態）
    // 2026-07-30 新增：web 控制介面 master-only USB 需求，補上 T_QUERY
    // 沒涵蓋的兩個欄位，見 NOTES.md「QT_BOARD_INFO 擴充規格」。都已是
    // sys_clk domain 準靜態值，不需要額外 CDC。
    input  wire [11:0] bi_dac_mode_ramp,      // board_cfg_reg_0/dac_mode_ramp
    input  wire [31:0] bi_ext_clk_freq_count, // ext_clk_freq_counter_0/freq_count_sync
    // 2026-08-20 新增：dispatcher_0/diag_tx_timeout_seen（sys_clk domain，
    // 不需要 CDC），塞進下面 board_info_word1 原本剩的 3-bit padding
    // 裡——master 送 T_QUERY(QT_BOARD_INFO) 給任一片板子就能問到「這片
    // 板子自己往下一棒送封包時，link 有搭起來但曾經逾時過嗎」，見
    // rtl/dispatcher.v tx_timeout 說明/PROJECT.md「test1 面板」章節。
    input  wire        bi_tx_timeout_seen,
    // 2026-08-20 Phase 7 新增：DIAG（enum 失敗斷點定位）查詢結果，已透過
    // level_cdc 從 aurora_ctrl_channel_0（aurora_clk）跨到 sys_clk，見
    // create_bd.tcl diag_ok_cdc_0/diag_busy_cdc_0/diag_r_channel_up_0_cdc_0/
    // diag_r_channel_up_1_cdc_0/diag_r_relay_blocked_cdc_0。word1 原本的
    // 3-bit padding已被 bi_tx_timeout_seen 用掉 1 bit 剩 2 bit，5 個新欄位
    // 放不下，改開 board_info_word2（見下方），QT_BOARD_INFO_WORDS 從 2
    // 擴充為 3。
    input  wire        bi_diag_ok,
    input  wire        bi_diag_busy,
    input  wire        bi_diag_r_channel_up_0,
    input  wire        bi_diag_r_channel_up_1,
    input  wire        bi_diag_r_relay_blocked,

    // ── QT_DDR_STATUS 欄位（2026-07-30 新增，dac_clk domain 經
    // level_cdc 跨到 sys_clk，見 create_bd.tcl ddr_status_cdc_0~3）：
    // 4 個 channel 各 11-bit，格式跟既有 WO 0x30-33／concat_port_
    // status_$ch 完全一致（bits[2:0]=current_idx、bits[5:3]=next_idx、
    // bit[6]=mux_sel、bits[10:7]=play_pos signed），不重新設計欄位
    // 格式，直接沿用 ────────────────────────────────────────────────
    input  wire [10:0] di_ddr_status_ch0,
    input  wire [10:0] di_ddr_status_ch1,
    input  wire [10:0] di_ddr_status_ch2,
    input  wire [10:0] di_ddr_status_ch3,

    // ── QT_SINE_STATUS 欄位 ────────────────────────────────────────────
    // sine_stage_a/b：8 channel x 6 個 32-bit 參數（tuning_word/phase/
    // start_amp/step/duration_cycles/loop_mode，channel-major、
    // param-minor，跟 create_bd.tcl 的 xlconcat 打包順序一致，見該檔案
    // 接線時的說明），sys_clk domain（sine_ctrl_regs.v 的 *_stage output）
    input  wire [8*6*32-1:0] sine_stage_a,
    input  wire [8*6*32-1:0] sine_stage_b,
    input  wire [7:0]        sine_mux_sel_sync,  // sine_ctrl_regs_0/mux_sel_sync
    // phase_acc_active：dac_clk domain，來自 sine_phase_acc_mux_0（已經
    // 依 dac_clk 版 mux_sel 選好 active 側），這個模組內部做 CDC
    input  wire        dac_clk,
    input  wire        dac_rst,
    input  wire [255:0] phase_acc_active,

    // ── QT_CALIB_STATUS 欄位（sys_clk domain）─────────────────────────
    input  wire [7:0]        cal_scale_cfg,     // board_cfg_reg_0/scale_cfg
    input  wire [8*18-1:0]   cal_amp_ctrl,       // aurora_ctrl_mux_0/out_amp_ctrl_0..7 攤平
    input  wire [32*18-1:0]  cal_coef_all,       // awg_calib_regs_0/coef_all

    // ── QT_TRIGGER_GROUP 欄位（sys_clk domain，board_cfg_reg_0）───────
    input  wire [1:0] grp_id_a,
    input  wire [1:0] grp_id_b,
    input  wire [1:0] grp_id_c,
    input  wire [1:0] grp_id_d,

    // ── QT_FLASH_STATUS 欄位（2026-07-31 新增，見 NOTES.md「QT_FLASH_
    // STATUS 擴充規格」）：fpga_flash_ctrl_0/flash_status 是 okClk
    // domain，經 create_bd.tcl 的 xpm_cdc_array_single 同步到 sys_clk
    // 才接進來，跟 QT_BOARD_INFO 那批 sys_clk 準靜態值不同，這個需要
    // 額外 CDC ────────────────────────────────────────────────────
    input  wire [3:0] bi_flash_status,   // [0]=busy [1]=done [2]=err [3]=loader_done
    // 2026-07-31 同一天擴充：flash 實際內容（跟 bi_flash_status 一起
    // 併入同一個 QT_FLASH_STATUS 回覆，不新增 query_type）
    input  wire [7:0]       bi_flash_scale_cfg, // fpga_flash_ctrl_0/init_scale_cfg，CDC 後 sys_clk
    input  wire [32*18-1:0] bi_flash_coef_all,  // fpga_flash_ctrl_0/init_coef_all，CDC 後 sys_clk

    // ── 輸出：sys_clk domain AXI-stream-like 介面 ─────────────────────
    output wire [63:0] reply_tdata,
    output wire        reply_tvalid,
    output wire        reply_tlast,
    input  wire        reply_tready,

    // ── 2026-08 新增：T_QUERY 回覆本機捷徑（修 PROJECT.md 記錄的已知
    // 設計缺口）────────────────────────────────────────────────────────
    // 原設計裡，即使是「查自己」（reply_dest_id==bi_board_id），回覆
    // 封包還是無條件走 reply_tdata/tvalid，得真的送出實體 Aurora TX、
    // 繞完整個環路才會被 local_reg_handler.v 解碼寫回，SFP 沒接或環路
    // 沒通時永遠讀不到（見 NOTES.md 2026-07-29/2026-08-05 對應記錄）。
    // 這 4 個新 output 直接接 local_reg_handler_0 對應同名 input（都在
    // sys_clk domain，不需要 CDC，見 create_bd.tcl），比照 dispatcher.v
    // 對「請求」封包本來就有的 dest_id==自己→RT_LOCAL 判斷，這裡是幫
    // 「回覆」封包補上對稱的本機捷徑。是自己查自己時，下面的送出狀態機
    // 完全不會進入 ST_HDR（見該邏輯改動），不會產生任何 reply_tvalid。
    output wire        local_reply_valid,
    output wire [15:0] local_reply_src,
    output wire [7:0]  local_reply_query_type,
    output wire [MAXW*64-1:0] local_reply_payload
);

    // dest_id 就是自己 board_id → 本機捷徑，不進送出狀態機
    wire is_self_query = (reply_dest_id == bi_board_id);

    assign local_reply_valid      = req_reply && is_self_query;
    assign local_reply_src        = bi_board_id;
    assign local_reply_query_type = req_query_type;
    assign local_reply_payload    = payload_c;

    localparam [7:0] QT_BOARD_INFO    = 8'd0;
    localparam [7:0] QT_SINE_STATUS   = 8'd1;
    localparam [7:0] QT_CALIB_STATUS  = 8'd2;
    localparam [7:0] QT_TRIGGER_GROUP = 8'd3;
    localparam [7:0] QT_DDR_STATUS    = 8'd4;   // 2026-07-30 新增
    localparam [7:0] QT_FLASH_STATUS  = 8'd5;   // 2026-07-31 新增

    localparam TYPE_STATUS_REPORT = 8'h11;

    // 2026-07-30：QT_BOARD_INFO 因為新增 bi_trig_delay(16)+bi_manual_
    // delay_active(1) 共 17 bit，原本 63 bit 的內容變成 80 bit，超過
    // 單一 64-bit word，WORDS 從 1 改成 2（host 端 decode_rt_board_
    // info 需要同步更新讀 2 個 word，見 host/awg_common.py）。
    // 2026-08-20 Phase 7：DIAG 5 個新欄位（bi_diag_ok/busy/r_channel_up_0/
    // r_channel_up_1/r_relay_blocked）word1 剩的 2-bit padding 塞不下，
    // 新開 board_info_word2，WORDS 從 2 再擴充為 3（host 端同步更新讀
    // 3 個 word，見 host/awg_common.py decode_qt_board_info）。
    localparam QT_BOARD_INFO_WORDS    = 8'd3;
    localparam QT_SINE_STATUS_WORDS   = 8'd29;
    localparam QT_CALIB_STATUS_WORDS  = 8'd12;
    localparam QT_TRIGGER_GROUP_WORDS = 8'd1;
    localparam QT_DDR_STATUS_WORDS    = 8'd1;   // 4*11=44 bit，一個 word 夠
    localparam QT_FLASH_STATUS_WORDS  = 8'd10;  // 2026-07-31 擴充：4+8+576=588 bit，10 個 word

    // ══════════════════════════════════════════════════════════════════
    //  phase_acc CDC：dac_clk 自由跑快照 pump + xpm_cdc_handshake
    // ══════════════════════════════════════════════════════════════════
    reg [255:0] phase_acc_snap_r;
    reg         phase_acc_send_r;
    wire        phase_acc_rcv;
    wire [255:0] phase_acc_synced;

    always @(posedge dac_clk) begin
        if (dac_rst) begin
            phase_acc_snap_r <= 256'd0;
            phase_acc_send_r <= 1'b0;
        end else if (!phase_acc_send_r) begin
            // 凍結目前值，開始送出
            phase_acc_snap_r <= phase_acc_active;
            phase_acc_send_r <= 1'b1;
        end else if (phase_acc_rcv) begin
            // 對方已收到，準備下一次凍結
            phase_acc_send_r <= 1'b0;
        end
    end

    xpm_cdc_handshake #(
        .DEST_EXT_HSK (0),   // 目的端自動 ack，不需要 sys_clk 端額外邏輯
        .DEST_SYNC_FF (4),
        .SRC_SYNC_FF  (4),
        .WIDTH        (256)
    ) u_phase_acc_cdc (
        .src_clk  (dac_clk),
        .src_in   (phase_acc_snap_r),
        .src_send (phase_acc_send_r),
        .src_rcv  (phase_acc_rcv),
        .dest_clk (sys_clk),
        .dest_out (phase_acc_synced),
        .dest_req (),
        .dest_ack (1'b0)
    );

    // ══════════════════════════════════════════════════════════════════
    //  Payload 組裝（純組合邏輯，依 query_type 選擇，見檔頭 word 配置）
    // ══════════════════════════════════════════════════════════════════

    // -- QT_SINE_STATUS：先依 sine_mux_sel_sync 選出每個 channel 的 active
    //    6-param block（跟 sine_ctrl_regs.v 內部的 mux_sel 語意一致：
    //    0=a active，1=b active）--------------------------------------
    wire [8*6*32-1:0] sine_stage_eff;
    genvar sci;
    generate
        for (sci = 0; sci < 8; sci = sci + 1) begin : g_sine_eff
            assign sine_stage_eff[sci*6*32 +: 6*32] =
                sine_mux_sel_sync[sci] ? sine_stage_b[sci*6*32 +: 6*32]
                                       : sine_stage_a[sci*6*32 +: 6*32];
        end
    endgenerate

    // 2026-07-30：原本 63 bit 補 1 bit padding 湊滿一個 64-bit word
    // （word0），新增的 bi_trig_delay/bi_manual_delay_active 從 word1
    // 的最低位開始擺，不跨 word 邊界，host 端解碼比較單純。
    wire [63:0] board_info_word0 = {1'b0, bi_per_hop_value, bi_ext_clk_sel,
                                      bi_init_ok, bi_channel_up_1, bi_channel_up_0,
                                      bi_board_index, bi_total_boards,
                                      bi_is_master, bi_board_id};
    // 2026-07-30 新增：bi_dac_mode_ramp(12)/bi_ext_clk_freq_count(32) 接續
    // 擺在 bi_manual_delay_active(1)/bi_trig_delay(16) 之後，
    // 3+32+12+1+16=64，剛好填滿 word1，不需要新增 word2。
    // 2026-08-20：3-bit padding 用掉其中 1 bit塞 bi_tx_timeout_seen，
    // 還剩 2-bit padding 給之後用。host 端對應 bit 位置見 awg_common.py
    // decode_qt_board_info()。
    wire [63:0] board_info_word1 = {2'd0, bi_tx_timeout_seen, bi_ext_clk_freq_count, bi_dac_mode_ramp,
                                      bi_manual_delay_active, bi_trig_delay};
    // 2026-08-20 Phase 7 新增：DIAG 查詢結果 5 bit，word1 已無空間，開
    // word2，其餘 59 bit padding 給之後用。host 端對應 bit 位置見
    // awg_common.py decode_qt_board_info()。
    wire [63:0] board_info_word2 = {59'd0, bi_diag_r_relay_blocked, bi_diag_r_channel_up_1,
                                      bi_diag_r_channel_up_0, bi_diag_busy, bi_diag_ok};
    wire [191:0] board_info_bits = {board_info_word2, board_info_word1, board_info_word0};
    wire [MAXW*64-1:0] payload_board_info = {{(MAXW*64-192){1'b0}}, board_info_bits};

    wire [1799:0] sine_status_bits = {phase_acc_synced, sine_mux_sel_sync, sine_stage_eff};
    wire [MAXW*64-1:0] payload_sine = {{(MAXW*64-1800){1'b0}}, sine_status_bits};

    wire [727:0] calib_status_bits = {cal_coef_all, cal_amp_ctrl, cal_scale_cfg};
    wire [MAXW*64-1:0] payload_calib = {{(MAXW*64-728){1'b0}}, calib_status_bits};

    wire [7:0] trigger_group_bits = {grp_id_d, grp_id_c, grp_id_b, grp_id_a};
    wire [MAXW*64-1:0] payload_trigger = {{(MAXW*64-8){1'b0}}, trigger_group_bits};

    // 2026-07-30 新增：QT_DDR_STATUS，channel 0 在最低位，跟既有 WO
    // 0x30-33 的 channel 順序（A/B/C/D = ch0/1/2/3）一致
    wire [43:0] ddr_status_bits = {di_ddr_status_ch3, di_ddr_status_ch2,
                                    di_ddr_status_ch1, di_ddr_status_ch0};
    wire [MAXW*64-1:0] payload_ddr_status = {{(MAXW*64-44){1'b0}}, ddr_status_bits};

    // 2026-07-31 擴充：QT_FLASH_STATUS 加上 flash 實際內容
    // （scale_cfg/calib_coef），bit 排列比照 calib_status_bits
    // 「高位放陣列、低位放單一欄位」同一種排法：
    //   bit[3:0]    bi_flash_status
    //   bit[11:4]   bi_flash_scale_cfg
    //   bit[587:12] bi_flash_coef_all
    wire [587:0] flash_status_bits = {bi_flash_coef_all, bi_flash_scale_cfg, bi_flash_status};
    wire [MAXW*64-1:0] payload_flash_status = {{(MAXW*64-588){1'b0}}, flash_status_bits};

    reg [MAXW*64-1:0] payload_c;
    reg [7:0]         word_count_c;
    always @(*) begin
        case (req_query_type)
            QT_BOARD_INFO:    begin payload_c = payload_board_info; word_count_c = QT_BOARD_INFO_WORDS;    end
            QT_SINE_STATUS:   begin payload_c = payload_sine;       word_count_c = QT_SINE_STATUS_WORDS;   end
            QT_CALIB_STATUS:  begin payload_c = payload_calib;      word_count_c = QT_CALIB_STATUS_WORDS;  end
            QT_TRIGGER_GROUP: begin payload_c = payload_trigger;    word_count_c = QT_TRIGGER_GROUP_WORDS; end
            QT_DDR_STATUS:    begin payload_c = payload_ddr_status; word_count_c = QT_DDR_STATUS_WORDS;    end
            QT_FLASH_STATUS:  begin payload_c = payload_flash_status; word_count_c = QT_FLASH_STATUS_WORDS; end
            default:          begin payload_c = {(MAXW*64){1'b0}};  word_count_c = 8'd1;                   end
        endcase
    end

    // ══════════════════════════════════════════════════════════════════
    //  送出狀態機（sys_clk domain）：HDR(beat0) -> QTYPE(beat1) -> PAYLOAD(beat2..N)
    // ══════════════════════════════════════════════════════════════════
    localparam ST_IDLE    = 2'd0;
    localparam ST_HDR     = 2'd1;
    localparam ST_QTYPE   = 2'd2;
    localparam ST_PAYLOAD = 2'd3;

    reg [1:0]  state;
    reg [15:0] dest_id_r;
    reg [7:0]  query_type_r;
    reg [23:0] pkt_len_r;
    reg [7:0]  word_count_r;
    reg [7:0]  word_idx_r;
    reg [MAXW*64-1:0] payload_shift_r;

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            state <= ST_IDLE;
        end else begin
            case (state)
                ST_IDLE: begin
                    // 2026-08 新增：is_self_query 時完全不進 ST_HDR，走
                    // local_reply_valid 那條本機捷徑（見上方 assign），
                    // 不產生任何 reply_tvalid、不碰實體 Aurora TX。
                    if (req_reply && !is_self_query) begin
                        dest_id_r       <= reply_dest_id;
                        query_type_r    <= req_query_type;
                        payload_shift_r <= payload_c;
                        word_count_r    <= word_count_c;
                        pkt_len_r       <= 24'd2 + {16'd0, word_count_c};
                        word_idx_r      <= 8'd0;
                        state           <= ST_HDR;
                    end
                end
                ST_HDR:   if (reply_tready) state <= ST_QTYPE;
                ST_QTYPE: if (reply_tready) state <= ST_PAYLOAD;
                ST_PAYLOAD: if (reply_tready) begin
                    if (word_idx_r == word_count_r - 8'd1) begin
                        state <= ST_IDLE;
                    end else begin
                        payload_shift_r <= payload_shift_r >> 64;
                        word_idx_r      <= word_idx_r + 8'd1;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

    assign reply_tdata = (state == ST_HDR)   ? {pkt_len_r, TYPE_STATUS_REPORT, bi_board_id, dest_id_r} :
                          (state == ST_QTYPE) ? {56'd0, query_type_r} :
                          (state == ST_PAYLOAD) ? payload_shift_r[63:0] : 64'd0;

    assign reply_tvalid = (state == ST_HDR) || (state == ST_QTYPE) || (state == ST_PAYLOAD);

    assign reply_tlast = (state == ST_PAYLOAD) && (word_idx_r == word_count_r - 8'd1);

endmodule
`default_nettype wire
