`timescale 1ns/1ps
`default_nettype none

// fpga_flash_ctrl — FPGA flash 頂層封裝
//
// 仲裁優先順序：startup_loader → config_writer
// PipeIn 介面：BTPI 32-bit wide，host 先送 64 words，再觸發 fp_flash_cfg_wr
//
// 2026-07-15：新增 target_sector_sel（2-bit）決定這次 fp_flash_cfg_wr/
// fp_flash_cfg_erase 要動哪個 64KB sector（0=身份/1=scale_cfg/2=amp_ctrl/
// 3=calib_coef，「分開儲存」，見 PROJECT.md「Flash 持久化層盤點」章節）。
// 同時把 flash_startup_loader.v 新增的 3 組獨立 load_valid_*/raw_magic_*
// 透傳出去。

module fpga_flash_ctrl (
    input  wire        clk,
    input  wire        rst,

    // ── FrontPanel triggers ───────────────────────────────────────────
    input  wire        fp_flash_cfg_wr,      // TI bit[13]
    input  wire        fp_flash_cfg_erase,   // TI bit[14]

    // ── 目標 sector 選擇（2026-07-15 新增）──────────────────────────────
    input  wire [1:0]  target_sector_sel,    // 0=身份 1=scale_cfg 2=amp_ctrl 3=calib_coef

    // ── PipeIn 資料（BTPI 32-bit wide）────────────────────────────────
    input  wire [31:0] pipe_wdata,
    input  wire        pipe_wvalid,          // btpi80_ep_write

    // ── 狀態輸出 → WireOut ────────────────────────────────────────────
    output wire [3:0]  flash_status,   // [0]=busy [1]=done [2]=err [3]=loader_done

    // ── Startup loader 輸出（上電後有效）─────────────────────────────
    output wire        load_valid,
    output wire [15:0] init_board_id,
    output wire        init_is_master,
    (* X_INTERFACE_IGNORE = "TRUE" *) output wire [95:0] init_dna,
    output wire [17:0] init_amp_ctrl_0,
    output wire [17:0] init_amp_ctrl_1,
    output wire [17:0] init_amp_ctrl_2,
    output wire [17:0] init_amp_ctrl_3,
    output wire [17:0] init_amp_ctrl_4,
    output wire [17:0] init_amp_ctrl_5,
    output wire [17:0] init_amp_ctrl_6,
    output wire [17:0] init_amp_ctrl_7,
    output wire [7:0]  init_scale_cfg,
    // 2026-07-15：scale_cfg/amp_ctrl/calib_coef 各自獨立 load_valid（分別
    // 存在各自的 sector），身份群組（board_id/is_master/DNA/trig_delay）
    // 沿用既有的 load_valid
    output wire        load_valid_scale,
    output wire        load_valid_amp,
    output wire        load_valid_coef,
    output wire [17:0] init_coef_0,  output wire [17:0] init_coef_1,
    output wire [17:0] init_coef_2,  output wire [17:0] init_coef_3,
    output wire [17:0] init_coef_4,  output wire [17:0] init_coef_5,
    output wire [17:0] init_coef_6,  output wire [17:0] init_coef_7,
    output wire [17:0] init_coef_8,  output wire [17:0] init_coef_9,
    output wire [17:0] init_coef_10, output wire [17:0] init_coef_11,
    output wire [17:0] init_coef_12, output wire [17:0] init_coef_13,
    output wire [17:0] init_coef_14, output wire [17:0] init_coef_15,
    output wire [17:0] init_coef_16, output wire [17:0] init_coef_17,
    output wire [17:0] init_coef_18, output wire [17:0] init_coef_19,
    output wire [17:0] init_coef_20, output wire [17:0] init_coef_21,
    output wire [17:0] init_coef_22, output wire [17:0] init_coef_23,
    output wire [17:0] init_coef_24, output wire [17:0] init_coef_25,
    output wire [17:0] init_coef_26, output wire [17:0] init_coef_27,
    output wire [17:0] init_coef_28, output wire [17:0] init_coef_29,
    output wire [17:0] init_coef_30, output wire [17:0] init_coef_31,
    // 2026-07-31 新增：32 個 init_coef_N 攤平打包成一條 bus，給
    // QT_FLASH_STATUS 遠端查詢用（不用在 create_bd.tcl 疊 32 個
    // xlconcat），bit 排列比照 awg_calib_regs.v/coef_all（coef[0] 在
    // 最低位），既有 32 個獨立 port 不動，這是額外再接出去的一份。
    output wire [32*18-1:0] init_coef_all,
    output wire [31:0] raw_magic,
    // 2026-07-15：新 sector 各自的 magic 診斷
    output wire [31:0] raw_magic_scale,
    output wire [31:0] raw_magic_amp,
    output wire [31:0] raw_magic_coef,

    // 2026-07-08 (step 15b) 新增：trig_delay 初始值透傳
    output wire [15:0] init_trig_delay,

    // ── EOS（End of Startup）輸出 → 用於 ZMod AWG reset gating ────────
    output wire        fpga_eos,

    // ── 診斷輸出（2026-07-08 step 15b 新增，透傳 flash_config_writer 內部
    //    state，用來除錯 T_FLASH_ERASE 完全不動作的問題）─────────────────
    output wire [3:0]  diag_cfg_state,
    output wire        diag_cfg_do_program,

    // ── SPI bus 除錯輸出（2026-07-09 新增，見 PROJECT.md 第 30 節）──────
    // CDC/觸發鏈路已確認正常但寫入結果仍是空的，這裡把 spi_clk_w/
    // spi_cs_n_w/spi_mosi_w/spi_miso_w（原本只有 MARK_DEBUG、沒有真正
    // 接出去）跟 spi_flash_ctrl 內部 FSM state/op_cmd/busy/done/err
    // 透傳出來，直接觀察 SPI 實體訊號跟狀態機行為
    output wire        dbg_spi_clk,
    output wire        dbg_spi_cs_n,
    output wire        dbg_spi_mosi,
    output wire        dbg_spi_miso,
    output wire [3:0]  dbg_spi_state,
    output wire [2:0]  dbg_spi_op_cmd,
    output wire        dbg_spi_busy,
    output wire        dbg_spi_done,
    output wire        dbg_spi_err
);

// ── Debug probes ──────────────────────────────────────────────────────
(* MARK_DEBUG="TRUE" *) wire dbg_rst = rst;

// ── EOS 同步器 + latch ────────────────────────────────────────────────
// EOS 來自 STARTUPE3（config clock domain），同步至 clk_100
wire eos_raw;
(* ASYNC_REG = "TRUE" *) reg eos_s0 = 0;
(* ASYNC_REG = "TRUE" *) reg eos_s1 = 0;
reg eos_latched = 0;

always @(posedge clk) begin
    eos_s0 <= eos_raw;
    eos_s1 <= eos_s0;
    if (eos_s1) eos_latched <= 1'b1;
end

assign fpga_eos = eos_latched;

// flash_startup_loader 等 EOS 後才開始，避免與 startup sequence STARTUPE3 衝突
wire startup_rst = rst | !eos_latched;

// ── SPI 訊號 ──────────────────────────────────────────────────────────
(* MARK_DEBUG="TRUE" *) wire spi_clk_w;
(* MARK_DEBUG="TRUE" *) wire spi_cs_n_w;
(* MARK_DEBUG="TRUE" *) wire spi_mosi_w;
(* MARK_DEBUG="TRUE" *) wire spi_miso_w;
wire spi_clkts_w, spi_fcsbts_w;
wire [3:0] spi_dts_w;
wire [3:0] spi_di_w;
assign spi_miso_w = spi_di_w[1];

// 2026-07-09（第 30 節）：SPI bus 除錯訊號透傳
assign dbg_spi_clk   = spi_clk_w;
assign dbg_spi_cs_n  = spi_cs_n_w;
assign dbg_spi_mosi  = spi_mosi_w;
assign dbg_spi_miso  = spi_miso_w;
assign dbg_spi_state = u_spi.state;      // 階層參照 spi_flash_ctrl 內部 FSM state
assign dbg_spi_op_cmd = spi_op_cmd;
assign dbg_spi_busy  = spi_busy;
assign dbg_spi_done  = spi_done;
assign dbg_spi_err   = spi_err;

// ── STARTUPE3 ─────────────────────────────────────────────────────────
STARTUPE3 #(
    .PROG_USR    ("FALSE"),
    .SIM_CCLK_FREQ(0.0)
) u_startupe3 (
    .CFGCLK   (),
    .CFGMCLK  (),
    .DI       (spi_di_w),
    .EOS      (eos_raw),
    .PREQ     (),
    .DO       ({2'b11, 1'b0, spi_mosi_w}),
    .DTS      (spi_dts_w),
    .FCSBO    (spi_cs_n_w),
    .FCSBTS   (spi_fcsbts_w),
    .GSR      (1'b0),
    .GTS      (1'b0),
    .KEYCLEARB(1'b1),
    .PACK     (1'b0),
    .USRCCLKO (spi_clk_w),
    .USRCCLKTS(spi_clkts_w),
    .USRDONEO (1'b1),
    .USRDONETS(1'b0)
);

// ── PipeIn word counter（mod-64，對應 spi_flash_ctrl wbuf index）────────
reg [5:0] word_cnt = 6'd0;
always @(posedge clk) begin
    if (rst)
        word_cnt <= 6'd0;
    else if (pipe_wvalid)
        word_cnt <= word_cnt + 1'b1;
end

// ── 仲裁狀態 ──────────────────────────────────────────────────────────
// 優先順序：startup_loader > config_writer
reg loader_done_r = 1'b0;
wire loader_done_w;

always @(posedge clk) begin
    if (rst)
        loader_done_r <= 1'b0;
    else if (loader_done_w)
        loader_done_r <= 1'b1;
end

// ── SPI 共用介面 ──────────────────────────────────────────────────────
wire spi_op_start, spi_busy, spi_done, spi_err;
wire [2:0]  spi_op_cmd;
wire [23:0] spi_op_addr;
wire [8:0]  spi_op_len;
wire [7:0]  spi_rdata_byte;
wire [7:0]  spi_rdata_idx;
wire        spi_rdata_valid;

// loader 介面
wire loader_op_start;
wire [2:0]  loader_op_cmd;
wire [23:0] loader_op_addr;
wire [8:0]  loader_op_len;

// config writer 介面
wire cfg_op_start;
wire [2:0]  cfg_op_cmd;
wire [23:0] cfg_op_addr;
wire [8:0]  cfg_op_len;
wire [2:0]  cfg_flash_status;

// ── SPI MUX（2 路仲裁）────────────────────────────────────────────────
// loader 未完成時：loader 主導；之後：config_writer 主導
assign spi_op_start = !loader_done_r ? loader_op_start : cfg_op_start;
assign spi_op_cmd   = !loader_done_r ? loader_op_cmd   : cfg_op_cmd;
assign spi_op_addr  = !loader_done_r ? loader_op_addr  : cfg_op_addr;
assign spi_op_len   = !loader_done_r ? loader_op_len   : cfg_op_len;

wire cfg_spi_busy = !loader_done_r ? 1'b1 : spi_busy;
wire cfg_spi_done = !loader_done_r ? 1'b0 : spi_done;
wire cfg_spi_err  = !loader_done_r ? 1'b0 : spi_err;

// ── spi_flash_ctrl ────────────────────────────────────────────────────
spi_flash_ctrl u_spi (
    .clk          (clk),
    .rst          (rst),
    .op_start     (spi_op_start),
    .op_cmd       (spi_op_cmd),
    .op_addr      (spi_op_addr),
    .op_len       (spi_op_len),
    .wdata_word   (pipe_wdata),
    .wdata_widx   (word_cnt),
    .wdata_wvalid (pipe_wvalid),
    .rdata_byte   (spi_rdata_byte),
    .rdata_idx    (spi_rdata_idx),
    .rdata_valid  (spi_rdata_valid),
    .busy         (spi_busy),
    .done         (spi_done),
    .err          (spi_err),
    .spi_clk      (spi_clk_w),
    .spi_clkts    (spi_clkts_w),
    .spi_cs_n     (spi_cs_n_w),
    .spi_fcsbts   (spi_fcsbts_w),
    .spi_mosi     (spi_mosi_w),
    .spi_dts      (spi_dts_w),
    .spi_miso     (spi_miso_w)
);

// ── flash_startup_loader ──────────────────────────────────────────────
flash_startup_loader u_startup_loader (
    .clk         (clk),
    .rst         (startup_rst),
    .op_start    (loader_op_start),
    .op_cmd      (loader_op_cmd),
    .op_addr     (loader_op_addr),
    .op_len      (loader_op_len),
    .spi_busy    (spi_busy),
    .spi_done    (spi_done),
    .spi_err     (spi_err),
    .rdata_byte  (spi_rdata_byte),
    .rdata_idx   (spi_rdata_idx),
    .rdata_valid (spi_rdata_valid),
    .loader_done (loader_done_w),
    .load_valid  (load_valid),
    .init_board_id   (init_board_id),
    .init_is_master  (init_is_master),
    .init_dna        (init_dna),
    .init_amp_ctrl_0 (init_amp_ctrl_0),
    .init_amp_ctrl_1 (init_amp_ctrl_1),
    .init_amp_ctrl_2 (init_amp_ctrl_2),
    .init_amp_ctrl_3 (init_amp_ctrl_3),
    .init_amp_ctrl_4 (init_amp_ctrl_4),
    .init_amp_ctrl_5 (init_amp_ctrl_5),
    .init_amp_ctrl_6 (init_amp_ctrl_6),
    .init_amp_ctrl_7 (init_amp_ctrl_7),
    .init_scale_cfg  (init_scale_cfg),
    .load_valid_scale(load_valid_scale),
    .load_valid_amp  (load_valid_amp),
    .load_valid_coef (load_valid_coef),
    .raw_magic_scale (raw_magic_scale),
    .raw_magic_amp   (raw_magic_amp),
    .raw_magic_coef  (raw_magic_coef),
    .init_coef_0 (init_coef_0),   .init_coef_1 (init_coef_1),
    .init_coef_2 (init_coef_2),   .init_coef_3 (init_coef_3),
    .init_coef_4 (init_coef_4),   .init_coef_5 (init_coef_5),
    .init_coef_6 (init_coef_6),   .init_coef_7 (init_coef_7),
    .init_coef_8 (init_coef_8),   .init_coef_9 (init_coef_9),
    .init_coef_10(init_coef_10),  .init_coef_11(init_coef_11),
    .init_coef_12(init_coef_12),  .init_coef_13(init_coef_13),
    .init_coef_14(init_coef_14),  .init_coef_15(init_coef_15),
    .init_coef_16(init_coef_16),  .init_coef_17(init_coef_17),
    .init_coef_18(init_coef_18),  .init_coef_19(init_coef_19),
    .init_coef_20(init_coef_20),  .init_coef_21(init_coef_21),
    .init_coef_22(init_coef_22),  .init_coef_23(init_coef_23),
    .init_coef_24(init_coef_24),  .init_coef_25(init_coef_25),
    .init_coef_26(init_coef_26),  .init_coef_27(init_coef_27),
    .init_coef_28(init_coef_28),  .init_coef_29(init_coef_29),
    .init_coef_30(init_coef_30),  .init_coef_31(init_coef_31),
    .raw_magic   (raw_magic),
    .init_trig_delay(init_trig_delay)
);

// ── 目標 sector 位址查表（2026-07-15 新增，跟 flash_startup_loader.v
//    用同一組常數，各自獨立宣告——純位址常數，沒有共用狀態，不需要真的
//    共用一份程式碼）─────────────────────────────────────────────────────
localparam FLASH_ADDR_IDENTITY = 24'hC80000;
localparam FLASH_ADDR_SCALE    = 24'hC90000;
localparam FLASH_ADDR_AMP      = 24'hCA0000;
localparam FLASH_ADDR_COEF     = 24'hCB0000;

wire [23:0] cfg_op_base_addr =
    (target_sector_sel == 2'd0) ? FLASH_ADDR_IDENTITY :
    (target_sector_sel == 2'd1) ? FLASH_ADDR_SCALE    :
    (target_sector_sel == 2'd2) ? FLASH_ADDR_AMP      :
                                   FLASH_ADDR_COEF;

// ── flash_config_writer ───────────────────────────────────────────────
flash_config_writer u_writer (
    .clk               (clk),
    .rst               (rst),
    .fp_flash_cfg_wr   (fp_flash_cfg_wr   & loader_done_r),
    .fp_flash_cfg_erase(fp_flash_cfg_erase & loader_done_r),
    .op_base_addr      (cfg_op_base_addr),
    .op_start          (cfg_op_start),
    .op_cmd            (cfg_op_cmd),
    .op_addr           (cfg_op_addr),
    .op_len            (cfg_op_len),
    .spi_busy          (cfg_spi_busy),
    .spi_done          (cfg_spi_done),
    .spi_err           (cfg_spi_err),
    .flash_status      (cfg_flash_status),
    .diag_state        (diag_cfg_state),
    .diag_do_program   (diag_cfg_do_program)
);

// ── 組合 flash_status 輸出 ────────────────────────────────────────────
// [3]=loader_done, [2]=err, [1]=done, [0]=busy
assign flash_status = {loader_done_r, cfg_flash_status};

// 2026-07-31 新增：init_coef_all 打包，coef[0] 在最低位，跟
// awg_calib_regs.v/coef_all 同一種排法
assign init_coef_all = {init_coef_31, init_coef_30, init_coef_29, init_coef_28,
                         init_coef_27, init_coef_26, init_coef_25, init_coef_24,
                         init_coef_23, init_coef_22, init_coef_21, init_coef_20,
                         init_coef_19, init_coef_18, init_coef_17, init_coef_16,
                         init_coef_15, init_coef_14, init_coef_13, init_coef_12,
                         init_coef_11, init_coef_10, init_coef_9,  init_coef_8,
                         init_coef_7,  init_coef_6,  init_coef_5,  init_coef_4,
                         init_coef_3,  init_coef_2,  init_coef_1,  init_coef_0};

endmodule
`default_nettype wire
