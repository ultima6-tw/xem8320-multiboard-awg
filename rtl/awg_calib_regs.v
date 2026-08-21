`timescale 1ns/1ps
`default_nettype none

// 2026-07-15（step-16 fork，原檔案共用自 awg-test-step-14/rtl/）：新增
// coef_readback output = coef[calib_sel]（純組合邏輯，跟既有寫入路徑共用
// 同一個 calib_sel 選擇器索引），讓 host 可以讀出目前 32 筆校準係數裡
// 任一筆的即時值。Group 2（scale_cfg/amp_ctrl/calib_coef 讀回）的一部分。
// 其餘邏輯完全不動。

// awg_calib_regs: Calibration coefficient register file for 4 × ZmodAWGController.
//
// Step 9 新增：8 個 amp_ctrl 暫存器（Q1.16 18-bit，預設 0x10000 = ×1.0）
//   MultCoef 輸出 = amp_ctrl × gain_calib，取 [33:16]（Q1.16 × Q1.16 → Q1.16）
//   AddCoef  輸出 = gain_calib（直接，offset 與振幅無關）
//
// Address encoding — calib_sel[4:0]:
//   [1:0] = instance  (0=zmod0, 1=zmod1, 2=zmod2, 3=zmod3)
//   [2]   = channel   (0=Ch1, 1=Ch2)
//   [3]   = mode      (0=HG,  1=LG)
//   [4]   = coef type (0=Mult, 1=Add)
//
// amp_ctrl mapping（8 channels）：
//   amp_ctrl_0 = z0_ch1, amp_ctrl_1 = z0_ch2
//   amp_ctrl_2 = z1_ch1, amp_ctrl_3 = z1_ch2
//   amp_ctrl_4 = z2_ch1, amp_ctrl_5 = z2_ch2
//   amp_ctrl_6 = z3_ch1, amp_ctrl_7 = z3_ch2

module awg_calib_regs (
    input  wire        clk,
    input  wire        rst,         // sync, active high

    // ── Write interface (from FP TI pulse) ────────────────────────────────
    input  wire [4:0]  calib_sel,
    input  wire [17:0] calib_data,
    input  wire        calib_wr,    // single-cycle pulse

    // ── Scale config (quasi-static, direct from WI) ───────────────────────
    input  wire [7:0]  scale_cfg,

    // ── Flash startup loader 初始值（Step 10）────────────────────────────
    input  wire        flash_load_valid,
    input  wire [7:0]  flash_scale_cfg,
    input  wire [17:0] flash_coef_0,  input wire [17:0] flash_coef_1,
    input  wire [17:0] flash_coef_2,  input wire [17:0] flash_coef_3,
    input  wire [17:0] flash_coef_4,  input wire [17:0] flash_coef_5,
    input  wire [17:0] flash_coef_6,  input wire [17:0] flash_coef_7,
    input  wire [17:0] flash_coef_8,  input wire [17:0] flash_coef_9,
    input  wire [17:0] flash_coef_10, input wire [17:0] flash_coef_11,
    input  wire [17:0] flash_coef_12, input wire [17:0] flash_coef_13,
    input  wire [17:0] flash_coef_14, input wire [17:0] flash_coef_15,
    input  wire [17:0] flash_coef_16, input wire [17:0] flash_coef_17,
    input  wire [17:0] flash_coef_18, input wire [17:0] flash_coef_19,
    input  wire [17:0] flash_coef_20, input wire [17:0] flash_coef_21,
    input  wire [17:0] flash_coef_22, input wire [17:0] flash_coef_23,
    input  wire [17:0] flash_coef_24, input wire [17:0] flash_coef_25,
    input  wire [17:0] flash_coef_26, input wire [17:0] flash_coef_27,
    input  wire [17:0] flash_coef_28, input wire [17:0] flash_coef_29,
    input  wire [17:0] flash_coef_30, input wire [17:0] flash_coef_31,

    // ── amp_ctrl inputs（from aurora_ctrl_mux，Q1.16，預設 0x10000）────────
    input  wire [17:0] amp_ctrl_0,  // z0_ch1
    input  wire [17:0] amp_ctrl_1,  // z0_ch2
    input  wire [17:0] amp_ctrl_2,  // z1_ch1
    input  wire [17:0] amp_ctrl_3,  // z1_ch2
    input  wire [17:0] amp_ctrl_4,  // z2_ch1
    input  wire [17:0] amp_ctrl_5,  // z2_ch2
    input  wire [17:0] amp_ctrl_6,  // z3_ch1
    input  wire [17:0] amp_ctrl_7,  // z3_ch2

    // ── ZmodAWG 0 coefficient outputs ─────────────────────────────────────
    output wire [17:0] z0_ch1_hg_mult, output wire [17:0] z0_ch1_hg_add,
    output wire [17:0] z0_ch1_lg_mult, output wire [17:0] z0_ch1_lg_add,
    output wire [17:0] z0_ch2_hg_mult, output wire [17:0] z0_ch2_hg_add,
    output wire [17:0] z0_ch2_lg_mult, output wire [17:0] z0_ch2_lg_add,

    // ── ZmodAWG 1 coefficient outputs ─────────────────────────────────────
    output wire [17:0] z1_ch1_hg_mult, output wire [17:0] z1_ch1_hg_add,
    output wire [17:0] z1_ch1_lg_mult, output wire [17:0] z1_ch1_lg_add,
    output wire [17:0] z1_ch2_hg_mult, output wire [17:0] z1_ch2_hg_add,
    output wire [17:0] z1_ch2_lg_mult, output wire [17:0] z1_ch2_lg_add,

    // ── ZmodAWG 2 coefficient outputs ─────────────────────────────────────
    output wire [17:0] z2_ch1_hg_mult, output wire [17:0] z2_ch1_hg_add,
    output wire [17:0] z2_ch1_lg_mult, output wire [17:0] z2_ch1_lg_add,
    output wire [17:0] z2_ch2_hg_mult, output wire [17:0] z2_ch2_hg_add,
    output wire [17:0] z2_ch2_lg_mult, output wire [17:0] z2_ch2_lg_add,

    // ── ZmodAWG 3 coefficient outputs ─────────────────────────────────────
    output wire [17:0] z3_ch1_hg_mult, output wire [17:0] z3_ch1_hg_add,
    output wire [17:0] z3_ch1_lg_mult, output wire [17:0] z3_ch1_lg_add,
    output wire [17:0] z3_ch2_hg_mult, output wire [17:0] z3_ch2_hg_add,
    output wire [17:0] z3_ch2_lg_mult, output wire [17:0] z3_ch2_lg_add,

    // ── Scale select outputs ───────────────────────────────────────────────
    output wire z0_ch1_scale, output wire z0_ch2_scale,
    output wire z1_ch1_scale, output wire z1_ch2_scale,
    output wire z2_ch1_scale, output wire z2_ch2_scale,
    output wire z3_ch1_scale, output wire z3_ch2_scale,

    // ── 讀回輸出（2026-07-15 新增）─────────────────────────────────────────
    output wire [17:0] coef_readback,

    // ── 2026-07-27 新增：全部 32 筆一次攤平輸出（QT_CALIB_STATUS 查詢
    // 用）。不能沿用 calib_sel/coef_readback 這組單筆索引介面掃描讀取
    // ——calib_sel 同時也是 calib_mux_0 的寫入位址選擇線，aurora_reply_
    // tx.v 掃描讀取會跟寫入互相干擾，所以另開一條純組合邏輯的全量輸出，
    // 不影響既有寫入/單筆讀回路徑。
    output wire [32*18-1:0] coef_all
);

// ── Coefficient register file: 32 entries ─────────────────────────────────
reg [17:0] coef [31:0];

// ── 讀回：coef[calib_sel] 即時值（純組合邏輯，不影響既有寫入時序）───────
assign coef_readback = coef[calib_sel];

// ── 讀回：全部 32 筆攤平輸出（2026-07-27 新增，純組合邏輯）────────────────
genvar cai;
generate
    for (cai = 0; cai < 32; cai = cai + 1) begin : g_coef_all
        assign coef_all[cai*18 +: 18] = coef[cai];
    end
endgenerate

// Flash 初始值匯總（方便 flash_load_valid 時整批載入）
wire [17:0] flash_coef_arr [31:0];
assign flash_coef_arr[0]  = flash_coef_0;  assign flash_coef_arr[1]  = flash_coef_1;
assign flash_coef_arr[2]  = flash_coef_2;  assign flash_coef_arr[3]  = flash_coef_3;
assign flash_coef_arr[4]  = flash_coef_4;  assign flash_coef_arr[5]  = flash_coef_5;
assign flash_coef_arr[6]  = flash_coef_6;  assign flash_coef_arr[7]  = flash_coef_7;
assign flash_coef_arr[8]  = flash_coef_8;  assign flash_coef_arr[9]  = flash_coef_9;
assign flash_coef_arr[10] = flash_coef_10; assign flash_coef_arr[11] = flash_coef_11;
assign flash_coef_arr[12] = flash_coef_12; assign flash_coef_arr[13] = flash_coef_13;
assign flash_coef_arr[14] = flash_coef_14; assign flash_coef_arr[15] = flash_coef_15;
assign flash_coef_arr[16] = flash_coef_16; assign flash_coef_arr[17] = flash_coef_17;
assign flash_coef_arr[18] = flash_coef_18; assign flash_coef_arr[19] = flash_coef_19;
assign flash_coef_arr[20] = flash_coef_20; assign flash_coef_arr[21] = flash_coef_21;
assign flash_coef_arr[22] = flash_coef_22; assign flash_coef_arr[23] = flash_coef_23;
assign flash_coef_arr[24] = flash_coef_24; assign flash_coef_arr[25] = flash_coef_25;
assign flash_coef_arr[26] = flash_coef_26; assign flash_coef_arr[27] = flash_coef_27;
assign flash_coef_arr[28] = flash_coef_28; assign flash_coef_arr[29] = flash_coef_29;
assign flash_coef_arr[30] = flash_coef_30; assign flash_coef_arr[31] = flash_coef_31;

integer i;
always @(posedge clk) begin
    if (rst) begin
        for (i = 0; i < 32; i = i + 1)
            coef[i] <= i[4] ? 18'h00000 : 18'h10000;
    end else if (flash_load_valid) begin
        // 上電 one-shot：從 flash 載入初始值
        for (i = 0; i < 32; i = i + 1)
            coef[i] <= flash_coef_arr[i];
    end else if (calib_wr) begin
        coef[calib_sel] <= calib_data;
    end
end

// scale_cfg：flash 載入值（由外部 mux 在 flash_load_valid 時切換）
// awg_calib_regs 的 scale_cfg 為組合邏輯輸入，不在此模組 register
// 由 BD 外部 mux：flash_load_valid ? flash_scale_cfg : WI_SCALE_CFG

// ── Multipliers：amp_ctrl × gain_calib → final MultCoef ──────────────────
// Q1.16 × Q1.16: 18-bit × 18-bit signed product = 36-bit, 取 [33:16]
wire signed [35:0] prod_z0_ch1_hg = $signed(coef[5'h00]) * $signed(amp_ctrl_0);
wire signed [35:0] prod_z0_ch1_lg = $signed(coef[5'h08]) * $signed(amp_ctrl_0);
wire signed [35:0] prod_z0_ch2_hg = $signed(coef[5'h04]) * $signed(amp_ctrl_1);
wire signed [35:0] prod_z0_ch2_lg = $signed(coef[5'h0C]) * $signed(amp_ctrl_1);

wire signed [35:0] prod_z1_ch1_hg = $signed(coef[5'h01]) * $signed(amp_ctrl_2);
wire signed [35:0] prod_z1_ch1_lg = $signed(coef[5'h09]) * $signed(amp_ctrl_2);
wire signed [35:0] prod_z1_ch2_hg = $signed(coef[5'h05]) * $signed(amp_ctrl_3);
wire signed [35:0] prod_z1_ch2_lg = $signed(coef[5'h0D]) * $signed(amp_ctrl_3);

wire signed [35:0] prod_z2_ch1_hg = $signed(coef[5'h02]) * $signed(amp_ctrl_4);
wire signed [35:0] prod_z2_ch1_lg = $signed(coef[5'h0A]) * $signed(amp_ctrl_4);
wire signed [35:0] prod_z2_ch2_hg = $signed(coef[5'h06]) * $signed(amp_ctrl_5);
wire signed [35:0] prod_z2_ch2_lg = $signed(coef[5'h0E]) * $signed(amp_ctrl_5);

wire signed [35:0] prod_z3_ch1_hg = $signed(coef[5'h03]) * $signed(amp_ctrl_6);
wire signed [35:0] prod_z3_ch1_lg = $signed(coef[5'h0B]) * $signed(amp_ctrl_6);
wire signed [35:0] prod_z3_ch2_hg = $signed(coef[5'h07]) * $signed(amp_ctrl_7);
wire signed [35:0] prod_z3_ch2_lg = $signed(coef[5'h0F]) * $signed(amp_ctrl_7);

// ── Output assignments ────────────────────────────────────────────────────
// inst=0 (z0)
assign z0_ch1_hg_mult = prod_z0_ch1_hg[33:16];
assign z0_ch1_hg_add  = coef[5'h10];
assign z0_ch1_lg_mult = prod_z0_ch1_lg[33:16];
assign z0_ch1_lg_add  = coef[5'h18];
assign z0_ch2_hg_mult = prod_z0_ch2_hg[33:16];
assign z0_ch2_hg_add  = coef[5'h14];
assign z0_ch2_lg_mult = prod_z0_ch2_lg[33:16];
assign z0_ch2_lg_add  = coef[5'h1C];

// inst=1 (z1)
assign z1_ch1_hg_mult = prod_z1_ch1_hg[33:16];
assign z1_ch1_hg_add  = coef[5'h11];
assign z1_ch1_lg_mult = prod_z1_ch1_lg[33:16];
assign z1_ch1_lg_add  = coef[5'h19];
assign z1_ch2_hg_mult = prod_z1_ch2_hg[33:16];
assign z1_ch2_hg_add  = coef[5'h15];
assign z1_ch2_lg_mult = prod_z1_ch2_lg[33:16];
assign z1_ch2_lg_add  = coef[5'h1D];

// inst=2 (z2)
assign z2_ch1_hg_mult = prod_z2_ch1_hg[33:16];
assign z2_ch1_hg_add  = coef[5'h12];
assign z2_ch1_lg_mult = prod_z2_ch1_lg[33:16];
assign z2_ch1_lg_add  = coef[5'h1A];
assign z2_ch2_hg_mult = prod_z2_ch2_hg[33:16];
assign z2_ch2_hg_add  = coef[5'h16];
assign z2_ch2_lg_mult = prod_z2_ch2_lg[33:16];
assign z2_ch2_lg_add  = coef[5'h1E];

// inst=3 (z3)
assign z3_ch1_hg_mult = prod_z3_ch1_hg[33:16];
assign z3_ch1_hg_add  = coef[5'h13];
assign z3_ch1_lg_mult = prod_z3_ch1_lg[33:16];
assign z3_ch1_lg_add  = coef[5'h1B];
assign z3_ch2_hg_mult = prod_z3_ch2_hg[33:16];
assign z3_ch2_hg_add  = coef[5'h17];
assign z3_ch2_lg_mult = prod_z3_ch2_lg[33:16];
assign z3_ch2_lg_add  = coef[5'h1F];

// ── Scale select: direct from WI_SCALE_CFG (quasi-static) ─────────────────
assign z0_ch1_scale = scale_cfg[0];
assign z0_ch2_scale = scale_cfg[1];
assign z1_ch1_scale = scale_cfg[2];
assign z1_ch2_scale = scale_cfg[3];
assign z2_ch1_scale = scale_cfg[4];
assign z2_ch2_scale = scale_cfg[5];
assign z3_ch1_scale = scale_cfg[6];
assign z3_ch2_scale = scale_cfg[7];

endmodule
`default_nettype wire
