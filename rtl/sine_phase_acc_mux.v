`timescale 1ns/1ps
`default_nettype none

// sine_phase_acc_mux.v -- 2026-07-27 新增（統一讀取/寫入架構，
// QT_SINE_STATUS 查詢用）。純組合邏輯，dac_clk domain：8 個physical
// channel 各自的 a/b 兩個 sine_gen 實例都有自己的 phase_acc_out，這裡
// 依 mux_sel（dac_clk domain 原始版，不是 sine_ctrl_regs.v 那個 sys_clk
// 已同步的 mux_sel_sync）選出目前 active 那顆的 phase_acc_out，攤平成
// 8×32-bit 輸出。跟 amp_ctrl_read_mux.v 是同一種「純組合邏輯多路選一」
// 慣例。
//
// 為什麼要在 dac_clk domain 先選好，不是把 16 個 phase_acc_out 全部
// 原始送去跨時域：phase_acc 是持續變動的即時累加器（見 aurora_reply_
// tx.v 檔頭說明），跨域一次只做 8 個「目前 active」的值，比對 16 個
// a/b 全部都做跨域省一半硬體，也避免下游還要在 sys_clk domain 重新做
// 一次選擇（那時候 mux_sel 早就跟 phase_acc 不同時間點，語意上更複雜）。

module sine_phase_acc_mux (
    input  wire        mux_sel_0,
    input  wire        mux_sel_1,
    input  wire        mux_sel_2,
    input  wire        mux_sel_3,
    input  wire        mux_sel_4,
    input  wire        mux_sel_5,
    input  wire        mux_sel_6,
    input  wire        mux_sel_7,

    input  wire [31:0] phase_acc_0_a, input wire [31:0] phase_acc_0_b,
    input  wire [31:0] phase_acc_1_a, input wire [31:0] phase_acc_1_b,
    input  wire [31:0] phase_acc_2_a, input wire [31:0] phase_acc_2_b,
    input  wire [31:0] phase_acc_3_a, input wire [31:0] phase_acc_3_b,
    input  wire [31:0] phase_acc_4_a, input wire [31:0] phase_acc_4_b,
    input  wire [31:0] phase_acc_5_a, input wire [31:0] phase_acc_5_b,
    input  wire [31:0] phase_acc_6_a, input wire [31:0] phase_acc_6_b,
    input  wire [31:0] phase_acc_7_a, input wire [31:0] phase_acc_7_b,

    // 攤平輸出：8 × 32-bit，channel 0 在 bit[31:0]，channel 7 在 bit[255:224]
    output wire [255:0] phase_acc_active
);

    assign phase_acc_active[31:0]    = mux_sel_0 ? phase_acc_0_b : phase_acc_0_a;
    assign phase_acc_active[63:32]   = mux_sel_1 ? phase_acc_1_b : phase_acc_1_a;
    assign phase_acc_active[95:64]   = mux_sel_2 ? phase_acc_2_b : phase_acc_2_a;
    assign phase_acc_active[127:96]  = mux_sel_3 ? phase_acc_3_b : phase_acc_3_a;
    assign phase_acc_active[159:128] = mux_sel_4 ? phase_acc_4_b : phase_acc_4_a;
    assign phase_acc_active[191:160] = mux_sel_5 ? phase_acc_5_b : phase_acc_5_a;
    assign phase_acc_active[223:192] = mux_sel_6 ? phase_acc_6_b : phase_acc_6_a;
    assign phase_acc_active[255:224] = mux_sel_7 ? phase_acc_7_b : phase_acc_7_a;

endmodule
`default_nettype wire
