`timescale 1ns/1ps
`default_nettype none

// diag_cdc.v -- local copy (not the shared awg-test-step-14/rtl/
// diag_cdc.v)，2026-08-18。awg-test-step-14 已搬進 Projects/FPGA/
// _archive/（2026-08-10），create_bd.tcl 原本寫死指向該資料夾的路徑因此
// 失效；比照這個專案既有慣例（多數 $src_dir14 檔案早就已經是「local
// copy」模式），直接複製進來、不再依賴外部/已封存的資料夾。內容跟原檔
// 完全相同，未做任何邏輯改動。
//
// diag_cdc.v — aurora_clk 診斷信號同步到 sys_clk
//
// 1-bit sticky 信號用 2-stage FF 同步
// 32-bit 資料是 quasi-static（set once, never cleared），直接透過

module diag_cdc (
    input  wire        aurora_clk,
    input  wire        sys_clk,

    // aurora_clk 輸入（來自 dispatcher）
    input  wire        in_fp_seen,
    input  wire        in_local_seen,
    input  wire [31:0] in_first_lo,
    input  wire [31:0] in_first_hi,

    // sys_clk 輸出（接 WireOut）
    output reg         out_fp_seen    = 1'b0,
    output reg         out_local_seen = 1'b0,
    output wire [31:0] out_first_lo,
    output wire [31:0] out_first_hi
);

    // 2-stage同步器
    reg fp_s1 = 1'b0, fp_s2 = 1'b0;
    reg ls_s1 = 1'b0, ls_s2 = 1'b0;

    always @(posedge sys_clk) begin
        fp_s1 <= in_fp_seen;
        fp_s2 <= fp_s1;
        ls_s1 <= in_local_seen;
        ls_s2 <= ls_s1;

        // sticky: 一旦 1 就不清除
        if (fp_s2) out_fp_seen    <= 1'b1;
        if (ls_s2) out_local_seen <= 1'b1;
    end

    // quasi-static 32-bit 資料直接穿透（set once, 可接受 CDC 風險）
    assign out_first_lo = in_first_lo;
    assign out_first_hi = in_first_hi;

endmodule

`default_nettype wire
