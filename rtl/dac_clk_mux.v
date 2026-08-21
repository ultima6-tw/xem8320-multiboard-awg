`timescale 1ns/1ps
`default_nettype none

// dac_clk_mux.v -- local copy (not the shared awg-test-step-7/vivado/
// rtl/dac_clk_mux.v)，2026-08-18。awg-test-step-7 已搬進 Projects/FPGA/
// _archive/（2026-08-10），create_bd.tcl 原本寫死指向該資料夾的路徑因此
// 失效；理由/慣例跟同一批新增的 diag_cdc.v（awg-test-step-14 那邊）
// 完全相同，見該檔案檔頭說明。內容跟原檔完全相同，未做任何邏輯改動。
//
// dac_clk_mux: glitch-free 2-to-1 clock mux using BUFGMUX_CTRL
// S=0 → I0 (internal, e.g. clk_wiz_0 output)
// S=1 → I1 (external, e.g. clk_wiz_1 output from SI5332)
// BUFGMUX_CTRL waits for both clocks low before switching — no output glitch
module dac_clk_mux (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 I0 CLK" *)
    input  wire I0,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 I1 CLK" *)
    input  wire I1,
    input  wire S,
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 O CLK" *)
    output wire O
);
    BUFGMUX_CTRL inst (
        .I0(I0),
        .I1(I1),
        .S(S),
        .O(O)
    );
endmodule
`default_nettype wire
