`timescale 1ns/1ps
`default_nettype none

// aurora_refclk_ibuf.v -- local copy (not the shared awg-test-step-6/
// vivado/rtl/aurora_refclk_ibuf.v), 2026-08-18，理由/慣例跟同一批新增
// 的 fp_ddr4_rw.v 完全相同，見該檔案檔頭說明。內容跟原檔完全相同，
// 未做任何邏輯改動。
//
// aurora_refclk_ibuf — MGTREFCLK0 差動輸入 → IBUFDS_GTE4 → Aurora refclk1_in
// 不加 BUFG_GT，保持 GT 參考時鐘在專用 refclk 網路，不進 fabric。
// X_INTERFACE_PARAMETER 告知 Vivado BD refclk_out 為 125 MHz 時鐘。

module aurora_refclk_ibuf (
    (* X_INTERFACE_INFO      = "xilinx.com:interface:diff_clock_rtl:1.0 refclk CLK_P" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 125000000" *)
    input  wire refclk_p,
    (* X_INTERFACE_INFO = "xilinx.com:interface:diff_clock_rtl:1.0 refclk CLK_N" *)
    input  wire refclk_n,
    (* X_INTERFACE_INFO      = "xilinx.com:signal:clock:1.0 refclk_out CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 125000000" *)
    output wire refclk_out
);
    IBUFDS_GTE4 #(
        .REFCLK_EN_TX_PATH  (1'b0),
        .REFCLK_HROW_CK_SEL (2'b00),
        .REFCLK_ICNTL_RX    (2'b00)
    ) ibuf (
        .I     (refclk_p),
        .IB    (refclk_n),
        .CEB   (1'b0),
        .O     (refclk_out),
        .ODIV2 ()
    );

endmodule

`default_nettype wire
