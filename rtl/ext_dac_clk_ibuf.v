`timescale 1ns/1ps
`default_nettype none

// ext_dac_clk_ibuf — 把 MGTREFCLK1P/N_226（XEM8320 M7/M6，外部 Si5332
// 100MHz 差動時脈輸入）透過 GT quad 專用的 IBUFDS_GTE4 + BUFG_GT 帶進
// fabric，供後面的 Clocking Wizard 產生 AWG 用的 0 度/90 度時脈對。
//
// 2026-07-11 新增，見 PROJECT.md「時脈路徑設計」章節。做法（UltraScale+
// 標準模式，多個獨立來源交叉驗證過，見 PROJECT.md 附的 sources）：
// IBUFDS_GTE4 -> ODIV2 -> BUFG_GT -> fabric clock。
//
// ⚠️ 信心不足、尚未上機驗證的部分（見 PROJECT.md 待辦事項第一項）：
//   - REFCLK_HROW_CK_SEL/ODIV2 的實際分頻行為、BUFG_GT 的 DIV 設定
//     疊加後最終輸出頻率，官方文件查證時沒能拿到明確數字，需要在
//     Vivado 裡實際驗證（Language Template 註解或 clock 頻率報告）
//   - CEB/CE/CLR/CEMASK/CLRMASK 這幾個控制訊號目前用最保守的「一直
//     致能、不清除」設法，尚未對照官方 template 逐一核對
//
// 這顆 refclk 目前沒有任何 GT transceiver 使用它（Aurora 用的是
// REFCLK0/P7-P6，不是這組 REFCLK1/M7-M6），所以 IBUFDS_GTE4 的 O 輸出
// （送給 GT PLL 用）不需要接，只用 ODIV2 這條路徑餵 fabric。

module ext_dac_clk_ibuf (
    (* X_INTERFACE_INFO      = "xilinx.com:interface:diff_clock_rtl:1.0 ext_dac_refclk CLK_P" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 100000000" *)
    input  wire refclk_p,      // MGTREFCLK1P_226，board pin M7，Si5332 100MHz
    (* X_INTERFACE_INFO = "xilinx.com:interface:diff_clock_rtl:1.0 ext_dac_refclk CLK_N" *)
    input  wire refclk_n,      // MGTREFCLK1N_226，board pin M6
    // 2026-07-13：實測 ext_clk_out ≈99.98MHz（measure_ext_clk_freq.py，
    // 見 PROJECT.md「端到端測試結果」），確認 ODIV2+BUFG_GT 疊加後是
    // 1:1 直通，補上 FREQ_HZ 標籤（標稱 100MHz，量測誤差來自軟體讀值
    // 間隔，非硬體時脈本身）。
    (* X_INTERFACE_INFO      = "xilinx.com:signal:clock:1.0 ext_clk_out CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 100000000" *)
    output wire ext_clk_out    // fabric clock，餵給下游 Clocking Wizard
);

    wire odiv2;

    IBUFDS_GTE4 #(
        .REFCLK_EN_TX_PATH  (1'b0),
        .REFCLK_HROW_CK_SEL (2'b00),
        .REFCLK_ICNTL_RX    (2'b00)
    ) u_ibufds_gte4 (
        .O      (),        // 沒有 GT 使用這組 refclk，不接
        .ODIV2  (odiv2),
        .CEB    (1'b0),    // clock enable，低有效，固定致能
        .I      (refclk_p),
        .IB     (refclk_n)
    );

    BUFG_GT u_bufg_gt (
        .O       (ext_clk_out),
        .CE      (1'b1),
        .CEMASK  (1'b0),
        .CLR     (1'b0),
        .CLRMASK (1'b0),
        .DIV     (3'b000), // 0 = 不分頻，直通
        .I       (odiv2)
    );

endmodule
`default_nettype wire
