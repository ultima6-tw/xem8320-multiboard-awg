`timescale 1ns/1ps
`default_nettype none

// okclk_rst_sync — sys_clk 域的 reset -> okClk domain 安全同步
//
// 2026-07-09 新增，見 awg-test-step-16/PROJECT.md 第 25 節。fpga_flash_
// ctrl_0 搬到 okClk 域後，需要一個在 okClk 域安全可用的 reset，供
// fpga_flash_ctrl_0/flash_payload_cdc_0/flash_boot_relay_0/
// flash_trigger_cdc_0 這幾個新的 okClk 域模組共用。
//
// 2026-07-09 修正：rst_in 實際上是 or_flash_ctrl_rst/Res（peripheral_reset
// OR ti_flash_ctrl_rst），其中 ti_flash_ctrl_rst 是 TI bit29 觸發的單週期
// pulse（不是持續 level）——原始版本只做 2-flop 同步、沒有先展寬，narrow
// pulse 可能被 okClk 完全跳過取樣不到。改成沿用 fp_input.v 的 half_reset
// 手法：sys_clk 側先展寬成 16-cycle level（本來就是 level 的
// peripheral_reset 也一起展寬，多撐幾個 cycle 無害），再用 2-flop
// 同步器帶進 okClk domain。

module okclk_rst_sync (
    input  wire sys_clk,
    input  wire ok_clk,
    input  wire rst_in,     // sys_clk 域的既有 reset（peripheral_reset OR debug soft-reset pulse）

    output wire rst_ok      // okClk 域可安全使用的同步後 reset（level）
);

reg [3:0] rst_stretch = 4'd15;   // 上電時預設撐滿，跟其他 reset 一樣預設 asserted

always @(posedge sys_clk) begin
    if (rst_in) begin
        rst_stretch <= 4'd15;
    end else if (rst_stretch != 4'd0) begin
        rst_stretch <= rst_stretch - 4'd1;
    end
end

(* ASYNC_REG = "TRUE" *) reg rst_ok_meta = 1'b1;
reg                          rst_ok_r    = 1'b1;

always @(posedge ok_clk) begin
    rst_ok_meta <= (rst_stretch != 4'd0);
    rst_ok_r    <= rst_ok_meta;
end

assign rst_ok = rst_ok_r;

endmodule

`default_nettype wire
