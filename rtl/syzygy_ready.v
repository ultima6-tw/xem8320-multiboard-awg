`timescale 1ns/1ps
`default_nettype none

// syzygy_ready.v -- local copy (not the shared awg-test-step-14/rtl/
// syzygy_ready.v)，2026-08-18，理由/慣例跟同一批新增的 diag_cdc.v
// 完全相同，見該檔案檔頭說明。內容跟原檔完全相同，未做任何邏輯改動。
//
// syzygy_ready — 在 MMCM locked & FPGA EOS 後延遲 DELAY_CYCLES 才釋放 ZMod AWG reset
// 目的：給 SYZYGY DC-DC converter（±5V DAC 電源）足夠穩定時間

module syzygy_ready #(
    parameter DELAY_CYCLES = 500_000_000  // 5s @ 100MHz
) (
    input  wire clk,
    input  wire locked,
    input  wire eos,
    output wire rst_n   // 接 zmod_awg/aRst_n（active-low reset）
);

localparam CNT_W = 30;  // 2^30 = 1073M > 500M

reg [CNT_W-1:0] cnt  = 0;
reg             done = 0;
wire            enable = locked & eos;

always @(posedge clk) begin
    if (!enable) begin
        cnt  <= 0;
        done <= 0;
    end else if (!done) begin
        if (cnt == DELAY_CYCLES - 1)
            done <= 1;
        else
            cnt <= cnt + 1;
    end
end

assign rst_n = done;

endmodule
`default_nettype wire
