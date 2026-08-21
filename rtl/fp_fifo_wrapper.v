`timescale 1ns/1ps
`default_nettype none

// fp_fifo_wrapper — xpm_fifo_async 薄殼
//
// 2026-07-10（見 PROJECT.md 第 32 節）：從 fp_input.v 拆出，讓 FIFO 本身
// 在 BD 裡是一個獨立可見的節點（wr_clk=ok_clk / rd_clk=sys_clk 的 CDC
// 邊界一目了然），方便之後在正確的 clock domain 上加探測點。純 1:1
// wrapping 官方 xpm_fifo_async，不加任何邏輯。
//
// 2026-07-29 新增 WIDTH 參數（預設 64，原本呼叫方不用改就維持既有行為）：
// PO_DIAG(0xA0)/PO_STATUS_REPLY(0xA1) 這兩個 BTPipeOut 補 okClk<->sys_clk
// CDC 修法時複用同一顆 wrapper，只是方向對調（wr_clk=sys_clk/rd_clk=
// okClk）、寬度改 32-bit，見 rtl/status_reply_capture.v/diag_capture.v
// 檔頭說明。

module fp_fifo_wrapper #(
    parameter WIDTH = 64
) (
    input  wire        wr_clk,
    input  wire        rd_clk,
    input  wire        rst,

    input  wire [WIDTH-1:0] din,
    input  wire        wr_en,
    output wire         prog_full,
    output wire         full,
    output wire         wr_rst_busy,

    output wire [WIDTH-1:0] dout,
    input  wire        rd_en,
    output wire         empty
);

    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH (4096),
        .WRITE_DATA_WIDTH (WIDTH),
        .READ_DATA_WIDTH  (WIDTH),
        .READ_MODE        ("fwft"),
        .PROG_FULL_THRESH (3900),
        // EN_PF=EN_ADV_FEATURE[1]、EN_PE=EN_ADV_FEATURE[9]（見 fp_input.v
        // 舊版歷史註解 2026-07-05 修正）
        .USE_ADV_FEATURES ("0002"),   // bit1 = 開 prog_full
        .DOUT_RESET_VALUE ("0"),
        .CDC_SYNC_STAGES  (2),
        .RELATED_CLOCKS   (0)
    ) fp_fifo (
        .wr_clk       (wr_clk),
        .rd_clk       (rd_clk),
        .rst          (rst),     // 官方單一 async rst，內部自己處理雙時脈同步
        .sleep        (1'b0),
        .wr_en        (wr_en),
        .din          (din),
        .rd_en        (rd_en),
        .dout         (dout),
        .empty        (empty),
        .full         (full),
        .prog_full    (prog_full),
        .wr_rst_busy  (wr_rst_busy),
        .rd_rst_busy  (),
        .data_valid   (),
        .wr_data_count(),
        .rd_data_count(),
        .almost_empty (),
        .almost_full  (),
        .wr_ack       (),
        .overflow     (),
        .underflow    (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      ()
    );

endmodule
`default_nettype wire
