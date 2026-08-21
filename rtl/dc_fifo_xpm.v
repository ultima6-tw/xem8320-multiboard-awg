`timescale 1ns/1ps
`default_nettype none

// dc_fifo_xpm: 薄 wrapper，包住 Xilinx 官方 xpm_fifo_async macro，暴露跟
// simple_dc_fifo.v 相同風格的介面（WIDTH/DEPTH/PROG_FULL_THRESH 參數名稱一致），
// 讓它可以直接取代 simple_dc_fifo 接進既有的 BD 接線，改動範圍最小。
//
// 2026-07-04：依使用者要求，盡量用官方驗證過的 IP 取代自己手寫的 CDC 邏輯
// （原本的 simple_dc_fifo.v 邏輯檢查起來是對的教科書寫法，但沒有正式測試過；
// xpm_fifo_async 是矽驗證過的官方巨集，用來排除／取代自製 FIFO 的風險）。
//
// 跟 simple_dc_fifo 的介面差異：
//   - 原本 wr_rst/rd_rst/flush 三個輸入，這裡合併成單一 rst（xpm_fifo_async
//     內部的 xpm_fifo_rst 子模組自己處理雙時脈域同步，呼叫端不用先同步）
//   - 新增 wr_rst_busy/rd_rst_busy 輸出，讓上層（waveform_controller.v）可以
//     確認官方 IP 的內部重置流程真的跑完，不用只靠猜測的固定 cycle 數等待

module dc_fifo_xpm #(
    parameter WIDTH            = 32,
    parameter DEPTH            = 1024,   // FIFO_WRITE_DEPTH
    parameter PROG_FULL_THRESH = 768
) (
    // Write port (wr_clk domain)
    input  wire             wr_clk,
    input  wire             wr_en,
    input  wire [WIDTH-1:0] din,
    output wire             full,
    output wire             prog_full,

    // Read port (rd_clk domain)
    input  wire             rd_clk,
    input  wire             rd_en,
    output wire [WIDTH-1:0] dout,
    output wire             empty,

    // 合併後的單一 reset（async assert 即可，xpm_fifo_rst 內部處理雙時脈同步）
    input  wire             rst,
    output wire             wr_rst_busy,   // wr_clk domain
    output wire             rd_rst_busy    // rd_clk domain
);

    xpm_fifo_async #(
        .FIFO_WRITE_DEPTH (DEPTH),
        .WRITE_DATA_WIDTH (WIDTH),
        .READ_DATA_WIDTH  (WIDTH),
        .READ_MODE        ("fwft"),
        .PROG_FULL_THRESH (PROG_FULL_THRESH),
        // 2026-07-05 修正：prog_full 是 bit1，不是 bit9（bit9 是 prog_empty，
        // 我們沒在用）。查證 awg-test-step-14.3/sim/xpm_sim/xpm_fifo.sv 212-221
        // 行的官方定義：EN_PF=EN_ADV_FEATURE[1]、EN_PE=EN_ADV_FEATURE[9]。原本
        // "0200"（bit9=1）誤開了沒接出去的 prog_empty，真正要用的 prog_full
        // 卡在 0（538 行：EN_PF==0 時 prog_full 被硬接 1'b0），導致
        // reader_a/b_$ch 完全收不到 backpressure 預警，只能撞到硬 full 才反應。
        .USE_ADV_FEATURES ("0002"),   // bit1 = 開 prog_full
        .DOUT_RESET_VALUE ("0"),
        .CDC_SYNC_STAGES  (2),
        .RELATED_CLOCKS   (0)
    ) fifo_inst (
        .wr_clk        (wr_clk),
        .rd_clk        (rd_clk),
        .rst           (rst),
        .sleep         (1'b0),
        .wr_en         (wr_en),
        .din           (din),
        .rd_en         (rd_en),
        .dout          (dout),
        .empty         (empty),
        .full          (full),
        .prog_full     (prog_full),
        .wr_rst_busy   (wr_rst_busy),
        .rd_rst_busy   (rd_rst_busy),
        .data_valid    (),
        .wr_data_count (),
        .rd_data_count (),
        .almost_empty  (),
        .almost_full   (),
        .wr_ack        (),
        .overflow      (),
        .underflow     (),
        .prog_empty    (),
        .injectsbiterr (1'b0),
        .injectdbiterr (1'b0),
        .sbiterr       (),
        .dbiterr       ()
    );

endmodule
`default_nettype wire
