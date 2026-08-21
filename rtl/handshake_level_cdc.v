`timescale 1ns/1ps
`default_nettype none

// handshake_level_cdc.v -- 2026-07-29 新增
//
// level_cdc.v（xpm_cdc_array_single）的替代實作，用官方 xpm_cdc_handshake
// 做「凍結快照→送出→等對方收到→下一筆」pattern（跟 aurora_reply_tx.v 的
// phase_acc CDC、status_reply_capture.v/diag_capture.v 的 CDC v2 修法同一套
// 已驗證手法），保證多 bit 之間的關聯性在跨域搬遷時不會撕裂——PG382 官方
// 文件明確說 xpm_cdc_array_single「假設陣列裡每個 bit 互相獨立」，不適合
// board_id/board_index 這類需要保持 bit 間關聯性的多 bit 數值，這個模組是
// 官方建議的替代方案。
//
// 這次先只用在 board_index_cdc_0（跟 board_id_cdc_0 做對照實驗，board_id_
// cdc_0 本身還沒有硬體證據前刻意不動，見 NOTES.md 2026-07-29 對應章節）。
//
// port 介面刻意跟 level_cdc.v 完全一致（src_clk/src_in/dst_clk/dst_out，
// 沒有 rst port），create_bd.tcl 只需要把 `-reference level_cdc` 換成
// `-reference handshake_level_cdc`，接線完全不用改。沒有 rst port是因為
// snap_r/send_r 的 reset 值本身就是安全的起始狀態（send_r=0 代表「還沒開始
// 送」，第一拍就會凍結 src_in 目前值送出），跟 level_cdc.v 一樣不需要額外
// reset 邏輯。

module handshake_level_cdc #(
    parameter WIDTH = 1
)(
    input  wire             src_clk,
    input  wire [WIDTH-1:0] src_in,
    input  wire             dst_clk,
    output wire [WIDTH-1:0] dst_out
);

    reg [WIDTH-1:0] snap_r = {WIDTH{1'b0}};
    reg             send_r = 1'b0;
    wire            rcv;

    always @(posedge src_clk) begin
        if (!send_r) begin
            // 凍結目前值，開始送出
            snap_r <= src_in;
            send_r <= 1'b1;
        end else if (rcv) begin
            // 對方已收到，準備下一次凍結
            send_r <= 1'b0;
        end
    end

    xpm_cdc_handshake #(
        .DEST_EXT_HSK (0),   // 目的端自動 ack，不需要 dst_clk 端額外邏輯
        .DEST_SYNC_FF (4),
        .SRC_SYNC_FF  (4),
        .WIDTH        (WIDTH)
    ) cdc_inst (
        .src_clk  (src_clk),
        .src_in   (snap_r),
        .src_send (send_r),
        .src_rcv  (rcv),
        .dest_clk (dst_clk),
        .dest_out (dst_out),
        .dest_req (),
        .dest_ack (1'b0)
    );

endmodule
`default_nettype wire
