`timescale 1ns/1ps
`default_nettype none

// sticky_latch.v -- 2026-07-08 debug-only
//
// 極簡 sticky 診斷閂鎖：set_in 只要出現過一次（哪怕只有 1 個 clk cycle），
// sticky_out 就永遠拉高，直到 rst。用來監測瞬間訊號（例如 Aurora 的
// hard_err，link training 過程中可能只閃一下就消失），事後仍能用 host
// 讀回「是否曾經發生過」。

module sticky_latch (
    input  wire clk,
    input  wire rst,
    input  wire set_in,
    output reg  sticky_out
);

    always @(posedge clk) begin
        if (rst) sticky_out <= 1'b0;
        else if (set_in) sticky_out <= 1'b1;
    end

endmodule
`default_nettype wire
