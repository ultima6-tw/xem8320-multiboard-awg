`timescale 1ns/1ps
`default_nettype none

// fp_input_rd — FIFO 讀出端（sys_clk domain）→ Dispatcher fp 輸入
//
// 2026-07-10 從 fp_input.v 拆出（見 PROJECT.md 第 32 節）。

module fp_input_rd (
    input  wire        sys_clk,
    input  wire        sys_rst,

    // ── FIFO 讀出端介面（接到 fp_fifo_wrapper）─────────────────────────────
    input  wire [63:0] fifo_dout,
    input  wire        fifo_empty,
    output wire        rd_en,

    // ── 輸出（sys_clk domain）→ Dispatcher fp 輸入 ─────────────────────────
    output wire [63:0] fp_tdata,
    output wire        fp_tvalid,
    input  wire        fp_tready,

    output wire [15:0] debug_fp_tvalid_cnt  // fp_tvalid && fp_tready 次數
);

    assign fp_tdata  = fifo_dout;
    assign fp_tvalid = !fifo_empty;   // !empty 在 FWFT 模式等同 data_valid
    assign rd_en     = fp_tready && fp_tvalid;

    reg [15:0] fp_tvalid_cnt = 16'd0;
    always @(posedge sys_clk) begin
        if (sys_rst) begin
            fp_tvalid_cnt <= 16'd0;
        end else if (fp_tvalid && fp_tready) begin
            fp_tvalid_cnt <= fp_tvalid_cnt + 16'd1;
        end
    end
    assign debug_fp_tvalid_cnt = fp_tvalid_cnt;

endmodule
`default_nettype wire
