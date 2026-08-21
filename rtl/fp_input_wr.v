`timescale 1ns/1ps
`default_nettype none

// fp_input_wr — FP BTPipeIn（ok_clk domain, 32-bit）寫入端 pair-acc
//
// 2026-07-10 從 fp_input.v 拆分（見 PROJECT.md 第 32 節）：原本 fp_input.v
// 把「寫入端 pair-acc」「FIFO 本身」「讀出端」全部包在同一個模組裡，BD 裡
// 看不到中間的 CDC 節點，加探測點時容易把不同 clock domain 的訊號誤接到
// 同一顆 ILA 上（例如把 ok_clk 訊號接到 sys_clk 的 ILA，沒做同步，有
// metastability 風險——這正是追 flash-erase 卡住問題時發生的狀況）。拆成
// 三個獨立 BD cell：fp_input_wr（本檔案，ok_clk）→ fp_fifo_wrapper
// （xpm_fifo_async 薄殼，CDC 邊界本身）→ fp_input_rd（sys_clk）。純結構
// 重組，邏輯跟原本完全一樣。
//
// pair-acc 順序：第 1 個 word → lo_word；第 2 個 word → hi_word
// beat = {hi_word[31:0], lo_word[31:0]}（[63:32]=hi, [31:0]=lo）

module fp_input_wr (
    input  wire        sys_clk,
    input  wire        sys_rst,

    // fp0/okClk：BTPI 訊號實際同步的時脈
    input  wire        ok_clk,

    // FP BTPipeIn（ok_clk domain！不是 sys_clk）
    input  wire [31:0] pi_data,
    input  wire        pi_write,
    output wire        ep_ready,    // 暫存（ok_clk, 1-cycle 延遲）：!prog_full && !wr_rst_busy

    // half_reset：sys_clk domain 進來，內部同步到 ok_clk domain 再用
    input  wire        half_reset,

    // ── FIFO 寫入端介面（接到 fp_fifo_wrapper）─────────────────────────────
    output wire [63:0] fifo_din,
    output wire        fifo_wr_en,
    input  wire        fifo_prog_full,
    input  wire        fifo_wr_rst_busy,

    // ── 診斷輸出（ok_clk domain）────────────────────────────────────────────
    output reg         diag_fp_wr_seen,    // sticky: fifo_wr_en 曾觸發
    output reg  [31:0] diag_fp_first_lo,   // 第一個 beat 的 lo word（[31:0]）
    output reg  [31:0] diag_fp_first_hi,   // 第一個 beat 的 hi word（[63:32]）
    output wire [383:0] diag_raw_data,     // raw pi_data 捕捉：12 x 32-bit

    output wire [15:0] debug_pi_write_cnt, // pi_write high 次數（不管 ep_ready）
    output wire [15:0] debug_fifo_wr_cnt   // fifo_wr_en 次數（成功湊對、寫進 FIFO 的 beat 數）
);

    // ── sys_rst -> ok_clk domain 2-flop 同步器 ──────────────────────────────
    (* ASYNC_REG = "TRUE" *) reg sys_rst_ok_meta = 1'b1;
    reg                          sys_rst_ok       = 1'b1;
    always @(posedge ok_clk) begin
        sys_rst_ok_meta <= sys_rst;
        sys_rst_ok      <= sys_rst_ok_meta;
    end

    // ── half_reset（sys_clk 產生的窄 pulse）-> ok_clk domain ────────────────
    reg [3:0] half_reset_stretch = 4'd0;
    always @(posedge sys_clk) begin
        if (sys_rst) begin
            half_reset_stretch <= 4'd0;
        end else if (half_reset) begin
            half_reset_stretch <= 4'd15;
        end else if (half_reset_stretch != 4'd0) begin
            half_reset_stretch <= half_reset_stretch - 4'd1;
        end
    end

    (* ASYNC_REG = "TRUE" *) reg half_reset_ok_meta  = 1'b0;
    reg                          half_reset_ok_level = 1'b0;
    always @(posedge ok_clk) begin
        half_reset_ok_meta  <= (half_reset_stretch != 4'd0);
        half_reset_ok_level <= half_reset_ok_meta;
    end

    // ── pair-acc（ok_clk domain）─────────────────────────────────────────────
    reg        half       = 1'b0;
    reg [31:0] lo_word    = 32'd0;
    reg [63:0] fifo_din_r;
    reg        fifo_wr_en_r = 1'b0;

    assign fifo_din   = fifo_din_r;
    assign fifo_wr_en = fifo_wr_en_r;

    reg ep_ready_r = 1'b0;
    assign ep_ready = ep_ready_r;

    always @(posedge ok_clk) begin
        if (sys_rst_ok) begin
            ep_ready_r <= 1'b0;
        end else begin
            ep_ready_r <= !fifo_prog_full && !fifo_wr_rst_busy;
        end
    end

    always @(posedge ok_clk) begin
        fifo_wr_en_r <= 1'b0;
        if (sys_rst_ok || half_reset_ok_level) begin
            half             <= 1'b0;
            diag_fp_wr_seen  <= 1'b0;
            diag_fp_first_lo <= 32'd0;
            diag_fp_first_hi <= 32'd0;
        end else if (ep_ready && pi_write) begin
            if (!half) begin
                lo_word <= pi_data;
                half    <= 1'b1;
            end else begin
                fifo_din_r   <= {pi_data, lo_word};  // {hi, lo}
                fifo_wr_en_r <= 1'b1;
                half         <= 1'b0;
                // 診斷：記錄第一次寫入
                if (!diag_fp_wr_seen) begin
                    diag_fp_wr_seen  <= 1'b1;
                    diag_fp_first_lo <= lo_word;
                    diag_fp_first_hi <= pi_data;
                end
            end
        end
    end

    // ── 診斷 counter（ok_clk domain，free-running，sys_rst_ok 才清零）───────
    reg [15:0] pi_write_cnt = 16'd0;
    reg [15:0] fifo_wr_cnt  = 16'd0;

    always @(posedge ok_clk) begin
        if (sys_rst_ok) begin
            pi_write_cnt <= 16'd0;
        end else if (pi_write) begin
            pi_write_cnt <= pi_write_cnt + 16'd1;
        end
    end

    always @(posedge ok_clk) begin
        if (sys_rst_ok) begin
            fifo_wr_cnt <= 16'd0;
        end else if (fifo_wr_en) begin
            fifo_wr_cnt <= fifo_wr_cnt + 16'd1;
        end
    end

    assign debug_pi_write_cnt = pi_write_cnt;
    assign debug_fifo_wr_cnt  = fifo_wr_cnt;

    // ── Raw pi_data 捕捉（ok_clk domain）─────────────────────────────────────
    reg [31:0] raw_word_cap [0:11];
    reg [3:0]  raw_cap_cnt = 4'd0;

    integer ri;
    always @(posedge ok_clk) begin
        if (sys_rst_ok || half_reset_ok_level) begin
            raw_cap_cnt <= 4'd0;
            for (ri = 0; ri < 12; ri = ri + 1)
                raw_word_cap[ri] <= 32'd0;
        end else if (pi_write && raw_cap_cnt != 4'd12) begin
            raw_word_cap[raw_cap_cnt] <= pi_data;
            raw_cap_cnt <= raw_cap_cnt + 4'd1;
        end
    end

    genvar gi;
    generate
        for (gi = 0; gi < 12; gi = gi + 1) begin : g_raw
            assign diag_raw_data[gi*32 +: 32] = raw_word_cap[gi];
        end
    endgenerate

endmodule
`default_nettype wire
