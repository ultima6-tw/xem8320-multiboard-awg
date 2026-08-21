`timescale 1ns/1ps
`default_nettype none

// flash_config_writer — 接收 FrontPanel PipeIn 256 bytes，寫入 FPGA flash sector 256
//
// 流程：
//   1. fp_flash_cfg_wr pulse → 觸發 WREN → SECTOR_ERASE → WREN → PAGE_PROGRAM
//   2. fp_flash_cfg_erase pulse → 觸發 WREN → SECTOR_ERASE only
//   3. flash_status[0]=busy, [1]=done, [2]=err
//
// 2026-07-08 (step 15b) local copy（原本共用 awg-test-step-14/rtl/ 的檔案）：
// 新增 diag_state[3:0]/diag_do_program 兩個純 assign 診斷輸出 port，用來除錯
// T_FLASH_ERASE(0x0F) 封包送出後 flash_status 的 busy/done/err 三個 bit 持續
// 為 0（erase 完全沒有被觸發，但 T_FLASH_SAVE 已確認正常）——需要看到這個
// 模組內部 state 才能判斷是 S_IDLE 從未被觸發，還是進了 S_IDLE 之後某個
// 後續狀態卡住。不改動任何既有邏輯，純粹多接出兩個 wire。
//
// 2026-07-15：`FLASH_ADDR` 從寫死的 localparam 改成 `op_base_addr` input
// port，讓 `scale_cfg`/`amp_ctrl`/`calib_coef` 可以各自存到獨立的 64KB
// sector（「分開儲存」，不用整包 256-byte blob 互相牽連）。呼叫端
// （`fpga_flash_ctrl.v`）依照目前選中的目標群組算出對應位址餵進來，這個
// 模組本身不需要知道「群組」的概念，只是單純的 erase+program 狀態機。

module flash_config_writer (
    input  wire        clk,
    input  wire        rst,

    // ── FrontPanel triggers ───────────────────────────────────────────
    input  wire        fp_flash_cfg_wr,     // TI bit[13]
    input  wire        fp_flash_cfg_erase,  // TI bit[14]

    // ── 目標 sector 位址（2026-07-15 新增）────────────────────────────
    input  wire [23:0] op_base_addr,

    // ── PipeIn 資料（FrontPanel 直接寫入 spi_flash_ctrl wbuf）─────────
    // PipeIn 需在 BD 中直接連到 spi_flash_ctrl 的 wdata 介面
    // 此模組只負責控制 op_start / op_cmd 時序

    // ── spi_flash_ctrl 介面 ───────────────────────────────────────────
    output reg         op_start,
    output reg  [2:0]  op_cmd,
    output reg  [23:0] op_addr,
    output reg  [8:0]  op_len,
    input  wire        spi_busy,
    input  wire        spi_done,
    input  wire        spi_err,

    // ── 狀態輸出 → WireOut ────────────────────────────────────────────
    output wire [2:0]  flash_status,   // [0]=busy [1]=done [2]=err

    // ── 診斷輸出（2026-07-08 step 15b 新增，ILA 用）───────────────────
    output wire [3:0]  diag_state,
    output wire        diag_do_program
);

// 2026-07-15：FLASH_ADDR 改成 op_base_addr input port（見上方檔頭註解），
// 這裡不再是寫死的常數。

localparam OP_WREN = 3'd1;
localparam OP_PP   = 3'd2;
localparam OP_SE   = 3'd3;

localparam S_IDLE       = 4'd0;
localparam S_WREN1      = 4'd1;  // WREN before erase
localparam S_WAIT_WREN1 = 4'd2;
localparam S_SE         = 4'd3;  // Sector Erase
localparam S_WAIT_SE    = 4'd4;
localparam S_WREN2      = 4'd5;  // WREN before program
localparam S_WAIT_WREN2 = 4'd6;
localparam S_PP         = 4'd7;  // Page Program
localparam S_WAIT_PP    = 4'd8;
localparam S_DONE       = 4'd9;
localparam S_ERR        = 4'd10;

reg [3:0] state = S_IDLE;
reg       do_program = 1'b0;  // 1=write, 0=erase only
reg       busy_r = 1'b0;
reg       done_r = 1'b0;
reg       err_r  = 1'b0;

assign flash_status = {err_r, done_r, busy_r};
assign diag_state       = state;
assign diag_do_program  = do_program;

always @(posedge clk) begin
    op_start <= 1'b0;
    // done_r / err_r are LATCHES — not cleared every cycle
    // They stay high until the next operation starts

    case (state)
    S_IDLE: begin
        if (fp_flash_cfg_wr || fp_flash_cfg_erase) begin
            do_program <= fp_flash_cfg_wr;
            busy_r     <= 1'b1;
            done_r     <= 1'b0;   // clear prev done/err when starting new op
            err_r      <= 1'b0;
            state      <= S_WREN1;
        end
    end

    S_WREN1: begin
        if (!spi_busy) begin
            op_start <= 1'b1;
            op_cmd   <= OP_WREN;
            op_addr  <= 24'd0;
            op_len   <= 9'd0;
            state    <= S_WAIT_WREN1;
        end
    end

    S_WAIT_WREN1: begin
        if (spi_err) state <= S_ERR;
        else if (spi_done) state <= S_SE;
    end

    S_SE: begin
        if (!spi_busy) begin
            op_start <= 1'b1;
            op_cmd   <= OP_SE;
            op_addr  <= op_base_addr;
            op_len   <= 9'd0;
            state    <= S_WAIT_SE;
        end
    end

    S_WAIT_SE: begin
        if (spi_err) state <= S_ERR;
        else if (spi_done) begin
            if (do_program) state <= S_WREN2;
            else            state <= S_DONE;
        end
    end

    S_WREN2: begin
        if (!spi_busy) begin
            op_start <= 1'b1;
            op_cmd   <= OP_WREN;
            op_addr  <= 24'd0;
            op_len   <= 9'd0;
            state    <= S_WAIT_WREN2;
        end
    end

    S_WAIT_WREN2: begin
        if (spi_err) state <= S_ERR;
        else if (spi_done) state <= S_PP;
    end

    S_PP: begin
        if (!spi_busy) begin
            op_start <= 1'b1;
            op_cmd   <= OP_PP;
            op_addr  <= op_base_addr;
            op_len   <= 9'd0;   // 0 = 256 bytes
            state    <= S_WAIT_PP;
        end
    end

    S_WAIT_PP: begin
        if (spi_err) state <= S_ERR;
        else if (spi_done) state <= S_DONE;
    end

    S_DONE: begin
        busy_r <= 1'b0;
        done_r <= 1'b1;
        state  <= S_IDLE;
    end

    S_ERR: begin
        busy_r <= 1'b0;
        err_r  <= 1'b1;   // stays 1 until next op (no preamble clear)
        state  <= S_IDLE;
    end
    endcase

    if (rst) begin
        state    <= S_IDLE;
        op_start <= 1'b0;
        busy_r   <= 1'b0;
        done_r   <= 1'b0;
        err_r    <= 1'b0;
    end
end

endmodule
`default_nettype wire
