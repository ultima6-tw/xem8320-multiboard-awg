`timescale 1ns/1ps
`default_nettype none

// flash_startup_loader — 上電後從 FPGA flash 讀取 config，2026-07-15 起
// 改成依序讀 4 個獨立的 64KB sector（「分開儲存」，見 PROJECT.md「Flash
// 持久化層盤點」章節）：
//   sector 0（身份，0xC80000，原 sector 200，位置/格式不變）：
//     magic / board_id / is_master / DNA / trig_delay
//   sector 1（scale_cfg，0xC90000，新）：magic / scale_cfg（1 byte）
//   sector 2（amp_ctrl，0xCA0000，新）：magic / amp_ctrl×8（24 bytes）
//   sector 3（calib_coef，0xCB0000，新）：magic / calib_coef×32（96 bytes）
//
// 4 個 sector 各自獨立驗 magic、各自觸發自己的 load_valid_*，互不影響——
// 某個新 sector 還沒存過（magic 不符），只有那個群組維持硬體預設值，不會
// 卡住其他群組正常載入。SPI RSTEN/RST 只需要在開機時做一次，不用每個
// sector 重做。
//
// 流程：
//   1. rst 釋放後等待 1024 clk（讓 SPI flash 完成 power-on reset）
//      （註解沿用舊版，實際上是 100_000_000 clk = 1 秒 @100MHz，跟程式碼
//      不一致，2026-07-08 就查證過，還沒回頭修，這次沿用不動）
//   2. RSTEN → RST → 等 tRST
//   3. 依序讀 sector 0..3，各自 256 bytes、各自驗 magic、各自解析
//   4. loader_done 在全部 4 個 sector 都處理完之後拉高

module flash_startup_loader (
    input  wire        clk,
    input  wire        rst,

    // ── spi_flash_ctrl 介面 ───────────────────────────────────────────
    output reg         op_start,
    output reg  [2:0]  op_cmd,
    output reg  [23:0] op_addr,
    output reg  [8:0]  op_len,
    input  wire        spi_busy,
    input  wire        spi_done,
    input  wire        spi_err,
    input  wire [7:0]  rdata_byte,
    input  wire [7:0]  rdata_idx,
    input  wire        rdata_valid,

    // ── 輸出初始值（load_valid 高電位時有效）────────────────────────────
    output reg         loader_done,  // 上電全部 4 個 sector 讀取流程結束
    output reg         load_valid,   // sector 0（身份）magic 驗證通過
    output reg  [15:0] init_board_id,
    output reg         init_is_master,
    output reg  [95:0] init_dna,

    // 2026-07-15：amp_ctrl/scale_cfg/calib_coef 各自獨立 load_valid，
    // 因為現在分別存在各自獨立的 sector（sector 1/2/3）
    output reg         load_valid_scale,  // sector 1（scale_cfg）magic 驗證通過
    output reg  [7:0]  init_scale_cfg,

    output reg         load_valid_amp,    // sector 2（amp_ctrl）magic 驗證通過
    output reg  [17:0] init_amp_ctrl_0,
    output reg  [17:0] init_amp_ctrl_1,
    output reg  [17:0] init_amp_ctrl_2,
    output reg  [17:0] init_amp_ctrl_3,
    output reg  [17:0] init_amp_ctrl_4,
    output reg  [17:0] init_amp_ctrl_5,
    output reg  [17:0] init_amp_ctrl_6,
    output reg  [17:0] init_amp_ctrl_7,

    output reg         load_valid_coef,   // sector 3（calib_coef）magic 驗證通過
    // calib_coef[0..31]，18-bit each
    output reg  [17:0] init_coef_0,  output reg [17:0] init_coef_1,
    output reg  [17:0] init_coef_2,  output reg [17:0] init_coef_3,
    output reg  [17:0] init_coef_4,  output reg [17:0] init_coef_5,
    output reg  [17:0] init_coef_6,  output reg [17:0] init_coef_7,
    output reg  [17:0] init_coef_8,  output reg [17:0] init_coef_9,
    output reg  [17:0] init_coef_10, output reg [17:0] init_coef_11,
    output reg  [17:0] init_coef_12, output reg [17:0] init_coef_13,
    output reg  [17:0] init_coef_14, output reg [17:0] init_coef_15,
    output reg  [17:0] init_coef_16, output reg [17:0] init_coef_17,
    output reg  [17:0] init_coef_18, output reg [17:0] init_coef_19,
    output reg  [17:0] init_coef_20, output reg [17:0] init_coef_21,
    output reg  [17:0] init_coef_22, output reg [17:0] init_coef_23,
    output reg  [17:0] init_coef_24, output reg [17:0] init_coef_25,
    output reg  [17:0] init_coef_26, output reg [17:0] init_coef_27,
    output reg  [17:0] init_coef_28, output reg [17:0] init_coef_29,
    output reg  [17:0] init_coef_30, output reg [17:0] init_coef_31,

    output reg  [31:0] raw_magic,     // diagnostic: sector 0（身份）讀到的 magic
    // 2026-07-15：新 sector 各自的 magic 診斷（供 ILA/之後排查用）
    output reg  [31:0] raw_magic_scale,
    output reg  [31:0] raw_magic_amp,
    output reg  [31:0] raw_magic_coef,

    // 2026-07-08 (step 15b) 新增：trig_delay 初始值透傳（跟身份群組一起，
    // 留在 sector 0，位置不變）
    output reg  [15:0] init_trig_delay
);

// ── Flash 位址常數（2026-07-15：從單一 FLASH_ADDR 改成 4 個獨立 sector）──
localparam FLASH_ADDR_IDENTITY = 24'hC80000;  // sector 200（不動，向下相容）
localparam FLASH_ADDR_SCALE    = 24'hC90000;  // sector 201（新）
localparam FLASH_ADDR_AMP      = 24'hCA0000;  // sector 202（新）
localparam FLASH_ADDR_COEF     = 24'hCB0000;  // sector 203（新）
localparam FLASH_MAGIC = 32'h41574739;

localparam OP_READ  = 3'd0;
localparam OP_RSTEN = 3'd5;
localparam OP_RST   = 3'd6;

// ── 接收緩衝（4 個 sector 依序共用同一塊，處理完一個才讀下一個）─────────
reg [7:0] rbuf [255:0];

always @(posedge clk)
    if (rdata_valid) rbuf[rdata_idx] <= rdata_byte;

// ── sector 位址查表 ──────────────────────────────────────────────────────
reg [1:0] sector_idx = 2'd0;   // 0=身份 1=scale_cfg 2=amp_ctrl 3=calib_coef

function [23:0] sector_addr;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_addr = FLASH_ADDR_IDENTITY;
            2'd1: sector_addr = FLASH_ADDR_SCALE;
            2'd2: sector_addr = FLASH_ADDR_AMP;
            default: sector_addr = FLASH_ADDR_COEF;
        endcase
    end
endfunction

// ── FSM ───────────────────────────────────────────────────────────────
localparam S_WAIT        = 4'd0;  // 等待上電穩定（1024 clk）
localparam S_RSTEN       = 4'd1;  // 發出 RSTEN (0x66)
localparam S_WAIT_RSTEN  = 4'd2;  // 等待 RSTEN 完成
localparam S_RST         = 4'd3;  // 發出 RST (0x99)
localparam S_WAIT_RST    = 4'd4;  // 等待 RST 完成
localparam S_RST_PAUSE   = 4'd5;  // 等待 tRST 30μs（3000 clk @100MHz）
localparam S_READ        = 4'd6;  // 發出 READ 指令（目前 sector = sector_idx）
localparam S_WAIT_R      = 4'd7;  // 等待 READ 完成
localparam S_PARSE       = 4'd8;  // 解析 buffer（目前 sector = sector_idx）
localparam S_NEXT        = 4'd9;  // sector_idx++ 或收尾
localparam S_DONE        = 4'd10; // 全部 sector 處理完（loader_done 保持）
localparam S_ERR         = 4'd11; // SPI 硬錯誤（RSTEN/RST/spi_err，非 magic 不符）

(* MARK_DEBUG="TRUE" *) reg [3:0]  state     = S_WAIT;
reg [26:0] wait_cnt  = 0;     // 上電延遲計數（100M clk = 1s @100MHz，供 ILA arm 用）
reg [11:0] rst_wait  = 0;     // tRST 等待計數（3000 clk = 30μs）

// 3-byte 18-bit unpack helper（LE）
function [17:0] unpack18;
    input [7:0] b0, b1, b2;
    begin
        unpack18 = {b2[1:0], b1, b0};
    end
endfunction

always @(posedge clk) begin
    op_start <= 1'b0;

    case (state)
    S_WAIT: begin
        if (wait_cnt == 27'd100_000_000)
            state <= S_RSTEN;
        else
            wait_cnt <= wait_cnt + 1;
    end

    S_RSTEN: begin
        op_start <= 1'b1;
        op_cmd   <= OP_RSTEN;
        op_addr  <= 24'h0;
        op_len   <= 9'd0;
        state    <= S_WAIT_RSTEN;
    end

    S_WAIT_RSTEN: begin
        if (spi_err) begin
            raw_magic <= 32'hEEEE0066;  // sentinel: RSTEN failed
            state     <= S_ERR;
        end else if (spi_done)
            state <= S_RST;
    end

    S_RST: begin
        op_start <= 1'b1;
        op_cmd   <= OP_RST;
        op_addr  <= 24'h0;
        op_len   <= 9'd0;
        state    <= S_WAIT_RST;
    end

    S_WAIT_RST: begin
        if (spi_err) begin
            raw_magic <= 32'hEEEE0099;  // sentinel: RST failed
            state     <= S_ERR;
        end else if (spi_done)
            state <= S_RST_PAUSE;
    end

    S_RST_PAUSE: begin
        // tRST = 30μs max = 3000 clk @100MHz
        if (rst_wait == 12'd3000) begin
            rst_wait   <= 0;
            sector_idx <= 2'd0;
            state      <= S_READ;
        end else
            rst_wait <= rst_wait + 1;
    end

    S_READ: begin
        op_start <= 1'b1;
        op_cmd   <= OP_READ;
        op_addr  <= sector_addr(sector_idx);
        op_len   <= 9'd0;       // 0 = 256 bytes
        state    <= S_WAIT_R;
    end

    S_WAIT_R: begin
        if (spi_err) begin
            // 讀取本身的 SPI 錯誤（不是 magic 不符）：整個 loader 中止，
            // 跟舊版行為一致（保守處理，不猜測部分 sector 讀失敗還能不能
            // 繼續讀下一個）
            raw_magic <= 32'hEEEEEEEE;  // sentinel: spi_err occurred
            state     <= S_ERR;
        end else if (spi_done)
            state <= S_PARSE;
    end

    S_PARSE: begin
        case (sector_idx)
        2'd0: begin // ── 身份群組（board_id/is_master/DNA/trig_delay）──
            raw_magic <= {rbuf[3], rbuf[2], rbuf[1], rbuf[0]};
            if ({rbuf[3], rbuf[2], rbuf[1], rbuf[0]} == FLASH_MAGIC) begin
                init_board_id  <= {rbuf[7], rbuf[6]};
                init_is_master <= rbuf[8][0];
                init_dna <= {rbuf[23], rbuf[22], rbuf[21], rbuf[20],
                             rbuf[19], rbuf[18], rbuf[17], rbuf[16],
                             rbuf[15], rbuf[14], rbuf[13], rbuf[12]};
                init_trig_delay <= {rbuf[8'hA1], rbuf[8'hA0]};
                load_valid <= 1'b1;
            end
        end

        2'd1: begin // ── scale_cfg（1 byte，offset 0x04）──
            raw_magic_scale <= {rbuf[3], rbuf[2], rbuf[1], rbuf[0]};
            if ({rbuf[3], rbuf[2], rbuf[1], rbuf[0]} == FLASH_MAGIC) begin
                init_scale_cfg   <= rbuf[8'h04];
                load_valid_scale <= 1'b1;
            end
        end

        2'd2: begin // ── amp_ctrl×8（24 bytes，offset 0x04..0x1B）──
            raw_magic_amp <= {rbuf[3], rbuf[2], rbuf[1], rbuf[0]};
            if ({rbuf[3], rbuf[2], rbuf[1], rbuf[0]} == FLASH_MAGIC) begin
                init_amp_ctrl_0 <= unpack18(rbuf[8'h04], rbuf[8'h05], rbuf[8'h06]);
                init_amp_ctrl_1 <= unpack18(rbuf[8'h07], rbuf[8'h08], rbuf[8'h09]);
                init_amp_ctrl_2 <= unpack18(rbuf[8'h0A], rbuf[8'h0B], rbuf[8'h0C]);
                init_amp_ctrl_3 <= unpack18(rbuf[8'h0D], rbuf[8'h0E], rbuf[8'h0F]);
                init_amp_ctrl_4 <= unpack18(rbuf[8'h10], rbuf[8'h11], rbuf[8'h12]);
                init_amp_ctrl_5 <= unpack18(rbuf[8'h13], rbuf[8'h14], rbuf[8'h15]);
                init_amp_ctrl_6 <= unpack18(rbuf[8'h16], rbuf[8'h17], rbuf[8'h18]);
                init_amp_ctrl_7 <= unpack18(rbuf[8'h19], rbuf[8'h1A], rbuf[8'h1B]);
                load_valid_amp  <= 1'b1;
            end
        end

        default: begin // 2'd3 ── calib_coef×32（96 bytes，offset 0x04..0x63）──
            raw_magic_coef <= {rbuf[3], rbuf[2], rbuf[1], rbuf[0]};
            if ({rbuf[3], rbuf[2], rbuf[1], rbuf[0]} == FLASH_MAGIC) begin
                init_coef_0  <= unpack18(rbuf[8'h04], rbuf[8'h05], rbuf[8'h06]);
                init_coef_1  <= unpack18(rbuf[8'h07], rbuf[8'h08], rbuf[8'h09]);
                init_coef_2  <= unpack18(rbuf[8'h0A], rbuf[8'h0B], rbuf[8'h0C]);
                init_coef_3  <= unpack18(rbuf[8'h0D], rbuf[8'h0E], rbuf[8'h0F]);
                init_coef_4  <= unpack18(rbuf[8'h10], rbuf[8'h11], rbuf[8'h12]);
                init_coef_5  <= unpack18(rbuf[8'h13], rbuf[8'h14], rbuf[8'h15]);
                init_coef_6  <= unpack18(rbuf[8'h16], rbuf[8'h17], rbuf[8'h18]);
                init_coef_7  <= unpack18(rbuf[8'h19], rbuf[8'h1A], rbuf[8'h1B]);
                init_coef_8  <= unpack18(rbuf[8'h1C], rbuf[8'h1D], rbuf[8'h1E]);
                init_coef_9  <= unpack18(rbuf[8'h1F], rbuf[8'h20], rbuf[8'h21]);
                init_coef_10 <= unpack18(rbuf[8'h22], rbuf[8'h23], rbuf[8'h24]);
                init_coef_11 <= unpack18(rbuf[8'h25], rbuf[8'h26], rbuf[8'h27]);
                init_coef_12 <= unpack18(rbuf[8'h28], rbuf[8'h29], rbuf[8'h2A]);
                init_coef_13 <= unpack18(rbuf[8'h2B], rbuf[8'h2C], rbuf[8'h2D]);
                init_coef_14 <= unpack18(rbuf[8'h2E], rbuf[8'h2F], rbuf[8'h30]);
                init_coef_15 <= unpack18(rbuf[8'h31], rbuf[8'h32], rbuf[8'h33]);
                init_coef_16 <= unpack18(rbuf[8'h34], rbuf[8'h35], rbuf[8'h36]);
                init_coef_17 <= unpack18(rbuf[8'h37], rbuf[8'h38], rbuf[8'h39]);
                init_coef_18 <= unpack18(rbuf[8'h3A], rbuf[8'h3B], rbuf[8'h3C]);
                init_coef_19 <= unpack18(rbuf[8'h3D], rbuf[8'h3E], rbuf[8'h3F]);
                init_coef_20 <= unpack18(rbuf[8'h40], rbuf[8'h41], rbuf[8'h42]);
                init_coef_21 <= unpack18(rbuf[8'h43], rbuf[8'h44], rbuf[8'h45]);
                init_coef_22 <= unpack18(rbuf[8'h46], rbuf[8'h47], rbuf[8'h48]);
                init_coef_23 <= unpack18(rbuf[8'h49], rbuf[8'h4A], rbuf[8'h4B]);
                init_coef_24 <= unpack18(rbuf[8'h4C], rbuf[8'h4D], rbuf[8'h4E]);
                init_coef_25 <= unpack18(rbuf[8'h4F], rbuf[8'h50], rbuf[8'h51]);
                init_coef_26 <= unpack18(rbuf[8'h52], rbuf[8'h53], rbuf[8'h54]);
                init_coef_27 <= unpack18(rbuf[8'h55], rbuf[8'h56], rbuf[8'h57]);
                init_coef_28 <= unpack18(rbuf[8'h58], rbuf[8'h59], rbuf[8'h5A]);
                init_coef_29 <= unpack18(rbuf[8'h5B], rbuf[8'h5C], rbuf[8'h5D]);
                init_coef_30 <= unpack18(rbuf[8'h5E], rbuf[8'h5F], rbuf[8'h60]);
                init_coef_31 <= unpack18(rbuf[8'h61], rbuf[8'h62], rbuf[8'h63]);
                load_valid_coef <= 1'b1;
            end
        end
        endcase

        state <= S_NEXT;
    end

    S_NEXT: begin
        // load_valid_* one-shot：S_PARSE 進入時可能設 1，這裡下一拍立即清 0
        load_valid       <= 1'b0;
        load_valid_scale <= 1'b0;
        load_valid_amp   <= 1'b0;
        load_valid_coef  <= 1'b0;

        if (sector_idx == 2'd3) begin
            state <= S_DONE;
        end else begin
            sector_idx <= sector_idx + 2'd1;
            state      <= S_READ;
        end
    end

    S_DONE: begin
        loader_done <= 1'b1;
    end

    S_ERR: begin
        // SPI 硬錯誤：中止整個流程，全部群組維持硬體預設值
        load_valid       <= 1'b0;
        load_valid_scale <= 1'b0;
        load_valid_amp   <= 1'b0;
        load_valid_coef  <= 1'b0;
        loader_done      <= 1'b1;
        state            <= S_DONE;
    end
    endcase

    if (rst) begin
        state            <= S_WAIT;
        wait_cnt         <= 27'd0;
        rst_wait         <= 0;
        sector_idx       <= 2'd0;
        load_valid       <= 1'b0;
        load_valid_scale <= 1'b0;
        load_valid_amp   <= 1'b0;
        load_valid_coef  <= 1'b0;
        loader_done      <= 1'b0;
        op_start         <= 1'b0;
        raw_magic        <= 32'h00000000;
        raw_magic_scale  <= 32'h00000000;
        raw_magic_amp    <= 32'h00000000;
        raw_magic_coef   <= 32'h00000000;
    end
end

endmodule
`default_nettype wire
