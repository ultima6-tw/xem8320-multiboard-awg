`timescale 1ns/1ps
`default_nettype none

// spi_flash_ctrl.v -- local copy (not the shared awg-test-step-14/rtl/
// spi_flash_ctrl.v)，2026-08-18，理由/慣例跟同一批新增的 diag_cdc.v
// 完全相同，見該檔案檔頭說明。內容跟原檔完全相同，未做任何邏輯改動。
//
// spi_flash_ctrl — SPI x1 NOR flash controller via STARTUPE3
//
// 支援 ISSI IS25WP256D-JLLE（XEM8320 FPGA config flash）
// SPI x1 mode，clk 為 sys_clk（100 MHz），SPI 時脈 = clk/4 = 25 MHz
//
// MOSI 驅動時序：SPI Mode 0 正確實作
//   MOSI 在 FALLING EDGE 更新（SCK LOW 期間穩定），
//   flash 在 RISING EDGE 採樣時已有足夠 setup time。
//   第一個 bit 在 S_CS_LO 進入 S_CMD 前就預先設好。
//
// 指令集（2026-07-10 起改用「專用 4-byte address」指令，不受 flash 的
// EXTADD/Bank Address Register 狀態影響——不管 flash 開機時停在 3-byte
// 還是 4-byte legacy 模式，這些專用指令永遠固定是 4-byte address，行為
// 確定。op_addr 介面仍是 24-bit（實際用到的位址範圍 < 16MB），SPI 層會
// 自動在最前面補一個 0x00 當第 4 個 address byte，op_addr 語意不變）：
//   OP_READ  (0x0C 4FRD)：4-byte Address Fast Read（4-byte addr + 8 dummy clocks）
//   OP_WREN  (0x06)：Write Enable（write/erase 前必須先送）
//   OP_PP    (0x12 4PP)：4-byte Address Page Program（最多 256 bytes）
//   OP_SE    (0xDC 4BER64)：4-byte Address Block Erase 64KB
//   OP_RDSR  (0x05)：Read Status Register（等待 WIP=0，no address）
//   OP_RSTEN (0x66)：Reset Enable（必須在 OP_RST 前送）
//   OP_RST   (0x99)：Software Reset（flash 回到 3-byte standby）
//
// 介面：
//   op_start   : 1-cycle pulse 啟動操作
//   op_cmd     : 操作類型（OP_READ / OP_WREN / OP_PP / OP_SE / OP_RDSR）
//   op_addr    : 24-bit flash 位址
//   op_len     : 傳輸 byte 數（READ / PP 用，max 256）
//   wdata_*    : 寫入資料介面（256-byte buffer，由外部在 op_start 前填好）
//   rdata_*    : 讀取資料介面
//   busy       : 操作進行中
//   done       : 操作完成（1 cycle pulse）
//   err        : 錯誤（timeout 等）

module spi_flash_ctrl #(
    parameter CLK_DIV = 4    // SPI clk = sys_clk / CLK_DIV（需為偶數，>=4）
) (
    input  wire        clk,
    input  wire        rst,

    // ── 操作介面 ─────────────────────────────────────────────────────
    input  wire        op_start,
    input  wire [2:0]  op_cmd,
    input  wire [23:0] op_addr,
    input  wire [8:0]  op_len,     // 1-256 bytes（0 = 256）

    // 寫入資料：op_start 前由外部寫入（32-bit word 介面，word_idx 0-63）
    input  wire [31:0] wdata_word,
    input  wire [5:0]  wdata_widx,  // word index 0-63 (byte = widx×4 .. widx×4+3)
    input  wire        wdata_wvalid,

    // 讀取資料：done 後有效
    output reg  [7:0]  rdata_byte,
    output reg  [7:0]  rdata_idx,
    output reg         rdata_valid,

    output reg         busy,
    output reg         done,
    output reg         err,

    // ── STARTUPE3 連接 ───────────────────────────────────────────────
    output reg         spi_clk,    // → STARTUPE3 USRCCLKO
    output wire        spi_clkts,  // → STARTUPE3 USRCCLKTS（常 0）
    output reg         spi_cs_n,   // → STARTUPE3 FCSBO
    output wire        spi_fcsbts, // → STARTUPE3 FCSBTS（常 0）
    output reg         spi_mosi,   // → STARTUPE3 DO[0]
    output wire [3:0]  spi_dts,    // → STARTUPE3 DTS（4'b0010: DQ3/DQ2 driven hi, DQ1 tri, DQ0 driven）
    input  wire        spi_miso    // ← STARTUPE3 DI[1]
);

// ── 操作碼常數 ────────────────────────────────────────────────────────
localparam OP_READ  = 3'd0;
localparam OP_WREN  = 3'd1;
localparam OP_PP    = 3'd2;
localparam OP_SE    = 3'd3;
localparam OP_RDSR  = 3'd4;
localparam OP_RSTEN = 3'd5;  // Reset Enable (0x66)，必須在 RST 前送
localparam OP_RST   = 3'd6;  // Reset (0x99)，reset flash 回 3-byte standby

// Flash 指令位元組（2026-07-10：改用專用 4-byte address 指令，不受 EXTADD 影響）
localparam FC_READ  = 8'h0C;  // 4FRD: 4-byte Address Fast Read (+ 8 dummy clocks)
localparam FC_WREN  = 8'h06;  // Write Enable (no address)
localparam FC_PP    = 8'h12;  // 4PP: 4-byte Address Page Program
localparam FC_SE    = 8'hDC;  // 4BER64: 4-byte Address Block Erase 64KB
localparam FC_RDSR  = 8'h05;  // Read Status Register (no address)
localparam FC_RSTEN = 8'h66;  // Reset Enable (no address)
localparam FC_RST   = 8'h99;  // Reset (no address)

// ── STARTUPE3 常數接腳 ────────────────────────────────────────────────
assign spi_clkts  = 1'b0;   // user drives CCLK
assign spi_fcsbts = 1'b0;   // user drives CS
assign spi_dts    = 4'b0010; // DQ3(HOLD#)/DQ2(WP#) output driven hi; DQ1(MISO) tristate; DQ0(MOSI) output

// ── 寫入資料 buffer（256 bytes，32-bit word 寫入）──────────────────────
reg [7:0] wbuf [255:0];
always @(posedge clk)
    if (wdata_wvalid) begin
        wbuf[{wdata_widx, 2'b00}] <= wdata_word[7:0];
        wbuf[{wdata_widx, 2'b01}] <= wdata_word[15:8];
        wbuf[{wdata_widx, 2'b10}] <= wdata_word[23:16];
        wbuf[{wdata_widx, 2'b11}] <= wdata_word[31:24];
    end

// ── 讀取資料 buffer ───────────────────────────────────────────────────
reg [7:0] rbuf [255:0];

// ── SPI clock divider ─────────────────────────────────────────────────
localparam HALF = CLK_DIV / 2;
reg [$clog2(CLK_DIV)-1:0] clk_cnt = 0;
reg spi_clk_en = 0;  // SPI clock edge strobe（每 HALF 週期一次）

always @(posedge clk) begin
    if (!busy) begin
        clk_cnt    <= 0;
        spi_clk_en <= 0;
    end else begin
        if (clk_cnt == HALF - 1) begin
            clk_cnt    <= 0;
            spi_clk_en <= 1;
        end else begin
            clk_cnt    <= clk_cnt + 1;
            spi_clk_en <= 0;
        end
    end
end

// ── FSM ───────────────────────────────────────────────────────────────
localparam S_IDLE    = 4'd0;
localparam S_CS_LO   = 4'd1;   // CS assert setup
localparam S_CMD     = 4'd2;   // 送 command byte
localparam S_ADDR    = 4'd3;   // 送 24-bit address
localparam S_DATA_TX = 4'd4;   // 送資料（PP）
localparam S_DATA_RX = 4'd5;   // 收資料（READ/RDSR）
localparam S_CS_HI   = 4'd6;   // CS deassert hold
localparam S_POLL    = 4'd7;   // 等待 WIP=0（SE/PP 後）
localparam S_DONE    = 4'd8;
localparam S_DUMMY   = 4'd9;   // 8 dummy clocks after address（FC_READ 0x0C 需要）

(* MARK_DEBUG="TRUE" *) reg [3:0]  state    = S_IDLE;
reg [2:0]  saved_cmd;
reg [23:0] saved_addr;
reg [8:0]  saved_len;

reg [7:0]  shift_out;   // 待送 byte 的剩餘 bits（MSB 先，已預先 shift 一次）
reg [7:0]  shift_in;    // 當前收到的 byte
reg [2:0]  bit_cnt;     // 0-7，MSB 先
reg [8:0]  byte_cnt;    // 送/收的 byte 計數
reg [3:0]  cs_hold;     // CS hold counter
reg [7:0]  status_byte;

// WIP timeout counter（約 3 秒 @100MHz，SE 最壞情況）
reg [28:0] wip_timer;
localparam WIP_TIMEOUT = 29'd300_000_000;

// ── helper：設定下一個要送的 byte（MOSI=MSB, shift_out=剩餘 bits）────
// 使用 task 讓程式碼更清晰
// 直接在 always 中 inline

always @(posedge clk) begin
    done       <= 1'b0;
    err        <= 1'b0;
    rdata_valid<= 1'b0;

    case (state)
    S_IDLE: begin
        spi_clk  <= 1'b0;
        spi_cs_n <= 1'b1;
        spi_mosi <= 1'b0;
        busy     <= 1'b0;
        if (op_start) begin
            saved_cmd  <= op_cmd;
            saved_addr <= op_addr;
            saved_len  <= (op_len == 0) ? 9'd256 : {1'b0, op_len[7:0]};
            busy       <= 1'b1;
            cs_hold    <= 4'd3;
            state      <= S_CS_LO;
        end
    end

    S_CS_LO: begin
        spi_cs_n <= 1'b0;
        if (cs_hold == 0) begin
            // 預先設好 MOSI 第一個 bit（SPI Mode 0: MOSI 在 CS 拉低後、
            // 第一個 rising edge 前就要穩定）
            case (saved_cmd)
                OP_READ:  begin spi_mosi <= FC_READ[7];  shift_out <= {FC_READ[6:0],  1'b0}; end
                OP_WREN:  begin spi_mosi <= FC_WREN[7];  shift_out <= {FC_WREN[6:0],  1'b0}; end
                OP_PP:    begin spi_mosi <= FC_PP[7];    shift_out <= {FC_PP[6:0],    1'b0}; end
                OP_SE:    begin spi_mosi <= FC_SE[7];    shift_out <= {FC_SE[6:0],    1'b0}; end
                OP_RDSR:  begin spi_mosi <= FC_RDSR[7];  shift_out <= {FC_RDSR[6:0],  1'b0}; end
                OP_RSTEN: begin spi_mosi <= FC_RSTEN[7]; shift_out <= {FC_RSTEN[6:0], 1'b0}; end
                OP_RST:   begin spi_mosi <= FC_RST[7];   shift_out <= {FC_RST[6:0],   1'b0}; end
                default:  begin spi_mosi <= FC_READ[7];  shift_out <= {FC_READ[6:0],  1'b0}; end
            endcase
            bit_cnt   <= 3'd7;
            state     <= S_CMD;
        end else begin
            cs_hold <= cs_hold - 1;
        end
    end

    // ── S_CMD: 送 command byte ─────────────────────────────────────────
    // MOSI 在 falling edge 更新（SPI Mode 0 正確時序）
    // 第一個 bit 已在 S_CS_LO 設好，rising edge 採樣時已穩定
    S_CMD: begin
        if (spi_clk_en) begin
            spi_clk <= ~spi_clk;
            if (spi_clk == 1'b0) begin
                // rising edge：flash 採樣 MOSI（已在前一個 falling edge 設好）
                // 不做任何事
            end else begin
                // falling edge：更新 MOSI 給下一個 rising edge 使用
                if (bit_cnt == 0) begin
                    // command byte 完成
                    spi_clk <= 1'b0;
                    if (saved_cmd == OP_WREN || saved_cmd == OP_RSTEN || saved_cmd == OP_RST) begin
                        state <= S_CS_HI;
                        cs_hold <= 4'd3;
                    end else if (saved_cmd == OP_RDSR) begin
                        // RDSR 無 address，直接收 1 byte status
                        // MOSI 不重要（dummy 0 即可，已保持低）
                        saved_len <= 9'd1;
                        bit_cnt   <= 3'd7;
                        byte_cnt  <= 9'd0;
                        state     <= S_DATA_RX;
                    end else begin
                        // 送 4-byte address：byte0 固定 0x00（op_addr 只有
                        // 24-bit，實際定址範圍 < 16MB），byte1-3 才是
                        // saved_addr[23:0]
                        spi_mosi  <= 1'b0;
                        shift_out <= 7'b0000000;
                        bit_cnt   <= 3'd7;
                        byte_cnt  <= 9'd0;
                        state     <= S_ADDR;
                    end
                end else begin
                    // 更新 MOSI 給下一個 rising edge
                    bit_cnt   <= bit_cnt - 1;
                    spi_mosi  <= shift_out[7];
                    shift_out <= {shift_out[6:0], 1'b0};
                end
            end
        end
    end

    // ── S_ADDR: 送 4-byte address（2026-07-10 起固定 4-byte，不受 EXTADD 影響）
    // byte_cnt: 0=固定0x00（已在 S_CMD 結尾送出), 1=addr[23:16], 2=addr[15:8], 3=addr[7:0]
    // MOSI 在 falling edge 更新；第一個 byte（0x00）的 MSB 已在 S_CMD 結尾設好
    S_ADDR: begin
        if (spi_clk_en) begin
            spi_clk <= ~spi_clk;
            if (spi_clk == 1'b0) begin
                // rising edge：不做任何事
            end else begin
                // falling edge
                if (bit_cnt == 0) begin
                    spi_clk  <= 1'b0;
                    byte_cnt <= byte_cnt + 1;
                    if (byte_cnt == 3) begin
                        // 4 address bytes 送完
                        bit_cnt  <= 3'd7;
                        byte_cnt <= 9'd0;
                        if (saved_cmd == OP_READ) begin
                            // 4FRD 0x0C：進入 dummy clocks，MOSI 保持 0
                            spi_mosi  <= 1'b0;
                            state <= S_DUMMY;
                        end else if (saved_cmd == OP_PP) begin
                            // Page Program：預先設好第一個資料 byte
                            spi_mosi  <= wbuf[0][7];
                            shift_out <= {wbuf[0][6:0], 1'b0};
                            state     <= S_DATA_TX;
                        end else begin
                            // Block Erase：address 完即 CS hi
                            state   <= S_CS_HI;
                            cs_hold <= 4'd3;
                        end
                    end else if (byte_cnt == 2) begin
                        // byte_cnt 從 2→3，即將送 addr[7:0]（第4個 byte）
                        spi_mosi  <= saved_addr[7];
                        shift_out <= {saved_addr[6:0], 1'b0};
                        bit_cnt   <= 3'd7;
                    end else if (byte_cnt == 1) begin
                        // byte_cnt 從 1→2，即將送 addr[15:8]（第3個 byte）
                        spi_mosi  <= saved_addr[15];
                        shift_out <= {saved_addr[14:8], 1'b0};
                        bit_cnt   <= 3'd7;
                    end else begin
                        // byte_cnt == 0: 固定 0x00 送完，即將送 addr[23:16]（第2個 byte）
                        spi_mosi  <= saved_addr[23];
                        shift_out <= {saved_addr[22:16], 1'b0};
                        bit_cnt   <= 3'd7;
                    end
                end else begin
                    // 繼續送當前 byte
                    bit_cnt   <= bit_cnt - 1;
                    spi_mosi  <= shift_out[7];
                    shift_out <= {shift_out[6:0], 1'b0};
                end
            end
        end
    end

    // ── S_DATA_TX: 送資料（PP 寫入）───────────────────────────────────
    // MOSI 在 falling edge 更新
    S_DATA_TX: begin
        if (spi_clk_en) begin
            spi_clk <= ~spi_clk;
            if (spi_clk == 1'b0) begin
                // rising edge：不做任何事
            end else begin
                // falling edge
                if (bit_cnt == 0) begin
                    spi_clk  <= 1'b0;
                    byte_cnt <= byte_cnt + 1;
                    if (byte_cnt + 1 >= saved_len) begin
                        state   <= S_CS_HI;
                        cs_hold <= 4'd3;
                    end else begin
                        // 預先設好下一個 byte 的 MSB
                        spi_mosi  <= wbuf[byte_cnt + 1][7];
                        shift_out <= {wbuf[byte_cnt + 1][6:0], 1'b0};
                        bit_cnt   <= 3'd7;
                    end
                end else begin
                    bit_cnt   <= bit_cnt - 1;
                    spi_mosi  <= shift_out[7];
                    shift_out <= {shift_out[6:0], 1'b0};
                end
            end
        end
    end

    // ── S_DATA_RX: 收資料（READ/RDSR）─────────────────────────────────
    // MISO 在 rising edge 採樣（SPI Mode 0: flash 在 falling 後驅動 MISO）
    S_DATA_RX: begin
        if (spi_clk_en) begin
            spi_clk <= ~spi_clk;
            if (spi_clk == 1'b0) begin
                // rising edge：採樣 MISO（flash 已在前一個 falling edge 驅動好）
                shift_in <= {shift_in[6:0], spi_miso};
            end else begin
                // falling edge
                if (bit_cnt == 0) begin
                    spi_clk  <= 1'b0;
                    // byte 完成
                    rbuf[byte_cnt[7:0]] <= shift_in;
                    rdata_byte  <= shift_in;
                    rdata_idx   <= byte_cnt[7:0];
                    rdata_valid <= 1'b1;
                    byte_cnt    <= byte_cnt + 1;
                    bit_cnt     <= 3'd7;
                    if (byte_cnt + 1 >= saved_len) begin
                        // RDSR：取 status byte
                        if (saved_cmd == OP_RDSR)
                            status_byte <= shift_in;
                        state   <= S_CS_HI;
                        cs_hold <= 4'd3;
                    end
                end else begin
                    bit_cnt <= bit_cnt - 1;
                end
            end
        end
    end

    S_CS_HI: begin
        spi_cs_n <= 1'b1;
        spi_clk  <= 1'b0;
        spi_mosi <= 1'b0;
        if (cs_hold == 0) begin
            if (saved_cmd == OP_SE || saved_cmd == OP_PP) begin
                // 需要輪詢 WIP
                wip_timer <= WIP_TIMEOUT;
                cs_hold   <= 4'd3;
                state     <= S_POLL;
            end else if (saved_cmd == OP_RDSR) begin
                // RDSR 完成，回 S_POLL 檢查 status_byte WIP bit
                cs_hold <= 4'd3;
                state   <= S_POLL;
            end else begin
                state <= S_DONE;
            end
        end else begin
            cs_hold <= cs_hold - 1;
        end
    end

    S_POLL: begin
        // 發送 RDSR，取回 status，若 WIP=1 繼續輪詢
        // timeout 最高優先（嵌套 else 確保不衝突）
        if (wip_timer == 0) begin
            err   <= 1'b1;
            busy  <= 1'b0;
            state <= S_IDLE;
        end else begin
            wip_timer <= wip_timer - 1;
            if (cs_hold == 0) begin
                // 重新啟動 RDSR 序列
                spi_cs_n  <= 1'b0;
                spi_mosi  <= FC_RDSR[7];
                shift_out <= {FC_RDSR[6:0], 1'b0};
                bit_cnt   <= 3'd7;
                byte_cnt  <= 9'd0;
                saved_cmd <= OP_RDSR;
                state     <= S_CMD;
            end else begin
                cs_hold <= cs_hold - 1;
                // 檢查上次 RDSR 結果（saved_cmd==OP_RDSR 時 status_byte 有效）
                if (saved_cmd == OP_RDSR && (status_byte & 8'h01) == 0) begin
                    state <= S_DONE;
                end
            end
        end
    end

    S_DUMMY: begin
        // 8 dummy clocks for FC_READ (0x0C Fast Read 4B)
        // MOSI = 0（保持，不需要改）
        if (spi_clk_en) begin
            spi_clk <= ~spi_clk;
            if (spi_clk == 1'b1) begin  // falling edge
                if (bit_cnt == 0) begin
                    spi_clk  <= 1'b0;
                    bit_cnt  <= 3'd7;
                    byte_cnt <= 9'd0;
                    state    <= S_DATA_RX;
                end else begin
                    bit_cnt <= bit_cnt - 1;
                end
            end
        end
    end

    S_DONE: begin
        done  <= 1'b1;
        busy  <= 1'b0;
        state <= S_IDLE;
    end

    default: state <= S_IDLE;
    endcase

    if (rst) begin
        state    <= S_IDLE;
        busy     <= 1'b0;
        done     <= 1'b0;
        err      <= 1'b0;
        spi_clk  <= 1'b0;
        spi_cs_n <= 1'b1;
        spi_mosi <= 1'b0;
    end
end

endmodule
`default_nettype wire
