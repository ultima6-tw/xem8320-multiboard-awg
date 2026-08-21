`timescale 1ns/1ps
`default_nettype none

// ddr4_stream_reader: continuously reads DDR4 → 32-bit FIFO for DAC streaming.
// Runs on DDR4 UI clock (~300 MHz). play_en/start_addr/wave_len are quasi-static
// inputs from clk_100 domain; play_en is 2-FF synced, start_addr/wave_len are
// latched in S_IDLE (safe: they are stable well before play_en asserts).
// Burst size: variable, 1–16 beats × 128-bit (4–64 × 32-bit words per refill).
// wave_len (32-bit words) can be any positive integer.
// The last AXI burst reads ceil(wave_len/4)*4 samples; samples beyond wave_len
// are suppressed (not written to FIFO). start_addr must be 256-byte aligned.
//
// ── 2026-07-26 本地修改（step-16）：local copy (not the shared
// awg-test-step-14/rtl/ file) ──────────────────────────────────────────
// 新增 `idle` output（= state==S_IDLE 的組合邏輯），修正一個真實 bug：
// S_AR/S_R/S_WRITE4（burst 進行中）完全不檢查 play_en，只有 S_IDLE/
// S_WAIT 會檢查——如果 trigger 發生時 reader 剛好在跑 burst，play_en
// 拉低對它沒用，burst 跑完時 play_en 可能已經重新拉高，reader 永遠不會
// 經過 S_IDLE，rd_ptr/samp_cnt 不會歸零，剛 flush 完的 FIFO 被填入從
// 舊位置接續的樣本，不是樣本 0（造成同一塊板子上不同 channel 重複觸發
// 後相位偶發性大幅跳動，見 PROJECT.md「重大突破：用 Opus 深度推理」
// 小節完整根因分析）。這裡只新增一個唯讀的狀態輸出，不改變任何既有
// FSM 行為；實際修法（等這個訊號確認閒置才真正 flush）在
// waveform_controller.v。

module ddr4_stream_reader (
    input  wire        clk,
    input  wire        rst,
    input  wire        calib_done,
    input  wire        play_en,       // level: high = playing

    input  wire [31:0] start_addr,   // byte address, 256-byte aligned
    input  wire [31:0] wave_len,     // waveform length in 32-bit words (any value)

    // AXI4 read-only master (128-bit)
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME m_axi, PROTOCOL AXI4, DATA_WIDTH 128, ADDR_WIDTH 32, HAS_BURST 1, HAS_LOCK 0, HAS_CACHE 0, HAS_PROT 0, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 0, HAS_BRESP 0, HAS_RRESP 1, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 0, MAX_BURST_LENGTH 64, READ_WRITE_MODE READ_ONLY" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARADDR" *)
    output reg  [31:0] m_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARLEN" *)
    output reg  [7:0]  m_axi_arlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARSIZE" *)
    output wire [2:0]  m_axi_arsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARBURST" *)
    output wire [1:0]  m_axi_arburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARVALID" *)
    output reg         m_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARREADY" *)
    input  wire        m_axi_arready,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RDATA" *)
    input  wire [127:0] m_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RRESP" *)
    input  wire [1:0]   m_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RLAST" *)
    input  wire         m_axi_rlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RVALID" *)
    input  wire         m_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RREADY" *)
    output wire         m_axi_rready,

    // FIFO write port (32-bit, synchronous)
    output reg  [31:0] fifo_din,
    output reg         fifo_we,
    input  wire        fifo_prog_full,  // high when FIFO level >= 256

    output wire        active,

    // ── idle（2026-07-26 新增）────────────────────────────────────────
    // 高電位代表這個 reader 目前真的閒置在 S_IDLE（rd_ptr/samp_cnt 已經
    // 歸零、下次 play_en 拉高會從乾淨的樣本 0 重新開始）。waveform_
    // controller.v 用這個訊號（2-flop sync 進 dac_clk）決定 flush 是否
    // 真的可以視為完成，不能只看 FIFO 自己的 wr_rst_busy/rd_rst_busy。
    output wire        idle
);

// ── AXI4 constants ────────────────────────────────────────────────────
// 狀態機宣告搬到最前面（跟 xvlog 相容，見 sim/ 底下 scratch 複本的
// forward-reference 說明），localparam/reg 宣告先於使用它們的 assign。
localparam S_IDLE    = 3'd0,
           S_WAIT    = 3'd1,
           S_AR      = 3'd2,
           S_R       = 3'd3,
           S_WRITE4  = 3'd4;

reg [2:0]   state;
(* ASYNC_REG = "TRUE" *) reg play_en_s1;
(* ASYNC_REG = "TRUE" *) reg play_en_s2;

assign m_axi_arsize  = 3'd4;     // 16 bytes per beat
assign m_axi_arburst = 2'b01;    // INCR
assign m_axi_rready  = (state == S_R);
assign active        = play_en_s2 && calib_done;
assign idle          = (state == S_IDLE);

// ── 2-FF sync: play_en (clk_100) → clk domain ─────────────────────────
always @(posedge clk or posedge rst) begin
    if (rst) begin
        play_en_s1 <= 1'b0;
        play_en_s2 <= 1'b0;
    end else begin
        play_en_s1 <= play_en;
        play_en_s2 <= play_en_s1;
    end
end

// ── State machine ─────────────────────────────────────────────────────
reg [27:0]  rd_ptr;         // in 128-bit words (each = 16 bytes)
reg [27:0]  len_128;        // ceil(wave_len / 4): total 128-bit words to read per loop
reg [31:0]  wave_len_r;     // exact sample count (latched from wave_len in S_IDLE)
reg [31:0]  samp_cnt;       // absolute sample index within current loop (0..len_128*4-1)
reg [31:0]  start_addr_r;   // latched from quasi-static input in S_IDLE
reg [1:0]   sub;
reg [127:0] rd_buf;
reg         rd_last;
reg [7:0]   burst_beats_r;  // actual beats in current burst (1..16)

// Combinatorial: remaining 128-bit words from rd_ptr to end of wave
wire [27:0] remaining_128 = len_128 - rd_ptr;

always @(posedge clk) begin
    if (rst) begin
        state         <= S_IDLE;
        rd_ptr        <= 28'd0;
        len_128       <= 28'd0;
        wave_len_r    <= 32'd0;
        samp_cnt      <= 32'd0;
        start_addr_r  <= 32'd0;
        m_axi_araddr  <= 32'd0;
        m_axi_arlen   <= 8'd63;
        m_axi_arvalid <= 1'b0;
        fifo_we       <= 1'b0;
        fifo_din      <= 32'd0;
        sub           <= 2'd0;
        rd_buf        <= 128'd0;
        rd_last       <= 1'b0;
        burst_beats_r <= 8'd64;
    end else begin
        fifo_we <= 1'b0;

        case (state)
            S_IDLE: begin
                if (calib_done && play_en_s2) begin
                    rd_ptr       <= 28'd0;
                    samp_cnt     <= 32'd0;
                    // ceil(wave_len / 4): add 1 if any low 2 bits set
                    len_128      <= wave_len[29:2] + {27'd0, |wave_len[1:0]};
                    wave_len_r   <= wave_len;
                    start_addr_r <= start_addr;
                    state        <= S_WAIT;
                end
            end

            S_WAIT: begin
                if (!play_en_s2) begin
                    state <= S_IDLE;
                end else if (!fifo_prog_full) begin
                    // Compute burst length: full 16-beat burst or shorter tail
                    if (remaining_128 >= 28'd64) begin
                        burst_beats_r <= 8'd64;
                        m_axi_arlen   <= 8'd63;
                    end else begin
                        burst_beats_r <= remaining_128[7:0];
                        m_axi_arlen   <= remaining_128[7:0] - 8'd1;
                    end
                    m_axi_araddr  <= start_addr_r + {rd_ptr, 4'b0};
                    m_axi_arvalid <= 1'b1;
                    state         <= S_AR;
                end
            end

            S_AR: begin
                if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    state         <= S_R;
                end
            end

            // rready is high in this state; stalls between beats while in S_WRITE4
            S_R: begin
                if (m_axi_rvalid) begin
                    rd_buf  <= m_axi_rdata;
                    rd_last <= m_axi_rlast;
                    sub     <= 2'd0;
                    state   <= S_WRITE4;
                end
            end

            S_WRITE4: begin
                // Suppress samples beyond wave_len (tail of last 128-bit word)
                fifo_we <= (samp_cnt < wave_len_r);
                case (sub)
                    2'd0: fifo_din <= rd_buf[31:0];
                    2'd1: fifo_din <= rd_buf[63:32];
                    2'd2: fifo_din <= rd_buf[95:64];
                    2'd3: fifo_din <= rd_buf[127:96];
                endcase

                if (sub == 2'd3 && rd_last &&
                    rd_ptr + {20'd0, burst_beats_r} >= len_128) begin
                    // Last sample of last burst: waveform wraps
                    rd_ptr   <= 28'd0;
                    samp_cnt <= 32'd0;
                    state    <= S_WAIT;
                end else begin
                    samp_cnt <= samp_cnt + 32'd1;
                    if (sub == 2'd3) begin
                        if (rd_last) begin
                            rd_ptr <= rd_ptr + {20'd0, burst_beats_r};
                            state  <= S_WAIT;
                        end else begin
                            state <= S_R;
                        end
                    end else begin
                        sub <= sub + 2'd1;
                    end
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
`default_nettype wire
