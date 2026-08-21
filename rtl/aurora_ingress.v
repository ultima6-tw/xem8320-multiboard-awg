`timescale 1ns/1ps
`default_nettype none

// aurora_ingress.v -- Step 15a Layer 1: single-side ingress processor
//
// 2026-07-05 首版草稿（DRAFT，尚未模擬/上板驗證）。
//
// 職責：看「一個方向」進來的 rx_tdata/rx_tvalid（來自其中一顆
// aurora_64b66b 核心的 m_axi_rx_*），解出 beat0 header
// （pkt_len/type/src_id/dest_id），套用通用規則判斷這個封包（**不分
// type**）要不要交給本地、要不要轉送到另一側：
//
//   dest_id == board_id 或 dest_id == 0xFFFF（broadcast）
//     -> 交給本地（local_tdata/tvalid），給上層（sys_clk 或本板控制邏輯）處理
//   dest_id != board_id（且不是自己發出的 broadcast 繞回來）
//     -> 轉送到另一側的 TX（relay_tdata/tvalid/tlast）
//   broadcast 且 src_id == board_id（自己發的 broadcast 繞完一圈回來）
//     -> ring-return，整包丟棄，不轉送、不交給本地
//
// 這個通用規則對 data（0x13/0x14）跟大部分廣播型控制封包
// （T_ENUM_TOTAL/T_TRIG_PAUSE_REQ/T_TRIG_GO）都適用，也剛好正確處理
// T_ENUM_COUNT 的自我定址情況（dest_id=發起板自己的 board_id）：中途站
// dest_id 跟自己不同 -> 轉送；繞回發起板時 dest_id 等於自己 -> 交給本地、
// 不再轉送，不需要特殊處理。
//
// **這一層不做的事**（留給上層 data/ctrl channel 模組）：
//   - 不修改封包內容——這裡是「原封不動轉送」，像 T_ENUM_COUNT 需要
//     「payload 減 1 才轉送」這種逐站修改內容的行為，這一層不支援，
//     ctrl channel 要自己重新處理這類封包，不能直接沿用這個模組的輸出
//   - 不做 T_DATA_RESERVE_REQ 的 fail-fast（忙碌就不繼續轉送）——
//     這是上層的協定邏輯，這一層永遠照「dest_id 決定要不要轉送」的
//     通用規則做，不會自己判斷「忙不忙」
//   - 不做本地端要「發起」新封包的邏輯（本板自己的 FP 資料、本板自己
//     要發起的控制封包）——這裡純粹是「收到什麼就依規則轉送/交本地」
//
// Aurora RX 沒有 tready/tlast（Framing mode），封包邊界靠 pkt_len 倒數
// （沿用 dispatcher.v 的慣例）。Store-and-forward：收完整個封包才開始
// 轉送，不做 cut-through（2026-07-05 使用者確認：資料完整性優先）。
//
// 已知限制：Aurora RX 無 backpressure，如果 relay buffer 還沒排空、
// 下一個要轉送的封包就緊接著進來，會被丟棄（dbg_overflow 可觀察）。
// 目前預期流量下（控制封包低頻率、data chunk 16 beats）應該不會撞到，
// 待模擬/實測驗證。

module aurora_ingress #(
    parameter PKT_BUF_DEPTH = 16   // beats; covers both 16-beat data chunk and 2-beat control msg
)(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] board_id,

    // ── From one Aurora RX core (m_axi_rx_tdata/tvalid, no tready/tlast) ──
    input  wire [63:0] rx_tdata,
    input  wire        rx_tvalid,

    // ── Local delivery (destined here, or broadcast) ──────────────────────
    output wire [63:0] local_tdata,
    output wire        local_tvalid,

    // ── Relay output (feeds the OTHER side's TX, wired by the parent
    //    aurora_dir_switch.v -- no backpressure expected downstream, matches
    //    existing aurora_tx_arbiter.v's relay_* convention) ────────────────
    output wire [63:0] relay_tdata,
    output wire        relay_tvalid,
    output wire        relay_tlast,

    output wire        dbg_overflow
);

    localparam ST_IDLE = 1'b0;
    localparam ST_DATA = 1'b1;

    reg        state = ST_IDLE;
    reg [23:0] remaining;
    reg        cur_to_local;
    reg        cur_to_relay;

    wire [23:0] beat0_pkt_len = rx_tdata[63:40];
    wire [15:0] beat0_src_id  = rx_tdata[31:16];
    wire [15:0] beat0_dest_id = rx_tdata[15:0];

    wire beat0_bcast       = (beat0_dest_id == 16'hFFFF);
    wire beat0_ring_return = beat0_bcast && (beat0_src_id == board_id);
    wire beat0_to_local    = !beat0_ring_return && ((beat0_dest_id == board_id) || beat0_bcast);
    wire beat0_to_relay    = !beat0_ring_return && (beat0_dest_id != board_id);

    wire rx_fire = rx_tvalid;

    // LOCAL path: no buffering here -- relies on downstream (async_fifo_aurora_rx,
    // 256-deep) for elasticity, same assumption as the pre-15a single-direction design.
    assign local_tdata  = rx_tdata;
    assign local_tvalid = rx_fire && ((state == ST_IDLE) ? beat0_to_local : cur_to_local);

    // ── relay packet buffer (store-and-forward) ─────────────────────────
    reg [63:0] pkt_buf [0:PKT_BUF_DEPTH-1];
    reg [4:0]  pkt_wr_idx;    // next free slot while capturing
    reg [4:0]  pkt_len_r;     // latched beat count once capture completes
    reg        pkt_busy;      // 1 = buffer holds a complete packet awaiting TX drain
    reg        pkt_overflow;  // sticky diagnostic

    wire capturing_idle = rx_fire && (state == ST_IDLE) && beat0_to_relay;
    wire capturing_data = rx_fire && (state == ST_DATA) && cur_to_relay;
    wire pkt_last_beat  = (state == ST_IDLE) ? (beat0_pkt_len == 24'd1) : (remaining == 24'd1);
    wire capture_done   = (capturing_idle || capturing_data) && pkt_last_beat && !pkt_busy;
    wire drain_done;   // driven by the TX-side logic below

    // Bug fix (2026-07-05, found via testbench T4): pkt_wr_idx is a *registered*
    // value that only reflects prior cycles' writes -- in ST_IDLE it still holds
    // a stale leftover from whatever packet was captured previously (e.g. a
    // single-beat packet right after a 3-beat one would wrongly compute
    // pkt_len_r = 3+1 instead of 1). The write position for the beat being
    // captured *this* cycle is always 0 in ST_IDLE (first beat of a new packet),
    // and only equals the registered pkt_wr_idx in ST_DATA (where it correctly
    // accumulated over prior cycles of the *same* in-progress packet).
    wire [4:0] cur_wr_pos = (state == ST_IDLE) ? 5'd0 : pkt_wr_idx;

    always @(posedge clk) begin
        if (rst) begin
            state        <= ST_IDLE;
            remaining    <= 24'd0;
            cur_to_local <= 1'b0;
            cur_to_relay <= 1'b0;
            pkt_wr_idx   <= 5'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (rx_fire) begin
                        cur_to_local <= beat0_to_local;
                        cur_to_relay <= beat0_to_relay;

                        if (capturing_idle && !pkt_busy) begin
                            pkt_buf[0] <= rx_tdata;
                            pkt_wr_idx <= 5'd1;
                        end

                        if (beat0_pkt_len > 24'd1) begin
                            remaining <= beat0_pkt_len - 24'd1;
                            state     <= ST_DATA;
                        end
                    end
                end
                ST_DATA: begin
                    if (rx_fire) begin
                        remaining <= remaining - 24'd1;
                        if (capturing_data && !pkt_busy) begin
                            pkt_buf[pkt_wr_idx] <= rx_tdata;
                            pkt_wr_idx          <= pkt_wr_idx + 5'd1;
                        end
                        if (remaining == 24'd1) state <= ST_IDLE;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            pkt_busy     <= 1'b0;
            pkt_len_r    <= 5'd0;
            pkt_overflow <= 1'b0;
        end else begin
            if (capture_done) begin
                pkt_busy  <= 1'b1;
                pkt_len_r <= cur_wr_pos + 5'd1;   // beats written so far, incl. this one
            end else if (drain_done) begin
                pkt_busy <= 1'b0;
            end

            if ((capturing_idle || capturing_data) && pkt_busy) begin
                pkt_overflow <= 1'b1;   // sticky -- beat dropped, buffer was still busy
            end
        end
    end

    assign dbg_overflow = pkt_overflow;

    // ── TX side: drain relay buffer, no backpressure expected downstream ──
    reg [4:0] relay_rd_idx;
    reg       draining;

    wire relay_is_last = (relay_rd_idx == pkt_len_r - 5'd1);
    assign drain_done = draining && relay_is_last;

    always @(posedge clk) begin
        if (rst) begin
            relay_rd_idx <= 5'd0;
            draining     <= 1'b0;
        end else begin
            if (!draining && pkt_busy) begin
                draining     <= 1'b1;
                relay_rd_idx <= 5'd0;
            end else if (draining) begin
                if (relay_is_last) draining <= 1'b0;
                else               relay_rd_idx <= relay_rd_idx + 5'd1;
            end
        end
    end

    assign relay_tdata  = pkt_buf[relay_rd_idx];
    assign relay_tvalid = draining;
    assign relay_tlast  = draining && relay_is_last;

endmodule
`default_nettype wire
