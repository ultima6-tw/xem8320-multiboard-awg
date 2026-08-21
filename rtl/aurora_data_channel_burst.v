`timescale 1ns/1ps
`default_nettype none

// aurora_data_channel_burst.v — 2026-07-16 實驗版本，只為了模擬測試用
// （不是要取代 aurora_data_channel.v，還沒確認要不要正式改）。
//
// 跟原版唯一的差異：「本機發起」這條路，原本是直接串流 pass-through
// （local_tx_tready = (tx_state==TX_LOCAL) && data_tx1_tready，要求整包
// 6 拍之間 local_tx_tvalid 不能斷），這裡改成跟 relay 那條路一樣的
// store-and-forward buffer：先把整包收滿、鎖存起來（local_tx_tready
// 只要 buffer 空就立刻接受，不受 tx_state/data_tx1_tready 影響），
// TX_LOCAL 送出時改成從 buffer 讀，不受上游 CDC FIFO/dispatcher 中間
// 斷檔影響。
//
// 其餘（RX 解碼、relay、lock/unlock）完全不動，逐字複製自
// aurora_data_channel.v（2026-07-16 版本）。

module aurora_data_channel_burst #(
    parameter PKT_BUF_DEPTH = 128
)(
    input  wire        aurora_clk,
    input  wire        rst,
    input  wire [15:0] board_id,

    input  wire [63:0] rx0_tdata,
    input  wire        rx0_tvalid,

    output wire [63:0] local_tdata,
    output wire        local_tvalid,

    output wire [63:0] data_tx1_tdata,
    output wire        data_tx1_tvalid,
    output wire        data_tx1_tlast,
    input  wire        data_tx1_tready,

    input  wire [63:0] local_tx_tdata,
    input  wire        local_tx_tvalid,
    input  wire        local_tx_tlast,
    output wire        local_tx_tready,

    input  wire        lock_req,
    input  wire        unlock_req,
    output wire        is_busy,
    output wire        locked,

    output wire        dbg_overflow,

    output wire        debug_pkt_busy,
    output wire [1:0]  debug_tx_state,

    // 2026-07-16 新增（burst 版本專用診斷）：本機發送 buffer 的狀態
    output wire        debug_local_pkt_busy,
    output wire [6:0]  debug_local_wr_idx,
    output wire [6:0]  debug_local_pkt_len
);

    localparam [7:0] TYPE_DATA      = 8'h13;
    localparam [7:0] TYPE_DATA_LAST = 8'h14;
    localparam [7:0] TYPE_ENUM_COUNT       = 8'h20;
    localparam [7:0] TYPE_TRIG_PAUSE_REQ   = 8'h22;
    localparam [7:0] TYPE_TRIG_PAUSE_ACK   = 8'h23;
    localparam [7:0] TYPE_TRIG_GO          = 8'h24;
    localparam [7:0] TYPE_DATA_RESERVE_REQ = 8'h25;
    localparam [7:0] TYPE_DATA_RESERVE_ACK = 8'h26;

    localparam ST_IDLE = 1'b0;
    localparam ST_DATA = 1'b1;

    reg        rx_state = ST_IDLE;
    reg [23:0] rx_remaining;
    reg        cur_is_mine;
    reg        cur_to_local;
    reg        cur_to_relay;
    reg        cur_is_last;

    wire [23:0] beat0_pkt_len  = rx0_tdata[63:40];
    wire [7:0]  beat0_type     = rx0_tdata[39:32];
    wire [15:0] beat0_src_id   = rx0_tdata[31:16];
    wire [15:0] beat0_dest_id  = rx0_tdata[15:0];

    wire beat0_is_data = (beat0_type == TYPE_DATA) || (beat0_type == TYPE_DATA_LAST);
    wire beat0_is_last = (beat0_type == TYPE_DATA_LAST);

    wire beat0_is_ctrl_owned = (beat0_type == TYPE_ENUM_COUNT)       ||
                               (beat0_type == TYPE_TRIG_PAUSE_REQ)   ||
                               (beat0_type == TYPE_TRIG_PAUSE_ACK)   ||
                               (beat0_type == TYPE_TRIG_GO)          ||
                               (beat0_type == TYPE_DATA_RESERVE_REQ) ||
                               (beat0_type == TYPE_DATA_RESERVE_ACK);
    wire beat0_is_other = !beat0_is_data && !beat0_is_ctrl_owned;
    wire beat0_is_mine  = beat0_is_data || beat0_is_other;

    wire beat0_ring_return = (beat0_dest_id == 16'hFFFF) && (beat0_src_id == board_id);

    wire beat0_to_local = (beat0_is_data  && (beat0_dest_id == board_id)) ||
                          (beat0_is_other && ((beat0_dest_id == board_id) ||
                                              ((beat0_dest_id == 16'hFFFF) && !beat0_ring_return)));
    wire beat0_to_relay = (beat0_is_data  && (beat0_dest_id != board_id)) ||
                          (beat0_is_other && (beat0_dest_id != board_id) && !beat0_ring_return);

    wire rx_fire = rx0_tvalid;

    assign local_tdata  = rx0_tdata;
    assign local_tvalid = rx_fire && ((rx_state == ST_IDLE) ? beat0_to_local : (cur_is_mine && cur_to_local));

    // ── relay packet buffer（跟原版逐字相同，不動）───────────────────────
    reg [63:0] pkt_buf [0:PKT_BUF_DEPTH-1];
    reg [6:0]  pkt_wr_idx;
    reg [6:0]  pkt_len_r;
    reg        pkt_busy;
    reg        pkt_overflow;
    reg        pkt_is_last_r;

    wire capturing_idle = rx_fire && (rx_state == ST_IDLE) && beat0_to_relay;
    wire capturing_data = rx_fire && (rx_state == ST_DATA) && cur_is_mine && cur_to_relay;
    wire pkt_last_beat  = (rx_state == ST_IDLE) ? (beat0_pkt_len == 24'd1) : (rx_remaining == 24'd1);
    wire capture_done   = (capturing_idle || capturing_data) && pkt_last_beat && !pkt_busy;
    wire drain_done;

    wire [6:0] cur_wr_pos = (rx_state == ST_IDLE) ? 7'd0 : pkt_wr_idx;

    always @(posedge aurora_clk) begin
        if (rst) begin
            rx_state     <= ST_IDLE;
            rx_remaining <= 24'd0;
            cur_is_mine  <= 1'b0;
            cur_to_local <= 1'b0;
            cur_to_relay <= 1'b0;
            cur_is_last  <= 1'b0;
            pkt_wr_idx   <= 7'd0;
        end else begin
            case (rx_state)
                ST_IDLE: begin
                    if (rx_fire) begin
                        cur_is_mine  <= beat0_is_mine;
                        cur_to_local <= beat0_to_local;
                        cur_to_relay <= beat0_to_relay;
                        cur_is_last  <= beat0_is_last;

                        if (capturing_idle && !pkt_busy) begin
                            pkt_buf[0] <= rx0_tdata;
                            pkt_wr_idx <= 7'd1;
                        end

                        if (beat0_pkt_len > 24'd1) begin
                            rx_remaining <= beat0_pkt_len - 24'd1;
                            rx_state     <= ST_DATA;
                        end
                    end
                end
                ST_DATA: begin
                    if (rx_fire) begin
                        rx_remaining <= rx_remaining - 24'd1;
                        if (capturing_data && !pkt_busy) begin
                            pkt_buf[pkt_wr_idx] <= rx0_tdata;
                            pkt_wr_idx          <= pkt_wr_idx + 7'd1;
                        end
                        if (rx_remaining == 24'd1) rx_state <= ST_IDLE;
                    end
                end
                default: rx_state <= ST_IDLE;
            endcase
        end
    end

    always @(posedge aurora_clk) begin
        if (rst) begin
            pkt_busy      <= 1'b0;
            pkt_len_r     <= 7'd0;
            pkt_overflow  <= 1'b0;
            pkt_is_last_r <= 1'b0;
        end else begin
            if (capture_done) begin
                pkt_busy      <= 1'b1;
                pkt_len_r     <= cur_wr_pos + 7'd1;
                pkt_is_last_r <= (rx_state == ST_IDLE) ? beat0_is_last : cur_is_last;
            end else if (drain_done) begin
                pkt_busy <= 1'b0;
            end

            if ((capturing_idle || capturing_data) && pkt_busy) begin
                pkt_overflow <= 1'b1;
            end
        end
    end

    assign dbg_overflow = pkt_overflow;
    assign debug_pkt_busy = pkt_busy;
    assign debug_tx_state = tx_state;

    wire pkt0_complete_is_last =
        (rx_state == ST_IDLE) ? (rx_fire && beat0_is_data && beat0_is_last && pkt_last_beat) :
                                 (rx_fire && cur_is_mine  && cur_is_last  && pkt_last_beat);
    wire unlock_pulse = pkt0_complete_is_last || unlock_req;

    reg locked_r;
    always @(posedge aurora_clk) begin
        if (rst) begin
            locked_r <= 1'b0;
        end else if (unlock_pulse) begin
            locked_r <= 1'b0;
        end else if (lock_req) begin
            locked_r <= 1'b1;
        end
    end
    assign locked = locked_r;

    // ══════════════════════════════════════════════════════════════════
    //  本機發送 buffer（新增，跟 relay 的 pkt_buf 同一種 store-and-forward
    //  模式）：local_tx_tready 只要 buffer 空就立刻接受，不受 tx_state/
    //  data_tx1_tready 影響，整包收滿才開始送
    // ══════════════════════════════════════════════════════════════════
    reg [63:0] local_pkt_buf [0:PKT_BUF_DEPTH-1];
    reg [6:0]  local_wr_idx;
    reg [6:0]  local_pkt_len_r;
    reg        local_pkt_busy;   // 1 = 已經收滿一包、等待/正在送出

    wire local_capturing = local_tx_tvalid && !local_pkt_busy;
    wire local_drain_done;

    assign local_tx_tready = !local_pkt_busy;

    always @(posedge aurora_clk) begin
        if (rst) begin
            local_wr_idx   <= 7'd0;
            local_pkt_busy <= 1'b0;
            local_pkt_len_r <= 7'd0;
        end else begin
            if (local_capturing) begin
                local_pkt_buf[local_wr_idx] <= local_tx_tdata;
                if (local_tx_tlast) begin
                    local_pkt_busy  <= 1'b1;
                    local_pkt_len_r <= local_wr_idx + 7'd1;
                    local_wr_idx    <= 7'd0;
                end else begin
                    local_wr_idx <= local_wr_idx + 7'd1;
                end
            end else if (local_drain_done) begin
                local_pkt_busy <= 1'b0;
            end
        end
    end

    assign debug_local_pkt_busy = local_pkt_busy;
    assign debug_local_wr_idx   = local_wr_idx;
    assign debug_local_pkt_len  = local_pkt_len_r;

    // ── TX side: relay 排空 + 本機發起（改讀 buffer），2-選-1 round-robin ──
    localparam TX_IDLE  = 2'd0;
    localparam TX_RELAY = 2'd1;
    localparam TX_LOCAL = 2'd2;

    reg [1:0] tx_state = TX_IDLE;
    reg [6:0] relay_rd_idx;
    reg [6:0] local_rd_idx;
    reg       rr_favor_local;

    wire relay_is_last = (relay_rd_idx == pkt_len_r - 7'd1);
    wire local_is_last  = (local_rd_idx == local_pkt_len_r - 7'd1);
    assign drain_done       = (tx_state == TX_RELAY) && data_tx1_tready && relay_is_last;
    assign local_drain_done = (tx_state == TX_LOCAL) && data_tx1_tready && local_is_last;

    // local origination 只允許在「整包已經收滿(local_pkt_busy)、沒有鎖定」
    // 時開始送出——不再依賴 local_tx_tvalid 這個即時串流訊號
    wire local_can_start = local_pkt_busy && !locked_r;

    always @(posedge aurora_clk) begin
        if (rst) begin
            tx_state       <= TX_IDLE;
            relay_rd_idx   <= 7'd0;
            local_rd_idx   <= 7'd0;
            rr_favor_local <= 1'b0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    relay_rd_idx <= 7'd0;
                    local_rd_idx <= 7'd0;
                    if (pkt_busy && local_can_start) begin
                        if (rr_favor_local) begin
                            tx_state       <= TX_LOCAL;
                            rr_favor_local <= 1'b0;
                        end else begin
                            tx_state       <= TX_RELAY;
                            rr_favor_local <= 1'b1;
                        end
                    end else if (pkt_busy) begin
                        tx_state <= TX_RELAY;
                    end else if (local_can_start) begin
                        tx_state <= TX_LOCAL;
                    end
                end
                TX_RELAY: if (data_tx1_tready) begin
                    if (relay_is_last) tx_state <= TX_IDLE;
                    else                relay_rd_idx <= relay_rd_idx + 7'd1;
                end
                TX_LOCAL: if (data_tx1_tready) begin
                    if (local_is_last) tx_state <= TX_IDLE;
                    else                local_rd_idx <= local_rd_idx + 7'd1;
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    assign data_tx1_tdata  = (tx_state == TX_RELAY) ? pkt_buf[relay_rd_idx] :
                             (tx_state == TX_LOCAL) ? local_pkt_buf[local_rd_idx] : 64'd0;
    assign data_tx1_tvalid = (tx_state == TX_RELAY) ? 1'b1 :
                             (tx_state == TX_LOCAL) ? 1'b1 : 1'b0;
    assign data_tx1_tlast  = (tx_state == TX_RELAY) ? relay_is_last :
                             (tx_state == TX_LOCAL) ? local_is_last : 1'b0;

    // is_busy：本機資料目前是否正在用 tx1（跟原版語意相同：TX_LOCAL 狀態）
    assign is_busy = (tx_state == TX_LOCAL);

endmodule
`default_nettype wire
