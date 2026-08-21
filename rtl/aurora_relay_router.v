`timescale 1ns/1ps
`default_nettype none

// aurora_relay_router.v -- Step 15a: aurora_clk domain packet relay/router
//
// 2026-07-05 首版草稿（DRAFT，尚未模擬/上板驗證）：只處理 DATA 通道
// （type==0x13 T_WAVEFORM_STREAM 封包的 relay）。控制通道（開機編號/
// pause/trigger，type 0x20-0x24）由另一個獨立模組
// `aurora_ctrl_channel.v` 處理，走既有 aurora_tx_arbiter.v 的 relay_*
// 輸入（不是這裡的 wave_*）。兩個模組平行看同一份 rx_tdata/rx_tvalid，
// 各自只認自己的 type，互不干擾，不需要協調狀態機。這個模組**只在
// beat0 type==0x13 時才動作**，其他 type 一律忽略（不進 pkt_buf、
// local_tvalid 不拉高），留給 `aurora_ctrl_channel.v` 處理。
//
// 動機：dispatcher.v 的 RT_TX 轉送每一站都要繞 aurora_clk->sys_clk->
// aurora_clk 兩次 CDC，多站累加的延遲/jitter 不夠乾淨。這裡整段搬到
// aurora_clk domain 直接處理，結構比照 dispatcher.v 既有驗證過的
// IDLE/DATA 狀態機模式（beat0 解 header -> remaining 倒數轉發），只是
// 輸出目的地換成 aurora 這邊：
//   dest_id==自己 board_id  -> LOCAL（給既有 async_fifo_aurora_rx，接到 sys_clk dispatcher）
//   dest_id==0xFFFF         -> LOCAL + RELAY 都送
//   以上都不符合             -> RELAY（繼續轉送到下一站 Aurora TX）
//
// 封包格式（跟 dispatcher.v 完全一致，beat0，64-bit）：
//   [63:40] pkt_len (24-bit，含 beat0 本身)
//   [39:32] type    (8-bit，這個模組不解讀 type，留給下游 sys_clk dispatcher 處理)
//   [31:16] src_id  (16-bit)
//   [15:0]  dest_id (16-bit，0xFFFF=broadcast)
//
// board_id 比照 dispatcher.v，是 runtime input（board_cfg_reg 設定），
// 不是 compile-time parameter -- 所有板子燒同一份 bitstream。
//
// Aurora RX 沒有 tready/tlast（Framing mode。查證
// awg-test-step-14.3/vivado/create_bd.tcl：aurora_64b66b_0/m_axi_rx_tdata
// + m_axi_rx_tvalid 只有這兩條線）。封包邊界跟 dispatcher.v 一樣靠
// pkt_len 倒數計算，不依賴 hardware tlast。
//
// RELAY 封包用 store-and-forward：收完整個封包（最多 16 beat）才開始
// 往下一站送，不做 cut-through（2026-07-05 使用者確認：資料完整性優先
// 於即時性）。
//
// 已知限制（尚未解決，留待後續評估）：Aurora RX 沒有 backpressure，如果
// relay buffer（pkt_busy）還沒排空、下一個要轉送的封包就緊接著進來，
// 會被丟棄（dbg_pkt_overflow 會 sticky 拉高，可用 ILA 觀察）。目前 3 板
// 環路、16-beat chunk 的低負載場景下應該不會撞到，若之後要拉高資料率，
// 這裡需要重新評估（例如加大深度做雙緩衝）。
//
// wave_* 輸出（接既有 aurora_tx_arbiter.v 的 wave_tdata/tvalid/tlast/tready）
// 現在有兩個來源會搶：(a) 本板 dispatcher.v 自己要送出去的資料
// （local_tx_* input，經既有 async_fifo_aurora_tx CDC 進來，不變）
// (b) 這裡轉送別的板子送來的封包（relay buffer 排空）。用簡單
// round-robin 在兩者間選擇（沒有特別理由偏好誰優先，之後有需要再調整）。

module aurora_relay_router #(
    parameter PKT_BUF_DEPTH = 16   // beats per relay chunk (matches 16-beat chunk design)
)(
    input  wire        aurora_clk,
    input  wire        rst,

    input  wire [15:0] board_id,

    // ── From Aurora RX core (aurora_64b66b_0/m_axi_rx_*) ──────────────────
    // No tready/tlast -- Framing mode, must always be able to accept a beat.
    input  wire [63:0] rx_tdata,
    input  wire        rx_tvalid,

    // ── To local sys_clk domain (existing async_fifo_aurora_rx/din,wr_en) ──
    output wire [63:0] local_tdata,
    output wire        local_tvalid,

    // ── From this board's own dispatcher.v, via existing async_fifo_aurora_tx
    //    (sys_clk->aurora_clk CDC, unchanged) ──────────────────────────────
    input  wire [63:0] local_tx_tdata,
    input  wire        local_tx_tvalid,
    input  wire        local_tx_tlast,
    output wire        local_tx_tready,

    // ── To existing aurora_tx_arbiter.v's wave_* input (data channel) ──────
    output wire [63:0] wave_tdata,
    output wire        wave_tvalid,
    output wire        wave_tlast,
    input  wire        wave_tready,

    // ── Diagnostics ─────────────────────────────────────────────────────
    output wire        dbg_pkt_overflow   // sticky: a relay beat was dropped (see comment above)
);

    // ══════════════════════════════════════════════════════════════════
    //  RX side: decode beat0, ring-return guard, route to LOCAL/RELAY/BOTH
    // ══════════════════════════════════════════════════════════════════
    localparam ST_IDLE = 1'b0;
    localparam ST_DATA = 1'b1;

    reg        rx_state    = ST_IDLE;
    reg [23:0] rx_remaining;
    reg        rx_to_local;
    reg        rx_to_relay;
    reg        rx_discard;   // ring-return: burn through beats, output nowhere

    localparam [7:0] TYPE_WAVEFORM = 8'h13;   // T_WAVEFORM_STREAM -- the only type this module acts on

    wire [23:0] beat0_pkt_len = rx_tdata[63:40];
    wire [7:0]  beat0_type    = rx_tdata[39:32];
    wire [15:0] beat0_src_id  = rx_tdata[31:16];
    wire [15:0] beat0_dest_id = rx_tdata[15:0];

    wire beat0_is_data     = (beat0_type == TYPE_WAVEFORM);
    wire beat0_ring_return = (beat0_src_id  == board_id);
    wire beat0_to_local    = beat0_is_data && ((beat0_dest_id == board_id) || (beat0_dest_id == 16'hFFFF));
    wire beat0_to_relay    = beat0_is_data && (beat0_dest_id != board_id);   // covers "not mine" and broadcast

    wire rx_fire = rx_tvalid;

    // LOCAL path: no buffering here -- straight combinational hand-off,
    // relies on the existing async_fifo_aurora_rx (256-deep) downstream for
    // any elasticity, same assumption the pre-15a design made.
    assign local_tdata  = rx_tdata;
    assign local_tvalid = rx_fire &&
                          ((rx_state == ST_IDLE) ? (beat0_to_local && !beat0_ring_return)
                                                  : (rx_to_local   && !rx_discard));

    // ── relay packet buffer ─────────────────────────────────────────────
    reg [63:0] pkt_buf [0:PKT_BUF_DEPTH-1];
    reg [4:0]  pkt_wr_idx;    // next free slot while capturing
    reg [4:0]  pkt_len_r;     // latched beat count once capture completes
    reg        pkt_busy;      // 1 = buffer holds a complete packet awaiting TX drain
    reg        pkt_overflow;  // sticky diagnostic

    wire capturing_idle = rx_fire && (rx_state == ST_IDLE) && beat0_to_relay && !beat0_ring_return;
    wire capturing_data = rx_fire && (rx_state == ST_DATA) && rx_to_relay   && !rx_discard;
    wire pkt_last_beat  = (rx_state == ST_IDLE) ? (beat0_pkt_len == 24'd1)
                                                 : (rx_remaining  == 24'd1);
    wire capture_done   = (capturing_idle || capturing_data) && pkt_last_beat && !pkt_busy;
    wire drain_done;   // driven by the TX-side state machine below

    always @(posedge aurora_clk) begin
        if (rst) begin
            rx_state     <= ST_IDLE;
            rx_remaining <= 24'd0;
            rx_to_local  <= 1'b0;
            rx_to_relay  <= 1'b0;
            rx_discard   <= 1'b0;
            pkt_wr_idx   <= 5'd0;
        end else begin
            case (rx_state)
                ST_IDLE: begin
                    if (rx_fire) begin
                        rx_to_local <= beat0_to_local;
                        rx_to_relay <= beat0_to_relay;
                        rx_discard  <= beat0_ring_return;

                        if (capturing_idle && !pkt_busy) begin
                            pkt_buf[0] <= rx_tdata;
                            pkt_wr_idx <= 5'd1;
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
                            pkt_buf[pkt_wr_idx] <= rx_tdata;
                            pkt_wr_idx          <= pkt_wr_idx + 5'd1;
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
            pkt_busy     <= 1'b0;
            pkt_len_r    <= 5'd0;
            pkt_overflow <= 1'b0;
        end else begin
            if (capture_done) begin
                pkt_busy  <= 1'b1;
                pkt_len_r <= pkt_wr_idx + 5'd1;   // beats written so far, incl. this one
            end else if (drain_done) begin
                pkt_busy  <= 1'b0;
            end

            if ((capturing_idle || capturing_data) && pkt_busy) begin
                pkt_overflow <= 1'b1;   // sticky -- beat dropped, buffer was still busy
            end
        end
    end

    assign dbg_pkt_overflow = pkt_overflow;

    // ══════════════════════════════════════════════════════════════════
    //  TX side: drain relay buffer + arbitrate with local_tx_* onto wave_*
    // ══════════════════════════════════════════════════════════════════
    localparam TX_IDLE  = 2'd0;
    localparam TX_RELAY = 2'd1;
    localparam TX_LOCAL = 2'd2;

    reg [1:0] tx_state       = TX_IDLE;
    reg [4:0] relay_rd_idx;
    reg       rr_favor_local;   // round-robin toggle for when both sources are ready

    wire relay_is_last = (relay_rd_idx == pkt_len_r - 5'd1);
    assign drain_done  = (tx_state == TX_RELAY) && wave_tready && relay_is_last;

    always @(posedge aurora_clk) begin
        if (rst) begin
            tx_state       <= TX_IDLE;
            relay_rd_idx   <= 5'd0;
            rr_favor_local <= 1'b0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    relay_rd_idx <= 5'd0;
                    if (pkt_busy && local_tx_tvalid) begin
                        if (rr_favor_local) begin
                            tx_state       <= TX_LOCAL;
                            rr_favor_local <= 1'b0;
                        end else begin
                            tx_state       <= TX_RELAY;
                            rr_favor_local <= 1'b1;
                        end
                    end else if (pkt_busy) begin
                        tx_state <= TX_RELAY;
                    end else if (local_tx_tvalid) begin
                        tx_state <= TX_LOCAL;
                    end
                end
                TX_RELAY: if (wave_tready) begin
                    if (relay_is_last) tx_state <= TX_IDLE;
                    else                relay_rd_idx <= relay_rd_idx + 5'd1;
                end
                TX_LOCAL: if (wave_tready && local_tx_tvalid && local_tx_tlast) begin
                    tx_state <= TX_IDLE;
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    assign wave_tdata  = (tx_state == TX_RELAY) ? pkt_buf[relay_rd_idx] :
                         (tx_state == TX_LOCAL) ? local_tx_tdata        : 64'd0;
    assign wave_tvalid = (tx_state == TX_RELAY) ? 1'b1 :
                         (tx_state == TX_LOCAL) ? local_tx_tvalid       : 1'b0;
    assign wave_tlast  = (tx_state == TX_RELAY) ? relay_is_last :
                         (tx_state == TX_LOCAL) ? local_tx_tlast       : 1'b0;
    assign local_tx_tready = (tx_state == TX_LOCAL) && wave_tready;

endmodule
`default_nettype wire
