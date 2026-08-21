`timescale 1ns/1ps
`default_nettype none

// aurora_data_channel_relayfifo.v -- the relay portion of
// aurora_data_channel.v, switched to an official fifo_generator for
// streaming buffering, replacing the original hand-written
// pkt_buf/pkt_wr_idx/relay_rd_idx/pkt_len_r whole-packet
// store-and-forward logic.
//
// Background (2026-07-17): `sim/tb_wave_stream_relay_overflow.v` proved
// that the original design -- "capture the whole packet before
// forwarding it" -- relies on a 7-bit pkt_wr_idx (PKT_BUF_DEPTH=128) to
// remember the packet's position. Once a packet exceeds 128 beats
// (common for real T_WAVEFORM_STREAM traffic), pkt_wr_idx wraps around
// and overwrites the packet header; the whole packet vanishes at the
// relay hop and the receiver gets nothing.
//
// Fix: change the relay to a "forward while receiving" streaming
// design. The FIFO is only a rate buffer (absorbing the brief delay
// when local origination and relay traffic both compete for the tx1
// arbiter) -- it no longer needs to remember the packet length. The
// tlast bit is stored alongside each beat in the FIFO; on readout,
// seeing tlast=1 tells us the forwarded packet is done. There's no
// longer an upper bound on "how long can a packet be."
//
// **This module no longer has a built-in FIFO** -- following this
// project's existing convention (async_fifo_local/aurora_rx/aurora_tx
// are all top-level fifo_generator BD cells, with RTL modules only
// exposing wr/rd interfaces), the relay FIFO is built in create_bd.tcl
// with the official xilinx.com:ip:fifo_generator
// (Fifo_Implementation=Common_Clock_Block_RAM, since both read and
// write are in the aurora_clk domain; Input/Output_Data_Width=65
// {tlast,tdata}; Input_Depth=256, matching the same convention as
// async_fifo_aurora_tx), connected externally via the relay_wr_*/
// relay_rd_* ports.
//
// This version has not yet been wired into create_bd.tcl / the real
// fifo_generator IP. For now it's only verified in iverilog simulation
// against a behavioral stub (sim/fifo_stub_common_clock.v, with port
// naming matching the fifo_generator Native interface:
// clk/rst/din/wr_en/dout/valid/rd_en), which verifies this module's own
// state machine/arbitration logic. The FIFO IP itself doesn't need (and
// can't, with iverilog) be verified -- that's the Xilinx-verified part.
// Real wiring correctness will be confirmed with ILA on hardware.
//
// Other than the relay section, all other logic (RX beat0 decoding,
// local delivery decision, TX_LOCAL local origination, round-robin
// arbitration, lock/unlock) is identical to aurora_data_channel.v,
// unchanged.

module aurora_data_channel_relayfifo (
    input  wire        aurora_clk,
    input  wire        rst,
    input  wire [15:0] board_id,

    // 2026-07-21 新增：src_id 合理性檢查用。接 aurora_ctrl_channel_0 的
    // 既有輸出（同一個 aurora_clk domain，不需要 CDC）。enum 完成前是
    // 5'd0，用保守預設值 DEFAULT_MAX_BOARDS 當上限；enum 完成後變成
    // 實際板數，範圍收緊成真正存在的板子數。見 NOTES.md「Aurora Layer 2
    // 資料轉送與 TX 仲裁」章節 2026-07-21 小節「轉送邏輯缺少通用迴圈
    // 終止機制」的根因分析。
    input  wire [4:0]  total_boards,

    // 2026-07-29 新增：channel_up_0 閘控用。接同一個 aurora_64b66b_0/
    // channel_up（跟 aurora_ctrl_channel_0/channel_up_0 同來源）。連線
    // 還沒真正建立前不信任 rx0_tvalid，見 rtl/aurora_ctrl_channel.v
    // 檔頭同日說明、NOTES.md 對應章節。
    input  wire        channel_up_0,

    // -- Incoming relay source (fixed direction: rx0 = aurora_64b66b_0/SFP1) --
    input  wire [63:0] rx0_tdata,
    input  wire        rx0_tvalid,

    // 2026-07-30 新增：Layer3(aurora_ctrl_channel_0)的 forward rx0_tvalid
    // 改接這個訊號，不再直接接實體 GT 的 tvalid（見該檔案 rx0_tvalid
    // port 註解、NOTES.md 對應章節）。根因：Layer2/Layer3 平行看同一條
    // rx0_tdata，但 Layer3 的 FRX 狀態機完全沒有 framing（不知道現在
    // 是不是卡在 Layer2 自己的長封包中間），只要某個 payload word 剛好
    // 湊出 Layer3 認得的 type+合理 src/dest，就會被誤判成合法 beat0。
    // 這裡只有「真的是新封包開頭、且這個新封包是 ctrl-owned 類型」或
    // 「這是剛才那個 ctrl 封包的第二拍」這兩種情況才是 1，其餘時間
    // （尤其是 Layer2 自己在追蹤的長封包中間，rx_state==ST_DATA）一律
    // 是 0，Layer3 完全看不到那些拍，不會再誤判。
    //
    // 用 beat0_is_ctrl_owned（不是只有 beat0_is_fresh）當條件，是為了
    // 保證跟 Layer2 自己「這一拍是不是我的」的判斷互斥：beat0_is_ctrl_
    // owned=1 時，beat0_is_other 一定是 0，連帶 beat0_is_mine 一定是
    // 0——只要這個訊號因為這個條件變成 1，Layer2 保證不會把同一拍也
    // 拿去 relay/本地送達，不是巧合對上，是同一個 beat0_is_ctrl_owned
    // 同時餵給兩邊。
    output wire        rx0_valid_for_ctrl,

    // -- Local-side output (when data's destination is this board) --
    output wire [63:0] local_out_tdata,
    output wire        local_out_tvalid,

    // -- Request to tx1 (merged with Layer 3 control output by an external arbiter) --
    output wire [63:0] data_tx1_tdata,
    output wire        data_tx1_tvalid,
    output wire        data_tx1_tlast,
    input  wire        data_tx1_tready,

    // -- Data this board originates itself (from dispatcher.v, via async_fifo_aurora_tx) --
    input  wire [63:0] local_tx_tdata,
    input  wire        local_tx_tvalid,
    input  wire        local_tx_tlast,
    output wire        local_tx_tready,

    // -- Interface with Layer 3 (reservation/trigger protocol) --
    input  wire        lock_req,
    input  wire        unlock_req,
    output wire        is_busy,
    output wire        locked,

    // -- Relay FIFO write side (connects to external fifo_generator's
    //    din+wr_en, 65-bit = {tlast, tdata}) --
    output wire [64:0] relay_wr_din,     // {tlast, tdata}
    output wire        relay_wr_en,
    input  wire        relay_wr_full,    // currently only used for the overflow
                                          // diagnostic flag -- rx0 itself has no
                                          // tready to actually apply backpressure
                                          // (the physical Aurora RX must be
                                          // consumed once received); depth=256
                                          // is sized to not actually fill up

    // -- Relay FIFO read side (connects to external fifo_generator's dout+valid+rd_en) --
    input  wire [64:0] relay_rd_dout,    // {tlast, tdata}
    input  wire        relay_rd_valid,
    output wire        relay_rd_en,

    output wire        dbg_overflow,     // relay_wr_en && relay_wr_full occurring together
    output wire        debug_pkt_busy,   // relay_rd_valid (FIFO still has data waiting to relay)
    output wire [1:0]  debug_tx_state,

    // 2026-07-21 新增：排查 rx0 疑似雜訊被誤判成合法封包、持續佔滿
    // TX_RELAY 仲裁時間片的問題。純 assign 鏡像輸出，不影響其他邏輯。
    output wire        dbg_rx_state,
    output wire        dbg_cur_is_mine,
    output wire        dbg_cur_to_relay,
    output wire        dbg_cur_to_local,
    output wire        dbg_beat0_to_relay,

    // 2026-07-21 新增：src_id 合理性檢查（RX 端 + TX 端雙重防護，見上方
    // port 說明）的偵錯輸出。dbg_rx_src_rejected：RX 端收到 beat0 時
    // src_id 不合理、拒絕進入 ST_DATA 追蹤（1-cycle pulse）。
    // dbg_tx_drain：TX 端偵測到要送出的內容 src_id 不合理，正在排空
    // （level，排空期間持續為 1）。
    output wire        dbg_rx_src_rejected,
    output wire        dbg_tx_drain,

    // 2026-07-28 新增，同一天改版（Opus 覆查抓到第一版用純計時當
    // watchdog 判據會誤傷合法長封包，改成用封包自己宣告的 pkt_len
    // 當預算，見下方 local_pkt_budget 說明）：TX_LOCAL/TX_DRAIN(local)
    // watchdog 逾時次數，飽和計數（8'hFF 封頂，不 wrap），不是單一
    // sticky bit——只知道「發生過」不夠，需要知道「幾次」才能判斷是
    // 偶發還是持續發生。`async_fifo_aurora_tx`（餵這裡的 local_tx_*）
    // 已經直接用 xsim 真實 IP 證實跟 `async_fifo_reply_tx` 有一樣的
    // reset race（見 NOTES.md「async_fifo_reply_tx reset race 用
    // xsim 真實 IP 重現」+「補測：對稱風險點」兩個章節）。正常運作
    // 這個訊號應該永遠是 0。
    output reg  [7:0]    dbg_tx_local_watchdog_count = 8'd0
);

    localparam [7:0] TYPE_DATA      = 8'h13;
    localparam [7:0] TYPE_DATA_LAST = 8'h14;
    localparam [7:0] TYPE_ENUM_COUNT       = 8'h20;
    localparam [7:0] TYPE_TRIG_PAUSE_REQ   = 8'h22;
    localparam [7:0] TYPE_TRIG_PAUSE_ACK   = 8'h23;
    localparam [7:0] TYPE_TRIG_GO          = 8'h24;
    localparam [7:0] TYPE_DATA_RESERVE_REQ = 8'h25;
    localparam [7:0] TYPE_DATA_RESERVE_ACK = 8'h26;
    // 2026-08-20 新增：enum 失敗時定位斷點用的診斷 REQ/ACK（見
    // aurora_ctrl_channel.v 檔頭同日說明）。0x27/0x28 已經是 T_REINIT/
    // T_EXT_CLK_SEL（local_reg_handler.v 認的完全不同封包空間），改用
    // 0x2F/0x30——目前這個 8-bit type 空間實際用到 0x2D
    // （T_SINE_LIST_CTRL），0x2F/0x30 確認未被使用。
    localparam [7:0] TYPE_DIAG_REQ         = 8'h2F;
    localparam [7:0] TYPE_DIAG_ACK         = 8'h30;

    // ══════════════════════════════════════════════════════════════════
    //  RX0 side: identical beat0 decode/state machine to
    //  aurora_data_channel.v, except "capturing" no longer writes into a
    //  hand-written array -- it writes into the external FIFO instead
    // ══════════════════════════════════════════════════════════════════
    localparam ST_IDLE = 1'b0;
    localparam ST_DATA = 1'b1;

    reg        rx_state = ST_IDLE;
    reg [23:0] rx_remaining;
    reg        cur_is_mine;
    reg        cur_to_local;
    reg        cur_to_relay;
    reg        cur_is_last;

    // 2026-07-22 新增（根因4修法）：ctrl-owned 封包（enum/trig_pause/
    // trig_go/reserve，見下方 beat0_is_ctrl_owned）在 aurora_ctrl_
    // channel.v 裡固定是 2 拍，但這個模組只認 beat0，完全不知道還有
    // 第二拍 payload——沒有這個 flag 的話，下一拍進來時 rx_state 還在
    // ST_IDLE，payload 的任意內容會被誤判成一個全新的 beat0（count 值
    // 常常剛好解出 pkt_len=0），垃圾寫進 relay FIFO 卡死 TX_RELAY。
    // 見到 ctrl-owned 的 beat0 時設成 1，下一拍無條件當成「已經被
    // ctrl_channel 消化掉的 payload」跳過，不重新解析、不本地送達、
    // 不轉送。詳見 NOTES.md「Aurora Layer 2 資料轉送與 TX 仲裁」章節
    // 2026-07-22 根因4小節。
    reg        expect_ctrl_beat1 = 1'b0;

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
                               (beat0_type == TYPE_DATA_RESERVE_ACK) ||
                               (beat0_type == TYPE_DIAG_REQ)         ||
                               (beat0_type == TYPE_DIAG_ACK);
    wire beat0_is_other = !beat0_is_data && !beat0_is_ctrl_owned;

    // 2026-07-21 新增，這裡繼續留著（TX 端第二道防護要單獨用，見下方
    // relay_out_src_valid/local_tx_src_valid 說明，跟下面 RX beat0
    // 合法性判斷是兩件事、各自獨立計算，不要共用同一個 instance）。
    localparam [4:0] DEFAULT_MAX_BOARDS = 5'd4;
    wire [4:0] effective_max_boards = (total_boards != 5'd0) ? total_boards : DEFAULT_MAX_BOARDS;

    // 2026-07-29 改用集中化的 aurora_rx_beat0_gate.v（見該檔案檔頭
    // 說明）：src_id/dest_id 合理性 + channel_up 建立與否，一次判斷完。
    // dest_id 合法性只看實際值本身（==0xFFFF 或落在合法板數範圍內），
    // 不看 beat0 type，不需要在這裡另外維護 broadcast type 白名單
    // （修正：舊版用 per-type 白名單漏列 T_BOARD_ID_ASSIGN，導致它
    // broadcast 時被誤判成 unicast 而整包被丟棄，見 NOTES.md 2026-07-29
    // 「build9 上機：發現一個新的、獨立的迴歸 bug」章節）。
    wire beat0_gate_ok;
    aurora_rx_beat0_gate u_beat0_gate (
        .channel_up             (channel_up_0),
        .total_boards_in        (total_boards),
        .rx_tvalid              (rx0_tvalid),
        .beat0_src_id           (beat0_src_id),
        .beat0_dest_id          (beat0_dest_id),
        .beat0_gate_ok          (beat0_gate_ok)
    );

    // src_id/dest_id 不合理、或 channel_up 還沒建立的封包一律不認
    // （不是「我的」），RX 狀態機不會為它進入 ST_DATA 追蹤，也不會
    // 本地送達或轉送——一次判斷同時擋住「這片板子自己被雜訊卡住」跟
    // 「雜訊被轉送出去」兩個問題。
    wire beat0_is_mine  = (beat0_is_data || beat0_is_other) && beat0_gate_ok;

    wire beat0_ring_return = (beat0_dest_id == 16'hFFFF) && (beat0_src_id == board_id);

    // 2026-07-29：改用 beat0_gate_ok（原本是 beat0_src_valid），現在
    // 同時涵蓋 dest_id 合理性 + channel_up，不是只有 src_id。
    wire beat0_to_local = beat0_gate_ok && (
                          (beat0_is_data  && (beat0_dest_id == board_id)) ||
                          (beat0_is_other && ((beat0_dest_id == board_id) ||
                                              ((beat0_dest_id == 16'hFFFF) && !beat0_ring_return))));
    wire beat0_to_relay = beat0_gate_ok && (
                          (beat0_is_data  && (beat0_dest_id != board_id)) ||
                          (beat0_is_other && (beat0_dest_id != board_id) && !beat0_ring_return));

    wire rx_fire = rx0_tvalid;

    // 2026-07-22 新增（根因4修法）：只有「真的是一個新封包的 beat0」才
    // 相信 beat0_* 這幾個解碼結果——rx_state==ST_IDLE 這個條件本身不夠，
    // 還要排除「這一拍其實是上一個 ctrl-owned 封包的第二拍」的情況。
    wire beat0_is_fresh = (rx_state == ST_IDLE) && !expect_ctrl_beat1;

    // 2026-07-30 新增：見 rx0_valid_for_ctrl port 註解。只有「這是新
    // 封包的開頭、且是 ctrl-owned 類型」或「這是剛才那個 ctrl 封包的
    // 第二拍」，Layer3 才看得到這一拍——跟 beat0_is_mine 用同一個
    // beat0_is_ctrl_owned，保證兩邊判斷互斥。
    assign rx0_valid_for_ctrl = rx0_tvalid &&
                                 ((beat0_is_fresh && beat0_is_ctrl_owned) || expect_ctrl_beat1);

    assign local_out_tdata  = rx0_tdata;
    assign local_out_tvalid = rx_fire &&
                               (beat0_is_fresh ? beat0_to_local :
                                (rx_state == ST_IDLE) ? 1'b0 :   // consuming ctrl beat1, never local
                                (cur_is_mine && cur_to_local));

    // -- Relay decision: same as the original, except "should this beat
    //    be captured into the buffer" no longer needs the !pkt_busy
    //    condition -- the FIFO natively supports queuing multiple
    //    packets, so there's no need for the "must finish forwarding
    //    the previous one before accepting the next" serialization
    //    restriction --------------------------------------------------
    wire capturing_idle = rx_fire && beat0_is_fresh && beat0_to_relay;
    wire capturing_data = rx_fire && (rx_state == ST_DATA) && cur_is_mine && cur_to_relay;
    wire pkt_last_beat  = beat0_is_fresh ? (beat0_pkt_len == 24'd1) : (rx_remaining == 24'd1);

    assign relay_wr_en  = capturing_idle || capturing_data;
    assign relay_wr_din = {pkt_last_beat, rx0_tdata};   // {tlast, tdata}
    assign dbg_overflow = relay_wr_en && relay_wr_full;

    // 2026-07-21 新增，2026-07-29 擴充（beat0 型別上會被認得，但
    // src_id/dest_id 不合理、或 channel_up 還沒建立，才算「拒絕」；
    // type 本來就不認得的 6 種 Layer3 專屬 type 不算，那邊本來就不是
    // 這個模組要管的）。port 名稱沿用「src_rejected」歷史命名，現在
    // 涵蓋 src_id/dest_id/channel_up 三種拒絕原因。
    assign dbg_rx_src_rejected = rx_fire && beat0_is_fresh &&
                                  (beat0_is_data || beat0_is_other) && !beat0_gate_ok;

    always @(posedge aurora_clk) begin
        if (rst) begin
            rx_state          <= ST_IDLE;
            rx_remaining      <= 24'd0;
            cur_is_mine       <= 1'b0;
            cur_to_local      <= 1'b0;
            cur_to_relay      <= 1'b0;
            cur_is_last       <= 1'b0;
            expect_ctrl_beat1 <= 1'b0;
        end else begin
            case (rx_state)
                ST_IDLE: begin
                    if (rx_fire) begin
                        if (expect_ctrl_beat1) begin
                            // 2026-07-22 新增（根因4修法）：這一拍是上一個
                            // ctrl-owned 封包（enum/trig_pause/trig_go/
                            // reserve，固定 2 拍）的第二拍 payload，已經被
                            // aurora_ctrl_channel.v 平行消化掉——不重新
                            // 解析、不更新 cur_*、不本地送達、不轉送，只
                            // 把這個 flag 清掉即可。
                            expect_ctrl_beat1 <= 1'b0;
                        end else begin
                            cur_is_mine       <= beat0_is_mine;
                            cur_to_local      <= beat0_to_local;
                            cur_to_relay      <= beat0_to_relay;
                            cur_is_last       <= beat0_is_last;
                            expect_ctrl_beat1 <= beat0_is_ctrl_owned;

                            // 2026-07-21 新增：只有 beat0_is_mine（已經內含
                            // src_id 合理性檢查）才相信這個封包宣稱的
                            // pkt_len、進入 ST_DATA 追蹤後續拍數。src_id 不
                            // 合理的封包（不管是雜訊還是任何我們沒預料到的
                            // 內容）視為單一拍的雜訊事件，忽略掉、留在
                            // ST_IDLE，下一拍 rx0_tvalid 進來時可以重新
                            //正常解析，不會被假造的巨大 pkt_len 拖住。
                            if (beat0_is_mine && (beat0_pkt_len > 24'd1)) begin
                                rx_remaining <= beat0_pkt_len - 24'd1;
                                rx_state     <= ST_DATA;
                            end
                        end
                    end
                end
                ST_DATA: begin
                    if (rx_fire) begin
                        rx_remaining <= rx_remaining - 24'd1;
                        if (rx_remaining == 24'd1) rx_state <= ST_IDLE;
                    end
                end
                default: rx_state <= ST_IDLE;
            endcase
        end
    end

    // -- Unlock detection: seeing 0x14 pass through rx0 auto-unlocks, same as the original --
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
    //  TX side: draining the relay (reading from the FIFO) + local
    //  origination, 2-way round-robin, same logic as the original,
    //  except "does relay still have something to send" now checks
    //  relay_rd_valid -- no longer need pkt_len_r/relay_rd_idx to
    //  compute "how many beats remain" -- the FWFT FIFO's tlast bit
    //  directly tells us whether this is the last beat
    // ══════════════════════════════════════════════════════════════════
    localparam TX_IDLE  = 2'd0;
    localparam TX_RELAY = 2'd1;
    localparam TX_LOCAL = 2'd2;
    // 2026-07-21 新增：TX 端第二道防護（見 port 宣告處註解）。發現要送
    // 出去的內容 src_id 不合理時，不進 TX_RELAY/TX_LOCAL 真的送出去，
    // 改進這個狀態把整個不合法的封包排空（relay FIFO 讀到 tlast，或
    // local_tx 讀到 tlast），不佔用 data_tx1，排空完再回 TX_IDLE 重新
    // 判斷——只是擋住 tvalid 而不排空的話，仲裁器會一直卡在等這個
    // 已經被拒絕的封包，等於在 TX 端重新製造一次 RX 端本來要解決的
    // 同一種卡死問題。
    localparam TX_DRAIN = 2'd3;

    reg [1:0] tx_state = TX_IDLE;
    reg       rr_favor_local;
    reg       drain_is_relay;   // 排空中的來源：1=relay FIFO，0=local_tx

    wire relay_has_data  = relay_rd_valid;
    wire relay_is_last   = relay_rd_dout[64];   // tlast bit
    wire local_can_start = local_tx_tvalid && !locked_r;

    wire [15:0] relay_out_src_id   = relay_rd_dout[31:16];
    wire        relay_out_src_valid = (relay_out_src_id < {11'd0, effective_max_boards});
    wire [15:0] local_tx_src_id    = local_tx_tdata[31:16];
    wire        local_tx_src_valid = (local_tx_src_id < {11'd0, effective_max_boards});

    // 2026-07-28 新增，同一天改版（Opus 覆查 + iverilog 實測抓到第一版
    // 的致命 bug：純計時 watchdog 對 TX_LOCAL 不安全——`local_tx` 走的
    // 是 `T_WAVEFORM_STREAM`，`pkt_len` 是 24-bit，合法封包可以遠遠
    // 超過任何固定的計時上限，第一版拿 `aurora_tx1_arbiter.v`
    // ctrl/reply 那組逾時值原封不動抄過來，會把正常、只是很長的波形
    // 封包腰斬，親自用真實 RTL 測過 70000 拍合法封包會被誤傷）：
    //
    // 改成用封包**自己宣告的 `pkt_len`** 當預算，不是固定的計時上限
    // ——進入 TX_LOCAL/TX_DRAIN(local) 時，從 `local_tx_tdata[63:40]`
    // （beat0 header，此時 `local_can_start`/`local_tx_tvalid` 已經
    // 確認為真，這個欄位穩定可讀）鎖存要送幾拍，每消耗一拍就減一，
    // 減到 0 還沒看到 `tlast`，才判定「這個封包實際上比自己宣告的
    // 還長，一定是異常」而中止——這個判據拿 RTL 自己既有的規格當
    // 標準，不是憑空猜一個數字，而且對任何長度的合法封包都不會誤判
    // （跟這個模組 RX 側 `rx_remaining`(:158,289-299) 判斷 pkt_len
    // 的既有慣例精神一致，只是這裡是 TX 側）。
    //
    // 殘留限制（老實記錄，不是完美方案）：如果 reset race 產生的垃圾
    // `pkt_len`（同樣未定義/隨機）剛好是一個很大的數字，這個機制仍然
    // 會等到消耗完那個（垃圾）數字的拍數才中止，不是每次都能在固定
    // 短時間內恢復——但比第一版「完全沒有上限、可能永久卡死」已經好
    // 很多，而且不會誤傷任何真正合法的封包，這是這兩個目標之間可以
    // 做到的最佳平衡，沒有再加一個額外的「絕對逾時」保底機制（那需要
    // 另外猜一個「真實環境下最長合法封包大概多久傳完」的數字，這個
    // session 沒有實測數據支持，不確定的地方不猜測，先不加）。
    //
    // relay 側（relay_fifo_0）是 Common_Clock（aurora_clk 兩側同一個
    // clock，非 CDC），沒有 reset race 風險，TX_RELAY/TX_DRAIN(relay)
    // 刻意不加任何 watchdog，維持既有驗證過行為不變。
    localparam [63:0] POISON_TDATA = 64'h0000_0000_FFFF_0000;

    reg [23:0] local_pkt_budget;
    reg        abort_local_pending;

    wire abort_pending = abort_local_pending;

    always @(posedge aurora_clk) begin
        if (rst) begin
            tx_state                    <= TX_IDLE;
            rr_favor_local               <= 1'b0;
            drain_is_relay               <= 1'b0;
            local_pkt_budget             <= 24'd0;
            abort_local_pending          <= 1'b0;
            dbg_tx_local_watchdog_count  <= 8'd0;
        end else begin
            // poison beat 送出優先權最高：只要還 pending，先把它排空
            // （見下方 assign data_tx1_* 的優先權），不做任何其他狀態
            // 轉換（TX_IDLE 的轉換邏輯下面用 !abort_pending 擋住）。
            if (abort_local_pending && data_tx1_tready) abort_local_pending <= 1'b0;

            case (tx_state)
                TX_IDLE: begin
                    if (!abort_pending) begin
                        // src_id 不合理的優先排空掉，不管另一條路徑當下
                        // 狀態如何——先確保不會把不合法內容送上 tx1。
                        if (relay_has_data && !relay_out_src_valid) begin
                            tx_state       <= TX_DRAIN;
                            drain_is_relay <= 1'b1;
                        end else if (local_can_start && !local_tx_src_valid) begin
                            tx_state         <= TX_DRAIN;
                            drain_is_relay   <= 1'b0;
                            local_pkt_budget <= local_tx_tdata[63:40];
                        end else if (relay_has_data && local_can_start) begin
                            if (rr_favor_local) begin
                                tx_state         <= TX_LOCAL;
                                rr_favor_local   <= 1'b0;
                                local_pkt_budget <= local_tx_tdata[63:40];
                            end else begin
                                tx_state       <= TX_RELAY;
                                rr_favor_local <= 1'b1;
                            end
                        end else if (relay_has_data) begin
                            tx_state <= TX_RELAY;
                        end else if (local_can_start) begin
                            tx_state         <= TX_LOCAL;
                            local_pkt_budget <= local_tx_tdata[63:40];
                        end
                    end
                end
                TX_RELAY: if (data_tx1_tready) begin
                    if (relay_is_last) tx_state <= TX_IDLE;
                    // Not the last beat: stay in TX_RELAY, keep reading
                    // the FIFO next beat (FWFT: once rd_en fires this
                    // beat, dout automatically advances to the next
                    // entry on the following clk)
                end
                TX_LOCAL: begin
                    if (data_tx1_tready && local_tx_tvalid && local_tx_tlast) begin
                        tx_state <= TX_IDLE;
                    end else if (data_tx1_tready && local_tx_tvalid) begin
                        // 消耗掉非最後一拍。budget 用完還沒看到 tlast
                        // -> 這個封包已經超過自己宣告的長度，判定異常。
                        if (local_pkt_budget <= 24'd1) begin
                            tx_state             <= TX_IDLE;
                            abort_local_pending  <= 1'b1;
                            if (dbg_tx_local_watchdog_count != 8'hFF)
                                dbg_tx_local_watchdog_count <= dbg_tx_local_watchdog_count + 8'd1;
                        end else begin
                            local_pkt_budget <= local_pkt_budget - 24'd1;
                        end
                    end
                    // else：這一拍沒被真正消耗（tvalid=0 或
                    // data_tx1_tready=0），維持原狀，繼續等——backpressure
                    // 不會被誤判成異常，因為判據是「消耗掉的拍數」不是
                    // 「經過的時間」。
                end
                TX_DRAIN: begin
                    // 排空不佔用 data_tx1（local 分支 tready 也不用等
                    // data_tx1_tready，見下方 assign），直接用
                    // relay_rd_en/local_tx_tready 把整個不合法封包讀完
                    // 丟掉，讀到 tlast 才回 TX_IDLE。local 分支一樣用
                    // pkt_len 預算保底，避免同一種 reset race 垃圾讓
                    // 這裡也卡死（雖然這裡從沒對 data_tx1 asserted
                    // tvalid，卡死頂多是「這個模組整個沒辦法處理任何
                    // 流量」的可用性問題，不是資料損毀，不需要補送
                    // 終結 beat）。
                    if (drain_is_relay) begin
                        if (relay_has_data && relay_is_last) tx_state <= TX_IDLE;
                    end else begin
                        if (local_tx_tvalid && local_tx_tlast) begin
                            tx_state <= TX_IDLE;
                        end else if (local_tx_tvalid) begin
                            if (local_pkt_budget <= 24'd1) begin
                                tx_state <= TX_IDLE;
                                if (dbg_tx_local_watchdog_count != 8'hFF)
                                    dbg_tx_local_watchdog_count <= dbg_tx_local_watchdog_count + 8'd1;
                            end else begin
                                local_pkt_budget <= local_pkt_budget - 24'd1;
                            end
                        end
                    end
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    // relay_rd_en: FWFT mode，正常轉送時要等 data_tx1_tready；排空時不用等
    assign relay_rd_en = ((tx_state == TX_RELAY) && relay_has_data && data_tx1_tready) ||
                         ((tx_state == TX_DRAIN) && drain_is_relay && relay_has_data);

    // poison beat 優先權最高，其次才是正常的 relay/local 選擇——確保
    // watchdog 觸發後一定能送出終結 beat，讓 Aurora 硬體 frame 正常
    // 結束（不然下一個真正的封包會被接續在同一個 frame 裡）。poison
    // beat 內容 pkt_len=0/type=0x00/src_id=0xFFFF/dest_id=0x0000，
    // 同時符合下游兩層既有的安全機制：`dispatcher.v` 把 pkt_len==0
    // 當 padding 直接丟棄；下一站的這個模組/`aurora_ctrl_channel.v`
    // 的 src_id 合理性檢查會判定 src_id=0xFFFF 不合理，直接當噪音
    // 丟棄，不會被 relay 或本地送達，兩層都會忽略這個 poison beat。
    assign data_tx1_tdata  = abort_local_pending  ? POISON_TDATA      :
                             (tx_state == TX_RELAY) ? relay_rd_dout[63:0] :
                             (tx_state == TX_LOCAL) ? local_tx_tdata      : 64'd0;
    assign data_tx1_tvalid = abort_local_pending  ? 1'b1                :
                             (tx_state == TX_RELAY) ? relay_has_data      :
                             (tx_state == TX_LOCAL) ? local_tx_tvalid     : 1'b0;
    assign data_tx1_tlast  = abort_local_pending  ? 1'b1                :
                             (tx_state == TX_RELAY) ? relay_is_last       :
                             (tx_state == TX_LOCAL) ? local_tx_tlast      : 1'b0;
    // local_tx_tready：正常轉本機發起時要等 data_tx1_tready；排空
    // local_tx 時不用等（直接吃掉丟棄）。abort_local_pending 期間
    // tx_state 已經是 TX_IDLE，兩個條件天然都不成立，不會跟 poison
    // beat 同時誤收真正的 local_tx 資料。
    assign local_tx_tready = ((tx_state == TX_LOCAL) && data_tx1_tready) ||
                             ((tx_state == TX_DRAIN) && !drain_is_relay);

    assign is_busy         = (tx_state == TX_LOCAL);
    assign debug_pkt_busy  = relay_has_data;
    assign debug_tx_state  = tx_state;
    assign dbg_tx_drain    = (tx_state == TX_DRAIN);

    assign dbg_rx_state      = rx_state;
    assign dbg_cur_is_mine   = cur_is_mine;
    assign dbg_cur_to_relay  = cur_to_relay;
    assign dbg_cur_to_local  = cur_to_local;
    assign dbg_beat0_to_relay = beat0_to_relay;

endmodule
`default_nettype wire
