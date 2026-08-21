`timescale 1ns/1ps
`default_nettype none

// aurora_data_channel.v -- Step 15a Layer 2: data channel (relay + local
// origination + reservation lock/unlock) + 通用 catch-all 轉送
//
// 2026-07-15（step-16）修正兩個實測發現的 bug：
// 1. `PKT_BUF_DEPTH`（relay 用的 store-and-forward 緩衝區）原本是 16，
//    `pkt_wr_idx`/`relay_rd_idx`/`pkt_len_r` 都是 5-bit。轉送
//    `T_FLASH_WRITE_DATA`（33-beat）這種超過 16 beat 的封包時，寫入
//    索引沒有邊界檢查，第 17 個 beat 開始會繞回去蓋掉前面已經存的內容，
//    封包長度也算錯——本機處理（dest_id 是自己）完全不會走這個緩衝區，
//    只有真的需要「轉送給別人」時才會踩到，這就是為什麼之前的測試
//    （本機自己送給自己、trig_delay/board_id_assign 這些 2-beat 小封包）
//    都測不出來。改成 128（33-beat 的 ~4 倍，7-bit 計數器剛好對應
//    0-127，8Kbit BRAM 用量，跟這次 session 真的踩到 BRAM 緊張問題的
//    dispatcher_ila_0 差兩個數量級，資源上完全不是問題）。
// 2. `T_FLASH_WRITE_DATA` 原本用 `0x14`，跟這個模組自己定義的
//    `TYPE_DATA_LAST`（波形串流最後一個 chunk，也是 `0x14`）撞號——
//    2026-07-09 新增 `T_FLASH_WRITE_DATA` 時沒有跟這個模組已經佔用的
//    編號核對過。已經在 dispatcher.v／host 腳本把 `T_FLASH_WRITE_DATA`
//    改成 `0x16`，這裡不用改（這個模組的 `TYPE_DATA`/`TYPE_DATA_LAST`
//    定義維持 `0x13`/`0x14` 不動，兩者不再撞號）。
//
// 2026-07-05 首版草稿（DRAFT，尚未模擬）。
// 2026-07-05 補上通用 catch-all：BD 整合時發現 dispatcher.v 原本兼任「其他
// type 封包（0x01~0x11，board_cfg/calib/play_ctrl 等既有暫存器控制訊息，
// 見 local_reg_handler.v）跨板轉送＋本地送達」的角色，改用 Layer2/3 架構
// 後這個責任沒人接手。Layer 3 自己認的 6 種 type（ENUM_COUNT/TRIG_PAUSE_
// REQ/ACK/TRIG_GO/DATA_RESERVE_REQ/ACK）需要「有腦」轉送（payload 要改、
// 要查 is_busy），不能用通用規則，所以不能讓 Layer 2 通用轉送去搶——這裡
// 明確排除這 6 種 type，其餘（不是 0x13/0x14、也不是 Layer3 那 6 種）一律
// 套用通用 dest_id 規則（沿用 `aurora_ingress.v` 當初驗證過的規則：
// dest_id==自己或 broadcast→本地、其餘→轉送、broadcast 且 src_id==自己→
// 丟棄），取代 dispatcher.v 原本的角色。
//
// 只有 type==0x13(T_WAVEFORM_STREAM)/0x14(T_WAVEFORM_STREAM_LAST) 才會觸發
// lock/unlock 相關邏輯；Layer 3 自己認的 6 種 type 完全不經過這個模組（留給
// `aurora_ctrl_channel.v` 處理，兩者平行看同一份 rx0_tdata/rx0_tvalid，各自
// 只認自己的 type，這個模式沿用 15a 開發初期單向環版本就用過的做法）。
//
// **方向慣例**：data 永遠固定走 rx0(aurora_64b66b_0/SFP1) -> tx1
// (aurora_64b66b_1/SFP2) 這個方向（2026-07-05 確認：不做最短路徑選擇，
// 永遠固定方向）。這個模組不碰 rx1/tx0（控制 ACK 反向轉送是 Layer 3 的
// 職責，不是這裡）。
//
// **這個模組的 tx1 輸出不是直接接физ理 TX**：因為 tx1 這個方向同時要載
// data（這裡）跟控制封包（Layer 3，優先權更高），需要一個更上層的
// 仲裁器（Layer 4/頂層整合）在兩者之間仲裁，這裡只負責產生「data 通道
// 想送的東西」，`data_tx1_tready` 是外部仲裁器給的授權/backpressure
// 訊號，不是接到實體 Aurora TX 的 tready。
//
// **鎖定/解鎖機制**：`lock_req`（來自 Layer 3 的 `T_DATA_RESERVE_REQ`
// 處理邏輯，查完不忙碌就立刻觸發）拉高時鎖住本板本機資料的注入
// （`local_tx_tready` 不再允許新的 local 封包開始，但已經在傳的會讓它
// 傳完，不強行中斷）；看到 type==0x14 的封包經過 rx0（不管是要轉送還是
// 交本地）就自動解鎖（`locked` 清除），不需要額外的「結束」封包。
//
// `is_busy` 給 Layer 3 查詢「這一站忙不忙」（`T_DATA_RESERVE_REQ` fail-fast
// 用）：忙碌的定義是「本板自己的本機資料目前正在使用 tx1」（relay 流量
// 不算，因為 reservation 天生序列化，不會有「正在幫別人轉送別的 reservation」
// 這種情況，2026-07-05 確認）。

module aurora_data_channel #(
    parameter PKT_BUF_DEPTH = 128
)(
    input  wire        aurora_clk,
    input  wire        rst,
    input  wire [15:0] board_id,

    // ── 進來的 relay 來源（固定方向：rx0 = aurora_64b66b_0/SFP1）───────
    // 2026-07-10：mark_debug 保留 bus 分組資訊給 debug core insertion
    // 用（見 rtl/aurora_ctrl_channel.v 同一組訊號的說明）
    (* mark_debug = "true" *) input  wire [63:0] rx0_tdata,
    (* mark_debug = "true" *) input  wire        rx0_tvalid,

    // ── 本地端輸出（data 目的地是本板時）──────────────────────────────
    output wire [63:0] local_out_tdata,
    output wire        local_out_tvalid,

    // ── 送往 tx1 的請求（要跟 Layer 3 的控制輸出經外部仲裁器合併，
    //    不是直接接實體 TX，見檔頭說明）────────────────────────────────
    output wire [63:0] data_tx1_tdata,
    output wire        data_tx1_tvalid,
    output wire        data_tx1_tlast,
    input  wire        data_tx1_tready,

    // ── 本板自己要發起的資料（來自既有 dispatcher.v，經既有
    //    async_fifo_aurora_tx CDC bridge 進來，不變）─────────────────────
    input  wire [63:0] local_tx_tdata,
    input  wire        local_tx_tvalid,
    input  wire        local_tx_tlast,
    output wire        local_tx_tready,

    // ── 跟 Layer 3（reservation/trigger 協定）的介面 ───────────────────
    // 2026-07-05 補上 unlock_req：trigger 的 T_TRIG_PAUSE_REQ 也會鎖住本機
    // 資料（跟 reservation 用同一個 locked 狀態，兩者不會同時發生，見
    // PROJECT.md「待確認」——host 不會同時跑 trigger 跟 reservation
    // 協定，這裡先信任這個假設），但 trigger 的解鎖時機是「收到
    // T_TRIG_GO」，不是「看到 0x14 經過」，所以需要一個獨立的強制解鎖
    // 輸入，跟 reservation 那條「看到 0x14 自動解鎖」的路徑分開。
    input  wire        lock_req,    // pulse: 查完不忙碌，立刻鎖住本機資料注入
    input  wire        unlock_req,  // pulse: 強制解鎖（給 T_TRIG_GO 用）
    output wire        is_busy,     // level: 本機資料目前是否正在用 tx1
    output wire        locked,      // level: 目前是否鎖定中

    output wire        dbg_overflow,

    // 2026-07-16 新增：排查 T_DDR4_WRITE(0x04) 遠端寫入送不出去的問題，
    // pkt_busy/tx_state 是內部 reg，沒有既有 port 可以直接接 ILA，純 assign
    // 鏡像輸出（不影響其他邏輯）。
    output wire        debug_pkt_busy,
    output wire [1:0]  debug_tx_state
);

    localparam [7:0] TYPE_DATA      = 8'h13;
    localparam [7:0] TYPE_DATA_LAST = 8'h14;
    // Layer 3（aurora_ctrl_channel.v）專屬 type，Layer 2 完全不碰（catch-all
    // 排除清單，避免同一個封包被兩層都轉送一次）
    localparam [7:0] TYPE_ENUM_COUNT       = 8'h20;
    localparam [7:0] TYPE_TRIG_PAUSE_REQ   = 8'h22;
    localparam [7:0] TYPE_TRIG_PAUSE_ACK   = 8'h23;
    localparam [7:0] TYPE_TRIG_GO          = 8'h24;
    localparam [7:0] TYPE_DATA_RESERVE_REQ = 8'h25;
    localparam [7:0] TYPE_DATA_RESERVE_ACK = 8'h26;

    // ══════════════════════════════════════════════════════════════════
    //  RX0 side: 只認 0x13/0x14，decode + ring 不需要（data 不是 broadcast，
    //  永遠是 unicast 到某個 board_id），store-and-forward 緩衝待轉送
    // ══════════════════════════════════════════════════════════════════
    localparam ST_IDLE = 1'b0;
    localparam ST_DATA = 1'b1;

    reg        rx_state = ST_IDLE;
    reg [23:0] rx_remaining;
    reg        cur_is_mine;    // 這個封包是 Layer 2 該處理的（0x13/0x14 或通用 catch-all）
    reg        cur_to_local;
    reg        cur_to_relay;
    reg        cur_is_last;    // 這個封包是 0x14（最後一個 chunk，只有真的 0x14 才會是 1）

    wire [23:0] beat0_pkt_len  = rx0_tdata[63:40];
    wire [7:0]  beat0_type     = rx0_tdata[39:32];
    wire [15:0] beat0_src_id   = rx0_tdata[31:16];
    wire [15:0] beat0_dest_id  = rx0_tdata[15:0];

    wire beat0_is_data = (beat0_type == TYPE_DATA) || (beat0_type == TYPE_DATA_LAST);
    wire beat0_is_last = (beat0_type == TYPE_DATA_LAST);

    // 通用 catch-all：不是 0x13/0x14、也不是 Layer 3 認的 6 種 type，
    // 一律用通用 dest_id 規則（詳見檔頭說明）
    wire beat0_is_ctrl_owned = (beat0_type == TYPE_ENUM_COUNT)       ||
                               (beat0_type == TYPE_TRIG_PAUSE_REQ)   ||
                               (beat0_type == TYPE_TRIG_PAUSE_ACK)   ||
                               (beat0_type == TYPE_TRIG_GO)          ||
                               (beat0_type == TYPE_DATA_RESERVE_REQ) ||
                               (beat0_type == TYPE_DATA_RESERVE_ACK);
    wire beat0_is_other = !beat0_is_data && !beat0_is_ctrl_owned;
    wire beat0_is_mine  = beat0_is_data || beat0_is_other;

    // ring-return：只有 broadcast 才需要（0x13/0x14 是 unicast，不會碰到這個
    // 情況；catch-all 的 0x01~0x11/0x1A~0x1D 這類 host 封包如果用 broadcast
    // 送，繞完一圈要丟棄，否則會無限循環）。
    // 2026-07-08 bugfix：原本這個保護只接在 beat0_to_relay，沒有同時接在
    // beat0_to_local——繞一圈回到發起者自己時，relay 正確停止轉送，但
    // 「送達本地」判斷完全沒擋，導致發起者自己重新把這個廣播當成一筆新的
    // 本地送達再處理一次，又觸發一次本地動作、又送出新的廣播，形成雪球式
    // 重複觸發。上機用 ILA 直接量到 T_RESERVE_START(0x1C) 封包被同一個
    // aurora_data_channel_0 反覆送達本地上百次，才抓到這個漏洞。
    wire beat0_ring_return = (beat0_dest_id == 16'hFFFF) && (beat0_src_id == board_id);

    wire beat0_to_local = (beat0_is_data  && (beat0_dest_id == board_id)) ||
                          (beat0_is_other && ((beat0_dest_id == board_id) ||
                                              ((beat0_dest_id == 16'hFFFF) && !beat0_ring_return)));
    wire beat0_to_relay = (beat0_is_data  && (beat0_dest_id != board_id)) ||
                          (beat0_is_other && (beat0_dest_id != board_id) && !beat0_ring_return);

    wire rx_fire = rx0_tvalid;

    assign local_out_tdata  = rx0_tdata;
    assign local_out_tvalid = rx_fire && ((rx_state == ST_IDLE) ? beat0_to_local : (cur_is_mine && cur_to_local));

    // ── relay packet buffer (store-and-forward) ─────────────────────────
    reg [63:0] pkt_buf [0:PKT_BUF_DEPTH-1];
    reg [6:0]  pkt_wr_idx;
    reg [6:0]  pkt_len_r;
    reg        pkt_busy;
    reg        pkt_overflow;
    reg        pkt_is_last_r;   // latched: is the buffered packet the 0x14 (last chunk)?

    wire capturing_idle = rx_fire && (rx_state == ST_IDLE) && beat0_to_relay;
    wire capturing_data = rx_fire && (rx_state == ST_DATA) && cur_is_mine && cur_to_relay;
    wire pkt_last_beat  = (rx_state == ST_IDLE) ? (beat0_pkt_len == 24'd1) : (rx_remaining == 24'd1);
    wire capture_done   = (capturing_idle || capturing_data) && pkt_last_beat && !pkt_busy;
    wire drain_done;

    // Layer 1 的教訓（2026-07-05 testbench 抓到的 bug）：ST_IDLE 時
    // pkt_wr_idx 還是上一個封包的殘值，不能直接拿來算 pkt_len_r，要用
    // cur_wr_pos 這個組合邏輯依 state 判斷。
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

    // ── 解鎖偵測：看到 0x14 經過 rx0（轉送或交本地都算），就解鎖 ─────────
    // 用「這個封包收完最後一個 beat」的當下當觸發點，不管最終是轉送還是
    // 本地送達，反正本板對這次傳輸的參與到此結束。
    wire pkt0_complete_is_last =
        (rx_state == ST_IDLE) ? (rx_fire && beat0_is_data && beat0_is_last && pkt_last_beat) :
                                 (rx_fire && cur_is_mine  && cur_is_last  && pkt_last_beat);
    // unlock_pulse：resevation 用（看到 0x14）或 trigger 用（外部
    // unlock_req，來自 T_TRIG_GO）任一個觸發都解鎖。
    wire unlock_pulse = pkt0_complete_is_last || unlock_req;

    // ── 鎖定狀態 ─────────────────────────────────────────────────────────
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

    // ── TX side: relay 排空 + 本機發起，2-選-1 round-robin，餵給 data_tx1_* ──
    localparam TX_IDLE  = 2'd0;
    localparam TX_RELAY = 2'd1;
    localparam TX_LOCAL = 2'd2;

    reg [1:0] tx_state = TX_IDLE;
    reg [6:0] relay_rd_idx;
    reg       rr_favor_local;

    wire relay_is_last = (relay_rd_idx == pkt_len_r - 7'd1);
    assign drain_done  = (tx_state == TX_RELAY) && data_tx1_tready && relay_is_last;

    // local origination 只允許在「沒有鎖定」時開始一個新封包；已經開始的
    // 封包讓它送完，不強行中斷（`lock_req` 只擋「開始新的」，不擋「正在送
    // 的」，這是稍早設計就講好的：暫停不需要等目前在傳的東西結束）
    wire local_can_start = local_tx_tvalid && !locked_r;

    always @(posedge aurora_clk) begin
        if (rst) begin
            tx_state       <= TX_IDLE;
            relay_rd_idx   <= 7'd0;
            rr_favor_local <= 1'b0;
        end else begin
            case (tx_state)
                TX_IDLE: begin
                    relay_rd_idx <= 7'd0;
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
                TX_LOCAL: if (data_tx1_tready && local_tx_tvalid && local_tx_tlast) begin
                    tx_state <= TX_IDLE;
                end
                default: tx_state <= TX_IDLE;
            endcase
        end
    end

    assign data_tx1_tdata  = (tx_state == TX_RELAY) ? pkt_buf[relay_rd_idx] :
                             (tx_state == TX_LOCAL) ? local_tx_tdata        : 64'd0;
    assign data_tx1_tvalid = (tx_state == TX_RELAY) ? 1'b1 :
                             (tx_state == TX_LOCAL) ? local_tx_tvalid       : 1'b0;
    assign data_tx1_tlast  = (tx_state == TX_RELAY) ? relay_is_last :
                             (tx_state == TX_LOCAL) ? local_tx_tlast       : 1'b0;
    assign local_tx_tready = (tx_state == TX_LOCAL) && data_tx1_tready;

    // is_busy：本機資料目前是否正在用 tx1（不算 relay，見檔頭說明）
    assign is_busy = (tx_state == TX_LOCAL);

endmodule
`default_nettype wire
