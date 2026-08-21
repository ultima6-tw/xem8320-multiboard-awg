`timescale 1ns/1ps
`default_nettype none

// aurora_tx1_arbiter.v -- Step 15a: 合併 Layer 2（data）跟 Layer 3（控制）
// 的 tx1 請求，變成真正接 aurora_64b66b_1/s_axi_tx_* 的單一輸出
//
// 2026-07-05 首版草稿（DRAFT，尚未上板驗證，已用 iverilog 模擬）。
//
// 2026-07-27 擴充（統一讀取/寫入架構，見 PROJECT.md「統一讀取/寫入
// 架構 — 完整規格」小節）：新增第三個來源 `reply_tx1_*`，接
// `aurora_reply_tx.v` 組好的 `T_STATUS_REPORT` 封包（經 create_bd.tcl
// 的 xpm_fifo_async 從 sys_clk 跨到這裡的 aurora_clk domain 之後）。
// 三路仲裁規則（沿用原本「只在 chunk 邊界插隊」的精神，一樣不會中斷
// 正在傳輸中的 chunk）：**ctrl > reply > data**——ctrl 是多板同步時序
// 關鍵路徑（enum/trigger），維持最高優先權；reply 是查詢回覆，host
// 端在等，優先於一般 waveform bulk 資料；data（waveform stream）bulk
// 傳輸，最低優先權，一旦選定一樣讓它送完整個封包才回 IDLE。
//
// 這個模組沒有 tready 往回給上層 Layer2/Layer3/reply（假設三者都能
// 承受被 backpressure，各自狀態機在等 tx1_tready 期間會停在原地，
// 不會漏資料，因為 store-and-forward 的資料在各自的 buffer 裡等）。
//
// 2026-07-28 第一版 watchdog（已知有 bug，這次修正）：曾經加過
// ST_CTRL/ST_REPLY 逾時強制回 ST_IDLE 的機制，但 Opus 覆查 + iverilog
// 實測抓到致命問題——ST_IDLE 沒有鎖定機制，逾時回 ST_IDLE 後，如果
// 卡住的來源（例如 reply_tx1_tvalid 因為 async_fifo_reply_tx 的
// reset race 持續卡在 1，見 NOTES.md「async_fifo_reply_tx reset
// race 用 xsim 真實 IP 重現」章節）下一拍還是 tvalid=1，會立刻又被
// 選回同一個狀態，watchdog 事件訊號會亮，但 data 依然一拍都拿不到。
// 這次修正見下方 WATCHDOG_LIMIT/lockout 說明。

module aurora_tx1_arbiter (
    input  wire        clk,
    input  wire        rst,

    input  wire [63:0] data_tx1_tdata,
    input  wire        data_tx1_tvalid,
    input  wire        data_tx1_tlast,
    output wire        data_tx1_tready,

    input  wire [63:0] ctrl_tx1_tdata,
    input  wire        ctrl_tx1_tvalid,
    input  wire        ctrl_tx1_tlast,
    output wire        ctrl_tx1_tready,

    // 2026-07-27 新增：T_STATUS_REPORT 回覆封包（aurora_clk domain，
    // 經 xpm_fifo_async 從 aurora_reply_tx.v 的 sys_clk 輸出跨域而來）
    input  wire [63:0] reply_tx1_tdata,
    input  wire        reply_tx1_tvalid,
    input  wire        reply_tx1_tlast,
    output wire        reply_tx1_tready,

    output wire [63:0] tx1_tdata,
    output wire        tx1_tvalid,
    output wire        tx1_tlast,
    input  wire        tx1_tready,

    // 2026-07-28 新增：內部仲裁狀態原本完全沒有導出，ILA 看不到「卡在
    // 哪個狀態」（懷疑三路仲裁改版後 ctrl/reply 其中一路卡住不放，會
    // 讓 data 永遠輪不到，見 PROJECT.md/NOTES.md 2026-07-28 T_EXT_CLK_
    // SEL 兩跳 relay 失敗排查記錄）。純組合邏輯鏡像 state，不影響原本
    // 的仲裁邏輯。
    output wire [1:0]  dbg_state,

    // 2026-07-28 新增（Opus 覆查後改版）：ctrl/reply watchdog 逾時
    // 次數，飽和計數（8'hFF 封頂，不 wrap），不是單一 sticky bit——
    // 只知道「發生過」不夠，需要知道「幾次」才能判斷是偶發還是持續
    // 發生。正常運作時這兩個訊號應該永遠是 0。
    output reg  [7:0]   dbg_ctrl_watchdog_count  = 8'd0,
    output reg  [7:0]   dbg_reply_watchdog_count = 8'd0,

    // 2026-07-29 新增：`T_QUERY` 上機持續失敗，需要把「寫入端（reply
    // 有沒有真的被送到這裡、有沒有真的被這裡送上 tx1）」也弄成不需要
    // ILA、單純用 USB 讀 WO 就能確認的診斷（見 PROJECT.md/NOTES.md
    // 同日「reply 寫入端 USB 可讀診斷」章節）。sticky，rst 後第一次
    // 出現就永久保持 1，直到下次 rst（跟既有 `dbg_ctrl/reply_
    // watchdog_count` reset 慣例一致）。create_bd.tcl 用
    // `xpm_cdc_single` 跨到 sys_clk，接進 `concat_reserve_diag`
    // （WO 0x2d）目前空出來的 bit。
    output reg          dbg_reply_seen    = 1'b0,  // reply_tx1_tvalid 曾經為 1（aurora_reply_tx.v 組好的 reply 有被送到這個 arbiter 面前）
    output reg          dbg_reply_granted = 1'b0   // reply_tx1_tready && reply_tx1_tvalid 曾經同時為 1（reply 第一拍真的被這個 arbiter 接受、送上 tx1）
);

    localparam ST_IDLE  = 2'd0;
    localparam ST_DATA  = 2'd1;
    localparam ST_CTRL  = 2'd2;
    localparam ST_REPLY = 2'd3;

    reg [1:0] state = ST_IDLE;

    // 2026-07-28 修正版 watchdog：
    //
    // 1. **lockout 機制**（修正第一版的致命 bug）：ST_CTRL/ST_REPLY
    //    逾時時，除了回 ST_IDLE，還設對應的 lockout bit——ST_IDLE
    //    選擇下一個狀態時，鎖定中的來源直接跳過（即使 tvalid 還卡著
    //    1），確保 data 至少能拿到一次機會。lockout 在以下任一情況
    //    清除：①該來源自己正常送完一個封包（ctrl_done/reply_done）；
    //    ②ST_IDLE 當下三個來源都沒有東西要送（代表狀況可能已經恢復，
    //    給它一個重新嘗試的機會，不永久鎖死）；③data 送完一整個封包
    //    （讓 ctrl/reply 至少每次 data 完成後都能再嘗試一次，避免
    //    data 又反過來永久獨佔）。
    //
    // 2. **watchdog 判據改成「連續 N 拍沒有任何進展」，不是「總共
    //    經過 N 拍」**：只要這一拍有正常接受一個 beat（tready &&
    //    tvalid），就归零重算——這樣一個合法但被上游間歇 backpressure
    //    拖慢的 reply 封包不會被誤判，只有「真的完全沒有任何 beat
    //    被接受」持續 N 拍才會觸發。
    //
    // 3. **逾時值改小到 12-bit（4095 cycle ≈ 26us @156.25MHz）**：
    //    ctrl 固定 2 beat、reply 最多 REPLY_MAX_WORDS=30+header≈32
    //    beat，就算逐拍都被 backpressure 卡一下，4095 cycle 也綽綽
    //    有餘；比第一版的 16-bit/419us 好上 16 倍，運氣好的話能落進
    //    aurora_ila_0 的擷取視窗（C_DATA_DEPTH=2048 @156.25MHz≈13us），
    //    也讓 256 深的 relay_fifo_0 在等待期間 overflow 的機率大幅
    //    降低。
    //
    // 4. **逾時強制回 IDLE 時，補送一拍終結 beat（poison beat）**，
    //    不能只是把 tvalid 收掉——不然 Aurora 硬體 frame 沒有正常
    //    結束，下一個真正的封包會被接續在同一個 frame 裡。poison beat
    //    的內容是 pkt_len=0/type=0x00/src_id=0xFFFF/dest_id=0x0000，
    //    同時符合兩個下游既有的安全機制：①`dispatcher.v` 把
    //    pkt_len==0 當成 padding 直接丟棄（既有慣例，見該檔案
    //    idle_is_padding 說明）；②`aurora_data_channel_relayfifo.v`/
    //    `aurora_ctrl_channel.v` 的 src_id 合理性檢查會判定
    //    src_id=0xFFFF 不合理（不會等於任何真正的 board_id），直接
    //    當噪音丟棄，不會被 relay 或本地送達。兩層防護都會忽略這個
    //    poison beat，不需要新增下游邏輯。
    localparam [11:0] WATCHDOG_LIMIT = 12'hFFF;
    localparam [63:0] POISON_TDATA   = 64'h0000_0000_FFFF_0000;

    reg [11:0] ctrl_watchdog_cnt;
    reg [11:0] reply_watchdog_cnt;
    reg        ctrl_lockout;
    reg        reply_lockout;
    reg        abort_ctrl_pending;
    reg        abort_reply_pending;

    wire ctrl_progress  = tx1_tready && ctrl_tx1_tvalid;
    wire reply_progress = tx1_tready && reply_tx1_tvalid;
    wire ctrl_done   = ctrl_progress  && ctrl_tx1_tlast;
    wire reply_done  = reply_progress && reply_tx1_tlast;
    wire ctrl_watchdog_hit  = (ctrl_watchdog_cnt  == WATCHDOG_LIMIT);
    wire reply_watchdog_hit = (reply_watchdog_cnt == WATCHDOG_LIMIT);

    wire abort_pending = abort_ctrl_pending || abort_reply_pending;

    always @(posedge clk) begin
        if (rst) begin
            state                    <= ST_IDLE;
            ctrl_watchdog_cnt        <= 12'd0;
            reply_watchdog_cnt       <= 12'd0;
            ctrl_lockout             <= 1'b0;
            reply_lockout            <= 1'b0;
            abort_ctrl_pending       <= 1'b0;
            abort_reply_pending      <= 1'b0;
            dbg_ctrl_watchdog_count  <= 8'd0;
            dbg_reply_watchdog_count <= 8'd0;
            dbg_reply_seen           <= 1'b0;
            dbg_reply_granted        <= 1'b0;
        end else begin
            // poison beat 送出優先權最高：只要還有一個 pending，先把
            // 它排空，不做任何其他狀態轉換（ST_IDLE 的轉換邏輯下面另外
            // 用 !abort_pending 擋住，避免同一拍又選新的來源）。
            if (abort_ctrl_pending && tx1_tready)  abort_ctrl_pending  <= 1'b0;
            if (abort_reply_pending && tx1_tready) abort_reply_pending <= 1'b0;

            // 2026-07-29 新增：sticky，跟 case (state) 的狀態轉換無關，
            // 每一拍都檢查，只會被設成 1、不會被清掉（除了 rst）。
            if (reply_tx1_tvalid)                    dbg_reply_seen    <= 1'b1;
            if (reply_tx1_tready && reply_tx1_tvalid) dbg_reply_granted <= 1'b1;

            case (state)
                ST_IDLE: begin
                    ctrl_watchdog_cnt  <= 12'd0;
                    reply_watchdog_cnt <= 12'd0;
                    if (!abort_pending) begin
                        if (ctrl_tx1_tvalid && !ctrl_lockout) begin
                            state <= ST_CTRL;   // 控制最優先
                        end else if (reply_tx1_tvalid && !reply_lockout) begin
                            state <= ST_REPLY;  // 查詢回覆次之
                        end else if (data_tx1_tvalid) begin
                            state <= ST_DATA;
                        end else begin
                            // 三個來源都沒有東西要送——藉機解除鎖定，
                            // 避免鎖定狀態在沒有流量時無限期殘留。
                            ctrl_lockout  <= 1'b0;
                            reply_lockout <= 1'b0;
                        end
                    end
                end
                ST_DATA: if (tx1_tready && data_tx1_tvalid && data_tx1_tlast) begin
                    state         <= ST_IDLE;
                    // data 送完一整個封包，讓 ctrl/reply 至少每次都
                    // 有機會再嘗試一次，避免任何一路（含 data 自己）
                    // 反過來永久獨佔 tx1。
                    ctrl_lockout  <= 1'b0;
                    reply_lockout <= 1'b0;
                end
                ST_CTRL: begin
                    if (ctrl_done) begin
                        state        <= ST_IDLE;
                        ctrl_lockout <= 1'b0;
                    end else if (ctrl_watchdog_hit) begin
                        state              <= ST_IDLE;
                        ctrl_lockout       <= 1'b1;
                        abort_ctrl_pending <= 1'b1;
                        if (dbg_ctrl_watchdog_count != 8'hFF)
                            dbg_ctrl_watchdog_count <= dbg_ctrl_watchdog_count + 8'd1;
                    end else begin
                        ctrl_watchdog_cnt <= ctrl_watchdog_cnt + 12'd1;
                    end
                end
                ST_REPLY: begin
                    if (reply_done) begin
                        state         <= ST_IDLE;
                        reply_lockout <= 1'b0;
                    end else if (reply_watchdog_hit) begin
                        state               <= ST_IDLE;
                        reply_lockout       <= 1'b1;
                        abort_reply_pending <= 1'b1;
                        if (dbg_reply_watchdog_count != 8'hFF)
                            dbg_reply_watchdog_count <= dbg_reply_watchdog_count + 8'd1;
                    end else begin
                        reply_watchdog_cnt <= reply_watchdog_cnt + 12'd1;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

    // poison beat 優先權最高，其次才是正常三路仲裁——確保 watchdog
    // 觸發後一定能送出終結 beat，不會被下一輪的 ST_CTRL/ST_REPLY/
    // ST_DATA 選擇蓋過去（ST_IDLE 轉換邏輯本身也用 !abort_pending
    // 擋住了同一拍搶著選新狀態）。
    assign tx1_tdata  = abort_pending          ? POISON_TDATA    :
                        (state == ST_DATA)  ? data_tx1_tdata  :
                        (state == ST_CTRL)  ? ctrl_tx1_tdata  :
                        (state == ST_REPLY) ? reply_tx1_tdata : 64'd0;
    assign tx1_tvalid = abort_pending          ? 1'b1            :
                        (state == ST_DATA)  ? data_tx1_tvalid  :
                        (state == ST_CTRL)  ? ctrl_tx1_tvalid  :
                        (state == ST_REPLY) ? reply_tx1_tvalid : 1'b0;
    assign tx1_tlast  = abort_pending          ? 1'b1            :
                        (state == ST_DATA)  ? data_tx1_tlast  :
                        (state == ST_CTRL)  ? ctrl_tx1_tlast  :
                        (state == ST_REPLY) ? reply_tx1_tlast : 1'b0;

    // abort_pending 期間 state 已經是 ST_IDLE（watchdog 觸發那一拍
    // 就把 state 設回去了），三個 tready 天然都是 0，不會跟 poison
    // beat 同時誤收真正的 producer 資料。
    assign data_tx1_tready  = (state == ST_DATA)  && tx1_tready;
    assign ctrl_tx1_tready  = (state == ST_CTRL)  && tx1_tready;
    assign reply_tx1_tready = (state == ST_REPLY) && tx1_tready;

    assign dbg_state = state;

endmodule
`default_nettype wire
