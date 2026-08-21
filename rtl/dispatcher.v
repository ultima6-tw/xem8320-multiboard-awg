`timescale 1ns/1ps
`default_nettype none

// dispatcher v2 — Step 14 v9：移到 sys_clk domain
//
// 統一封包路由器（sys_clk domain, 64-bit）
// 取代：aurora_wave_tx.v（FP 輸入路徑） + aurora_packet_rx.v（Aurora RX 解包路徑）
//
// ── 封包格式（beat0）──────────────────────────────────────────────────────────
//   [63:40] = pkt_len（24-bit，總 beats 數含 beat0）
//   [39:32] = type（8-bit）
//   [31:16] = src_id（FP 來源時由本板 board_id 替換；Aurora 封包不動）
//   [15:0]  = dest_id（16-bit）
//
// ── 路由規則 ──────────────────────────────────────────────────────────────────
//   dest_id == board_id, type == 0x13  → DDR   (T_WAVEFORM_STREAM → ddr_writer)
//   dest_id == board_id, type == 0x16  → FLASH (T_FLASH_WRITE_DATA → flash CDC，
//                                         2026-07-09 新增，見 awg-test-step-16
//                                         PROJECT.md 第 25 節；2026-07-15 從
//                                         0x14 改成 0x16——原本的 0x14 跟
//                                         aurora_data_channel.v 的
//                                         TYPE_DATA_LAST 撞號，見該檔案
//                                         2026-07-15 章節說明）
//   dest_id == board_id,  其他 type   → Local (local_reg_handler)
//   dest_id == 0xFFFF（broadcast）    → Local + Aurora TX（同時送）
//   dest_id != board_id               → Aurora TX（relay 到下游）
//
// ── 優先權 ────────────────────────────────────────────────────────────────────
//   Aurora RX > FP（Aurora 為主要路徑，FP 為設定階段）
//
// ── tx_tlast ─────────────────────────────────────────────────────────────────
//   由 pkt_len 計數產生（Aurora IP 硬體需要），Local/DDR 輸出無 tlast
//
// ── Broadcast 處理 ────────────────────────────────────────────────────────────
//   同時寫入 Local + Aurora TX，等兩者都 ready 才消費 input beat（無資料遺失）

module dispatcher (
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF rx:fp:tx:lrh:ddr" *)
    input  wire        sys_clk,
    input  wire        sys_rst,

    // 本板 ID（dest 判斷 + FP 封包 src_id 替換）
    input  wire [15:0] board_id,

    // 2026-07-23 新增：au_trig_delay 全自動校準機制。master 的
    // aurora_ctrl_channel_0.per_hop_value/total_boards（經 level_cdc
    // 同步到 sys_clk）。只有在本板「原生發起」（src_fp=1，即 host 透過
    // FP 送出）的 T_BOARD_ID_ASSIGN（0x1E）封包，才會把 beat1 payload
    // 換成這兩個值再送出去；下游各板單純轉送收到的值，不會再次替換
    // （見下方 idle_fire / subst_beat1_pending 邏輯）。slave board 這
    // 兩個 port 恆為 0，但用不到（slave 從不會是 T_BOARD_ID_ASSIGN 的
    // src_fp 發起點）。
    //
    // 2026-07-23 新增 total_boards 的原因（模擬中發現的真實缺口，不是
    // 一開始就規劃好的）：aurora_ctrl_channel.v 的 total_boards 暫存器
    // 只有 master 自己在 ring-return 那一拍才會更新，slave 自己的
    // total_boards 恆為 0（歷史設計如此——過去唯一用到 total_boards
    // 的地方是 master 自己算 ack_count，slave 從不需要）。
    // local_reg_handler.v 算 hops=total_boards-1-board_index 需要
    // slave 也知道正確的 total_boards，所以連同 per_hop_value 一起
    // piggyback 在同一個 beat1 送出去，不新增封包/CDC。
    input  wire [31:0] per_hop_value,
    input  wire [4:0]  total_boards,

    // 2026-08-20 新增：aurora_64b66b_1/channel_up 直接接線（跟
    // aurora_ctrl_channel_0/channel_up_1 同一個 sys_clk domain 的既有訊號，
    // 不需要新 CDC）。tx_tdata/tx_tvalid/tx_tready 這組輸出最終接的是
    // aurora_64b66b_1（見 aurora_tx1_arbiter.v），所以只需要這一個，不用
    // channel_up_0。單板/無 SFP 迴路時這個訊號恆為 0——見下方
    // tx_can_accept／channel_up_1 gating 說明，解決「沒有迴路時廣播封包
    // 把 dispatcher 永久卡死」的問題（PROJECT.md「test1 面板」章節）。
    input  wire        channel_up_1,

    // Aurora RX 輸入（sys_clk，來自 aurora_64b66b_0 RX）
    input  wire [63:0] rx_tdata,
    input  wire        rx_tvalid,
    output wire        rx_tready,

    // FP 輸入（sys_clk，fp_input pair-acc + async FIFO 之後）
    input  wire [63:0] fp_tdata,
    input  wire        fp_tvalid,
    output wire        fp_tready,

    // Aurora TX 輸出（sys_clk，送往下游板）
    output reg  [63:0] tx_tdata,
    output reg         tx_tvalid,
    output reg         tx_tlast,
    input  wire        tx_tready,

    // Local reg 輸出（sys_clk，接 async FIFO → local_reg_handler sys_clk）
    output reg  [63:0] lrh_tdata,
    output reg         lrh_tvalid,
    input  wire        lrh_tready,

    // DDR 輸出（sys_clk，直接接 ddr_writer）
    output reg  [63:0] ddr_tdata,
    output reg         ddr_tvalid,
    input  wire        ddr_tready,

    // Flash payload 輸出（sys_clk，2026-07-09 新增，接 sys_clk->okClk CDC
    // 模組，再送進 fpga_flash_ctrl_0）
    output reg  [63:0] flash_tdata,
    output reg         flash_tvalid,
    input  wire        flash_tready,

    // ── 診斷輸出（sys_clk）───────────────────────────────────────────────
    output reg         diag_disp_fp_seen,      // sticky: fp_tvalid 在 IDLE 曾觸發
    output reg         diag_disp_local_seen,   // sticky: lrh_tvalid 曾 set（beat0 路由成功）
    output reg  [31:0] diag_disp_first_lo,     // 第一個 fp_tdata[31:0] 觸發時
    output reg  [31:0] diag_disp_first_hi,     // 第一個 fp_tdata[63:32] 觸發時

    // 2026-08-20 新增：sticky，tx_timeout 曾經發生過一次就設 1，只有
    // sys_rst 才清掉——給 QT_BOARD_INFO 用（見 aurora_reply_tx.v
    // bi_tx_timeout_seen），讓 master 可以透過既有 T_QUERY 機制逐一問
    // 每片板子「你自己往下一棒送的時候，link 有搭起來但曾經逾時過嗎」，
    // 不用等資料繞一圈回來才能推論。跟 channel_up_1（即時值，QT_BOARD_
    // INFO 本來就有）是互補的兩種診斷資訊，channel_up_1=0 時這裡不會
    // 被設 1（tx_timeout 定義上只在 channel_up_1=1 期間才計時，見上方
    // tx_timeout 說明），兩種原因不會混在一起。
    output reg         diag_tx_timeout_seen,

    // Step 14.3b ILA 診斷用：即時導出（非 sticky），供 system_ila 直接觀察
    output wire         diag_state,       // 目前 state（ST_IDLE=0 / ST_DATA=1）
    output wire [2:0]   diag_idle_route,  // idle_route 組合邏輯即時值（RT_LOCAL=0/RT_DDR=1/RT_TX=2/RT_BCAST=3/RT_FLASH=4）

    // Step 14.3b 診斷：free-running counter（sys_rst 才清零，不隨封包重置）
    output wire [15:0]  debug_disp_fwd_cnt   // idle_fire || data_fire 次數（從來源消費並轉發出去的 beat 數）
);

    // ── 路由常數（2026-07-09：從 2-bit 拓寬成 3-bit，新增 RT_FLASH）──────────
    localparam [2:0] RT_LOCAL = 3'd0;
    localparam [2:0] RT_DDR   = 3'd1;
    localparam [2:0] RT_TX    = 3'd2;
    localparam [2:0] RT_BCAST = 3'd3;   // local + tx 同時送
    localparam [2:0] RT_FLASH = 3'd4;

    // ── 狀態 ─────────────────────────────────────────────────────────────────
    localparam ST_IDLE = 1'b0;
    localparam ST_DATA = 1'b1;

    reg        state       = ST_IDLE;
    reg [23:0] remaining;   // 距離封包結束還剩幾個 beats
    reg [2:0]  route;       // 目前封包路由（beat0 鎖定後不變）
    reg        src_fp;      // 1 = 目前封包來自 FP（需替換 src_id）
    reg        discard_pkt; // 1 = 從 RX 收到自己發出的封包（已繞一圈），整包丟棄

    // 2026-07-23 新增：見上方 per_hop_value port 註解。beat0 鎖定時判斷
    // 「這是本板原生發起的 T_BOARD_ID_ASSIGN」，只對緊接著的第一個 ST_DATA
    // beat（beat1）生效，用完立即清掉（data_fire 時），避免萬一 pkt_len
    // 之後改大也只影響 beat1，不會誤替換 beat2 以後的內容。
    reg        subst_beat1_pending;

    // 2026-08-20 新增：tx_tready 逾時保護（見 PROJECT.md「test1 面板」章節
    // 的除錯過程——沒有 SFP 迴路時 tx_tready 永遠不會來，dispatcher 原本
    // 會永久卡在等待 tx_tready，連帶讓 idle_fire 再也無法發生，之後任何
    // 封包（包含跟 Aurora TX 完全無關的 T_QUERY）都進不去）。只在
    // channel_up_1=1（link 真的有訓練起來）期間才累計卡住的週期數——
    // channel_up_1=0 的情況由下面 tx_can_accept 的 !channel_up_1 那一項
    // 直接處理，不會走到這個計數器，兩種情況分開處理、原因不同：
    // channel_up_1=0 是「本來就沒有連線」，逾時是「有連線但下游沒回應」
    // （例如環路上其他板子沒反應、或既有殘留還沒清乾淨的 Aurora TX FIFO
    // backlog）。TX_TIMEOUT_CYCLES 抓 100,000 個 sys_clk cycle（100MHz
    // fabric clock，約 1ms），遠超過正常環路真實來回時間（NOTES.md 記錄
    // 是微秒等級），正常多板情境這個逾時不會被觸發，行為不變。
    localparam [19:0] TX_TIMEOUT_CYCLES = 20'd100_000;
    reg  [19:0] tx_stall_cnt;
    wire        tx_stalled = tx_tvalid && !tx_tready && channel_up_1;
    wire        tx_timeout = tx_stalled && (tx_stall_cnt >= TX_TIMEOUT_CYCLES);

    // ── 輸出空閒判斷（有效：空，或正在被下游消費）────────────────────────────
    // !channel_up_1：link 沒訓練起來，不等 tx_tready，視為隨時可以繼續
    //   （下面 case 分支寫入時另外用 channel_up_1 擋住，不會真的寫入 TX，
    //   純粹只是不讓狀態機卡住）。
    // tx_timeout：link 有訓練起來，但 tx_tready 遲遲不來，逾時後放棄這
    //   一拍，效果同上，讓狀態機能繼續往下走。
    wire tx_can_accept    = !tx_tvalid || tx_tready || !channel_up_1 || tx_timeout;
    wire local_can_accept = !lrh_tvalid || lrh_tready;
    wire ddr_can_accept   = !ddr_tvalid   || ddr_tready;
    wire flash_can_accept = !flash_tvalid || flash_tready;

    // ── ST_IDLE：解碼 beat0（combinatorial）──────────────────────────────────
    // Aurora RX 優先
    wire        idle_use_rx  = rx_tvalid;
    wire [63:0] idle_data    = idle_use_rx ? rx_tdata : fp_tdata;
    wire        idle_valid   = rx_tvalid || fp_tvalid;

    wire [23:0] idle_pkt_len = idle_data[63:40];
    wire [7:0]  idle_type    = idle_data[39:32];
    wire [15:0] idle_src_id  = idle_data[31:16];
    wire [15:0] idle_dest_id = idle_data[15:0];

    // 2026-07-06 bugfix：這個環型防護是舊架構遺留（dispatcher_0 以前直接接
    // 實體 Aurora RX，需要自己判斷「src_id==自己 board_id → 這是繞了一圈的
    // 舊 broadcast，丟棄」）。現在 rx_tdata 改接 aurora_rx_merge_0（Layer 2
    // 本地送達 + Layer 3 合成封包），ring-return 判斷已經在 Layer 2
    // （aurora_data_channel.v 的 catch-all 邏輯）做過一次，這裡是多餘、而且
    // 會誤判：Layer 3 合成的 TRIGGER 封包故意 src_id==dest_id==自己
    // board_id（自我定址），被這條防護誤認成「舊封包」整包丟棄，導致
    // trigger 完全無法送達 local_reg_handler_0（三板實測 diag_lrh_rx_seen
    // 恆為 0）。關閉這個防護，交給 Layer 2 負責。
    wire idle_ring_return = 1'b0;

    // 2026-07-10 bugfix（見 PROJECT.md 第 32 節）：BTPipeIn 為了滿足 USB
    // block size（host 端 BTPIPE_BLOCK=1024 bytes），會在真正的封包內容
    // 後面墊全零 padding。dispatcher 消耗完 pkt_len 指定的 beat 數後回到
    // ST_IDLE，會把下一個 padding beat（全零）誤判成新封包的 beat0，解出
    // pkt_len=0/dest_id=0，因為不等於 board_id 也不等於 0xFFFF，被路由到
    // RT_TX；單板測試沒有環路夥伴消耗 Aurora TX，只要曾經 fire 進去一次，
    // tx_tvalid 就會永久卡住，殃及後續所有 RT_TX/RT_BCAST 封包，連帶讓
    // fp_input 的 FIFO 永遠無法清空（backlog，卡住的舊項目擋在隊首）。
    // 真正的協定封包 pkt_len 最小是 1（header-only，如 T_QUERY），
    // pkt_len==0 只會出現在 padding 上，視為「非封包」直接丟棄、不路由到
    // 任何下游，適用所有封包類型（不限定 flash）。
    wire idle_is_padding = (idle_pkt_len == 24'd0);

    // 路由決策
    wire [2:0] idle_route;
    assign idle_route = (idle_dest_id == board_id && idle_type == 8'h13) ? RT_DDR   :
                        (idle_dest_id == board_id && idle_type == 8'h16) ? RT_FLASH :
                        (idle_dest_id == board_id)                        ? RT_LOCAL :
                        (idle_dest_id == 16'hFFFF)                        ? RT_BCAST :
                                                                            RT_TX;

    // Step 14.3b ILA 診斷：直接導出 state 跟 idle_route，供 system_ila 觀察
    assign diag_state       = state;
    assign diag_idle_route  = idle_route;

    // 輸出可接受 for idle_route（ring_return/padding 時直接 1：不需要下游 ready）
    wire idle_out_ok;
    assign idle_out_ok = (idle_ring_return || idle_is_padding) ? 1'b1 :
                         (idle_route == RT_LOCAL) ? local_can_accept :
                         (idle_route == RT_DDR)   ? ddr_can_accept   :
                         (idle_route == RT_FLASH) ? flash_can_accept :
                         (idle_route == RT_TX)    ? tx_can_accept    :
                         (local_can_accept && tx_can_accept);    // RT_BCAST

    // FP 封包 beat0：替換 src_id=0 → board_id；Aurora 封包不動
    wire [63:0] idle_beat0_out = idle_use_rx ? idle_data :
                                 {idle_data[63:32], board_id, idle_data[15:0]};

    // beat0 消費觸發
    wire idle_fire = (state == ST_IDLE) && idle_valid && idle_out_ok;

    // ── ST_DATA：轉發 beat1..N ──────────────────────────────────────────────
    wire [63:0] data_in    = src_fp ? fp_tdata  : rx_tdata;
    wire        data_valid = src_fp ? fp_tvalid : rx_tvalid;

    // 2026-07-23 新增：subst_beat1_pending 時，beat1 的實際內容換成
    // per_hop_value + total_boards（host 送的原始 beat1 內容不重要，
    // 硬體直接覆蓋），格式：{27'd0, total_boards, per_hop_value}——
    // total_boards 放在 per_hop_value 正上方，兩者都不會用滿 64-bit，
    // 不需要額外封包/CDC 就能讓 slave 也拿到正確的 total_boards（見上方
    // total_boards port 註解，模擬中才發現的缺口）。
    //
    // 2026-07-27 修正（統一讀取/寫入架構收尾時發現的真正 bug）：原本
    // 這裡直接用 27'd0 蓋掉 bits[63:37]，但同一次改版在 beat1[37] 新增
    // 了 is_master 欄位（見 local_reg_handler.v bid_is_master），兩者
    // 沒有互相考慮到——is_master 會被這個替換邏輯永遠清成 0，不管 host
    // 送什麼值，master 自己跟被轉送的 slave 收到的都是 0。改成只清
    // bits[63:38]，保留 data_in[37]（host 原始送的 is_master 值）不被
    // 替換掉。
    wire [63:0] data_in_used = subst_beat1_pending ? {26'd0, data_in[37], total_boards, per_hop_value} : data_in;

    // discard_pkt 時直接 1：burn through remaining beats，不等下游 ready
    wire data_out_ok;
    assign data_out_ok = discard_pkt ? 1'b1 :
                         (route == RT_LOCAL) ? local_can_accept :
                         (route == RT_DDR)   ? ddr_can_accept   :
                         (route == RT_FLASH) ? flash_can_accept :
                         (route == RT_TX)    ? tx_can_accept    :
                         (local_can_accept && tx_can_accept);    // RT_BCAST

    wire data_fire    = (state == ST_DATA) && data_valid && data_out_ok;
    wire data_is_last = (remaining == 24'd1);

    // Step 14.3b 診斷 counter：dispatcher 從來源（fp/rx）消費並轉發出去的 beat 總數
    reg [15:0] disp_fwd_cnt = 16'd0;
    always @(posedge sys_clk) begin
        if (sys_rst) begin
            disp_fwd_cnt <= 16'd0;
        end else if (idle_fire || data_fire) begin
            disp_fwd_cnt <= disp_fwd_cnt + 16'd1;
        end
    end
    assign debug_disp_fwd_cnt = disp_fwd_cnt;

    // ── Source tready ────────────────────────────────────────────────────────
    // ST_IDLE：rx 優先；ST_DATA：只給鎖定的 source
    assign rx_tready = (state == ST_IDLE) ?  idle_use_rx && idle_out_ok :
                                            !src_fp      && data_out_ok;

    assign fp_tready = (state == ST_IDLE) ? !idle_use_rx && fp_tvalid && idle_out_ok :
                                             src_fp       && data_out_ok;

    // ── Sequential：狀態機 + 輸出暫存器 ──────────────────────────────────────
    always @(posedge sys_clk) begin
        if (sys_rst) begin
            state                <= ST_IDLE;
            remaining            <= 24'd0;
            route                <= RT_LOCAL;
            src_fp               <= 1'b0;
            discard_pkt          <= 1'b0;
            subst_beat1_pending  <= 1'b0;
            tx_tvalid            <= 1'b0;
            tx_tlast             <= 1'b0;
            tx_stall_cnt         <= 20'd0;
            lrh_tvalid         <= 1'b0;
            ddr_tvalid           <= 1'b0;
            flash_tvalid         <= 1'b0;
            diag_disp_fp_seen    <= 1'b0;
            diag_disp_local_seen <= 1'b0;
            diag_disp_first_lo   <= 32'd0;
            diag_disp_first_hi   <= 32'd0;
            diag_tx_timeout_seen <= 1'b0;
        end else begin

            // tx_stall_cnt：只在真的卡住（tx_stalled）時累計，其餘情況
            // （沒有 tvalid、剛被接受、或 channel_up_1=0 走另一條路）歸零。
            tx_stall_cnt <= tx_stalled ? (tx_timeout ? tx_stall_cnt : tx_stall_cnt + 20'd1) : 20'd0;

            if (tx_timeout) diag_tx_timeout_seen <= 1'b1;  // sticky，只有 sys_rst 才清

            // ── 清除已被下游消費的輸出 ──────────────────────────────────────
            // （若同一 cycle 有新資料，idle_fire/data_fire 的 assign 會覆蓋）
            if (tx_tvalid    && tx_tready)    begin tx_tvalid <= 1'b0; tx_tlast <= 1'b0; end
            else if (tx_tvalid && tx_timeout) begin tx_tvalid <= 1'b0; tx_tlast <= 1'b0; end  // 逾時放棄這一拍，見上方 tx_timeout 說明
            if (lrh_tvalid && lrh_tready) lrh_tvalid <= 1'b0;
            if (ddr_tvalid   && ddr_tready)   ddr_tvalid   <= 1'b0;
            if (flash_tvalid && flash_tready) flash_tvalid <= 1'b0;

            // ── ST_IDLE：處理 beat0 ─────────────────────────────────────────
            if (idle_fire) begin
                route       <= idle_route;
                src_fp      <= !idle_use_rx;
                discard_pkt <= idle_ring_return;
                subst_beat1_pending <= !idle_use_rx && (idle_type == 8'h1E) && (idle_pkt_len > 24'd1);
                // 診斷：記錄第一次 FP 觸發
                if (!idle_use_rx && fp_tvalid && !diag_disp_fp_seen) begin
                    diag_disp_fp_seen  <= 1'b1;
                    diag_disp_first_lo <= fp_tdata[31:0];
                    diag_disp_first_hi <= fp_tdata[63:32];
                end

                // ring_return / padding：整包丟棄，不輸出到任何下游
                if (!idle_ring_return && !idle_is_padding) begin
                    case (idle_route)
                        RT_LOCAL: begin
                            lrh_tdata          <= idle_beat0_out;
                            lrh_tvalid         <= 1'b1;
                            diag_disp_local_seen <= 1'b1;
                        end
                        RT_DDR: begin
                            ddr_tdata  <= idle_beat0_out;
                            ddr_tvalid <= 1'b1;
                        end
                        RT_FLASH: begin
                            flash_tdata  <= idle_beat0_out;
                            flash_tvalid <= 1'b1;
                        end
                        RT_TX: begin
                            // 2026-08-20：channel_up_1=0 時完全不嘗試寫入
                            // TX（沒有路可以送到，寫了也只是塞爆 Aurora TX
                            // FIFO，之後才卡住），直接靜默丟棄這個 beat。
                            if (channel_up_1) begin
                                tx_tdata  <= idle_beat0_out;
                                tx_tvalid <= 1'b1;
                                tx_tlast  <= (idle_pkt_len == 24'd1);
                            end
                        end
                        RT_BCAST: begin
                            lrh_tdata          <= idle_beat0_out;
                            lrh_tvalid         <= 1'b1;
                            diag_disp_local_seen <= 1'b1;
                            // 同上 RT_TX 說明：channel_up_1=0 時 TX 那份不
                            // 寫，Local 那份照常送達（等同純本機廣播）。
                            if (channel_up_1) begin
                                tx_tdata             <= idle_beat0_out;
                                tx_tvalid            <= 1'b1;
                                tx_tlast             <= (idle_pkt_len == 24'd1);
                            end
                        end
                    endcase
                end

                if (idle_pkt_len > 24'd1) begin
                    remaining <= idle_pkt_len - 24'd1;
                    state     <= ST_DATA;
                end
                // pkt_len == 1（如 T_QUERY）→ 留在 ST_IDLE
            end

            // ── ST_DATA：轉發 beat1..N ─────────────────────────────────────
            if (data_fire) begin
                remaining <= remaining - 24'd1;
                subst_beat1_pending <= 1'b0;   // 只替換第一個 ST_DATA beat

                // discard_pkt：burn through beats，不輸出
                if (!discard_pkt) begin
                    case (route)
                        RT_LOCAL: begin
                            lrh_tdata  <= data_in_used;
                            lrh_tvalid <= 1'b1;
                        end
                        RT_DDR: begin
                            ddr_tdata  <= data_in_used;
                            ddr_tvalid <= 1'b1;
                        end
                        RT_FLASH: begin
                            flash_tdata  <= data_in_used;
                            flash_tvalid <= 1'b1;
                        end
                        RT_TX: begin
                            // 同上 ST_IDLE 的 RT_TX 說明：channel_up_1=0
                            // 時不寫入，靜默丟棄這個 beat。
                            if (channel_up_1) begin
                                tx_tdata  <= data_in_used;
                                tx_tvalid <= 1'b1;
                                tx_tlast  <= data_is_last;
                            end
                        end
                        RT_BCAST: begin
                            lrh_tdata  <= data_in_used;
                            lrh_tvalid <= 1'b1;
                            if (channel_up_1) begin
                                tx_tdata     <= data_in_used;
                                tx_tvalid    <= 1'b1;
                                tx_tlast     <= data_is_last;
                            end
                        end
                    endcase
                end

                if (data_is_last) begin
                    state       <= ST_IDLE;
                    discard_pkt <= 1'b0;
                end
            end

        end
    end

endmodule
`default_nettype wire
