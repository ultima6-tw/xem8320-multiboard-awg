`timescale 1ns/1ps
`default_nettype none

// aurora_ctrl_channel.v -- Step 15a Layer 3: control channel
// (開機自動編號 + trigger 多板同步；data 路徑預約留待下一輪)
//
// 2026-07-05 首版草稿（DRAFT，尚未模擬）。
//
// 跟 Layer 1/2 平行看同一份 rx0/rx1，只認 type 0x20-0x24（開機編號/
// trigger 用的 5 種），其他 type（0x13/0x14 data，未來的 0x25/0x26
// reserve）不理會。
//
// **方向慣例**（跟 PROJECT.md 設計文件 v2 一致）：
//   forward（rx0 -> tx1）：T_ENUM_COUNT 轉送、T_ENUM_TOTAL/T_TRIG_PAUSE_REQ/
//     T_TRIG_GO 這些 broadcast 的轉送
//   backward（rx1 -> tx0）：T_TRIG_PAUSE_ACK 的反向 retrace 轉送
//
// **這個模組完全在 aurora_clk domain 運作，不進 sys_clk**。下面幾個輸出
// 需要跨到 sys_clk/dac_clk domain，跨域交給 BD 層接官方 XPM 巨集（不在
// 這個模組內做）：init_ok/total_boards/board_index(xpm_cdc_single)。
// init_start_pulse/trig_start_pulse 也假設外部已經用 xpm_cdc_pulse 從
// sys_clk 跨過來，這裡當 aurora_clk domain 的單週期 pulse 直接用。
//
// **2026-07-05 trigger 改走 dispatcher 既有管線（取代原本獨立的
// trigger_pulse 硬體訊號）**：使用者要求「不管 trigger 是本機發起還是
// 收到別人的，都要走同一條路徑」，所以收到/決定要觸發 T_TRIG_GO 時，不
// 是拉一條旁路訊號線，而是**合成一個真正的 TRIGGER(0x01) 封包**（跟
// local_reg_handler.v 認的既有格式一樣），從 `local_inject_*` 送出去，
// 跟 Layer 2 的 `local_tdata`（本地送達）在 BD 層經一個小合併器
// （`aurora_rx_merge.v`）合流，再進 `async_fifo_aurora_rx` -> dispatcher_0
// -> local_reg_handler_0 -> 既有的 trigger_out/CDC -> wctrl，這樣不管
// trigger 是哪裡來的都走同一段管線、延遲特性一致，也不需要另外接
// xpm_cdc_pulse。
//
// **封包格式**（beat0，跟 dispatcher.v 一致）：
//   [63:40] pkt_len  [39:32] type  [31:16] src_id  [15:0] dest_id
// 全部固定 2-beat（`pkt_len=2`，1-beat 封包會被既有 aurora_tx_arbiter.v
// 的 relay capture 邏輯漏接）。
//
// 2026-07-29 新增：FRX_IDLE/BRX_IDLE 進入條件加上 channel_up_0/
// channel_up_1 閘控（使用者提出的假設：開機時連線還沒真正建立前，RX
// 匯流排可能有殘留/不確定內容，我們的 RTL 原本從 reset 解除那一刻就
// 無條件解析 rx0_tvalid/rx1_tvalid，完全沒有先確認連線建立）。查證
// 官方 Aurora 64B/66B Product Guide（PG074）確認：文件並未保證
// `m_axi_rx_tvalid` 在 `channel_up=0` 期間一定維持低電位，這件事留給
// 使用邏輯自己判斷，不是 IP 內建保證——加這層閘控是補齊文件沒承諾的
// 缺口，不是重造 IP 已有的功能。見 NOTES.md 同日對應章節。

// 2026-07-05 再簡化：改成從 0 開始遞增（不是從固定初始值遞減）。
// Master 送 0 出去，每一站收到就 +1，這個加完的值同時就是自己的
// board_index（不用再額外算），直接往下送；master 繞一圈收到最終值，
// total_boards = 最終值 + 1（+1 是加回 master 自己這一片）。完全不需要
// 知道/傳遞任何初始值常數。計數器寬度 4-bit（0-15），最多支援 16 片板子。

module aurora_ctrl_channel (
    input  wire        aurora_clk,
    input  wire        rst,
    input  wire [15:0] board_id,
    input  wire        is_master,

    input  wire        channel_up_0,
    input  wire        channel_up_1,

    input  wire        init_start_pulse,
    input  wire        trig_start_pulse,

    // ── data 路徑預約（host 觸發，只有 master 會用；2026-07-05 新增）───
    // host 已經知道要送給哪個 board（本來就是 host 決定要送資料過去的），
    // 所以直接由 host 提供 dest_id，不需要 ctrl_channel 反過來偷看 Layer 2
    // 排隊中的資料去猜目的地。
    input  wire [15:0] reserve_dest_id,
    input  wire        reserve_start_pulse,
    output reg         reserve_done = 1'b0,   // pulse: 這次請求已經有結果了（成功或忙碌/逾時）
    output reg         reserve_ok   = 1'b0,   // level: 上一次請求的結果
    // level: 這次請求還在等結果（2026-07-05 補上，讓 host 分得清 reserve_ok
    // 是這次的結果還是上一次留下的舊值——reserve_ok 只在 reserve_done 那一拍
    // 才更新，中間這段等待期間讀到的是舊值，需要靠這個訊號分辨）
    output wire        reserve_busy,

    // ── enum 斷點定位（host 觸發，只有 master 會用；2026-08-20 新增）───
    // 跟上面 reserve_* 同一種介面樣式（host 給 dest_id，模組跑一輪
    // forward REQ + backward ACK，結果用 done/ok pulse+level 回報），
    // 但這裡查的是「dest_id 那片板子自己的連線狀態」，不是資料層預約。
    // **注意（跟 reserve_start_pulse 不同）**：這裡的 host 端呼叫必須
    // 保證 !diag_busy 才能發下一次請求（Opus 覆核抓到 reserve_start_
    // pulse 目前沒有這層 guard，同一時間發兩次會讓 AXI4-Stream tdata
    // 在 tvalid 高電位時被改變，是協定違規——這裡用 diag_busy 明確擋
    // 掉，不重蹈覆轍，但這代表 host 端呼叫這個功能必須是「一次一個、
    // 等這次 diag_done 才發下一個」的序列式呼叫，不能並發）。
    input  wire [15:0] diag_dest_id,
    input  wire        diag_start_pulse,
    output reg         diag_done = 1'b0,   // pulse：這次請求已經有結果了
    output reg         diag_ok   = 1'b0,   // level：有收到回覆（不是逾時/查不到）
    output reg         diag_r_channel_up_0  = 1'b0,  // 被查板子自己的 channel_up_0
    output reg         diag_r_channel_up_1  = 1'b0,  // 被查板子自己的 channel_up_1
    output reg         diag_r_relay_blocked = 1'b0,  // 中繼站轉不出去、原地回報（不是 target 本身）
    output wire        diag_busy,

    // ── forward 方向（rx0 進、tx1 出）───────────────────────────────────
    // 2026-07-05 補上 ctrl_tx1_tready：原本假設「無 backpressure」（沿用
    // 舊版 relay_* 的慣例），但這次要接的新仲裁器（aurora_tx1_arbiter.v）
    // 需要真正的 tready 回壓，不能假設來多少都收，否則遇到實體 TX 忙碌時
    // 資料會憑空消失（狀態機以為送完了，實際上沒有）。
    // 2026-07-10：mark_debug 保留 rx0/rx1_tdata 整條匯流排的 bus 分組資訊
    // 給 debug core insertion 用（Chipscope 16-308 "missing CONN_BUS_INFO"
    // 警告——這條路徑是 aurora_64b66b IP 的 raw tap，沒有 tready，本來就
    // 不構成 AXI4-Stream interface，connect_bd_intf_net 用不上，改用官方
    // 建議的 mark_debug/KEEP 屬性防止 synthesis 把 bus 打散、遺失標籤）。
    // 2026-07-30：rx0_tvalid 語意改變——不再直接接實體 GT 的 tvalid，
    // 改接 aurora_data_channel_relayfifo.v 的 rx0_valid_for_ctrl（見該
    // 檔案 port 註解）。根因：這個模組的 FRX 狀態機完全沒有 framing，
    // 不知道現在是不是卡在 Layer2 自己的長封包（T_STATUS_REPORT/
    // T_WAVEFORM_STREAM）中間，只要某個 payload word 剛好湊出這裡認得
    // 的 type+合理 src/dest，就會被誤判成合法 beat0（2026-07-29/30
    // 在 board B 上機發現的離奇 ENUM_COUNT 就是這個機制）。改接 Layer2 的
    // 過濾後訊號，只有 Layer2 判斷「這是新封包開頭且是 ctrl-owned 類型」
    // 或「這是剛才那個 ctrl 封包的第二拍」時才是 1，其餘時間（尤其是
    // Layer2 自己在追蹤的長封包中間）Layer3 完全看不到那些拍，不會再
    // 誤判。這個模組內部的 FRX_IDLE/FRX_BEAT1 狀態機邏輯完全不用改，
    // 純粹是輸入來源換了。詳見 NOTES.md 2026-07-30 對應章節。
    (* mark_debug = "true" *) input  wire [63:0] rx0_tdata,
    (* mark_debug = "true" *) input  wire        rx0_tvalid,
    output wire [63:0] ctrl_tx1_tdata,
    output wire        ctrl_tx1_tvalid,
    output wire        ctrl_tx1_tlast,
    input  wire        ctrl_tx1_tready,

    // ── backward 方向（rx1 進、tx0 出）──────────────────────────────────
    (* mark_debug = "true" *) input  wire [63:0] rx1_tdata,
    (* mark_debug = "true" *) input  wire        rx1_tvalid,
    output wire [63:0] ctrl_tx0_tdata,
    output wire        ctrl_tx0_tvalid,
    output wire        ctrl_tx0_tlast,
    input  wire        ctrl_tx0_tready,

    // ── 狀態輸出（給 BD 層接 xpm_cdc_single 過 sys_clk）──────────────────
    output reg         init_ok      = 1'b0,
    // 5-bit：計數器本身是 4-bit（0-15），total_boards = 最終值+1 最大可達
    // 16，4-bit 裝不下（會 wrap），所以這裡要比計數器寬度多 1 bit
    output reg  [4:0]  total_boards = 5'd0,
    output reg  [4:0]  board_index  = 5'd0,

    // 2026-07-23 新增：au_trig_delay 自動校準用，master 量測 T_ENUM_COUNT
    // 繞完整圈（環路 RTT）所需的 aurora_clk cycle 數。只有 master 會更新
    // 非零值，slave 恆為 0。
    output reg  [31:0] ring_rtt_value = 32'd0,

    // 2026-07-23 新增：ring_rtt_value 鎖存後，master 硬體自動用簡單的逐次
    // 減法除法器算出 per_hop_value = ring_rtt_value / total_boards（完全
    // 自動，不需要 host 讀值計算——這是這次自動校準機制設計的核心要求）。
    // dispatcher.v 送出 T_BOARD_ID_ASSIGN 時會自動把這個值代填進封包的
    // beat1 payload，broadcast 給所有板子，每片收到後各自用自己已知的
    // board_index 算出 au_trig_delay 並直接寫入（見 local_reg_handler.v）。
    // 除法器只在 master 上跑；slave 的 total_boards/ring_rtt_value 恆為
    // 0，除法器也恆為 0，不會誤動作（下面除法邏輯有 total_boards==0 的
    // 保護，不會除以 0）。
    output reg  [31:0] per_hop_value = 32'd0,

    // 2026-07-23 再追加：使用者要求 trigger 的準確度是整個架構最優先的
    // 目標，追查發現原本「GO -> 合成 TRIGGER(0x01) 封包 -> 走 dispatcher/
    // local_reg_handler.v 通用管線 -> au_trig_delay 倒數（sys_clk）->
    // trig_ext_cdc_0 -> dac_clk」這條路徑要跨兩次時脈域（aurora_clk->
    // sys_clk 收封包、sys_clk->dac_clk 出給 DAC），而且 dac_clk 選外部
    // Si5332 時（正式多板同步的操作模式）跟 sys_clk（板內 Fabric 100MHz
    // 振盪器）是兩個完全獨立、沒有鎖相關係的時脈，每次 CDC 跨域的相位
    // 關係都會漂移，導致殘留相位差每次觸發都不一樣（實測同一組
    // au_trig_delay，兩次觸發量到 22°/6° 又變成 35°/16°，換算 ns 對不上
    // 純量化誤差能解釋的範圍）。
    //
    // 改法：GO 完成後不再合成封包繞回通用管線，直接在這裡（aurora_clk
    // domain）用 per_hop_value_in/total_boards_in（見下方 port 註解）
    // 乘上本板 board_index 算出的 hops，本地倒數，倒完直接跨一次到
    // dac_clk（BD 層新增一個 pulse CDC）。從兩次跨域減成一次，且不再需要
    // aurora_clk<->sys_clk 頻率換算（全程留在 aurora_clk 自己的單位）。
    //
    // 舊的合成封包注入機制（inject_pending/local_inject_*）刻意保留不動
    // ——現在沒有任何地方再把 inject_pending 設成 1，變成沒有作用的
    // 既有基礎設施，但保留可以縮小這次改動的影響範圍（不用同時牽動
    // aurora_rx_merge.v/create_bd.tcl 既有接線），之後有空再清。

    // per_hop_value（來自 T_BOARD_ID_ASSIGN beat1，master 自己是原生值、
    // slave 是收到封包後由 local_reg_handler.v 解出來、CDC 回這裡的值，
    // 見 local_reg_handler.v 的 bid_per_hop_value port 註解）
    input  wire [31:0] per_hop_value_in,
    // total_boards（同上，slave 的 total_boards 這個既有 output port 恆為
    // 0，不能拿來用，必須用這個從封包收到、CDC 回來的版本）
    input  wire [4:0]  total_boards_in,

    // 2026-07-24 新增：trigger 統一化架構改版——T_TRIG_DELAY_CFG(0x1D) 手動
    // 覆寫路徑，CDC 自 local_reg_handler_0（見該檔案 manual_delay_override_
    // active port 註解）。manual_trig_delay_in 單位是 aurora_clk cycle 數
    // （不做頻率轉換，維持這次改版全程留在 aurora_clk 單位的精神，2026-07-24
    // 已跟使用者確認）。manual_trig_delay_active_in 是 sticky，一旦手動設過
    // 值就永遠優先於自動算出的 native_trig_delay_w，直到 rst。
    input  wire [15:0] manual_trig_delay_in,
    input  wire        manual_trig_delay_active_in,

    // 2026-07-24 再追加：per_hop_value 手動覆寫（au_trig_delay 自動校準
    // 機制除錯用）。跟上面 manual_trig_delay_in（整個最終延遲值手動
    // 覆寫）不一樣——這裡只換掉「除法器算出來的 per_hop_value 這個因子
    // 本身」，hops 仍然用這片板子自己算出的 native_hops，藉此區分
    // 「per_hop_value 這個常數不準」跟「hops×per_hop_value 這段乘法/
    // 倒數邏輯本身有問題」（後者已用 sim/tb_ring_rtt.v 模擬驗證過是對
    // 的）。CDC 自 board_cfg_reg_0（每片板子各自 USB 手動設定，不經
    // Aurora broadcast，見 board_cfg_reg.v manual_per_hop_active port
    // 註解）。manual_per_hop_active_in 是 sticky，一旦手動設過值就永遠
    // 優先於 per_hop_value_in，直到 rst。
    input  wire [31:0] manual_per_hop_in,
    input  wire        manual_per_hop_active_in,

    // 2026-07-24 新增：單板 bench test 用（USB-only 功能盤點延伸需求）。
    // host 手動宣告「這個環路只有 N 片板子」，繞過 enum——`total_boards`
    // 這個 reg 只有 enum 真正繞完整圈才會被設定，需要實體 SFP 連線/
    // channel_up；單板沒有任何連線時 enum 永遠無法啟動（`channel_up_0`/
    // `channel_up_1` 永遠是 0），`total_boards` 永遠卡在 0，GO 判斷式
    // 永遠不成立。`manual_total_boards_in`（CDC 自 local_reg_handler_0/
    // au_manual_total_boards）非 0 時優先於這裡真正 enum 算出的
    // `total_boards`，見下方 `total_boards_eff` 組合邏輯。
    input  wire [4:0]  manual_total_boards_in,

    // 2026-07-27 新增（Group-based Trigger 架構）：T_TRIG_START(0x1B)
    // beat1[3:0]=group_select，CDC 自 local_reg_handler_0/au_trig_group_
    // select（sys_clk→aurora_clk，level_cdc，WIDTH=4，見 create_bd.tcl
    // au_trig_group_cdc_0）。只有 master 會真正收到 T_TRIG_START 封包，
    // 這個值只在本板是 master 時有意義；multi-board 情境下這個值會被
    // piggyback 進 TYPE_TRIG_GO 封包 beat1 傳給 slave（見下方
    // pending_trig_group_r/trig_fire_group 說明），slave 自己的這個
    // input 值不會被使用。
    input  wire [3:0]  au_trig_group_select_in,

    // 2026-07-30 改版：補償延遲倒數本身搬到 dac_clk domain 做（新模組
    // dac_trig_queue.v），原因見 NOTES.md 2026-07-30「sine wave mode
    // 精確度討論」章節——①短間隔連續 trigger 時，這裡原本的 native_
    // trig_cnt/native_trig_pending 沒有任何保護，新事件會直接蓋掉還
    // 沒 fire 的舊倒數；②倒數本身用 aurora_clk（每片板子各自獨立的
    // 板上振盪器）計時，理論上有 ppm 等級跨板誤差，改用 dac_clk（外部
    // Si5332，真正跨板共用）計時可以同時解決兩個問題。
    //
    // 這個模組不再自己倒數，只在「決定要 fire」的那一拍（跟原本
    // native_trig_cnt 被載入值的 3 個時機完全對應：單板 bypass 分支/
    // slave 收到 TYPE_TRIG_GO/master 送完 GO）送出一個 1-cycle pulse
    // + 這次的 group_select，交給 BD 層的 xpm_fifo_async（aurora_clk
    // 寫入側）安全跨到 dac_clk，深度 8 的佇列由 dac_trig_queue.v 在
    // dac_clk domain 處理（含 fire timestamp 計算、×16÷25 頻率換算），
    // 不會因為連續觸發把還沒 fire 的請求蓋掉。
    output reg         trig_fire_req   = 1'b0,
    output reg  [3:0]  trig_fire_group = 4'd0,

    // 2026-07-30 新增：這片板子的補償延遲量（native_hops×per_hop_
    // value_eff，aurora_clk cycle 單位，純組合邏輯，quasi-static——
    // 只有 enum/T_BOARD_ID_ASSIGN 之後才會變，不是每次 trigger 都
    // 重算）。BD 層用既有 level_cdc 慣例跨到 dac_clk，給 dac_trig_
    // queue_0 換算成 dac_clk cycle 數用。
    output wire [36:0] native_trig_delay_out,

    // 2026-07-24 再追加：per_hop_value_eff 讀回（見上方 manual_per_hop_in
    // port 註解）——這片板子目前真正拿去乘 native_hops 的值，不管來源是
    // 自動算的還是手動蓋的。純組合邏輯，經 BD 層 level_cdc 跨回 sys_clk
    // 給 host WO 讀回，讓「自動算出的值有沒有真的存入/生效」不用等 ILA
    // 就能直接確認。
    output wire [31:0] per_hop_value_eff,

    // ── 跟 Layer 2（data channel）的介面 ────────────────────────────────
    output reg         lock_req   = 1'b0,
    output reg         unlock_req = 1'b0,
    input  wire        is_busy,   // 2026-07-05 新增：本站是否正在發本機資料（RESERVE fail-fast 用）

    // ── 合成 TRIGGER(0x01) 封包注入（取代原本的 trigger_pulse 硬體訊號，
    //    2026-07-05 改走 dispatcher 既有管線）──────────────────────────────
    // 固定 2-beat，格式跟 local_reg_handler.v 認的既有 TRIGGER 封包一樣。
    // 沒有 tready 就不會被 BD 層的 aurora_rx_merge.v 合併器接受，這裡需要
    // 真正等 tready 才能前進（不像 fwd/bwd 那樣假設對方一定能收）。
    output wire [63:0] local_inject_tdata,
    output wire        local_inject_tvalid,
    input  wire        local_inject_tready,

    // ── 2026-07-06 debug：ILA 用，trigger 協定進度可見性（三板實測
    // ctrl_tx1_tvalid 完全沒動過，需要看 ack_count/go_sent 內部狀態才能
    // 定位卡在哪一步）───────────────────────────────────────────────────
    output wire [4:0]  diag_ack_count,
    output wire        diag_go_sent,

    // ── 2026-07-06 debug 追加：ack_count/go_sent 都確認正確，但
    // local_inject_tvalid 三板實測從頭到尾沒觸發過，兩份 iverilog 模擬
    // （單獨 ctrl_channel、含 Layer2+仲裁器）都證實邏輯本身沒問題，需要
    // 直接看 inject_pending 有沒有被設過、inj_state 有沒有真的離開 IDLE
    // 才能定位卡在哪 ──────────────────────────────────────────────────
    output wire        diag_inject_pending,
    output wire [1:0]  diag_inj_state,

    // ── 2026-07-08 debug 追加：T_RESERVE_START 封包路徑 15b 上機測試持續
    // 失敗（1-hop/2-hop 都是 reserve_ok=0，負向對照也是 reserve_ok=0，無法
    // 從 host 端分辨 reserve_start_pulse 到底有沒有真的送到這裡）。
    // reserve_start_pulse/reserve_dest_id/reserve_busy/reserve_ok/is_busy
    // 都已經是既有 top-level port，可以直接在 create_bd.tcl 接到 ILA，不用
    // 新增診斷 port；只有 fwd_pending（送出 REQ 的旗標）跟 reserve_dest_id_r
    // （鎖存後、state machine 實際使用的值）是內部 reg，需要額外導出 ──────
    output wire        diag_reserve_fwd_pending,
    output wire [15:0] diag_reserve_dest_id_r,

    // 2026-07-08 新增：純組合邏輯讀既有 reg，區分「逾時、完全沒收到回覆」
    // 跟「有收到明確的 RESERVE_ACK 回覆（不管內容是通過還是忙碌）」這兩種
    // 不同的失敗/成功來源──reserve_ok=0 目前沒辦法分辨這兩種情況。
    output wire        diag_reserve_timeout_hit,
    output wire        diag_reserve_reply_hit,

    // 2026-07-29 新增：beat0 合法性檢查的偵測 port（見 rtl/aurora_rx_
    // beat0_gate.v，經 u_f_beat0_gate/u_b_beat0_gate 兩個 instance）。
    // pulse：這一拍看到一個 type 欄位命中這個模組認得的 5 種封包之一，
    // 但 src_id/dest_id 不合理、或 channel_up 還沒建立，判定不能信任、
    // 直接擋下不處理。port 名稱沿用「src_rejected」歷史命名，**現在
    // 涵蓋 src_id/dest_id/channel_up 三種拒絕原因，不是只有 src_id**
    // （2026-07-29 同一天再擴充，見 aurora_rx_beat0_gate.v 檔頭說明）。
    // forward（rx0/beat0_is_ours 那組）跟 backward（rx1/PAUSE_ACK/
    // DATA_RESERVE_ACK 那組）分開兩個 port，三片板子各自的 aurora_
    // ctrl_channel_0 都有這兩個訊號，接上 ILA 後可以直接看出「垃圾
    // 封包是在哪一站被擋下來的」。
    output wire        dbg_rx_src_rejected,
    output wire        dbg_bwd_src_rejected
);

    localparam [7:0] TYPE_ENUM_COUNT     = 8'h20;
    // 0x21 T_ENUM_TOTAL 已取消（2026-07-05）：改成從 0 遞增後，board_index
    // 收到當下 +1 就直接是答案，不需要等額外的廣播；total_boards 也只有
    // master 自己需要，不用告訴其他板子。
    localparam [7:0] TYPE_TRIG_PAUSE_REQ    = 8'h22;
    localparam [7:0] TYPE_TRIG_PAUSE_ACK    = 8'h23;
    localparam [7:0] TYPE_TRIG_GO           = 8'h24;
    localparam [7:0] TYPE_DATA_RESERVE_REQ  = 8'h25;
    localparam [7:0] TYPE_DATA_RESERVE_ACK  = 8'h26;
    // 2026-08-20 新增：enum 失敗時定位斷點用（Opus 覆核過的設計，見
    // PROJECT.md「DIAG 定位斷點功能」章節）。跟 TYPE_DATA_RESERVE_REQ/
    // ACK 同一種 forward unicast REQ + backward unicast ACK 樣板，差異：
    //   - 沒有 is_busy fail-fast、不設 lock_req（純讀取，不是資料層預約）
    //   - **刻意不排除在 ring-return 之外**（跟 RESERVE 相反）：host 會
    //     主動查可能不存在的 dest_id（逐一走訪 1,2,3...），繞完整圈回到
    //     自己就是「這個 dest_id 不存在」的天然 TTL，RESERVE 目前沒有
    //     這個保護、對不存在的 dest_id 會在閉環上永遠繞圈（既有 bug，
    //     不在這次改動範圍內修）。
    //   - relay 前會檢查自己的 channel_up_1，轉不出去就地回 ACK（帶
    //     relay_blocked=1），不會靜默丟包，讓斷點定位更精確（不用等
    //     master 自己 timeout 才知道「查不到」，中繼站會主動回報）。
    // 0x27/0x28 已經是 T_REINIT/T_EXT_CLK_SEL（local_reg_handler.v 的
    // 封包空間），這裡改用 0x2F/0x30，跟 aurora_data_channel_relayfifo.v
    // 的 beat0_is_ctrl_owned 白名單同步。
    localparam [7:0] TYPE_DIAG_REQ          = 8'h2F;
    localparam [7:0] TYPE_DIAG_ACK          = 8'h30;
    // local_reg_handler.v 既有的 TRIGGER type（跟這個模組認的 0x20/0x22-
    // 0x26 是完全不同的既有格式，這裡只是「借用」這個 type 值合成封包）
    localparam [7:0] TYPE_TRIGGER_INJECT    = 8'h01;

    // ══════════════════════════════════════════════════════════════════
    //  Forward RX (rx0)：0x20/0x22/0x24，固定 2-beat
    // ══════════════════════════════════════════════════════════════════
    localparam FRX_IDLE  = 1'b0;
    localparam FRX_BEAT1 = 1'b1;

    reg        frx_state = FRX_IDLE;
    reg [7:0]  frx_type_r;
    reg [15:0] frx_src_id_r;
    reg [15:0] frx_dest_id_r;
    reg        frx_ring_return_r;   // 2026-07-05: 必須在 beat0 當下鎖存，
                                     // 不能在 beat1 用組合邏輯重算（beat1
                                     // 的 rx0_tdata 是 payload 不是 header，
                                     // Layer1/2 都踩過這個 bug）
    reg        frx_needs_relay_r;

    wire [7:0]  f_beat0_type    = rx0_tdata[39:32];
    wire [15:0] f_beat0_src_id  = rx0_tdata[31:16];
    wire [15:0] f_beat0_dest_id = rx0_tdata[15:0];

    wire f_beat0_is_ours = (f_beat0_type == TYPE_ENUM_COUNT) ||
                           (f_beat0_type == TYPE_TRIG_PAUSE_REQ) || (f_beat0_type == TYPE_TRIG_GO) ||
                           (f_beat0_type == TYPE_DATA_RESERVE_REQ) ||
                           (f_beat0_type == TYPE_DIAG_REQ);

    // 2026-07-29 改用集中化的 aurora_rx_beat0_gate.v（見該檔案檔頭
    // 說明）：src_id/dest_id/channel_up 合法性判斷不再各自重寫。dest_id
    // 合法性只看實際值本身（==0xFFFF 或落在合法板數範圍內），不看
    // beat0 type（修正：舊版靠呼叫端傳 per-type 白名單判斷是不是
    // broadcast，這份名單漏列 T_BOARD_ID_ASSIGN，導致它 broadcast 時
    // 被誤判成 unicast 而整包被丟棄，見 NOTES.md 2026-07-29「build9
    // 上機：發現一個新的、獨立的迴歸 bug」章節）。
    wire f_beat0_gate_ok;
    aurora_rx_beat0_gate u_f_beat0_gate (
        .channel_up             (channel_up_0),
        .total_boards_in        (total_boards_in),
        .rx_tvalid              (rx0_tvalid),
        .beat0_src_id           (f_beat0_src_id),
        .beat0_dest_id          (f_beat0_dest_id),
        .beat0_gate_ok          (f_beat0_gate_ok)
    );

    // 2026-07-29：src_id/dest_id 不合理、或 channel_up 還沒建立的封包
    // 一律不認（不是「我的」），frx_state 不會為它進入 FRX_BEAT1、
    // 不會被本地處理也不會被轉送——一次判斷同時擋住「這片板子自己被
    // 雜訊誤導」跟「雜訊被轉送出去」兩個問題，比照 relayfifo.v 的
    // beat0_is_mine 做法。port 名稱沿用 dbg_rx_src_rejected（歷史
    // 名稱），現在涵蓋 src_id/dest_id/channel_up 三種拒絕原因，不是
    // 只有 src_id。
    assign dbg_rx_src_rejected = rx0_tvalid && f_beat0_is_ours && !f_beat0_gate_ok;

    // 標準 broadcast 類（PAUSE_REQ/GO）繞回發起者（src_id==自己）就停止
    // 轉送；ENUM_COUNT/DATA_RESERVE_REQ 用 dest_id 到站即停（unicast，
    // 不是 broadcast），不需要這個判斷，但保留當作安全防呆（正常情況下
    // 不會觸發，因為都會在 dest_id 或 busy 那一站先停下）。
    wire f_beat0_ring_return =
        (f_beat0_type != TYPE_ENUM_COUNT) && (f_beat0_type != TYPE_DATA_RESERVE_REQ) &&
        (f_beat0_src_id == board_id);

    // 2026-07-05: T_DATA_RESERVE_REQ 不用這個通用公式算轉送與否——它的
    // relay 決定要看 is_busy（fail-fast），在 FRX_BEAT1 的 case 分支裡
    // 直接處理，不透過 frx_needs_relay_r。這裡列出來的值對它而言是
    // dead value（不會被讀取），只有 ENUM_COUNT/PAUSE_REQ/GO 會用到。
    wire f_beat0_needs_relay =
        f_beat0_ring_return ? 1'b0 :
        (f_beat0_type == TYPE_ENUM_COUNT) ? (f_beat0_dest_id != board_id) :
                                             1'b1;

    // ══════════════════════════════════════════════════════════════════
    //  Backward RX (rx1)：認 0x23（T_TRIG_PAUSE_ACK）跟 0x26
    //  （T_DATA_RESERVE_ACK）
    // ══════════════════════════════════════════════════════════════════
    localparam BRX_IDLE  = 1'b0;
    localparam BRX_BEAT1 = 1'b1;

    reg        brx_state = BRX_IDLE;
    reg [7:0]  brx_type_r;
    reg [15:0] brx_src_id_r;
    reg [15:0] brx_dest_id_r;

    wire [7:0]  b_beat0_type    = rx1_tdata[39:32];
    wire [15:0] b_beat0_src_id  = rx1_tdata[31:16];
    wire [15:0] b_beat0_dest_id = rx1_tdata[15:0];
    wire b_beat0_is_ours = (b_beat0_type == TYPE_TRIG_PAUSE_ACK) ||
                           (b_beat0_type == TYPE_DATA_RESERVE_ACK) ||
                           (b_beat0_type == TYPE_DIAG_ACK);

    // 2026-07-29 改用集中化的 aurora_rx_beat0_gate.v（見該檔案檔頭
    // 說明，跟 forward 方向 u_f_beat0_gate 同一套）：backward 方向的
    // 對稱防護——Opus 覆查時特別指出反向的 PAUSE_ACK/DATA_RESERVE_ACK
    // 誤判每一圈還會額外增生（BRX_BEAT1 case 的「不是給我的就繼續
    // 轉送」分支），不只 forward 方向需要擋。TRIG_PAUSE_ACK/DATA_
    // RESERVE_ACK 兩種都是 unicast（dest_id 回給原始 unicast 請求方，
    // 見下方 bwd_beat0 組裝處），dest_id 合法性判斷交給 gate 模組直接
    // 看值本身。
    wire b_beat0_gate_ok;
    aurora_rx_beat0_gate u_b_beat0_gate (
        .channel_up             (channel_up_1),
        .total_boards_in        (total_boards_in),
        .rx_tvalid              (rx1_tvalid),
        .beat0_src_id           (b_beat0_src_id),
        .beat0_dest_id          (b_beat0_dest_id),
        .beat0_gate_ok          (b_beat0_gate_ok)
    );
    assign dbg_bwd_src_rejected = rx1_tvalid && b_beat0_is_ours && !b_beat0_gate_ok;

    // ══════════════════════════════════════════════════════════════════
    //  TX 佇列（各 1-deep：forward 一份、backward 一份）
    // ══════════════════════════════════════════════════════════════════
    reg [63:0] fwd_beat0, fwd_beat1;
    reg        fwd_pending;
    reg [63:0] bwd_beat0, bwd_beat1;
    reg        bwd_pending;

    // ── ENUM 相關 ────────────────────────────────────────────────────────
    // ── TRIG 相關（master 用）────────────────────────────────────────────
    reg [9:0] ack_bitmask;
    reg [4:0] ack_count;
    reg       go_sent;   // sticky: 這輪 TRIG_GO 已經發過，避免重複發送
    assign    diag_ack_count = ack_count;
    assign    diag_go_sent   = go_sent;

    // 2026-07-24 新增：明確追蹤「這一輪多板協調是不是真的已經送出
    // PAUSE_REQ、正在等 ACK」。原本下面「收滿 ACK 發 GO」那個判斷式
    // 單純檢查 go_sent==0 && ack_count==total_boards-1 && !fwd_pending，
    // 隱含假設是「total_boards 只有 enum 真正跑完才會變成非 0，不會
    // 平白無故成立」——manual_total_boards_in 打破了這個假設：只要手動
    // 設成 1，配合 reset 後 go_sent/ack_count/fwd_pending 剛好都是
    // 「乾淨」的初始值，判斷式會不需要真的送過 trig_start_pulse 就自己
    // 成立，平白觸發一次 GO。這個旗標只在 trig_start_pulse 的多板分支
    // （見下方 else 分支）真正送出 PAUSE_REQ 時才會設成 1，GO 判斷式
    // 改成明確檢查這個旗標，不再依賴其他 reg 剛好是初始值這個巧合。
    reg       waiting_for_pause_acks;

    // 2026-07-24 新增：單板 bench test 用（見上方 manual_total_boards_in
    // port 註解）。manual_total_boards_in 非 0 時優先於真正 enum 算出的
    // total_boards。純組合邏輯，manual_total_boards_in 未設值（0）時
    // 完全不影響既有行為。
    wire [4:0] total_boards_eff = (manual_total_boards_in != 5'd0) ?
                                  manual_total_boards_in : total_boards;
    // 2026-07-05: master 自己的 trigger_pulse 不能在「決定要送 T_TRIG_GO」
    // 那一拍就馬上觸發（會比其他板子搶快——其他板子要等封包兩個 beat
    // 解析完才觸發）。改成等 T_TRIG_GO 真的送出去（ftx 狀態機送完最後一個
    // beat、ctrl_tx1_tready 完成握手）才觸發，這樣 master 的延遲至少對齊
    // 「自己排隊等發送」的時間，不是憑空猜一個延遲常數；環路實際傳輸/
    // SERDES 延遲仍然是既有已知、無法用設計消除的部分（PROJECT.md 已記錄
    // 待實測）。
    reg       trig_go_tx_pending;

    // 2026-07-23 新增：見上方 ring_rtt_value port 註解
    reg [31:0] ring_rtt_counter;
    reg        ring_rtt_running;

    // 2026-07-23 新增：per_hop_value 除法器（見上方 port 註解）。逐次減法
    // （每個 cycle 減一次 total_boards，減到不夠減為止），不是移位相減的
    // 二進位長除法——這個場合完全不趕時間（enum 完成到 host 真的觸發
    // T_BOARD_ID_ASSIGN 之間，USB 來回至少要幾百微秒，除法器跑幾千個
    // aurora_clk cycle 完全來得及），選最不容易寫錯的做法，不是最快的
    // 做法。total_boards==0（reset 剛過、enum 還沒跑過）時不啟動，避免
    // 除以 0。
    reg [31:0] div_remainder;
    reg [31:0] div_quotient;
    reg [4:0]  div_divisor;
    reg        div_running;

    // 2026-07-23 新增，2026-07-30 改版：這片板子的補償延遲量。hops/
    // native_trig_delay_w 是純組合邏輯（輸入都已經是穩定值：board_
    // index 是本板 enum 算好的既有 reg，per_hop_value_in/total_
    // boards_in 是外部 CDC 進來的準穩態值）。**2026-07-30 起這裡只
    // 算出延遲量本身，不再自己倒數**——實際倒數搬到 dac_clk domain
    // 的 dac_trig_queue.v，見上方 trig_fire_req/native_trig_delay_
    // out port 註解。
    wire [4:0]  native_hops = (total_boards_in >= (board_index + 5'd1)) ?
                              (total_boards_in - 5'd1 - board_index) : 5'd0;
    // 2026-07-24 再追加：per_hop_value_eff，見上方 manual_per_hop_in port
    // 註解——manual_per_hop_active_in 為 1 時用手動值取代自動算出的
    // per_hop_value_in，native_hops 不受影響（仍然是這片板子自己算出的
    // 真實 hops）。
    assign      per_hop_value_eff = manual_per_hop_active_in ? manual_per_hop_in : per_hop_value_in;
    wire [36:0] native_trig_delay_w = per_hop_value_eff * {32'd0, native_hops};
    // 2026-07-24 新增：手動覆寫值（T_TRIG_DELAY_CFG，aurora_clk cycle 單位）
    // sticky 優先於自動算出的 native_trig_delay_w，見上方 manual_trig_
    // delay_in/manual_trig_delay_active_in port 註解。
    wire [36:0] native_trig_delay_effective_w =
        manual_trig_delay_active_in ? {21'd0, manual_trig_delay_in} : native_trig_delay_w;
    assign native_trig_delay_out = native_trig_delay_effective_w;

    // 2026-07-27 新增（Group-based Trigger 架構）：
    // - pending_trig_group_r：multi-board 分支專用暫存——host 觸發
    //   trig_start_pulse 當下先存住這次的 group_select，因為要等 PAUSE_
    //   REQ/ACK 走完、真正組出 TYPE_TRIG_GO 封包時才會用到（可能是好幾拍
    //   之後），不能只信任那時候的 au_trig_group_select_in（host 理論上
    //   可能在等待期間又送了下一次不相關的 T_TRIG_START，那個值不該套用
    //   到這一輪還在進行中的觸發）。
    // - trig_fire_group：這片板子這次真正要 fire 的 group_select，跟
    //   trig_fire_req 同一拍生效（見上方 port 註解），3 個設定點：
    //   ①單板 bypass 分支（trig_start_pulse 當下直接用 au_trig_group_
    //   select_in，本板剛收到封包，值是新鮮的）②slave 收到 TYPE_TRIG_GO
    //   時從封包內容（rx0_tdata，而不是自己的 au_trig_group_select_in——
    //   slave 從沒收過 T_TRIG_START）解出來③master 自己送完 GO 後，用
    //   pending_trig_group_r（不是即時的 au_trig_group_select_in，理由
    //   同上）。
    reg  [3:0]  pending_trig_group_r;

    // ── 合成 TRIGGER(0x01) 封包注入（2026-07-05，取代原本的 trigger_pulse）──
    reg       inject_pending;   // 1 = 有一個合成封包排隊要送給 aurora_rx_merge.v
    localparam INJ_IDLE  = 2'd0;
    localparam INJ_BEAT0 = 2'd1;
    localparam INJ_BEAT1 = 2'd2;
    reg [1:0] inj_state;
    assign    diag_inject_pending = inject_pending;
    assign    diag_inj_state      = inj_state;

    // ── enum timeout（master 用）─────────────────────────────────────────
    reg        enum_wait;
    reg [15:0] enum_timeout_cnt;
    localparam [15:0] ENUM_TIMEOUT = 16'd8192;

    // ── data reservation（master 用，2026-07-05 新增）───────────────────
    reg        reserve_wait;         // 1 = 正在等 RESERVE_ACK 回來
    assign     reserve_busy = reserve_wait;
    reg [15:0] reserve_dest_id_r;    // 鎖存 host 給的目的地（跑的過程中 host 可能又改了輸入）
    reg [15:0] reserve_timeout_cnt;
    localparam [15:0] RESERVE_TIMEOUT = 16'd8192;   // 跟 enum 用同樣的保守值
    // 2026-07-08 debug（見上方 port 宣告處說明）
    assign     diag_reserve_fwd_pending = fwd_pending;
    assign     diag_reserve_dest_id_r   = reserve_dest_id_r;
    assign     diag_reserve_timeout_hit = reserve_wait && (reserve_timeout_cnt == RESERVE_TIMEOUT);
    assign     diag_reserve_reply_hit   = reserve_wait && (brx_state == BRX_BEAT1) && rx1_tvalid &&
                                           (brx_dest_id_r == board_id) && (brx_type_r == TYPE_DATA_RESERVE_ACK);

    // ── enum 斷點定位（master 用，2026-08-20 新增）──────────────────────
    reg        diag_wait;            // 1 = 正在等 DIAG_ACK 回來
    assign     diag_busy = diag_wait;
    reg [15:0] diag_timeout_cnt;
    localparam [15:0] DIAG_TIMEOUT = 16'd8192;   // 跟 enum/reserve 同樣的保守值，Opus 覆核建議維持一致不自創新數字

    always @(posedge aurora_clk) begin
        if (rst) begin
            frx_state         <= FRX_IDLE;
            brx_state         <= BRX_IDLE;
            fwd_pending       <= 1'b0;
            bwd_pending       <= 1'b0;
            init_ok           <= 1'b0;
            total_boards      <= 5'd0;
            board_index       <= 5'd0;
            lock_req          <= 1'b0;
            unlock_req        <= 1'b0;
            ack_bitmask       <= 10'd0;
            ack_count         <= 5'd0;
            go_sent           <= 1'b0;
            waiting_for_pause_acks <= 1'b0;
            trig_go_tx_pending <= 1'b0;
            ring_rtt_counter   <= 32'd0;
            ring_rtt_running   <= 1'b0;
            ring_rtt_value     <= 32'd0;
            div_remainder      <= 32'd0;
            div_quotient       <= 32'd0;
            div_divisor        <= 5'd0;
            div_running        <= 1'b0;
            per_hop_value      <= 32'd0;
            trig_fire_req      <= 1'b0;
            trig_fire_group    <= 4'd0;
            pending_trig_group_r <= 4'd0;
            inject_pending    <= 1'b0;
            // 2026-07-07 bugfix：inj_state 拿掉，它有自己專屬的 always
            // block（下面 INJ_IDLE/BEAT0/BEAT1 那個，本身就有完整的
            // reset 處理）——這裡重複驅動同一個 reg 造成兩個 always
            // block 同時是 inj_state 的來源，三板實測 inj_state 永遠
            // 卡在 INJ_IDLE 不會轉換，懷疑就是這個雙重驅動在真實
            // Vivado 合成時跟模擬行為不一致造成的（iverilog 模擬沒有
            // 抓到，兩份模擬都顯示邏輯正常，但硬體上就是不會動）。
            enum_wait         <= 1'b0;
            enum_timeout_cnt  <= 16'd0;
            reserve_wait      <= 1'b0;
            reserve_done      <= 1'b0;
            reserve_ok        <= 1'b0;
            reserve_timeout_cnt <= 16'd0;
            diag_wait            <= 1'b0;
            diag_done            <= 1'b0;
            diag_ok              <= 1'b0;
            diag_r_channel_up_0  <= 1'b0;
            diag_r_channel_up_1  <= 1'b0;
            diag_r_relay_blocked <= 1'b0;
            diag_timeout_cnt     <= 16'd0;
        end else begin
            // defaults：單週期 pulse，除非下面重新設 1，否則清成 0
            lock_req      <= 1'b0;
            unlock_req    <= 1'b0;
            diag_done     <= 1'b0;
            reserve_done  <= 1'b0;
            trig_fire_req <= 1'b0;

            // fwd_pending/bwd_pending 的清除放在最前面（低優先權），這樣
            // 如果同一個 cycle 也有新的請求要排（下面任何一個「set」的
            // 分支），後面的指定會覆蓋這裡，正確地不會把新請求洗掉。
            // （fwd_pending/bwd_pending 只能在這個 always block 裡驅動，
            // 不能另外開一個 block 處理，否則兩個 block 同時驅動同一個
            // reg 在 Vivado 合成會出錯——這是這次重寫時修正的地方）
            //
            // 用 FTX_BEAT1/BTX_BEAT1（正在送最後一個 beat 的當下）當清除
            // 條件，不能用「ftx_state==IDLE && fwd_pending」——因為
            // fwd_pending 剛被設成 1 的那個 cycle，ftx_state 還沒來得及
            // 離開 IDLE（狀態轉換要下一拍才生效），用那個條件會在真正
            // 送出去之前就把 pending 清掉。2026-07-05 補上 tready 條件：
            // 沒有 tready 就代表最後一個 beat 還沒真的被下游收下，這時候
            // 不能清 pending，否則資料會憑空消失。
            if (ftx_state == FTX_BEAT1 && ctrl_tx1_tready) fwd_pending <= 1'b0;
            if (btx_state == BTX_BEAT1 && ctrl_tx0_tready) bwd_pending <= 1'b0;
            // inject_pending 清除：同樣道理，用 INJ_BEAT1 && tready 當條件
            if (inj_state == INJ_BEAT1 && local_inject_tready) inject_pending <= 1'b0;

            // ── host 觸發初始化（只有 master 動作）────────────────────
            // 2026-08-20 新增：`total_boards` 在這裡無條件先歸零，不管
            // 這次 enum 最後成功還是失敗——原本這個值只有真正 enum 完整
            // 跑完（TYPE_ENUM_COUNT 繞完整圈）才會被覆寫，enum 失敗
            // （逾時／一開始 channel_up 就不通）完全不會動到它，導致
            // 「跑過一次成功的多板 enum、環路才斷開、再按一次 Initialize
            // System」時這個值繼續停在舊的板數，被 trigger 邏輯誤判成
            // 還在多板模式（見上方 trig_start_pulse 分支 `|| !channel_
            // up_1` 那次修復的完整說明）。使用者要求「Initialize System
            // 時應該要重置這個值，這樣才是真正的初始化」——這裡才是
            // 真正的源頭修法：每次按下 Initialize System 就立刻回到
            // 乾淨狀態，不用等 trigger 那種下游消費者各自加 escape
            // valve 來將就一個本來就不該殘留的舊值。
            if (init_start_pulse && is_master) begin
                total_boards <= 5'd0;
                if (channel_up_0 && channel_up_1) begin
                    fwd_beat0   <= {24'd2, TYPE_ENUM_COUNT, 16'd0, board_id};
                    fwd_beat1   <= 64'd0;   // 從 0 開始遞增（2026-07-05 簡化）
                    fwd_pending <= 1'b1;
                    enum_wait   <= 1'b1;
                    enum_timeout_cnt <= 16'd0;
                    init_ok     <= 1'b0;
                    // 2026-07-23 新增（改掛 enum，見 ring_rtt_value port
                    // 註解）：ring RTT 計數器歸零開始累加，量測
                    // T_ENUM_COUNT 繞完整圈的時間——enum 只在拓樸真的
                    // 改變（開機/reload/換 master）時才會重跑，比原本
                    // 掛在 TRIG_GO（每次播放觸發都重算）更符合這個值
                    // 該有的生命週期。
                    ring_rtt_counter <= 32'd0;
                    ring_rtt_running <= 1'b1;
                end else begin
                    init_ok <= 1'b0;
                end
            end

            // ── ring RTT 計數器：見 ring_rtt_value port 註解（2026-07-23）
            // ── 位置刻意放在「起點」（上面 init_start_pulse 區塊）之後、
            // 「終點」（下面 TYPE_ENUM_COUNT case）之前，跟 enum_timeout_
            // cnt/enum_wait 這組既有、行為已知正確的類似 pattern（先設
            // enable flag 的區塊、緊接著才是靠這個 flag 累加的區塊）保持
            // 一致的程式碼順序 ──────────────────────────────────────
            if (ring_rtt_running) ring_rtt_counter <= ring_rtt_counter + 32'd1;

            // ── enum timeout（master 用）────────────────────────────────
            if (enum_wait) begin
                if (enum_timeout_cnt == ENUM_TIMEOUT) begin
                    enum_wait <= 1'b0;
                    init_ok   <= 1'b0;
                end else begin
                    enum_timeout_cnt <= enum_timeout_cnt + 16'd1;
                end
            end

            // ── host 觸發 trigger（只有 master 動作）────────────────────
            // 2026-07-24 新增：total_boards_eff==1（單板，不管是手動宣告
            // 還是真的自環 enum 出 1）時，沒有其他板子要協調，跳過
            // PAUSE_REQ/等 ACK 整套流程，直接進本地補償倒數（hops 天生
            // 是 0，倒數幾乎立即 fire）。這不只是簡化——單板完全沒有實體
            // SFP 連線時，channel_up_0/1 恆為 0，Aurora TX 側的
            // ctrl_tx1_tready 永遠不會拉高（已用 ILA 實測證實，見
            // NOTES.md），送 PAUSE_REQ 這個分支的狀態機會永遠卡住等
            // tready，這個分支完全不會嘗試任何 Aurora 傳輸，從根本避開
            // 這個硬體限制。
            // 2026-08-20 新增 `|| !channel_up_1`：`total_boards_eff` 是
            // enum 算出來的 sticky 值，環路斷線後不會自動歸零（跟
            // `channel_up_0/1` 這種即時訊號是兩件事）——上機實測發現
            // 「先跑過一次成功的多板 enum、之後環路才斷開」時
            // `total_boards_eff` 還停在舊的板數，走進下面 else 分支送
            // `PAUSE_REQ`，結果卡在跟上面同一段註解描述的一模一樣的
            // 情況（`ctrl_tx1_tready` 永遠不會拉高），`trig_fire_req`
            // 永遠沒機會被設起來，host 端 `board_ctrl.trigger()` 不等
            // 確認就回傳，完全看不出來這次觸發其實沒有真的生效
            // （test1 面板上機測試撞到，見 PROJECT.md 對應章節）。改成
            // 只要「這片板子自己當下沒有連線夥伴」就跟真正單板一樣直接
            // 本地觸發，不管 `total_boards_eff` 記錄的是什麼——跟
            // `init_start_pulse` 那個分支（上方 `channel_up_0 &&
            // channel_up_1` 才嘗試 enum）同一種「先看即時連線狀態，不要
            // 只信 sticky 值」的判斷原則保持一致。
            if (trig_start_pulse && is_master) begin
                if (total_boards_eff == 5'd1 || !channel_up_1) begin
                    trig_fire_req   <= 1'b1;
                    trig_fire_group <= au_trig_group_select_in;   // 2026-07-27：本板剛收到 T_TRIG_START，值是新鮮的，可直接用
                    go_sent             <= 1'b1;
                end else begin
                    fwd_beat0   <= {24'd2, TYPE_TRIG_PAUSE_REQ, board_id, 16'hFFFF};
                    fwd_beat1   <= 64'd0;
                    fwd_pending <= 1'b1;
                    ack_bitmask <= 10'd0;
                    ack_count   <= 5'd0;
                    go_sent     <= 1'b0;
                    lock_req    <= 1'b1;   // master 自己也要暫停本機資料
                    waiting_for_pause_acks <= 1'b1;   // 真正送出 PAUSE_REQ，開始等 ACK
                    pending_trig_group_r <= au_trig_group_select_in;   // 2026-07-27：先存住，等真正送出 TYPE_TRIG_GO 時才用（見該處說明）
                end
            end

            // ── host 觸發 data reservation（只有 master 動作）───────────
            // host 已經知道目的地（`reserve_dest_id`），直接發起，不用偷看
            // Layer 2 的資料去猜。
            // 2026-08-20 新增 `!channel_up_1` 立即失敗分支：PROJECT.md
            // 既有待辦——這裡原本沒有 guard，`channel_up_1==0` 時送出去
            // 會卡進跟 trigger 那個分支（上方）今天實測到的同一種
            // 永久卡死（FTX 卡在等 `ctrl_tx1_tready`，`fwd_pending`
            // 永遠不清）。比照 DIAG 既有的 `!channel_up_1` 立即失敗
            // 分支（見下方 diag_start_pulse 區塊），不嘗試送 REQ、直接
            // 判定失敗。
            if (reserve_start_pulse && is_master) begin
                if (!channel_up_1) begin
                    reserve_done <= 1'b1;
                    reserve_ok   <= 1'b0;
                end else begin
                    fwd_beat0         <= {24'd2, TYPE_DATA_RESERVE_REQ, board_id, reserve_dest_id};
                    fwd_beat1         <= 64'd0;
                    fwd_pending       <= 1'b1;
                    reserve_dest_id_r <= reserve_dest_id;
                    reserve_wait      <= 1'b1;
                    reserve_timeout_cnt <= 16'd0;
                end
            end

            // ── reservation timeout（master 用）─────────────────────────
            if (reserve_wait) begin
                if (reserve_timeout_cnt == RESERVE_TIMEOUT) begin
                    reserve_wait <= 1'b0;
                    reserve_done <= 1'b1;
                    reserve_ok   <= 1'b0;   // 逾時，明確判定失敗
                end else begin
                    reserve_timeout_cnt <= reserve_timeout_cnt + 16'd1;
                end
            end

            // ── host 觸發 enum 斷點定位（只有 master 動作，2026-08-20 新增）──
            // Opus 覆核抓到的 bug：reserve_start_pulse 沒有 !fwd_pending/
            // ftx_state==FTX_IDLE 這層 guard，host 如果在前一次還沒完成時
            // 又發一次，會在 tvalid 已經拉高時改變 tdata（AXI4-Stream
            // 協定違規）。這裡明確補上這層 guard，host 端必須序列式呼叫
            // （見上方 diag_start_pulse port 註解）。
            // channel_up_1==0 時直接判定失敗、完全不碰 Aurora TX——這是
            // 跟 dispatcher.v 今天修過的同一種道理：明知送不出去就不要
            // 嘗試，不要指望下面的 FTX timeout 兜底（那是給「有連線但
            // 卡住」的情況用的，不是給「本來就沒連線」用的，兩者原因
            // 不同，處理方式也該分開，見 PROJECT.md「dispatcher.v channel_
            // up_1 修法」章節同樣的分類方式）。
            if (diag_start_pulse && is_master && !diag_wait && !fwd_pending &&
                ftx_state == FTX_IDLE) begin
                if (diag_dest_id == board_id) begin
                    // 查自己：本機直接短路回覆，不需要送任何封包
                    diag_done            <= 1'b1;
                    diag_ok              <= 1'b1;
                    diag_r_channel_up_0  <= channel_up_0;
                    diag_r_channel_up_1  <= channel_up_1;
                    diag_r_relay_blocked <= 1'b0;
                end else if (!channel_up_1) begin
                    diag_done <= 1'b1;
                    diag_ok   <= 1'b0;
                end else begin
                    fwd_beat0   <= {24'd2, TYPE_DIAG_REQ, board_id, diag_dest_id};
                    fwd_beat1   <= 64'd0;
                    fwd_pending <= 1'b1;
                    diag_wait   <= 1'b1;
                    diag_timeout_cnt <= 16'd0;
                end
            end

            // ── diag timeout（master 用）─────────────────────────────────
            // 刻意放在下面 backward RX 狀態機（BRX）之前——跟既有 reserve
            // timeout/BRX 的相對順序一致：同一拍 ACK 跟 timeout 都成立時，
            // 後面的 BRX case（往後執行、non-blocking 賦值後寫者贏）會
            // 覆蓋這裡，ACK 優先於 timeout，語意才對。
            if (diag_wait) begin
                if (diag_timeout_cnt == DIAG_TIMEOUT) begin
                    diag_wait <= 1'b0;
                    diag_done <= 1'b1;
                    diag_ok   <= 1'b0;   // 逾時，明確判定失敗
                end else begin
                    diag_timeout_cnt <= diag_timeout_cnt + 16'd1;
                end
            end

            // ── forward RX 狀態機 ───────────────────────────────────────
            case (frx_state)
                FRX_IDLE: begin
                    // 2026-07-29 新增：channel_up_0 閘控（見檔頭同日
                    // 說明）——連線還沒真正建立前，不信任 rx0_tvalid，
                    // 避免 link 還沒 lock 時的殘留/不確定內容被誤判成
                    // 真封包。
                    if (f_beat0_is_ours && f_beat0_gate_ok) begin
                        frx_type_r         <= f_beat0_type;
                        frx_src_id_r       <= f_beat0_src_id;
                        frx_dest_id_r      <= f_beat0_dest_id;
                        frx_ring_return_r  <= f_beat0_ring_return;
                        frx_needs_relay_r  <= f_beat0_needs_relay;
                        frx_state          <= FRX_BEAT1;
                    end
                end
                FRX_BEAT1: begin
                    if (rx0_tvalid) begin
                        case (frx_type_r)
                            TYPE_ENUM_COUNT: begin
                                if (frx_dest_id_r == board_id) begin
                                    // 繞回發起者：最終值 = 經過的「其他」板子數（每站+1），
                                    // +1 是加回 master 自己這一片
                                    total_boards <= {1'b0, rx0_tdata[3:0]} + 5'd1;
                                    init_ok      <= 1'b1;
                                    enum_wait    <= 1'b0;
                                    // 2026-07-23 新增：master 收到自己送出的
                                    // enum 繞完整圈回來，鎖存 ring RTT 計數值、
                                    // 停止計數（見 ring_rtt_value port 註解）
                                    ring_rtt_value   <= ring_rtt_counter;
                                    ring_rtt_running <= 1'b0;
                                    // 2026-07-23 新增：同一拍啟動除法器算
                                    // per_hop_value。除數不能直接讀 total_
                                    // boards（這個 reg 這一拍才剛被指定新值，
                                    // 這一拍讀到的還是舊值），要用跟
                                    // total_boards 同一個算式（{1'b0,
                                    // rx0_tdata[3:0]}+5'd1）現算，兩邊保證
                                    // 一致。
                                    div_remainder <= ring_rtt_counter;
                                    div_quotient  <= 32'd0;
                                    div_divisor   <= {1'b0, rx0_tdata[3:0]} + 5'd1;
                                    div_running   <= 1'b1;
                                end else begin
                                    // 中途站：收到值 +1 之後，這個新值同時就是自己的
                                    // board_index（不用再額外計算），直接轉送下去
                                    board_index    <= {1'b0, rx0_tdata[3:0]} + 5'd1;
                                    fwd_beat0      <= {24'd2, TYPE_ENUM_COUNT, frx_src_id_r, frx_dest_id_r};
                                    fwd_beat1      <= {56'd0, rx0_tdata[3:0] + 4'd1};
                                    fwd_pending    <= 1'b1;
                                end
                            end
                            TYPE_TRIG_PAUSE_REQ: begin
                                lock_req <= 1'b1;
                                if (frx_needs_relay_r) begin
                                    fwd_beat0   <= {24'd2, TYPE_TRIG_PAUSE_REQ, frx_src_id_r, 16'hFFFF};
                                    fwd_beat1   <= 64'd0;
                                    fwd_pending <= 1'b1;
                                end
                                if (!is_master) begin
                                    bwd_beat0   <= {24'd2, TYPE_TRIG_PAUSE_ACK, board_id, frx_src_id_r};
                                    bwd_beat1   <= {59'd0, board_index};
                                    bwd_pending <= 1'b1;
                                end
                            end
                            TYPE_TRIG_GO: begin
                                unlock_req    <= 1'b1;
                                // 2026-07-23 改版、2026-07-30 再改版：不再
                                // 合成封包注入，也不再本地倒數（見上方
                                // trig_fire_req port 註解，倒數搬到 dac_
                                // clk domain 的 dac_trig_queue.v），這裡只
                                // 送出「決定要 fire」的 pulse + group_
                                // select，交給 BD 層 xpm_fifo_async 送過去。
                                // 跟舊版同樣的 ring_return 防護邏輯（master
                                // 自己的廣播繞完一圈回到自己時，不能重複
                                // 觸發——master 已經在下面 trig_go_tx_
                                // pending 那條路徑觸發過一次了）。
                                // 2026-07-27 新增（Group-based Trigger 架構）：
                                // group_select 從收到的封包內容（rx0_tdata
                                // 低 4 bit）解出來，不是自己本機的
                                // au_trig_group_select_in——slave 從沒收過
                                // T_TRIG_START 封包，這個值必須靠 master
                                // piggyback 進 TYPE_TRIG_GO beat1 傳過來
                                // （比照 per_hop_value/total_boards 的既有
                                // 模式）。
                                if (!frx_ring_return_r) begin
                                    trig_fire_req   <= 1'b1;
                                    trig_fire_group <= rx0_tdata[3:0];
                                end
                                if (frx_needs_relay_r) begin
                                    fwd_beat0   <= {24'd2, TYPE_TRIG_GO, frx_src_id_r, 16'hFFFF};
                                    // 2026-07-27：原封不動把收到的 group_select
                                    // 繼續帶給下一站，不能繼續填 64'd0，否則
                                    // 環路後段的板子會收到錯誤的（全 0）分組值
                                    fwd_beat1   <= {60'd0, rx0_tdata[3:0]};
                                    fwd_pending <= 1'b1;
                                end
                            end
                            TYPE_DATA_RESERVE_REQ: begin
                                // fail-fast：忙碌就在這一站直接停止轉送，原路退回
                                if (is_busy) begin
                                    bwd_beat0   <= {24'd2, TYPE_DATA_RESERVE_ACK, board_id, frx_src_id_r};
                                    bwd_beat1   <= 64'd0;   // bit0=0: 忙碌
                                    bwd_pending <= 1'b1;
                                end else if (frx_dest_id_r == board_id) begin
                                    // 不忙碌 + 是最終目的地：立刻鎖住、回「全部通過」
                                    lock_req    <= 1'b1;
                                    bwd_beat0   <= {24'd2, TYPE_DATA_RESERVE_ACK, board_id, frx_src_id_r};
                                    bwd_beat1   <= 64'd1;   // bit0=1: 通過
                                    bwd_pending <= 1'b1;
                                end else begin
                                    // 不忙碌 + 還不是最終目的地：立刻鎖住、默默繼續轉送
                                    lock_req    <= 1'b1;
                                    fwd_beat0   <= {24'd2, TYPE_DATA_RESERVE_REQ, frx_src_id_r, frx_dest_id_r};
                                    fwd_beat1   <= 64'd0;
                                    fwd_pending <= 1'b1;
                                end
                            end
                            TYPE_DIAG_REQ: begin
                                // 沒有 is_busy 分支、不設 lock_req——純讀取，
                                // 診斷功能不該被資料層忙碌狀態擋住（這正是
                                // 要在系統出問題時也能用的工具）。
                                if (frx_dest_id_r == board_id) begin
                                    // 我就是被查的目標：不管 is_busy，一律回覆
                                    bwd_beat0 <= {24'd2, TYPE_DIAG_ACK, board_id, frx_src_id_r};
                                    // bit0=is_target bit1=channel_up_0
                                    // bit2=channel_up_1 bit3=relay_blocked
                                    // bits[8:4]=board_index
                                    bwd_beat1 <= {55'd0, board_index, 1'b0,
                                                  channel_up_1, channel_up_0, 1'b1};
                                    bwd_pending <= 1'b1;
                                end else if (frx_ring_return_r) begin
                                    // 繞完整圈回到發起者，沒有任何板子認領
                                    // 這個 dest_id——判定「查詢的板子不存在」，
                                    // 不用等 8192 cycle timeout 才知道結果
                                    // （只有 is_master 時 diag_wait 才可能是
                                    // 1，非 master 板子這裡是無害的 no-op）。
                                    if (diag_wait) begin
                                        diag_wait <= 1'b0;
                                        diag_done <= 1'b1;
                                        diag_ok   <= 1'b0;
                                    end
                                end else if (!channel_up_1) begin
                                    // 這一站往下一棒送不出去：不要靜默丟包，
                                    // 原地回報「轉不出去」，讓斷點定位更精確
                                    // （不用等 master 自己 timeout 才知道
                                    // 「查不到」，中繼站主動回報自己的位置）。
                                    bwd_beat0 <= {24'd2, TYPE_DIAG_ACK, board_id, frx_src_id_r};
                                    bwd_beat1 <= {55'd0, board_index, 1'b1,
                                                  channel_up_1, channel_up_0, 1'b0};
                                    bwd_pending <= 1'b1;
                                end else begin
                                    // 轉不出去以外的正常情況：繼續往下一棒轉送
                                    fwd_beat0   <= {24'd2, TYPE_DIAG_REQ, frx_src_id_r, frx_dest_id_r};
                                    fwd_beat1   <= 64'd0;
                                    fwd_pending <= 1'b1;
                                end
                            end
                            default: ;
                        endcase
                        frx_state <= FRX_IDLE;
                    end
                end
                default: frx_state <= FRX_IDLE;
            endcase

            // ── per_hop_value 除法器逐拍步進（見上方 port/reg 註解，
            // 2026-07-23）：刻意放在整個 case(frx_state) 區塊（div_running
            // 的啟動點在裡面的 TYPE_ENUM_COUNT 分支）之後，比照
            // ring_rtt_counter 那組「enable 設定區塊在前、累加區塊緊接在
            // 後」的既有順序慣例，避免重蹈 xsim/iverilog 排程分歧的
            // 覆轍（見 NOTES.md 2026-07-23 小節）。div_divisor==0 理論上
            // 不會發生（enum 真的繞完整圈回來，至少代表有 2 片板子），
            // 這裡仍保留防呆，避免萬一真的是 0 造成永遠減不完。
            if (div_running) begin
                if (div_divisor == 5'd0) begin
                    div_running   <= 1'b0;
                    per_hop_value <= 32'd0;
                end else if (div_remainder >= {27'd0, div_divisor}) begin
                    div_remainder <= div_remainder - {27'd0, div_divisor};
                    div_quotient  <= div_quotient + 32'd1;
                end else begin
                    div_running   <= 1'b0;
                    per_hop_value <= div_quotient;
                end
            end

            // ── backward RX 狀態機（rx1，認 PAUSE_ACK 跟 DATA_RESERVE_ACK）──
            case (brx_state)
                BRX_IDLE: begin
                    // 2026-07-29 新增：channel_up_1 閘控，同 FRX_IDLE。
                    if (b_beat0_is_ours && b_beat0_gate_ok) begin
                        brx_type_r    <= b_beat0_type;
                        brx_dest_id_r <= b_beat0_dest_id;
                        brx_src_id_r  <= b_beat0_src_id;
                        brx_state     <= BRX_BEAT1;
                    end
                end
                BRX_BEAT1: begin
                    if (rx1_tvalid) begin
                        if (brx_dest_id_r == board_id) begin
                            case (brx_type_r)
                                TYPE_TRIG_PAUSE_ACK: begin
                                    if (!ack_bitmask[rx1_tdata[3:0]]) begin
                                        ack_bitmask[rx1_tdata[3:0]] <= 1'b1;
                                        ack_count                   <= ack_count + 5'd1;
                                    end
                                end
                                TYPE_DATA_RESERVE_ACK: begin
                                    if (reserve_wait) begin
                                        reserve_wait <= 1'b0;
                                        reserve_done <= 1'b1;
                                        reserve_ok   <= rx1_tdata[0];   // bit0: 1=通過 0=忙碌/失敗
                                    end
                                end
                                TYPE_DIAG_ACK: begin
                                    if (diag_wait) begin
                                        diag_wait            <= 1'b0;
                                        diag_done            <= 1'b1;
                                        diag_ok              <= 1'b1;
                                        diag_r_channel_up_0  <= rx1_tdata[1];
                                        diag_r_channel_up_1  <= rx1_tdata[2];
                                        diag_r_relay_blocked <= rx1_tdata[3];
                                    end
                                end
                                default: ;
                            endcase
                        end else begin
                            // 不是給我的：繼續往反向轉送（type 沿用鎖存值，
                            // 適用 PAUSE_ACK/DATA_RESERVE_ACK/DIAG_ACK 三種）
                            bwd_beat0   <= {24'd2, brx_type_r, brx_src_id_r, brx_dest_id_r};
                            bwd_beat1   <= rx1_tdata;
                            bwd_pending <= 1'b1;
                        end
                        brx_state <= BRX_IDLE;
                    end
                end
                default: brx_state <= BRX_IDLE;
            endcase

            // ── master：收滿全部 ack，發起 TRIG_GO（sticky go_sent 避免重複）──
            // 2026-07-24：改用 total_boards_eff（尊重 manual_total_boards_in
            // 覆寫），跟上面 trig_start_pulse 分支判斷用同一個有效值。
            // total_boards_eff==1 的情況已經在上面分支直接處理、go_sent
            // 已經被設成 1，正常不會走到這裡；但這個判斷式原本隱含假設
            // 「total_boards 只有 enum 真正跑完才會變成非 0，不會平白
            // 成立」——manual_total_boards_in 打破了這個假設：光是手動
            // 設成非 0，配合 reset 後 go_sent/ack_count/fwd_pending 剛好
            // 都是「乾淨」的初始值，這個判斷式會不需要真的送過
            // trig_start_pulse 就自己成立，平白多發一次 GO（這是實測
            // 模擬抓到的真實 bug，不是理論推演）。修法：新增
            // waiting_for_pause_acks 這個明確旗標，只有上面 trig_start_
            // pulse 的多板分支真正送出 PAUSE_REQ 時才會設成 1，這裡
            // 改成明確檢查這個旗標，不再依賴其他 reg 剛好是初始值的
            // 巧合。額外加上 !trig_start_pulse 當第二層防護，避免這個
            // 判斷式跟「剛送出的新一輪 trigger 請求」同一拍觸發（non-
            // blocking 賦值語意下，同一拍讀到的都還是上一拍的舊值）。
            if (is_master && waiting_for_pause_acks && !go_sent &&
                (total_boards_eff > 5'd0) &&
                (ack_count == total_boards_eff - 5'd1) && !fwd_pending &&
                !trig_start_pulse) begin
                fwd_beat0   <= {24'd2, TYPE_TRIG_GO, board_id, 16'hFFFF};
                // 2026-07-27（Group-based Trigger 架構）：piggyback
                // pending_trig_group_r（trig_start_pulse 當下存住的
                // group_select），讓 slave 收到 GO 時可以解出這次要
                // fire 哪些 group，比照 per_hop_value/total_boards
                // piggyback 進 T_BOARD_ID_ASSIGN 的既有模式。
                fwd_beat1   <= {60'd0, pending_trig_group_r};
                fwd_pending <= 1'b1;
                waiting_for_pause_acks <= 1'b0;
                go_sent     <= 1'b1;
                unlock_req  <= 1'b1;
                // 合成封包不在這裡注入，見下面 trig_go_tx_pending 說明
                trig_go_tx_pending <= 1'b1;
            end

            // ── master 自己「決定要 fire」：等 T_TRIG_GO 真的送出去
            // （ftx 最後一個 beat 完成握手）才觸發，不能提早 ──
            // 2026-07-23 改版、2026-07-30 再改版：不再合成封包注入，
            // 也不再本地倒數，直接送出 trig_fire_req pulse（見上方
            // port 註解，真正的倒數在 dac_clk domain 的 dac_trig_
            // queue.v 做）。
            if (trig_go_tx_pending && ftx_state == FTX_BEAT1 && ctrl_tx1_tready) begin
                trig_fire_req   <= 1'b1;
                // 2026-07-27：用 pending_trig_group_r（trig_start_pulse
                // 當下存住的值），不是即時的 au_trig_group_select_in——
                // 這裡可能是 trig_start_pulse 發生後好幾拍才執行到（等
                // PAUSE_REQ/ACK 走完），host 理論上可能在等待期間又送了
                // 下一次不相關的 T_TRIG_START，用即時值會讀到錯誤的資料，
                // 且會跟這次真正送給 slave 的 GO 封包內容（見上方 fwd_
                // beat1）不一致。
                trig_fire_group <= pending_trig_group_r;
                trig_go_tx_pending  <= 1'b0;
            end

            // ── forward TX：drain fwd_beat0/fwd_beat1（見下方獨立狀態機）──
            // ── backward TX：drain bwd_beat0/bwd_beat1（見下方獨立狀態機）──
        end
    end

    // ══════════════════════════════════════════════════════════════════
    //  Forward TX：drain fwd_beat0/fwd_beat1 -> ctrl_tx1_*
    // ══════════════════════════════════════════════════════════════════
    localparam FTX_IDLE  = 2'd0;
    localparam FTX_BEAT0 = 2'd1;
    localparam FTX_BEAT1 = 2'd2;

    reg [1:0] ftx_state = FTX_IDLE;

    always @(posedge aurora_clk) begin
        if (rst) begin
            ftx_state <= FTX_IDLE;
        end else begin
            case (ftx_state)
                FTX_IDLE:  if (fwd_pending) ftx_state <= FTX_BEAT0;
                FTX_BEAT0: if (ctrl_tx1_tready) ftx_state <= FTX_BEAT1;
                FTX_BEAT1: if (ctrl_tx1_tready) ftx_state <= FTX_IDLE;
                default:   ftx_state <= FTX_IDLE;
            endcase
        end
    end

    assign ctrl_tx1_tdata  = (ftx_state == FTX_BEAT0) ? fwd_beat0 :
                             (ftx_state == FTX_BEAT1) ? fwd_beat1 : 64'd0;
    assign ctrl_tx1_tvalid = (ftx_state == FTX_BEAT0) || (ftx_state == FTX_BEAT1);
    assign ctrl_tx1_tlast  = (ftx_state == FTX_BEAT1);

    // ══════════════════════════════════════════════════════════════════
    //  Backward TX：drain bwd_beat0/bwd_beat1 -> ctrl_tx0_*
    // ══════════════════════════════════════════════════════════════════
    localparam BTX_IDLE  = 2'd0;
    localparam BTX_BEAT0 = 2'd1;
    localparam BTX_BEAT1 = 2'd2;

    reg [1:0] btx_state = BTX_IDLE;

    always @(posedge aurora_clk) begin
        if (rst) begin
            btx_state <= BTX_IDLE;
        end else begin
            case (btx_state)
                BTX_IDLE:  if (bwd_pending) btx_state <= BTX_BEAT0;
                BTX_BEAT0: if (ctrl_tx0_tready) btx_state <= BTX_BEAT1;
                BTX_BEAT1: if (ctrl_tx0_tready) btx_state <= BTX_IDLE;
                default:   btx_state <= BTX_IDLE;
            endcase
        end
    end

    assign ctrl_tx0_tdata  = (btx_state == BTX_BEAT0) ? bwd_beat0 :
                             (btx_state == BTX_BEAT1) ? bwd_beat1 : 64'd0;
    assign ctrl_tx0_tvalid = (btx_state == BTX_BEAT0) || (btx_state == BTX_BEAT1);
    assign ctrl_tx0_tlast  = (btx_state == BTX_BEAT1);

    // ══════════════════════════════════════════════════════════════════
    //  合成 TRIGGER(0x01) 封包注入：drain -> local_inject_*（2026-07-05）
    //  跟 ftx/btx 同樣的 IDLE/BEAT0/BEAT1 pattern，差別是內容固定（不用
    //  額外的 beat0/beat1 暫存器，直接組合邏輯給常數）
    // ══════════════════════════════════════════════════════════════════
    always @(posedge aurora_clk) begin
        if (rst) begin
            inj_state <= INJ_IDLE;
        end else begin
            case (inj_state)
                INJ_IDLE:  if (inject_pending)       inj_state <= INJ_BEAT0;
                INJ_BEAT0: if (local_inject_tready)  inj_state <= INJ_BEAT1;
                INJ_BEAT1: if (local_inject_tready)  inj_state <= INJ_IDLE;
                default:   inj_state <= INJ_IDLE;
            endcase
        end
    end

    assign local_inject_tdata  = (inj_state == INJ_BEAT0) ?
                                  {24'd2, TYPE_TRIGGER_INJECT, board_id, board_id} : 64'd0;
    assign local_inject_tvalid = (inj_state == INJ_BEAT0) || (inj_state == INJ_BEAT1);

    // ══════════════════════════════════════════════════════════════════
    //  已知限制 / 下一輪待做（2026-07-05）：
    //  1. fwd_pending/bwd_pending 都只有 1-deep，沒有真正的 FIFO，如果
    //     下一個要送的東西在前一個還沒送完時就到，會被蓋掉——控制封包
    //     頻率低、目前測試場景下應該不會撞到，但沒有嚴謹證明，跟
    //     Layer 1/2 的已知限制同一類
    //  2. T_DATA_RESERVE_REQ/ACK（0x25/0x26）完全還沒實作，這裡只有
    //     開機編號 + trigger 同步兩組協定
    //  3. ctrl_tx1_*/ctrl_tx0_* 沒有 tready 輸入，假設外部（跟 Layer 2
    //     的 data_tx1_* 合併的仲裁器）不會 backpressure 控制封包，這個
    //     假設沿用既有 aurora_tx_arbiter.v 對 relay_* 的處理方式
    //  4. is_master 的 trigger/enum 觸發跟 fwd_pending 共用同一個 1-deep
    //     佇列，如果 host 太快連續下兩個 TI bit（先 init 又馬上 trig），
    //     可能會有請求互相蓋掉的風險，目前信任 host 端不會這樣做
    // ══════════════════════════════════════════════════════════════════

endmodule
`default_nettype wire
