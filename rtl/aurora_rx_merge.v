`timescale 1ns/1ps
`default_nettype none

// aurora_rx_merge.v -- Step 15a: 合併 Layer 2 的本地送達輸出（data_local_*）
// 跟 Layer 3 的合成 TRIGGER(0x01) 封包注入（inject_*），變成單一一條餵給
// async_fifo_aurora_rx -> dispatcher_0 的 local_out_tdata/local_out_tvalid。
//
// 2026-07-05 首版草稿（DRAFT，尚未上板驗證，已用 iverilog 模擬）。
//
// **為什麼需要這個**：使用者要求「不管 trigger 是本機發起還是收到別人的，
// 都要走同一條路徑（都要經過 dispatcher）」，所以 Layer 3 收到/決定要
// 觸發 T_TRIG_GO 時，改成合成一個真正的 TRIGGER(0x01) 封包（見
// aurora_ctrl_channel.v 的 local_inject_*），而不是拉一條獨立的
// trigger_pulse 訊號線直接跨到 sys_clk。但 Layer 2 的 local_out_tdata 已經是
// 單一輸出，Layer 3 不能也直接接同一條線，所以需要這個小合併器。
//
// **優先權規則（不對稱，故意的）**：data_local_* 優先權最高、且**不能有
// 任何一拍的延遲**——這是因為它上游最終接的是實體 Aurora RX，Framing
// mode 沒有 tready 這種東西，一旦資料進來就一定要「這一拍立刻」接住，
// 接不住就是真的漏資料。2026-07-05 修正：原本用一個 registered state
// machine 決定要接誰的資料，結果 state 從 IDLE 追上 DATA 需要一拍，這一拍
// 的空窗期會讓 data_local_tvalid 剛從 idle 轉成忙碌的第一個 beat 被誤判
// 成「還沒輪到你」而漏接（`sim/tb_aurora_rx_merge.v` Test 2 抓到這個真
// bug）。改成**純組合邏輯**：data_local_tvalid 這一拍是 1，就這一拍立刻
// 直通，完全不經過任何暫存器判斷。
//
// inject_* 則是「查完不忙碌才觸發」的合成事件，本來就可以稍微等待，所以
// inject 只有在 data_local_tvalid 是 0 的那些 cycle 才會被授權
// （`inject_tready`），不需要額外的 state：如果 inject 傳到一半、
// data_local_tvalid 突然變 1（真正的 Aurora 資料進來了），inject 那一拍
// 會被暫停（`inject_tready` 變 0），但因為 Layer 3 自己的 `inj_state`
// 狀態機是「沒 tready 就停在原地，不會前進」，data 結束後 inject 會自動
// 從暫停的地方繼續，不需要在這裡額外處理「怎麼恢復」。
//
// **已知的殘留 jitter（跟使用者確認過，屬於可接受的 tradeoff）**：如果
// trigger 事件發生的當下，剛好有其他封包正在經過本站的 relay/本地送達
// （不受 T_TRIG_PAUSE_REQ 的本機發送鎖影響，因為那個鎖只擋本機「發起」，
// 不擋「收到別人的資料在轉送/送達」），trigger 封包注入會被延後到目前
// 這個封包送完才插入。這跟環路實際傳輸/SERDES 延遲一樣，都是既有已知、
// 無法用設計完全消除的部分。

module aurora_rx_merge (
    input  wire        clk,
    input  wire        rst,

    // Layer 2 本地送達輸出：優先權最高，無法回壓、不能有任何延遲
    input  wire [63:0] data_local_tdata,
    input  wire        data_local_tvalid,

    // Layer 3 合成封包注入：只有在 data 沒有用這條線的那些 cycle 才輪得到
    input  wire [63:0] inject_tdata,
    input  wire        inject_tvalid,
    output wire        inject_tready,

    // 合併後餵給 async_fifo_aurora_rx
    output wire [63:0] local_out_tdata,
    output wire        local_out_tvalid
);

    // 純組合邏輯，沒有暫存器——data 這一拍是 1 就立刻直通，不等任何 state
    // 追上；inject 只在 data 這一拍是 0 時才有機會，逐拍判斷，不是「鎖定
    // 一次就不管 data」。
    assign local_out_tdata   = data_local_tvalid ? data_local_tdata :
                            inject_tvalid    ? inject_tdata     : 64'd0;
    assign local_out_tvalid  = data_local_tvalid || inject_tvalid;
    assign inject_tready = !data_local_tvalid;

endmodule
`default_nettype wire
