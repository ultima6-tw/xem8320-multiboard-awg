`timescale 1ns/1ps
`default_nettype none

// aurora_rx_beat0_gate.v -- 2026-07-29 新增
//
// 集中處理「這個 beat0 合不合法」的判斷，供 aurora_ctrl_channel.v
// （forward/backward 各一份 instance）+ aurora_data_channel_
// relayfifo.v（forward 一份）共用，避免同一份邏輯散落在多個模組各自
// 重寫一次（這幾輪 debug 過程中已經各自加過一次 src_id 檢查、一次
// channel_up 檢查，每次都要記得同步改兩三個地方，容易漏）。
//
// **職責範圍**：只判斷「這個 beat0 合不合法」，不管「這個 type 是不是
// 我關心的」——後者留給呼叫端各自判斷（因為三個呼叫點關心的 type
// 集合本來就不一樣），呼叫端把自己已經解碼好的 beat0 欄位餵進來即可，
// 這個模組不重新解碼 rx_tdata，避免跟呼叫端各自的欄位宣告不一致。
//
// **三個條件，channel_up 是優先前提關卡**（2026-07-29 使用者要求：
// 避免其他檢查把 channel_up 這件事略過）：
//   1. channel_up：連線沒建立前，不信任 rx_tdata 的任何內容——
//      `beat0_decode_en = rx_tvalid && channel_up` 先決條件
//   2. src_id 合理性：< effective_max_boards（沿用既有 2026-07-21/29
//      的做法，`total_boards_in` 是 T_BOARD_ID_ASSIGN 廣播來的正確值，
//      enum 還沒跑完（==0）時用 DEFAULT_MAX_BOARDS 當保守上限）
//   3. dest_id 合理性：只看 `dest_id` 實際值本身——`==16'hFFFF`（合法
//      broadcast）或落在 effective_max_boards 範圍內（合法 unicast），
//      兩者任一成立即可，不看 beat0 type（2026-07-29 修正：原本靠
//      呼叫端傳入 `dest_is_broadcast_type`，用 per-type 白名單判斷是
//      不是 broadcast，這份名單漏列 `T_BOARD_ID_ASSIGN`，導致它
//      broadcast 時 `dest_id=0xFFFF` 被誤判成 unicast 而整包被丟棄，
//      見 `NOTES.md` 2026-07-29「build9 上機：發現一個新的、獨立的
//      迴歸 bug」章節——改成直接看值本身，不需要呼叫端維護白名單，
//      不會再有漏列的問題）
//
// 純組合邏輯，跟呼叫端同一個 aurora_clk domain，不需要額外 CDC。
// 沒有新增任何偵測/debug port（使用者明確表示不需要分辨是哪個條件
//擋下的，單純阻擋即可）。

module aurora_rx_beat0_gate (
    input  wire        channel_up,
    input  wire [4:0]  total_boards_in,        // 0 = enum 還沒跑完，用 DEFAULT_MAX_BOARDS
    input  wire        rx_tvalid,
    input  wire [15:0] beat0_src_id,
    input  wire [15:0] beat0_dest_id,

    output wire        beat0_gate_ok           // channel_up 優先關卡 && src/dest 合法
);

    localparam [4:0] DEFAULT_MAX_BOARDS = 5'd4;
    wire [4:0] effective_max_boards =
        (total_boards_in != 5'd0) ? total_boards_in : DEFAULT_MAX_BOARDS;

    // channel_up 優先前提關卡：連線沒建立就不繼續判斷 src/dest（語意上
    // 明確表達「channel_up 是前提，不是平行條件」，即使合成結果等同
    // 平行 AND）。
    wire beat0_decode_en = rx_tvalid && channel_up;

    wire src_id_valid  = (beat0_src_id  < {11'd0, effective_max_boards});
    wire dest_id_valid = (beat0_dest_id == 16'hFFFF) ||
                          (beat0_dest_id < {11'd0, effective_max_boards});

    assign beat0_gate_ok = beat0_decode_en && src_id_valid && dest_id_valid;

endmodule
`default_nettype wire
