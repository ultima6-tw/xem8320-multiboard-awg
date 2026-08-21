`timescale 1ns/1ps
`default_nettype none

// dac_trig_queue.v -- 2026-07-30 新增
//
// 背景：`aurora_ctrl_channel.v` 原本的補償延遲倒數（native_trig_cnt/
// native_trig_pending）全程用 aurora_clk（每片板子各自獨立的板上
// 固定振盪器，板跟板之間沒有共用參考）計時，短間隔連續 trigger 時
// 新事件會直接蓋掉還沒 fire 的舊倒數（沒有任何保護）。這個模組把
// 「倒數」這件事整個搬到 dac_clk domain（多板同步模式下是外部
// Si5332，真正跨板共用的參考時脈），同時解決兩個問題：
//   ①用一個深度 QUEUE_DEPTH 的佇列取代單一倒數暫存器，短間隔連續
//     trigger 不會再互相蓋掉
//   ②補償延遲的「等多久」全程用跨板共用的 dac_clk 計時，不再受限於
//     aurora_clk 各板獨立振盪器的 ppm 誤差
//
// 詳細討論見 NOTES.md 2026-07-30「sine wave mode 精確度討論 + 短
// 間隔連續 trigger 補償延遲被蓋掉的問題」章節。
//
// 架構（跟 aurora_ctrl_channel.v 的分工）：
//   aurora_ctrl_channel.v（aurora_clk domain，不變）：協商（PAUSE_
//   REQ/收 ACK/送 GO）+ 算出這片板子的補償延遲量（native_hops×
//   per_hop_value_eff，aurora_clk cycle 單位，quasi-static）。決定
//   「這輪真的要 fire」的那一拍，送出 trig_fire_req（1-cycle pulse）
//   + trig_fire_group（4-bit）。
//   BD 層：trig_fire_req/trig_fire_group 寫進一顆 xpm_fifo_async
//   （aurora_clk 寫入側，深度跟這個模組的 QUEUE_DEPTH 一致或更深，
//   FWFT 模式），這個模組只碰 dac_clk 讀側——CDC 本身用官方 IP，這裡
//   完全不用自己手刻 CDC 邏輯。native_trig_delay_out（aurora_clk
//   cycle 單位的補償延遲量，quasi-static）另外用既有 level_cdc 慣例
//   跨過來。
//   這個模組（dac_clk domain）：把 aurora_clk cycle 單位的延遲量
//   換算成 dac_clk cycle 數（aurora_clk=156.25MHz、dac_clk=100MHz
//   標稱比例 16/25，沿用 2026-07-23 之前 au_trig_delay 機制用過的
//   同一個換算比例，見 NOTES.md 對應章節），維護一個自由跑的 dac_
//   clk cycle 計數器，每次從 async FIFO 收到一筆新的 fire 請求就
//   算出 fire_at = 目前 cycle 數 + 換算後延遲，塞進深度 QUEUE_DEPTH
//   的 fire-timestamp 佇列（同時脈域，手刻小型 circular buffer，
//   不是 CDC 敏感邏輯，不強制用官方 IP）；每個 cycle 檢查佇列最前面
//   那筆，時間到了就真正 pulse native_trig_out（本身已經是 dac_clk
//   domain，不用再額外 CDC 一次——比現行架構還少一次 CDC）。
//
// 佇列採 FIFO 順序（不用排序邏輯）：因為 fire_at = push 當下的
// dac_cycle_cnt + 固定延遲量，只要 push 順序跟真實時間順序一致
// （aurora_clk 端事件本來就是照時間順序處理），fire_at 值天生就是
// 遞增的，佇列最前面永遠是下一個該 fire 的。

module dac_trig_queue #(
    parameter QUEUE_DEPTH = 8,
    parameter PTR_WIDTH   = 3   // $clog2(QUEUE_DEPTH)，QUEUE_DEPTH=8 時固定 3
) (
    input  wire        dac_clk,
    input  wire        dac_rst,

    // ── 讀 xpm_fifo_async 的 dac_clk 側（BD 層接線，FWFT 模式）───────
    // aurora_clk 寫入側由 aurora_ctrl_channel_0/trig_fire_req(wr_en)+
    // trig_fire_group(din) 驅動，這裡只碰讀側。fifo_rd_en 是組合邏輯
    // （見下方 push_now），跟 fifo_rd_data 同一拍讀取同一拍就要求
    // FIFO 前進，不能registered延遲一拍，否則 FWFT 模式下會把同一筆
    // 資料重複 push 兩次。
    input  wire [3:0]  fifo_rd_data,
    input  wire        fifo_rd_empty,
    output wire        fifo_rd_en,

    // ── 補償延遲量（aurora_clk cycle 單位）──────────────────────────
    // level_cdc 從 aurora_ctrl_channel_0/native_trig_delay_out 跨過來
    // （quasi-static，只有 enum/T_BOARD_ID_ASSIGN 之後才會變）。
    input  wire [36:0] delay_aurora_cycles,

    output reg         native_trig_out       = 1'b0,
    output reg  [3:0]  native_trig_group_out = 4'd0
);

    // ── ×16÷25 頻率換算（aurora_clk=156.25MHz -> dac_clk=100MHz 標稱
    // 比例，2026-07-23 之前 au_trig_delay 機制用過的同一個換算比例，
    // 這次只是換到不同的目的時脈）。×16 是左移，÷25 用逐次減法（沿用
    // aurora_ctrl_channel.v 既有 per_hop_value 除法器的寫法/風格，
    // quasi-static 不趕時間，不用快速除法器）。change-detect 觸發：
    // delay_aurora_cycles 變了才重新算一次，中間這段時間繼續沿用上一次
    // 算好的 delay_dac_cycles。────────────────────────────────────────
    reg [36:0] delay_aurora_cycles_last;
    reg [40:0] div_remainder;
    reg [31:0] div_quotient;
    reg        div_running;
    reg [31:0] delay_dac_cycles;

    always @(posedge dac_clk) begin
        if (dac_rst) begin
            delay_aurora_cycles_last <= 37'd0;
            div_remainder    <= 41'd0;
            div_quotient     <= 32'd0;
            div_running      <= 1'b0;
            delay_dac_cycles <= 32'd0;
        end else begin
            if (!div_running && (delay_aurora_cycles != delay_aurora_cycles_last)) begin
                delay_aurora_cycles_last <= delay_aurora_cycles;
                div_remainder <= {4'd0, delay_aurora_cycles} << 4;   // ×16
                div_quotient  <= 32'd0;
                div_running   <= 1'b1;
            end else if (div_running) begin
                if (div_remainder >= 41'd25) begin
                    div_remainder <= div_remainder - 41'd25;
                    div_quotient  <= div_quotient + 32'd1;
                end else begin
                    div_running      <= 1'b0;
                    delay_dac_cycles <= div_quotient;
                end
            end
        end
    end

    // ── 自由跑的 dac_clk cycle 計數器 ────────────────────────────────
    reg [31:0] dac_cycle_cnt;
    always @(posedge dac_clk) begin
        if (dac_rst) dac_cycle_cnt <= 32'd0;
        else         dac_cycle_cnt <= dac_cycle_cnt + 32'd1;
    end

    // ── fire-timestamp 佇列（同時脈域，手刻 circular buffer）────────
    reg [31:0] fire_at_q [0:QUEUE_DEPTH-1];
    reg [3:0]  group_q   [0:QUEUE_DEPTH-1];
    reg [PTR_WIDTH-1:0] wr_ptr, rd_ptr;
    reg [PTR_WIDTH:0]   q_count;   // 0..QUEUE_DEPTH，多 1 bit 才裝得下 QUEUE_DEPTH 本身

    // push：async FIFO 有資料且佇列還沒滿。fifo_rd_en 直接 assign 成
    // push_now（組合邏輯），確保跟 fifo_rd_data 讀取同一拍要求 FIFO
    // 前進（見上方 port 註解）。
    wire push_now = !fifo_rd_empty && (q_count < QUEUE_DEPTH[PTR_WIDTH:0]);
    assign fifo_rd_en = push_now;

    // pop：佇列非空，且目前 cycle 數已經到（或超過）最前面那筆的
    // fire_at。用有號數相減判斷，wraparound-safe（dac_cycle_cnt 32-bit
    // 自由跑，補償延遲相對計數器週期(~42.9秒@100MHz)極小，不會誤判）。
    wire pop_now = (q_count != {(PTR_WIDTH+1){1'b0}}) &&
                   ($signed(dac_cycle_cnt - fire_at_q[rd_ptr]) >= 0);

    always @(posedge dac_clk) begin
        if (dac_rst) begin
            wr_ptr  <= {PTR_WIDTH{1'b0}};
            rd_ptr  <= {PTR_WIDTH{1'b0}};
            q_count <= {(PTR_WIDTH+1){1'b0}};
            native_trig_out       <= 1'b0;
            native_trig_group_out <= 4'd0;
        end else begin
            native_trig_out <= pop_now;
            if (pop_now) native_trig_group_out <= group_q[rd_ptr];

            if (push_now) begin
                fire_at_q[wr_ptr] <= dac_cycle_cnt + delay_dac_cycles;
                group_q[wr_ptr]   <= fifo_rd_data;
                wr_ptr <= wr_ptr + {{(PTR_WIDTH-1){1'b0}}, 1'b1};
            end
            if (pop_now) rd_ptr <= rd_ptr + {{(PTR_WIDTH-1){1'b0}}, 1'b1};

            case ({push_now, pop_now})
                2'b10:   q_count <= q_count + {{PTR_WIDTH{1'b0}}, 1'b1};
                2'b01:   q_count <= q_count - {{PTR_WIDTH{1'b0}}, 1'b1};
                default: q_count <= q_count;
            endcase
        end
    end

endmodule
`default_nettype wire
