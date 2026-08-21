`timescale 1ns/1ps
`default_nettype none

// sync_start.v -- local copy (not the shared awg-test-step-14/rtl/
// sync_start.v)，2026-08-18，理由/慣例跟同一批新增的 diag_cdc.v
// 完全相同，見該檔案檔頭說明。內容跟原檔完全相同，未做任何邏輯改動。
//
// sync_start — 當所有 active channel 的 active FIFO 都有資料時，鎖存 all_ready。
//
// 解決多 channel 相位差問題：
//   DDR4 SmartConnect 對各 channel reader 依序服務，導致 FIFO 先後填滿。
//   all_ready 保持 0 直到每個有在播放（play_en=1）的 channel 的 active FIFO 都有至少一筆資料，
//   才同時開放所有 channel 的 dac_valid → ZMod AWG 從同一個 sample 開始輸出。
//
// mux_sel[i]=0 → 監聽 fifo_a_empty[i]（FIFO_A 為 active）
// mux_sel[i]=1 → 監聽 fifo_b_empty[i]（FIFO_B 為 active）
//
// 鎖存行為：once all_ready=1，保持 1，直到全部 channel 停播（play_en==0）。
//
// Flush guard：play_en 重新變高後先等 FLUSH_GUARD 個 dac_clk cycle，讓 waveform_controller
//   完成 FIFO flush（清除前次殘留資料），再開始偵測 fifo empty。
//   FLUSH_CYCLES=8 + CDC 延遲 4 cycle = 12，設 FLUSH_GUARD=16 有充足 margin。
//
// Runs on dac_clk (same as wctrl and FIFO read side).

module sync_start #(
    parameter FLUSH_GUARD = 16   // dac_clk cycles to wait after play_en asserts
                                 // must be > wctrl's FLUSH_CYCLES (8) + CDC margin (4)
) (
    input  wire       clk,
    input  wire [3:0] play_en,       // per-channel play enable (quasi-static, from FP WireIn)
    input  wire [3:0] mux_sel,       // per-channel mux_sel (0=FIFO_A active, 1=FIFO_B active)
    input  wire [3:0] fifo_a_empty,  // per-channel FIFO_A read-side empty (dac_clk domain)
    input  wire [3:0] fifo_b_empty,  // per-channel FIFO_B read-side empty (dac_clk domain)
    output reg        all_ready      // 1 = all active ch have data → release DAC
);

// 依 mux_sel 選出每個 channel 的 active FIFO empty 信號
wire [3:0] active_empty = (mux_sel & fifo_b_empty) | (~mux_sel & fifo_a_empty);

// 對每個 channel：play_en=0（不播）→ 不擋；play_en=1 → 必須 active FIFO 有資料
wire [3:0] ch_ok = ~play_en | ~active_empty;
wire       raw   = &ch_ok;  // all 4 channels OK

// Flush guard counter: count up from 0 after play_en asserts
reg [4:0] guard_cnt;   // 5-bit: counts 0..FLUSH_GUARD-1
reg       guard_done;  // 1 after guard_cnt reaches FLUSH_GUARD

always @(posedge clk) begin
    if (play_en == 4'b0000) begin
        all_ready  <= 1'b0;   // 全停播 → 重置，下次 play 重新同步
        guard_cnt  <= 5'd0;
        guard_done <= 1'b0;
    end else begin
        // Phase 1: wait for flush to complete
        if (!guard_done) begin
            if (guard_cnt == FLUSH_GUARD - 1)
                guard_done <= 1'b1;
            else
                guard_cnt <= guard_cnt + 5'd1;
        end
        // Phase 2: once flush is done, release immediately.
        // Active FIFO may be intentionally empty (silence before trigger); standby fills in background.
        else
            all_ready <= 1'b1;   // set-dominant latch：一旦就緒，鎖住
    end
end

endmodule
`default_nettype wire
