`timescale 1ns/1ps
`default_nettype none

// trig_timer v2 — hardware trigger sequencer with interval list + armed mode
//
// 2026-07-14（step-16 fork，原檔案共用自 awg-test-step-14/rtl/）：新增
// reinit_req，跟 waveform_controller.v 的 reinit_req 同一套「初始化」
// 指令一起觸發（跟單純的 play_en=0 播放停止不同，不要混淆），清空這個
// port 的 trigger list（mem[]，8 個 interval slot 全部歸零）並強制
// 停止/解除 armed 狀態，不管當下 run 這個 WireIn 還停在什麼值——見
// PROJECT.md「stop/reset 需求」章節。
//
// Armed mode (new): run rising edge → armed state; waits for first_trigger pulse.
// On first_trigger: starts from slot 0, counts down mem[0] cycles, fires trigger_out,
// advances to slot 1, counts down mem[1], etc.
// run falling edge: immediately stops (clears armed and running).
//
// mem[] has 16 slots of 32-bit intervals (sys_clk cycles, 100 MHz = 10 ns
// resolution), all individually addressable/writable via list_wr_slot
// ([3:0], 0-15).
//
// 2026-08-04 修復（原本記在這裡的一則查證，同日修好）：list_wr_slot/
// list_depth/current_idx were only [2:0] (3 bits, 8 slots) while mem[]
// already had 8 entries — depth=8 couldn't be encoded (needed a 4th bit),
// so mem[7] was writable but the playback logic
// (`current_idx + 1 < list_depth`) could never reach it. Widened
// list_wr_slot/current_idx to [3:0] (0-15, matches mem[]'s 16 entries)
// and mem[] to 16 entries. **list_depth widened to [4:0], not [3:0]**
// — widening to only 4 bits would have reproduced the exact same class
// of bug one boundary later (4 bits maxes at 15, one short of the 16
// needed to make mem[15] reachable); 5 bits can represent up to 16 so
// all 16 slots are genuinely usable. This ripples into
// aurora_ctrl_mux.v's shared au_timer_ctrl_depth/out_timer_depth_$port
// (all 4 ports, not just this module) + local_reg_handler.v's
// T_TIMER_CTRL decode + the timer_depth_cdc_$port CDC width in
// create_bd.tcl, re-synthesis required — see PROJECT.md trigger group
// 排程功能三部曲「B」條目. Effective usable range is now the full 1-16.
// loop_en=1: wrap back to slot 0 after last entry.
// loop_en=0: stop after last entry; re-toggle run to restart.
module trig_timer (
    input  wire        clk,
    input  wire        rst,

    // List write (clk domain, from TI strobe)
    input  wire [3:0]  list_wr_slot,
    input  wire [31:0] list_wr_intv,
    input  wire        list_wr_en,

    // Control (WireIn levels, quasi-static)
    input  wire [4:0]  list_depth,   // number of active entries, 1-16; 0 = disabled（2026-08-04 3-bit→5-bit——4-bit 仍不夠表示 16，會讓 mem[15] 死格，跟原本 bug 同一種性質，見上方 header comment；list_wr_slot/current_idx 維持 4-bit，index 0-15 已足夠）
    input  wire        run,          // rising edge → armed; falling edge → stop
    input  wire        loop_en,
    input  wire        first_trigger, // 1-cycle pulse in clk domain: start the sequence

    // reinit_req（2026-07-14 新增）：single-cycle pulse，清空 mem[]（trigger
    // list）、強制停止/解除 armed，不管 run 當下是什麼值
    input  wire        reinit_req,

    // Status
    output reg  [3:0]  current_idx,
    output reg         running,

    // Trigger output (1-cycle pulse, clk domain)
    output reg         trigger_out
);
    integer ti;
    reg [31:0] mem [15:0];
    reg [31:0] countdown;
    reg        run_r;
    reg        armed;   // run=1 but waiting for first_trigger

    always @(posedge clk) begin
        trigger_out <= 1'b0;
        run_r       <= run;

        if (reinit_req) begin
            for (ti = 0; ti < 16; ti = ti + 1)
                mem[ti] <= 32'd0;
        end else if (list_wr_en)
            mem[list_wr_slot] <= list_wr_intv;

        if (rst) begin
            running     <= 1'b0;
            armed       <= 1'b0;
            countdown   <= 32'd0;
            current_idx <= 4'd0;
            run_r       <= 1'b0;
        end else if (reinit_req) begin
            running     <= 1'b0;
            armed       <= 1'b0;
            countdown   <= 32'd0;
            current_idx <= 4'd0;
        end else begin
            // Rising edge of run → arm the timer
            if (run && !run_r && list_depth != 5'd0) begin
                armed   <= 1'b1;
                running <= 1'b0;
            end
            // Falling edge of run → stop and disarm
            else if (!run) begin
                running <= 1'b0;
                armed   <= 1'b0;
            end
            // Armed + first_trigger → start running from slot 0
            else if (armed && first_trigger) begin
                armed       <= 1'b0;
                running     <= 1'b1;
                current_idx <= 4'd0;
                countdown   <= mem[0];
            end
            // Active countdown
            else if (running) begin
                if (countdown <= 32'd1) begin
                    trigger_out <= 1'b1;
                    // {1'b0,current_idx}+5'd1 明確用 5-bit 做比較，跟
                    // list_depth 同寬——current_idx 本身維持 4-bit
                    // 累加（current_idx<=current_idx+4'd1）是安全的，
                    // 這個分支只有在 current_idx<=14 時才會被走到
                    // （current_idx=15 時 15+1=16，只有 list_depth=16
                    // 才會 < 判斷為真，但 5'd16 已經是 list_depth 的
                    // 最大合法值，不會 >16，所以到不了這個分支，
                    // current_idx 本身不會累加溢位）
                    if ({1'b0, current_idx} + 5'd1 < list_depth) begin
                        countdown   <= mem[current_idx + 4'd1];
                        current_idx <= current_idx + 4'd1;
                    end else if (loop_en) begin
                        countdown   <= mem[0];
                        current_idx <= 4'd0;
                    end else begin
                        running <= 1'b0;
                    end
                end else begin
                    countdown <= countdown - 32'd1;
                end
            end
        end
    end

endmodule
`default_nettype wire
