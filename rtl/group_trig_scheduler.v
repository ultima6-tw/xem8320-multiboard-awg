`timescale 1ns/1ps
`default_nettype none

// group_trig_scheduler.v -- 2026-08-04 新增（trigger group 排程功能
// 三部曲「C」，見 PROJECT.md「目前狀態」對應章節）。
//
// 目的：讓一片板子能照排程「在什麼時間廣播哪一個 trigger group」，取代
// 手動逐次呼叫 /api/trigger。仿 rtl/trig_timer.v 架構（同一套 16-slot
// interval list + armed/running 狀態機骨架），跟 trig_timer.v 的兩個
// 差異：
//   1. 每個 slot 多帶一個 group_select（4-bit），倒數完成時輸出
//      fire（1-cycle pulse）+ group_select_out（同一拍有效），而不是
//      單純的 trigger_out——下游在 BD 中把這兩個訊號 OR 進既有
//      local_reg_handler_0/au_trig_start/au_trig_group_select（見
//      create_bd.tcl 對應接線處），完全重用已驗證的多板 PAUSE/ACK/GO
//      握手機制，不用碰 Aurora TX 傳送層。
//   2. run=1 直接開始（current_idx=0，開始倒數 mem_intv[0]），不像
//      trig_timer 那樣先進 armed 狀態等外部 first_trigger——這個模組
//      本身就是要「自己當觸發源」，沒有「等誰先觸發」的需求。
//
// list_depth 是 5-bit（0-16），不是 4-bit：4-bit 只能表示到 15，會讓
// mem[15]（第 16 格）永遠打不到，重演 rtl/trig_timer.v 2026-08-04
// header comment 記錄過的同一種 bug，這次從一開始就用對的寬度，見
// PROJECT.md trigger group 排程功能三部曲「B」條目的完整推導。
// list_wr_slot/current_idx 是 4-bit（index 0-15 已足夠，不需要跟
// list_depth 一樣到 16）。
//
// dac_clk domain（不是 sys_clk）：理由跟當年 trig_timer 搬時脈域一樣
// ——多板排程要避免板間振盪器漂移，倒數本身必須在跟 DAC 輸出同一個
// clock domain 做，靠 dac_clk 頻率一致（外部 Si5332 共用時脈）保證
// 多板排程節奏同步，不是靠 sys_clk（每片板子各自的本地振盪器，會
// 漂移）。
//
// fire/group_select_out 的 OR-安全設計：group_select_out 只在 fire=1
// 那一拍非 0，其餘全部 0（reg 預設值、每個 cycle 開頭都先清零）——這樣
// create_bd.tcl 可以直接用 util_vector_logic OR 把這個模組的
// group_select_out 跟 local_reg_handler_0 既有的 au_trig_group_select
// 合併，不需要額外的 mux，因為沒 fire 的時候這個模組貢獻的值永遠是
// all-zero，不會污染 OR 的結果。
//
// 2026-08-20 新增 arm_mode + first_trigger（多板同步輪播用，見
// PROJECT.md「test2/test3 多板同步」章節、Opus 分析報告）：原本
// run=1 直接開始這件事，本質上是「每次切換都重新走一次跨板 PAUSE_
// REQ/ACK/GO 廣播」這條路徑（Architecture A）——上機實測過（NOTES.md
// 2026-07-14/07-18）這條路徑本身有 trigger-to-trigger jitter（±10-
// 60ns 量級，改版前實測值），固定的延遲補償常數沒辦法消除，因為補償
// 的是「假設的平均延遲」不是「那一次的實際延遲」。
//
// 改法（仿 trig_timer.v 既有、已驗證過的 armed + first_trigger
// pattern，不是新發明）：arm_mode=1 時，run 上升緣只進 armed 狀態，
// 等 first_trigger（接 dac_trig_queue_0/native_trig_out，已經是
// dac_clk domain、已經是這個專案唯一一套「先走一次完整跨板廣播+補償
// 同步起始點」的機制，不需要新的 CDC）才真正開始倒數。同步只發生
// **一次**（起始點），之後每片板子各自在共用的 dac_clk 上倒數
// mem_intv[]，不再需要每次切換都繞一次 Aurora ring——誤差性質從
// 「每次切換都是獨立的隨機 jitter」變成「整輪固定、可校準掉的
// DC offset」。arm_mode=0 維持原本行為（run=1 直接開始，不經過
// armed），給單板/legacy 情境使用，行為完全不變。

module group_trig_scheduler (
    input  wire        clk,
    input  wire        rst,

    // List write (clk domain, from TI strobe 或 Aurora T_GROUP_SCHED_CTRL 解碼)
    input  wire [3:0]  list_wr_slot,
    input  wire [31:0] list_wr_intv,
    input  wire [3:0]  list_wr_group,  // 這個 slot 倒數完成時要廣播哪個 trigger group
    input  wire        list_wr_en,

    // Control (WireIn levels, quasi-static)
    input  wire [4:0]  list_depth,   // number of active entries, 1-16; 0 = disabled（5-bit，見上方檔頭說明）
    input  wire        run,          // rising edge → arm_mode=0 時直接開始；arm_mode=1 時只進 armed，falling edge → stop（兩種模式皆然）
    input  wire        loop_en,
    input  wire        arm_mode,     // 0=legacy（run=1 直接開始，原本行為）1=armed+等 first_trigger（多板同步用，見上方 2026-08-20 說明）
    input  wire        first_trigger, // 1-cycle pulse (clk domain)：arm_mode=1 時，armed 狀態下收到這個 pulse 才真正開始倒數

    // reinit_req：single-cycle pulse，清空 mem_intv[]/mem_group[]（trigger
    // list）、強制停止，不管 run 當下是什麼值（比照 trig_timer.v 同名
    // port 的既有慣例）
    input  wire        reinit_req,

    // Status
    output reg  [3:0]  current_idx,
    output reg         running,

    // Fire output（1-cycle pulse, clk domain）+ group_select（只在對應
    // 那個 fire=1 那一拍非 0，其餘全 0，見上方 OR-安全設計說明）——
    // 2026-08-20 拆成兩個互斥的 fire 訊號，避免 arm_mode 切換時雙重
    // 觸發（下方 always block 裡兩者只會有一個在同一拍被設成 1，由
    // arm_mode 決定，不會同時發生）：
    //   fire       ：arm_mode=0（legacy）時使用，走原本 gsc_fire_fifo_0
    //                → 跨板 PAUSE/ACK/GO 廣播那條路（Architecture A）。
    //   fire_local ：arm_mode=1（多板同步）時使用，直接在 dac_clk 域內
    //                跟 dac_trig_queue_0 的輸出合併（trig_merge.v），
    //                不繞 Aurora ring（Architecture B）。
    // group_select_out 兩種模式共用同一份（在同一個 assignment 分支
    // 設定，兩個 fire 訊號其中一個非 0 時就有效）。
    output reg         fire,
    output reg         fire_local,
    output reg  [3:0]  group_select_out
);
    integer ti;
    reg [31:0] mem_intv  [15:0];
    reg [3:0]  mem_group [15:0];
    reg [31:0] countdown;
    reg        run_r;
    reg        armed;   // arm_mode=1 時：run=1 但還在等 first_trigger（仿 trig_timer.v 同名 reg）

    always @(posedge clk) begin
        fire             <= 1'b0;
        fire_local       <= 1'b0;
        group_select_out <= 4'd0;
        run_r            <= run;

        if (reinit_req) begin
            for (ti = 0; ti < 16; ti = ti + 1) begin
                mem_intv[ti]  <= 32'd0;
                mem_group[ti] <= 4'd0;
            end
        end else if (list_wr_en) begin
            mem_intv[list_wr_slot]  <= list_wr_intv;
            mem_group[list_wr_slot] <= list_wr_group;
        end

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
            // Rising edge of run：arm_mode=0（legacy）直接開始，不經過
            // armed；arm_mode=1（多板同步）只進 armed，等 first_trigger
            // （見上方 2026-08-20 說明）。
            if (run && !run_r && list_depth != 5'd0) begin
                if (arm_mode) begin
                    armed   <= 1'b1;
                    running <= 1'b0;
                end else begin
                    running     <= 1'b1;
                    current_idx <= 4'd0;
                    countdown   <= mem_intv[0];
                end
            end
            // Falling edge of run → 立刻停止（兩種模式皆然）
            else if (!run) begin
                running <= 1'b0;
                armed   <= 1'b0;
            end
            // Armed + first_trigger → 真正開始倒數（仿 trig_timer.v
            // 同一段邏輯）
            else if (armed && first_trigger) begin
                armed       <= 1'b0;
                running     <= 1'b1;
                current_idx <= 4'd0;
                countdown   <= mem_intv[0];
            end
            // Active countdown
            else if (running) begin
                if (countdown <= 32'd1) begin
                    // arm_mode 決定要走哪一條路（見上方 port 宣告處
                    // 說明），兩者互斥，不會同一拍都設成 1。
                    if (arm_mode) fire_local <= 1'b1;
                    else          fire       <= 1'b1;
                    group_select_out <= mem_group[current_idx];
                    if ({1'b0, current_idx} + 5'd1 < list_depth) begin
                        countdown   <= mem_intv[current_idx + 4'd1];
                        current_idx <= current_idx + 4'd1;
                    end else if (loop_en) begin
                        countdown   <= mem_intv[0];
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
