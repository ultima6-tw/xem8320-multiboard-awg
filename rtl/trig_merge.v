`timescale 1ns/1ps
`default_nettype none

// trig_merge.v -- 2026-08-20 新增（多板同步輪播 Architecture B，見
// rtl/group_trig_scheduler.v 檔頭「2026-08-20 新增 arm_mode +
// first_trigger」說明、PROJECT.md「test2/test3 多板同步」章節）。
//
// 目的：把 dac_trig_queue_0（既有、走完整跨板 PAUSE/ACK/GO 廣播+補償
// 延遲那條路徑的觸發輸出）跟 group_trig_scheduler_0/fire_local（
// arm_mode=1 時，本地自主倒數直接觸發，不繞 Aurora ring）合併成一組，
// 接去 4 顆 group_trig_select_$ch，取代原本直接接 dac_trig_queue_0
// 的接法。
//
// 兩端 clock domain 相同（都是 dac_clk），純組合邏輯合併，不需要任何
// CDC——這是 Architecture B 相對 Architecture A 的結構性優勢，A 為了
// 做同一個 clock domain 內的事反而要繞 dac_clk→sys_clk→aurora_clk→
// 環路→aurora_clk→dac_clk 三次非同步跨域。
//
// ⚠️ 不能用單純的 bitwise OR 合併 group 欄位：dac_trig_queue_0/
// native_trig_group_out 是 held level（dac_trig_queue.v 只在 pop_now
// 那一拍更新，其餘時間維持上次的值，不會自動歸零，見該檔案 145-146
// 行），group_trig_scheduler_0/group_select_out 則是 OR-safe（只在
// fire_local=1 那一拍非 0，其餘全 0，見 group_trig_scheduler.v 檔頭
// 說明）——兩者語意不同，直接 OR 會讓 scheduler 本地 fire 那一拍的
// group 被 queue 殘留的舊值污染。改成各自用自己的 pulse 訊號 gate 住
// 再合併，兩邊都只在真正 fire 的那一拍才貢獻非零值，這樣才是安全的
// mux（等效於一個 2:1 mux，用 OR 實作是因為兩個來源保證不會同一拍
// 都是 1——group_trig_scheduler.v 的 arm_mode 是 quasi-static，不會
// 在同一輪輪播中途切換，legacy 模式下 fire_local 永遠是 0，多板同步
// 模式下這個板子理論上不會再收到 dac_trig_queue_0 的 native_trig_out
// ——但仍然保留 OR 而非直接互斥假設，多一層保護）。
module trig_merge (
    input  wire        queue_trig,   // dac_trig_queue_0/native_trig_out
    input  wire [3:0]  queue_group,  // dac_trig_queue_0/native_trig_group_out（held level）
    input  wire        sched_trig,   // group_trig_scheduler_0/fire_local（OR-safe pulse）
    input  wire [3:0]  sched_group,  // group_trig_scheduler_0/group_select_out（OR-safe）

    output wire        trig_out,
    output wire [3:0]  group_out
);

    assign trig_out  = queue_trig | sched_trig;
    assign group_out = (queue_trig ? queue_group : 4'd0) | (sched_trig ? sched_group : 4'd0);

endmodule
`default_nettype wire
