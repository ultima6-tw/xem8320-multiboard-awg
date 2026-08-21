`timescale 1ns/1ps
`default_nettype none

// fifo_event_reader.v -- 2026-08-04 新增（trigger group 排程功能三部曲
// 「C」CDC 修法）。
//
// 目的：把一顆 FWFT（First Word Fall Through）async fifo_generator 的
// 讀出端，轉成「單週期 pulse + 同一拍有效的資料」這個下游（OR gate →
// 既有 au_trig_pulse_cdc_0/au_trig_group_cdc_0 那條路徑）預期的介面。
//
// 背景：group_trig_scheduler_0 的 group_select_out 是只維持 1 個
// dac_clk 週期的瞬態訊號（配合 fire pulse 才有效），原本用 level_cdc
// （純多級同步器）跨到 sys_clk，被 Windows 端上機測試 + 獨立覆核抓到
// 這是錯的 CDC 選型——level_cdc 沒有機制保證正確捕捉窄脈衝，這個
// 專案自己在 2026-07-30 就修過幾乎一樣的問題（aurora_ctrl_channel_0/
// trig_fire_group，見 create_bd.tcl trig_fire_fifo_0 建立處註解），
// 修法是不用 level_cdc，改成跟 pulse 一起塞進同一筆 FIFO entry 安全
// 跨域。這次照抄同一個模式，這個模組就是 trig_fire_fifo_0 讀出端
// （rtl/dac_trig_queue.v 的 push_now/fifo_rd_en 那段邏輯）抽出來的
// 通用版本——因為這次讀出端不需要像 dac_trig_queue.v 那樣維護一個
// 深度 8 的 timestamp 佇列，單純「有資料就讀出來轉成一拍 pulse」就
// 夠用，不需要重複整個 dac_trig_queue.v 的複雜度。
//
// FWFT 特性：fifo_empty=0 時 fifo_dout 已經是組合邏輯有效值，跟
// dac_trig_queue.v 的既有手法一致，同一拍 assert fifo_rd_en 就能
// 同時把 fifo_dout 正確鎖存進暫存器（不需要額外的 1-cycle 對齊邏輯）。
// 假設事件不會每個 clk cycle 都連續發生（trigger group 排程事件本來
// 就是低頻），逐拍清空 FIFO 已經足夠快，不需要更複雜的節流機制。

module fifo_event_reader #(
    parameter WIDTH = 4
)(
    input  wire             clk,

    // FWFT fifo_generator 讀出端（Independent_Clocks，rd_clk 接同一個
    // clk）
    input  wire [WIDTH-1:0] fifo_dout,
    input  wire             fifo_empty,
    output wire             fifo_rd_en,

    // 轉換後的輸出：event_pulse 為 1-cycle pulse，event_data 跟它同一拍
    // 有效（event_pulse=0 時 event_data 維持上次的值，不清零——下游只
    // 在 event_pulse=1 那一拍取用 event_data，沿用既有 OR-安全設計
    // 慣例不需要，因為這裡下游接的是 OR gate + pulse CDC 組合，只有
    // pulse 真正發生那一拍才會被採樣）
    output reg               event_pulse,
    output reg  [WIDTH-1:0]  event_data
);

    wire pop = !fifo_empty;
    assign fifo_rd_en = pop;

    always @(posedge clk) begin
        event_pulse <= pop;
        if (pop)
            event_data <= fifo_dout;
    end

endmodule
`default_nettype wire
