`timescale 1ns/1ps
`default_nettype none

// aurora_dir_switch.v -- Step 15a Layer 1: bidirectional relay switch
//
// 2026-07-05 首版草稿（DRAFT，尚未模擬/上板驗證）。這是「拆成幾層 RTL」
// 計畫的第 1 層：只驗證「雙向環路的封包能不能正確流動」這件事本身，
// 不含 16-beat chunk 的鎖定/解鎖邏輯（data channel，第 2 層）、不含
// 開機編號/trigger/data 預約協定的逐站修改內容邏輯（ctrl channel，
// 第 3 層）。
//
// 結構：直接實例化兩份 aurora_ingress.v（每一側各一份），把其中一側解出
// 的「需要轉送」的封包，接到**另一側**的 TX（環路 relay 的基本模式：
// 「從哪一邊進來，就從另一邊出去」）：
//   side0（例如 aurora_64b66b_0/SFP1）進來的 relay 封包 -> side1 的 TX 送出
//   side1（例如 aurora_64b66b_1/SFP2）進來的 relay 封包 -> side0 的 TX 送出
// 兩側解出「給本地」的封包合併成同一份輸出（給既有 async_fifo_aurora_rx）。
//
// 這一層**不含**本板自己要發起封包的邏輯（本板 FP 資料、本板要發起的
// 控制封包）——tx0/tx1 目前只有 relay（轉送）流量，本地發起的注入是
// 上層（data/ctrl channel）模組的職責。

module aurora_dir_switch (
    input  wire        aurora_clk,
    input  wire        rst,
    input  wire [15:0] board_id,

    // ── Side 0（例如 aurora_64b66b_0/SFP1 的 m_axi_rx_*）─────────────────
    input  wire [63:0] rx0_tdata,
    input  wire        rx0_tvalid,
    output wire [63:0] tx0_tdata,
    output wire        tx0_tvalid,
    output wire        tx0_tlast,

    // ── Side 1（例如 aurora_64b66b_1/SFP2 的 m_axi_rx_*）─────────────────
    input  wire [63:0] rx1_tdata,
    input  wire        rx1_tvalid,
    output wire [63:0] tx1_tdata,
    output wire        tx1_tvalid,
    output wire        tx1_tlast,

    // ── 合併後的本地端輸出（接既有 async_fifo_aurora_rx/din,wr_en）──────
    output wire [63:0] local_tdata,
    output wire        local_tvalid,

    output wire        dbg_overflow_0,
    output wire        dbg_overflow_1
);

    wire [63:0] local0_tdata, local1_tdata;
    wire        local0_tvalid, local1_tvalid;

    aurora_ingress ing0 (
        .clk          (aurora_clk),
        .rst          (rst),
        .board_id     (board_id),
        .rx_tdata     (rx0_tdata),
        .rx_tvalid    (rx0_tvalid),
        .local_tdata  (local0_tdata),
        .local_tvalid (local0_tvalid),
        .relay_tdata  (tx1_tdata),
        .relay_tvalid (tx1_tvalid),
        .relay_tlast  (tx1_tlast),
        .dbg_overflow (dbg_overflow_0)
    );

    aurora_ingress ing1 (
        .clk          (aurora_clk),
        .rst          (rst),
        .board_id     (board_id),
        .rx_tdata     (rx1_tdata),
        .rx_tvalid    (rx1_tvalid),
        .local_tdata  (local1_tdata),
        .local_tvalid (local1_tvalid),
        .relay_tdata  (tx0_tdata),
        .relay_tvalid (tx0_tvalid),
        .relay_tlast  (tx0_tlast),
        .dbg_overflow (dbg_overflow_1)
    );

    // 合併兩側的本地端輸出。已知簡化/風險：這裡用簡單的「side0 優先」
    // 選擇，假設同一個 cycle 不會兩側同時都有「給本地」的封包在輸出
    // （目前流量模式下：data 固定單一方向、大部分控制封包也是固定方向
    // 或明確的 unicast，兩側同時撞在同一拍的機率低，但沒有嚴謹證明）。
    // 如果之後發現會撞（例如兩個方向同時有東西要交本地），這裡需要改成
    // 真正的小型 round-robin 仲裁，不能繼續用這個簡單 mux。
    assign local_tdata  = local0_tvalid ? local0_tdata : local1_tdata;
    assign local_tvalid = local0_tvalid || local1_tvalid;

endmodule
`default_nettype wire
