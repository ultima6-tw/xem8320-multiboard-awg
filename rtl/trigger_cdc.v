`timescale 1ns/1ps
`default_nettype none

// trigger_cdc: 薄 wrapper，包住 Xilinx 官方 xpm_cdc_pulse macro，暴露
// 跟原本自己寫的 trigger_cdc.v 完全一樣的 port 名稱（src_clk/src_pulse/
// dst_clk/dst_pulse），所以 create_bd.tcl 完全不用改，直接換掉底層實作。
//
// 2026-07-04：依使用者要求，盡量用官方驗證過的 IP 取代自己手寫的 CDC 邏輯。
// 原本的實作（toggle in src domain + 2-flop sync + edge-detect）邏輯上是標準
// 教科書寫法，跟 xpm_cdc_pulse 內部原理相同，但沒有正式測試過；
// xpm_cdc_pulse 是矽驗證過的官方巨集，用來排除／取代自製版本的風險，也方便
// 之後給別人看時，用的是業界通用、可辨識的官方元件。
//
// 2026-07-08：新增 src_rst/dst_rst 兩個 port，RST_USED 改成 1、
// INIT_SYNC_FF 改成 1。原本兩個 reset port 都固定接 1'b0（等於完全沒有
// reset，只能靠上電時的暫存器初值），懷疑這是 reserve 協定 CDC
// （au_reserve_pulse_cdc_0）偶爾丟脈波、且同一次開機內結果一致（一旦
// 內部同步狀態被某次開機瞬間的訊號污染就卡住到當次開機結束、沒有任何
// 機制能恢復）的根因之一——信心約 55-60%，還沒有 ILA 波形直接證實，是
// 合理推論但非定論。這是 port list 的介面擴充，所有既有 instantiation
// 都要明確接上這兩個新 port；沒有實際 reset 需求的呼叫端明確接 1'b0，
// 效果跟改動前完全相同，只有 Aurora 協定相關的 4 個 CDC 才接上真正的
// reset，把改動風險侷限在跟這次 bug 相關的範圍（見 create_bd.tcl）。

module trigger_cdc (
    input  wire src_clk,
    input  wire src_rst,
    input  wire src_pulse,
    input  wire dst_clk,
    input  wire dst_rst,
    output wire dst_pulse
);

    xpm_cdc_pulse #(
        .DEST_SYNC_FF   (4),
        .INIT_SYNC_FF   (1),
        .REG_OUTPUT     (0),
        .RST_USED       (1),
        .SIM_ASSERT_CHK (0)
    ) cdc_inst (
        .src_clk    (src_clk),
        .src_pulse  (src_pulse),
        .dest_clk   (dst_clk),
        .src_rst    (src_rst),
        .dest_rst   (dst_rst),
        .dest_pulse (dst_pulse)
    );

endmodule
`default_nettype wire
