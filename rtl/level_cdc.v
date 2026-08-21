`timescale 1ns/1ps
`default_nettype none

// level_cdc: 薄 wrapper，包住 Xilinx 官方 xpm_cdc_array_single macro，跟
// trigger_cdc.v（包 xpm_cdc_pulse）同一套模式，差別是這個包的是「準穩態多
// bit 訊號」（設定值/狀態值，不要求多 bit 同時原子性更新），不是單拍 pulse。
//
// 2026-07-06：aurora_ctrl_channel.v/aurora_data_channel.v 檔頭註解明確假設
// board_id/is_master/reserve_dest_id（sys_clk -> aurora_clk）跟
// init_ok/total_boards/board_index/reserve_ok/reserve_busy（aurora_clk ->
// sys_clk）這些跨域訊號「外部已經用官方 XPM 巨集同步」，但 create_bd.tcl
// 接線時漏掉了（直接接線，完全沒有 CDC），這裡補上。channel_up（aurora_clk
// -> sys_clk 給 WO/LED 讀）也是同樣缺口，一併用這個模組補。
//
// SRC_INPUT_REG=1：來源端多加一級輸入暫存器（xlslice/暫存器輸出本身已經是
// sys_clk/aurora_clk 同步訊號，加這級是官方巨集的建議預設值，用來過濾潛在
// 的組合邏輯毛刺，不是必要但符合官方建議用法）。

module level_cdc #(
    parameter WIDTH = 1
)(
    input  wire             src_clk,
    input  wire [WIDTH-1:0] src_in,
    input  wire             dst_clk,
    output wire [WIDTH-1:0] dst_out
);

    xpm_cdc_array_single #(
        .DEST_SYNC_FF   (4),
        .INIT_SYNC_FF   (0),
        .SIM_ASSERT_CHK (0),
        .SRC_INPUT_REG  (1),
        .WIDTH          (WIDTH)
    ) cdc_inst (
        .dest_out (dst_out),
        .dest_clk (dst_clk),
        .src_clk  (src_clk),
        .src_in   (src_in)
    );

endmodule
`default_nettype wire
