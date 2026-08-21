`timescale 1ns/1ps
`default_nettype none

// group_trig_select.v — Group-based Trigger 架構（2026-07-27 新增）
//
// 純組合邏輯 4:1 mux：native_trig_cdc_0/dst_pulse（跨域後的觸發 pulse
// 本身，全板共用同一顆）跟 native_trig_group_cdc_0/dst_out（跨域後的
// 4-bit group_select，全板共用同一份）到這裡才第一次跟「這個模組屬於
// 哪個 group」（group_id，來自 board_cfg_reg_0/group_id_$ch，每個模組
// 各自不同）交會，決定這個模組這次要不要真的動作。
//
// create_bd.tcl 會 instantiate 4 個實例（group_trig_select_a/b/c/d，
// A~D 對應 wctrl_0..3/port A-D），每個模組（含它底下的 wctrl_$ch/
// sine_ctrl_regs 對應 channel/amp_ramp_gen 對應 channel）都接自己那個
// 實例的 trig_out，取代原本全部直接接 native_trig_cdc_0/dst_pulse 的
// 接法。
//
// group_select/group_id 都是 dac_clk domain 的 level 訊號，在
// trig_pulse 抵達的那一拍已經穩定（見 aurora_ctrl_channel.v
// native_trig_group_r/PROJECT.md「CDC 設計決策」小節的時序分析），
// 這裡不需要也不做任何額外同步。

module group_trig_select (
    input  wire [3:0] group_select,  // dac_clk domain，已跨域穩定，這次要 fire 哪些 group
    input  wire       trig_pulse,    // dac_clk domain，native_trig_cdc_0/dst_pulse
    input  wire [1:0] group_id,      // 這個模組屬於哪個 group（board_cfg_reg_0/group_id_$ch）
    output wire       trig_out       // 這個模組實際收到的觸發 pulse
);

    assign trig_out = trig_pulse & group_select[group_id];

endmodule
`default_nettype wire
