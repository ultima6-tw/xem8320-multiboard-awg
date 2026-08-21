`timescale 1ns/1ps
`default_nettype none

// amp_ctrl_read_mux — 純組合邏輯 8:1 mux，讀回 aurora_ctrl_mux_0/
// out_amp_ctrl_0..7（既有輸出，本來就存在，這裡只是多接一份 tap 出來
// 讀，不影響原本接到 awg_calib_regs_0 的那條路徑）。
//
// 2026-07-15 新增，Group 2（scale_cfg/amp_ctrl/calib_coef 讀回）的一部分。
// sel 沿用 WI 0x19（跟寫入用的 fp_amp_ctrl_sel 是同一個選擇器，讀寫共用
// 同一個「目前選中的 amp_ctrl channel」概念），只取低 3 位（channel 0-7），
// 忽略 bit[3]（那是寫入專用的「廣播到全部 8 個 channel」旗標，讀取沒有
// 對應語意）。

module amp_ctrl_read_mux (
    input  wire [2:0]  sel,
    input  wire [17:0] amp_ctrl_0,
    input  wire [17:0] amp_ctrl_1,
    input  wire [17:0] amp_ctrl_2,
    input  wire [17:0] amp_ctrl_3,
    input  wire [17:0] amp_ctrl_4,
    input  wire [17:0] amp_ctrl_5,
    input  wire [17:0] amp_ctrl_6,
    input  wire [17:0] amp_ctrl_7,
    output reg  [17:0] amp_ctrl_sel_out
);

always @(*) begin
    case (sel)
        3'd0: amp_ctrl_sel_out = amp_ctrl_0;
        3'd1: amp_ctrl_sel_out = amp_ctrl_1;
        3'd2: amp_ctrl_sel_out = amp_ctrl_2;
        3'd3: amp_ctrl_sel_out = amp_ctrl_3;
        3'd4: amp_ctrl_sel_out = amp_ctrl_4;
        3'd5: amp_ctrl_sel_out = amp_ctrl_5;
        3'd6: amp_ctrl_sel_out = amp_ctrl_6;
        default: amp_ctrl_sel_out = amp_ctrl_7;
    endcase
end

endmodule
`default_nettype wire
