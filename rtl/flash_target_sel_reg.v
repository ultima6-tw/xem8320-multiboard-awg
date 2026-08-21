`timescale 1ns/1ps
`default_nettype none

// flash_target_sel_reg — 「這次 flash-save/erase 要動哪個 sector」的持久
// 暫存器（2-bit：0=身份/1=scale_cfg/2=amp_ctrl/3=calib_coef）。
//
// 2026-07-15 新增，Flash「分開儲存」的一部分。host WI 跟 Aurora
// T_FLASH_TARGET_SEL(0x15) 都能設定，優先序比照這次 session 統一採用的
// 「誰晚寫誰生效」規則（fp 跟 au 同一拍撞在一起時 fp 贏，否則誰晚寫誰
// 生效，沒有鎖定旗標，跟 board_id/scale_cfg 一致）。設定完之後值持續
// 保持，直到下一次被覆寫或 rst——因為 host/Aurora 要先設定目標 sector，
// 再送 T_FLASH_WRITE_DATA payload，中間可能間隔好幾拍，不能用 pulse。

module flash_target_sel_reg (
    input  wire       clk,
    input  wire       rst,

    input  wire        fp_wr,
    input  wire [1:0]  fp_sel,

    input  wire        au_wr,
    input  wire [1:0]  au_sel,

    output reg  [1:0]  sel
);

always @(posedge clk) begin
    if (rst) begin
        sel <= 2'd0;   // 預設身份群組
    end else begin
        if (fp_wr)
            sel <= fp_sel;
        else if (au_wr)
            sel <= au_sel;
    end
end

endmodule
`default_nettype wire
