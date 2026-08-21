`timescale 1ns/1ps
`default_nettype none

// ext_clk_freq_counter — 量測 ext_clk_out(見 ext_dac_clk_ibuf.v)實際
// 頻率用的除錯模組。2026-07-11 新增，見 PROJECT.md 待辦事項第一項。
//
// 做法：在 ext_clk 域跑一個 free-running 32-bit counter，用官方
// xpm_cdc_gray（CDC 用途，計數器/連續遞增值的標準同步做法，符合專案
// 「跨時脈域優先用官方 IP」的原則，不手寫 toggle+2-flop）把值同步到
// sys_clk 域，host 端連續讀兩次（間隔已知的時間，例如 1 秒），用差值
// 除以時間間隔算出 ext_clk 實際頻率——不需要精準的單一 cycle 同步，
// 只需要粗略的頻率量測，xpm_cdc_gray 的 gray code 特性保證讀到的值
// 一定是遞增序列中的某個有效值，不會因為跨域取樣到過渡態而錯亂。
//
// ext_clk 有沒有真的存在/穩定，在硬體到位前未知，所以這個 counter
// 沒有 reset（沒有已知安全的 reset 來源可以跨進一個可能不穩定的
// 外部時脈域），開機後直接自由遞增，只看兩次讀值的差。

module ext_clk_freq_counter (
    input  wire        ext_clk,        // 待量測的外部時脈（ext_dac_clk_ibuf_0/ext_clk_out）

    input  wire        sys_clk,
    output wire [31:0] freq_count_sync // sys_clk 域讀到的 counter 值（WireOut 用）
);

    reg [31:0] cnt = 32'd0;
    always @(posedge ext_clk) begin
        cnt <= cnt + 32'd1;
    end

    xpm_cdc_gray #(
        .DEST_SYNC_FF   (4),
        .INIT_SYNC_FF   (0),
        .REG_OUTPUT     (1),
        .SIM_ASSERT_CHK (0),
        .SIM_LOSSLESS_GRAY_CHK (0),
        .WIDTH          (32)
    ) u_cdc_gray (
        .src_clk      (ext_clk),
        .src_in_bin   (cnt),
        .dest_clk     (sys_clk),
        .dest_out_bin (freq_count_sync)
    );

endmodule
`default_nettype wire
