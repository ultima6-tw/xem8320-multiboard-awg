`timescale 1ns/1ps
`default_nettype none

// aurora_user_clk_buf — tx_out_clk → BUFG_GT → user clock tree
// Aurora 64B/66B 在 BD 中 user_clk / sync_clk 為 INPUT，需外部 BUFG_GT 緩衝後回饋。
// bufg_gt_clr_out 由 Aurora IP 控制，GT 初始化期間清除 BUFG_GT。
//
// 2026-07-08 (step 15b) local copy（原本共用 awg-test-step-6/vivado/rtl/
// 的檔案）：線速率從 1.25Gbps 改成 1.0Gbps（DAC 線材規格「10/1 Gbps」，
// 不支援 1.25Gbps），FREQ_HZ 屬性要跟著改，不能沿用共用檔案裡寫死的
// 19531250（1.25Gbps 那組數字）——Vivado BD 的 clock frequency
// propagation 會拿這個屬性去跟 Aurora IP 本身依線速率算出來的期望值
// 比對，兩者對不上會讓 generate_target 直接報錯（user_clk/sync_clk
// configured for 15625000 Hz but input frequency is 19531250 Hz）。
//
// 2026-07-17：線速率改成 10.0Gbps（透過 Vivado GUI 正確設定
// aurora_64b66b_0 的 Shared Logic=in core、aurora_64b66b_1 維持 in
// example design，兩者共用 QPLL，見 PROJECT.md 2026-07-17 章節）。
// aurora_64b66b_0 現在自己內部處理 clock buffering（SupportLevel=1，
// 不再使用 aurora_user_clk_buf_0，那個 cell 目前是孤兒，還沒清除）；
// aurora_64b66b_1 維持外部緩衝模式，還是要用這個模組
// （aurora_user_clk_buf_1），FREQ_HZ 跟著 line rate 改成 156250000
// （= 10Gbps / 64 = 156.25MHz），不然 validate_bd_design 會報
// user_clk/sync_clk 頻率宣告值（這個屬性寫死的舊值）跟 aurora_64b66b_1
// 實際期望值（依它自己的 C_LINE_RATE 算出來）對不上。

module aurora_user_clk_buf (
    input  wire clk_in,  // Aurora tx_out_clk
    input  wire clr,     // Aurora bufg_gt_clr_out
    (* X_INTERFACE_INFO      = "xilinx.com:signal:clock:1.0 clk_out CLK" *)
    (* X_INTERFACE_PARAMETER = "FREQ_HZ 156250000" *)
    output wire clk_out  // 驅動 user_clk / sync_clk / RTL aurora_clk（156.25 MHz = 10.0 Gbps / 64）
);
    BUFG_GT bufg (
        .I       (clk_in),
        .CE      (1'b1),
        .CEMASK  (1'b0),
        .CLR     (clr),
        .CLRMASK (1'b0),
        .DIV     (3'b000),
        .O       (clk_out)
    );
endmodule

`default_nettype wire
