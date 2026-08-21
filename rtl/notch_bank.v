`timescale 1ns/1ps
`default_nettype none

// notch_bank.v -- 2026-08-18 新增（取代已刪除的 notch_sweep_ctrl.v 自動
// 掃頻設計，改版為固定多頻抵銷，見 PROJECT.md 2026-08-18 對應章節）。
//
// 包 10 組 notch_sine_gen（完全原封不動複用），每組各自的目標頻率
// （W_k/P_k，host 端 Python 算好）由 aurora_ctrl_mux.v 打包成 3 條寬
// bus 送進來（經過 create_bd.tcl 的 level_cdc），寫哪組就更新哪組，
// 之後硬體持續輸出，不會自動變動——沒設定的組別 amp=0，加總後貢獻
//自然是 0，不需要額外的 per-slot enable。
//
// 10 組輸出加總 + 飽和保護，對外只輸出一個 notch_sum_value，讓
// dac_output_mux.v 完全不用改（接的還是原本那個單一 notch_value_ch1
// port，只是來源換成這裡的 notch_sum_value）。
//
// Pipeline latency：跟 notch_sine_gen.v 完全一樣，2 個 dac_clk cycle
// （phase_acc 組合邏輯 -> lut_val_r +1 cycle -> notch_value +1
// cycle）。加總/飽和刻意用純組合邏輯、不額外插暫存器，否則會跟
// dac_output_mux.v 既有的 2-cycle 延遲線對不齊（見該檔案 PORTS.md
// 對應章節「Pipeline latency」說明）。

module notch_bank #(
    parameter LUT_FILE = "sine_lut_16384.mem"
) (
    input  wire        dac_clk,
    input  wire        rst,

    // ── 來自 aurora_ctrl_mux.v（經 create_bd.tcl 的 level_cdc，已經在
    // dac_clk domain）─────────────────────────────────────────────────
    input  wire [319:0] notch_w_all,    // 10 x 32-bit，slot i = [32*i +: 32]
    input  wire [319:0] notch_p_all,    // 10 x 32-bit
    input  wire [179:0] notch_amp_all,  // 10 x 18-bit，slot i = [18*i +: 18]

    // ── 同步訊號（來自 waveform_controller.v，跟 10 組共用同一份）─────────
    input  wire        reload_pulse,    // = dac_samp_idx_wrap
    input  wire        step_en,         // = dac_valid
    input  wire        dac_live,

    output wire signed [15:0] notch_sum_value
);

    wire signed [15:0] notch_value [0:9];

    genvar gi;
    generate
        for (gi = 0; gi < 10; gi = gi + 1) begin : gen_notch
            notch_sine_gen #(.LUT_FILE(LUT_FILE)) u_notch (
                .dac_clk(dac_clk), .rst(rst),
                .tuning_word_active(notch_w_all[32*gi +: 32]),
                .phase_seed_active(notch_p_all[32*gi +: 32]),
                .amp_stage(notch_amp_all[18*gi +: 18]),
                .reload_pulse(reload_pulse),
                .step_en(step_en),
                .dac_live(dac_live),
                .notch_value(notch_value[gi])
            );
        end
    endgenerate

    // 10 組加總（最壞情況 10 x ±32767，需要 20-bit 才不會溢位）+
    // 飽和保護，純組合邏輯，理由見檔頭 Pipeline latency 說明。
    wire signed [19:0] notch_sum_wide =
        notch_value[0] + notch_value[1] + notch_value[2] + notch_value[3] +
        notch_value[4] + notch_value[5] + notch_value[6] + notch_value[7] +
        notch_value[8] + notch_value[9];

    assign notch_sum_value =
        (notch_sum_wide >  20'sd32767) ? 16'sd32767  :
        (notch_sum_wide < -20'sd32768) ? -16'sd32768 :
        notch_sum_wide[15:0];

endmodule
`default_nettype wire
