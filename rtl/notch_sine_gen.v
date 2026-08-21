`timescale 1ns/1ps
`default_nettype none

// notch_sine_gen.v -- 2026-08-18 新增。IFFT 梳狀波形＋即時掃頻相減功能
// 的正弦波產生器，架構直接照抄 sine_gen.v 的相位累加器＋LUT 設計（見
// 該檔案完整註解），差異只有一點：sine_gen.v 在 trig_start 那個單一
// 事件才重新載入相位（給多板同步用），這裡改成**每次 reload_pulse**
// （來自 waveform_controller.v 的 dac_samp_idx_wrap，即梳狀波形每次
// 播完一輪繞回開頭那一刻）就重新載入——因為要抵銷的諧波在梳狀波形裡
// 有一個 Schroeder phase 設計（非零、per-harmonic 固定相位偏移，見
// host/multitone_scan.py precompute_full_sum()），如果讓這裡的相位
// 累加器自由跑、只在某個外部 trigger 才對齊一次，長期下來/切換諧波時
// 都會跟梳狀波形本身的相位對不上，抵銷不乾淨（詳見 PROJECT.md
// 2026-08-18 對應章節的完整討論，含 Opus 覆核）。「每個播放週期都
// 重新對齊一次」保證絕對不會累積漂移。
//
// 振幅：sine_gen.v 原本直接把 16-bit LUT 值四捨五入成 14-bit 送出（跟
// DAC 滿幅一樣大）。這裡要抵銷的單一諧波振幅遠小於滿幅（實測
// multitone 梳狀波形整體 A≈207 codes，見 PROJECT.md 分析），如果先
// round 到 14-bit 才縮放，等於先把解析度砍到只剩 14-bit 才處理一個
// 本來就只需要小振幅的訊號，浪費精度。這裡改成直接用 16-bit LUT 值
// 乘上 Q1.16 格式的振幅縮放（amp_stage，跟既有 amp_ctrl_0..7/
// amp_ramp_gen.v 的 amp_out 同一種格式慣例），縮放後才是真正要拿去
// 相減的 16-bit 有號值。
//
// Pipeline latency（2026-08-18，跟 dac_output_mux.v 的相減邏輯配合時
// 極重要）：phase_acc（組合邏輯算 lut_addr）-> lut_val_r（+1 cycle，
// BRAM 同步讀取延遲，跟 sine_gen.v 完全一樣）-> notch_value（+1
// cycle，振幅乘法器）。總延遲 = 2 個 dac_clk cycle。dac_output_mux.v
// 那邊的 ddr_data/ddr_valid 必須用同樣長度的延遲線對齊，才不會相減到
// 不同時間點的樣本——這是掃頻抵銷乾不乾淨的關鍵，差 1 個 cycle 在高
// 諧波（k=1000）就會少掉約 30dB 的抵銷深度（Opus 覆核估算）。

module notch_sine_gen #(
    parameter LUT_ADDR_WIDTH = 14,   // 跟 sine_gen.v 共用同一份 LUT 檔案/位址寬度
    parameter LUT_FILE       = "sine_lut_16384.mem"
) (
    input  wire        dac_clk,
    input  wire        rst,

    // ── 目前作用中的諧波參數（來自 notch_sweep_ctrl.v，dac_clk domain，
    // 準靜態，reload_pulse 那一拍才真正被採用）───────────────────────────
    input  wire [31:0] tuning_word_active,  // W_k：這個諧波的相位累加步進量
    input  wire [31:0] phase_seed_active,   // P_k：這個諧波在梳狀波形裡的固定相位偏移（已含 cos->sin 換算的 +1/4 turn）
    input  wire [17:0] amp_stage,           // Q1.16 有號振幅縮放（跟 amp_ctrl_0..7 同一種格式）

    // ── 同步訊號（來自 waveform_controller.v）─────────────────────────────
    input  wire        reload_pulse,        // = dac_samp_idx_wrap，梳狀波形每播完一輪就對齊一次相位
    input  wire        step_en,             // = dac_valid，見下方 phase_acc 累加條件的說明
    input  wire        dac_live,            // 安全閘控：非播放狀態時輸出強制 0（見下方）

    output wire signed [15:0] notch_value   // 縮放後的抵銷值，跟 ddr_data 對齊延遲後才能相減
);

    // -- Phase accumulator（照抄 sine_gen.v，reload 條件從 trig_start 換成 reload_pulse）--
    // 2026-08-18：累加條件加上 step_en（= waveform_controller.v 的
    // dac_valid）——dac_samp_idx 只有在真正送出一筆新樣本（dac_valid）
    // 那一拍才會前進，如果這裡的 phase_acc 不管 dac_valid 每個 dac_clk
    // cycle 都累加，一旦 FIFO 偶爾空拍（例如剛開始播放、all_ready 還沒
    // 生效那段期間），dac_samp_idx 會停住但 phase_acc 繼續往前跑，
    // 兩邊從此永久對不上、而且沒有機制會自動追回來。
    reg [31:0] active_tuning;
    reg [31:0] phase_acc;

    always @(posedge dac_clk) begin
        if (rst) begin
            active_tuning <= 32'd0;
            phase_acc     <= 32'd0;
        end else if (reload_pulse) begin
            active_tuning <= tuning_word_active;
            phase_acc     <= phase_seed_active;
        end else if (step_en) begin
            phase_acc <= phase_acc + active_tuning;
        end
    end

    wire [LUT_ADDR_WIDTH-1:0] lut_addr = phase_acc[31 -: LUT_ADDR_WIDTH];

    // -- Lookup table（跟 sine_gen.v 共用同一份 .mem 檔案，同步 BRAM 讀取，1-cycle latency）--
    (* ram_style = "block" *) reg signed [15:0] sine_lut [0:(1<<LUT_ADDR_WIDTH)-1];
    initial $readmemh(LUT_FILE, sine_lut);

    reg signed [15:0] lut_val_r;
    always @(posedge dac_clk) begin
        lut_val_r <= sine_lut[lut_addr];
    end

    // -- 振幅縮放（Q1.16 乘法，+1 cycle）：16-bit LUT 值 * 18-bit Q1.16 振幅 >> 16 --
    // dac_live=0 時強制輸出 0（安全閘控，見檔頭說明跟 waveform_controller.v
    // 的 dac_live port 定義處的完整理由——force_silence 期間 reload_pulse
    // 一直脈動、phase_acc 會被一直釘在 phase_seed_active，如果這裡不額外
    // 擋，靜音狀態下還是會算出一個非零值）。
    wire signed [33:0] amp_mult = $signed(lut_val_r) * $signed({1'b0, amp_stage});
    reg  signed [15:0] notch_value_r;
    always @(posedge dac_clk) begin
        notch_value_r <= dac_live ? amp_mult[31:16] : 16'sd0;
    end

    assign notch_value = notch_value_r;

endmodule
`default_nettype wire
