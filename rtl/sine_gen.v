`timescale 1ns/1ps
`default_nettype none

// sine_gen.v -- Real-time sine wave generator (NCO/DDS) for a single
// channel, added 2026-07-17.
//
// Background: the existing waveform output only has one path (DDR4
// playback -- host precomputes a sample table and writes it into DDR4,
// ddr4_stream_reader -> dc_fifo_xpm -> waveform_controller -> zmod_awg_$ch).
// This module is a parallel second path: computes a sine wave in real
// time, no need to pre-write DDR4, frequency/phase can be adjusted live
// via registers (see sine_ctrl_regs.v). Output format is compatible with
// waveform_controller's dac_data (14-bit DAC code); a 2:1 mux in
// create_bd.tcl selects one of the two paths into zmod_awg_$ch.
//
// Parameters (finalized 2026-07-17, see PROJECT.md):
//   - 32-bit phase accumulator: frequency resolution = dac_clk / 2^32
//     ~= 0.023 Hz/LSB (dac_clk=100MHz)
//   - 14-bit LUT address (top bits of phase_acc): 16384 points
//   - 16-bit signed LUT value: 2 extra bits of headroom above the 14-bit
//     DAC so the LUT itself doesn't become the resolution bottleneck
//   - Output is rounded (not truncated) to 14-bit, packing format
//     compatible with dac_data[31:18]
//
// Converting tuning_word (frequency) into "how much to add to the phase
// accumulator each dac_clk cycle" is done on the host side
// (tuning_word = round(freq_Hz * 2^32 / dac_clk_Hz)); the hardware only
// stores the final 32-bit integer, no division in hardware.
//
// Trigger synchronization (for multi-board sync): writing
// tuning_word_stage/phase_stage does not take effect immediately -- it
// only applies at the moment trig_start (the same net as
// wctrl_$ch/sw_trigger, see "or_total_trig_$ch/Res" in create_bd.tcl)
// fires, at which point the staged values are latched into the active
// registers and phase_acc is loaded with phase_stage to start
// accumulating. This way the DDR4 playback buffer swap and this
// module's phase start point share the same trigger moment -- symmetric
// behavior, no redesign needed when Aurora multi-board sync is added
// later.
//
// The LUT is loaded from an external .mem file via $readmemh (see
// host/gen_sine_lut.py); ram_style="block" forces BRAM inference
// (following project convention, no blk_mem_gen IP). Synchronous BRAM
// read has 1-cycle latency, so the output lags phase_acc by 1 dac_clk
// cycle -- a fixed delay that doesn't affect steady-state frequency and
// is identical across channels, so it doesn't affect multi-board sync.

module sine_gen #(
    parameter LUT_ADDR_WIDTH = 14,   // 16384 points
    parameter LUT_FILE       = "sine_lut_16384.mem"
) (
    input  wire        dac_clk,
    input  wire        rst,

    input  wire [31:0] tuning_word_stage,   // host-computed frequency tuning word (staged, takes effect on trig_start)
    input  wire [31:0] phase_stage,         // starting phase (staged, takes effect on trig_start)
    input  wire        trig_start,          // single-cycle pulse, same trigger moment as wctrl_$ch/sw_trigger

    output wire [13:0] dac_code,            // rounded 14-bit signed DAC code
    output wire [31:0] phase_acc_out         // current phase accumulator value (for status query, see aurora_reply_tx.v)
);

    // -- Phase accumulator ---------------------------------------------
    reg [31:0] active_tuning;
    reg [31:0] phase_acc;

    always @(posedge dac_clk) begin
        if (rst) begin
            active_tuning <= 32'd0;
            phase_acc     <= 32'd0;
        end else if (trig_start) begin
            active_tuning <= tuning_word_stage;
            phase_acc     <= phase_stage;
        end else begin
            phase_acc <= phase_acc + active_tuning;
        end
    end

    wire [LUT_ADDR_WIDTH-1:0] lut_addr = phase_acc[31 -: LUT_ADDR_WIDTH];

    // -- Lookup table (BRAM, synchronous read) --------------------------
    (* ram_style = "block" *) reg signed [15:0] sine_lut [0:(1<<LUT_ADDR_WIDTH)-1];
    initial $readmemh(LUT_FILE, sine_lut);

    reg signed [15:0] lut_val_r;
    always @(posedge dac_clk) begin
        lut_val_r <= sine_lut[lut_addr];
    end

    // -- 16-bit -> 14-bit rounding (with saturation, avoids overflow when rounding the +peak value) --
    localparam signed [16:0] DAC_MAX = 17'sd8191;
    localparam signed [16:0] DAC_MIN = -17'sd8192;

    wire signed [16:0] round_sum = {lut_val_r[15], lut_val_r} + 17'sd2;
    wire signed [16:0] shifted   = round_sum >>> 2;

    assign dac_code = (shifted > DAC_MAX) ? DAC_MAX[13:0] :
                       (shifted < DAC_MIN) ? DAC_MIN[13:0] :
                       shifted[13:0];

    assign phase_acc_out = phase_acc;

endmodule
`default_nettype wire
