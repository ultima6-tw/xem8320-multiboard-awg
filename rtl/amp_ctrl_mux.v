`timescale 1ns/1ps
`default_nettype none

// amp_ctrl_mux.v -- Per-physical-channel 2:1 select between the existing
// static amp_ctrl hold register (aurora_ctrl_mux_0/out_amp_ctrl_$i) and the
// amp_ramp_gen's live ramping output, added 2026-07-17, rewritten 2026-07-23
// for 8-channel independent output (one instance per physical channel, i=0-7,
// matching the pre-existing amp_ctrl_0..7 numbering in aurora_ctrl_mux.v --
// see NOTES.md 2026-07-23 sine_gen 8-channel design section). Pure
// combinational, same pattern as dac_output_mux.v (small dedicated mux
// module rather than building it out of BD primitive logic gates). Output
// feeds awg_calib_regs_0/amp_ctrl_$i, which is downstream of BOTH the DDR4
// playback and sine generator paths, so a ramp applies regardless of which
// waveform source is currently selected.
//
// As of 2026-07-23, amp_ramp_gen also has a true active/idle hardware pair
// per physical channel (mirroring sine_gen), so this module now picks the
// currently-active one (mux_sel, same signal sine_ctrl_regs_0 uses to
// select the active sine_gen instance for this physical channel) instead of
// taking a single ramp_amp input.

module amp_ctrl_mux (
    input  wire        ramp_en,      // 0=static amp_ctrl, 1=amp_ramp_gen live output

    input  wire [17:0] static_amp,   // aurora_ctrl_mux_0/out_amp_ctrl_$i

    input  wire [17:0] ramp_amp_a,   // amp_ramp_gen_${inst}_${sub}_a/amp_out
    input  wire [17:0] ramp_amp_b,   // amp_ramp_gen_${inst}_${sub}_b/amp_out
    input  wire        mux_sel,      // 0=a active, 1=b active (sine_ctrl_regs_0/mux_sel_$i)

    output wire [17:0] amp_ctrl_out  // -> awg_calib_regs_0/amp_ctrl_$i
);

    wire [17:0] ramp_amp = mux_sel ? ramp_amp_b : ramp_amp_a;

    assign amp_ctrl_out = ramp_en ? ramp_amp : static_amp;

endmodule
`default_nettype wire
