`timescale 1ns/1ps
`default_nettype none

// dac_output_mux.v -- Per-module 2:1 output selector between the
// existing DDR4 playback path (wctrl_$ch/dac_data) and the sine
// generator path, added 2026-07-17, rewritten 2026-07-23 for 8-channel
// independent sine wave output (see NOTES.md 2026-07-23 sine_gen
// 8-channel design section).
//
// mode=0: DDR4 playback (pass wctrl_$ch/dac_data and dac_valid straight
//         through). DDR4 already supports independent ch1/ch2 content --
//         the host packs both channels into the sample stream it writes
//         to DDR4 itself, no RTL change needed here.
// mode=1: sine generator. Each ZmodAWG module physically has 2 channels
//         (ch1/ch2, see PORTS.md ZmodAWGController: cCh1In<-tdata[31:18],
//         cCh2In<-tdata[15:2]), and as of 2026-07-23 each physical
//         channel has its own active/idle sine_gen hardware pair
//         (sine_ctrl_regs.v's mux_sel selects which one is active).
//         This module just picks the two ACTIVE codes (one per physical
//         channel) and packs them into the format zmod_awg_$ch expects.
//         Mode selection itself stays per-module (not per-channel) --
//         the whole module's output switches between DDR and sine
//         together, no per-physical-channel DDR/sine mixing (confirmed
//         with user 2026-07-23).

module dac_output_mux (
    input  wire        mode,        // 0=DDR playback, 1=sine generator (WI 0x1D, direct-mapped, no CDC)

    input  wire [31:0] ddr_data,    // wctrl_$ch/dac_data
    input  wire        ddr_valid,   // wctrl_$ch/dac_valid

    // ch1 active/idle pair + select (sine_gen_${inst}_1_a/_b, sine_ctrl_regs_0/mux_sel_*)
    input  wire [13:0] sine_code_ch1_a,
    input  wire [13:0] sine_code_ch1_b,
    input  wire        sine_mux_sel_ch1,  // 0=ch1_a active, 1=ch1_b active

    // ch2 active/idle pair + select (sine_gen_${inst}_2_a/_b, sine_ctrl_regs_0/mux_sel_*)
    input  wire [13:0] sine_code_ch2_a,
    input  wire [13:0] sine_code_ch2_b,
    input  wire        sine_mux_sel_ch2,  // 0=ch2_a active, 1=ch2_b active

    output wire [31:0] out_data,    // -> zmod_awg_$ch/cDataAxisTdata
    output wire        out_valid    // -> zmod_awg_$ch/cDataAxisTvalid
);

    wire [13:0] sine_code_ch1 = sine_mux_sel_ch1 ? sine_code_ch1_b : sine_code_ch1_a;
    wire [13:0] sine_code_ch2 = sine_mux_sel_ch2 ? sine_code_ch2_b : sine_code_ch2_a;

    assign out_data  = mode ? {sine_code_ch1, 2'd0, sine_code_ch2, 2'd0} : ddr_data;
    assign out_valid = mode ? 1'b1 : ddr_valid;

endmodule
`default_nettype wire
