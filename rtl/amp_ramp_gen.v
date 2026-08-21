`timescale 1ns/1ps
`default_nettype none

// amp_ramp_gen.v -- Per-channel hardware amplitude ramp/envelope
// generator, added 2026-07-17. On trig_start, linearly interpolates the
// amplitude from start_amp to (implicitly) end_amp over duration_cycles
// dac_clk cycles. loop_mode selects what happens once duration_cycles
// is reached:
//   0 = one-shot: hold at the final value forever
//   1 = sawtooth loop: snap back to start_amp and ramp up again (repeats)
//   2 = triangle loop: reverse direction (start<->end ping-pong, repeats)
//   3 = reserved (behaves as one-shot)
//
// Same "host precomputes, hardware only accumulates" pattern as
// sine_gen.v: host computes step = round(((end_amp - start_amp) << 14) /
// duration_cycles) -- no division in hardware. The 14 extra fractional
// bits (32-bit accumulator, 18-bit Q1.16 output) give smooth
// sub-output-LSB stepping so the ramp doesn't look like visible stair
// steps, mirroring sine_gen's phase-accumulator precision technique.
// For triangle mode, the reverse leg reuses the same duration_cycles and
// the negated step, so it returns to exactly start_amp (matching
// magnitude, opposite sign, same cycle count).
//
// amp_out is Q1.16 signed (same format as the existing amp_ctrl_0..7
// registers in aurora_ctrl_mux.v), so it can feed the same downstream
// multiplier in awg_calib_regs.v via a new per-channel select mux (see
// amp_ctrl_mux_$ch in create_bd.tcl) -- this module does NOT modify
// aurora_ctrl_mux.v (a file shared across other awg-test-step-N
// projects).
//
// trig_start is the SAME net as wctrl_$ch/sw_trigger and
// sine_gen_$ch/trig_start (or_total_trig_$ch/Res), so DDR playback,
// sine phase, and amplitude ramps all start at the same triggered
// instant across channels (and, later, across boards).

module amp_ramp_gen (
    input  wire        dac_clk,
    input  wire        rst,

    input  wire [31:0] start_amp_stage,       // staged: Q1.16 value in low 18 bits
    input  wire [31:0] step_stage,            // staged: host-computed per-cycle increment (start->end direction)
    input  wire [31:0] duration_cycles_stage, // staged: how many cycles one leg of the ramp takes
    input  wire [31:0] loop_mode_stage,       // staged: low 2 bits, see header comment
    input  wire        trig_start,            // single-cycle pulse, shared trigger net

    output wire [17:0] amp_out                // Q1.16 signed, -> amp_ctrl_mux_$ch
);

    reg [31:0] acc;
    reg [31:0] active_step;
    reg [31:0] active_duration;
    reg [31:0] cycle_count;
    reg [1:0]  active_loop_mode;
    reg [31:0] active_start_amp_full;   // {start_amp[17:0],14'd0}, kept for sawtooth restart

    wire duration_reached = (cycle_count >= active_duration);

    always @(posedge dac_clk) begin
        if (rst) begin
            acc                    <= 32'd0;
            active_step            <= 32'd0;
            active_duration        <= 32'd0;
            cycle_count            <= 32'd0;
            active_loop_mode       <= 2'd0;
            active_start_amp_full  <= 32'd0;
        end else if (trig_start) begin
            acc                    <= {start_amp_stage[17:0], 14'd0};
            active_step            <= step_stage;
            active_duration        <= duration_cycles_stage;
            cycle_count            <= 32'd0;
            active_loop_mode       <= loop_mode_stage[1:0];
            active_start_amp_full  <= {start_amp_stage[17:0], 14'd0};
        end else if (!duration_reached) begin
            acc         <= acc + active_step;
            cycle_count <= cycle_count + 32'd1;
        end else begin
            case (active_loop_mode)
                2'd1: begin // sawtooth loop: snap back to start_amp, restart
                    acc         <= active_start_amp_full;
                    cycle_count <= 32'd0;
                end
                2'd2: begin // triangle loop: reverse direction, keep running from current position
                    active_step <= -active_step;
                    cycle_count <= 32'd0;
                end
                default: begin
                    // one-shot: hold acc/cycle_count unchanged
                end
            endcase
        end
    end

    assign amp_out = acc[31:14];

endmodule
`default_nettype wire
