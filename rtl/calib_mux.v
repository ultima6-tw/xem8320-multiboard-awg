`timescale 1ns/1ps
`default_nettype none

// calib_mux.v -- local copy (not the shared awg-test-step-6/vivado/rtl/
// calib_mux.v), forked 2026-07-27 for the「統一讀取/寫入架構」collapse
// (see PROJECT.md「統一讀取/寫入架構 — 完整規格」小節). The original file
// is referenced by several older projects (step-6 through step-14) via
// add_files pointing at $src_dir6 -- editing it in place would have
// changed those projects' source too, so step-16 gets its own copy
// instead (same convention already used for ddr4_stream_reader.v /
// awg_calib_regs.v).
//
// Change from the shared original: removed the fp_wr/fp_sel/fp_data
// (FrontPanel local-write) path entirely. calib_coef is now only ever
// written via the Aurora T_CALIB_WR/T_CALIB_RST packet path (host loops
// back to itself via dest_id=own board_id, same mechanism as every other
// collapsed write path -- see PROJECT.md for the full rationale).
//
// `ext_rst` kept (renamed from the original `fp_rst`): despite the old
// name, this was never part of the FrontPanel write path -- create_bd.tcl
// ties it to the board-wide `rst_clk100/peripheral_reset` net (the
// general power-on reset shared by trig_timer/wctrl/sine_gen/etc, see
// PROJECT.md), not to any host-triggered "reset calib" command. It's the
// only thing that actually resets awg_calib_regs_0/coef[] at power-on --
// au_rst (T_CALIB_RST=0x03) is a separate, intentional host-triggered
// reset, not a substitute for it. Removing this entirely (as the first
// draft of this fork did) would have left the calib coefficients with
// undefined post-configuration values until someone explicitly sent
// T_CALIB_RST -- caught and fixed before touching create_bd.tcl.

module calib_mux (
    // Aurora 路徑（sys_clk domain，來自 aurora_packet_rx 輸出；2026-07-27
    // 起唯一寫入路徑，本機直寫已拔除）
    input  wire        au_wr,
    input  wire [4:0]  au_sel,
    input  wire [17:0] au_data,
    input  wire        au_rst,

    // 板子整體 power-on reset（見上方說明，不是本機直寫路徑的一部分）
    input  wire        ext_rst,

    // 輸出至 awg_calib_regs
    output wire        calib_wr,
    output wire [4:0]  calib_sel,
    output wire [17:0] calib_data,
    output wire        calib_rst
);
    assign calib_wr   = au_wr;
    assign calib_rst  = au_rst | ext_rst;
    assign calib_sel  = au_sel;
    assign calib_data = au_data;

endmodule
`default_nettype wire
