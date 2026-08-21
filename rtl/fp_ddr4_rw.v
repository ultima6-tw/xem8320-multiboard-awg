`timescale 1ns/1ps
`default_nettype none

// fp_ddr4_rw.v -- local copy (not the shared awg-test-step-6/vivado/rtl/
// fp_ddr4_rw.v), 2026-08-18. awg-test-step-6 已搬進 Projects/FPGA/
// _archive/（2026-08-10），create_bd.tcl 原本寫死指向該資料夾的路徑因此
// 失效（dry-run 抓到）；比照這個專案既有慣例（aurora_ctrl_mux.v/
// waveform_controller.v/calib_mux.v/aurora_user_clk_buf.v 都已經是同樣
// 「local copy」模式），直接複製進來、不再依賴外部/已封存的資料夾。
// 內容跟原檔完全相同，未做任何邏輯改動。
//
// fp_ddr4_rw: FrontPanel-controlled AXI4 128-bit single-beat read/write.
// Phase 0: directly tests axi_cc_ddr4 + DDR4 path without AXI DMA.
//
// wo_status bit layout:
//   [0]   = wr_done  (cleared when new op starts, set when BVALID received)
//   [1]   = rd_done  (cleared when new op starts, set when RVALID received)
//   [2]   = busy     (1 while transaction in progress)
//   [4:3] = bresp    (last write response)
//   [6:5] = rresp    (last read response)
//
// AXI4 single beat: AWLEN=0, AWSIZE=4 (16B), WLAST=1 always.
// Transactions gated by calib_done — write to address 0x00000000 for first beat.
// Address must be 16-byte aligned (bits[3:0] = 0).

module fp_ddr4_rw (
    input  wire        clk,
    input  wire        resetn,
    input  wire        calib_done,    // from DDR4 c0_init_calib_complete

    // FrontPanel WireIn
    input  wire [31:0] wi_addr,       // AXI address (16-byte aligned)
    input  wire [31:0] wi_wdata_0,    // write data [31:0]
    input  wire [31:0] wi_wdata_1,    // write data [63:32]
    input  wire [31:0] wi_wdata_2,    // write data [95:64]
    input  wire [31:0] wi_wdata_3,    // write data [127:96]

    // FrontPanel TriggerIn (32-bit bus)
    input  wire [31:0] ti_cmd,        // bit[0]=start_write  bit[1]=start_read

    // FrontPanel WireOut
    output reg  [31:0] wo_status,
    output reg  [31:0] wo_rdata_0,    // read data [31:0]
    output reg  [31:0] wo_rdata_1,    // read data [63:32]
    output reg  [31:0] wo_rdata_2,    // read data [95:64]
    output reg  [31:0] wo_rdata_3,    // read data [127:96]

    // ── AXI4 master (128-bit) ─────────────────────────────────────────
    // Vivado BD interface recognition via X_INTERFACE_INFO annotations.
    // No ID signals: SmartConnect manages IDs.

    // Write address channel
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME m_axi, PROTOCOL AXI4, DATA_WIDTH 128, ADDR_WIDTH 32, HAS_BURST 1, HAS_LOCK 0, HAS_CACHE 1, HAS_PROT 1, HAS_QOS 0, HAS_REGION 0, HAS_WSTRB 1, HAS_BRESP 1, HAS_RRESP 1, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 1, MAX_BURST_LENGTH 1, READ_WRITE_MODE READ_WRITE" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWADDR" *)
    output reg  [31:0]  m_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWLEN" *)
    output wire [7:0]   m_axi_awlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWSIZE" *)
    output wire [2:0]   m_axi_awsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWBURST" *)
    output wire [1:0]   m_axi_awburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWCACHE" *)
    output wire [3:0]   m_axi_awcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWPROT" *)
    output wire [2:0]   m_axi_awprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWVALID" *)
    output reg          m_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWREADY" *)
    input  wire         m_axi_awready,

    // Write data channel
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WDATA" *)
    output reg  [127:0] m_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WSTRB" *)
    output wire [15:0]  m_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WLAST" *)
    output wire         m_axi_wlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WVALID" *)
    output reg          m_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WREADY" *)
    input  wire         m_axi_wready,

    // Write response channel
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BRESP" *)
    input  wire [1:0]   m_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BVALID" *)
    input  wire         m_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BREADY" *)
    output wire         m_axi_bready,

    // Read address channel
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARADDR" *)
    output reg  [31:0]  m_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARLEN" *)
    output wire [7:0]   m_axi_arlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARSIZE" *)
    output wire [2:0]   m_axi_arsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARBURST" *)
    output wire [1:0]   m_axi_arburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARCACHE" *)
    output wire [3:0]   m_axi_arcache,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARPROT" *)
    output wire [2:0]   m_axi_arprot,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARVALID" *)
    output reg          m_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARREADY" *)
    input  wire         m_axi_arready,

    // Read data channel
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RDATA" *)
    input  wire [127:0] m_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RRESP" *)
    input  wire [1:0]   m_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RLAST" *)
    input  wire         m_axi_rlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RVALID" *)
    input  wire         m_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RREADY" *)
    output reg          m_axi_rready
);

// ── Fixed AXI4 burst-field constants ─────────────────────────────────
assign m_axi_awlen   = 8'd0;      // 1 beat
assign m_axi_awsize  = 3'd4;      // 2^4 = 16 bytes (128-bit)
assign m_axi_awburst = 2'b01;     // INCR
assign m_axi_awcache = 4'b0010;   // Normal Non-cacheable Bufferable
assign m_axi_awprot  = 3'b000;

assign m_axi_wstrb   = 16'hFFFF;  // all 16 bytes valid
assign m_axi_wlast   = 1'b1;      // single beat → last == first

assign m_axi_bready  = 1'b1;      // always accept response

assign m_axi_arlen   = 8'd0;
assign m_axi_arsize  = 3'd4;
assign m_axi_arburst = 2'b01;
assign m_axi_arcache = 4'b0010;
assign m_axi_arprot  = 3'b000;

// ── State machine ─────────────────────────────────────────────────────
localparam S_IDLE  = 3'd0,
           S_WR    = 3'd1,   // waiting for AW + W accepted
           S_WR_B  = 3'd2,   // waiting for BVALID
           S_RD_AR = 3'd3,   // waiting for ARREADY
           S_RD_R  = 3'd4;   // waiting for RVALID

reg [2:0] state;
reg aw_done_r, w_done_r;

wire aw_cleared = aw_done_r || (m_axi_awvalid && m_axi_awready);
wire w_cleared  = w_done_r  || (m_axi_wvalid  && m_axi_wready);

// TI is a single-cycle pulse from FrontPanel; gate by calib_done
wire start_wr = ti_cmd[0] && calib_done;
wire start_rd = ti_cmd[1] && calib_done;

always @(posedge clk) begin
    if (!resetn) begin
        state          <= S_IDLE;
        m_axi_awvalid  <= 1'b0;
        m_axi_wvalid   <= 1'b0;
        m_axi_arvalid  <= 1'b0;
        m_axi_rready   <= 1'b0;
        m_axi_awaddr   <= 32'd0;
        m_axi_araddr   <= 32'd0;
        m_axi_wdata    <= 128'd0;
        aw_done_r      <= 1'b0;
        w_done_r       <= 1'b0;
        wo_status      <= 32'd0;
        wo_rdata_0     <= 32'd0;
        wo_rdata_1     <= 32'd0;
        wo_rdata_2     <= 32'd0;
        wo_rdata_3     <= 32'd0;
    end else begin
        case (state)

            S_IDLE: begin
                if (start_wr) begin
                    m_axi_awaddr  <= wi_addr;
                    m_axi_wdata   <= {wi_wdata_3, wi_wdata_2, wi_wdata_1, wi_wdata_0};
                    m_axi_awvalid <= 1'b1;
                    m_axi_wvalid  <= 1'b1;
                    aw_done_r     <= 1'b0;
                    w_done_r      <= 1'b0;
                    wo_status     <= 32'h0000_0004;   // busy
                    state         <= S_WR;
                end else if (start_rd) begin
                    m_axi_araddr  <= wi_addr;
                    m_axi_arvalid <= 1'b1;
                    wo_status     <= 32'h0000_0004;   // busy
                    state         <= S_RD_AR;
                end
            end

            // Both AW and W issued; deassert only on valid handshake
            S_WR: begin
                if (m_axi_awvalid && m_axi_awready) begin
                    m_axi_awvalid <= 1'b0;
                    aw_done_r     <= 1'b1;
                end
                if (m_axi_wvalid && m_axi_wready) begin
                    m_axi_wvalid  <= 1'b0;
                    w_done_r      <= 1'b1;
                end
                if (aw_cleared && w_cleared)
                    state <= S_WR_B;
            end

            S_WR_B: begin
                if (m_axi_bvalid) begin
                    // [0]=wr_done  [4:3]=bresp
                    wo_status <= {27'b0, m_axi_bresp, 3'b001};
                    state     <= S_IDLE;
                end
            end

            S_RD_AR: begin
                if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    m_axi_rready  <= 1'b1;
                    state         <= S_RD_R;
                end
            end

            S_RD_R: begin
                if (m_axi_rvalid) begin
                    wo_rdata_0   <= m_axi_rdata[31:0];
                    wo_rdata_1   <= m_axi_rdata[63:32];
                    wo_rdata_2   <= m_axi_rdata[95:64];
                    wo_rdata_3   <= m_axi_rdata[127:96];
                    m_axi_rready <= 1'b0;
                    // [1]=rd_done  [6:5]=rresp
                    wo_status    <= {25'b0, m_axi_rresp, 5'b00010};
                    state        <= S_IDLE;
                end
            end

            default: state <= S_IDLE;
        endcase
    end
end

endmodule
`default_nettype wire
