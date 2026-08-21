`timescale 1ns/1ps
`default_nettype none

// ddr_zero_writer — 2026-07-14 新增，stop 指令的一部分：掃過整個 DDR4
// 位址空間，把每個位置寫成 0。
//
// AXI4 burst write 狀態機完全比照 ddr_writer.v 已經驗證過在跑的單拍
// （AWLEN=0）寫入邏輯（本專案歷史上懷疑過多拍 burst 寫 DDR4 在硬體上
// 有問題，ddr_writer.v 因此降級成單拍，這裡直接沿用同樣保守的做法，
// 不冒風險）。跟 ddr_writer.v 的差異：不用接收輸入封包/不用 FIFO，
// wdata 固定 32'd0，位址從 0 開始每次 +16 bytes，直到掃完 DDR4_TOTAL_BYTES
// 為止。
//
// DDR4 容量：MT40A512M16LY-075，DDR4_DataWidth=16，512M x 16bit = 8Gb
// = 1GB，見 create_bd.tcl 的 ddr4_0 CONFIG。單拍（16 bytes/beat）估計
// 全部掃完數量級是十幾秒，使用者已確認可接受（這是 stop，不是即時路徑，
// 0V 安全機制已經在 waveform_controller.v 立即生效，不依賴這個模組）。
module ddr_zero_writer #(
    parameter [31:0] DDR4_TOTAL_BYTES = 32'h4000_0000   // 1GB
) (
    (* X_INTERFACE_INFO      = "xilinx.com:signal:clock:1.0 sys_clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF m_axi, ASSOCIATED_RESET sys_rst, FREQ_HZ 100000000" *)
    input  wire         sys_clk,
    input  wire         sys_rst,

    // single-cycle pulse：開始掃描（來自 stop 指令）
    input  wire         start,

    // AXI4 Master（128-bit，sys_clk -> axi_cc -> ddr4_ui_clk，同
    // ddr_writer.v 的參數慣例）
    (* X_INTERFACE_PARAMETER = "PROTOCOL AXI4, DATA_WIDTH 128, ADDR_WIDTH 32, MAX_BURST_LENGTH 1, NUM_READ_OUTSTANDING 0, NUM_WRITE_OUTSTANDING 1, HAS_BURST 1, HAS_LOCK 0, HAS_CACHE 0, HAS_REGION 0, HAS_QOS 0, SUPPORTS_NARROW_BURST 0" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWADDR" *)
    output reg  [31:0]  m_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWLEN" *)
    output wire [7:0]   m_axi_awlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWSIZE" *)
    output wire [2:0]   m_axi_awsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWBURST" *)
    output wire [1:0]   m_axi_awburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWVALID" *)
    output reg          m_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWREADY" *)
    input  wire         m_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WDATA" *)
    output wire [127:0] m_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WSTRB" *)
    output wire [15:0]  m_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WVALID" *)
    output reg          m_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WLAST" *)
    output wire         m_axi_wlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WREADY" *)
    input  wire         m_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BRESP" *)
    input  wire [1:0]   m_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BVALID" *)
    input  wire         m_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BREADY" *)
    output wire         m_axi_bready,
    // AR/R 通道不使用（同 ddr_writer.v 慣例）
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARADDR" *)
    output wire [31:0]  m_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARLEN" *)
    output wire [7:0]   m_axi_arlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARSIZE" *)
    output wire [2:0]   m_axi_arsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARBURST" *)
    output wire [1:0]   m_axi_arburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARVALID" *)
    output wire         m_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi ARREADY" *)
    input  wire         m_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RDATA" *)
    input  wire [127:0] m_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RRESP" *)
    input  wire [1:0]   m_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RLAST" *)
    input  wire         m_axi_rlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RVALID" *)
    input  wire         m_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi RREADY" *)
    output wire         m_axi_rready,

    output reg          busy,   // 1 = 掃描進行中
    output reg          done    // single-cycle pulse：整個 DDR4 已經清完
);

    // ── AXI4 靜態接線（單拍：AWLEN=0）────────────────────────────────────────
    assign m_axi_awlen   = 8'd0;
    assign m_axi_awsize  = 3'b100;   // 16 bytes/beat（128-bit）
    assign m_axi_awburst = 2'b01;    // INCR
    assign m_axi_wdata   = 128'd0;   // 固定寫 0
    assign m_axi_wstrb   = 16'hFFFF;
    assign m_axi_wlast   = 1'b1;     // 單拍，每次 W 都是最後一拍
    assign m_axi_bready  = 1'b1;
    assign m_axi_araddr  = 32'd0;
    assign m_axi_arlen   = 8'd0;
    assign m_axi_arsize  = 3'b100;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arvalid = 1'b0;
    assign m_axi_rready  = 1'b0;

    localparam ST_IDLE = 2'd0;
    localparam ST_AW   = 2'd1;
    localparam ST_W    = 2'd2;
    localparam ST_RESP = 2'd3;

    reg [1:0]  state;
    reg [31:0] cur_addr;

    always @(posedge sys_clk) begin
        done <= 1'b0;

        if (sys_rst) begin
            state         <= ST_IDLE;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid  <= 1'b0;
            busy          <= 1'b0;
            cur_addr      <= 32'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    if (start) begin
                        cur_addr <= 32'd0;
                        busy     <= 1'b1;
                        state    <= ST_AW;
                    end
                end

                ST_AW: begin
                    m_axi_awaddr  <= cur_addr;
                    m_axi_awvalid <= 1'b1;
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        m_axi_wvalid  <= 1'b1;
                        state         <= ST_W;
                    end
                end

                ST_W: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        m_axi_wvalid <= 1'b0;
                        state        <= ST_RESP;
                    end
                end

                ST_RESP: begin
                    if (m_axi_bvalid) begin
                        if (cur_addr + 32'd16 >= DDR4_TOTAL_BYTES) begin
                            busy  <= 1'b0;
                            done  <= 1'b1;
                            state <= ST_IDLE;
                        end else begin
                            cur_addr <= cur_addr + 32'd16;
                            state    <= ST_AW;
                        end
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule
`default_nettype wire
