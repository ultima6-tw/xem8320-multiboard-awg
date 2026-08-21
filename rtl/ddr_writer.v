`timescale 1ns/1ps
`default_nettype none

// ddr_writer v2 — Step 14 v9：移到 sys_clk domain
//
// 接收 Dispatcher DDR 輸出的 T_WAVEFORM_STREAM 封包（sys_clk），
// 解析 header 後將波形資料寫入 DDR4（AXI4 burst write）。
// 取代 step-12 aurora_wave_rx.v。
//
// ── 輸入封包格式（Dispatcher 已過濾為 T_WAVEFORM_STREAM）────────────────────
//   beat0：{pkt_len[23:0], 0x13, src_id[15:0], dest_id[15:0]}  → skip
//   beat1：{wave_total_bytes[31:0], wave_start_addr[31:0]}      → latch params
//   beat2..N：wave data（64-bit each）                           → DDR4
//
// ── 資料流 ───────────────────────────────────────────────────────────────────
//   beat2..N → 64→128 pair-acc → 256-entry 128-bit FIFO → AXI4 burst write
//
// ── 限制 ─────────────────────────────────────────────────────────────────────
//   wave_total_bytes 必須是 16 的倍數（128-bit AXI4 beat 對齊）
//
// ── 2026-07-15 改動記錄（排查 T_WAVEFORM_STREAM 寫入 DDR4 內容錯位問題，
//    連續第 3 次 AXI 寫入開始出錯，見 PROJECT.md「T_WAVEFORM_STREAM」章節
//    完整討論）────────────────────────────────────────────────────────────
// 1. pair-acc 用的 256-entry 128-bit 緩衝，原本是手寫的 reg array +
//    wr_ptr/rd_ptr/fifo_cnt 指標邏輯，逐行核對多次找不到明確 bug，改用
//    官方 xpm_fifo_sync 取代（沿用 awg-test-step-14/rtl/fp_input.v 已經
//    驗證過的 FWFT 模式接線慣例）。這個 FIFO 純粹是 sys_clk 域內部的速度
//    緩衝（封包到達速度 vs AXI 單拍寫入速度不同），不是跨時脈域用的。
// 2. 跨到 ddr4_ui_clk 的 axi_cc_ddr4（axi_clock_converter）改成
//    fifo_generator（AXI4 介面、只選 Write Channels、Independent
//    Clocks，見 create_bd.tcl）——查證 PG057 文件確認 fifo_generator 的
//    AXI4 模式會在單一 IP 裡自動產生 AW/W/B 三個互相協調好的內部 FIFO，
//    且明確處理 AW/W 之間的耦合關係（Packet FIFO on Write Channels：
//    等完整資料收到才送出 AW），是可以直接取代 axi_clock_converter 的
//    方案。ddr_writer.v 這邊的 m_axi_* 介面/AXI4 burst-write 狀態機完全
//    不用改，只是下游接的 BD cell 換了。

module ddr_writer (
    (* X_INTERFACE_INFO      = "xilinx.com:signal:clock:1.0 sys_clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF m_axi, ASSOCIATED_RESET sys_rst, FREQ_HZ 100000000" *)
    input  wire         sys_clk,
    input  wire         sys_rst,

    // Dispatcher DDR 輸出（sys_clk）
    input  wire [63:0]  wave_in_tdata,
    input  wire         wave_in_tvalid,
    output wire         wave_in_tready,

    // AXI4 Master（128-bit，sys_clk → fifo_generator(AXI4) → ddr4_ui_clk）
    (* X_INTERFACE_PARAMETER = "PROTOCOL AXI4, DATA_WIDTH 128, ADDR_WIDTH 32, MAX_BURST_LENGTH 256, NUM_READ_OUTSTANDING 0, NUM_WRITE_OUTSTANDING 4, HAS_BURST 1, HAS_LOCK 0, HAS_CACHE 0, HAS_REGION 0, HAS_QOS 0, SUPPORTS_NARROW_BURST 0" *)
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWADDR" *)
    output reg  [31:0]  m_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWLEN" *)
    output reg  [7:0]   m_axi_awlen,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWSIZE" *)
    output wire [2:0]   m_axi_awsize,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWBURST" *)
    output wire [1:0]   m_axi_awburst,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWVALID" *)
    output reg          m_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi AWREADY" *)
    input  wire         m_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WDATA" *)
    output reg  [127:0] m_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WSTRB" *)
    output wire [15:0]  m_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WVALID" *)
    output reg          m_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WLAST" *)
    output reg          m_axi_wlast,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi WREADY" *)
    input  wire         m_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BRESP" *)
    input  wire [1:0]   m_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BVALID" *)
    input  wire         m_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 m_axi BREADY" *)
    output wire         m_axi_bready,
    // AR/R 通道不使用
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

    output reg          busy,
    output wire [3:0]   debug_state,
    output reg  [1:0]   debug_bresp,
    output wire [8:0]   debug_fifo_cnt,
    output wire [7:0]   debug_wr_ptr,
    output wire [7:0]   debug_rd_ptr,
    output wire [7:0]   debug_rxbeatcnt,
    output wire         debug_half_valid,
    output wire [31:0]  debug_remaining_beats,
    output wire [31:0]  debug_wave_total_bytes,
    // 2026-07-16 新增：wr_data_count（debug_fifo_cnt）上機驗證懷疑不可靠
    // （收到2筆beat、half_valid已經正確toggle，但debug_fifo_cnt整個
    // capture window都是0），改把pacc_fifo_inst原本空接的wr_rst_busy/
    // rd_rst_busy/full/empty也曝露出來，下次再懷疑這顆FIFO不用再等一輪
    // 重建才能看
    output wire         debug_pacc_empty,
    output wire         debug_pacc_full,
    output wire [1:0]   debug_pacc_rst_busy,   // {wr_rst_busy, rd_rst_busy}

    // 2026-07-16 新增：m_axi_awvalid/awaddr/wvalid/wdata/wlast/bready 的
    // 鏡像輸出，跟真正的 m_axi interface port 完全分開，專供 ILA 使用。
    // 根因：ddr_writer_ila_0 之前直接對這幾個「已屬於 m_axi interface
    // 成員」的 pin 下 connect_bd_net，導致 Vivado 把該訊號排除出 m_axi
    // 的 interface 整合網路，ddr_writer_axi_fifo 那端收到的變成沒有驅動
    // 來源、被靜默 tie 常數 0（build 完全沒有錯誤或警告），造成
    // T_WAVEFORM_STREAM AXI burst-write 狀態機卡死在 ST_AW，見
    // PROJECT.md「T_WAVEFORM_STREAM」章節 2026-07-16 postmortem。之後
    // ILA 一律探測這幾個鏡像 port，不再直接碰 m_axi_* 本身。
    output wire         debug_axi_awvalid,
    output wire [31:0]  debug_axi_awaddr,
    output wire         debug_axi_wvalid,
    output wire [127:0] debug_axi_wdata,
    output wire         debug_axi_wlast,
    output wire         debug_axi_bready,

    // 2026-07-16 新增（第四輪）：dispatcher 送進來的原始 wave_in_tdata，跟
    // pair-acc 組出來、要塞進 pacc_fifo 的 128-bit 合併值，同時接上 ILA，
    // 用來直接比對「進來的兩個 64-bit beat」跟「組出來的 128-bit 值」
    // 是否正確對應。同樣是純鏡像 assign，不碰任何 interface pin。
    output wire [63:0]  debug_in_tdata,
    output wire         debug_in_tvalid,
    output wire         debug_in_tready,
    output wire [127:0] debug_pacc_din,
    output wire         debug_pacc_wr_en,

    // Raw beat readback（診斷用，sys_clk 靜止後 sys_clk 讀取）
    // 2026-07-15：raw_rd_data 從單一 64-bit port 改成 lo/hi 兩個 32-bit
    // port（WO 是 32-bit，這樣 BD 不用另外加 xlslice，沿用 diag_lrh_
    // first_lo/hi 同樣的慣例）
    input  wire [6:0]   raw_rd_addr,
    output wire [31:0]  raw_rd_data_lo,
    output wire [31:0]  raw_rd_data_hi
);

    // ── AXI4 靜態接線 ─────────────────────────────────────────────────────────
    assign m_axi_awsize  = 3'b100;   // 16 bytes/beat（128-bit）
    assign m_axi_awburst = 2'b01;    // INCR
    assign m_axi_wstrb   = 16'hFFFF;
    assign m_axi_bready  = 1'b1;
    assign m_axi_araddr  = 32'd0;
    assign m_axi_arlen   = 8'd0;
    assign m_axi_arsize  = 3'b100;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arvalid = 1'b0;
    assign m_axi_rready  = 1'b0;

    // debug_axi_* 鏡像輸出（純供 ILA 用，見上方 port 宣告處說明）
    assign debug_axi_awvalid = m_axi_awvalid;
    assign debug_axi_awaddr  = m_axi_awaddr;
    assign debug_axi_wvalid  = m_axi_wvalid;
    assign debug_axi_wdata   = m_axi_wdata;
    assign debug_axi_wlast   = m_axi_wlast;
    assign debug_axi_bready  = m_axi_bready;

    // debug_in_*（見 port 宣告處說明）
    assign debug_in_tdata   = wave_in_tdata;
    assign debug_in_tvalid  = wave_in_tvalid;
    assign debug_in_tready  = wave_in_tready;
    // 2026-07-30 修正：debug_pacc_din/debug_pacc_wr_en 原本在這裡就
    // assign（比 pacc_fifo_din/pacc_fifo_wr_en 宣告處早），iverilog
    // 對這種 continuous assign 的宣告順序不嚴格、能過，但改用真正的
    // xpm_fifo_sync 跑 xvlog（Vivado 內建 simulator）驗證這次 ddr_
    // writer.v 大封包資料錯位 bug 時發現 xvlog 對 forward reference
    // 判定為 error（`identifier used before its declaration`），純粹
    // 搬到 pacc_fifo_din/pacc_fifo_wr_en 宣告之後（見下方），語意完全
    // 不變，只是為了兩套工具都能編譯。見 NOTES.md 2026-07-30「ddr_
    // writer.v 大封包資料錯位 bug 追查」章節。

    // ── 輸入封包解析狀態機 ────────────────────────────────────────────────────
    localparam IN_IDLE   = 2'd0;   // 等 beat0
    localparam IN_META   = 2'd1;   // 等 beat1（wave params）
    localparam IN_STREAM = 2'd2;   // 接收 data beats

    reg [1:0]  in_state  = IN_IDLE;
    reg [31:0] wave_start_addr;
    reg [31:0] wave_total_bytes;
    reg        meta_valid = 1'b0;   // 1-cycle pulse：wave params 已鎖定
    assign debug_wave_total_bytes = wave_total_bytes;

    // beat counter（診斷）
    reg [7:0] rxbeat_cnt = 8'd0;
    always @(posedge sys_clk) begin
        if (sys_rst || (in_state == IN_META && wave_in_tvalid && wave_in_tready))
            rxbeat_cnt <= 8'd0;
        else if (in_state == IN_STREAM && wave_in_tvalid && wave_in_tready)
            rxbeat_cnt <= rxbeat_cnt + 8'd1;
    end
    assign debug_rxbeatcnt = rxbeat_cnt;

    // raw beat capture（診斷，distributed RAM）
    (* ram_style = "distributed" *)
    reg [63:0] raw_fifo   [127:0];
    reg [6:0]  raw_wr_ptr = 7'd0;
    always @(posedge sys_clk) begin
        if (sys_rst || (in_state == IN_META && wave_in_tvalid && wave_in_tready)) begin
            raw_wr_ptr <= 7'd0;
        end else if (in_state == IN_STREAM && wave_in_tvalid && wave_in_tready
                     && raw_wr_ptr != 7'd127) begin
            raw_fifo[raw_wr_ptr] <= wave_in_tdata;
            raw_wr_ptr           <= raw_wr_ptr + 7'd1;
        end
    end
    assign raw_rd_data_lo = raw_fifo[raw_rd_addr][31:0];
    assign raw_rd_data_hi = raw_fifo[raw_rd_addr][63:32];

    // ── 256-entry 128-bit 速度緩衝 FIFO（xpm_fifo_sync，2026-07-15 取代
    //    原本手寫的 reg array + wr_ptr/rd_ptr/fifo_cnt 指標邏輯，見檔頭
    //    說明）────────────────────────────────────────────────────────────
    wire         pacc_fifo_wr_en;
    wire [127:0] pacc_fifo_din;
    wire         pacc_fifo_full;
    reg          pacc_fifo_rd_en;
    wire [127:0] pacc_fifo_dout;
    wire         pacc_fifo_empty;
    wire [8:0]   pacc_fifo_cnt;

    assign debug_pacc_din   = pacc_fifo_din;
    assign debug_pacc_wr_en = pacc_fifo_wr_en;

    // debug_wr_ptr/debug_rd_ptr：XPM 內部指標不對外曝露，這兩個 port 保留
    // （避免要跟著改 create_bd.tcl 的 ILA 接線），固定接 0，不再有意義。
    assign debug_wr_ptr = 8'd0;
    assign debug_rd_ptr = 8'd0;
    assign debug_fifo_cnt   = pacc_fifo_cnt;
    assign debug_pacc_empty = pacc_fifo_empty;
    assign debug_pacc_full  = pacc_fifo_full;

    wire pacc_wr_rst_busy;
    wire pacc_rd_rst_busy;
    assign debug_pacc_rst_busy = {pacc_wr_rst_busy, pacc_rd_rst_busy};

    xpm_fifo_sync #(
        .FIFO_WRITE_DEPTH (256),
        .WRITE_DATA_WIDTH (128),
        .READ_DATA_WIDTH  (128),
        .READ_MODE        ("fwft"),
        .USE_ADV_FEATURES ("0004"),   // bit[2]=EN_WDC，開啟 wr_data_count（否則被強制拉0，
                                       // 見 PROJECT.md「T_WAVEFORM_STREAM」章節 2026-07-16 postmortem）
                                       // 2026-07-16 上機驗證：即使開了這個 bit，debug_fifo_cnt
                                       // 仍觀察到停在 0（見 PROJECT.md 同一章節），原因未定案，
                                       // ST_FILL 判斷式已經改成不依賴這個值，這個 port 保留純供
                                       // 之後繼續排查參考
        .DOUT_RESET_VALUE ("0")
    ) pacc_fifo_inst (
        .wr_clk       (sys_clk),
        .rst          (sys_rst),
        .sleep        (1'b0),
        .wr_en        (pacc_fifo_wr_en),
        .din          (pacc_fifo_din),
        .rd_en        (pacc_fifo_rd_en),
        .dout         (pacc_fifo_dout),
        .empty        (pacc_fifo_empty),
        .full         (pacc_fifo_full),
        .prog_full    (),
        .wr_rst_busy  (pacc_wr_rst_busy),
        .rd_rst_busy  (pacc_rd_rst_busy),
        .data_valid   (),
        .wr_data_count(pacc_fifo_cnt),
        .rd_data_count(),
        .almost_empty (),
        .almost_full  (),
        .injectsbiterr(1'b0),
        .injectdbiterr(1'b0),
        .sbiterr      (),
        .dbiterr      ()
    );

    // ── 64→128 pair-acc ───────────────────────────────────────────────────────
    reg [63:0] half_buf   = 64'd0;
    reg        half_valid = 1'b0;

    assign pacc_fifo_wr_en = (in_state == IN_STREAM) && wave_in_tvalid && wave_in_tready
                             && half_valid && !pacc_fifo_full;
    assign pacc_fifo_din   = {wave_in_tdata, half_buf};

    assign debug_half_valid = half_valid;

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            half_valid <= 1'b0;
        end else if (in_state == IN_META && wave_in_tvalid && wave_in_tready) begin
            // beat1 消費當下清除 pair-acc（比 meta_valid 早 1 cycle，避免與 beat2 衝突）
            half_valid <= 1'b0;
        end else if (in_state == IN_STREAM && wave_in_tvalid && wave_in_tready && !pacc_fifo_full) begin
            if (!half_valid) begin
                half_buf   <= wave_in_tdata;
                half_valid <= 1'b1;
            end else begin
                half_valid <= 1'b0;
            end
        end
    end

    // ── wave_in_tready（combinatorial，無 lag）────────────────────────────────────
    // IN_IDLE/IN_META：無條件接受（只有 1 個 beat 需要處理）
    // IN_STREAM：由 pacc_fifo_full 反壓
    assign wave_in_tready = (in_state == IN_STREAM) ? !pacc_fifo_full : 1'b1;

    // ── 前向宣告（in_state SM 需要 axi_done）────────────────────────────────
    reg        axi_done = 1'b0;

    // ── 輸入封包解析（in_state） ──────────────────────────────────────────────
    always @(posedge sys_clk) begin
        meta_valid <= 1'b0;

        if (sys_rst) begin
            in_state <= IN_IDLE;
        end else begin
            case (in_state)
                IN_IDLE: begin
                    // beat0：消費（pkt_len 不使用，由 wave_total_bytes 計算 remaining）
                    if (wave_in_tvalid && wave_in_tready)
                        in_state <= IN_META;
                end

                IN_META: begin
                    if (wave_in_tvalid && wave_in_tready) begin
                        // beat1：{wave_total_bytes[31:0], wave_start_addr[31:0]}
                        wave_start_addr  <= wave_in_tdata[31:0];
                        wave_total_bytes <= wave_in_tdata[63:32];
                        meta_valid       <= 1'b1;   // 1-cycle pulse：觸發 AXI SM
                        in_state         <= IN_STREAM;
                    end
                end

                IN_STREAM: begin
                    // data beats 由 pair-acc always block 處理
                    // 結束條件：AXI SM 完成整個封包寫入
                    if (axi_done)
                        in_state <= IN_IDLE;
                end

                default: in_state <= IN_IDLE;
            endcase
        end
    end

    // ── AXI4 Burst Write 狀態機 ──────────────────────────────────────────────
    localparam ST_IDLE_A  = 3'd0;
    localparam ST_FILL    = 3'd1;   // 等 FIFO 累積足夠一次 burst
    localparam ST_AW      = 3'd2;
    localparam ST_W       = 3'd3;
    localparam ST_RESP    = 3'd4;

    reg [2:0]  axi_state       = ST_IDLE_A;
    reg [31:0] cur_addr        = 32'd0;
    reg [31:0] remaining_beats = 32'd0;  // 剩餘 128-bit beats
    reg [4:0]  burst_beats     = 5'd0;   // 本次 burst（1-16）
    reg [3:0]  beat_cnt_axi    = 4'd0;
    // axi_done declared above (forward ref for in_state SM)
    assign debug_remaining_beats = remaining_beats;

    // 2026-07-30 revert：Step 14.3b 當年為了排除「多拍 burst 是否為 DDR
    // write 在硬體上失敗的原因」而降級成單拍（AWLEN=0），但那次診斷本身
    // 沒有定論（同一設定連續跑 3 次出現 3 種不同失敗症狀，判斷是另一個
    // timing race，burst 長度沒被證實也沒被排除，見 PROJECT.md「DDR4
    // 清空時間估算」章節 2026-07-15 agent 查證結果）。後來真正查到的兩個
    // 根因都跟 burst 長度無關（BD 接線意外斷開、fp0/okClk 缺 CDC），都已
    // 修好且沿用至今。這次改回多拍 streaming，作為 T_WAVEFORM_STREAM 大
    // 封包資料錯位 bug（見 NOTES.md 2026-07-30「驗證使用者提出的假說」
    // 章節）的對照實驗：如果問題消失/門檻移動，是縮短單拍 handshake
    // 暴露時間的間接證據；如果完全不變，可進一步排除這個因素。
    wire [4:0] next_burst = (remaining_beats >= 32'd16) ? 5'd16 : remaining_beats[4:0];

    assign debug_state = {1'b0, axi_state};

    always @(posedge sys_clk) begin
        pacc_fifo_rd_en <= 1'b0;
        axi_done <= 1'b0;

        if (sys_rst) begin
            axi_state      <= ST_IDLE_A;
            m_axi_awvalid  <= 1'b0;
            m_axi_wvalid   <= 1'b0;
            m_axi_wlast    <= 1'b0;
            busy           <= 1'b0;
        end else begin
            case (axi_state)

                ST_IDLE_A: begin
                    m_axi_awvalid <= 1'b0;
                    m_axi_wvalid  <= 1'b0;
                    m_axi_wlast   <= 1'b0;
                    busy          <= 1'b0;
                    if (meta_valid) begin
                        cur_addr        <= wave_start_addr;
                        remaining_beats <= wave_total_bytes >> 4;  // bytes / 16
                        busy            <= 1'b1;
                        axi_state       <= ST_FILL;
                    end
                end

                ST_FILL: begin
                    // 2026-07-16：改成只看 !pacc_fifo_empty，不依賴 pacc_fifo_cnt
                    // （wr_data_count，上機驗證發現即使 USE_ADV_FEATURES 開對
                    // bit 仍觀察到停在 0，原因未定案，見 PROJECT.md）。
                    // next_burst 目前固定是 1，!empty 在 FWFT 模式下等同「至少
                    // 有 1 筆資料」，語意完全足夠（比照 fp_input.v 的既有慣例，
                    // 不依賴 wr_data_count）。
                    if (!pacc_fifo_empty) begin
                        burst_beats   <= next_burst;
                        axi_state     <= ST_AW;
                    end
                end

                ST_AW: begin
                    m_axi_awaddr  <= cur_addr;
                    m_axi_awlen   <= {3'd0, burst_beats - 5'd1};
                    m_axi_awvalid <= 1'b1;
                    if (m_axi_awvalid && m_axi_awready) begin
                        m_axi_awvalid <= 1'b0;
                        m_axi_wdata   <= pacc_fifo_dout;
                        m_axi_wvalid  <= 1'b1;
                        m_axi_wlast   <= (burst_beats == 5'd1);
                        pacc_fifo_rd_en <= 1'b1;
                        beat_cnt_axi  <= 4'd1;
                        axi_state     <= ST_W;
                    end
                end

                // 2026-07-30 修正（build15 上機嚴重回歸的根因）：pacc_fifo_rd_en
                // 是 registered（見上方宣告），rd_en 這拍 fire，pacc_fifo_dout
                // 要等「下一拍」才會真的翻新到下一筆（FWFT 語意）。原本兩個
                // 分支只看 !pacc_fifo_empty 就直接再抓一次 dout，在下游
                // WREADY 連續多拍為 1（真正的 fifo_generator 就是這樣，行為級
                // 測試台故意讓 WREADY 不會連續兩拍為 1，這裡的 bug 一直被
                // 蓋住)時，會在 rd_en 剛 fire、dout 都還沒翻新的那一拍又抓一次
                // ——等於同一筆資料被重複送出兩次、實際少送一筆，導致送出的
                // AXI beat 數量比消耗的 pacc_fifo 筆數多，`axi_done` 提早
                // fire、封包還沒送完就被當作下一包的 header 解析，造成寫入
                // 位址被解析成波形資料本身（見 NOTES.md 2026-07-30「build15
                // 嚴重回歸根因」章節，xsim 用 ALWAYS_READY=1 重現＋驗證此修法）
                // 。加 !pacc_fifo_rd_en 讓 FSM 等 rd_en 完全「退回 0」（代表
                // dout 已經翻新到下一筆）才敢再抓，代價是每拍多等 1 cycle
                // （AXI 側上限變成 1 beat/2 cycle=800MB/s，仍遠高於 pacc_fifo
                // 實際填入速度，不影響吞吐瓶頸）。
                ST_W: begin
                    if (m_axi_wvalid && m_axi_wready) begin
                        if (m_axi_wlast) begin
                            m_axi_wvalid <= 1'b0;
                            m_axi_wlast  <= 1'b0;
                            axi_state    <= ST_RESP;
                        end else if (!pacc_fifo_empty && !pacc_fifo_rd_en) begin
                            m_axi_wdata  <= pacc_fifo_dout;
                            m_axi_wlast  <= (beat_cnt_axi + 4'd1 == burst_beats[3:0]);
                            pacc_fifo_rd_en <= 1'b1;
                            beat_cnt_axi <= beat_cnt_axi + 4'd1;
                        end else begin
                            m_axi_wvalid <= 1'b0;   // 暫停等 FIFO（含 rd_en 剛 fire、dout 還沒真的翻新這一拍）
                        end
                    end else if (!m_axi_wvalid && !pacc_fifo_empty && !pacc_fifo_rd_en) begin
                        m_axi_wdata  <= pacc_fifo_dout;
                        m_axi_wvalid <= 1'b1;
                        m_axi_wlast  <= (beat_cnt_axi == burst_beats[3:0] - 4'd1);
                        pacc_fifo_rd_en <= 1'b1;
                        beat_cnt_axi <= beat_cnt_axi + 4'd1;
                    end
                end

                ST_RESP: begin
                    if (m_axi_bvalid) begin
                        debug_bresp     <= m_axi_bresp;
                        cur_addr        <= cur_addr + ({27'd0, burst_beats} << 4);
                        remaining_beats <= remaining_beats - {27'd0, burst_beats};
                        if (remaining_beats <= {27'd0, burst_beats}) begin
                            axi_done  <= 1'b1;
                            axi_state <= ST_IDLE_A;
                        end else begin
                            axi_state <= ST_FILL;
                        end
                    end
                end

                default: axi_state <= ST_IDLE_A;
            endcase
        end
    end

endmodule
`default_nettype wire
