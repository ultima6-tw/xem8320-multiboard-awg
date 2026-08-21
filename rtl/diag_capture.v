`timescale 1ns/1ps
`default_nettype none

// diag_capture — Step 14.3 診斷 PipeOut 模組
//
// 2026-07-29：**這是 step-16 專屬的本地副本**（原本是
// awg-test-step-14/rtl/diag_capture.v 共用檔案，這次為了修一個真實存在
// 的 CDC bug 而 fork，見下方說明；範圍刻意限縮在這個專案，
// awg-test-step-14 不受影響）。
//
// 將 fp_input raw capture（12 × 32-bit）和 LRH capture（17 × 32-bit）
// 組成 32-entry 診斷表，透過 BTPipeOut 0xA0 讓 host 一次讀出 1024 bytes。
//
// diag 表 index 對應：
//   [0..11]  = fp_input raw pi_data（half_reset 後逐 word 捕捉）
//   [12..23] = LRH rx beat0-5（lo/hi interleaved，dispatcher 送給 LRH 的封包）
//   [24]     = LRH 擷取的 au_ddr4_addr
//   [25..28] = LRH 擷取的 au_ddr4_w0-w3
//   [29..31] = 0（保留）
//
// 2026-07-29 CDC 修法 v2（見 NOTES.md 同日章節，跟 status_reply_
// capture.v 同一批、同一個理由改版）：v1 用「sys_clk 域 wr_ptr 自由跑 +
// FIFO」的做法解決了原本的 CDC 違規，但引入新問題——host 每次開始讀取
// 時第一個 word 對應到 tbl[] 哪一格完全不受控制，不符合這個介面「每次
// 讀取都要從 tbl[0] 開始」的語意（詳細理由見 status_reply_capture.v
// 檔頭，這裡不重複）。
//
// v2 改回「host 讀取驅動、okClk 域定址」，資料來源留在 sys_clk 域，用
// 官方 `xpm_cdc_handshake` 把整張表搬進 okClk 域的暫存器組。這裡只有
// 29 個有效 word（928-bit），一個 handshake channel（WIDTH=928，未超過
// 1024 上限）就夠，不用像 status_reply_capture.v 拆兩個 channel。
// `tbl_okclk[]` 只在每個讀取 pass 的第一拍（rd_ptr==0）重新鎖存一次最新
// 快照（index 0 當拍直接繞過暫存器讀最新值，理由同 status_reply_
// capture.v），保證同一次 host 讀取內部自洽，從 reset 後第一次讀取就
// 正確。

module diag_capture (
    input  wire         sys_clk,
    input  wire         sys_rst,

    // 來自 fp_input：raw pi_data 12 × 32-bit
    input  wire [383:0] raw_data,

    // 來自 local_reg_handler：rx beats + DDR4 輸出，17 × 32-bit
    input  wire [543:0] lrh_data,

    // okClk 域 BTPipeOut 介面（直接接 fp0/poa0_ep_datain/poa0_ep_read，
    // 不需要中間的 FIFO wrapper）
    input  wire         ok_clk,
    input  wire         ok_rst,          // okClk 域已同步的 reset（okclk_rst_sync 產生）
    output reg  [31:0]  po_ep_datain,
    input  wire         po_ep_read
);

    // ── sys_clk 域：組表格（純組合邏輯）────────────────────────────────
    wire [31:0] tbl [0:28];
    genvar gi;
    generate
        for (gi = 0; gi < 12; gi = gi + 1) begin : g_raw
            assign tbl[gi] = raw_data[gi*32 +: 32];
        end
        for (gi = 0; gi < 17; gi = gi + 1) begin : g_lrh
            assign tbl[12 + gi] = lrh_data[gi*32 +: 32];
        end
    endgenerate

    // ── sys_clk 域：快照 pump（跟 aurora_reply_tx.v phase_acc CDC 同一種
    //    pattern）───────────────────────────────────────────────────────
    reg [927:0] chunk_snap_r;
    reg         hs_send_r;
    wire        chunk_rcv;
    wire [927:0] chunk_synced;

    wire [927:0] chunk_live = {tbl[28],tbl[27],tbl[26],tbl[25],tbl[24],tbl[23],tbl[22],tbl[21],
                                tbl[20],tbl[19],tbl[18],tbl[17],tbl[16],tbl[15],tbl[14],tbl[13],
                                tbl[12],tbl[11],tbl[10],tbl[9], tbl[8], tbl[7], tbl[6], tbl[5],
                                tbl[4], tbl[3], tbl[2], tbl[1], tbl[0]};

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            hs_send_r <= 1'b0;
        end else if (!hs_send_r) begin
            chunk_snap_r <= chunk_live;
            hs_send_r    <= 1'b1;
        end else if (chunk_rcv) begin
            hs_send_r <= 1'b0;
        end
    end

    xpm_cdc_handshake #(
        .DEST_EXT_HSK (0),
        .DEST_SYNC_FF (4),
        .SRC_SYNC_FF  (4),
        .WIDTH        (928)
    ) u_chunk_cdc (
        .src_clk  (sys_clk),
        .src_in   (chunk_snap_r),
        .src_send (hs_send_r),
        .src_rcv  (chunk_rcv),
        .dest_clk (ok_clk),
        .dest_out (chunk_synced),
        .dest_req (),
        .dest_ack (1'b0)
    );

    // ── okClk 域：解包 + pass 邊界鎖存 + rd_ptr 定址 ──────────────────
    wire [31:0] chunk_words [0:28];
    genvar gj;
    generate
        for (gj = 0; gj < 29; gj = gj + 1) begin : g_unpack
            assign chunk_words[gj] = chunk_synced[gj*32 +: 32];
        end
    endgenerate

    reg [31:0] tbl_okclk [1:28];
    reg [4:0]  rd_ptr;
    integer    k;

    wire [31:0] tbl_okclk_read =
        (rd_ptr == 5'd0)  ? chunk_words[0] :
        (rd_ptr <= 5'd28) ? tbl_okclk[rd_ptr] :
                             32'd0;   // rd_ptr 29..31：保留位

    always @(posedge ok_clk) begin
        if (ok_rst) begin
            rd_ptr       <= 5'd0;
            po_ep_datain <= 32'd0;
        end else if (po_ep_read) begin
            po_ep_datain <= tbl_okclk_read;
            if (rd_ptr == 5'd0) begin
                for (k = 1; k < 29; k = k + 1) tbl_okclk[k] <= chunk_words[k];
            end
            rd_ptr <= rd_ptr + 5'd1;
        end
    end

endmodule
`default_nettype wire
