`timescale 1ns/1ps
`default_nettype none

// status_reply_capture — 統一讀取/寫入架構的查詢回覆 BTPipeOut 模組
// （PO_STATUS_REPLY = 0xA1，2026-07-27 新增，見 PROJECT.md「統一讀取/
// 寫入架構 — 完整規格」小節）。
//
// 表 index 對應（61 entries + 3 個保留 0）：
//   [0]      = {8'd0, au_reply_query_type[7:0], au_reply_src[15:0]}
//   [1..60]  = au_reply_data[1919:0]（60 個 32-bit word，
//              word[i]=au_reply_data[i*32 +: 32]，跟 local_reg_
//              handler.v/aurora_reply_tx.v 兩端的 word 排列一致）
//   [61..63] = 0（保留）
//
// 2026-07-29 CDC 修法 v2（見 NOTES.md 同日章節）：v1 用「sys_clk 域
// wr_ptr 自由跑 + FIFO」的做法解決了原本的 CDC 違規（po_ep_read 這個
// okClk 域訊號被 sys_clk 域無同步取樣），但引入新問題——host 每次開始
// 讀取時，第一個 word 對應到 tbl[] 哪一格完全不受控制（取決於 wr_ptr
// 自由跑到哪裡）。上機驗證顯示三片板子讀出結果彼此一致（證明 CDC 本身
// 修好了，不再有 metastability），但都固定停在同一個錯誤的相位偏移，
// 不是 tbl[0]——這種「可重現但對不齊」不符合這個介面本來的語意：host
// 每次讀取都必須從 tbl[0] 開始，才能拿到一份完整、可預期的快照。
//
// v2 改回「host 讀取驅動、okClk 域定址」的原始設計精神（v1 之前的版本
// 語意是對的，只是跑錯時鐘域），資料來源正確地留在 sys_clk 域，用官方
// `xpm_cdc_handshake`（跟 aurora_reply_tx.v 處理 phase_acc 用的同一個
// 巨集、同一種「凍結快照→送出→等對方收到→下一筆」pattern）把整張表
// 搬進 okClk 域的暫存器組，`po_ep_read`/`po_ep_datain` 這條 handshake
// 介面完全留在 okClk 域，不再有任何跨域取樣。
//
// `xpm_cdc_handshake` 的 WIDTH 上限是 1024（Xilinx 官方文件），整張表
// 61 個有效 word（1952-bit）超過上限，拆成兩個 handshake channel：
//   chunk_a：tbl[0..31]（32 word，1024-bit，剛好頂到上限）
//   chunk_b：tbl[32..60]（29 word，928-bit）
// 兩個 channel 共用同一個 sys_clk 側送出時序（`hs_send_r`），要等兩邊
// 都確認收到才會鎖存下一次快照，確保兩個 channel 屬於同一個瞬間的
// 快照，不會有 A、B 兩塊來自不同時間點的「撕裂」問題。
//
// okClk 側：`tbl_okclk[]` 只在每個讀取 pass 的第一拍（rd_ptr==0）重新
// 鎖存一次最新快照（index 0 當拍直接繞過暫存器讀最新的 chunk_a_words[0]，
// 避免 nonblocking assignment 順序造成的「這拍鎖存、這拍卻讀到鎖存前
// 舊值」問題），同一個 pass 剩下的 60 個 word 都從這份鎖存值讀出——
// 保證同一次 host 讀取（不管背後 CDC 快照什麼時候更新）看到的是同一份、
// 內部自洽的快照，不會發生「pass 讀到一半快照換了」的撕裂讀取，從
// reset 後第一次讀取就正確（不需要先跑過一輪才「暖機」）。

module status_reply_capture (
    input  wire         sys_clk,
    input  wire         sys_rst,

    input  wire [15:0]  au_reply_src,
    input  wire [7:0]   au_reply_query_type,
    input  wire [1919:0] au_reply_data,   // REPLY_MAX_WORDS(30)*64

    // okClk 域 BTPipeOut 介面（直接接 fp0/poa1_ep_datain/poa1_ep_read，
    // 不需要中間的 FIFO wrapper）
    input  wire         ok_clk,
    input  wire         ok_rst,          // okClk 域已同步的 reset（okclk_rst_sync 產生）
    output reg  [31:0]  po_ep_datain,
    input  wire         po_ep_read
);

    // ── sys_clk 域：組表格（純組合邏輯）────────────────────────────────
    wire [31:0] tbl [0:60];
    assign tbl[0] = {8'd0, au_reply_query_type, au_reply_src};
    genvar gi;
    generate
        for (gi = 0; gi < 60; gi = gi + 1) begin : g_reply
            assign tbl[1 + gi] = au_reply_data[gi*32 +: 32];
        end
    endgenerate

    // ── sys_clk 域：快照 pump（跟 aurora_reply_tx.v phase_acc CDC 同一種
    //    pattern，兩個 channel 共用同一個 send 時序，等兩邊都收到才鎖存
    //    下一次快照）──────────────────────────────────────────────────
    reg [1023:0] chunk_a_snap_r;
    reg [927:0]  chunk_b_snap_r;
    reg          hs_send_r;
    reg          chunk_a_rcv_seen, chunk_b_rcv_seen;
    wire         chunk_a_rcv, chunk_b_rcv;
    wire [1023:0] chunk_a_synced;
    wire [927:0]  chunk_b_synced;

    wire [1023:0] chunk_a_live = {tbl[31],tbl[30],tbl[29],tbl[28],tbl[27],tbl[26],tbl[25],tbl[24],
                                   tbl[23],tbl[22],tbl[21],tbl[20],tbl[19],tbl[18],tbl[17],tbl[16],
                                   tbl[15],tbl[14],tbl[13],tbl[12],tbl[11],tbl[10],tbl[9], tbl[8],
                                   tbl[7], tbl[6], tbl[5], tbl[4], tbl[3], tbl[2], tbl[1], tbl[0]};
    wire [927:0]  chunk_b_live = {tbl[60],tbl[59],tbl[58],tbl[57],tbl[56],tbl[55],tbl[54],tbl[53],
                                   tbl[52],tbl[51],tbl[50],tbl[49],tbl[48],tbl[47],tbl[46],tbl[45],
                                   tbl[44],tbl[43],tbl[42],tbl[41],tbl[40],tbl[39],tbl[38],tbl[37],
                                   tbl[36],tbl[35],tbl[34],tbl[33],tbl[32]};

    always @(posedge sys_clk) begin
        if (sys_rst) begin
            hs_send_r        <= 1'b0;
            chunk_a_rcv_seen <= 1'b0;
            chunk_b_rcv_seen <= 1'b0;
        end else if (!hs_send_r) begin
            // 凍結目前值，開始送出兩個 channel
            chunk_a_snap_r   <= chunk_a_live;
            chunk_b_snap_r   <= chunk_b_live;
            hs_send_r        <= 1'b1;
            chunk_a_rcv_seen <= 1'b0;
            chunk_b_rcv_seen <= 1'b0;
        end else begin
            if (chunk_a_rcv) chunk_a_rcv_seen <= 1'b1;
            if (chunk_b_rcv) chunk_b_rcv_seen <= 1'b1;
            // 兩個 channel 都確認收到（含這一拍剛好收到的），才準備下一次快照
            if ((chunk_a_rcv_seen || chunk_a_rcv) && (chunk_b_rcv_seen || chunk_b_rcv))
                hs_send_r <= 1'b0;
        end
    end

    xpm_cdc_handshake #(
        .DEST_EXT_HSK (0),   // 目的端自動 ack，不需要 okClk 端額外邏輯
        .DEST_SYNC_FF (4),
        .SRC_SYNC_FF  (4),
        .WIDTH        (1024)
    ) u_chunk_a_cdc (
        .src_clk  (sys_clk),
        .src_in   (chunk_a_snap_r),
        .src_send (hs_send_r),
        .src_rcv  (chunk_a_rcv),
        .dest_clk (ok_clk),
        .dest_out (chunk_a_synced),
        .dest_req (),
        .dest_ack (1'b0)
    );

    xpm_cdc_handshake #(
        .DEST_EXT_HSK (0),
        .DEST_SYNC_FF (4),
        .SRC_SYNC_FF  (4),
        .WIDTH        (928)
    ) u_chunk_b_cdc (
        .src_clk  (sys_clk),
        .src_in   (chunk_b_snap_r),
        .src_send (hs_send_r),
        .src_rcv  (chunk_b_rcv),
        .dest_clk (ok_clk),
        .dest_out (chunk_b_synced),
        .dest_req (),
        .dest_ack (1'b0)
    );

    // ── okClk 域：解包 + pass 邊界鎖存 + rd_ptr 定址 ──────────────────
    wire [31:0] chunk_a_words [0:31];
    wire [31:0] chunk_b_words [0:28];
    genvar gj;
    generate
        for (gj = 0; gj < 32; gj = gj + 1) begin : g_unpack_a
            assign chunk_a_words[gj] = chunk_a_synced[gj*32 +: 32];
        end
        for (gj = 0; gj < 29; gj = gj + 1) begin : g_unpack_b
            assign chunk_b_words[gj] = chunk_b_synced[gj*32 +: 32];
        end
    endgenerate

    reg [31:0] tbl_okclk [1:60];
    reg [5:0]  rd_ptr;
    integer    k;

    // rd_ptr==0：這一拍剛好在鎖存新快照，直接繞過暫存器讀最新的
    // chunk_a_words[0]（nonblocking assignment 同一拍寫入 tbl_okclk 的話，
    // 讀到的會是鎖存前的舊值，不是這次剛鎖存的新值）
    wire [31:0] tbl_okclk_read =
        (rd_ptr == 6'd0)  ? chunk_a_words[0] :
        (rd_ptr <= 6'd60) ? tbl_okclk[rd_ptr] :
                             32'd0;   // rd_ptr 61..63：保留位

    always @(posedge ok_clk) begin
        if (ok_rst) begin
            rd_ptr       <= 6'd0;
            po_ep_datain <= 32'd0;
        end else if (po_ep_read) begin
            po_ep_datain <= tbl_okclk_read;
            if (rd_ptr == 6'd0) begin
                for (k = 1; k < 32; k = k + 1) tbl_okclk[k]    <= chunk_a_words[k];
                for (k = 0; k < 29; k = k + 1) tbl_okclk[32+k] <= chunk_b_words[k];
            end
            rd_ptr <= rd_ptr + 6'd1;
        end
    end

endmodule
`default_nettype wire
