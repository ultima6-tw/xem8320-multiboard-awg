`timescale 1ns/1ps
`default_nettype none

// flash_payload_cdc — dispatcher_0(sys_clk, 64-bit) -> fpga_flash_ctrl_0
// (okClk, 32-bit) 的 CDC + 寬度轉換
//
// 2026-07-09 新增，移植自 awg-test-step-15c 的 T3 驗證（60/60、30/30 輪
// burn-in 皆 0 錯誤，見 awg-test-step-15c/PROJECT.md、
// awg-test-step-16/PROJECT.md 第 25 節）。
//
// 寬度轉換信心考量：xpm_fifo_async 原生支援 WRITE_DATA_WIDTH/
// READ_DATA_WIDTH 不同寬度（見 awg-test-step-14.3/sim/xpm_sim/
// xpm_fifo.sv 的 WIDTH_RATIO 邏輯），但沒有把握內部窄字組的輸出順序
// （先出低位半字還是高位半字）是否跟 fp_input.v pair-acc 的組裝順序
// 對稱——沒有查到明確可信的文件根據，屬於「無參考、需自行推斷」的
// 情況。為了不用猜的，這裡改成用一個「先在 sys_clk 域自己拆成兩次
// 32-bit 寫入」的做法，FIFO 本身維持跟 T3 完全一樣的同寬度用法
// （32-bit -> 32-bit，只做 CDC 不做寬度轉換），順序完全由這裡的 RTL
// 自己控制，不依賴 FIFO 內部未驗證過的行為。
//
// 拆字順序：跟 fp_input.v 的 {pi_data(hi), lo_word} 組裝方式對稱還原——
// 一個 64-bit beat 的 [31:0] 是先收到的字（lo），[63:32] 是後收到的字
// (hi)；這裡先送 lo 進 FIFO，再送 hi，順序跟原始 32-bit word 收到的
// 先後一致。

module flash_payload_cdc (
    input  wire        sys_clk,
    input  wire        ok_clk,
    input  wire        rst,          // 單一 async rst，xpm_fifo_async 內部
                                      // 自己處理雙時脈域同步（CDC_SYNC_STAGES=2）

    // sys_clk 側輸入（來自 dispatcher_0/flash_tdata,flash_tvalid,flash_tready）
    input  wire [63:0] in_data,
    input  wire        in_valid,
    output wire        in_ready,

    // okClk 側輸出（送進 fpga_flash_ctrl_0/pipe_wdata,pipe_wvalid）
    output wire [31:0] out_data,
    output wire        out_valid,

    // ── payload 收滿偵測（2026-07-09 新增，見 PROJECT.md 第 28 節）───────
    // okClk 域計數送出的 word 數，收滿 64 個（=256 bytes，跟
    // fpga_flash_ctrl.v 的 word_cnt 語意一致）時脈衝一次，供
    // flash_save_gate 用來確保「payload 真的收完才允許觸發寫入」，
    // 避免 SAVE 觸發訊號（走另一條延遲較短的 trigger_cdc 路徑）比
    // payload 資料還早到達 fpga_flash_ctrl_0 造成的競爭問題。
    output wire        payload_complete,

    // ── 除錯輸出（2026-07-09 新增，burn-in 發現寫入路徑 100% 失敗後
    //    加來定位問題，見 PROJECT.md 第 27 節）─────────────────────────
    output wire        dbg_fifo_wr_en,       // sys_clk：unpack 狀態機是否真的觸發 FIFO 寫入
    output wire        dbg_unpack_half,      // sys_clk：拆字狀態機目前狀態
    output wire        dbg_fifo_prog_full,   // sys_clk：FIFO 寫側 almost-full（影響 in_ready）
    output wire        dbg_fifo_wr_rst_busy  // sys_clk：FIFO 寫側是否還在 reset 恢復期（此時寫入會被吃掉）
);

// ── sys_clk 域：64-bit beat 拆成兩次 32-bit 寫入 ─────────────────────────
reg        unpack_half;   // 0 = 準備接收新 beat；1 = 上一個 beat 的 hi 半字還沒寫完
reg [31:0] latched_hi;
reg        fifo_wr_en;
reg [31:0] fifo_din;

// 2026-07-11 bugfix（見 PROJECT.md 第 32 節）：dispatcher_0 的 RT_FLASH
// case 會把封包的 beat0（header，含 dest_id/pkt_len/type，不是真正的
// payload）也轉發過來——T_FLASH_WRITE_DATA 的 pkt_len=33（1 header + 32
// 資料 beat），dispatcher 因此總共送 33 個 beat 進來，但這裡原本沒有
// 分辨「第一個 beat 是 header」，把 33 個 beat 全部當資料拆解成 66 個
// word，比 payload_complete 預期的 64 個 word 多出 2 個。多出來的 2 個
// header word 會擠掉真正 payload 尾端 2 個 word（payload_complete 提前
// 觸發，剩下的 2 個真正資料 word 變成下一次傳輸開頭的雜訊），造成
// fpga_flash_ctrl.v 的 word_cnt 索引每次 save 都累積偏移、越測越歪。
// 修法：新增 expect_header/beat_cnt，固定丟棄每次傳輸的第一個 beat
// （header，不寫進 FIFO），之後收滿 32 個真正的資料 beat（=64 個 word）
// 才算一個完整 payload，下一個 beat 又重新視為新封包的 header。
reg        expect_header = 1'b1;   // 1 = 下一個 beat 是 header，要丟棄
reg [4:0]  beat_cnt      = 5'd0;   // 已收到的「真正資料」beat 數（0-31）

wire fifo_prog_full;
wire fifo_wr_rst_busy;

assign in_ready = !unpack_half && !fifo_prog_full;

assign dbg_fifo_wr_en      = fifo_wr_en;
assign dbg_unpack_half     = unpack_half;
assign dbg_fifo_prog_full  = fifo_prog_full;
assign dbg_fifo_wr_rst_busy = fifo_wr_rst_busy;

always @(posedge sys_clk) begin
    fifo_wr_en <= 1'b0;
    if (rst) begin
        unpack_half   <= 1'b0;
        latched_hi    <= 32'd0;
        expect_header <= 1'b1;
        beat_cnt      <= 5'd0;
    end else if (in_valid && in_ready) begin
        if (expect_header) begin
            // 這個 beat 是封包 header，丟棄，不拆解、不寫進 FIFO
            expect_header <= 1'b0;
        end else begin
            fifo_din    <= in_data[31:0];   // lo（先收到的字）先寫
            fifo_wr_en  <= 1'b1;
            latched_hi  <= in_data[63:32];  // hi（後收到的字）鎖存，下個 cycle 寫
            unpack_half <= 1'b1;
            if (beat_cnt == 5'd31) begin
                beat_cnt      <= 5'd0;
                expect_header <= 1'b1;   // 32 個資料 beat 收完，下一個 beat 是新封包的 header
            end else begin
                beat_cnt <= beat_cnt + 5'd1;
            end
        end
    end else if (unpack_half) begin
        fifo_din    <= latched_hi;
        fifo_wr_en  <= 1'b1;
        unpack_half <= 1'b0;
    end
end

// ── CDC：32-bit -> 32-bit（跟 T3 的 t3_fifo_sys_to_ok.v 完全同寬度用法）──
wire empty;

assign out_valid = !empty;

// ── okClk 域：word 計數，偵測 payload 是否收滿 64 個 word ────────────────
reg [5:0] ok_word_cnt = 6'd0;
always @(posedge ok_clk) begin
    if (rst) begin
        ok_word_cnt <= 6'd0;
    end else if (out_valid) begin
        ok_word_cnt <= ok_word_cnt + 1'b1;
    end
end
assign payload_complete = out_valid && (ok_word_cnt == 6'd63);

xpm_fifo_async #(
    .FIFO_WRITE_DEPTH (128),
    .WRITE_DATA_WIDTH (32),
    .READ_DATA_WIDTH  (32),
    .READ_MODE        ("fwft"),
    .PROG_FULL_THRESH (120),
    .USE_ADV_FEATURES ("0002"),   // bit1 = 開 prog_full（比照 fp_input.v/T3）
    .DOUT_RESET_VALUE ("0"),
    .CDC_SYNC_STAGES  (2),
    .RELATED_CLOCKS   (0)
) u_fifo (
    .wr_clk       (sys_clk),
    .rd_clk       (ok_clk),
    .rst          (rst),
    .sleep        (1'b0),
    .wr_en        (fifo_wr_en),
    .din          (fifo_din),
    .rd_en        (!empty),
    .dout         (out_data),
    .empty        (empty),
    .full         (),
    .prog_full    (fifo_prog_full),
    .wr_rst_busy  (fifo_wr_rst_busy),
    .rd_rst_busy  (),
    .data_valid   (),
    .wr_data_count(),
    .rd_data_count(),
    .almost_empty (),
    .almost_full  (),
    .wr_ack       (),
    .overflow     (),
    .underflow    (),
    .injectsbiterr(1'b0),
    .injectdbiterr(1'b0),
    .sbiterr      (),
    .dbiterr      ()
);

endmodule

`default_nettype wire
