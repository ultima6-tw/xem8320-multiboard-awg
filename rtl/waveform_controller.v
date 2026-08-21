`timescale 1ns/1ps
`default_nettype none

// waveform_controller: per-channel waveform sequencer with atomic trigger switching.
//
// Manages two ddr4_stream_readers (A=active, B=standby) and two FIFOs.
// On trigger: instantly swaps which FIFO feeds the DAC, flushes old FIFO,
// loads next waveform into new standby. Runs on dac_clk.
//
// Phase sync: dac_valid and fifo_rd_en are gated by all_ready (from sync_start module).
// all_ready is asserted only when all active channels' FIFO_A have data,
// ensuring all channels start outputting sample 0 simultaneously.
//
// 2026-07-27（Group-based Trigger 架構）：trigger_mask input port 移除
// ——masking 邏輯上移到 group_trig_select.v 這一層（在 sw_trigger/
// ext_trigger 抵達這個模組之前就已經依 group 分組過濾過），這個模組
// 收到的 sw_trigger/ext_trigger 已經是「確定要動作」的訊號，不需要再
// 重複判斷一次。
//
// ── 2026-07-26 本地修改：flush 完成條件加入等待 reader 確認閒置 ──────
// 根因：ddr4_stream_reader.v 的 S_AR/S_R/S_WRITE4（burst 進行中）完全
// 不檢查 play_en，只有 S_IDLE/S_WAIT 會檢查。如果 trigger 發生時
// reader 剛好在跑 burst，play_en 拉低對它沒用，burst 跑完時 play_en
// 可能已經被這裡重新拉高，reader 永遠不會經過 S_IDLE，rd_ptr/samp_cnt
// 不會歸零——剛 flush 完的 FIFO 被填入從舊位置接續的樣本，不是樣本 0。
// 這是同一塊板子上不同 channel 重複觸發後相位偶發性大幅跳動的根因
// （見 PROJECT.md「重大突破：用 Opus 深度推理」小節完整分析）。
// 修法：新增 ST_WAIT_READER_IDLE 狀態，trigger 發生時先只停 reader
// （play_en<=0），不立刻 flush；等對應 reader 的 idle 訊號（2-flop
// synced 進 dac_clk，見下方 ra_idle_s1/s2）確認為 1，才真正發 flush
// 脈衝，之後流程（等 FIFO busy 清除、重啟 reader）完全不變。
//
// ── 2026-07-04 本地修改（14.3b）：simple_dc_fifo -> xpm_fifo_async，加 busy 檢查 ──
// 原本用 FLUSH_CYCLES（固定 8 cycle）等 FIFO flush 完成，是針對
// simple_dc_fifo.v 自己的重置行為調校的估計值。換成官方 xpm_fifo_async 後，
// 官方 IP 內部有自己的重置流程（wr_rst_busy/rd_rst_busy 會指示重置有沒有真的
// 完成），改成除了維持原本的 FLUSH_CYCLES 最小等待時間之外，額外要求
// wr_rst_busy/rd_rst_busy 都確認降回 0 才真正離開 flush 狀態——雙重保護，
// 不是單純信任一個猜測的固定 cycle 數。
//
// wr_rst_busy 是 wr_clk（ddr4_ui_clk, ~300MHz）domain 的訊號，跟本模組
// 所在的 dac_clk domain 不同，用標準 2-flop synchronizer 帶進來（level 訊號，
// 不需要 pulse stretch）。rd_rst_busy 已經是 dac_clk domain（跟 FIFO 的
// rd_clk 同一個），不需要額外同步，直接用。
//
// CDC notes:
//   - ra/rb play_en: 2-FF synced inside ddr4_stream_reader (ASYNC_REG)
//   - ra/rb start_addr/wave_len: quasi-static, stable before play_en asserts
//   - fifo_a/b_flush: xpm_fifo_async 內部 xpm_fifo_rst 子模組處理雙時脈同步
//   - fifo_a/b_wr_rst_busy: 本檔案內 2-FF synced（見上）
//   - XDC: set_false_path from mmcm0_clk0 to mmcm_clkout0 covers addr/len paths

module waveform_controller #(
    parameter CH_ID        = 0,    // channel ID for list write decode
    parameter FLUSH_CYCLES = 8,    // clk cycles to hold FIFO flush (~80 ns @ 100MHz)
    parameter MUX_SEL_INIT = 1'b0  // initial mux_sel: 0=FIFO_A active, 1=FIFO_B active
) (
    input  wire        clk,       // dac_clk
    input  wire        rst,       // sync, active high

    // ── List write interface (clk domain, from FP TI pulse) ────────────────
    // list_wr_sel[1:0] = channel, list_wr_sel[4:2] = slot (0-7)
    input  wire [4:0]  list_wr_sel,
    input  wire [31:0] list_wr_addr,
    input  wire [31:0] list_wr_len,
    input  wire        list_wr_en,   // single-cycle pulse

    // ── List depth (quasi-static, 1-8) ────────────────────────────────────
    input  wire [2:0]  list_depth,

    // ── Control ────────────────────────────────────────────────────────────
    input  wire        play_en,
    input  wire        sw_trigger,    // single-cycle pulse from FP TI
    input  wire        ext_trigger,   // tie to 0
    input  wire        all_ready,     // global: all active ch FIFOs have data

    // ── reinit_req（2026-07-14 新增）─────────────────────────────────────────
    // single-cycle pulse，跟 rst 同等優先權（不管當下在哪個 state，收到
    // 立刻回到 ST_IDLE），但這是 host 可控制的獨立指令，不是硬體重置。
    // 觸發時：強制 0V（play_pos<=-1）、兩個 FIFO 都下 flush、清空這個
    // channel 的 playlist（list_addr/list_len 全部歸零）。
    input  wire        reinit_req,

    // ── Reader A outputs (to ddr4_stream_reader, quasi-static) ─────────────
    output reg  [31:0] ra_start_addr,
    output reg  [31:0] ra_wave_len,
    output reg         ra_play_en,

    // ── Reader B outputs ────────────────────────────────────────────────────
    output reg  [31:0] rb_start_addr,
    output reg  [31:0] rb_wave_len,
    output reg         rb_play_en,

    // ── FIFO flush（接 xpm_fifo_async 的合併 rst，內部 xpm_fifo_rst 處理雙時脈同步）
    output reg         fifo_a_flush,
    output reg         fifo_b_flush,

    // ── FIFO reset-busy 回報（2026-07-04 新增）───────────────────────────────
    // wr_rst_busy: ddr4_ui_clk domain（本模組內 2-FF sync 後使用）
    // rd_rst_busy: dac_clk domain（跟本模組同 domain，直接用）
    input  wire        fifo_a_wr_rst_busy,
    input  wire        fifo_a_rd_rst_busy,
    input  wire        fifo_b_wr_rst_busy,
    input  wire        fifo_b_rd_rst_busy,

    // ── reader idle（2026-07-26 新增）───────────────────────────────────
    // 來自 reader_a/reader_b（ddr4_stream_reader.v 的 idle output，
    // ui_clk domain，raw，本檔案內做 2-flop sync）。高電位代表這個
    // reader 真的閒置在 S_IDLE，flush 才可以視為安全。
    input  wire        ra_idle,
    input  wire        rb_idle,

    // ── FIFO read side inputs (rd_clk = dac_clk) ───────────────────────────
    input  wire [31:0] fifo_a_dout,
    input  wire        fifo_a_empty,
    input  wire [31:0] fifo_b_dout,
    input  wire        fifo_b_empty,

    // ── FIFO read enables ───────────────────────────────────────────────────
    output wire        fifo_a_rd_en,
    output wire        fifo_b_rd_en,

    // ── DAC interface (dac_clk domain) ─────────────────────────────────────
    output wire [31:0] dac_data,
    output wire        dac_valid,

    // ── Status ──────────────────────────────────────────────────────────────
    output reg  [2:0]  current_idx,  // currently playing slot
    output reg  [2:0]  next_idx,     // standby slot (for host status read)
    output reg         mux_sel,      // 0=FIFO_A active, 1=FIFO_B active

    // ── play_pos（2026-07-14 新增）─────────────────────────────────────────
    // current_idx/next_idx 是「控制用」，trigger 當下就立刻更新，不代表
    // DAC 現在真正輸出哪個 slot（今天實測發現 master/slave 在這個回報值
    // 上會差一拍，且兩者差的拍數還不一樣，不能拿 current_idx 當作精準
    // 判斷依據）。play_pos 改成「純粹數 trigger 事件次數」：
    //   -1 = 已上膛、還沒被真正 trigger 過
    //    0..list_depth-1 = 第幾次 trigger 生效（mux_sel 真的翻轉那一拍
    //    才 +1），因為 slot 填入順序保證是嚴格依序 0,1,2,...,depth-1
    //    循環，trigger 次數 mod depth 直接就是目前真正在播的 slot 編號，
    //    不需要額外記錄「填入時是哪個 slot」。
    output reg  signed [3:0] play_pos,

    // ── dac_samp_idx（2026-08-18 新增）───────────────────────────────────
    // 目前送到 DAC 的樣本，在目前這個 slot 的 buffer 裡是第幾個
    // sample（0..list_len[current_idx]-1，繞回目前這個 slot 的開頭時
    // 歸零，不是整個模組重置才歸零）。用途：IFFT 梳狀波形＋即時掃頻
    // sine 相減功能，掃頻那邊需要跟 DDR 播放位置精確同步的相位參考，
    // 不能用 ddr4_stream_reader.v 的 samp_cnt（那是 ddr4_ui_clk domain、
    // FIFO 寫入端，跟真正送到 DAC 的這一側之間隔著不確定的緩衝延遲，
    // 沒辦法拿來算精準相位）。純附加邏輯，獨立 always block，不影響
    // 上面任何既有狀態機/CDC 行為。
    output reg  [31:0] dac_samp_idx,

    // ── dac_live（2026-08-18 新增，Opus 覆核指出的安全性必要項）─────────
    // 「現在是不是真的在正常播放」的訊號，= ~force_silence。安全考量：
    // dac_samp_idx 在 force_silence 期間會釘在 0，如果掃頻相減電路只看
    // dac_samp_idx 本身，會在「介面顯示 0V、後端是高壓系統」的靜音狀態
    // 下，誤把「第 0 個 sample 對應的相位」當真、算出一個非零的相減值，
    // 偷偷疊加一個直流偏壓到理應是 0V 的輸出上——違反這個專案「靜音要
    // 真的持續灌 0，不是單純不送新資料」的既有安全設計（見上面
    // force_silence 定義處的說明）。任何要用 dac_samp_idx 做相減運算的
    // 下游模組，都必須額外用這個訊號閘控（dac_live=0 時，相減邏輯要
    // 完全不介入，讓 dac_data 原封不動通過，不能自己再疊加任何東西）。
    output wire dac_live,

    // ── dac_samp_idx_wrap（2026-08-18 新增）───────────────────────────────
    // 1-cycle pulse：dac_samp_idx 這一拍剛好被設回 0（不管是繞到
    // list_len 邊界、還是 trigger 換了新 slot）。掃頻排程器
    // （notch_sweep_ctrl.v）用這個訊號當作「梳狀波形剛好播完一輪」的
    // 時機點去換下一個要抵銷的諧波、notch_sine_gen.v 也用同一個訊號
    // 重新載入相位——確保換頻率永遠發生在 buffer 邊界，不會在播放中途
    // 换，這是相位對齊的關鍵。force_silence 期間也會一直脈動（因為
    // dac_samp_idx 一直被釘在 0），但下游本來就要另外看 dac_live，
    // 這段時間脈動與否不影響正確性。
    output reg dac_samp_idx_wrap
);

// ── wr_rst_busy CDC：ddr4_ui_clk -> dac_clk，標準 2-flop（level 訊號）──────
(* ASYNC_REG = "TRUE" *) reg fifo_a_wr_rst_busy_s1 = 1'b1, fifo_a_wr_rst_busy_s2 = 1'b1;
(* ASYNC_REG = "TRUE" *) reg fifo_b_wr_rst_busy_s1 = 1'b1, fifo_b_wr_rst_busy_s2 = 1'b1;

always @(posedge clk) begin
    fifo_a_wr_rst_busy_s1 <= fifo_a_wr_rst_busy;
    fifo_a_wr_rst_busy_s2 <= fifo_a_wr_rst_busy_s1;
    fifo_b_wr_rst_busy_s1 <= fifo_b_wr_rst_busy;
    fifo_b_wr_rst_busy_s2 <= fifo_b_wr_rst_busy_s1;
end

// rd_rst_busy 已經是 dac_clk domain，直接用（不需要額外同步）
wire fifo_a_busy = fifo_a_wr_rst_busy_s2 | fifo_a_rd_rst_busy;
wire fifo_b_busy = fifo_b_wr_rst_busy_s2 | fifo_b_rd_rst_busy;

// ── reader idle CDC：ddr4_ui_clk -> dac_clk，標準 2-flop（level 訊號，
// 2026-07-26 新增）── 初始值 0（=尚未確認閒置），安全預設值跟
// fifo_x_wr_rst_busy_s1/s2 初始值 1（=忙碌）同一個道理，避免同步器
// 還沒穩定前就誤判成「已經閒置」
(* ASYNC_REG = "TRUE" *) reg ra_idle_s1 = 1'b0, ra_idle_s2 = 1'b0;
(* ASYNC_REG = "TRUE" *) reg rb_idle_s1 = 1'b0, rb_idle_s2 = 1'b0;

always @(posedge clk) begin
    ra_idle_s1 <= ra_idle;
    ra_idle_s2 <= ra_idle_s1;
    rb_idle_s1 <= rb_idle;
    rb_idle_s2 <= rb_idle_s1;
end

// ── List storage ──────────────────────────────────────────────────────────
reg [31:0] list_addr [7:0];
reg [31:0] list_len  [7:0];

integer li;
always @(posedge clk) begin
    if (reinit_req) begin
        for (li = 0; li < 8; li = li + 1) begin
            list_addr[li] <= 32'd0;
            list_len [li] <= 32'd0;
        end
    end else if (list_wr_en && list_wr_sel[1:0] == CH_ID[1:0]) begin
        list_addr[list_wr_sel[4:2]] <= list_wr_addr;
        list_len [list_wr_sel[4:2]] <= list_wr_len;
    end
end

// ── Trigger（2026-07-27 起：masking 已在 group_trig_select.v 做過，這裡
// 收到的 sw_trigger/ext_trigger 已經是確定要動作的訊號）───────────────
wire trigger = sw_trigger | ext_trigger;

// ── Next-index helper ─────────────────────────────────────────────────────
function [2:0] inc_idx;
    input [2:0] idx;
    input [2:0] depth;
    inc_idx = (idx >= depth - 3'd1) ? 3'd0 : idx + 3'd1;
endfunction

// ── 安全機制（2026-07-14 新增）：非「已真正被 trigger 過、正常播放中」
// 的狀態，一律強制輸出 0V，不是單純停止送資料——後端接的是高壓系統，
// dac_valid=0 只代表「不送新資料」，下游會凍結在殘留電壓上，必須主動
// 持續灌 0 才會確實輸出 0V。三種情況都要擋：rst（開機/重置瞬間）、
// !play_en（明確停止播放）、play_pos==-1（已上膛但還沒真正被 trigger
// 過的待機期間）。
wire force_silence = rst | ~play_en | (play_pos == -4'sd1);

// dac_live：見上面 port 宣告處的安全性說明，單純是 force_silence 的反相，
// 給下游（掃頻相減電路）閘控用，不影響這個模組自己的任何行為。
assign dac_live = ~force_silence;

// ── MUX: 0=FIFO_A feeds DAC, 1=FIFO_B feeds DAC ─────────────────────────
assign dac_data     = force_silence ? 32'd0 : (mux_sel ? fifo_b_dout   : fifo_a_dout);
// all_ready gates dac_valid and rd_en：所有 active channel FIFO 有資料後才開始輸出
assign dac_valid    = force_silence ? 1'b1  : (all_ready & (mux_sel ? ~fifo_b_empty : ~fifo_a_empty));
assign fifo_a_rd_en = all_ready & ~mux_sel & ~fifo_a_empty;
assign fifo_b_rd_en = all_ready &  mux_sel & ~fifo_b_empty;

// ── State machine ─────────────────────────────────────────────────────────
localparam ST_IDLE           = 3'd0,
           ST_FLUSH_INIT     = 3'd1,  // flush both FIFOs at startup
           ST_CFG_INIT       = 3'd2,  // configure and start both readers
           ST_RUNNING        = 3'd3,
           ST_FLUSH_SWAP     = 3'd4,  // flush old-active FIFO after trigger
           ST_WAIT_READER_IDLE = 3'd5; // 2026-07-26 新增：等舊 active reader
                                       // 確認閒置（S_IDLE）才真正發 flush，
                                       // 見檔頭 2026-07-26 說明

// PROG_FULL=768 provides 7.68 µs buffer; at trigger the active FIFO has 768 samples
// and fill > drain (64-beat: +256 vs 224 drain per round with 8 readers), so
// ST_STABLE_WAIT is no longer needed.

reg [2:0]  state;
reg [3:0]  flush_cnt;
reg        flush_target; // which FIFO to flush in ST_FLUSH_SWAP (0=A, 1=B)

always @(posedge clk) begin
    if (rst) begin
        state         <= ST_IDLE;
        current_idx   <= 3'd0;
        next_idx      <= 3'd0;
        flush_cnt     <= 4'd0;
        flush_target  <= 1'b0;
        mux_sel       <= MUX_SEL_INIT;
        ra_start_addr <= 32'd0; ra_wave_len <= 32'd0; ra_play_en <= 1'b0;
        rb_start_addr <= 32'd0; rb_wave_len <= 32'd0; rb_play_en <= 1'b0;
        fifo_a_flush  <= 1'b0;
        fifo_b_flush  <= 1'b0;
        play_pos      <= -4'sd1;
    end else if (reinit_req) begin
        // 跟 rst 同等優先權，不管當下在哪個 state，立刻強制回到乾淨的
        // idle 狀態：0V（play_pos<=-1）、兩個 FIFO 都下 flush、停止兩個
        // reader。playlist 清空在上面「List storage」那個獨立 always
        // block 處理（同一個 reinit_req，不同 reg，不會互相衝突）。
        state         <= ST_IDLE;
        current_idx   <= 3'd0;
        next_idx      <= 3'd0;
        flush_cnt     <= 4'd0;
        flush_target  <= 1'b0;
        mux_sel       <= MUX_SEL_INIT;
        ra_play_en    <= 1'b0;
        rb_play_en    <= 1'b0;
        fifo_a_flush  <= 1'b1;
        fifo_b_flush  <= 1'b1;
        play_pos      <= -4'sd1;
    end else begin
        fifo_a_flush <= 1'b0;
        fifo_b_flush <= 1'b0;

        case (state)

            // ── Idle: wait for play_en ──────────────────────────────────
            ST_IDLE: begin
                ra_play_en <= 1'b0;
                rb_play_en <= 1'b0;
                mux_sel    <= MUX_SEL_INIT;
                play_pos   <= -4'sd1;
                if (play_en) begin
                    current_idx  <= 3'd0;
                    next_idx     <= inc_idx(3'd0, list_depth);
                    fifo_a_flush <= 1'b1;
                    fifo_b_flush <= 1'b1;
                    flush_cnt    <= FLUSH_CYCLES[3:0] - 4'd1;
                    state        <= ST_FLUSH_INIT;
                end
            end

            // ── Flush both FIFOs before starting ───────────────────────
            // 2026-07-04：修正版。flush 只在 flush_cnt 倒數期間拉高，倒數
            // 結束後放開 flush（讓 xpm_fifo_async 的 rst 真的降回 0），才去
            // 檢查 fifo_a_busy/fifo_b_busy 是否確認清除。
            // BUG（已修）：原本 fifo_a_flush/fifo_b_flush 在整個 ST_FLUSH_INIT
            // 狀態期間都無條件拉高（包含等待 busy 清除的階段），導致 rst
            // （由 flush 經 OR gate 產生）永遠不會放開，busy 永遠不會降回 0，
            // 形成死結：離開條件需要 busy=0，但只要沒離開 flush 就一直是 1，
            // 使 busy 永遠不是 0——4 個 channel 全部卡住，完全沒有輸出。
            ST_FLUSH_INIT: begin
                if (flush_cnt != 4'd0) begin
                    fifo_a_flush <= 1'b1;
                    fifo_b_flush <= 1'b1;
                    flush_cnt    <= flush_cnt - 4'd1;
                end else if (!fifo_a_busy && !fifo_b_busy) begin
                    state <= ST_CFG_INIT;
                end
            end

            // ── Configure readers (one cycle, then start) ───────────────
            ST_CFG_INIT: begin
                if (MUX_SEL_INIT == 1'b0) begin
                    // FIFO_A = active but silent (ra not started)
                    // FIFO_B = standby, pre-fills waveform for first trigger
                    rb_start_addr <= list_addr[current_idx];
                    rb_wave_len   <= list_len [current_idx];
                    ra_start_addr <= list_addr[next_idx];
                    ra_wave_len   <= list_len [next_idx];
                end else begin
                    // FIFO_B = active but silent (rb not started)
                    // FIFO_A = standby, pre-fills waveform for first trigger
                    ra_start_addr <= list_addr[current_idx];
                    ra_wave_len   <= list_len [current_idx];
                    rb_start_addr <= list_addr[next_idx];
                    rb_wave_len   <= list_len [next_idx];
                end
                // Only start the standby reader; active FIFO stays empty (silence)
                ra_play_en <= MUX_SEL_INIT;   // 0 when MUX_SEL=0: FIFO_A silent
                rb_play_en <= ~MUX_SEL_INIT;  // 1 when MUX_SEL=0: FIFO_B fills waveform
                mux_sel    <= MUX_SEL_INIT;
                state      <= ST_RUNNING;
            end

            // ── Running: active playing, standby pre-loading ────────────
            ST_RUNNING: begin
                if (!play_en) begin
                    ra_play_en <= 1'b0;
                    rb_play_en <= 1'b0;
                    state      <= ST_IDLE;
                end else if (trigger) begin
                    // Instant MUX swap (mux_sel uses old value in this cycle's logic)
                    flush_target <= mux_sel;   // old active FIFO = old mux_sel
                    mux_sel      <= ~mux_sel;  // new active FIFO

                    // Advance list index
                    current_idx <= next_idx;
                    next_idx    <= inc_idx(next_idx, list_depth);

                    // play_pos：這一拍是 mux_sel 真正翻轉、trigger 真正生效
                    // 的那一拍，跟 current_idx/next_idx 的「控制用提前更新」
                    // 不同，這裡才是 DAC 實際換內容的時間點。
                    if (play_pos + 4'sd1 >= $signed({1'b0, list_depth}))
                        play_pos <= 4'sd0;
                    else
                        play_pos <= play_pos + 4'sd1;

                    // Stop old-active reader（2026-07-26：不在這裡立刻
                    // flush FIFO——要先等這個 reader 真的閒置下來，見
                    // ST_WAIT_READER_IDLE 跟檔頭 2026-07-26 root-cause
                    // 說明。在這裡就發 flush 曾經讓一個還在跑 in-flight
                    // burst 的 reader，把跑完剩下的資料寫進剛清空的
                    // FIFO，samp_cnt 沒歸零，造成下次這顆 FIFO 變 active
                    // 時第一筆樣本不是 0）
                    if (mux_sel == 1'b0) begin
                        // FIFO_A was active → stop Reader_A, wait for it to idle
                        ra_play_en <= 1'b0;
                    end else begin
                        // FIFO_B was active → stop Reader_B, wait for it to idle
                        rb_play_en <= 1'b0;
                    end

                    state <= ST_WAIT_READER_IDLE;
                end
            end

            // ── Wait for the stopped reader to confirm it's really idle
            // (S_IDLE，rd_ptr/samp_cnt 已歸零) 才真正發 flush 脈衝。
            // 2026-07-26 新增，見檔頭 root-cause 說明。 ──────────────────
            ST_WAIT_READER_IDLE: begin
                if (!play_en) begin
                    ra_play_en <= 1'b0;
                    rb_play_en <= 1'b0;
                    state      <= ST_IDLE;
                end else if ((flush_target == 1'b0) ? ra_idle_s2 : rb_idle_s2) begin
                    if (flush_target == 1'b0)
                        fifo_a_flush <= 1'b1;
                    else
                        fifo_b_flush <= 1'b1;
                    flush_cnt <= FLUSH_CYCLES[3:0] - 4'd1;
                    state     <= ST_FLUSH_SWAP;
                end
            end

            // ── Flush old-active FIFO, then reconfigure as new standby ──
            // 2026-07-04：修正版。flush 只在 flush_cnt 倒數期間拉高（同
            // ST_FLUSH_INIT 的修正理由——flush 不放開，rst 就不會放開，busy
            // 就永遠不會降回 0，形成死結）。倒數結束、flush 放開後才檢查
            // flush_target 那顆 FIFO 的 busy 訊號是否確認清除。
            ST_FLUSH_SWAP: begin
                if (!play_en) begin
                    ra_play_en <= 1'b0;
                    rb_play_en <= 1'b0;
                    state      <= ST_IDLE;
                end else if (flush_cnt != 4'd0) begin
                    if (flush_target == 1'b0)
                        fifo_a_flush <= 1'b1;
                    else
                        fifo_b_flush <= 1'b1;
                    flush_cnt <= flush_cnt - 4'd1;
                end else if ((flush_target == 1'b0) ? !fifo_a_busy : !fifo_b_busy) begin
                    // Reconfigure flushed FIFO as new standby reader and start immediately.
                    // Use current_idx (already advanced at trigger): fills the slot
                    // that plays after the NEXT trigger, keeping sequential order.
                    // PROG_FULL=768 (7.68 µs buffer) + 64-beat fill>drain makes this safe.
                    if (flush_target == 1'b0) begin
                        ra_start_addr <= list_addr[current_idx];
                        ra_wave_len   <= list_len [current_idx];
                        ra_play_en    <= 1'b1;
                    end else begin
                        rb_start_addr <= list_addr[current_idx];
                        rb_wave_len   <= list_len [current_idx];
                        rb_play_en    <= 1'b1;
                    end
                    state <= ST_RUNNING;
                end
            end

            default: state <= ST_IDLE;

        endcase
    end
end

// ── dac_samp_idx（2026-08-18 新增，2026-08-18 補上 trigger 歸零修正）─────
// 獨立於上面主狀態機的 always block，純粹「這個 slot 目前播到第幾個
// sample」的計數器：force_silence（含 rst/reinit_req 的效果，見上面
// force_silence 的定義本身已經涵蓋 rst）或還沒真正被 trigger 過時歸零；
// 真正送出樣本（dac_valid）時 +1，數到這個 slot 的長度
// （list_len[current_idx]）就繞回 0，跟 ddr4_stream_reader.v 的
// samp_cnt 在各自 loop 一輪後歸零是同一種「這個 slot 的 buffer 有多長
// 就繞多快」邏輯，只是這裡算的是「已經送到 DAC」的那一側，不是「已經
// 寫進 FIFO」的那一側。
//
// 2026-08-18（Opus 覆核抓到的真實 bug，已修正）：trigger 生效那一拍，
// dac_data/current_idx 已經切到新 slot 的 sample 0（見上面 ST_RUNNING
// 的 trigger 分支，current_idx<=next_idx 跟 mux_sel<=~mux_sel 同一拍
// 生效），但原本的 dac_samp_idx 邏輯只有「繞到 list_len 邊界」跟
// 「force_silence」兩種歸零條件，沒有涵蓋「換了一個全新的 slot」這種
// 情況，會沿用舊 slot 的計數值繼續往下加，從第二次 trigger 開始就永久
// 對不上。新增最高優先權的 trigger 歸零分支：只在 ST_RUNNING 狀態才
// 生效（跟主狀態機 trigger 判斷同一個限定條件，ST_WAIT_READER_IDLE/
// ST_FLUSH_SWAP 期間的 trigger 本來就會被主狀態機忽略，這裡也不能算）。
always @(posedge clk) begin
    dac_samp_idx_wrap <= 1'b0;  // default每拍清成 0，跟 fifo_a_flush/fifo_b_flush 上面主狀態機同一種寫法
    if (force_silence) begin
        dac_samp_idx      <= 32'd0;
        dac_samp_idx_wrap <= 1'b1;
    end else if (play_en && (state == ST_RUNNING) && trigger) begin
        dac_samp_idx      <= 32'd0;
        dac_samp_idx_wrap <= 1'b1;
    end else if (dac_valid) begin
        if (dac_samp_idx + 32'd1 >= list_len[current_idx]) begin
            dac_samp_idx      <= 32'd0;
            dac_samp_idx_wrap <= 1'b1;
        end else begin
            dac_samp_idx <= dac_samp_idx + 32'd1;
        end
    end
end

endmodule
`default_nettype wire
