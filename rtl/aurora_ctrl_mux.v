`timescale 1ns/1ps
`default_nettype none

// aurora_ctrl_mux.v -- local copy (not the shared awg-test-step-14/rtl/
// aurora_ctrl_mux.v), forked 2026-07-27 for the「統一讀取/寫入架構」
// collapse (see PROJECT.md「統一讀取/寫入架構 — 完整規格」小節). The
// original file is referenced by several older projects (step-8 through
// step-14) via add_files pointing at $src_dir14 -- editing it in place
// would have changed those projects' source too, so step-16 gets its own
// copy instead (same convention already used for ddr4_stream_reader.v /
// awg_calib_regs.v / calib_mux.v).
//
// Two changes from the shared original, both scoped to this round's
// write-path collapse -- everything else (DDR4/list/play_ctrl/scale_cfg/
// trig_slot/timer/trig_mask/flash_save/erase) is untouched, including the
// already-dead scale_cfg_hold/scale_au_active/out_scale_cfg path (dead
// since 2026-07-15, see that section's original comment -- not this
// round's concern):
//
// 1. amp_ctrl: removed fp_amp_ctrl_wr/fp_amp_ctrl_sel/fp_amp_ctrl_data
//    (FrontPanel local-write) entirely. amp_ctrl is now only ever written
//    via the Aurora T_AMP_CTRL packet path (host loops back to itself via
//    dest_id=own board_id).
// 2. board_cfg: removed au_board_cfg_wr/au_board_cfg_id/
//    au_board_cfg_is_master inputs and out_board_cfg_wr/out_board_cfg_id/
//    out_board_cfg_is_master outputs (plus the bcfg_*_hold pass-through
//    logic) entirely. This whole path fed T_BOARD_CFG(0x10) into
//    board_cfg_reg.v -- T_BOARD_CFG is deleted this round (confirmed dead
//    packet, never actually used to set is_master by any script, fully
//    superseded by the extended T_BOARD_ID_ASSIGN), so both the producer
//    (local_reg_handler.v) and the consumer (board_cfg_reg.v) sides of
//    this signal are gone; keeping it here as a dangling pass-through
//    would leave an unconnected input in create_bd.tcl for no reason.
//
// Step 9：8 個 amp_ctrl hold 暫存器（Q1.16 18-bit）
// Step 10 新增：flash_load_* 輸入，startup loader 提供 flash 初始值
// Step 12 新增：7 個新控制路徑（flush_standby / trig_port / timer_ctrl /
//               trig_mask / flash_save / flash_erase / board_cfg
//               -- board_cfg 已於 2026-07-27 移除，見上方說明）
//
// 優先順序（amp ctrl，2026-07-27 起只剩 2 層）：flash > Aurora
// 其餘：各路徑 OR 或 hold register，詳見各欄位說明

module aurora_ctrl_mux (
    input  wire        clk,

    // ── FP TI bus
    input  wire [31:0] fp_ti_cmd,

    // ── FP WI 資料（準靜態電位）
    input  wire [31:0] fp_ddr4_addr,
    input  wire [31:0] fp_ddr4_w0, fp_ddr4_w1, fp_ddr4_w2, fp_ddr4_w3,
    input  wire [31:0] fp_list_sel,
    input  wire [31:0] fp_list_addr,
    input  wire [31:0] fp_list_len,
    input  wire [31:0] fp_depth,
    input  wire [31:0] fp_play_en,
    input  wire [31:0] fp_scale_cfg,

    // ── FP timer interval（ports 1-3，Step 12）
    input  wire [31:0] fp_timer_p1_intv,
    input  wire [31:0] fp_timer_p2_intv,
    input  wire [31:0] fp_timer_p3_intv,

    // ── Flash startup loader 初始值（Step 10）
    input  wire        flash_load_valid,
    input  wire [17:0] flash_amp_ctrl_0,
    input  wire [17:0] flash_amp_ctrl_1,
    input  wire [17:0] flash_amp_ctrl_2,
    input  wire [17:0] flash_amp_ctrl_3,
    input  wire [17:0] flash_amp_ctrl_4,
    input  wire [17:0] flash_amp_ctrl_5,
    input  wire [17:0] flash_amp_ctrl_6,
    input  wire [17:0] flash_amp_ctrl_7,

    // ── Aurora RX 輸出（sys_clk 單週期 pulse + 穩定資料）─ 既有
    input  wire        au_ddr4_wr,
    input  wire [31:0] au_ddr4_addr, au_ddr4_w0, au_ddr4_w1, au_ddr4_w2, au_ddr4_w3,
    input  wire        au_list_wr,
    input  wire [31:0] au_list_sel, au_list_addr, au_list_len,
    input  wire        au_play_ctrl_wr,
    input  wire [31:0] au_play_ctrl,
    input  wire        au_scale_cfg_wr,
    input  wire [31:0] au_scale_cfg,
    input  wire        au_amp_ctrl_wr,
    input  wire [3:0]  au_amp_ch_sel,
    input  wire [17:0] au_amp_val,

    // ── Aurora RX 輸出（Step 12 新增）
    input  wire        au_flush_standby,
    input  wire        au_trig_port_wr,
    input  wire [3:0]  au_trig_port_mask,
    input  wire        au_timer_ctrl_wr,
    input  wire [1:0]  au_timer_ctrl_port,
    input  wire        au_timer_ctrl_run,
    input  wire        au_timer_ctrl_loop,
    input  wire [4:0]  au_timer_ctrl_depth,
    input  wire [3:0]  au_timer_ctrl_slot,
    input  wire [31:0] au_timer_ctrl_intv,
    input  wire        au_trig_mask_wr,
    input  wire [3:0]  au_trig_mask_val,
    input  wire        au_flash_save,
    input  wire        au_flash_erase,

    // ── 輸出 → 硬體（既有）
    output wire [31:0] out_ti_cmd,
    output wire [31:0] out_ddr4_addr,
    output wire [31:0] out_ddr4_w0, out_ddr4_w1, out_ddr4_w2, out_ddr4_w3,
    output wire [31:0] out_list_sel,
    output wire [31:0] out_list_addr,
    output wire [31:0] out_list_len,
    // out_list_wr：list write enable，取代舊的 out_ti_cmd[2]/ti_list_wr
    // 路徑（2026-08-10，見下方 list_au_wr_dly 宣告處說明）
    output wire         out_list_wr,
    output wire [31:0] out_depth,
    output wire [31:0] out_play_en,
    output wire [31:0] out_scale_cfg,

    // amp_ctrl 輸出 → awg_calib_regs
    output wire [17:0] out_amp_ctrl_0,
    output wire [17:0] out_amp_ctrl_1,
    output wire [17:0] out_amp_ctrl_2,
    output wire [17:0] out_amp_ctrl_3,
    output wire [17:0] out_amp_ctrl_4,
    output wire [17:0] out_amp_ctrl_5,
    output wire [17:0] out_amp_ctrl_6,
    output wire [17:0] out_amp_ctrl_7,

    // ── Step 12 新增輸出 ─────────────────────────────────────────────────────

    // flush_standby：pass-through，在 BD 中與 FP ti_flush_standby OR 後送 flush_standby_cdc
    output wire        out_flush_standby,

    // per-port trigger pulses（Aurora T_TRIG_PORT，sys_clk domain）
    // 在 BD 中各自接 trigger_cdc → dac_clk → OR 到 wctrl sw_trigger
    output wire        out_trig_port_0,
    output wire        out_trig_port_1,
    output wire        out_trig_port_2,
    output wire        out_trig_port_3,

    // timer ports 0-3：interval hold（last-write-wins Aurora vs FP，
    // port 0 除外——2026-08-04 z0 合併進 T_TIMER_CTRL 之後，port 0
    // 沒有 FP 路徑，純 Aurora，見下方 always block 說明）
    output wire [31:0] out_timer_p0_intv,
    output wire [31:0] out_timer_p1_intv,
    output wire [31:0] out_timer_p2_intv,
    output wire [31:0] out_timer_p3_intv,
    // timer write pulse（1-cycle delayed after Aurora write，讓 intv 穩定）
    output wire        out_timer_p0_wr,
    output wire        out_timer_p1_wr,
    output wire        out_timer_p2_wr,
    output wire        out_timer_p3_wr,
    // timer run/loop/depth（Aurora hold，在 BD 中與 FP slice OR）
    // port 0（module z0）於 2026-08-04 補上，修復先前完全沒有 Aurora
    // 遠端路徑的缺口（見下方 always block 內的查證/修復記錄）
    output wire        out_timer_run_0,
    output wire        out_timer_loop_0,
    output wire [4:0]  out_timer_depth_0,
    output wire        out_timer_run_1,
    output wire        out_timer_run_2,
    output wire        out_timer_run_3,
    output wire        out_timer_loop_1,
    output wire        out_timer_loop_2,
    output wire        out_timer_loop_3,
    // depth 2026-08-04 3-bit→5-bit（16 slot 排程功能，見 PROJECT.md
    // trigger group 排程功能三部曲「B」條目；4-bit 仍不足以表示到 16，
    // 會重演 mem[] 最後一格死格的同一種 bug，見 rtl/trig_timer.v 同日
    // header comment 的完整推導）
    output wire [4:0]  out_timer_depth_1,
    output wire [4:0]  out_timer_depth_2,
    output wire [4:0]  out_timer_depth_3,
    // timer slot（Aurora slot when wr_dly, else 4'd0——2026-08-04 起不
    // 再 fall back 到已退役的 out_trig_slot，這個 fallback 值其實從
    // 不影響正確性：mem[] 只有 list_wr_en 那一拍才會真的用到 slot 值，
    // 而 list_wr_en 恰好只在 wr_dly=1 那一拍才會脈衝，所以 else 分支
    // 的值理論上永遠不會被實際消費，選 4'd0 純粹是給個乾淨的常數）
    output wire [3:0]  out_timer_p0_slot,
    output wire [3:0]  out_timer_p1_slot,
    output wire [3:0]  out_timer_p2_slot,
    output wire [3:0]  out_timer_p3_slot,

    // trigger mask（hold register，在 BD 中與 FP WI slice OR）
    output wire        out_trig_mask_0,
    output wire        out_trig_mask_1,
    output wire        out_trig_mask_2,
    output wire        out_trig_mask_3,

    // flash save/erase：pass-through，在 BD 中與 FP TI OR
    output wire        out_flash_save,
    output wire        out_flash_erase
);

    // ── 脈衝類：OR 使能，Aurora 發生時用 Aurora 資料 ─────────────────────────

    assign out_ti_cmd = fp_ti_cmd
                      | (au_ddr4_wr     ? 32'h00000001 : 32'd0)
                      | (au_list_wr     ? 32'h00000004 : 32'd0);

    assign out_ddr4_addr = au_ddr4_wr ? au_ddr4_addr : fp_ddr4_addr;
    assign out_ddr4_w0   = au_ddr4_wr ? au_ddr4_w0   : fp_ddr4_w0;
    assign out_ddr4_w1   = au_ddr4_wr ? au_ddr4_w1   : fp_ddr4_w1;
    assign out_ddr4_w2   = au_ddr4_wr ? au_ddr4_w2   : fp_ddr4_w2;
    assign out_ddr4_w3   = au_ddr4_wr ? au_ddr4_w3   : fp_ddr4_w3;

    // out_list_sel/addr/len 2026-08-10 搬到下面「準靜態類：hold 暫存器」
    // 區塊（改成 hold 暫存器，不再是這裡的 1-cycle 組合邏輯 mux）。

    // ── 準靜態類：hold 暫存器 ─────────────────────────────────────────────────
    // scale_cfg 這組（scale_cfg_hold/scale_au_active/out_scale_cfg）自
    // 2026-07-15 起已經是死路（scale_cfg 管理搬到 board_cfg_reg.v，這裡
    // 沒有消費者），維持不動，不在這次改動範圍內。
    reg [31:0] play_ctrl_hold  = 32'd0;
    reg        play_au_active  = 1'b0;

    reg [7:0]  scale_cfg_hold  = 8'd0;
    reg        scale_au_active = 1'b0;

    // list_sel/addr/len 2026-08-10 從「au_list_wr 那 1 個 sys_clk cycle
    // 才正確、其餘時間掉回 fp_list_*」的組合邏輯 mux 改成 hold 暫存器
    // （比照上面 play_ctrl_hold/play_au_active 的既有寫法）。原因：
    // out_list_sel/addr/len → wctrl_$ch（dac_clk domain）這段跨時脈域
    // 路徑本來完全沒有 CDC，這次要在 create_bd.tcl 補上 level_cdc/
    // trigger_cdc（比照 trig_timer_$port 既有模式），但 CDC 同步器需要
    // 來源值連續穩定好幾個 dst_clk 週期才抓得到，原本的 1-cycle mux
    // 撐不了這麼久，必須先在這裡把值 hold 住。詳見 NOTES.md 2026-08-10
    // 「Bulk DDR 階梯測試圖案...根因鎖定 T_LIST_WRITE 缺 CDC」章節。
    reg [31:0] list_sel_hold  = 32'd0;
    reg [31:0] list_addr_hold = 32'd0;
    reg [31:0] list_len_hold  = 32'd0;
    reg        list_au_active = 1'b0;

    // list_au_wr_dly：out_list_wr（餵給 create_bd.tcl 新增的 list_wr_cdc_0/
    // src_pulse）的 Aurora 那一路必須比 au_list_wr 本身晚 1 個 sys_clk
    // cycle 送出，理由：list_sel/addr/len_hold 是上面那個 always block
    // 用 non-blocking assignment 寫的暫存器，au_list_wr 那一拍當下讀到的
    // 還是「上一次」的舊值，新值要到下一拍才看得到。如果直接把 au_list_wr
    // 原始 pulse 送去驅動 CDC 的 src_pulse（這是這次修法第一版犯的
    // 錯，被新增的 sim/tb_list_write_cdc.v 抓到——level_cdc 抓到的是還
    // 沒更新的舊值），trigger_cdc 送出的 pulse 抵達 dac_clk 時，
    // level_cdc 那三條路徑很可能還在同步「舊值」，導致寫入用錯資料甚至
    // 完全漏寫。修法比照這個檔案裡 timer_p0_wr_dly（Step 12 新增的既有
    // 模式，同樣是「hold 暫存器 + 延後 1 拍的 wr pulse」）：out_list_wr
    // 用延後 1 拍的版本，讓它送達時 hold 暫存器保證已經是新值。FP 直寫
    // 路徑（fp_ti_cmd[2]）不需要延遲——WI 暫存器在 TI pulse 送出之前
    // 早就穩定好了，沒有這個 race，所以只延遲 Aurora 這一路，兩路在
    // out_list_wr 這裡 OR 在一起。
    reg        list_au_wr_dly = 1'b0;

    always @(posedge clk) begin
        if (au_play_ctrl_wr) begin
            play_ctrl_hold <= au_play_ctrl;
            play_au_active <= 1'b1;
        end
        if (au_scale_cfg_wr) begin
            scale_cfg_hold <= au_scale_cfg[7:0];
            scale_au_active<= 1'b1;
        end
        if (au_list_wr) begin
            list_sel_hold  <= au_list_sel;
            list_addr_hold <= au_list_addr;
            list_len_hold  <= au_list_len;
            list_au_active <= 1'b1;
        end
        list_au_wr_dly <= au_list_wr;
    end

    assign out_list_wr = fp_ti_cmd[2] | list_au_wr_dly;

    wire [31:0] depth_from_au   = {20'd0, play_ctrl_hold[15:4]};
    wire [31:0] play_en_from_au = {28'd0, play_ctrl_hold[3:0]};

    assign out_depth     = play_au_active  ? depth_from_au           : fp_depth;
    assign out_play_en   = play_au_active  ? play_en_from_au         : fp_play_en;
    assign out_scale_cfg = scale_au_active ? {24'd0, scale_cfg_hold} : {24'd0, fp_scale_cfg[7:0]};

    assign out_list_sel  = list_au_active ? list_sel_hold  : fp_list_sel;
    assign out_list_addr = list_au_active ? list_addr_hold : fp_list_addr;
    assign out_list_len  = list_au_active ? list_len_hold  : fp_list_len;

    // ── amp_ctrl hold 暫存器（8 × 18-bit，init 0x10000 = Q1.16 1.0）──────────
    // 2026-07-27 起只剩 flash > Aurora 兩層，本機 fp_amp_ctrl_wr 已拔除
    reg [17:0] amp_reg_0 = 18'h10000;
    reg [17:0] amp_reg_1 = 18'h10000;
    reg [17:0] amp_reg_2 = 18'h10000;
    reg [17:0] amp_reg_3 = 18'h10000;
    reg [17:0] amp_reg_4 = 18'h10000;
    reg [17:0] amp_reg_5 = 18'h10000;
    reg [17:0] amp_reg_6 = 18'h10000;
    reg [17:0] amp_reg_7 = 18'h10000;

    always @(posedge clk) begin
        if (flash_load_valid) begin
            amp_reg_0 <= flash_amp_ctrl_0;
            amp_reg_1 <= flash_amp_ctrl_1;
            amp_reg_2 <= flash_amp_ctrl_2;
            amp_reg_3 <= flash_amp_ctrl_3;
            amp_reg_4 <= flash_amp_ctrl_4;
            amp_reg_5 <= flash_amp_ctrl_5;
            amp_reg_6 <= flash_amp_ctrl_6;
            amp_reg_7 <= flash_amp_ctrl_7;
        end else if (au_amp_ctrl_wr) begin
            if (au_amp_ch_sel[3]) begin
                amp_reg_0 <= au_amp_val;
                amp_reg_1 <= au_amp_val;
                amp_reg_2 <= au_amp_val;
                amp_reg_3 <= au_amp_val;
                amp_reg_4 <= au_amp_val;
                amp_reg_5 <= au_amp_val;
                amp_reg_6 <= au_amp_val;
                amp_reg_7 <= au_amp_val;
            end else begin
                case (au_amp_ch_sel[2:0])
                    3'd0: amp_reg_0 <= au_amp_val;
                    3'd1: amp_reg_1 <= au_amp_val;
                    3'd2: amp_reg_2 <= au_amp_val;
                    3'd3: amp_reg_3 <= au_amp_val;
                    3'd4: amp_reg_4 <= au_amp_val;
                    3'd5: amp_reg_5 <= au_amp_val;
                    3'd6: amp_reg_6 <= au_amp_val;
                    3'd7: amp_reg_7 <= au_amp_val;
                endcase
            end
        end
    end

    assign out_amp_ctrl_0 = amp_reg_0;
    assign out_amp_ctrl_1 = amp_reg_1;
    assign out_amp_ctrl_2 = amp_reg_2;
    assign out_amp_ctrl_3 = amp_reg_3;
    assign out_amp_ctrl_4 = amp_reg_4;
    assign out_amp_ctrl_5 = amp_reg_5;
    assign out_amp_ctrl_6 = amp_reg_6;
    assign out_amp_ctrl_7 = amp_reg_7;

    // ────────────────────────────────────────────────────────────────────────
    // Step 12 新增邏輯
    // ────────────────────────────────────────────────────────────────────────

    // flush_standby / flash：直接 pass-through（BD 中再與 FP TI OR）
    assign out_flush_standby = au_flush_standby;
    assign out_flash_save    = au_flash_save;
    assign out_flash_erase   = au_flash_erase;

    // per-port trigger：au_trig_port_wr & mask bit
    assign out_trig_port_0 = au_trig_port_wr & au_trig_port_mask[0];
    assign out_trig_port_1 = au_trig_port_wr & au_trig_port_mask[1];
    assign out_trig_port_2 = au_trig_port_wr & au_trig_port_mask[2];
    assign out_trig_port_3 = au_trig_port_wr & au_trig_port_mask[3];

    // trigger mask hold register（slave 可透過 Aurora 設定，與 FP WI OR 於 BD）
    reg [3:0] trig_mask_hold = 4'd0;
    always @(posedge clk)
        if (au_trig_mask_wr)
            trig_mask_hold <= au_trig_mask_val;

    assign out_trig_mask_0 = trig_mask_hold[0];
    assign out_trig_mask_1 = trig_mask_hold[1];
    assign out_trig_mask_2 = trig_mask_hold[2];
    assign out_trig_mask_3 = trig_mask_hold[3];

    // ── Timer hold registers（ports 0-3，全部走同一套機制）───────────────────
    // intv：port 1-3 是 last-write-wins（Aurora 寫入後 au_valid=1，往後
    //        使用 Aurora hold；否則 pass-through FP WI，使 FP 寫入仍
    //        有效）；port 0 沒有 FP 路徑，直接用暫存器值，見下方說明
    // run/loop/depth：Aurora hold（在 BD 中與 FP WI slice OR；
    //   slave 板 FP=0，所以 OR = Aurora value；master 板由 FP 控制，不走此路徑）
    // slot：wr_dly 期間用 Aurora slot，其餘時間 fall-back 到 4'd0
    //
    // ⚠️ 2026-08-04 修復歷程（同一天分兩階段，完整推導見 PROJECT.md
    // trigger group 排程功能三部曲「A」「D」條目）：
    // **階段一（run/loop/depth）**：port 1-3 專用的機制原本完全沒有
    // port 0（module z0）的對應分支——au_timer_ctrl_port 原本只跟
    // 2'd1/2'd2/2'd3 比對，host 端送 T_TIMER_CTRL(0x0C) port=0 完全
    // 沒有效果（`host/test_trig_timer.py` 上機驗證確認：port=0
    // play_pos 不會自動變化，port=1 立刻正常）。新增
    // out_timer_run_0/loop_0/depth_0 三個 output + port==2'd0 分支 +
    // create_bd.tcl 的 or_timer_run_0/loop_0/depth_0 三個 OR gate。
    // **階段二（intv/slot，2026-08-04 同一天稍後）**：上機測試 16-slot
    // 加寬（B 步驟）時發現 z1 完全正常、z0 嚴重異常，往下查出 port 0
    // 的排程資料（intv/slot）當時還是走完全獨立、從無 host wrapper的
    // 舊封包 T_TRIG_SLOT(0x08)（`out_trig_slot`/`out_trig_intv`，已
    // 移除），跟 T_TIMER_CTRL 完全無關——這是階段一沒處理到的既有
    // 架構缺口，不是階段一的 bug。使用者確認「z0 應該跟 z1-z3 行為
    // 一致」後，把 intv/slot 也併進 T_TIMER_CTRL（新增
    // out_timer_p0_intv/out_timer_p0_wr/out_timer_p0_slot + port==2'd0
    // 分支補上 intv/slot latch），T_TRIG_SLOT(0x08) 整條路徑（含
    // local_reg_handler.v 的解碼、WI 0x0E/0x0F FP 直寫）已完整退役。
    // 至此 z0-z3 四個 port 用完全同一套機制，唯一差異是 port 0 沒有
    // FP 直寫路徑（run/loop/depth 有 FP OR，intv/slot 純 Aurora）。

    reg        timer_p0_run   = 1'b0;  reg timer_p0_loop     = 1'b0;
    reg [4:0]  timer_p0_depth = 5'd0;
    reg [31:0] timer_p0_intv  = 32'd0;
    reg [3:0]  timer_p0_slot  = 4'd0;
    reg        timer_p0_wr_dly= 1'b0;

    reg [31:0] timer_p1_intv  = 32'd0; reg timer_p1_au_valid = 1'b0;
    reg        timer_p1_run   = 1'b0;  reg timer_p1_loop     = 1'b0;
    reg [4:0]  timer_p1_depth = 5'd0;  reg [3:0] timer_p1_slot = 4'd0;
    reg        timer_p1_wr_dly= 1'b0;

    reg [31:0] timer_p2_intv  = 32'd0; reg timer_p2_au_valid = 1'b0;
    reg        timer_p2_run   = 1'b0;  reg timer_p2_loop     = 1'b0;
    reg [4:0]  timer_p2_depth = 5'd0;  reg [3:0] timer_p2_slot = 4'd0;
    reg        timer_p2_wr_dly= 1'b0;

    reg [31:0] timer_p3_intv  = 32'd0; reg timer_p3_au_valid = 1'b0;
    reg        timer_p3_run   = 1'b0;  reg timer_p3_loop     = 1'b0;
    reg [4:0]  timer_p3_depth = 5'd0;  reg [3:0] timer_p3_slot = 4'd0;
    reg        timer_p3_wr_dly= 1'b0;

    always @(posedge clk) begin
        timer_p0_wr_dly <= (au_timer_ctrl_wr && au_timer_ctrl_port == 2'd0);
        timer_p1_wr_dly <= (au_timer_ctrl_wr && au_timer_ctrl_port == 2'd1);
        timer_p2_wr_dly <= (au_timer_ctrl_wr && au_timer_ctrl_port == 2'd2);
        timer_p3_wr_dly <= (au_timer_ctrl_wr && au_timer_ctrl_port == 2'd3);

        if (au_timer_ctrl_wr) begin
            if (au_timer_ctrl_port == 2'd0) begin
                timer_p0_run      <= au_timer_ctrl_run;
                timer_p0_loop     <= au_timer_ctrl_loop;
                timer_p0_depth    <= au_timer_ctrl_depth;
                timer_p0_intv     <= au_timer_ctrl_intv;
                timer_p0_slot     <= au_timer_ctrl_slot;
            end
            if (au_timer_ctrl_port == 2'd1) begin
                timer_p1_intv     <= au_timer_ctrl_intv;
                timer_p1_run      <= au_timer_ctrl_run;
                timer_p1_loop     <= au_timer_ctrl_loop;
                timer_p1_depth    <= au_timer_ctrl_depth;
                timer_p1_slot     <= au_timer_ctrl_slot;
                timer_p1_au_valid <= 1'b1;
            end
            if (au_timer_ctrl_port == 2'd2) begin
                timer_p2_intv     <= au_timer_ctrl_intv;
                timer_p2_run      <= au_timer_ctrl_run;
                timer_p2_loop     <= au_timer_ctrl_loop;
                timer_p2_depth    <= au_timer_ctrl_depth;
                timer_p2_slot     <= au_timer_ctrl_slot;
                timer_p2_au_valid <= 1'b1;
            end
            if (au_timer_ctrl_port == 2'd3) begin
                timer_p3_intv     <= au_timer_ctrl_intv;
                timer_p3_run      <= au_timer_ctrl_run;
                timer_p3_loop     <= au_timer_ctrl_loop;
                timer_p3_depth    <= au_timer_ctrl_depth;
                timer_p3_slot     <= au_timer_ctrl_slot;
                timer_p3_au_valid <= 1'b1;
            end
        end
    end

    assign out_timer_p0_intv  = timer_p0_intv;
    assign out_timer_p1_intv  = timer_p1_au_valid ? timer_p1_intv  : fp_timer_p1_intv;
    assign out_timer_p2_intv  = timer_p2_au_valid ? timer_p2_intv  : fp_timer_p2_intv;
    assign out_timer_p3_intv  = timer_p3_au_valid ? timer_p3_intv  : fp_timer_p3_intv;

    assign out_timer_p0_wr    = timer_p0_wr_dly;
    assign out_timer_p1_wr    = timer_p1_wr_dly;
    assign out_timer_p2_wr    = timer_p2_wr_dly;
    assign out_timer_p3_wr    = timer_p3_wr_dly;

    assign out_timer_run_0    = timer_p0_run;
    assign out_timer_loop_0   = timer_p0_loop;
    assign out_timer_depth_0  = timer_p0_depth;

    assign out_timer_run_1    = timer_p1_run;
    assign out_timer_run_2    = timer_p2_run;
    assign out_timer_run_3    = timer_p3_run;
    assign out_timer_loop_1   = timer_p1_loop;
    assign out_timer_loop_2   = timer_p2_loop;
    assign out_timer_loop_3   = timer_p3_loop;
    assign out_timer_depth_1  = timer_p1_depth;
    assign out_timer_depth_2  = timer_p2_depth;
    assign out_timer_depth_3  = timer_p3_depth;

    // slot：wr_dly=1 時用 Aurora slot，否則 fall-back 到 4'd0（見上方
    // output port 宣告處的完整解釋——這個 fallback 值不影響正確性）
    assign out_timer_p0_slot  = timer_p0_wr_dly ? timer_p0_slot : 4'd0;
    assign out_timer_p1_slot  = timer_p1_wr_dly ? timer_p1_slot : 4'd0;
    assign out_timer_p2_slot  = timer_p2_wr_dly ? timer_p2_slot : 4'd0;
    assign out_timer_p3_slot  = timer_p3_wr_dly ? timer_p3_slot : 4'd0;

endmodule
`default_nettype wire
