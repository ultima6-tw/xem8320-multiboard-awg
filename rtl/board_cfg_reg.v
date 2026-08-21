`timescale 1ns/1ps
`default_nettype none

// board_cfg_reg — board_id / is_master / scale_cfg 暫存器
//
// 2026-07-14（step-16 fork，原檔案共用自 awg-test-step-14/rtl/）：新增
// au_board_id_assign_wr/au_board_id_assign_value，讓 board_id 可以直接從
// Aurora enum 算出的 board_index 產生，不用每片板子各自接 USB 手動下
// set_board_cfg_direct.py。動機：未來實際部署只會有一條 USB 接 master，
// slave 完全沒有 USB 可以手動設定，board_id 必須能透過 Aurora 廣播遠端
// 產生。不是 enum 完成就自動觸發——由 host 在 enum 完成後另外送一個獨立的
// T_BOARD_ID_ASSIGN 封包才觸發，讓使用者可以先確認 enum 結果（total_boards/
// 環路順序）正確再決定要不要套用。這個來源不寫入 flash。
//
// 2026-07-27（統一讀取/寫入架構收斂，見 PROJECT.md「統一讀取/寫入架構 —
// 完整規格」小節）：拔除本機 WI+TI 直寫路徑（原本的 fp_board_cfg_wr/
// fp_ext_clk_sel_wr/fp_dac_mode_ramp_wr/fp_scale_wr 這一整組），board_id/
// is_master/ext_clk_sel/dac_mode_ramp/scale_cfg 全部改成只認 Aurora 封包
// 一條路徑（本機也走同一條路徑的 loopback，dest_id=自己時由 dispatcher.v
// 判斷走 RT_LOCAL，不再需要另外的「本機直寫」暫存器分支）。同時刪除
// au_board_cfg_wr（Aurora T_BOARD_CFG=0x10）——確認從未被任何腳本用來
// 設定過 is_master，純粹死路，功能跟 au_board_id_assign_wr 重複。
// au_board_id_assign_wr（Aurora T_BOARD_ID_ASSIGN=0x1E）原本只寫
// board_id，這次擴充新增 au_board_id_assign_is_master input（對應
// local_reg_handler.v 新解碼的封包 beat1[37]），讓它同時寫 board_id
// （沿用 board_index）跟 is_master（host 明確指定），成為 board 身份
// 指定的唯一入口。
//
// 2026-08-05：dac_mode_ramp（連同 au_dac_mode_ramp_wr/au_dac_mode_ramp
// port）從本檔案整個移除，併入 rtl/sine_ctrl_regs.v（改成 idle/active
// 雙緩衝、per-module/per-channel 獨立定址，取代這裡原本的整包覆寫、
// 無緩衝設計）。T_DAC_MODE_RAMP(0x2A) 封包退役，改用既有 T_SINE_CTRL
// （param_sel=6/7）。見 sine_ctrl_regs.v 檔頭 + PROJECT.md/NOTES.md
// 2026-08-05 對應章節。
//
// board_id / is_master 優先順序（高到低，2026-07-27 起只剩 2 層）：
//   1. rst：硬體預設值（16'hFFFE / 0，保證跟 master 固定的 0x0000 不同；
//      2026-07-22 從 16'hFFFF 改成 16'hFFFE——0xFFFF 同時是 dispatcher.v
//      的 broadcast dest_id，board_id 停在 reset 值時兩者相等，會讓
//      dispatcher.v 的路由判斷式（`dest_id==board_id` 排在 `dest_id==
//      0xFFFF` 之前）誤判 broadcast 封包成「dest_id 剛好等於自己」，
//      只送本機不轉送下一站，導致 T_BOARD_ID_ASSIGN 等 broadcast 封包
//      在還沒被賦予 board_id 的板子上斷link、傳不到下一片板子。0xFFFE
//      保證不等於任何真正的 board_id（環路最多 31 片，5-bit board_index）
//      也不等於 broadcast 位址，從根本消除這個碰撞，不需要改
//      dispatcher.v 本身的判斷邏輯）
//   2. au_board_id_assign_wr（Aurora T_BOARD_ID_ASSIGN）：board_id <=
//      board_index、is_master <= 封包指定值
//
// scale_cfg 優先順序（高到低，2026-07-27 起 fp_scale_wr 已移除）：
//   1. rst：硬體預設值
//   2. flash_scale_load_valid：上電 one-shot，從 flash 載入
//   3. au_scale_wr（Aurora T_SCALE_CFG）：Aurora 覆寫 scale_cfg
//
// 2026-07-15：scale_cfg 從 aurora_ctrl_mux.v 的 scale_cfg_hold/scale_au_active
// 機制搬過來這裡管理，理由：aurora_ctrl_mux.v 原本的行為是「Aurora 寫過一次
// 就永久鎖定，host 之後再也蓋不掉」，跟 board_id 這裡「誰晚寫誰生效」的簡單
// 規則不一致；使用者要求統一成跟 board_id 一樣的規則，所以新增 au_scale_wr/
// au_scale_cfg 這兩個 port，優先序比照 fp_board_cfg_wr/au_board_cfg_wr 的
// pattern（fp 跟 au 同一拍撞在一起時 fp 贏，否則誰晚寫誰生效，沒有鎖定旗標）。
// aurora_ctrl_mux.v 本身不動（跟 step-14 共用），它的 out_scale_cfg 變成沒有
// 消費者的死路，不影響原本的 step-14。
//
// 2026-07-15（同一輪）：flash 分開儲存後，scale_cfg 存在獨立的 sector（跟
// board_id/is_master 所在的身份 sector 不同），開機時兩者的 flash 讀取
// 「完成時間點」不一樣（flash_startup_loader.v 依序讀 sector，scale_cfg
// 那個 sector 讀完的時間點在身份 sector 之後）。原本共用同一個
// flash_load_valid/flash_loaded 會導致 scale_cfg 在錯誤時機被鎖存（採樣到
// 身份 sector 讀完當下、scale_cfg 那個 sector 都還沒開始讀的舊值/預設值）。
// 拆成兩組獨立的 valid/loaded：flash_load_valid（身份，board_id/is_master）
// 跟 flash_scale_load_valid（scale_cfg，跟著各自 sector 的讀取完成時間點）。

module board_cfg_reg (
    input  wire        clk,
    input  wire        rst,

    // ── Flash 初始值（身份：board_id/is_master，sector 0）
    input  wire        flash_load_valid,
    input  wire [15:0] flash_board_id,
    input  wire        flash_is_master,

    // ── Flash 初始值（scale_cfg，獨立 sector，2026-07-15）
    input  wire        flash_scale_load_valid,
    input  wire [7:0]  flash_scale_cfg,

    // ── Aurora 覆寫 scale_cfg（Step 16 新增，2026-07-15）
    input  wire        au_scale_wr,
    input  wire [7:0]  au_scale_cfg,

    // ── Aurora board_index 自動產生（Step 16 新增，2026-07-14；2026-07-27
    // 擴充新增 au_board_id_assign_is_master，見上方檔頭說明）
    input  wire        au_board_id_assign_wr,
    input  wire [4:0]  au_board_id_assign_value,
    input  wire        au_board_id_assign_is_master,
    // 2026-07-30 新增：這次 T_BOARD_ID_ASSIGN 是不是 broadcast——
    // broadcast 只更新 board_id，不動 is_master（is_master 只由明確
    // 指定目標板子的 unicast 封包設定）。根因：enum 完成後套用
    // board_index 那次 broadcast 只能帶一個全部板子共用的 is_master
    // 值，會把剛設好的 master 也一起洗掉，見 rtl/local_reg_handler.v
    // bid_is_broadcast port 註解、NOTES.md 2026-07-30 對應章節。
    input  wire        au_board_id_assign_is_broadcast,

    // ── ext_clk_sel（2026-07-24 新增，USB-only 功能盤點第 1 項；2026-07-27
    // 拔除本機 fp_ext_clk_sel_wr 路徑，只留 Aurora T_EXT_CLK_SEL=0x28）
    input  wire        au_ext_clk_sel_wr,
    input  wire        au_ext_clk_sel,

    // ── per_hop_value 手動覆寫（2026-07-24 新增，au_trig_delay 自動校準
    // 機制除錯用）：跟 ext_clk_sel/dac_mode_ramp 不同，這裡**沒有** Aurora
    // 覆寫路徑（每片板子各自用自己的 USB 連線單獨設定，不經 Aurora
    // broadcast——per_hop_value 這次刻意選單板 USB 直連，不是「誰晚寫誰
    // 生效」，而是比照 local_reg_handler.v 的 manual_delay_override_active
    // sticky 慣例：設過就一直優先於 aurora_ctrl_channel_0 自動算出的
    // per_hop_value，直到 rst，沒有清除回自動模式的指令。詳見下方
    // manual_per_hop_active 獨立 always block 說明。
    input  wire        fp_per_hop_value_wr,
    input  wire [31:0] fp_per_hop_value,

    // ── Group-based Trigger 架構（2026-07-27 新增，取代 aurora_ctrl_mux.v
    // 的 au_trig_mask_wr/au_trig_mask_val 機制）：T_TRIG_MASK(0x0D) 新語意
    // 送來的是「這個 group 涵蓋哪些模組」（au_group_cfg_mask，[A,B,C,D]=
    // bit[3:0]），這裡要轉換成「每個模組屬於哪個 group」（group_id_a/b/
    // c/d[1:0]）。跟 ext_clk_sel/dac_mode_ramp 不同：**沒有** fp_*（本機
    // WI/TI）路徑——這個功能沒有 WI 位址可用（WI 0x00-0x1F 已配置完），
    // 且「本機」支援本來就靠 host 把封包 dest_id 指定成自己的 board_id
    // 達成（見 local_reg_handler.v 對應 port 註解），不需要另外的暫存器
    // 路徑。沒有 flash 層（跟 ext_clk_sel/dac_mode_ramp 一樣，每次開機
    // 由 host 明確設定）。
    input  wire        au_group_cfg_wr,
    input  wire [1:0]  au_group_cfg_group_id,
    input  wire [3:0]  au_group_cfg_mask,

    // ── 輸出
    output reg  [15:0] board_id,
    output reg         is_master,
    output reg  [7:0]  scale_cfg,
    output reg         ext_clk_sel,
    output reg  [31:0] per_hop_value_manual,
    output reg         manual_per_hop_active,
    output reg  [1:0]  group_id_a,
    output reg  [1:0]  group_id_b,
    output reg  [1:0]  group_id_c,
    output reg  [1:0]  group_id_d
);

    // flash_scale_load_valid 保持高電位，但只在第一次（!flash_scale_loaded）
    // 採樣（board_id/is_master 2026-07-15 起不再從 flash 載入，見上方說明，
    // 不需要對應的 flash_loaded 旗標）
    reg flash_scale_loaded = 1'b0;

always @(posedge clk) begin
    if (rst) begin
        board_id            <= 16'hFFFE;   // 2026-07-22：見上方「board_id / is_master 優先順序」章節說明
        is_master            <= 1'b0;
        scale_cfg            <= 8'hFF;
        ext_clk_sel          <= 1'b0;      // 2026-07-24：reset 預設內部時脈，跟改版前 wi_ext_clk_sel 的預設行為一致
        per_hop_value_manual <= 32'd0;     // 2026-07-24：手動覆寫值，reset 後清空（配合下方 manual_per_hop_active 一起回到自動模式）
        group_id_a           <= 2'd0;      // 2026-07-27：預設全部模組都在 group 0，配合 T_TRIG_START 預設 group_select=4'hF（全部 group 都 fire），沒設定過分組表時行為等同「全部一起觸發」（跟舊 trigger_mask 全 0 的預設行為一致）
        group_id_b           <= 2'd0;
        group_id_c           <= 2'd0;
        group_id_d           <= 2'd0;
        flash_scale_loaded   <= 1'b0;
    end else begin
        // ── board_id / is_master：2026-07-15 起不再從 flash 載入──────────
        // （見 PROJECT.md「board_id flash 載入路徑移除」章節：flash 殘留值
        // 可能是舊資料或損壞資料，一旦剛好等於 master 的固定 board_id
        // （0x0000）就會讓 enum 繞行封包在該站被誤判成「已經回到發起者」
        // 而不再往下一跳轉送，導致 total_boards 卡住——2026-07-15 board B
        // 身份 sector 損壞事件就是實際踩到這個問題。board_id/is_master
        // 永遠只信任 reset 預設值（16'hFFFE/0，保證跟 master 的 0x0000
        // 不同，2026-07-22 起也保證不等於 broadcast 位址 0xFFFF）+
        // host/Aurora 明確指令，不再信任 flash 內容。
        // flash_load_valid/flash_board_id/flash_is_master 這三個 port
        // 保留（BD 連線不用動），只是這裡不再使用它們。2026-07-27：
        // fp_board_cfg_wr/au_board_cfg_wr 已移除，board_id/is_master
        // 只剩 au_board_id_assign_wr 一條路徑。2026-07-30：is_master
        // 只在 unicast 時更新——broadcast（enum 後套用 board_index 那
        // 次）只會更新 board_id，不會把 is_master 一起洗掉，見上方
        // au_board_id_assign_is_broadcast port 註解。
        if (au_board_id_assign_wr) begin
            board_id <= {11'd0, au_board_id_assign_value};
            if (!au_board_id_assign_is_broadcast)
                is_master <= au_board_id_assign_is_master;
        end

        // ── ext_clk_sel：只認 Aurora T_EXT_CLK_SEL（2026-07-27 拔除
        // 本機 fp_ext_clk_sel_wr 路徑）──────────────────────────────
        if (au_ext_clk_sel_wr)
            ext_clk_sel <= au_ext_clk_sel;

        // ── per_hop_value_manual：TriggerIn 觸發式載入手動值本身，跟
        // ext_clk_sel/dac_mode_ramp 用同一套 wr pulse 慣例——但 sticky
        // flag（manual_per_hop_active）不在這裡，見下方獨立 always block
        // ──────────────────────────────────────────────────────────
        if (fp_per_hop_value_wr)
            per_hop_value_manual <= fp_per_hop_value;

        // ── group_id_a/b/c/d：Group-based Trigger 架構，「mask→group_id」
        // 轉換——au_group_cfg_mask 是「這個 group 涵蓋哪些模組」，這裡要
        // 拆成「每個模組屬於哪個 group」。衝突規則：後寫入的覆蓋前面的
        // （沒有額外的優先權暫存器，單純每次 wr pulse 只更新這次 mask
        // 有涵蓋到的模組，沒被這次 mask 涵蓋的模組維持原值不動）────────
        if (au_group_cfg_wr) begin
            if (au_group_cfg_mask[3]) group_id_a <= au_group_cfg_group_id;
            if (au_group_cfg_mask[2]) group_id_b <= au_group_cfg_group_id;
            if (au_group_cfg_mask[1]) group_id_c <= au_group_cfg_group_id;
            if (au_group_cfg_mask[0]) group_id_d <= au_group_cfg_group_id;
        end

        // ── scale_cfg：獨立 sector（獨立於 board_id/is_master）；2026-07-27
        // 拔除本機 fp_scale_wr 路徑，只留 Aurora T_SCALE_CFG ──────────
        if (flash_scale_load_valid && !flash_scale_loaded) begin
            scale_cfg          <= flash_scale_cfg;
            flash_scale_loaded <= 1'b1;
        end else if (au_scale_wr) begin
            scale_cfg <= au_scale_cfg;
        end
    end
end

    // 獨立 always block：latch manual_per_hop_active（2026-07-24 新增，
    // au_trig_delay 自動校準機制除錯用）。比照 local_reg_handler.v 的
    // manual_delay_override_active 手法，不跟上面主 always block 共用
    // 同一個 reg 的驅動來源。fp_per_hop_value_wr 讀到的是上一拍的
    // registered 值，1-cycle 延遲對 sticky flag 語意沒有影響。沒有
    // 「清除回自動模式」的指令，恢復自動模式必須整個 reset。
    always @(posedge clk) begin
        if (rst)
            manual_per_hop_active <= 1'b0;
        else if (fp_per_hop_value_wr)
            manual_per_hop_active <= 1'b1;
    end

endmodule
`default_nettype wire
