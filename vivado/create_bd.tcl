# create_bd.tcl -- AWG Step 15a (forked from awg-test-step-14.3b 2026-07-05;
# 14.3b finished the local single-board path -- CDC fix + DDR<->AWG playback --
# 15a starts the Aurora multi-board sync work on top of it)
# source {C:/path/to/awg-test-step-16/vivado/create_bd.tcl}
#
# 2026-07-05 (step 15a): Aurora 已復原並雙向化，接上新的三個模組
# （aurora_data_channel/aurora_ctrl_channel/aurora_tx1_arbiter，見 PORTS.md
# 「Aurora 多板協定模組」、PROJECT.md 設計文件 v2）。下面 Step 14.3b 那段是
# 移除當時的歷史記錄，保留供對照，不代表目前狀態。
#
# Step 14.3b (inherited history below): remove all Aurora-related cells/wiring from 14.3 v9 (commented out
# with #, not deleted, for easy diff/revert)
#   - dispatcher_0/rx_tdata <- const_64b0, rx_tvalid <- const_zero, tx_tready <- const_one
#   - WO 0x27 AURORA_STATUS: both channel_up inputs tied to const_zero
#   - LED: both aurora channel_up inputs tied to const_zero
#   - RTL: dispatcher.v/ddr_writer.v/fp_input.v are local copies (see rtl/), each with
#     debug-only counters/ports added for ILA visibility -- core packet-routing/DDR-write
#     logic itself is unchanged from the shared awg-test-step-14/rtl sources
#
# Step 14.3 v9: all-sys_clk architecture
#   - dispatcher / ddr_writer / aurora_tx_arbiter moved to sys_clk
#   - only two CDCs left: async_fifo_aurora_rx (aurora->sys) + async_fifo_aurora_tx (sys->aurora)
#   - removed axi_cc_aurora_sys; ddr_writer/m_axi connects directly to SmartConnect S00
#   - fp_input xpm_fifo_async -> xpm_fifo_sync (aurora_clk port removed)
#   - async_fifo_local -> Common_Clock_Block_RAM (both sides sys_clk)
# Step 14.3 v8: removed lrh_empty_sync + dispatcher lrh_fifo_empty guard (reverted to 14.2 behavior)
# Step 14.3 v7: fp_ddr4_rw_1 ti_cmd wired directly to full TI bus (same as step-12 architecture)
# removed xlslice(ti_ddr4_wr/rd) + xlconcat; fp0/ti40_ep_trigger connects directly to fp_ddr4_rw_1/ti_cmd
# ti_cmd[0]=start_wr (host: ActivateTriggerIn(0x40, 0))
# ti_cmd[1]=start_rd (host: ActivateTriggerIn(0x40, 1))
# Write: FP WI 0x12 addr + WI 0x14-0x17 data + TI bit0 -> fp_ddr4_rw_1 -> SmartConnect S01 -> DDR4
# Read: FP WI 0x12 addr + TI bit1 -> fp_ddr4_rw_1 -> DDR4 -> WO 0x21-0x24
# Dispatcher write: FP PipeIn(type=0x13) -> fp_input -> dispatcher -> ddr_writer_0 -> SmartConnect S00 -> DDR4

set proj_dir   "D:/Vivado/awg_step16"
set proj_name  "awg_step16"
# 2026-08-18：src_dir6/src_dir7/src_dir14（分別指向 awg-test-step-6/7/14
# 這三個舊專案的共用檔案）已整批移除——這三個資料夾都已在 2026-08-10
# 搬進 Projects/FPGA/_archive/，這幾個變數指向的路徑因此全部失效
# （dry-run 才抓到，之前搬移後沒人重新跑過完整 create_bd）。使用者確認
# awg-test-step-16 是目前唯一持續維護的最終版本，不應該再依賴外部/已
# 封存的資料夾，所以把最後還在用這三個變數的 5 個檔案（diag_cdc.v/
# spi_flash_ctrl.v/syzygy_ready.v/sync_start.v/dac_clk_mux.v，加上原本
# 就在用 src_dir6 的 fp_ddr4_rw.v/aurora_refclk_ibuf.v）全部複製進本地
# rtl/，比照這個檔案裡其他早就已經是「local copy」的檔案（aurora_ctrl_
# mux.v/waveform_controller.v/calib_mux.v 等）同一種模式。下面還留著的
# 「$src_dir14 file」/「$src_dir6 file」字樣是歷史記錄用的純文字註解
# （説明「這個檔案哪一天從共用檔案獨立出來」），不是變數參照，不影響
# 執行。
set fp_ip_repo "D:/Vivado/FrontPanel-Vivado-IP-Dist-v1.0.6/FrontPanel-Subsystem-v1.0.6"
set digi_repo  "D:/Vivado/ip_Digilent_vivado"

# 2026-07-26: master switch for all 6 debug system_ila cores (aurora_ila_0/
# dispatcher_ila_0/flash_ctrl_ila_0/ddr_writer_ila_0/system_ila_1/
# system_ila_2). Set to 0 to skip creating them entirely -- saves BRAM/LUT
# and shortens synth/impl time for builds where their debug visibility isn't
# needed; set back to 1 (and re-source) whenever ILA-based debugging is
# needed again. Each ILA's create_bd_cell/set_property/connect_bd_net block
# is wrapped in `if {$ENABLE_ILA} { ... }` right where it's defined below --
# search for "ENABLE_ILA" to find every gated block.
#
# 2026-07-28 重新打開（這台 Linux 機器）：T_EXT_CLK_SEL(0x28) 兩跳 relay
# 失敗，三站環路模擬（sim/tb_ext_clk_sel_ring_relay.v）PASS、無法重現，
# 懷疑收斂到 sys_clk<->aurora_clk 真正 CDC 時序或 SFP 實體層，模擬看不到，
# 下一步要在 Windows 機器拉 aurora_ila_0/dispatcher_ila_0 實測 board C 這一站
# （aurora_ila_0 的 dbg_beat0_to_relay/debug_tx_state/dbg_overflow/probe46-49
# 跟 dispatcher_ila_0 的 diag_idle_route/tx_tvalid/tx_tready 這次要看的
# probe 都已經接好，不用新增接線）。詳見 NOTES.md/PROJECT.md 2026-07-28
# 對應章節。重新關閉時機：這次 T_EXT_CLK_SEL relay 問題定案之後。
#
# 2026-08-05 關閉：T_EXT_CLK_SEL relay 問題已於 2026-07-28 同一天找到
# 根因並修好（src_id 合理性檢查 vs enum 前廣播的設計矛盾，見 NOTES.md
# 「T_EXT_CLK_SEL relay 失敗真正根因找到並修好」章節），觸發條件已滿足；
# 之後幾輪除錯（SFP TX_DISABLE/dac_mode_ramp 雙緩衝/group_trig_scheduler
# CDC race）都沒有再依賴這批 ILA。BRAM 已 93.33% 飽和，使用者要求先關閉
# 省資源。**`ENABLE_ILA=0` 這條路徑目前只跑過 create_bd dry-run，還沒
# 真正 build 出 bitstream 驗證過**（2026-07-26 加入當天就因為上面的
# debug 需求改回 1，從沒被完整建置測試過），下次 build 時留意。
set ENABLE_ILA 0

# -- Create project -----------------------------------------------------------
# 2026-07-06: rebuild failed with "Failed to remove the directory... might be
# in use by some other process" -- Hardware Manager's connection to the board
# (or its .ltx probes file, opened while looking at aurora_ila_0/dispatcher_ila_0)
# can hold a Windows file lock on $proj_dir/*.runs even after disconnect_hw_server/
# close_hw_manager, because those only close the CURRENT view/session object, not
# every open hw_target. Close every hw_target/hw_server explicitly first, then
# retry the delete a few times with a short pause -- these locks are usually
# released within a second or two once the handle is truly closed, not
# instantly at the point the disconnect command returns.
catch { close_hw_target [get_hw_targets -quiet] }
catch { foreach s [get_hw_servers -quiet] { disconnect_hw_server -quiet $s } }
catch { close_hw_manager }
catch { reset_run synth_1 }
catch { reset_run impl_1 }
catch { close_project }

set _cleanup_ok 0
for {set _try 0} {$_try < 5} {incr _try} {
    if {![file exists $proj_dir]} { set _cleanup_ok 1; break }
    if {![catch { file delete -force $proj_dir }]} { set _cleanup_ok 1; break }
    puts "WARN: $proj_dir still locked, retrying in 2s (attempt [expr {$_try+1}]/5)..."
    after 2000
}
if {!$_cleanup_ok} {
    error "Could not remove $proj_dir after 5 retries -- it's still locked by another\
process. Close any other Vivado window/instance with this project open, and in\
Hardware Manager make sure every hw_target is fully Disconnected (not just the\
window closed), then re-run this script."
}

# Opal Kelly's board_part definition (opalkelly.com:xem8320-au25p:...) is not
# bundled with Vivado -- it ships with Opal Kelly's own board files package.
# Point this at wherever you extracted/installed it (or register it globally
# via Vivado's board repository settings instead, and remove this line).
set_param board.repoPaths {/path/to/opalkelly/board_repo}

file mkdir $proj_dir
create_project $proj_name $proj_dir -force
set_property part xcau25p-ffvb676-2-e [current_project]
set_property board_part opalkelly.com:xem8320-au25p:part0:1.2 [current_project]

# -- IP repositories ------------------------------------------------------------
set_property IP_REPO_PATHS [list $fp_ip_repo $digi_repo] [current_project]
update_ip_catalog -rebuild

# -- Constraints ------------------------------------------------------------------
add_files -fileset constrs_1 -norecurse \
    "C:/path/to/awg-test-step-16/vivado/constraints/awg_step16.xdc"

# -- RTL sources --------------------------------------------------------------------
# Only files actually instantiated by a BD cell below are added. Step 14.3b removed
# Aurora entirely (aurora_packet_tx.v / aurora_tx_arbiter.v / sync_1bit.v -- all three
# superseded by the new Layer2/3/arbiter modules below, NOT revived) to rule out Aurora
# interference; leaving orphan (uninstantiated) sources in the fileset previously caused
# Vivado's auto top-detection to non-deterministically pick the wrong impl_1 top (seen in
# runme.log: "link_design -top aurora_packet_tx", bogus unplaced-IBUF errors) -- so keep
# this list exactly matching what's actually instantiated below.
foreach f { fp_ddr4_rw.v aurora_refclk_ibuf.v } {
    add_files -norecurse "C:/path/to/awg-test-step-16/rtl/$f"
}
# aurora_user_clk_buf.v: 2026-07-08 (step 15b) local copy (not the shared
# $src_dir6 file) -- FREQ_HZ attribute must match the actual line rate
# (15625000 Hz for our 1.0Gbps setting, not the shared file's 19531250 Hz
# for 1.25Gbps), see rtl/aurora_user_clk_buf.v header comment.
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/aurora_user_clk_buf.v"
# Step 15a Aurora 多板協定模組（見 PORTS.md「Aurora 多板協定模組」章節、
# PROJECT.md 設計文件 v2）：Layer 2（data channel + 通用 catch-all）、
# Layer 3（開機編號/trigger/data 預約三組協定）、tx1 仲裁器（合併兩者的
# tx1 請求成單一實體 TX）。
foreach f { aurora_data_channel.v aurora_ctrl_channel.v aurora_tx1_arbiter.v aurora_rx_merge.v } {
    add_files -norecurse "C:/path/to/awg-test-step-16/rtl/$f"
}
# aurora_data_channel_relayfifo.v: added 2026-07-17, replaces the module
# reference of aurora_data_channel_0 (see create_bd_cell below). The relay
# section now uses an external fifo_generator (relay_fifo_0, see below) for
# streaming buffering, replacing the original hand-written pkt_buf/pkt_wr_idx
# whole-packet store-and-forward logic (PKT_BUF_DEPTH=128 limit -- a
# T_WAVEFORM_STREAM packet exceeding 128 beats forwarded through a relay hop
# would lose the whole packet, confirmed with sim/tb_wave_stream_relay_
# overflow.v; fix verified in sim/tb_wave_stream_relay_fifo_fix.v /
# tb_wave_stream_relay_fifo_roundrobin.v, see PROJECT.md's 2026-07-17
# section). The original aurora_data_channel.v file is still kept in the
# add_files above (not removed yet, for comparison/rollback), but the BD no
# longer instantiates it.
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/aurora_data_channel_relayfifo.v"
# aurora_rx_beat0_gate.v: added 2026-07-29，集中處理 RX beat0 的
# src_id/dest_id/channel_up 合法性判斷，被 aurora_ctrl_channel.v（兩個
# instance：forward/backward）跟 aurora_data_channel_relayfifo.v（一個
# instance）當子模組 instantiate，不是獨立的 BD cell，只需要
# add_files 讓 Vivado 找得到原始碼，不需要額外 create_bd_cell/
# connect_bd_net。見 rtl/aurora_rx_beat0_gate.v 檔頭說明。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/aurora_rx_beat0_gate.v"
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/aurora_nfc_ctrl.v"
# dac_trig_queue.v: added 2026-07-30，trigger 補償延遲倒數本身搬到
# dac_clk domain（見該檔案檔頭說明、NOTES.md 2026-07-30「sine wave
# mode 精確度討論」章節），是真正的 BD cell（dac_trig_queue_0，見下方
# create_bd_cell），不是像 aurora_rx_beat0_gate.v 那樣的純子模組。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/dac_trig_queue.v"
# sine_gen.v/sine_ctrl_regs.v/dac_output_mux.v: added 2026-07-17, the new
# per-channel real-time sine wave generator feature (parallel to DDR4
# playback). See PROJECT.md's 2026-07-17 section for design rationale.
# sine_gen.v reads its LUT init values from rtl/sine_lut_16384.mem via
# $readmemh (generated by host/gen_sine_lut.py) -- referenced by absolute
# path via the LUT_FILE parameter override below, so it isn't added here
# as an HDL source.
foreach f { sine_gen.v sine_ctrl_regs.v dac_output_mux.v amp_ramp_gen.v amp_ctrl_mux.v } {
    add_files -norecurse "C:/path/to/awg-test-step-16/rtl/$f"
}
# board_cfg_reg.v: local copy (not the shared $src_dir14 file), 2026-07-14
# (step 16). Adds au_board_id_assign_wr/au_board_id_assign_value input --
# board_id can now be generated directly from the Aurora-enum board_index
# (T_BOARD_ID_ASSIGN=0x1E packet) instead of requiring set_board_cfg_direct.py
# over each board's own USB -- see rtl/board_cfg_reg.v header comment.
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/board_cfg_reg.v"
foreach f {
    diag_cdc.v
} {
    add_files -norecurse "C:/path/to/awg-test-step-16/rtl/$f"
}
# diag_capture.v: local copy (not the shared $src_dir14 file), 2026-07-29 —
# forked to fix a real okClk<->sys_clk CDC violation on the PO_DIAG(0xA0)
# BTPipeOut read side (po_ep_read sampled directly by sys_clk logic with no
# synchronizer). See rtl/diag_capture.v header comment + PROJECT.md/NOTES.md
# 2026-07-29 "status_reply_capture.v/diag_capture.v CDC 修法" section.
# Scoped to step-16 only so awg-test-step-14 is untouched.
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/diag_capture.v"
# local_reg_handler.v: local copy (not the shared $src_dir14 file), 2026-07-07
# (step 15b). Adds T_ENUM_START(0x1A)/T_TRIG_START(0x1B)/T_RESERVE_START(0x1C)
# decode (au_enum_start/au_trig_start/au_reserve_start/au_reserve_dest_id) --
# host-triggered Aurora protocols now go through the same packet path as every
# other control command (host->dispatcher->here), replacing the old TI
# bit 29/30/31 + WI 0x11 direct-wire approach -- see rtl/local_reg_handler.v
# header comment for the rationale.
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/local_reg_handler.v"
# sticky_latch.v: 2026-07-08 新增，debug-only 極簡 sticky 閂鎖，監測
# hard_err 這種瞬間訊號用（見下方 hard_err_cdc_0/1/hard_err_sticky_0/1）
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/sticky_latch.v"

# 2026-07-08 (step 15b): flash controller 接回來（14.3b 分支時刻意排除，見
# 本檔案別處 "Deviation: eos tied to const_one (no fpga_flash_ctrl in this
# scope)" 註解）。spi_flash_ctrl.v 跟資料格式無關（純 SPI 協定執行時序），
# 直接用 $src_dir14 共用檔案不用改；fpga_flash_ctrl.v/flash_startup_loader.v
# 是 15b local copy，新增 init_trig_delay 欄位（0xA0-0xA1，2 bytes，LE）存
# per-board trigger 延遲。
#
# 2026-07-08 第二輪：flash_config_writer.v 改成 15b local copy（原本跟
# spi_flash_ctrl.v 一起共用 $src_dir14）——T_FLASH_ERASE(0x0F) 上機測試完全
# 不動作（flash_status busy/done/err 三個 bit 持續 0），需要看這個模組內部
# state/do_program 才能判斷卡在哪，這兩個既有 port 沒有導出，新增
# diag_state[3:0]/diag_do_program 兩個純 assign 診斷輸出，不改動既有邏輯。
foreach f {
    spi_flash_ctrl.v
} {
    add_files -norecurse "C:/path/to/awg-test-step-16/rtl/$f"
}
foreach f {
    fpga_flash_ctrl.v
    flash_startup_loader.v
    flash_config_writer.v
} {
    add_files -norecurse "C:/path/to/awg-test-step-16/rtl/$f"
}
# Step 14.3b playback-path port: DDR4->AWG waveform playback (readers/FIFOs/wctrl/
# timers/calib), ported from awg-test-step-14. Aurora/flash/ext-clock excluded.
# 2026-07-04: simple_dc_fifo.v removed -- fifo_a/b_$ch replaced with the official
# xpm_fifo_async macro (inferred directly in RTL where instantiated, no add_files
# needed), per user request to prefer vendor-verified IP over custom hand-written
# CDC logic where possible.
# calib_mux.v: local copy (not the shared $src_dir6 file), 2026-07-27.
# 統一讀取/寫入架構收斂：拔除 fp_wr/fp_sel/fp_data（本機直寫路徑），
# fp_rst 改名 ext_rst 保留（board 整體 power-on reset，不是寫入路徑的
# 一部分，見 rtl/calib_mux.v 檔頭說明）。原檔案被 step-6~14 好幾個舊
# 專案共用引用，改指向 step-16 本地路徑不影響那些專案。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/calib_mux.v"
foreach f {
    dac_clk_mux.v
} {
    add_files -norecurse "C:/path/to/awg-test-step-16/rtl/$f"
}
# aurora_ctrl_mux.v: local copy (not the shared $src_dir14 file), 2026-07-27.
# 統一讀取/寫入架構收斂：拔除 fp_amp_ctrl_wr/fp_amp_ctrl_sel/fp_amp_
# ctrl_data（本機直寫路徑）+ au_board_cfg_wr/au_board_cfg_id/au_board_
# cfg_is_master/out_board_cfg_wr/id/is_master（T_BOARD_CFG 死路 pass-
# through），見 rtl/aurora_ctrl_mux.v 檔頭說明。原檔案被 step-8~14 好
# 幾個舊專案共用引用，改指向 step-16 本地路徑不影響那些專案。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/aurora_ctrl_mux.v"
foreach f {
    syzygy_ready.v
    sync_start.v
} {
    add_files -norecurse "C:/path/to/awg-test-step-16/rtl/$f"
}
# ddr4_stream_reader.v: local copy (not the shared $src_dir14 file), 2026-07-26.
# Adds an `idle` output (= state==S_IDLE) so waveform_controller.v's flush
# logic can wait for the reader to actually confirm idle before flushing its
# FIFO -- fixes a real bug where an in-flight AXI burst (S_AR/S_R/S_WRITE4
# never check play_en) could keep writing stale-index samples into a FIFO
# that was just reset, so the newly-active FIFO's first sample wasn't sample
# 0 -- see rtl/ddr4_stream_reader.v header comment and PROJECT.md「重大突破：
# 用 Opus 深度推理」小節 for the full root-cause analysis.
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/ddr4_stream_reader.v"
# awg_calib_regs.v: local copy (not the shared $src_dir14 file), 2026-07-15.
# Adds coef_readback output = coef[calib_sel]（Group 2 讀回功能的一部分），
# 其餘邏輯完全不動——見 rtl/awg_calib_regs.v 檔頭說明。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/awg_calib_regs.v"
# amp_ctrl_read_mux.v: 2026-07-15 新增，純組合邏輯 8:1 mux，讀
# aurora_ctrl_mux_0/out_amp_ctrl_0..7 這 8 條既有輸出做讀回用
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/amp_ctrl_read_mux.v"
# flash_target_sel_reg.v: 2026-07-15 新增，Flash「分開儲存」的目標 sector
# 選擇暫存器（host WI + Aurora T_FLASH_TARGET_SEL 合併，誰晚寫誰生效）
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/flash_target_sel_reg.v"
# sine_phase_acc_mux.v / aurora_reply_tx.v / status_reply_capture.v:
# 2026-07-27 新增，統一讀取/寫入架構的查詢機制（見 PROJECT.md「統一
# 讀取/寫入架構 — 完整規格」小節、PORTS.md 對應章節）。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/sine_phase_acc_mux.v"
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/aurora_reply_tx.v"
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/status_reply_capture.v"
# trig_timer.v: local copy (not the shared $src_dir14 file), 2026-07-14.
# Adds reinit_req（「初始化」指令的一部分，清空 trigger list）——見
# rtl/trig_timer.v 檔頭說明。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/trig_timer.v"
# group_trig_scheduler.v: 2026-08-04 新增（trigger group 排程功能三部曲
# 「C」）。每板一個實例（不是每 module 一個），仿 trig_timer.v 架構，見
# rtl/group_trig_scheduler.v 檔頭說明。2026-08-20 新增 arm_mode/
# first_trigger/fire_local（多板同步輪播 Architecture B），同一份檔案。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/group_trig_scheduler.v"
# trig_merge.v: 2026-08-20 新增（多板同步輪播 Architecture B，見
# rtl/trig_merge.v 檔頭說明）。純組合邏輯，把 dac_trig_queue_0 的既有
# 輸出跟 group_trig_scheduler_0/fire_local（本地自主觸發，不繞 Aurora
# ring）合併，兩端都是 dac_clk，不需要新的 CDC。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/trig_merge.v"
# fifo_event_reader.v: 2026-08-04 新增（trigger group 排程功能三部曲
# 「C」CDC 修法，取代原本誤用 level_cdc 跨越 group_select_out 這個
# 瞬態訊號的做法，見 rtl/fifo_event_reader.v 檔頭說明）。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/fifo_event_reader.v"
# ddr_zero_writer.v: 2026-07-14 新增，「初始化」指令的 DDR4 全區清空狀態機
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/ddr_zero_writer.v"
# waveform_controller.v: local copy (not the shared $src_dir14 file), 2026-07-04.
# Adds fifo_a/b_wr_rst_busy + fifo_a/b_rd_rst_busy inputs (2-FF synced where needed)
# so the flush/reconfigure state machine waits for xpm_fifo_async's own internal
# reset-busy handshake to clear, not just a fixed guessed FLUSH_CYCLES count -- see
# rtl/waveform_controller.v header comment for the full rationale.
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/waveform_controller.v"
# trigger_cdc.v: local copy (not the shared $src_dir14 file), 2026-07-04. Same
# module name/port list (src_clk/src_pulse/dst_clk/dst_pulse) as the original --
# internally now wraps the official xpm_cdc_pulse macro instead of a hand-written
# toggle+2FF synchronizer, per user request to prefer vendor-verified IP. Zero BD
# wiring changes needed -- see rtl/trigger_cdc.v header comment for the rationale.
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/trigger_cdc.v"
# level_cdc.v: 2026-07-06 (step 15a bugfix). Thin wrapper around the official
# xpm_cdc_array_single macro, same pattern as trigger_cdc.v but for quasi-static
# multi-bit signals (config/status) instead of single-cycle pulses. Added to fix
# a real bug found on hardware: aurora_ctrl_channel.v/aurora_data_channel.v header
# comments explicitly assume board_id/is_master/reserve_dest_id/init_ok/
# total_boards/board_index/reserve_ok/reserve_busy (and channel_up, same category)
# are synchronized externally by the BD via official XPM CDC macros, but the
# original wiring below connected them directly with zero synchronization --
# see rtl/level_cdc.v header comment for the full rationale.
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/level_cdc.v"
# handshake_level_cdc.v: 2026-07-29 新增，level_cdc.v 的替代實作（同一套
# port 介面，內部用 xpm_cdc_handshake 取代 xpm_cdc_array_single），這次
# 先只用在 board_index_cdc_0，見該檔案檔頭說明。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/handshake_level_cdc.v"
# group_trig_select.v: 2026-07-27 新增，Group-based Trigger 架構。純組合
# 邏輯 4:1 mux（trig_out = trig_pulse & group_select[group_id]），4 個
# instance（每個模組 A/B/C/D 各一個），見下方 group_trig_select_$ch
# 建立處註解。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/group_trig_select.v"
# dc_fifo_xpm.v: local copy, 2026-07-04. Thin wrapper around the official
# xpm_fifo_async macro (matches simple_dc_fifo.v's WIDTH/DEPTH/PROG_FULL_THRESH
# parameter names + wr_clk/wr_en/din/full/prog_full/rd_clk/rd_en/dout/empty port
# names for a minimal-diff swap); replaces fifo_a/b_$ch below -- see rtl/dc_fifo_xpm.v.
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/dc_fifo_xpm.v"
# ddr_writer.v: local copy (not the shared $src_dir14 file). Step 14.3b diagnostic
# single-beat (frame) version -- see the next_burst comment in rtl/ddr_writer.v
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/ddr_writer.v"
# dispatcher.v: local copy (not the shared $src_dir14 file). Adds diag_state /
# diag_idle_route / debug_disp_fwd_cnt diagnostic outputs for ILA -- see rtl/dispatcher.v.
# 2026-07-09: 新增 RT_FLASH 路由（type 0x14 -> flash_tdata/tvalid/tready），
# 見 PROJECT.md 第 25 節。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/dispatcher.v"
# flash_payload_cdc.v / okclk_rst_sync.v: 2026-07-09 新增，fpga_flash_ctrl_0
# 搬到 okClk 域後需要的 CDC 模組，見 PROJECT.md 第 25 節、rtl/ 各檔案檔頭
# 註解。save/erase 觸發跟 load_valid 這兩類單週期 pulse 的跨域改用既有的
# trigger_cdc（xpm_cdc_pulse 官方巨集 wrapper，見下方 dispatcher.v/
# board_cfg_reg.v 之後的 create_bd_cell），不再另外寫自訂模組。
foreach f {
    flash_payload_cdc.v
    okclk_rst_sync.v
} {
    add_files -norecurse "C:/path/to/awg-test-step-16/rtl/$f"
}
# 2026-07-10：fp_input.v 拆成三個檔案（見 PROJECT.md 第 32 節，fp_input_wr
# 為 ok_clk 寫入端、fp_fifo_wrapper 為獨立可見的 FIFO CDC 節點、fp_input_rd
# 為 sys_clk 讀出端），讓 BD 裡看得到中間的 CDC 邊界，方便之後加探測點。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/fp_input_wr.v"
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/fp_fifo_wrapper.v"
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/fp_input_rd.v"
# 2026-07-11：ext clock（AWG 多板 DAC 同步）第一階段——先只加
# IBUFDS_GTE4/BUFG_GT 薄殼 + 頻率量測 counter，還不接到 dac_clk_mux_0，
# 見 PROJECT.md「時脈路徑設計」章節。
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/ext_dac_clk_ibuf.v"
add_files -norecurse "C:/path/to/awg-test-step-16/rtl/ext_clk_freq_counter.v"
update_compile_order -fileset sources_1

# ==============================================================================
#  Block Design
# ==============================================================================
catch { close_bd_design -objects [get_bd_designs -quiet] }
catch {
    foreach bd_f [get_files -quiet -regexp {.*\.bd$}] {
        remove_files $bd_f
        file delete -force $bd_f
    }
}
create_bd_design "awg_step16_bd"
current_bd_design awg_step16_bd

# -- FrontPanel IP --------------------------------------------------------------
# keep the same WI/WO/TI/PI addresses as step-14 (host scripts compatibility)
create_bd_cell -type ip -vlnv opalkelly.com:ip:frontpanel:1.0 fp0
set_property -dict [list \
    CONFIG.BOARD       {XEM8320-AU25P} \
    CONFIG.WI.COUNT    {32}   \
    CONFIG.WI.ADDR_0   {0x00} CONFIG.WI.ADDR_1   {0x01} CONFIG.WI.ADDR_2   {0x02} \
    CONFIG.WI.ADDR_3   {0x03} CONFIG.WI.ADDR_4   {0x04} CONFIG.WI.ADDR_5   {0x05} \
    CONFIG.WI.ADDR_6   {0x06} CONFIG.WI.ADDR_7   {0x07} CONFIG.WI.ADDR_8   {0x08} \
    CONFIG.WI.ADDR_9   {0x09} CONFIG.WI.ADDR_10  {0x0A} CONFIG.WI.ADDR_11  {0x0B} \
    CONFIG.WI.ADDR_12  {0x0C} CONFIG.WI.ADDR_13  {0x0D} CONFIG.WI.ADDR_14  {0x0E} \
    CONFIG.WI.ADDR_15  {0x0F} CONFIG.WI.ADDR_16  {0x10} CONFIG.WI.ADDR_17  {0x11} \
    CONFIG.WI.ADDR_18  {0x12} CONFIG.WI.ADDR_19  {0x13} CONFIG.WI.ADDR_20  {0x14} \
    CONFIG.WI.ADDR_21  {0x15} CONFIG.WI.ADDR_22  {0x16} CONFIG.WI.ADDR_23  {0x17} \
    CONFIG.WI.ADDR_24  {0x18} CONFIG.WI.ADDR_25  {0x19} CONFIG.WI.ADDR_26  {0x1A} \
    CONFIG.WI.ADDR_27  {0x1B} CONFIG.WI.ADDR_28  {0x1C} CONFIG.WI.ADDR_29  {0x1D} \
    CONFIG.WI.ADDR_30  {0x1E} \
    CONFIG.WI.ADDR_31  {0x1F} \
    CONFIG.WO.COUNT    {32}   \
    CONFIG.WO.ADDR_0   {0x20} CONFIG.WO.ADDR_1   {0x21} CONFIG.WO.ADDR_2   {0x22} \
    CONFIG.WO.ADDR_3   {0x23} CONFIG.WO.ADDR_4   {0x24} CONFIG.WO.ADDR_5   {0x25} \
    CONFIG.WO.ADDR_6   {0x26} CONFIG.WO.ADDR_7   {0x27} CONFIG.WO.ADDR_8   {0x28} \
    CONFIG.WO.ADDR_9   {0x29} CONFIG.WO.ADDR_10  {0x2A} CONFIG.WO.ADDR_11  {0x2B} \
    CONFIG.WO.ADDR_12  {0x2C} CONFIG.WO.ADDR_13  {0x2D} CONFIG.WO.ADDR_14  {0x2E} \
    CONFIG.WO.ADDR_15  {0x2F} CONFIG.WO.ADDR_16  {0x30} CONFIG.WO.ADDR_17  {0x31} \
    CONFIG.WO.ADDR_18  {0x32} CONFIG.WO.ADDR_19  {0x33} CONFIG.WO.ADDR_20  {0x34} \
    CONFIG.WO.ADDR_21  {0x35} CONFIG.WO.ADDR_22  {0x36} CONFIG.WO.ADDR_23  {0x37} \
    CONFIG.WO.ADDR_24  {0x38} CONFIG.WO.ADDR_25  {0x39} CONFIG.WO.ADDR_26  {0x3A} \
    CONFIG.WO.ADDR_27  {0x3B} CONFIG.WO.ADDR_28  {0x3C} CONFIG.WO.ADDR_29  {0x3D} \
    CONFIG.WO.ADDR_30  {0x3E} CONFIG.WO.ADDR_31  {0x3F} \
    CONFIG.TI.COUNT    {1}  CONFIG.TI.ADDR_0   {0x40} \
    CONFIG.PO.COUNT    {2}  CONFIG.PO.ADDR_0   {0xA0} CONFIG.PO.ADDR_1 {0xA1} \
    CONFIG.BTPI.COUNT  {1}  CONFIG.BTPI.ADDR_0 {0x81} \
] [get_bd_cells fp0]
# 2026-07-27 新增：PO.ADDR_1=0xA1（PO_STATUS_REPLY，統一讀取/寫入架構
# 的查詢回覆），跟既有 PO.ADDR_0=0xA0（PO_DIAG）同一種 BTPipeOut，命名
# 慣例沿用「po」+ 位址十六進位（不含 0x）+「_ep_datain/_ep_read」，
# 0xA0 對應既有的 fp0/poa0_ep_*，故 0xA1 對應 fp0/poa1_ep_*（下面
# status_reply_capture_0 接線沿用這個推斷，若 BD 重建後 Vivado 產生的
# 實際 pin 名稱不同，需要照 get_bd_pins -of_objects [get_bd_cells fp0]
# 的實際輸出修正）。
# 2026-07-09：PI 0x80（原本的 flash payload 直接寫入端點）移除，見
# PROJECT.md 第 25 節——payload 改走 BTPI 0x81 既有封包管線，不需要
# PI.COUNT/ADDR_0 這個設定了。
# 2026-07-03: 0x81 (PI_WAVE_DATA) moved from PI to BTPI (Block-Throttled PipeIn).
# Root cause: okPipeIn (PI) hardcodes its READY status bit to 1 (see the unencrypted
# simulation model gateware/simulation/behavioral_model/okPipeIn_v.ttcl -- no ep_ready
# input pin exists on that module at all), so the host driver never knows whether our
# fp_input.v FIFO is actually ready -- it just blasts data through regardless. Our own
# ep_ready computation was doing nothing (nowhere to connect it). okBTPipeIn DOES have a
# real ep_ready input that drives okEH_READY, giving the host real block-level flow
# control -- this is the official-recommended way to do repeated/bulk PipeIn transfers
# reliably (see PipeTest example, C:/Program Files/Opal Kelly/FrontPanel-Platform/
# Examples/PipeTest/XEM8320-Verilog/). fp_input.v's ep_ready logic already updated to
# match (registered output, see rtl/fp_input.v). Port names on fp0 for 0x81 are NOT
# verified yet (BTPI likely exposes extra ep_blockstrobe/ep_ready ports beyond PI's
# ep_dataout/ep_write) -- confirm via `get_bd_pins -of_objects [get_bd_cells fp0]` after
# this BD rebuilds, before wiring fp_input_wr_0/dispatcher_0 connections below.

# -- Clock Wizard 0 ---------------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz clk_wiz_0
set_property -dict [list \
    CONFIG.CLK_IN1_BOARD_INTERFACE    {fixed_fabric_100mhz} \
    CONFIG.PRIM_SOURCE                {Differential_clock_capable_pin} \
    CONFIG.PRIM_IN_FREQ               {100.000} \
    CONFIG.CLKOUT1_USED               {true} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {100.0} \
    CONFIG.CLKOUT1_REQUESTED_PHASE    {0.0} \
    CONFIG.CLK_OUT1_PORT              {clk_100} \
    CONFIG.CLKOUT2_USED               {true} \
    CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {12.5} \
    CONFIG.CLK_OUT2_PORT              {clk_aurora_init} \
    CONFIG.CLKOUT3_USED               {true} \
    CONFIG.CLKOUT3_REQUESTED_OUT_FREQ {100.0} \
    CONFIG.CLKOUT3_REQUESTED_PHASE    {90.0} \
    CONFIG.CLK_OUT3_PORT              {clk_100_90} \
    CONFIG.RESET_TYPE                 {ACTIVE_LOW} \
    CONFIG.USE_LOCKED                 {true} \
] [get_bd_cells clk_wiz_0]

# -- proc_sys_reset -----------------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:proc_sys_reset rst_clk100
set_property -dict [list \
    CONFIG.C_EXT_RST_WIDTH      {1} \
    CONFIG.C_AUX_RST_WIDTH      {1} \
    CONFIG.C_NUM_PERP_ARESETN   {1} \
] [get_bd_cells rst_clk100]

# -- Aurora 64B/66B (SFP1, Lane X0Y8) ----------------------------------------------
# 2026-07-05 (step 15a): 復原（14.3b 為排除 Aurora 干擾整段註解掉，CONFIG 不變，
# IP 本身一直都有 m_axi_rx_*/s_axi_tx_* 兩個方向的 port，雙向化純粹是這次兩個
# 方向都接，不是 CONFIG 差異）
# 2026-07-08 第六輪：曾嘗試改成 10.0Gbps，但 generate_target 報
# gt_qpllclk_quad1_in/gt_qpllrefclk_quad1_in 沒接（1.25Gbps 用 CPLL，
# 10.0Gbps 需要 QPLL，aurora_64b66b_0/1 是同一個 quad X0Y2 上的兩個
# lane，QPLL 需要共用邏輯，正確的 Shared Logic 設定沒有把握盲改，風險
# 較高）。使用者確認 DAC 線材規格「Supported data rates: 10/1 Gbps」
# （不是 1.25/10.3125），改用 **125MHz × 8 = 1.0Gbps**（乾淨整數倍率，
# 比 1.25Gbps 更低，沿用原本能動的 CPLL 架構，不會有 QPLL 共用問題）。
# 10.0Gbps 這個方向擱置，之後如果要重新嘗試，要先用 Vivado GUI 的
# Re-customize IP 看清楚 Shared Logic 選項再動手，不要用 Tcl 盲猜參數。
#
# 2026-07-17：改回 10.0Gbps，這次透過 Vivado GUI 正確設定 Shared Logic
# （見上方擱置理由）：aurora_64b66b_0 改選 SupportLevel=1（Include Shared
# Logic in core），aurora_64b66b_1 維持 SupportLevel=0（in example
# design），兩者共用同一個 quad 的 QPLL，_0 產生、_1 消費。改了 Shared
# Logic 之後 aurora_64b66b_0 的 refclk1_in/user_clk/sync_clk/
# bufg_gt_clr_out/mmcm_not_locked 這 5 個 port 被拿掉，改成自己內部處理
# （直接吃實體差動 pin gt_refclk1_p/n，自己產生 user_clk_out/
# sync_clk_out），並新增 gt_qpllclk_quad1_out/gt_qplllock_quad1_out/
# gt_qpllrefclk_quad1_out/gt_qpllrefclklost_quad1_out 供 _1 共用。
# aurora_64b66b_1 的 port 完全不變，只多了 gt_qpllclk_quad1_in/
# gt_qplllock_quad1_in/gt_qpllrefclk_quad1_in/gt_qpllrefclklost_quad1
# （對應接上面 _0 的 4 個輸出）和一個 gt_to_common_qpllreset_out（_0
# 沒有對應輸入，驗證過留空不接不會報錯）。詳見 PROJECT.md 2026-07-17
# 章節。
# 2026-07-30：flow_mode 從 None 改成 Immediate_NFC，新增 s_axi_nfc_*
# port（tvalid/tdata[0:15]/tready），啟用 Native Flow Control——真正
# 根因是 async_fifo_aurora_rx/relay_fifo_0（都是 256 深）寫入側完全
# 沒有 flow control，本板下游處理跟不上時會無聲丟棄 Aurora beat，造成
# T_WAVEFORM_STREAM 大封包資料錯位（見 NOTES.md 2026-07-30「Opus agent
# 查出真正根因」章節）。啟用前已在 Vivado 裡直接驗證過：這個專案既有的
# interface_mode=Framing 設定已經自動滿足 NFC 需要的兩個前提
# （C_USER_INTERFACE=axi4_stream、dataflow_config=Duplex），只需要改
# 這一個參數，不用動其他 IP 設定。只在 aurora_64b66b_0（rx0/tx0）這顆
# 核心啟用——本板 Layer2 資料轉送（aurora_data_channel_0）固定只吃
# rx0、只送 tx1，rx1 只給 Layer3 低流量控制封包用，方向不會混淆，見
# 新增的 rtl/aurora_nfc_ctrl.v 檔頭完整說明。
create_bd_cell -type ip -vlnv xilinx.com:ip:aurora_64b66b aurora_64b66b_0
set_property -dict [list \
    CONFIG.C_LINE_RATE        {10}                        \
    CONFIG.C_REFCLK_FREQUENCY {125}                       \
    CONFIG.flow_mode          {Immediate_NFC}             \
    CONFIG.interface_mode     {Framing}                   \
    CONFIG.drp_mode           {Disabled}                  \
    CONFIG.SupportLevel       {1}                         \
    CONFIG.C_START_QUAD       {Quad_X0Y2}                 \
    CONFIG.C_START_LANE       {X0Y8}                      \
    CONFIG.C_REFCLK_SOURCE    {MGTREFCLK0_of_Quad_X0Y2}  \
] [get_bd_cells aurora_64b66b_0]
# aurora_refclk_ibuf_0：2026-07-17 第二輪修正時整個移除（不是留孤兒）——
# 見下方「Aurora refclk + user clock」段落的說明，這個模組自己的
# IBUFDS_GTE4 跟 aurora_64b66b_0 內部那顆撞同一組實體 pin，place_design
# 會報 unroutable。aurora_64b66b_1/refclk1_in 現在改吃
# aurora_64b66b_0/gt_refclk1_out。
# aurora_user_clk_buf_0：2026-07-17 起孤兒 cell（aurora_64b66b_0 改
# SupportLevel=1 後自己內部產生 user_clk_out/sync_clk_out，不再需要外部
# BUFG_GT 幫忙緩衝），2026-07-28 這顆孤兒的 BUFG_GT 首次真的冒出
# [DRC BFGTL-1] bad_BUFG_GT_muxing critical warning（跟 aurora_64b66b_0
# 內部的 BUFG_GT 共用同一條 txoutclk_out[0] clock net、但 CE/CLR 接的網
# 不一樣）。比照 2026-07-17 清 aurora_refclk_ibuf_0 孤兒 cell 的模式，
# 整個 cell 一起刪（不能只拔線留孤兒，完全懸空的 buffer 硬體原語本身
# 也會觸發另一個 DRC）。詳見 PROJECT.md/NOTES.md 2026-07-28 章節。
create_bd_cell -type module -reference aurora_user_clk_buf  aurora_user_clk_buf_1

# 2026-07-11：ext clock 第一階段（見 PROJECT.md）。MGTREFCLK1_226（M7/M6，
# 跟 Aurora 用的 MGTREFCLK0/P7-P6 是同 bank 不同組，不衝突）-> IBUFDS_GTE4
# -> ODIV2 -> BUFG_GT -> ext_clk_out，接一個頻率量測 counter，還不接到
# dac_clk_mux_0/dac_90_clk_mux_0（實際頻率待上機驗證後才接）。
create_bd_cell -type module -reference ext_dac_clk_ibuf     ext_dac_clk_ibuf_0
create_bd_cell -type module -reference ext_clk_freq_counter ext_clk_freq_counter_0

# -- Clock Wizard ext（2026-07-13，階段 A：實測 ext_clk_out ≈99.98MHz 後新增）---------
# PRIM_SOURCE=No_buffer：輸入已經是 ext_dac_clk_ibuf_0/ext_clk_out 這個 BUFG_GT
# 輸出（global buffer），不能再疊一層輸入緩衝器。CLKOUT1/2 對齊 clk_wiz_0 既有的
# clk_100/clk_100_90 命名慣例，之後階段 B 才接 dac_clk_mux_0/dac_90_clk_mux_0 的 I1。
create_bd_cell -type ip -vlnv xilinx.com:ip:clk_wiz clk_wiz_ext_0
set_property -dict [list \
    CONFIG.PRIM_SOURCE                {No_buffer} \
    CONFIG.PRIM_IN_FREQ               {100.000} \
    CONFIG.CLKOUT1_USED               {true} \
    CONFIG.CLKOUT1_REQUESTED_OUT_FREQ {100.0} \
    CONFIG.CLKOUT1_REQUESTED_PHASE    {0.0} \
    CONFIG.CLK_OUT1_PORT              {clk_ext_100} \
    CONFIG.CLKOUT2_USED               {true} \
    CONFIG.CLKOUT2_REQUESTED_OUT_FREQ {100.0} \
    CONFIG.CLKOUT2_REQUESTED_PHASE    {90.0} \
    CONFIG.CLK_OUT2_PORT              {clk_ext_100_90} \
    CONFIG.RESET_TYPE                 {ACTIVE_LOW} \
    CONFIG.USE_LOCKED                 {true} \
] [get_bd_cells clk_wiz_ext_0]

# -- Aurora 64B/66B (SFP2, Lane X0Y9) -----------------------------------------------
# 2026-07-17：C_LINE_RATE 跟著 aurora_64b66b_0 一起改成 10（見上方
# aurora_64b66b_0 建立處的說明）。SupportLevel 維持 {0}（in example
# design）不變，refclk1_in/user_clk/sync_clk/bufg_gt_clr_out/
# mmcm_not_locked 這些 port 全部沒變。
# 2026-07-30：flow_mode 從 None 改成 Immediate_NFC，理由見上方
# aurora_64b66b_0 同一天的修改註解——每片板子燒的是同一份 bitstream，
# 環路上收到 aurora_64b66b_0 送出 NFC 的一定是「前一片板子」的
# aurora_64b66b_1（同一顆 GT 核心的 RX 端），如果 aurora_64b66b_1 沒有
# 同步開 NFC，收到的 NFC control block 會被判定成 illegal BTF，觸發
# 連續 soft error（不是單純的 no-op，是真的會出錯）。這片板子自己的
# aurora_64b66b_1 不需要「發起」NFC（本板只在 rx0 方向可能壅塞，見
# rtl/aurora_nfc_ctrl.v 說明），所以 s_axi_nfc_tvalid 直接接 const_zero
# （見下方 wiring），不需要對應的控制模組。
create_bd_cell -type ip -vlnv xilinx.com:ip:aurora_64b66b aurora_64b66b_1
set_property -dict [list \
    CONFIG.C_LINE_RATE        {10}                        \
    CONFIG.C_REFCLK_FREQUENCY {125}                       \
    CONFIG.flow_mode          {Immediate_NFC}             \
    CONFIG.interface_mode     {Framing}                   \
    CONFIG.drp_mode           {Disabled}                  \
    CONFIG.SupportLevel       {0}                         \
    CONFIG.C_START_QUAD       {Quad_X0Y2}                 \
    CONFIG.C_START_LANE       {X0Y9}                      \
    CONFIG.C_REFCLK_SOURCE    {MGTREFCLK0_of_Quad_X0Y2}  \
] [get_bd_cells aurora_64b66b_1]

# -- RTL modules ---------------------------------------------------------------------
# aurora_packet_tx_0/aurora_tx_arbiter_0（14.3b 移除前的舊單向設計）確定不
# 復原，責任完全由下面三個新模組取代（見 PORTS.md「Aurora 多板協定模組」）
# 2026-07-17: module reference switched from aurora_data_channel to
# aurora_data_channel_relayfifo (relay now uses an external fifo_generator
# for streaming buffering, see the add_files comment above); the cell name
# stays aurora_data_channel_0, minimizing the change to existing wiring/probes.
create_bd_cell -type module -reference aurora_data_channel_relayfifo aurora_data_channel_0
create_bd_cell -type module -reference aurora_ctrl_channel aurora_ctrl_channel_0
create_bd_cell -type module -reference aurora_tx1_arbiter  aurora_tx1_arbiter_0
# aurora_rx_merge_0：合併 Layer2 的 local_tdata（本地送達）跟 Layer3 的
# local_inject_*（合成 TRIGGER 封包），見「trigger 改走 dispatcher 既有
# 管線」設計（PROJECT.md Trigger 多板同步協定章節）
create_bd_cell -type module -reference aurora_rx_merge      aurora_rx_merge_0
create_bd_cell -type module -reference board_cfg_reg     board_cfg_reg_0
# 2026-07-08 (step 15b): flash controller 接回來（見 add_files 處註解）。
# init_dna（96-bit）刻意不接任何 WO——這次範圍只管 board_id/is_master/
# scale_cfg/amp_ctrl/calib_coef/trig_delay 的存讀，DNA 顯示是獨立議題，
# 之後真的需要再加。
create_bd_cell -type module -reference fpga_flash_ctrl    fpga_flash_ctrl_0
# 2026-07-09：fpga_flash_ctrl_0 搬到 okClk 域（見 PROJECT.md 第 25 節）
# 需要的 CDC 模組。okclk_rst_sync_0：reset 進 okClk（比照 fp_input.v 的
# sys_rst_ok 手法）。flash_payload_cdc_0：dispatcher_0 新的 flash 路由
# 輸出（64-bit, sys_clk）-> fpga_flash_ctrl_0/pipe_wdata（32-bit, okClk）
# 的 CDC + 拆寬度。flash_erase_pulse_cdc_0/flash_load_valid_cdc_0：既有
# trigger_cdc（xpm_cdc_pulse 官方巨集 wrapper）處理觸發/開機載入這幾個
# 單週期 pulse 的跨域，不用自訂模組。
# 2026-07-09（第 29 節）：flash_save_pulse_cdc_0/flash_save_gate_0 已移除
# ——第 28 節加的「SAVE 觸發 + payload 收滿雙條件 AND」設計，上機測試
# 仍然失敗（save_trigger_in 觸發時 save_trigger_out 沒有反應），使用者
# 提出更好的做法：既然 flash_payload_cdc_0 本來就知道 payload 收滿的
# 時機（payload_complete），何必還要 host 額外送一個 T_FLASH_SAVE 封包
# 來觸發？直接讓 payload_complete 本身就是觸發訊號，收滿即自動寫入，
# 從根本上消除兩條路徑的協調問題。T_FLASH_ERASE 沒有 payload，不受
# 影響，維持獨立觸發。
create_bd_cell -type module -reference okclk_rst_sync     okclk_rst_sync_0
# 2026-07-10：第二顆 okclk_rst_sync，專門幫 TI bit28（ti_fifo_rst，見上方）
# 展寬+同步，餵給 fp_fifo_0/rst（跟 fpga_flash_ctrl_0 那顆 okclk_rst_sync_0
# 分開，這樣 fifo reset 不會連帶重置 flash controller，反之亦然）。
create_bd_cell -type module -reference okclk_rst_sync     fifo_rst_sync_0
create_bd_cell -type module -reference flash_payload_cdc  flash_payload_cdc_0
create_bd_cell -type module -reference trigger_cdc        flash_erase_pulse_cdc_0
create_bd_cell -type module -reference trigger_cdc        flash_load_valid_cdc_0
create_bd_cell -type module -reference fp_input_wr       fp_input_wr_0
create_bd_cell -type module -reference fp_fifo_wrapper   fp_fifo_0
create_bd_cell -type module -reference fp_input_rd       fp_input_rd_0
# 2026-07-29 新增 v2：PO_DIAG(0xA0)/PO_STATUS_REPLY(0xA1) 這兩個
# BTPipeOut 補 okClk<->sys_clk CDC（原本 diag_capture_0/status_reply_
# capture_0 直接被 okClk 域的 po_ep_read 取樣，是真實 CDC 違規，見
# rtl/diag_capture.v/status_reply_capture.v 檔頭說明、NOTES.md 2026-07-29
# 對應章節）。**v1（`fp_fifo_wrapper` 自由跑 wr_ptr + FIFO）上機驗證後
# 發現對齊不受控（host 讀到的第一個 word 不保證對應 tbl[0]），已捨棄**
# ——v2 改回「host 讀取驅動、okClk 域定址」，資料改用官方
# `xpm_cdc_handshake` 整批快照搬遷（跟 aurora_reply_tx_0 處理 phase_acc
# 同一種 pattern），CDC 邏輯完全包在 diag_capture_0/status_reply_
# capture_0 模組內部，BD 層只需要多接一個 okClk 域 reset。
create_bd_cell -type module -reference okclk_rst_sync    reply_diag_okclk_rst_sync_0
create_bd_cell -type module -reference dispatcher        dispatcher_0
create_bd_cell -type module -reference local_reg_handler local_reg_handler_0
create_bd_cell -type module -reference diag_cdc          diag_cdc_0
create_bd_cell -type module -reference diag_capture      diag_capture_0
# Step 14.3 v5: DDR write moved to ddr_writer_0; fp_ddr4_rw_1 kept for read-back
create_bd_cell -type module -reference ddr_writer  ddr_writer_0
create_bd_cell -type module -reference fp_ddr4_rw  fp_ddr4_rw_1
# 2026-07-14：「初始化」指令的一部分——掃過整個 DDR4 寫 0，見
# rtl/ddr_zero_writer.v 檔頭說明跟 PROJECT.md「stop/reset 需求」章節
create_bd_cell -type module -reference ddr_zero_writer ddr_zero_writer_0

# -- async FIFO local (dispatcher->local_reg_handler, same sys_clk -> Common Clock) --
# v9: dispatcher moved to sys_clk, both sides same clock, switched to Common_Clock_Block_RAM
create_bd_cell -type ip -vlnv xilinx.com:ip:fifo_generator async_fifo_local
set_property -dict [list \
    CONFIG.Fifo_Implementation          {Common_Clock_Block_RAM} \
    CONFIG.Input_Data_Width             {64}  \
    CONFIG.Input_Depth                  {256} \
    CONFIG.Output_Data_Width            {64}  \
    CONFIG.Performance_Options          {First_Word_Fall_Through} \
    CONFIG.Valid_Flag                   {true} \
    CONFIG.Programmable_Full_Type       {Single_Programmable_Full_Threshold_Constant} \
    CONFIG.Full_Threshold_Assert_Value  {200} \
] [get_bd_cells async_fifo_local]

# -- async FIFO Aurora RX (aurora_clk -> sys_clk, 64-bit) ----------------------------
# 2026-07-05 (step 15a): 復原，配置不變。下游改接 aurora_data_channel_0/
# local_tdata,local_tvalid（原本是直接接實體 aurora_64b66b_0/m_axi_rx_*，
# 現在 Layer 2 先做本地/轉送判斷，只有本地送達的才進這個 FIFO，見下方接線）
create_bd_cell -type ip -vlnv xilinx.com:ip:fifo_generator async_fifo_aurora_rx
set_property -dict [list \
    CONFIG.Fifo_Implementation          {Independent_Clocks_Block_RAM} \
    CONFIG.Input_Data_Width             {64}  \
    CONFIG.Input_Depth                  {256} \
    CONFIG.Output_Data_Width            {64}  \
    CONFIG.Performance_Options          {First_Word_Fall_Through} \
    CONFIG.Valid_Flag                   {true} \
    CONFIG.Programmable_Full_Type       {Single_Programmable_Full_Threshold_Constant} \
    CONFIG.Full_Threshold_Assert_Value  {128} \
] [get_bd_cells async_fifo_aurora_rx]

# -- async FIFO Aurora TX (sys_clk -> aurora_clk, 65-bit {tlast,tdata}) --------------
# 2026-07-05 (step 15a): 復原，配置不變。下游改接 aurora_data_channel_0/
# local_tx_tdata,local_tx_tvalid,local_tx_tlast（原本接舊的
# aurora_tx_arbiter_0/wave_*，現在接新 Layer 2 的本機發起輸入，port 語意
# 相同：都是「本板自己要發起的資料」）
create_bd_cell -type ip -vlnv xilinx.com:ip:fifo_generator async_fifo_aurora_tx
set_property -dict [list \
    CONFIG.Fifo_Implementation          {Independent_Clocks_Block_RAM} \
    CONFIG.Input_Data_Width             {65}  \
    CONFIG.Input_Depth                  {256} \
    CONFIG.Output_Data_Width            {65}  \
    CONFIG.Performance_Options          {First_Word_Fall_Through} \
    CONFIG.Valid_Flag                   {true} \
    CONFIG.Programmable_Full_Type       {Single_Programmable_Full_Threshold_Constant} \
    CONFIG.Full_Threshold_Assert_Value  {200} \
] [get_bd_cells async_fifo_aurora_tx]

# -- async FIFO reply TX (sys_clk -> aurora_clk, 65-bit {tlast,tdata}) ---------------
# 2026-07-27 新增（統一讀取/寫入架構）：aurora_reply_tx_0（sys_clk domain
# 組裝出的 T_STATUS_REPORT 回覆封包）要送進 aurora_tx1_arbiter_0
# （aurora_clk domain），配置照抄 async_fifo_aurora_tx（同樣是 sys_clk
# ->aurora_clk 的封包串流 CDC）。
create_bd_cell -type ip -vlnv xilinx.com:ip:fifo_generator async_fifo_reply_tx
set_property -dict [list \
    CONFIG.Fifo_Implementation          {Independent_Clocks_Block_RAM} \
    CONFIG.Input_Data_Width             {65}  \
    CONFIG.Input_Depth                  {256} \
    CONFIG.Output_Data_Width            {65}  \
    CONFIG.Performance_Options          {First_Word_Fall_Through} \
    CONFIG.Valid_Flag                   {true} \
    CONFIG.Programmable_Full_Type       {Single_Programmable_Full_Threshold_Constant} \
    CONFIG.Full_Threshold_Assert_Value  {200} \
] [get_bd_cells async_fifo_reply_tx]

# -- relay FIFO (aurora_clk -> aurora_clk, 65-bit {tlast,tdata}, Common Clock) -------
# Added 2026-07-17: streaming buffer for aurora_data_channel_relayfifo.v's
# relay section. Both read and write are in the aurora_clk domain (not a
# CDC, essentially the same same-clock situation as async_fifo_local), so
# it uses Common_Clock_Block_RAM, not Independent_Clocks. Width/depth/FWFT/
# Programmable_Full settings mirror async_fifo_aurora_tx above (also
# 65-bit={tlast,tdata}), keeping this project's existing fifo_generator
# configuration convention consistent.
create_bd_cell -type ip -vlnv xilinx.com:ip:fifo_generator relay_fifo_0
set_property -dict [list \
    CONFIG.Fifo_Implementation          {Common_Clock_Block_RAM} \
    CONFIG.Input_Data_Width             {65}  \
    CONFIG.Input_Depth                  {256} \
    CONFIG.Output_Data_Width            {65}  \
    CONFIG.Performance_Options          {First_Word_Fall_Through} \
    CONFIG.Valid_Flag                   {true} \
    CONFIG.Programmable_Full_Type       {Single_Programmable_Full_Threshold_Constant} \
    CONFIG.Full_Threshold_Assert_Value  {128} \
] [get_bd_cells relay_fifo_0]

# -- Aurora NFC 控制（2026-07-30 新增，見 rtl/aurora_nfc_ctrl.v 檔頭
#    完整背景說明）：async_fifo_aurora_rx/relay_fifo_0 任一個接近滿
#    （兩者都已經設定 Programmable_Full_Type，prog_full 這個 port
#    本來就存在，只是原本沒接出去），OR 起來當作「請求對面暫停送
#    資料」的觸發訊號，餵給 aurora_nfc_ctrl_0 送 NFC 給 aurora_64b66b_0
# ----------------------------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_nfc_congested
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_nfc_congested]
connect_bd_net [get_bd_pins async_fifo_aurora_rx/prog_full] [get_bd_pins or_nfc_congested/Op1]
connect_bd_net [get_bd_pins relay_fifo_0/prog_full]         [get_bd_pins or_nfc_congested/Op2]

create_bd_cell -type module -reference aurora_nfc_ctrl aurora_nfc_ctrl_0
connect_bd_net [get_bd_pins or_nfc_congested/Res] [get_bd_pins aurora_nfc_ctrl_0/congested]
connect_bd_net [get_bd_pins aurora_nfc_ctrl_0/nfc_tvalid] [get_bd_pins aurora_64b66b_0/s_axi_nfc_tvalid]
connect_bd_net [get_bd_pins aurora_nfc_ctrl_0/nfc_tdata]  [get_bd_pins aurora_64b66b_0/s_axi_nfc_tdata]
connect_bd_net [get_bd_pins aurora_64b66b_0/s_axi_nfc_tready] [get_bd_pins aurora_nfc_ctrl_0/nfc_tready]

# -- DDR4 MIG --------------------------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:ddr4 ddr4_0
set_property -dict [list \
    CONFIG.C0.DDR4_InputClockPeriod  {9996} \
    CONFIG.C0.DDR4_TimePeriod        {833}  \
    CONFIG.C0.DDR4_CasLatency        {17}   \
    CONFIG.C0.DDR4_CasWriteLatency   {12}   \
    CONFIG.C0.DDR4_MemoryPart        {MT40A512M16LY-075} \
    CONFIG.C0.DDR4_DataWidth         {16}   \
    CONFIG.C0.BANK_GROUP_WIDTH       {1}    \
    CONFIG.C0_CLOCK_BOARD_INTERFACE  {fixed_ddr4_100mhz} \
    CONFIG.C0_DDR4_BOARD_INTERFACE   {ddr4} \
] [get_bd_cells ddr4_0]

# -- SmartConnect (3 slaves: rw_0 + rw_1 + ddr_zero_writer_0, sys_clk side) -----------
# rw_0/rw_1/ddr_zero_writer_0 all on sys_clk, share SmartConnect -> each has its own
# upstream AXI CC crossing into ddr4_ui_clk (2026-07-14: NUM_SI 10->11 for
# ddr_zero_writer_0/m_axi, new S10)
create_bd_cell -type ip -vlnv xilinx.com:ip:smartconnect smartconnect_0
set_property -dict [list CONFIG.NUM_SI {11} CONFIG.NUM_MI {1} CONFIG.NUM_CLKS {1}] \
    [get_bd_cells smartconnect_0]

# -- ddr_writer_0 upstream CDC (sys_clk -> DDR4 UI clock) -----------------------------
# 2026-07-15：從 axi_clock_converter 改成 fifo_generator（AXI4 介面、只選
# Write Channels、Independent Clocks），排查 T_WAVEFORM_STREAM 寫入 DDR4
# 內容錯位問題時使用者提出的方向，查證 PG057 文件 + 使用者在 Vivado GUI
# 精靈裡實際設定過一次、用 report_property 核對過的確切數值（見
# PROJECT.md「T_WAVEFORM_STREAM」章節完整討論）。ddr_writer_0 這邊完全
# 不用改，只是下游接的 BD cell 換了（S_AXI 接 ddr_writer_0/m_axi，M_AXI
# 接 smartconnect_0/S00_AXI，跟原本 axi_cc_ddr4 的接法一樣）。
create_bd_cell -type ip -vlnv xilinx.com:ip:fifo_generator ddr_writer_axi_fifo
set_property -dict [list \
    CONFIG.INTERFACE_TYPE           {AXI_MEMORY_MAPPED} \
    CONFIG.PROTOCOL                 {AXI4} \
    CONFIG.READ_WRITE_MODE          {WRITE_ONLY} \
    CONFIG.Clock_Type_AXI           {Independent_Clock} \
    CONFIG.DATA_WIDTH               {128} \
    CONFIG.ADDRESS_WIDTH            {32} \
    CONFIG.FIFO_Implementation_wach {Independent_Clocks_Distributed_RAM} \
    CONFIG.FIFO_Implementation_wdch {Independent_Clocks_Builtin_FIFO} \
    CONFIG.FIFO_Implementation_wrch {Independent_Clocks_Distributed_RAM} \
    CONFIG.Reset_Type               {Asynchronous_Reset} \
] [get_bd_cells ddr_writer_axi_fifo]
# Step 14.3b playback port: second upstream converter for fp_ddr4_rw_1 (sys_clk ->
# ddr4_ui_clk). axi_cc_fpddr4 是 fp_ddr4_rw_1 用的（was S01），這條沒有動，
# 只換了 ddr_writer_0 這條（原本叫 axi_cc_ddr4，was S00）。
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_clock_converter axi_cc_fpddr4
set_property -dict [list \
    CONFIG.DATA_WIDTH             {128} \
    CONFIG.ACLK_ASYNC             {1}   \
    CONFIG.SYNCHRONIZATION_STAGES {3}   \
] [get_bd_cells axi_cc_fpddr4]
# 2026-07-14：第三個上游 converter，給 ddr_zero_writer_0/m_axi 用（was S10）
create_bd_cell -type ip -vlnv xilinx.com:ip:axi_clock_converter axi_cc_ddrzero
set_property -dict [list \
    CONFIG.DATA_WIDTH             {128} \
    CONFIG.ACLK_ASYNC             {1}   \
    CONFIG.SYNCHRONIZATION_STAGES {3}   \
] [get_bd_cells axi_cc_ddrzero]

# -- NOT gates -----------------------------------------------------------------------
# 2026-07-05 (step 15a): 復原 inv_mmcm_locked（Aurora reset_pb/mmcm_not_locked
# 用）跟 inv_tx_fifo_full（async_fifo_aurora_tx prog_full -> dispatcher
# tx_tready 用），配置不變
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic inv_mmcm_locked
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {not}] [get_bd_cells inv_mmcm_locked]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic inv_local_fifo_full
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {not}] [get_bd_cells inv_local_fifo_full]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic inv_ddr4_rst
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {not}] [get_bd_cells inv_ddr4_rst]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic inv_tx_fifo_full
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {not}] [get_bd_cells inv_tx_fifo_full]

# -- TI slices -----------------------------------------------------------------------
# 2026-07-27：ti_board_cfg_wr（bit15，board_id/is_master 本機直寫觸發）
# 已移除——統一讀取/寫入架構收斂，board_id/is_master 收斂到
# T_BOARD_ID_ASSIGN(0x1E) 一條路徑，bit15 空出可用（見 PORTS.md 表2）。
# 2026-07-07 (step 15b)：bit 29/30/31（原本 ti_au_init/ti_au_trig/
# ti_au_reserve，15a 用的 TI bit 直接接線）已拿掉——host 觸發 Aurora
# 多板協定改成跟其他控制命令一樣走封包（見下面 local_reg_handler_0/
# au_enum_start 等新輸出，直接接 CDC，不再需要這三個 TI slice）。
#
# 2026-07-08 第四輪（debug-only，暫時加回來）：T_RESERVE_START 封包路徑
# 上機測試持續失敗，且 ILA 顯示連 local_reg_handler_0/rx_tvalid 都沒有
# 因為 reserve 測試觸發（懷疑封包解碼本身有問題，但靜態閱讀找不出原因）。
# 加回 15a 原本的 ti_au_reserve（TI bit31）當平行除錯路徑，跟封包路徑的
# au_reserve_start 用 util_vector_logic OR 在一起餵給 au_reserve_pulse_cdc_0
# （見下方 CDC 接線處），藉此隔離「問題在封包解碼」還是「問題在
# aurora_ctrl_channel.v/CDC 本身」——不影響封包路徑，之後確認完可以拔掉。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_au_reserve
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {31} CONFIG.DIN_TO {31}] \
    [get_bd_cells ti_au_reserve]
# 2026-07-08 第五輪（debug-only 實驗）：TI bit30 = 手動觸發「額外重置
# aurora_ctrl_channel_0/aurora_data_channel_0（+ 同組的 tx1_arbiter/
# rx_merge/async_fifo_aurora_rx）」，測試「channel_up 確認穩定後再補一次
# reset，能不能把 reserve 隨機失敗的問題排除」這個假設。見下方
# au_ctrl_rst_pulse_cdc_0/or_aurora_extra_rst 接線處。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_au_ctrl_rst
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {30} CONFIG.DIN_TO {30}] \
    [get_bd_cells ti_au_ctrl_rst]
# 2026-07-22：TI bit7（PORTS.md 表2 標記空號）= 手動觸發「整個 Aurora
# 子系統回到乾淨狀態」，範圍比既有 ti_au_ctrl_rst 大很多——不只
# aurora_ctrl_channel_0/aurora_data_channel_0 這組，還包含 GT 本身
# （pma_init，強制重新做實體層鏈路訓練）跟 async_fifo_aurora_tx（TX
# 方向進 Aurora 前的 FIFO，sys_clk domain，之前完全沒被任何 debug
# reset 涵蓋到）。目的：讓 JTAG/ILA 全程保持連線的情況下，能重複
# 觸發鏈路重新建立，才有辦法直接擷取到雜訊出現的當下（見 NOTES.md
# 「Aurora Layer 2 資料轉送與 TX 仲裁」章節 2026-07-22 小節，
# reload_only.py 這類會讓 FPGA 整個重新配置的操作，JTAG debug core
# 也會跟著斷線，沒辦法擷取到重新配置那個瞬間）。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_au_full_reset
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {7} CONFIG.DIN_TO {7}] \
    [get_bd_cells ti_au_full_reset]
# 2026-07-08（debug-only，flash round-trip 除錯）：TI bit29（15b 移除
# ti_au_init 後空出來的位元）= 獨立觸發 fpga_flash_ctrl_0 自己的
# soft-reset，跟 board_cfg_reg_0/dispatcher_0 等共用 peripheral_reset 的
# 模組分開，讓 flash_startup_loader 可以在同一次開機內重新讀一次 flash，
# 不用真的 reload 就能驗證 flash-save 剛寫入的內容對不對（沿用
# step-10.1 spi_test_top.v 已驗證過的 write-then-verify 手法，改成 host
# 手動觸發）。見下方 or_flash_ctrl_rst 接線處。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_flash_ctrl_rst
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {29} CONFIG.DIN_TO {29}] \
    [get_bd_cells ti_flash_ctrl_rst]
# 2026-07-10（debug/測試用）：TI bit28 = 獨立觸發 fp_fifo_0（BTPipeIn ->
# dispatcher 那顆 xpm_fifo_async）+ fp_input_wr_0 pair-acc 狀態一起重置，
# 不動 board_id/dispatcher/其他模組狀態。測試時如果懷疑 FIFO 卡住舊資料
# （見 PROJECT.md 第 32 節 backlog 理論），host 可以呼叫這個而不用整個
# reload。見下方 fifo_rst_sync_0/or_fifo_rst/or_fp_half_reset 接線處。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_fifo_rst
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {28} CONFIG.DIN_TO {28}] \
    [get_bd_cells ti_fifo_rst]
# bit 16 = TI_BIT_FLUSH_STANDBY: resets fp_input half flag, call before WriteToPipeIn
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_fp_align
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {16} CONFIG.DIN_TO {16}] \
    [get_bd_cells ti_fp_align]
# bit 17 = TI_BIT_REINIT（2026-07-14 新增，「初始化」指令，跟單純的
# play_en=0 播放停止不同，見 PROJECT.md「stop/reset 需求」章節）：清空
# 每個 channel 的 playlist（wctrl_$ch）、trigger list（trig_timer_$port）、
# 整個 DDR4（ddr_zero_writer_0），並讓 wctrl 立即輸出 0V
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_reinit
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {17} CONFIG.DIN_TO {17}] \
    [get_bd_cells ti_reinit]
# 2026-07-27：ti_scale_wr（bit18）/ti_ext_clk_sel_wr（bit8）/ti_dac_mode_
# ramp_wr（bit9）已移除——統一讀取/寫入架構收斂，scale_cfg/ext_clk_sel/
# dac_mode_ramp 這 3 組本機直寫路徑都拔除，只剩 Aurora 封包路徑，
# bit8/9/18 空出可用（見 PORTS.md 表2）。
# bit 19 = flash_target_sel fp_wr（2026-07-15 新增，host 觸發把 WI 0x1a
# 目前的值寫進 flash_target_sel_reg_0/sel，決定下一次 flash-save/erase
# 要動哪個 sector）
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_flash_target_sel_wr
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {19} CONFIG.DIN_TO {19}] \
    [get_bd_cells ti_flash_target_sel_wr]
# bit 10 = fp_per_hop_value_wr（2026-07-24 新增，au_trig_delay 自動校準
# 機制除錯用，host 觸發把 WI 0x1F 目前的值寫進 board_cfg_reg_0/
# per_hop_value_manual + 設定 sticky manual_per_hop_active，見 rtl/
# board_cfg_reg.v port 註解）
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_per_hop_value_wr
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {10} CONFIG.DIN_TO {10}] \
    [get_bd_cells ti_per_hop_value_wr]
# bit 12 = ti_debug_reply_trig（2026-07-29 新增，診斷專用，取代原本
# T_DEBUG_REPLY_INJECT(0x1F) 封包機制——使用者要求最直接的觸發方式，
# TI 觸發直接進 local_reg_handler_0，不經過封包/dispatcher/case 解碼）。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_debug_reply_trig
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {12} CONFIG.DIN_TO {12}] \
    [get_bd_cells ti_debug_reply_trig]

# -- WI slices -----------------------------------------------------------------------
# WI 0x1a bits[1:0]：flash_target_sel（2026-07-15 新增，「分開儲存」目標
# sector 選擇，0=身份/1=scale_cfg/2=amp_ctrl/3=calib_coef）
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice flash_target_sel_slice
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {1} CONFIG.DIN_TO {0}] \
    [get_bd_cells flash_target_sel_slice]
# WI 0x1e bits[6:0]：ddr_writer_0/raw_rd_addr（2026-07-15 新增，排查
# T_WAVEFORM_STREAM 寫入 DDR4 內容錯位的診斷擷取，見 PROJECT.md）。
# 原本想用 0x1f，但 fp0 的 CONFIG.WI.COUNT 只到 31（涵蓋 0x00-0x1E），
# 0x1f 根本沒有實際配置在 IP 裡（PORTS.md 標的「可用」只代表沒被其他
# 功能佔用，不代表 IP 已經有這個位址），改用範圍內的 0x1e。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice raw_rd_addr_slice
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {6} CONFIG.DIN_TO {0}] \
    [get_bd_cells raw_rd_addr_slice]
# 2026-07-27：board_id_slice/is_master_slice（WI 0x0D 拆解，board_id/
# is_master 本機直寫用）已移除——收斂到 T_BOARD_ID_ASSIGN(0x1E) 一條
# 路徑，WI 0x0D 已孤立（見 PORTS.md 表3）。
# 2026-07-07 (step 15b): WI 0x11（原本 reserve_dest_id_slice，15a 用的直接
# 接線）已拿掉——`reserve_dest_id` 改成從 T_RESERVE_START(0x1C) 封包的
# beat1 帶過來（見 local_reg_handler_0/au_reserve_dest_id）。
#
# 2026-07-08 第四輪（debug-only，暫時加回來，理由同上方 ti_au_reserve）
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice reserve_dest_id_slice
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {15} CONFIG.DIN_TO {0}] \
    [get_bd_cells reserve_dest_id_slice]

# 2026-07-13（ext clock 階段 B）：WI 0x18 bit0 原本是 dac_clk_mux_0/
# dac_90_clk_mux_0 共用的 S select 暫存區，2026-07-24 改成先進
# board_cfg_reg_0 的 TriggerIn 觸發式暫存器（真正的 S 改讀 board_cfg_
# reg_0/ext_clk_sel，見下方 DAC clock mux 接線處）。**2026-07-27 起
# wi_ext_clk_sel 這條本機直寫暫存區整個移除**——統一讀取/寫入架構
# 收斂，只剩 Aurora T_EXT_CLK_SEL(0x28) 一條路徑，WI 0x18 已孤立（見
# PORTS.md 表3）。

# -- Constants -----------------------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_one
set_property -dict [list CONFIG.CONST_VAL {1} CONFIG.CONST_WIDTH {1}] [get_bd_cells const_one]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_zero
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {1}] [get_bd_cells const_zero]

# aurora_64b66b_1 開了 Immediate_NFC 後多出來的 s_axi_nfc_tvalid 這個
# 新 input pin，本板不需要對這個方向發起 NFC（見上方 aurora_64b66b_1
# 定義處註解），tie 常數 0，避免留下懸空 port（見 rtl/aurora_nfc_ctrl.v
# 章節 2026-07-30 review 記錄）。
connect_bd_net [get_bd_pins const_zero/dout] [get_bd_pins aurora_64b66b_1/s_axi_nfc_tvalid]

create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_3b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {3}] [get_bd_cells const_3b0]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_8hff
set_property -dict [list CONFIG.CONST_VAL {255} CONFIG.CONST_WIDTH {8}] [get_bd_cells const_8hff]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_28b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {28}] [get_bd_cells const_28b0]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_30b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {30}] [get_bd_cells const_30b0]
# 2026-07-08（debug-only）：hard_err sticky 狀態 -> WO 0x2C（[0]=aurora_0,
# [1]=aurora_1）
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_hard_err
set_property -dict [list CONFIG.NUM_PORTS {3} CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {30}] \
    [get_bd_cells concat_hard_err]
# 2026-07-08（debug-only）：reserve 診斷 sticky 狀態 -> WO 0x2D（[0]=曾經
# 送出過 REQ、[1]=曾經逾時沒回覆、[2]=曾經收到明確回覆、[3]=曾經看到
# reserve_start_pulse 本身送達 ctrl_channel（aurora_clk 側）、[4]=曾經看到
# au_reserve_start 本身觸發（sys_clk 側，CDC 之前，local_reg_handler_0
# 解碼直接輸出）——[3]/[4] 對照能分辨「封包解碼沒觸發」還是「CDC 弄丟了」
# 2026-07-29 追加：[5]=曾經看到 aurora_tx1_arbiter_0/reply_tx1_tvalid
# （reply 有沒有被送到這個 arbiter 面前）、[6]=曾經完成
# reply_tx1_tvalid&&reply_tx1_tready 握手（reply 有沒有真的被這個
# arbiter 接受、送上 tx1）——這兩個原本只有 aurora_ila_0 probe83/84
# 能看（需要 Hardware Manager），這裡另外提供 USB 直接可讀的 sticky
# 摘要版，見 rtl/aurora_tx1_arbiter.v dbg_reply_seen/dbg_reply_granted
# port 註解、NOTES.md 2026-07-29「reply 寫入端 USB 可讀診斷」章節。
# 2026-07-29 再追加：[7:22]（16-bit）=au_reply_src、[23:30]（8-bit）=
# au_reply_query_type——使用者要求「用最直接的方式讀那個暫存器」，取代
# 這幾輪一直在修的 PO_STATUS_REPLY/CDC 機制（那條路徑仍然是驗證完整
# 60-word payload的唯一方式，不會被取代，這裡只是額外提供一個不經過
# BTPipeOut/CDC、單純讀 WO 就能看到 au_reply_src/au_reply_query_type
# 這 24-bit「當下值」的捷徑）。原本 25-bit 常數佔位縮成 1-bit
# （32 - 5 個既有 bit - 2 個 reply_seen/granted - 24-bit src/query_type）。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_1b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {1}] [get_bd_cells const_1b0]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_reserve_diag
set_property -dict [list CONFIG.NUM_PORTS {10} CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} \
    CONFIG.IN2_WIDTH {1} CONFIG.IN3_WIDTH {1} CONFIG.IN4_WIDTH {1} CONFIG.IN5_WIDTH {1} \
    CONFIG.IN6_WIDTH {1} CONFIG.IN7_WIDTH {16} CONFIG.IN8_WIDTH {8} CONFIG.IN9_WIDTH {1}] \
    [get_bd_cells concat_reserve_diag]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_32b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {32}] [get_bd_cells const_32b0]

# 2026-07-23：const_64b0/const_7b0/const_4b0/const_8b0/const_18b0
# 移除——使用者在 Vivado GUI 檢查 build 結果時發現這 5 個 xlconstant
# 從來沒有被任何 connect_bd_net 接上（`grep connect_bd_net.*const_XXb0`
# 全部零筆），是歷史遺留的孤兒 cell（comment 講的「Step 14.3b playback
# port 用」那個用途後來改了別的接法，這幾個常數留下來沒清掉）。既然
# 這個專案的 create_bd.tcl 是每次從頭重建（見 CLAUDE.md「每次都重建
# BD」慣例），直接刪掉這幾行，不用保留孤兒 cell。
# 2026-07-05 (step 15a): WO 0x26 ENUM 狀態 concat 的補零用
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_21b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {21}] [get_bd_cells const_21b0]
# 2026-07-07 (step 15b): WO 0x29 trig_delay readback concat 的補零用
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_16b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {16}] [get_bd_cells const_16b0]
# 2026-07-15：WO 0x35（scale_cfg）/WO 0x36/0x37（amp_ctrl/calib_coef）
# 讀回 concat 的補零用
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_24b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {24}] [get_bd_cells const_24b0]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_14b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {14}] [get_bd_cells const_14b0]
# 2026-07-24（trigger 統一化架構改版）：aurora_ctrl_mux_0/au_trig_port_mask
# 的補零用——local_reg_handler_0 的 au_trig_port_wr/au_trig_port_mask
# output port 已移除（T_TRIG_PORT 機制拿掉），但 aurora_ctrl_mux_0
# （awg-test-step-14 共用檔案，不在這次修改範圍內）仍宣告這兩個 input
# port，必須明確 tie-off 常數，不能留下指向不存在 pin 的 connect_bd_net。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_4b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {4}] [get_bd_cells const_4b0]
# 2026-07-24：WO 0x34（concat_rt_board_info）剩餘 2-bit 保留位元補零用
# （NUM_PORTS 3->4->5，見該 cell 建立處註解）
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_2b0
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {2}] [get_bd_cells const_2b0]

# 2026-08-05 新增：SFP TX_DISABLE 固定拉低致能雷射，見 TDIS_1/TDIS_2
# port 建立處註解。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_tdis_1
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {1}] [get_bd_cells const_tdis_1]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconstant const_tdis_2
set_property -dict [list CONFIG.CONST_VAL {0} CONFIG.CONST_WIDTH {1}] [get_bd_cells const_tdis_2]

# -- Concat cells --------------------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_aurora_status
set_property -dict [list CONFIG.NUM_PORTS {3} \
    CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {30}] \
    [get_bd_cells concat_aurora_status]

# WO 0x26 ENUM 狀態：bit0=init_ok, bits[5:1]=total_boards(5b), bits[10:6]=board_index(5b)
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_enum_status
set_property -dict [list CONFIG.NUM_PORTS {4} \
    CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {5} CONFIG.IN2_WIDTH {5} CONFIG.IN3_WIDTH {21}] \
    [get_bd_cells concat_enum_status]

# WO 0x28 RESERVE 狀態：bit0=reserve_ok, bit1=reserve_busy
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_reserve_status
set_property -dict [list CONFIG.NUM_PORTS {3} \
    CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {30}] \
    [get_bd_cells concat_reserve_status]

# WO 0x29 trig_delay readback：[15:0]=au_trig_delay 現值（sys_clk cycle 數）
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_trig_delay
set_property -dict [list CONFIG.NUM_PORTS {2} \
    CONFIG.IN0_WIDTH {16} CONFIG.IN1_WIDTH {16}] \
    [get_bd_cells concat_trig_delay]

# WO 0x2A flash_status readback（2026-07-08 step 15b 新增）：[3:0]=flash_status
# ([0]=busy [1]=done [2]=err [3]=loader_done)
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_flash_status
set_property -dict [list CONFIG.NUM_PORTS {2} \
    CONFIG.IN0_WIDTH {4} CONFIG.IN1_WIDTH {28}] \
    [get_bd_cells concat_flash_status]

create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_diag
set_property -dict [list CONFIG.NUM_PORTS {5} \
    CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {1} \
    CONFIG.IN3_WIDTH {1} CONFIG.IN4_WIDTH {28}] [get_bd_cells concat_diag]

# 2026-07-24：NUM_PORTS 3->4->5，借用原本 15-bit 保留零位元讀回
# board_cfg_reg_0/ext_clk_sel（1 bit，bit16）+ dac_mode_ramp（12 bit，
# bit17-28，2026-07-24 再追加，理由同 ext_clk_sel：host 需要能讀回
# Aurora 有沒有把 dac_output_mux/amp_ctrl_mux 模式改掉）。剩 2 bit
# （bit29-30）維持保留零位元，純加法、不影響既有讀 board_id/is_master
# 的 host 程式碼。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_rt_board_info
set_property -dict [list CONFIG.NUM_PORTS {5} \
    CONFIG.IN0_WIDTH {16} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {1} \
    CONFIG.IN3_WIDTH {12} CONFIG.IN4_WIDTH {2}] \
    [get_bd_cells concat_rt_board_info]

# 2026-08-05 新增：dac_mode_ramp 併入 sine_ctrl_regs_0 後，原本 12-bit
# board_cfg_reg_0/dac_mode_ramp 這個單一訊號不存在了，改用這顆 concat
# 把 sine_ctrl_regs_0 輸出的 12 個獨立 1-bit（mode_active_0..3/
# ramp_en_active_0..7）重新組回原本的 12-bit 排列（bit[3:0]=mode，
# bit[11:4]=ramp_en，跟原本 dac_mode_ramp 的 bit 定義一致），餵給
# concat_rt_board_info/In3（WO 0x34 本機讀回）跟 aurora_reply_tx_0/
# bi_dac_mode_ramp（QT_BOARD_INFO 讀回）共用，見下方接線處。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat dac_mode_ramp_concat_0
set_property -dict [list CONFIG.NUM_PORTS {12}] [get_bd_cells dac_mode_ramp_concat_0]

# 2026-07-15 新增：WO 0x35/0x36/0x37 讀回（Group 2：scale_cfg/amp_ctrl/
# calib_coef 即時值）
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_scale_cfg_ro
set_property -dict [list CONFIG.NUM_PORTS {2} \
    CONFIG.IN0_WIDTH {8} CONFIG.IN1_WIDTH {24}] [get_bd_cells concat_scale_cfg_ro]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_amp_ctrl_ro
set_property -dict [list CONFIG.NUM_PORTS {2} \
    CONFIG.IN0_WIDTH {18} CONFIG.IN1_WIDTH {14}] [get_bd_cells concat_amp_ctrl_ro]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_coef_ro
set_property -dict [list CONFIG.NUM_PORTS {2} \
    CONFIG.IN0_WIDTH {18} CONFIG.IN1_WIDTH {14}] [get_bd_cells concat_coef_ro]

create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_diag_nodes
set_property -dict [list CONFIG.NUM_PORTS {5} \
    CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {1} \
    CONFIG.IN3_WIDTH {1} CONFIG.IN4_WIDTH {28}] [get_bd_cells concat_diag_nodes]

# 2026-07-05 (step 15a): 復原 TX relay path（dispatcher -> async_fifo_aurora_tx
# -> Layer 2 本機發起輸入，取代原本接 aurora_tx_arbiter_0 的做法）
# async_fifo_aurora_tx din packing: {tx_tlast[0], tx_tdata[63:0]} -> 65-bit
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_tx_fifo_din
set_property -dict [list CONFIG.NUM_PORTS {2} \
    CONFIG.IN0_WIDTH {64} CONFIG.IN1_WIDTH {1}] [get_bd_cells concat_tx_fifo_din]

# async_fifo_aurora_tx dout unpacking
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice slice_tx_tdata
set_property -dict [list CONFIG.DIN_WIDTH {65} CONFIG.DIN_FROM {63} CONFIG.DIN_TO {0}] \
    [get_bd_cells slice_tx_tdata]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice slice_tx_tlast
set_property -dict [list CONFIG.DIN_WIDTH {65} CONFIG.DIN_FROM {64} CONFIG.DIN_TO {64}] \
    [get_bd_cells slice_tx_tlast]

# async_fifo_reply_tx din packing/dout unpacking：跟上面 async_fifo_
# aurora_tx 完全同一種 65-bit={tlast,tdata} pattern
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_reply_fifo_din
set_property -dict [list CONFIG.NUM_PORTS {2} \
    CONFIG.IN0_WIDTH {64} CONFIG.IN1_WIDTH {1}] [get_bd_cells concat_reply_fifo_din]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice slice_reply_tdata
set_property -dict [list CONFIG.DIN_WIDTH {65} CONFIG.DIN_FROM {63} CONFIG.DIN_TO {0}] \
    [get_bd_cells slice_reply_tdata]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice slice_reply_tlast
set_property -dict [list CONFIG.DIN_WIDTH {65} CONFIG.DIN_FROM {64} CONFIG.DIN_TO {64}] \
    [get_bd_cells slice_reply_tlast]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic inv_reply_fifo_full
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {not}] [get_bd_cells inv_reply_fifo_full]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic and_replyfifo_wr_en
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and}] [get_bd_cells and_replyfifo_wr_en]

# ddr_writer_0/busy (1-bit) + ddr_zero_writer_0/busy (1-bit，2026-07-14
# 新增，「初始化」DDR4 清空進行中） -> 32-bit WO
# (In0=ddr_writer busy, In1=ddr_zero_writer busy, In2=const_30b0)
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_ddr_busy
set_property -dict [list CONFIG.NUM_PORTS {3} \
    CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {30}] [get_bd_cells concat_ddr_busy]

# -- LED concat (6-bit: [0]=mmcm_locked [1]=au0_up [2]=au1_up [3]=ext_clk_sel
#    [4]=int_clk_sel [5]=awg_init_done_all) --
# 2026-07-13：加 led[3]=clk_wiz_ext_0/locked，觀察 ext clock 這顆新 MMCM 能不能穩定鎖定。
# 2026-07-14：改版——led[3] 改成直接反映 dac_clk_mux 選擇狀態（wi_ext_clk_sel/
# Dout，1=目前選 ext），不再是 ext MMCM 是否鎖定（會漏掉「選了 ext 但根本沒
# 選中/沒接好」這種狀況）；新增 led[4]=NOT(led[3])（1=目前選 internal）、
# led[5]=4 個 channel 的 zmod_awg_$ch/sInitDoneDAC 全部 AND 在一起（4 個都
# success 才亮）。動機：board B 播放沒有波形、多板播放也沒有同相位，肉眼看不出
# 到底是「選錯 clock 源」還是「clock 根本沒進來」，加這幾個 LED 才能現場排查。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat led_concat
set_property -dict [list CONFIG.NUM_PORTS {6} \
    CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {1} CONFIG.IN3_WIDTH {1} \
    CONFIG.IN4_WIDTH {1} CONFIG.IN5_WIDTH {1}] \
    [get_bd_cells led_concat]

create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic led_int_sel_not
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {not}] [get_bd_cells led_int_sel_not]

# 2026-07-14 改版（使用者事後修正需求）：led[3]/led[4] 不能只看選擇位元
# 的正反，要「選中 且 該時脈源真的 locked」才亮，不然選了 ext 但接線
# 有問題/沒鎖定，led[3] 還是會亮，誤導判斷。
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic led_ext_sel_and_locked
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and}] [get_bd_cells led_ext_sel_and_locked]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic led_int_sel_and_locked
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and}] [get_bd_cells led_int_sel_and_locked]

create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic led_awg_init_and01
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and}] [get_bd_cells led_awg_init_and01]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic led_awg_init_and012
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and}] [get_bd_cells led_awg_init_and012]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic led_awg_init_and0123
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and}] [get_bd_cells led_awg_init_and0123]

# ==============================================================================
#  Step 14.3b playback path: DDR4 -> AWG cells (ported from awg-test-step-14)
#  Aurora / flash / ext-clock EXCLUDED. See report for deviations.
# ==============================================================================

# -- DAC clock muxes (BUFGMUX_CTRL). Deviation: no ext clock in 14.3b, so I1 is
#    tied to the SAME clk_wiz_0 tap as I0 and S=const_zero (always select I0). This
#    keeps the mux present so a future ext clock is a pure I1-net swap. --------------
create_bd_cell -type module -reference dac_clk_mux dac_clk_mux_0
create_bd_cell -type module -reference dac_clk_mux dac_90_clk_mux_0

# -- 8x ddr4_stream_reader + 8x dc_fifo_xpm(xpm_fifo_async) + 4x waveform_controller --
# 2026-07-04: shared power-on reset for all 8 dc_fifo_xpm instances (OR of the
# existing ui_clk-domain sync_rst + sys_clk-domain peripheral_reset -- both were
# previously wired separately to simple_dc_fifo's wr_rst/rd_rst; xpm_fifo_async only
# has one `rst` input, its own xpm_fifo_rst submodule handles both-domain sync
# internally regardless of which domain the assertion originates in).
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_fifo_poweron_rst
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_fifo_poweron_rst]

foreach ch {0 1 2 3} {
    create_bd_cell -type module -reference ddr4_stream_reader reader_a_$ch
    create_bd_cell -type module -reference ddr4_stream_reader reader_b_$ch
    create_bd_cell -type module -reference dc_fifo_xpm fifo_a_$ch
    set_property -dict [list CONFIG.DEPTH {1024} CONFIG.PROG_FULL_THRESH {768}] [get_bd_cells fifo_a_$ch]
    create_bd_cell -type module -reference dc_fifo_xpm fifo_b_$ch
    set_property -dict [list CONFIG.DEPTH {1024} CONFIG.PROG_FULL_THRESH {768}] [get_bd_cells fifo_b_$ch]
    # per-buffer OR: shared power-on reset OR this buffer's own flush pulse from wctrl_$ch
    create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_fifo_a_rst_$ch
    set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_fifo_a_rst_$ch]
    create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_fifo_b_rst_$ch
    set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_fifo_b_rst_$ch]
    create_bd_cell -type module -reference waveform_controller wctrl_$ch
    set_property CONFIG.CH_ID $ch [get_bd_cells wctrl_$ch]
}

# -- Sine wave generator feature (added 2026-07-17, rewritten 2026-07-23 for
# 8-channel independent output -- see NOTES.md 2026-07-23 sine_gen 8-channel
# design section) ------------------------------------------------------------
# Each ZmodAWG module physically has 2 channels (ch1/ch2). sine_ctrl_regs_0
# now covers 8 physical channels (inst 0-3 x sub 1-2), each with a TRUE
# active/idle hardware pair (suffix _a/_b) for both sine_gen (frequency/phase)
# and amp_ramp_gen (amplitude ramp) -- 16 instances each, 32 total. Host
# writes always land on whichever instance is currently idle; trig_start
# (shared across all 8 channels) swaps active<->idle. dac_output_mux stays
# one per ZmodAWG module (4 total, unchanged -- DDR/sine mode selection is
# still per-module, not per-physical-channel, confirmed with user). amp_ctrl_
# mux becomes one per physical channel (8 total, i=0-7) to match the existing
# amp_ctrl_0..7 numbering in aurora_ctrl_mux.v (i = 2*inst + (sub==2 ? 1 : 0),
# same module-major encoding as sine_ctrl_regs_0's ch_sel -- confirmed to
# already be the codebase's existing convention, see the amp_ctrl_$i loop
# further down this file).
create_bd_cell -type module -reference sine_ctrl_regs sine_ctrl_regs_0
foreach inst {0 1 2 3} {
    foreach sub {1 2} {
        foreach ab {a b} {
            create_bd_cell -type module -reference sine_gen sine_gen_${inst}_${sub}_${ab}
            set_property CONFIG.LUT_FILE {C:/path/to/awg-test-step-16/rtl/sine_lut_16384.mem} [get_bd_cells sine_gen_${inst}_${sub}_${ab}]
            create_bd_cell -type module -reference amp_ramp_gen amp_ramp_gen_${inst}_${sub}_${ab}
        }
    }
}
foreach ch {0 1 2 3} {
    create_bd_cell -type module -reference dac_output_mux dac_output_mux_$ch
}
foreach i {0 1 2 3 4 5 6 7} {
    create_bd_cell -type module -reference amp_ctrl_mux amp_ctrl_mux_$i
}
# 2026-07-27 新增（統一讀取/寫入架構，QT_SINE_STATUS 查詢用）：純組合
# 邏輯，dac_clk domain，依 sine_ctrl_regs_0/mux_sel_0..7（dac_clk 原始
# 版）選出每個 physical channel 目前 active 的 sine_gen phase_acc_out。
create_bd_cell -type module -reference sine_phase_acc_mux sine_phase_acc_mux_0
# 2026-07-27 新增（統一讀取/寫入架構）：依 query_type 組裝 T_STATUS_
# REPORT 回覆，見 rtl/aurora_reply_tx.v 檔頭說明、PORTS.md「統一讀取/
# 寫入架構」章節。
create_bd_cell -type module -reference aurora_reply_tx aurora_reply_tx_0
# 2026-07-27 新增：BTPipeOut 0xA1（PO_STATUS_REPLY），host 讀查詢回覆用
create_bd_cell -type module -reference status_reply_capture status_reply_capture_0

create_bd_cell -type module -reference sync_start sync_start_0
foreach port {0 1 2 3} {
    create_bd_cell -type module -reference trig_timer trig_timer_$port
}
# 2026-08-04 新增（trigger group 排程功能三部曲「C」）：每板一個，不是
# 每 module 一個，見 rtl/group_trig_scheduler.v 檔頭說明
create_bd_cell -type module -reference group_trig_scheduler group_trig_scheduler_0
create_bd_cell -type module -reference syzygy_ready       syzygy_ready_0
create_bd_cell -type module -reference aurora_ctrl_mux    aurora_ctrl_mux_0
create_bd_cell -type module -reference calib_mux          calib_mux_0
create_bd_cell -type module -reference awg_calib_regs     awg_calib_regs_0
# 2026-07-15 新增（Group 2：scale_cfg/amp_ctrl/calib_coef 讀回 + flash 分開儲存）
create_bd_cell -type module -reference amp_ctrl_read_mux   amp_ctrl_read_mux_0
create_bd_cell -type module -reference flash_target_sel_reg flash_target_sel_reg_0
create_bd_cell -type module -reference trigger_cdc         flash_scale_load_valid_cdc_0
create_bd_cell -type module -reference trigger_cdc         flash_amp_load_valid_cdc_0
create_bd_cell -type module -reference trigger_cdc         flash_coef_load_valid_cdc_0

# -- 4x ZmodAWGController (Digilent catalog IP) -----------------------------------
foreach ch {0 1 2 3} {
    create_bd_cell -type ip -vlnv digilent.com:user:ZmodAWGController:1.1 zmod_awg_$ch
    set_property -dict [list \
        CONFIG.kExtCalibEn        {true}  \
        CONFIG.kExtScaleConfigEn  {true}  \
        CONFIG.kExtCmdInterfaceEn {false} \
    ] [get_bd_cells zmod_awg_$ch]
}

# -- trigger_cdc instances (sys_clk -> dac_clk) -----------------------------------
# 2026-07-14：reinit_pulse_cdc_0 給「初始化」指令用，單一顆 CDC 輸出直接
# fan-out 給全部 4 個 wctrl_$ch（同一個 dac_clk domain，比照 global_trig_cdc
# 的既有 fan-out 模式）
create_bd_cell -type module -reference trigger_cdc reinit_pulse_cdc_0
# 2026-07-24 移除（trigger 統一化架構改版）：global_trig_cdc/timer_cdc_
# 0-3/au_trig_port_cdc_0-3/trig_ext_cdc_0/fp_port_cdc_1-3——本機 TI
# global trigger、T_TRIG_PORT、FP port trigger 三個機制整個拿掉，
# timer_cdc_$port 也不再需要（trig_timer 本身直接搬進 dac_clk domain，
# trigger_out 已經是 dac_clk 原生訊號，不用再 CDC 一次），詳見
# PROJECT.md「Trigger 統一化架構改版」小節。
# 2026-07-23 新增，2026-07-30 整個改版：au_trig_delay 全自動校準機制
# trigger 準確度改版——原本這裡是 native_trig_cdc_0（trigger_cdc，
# aurora_ctrl_channel_0 在 aurora_clk domain 本地補償倒數完，最終
# 觸發脈衝直接跨到 dac_clk）。2026-07-30 發現這個設計有兩個問題：
# ①短間隔連續 trigger 時，aurora_clk 那邊的補償倒數暫存器沒有任何
# 保護，新事件會直接蓋掉還沒 fire 的舊倒數；②倒數本身用 aurora_clk
# （每片板子各自獨立的板上振盪器，板跟板之間沒有共用參考）計時，
# 理論上有 ppm 等級跨板誤差。改法：aurora_ctrl_channel_0 不再自己
# 倒數，只送出 trig_fire_req/trig_fire_group（決定要 fire 的事件+
# group_select），經 trig_fire_fifo_0（官方 fifo_generator，depth 16，
# 取代單一 pulse CDC，短間隔連續事件不會互相蓋掉）安全跨到 dac_clk，
# 真正的倒數搬到 dac_trig_queue_0（新模組，dac_clk domain，含深度 8
# 的 fire-timestamp 佇列），詳見 rtl/dac_trig_queue.v 檔頭說明、
# NOTES.md 2026-07-30「sine wave mode 精確度討論」章節。
create_bd_cell -type ip -vlnv xilinx.com:ip:fifo_generator trig_fire_fifo_0
set_property -dict [list \
    CONFIG.Fifo_Implementation          {Independent_Clocks_Block_RAM} \
    CONFIG.Input_Data_Width             {4}   \
    CONFIG.Input_Depth                  {16}  \
    CONFIG.Output_Data_Width            {4}   \
    CONFIG.Performance_Options          {First_Word_Fall_Through} \
    CONFIG.Valid_Flag                   {true} \
] [get_bd_cells trig_fire_fifo_0]

# 補償延遲量（native_hops*per_hop_value_eff，aurora_clk cycle 單位，
# quasi-static，只有 enum/T_BOARD_ID_ASSIGN 之後才會變）跨到 dac_clk，
# 沿用既有 level_cdc 慣例（跟 manual_per_hop_cdc_0 等同一種準穩態值
# CDC 模式一致）。
create_bd_cell -type module -reference level_cdc native_trig_delay_cdc_0
set_property -dict [list CONFIG.WIDTH {37}] [get_bd_cells native_trig_delay_cdc_0]

# dac_trig_queue_0：dac_clk domain，×16÷25 頻率換算（aurora_clk=
# 156.25MHz -> dac_clk=100MHz 標稱比例）+ 深度 8 的 fire-timestamp
# 佇列，見 rtl/dac_trig_queue.v。四個 channel 共用同一份（trigger
# 對所有 channel 是共用的，不需要每個 channel 各自一份）。
create_bd_cell -type module -reference dac_trig_queue dac_trig_queue_0

# 2026-07-24 新增（trigger 統一化架構改版）：trig_timer_$port 搬到
# dac_clk domain 後，原本直接從 sys_clk 接過去的 list_wr_slot/
# list_wr_intv/list_depth/run/loop_en/list_wr_en 全部變成真正跨時脈域，
# 每個訊號各自一顆 CDC（不打包進單一多 bit level_cdc，避免還要另外用
# xlslice 拆解——這台機器沒有 Vivado 可以實際 source 驗證，分開接線
# 比較不容易出錯）。run/loop_en/list_depth 是準穩態 level（trig_timer.v
# 每個 cycle 持續讀取，不是靠 enable pulse 鎖存），用 level_cdc；
# list_wr_slot/list_wr_intv 雖然也是多 bit 值，但只有 list_wr_en pulse
# 那一拍才會被真正採用寫進 mem[]——這個「level_cdc 值 + 獨立 pulse_cdc
# 觸發」的組合模式，跟既有 reserve_dest_id_cdc_0（level_cdc）+
# au_reserve_pulse_cdc_0（trigger_cdc）餵給 aurora_ctrl_channel_0 的
# 既有、已驗證可用的模式完全一樣，沿用同一套慣例。
# 2026-08-04：slot 3→4-bit、depth 3→5-bit（16 slot 排程功能，見
# rtl/trig_timer.v 同日 header comment 的完整推導 -- depth 4-bit 仍不足
# 以表示 16，會重演 mem[] 最後一格死格的同一種 bug）。
foreach port {0 1 2 3} {
    create_bd_cell -type module -reference level_cdc timer_slot_cdc_$port
    set_property -dict [list CONFIG.WIDTH {4}] [get_bd_cells timer_slot_cdc_$port]
    create_bd_cell -type module -reference level_cdc timer_intv_cdc_$port
    set_property -dict [list CONFIG.WIDTH {32}] [get_bd_cells timer_intv_cdc_$port]
    create_bd_cell -type module -reference level_cdc timer_depth_cdc_$port
    set_property -dict [list CONFIG.WIDTH {5}] [get_bd_cells timer_depth_cdc_$port]
    create_bd_cell -type module -reference level_cdc timer_run_cdc_$port
    create_bd_cell -type module -reference level_cdc timer_loop_cdc_$port
    create_bd_cell -type module -reference trigger_cdc timer_wr_cdc_$port
}

# 2026-08-04 新增（trigger group 排程功能三部曲「C」）：
# group_trig_scheduler_0（每板一個，dac_clk domain）的控制路徑 CDC，
# sys_clk（local_reg_handler_0 的 au_group_sched_* 解碼輸出）->
# dac_clk，跟 timer_slot/intv/depth/run/loop/wr_cdc_$port 同一套模式
# （準穩態 level 用 level_cdc、list_wr_en pulse 用 trigger_cdc）。這個
# 模組沒有 FP WireIn 直寫路徑，只有這一組 CDC，不需要额外 OR gate。
create_bd_cell -type module -reference level_cdc gsc_slot_cdc_0
set_property -dict [list CONFIG.WIDTH {4}] [get_bd_cells gsc_slot_cdc_0]
create_bd_cell -type module -reference level_cdc gsc_intv_cdc_0
set_property -dict [list CONFIG.WIDTH {32}] [get_bd_cells gsc_intv_cdc_0]
create_bd_cell -type module -reference level_cdc gsc_group_cdc_0
set_property -dict [list CONFIG.WIDTH {4}] [get_bd_cells gsc_group_cdc_0]
create_bd_cell -type module -reference level_cdc gsc_depth_cdc_0
set_property -dict [list CONFIG.WIDTH {5}] [get_bd_cells gsc_depth_cdc_0]
create_bd_cell -type module -reference level_cdc gsc_run_cdc_0
create_bd_cell -type module -reference level_cdc gsc_loop_cdc_0
create_bd_cell -type module -reference trigger_cdc gsc_wr_cdc_0
# 2026-08-20 新增（多板同步輪播 Architecture B）：arm_mode 是跟 run/
# loop_en 同一類 quasi-static level 訊號，同一套 level_cdc 模式。
create_bd_cell -type module -reference level_cdc gsc_arm_mode_cdc_0

# 2026-08 新增（Sine mode N-slot 機制）：cell 建立要放在這裡（跟其他
# CDC 一起，早於下方 src_clk/dst_clk fan-out 的 connect_bd_net），不能
#放在後面實際接線（au_sine_* -> cdc -> sine_ctrl_regs_0）那個段落——
# 那個段落在檔案後段，晚於 fan-out 執行，會導致 fan-out 那批
# connect_bd_net 找不到這些 cell 的 pin（第一版犯過這個錯，dry-run
# 直接報 "clock pins are not connected to a valid clock source"）。
create_bd_cell -type module -reference level_cdc sine_sel_cdc_0
set_property -dict [list CONFIG.WIDTH {6}] [get_bd_cells sine_sel_cdc_0]
create_bd_cell -type module -reference level_cdc sine_data_cdc_0
set_property -dict [list CONFIG.WIDTH {32}] [get_bd_cells sine_data_cdc_0]
create_bd_cell -type module -reference trigger_cdc sine_wr_cdc_0

create_bd_cell -type module -reference level_cdc sine_list_sel_cdc_0
set_property -dict [list CONFIG.WIDTH {8}] [get_bd_cells sine_list_sel_cdc_0]
create_bd_cell -type module -reference level_cdc sine_list_data_cdc_0
set_property -dict [list CONFIG.WIDTH {32}] [get_bd_cells sine_list_data_cdc_0]
create_bd_cell -type module -reference trigger_cdc sine_list_wr_cdc_0

# 2026-08-10 新增：T_LIST_WRITE（DDR4 波形播放 list_sel/list_addr/
# list_len）sys_clk -> dac_clk 補 CDC，根因/修法規劃見 NOTES.md
# 2026-08-10「Bulk DDR 階梯測試圖案...根因鎖定 T_LIST_WRITE 缺 CDC」
# 章節。跟上面 timer_slot/intv/depth_cdc_$port + timer_wr_cdc_$port
# 同一套已驗證模式：多 bit 準穩態值（list_sel 經 list_sel_slice 縮到
# 5-bit、list_addr/list_len 直接 32-bit）用 level_cdc，enable pulse
# 用 trigger_cdc，靠 pulse CDC 天生比 level CDC 慢的既有時序假設確保
# 順序。**enable pulse 改接新增的 aurora_ctrl_mux_0/out_list_wr，不是
# 舊的 ti_list_wr/Dout**——第一版直接用 ti_list_wr（跟 au_list_wr 同一
# 拍送出）餵 CDC，被新增的 sim/tb_list_write_cdc.v 抓到一個 race：
# list_sel/addr/len_hold 是 non-blocking assignment 寫的暫存器，
# au_list_wr 那一拍讀到的還是舊值，新值要下一拍才穩定，pulse 送太早
# 會讓 CDC 抓到/送出舊資料。`out_list_wr` 已經把 Aurora 這一路延遲 1
# 拍對齊 hold 暫存器更新時機（FP 直寫那一路不用延遲，WI 暫存器早就
# 穩定），見 rtl/aurora_ctrl_mux.v `list_au_wr_dly` 宣告處完整說明。
# 原本的 `ti_list_wr` xlslice 已改用不到，整個移除（`out_ti_cmd` 保留
# bit2 的 Aurora 貢獻不影響正確性，純粹是沒人再讀的死值）。
# list_addr/list_len/list_wr_en 是全部 4 個 wctrl_$ch 共用同一份廣播值
# （由 wctrl 內部 list_wr_sel[1:0]==CH_ID 決定哪個 channel 真正採用），
# 所以只需要一組 CDC，不用比照 timer 那樣每個 port 各自一份。
create_bd_cell -type module -reference level_cdc list_sel_cdc_0
set_property -dict [list CONFIG.WIDTH {5}] [get_bd_cells list_sel_cdc_0]
create_bd_cell -type module -reference level_cdc list_addr_cdc_0
set_property -dict [list CONFIG.WIDTH {32}] [get_bd_cells list_addr_cdc_0]
create_bd_cell -type module -reference level_cdc list_len_cdc_0
set_property -dict [list CONFIG.WIDTH {32}] [get_bd_cells list_len_cdc_0]
create_bd_cell -type module -reference trigger_cdc list_wr_cdc_0

# 輸出側：group_trig_scheduler_0/fire+group_select_out（dac_clk）->
# sys_clk，才能跟 local_reg_handler_0/au_trig_start/au_trig_group_
# select OR 在一起（見下方 or_group_sched_fire_0/or_group_sched_
# group_0 建立處）。
#
# ⚠️ 2026-08-04 CDC 修法（Windows 端上機測試發現＋Opus 獨立覆核確認）：
# 原本這裡用 gsc_fire_cdc_0（trigger_cdc）+ gsc_fire_group_cdc_0
# （level_cdc）分開跨 fire/group_select_out 兩個訊號——group_select_out
# 只維持 1 個 dac_clk 週期（配合 fire 才有效，其餘時間清零），這是
# 「瞬態多 bit 資料」，不是 level_cdc 設計來處理的「準穩態、host 端
# sleep 保證 settling time」那種訊號，2-flop 同步器沒有機制保證正確
# 捕捉窄脈衝，可能讓 group_select_out 實質上一直傳到 0。這個專案自己
# 在 2026-07-30 就修過幾乎一樣的問題（aurora_ctrl_channel_0/
# trig_fire_group，見 trig_fire_fifo_0 建立處註解），正確做法是不用
# level_cdc，改成跟 pulse 一起塞進同一筆 FIFO entry——這次照抄同一個
# 已驗證模式：gsc_fire_fifo_0（fifo_generator，config 完全比照
# trig_fire_fifo_0）+ fifo_event_reader_0（新模組，把 FWFT 讀出端轉成
# 下游 OR gate 預期的 pulse+data 介面，見 rtl/fifo_event_reader.v
# 檔頭說明）。
create_bd_cell -type ip -vlnv xilinx.com:ip:fifo_generator gsc_fire_fifo_0
set_property -dict [list \
    CONFIG.Fifo_Implementation          {Independent_Clocks_Block_RAM} \
    CONFIG.Input_Data_Width             {4}   \
    CONFIG.Input_Depth                  {16}  \
    CONFIG.Output_Data_Width            {4}   \
    CONFIG.Performance_Options          {First_Word_Fall_Through} \
    CONFIG.Valid_Flag                   {true} \
] [get_bd_cells gsc_fire_fifo_0]
create_bd_cell -type module -reference fifo_event_reader fifo_event_reader_0
set_property -dict [list CONFIG.WIDTH {4}] [get_bd_cells fifo_event_reader_0]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_group_sched_fire_0
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_group_sched_fire_0]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_group_sched_group_0
set_property -dict [list CONFIG.C_SIZE {4} CONFIG.C_OPERATION {or}] [get_bd_cells or_group_sched_group_0]

# 2026-07-24 新增（trigger 統一化架構改版）：T_TRIG_DELAY_CFG(0x1D) 手動
# 覆寫路徑，local_reg_handler_0（sys_clk）-> aurora_ctrl_channel_0
# （aurora_clk），見 rtl/aurora_ctrl_channel.v manual_trig_delay_in/
# manual_trig_delay_active_in port 註解、rtl/local_reg_handler.v
# manual_delay_override_active port 註解。
create_bd_cell -type module -reference level_cdc manual_trig_delay_cdc_0
set_property -dict [list CONFIG.WIDTH {16}] [get_bd_cells manual_trig_delay_cdc_0]
create_bd_cell -type module -reference level_cdc manual_trig_delay_active_cdc_0
# 2026-07-24 新增：單板 bench test 用（T_MANUAL_TOTAL_BOARDS=0x2B），
# local_reg_handler_0（sys_clk）-> aurora_ctrl_channel_0（aurora_clk），
# 見 rtl/aurora_ctrl_channel.v manual_total_boards_in port 註解。
create_bd_cell -type module -reference level_cdc manual_total_boards_cdc_0
set_property -dict [list CONFIG.WIDTH {5}] [get_bd_cells manual_total_boards_cdc_0]
# 2026-07-24 再追加：per_hop_value 手動覆寫（au_trig_delay 自動校準機制
# 除錯用），board_cfg_reg_0（sys_clk）-> aurora_ctrl_channel_0
# （aurora_clk），見 rtl/aurora_ctrl_channel.v manual_per_hop_in/
# manual_per_hop_active_in port 註解、rtl/board_cfg_reg.v
# manual_per_hop_active port 註解。
create_bd_cell -type module -reference level_cdc manual_per_hop_cdc_0
set_property -dict [list CONFIG.WIDTH {32}] [get_bd_cells manual_per_hop_cdc_0]
create_bd_cell -type module -reference level_cdc manual_per_hop_active_cdc_0
# 2026-07-27 新增（Group-based Trigger 架構）：T_TRIG_START(0x1B) beat1
# 的 4-bit group_select，local_reg_handler_0（sys_clk）-> aurora_ctrl_
# channel_0（aurora_clk），見 rtl/aurora_ctrl_channel.v au_trig_group_
# select_in port 註解。**2026-07-30 起**：aurora_ctrl_channel_0/trig_
# fire_group（伴隨 trig_fire_req 的 group_select）不再另外走 level_cdc
# ——改成跟 trig_fire_req 一起進 trig_fire_fifo_0（同一筆 FIFO entry，
# 見上方 trig_fire_fifo_0 建立處註解），短間隔連續事件才不會讓 group
# 值互相蓋掉。
create_bd_cell -type module -reference level_cdc au_trig_group_cdc_0
set_property -dict [list CONFIG.WIDTH {4}] [get_bd_cells au_trig_group_cdc_0]
# 2026-07-08：trigger_cdc.v 新增 src_rst/dst_rst port（見 rtl/trigger_cdc.v
# header 說明），這 14 個 sys_clk->dac_clk 的單板內觸發分配 CDC 跟這次
# reserve bug 無關、已穩定運作，明確接 1'b0，效果跟改動前完全相同，不冒
# 額外風險。真正需要 reset 的是下面 4 個 Aurora 協定相關的 CDC。
connect_bd_net [get_bd_pins const_zero/dout] \
    [get_bd_pins reinit_pulse_cdc_0/src_rst]  [get_bd_pins reinit_pulse_cdc_0/dst_rst]  \
    [get_bd_pins timer_wr_cdc_0/src_rst]      [get_bd_pins timer_wr_cdc_0/dst_rst]      \
    [get_bd_pins timer_wr_cdc_1/src_rst]      [get_bd_pins timer_wr_cdc_1/dst_rst]      \
    [get_bd_pins timer_wr_cdc_2/src_rst]      [get_bd_pins timer_wr_cdc_2/dst_rst]      \
    [get_bd_pins timer_wr_cdc_3/src_rst]      [get_bd_pins timer_wr_cdc_3/dst_rst]      \
    [get_bd_pins gsc_wr_cdc_0/src_rst]        [get_bd_pins gsc_wr_cdc_0/dst_rst]        \
    [get_bd_pins sine_wr_cdc_0/src_rst]       [get_bd_pins sine_wr_cdc_0/dst_rst]       \
    [get_bd_pins sine_list_wr_cdc_0/src_rst]  [get_bd_pins sine_list_wr_cdc_0/dst_rst]  \
    [get_bd_pins list_wr_cdc_0/src_rst]       [get_bd_pins list_wr_cdc_0/dst_rst]
# gsc_fire_cdc_0/gsc_fire_group_cdc_0（原本接在這裡）2026-08-04 已移除
# ——group_select_out 這條瞬態訊號改用 gsc_fire_fifo_0，reset 沿用既有
# rst_clk100/peripheral_reset（使用者確認不新建 dac_clk 專屬 reset
# generator，見下方 gsc_fire_fifo_0/rst 接線處）。
# level_cdc（timer_slot/intv/depth/run/loop_cdc_$port、manual_trig_delay_
# cdc_0、manual_trig_delay_active_cdc_0）沒有 src_rst/dst_rst port（見
# rtl/level_cdc.v，只包 xpm_cdc_array_single，沒有 reset 輸入），不需要
# 接這裡。

# -- Aurora 多板協定 CDC (2026-07-06 bugfix)：host TI pulse -> aurora_clk ------
# 硬體實測發現 init_start_pulse/trig_start_pulse/reserve_start_pulse 原本直接
# 從 sys_clk (ti_auXXX/Dout) 接到 aurora_ctrl_channel_0（aurora_clk），完全
# 沒有 CDC，導致單拍 pulse 幾乎不可能被 aurora_clk 域正確採到（三塊實體板
# 測試：enum 完全沒有反應，board_index 全部停在 0）。用既有的 trigger_cdc
# 補上，跟其他 pulse CDC 完全同一套模式。
create_bd_cell -type module -reference trigger_cdc au_init_pulse_cdc_0
create_bd_cell -type module -reference trigger_cdc au_trig_pulse_cdc_0
create_bd_cell -type module -reference trigger_cdc au_reserve_pulse_cdc_0
# 2026-07-08：這三個是 host 觸發 enum/trigger/reserve 的 sys_clk->aurora_clk
# pulse CDC，接上真正的 reset（src_rst 用 sys_clk domain 的
# rst_clk100/peripheral_reset，dst_rst 用 aurora_clk domain、跟
# aurora_ctrl_channel_0 共用同一組的 or_aurora_extra_rst/Res）——啟用
# xpm_cdc_pulse 本來就有、但原本被停用的 reset 機制，讓每次開機/reload
# 都有明確、一致的起始狀態，不再只靠上電時的暫存器初值。
connect_bd_net [get_bd_pins rst_clk100/peripheral_reset] \
    [get_bd_pins au_init_pulse_cdc_0/src_rst]    \
    [get_bd_pins au_trig_pulse_cdc_0/src_rst]    \
    [get_bd_pins au_reserve_pulse_cdc_0/src_rst]
# dst_rst 要接 or_aurora_extra_rst/Res，但這個 cell 是下面 debug-only
# 額外 reset 實驗那段才建立的（第五輪加的），這裡先不能引用還不存在的
# pin——接線挪到 or_aurora_extra_rst 建立之後（見下方，2026-07-08 第
# 十輪修正：第一次 build 因為這個順序問題導致 impl_1 報 driverless net）。

# 2026-07-08 第四輪（debug-only）：ti_au_reserve/reserve_dest_id_slice 這條
# 平行除錯路徑，用 OR 跟封包路徑（au_reserve_start/au_reserve_dest_id）合併
# 後再進 CDC，見下方 src_pulse/src_in 接線處。
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_reserve_pulse
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_reserve_pulse]
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_reserve_dest_id
set_property -dict [list CONFIG.C_SIZE {16} CONFIG.C_OPERATION {or}] [get_bd_cells or_reserve_dest_id]

# 2026-07-08 第五輪（debug-only 實驗）：TI bit30 觸發的額外 reset pulse，
# 跟 au_reserve_pulse_cdc_0 同樣做法（sys_clk -> aurora_clk 的 pulse
# CDC），OR 進原本的 aurora_64b66b_0/sys_reset_out，測試「channel_up
# 穩定後手動補一次 reset」是否能解決 reserve 隨機失敗的問題。
create_bd_cell -type module -reference trigger_cdc au_ctrl_rst_pulse_cdc_0
# 2026-07-08：這是已放棄的 debug-only「額外補一次 reset」實驗路徑（證實
# 有害，見 PROJECT.md 第 11 節），不是正式系統行為的一部分，接 1'b0
# 保持跟其他 3 個 Aurora CDC 不同待遇——沒有必要給一個已知不用的除錯路徑
# 接真正的 reset，也避免它跟自己是 or_aurora_extra_rst/Res 的來源之一
# 形成不必要的自我參照接線。
connect_bd_net [get_bd_pins const_zero/dout] \
    [get_bd_pins au_ctrl_rst_pulse_cdc_0/src_rst] \
    [get_bd_pins au_ctrl_rst_pulse_cdc_0/dst_rst]

# 2026-07-22：ti_au_full_reset（TI bit7）觸發的兩個新 CDC pulse——
# au_full_reset_pulse_cdc_0（sys_clk -> aurora_clk，併入 or_aurora_
# extra_rst，見下方 or_aurora_extra_rst2）跟 au_pma_init_pulse_cdc_0
# （sys_clk -> init_clk，接 pma_init）。這兩個是正式功能（不是
# debug-only 實驗），src_rst/dst_rst 一樣接 const_zero（CDC 同步器
# 本身不需要額外重置，只是單純轉遞 pulse）。
create_bd_cell -type module -reference trigger_cdc au_full_reset_pulse_cdc_0
connect_bd_net [get_bd_pins const_zero/dout] \
    [get_bd_pins au_full_reset_pulse_cdc_0/src_rst] \
    [get_bd_pins au_full_reset_pulse_cdc_0/dst_rst]
connect_bd_net [get_bd_pins ti_au_full_reset/Dout] [get_bd_pins au_full_reset_pulse_cdc_0/src_pulse]

create_bd_cell -type module -reference trigger_cdc au_pma_init_pulse_cdc_0
connect_bd_net [get_bd_pins const_zero/dout] \
    [get_bd_pins au_pma_init_pulse_cdc_0/src_rst] \
    [get_bd_pins au_pma_init_pulse_cdc_0/dst_rst]
connect_bd_net [get_bd_pins ti_au_full_reset/Dout] [get_bd_pins au_pma_init_pulse_cdc_0/src_pulse]

create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_aurora_extra_rst
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_aurora_extra_rst]

# 2026-07-22：or_aurora_extra_rst2 把 or_aurora_extra_rst/Res（既有的
# sys_reset_out + ti_au_ctrl_rst debug-only 觸發）跟新的
# ti_au_full_reset（透過 au_full_reset_pulse_cdc_0，見上方建立處）
# OR 在一起——「整個 Aurora 回到乾淨狀態」這個新指令，共用既有這一整組
# 下游模組列表，不用重複列一次。建立位置刻意放在 or_aurora_extra_rst
# 建立之後、任何消費者接線之前，這樣兩處消費者（下面這個 + 更下面
# aurora_data_channel_0 等一大組）都能直接接 or_aurora_extra_rst2/Res。
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_aurora_extra_rst2
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_aurora_extra_rst2]
connect_bd_net [get_bd_pins or_aurora_extra_rst/Res] [get_bd_pins or_aurora_extra_rst2/Op1]
connect_bd_net [get_bd_pins au_full_reset_pulse_cdc_0/dst_pulse] [get_bd_pins or_aurora_extra_rst2/Op2]

# 2026-07-08 第十輪：au_init_pulse_cdc_0/au_trig_pulse_cdc_0/
# au_reserve_pulse_cdc_0 的 dst_rst 原本接這裡（or_aurora_extra_rst 現在才
# 建立，之前接線位置太早引用了還不存在的 pin，導致 impl_1 報
# driverless net，這裡是修正後的正確位置）。
# 2026-08-20 Phase 1（Opus 覆核發現的獨立 bug，見 PROJECT.md「Aurora reset
# 架構問題」章節）：改接 const_zero，不再接 or_aurora_extra_rst2/Res。
# 官方 XPM_CDC_PULSE 規範明確要求 src_rst/dest_rst 必須「同時」assert 才能
# 正確重置（否則 toggle 編碼的 src 端極性跟 dst 端不同步，reset 釋放瞬間
# 會被誤判成一次真實的 0->1 邊緣，憑空產生假 pulse）。這 4 個 CDC 的
# src_rst 接的是 rst_clk100/peripheral_reset（sys_clk 域，只在開機時
# assert 一次），但 dst_rst 原本接的 or_aurora_extra_rst2/Res 會跟著
# aurora_64b66b_0/sys_reset_out 在連線沒鎖定時反覆脈動——兩者從不同時
# assert，違反規範。2026-07-08 當時記錄過「開機/reload 後偵測到一個原因
# 不明的假 reserve_start_pulse」，被 is_master=0 條件擋掉才判定無害，現在
# 看很可能就是這個機制造成的。改成 const_zero 之後這 4 個 CDC 完全不做
# reset（回到 2026-07-08 之前的狀態，符合官方規範——pulse CDC 本身只是單純
# 轉遞 pulse，不需要額外重置）。
connect_bd_net [get_bd_pins const_zero/dout] \
    [get_bd_pins au_init_pulse_cdc_0/dst_rst]    \
    [get_bd_pins au_trig_pulse_cdc_0/dst_rst]    \
    [get_bd_pins au_reserve_pulse_cdc_0/dst_rst]

# 2026-07-08（debug-only）：監測 aurora_64b66b_0/1 的 hard_err（官方 PG074
# 確認 hard_err 發生時核心會自動 reset+reinit，懷疑這個自動恢復過程偶爾
# 跟我們自己的 FSM 初始化時機沒對齊，導致 reserve 隨機失敗）。hard_err
# 本身的暫存器 clock 是 tx_out_clk（GT 原始輸出時脈，經查證跟 aurora_clk
# 不同 domain，見 PROJECT.md 說明），需要正規 CDC 才能安全接。
# level_cdc（src_clk/dst_clk 都接 aurora_clk——tx_out_clk 是 GT dedicated
# clock 繞線資源沒辦法直接扇出到一般邏輯，會導致 route_design unroutable
# pin 錯誤，改用已經緩衝過、頻率相同的 aurora_clk，見下方接線處的說明）
# + sticky_latch（一旦發生過就永遠拉高，直到這五個模組共用的 reset
# group 被觸發）。
create_bd_cell -type module -reference level_cdc hard_err_cdc_0
create_bd_cell -type module -reference level_cdc hard_err_cdc_1
create_bd_cell -type module -reference sticky_latch hard_err_sticky_0
create_bd_cell -type module -reference sticky_latch hard_err_sticky_1

# 2026-07-08（debug-only，使用者要求盡量多加 reserve 相關診斷資訊）：
# aurora_ctrl_channel_0 的 diag_reserve_fwd_pending（既有 port，之前只接
# ILA）/ diag_reserve_timeout_hit / diag_reserve_reply_hit（新增，見
# rtl/aurora_ctrl_channel.v）三個訊號都在 aurora_clk domain、都是可能
# 一閃即逝的訊號，各自用 sticky_latch 閂鎖住（reset 用同一組
# or_aurora_extra_rst，隨每次開機自然歸零），再各自用 level_cdc 跨到
# sys_clk 餵 WO。三者搭配既有的 reserve_ok/reserve_busy（WO 0x28），
# 能分辨「從沒送出 REQ」/「送出但逾時沒回覆」/「有收到明確回覆（不論
# 通過或忙碌）」三種不同情況。
create_bd_cell -type module -reference sticky_latch reserve_fwd_sticky_0
create_bd_cell -type module -reference sticky_latch reserve_timeout_sticky_0
create_bd_cell -type module -reference sticky_latch reserve_reply_sticky_0
create_bd_cell -type module -reference level_cdc reserve_fwd_cdc_0
create_bd_cell -type module -reference level_cdc reserve_timeout_cdc_0
create_bd_cell -type module -reference level_cdc reserve_reply_cdc_0
# 2026-07-08 追加：fwd_pending/reply_hit/timeout_hit 三個都要求
# reserve_wait 先為 1，但上機測試發現 fwd_sent 在還沒送任何 reserve
# 指令前就已經是 1（fwd_pending 是 enum/trigger/reserve 共用訊號，懷疑
# 開機當下就被別的東西污染過），沒辦法單獨確認 reserve_wait 真的有沒有
# 被設過。直接 sticky-latch `reserve_start_pulse` 本身（100% reserve
# 專用，不跟其他協定共用），才能明確回答「這個 pulse 到底有沒有真的
# 到達 aurora_ctrl_channel_0」。
create_bd_cell -type module -reference sticky_latch reserve_pulse_sticky_0
create_bd_cell -type module -reference level_cdc reserve_pulse_cdc_0
# 2026-07-29 新增：T_QUERY 上機持續失敗，需要把 aurora_tx1_arbiter_0
# 的 reply 寫入端有沒有真的動作也弄成 USB 可讀（不用 ILA）。sticky
# latch 邏輯已經直接做在 aurora_tx1_arbiter.v 內部（dbg_reply_seen/
# dbg_reply_granted，本來就是 aurora_clk domain sticky），這裡只需要
# level_cdc 跨到 sys_clk，不需要額外的 sticky_latch cell。
create_bd_cell -type module -reference level_cdc reply_seen_cdc_0
create_bd_cell -type module -reference level_cdc reply_granted_cdc_0
# 2026-07-08 再追加：reserve_pulse_sticky_0 只能看到「到達 aurora_clk domain
# 之後」有沒有脈波，但實測抓到一種失敗（round 5）：is_master 已確認是 1、
# host 也真的送出封包，`fwd_sent` 卻整輪維持 0——這代表問題可能出在
# `au_reserve_start`（local_reg_handler_0 解碼輸出，還在 sys_clk domain，
# CDC 之前）本身沒有觸發，也可能是 CDC 把它弄丟了，光看 aurora_clk 側的
# reserve_pulse_sticky_0 沒辦法分辨。這裡直接 tap `au_reserve_start` 本身
# （sys_clk domain，跟 fp0/local_reg_handler_0 同一個 domain，不需要
# level_cdc），下次抓到類似失敗時能分辨「封包解碼沒觸發」vs「CDC 弄丟了」。
create_bd_cell -type module -reference sticky_latch au_reserve_start_sticky_0

# -- Aurora 多板協定 CDC (2026-07-06 bugfix)：準穩態多 bit 訊號 --------------
# 同一次稽核發現的其餘缺口：board_id/is_master/reserve_dest_id (sys_clk ->
# aurora_clk 設定值) 跟 init_ok/total_boards/board_index/reserve_ok/
# reserve_busy/channel_up (aurora_clk -> sys_clk 狀態值，含 WO 讀回跟 LED)
# 也全部是直接接線、沒有 CDC——aurora_ctrl_channel.v/aurora_data_channel.v
# 檔頭註解本來就假設這些訊號「外部已經用官方 XPM 巨集同步」，這次一併補齊。
# channel_up 只各做一次 CDC，同步後的輸出同時餵給 WO 讀回跟 LED，不用重複。
create_bd_cell -type module -reference level_cdc board_id_cdc_0
set_property -dict [list CONFIG.WIDTH {16}] [get_bd_cells board_id_cdc_0]
# 2026-07-15 新增：flash_target_sel_reg_0（sys_clk）-> fpga_flash_ctrl_0
# （okClk）跨域，準穩態 2-bit 值，比照 board_id_cdc_0 用官方 xpm_cdc_array_single
create_bd_cell -type module -reference level_cdc flash_target_sel_cdc_0
set_property -dict [list CONFIG.WIDTH {2}] [get_bd_cells flash_target_sel_cdc_0]
create_bd_cell -type module -reference level_cdc is_master_cdc_0
create_bd_cell -type module -reference level_cdc reserve_dest_id_cdc_0
set_property -dict [list CONFIG.WIDTH {16}] [get_bd_cells reserve_dest_id_cdc_0]
create_bd_cell -type module -reference level_cdc init_ok_cdc_0
create_bd_cell -type module -reference level_cdc total_boards_cdc_0
set_property -dict [list CONFIG.WIDTH {5}] [get_bd_cells total_boards_cdc_0]
# 2026-07-29：board_index_cdc_0 改用 handshake_level_cdc（xpm_cdc_
# handshake 包裝，見該檔案檔頭說明），board_id_cdc_0 維持不動當對照組
# ——這是 board_id_cdc_0 xpm_cdc_array_single 用法疑慮的對照實驗，見
# NOTES.md 2026-07-29「board_index_cdc_0 改用 xpm_cdc_handshake 對照
# 實驗」章節。port 介面跟 level_cdc 完全一致，接線不用改。
create_bd_cell -type module -reference handshake_level_cdc board_index_cdc_0
set_property -dict [list CONFIG.WIDTH {5}] [get_bd_cells board_index_cdc_0]
# 2026-07-23 新增：au_trig_delay 全自動校準機制（piggyback 在
# T_BOARD_ID_ASSIGN 上，見 rtl/dispatcher.v/local_reg_handler.v 檔頭
# 註解、NOTES.md「Aurora trigger 協定」章節）。aurora_ctrl_channel_0/
# per_hop_value（master-only，32-bit，逐次減法除法器算出）要跨到
# sys_clk 才能餵給 dispatcher_0，準穩態值，比照 total_boards_cdc_0 用
# 官方 xpm_cdc_array_single（level_cdc 包裝）。
create_bd_cell -type module -reference level_cdc per_hop_value_cdc_0
set_property -dict [list CONFIG.WIDTH {32}] [get_bd_cells per_hop_value_cdc_0]
# 2026-07-24 再追加：per_hop_value_eff（自動或手動，見 rtl/aurora_ctrl_
# channel.v per_hop_value_eff port 註解）讀回用，同樣是 aurora_clk ->
# sys_clk 方向，跟上面 per_hop_value_cdc_0 同一組。
create_bd_cell -type module -reference level_cdc per_hop_value_eff_cdc_0
set_property -dict [list CONFIG.WIDTH {32}] [get_bd_cells per_hop_value_eff_cdc_0]
# 2026-07-23 再追加：trigger 準確度改版，local_reg_handler_0 收到
# T_BOARD_ID_ASSIGN beat1 後把原始值（未經 hops 計算）CDC 回
# aurora_clk 給 aurora_ctrl_channel_0 用（方向跟上面 per_hop_value_
# cdc_0 相反：這次是 sys_clk -> aurora_clk），見 rtl/aurora_ctrl_
# channel.v native_trig_out port 註解。
create_bd_cell -type module -reference level_cdc bid_per_hop_cdc_0
set_property -dict [list CONFIG.WIDTH {32}] [get_bd_cells bid_per_hop_cdc_0]
create_bd_cell -type module -reference level_cdc bid_total_boards_cdc_0
set_property -dict [list CONFIG.WIDTH {5}] [get_bd_cells bid_total_boards_cdc_0]
create_bd_cell -type module -reference level_cdc reserve_ok_cdc_0
create_bd_cell -type module -reference level_cdc reserve_busy_cdc_0
create_bd_cell -type module -reference level_cdc channel_up0_cdc_0
create_bd_cell -type module -reference level_cdc channel_up1_cdc_0

# 2026-07-30 新增：DDR 播放狀態（current_idx/next_idx/mux_sel/
# play_pos，跟既有 WO 0x30-33/concat_port_status_$ch 格式一致的
# 11-bit 打包值）從 wctrl_$ch 所在的 dac_clk domain 跨到 sys_clk，
# 給 aurora_reply_tx_0 的 QT_DDR_STATUS 用（見 rtl/aurora_reply_tx.v
# di_ddr_status_ch0~3 port 註解、NOTES.md 對應章節）。準靜態值，
# 比照既有慣例用官方 xpm_cdc_array_single（level_cdc 包裝）。
# concat_port_status_$ch/dout 是既有 32-bit xlconcat 輸出（in0~3 共
# 11-bit 有效值 + in4 21-bit padding），CDC 只需要低 11 bit，所以
# 先用 xlslice 縮寬到 [10:0] 再進 CDC（ddr_status_slice_$ch 的接線在
# concat_port_status_$ch 建立之後，見該區塊）。
foreach ch {0 1 2 3} {
    create_bd_cell -type module -reference level_cdc ddr_status_cdc_$ch
    set_property -dict [list CONFIG.WIDTH {11}] [get_bd_cells ddr_status_cdc_$ch]
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ddr_status_slice_$ch
    set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {10} CONFIG.DIN_TO {0}] \
        [get_bd_cells ddr_status_slice_$ch]
}

# 2026-07-31 新增：QT_FLASH_STATUS 用，fpga_flash_ctrl_0/flash_status
# 是 okClk domain，跟 QT_BOARD_INFO 那批 sys_clk 準靜態值不同，需要
# 額外跨到 sys_clk 才能給 aurora_reply_tx_0 用。比照既有慣例用官方
# xpm_cdc_array_single（level_cdc 包裝），4-bit 準靜態值。
create_bd_cell -type module -reference level_cdc flash_status_cdc_0
set_property -dict [list CONFIG.WIDTH {4}] [get_bd_cells flash_status_cdc_0]

# 2026-07-31 新增（同一天，擴充）：QT_FLASH_STATUS 加上 flash 實際
# 內容（scale_cfg/calib_coef），同樣是 okClk→sys_clk 準靜態值，比照
# 上面 flash_status_cdc_0 同一套 level_cdc 手法，各自獨立一顆 CDC
# （跟 ddr_status_cdc_$ch 每個 channel各自一顆的既有慣例一致，不用
# 一顆大 CDC 再切片）。
create_bd_cell -type module -reference level_cdc flash_scale_cfg_cdc_0
set_property -dict [list CONFIG.WIDTH {8}] [get_bd_cells flash_scale_cfg_cdc_0]
create_bd_cell -type module -reference level_cdc flash_coef_all_cdc_0
set_property -dict [list CONFIG.WIDTH {576}] [get_bd_cells flash_coef_all_cdc_0]

# -- OR gates (trigger / timer merge trees) ---------------------------------------
# 2026-07-24（trigger 統一化架構改版）：or_final_trig/or_au_fp_trig/
# or_total_trig 這三組 4-way merge tree 整個拿掉（本機 TI global
# trigger、Aurora T_TRIG_PORT、FP port trigger 都移除了），改成單一
# 2-input OR gate `or_sw_trig_$ch` = group_trig_select_$ch/trig_out
# （2026-07-27 起，Group-based Trigger 架構，見下方建立處註解，取代
# 原本直接接 native_trig_cdc_0/dst_pulse）OR trig_timer_$ch/trigger_out
# （見下方 wctrl_$ch/sw_trigger 接線處）。
foreach ch {0 1 2 3} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_sw_trig_$ch
    set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_sw_trig_$ch]
}
foreach port {1 2 3} {
    foreach sig {wr run loop} {
        create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_timer_${sig}_$port
        set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_timer_${sig}_$port]
    }
    create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_timer_depth_$port
    # 2026-08-04：3→5-bit，見上方 timer_slot_cdc_$port/timer_depth_cdc_$port
    # 建立處註解
    set_property -dict [list CONFIG.C_SIZE {5} CONFIG.C_OPERATION {or}] [get_bd_cells or_timer_depth_$port]
}
# 2026-08-04：port 0（module z0）補上 run/loop/depth/wr 的 OR gate，
# 修復 aurora_ctrl_mux.v 那次一起發現的「z0 完全沒有 Aurora 遠端路徑」
# 缺口（見該檔同日修復記錄，分兩階段：run/loop/depth 先補，intv/slot/
# wr 後來發現 z0 走的是完全獨立、從無 host wrapper 的舊封包
# T_TRIG_SLOT，同一天稍後也併進這裡）。or_timer_wr_0 用既有的
# ti_trig_list_wr（T_TRIG_SLOT 舊路徑留下的 TI bit 5，現在純粹當
# port 0 的 FP 直寫觸發位元用，跟 ti_timer_wr_1/2/3 同一種角色）跟
# 新的 out_timer_p0_wr OR 在一起。
foreach sig {wr run loop} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_timer_${sig}_0
    set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_timer_${sig}_0]
}
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_timer_depth_0
set_property -dict [list CONFIG.C_SIZE {5} CONFIG.C_OPERATION {or}] [get_bd_cells or_timer_depth_0]
# 2026-07-05 (step 15a): 原本這裡有 au_trigger_cdc_0/or_au_trig_pulse，把新
# 協定的 trigger_pulse 直接 OR 進 local_reg_handler_0/trigger_out——後來
# 使用者要求「trigger 不管哪裡來的都要走同一條路徑」，改成 Layer 3 合成
# 真正的 TRIGGER(0x01) 封包經 aurora_rx_merge_0 走 dispatcher 既有管線
# （local_reg_handler_0/trigger_out 這條路本身完全不用改），這裡不需要
# 額外的 CDC/OR 了，已移除。
# 2026-07-24（trigger 統一化架構改版）：first_trig_or 也移除——
# trig_timer_$port/first_trigger 改直接接 native_trig_cdc_0/dst_pulse
# （見下方接線處），不再需要跟 ti_global_trig/local_reg_handler_0/
# trigger_out 合併（兩者都已移除）。

# -- TI slices (from fp0/ti40_ep_trigger unless noted) -----------------------------
#    ti_trig_list_wr(5) is fed from aurora_ctrl_mux_0/out_ti_cmd
# 2026-08-10：ti_list_wr（bit2 slice）已移除——list write enable 改用
# aurora_ctrl_mux_0/out_list_wr（見上方 list_wr_cdc_0 建立處說明），
# 不再需要對 out_ti_cmd bit2 額外切一次片。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_trig_list_wr
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {5}  CONFIG.DIN_TO {5}]  [get_bd_cells ti_trig_list_wr]
# 2026-07-27：ti_calib_wr（bit4）/ti_amp_ctrl_wr（bit11）/ti_sine_ctrl_wr
# （bit6）已移除——統一讀取/寫入架構收斂，calib_coef/amp_ctrl/sine_gen
# 參數這 3 組本機直寫路徑都拔除，只剩 Aurora 封包路徑，bit4/6/11 空出
# 可用（見 PORTS.md 表2）。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_timer_wr_1
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {20} CONFIG.DIN_TO {20}] [get_bd_cells ti_timer_wr_1]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_timer_wr_2
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {21} CONFIG.DIN_TO {21}] [get_bd_cells ti_timer_wr_2]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice ti_timer_wr_3
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {22} CONFIG.DIN_TO {22}] [get_bd_cells ti_timer_wr_3]
# 2026-07-24（trigger 統一化架構改版）：ti_fp_port_trig_1-3（FP port
# trigger 用的 TI slice）移除，這個機制完全拿掉。

# -- WI slices --------------------------------------------------------------------
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice list_sel_slice
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {4} CONFIG.DIN_TO {0}] [get_bd_cells list_sel_slice]
# 2026-07-27：calib_sel_slice（WI 0x0b）/calib_data_slice（WI 0x0c）已
# 移除——calib_coef 本機直寫路徑拔除，`calib_sel` 讀寫共用同一個 port
# 的問題（見 PORTS.md 表3 0x0b 條目說明）用新增的 `coef_all`（QT_
# CALIB_STATUS）解決，不再需要獨立的讀取任意 index 能力，WI 0x0b/0x0c
# 整個退役。
# trig_slot_slice（原本切 fp0/wi0e_ep_dataout 供 T_TRIG_SLOT FP 路徑
# 用）2026-08-04 移除——z0 已合併進 T_TIMER_CTRL，T_TRIG_SLOT 整條路徑
# 退役，比照上面 calib_sel_slice/calib_data_slice 的既有退役慣例。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice scale_cfg_slice_calib
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {7} CONFIG.DIN_TO {0}] [get_bd_cells scale_cfg_slice_calib]
foreach ch {0 1 2 3} {
    set hi [expr {$ch * 3 + 2}]
    set lo [expr {$ch * 3}]
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice depth_slice_$ch
    set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM $hi CONFIG.DIN_TO $lo] [get_bd_cells depth_slice_$ch]
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice play_en_slice_$ch
    set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM $ch CONFIG.DIN_TO $ch] [get_bd_cells play_en_slice_$ch]
}
foreach port {0 1 2 3} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice timer_run_slice_$port
    set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM $port CONFIG.DIN_TO $port] \
        [get_bd_cells timer_run_slice_$port]
    set llo [expr {$port + 4}]
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice timer_loop_slice_$port
    set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM $llo CONFIG.DIN_TO $llo] \
        [get_bd_cells timer_loop_slice_$port]
    # 2026-08-04：depth 3-bit→5-bit（16 slot 排程功能），WI 0x10 word 內
    # 4 個 port 的 depth 欄位改成各佔 5-bit，從 bit 8 起共 20-bit（bit
    # 8-27），run(bit 0-3)/loop(bit 4-7) 維持不動，bit 28-31 仍空著，
    # 不用開第二個 WireIn word（原本 3-bit 版本佔 bit 8-19，見
    # PROJECT.md trigger group 排程功能三部曲「B」條目）
    set dlo [expr {8 + $port * 5}]
    set dhi [expr {8 + $port * 5 + 4}]
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice timer_depth_slice_$port
    set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM $dhi CONFIG.DIN_TO $dlo] \
        [get_bd_cells timer_depth_slice_$port]
}

# -- sync_start input concats -----------------------------------------------------
foreach name {concat_play_en_4b concat_fifo_a_empty concat_fifo_b_empty concat_mux_sel_4b} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat $name
    set_property -dict [list CONFIG.NUM_PORTS {4} \
        CONFIG.IN0_WIDTH {1} CONFIG.IN1_WIDTH {1} CONFIG.IN2_WIDTH {1} CONFIG.IN3_WIDTH {1}] \
        [get_bd_cells $name]
}

# ==============================================================================
#  External ports
# ==============================================================================
create_bd_port -dir I -type clk sys_clk_p
create_bd_port -dir I           sys_clk_n
create_bd_port -dir I  -from 4 -to 0  okUH
create_bd_port -dir O  -from 2 -to 0  okHU
create_bd_port -dir IO -from 31 -to 0 okUHU
create_bd_port -dir IO                okAA
create_bd_port -dir O  -from 5 -to 0  led

# 2026-07-05 (step 15a): 復原 SFP/refclk 外部埠（14.3b 為排除 Aurora 干擾移除）
create_bd_port -dir O sfp1_tx_p
create_bd_port -dir O sfp1_tx_n
create_bd_port -dir I sfp1_rx_p
create_bd_port -dir I sfp1_rx_n

create_bd_port -dir O sfp2_tx_p
create_bd_port -dir O sfp2_tx_n
create_bd_port -dir I sfp2_rx_p
create_bd_port -dir I sfp2_rx_n

# 2026-08-05 新增：SFP TX_DISABLE（TDIS，active-high，拉低才會致能
# 雷射）。這個設計從一開始就沒有驅動過這兩根，過去用 DAC 銅纜測試
# 從沒踩到（銅纜不需要雷射、不受 TX_DISABLE 影響），換成真正的光纖
# SFP 後才發現 Aurora 完全連不上（channel_up 全 FAIL）。查 Opal
# Kelly 官方文件確認腳位：SFP1 TDIS=C13、SFP2 TDIS=F13（PACKAGE_PIN/
# IOSTANDARD 見 awg_step16.xdc），純新增、不動任何既有邏輯，接固定
# 0（拉低=致能）即可，見 NOTES.md 2026-08-05「換 SFP+光纖後 Aurora
# 連不上」章節。
create_bd_port -dir O TDIS_1
create_bd_port -dir O TDIS_2

create_bd_port -dir I -type clk aurora_refclk_p
create_bd_port -dir I           aurora_refclk_n

# 2026-07-11：ext clock 第一階段（MGTREFCLK1_226, M7/M6）
create_bd_port -dir I -type clk ext_dac_refclk_p
create_bd_port -dir I           ext_dac_refclk_n

# DDR4 external interface (board interface manages pin constraints)
make_bd_intf_pins_external [get_bd_intf_pins ddr4_0/C0_SYS_CLK]
set_property name ddr4_sys_clk [get_bd_intf_ports C0_SYS_CLK_0]
make_bd_intf_pins_external [get_bd_intf_pins ddr4_0/C0_DDR4]
set_property name C0_DDR4 [get_bd_intf_ports C0_DDR4_0]

# -- Step 14.3b playback port: SYZYGY / ZmodAWG DAC external ports (4 connectors) --
foreach ch {0 1 2 3} {
    create_bd_port -dir O                sZmodDAC_CS_$ch
    create_bd_port -dir O                sZmodDAC_SCLK_$ch
    create_bd_port -dir IO               sZmodDAC_SDIO_$ch
    create_bd_port -dir O                sZmodDAC_Reset_$ch
    create_bd_port -dir O                ZmodDAC_ClkIO_$ch
    create_bd_port -dir O                ZmodDAC_ClkIn_$ch
    create_bd_port -dir O -from 13 -to 0 dZmodDAC_Data_$ch
    create_bd_port -dir O                sZmodDAC_SetFS1_$ch
    create_bd_port -dir O                sZmodDAC_SetFS2_$ch
    create_bd_port -dir O                sZmodDAC_EnOut_$ch
}

# ==============================================================================
#  Connections
# ==============================================================================

# -- FrontPanel physical ---------------------------------------------------------
connect_bd_net [get_bd_ports okUH]  [get_bd_pins fp0/okUH]
connect_bd_net [get_bd_ports okHU]  [get_bd_pins fp0/okHU]
connect_bd_net [get_bd_ports okUHU] [get_bd_pins fp0/okUHU]
connect_bd_net [get_bd_ports okAA]  [get_bd_pins fp0/okAA]

# -- System clocks -----------------------------------------------------------------
connect_bd_net [get_bd_ports sys_clk_p] [get_bd_pins clk_wiz_0/clk_in1_p]
connect_bd_net [get_bd_ports sys_clk_n] [get_bd_pins clk_wiz_0/clk_in1_n]
connect_bd_net [get_bd_pins const_one/dout] [get_bd_pins clk_wiz_0/resetn]

connect_bd_net [get_bd_pins clk_wiz_0/clk_100] [get_bd_pins rst_clk100/slowest_sync_clk]
connect_bd_net [get_bd_pins const_one/dout]     [get_bd_pins rst_clk100/ext_reset_in]
connect_bd_net [get_bd_pins const_one/dout]     [get_bd_pins rst_clk100/aux_reset_in]
connect_bd_net [get_bd_pins const_zero/dout]    [get_bd_pins rst_clk100/mb_debug_sys_rst]
connect_bd_net [get_bd_pins clk_wiz_0/locked]  [get_bd_pins rst_clk100/dcm_locked]

# sys_clk fan-out (v9: added dispatcher/ddr_writer/aurora_tx_arbiter/async_fifo_aurora_tx)
# 2026-07-05 (step 15a): 復原 async_fifo_aurora_tx/wr_clk（dispatcher 寫入側）
# 跟 async_fifo_aurora_rx/rd_clk（dispatcher 讀出側），這兩個是這兩個 CDC
# FIFO 的 sys_clk 側（aurora_clk 側已在上面 aurora_clk fan-out 接好）
# Note: diag_cdc_0/aurora_clk kept -- this port name says aurora_clk, but under
# the v9 architecture it was always wired to sys_clk (diag_cdc_0 is a pass-through,
# see PROJECT.md); unrelated to physical Aurora
connect_bd_net [get_bd_pins clk_wiz_0/clk_100] \
    [get_bd_pins fp0/ti40_ep_clk]                  \
    [get_bd_pins fp_input_wr_0/sys_clk]             \
    [get_bd_pins ext_clk_freq_counter_0/sys_clk]    \
    [get_bd_pins fp_input_rd_0/sys_clk]             \
    [get_bd_pins fp_fifo_0/rd_clk]                  \
    [get_bd_pins local_reg_handler_0/sys_clk]       \
    [get_bd_pins board_cfg_reg_0/clk]               \
    [get_bd_pins diag_cdc_0/sys_clk]                \
    [get_bd_pins diag_capture_0/sys_clk]            \
    [get_bd_pins aurora_reply_tx_0/sys_clk]         \
    [get_bd_pins ddr_status_cdc_0/dst_clk] [get_bd_pins ddr_status_cdc_1/dst_clk] \
    [get_bd_pins ddr_status_cdc_2/dst_clk] [get_bd_pins ddr_status_cdc_3/dst_clk] \
    [get_bd_pins flash_status_cdc_0/dst_clk] \
    [get_bd_pins flash_scale_cfg_cdc_0/dst_clk] \
    [get_bd_pins flash_coef_all_cdc_0/dst_clk] \
    [get_bd_pins status_reply_capture_0/sys_clk]    \
    [get_bd_pins reply_diag_okclk_rst_sync_0/sys_clk] \
    [get_bd_pins async_fifo_aurora_tx/wr_clk]       \
    [get_bd_pins async_fifo_reply_tx/wr_clk]        \
    [get_bd_pins async_fifo_aurora_rx/rd_clk]       \
    [get_bd_pins async_fifo_local/clk]              \
    [get_bd_pins fp_ddr4_rw_1/clk]                  \
    [get_bd_pins ddr_writer_axi_fifo/s_aclk]        \
    [get_bd_pins axi_cc_fpddr4/s_axi_aclk]          \
    [get_bd_pins axi_cc_ddrzero/s_axi_aclk]         \
    [get_bd_pins ddr_zero_writer_0/sys_clk]         \
    [get_bd_pins dispatcher_0/sys_clk]              \
    [get_bd_pins ddr_writer_0/sys_clk]              \
    [get_bd_pins au_reserve_start_sticky_0/clk]     \
    [get_bd_pins diag_cdc_0/aurora_clk]             \
    [get_bd_pins okclk_rst_sync_0/sys_clk]          \
    [get_bd_pins flash_payload_cdc_0/sys_clk]       \
    [get_bd_pins flash_erase_pulse_cdc_0/src_clk]   \
    [get_bd_pins flash_load_valid_cdc_0/dst_clk]    \
    [get_bd_pins flash_target_sel_reg_0/clk]        \
    [get_bd_pins flash_scale_load_valid_cdc_0/dst_clk] \
    [get_bd_pins flash_amp_load_valid_cdc_0/dst_clk]   \
    [get_bd_pins flash_coef_load_valid_cdc_0/dst_clk]  \
    [get_bd_pins sine_ctrl_regs_0/clk]
# 2026-07-09：fpga_flash_ctrl_0/clk 從這裡移除，改接 fp0/okClk（見下方
# 新的 okClk fan-out），見 PROJECT.md 第 25 節
# Step 14.3b playback port: sys_clk (clk_wiz_0/clk_100) also drives the new
# playback control-plane cells (added as extra sinks on the same net)
connect_bd_net [get_bd_pins clk_wiz_0/clk_100] \
    [get_bd_pins aurora_ctrl_mux_0/clk]  \
    [get_bd_pins awg_calib_regs_0/clk]   \
    [get_bd_pins syzygy_ready_0/clk]     \
    [get_bd_pins reinit_pulse_cdc_0/src_clk]  \
    [get_bd_pins timer_slot_cdc_0/src_clk]    [get_bd_pins timer_intv_cdc_0/src_clk]  \
    [get_bd_pins timer_depth_cdc_0/src_clk]   [get_bd_pins timer_run_cdc_0/src_clk]   \
    [get_bd_pins timer_loop_cdc_0/src_clk]    [get_bd_pins timer_wr_cdc_0/src_clk]    \
    [get_bd_pins timer_slot_cdc_1/src_clk]    [get_bd_pins timer_intv_cdc_1/src_clk]  \
    [get_bd_pins timer_depth_cdc_1/src_clk]   [get_bd_pins timer_run_cdc_1/src_clk]   \
    [get_bd_pins timer_loop_cdc_1/src_clk]    [get_bd_pins timer_wr_cdc_1/src_clk]    \
    [get_bd_pins timer_slot_cdc_2/src_clk]    [get_bd_pins timer_intv_cdc_2/src_clk]  \
    [get_bd_pins timer_depth_cdc_2/src_clk]   [get_bd_pins timer_run_cdc_2/src_clk]   \
    [get_bd_pins timer_loop_cdc_2/src_clk]    [get_bd_pins timer_wr_cdc_2/src_clk]    \
    [get_bd_pins timer_slot_cdc_3/src_clk]    [get_bd_pins timer_intv_cdc_3/src_clk]  \
    [get_bd_pins timer_depth_cdc_3/src_clk]   [get_bd_pins timer_run_cdc_3/src_clk]   \
    [get_bd_pins timer_loop_cdc_3/src_clk]    [get_bd_pins timer_wr_cdc_3/src_clk]    \
    [get_bd_pins manual_trig_delay_cdc_0/src_clk] \
    [get_bd_pins manual_trig_delay_active_cdc_0/src_clk] \
    [get_bd_pins manual_total_boards_cdc_0/src_clk] \
    [get_bd_pins manual_per_hop_cdc_0/src_clk]    \
    [get_bd_pins manual_per_hop_active_cdc_0/src_clk] \
    [get_bd_pins au_trig_group_cdc_0/src_clk]     \
    [get_bd_pins gsc_slot_cdc_0/src_clk]  [get_bd_pins gsc_intv_cdc_0/src_clk]  \
    [get_bd_pins gsc_group_cdc_0/src_clk] [get_bd_pins gsc_depth_cdc_0/src_clk] \
    [get_bd_pins gsc_run_cdc_0/src_clk]   [get_bd_pins gsc_loop_cdc_0/src_clk]  \
    [get_bd_pins gsc_wr_cdc_0/src_clk]    [get_bd_pins gsc_arm_mode_cdc_0/src_clk] \
    [get_bd_pins sine_sel_cdc_0/src_clk]      [get_bd_pins sine_data_cdc_0/src_clk]      \
    [get_bd_pins sine_wr_cdc_0/src_clk]       \
    [get_bd_pins sine_list_sel_cdc_0/src_clk] [get_bd_pins sine_list_data_cdc_0/src_clk] \
    [get_bd_pins sine_list_wr_cdc_0/src_clk]  \
    [get_bd_pins list_sel_cdc_0/src_clk]  [get_bd_pins list_addr_cdc_0/src_clk] \
    [get_bd_pins list_len_cdc_0/src_clk]  [get_bd_pins list_wr_cdc_0/src_clk]   \
    [get_bd_pins gsc_fire_fifo_0/rd_clk]  [get_bd_pins fifo_event_reader_0/clk] \
    [get_bd_pins zmod_awg_0/SysClk100]        \
    [get_bd_pins zmod_awg_1/SysClk100]        \
    [get_bd_pins zmod_awg_2/SysClk100]        \
    [get_bd_pins zmod_awg_3/SysClk100]        \
    [get_bd_pins au_init_pulse_cdc_0/src_clk]     \
    [get_bd_pins au_trig_pulse_cdc_0/src_clk]     \
    [get_bd_pins au_reserve_pulse_cdc_0/src_clk]  \
    [get_bd_pins au_ctrl_rst_pulse_cdc_0/src_clk] \
    [get_bd_pins au_full_reset_pulse_cdc_0/src_clk] \
    [get_bd_pins au_pma_init_pulse_cdc_0/src_clk] \
    [get_bd_pins board_id_cdc_0/src_clk]          \
    [get_bd_pins flash_target_sel_cdc_0/src_clk]  \
    [get_bd_pins is_master_cdc_0/src_clk]         \
    [get_bd_pins reserve_dest_id_cdc_0/src_clk]   \
    [get_bd_pins init_ok_cdc_0/dst_clk]           \
    [get_bd_pins total_boards_cdc_0/dst_clk]      \
    [get_bd_pins board_index_cdc_0/dst_clk]       \
    [get_bd_pins per_hop_value_cdc_0/dst_clk]     \
    [get_bd_pins per_hop_value_eff_cdc_0/dst_clk] \
    [get_bd_pins bid_per_hop_cdc_0/src_clk]       \
    [get_bd_pins bid_total_boards_cdc_0/src_clk]  \
    [get_bd_pins reserve_ok_cdc_0/dst_clk]        \
    [get_bd_pins reserve_busy_cdc_0/dst_clk]      \
    [get_bd_pins reserve_fwd_cdc_0/dst_clk]       \
    [get_bd_pins reserve_timeout_cdc_0/dst_clk]   \
    [get_bd_pins reserve_reply_cdc_0/dst_clk]     \
    [get_bd_pins reserve_pulse_cdc_0/dst_clk]     \
    [get_bd_pins reply_seen_cdc_0/dst_clk]        \
    [get_bd_pins reply_granted_cdc_0/dst_clk]     \
    [get_bd_pins channel_up0_cdc_0/dst_clk]       \
    [get_bd_pins channel_up1_cdc_0/dst_clk]

# peripheral_reset (active HIGH)
# 2026-07-05 (step 15a): 復原 async_fifo_aurora_tx/rst，接 sys_clk domain 的
# reset（不是 aurora_64b66b_0/sys_reset_out）——因為它的 wr_clk 是 sys_clk
# （dispatcher 寫入側），rst 必須 synchronous to wr_clk，見上面的 rst domain
# 陷阱說明
# 2026-07-22：async_fifo_aurora_tx/rst 從這個共用清單移出，改接下面新的
# or_aurora_tx_fifo_rst（peripheral_reset OR ti_au_full_reset）——這樣
# 「整個 Aurora 回到乾淨狀態」這個新指令，也能重置這個 dispatcher 送進
# Aurora 前的 FIFO（sys_clk domain，不需要額外 CDC，ti_au_full_reset
# 這個 TI bit 本身就已經在 sys_clk domain），不用連帶重置這個清單裡
# board_id/scale_cfg 等完全跟 Aurora 無關的模組。
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_aurora_tx_fifo_rst
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_aurora_tx_fifo_rst]
connect_bd_net [get_bd_pins rst_clk100/peripheral_reset] [get_bd_pins or_aurora_tx_fifo_rst/Op1]
connect_bd_net [get_bd_pins ti_au_full_reset/Dout] [get_bd_pins or_aurora_tx_fifo_rst/Op2]
connect_bd_net [get_bd_pins or_aurora_tx_fifo_rst/Res] \
    [get_bd_pins async_fifo_aurora_tx/rst] \
    [get_bd_pins async_fifo_reply_tx/rst]

connect_bd_net [get_bd_pins rst_clk100/peripheral_reset] \
    [get_bd_pins fp_input_wr_0/sys_rst]         \
    [get_bd_pins fp_input_rd_0/sys_rst]         \
    [get_bd_pins local_reg_handler_0/sys_rst]   \
    [get_bd_pins board_cfg_reg_0/rst]           \
    [get_bd_pins flash_target_sel_reg_0/rst]    \
    [get_bd_pins diag_capture_0/sys_rst]        \
    [get_bd_pins dispatcher_0/sys_rst]          \
    [get_bd_pins ddr_writer_0/sys_rst]          \
    [get_bd_pins ddr_zero_writer_0/sys_rst]     \
    [get_bd_pins async_fifo_local/srst]         \
    [get_bd_pins au_reserve_start_sticky_0/rst] \
    [get_bd_pins sine_ctrl_regs_0/rst]          \
    [get_bd_pins aurora_reply_tx_0/sys_rst]     \
    [get_bd_pins status_reply_capture_0/sys_rst] \
    [get_bd_pins reply_diag_okclk_rst_sync_0/rst_in]

# 2026-07-08（debug-only，flash round-trip 除錯）：fpga_flash_ctrl_0 的
# rst 從上面共用的 peripheral_reset 網路獨立出來，改成
# peripheral_reset OR ti_flash_ctrl_rst（TI bit29），這樣 host 觸發這個
# bit 只會重置 flash controller 自己（重跑 flash_startup_loader），不會
# 連帶重置 board_cfg_reg_0/dispatcher_0 等模組的 board_id/is_master 等
# 狀態。正常上電時 peripheral_reset 仍會照常驅動這個模組初始化，行為
# 不變；只有多了 host 可手動觸發的第二個 reset 來源。
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_flash_ctrl_rst
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_flash_ctrl_rst]
connect_bd_net [get_bd_pins rst_clk100/peripheral_reset] [get_bd_pins or_flash_ctrl_rst/Op1]
connect_bd_net [get_bd_pins ti_flash_ctrl_rst/Dout]      [get_bd_pins or_flash_ctrl_rst/Op2]
# 2026-07-09：fpga_flash_ctrl_0 搬到 okClk 域，or_flash_ctrl_rst/Res
# （sys_clk 域）不能再直接接 fpga_flash_ctrl_0/rst，要先過
# okclk_rst_sync_0（比照 fp_input.v 的 sys_rst_ok 手法），見 PROJECT.md
# 第 25 節。同一個 sys_clk 域訊號也直接餵給 flash_payload_cdc_0/rst
# （它内部的 xpm_fifo_async 自己處理雙時脈同步，不需要先同步）跟三個
# trigger_cdc 的 src_rst/dst_rst（依訊號方向決定哪一側用同步後的
# okclk_rst_sync_0/rst_ok，哪一側直接用這裡的 sys_clk 域訊號）。
connect_bd_net [get_bd_pins or_flash_ctrl_rst/Res] \
    [get_bd_pins okclk_rst_sync_0/rst_in]            \
    [get_bd_pins flash_payload_cdc_0/rst]            \
    [get_bd_pins flash_erase_pulse_cdc_0/src_rst]    \
    [get_bd_pins flash_load_valid_cdc_0/dst_rst]     \
    [get_bd_pins flash_scale_load_valid_cdc_0/dst_rst] \
    [get_bd_pins flash_amp_load_valid_cdc_0/dst_rst]   \
    [get_bd_pins flash_coef_load_valid_cdc_0/dst_rst]
connect_bd_net [get_bd_pins okclk_rst_sync_0/rst_ok] \
    [get_bd_pins fpga_flash_ctrl_0/rst]                \
    [get_bd_pins flash_erase_pulse_cdc_0/dst_rst]      \
    [get_bd_pins flash_load_valid_cdc_0/src_rst]       \
    [get_bd_pins flash_scale_load_valid_cdc_0/src_rst] \
    [get_bd_pins flash_amp_load_valid_cdc_0/src_rst]   \
    [get_bd_pins flash_coef_load_valid_cdc_0/src_rst]

# 2026-07-10（測試用）：TI bit28（ti_fifo_rst）獨立觸發 fp_fifo_0 +
# fp_input_wr_0 pair-acc 重置，跟上面 flash controller 那組完全分開，
# 互不影響。fifo_rst_sync_0 展寬+同步這個 sys_clk 側的窄 pulse，跟
# okclk_rst_sync_0 是同一顆 module reference 的第二個實例。
connect_bd_net [get_bd_pins clk_wiz_0/clk_100] [get_bd_pins fifo_rst_sync_0/sys_clk]
connect_bd_net [get_bd_pins fp0/okClk]         [get_bd_pins fifo_rst_sync_0/ok_clk]
connect_bd_net [get_bd_pins ti_fifo_rst/Dout]  [get_bd_pins fifo_rst_sync_0/rst_in]

create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_fifo_rst
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_fifo_rst]
connect_bd_net [get_bd_pins rst_clk100/peripheral_reset] [get_bd_pins or_fifo_rst/Op1]
connect_bd_net [get_bd_pins fifo_rst_sync_0/rst_ok]      [get_bd_pins or_fifo_rst/Op2]
connect_bd_net [get_bd_pins or_fifo_rst/Res]             [get_bd_pins fp_fifo_0/rst]

# ti_fifo_rst 同時 OR 進 fp_input_wr_0/half_reset（清掉 pair-acc 的
# half/lo_word 狀態，避免 reset 後半組 word 卡住），沿用 fp_input_wr_0
# 內部既有的 half_reset 展寬+同步機制，不需要新增 port。local_reg_
# handler_0/half_reset 維持只接原本的 ti_fp_align/Dout，不受這個新
# trigger 影響。
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_fp_half_reset
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_fp_half_reset]
connect_bd_net [get_bd_pins ti_fp_align/Dout]  [get_bd_pins or_fp_half_reset/Op1]
connect_bd_net [get_bd_pins ti_fifo_rst/Dout]  [get_bd_pins or_fp_half_reset/Op2]
connect_bd_net [get_bd_pins or_fp_half_reset/Res] [get_bd_pins fp_input_wr_0/half_reset]

# peripheral_resetn (active LOW, for AXI modules)
connect_bd_net [get_bd_pins rst_clk100/peripheral_aresetn] \
    [get_bd_pins fp_ddr4_rw_1/resetn]          \
    [get_bd_pins ddr_writer_axi_fifo/s_aresetn] \
    [get_bd_pins axi_cc_fpddr4/s_axi_aresetn]

# Step 14.3b playback port: peripheral_reset (active HIGH) also drives the new
# playback cells on sys_clk / rd side
connect_bd_net [get_bd_pins rst_clk100/peripheral_reset] \
    [get_bd_pins trig_timer_0/rst]   \
    [get_bd_pins trig_timer_1/rst]   \
    [get_bd_pins trig_timer_2/rst]   \
    [get_bd_pins trig_timer_3/rst]   \
    [get_bd_pins group_trig_scheduler_0/rst] \
    [get_bd_pins gsc_fire_fifo_0/rst] \
    [get_bd_pins calib_mux_0/ext_rst] \
    [get_bd_pins wctrl_0/rst] [get_bd_pins wctrl_1/rst] \
    [get_bd_pins wctrl_2/rst] [get_bd_pins wctrl_3/rst] \
    [get_bd_pins or_fifo_poweron_rst/Op2] \
    [get_bd_pins sine_ctrl_regs_0/dac_rst] \
    [get_bd_pins dac_trig_queue_0/dac_rst] \
    [get_bd_pins aurora_reply_tx_0/dac_rst] \
    [get_bd_pins sine_gen_0_1_a/rst] [get_bd_pins sine_gen_0_1_b/rst] \
    [get_bd_pins sine_gen_0_2_a/rst] [get_bd_pins sine_gen_0_2_b/rst] \
    [get_bd_pins sine_gen_1_1_a/rst] [get_bd_pins sine_gen_1_1_b/rst] \
    [get_bd_pins sine_gen_1_2_a/rst] [get_bd_pins sine_gen_1_2_b/rst] \
    [get_bd_pins sine_gen_2_1_a/rst] [get_bd_pins sine_gen_2_1_b/rst] \
    [get_bd_pins sine_gen_2_2_a/rst] [get_bd_pins sine_gen_2_2_b/rst] \
    [get_bd_pins sine_gen_3_1_a/rst] [get_bd_pins sine_gen_3_1_b/rst] \
    [get_bd_pins sine_gen_3_2_a/rst] [get_bd_pins sine_gen_3_2_b/rst] \
    [get_bd_pins amp_ramp_gen_0_1_a/rst] [get_bd_pins amp_ramp_gen_0_1_b/rst] \
    [get_bd_pins amp_ramp_gen_0_2_a/rst] [get_bd_pins amp_ramp_gen_0_2_b/rst] \
    [get_bd_pins amp_ramp_gen_1_1_a/rst] [get_bd_pins amp_ramp_gen_1_1_b/rst] \
    [get_bd_pins amp_ramp_gen_1_2_a/rst] [get_bd_pins amp_ramp_gen_1_2_b/rst] \
    [get_bd_pins amp_ramp_gen_2_1_a/rst] [get_bd_pins amp_ramp_gen_2_1_b/rst] \
    [get_bd_pins amp_ramp_gen_2_2_a/rst] [get_bd_pins amp_ramp_gen_2_2_b/rst] \
    [get_bd_pins amp_ramp_gen_3_1_a/rst] [get_bd_pins amp_ramp_gen_3_1_b/rst] \
    [get_bd_pins amp_ramp_gen_3_2_a/rst] [get_bd_pins amp_ramp_gen_3_2_b/rst]

# -- DDR4 clocks & resets ----------------------------------------------------------
connect_bd_net [get_bd_pins const_zero/dout] [get_bd_pins ddr4_0/sys_rst]
connect_bd_net [get_bd_pins const_one/dout]  [get_bd_pins ddr4_0/c0_ddr4_aresetn]

# DDR4 UI clock -> both converters' m_axi side + smartconnect_0 (now ui_clk) +
# the 8 stream readers + FIFO write side (Step 14.3b playback port: smartconnect
# moved to ui_clk, M00 connects directly to ddr4, no CC between)
connect_bd_net [get_bd_pins ddr4_0/c0_ddr4_ui_clk] \
    [get_bd_pins ddr_writer_axi_fifo/m_aclk] \
    [get_bd_pins axi_cc_fpddr4/m_axi_aclk] \
    [get_bd_pins axi_cc_ddrzero/m_axi_aclk] \
    [get_bd_pins smartconnect_0/aclk]      \
    [get_bd_pins reader_a_0/clk] [get_bd_pins reader_a_1/clk] \
    [get_bd_pins reader_a_2/clk] [get_bd_pins reader_a_3/clk] \
    [get_bd_pins reader_b_0/clk] [get_bd_pins reader_b_1/clk] \
    [get_bd_pins reader_b_2/clk] [get_bd_pins reader_b_3/clk] \
    [get_bd_pins fifo_a_0/wr_clk] [get_bd_pins fifo_a_1/wr_clk] \
    [get_bd_pins fifo_a_2/wr_clk] [get_bd_pins fifo_a_3/wr_clk] \
    [get_bd_pins fifo_b_0/wr_clk] [get_bd_pins fifo_b_1/wr_clk] \
    [get_bd_pins fifo_b_2/wr_clk] [get_bd_pins fifo_b_3/wr_clk]

# DDR4 UI sync_rst -> inverted -> ui-domain aresetn (both CCs' m_axi + smartconnect);
# raw sync_rst -> reader rst (same ui_clk domain)
connect_bd_net [get_bd_pins ddr4_0/c0_ddr4_ui_clk_sync_rst] \
    [get_bd_pins inv_ddr4_rst/Op1] \
    [get_bd_pins reader_a_0/rst] [get_bd_pins reader_a_1/rst] \
    [get_bd_pins reader_a_2/rst] [get_bd_pins reader_a_3/rst] \
    [get_bd_pins reader_b_0/rst] [get_bd_pins reader_b_1/rst] \
    [get_bd_pins reader_b_2/rst] [get_bd_pins reader_b_3/rst] \
    [get_bd_pins or_fifo_poweron_rst/Op1]
connect_bd_net [get_bd_pins inv_ddr4_rst/Res] \
    [get_bd_pins axi_cc_fpddr4/m_axi_aresetn] \
    [get_bd_pins axi_cc_ddrzero/m_axi_aresetn] \
    [get_bd_pins smartconnect_0/aresetn]
# 註：ddr_writer_axi_fifo（fifo_generator）沒有獨立的 m_aresetn pin——
# 跟 axi_clock_converter 不同，PG057 文件明確說 s_aresetn 是整個 core
# 唯一的 reset，已經在上面 rst_clk100/peripheral_aresetn 那段接過了，
# 這裡不用（也不能）再接一次。

# 2026-07-04: or_fifo_poweron_rst = ui_clk_sync_rst OR peripheral_reset (shared
# power-on reset for all 8 dc_fifo_xpm instances). Per-buffer OR with that
# buffer's own flush pulse from wctrl_$ch happens below, once wctrl_$ch exists.
foreach ch {0 1 2 3} {
    connect_bd_net [get_bd_pins or_fifo_poweron_rst/Res] [get_bd_pins or_fifo_a_rst_$ch/Op1]
    connect_bd_net [get_bd_pins or_fifo_poweron_rst/Res] [get_bd_pins or_fifo_b_rst_$ch/Op1]
    connect_bd_net [get_bd_pins wctrl_$ch/fifo_a_flush]  [get_bd_pins or_fifo_a_rst_$ch/Op2]
    connect_bd_net [get_bd_pins wctrl_$ch/fifo_b_flush]  [get_bd_pins or_fifo_b_rst_$ch/Op2]
    connect_bd_net [get_bd_pins or_fifo_a_rst_$ch/Res]   [get_bd_pins fifo_a_$ch/rst]
    connect_bd_net [get_bd_pins or_fifo_b_rst_$ch/Res]   [get_bd_pins fifo_b_$ch/rst]
}

# DDR4 calib_done -> fp_ddr4_rw_1 (read-back) + all 8 stream readers
connect_bd_net [get_bd_pins ddr4_0/c0_init_calib_complete] \
    [get_bd_pins fp_ddr4_rw_1/calib_done] \
    [get_bd_pins reader_a_0/calib_done] [get_bd_pins reader_a_1/calib_done] \
    [get_bd_pins reader_a_2/calib_done] [get_bd_pins reader_a_3/calib_done] \
    [get_bd_pins reader_b_0/calib_done] [get_bd_pins reader_b_1/calib_done] \
    [get_bd_pins reader_b_2/calib_done] [get_bd_pins reader_b_3/calib_done]

# -- Aurora refclk + user clock -------------------------------------------------
# 2026-07-05 (step 15a): 復原，拓樸不變（雙向化不影響 clock/reset 這層）
# 2026-07-17：aurora_refclk_p/n → aurora_refclk_ibuf_0 這條線拿掉了，見下面
# 「aurora_64b66b_0 改 SupportLevel=1」的說明——這兩根實體差動 pin 现在只
# 接 aurora_64b66b_0/gt_refclk1_p/n，aurora_refclk_ibuf_0 整個不用了。

# 2026-07-11：ext clock 第一階段
connect_bd_net [get_bd_ports ext_dac_refclk_p] [get_bd_pins ext_dac_clk_ibuf_0/refclk_p]
connect_bd_net [get_bd_ports ext_dac_refclk_n] [get_bd_pins ext_dac_clk_ibuf_0/refclk_n]
connect_bd_net [get_bd_pins ext_dac_clk_ibuf_0/ext_clk_out] [get_bd_pins ext_clk_freq_counter_0/ext_clk]
connect_bd_net [get_bd_pins ext_clk_freq_counter_0/freq_count_sync] [get_bd_pins fp0/wo2e_ep_datain]

# 2026-07-13：ext clock 階段 A — clk_wiz_ext_0（還不接 dac_clk_mux_0，見上方 create_bd_cell 註解）
connect_bd_net [get_bd_pins ext_dac_clk_ibuf_0/ext_clk_out] [get_bd_pins clk_wiz_ext_0/clk_in1]
connect_bd_net [get_bd_pins const_one/dout]                 [get_bd_pins clk_wiz_ext_0/resetn]
# 2026-07-17：aurora_64b66b_0 改 SupportLevel=1 後自己內部處理 refclk
# buffering（自己生一顆 IBUFDS_GTE4），直接吃實體差動 pin。
#
# ⚠️ 2026-07-17 第二輪修正：原本這裡還讓 aurora_refclk_ibuf_0（獨立
# RTL 模組，裡面自己也包一顆 IBUFDS_GTE4）繼續接同一組 aurora_refclk_p/n
# 實體 pin，餵給 aurora_64b66b_1/refclk1_in——這條線在 validate_bd_design
# 沒被抓到（BD 連線層級合法），但 place_design 報錯：
#   ERROR: [Place 30-602] IO port 'aurora_refclk_p' is driving multiple
#   buffers（aurora_64b66b_0 自己的 IBUFDS_GTE4_refclk1 +
#   aurora_refclk_ibuf_0/inst/ibuf 這兩顆）
# 一組差動 pin 實體上只能接一顆 IBUFDS_GTE4。改成用 aurora_64b66b_0 自己
# 已經緩衝好的 gt_refclk1_out 這個 port 去餵 aurora_64b66b_1/refclk1_in
# （官方 IP 就是設計給「同 quad 共用 refclk」情境用這個 port），
# aurora_refclk_ibuf_0 完全不需要了（不是留著當孤兒，是整個不接、不用，
# 見下方 create_bd_cell 那行也拿掉了）。
connect_bd_net [get_bd_ports aurora_refclk_p] [get_bd_pins aurora_64b66b_0/gt_refclk1_p]
connect_bd_net [get_bd_ports aurora_refclk_n] [get_bd_pins aurora_64b66b_0/gt_refclk1_n]
connect_bd_net [get_bd_pins aurora_64b66b_0/gt_refclk1_out] \
    [get_bd_pins aurora_64b66b_1/refclk1_in]

# 2026-07-17：QPLL 共用（aurora_64b66b_0 產生、aurora_64b66b_1 消費，
# 同一個 quad 上的兩個 lane 共用一顆 QPLL）。gt_qpllrefclklost_quad1
# 這條刻意沒有 _in 字尾，名稱要抓精確。gt_to_common_qpllreset_out
# （aurora_64b66b_1 多出來的）留空不接，驗證過不會報錯。
connect_bd_net [get_bd_pins aurora_64b66b_0/gt_qpllclk_quad1_out]        [get_bd_pins aurora_64b66b_1/gt_qpllclk_quad1_in]
connect_bd_net [get_bd_pins aurora_64b66b_0/gt_qplllock_quad1_out]       [get_bd_pins aurora_64b66b_1/gt_qplllock_quad1_in]
connect_bd_net [get_bd_pins aurora_64b66b_0/gt_qpllrefclk_quad1_out]     [get_bd_pins aurora_64b66b_1/gt_qpllrefclk_quad1_in]
connect_bd_net [get_bd_pins aurora_64b66b_0/gt_qpllrefclklost_quad1_out] [get_bd_pins aurora_64b66b_1/gt_qpllrefclklost_quad1]

# aurora_user_clk_buf_0：2026-07-28 整個 cell 已從上方 create_bd_cell
# 移除（見該處說明），這裡不再接 tx_out_clk 進去。
connect_bd_net [get_bd_pins aurora_64b66b_1/tx_out_clk]      [get_bd_pins aurora_user_clk_buf_1/clk_in]
connect_bd_net [get_bd_pins aurora_64b66b_1/bufg_gt_clr_out] [get_bd_pins aurora_user_clk_buf_1/clr]

# 2026-07-08 修正：src_clk 原本直接接 tx_out_clk（GT 原始輸出，跟
# hard_err 暫存器本身同一個 clock domain），但 tx_out_clk 是 GT 專用的
# dedicated clock 繞線資源，只能直接接 BUFG_GT，扇出到一般邏輯的 clock
# pin 會導致 route_design 報 unroutable pin（兩個 txoutclk_out[0] net
# 佈不了線）。改接 aurora_clk（`aurora_user_clk_buf_0/1/clk_out`，
# tx_out_clk 經同一顆 BUFG_GT 純緩衝、無分頻/倍頻，頻率完全一樣，且已經
# 在整個設計裡穩定扇出），src_clk/dst_clk 相同也還是安全的 CDC 寫法
# （level_cdc 在同一時脈兩側只是多一層保險的同步器，不是錯誤用法）。
connect_bd_net [get_bd_pins aurora_64b66b_0/hard_err]   [get_bd_pins hard_err_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_64b66b_1/hard_err]   [get_bd_pins hard_err_cdc_1/src_in]

# aurora_clk fan-out：Aurora IP 本身 + 兩個 CDC 邊界 FIFO + 新的三個模組
# （取代原本只接 aurora_packet_tx_0 一個模組的做法）
# 2026-07-17：來源從 aurora_user_clk_buf_0/clk_out 改成
# aurora_64b66b_0/user_clk_out——SupportLevel=1 之後 _0 自己內部產生
# user_clk_out，不再需要外部 BUFG_GT 緩衝，_0 自己的 user_clk/sync_clk
# port 也沒了所以從目的地清單移除；aurora_64b66b_1 的 user_clk/sync_clk
# 維持原本拓樸（跟改之前一樣接在同一條共用 aurora_clk 上），只是訊號源頭
# 換了。
connect_bd_net [get_bd_pins aurora_64b66b_0/user_clk_out] \
    [get_bd_pins aurora_64b66b_1/user_clk]       \
    [get_bd_pins aurora_64b66b_1/sync_clk]       \
    [get_bd_pins aurora_data_channel_0/aurora_clk] \
    [get_bd_pins aurora_ctrl_channel_0/aurora_clk] \
    [get_bd_pins aurora_tx1_arbiter_0/clk]         \
    [get_bd_pins aurora_rx_merge_0/clk]            \
    [get_bd_pins async_fifo_aurora_rx/wr_clk]    \
    [get_bd_pins async_fifo_aurora_tx/rd_clk]    \
    [get_bd_pins async_fifo_reply_tx/rd_clk]     \
    [get_bd_pins relay_fifo_0/clk]                \
    [get_bd_pins au_init_pulse_cdc_0/dst_clk]     \
    [get_bd_pins au_trig_pulse_cdc_0/dst_clk]     \
    [get_bd_pins au_reserve_pulse_cdc_0/dst_clk]  \
    [get_bd_pins au_ctrl_rst_pulse_cdc_0/dst_clk] \
    [get_bd_pins au_full_reset_pulse_cdc_0/dst_clk] \
    [get_bd_pins board_id_cdc_0/dst_clk]          \
    [get_bd_pins is_master_cdc_0/dst_clk]         \
    [get_bd_pins reserve_dest_id_cdc_0/dst_clk]   \
    [get_bd_pins hard_err_cdc_0/dst_clk]          \
    [get_bd_pins hard_err_cdc_1/dst_clk]          \
    [get_bd_pins hard_err_cdc_0/src_clk]          \
    [get_bd_pins hard_err_cdc_1/src_clk]          \
    [get_bd_pins hard_err_sticky_0/clk]           \
    [get_bd_pins hard_err_sticky_1/clk]           \
    [get_bd_pins reserve_fwd_sticky_0/clk]        \
    [get_bd_pins reserve_timeout_sticky_0/clk]    \
    [get_bd_pins reserve_reply_sticky_0/clk]      \
    [get_bd_pins reserve_pulse_sticky_0/clk]      \
    [get_bd_pins reserve_fwd_cdc_0/src_clk]       \
    [get_bd_pins reserve_timeout_cdc_0/src_clk]   \
    [get_bd_pins reserve_reply_cdc_0/src_clk]     \
    [get_bd_pins reserve_pulse_cdc_0/src_clk]     \
    [get_bd_pins reply_seen_cdc_0/src_clk]        \
    [get_bd_pins reply_granted_cdc_0/src_clk]     \
    [get_bd_pins init_ok_cdc_0/src_clk]           \
    [get_bd_pins total_boards_cdc_0/src_clk]      \
    [get_bd_pins board_index_cdc_0/src_clk]       \
    [get_bd_pins per_hop_value_cdc_0/src_clk]     \
    [get_bd_pins per_hop_value_eff_cdc_0/src_clk] \
    [get_bd_pins bid_per_hop_cdc_0/dst_clk]       \
    [get_bd_pins bid_total_boards_cdc_0/dst_clk]  \
    [get_bd_pins manual_trig_delay_cdc_0/dst_clk]        \
    [get_bd_pins manual_trig_delay_active_cdc_0/dst_clk] \
    [get_bd_pins manual_total_boards_cdc_0/dst_clk]      \
    [get_bd_pins manual_per_hop_cdc_0/dst_clk]           \
    [get_bd_pins manual_per_hop_active_cdc_0/dst_clk]    \
    [get_bd_pins au_trig_group_cdc_0/dst_clk]     \
    [get_bd_pins native_trig_delay_cdc_0/src_clk] \
    [get_bd_pins trig_fire_fifo_0/wr_clk]         \
    [get_bd_pins reserve_ok_cdc_0/src_clk]        \
    [get_bd_pins reserve_busy_cdc_0/src_clk]      \
    [get_bd_pins channel_up0_cdc_0/src_clk]       \
    [get_bd_pins channel_up1_cdc_0/src_clk]       \
    [get_bd_pins aurora_nfc_ctrl_0/aurora_clk]

# aurora_64b66b_0/sys_reset_out -> 新三個模組的 rst（都在 aurora_clk domain，
# 同一份 reset）+ async_fifo_aurora_rx/rst（見下方 rst domain 說明）
#
# **rst domain 陷阱**（見 feedback_fifo_generator_rst_cdc 既有教訓）：
# fifo_generator Independent_Clocks 的 rst 必須 synchronous to wr_clk，不能
# 隨便接錯邊——async_fifo_aurora_rx 的 wr_clk 是 aurora_clk（Layer 2 寫入），
# 所以 rst 用這個 aurora_clk domain 的 reset 是對的；但 async_fifo_aurora_tx
# 的 wr_clk 是 sys_clk（dispatcher 寫入），rst 必須用 sys_clk domain 的
# reset（見下面 peripheral_reset 那段），不能跟這裡混在一起——不然啟動時
# 會有 empty=0 但 dout=0 的假訊號，下游可能死鎖。
# 2026-07-08 第五輪（debug-only 實驗）：原本這五個模組共用
# aurora_64b66b_0/sys_reset_out 當 reset，現在改成跟 TI bit30 觸發的
# 額外 reset pulse（見上方 au_ctrl_rst_pulse_cdc_0/or_aurora_extra_rst）
# 用 OR 合併——host 可以在 channel_up 確認穩定「之後」手動再補一次
# reset，測試能不能排除 reserve 隨機失敗的問題。五個模組一起重置（不是
# 只挑 ctrl_channel/data_channel），避免跟 tx1_arbiter/rx_merge 之間的
# 狀態不一致。
connect_bd_net [get_bd_pins aurora_64b66b_0/sys_reset_out] [get_bd_pins or_aurora_extra_rst/Op1]
connect_bd_net [get_bd_pins au_ctrl_rst_pulse_cdc_0/dst_pulse] [get_bd_pins or_aurora_extra_rst/Op2]
connect_bd_net [get_bd_pins ti_au_ctrl_rst/Dout] [get_bd_pins au_ctrl_rst_pulse_cdc_0/src_pulse]
connect_bd_net [get_bd_pins or_aurora_extra_rst2/Res] \
    [get_bd_pins async_fifo_aurora_rx/rst]  \
    [get_bd_pins relay_fifo_0/srst]         \
    [get_bd_pins trig_fire_fifo_0/rst]      \
    [get_bd_pins aurora_data_channel_0/rst] \
    [get_bd_pins aurora_ctrl_channel_0/rst] \
    [get_bd_pins aurora_tx1_arbiter_0/rst]  \
    [get_bd_pins aurora_rx_merge_0/rst]     \
    [get_bd_pins aurora_nfc_ctrl_0/rst]

# 2026-08-20 Phase 1（Opus 覆核發現，見 PROJECT.md「Aurora reset 架構
# 問題」章節）：hard_err_sticky_0/1、reserve_*_sticky_0 這 6 個 sticky
# latch 存在的目的是「記住發生過的事件」，但原本 rst 接
# or_aurora_extra_rst2/Res——這條線包含 aurora_64b66b_0/sys_reset_out，
# 而 sys_reset_out 正是這些 latch 想記住的那類事件（連線沒鎖定/hard_err）
# 造成的，等於被自己要監測的事件清掉，自我否定設計。改接更穩定的
# rst_ctrl（peripheral_reset 經 CDC 過來 OR ti_au_full_reset），只在真正
# 開機/host 手動觸發全部重置時才清，不會跟著 GT 每次重訓一起被清掉。
create_bd_cell -type module -reference level_cdc rst_ctrl_cdc_0
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_rst_ctrl
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_rst_ctrl]
connect_bd_net [get_bd_pins clk_wiz_0/clk_100]           [get_bd_pins rst_ctrl_cdc_0/src_clk]
connect_bd_net [get_bd_pins aurora_64b66b_0/user_clk_out] [get_bd_pins rst_ctrl_cdc_0/dst_clk]
connect_bd_net [get_bd_pins rst_clk100/peripheral_reset] [get_bd_pins rst_ctrl_cdc_0/src_in]
connect_bd_net [get_bd_pins rst_ctrl_cdc_0/dst_out]            [get_bd_pins or_rst_ctrl/Op1]
connect_bd_net [get_bd_pins au_full_reset_pulse_cdc_0/dst_pulse] [get_bd_pins or_rst_ctrl/Op2]
connect_bd_net [get_bd_pins or_rst_ctrl/Res] \
    [get_bd_pins hard_err_sticky_0/rst]        \
    [get_bd_pins hard_err_sticky_1/rst]        \
    [get_bd_pins reserve_fwd_sticky_0/rst]     \
    [get_bd_pins reserve_timeout_sticky_0/rst] \
    [get_bd_pins reserve_reply_sticky_0/rst]   \
    [get_bd_pins reserve_pulse_sticky_0/rst]

connect_bd_net [get_bd_pins hard_err_cdc_0/dst_out] [get_bd_pins hard_err_sticky_0/set_in]
connect_bd_net [get_bd_pins hard_err_cdc_1/dst_out] [get_bd_pins hard_err_sticky_1/set_in]

connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_reserve_fwd_pending] [get_bd_pins reserve_fwd_sticky_0/set_in]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_reserve_timeout_hit] [get_bd_pins reserve_timeout_sticky_0/set_in]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_reserve_reply_hit]   [get_bd_pins reserve_reply_sticky_0/set_in]

# reserve_start_pulse 是 aurora_ctrl_channel_0 的 input pin（host 端 CDC 產生的
# 觸發脈波)——這裡直接把 sticky latch 的 set_in 掛到同一條 net 上當第二個
# load，藉此驗證這個脈波「有沒有真的送到」ctrl_channel，不跟 fwd_pending 共用。
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/reserve_start_pulse]      [get_bd_pins reserve_pulse_sticky_0/set_in]
connect_bd_net [get_bd_pins reserve_fwd_sticky_0/sticky_out]     [get_bd_pins reserve_fwd_cdc_0/src_in]
connect_bd_net [get_bd_pins reserve_timeout_sticky_0/sticky_out] [get_bd_pins reserve_timeout_cdc_0/src_in]
connect_bd_net [get_bd_pins reserve_reply_sticky_0/sticky_out]   [get_bd_pins reserve_reply_cdc_0/src_in]
connect_bd_net [get_bd_pins reserve_pulse_sticky_0/sticky_out]   [get_bd_pins reserve_pulse_cdc_0/src_in]

# -- Aurora 64B/66B control -------------------------------------------------------
# 2026-07-17：aurora_64b66b_0 的 mmcm_not_locked（input）被 SupportLevel=1
# 拿掉了（改成自己輸出 mmcm_not_locked_out），這條線只保留 _0/reset_pb、
# _1/reset_pb、_1/mmcm_not_locked。
connect_bd_net [get_bd_pins clk_wiz_0/locked] [get_bd_pins inv_mmcm_locked/Op1]
connect_bd_net [get_bd_pins inv_mmcm_locked/Res] \
    [get_bd_pins aurora_64b66b_0/reset_pb] \
    [get_bd_pins aurora_64b66b_1/reset_pb] \
    [get_bd_pins aurora_64b66b_1/mmcm_not_locked]
# 2026-07-22：pma_init 原本寫死接 const_zero（永遠不觸發），改接
# au_pma_init_pulse_cdc_0/dst_pulse（TI bit7 觸發，見 ti_au_full_
# reset 建立處說明）——host 可以在不重新配置 FPGA、不斷開 JTAG 的
# 情況下，強制 GT 收發器重新做一次實體層鏈路訓練。官方文件確認只需要
# 一個短暫的 pulse，核心內部會自動延展成正確的重置時間長度，不需要
# 我們自己做計時器保持訊號。
connect_bd_net [get_bd_pins au_pma_init_pulse_cdc_0/dst_pulse] \
    [get_bd_pins aurora_64b66b_0/pma_init] \
    [get_bd_pins aurora_64b66b_1/pma_init]
connect_bd_net [get_bd_pins const_3b0/dout] \
    [get_bd_pins aurora_64b66b_0/loopback] \
    [get_bd_pins aurora_64b66b_1/loopback]
connect_bd_net [get_bd_pins const_zero/dout] \
    [get_bd_pins aurora_64b66b_0/power_down] \
    [get_bd_pins aurora_64b66b_1/power_down]
connect_bd_net [get_bd_pins const_zero/dout] \
    [get_bd_pins aurora_64b66b_0/gt_rxcdrovrden_in] \
    [get_bd_pins aurora_64b66b_1/gt_rxcdrovrden_in]
connect_bd_net [get_bd_pins clk_wiz_0/clk_aurora_init] \
    [get_bd_pins aurora_64b66b_0/init_clk] \
    [get_bd_pins aurora_64b66b_1/init_clk] \
    [get_bd_pins au_pma_init_pulse_cdc_0/dst_clk]

# -- SFP physical ports -------------------------------------------------------------
connect_bd_net [get_bd_pins aurora_64b66b_0/txp] [get_bd_ports sfp1_tx_p]
connect_bd_net [get_bd_pins aurora_64b66b_0/txn] [get_bd_ports sfp1_tx_n]
connect_bd_net [get_bd_pins aurora_64b66b_0/rxp] [get_bd_ports sfp1_rx_p]
connect_bd_net [get_bd_pins aurora_64b66b_0/rxn] [get_bd_ports sfp1_rx_n]

connect_bd_net [get_bd_pins aurora_64b66b_1/txp] [get_bd_ports sfp2_tx_p]
connect_bd_net [get_bd_pins aurora_64b66b_1/txn] [get_bd_ports sfp2_tx_n]
connect_bd_net [get_bd_pins aurora_64b66b_1/rxp] [get_bd_ports sfp2_rx_p]
connect_bd_net [get_bd_pins aurora_64b66b_1/rxn] [get_bd_ports sfp2_rx_n]

# 2026-08-05 新增：SFP TX_DISABLE 固定拉低致能雷射，見 TDIS_1/TDIS_2
# port 建立處註解。
connect_bd_net [get_bd_pins const_tdis_1/dout] [get_bd_ports TDIS_1]
connect_bd_net [get_bd_pins const_tdis_2/dout] [get_bd_ports TDIS_2]

# -- FrontPanel WI -----------------------------------------------------------------
# 2026-07-27：WI 0x0D（board_id/is_master 本機直寫）已移除，board_id/
# is_master 收斂到 T_BOARD_ID_ASSIGN(0x1E) 一條路徑。

# WI 0x11（2026-07-07 step 15b：拿掉，`reserve_dest_id` 改走
# T_RESERVE_START(0x1C) 封包的 beat1，不再直接接 WI）
# 2026-07-08 第四輪（debug-only，暫時加回來）
connect_bd_net [get_bd_pins fp0/wi11_ep_dataout] [get_bd_pins reserve_dest_id_slice/Din]

# 2026-07-27：WI 0x18（ext clock select 本機直寫暫存區）已移除，只剩
# Aurora T_EXT_CLK_SEL(0x28) 一條路徑。
connect_bd_net [get_bd_pins local_reg_handler_0/au_ext_clk_sel]    [get_bd_pins board_cfg_reg_0/au_ext_clk_sel]
connect_bd_net [get_bd_pins local_reg_handler_0/au_ext_clk_sel_wr] [get_bd_pins board_cfg_reg_0/au_ext_clk_sel_wr]

# WI 0x1a bits[1:0]：flash_target_sel（2026-07-15 新增）
connect_bd_net [get_bd_pins fp0/wi1a_ep_dataout] [get_bd_pins flash_target_sel_slice/Din]
connect_bd_net [get_bd_pins flash_target_sel_slice/Dout] [get_bd_pins flash_target_sel_reg_0/fp_sel]
# 2026-07-27：ti_scale_wr → board_cfg_reg_0/fp_scale_wr 這條已移除，
# scale_cfg 只剩 Aurora T_SCALE_CFG(0x07) 一條路徑。
connect_bd_net [get_bd_pins ti_flash_target_sel_wr/Dout] [get_bd_pins flash_target_sel_reg_0/fp_wr]
# 2026-07-29 新增：ti_debug_reply_trig（bit12）→ local_reg_handler_0，
# 取代已刪除的 T_DEBUG_REPLY_INJECT(0x1F) 封包機制
connect_bd_net [get_bd_pins ti_debug_reply_trig/Dout] [get_bd_pins local_reg_handler_0/ti_debug_reply_trig]

# WI 0x12: DDR4 read address (fp_ddr4_rw_1/wi_addr)
connect_bd_net [get_bd_pins fp0/wi12_ep_dataout] [get_bd_pins fp_ddr4_rw_1/wi_addr]

# -- TI bus ---------------------------------------------------------------------
# 2026-07-07 (step 15b): ti_au_init/ti_au_trig/ti_au_reserve（bit 29/30/31）
# 已拿掉，host 觸發 Aurora 多板協定改走封包（見 local_reg_handler_0/
# au_enum_start 等）
# 2026-07-27：ti_board_cfg_wr/ti_scale_wr/ti_ext_clk_sel_wr/ti_dac_
# mode_ramp_wr 這 4 個從這個 fan-out 清單移除（統一讀取/寫入架構
# 收斂，見 PORTS.md 表2）。
connect_bd_net [get_bd_pins fp0/ti40_ep_trigger] \
    [get_bd_pins ti_fp_align/Din]      \
    [get_bd_pins ti_au_reserve/Din]    \
    [get_bd_pins ti_au_ctrl_rst/Din]   \
    [get_bd_pins ti_au_full_reset/Din] \
    [get_bd_pins ti_flash_ctrl_rst/Din] \
    [get_bd_pins ti_fifo_rst/Din]      \
    [get_bd_pins ti_reinit/Din]        \
    [get_bd_pins ti_per_hop_value_wr/Din] \
    [get_bd_pins ti_flash_target_sel_wr/Din] \
    [get_bd_pins ti_debug_reply_trig/Din] \
    [get_bd_pins fp_ddr4_rw_1/ti_cmd]

# -- 2026-07-09：PI 0x80 直接寫入路徑整個移除（見 PROJECT.md 第 25 節）。
# flash payload 改走 dispatcher_0 新的 RT_FLASH 路由（見 dispatcher.v
# type 0x14），經 flash_payload_cdc_0 做 CDC + 拆寬度後送進
# fpga_flash_ctrl_0（okClk）。fp0 的 CONFIG.PI.COUNT/ADDR_0 設定也已移除
# （見上方 fp0 IP 設定）。
connect_bd_net [get_bd_pins dispatcher_0/flash_tdata]      [get_bd_pins flash_payload_cdc_0/in_data]
connect_bd_net [get_bd_pins dispatcher_0/flash_tvalid]     [get_bd_pins flash_payload_cdc_0/in_valid]
connect_bd_net [get_bd_pins flash_payload_cdc_0/in_ready]  [get_bd_pins dispatcher_0/flash_tready]
connect_bd_net [get_bd_pins flash_payload_cdc_0/out_data]  [get_bd_pins fpga_flash_ctrl_0/pipe_wdata]
connect_bd_net [get_bd_pins flash_payload_cdc_0/out_valid] [get_bd_pins fpga_flash_ctrl_0/pipe_wvalid]

# -- BTPipeIn 0x81 -> fp_input_wr_0 ------------------------------------------------
connect_bd_net [get_bd_pins fp0/btpi81_ep_dataout] [get_bd_pins fp_input_wr_0/pi_data]
connect_bd_net [get_bd_pins fp0/btpi81_ep_write]   [get_bd_pins fp_input_wr_0/pi_write]
connect_bd_net [get_bd_pins fp_input_wr_0/ep_ready] [get_bd_pins fp0/btpi81_ep_ready]

# 2026-07-04: fp0/okClk -- the clock BTPI's ep_dataout/ep_write/ep_ready/ep_blockstrobe
# are actually synchronous to (confirmed via the official BD-flow reference design at
# FrontPanel-Vivado-IP-Dist-v1.0.6/.../exampledesigns/flow-blockdesigner/pipetest/
# bd_pipetest_project_tcl.ttcl, which wires okClk directly to pipe_in_check/clk).
# fp_input_wr_0's pair-acc + fp_fifo_0's write side now run on this clock; FIFO's
# async read side stays on clk_wiz_0/clk_100 (the big sys_clk fan-out net above,
# unchanged). Verified in awg-test-step-14.3c's isolation test: debug_cap_cnt went
# from 508/512 (missing 4 beats) to 512/512 (content-verified correct) after adding
# this connection.
connect_bd_net [get_bd_pins fp0/okClk]              [get_bd_pins fp_input_wr_0/ok_clk]
connect_bd_net [get_bd_pins fp0/okClk]              [get_bd_pins fp_fifo_0/wr_clk]
# 2026-07-29 新增 v2：PO_DIAG(0xA0)/PO_STATUS_REPLY(0xA1) CDC 修法——
# diag_capture_0/status_reply_capture_0 內部用 xpm_cdc_handshake 把
# 整張表搬進 okClk 域自己的暫存器組，po_ep_datain/po_ep_read 這條
# handshake 介面完全留在 okClk 域，這裡只需要把 okClk 本身跟同步過的
# reset 接進去，不需要中間的 FIFO wrapper。
connect_bd_net [get_bd_pins fp0/okClk] \
    [get_bd_pins diag_capture_0/ok_clk]          \
    [get_bd_pins status_reply_capture_0/ok_clk]  \
    [get_bd_pins reply_diag_okclk_rst_sync_0/ok_clk]
connect_bd_net [get_bd_pins reply_diag_okclk_rst_sync_0/rst_ok] \
    [get_bd_pins diag_capture_0/ok_rst]          \
    [get_bd_pins status_reply_capture_0/ok_rst]
# 2026-07-09：fpga_flash_ctrl_0 搬到 okClk 域，見 PROJECT.md 第 25 節
connect_bd_net [get_bd_pins fp0/okClk] \
    [get_bd_pins fpga_flash_ctrl_0/clk]      \
    [get_bd_pins okclk_rst_sync_0/ok_clk]    \
    [get_bd_pins flash_payload_cdc_0/ok_clk] \
    [get_bd_pins flash_erase_pulse_cdc_0/dst_clk] \
    [get_bd_pins flash_load_valid_cdc_0/src_clk] \
    [get_bd_pins flash_scale_load_valid_cdc_0/src_clk] \
    [get_bd_pins flash_amp_load_valid_cdc_0/src_clk]   \
    [get_bd_pins flash_coef_load_valid_cdc_0/src_clk]  \
    [get_bd_pins flash_target_sel_cdc_0/dst_clk]       \
    [get_bd_pins flash_status_cdc_0/src_clk]           \
    [get_bd_pins flash_scale_cfg_cdc_0/src_clk]        \
    [get_bd_pins flash_coef_all_cdc_0/src_clk]
# btpi81_ep_blockstrobe intentionally left unconnected -- not used by fp_input_wr_0's
# current packet framing (dispatcher/ddr_writer parse beat0/beat1 themselves, no
# reliance on BTPipeIn block boundaries). Confirmed port names from the official
# code-gen template (param_loop.ttcl, wrapper_masterside / endpoint_instantiations),
# not guessed -- verify with `get_bd_pins -of_objects [get_bd_cells fp0]` after rebuild
# regardless, per project convention.
# fp_input_wr_0/half_reset 改由 or_fp_half_reset/Res 驅動（見上方，OR 進了
# 新的 TI bit28 ti_fifo_rst），不再直接接 ti_fp_align/Dout。

# -- fp_input_wr_0 <-> fp_fifo_0（寫入端 CDC 邊界，2026-07-10 拆分新增）------------
connect_bd_net [get_bd_pins fp_input_wr_0/fifo_din]        [get_bd_pins fp_fifo_0/din]
connect_bd_net [get_bd_pins fp_input_wr_0/fifo_wr_en]      [get_bd_pins fp_fifo_0/wr_en]
connect_bd_net [get_bd_pins fp_fifo_0/prog_full]           [get_bd_pins fp_input_wr_0/fifo_prog_full]
connect_bd_net [get_bd_pins fp_fifo_0/wr_rst_busy]         [get_bd_pins fp_input_wr_0/fifo_wr_rst_busy]

# -- fp_fifo_0 <-> fp_input_rd_0（讀出端 CDC 邊界，2026-07-10 拆分新增）-----------
connect_bd_net [get_bd_pins fp_fifo_0/dout]                [get_bd_pins fp_input_rd_0/fifo_dout]
connect_bd_net [get_bd_pins fp_fifo_0/empty]               [get_bd_pins fp_input_rd_0/fifo_empty]
connect_bd_net [get_bd_pins fp_input_rd_0/rd_en]           [get_bd_pins fp_fifo_0/rd_en]
connect_bd_net [get_bd_pins ti_fp_align/Dout]    [get_bd_pins local_reg_handler_0/half_reset]

# -- board_cfg_reg_0 ----------------------------------------------------------------
# 2026-07-27：fp_board_cfg_wr/fp_board_id/fp_is_master（本機直寫路徑）
# 這 3 條已移除，board_id/is_master 收斂到 T_BOARD_ID_ASSIGN(0x1E) 一
# 條路徑（見下方擴充後的接線）。
# 2026-07-08 (step 15b): flash controller 接回來，取代原本的 const_zero tie-off
# 2026-07-09：fpga_flash_ctrl_0/load_valid 現在是 okClk 域的單週期 pulse，
# 過 flash_load_valid_cdc_0（trigger_cdc）同步成 sys_clk 域再扇出給這裡
# 跟 local_reg_handler_0/aurora_ctrl_mux_0/awg_calib_regs_0（見 PROJECT.md
# 第 25 節）。init_board_id/init_is_master/init_scale_cfg 這組 quasi-static
# 資料直接接線不變（已核對安全，見 PROJECT.md 第 25 節查證紀錄；
# sys_clk<->mmcm0_clk0 之間已有 set_false_path，見 awg_step16.xdc:289-290）
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/load_valid]  [get_bd_pins flash_load_valid_cdc_0/src_pulse]
connect_bd_net [get_bd_pins flash_load_valid_cdc_0/dst_pulse] [get_bd_pins board_cfg_reg_0/flash_load_valid]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/init_board_id]  [get_bd_pins board_cfg_reg_0/flash_board_id]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/init_is_master] [get_bd_pins board_cfg_reg_0/flash_is_master]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/init_scale_cfg] [get_bd_pins board_cfg_reg_0/flash_scale_cfg]
# 2026-07-27：au_board_cfg_wr/au_board_cfg_id/au_board_cfg_is_master
# （T_BOARD_CFG=0x10 相關）這 3 條已移除——確認從未被任何腳本用來設定
# 過 is_master，純粹死路，board_id/is_master 收斂到 T_BOARD_ID_
# ASSIGN(0x1E) 一條路徑（見下方擴充後的接線）。

# 2026-07-15：scale_cfg 現在存在獨立 sector，讀取完成時間點跟身份 sector
# 不同，用專屬的 flash_scale_load_valid_cdc_0（不能沿用上面身份用的
# flash_load_valid_cdc_0，見 board_cfg_reg.v 檔頭「2026-07-15（同一輪）」
# 說明）
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/load_valid_scale] [get_bd_pins flash_scale_load_valid_cdc_0/src_pulse]
connect_bd_net [get_bd_pins flash_scale_load_valid_cdc_0/dst_pulse] [get_bd_pins board_cfg_reg_0/flash_scale_load_valid]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/load_valid_amp] [get_bd_pins flash_amp_load_valid_cdc_0/src_pulse]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/load_valid_coef] [get_bd_pins flash_coef_load_valid_cdc_0/src_pulse]

# Aurora 覆寫 scale_cfg（2026-07-15 新增，board_cfg_reg_0 的 au_scale_cfg
# 是 8-bit，local_reg_handler_0/au_scale_cfg 是既有的 32-bit port，沿用
# 既有 SCALE_CFG(0x07) 封包解碼結果，不需要新的封包類型）
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice au_scale_cfg_slice_bcfg
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {7} CONFIG.DIN_TO {0}] [get_bd_cells au_scale_cfg_slice_bcfg]
connect_bd_net [get_bd_pins local_reg_handler_0/au_scale_cfg_wr] [get_bd_pins board_cfg_reg_0/au_scale_wr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_scale_cfg]    [get_bd_pins au_scale_cfg_slice_bcfg/Din]
connect_bd_net [get_bd_pins au_scale_cfg_slice_bcfg/Dout]        [get_bd_pins board_cfg_reg_0/au_scale_cfg]

# flash_target_sel（2026-07-15 新增）：host WI/TI 跟 Aurora
# T_FLASH_TARGET_SEL(0x15) 都寫進 flash_target_sel_reg_0，結果經
# level_cdc（sys_clk -> okClk）送進 fpga_flash_ctrl_0/target_sector_sel
connect_bd_net [get_bd_pins local_reg_handler_0/au_flash_target_sel_wr] [get_bd_pins flash_target_sel_reg_0/au_wr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_flash_target_sel]    [get_bd_pins flash_target_sel_reg_0/au_sel]
connect_bd_net [get_bd_pins flash_target_sel_reg_0/sel] [get_bd_pins flash_target_sel_cdc_0/src_in]
connect_bd_net [get_bd_pins flash_target_sel_cdc_0/dst_out] [get_bd_pins fpga_flash_ctrl_0/target_sector_sel]

# 2026-07-14 新增：T_BOARD_ID_ASSIGN(0x1E) -> board_id <= board_index。
# board_index_cdc_0/dst_out 是既有訊號（原本只接 concat_enum_status/In2 給
# WO 0x26 讀回用），這裡多接一條線 fan-out 給 board_cfg_reg_0，不動原本的
# create_bd_cell/連線（board_index_cdc_0 建立於本檔案前段，dst_out 的其餘
# 接線見下方 aurora enum 狀態章節）。
connect_bd_net [get_bd_pins local_reg_handler_0/au_board_id_assign] [get_bd_pins board_cfg_reg_0/au_board_id_assign_wr]
connect_bd_net [get_bd_pins board_index_cdc_0/dst_out]              [get_bd_pins board_cfg_reg_0/au_board_id_assign_value]
# 2026-07-27 新增（統一讀取/寫入架構收斂）：T_BOARD_ID_ASSIGN 擴充的
# beat1[37]=is_master，跟 board_index 同一拍生效，讓這個封包同時決定
# board_id 跟 is_master，成為 board 身份指定的唯一入口。
connect_bd_net [get_bd_pins local_reg_handler_0/bid_is_master] [get_bd_pins board_cfg_reg_0/au_board_id_assign_is_master]
# 2026-07-30 新增：broadcast 只更新 board_id、不動 is_master 的修法，
# 見 rtl/local_reg_handler.v bid_is_broadcast、rtl/board_cfg_reg.v
# au_board_id_assign_is_broadcast port 註解、NOTES.md 對應章節。
connect_bd_net [get_bd_pins local_reg_handler_0/bid_is_broadcast] [get_bd_pins board_cfg_reg_0/au_board_id_assign_is_broadcast]

connect_bd_net [get_bd_pins board_cfg_reg_0/board_id] \
    [get_bd_pins dispatcher_0/board_id]

# -- 三個新 Aurora 多板協定模組（取代 aurora_packet_tx_0/aurora_tx_arbiter_0）───
# board_id/is_master：跟 dispatcher_0 共用同一份 board_cfg_reg_0 輸出，但
# dispatcher_0 是 sys_clk domain（跟 board_cfg_reg_0 同域，不需要 CDC），
# aurora_data_channel_0/aurora_ctrl_channel_0 是 aurora_clk domain，跨域必須
# 經過 board_id_cdc_0/is_master_cdc_0（2026-07-06 bugfix，見上方 cell 建立
# 處的說明；原本直接接線沒有 CDC）
connect_bd_net [get_bd_pins board_cfg_reg_0/board_id]  [get_bd_pins board_id_cdc_0/src_in]
connect_bd_net [get_bd_pins board_id_cdc_0/dst_out] \
    [get_bd_pins aurora_data_channel_0/board_id] \
    [get_bd_pins aurora_ctrl_channel_0/board_id]
connect_bd_net [get_bd_pins board_cfg_reg_0/is_master] [get_bd_pins is_master_cdc_0/src_in]
connect_bd_net [get_bd_pins is_master_cdc_0/dst_out]   [get_bd_pins aurora_ctrl_channel_0/is_master]

# channel_up：兩顆實體 Aurora IP 的環路狀態，餵給 aurora_ctrl_channel_0 這邊
# 本來就是 aurora_clk 同域（aurora_64b66b_0/1 的 channel_up 本身就在
# aurora_clk），不需要 CDC，維持直接接線
connect_bd_net [get_bd_pins aurora_64b66b_0/channel_up] [get_bd_pins aurora_ctrl_channel_0/channel_up_0]
connect_bd_net [get_bd_pins aurora_64b66b_1/channel_up] [get_bd_pins aurora_ctrl_channel_0/channel_up_1]
# 2026-07-29 新增：aurora_data_channel_0（relayfifo）新增的 channel_up_0
# 閘控，同一個來源，見 rtl/aurora_data_channel_relayfifo.v port 註解
connect_bd_net [get_bd_pins aurora_64b66b_0/channel_up] [get_bd_pins aurora_data_channel_0/channel_up_0]

# host 觸發（2026-07-07 step 15b：改成走封包，host->dispatcher_0->
# local_reg_handler_0 解碼 T_ENUM_START(0x1A)/T_TRIG_START(0x1B)/
# T_RESERVE_START(0x1C)，取代原本 15a 的 TI bit 29/30/31 + WI 0x11 直接
# 接線）：sys_clk -> aurora_clk 還是要走 CDC（local_reg_handler_0 本身在
# sys_clk domain，這段 CDC 的必要性跟 15a 一樣沒變，只是 pulse 的來源從
# TI bit 換成 local_reg_handler_0 的封包解碼輸出）。
connect_bd_net [get_bd_pins local_reg_handler_0/au_enum_start]    [get_bd_pins au_init_pulse_cdc_0/src_pulse]
connect_bd_net [get_bd_pins au_init_pulse_cdc_0/dst_pulse] [get_bd_pins aurora_ctrl_channel_0/init_start_pulse]
          # au_trig_pulse_cdc_0/src_pulse 2026-08-04 起改由 or_group_
          # sched_fire_0/Res 驅動（group_trig_scheduler_0 的 fire OR
          # 進 local_reg_handler_0/au_trig_start，見「group_trig_
          # scheduler_0 plumbing」章節），這行直連已移除，不是漏寫。
connect_bd_net [get_bd_pins au_trig_pulse_cdc_0/dst_pulse] [get_bd_pins aurora_ctrl_channel_0/trig_start_pulse]
# 2026-07-08 第四輪（debug-only）：封包路徑（au_reserve_start/dest_id）跟
# TI-direct 除錯路徑（ti_au_reserve/reserve_dest_id_slice）OR 在一起再進
# CDC。注意：dest_id 用 OR 合併是因為 au_reserve_dest_id 開機預設是 0、
# 只有真的收到過 0x1C 封包才會被寫入，所以「剛重開機、還沒送過封包版
# reserve」時直接測 TI-direct，OR 出來的值就等於 WI 0x11 的值，不會被
# 污染——這只是暫時的除錯接線，確認完後應該拔掉，不是正式架構。
connect_bd_net [get_bd_pins local_reg_handler_0/au_reserve_start] [get_bd_pins or_reserve_pulse/Op1]
connect_bd_net [get_bd_pins ti_au_reserve/Dout]                    [get_bd_pins or_reserve_pulse/Op2]
connect_bd_net [get_bd_pins or_reserve_pulse/Res]                  [get_bd_pins au_reserve_pulse_cdc_0/src_pulse]
connect_bd_net [get_bd_pins au_reserve_pulse_cdc_0/dst_pulse] [get_bd_pins aurora_ctrl_channel_0/reserve_start_pulse]
# 2026-07-08：au_reserve_start_sticky_0 直接 tap local_reg_handler_0 解碼出的
# au_reserve_start（sys_clk domain，CDC 之前）當第三個 load，跟上面的
# or_reserve_pulse/Op1、dispatcher_ila_0/probe12 並列，用來分辨封包解碼有沒有
# 真的觸發過，跟 aurora_clk domain 那邊的 reserve_pulse_sticky_0 對照。
connect_bd_net [get_bd_pins local_reg_handler_0/au_reserve_start] [get_bd_pins au_reserve_start_sticky_0/set_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_reserve_dest_id] [get_bd_pins or_reserve_dest_id/Op1]
connect_bd_net [get_bd_pins reserve_dest_id_slice/Dout]             [get_bd_pins or_reserve_dest_id/Op2]
connect_bd_net [get_bd_pins or_reserve_dest_id/Res]                 [get_bd_pins reserve_dest_id_cdc_0/src_in]
connect_bd_net [get_bd_pins reserve_dest_id_cdc_0/dst_out] [get_bd_pins aurora_ctrl_channel_0/reserve_dest_id]

# 2026-07-24 移除：trig_delay 開機從 flash 載入這條路徑（原本接
# flash_load_valid_cdc_0/dst_pulse -> local_reg_handler_0/flash_load_valid、
# fpga_flash_ctrl_0/init_trig_delay -> local_reg_handler_0/flash_trig_delay）。
# 上機實測發現這條路徑每次開機都會自動觸發，永久鎖住 manual_delay_
# override_active，導致自動計算補償永遠沒機會生效（見 rtl/local_reg_
# handler.v 對應 port 註解、PROJECT.md/NOTES.md 完整記錄）。local_reg_
# handler.v 的 flash_load_valid/flash_trig_delay 兩個 port 已移除。
# flash_load_valid_cdc_0 這顆 CDC 本身不動（board_cfg_reg_0/aurora_ctrl_
# mux_0/awg_calib_regs_0 還在用同一個 pulse 載入自己的 flash 欄位）；
# fpga_flash_ctrl_0/init_trig_delay 變成沒有消費者的孤兒 output，不需要
# tie-off（output 不像 input 需要驅動源，比照既有 out_scale_cfg 前例）。

# Layer 2 <-> Layer 3 內部介面（同板內，見 PORTS.md「三個新模組之間的內部接線」）
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/lock_req]   [get_bd_pins aurora_data_channel_0/lock_req]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/unlock_req] [get_bd_pins aurora_data_channel_0/unlock_req]
connect_bd_net [get_bd_pins aurora_data_channel_0/is_busy]    [get_bd_pins aurora_ctrl_channel_0/is_busy]

# aurora_data_channel_0 <-> relay_fifo_0 (added 2026-07-17, see the
# add_files/create_bd_cell comments above): streaming buffer for the relay
# section. The write side is a beat received on rx0 that's been decided to
# be forwarded (din={tlast,tdata}); the read side is a beat that TX_RELAY
# state needs to send to aurora_tx1_arbiter_0. FWFT mode means valid/dout
# always reflect what rd_ptr currently points to; relay_rd_en just
# "advances by one slot".
connect_bd_net [get_bd_pins aurora_data_channel_0/relay_wr_din] [get_bd_pins relay_fifo_0/din]
connect_bd_net [get_bd_pins aurora_data_channel_0/relay_wr_en]  [get_bd_pins relay_fifo_0/wr_en]
connect_bd_net [get_bd_pins relay_fifo_0/full]                  [get_bd_pins aurora_data_channel_0/relay_wr_full]
connect_bd_net [get_bd_pins relay_fifo_0/dout]                  [get_bd_pins aurora_data_channel_0/relay_rd_dout]
connect_bd_net [get_bd_pins relay_fifo_0/valid]                 [get_bd_pins aurora_data_channel_0/relay_rd_valid]
connect_bd_net [get_bd_pins aurora_data_channel_0/relay_rd_en]  [get_bd_pins relay_fifo_0/rd_en]

# rx0（forward，實體 aurora_64b66b_0 RX）fan-out 給 Layer 2 + Layer 3，
# 各自只認自己的 type（不重複處理，見 PORTS.md 說明）。
# 2026-07-30：rx0_tdata 內容維持兩邊都接原始訊號不變（Layer3 還是要
# 自己解出 type/src/dest），但 rx0_tvalid 不再讓 Layer3 直接看實體 GT
# 的 tvalid，改接 Layer2 過濾後的 rx0_valid_for_ctrl——根因：Layer3
# 的 FRX 狀態機沒有 framing，不知道現在是不是卡在 Layer2 自己的長封包
# 中間，會把長封包裡剛好符合 type+src/dest 格式的 payload word 誤判成
# 合法 beat0（2026-07-29/30 在 board B 上機發現的離奇 ENUM_COUNT）。詳見
# rtl/aurora_data_channel_relayfifo.v 的 rx0_valid_for_ctrl port 註解、
# NOTES.md 對應章節。
connect_bd_net [get_bd_pins aurora_64b66b_0/m_axi_rx_tdata]  \
    [get_bd_pins aurora_data_channel_0/rx0_tdata] \
    [get_bd_pins aurora_ctrl_channel_0/rx0_tdata]
connect_bd_net [get_bd_pins aurora_64b66b_0/m_axi_rx_tvalid] \
    [get_bd_pins aurora_data_channel_0/rx0_tvalid]
connect_bd_net [get_bd_pins aurora_data_channel_0/rx0_valid_for_ctrl] \
    [get_bd_pins aurora_ctrl_channel_0/rx0_tvalid]

# rx1（backward，實體 aurora_64b66b_1 RX）只有 Layer 3 用（ACK 走這個方向）
connect_bd_net [get_bd_pins aurora_64b66b_1/m_axi_rx_tdata]  [get_bd_pins aurora_ctrl_channel_0/rx1_tdata]
connect_bd_net [get_bd_pins aurora_64b66b_1/m_axi_rx_tvalid] [get_bd_pins aurora_ctrl_channel_0/rx1_tvalid]

# tx1（forward，實體 aurora_64b66b_1 TX）：Layer2 data_tx1_* + Layer3
# ctrl_tx1_* 先經仲裁器合併，仲裁器輸出才是真正的實體 tx1
connect_bd_net [get_bd_pins aurora_data_channel_0/data_tx1_tdata]  [get_bd_pins aurora_tx1_arbiter_0/data_tx1_tdata]
connect_bd_net [get_bd_pins aurora_data_channel_0/data_tx1_tvalid] [get_bd_pins aurora_tx1_arbiter_0/data_tx1_tvalid]
connect_bd_net [get_bd_pins aurora_data_channel_0/data_tx1_tlast]  [get_bd_pins aurora_tx1_arbiter_0/data_tx1_tlast]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/data_tx1_tready]  [get_bd_pins aurora_data_channel_0/data_tx1_tready]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx1_tdata]  [get_bd_pins aurora_tx1_arbiter_0/ctrl_tx1_tdata]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx1_tvalid] [get_bd_pins aurora_tx1_arbiter_0/ctrl_tx1_tvalid]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx1_tlast]  [get_bd_pins aurora_tx1_arbiter_0/ctrl_tx1_tlast]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/ctrl_tx1_tready]  [get_bd_pins aurora_ctrl_channel_0/ctrl_tx1_tready]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/tx1_tdata]        [get_bd_pins aurora_64b66b_1/s_axi_tx_tdata]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/tx1_tvalid]       [get_bd_pins aurora_64b66b_1/s_axi_tx_tvalid]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/tx1_tlast]        [get_bd_pins aurora_64b66b_1/s_axi_tx_tlast]
connect_bd_net [get_bd_pins aurora_64b66b_1/s_axi_tx_tready]       [get_bd_pins aurora_tx1_arbiter_0/tx1_tready]
connect_bd_net [get_bd_pins const_8hff/dout] [get_bd_pins aurora_64b66b_1/s_axi_tx_tkeep]

# tx0（backward，實體 aurora_64b66b_0 TX）：只有 Layer3 的 ACK/回覆會用，
# 不用跟 data 搶（Layer2 不碰 tx0），直接接實體 TX，不經仲裁器
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx0_tdata]  [get_bd_pins aurora_64b66b_0/s_axi_tx_tdata]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx0_tvalid] [get_bd_pins aurora_64b66b_0/s_axi_tx_tvalid]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx0_tlast]  [get_bd_pins aurora_64b66b_0/s_axi_tx_tlast]
connect_bd_net [get_bd_pins aurora_64b66b_0/s_axi_tx_tready]       [get_bd_pins aurora_ctrl_channel_0/ctrl_tx0_tready]
connect_bd_net [get_bd_pins const_8hff/dout] [get_bd_pins aurora_64b66b_0/s_axi_tx_tkeep]

# -- dispatcher_0 ------------------------------------------------------------------
# RX path: 2026-07-05 (step 15a) 復原，上游改接 aurora_rx_merge_0 的合併
# 輸出（本地送達的才會出現在這裡，轉送的不會，見 PORTS.md）——這個合併器
# 把 Layer 2 的 local_tdata（真實本地送達，優先權最高不能延遲）跟 Layer 3
# 的合成 TRIGGER(0x01) 封包（local_inject_*，data idle 時才輪得到）合流成
# 單一輸出，這樣不管 trigger 是本機發起還是收到別人的，都走這同一段
# dispatcher 既有管線（使用者要求，見 PROJECT.md）。
connect_bd_net [get_bd_pins aurora_data_channel_0/local_out_tdata]     [get_bd_pins aurora_rx_merge_0/data_local_tdata]
connect_bd_net [get_bd_pins aurora_data_channel_0/local_out_tvalid]    [get_bd_pins aurora_rx_merge_0/data_local_tvalid]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/local_inject_tdata]  [get_bd_pins aurora_rx_merge_0/inject_tdata]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/local_inject_tvalid] [get_bd_pins aurora_rx_merge_0/inject_tvalid]
connect_bd_net [get_bd_pins aurora_rx_merge_0/inject_tready]           [get_bd_pins aurora_ctrl_channel_0/local_inject_tready]
connect_bd_net [get_bd_pins aurora_rx_merge_0/local_out_tdata]  [get_bd_pins async_fifo_aurora_rx/din]
connect_bd_net [get_bd_pins aurora_rx_merge_0/local_out_tvalid] [get_bd_pins async_fifo_aurora_rx/wr_en]
connect_bd_net [get_bd_pins async_fifo_aurora_rx/dout]  [get_bd_pins dispatcher_0/rx_tdata]
connect_bd_net [get_bd_pins async_fifo_aurora_rx/valid] [get_bd_pins dispatcher_0/rx_tvalid]
connect_bd_net [get_bd_pins dispatcher_0/rx_tready]     [get_bd_pins async_fifo_aurora_rx/rd_en]

# 2026-07-23 新增：au_trig_delay 全自動校準機制，dispatcher_0 把
# per_hop_value/total_boards 夾帶進 T_BOARD_ID_ASSIGN beat1（見
# rtl/dispatcher.v port 註解、NOTES.md「Aurora trigger 協定」章節）。
# total_boards 直接沿用既有的 total_boards_cdc_0（不用新建，跟
# concat_enum_status/In1 共用同一個 dst_out，見上方 CDC 接線處）。
connect_bd_net [get_bd_pins per_hop_value_cdc_0/dst_out] [get_bd_pins dispatcher_0/per_hop_value]
connect_bd_net [get_bd_pins total_boards_cdc_0/dst_out]  [get_bd_pins dispatcher_0/total_boards]

# 2026-08-20 新增：channel_up_1 接給 dispatcher_0，解決沒有 SFP 迴路時
# 廣播/跨板封包永久卡死 dispatcher_0 狀態機的問題（PROJECT.md「test1
# 面板」章節除錯過程）。原本直接接 aurora_64b66b_1/channel_up（誤判為
# 跟 dispatcher_0 同一個 sys_clk domain，其實是 aurora_clk，屬於未經
# CDC 的違規接線）——Opus 覆核 DIAG 設計時抓到，改接已存在、正確做過
# CDC 的 channel_up1_cdc_0/dst_out（sys_clk 側，本來就餵給 aurora_
# reply_tx_0/bi_channel_up_1，見下方 bi_channel_up_1 wiring）。
connect_bd_net [get_bd_pins channel_up1_cdc_0/dst_out] [get_bd_pins dispatcher_0/channel_up_1]

connect_bd_net [get_bd_pins fp_input_rd_0/fp_tdata]    [get_bd_pins dispatcher_0/fp_tdata]
connect_bd_net [get_bd_pins fp_input_rd_0/fp_tvalid]   [get_bd_pins dispatcher_0/fp_tvalid]
connect_bd_net [get_bd_pins dispatcher_0/fp_tready]    [get_bd_pins fp_input_rd_0/fp_tready]

# DDR path: dispatcher -> ddr_writer_0 (same sys_clk)
connect_bd_net [get_bd_pins dispatcher_0/ddr_tdata]   [get_bd_pins ddr_writer_0/wave_in_tdata]
connect_bd_net [get_bd_pins dispatcher_0/ddr_tvalid]  [get_bd_pins ddr_writer_0/wave_in_tvalid]
connect_bd_net [get_bd_pins ddr_writer_0/wave_in_tready]   [get_bd_pins dispatcher_0/ddr_tready]
# raw_rd_addr (diagnostic use)：2026-07-15 從 const_7b0 tie-off 改接
# WI 0x1e（host 可以選要讀哪個 raw beat），排查 T_WAVEFORM_STREAM 寫入
# DDR4 內容錯位問題
connect_bd_net [get_bd_pins fp0/wi1e_ep_dataout]      [get_bd_pins raw_rd_addr_slice/Din]
connect_bd_net [get_bd_pins raw_rd_addr_slice/Dout]   [get_bd_pins ddr_writer_0/raw_rd_addr]
connect_bd_net [get_bd_pins ddr_writer_0/raw_rd_data_lo] [get_bd_pins fp0/wo38_ep_datain]
connect_bd_net [get_bd_pins ddr_writer_0/raw_rd_data_hi] [get_bd_pins fp0/wo39_ep_datain]

# Local path -> async_fifo_local (Common Clock sys_clk) -> local_reg_handler_0
#
# 2026-07-23（根因5修法）：wr_en 原本直接接 dispatcher_0/lrh_tvalid，
# backpressure（prog_full）期間 lrh_tvalid 撐著不放的每一拍都會被
# fifo_generator 的 native write 介面當成新寫入，同一筆資料重複寫進
# FIFO——跟 async_fifo_aurora_tx 下面那個是同一種接線 bug（見
# NOTES.md「Aurora Layer 2 資料轉送與 TX 仲裁」章節 2026-07-23 根因5
# 小節，用 sim/tb_async_fifo_tx_backpressure.v 實測 duplicate_write_
# count 從 521 降到 0 驗證過這個修法）。改成 wr_en = lrh_tvalid &&
# lrh_tready（等於 lrh_tvalid && !prog_full）。
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic and_localfifo_wr_en
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and}] [get_bd_cells and_localfifo_wr_en]
connect_bd_net [get_bd_pins dispatcher_0/lrh_tdata]     [get_bd_pins async_fifo_local/din]
connect_bd_net [get_bd_pins dispatcher_0/lrh_tvalid]    [get_bd_pins and_localfifo_wr_en/Op1]
connect_bd_net [get_bd_pins inv_local_fifo_full/Res]    [get_bd_pins and_localfifo_wr_en/Op2]
connect_bd_net [get_bd_pins and_localfifo_wr_en/Res]    [get_bd_pins async_fifo_local/wr_en]
connect_bd_net [get_bd_pins async_fifo_local/prog_full]  [get_bd_pins inv_local_fifo_full/Op1]
connect_bd_net [get_bd_pins inv_local_fifo_full/Res]     [get_bd_pins dispatcher_0/lrh_tready]
connect_bd_net [get_bd_pins async_fifo_local/valid]      [get_bd_pins local_reg_handler_0/rx_tvalid]
connect_bd_net [get_bd_pins async_fifo_local/dout]       [get_bd_pins local_reg_handler_0/rx_tdata]
connect_bd_net [get_bd_pins async_fifo_local/valid]      [get_bd_pins async_fifo_local/rd_en]

# 2026-07-23 再追加：au_trig_delay 全自動校準機制的 trigger 準確度改版
# ——hops×per_hop_value 的計算 + 倒數整個搬到 aurora_ctrl_channel.v
# （aurora_clk domain）做（見該檔案 native_trig_out port 註解，原因是
# 原本 sys_clk 這個中間跨域會讓殘留相位差每次觸發都不一樣）。
# local_reg_handler_0 不再需要 board_index（拿掉這個 port），改成把
# T_BOARD_ID_ASSIGN beat1 收到的 bid_per_hop_value/bid_total_boards
# 原始值 CDC 回 aurora_clk 給 aurora_ctrl_channel_0 用（方向跟其他
# level_cdc 相反：這次是 sys_clk -> aurora_clk）。
connect_bd_net [get_bd_pins local_reg_handler_0/bid_per_hop_value] [get_bd_pins bid_per_hop_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/bid_total_boards]  [get_bd_pins bid_total_boards_cdc_0/src_in]
connect_bd_net [get_bd_pins bid_per_hop_cdc_0/dst_out]      [get_bd_pins aurora_ctrl_channel_0/per_hop_value_in]
connect_bd_net [get_bd_pins bid_total_boards_cdc_0/dst_out] [get_bd_pins aurora_ctrl_channel_0/total_boards_in]

# 2026-07-24 新增（trigger 統一化架構改版）：T_TRIG_DELAY_CFG(0x1D)
# 手動覆寫路徑，local_reg_handler_0（sys_clk）-> aurora_ctrl_channel_0
# （aurora_clk），見 rtl/aurora_ctrl_channel.v manual_trig_delay_in/
# manual_trig_delay_active_in port 註解。
connect_bd_net [get_bd_pins local_reg_handler_0/au_trig_delay] [get_bd_pins manual_trig_delay_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/manual_delay_override_active] [get_bd_pins manual_trig_delay_active_cdc_0/src_in]
connect_bd_net [get_bd_pins manual_trig_delay_cdc_0/dst_out]        [get_bd_pins aurora_ctrl_channel_0/manual_trig_delay_in]
connect_bd_net [get_bd_pins manual_trig_delay_active_cdc_0/dst_out] [get_bd_pins aurora_ctrl_channel_0/manual_trig_delay_active_in]

# 2026-07-24 新增：單板 bench test 用（T_MANUAL_TOTAL_BOARDS=0x2B），見
# rtl/aurora_ctrl_channel.v manual_total_boards_in port 註解。
connect_bd_net [get_bd_pins local_reg_handler_0/au_manual_total_boards] [get_bd_pins manual_total_boards_cdc_0/src_in]
connect_bd_net [get_bd_pins manual_total_boards_cdc_0/dst_out] [get_bd_pins aurora_ctrl_channel_0/manual_total_boards_in]

# 2026-07-24 再追加：per_hop_value 手動覆寫（au_trig_delay 自動校準機制
# 除錯用），board_cfg_reg_0（sys_clk）-> aurora_ctrl_channel_0
# （aurora_clk），見 rtl/aurora_ctrl_channel.v manual_per_hop_in/
# manual_per_hop_active_in port 註解。
connect_bd_net [get_bd_pins board_cfg_reg_0/per_hop_value_manual] [get_bd_pins manual_per_hop_cdc_0/src_in]
connect_bd_net [get_bd_pins board_cfg_reg_0/manual_per_hop_active] [get_bd_pins manual_per_hop_active_cdc_0/src_in]
connect_bd_net [get_bd_pins manual_per_hop_cdc_0/dst_out]        [get_bd_pins aurora_ctrl_channel_0/manual_per_hop_in]
connect_bd_net [get_bd_pins manual_per_hop_active_cdc_0/dst_out] [get_bd_pins aurora_ctrl_channel_0/manual_per_hop_active_in]

# 2026-07-27 新增（Group-based Trigger 架構）：T_TRIG_START(0x1B) beat1
# 的 group_select，local_reg_handler_0（sys_clk）-> aurora_ctrl_channel_0
# （aurora_clk），見 rtl/aurora_ctrl_channel.v au_trig_group_select_in
# port 註解。
# au_trig_group_cdc_0/src_in 2026-08-04 起改由 or_group_sched_group_0/
# Res 驅動（group_trig_scheduler_0 的 group_select_out OR 進
# local_reg_handler_0/au_trig_group_select，見「group_trig_scheduler_0
# plumbing」章節），上面那行直連已移除，不是漏寫。
connect_bd_net [get_bd_pins au_trig_group_cdc_0/dst_out] [get_bd_pins aurora_ctrl_channel_0/au_trig_group_select_in]

# TX relay path: 2026-07-05 (step 15a) 復原，下游改接 Layer 2 的
# local_tx_tdata/tvalid/tlast（取代原本接 aurora_tx_arbiter_0/wave_* 的做法，
# port 語意相同：本板自己要發起的資料）
#
# 2026-07-23（根因5修法，重大發現）：wr_en 原本直接接 dispatcher_0/
# tx_tvalid，backpressure（prog_full）期間 tx_tvalid 撐著不放的每一拍
# 都會被當成新寫入，同一筆資料重複寫進 FIFO——這完全對得上歷史
# 「548/1024 之後資料卡在固定值」的症狀特徵，很可能是 async_fifo_
# aurora_tx 死鎖/資料錯位問題的真正根因。用 sim/tb_async_fifo_tx_
# backpressure.v 實測：修法前 duplicate_write_count=521，修法後降到
# 0，recv_count/mismatch_count 都恢復正確。詳見 NOTES.md「Aurora
# Layer 2 資料轉送與 TX 仲裁」章節 2026-07-23 根因5小節。改成
# wr_en = tx_tvalid && tx_tready（等於 tx_tvalid && !prog_full）。
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic and_txfifo_wr_en
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {and}] [get_bd_cells and_txfifo_wr_en]
connect_bd_net [get_bd_pins dispatcher_0/tx_tdata]              [get_bd_pins concat_tx_fifo_din/In0]
connect_bd_net [get_bd_pins dispatcher_0/tx_tlast]              [get_bd_pins concat_tx_fifo_din/In1]
connect_bd_net [get_bd_pins concat_tx_fifo_din/dout]            [get_bd_pins async_fifo_aurora_tx/din]
connect_bd_net [get_bd_pins dispatcher_0/tx_tvalid]             [get_bd_pins and_txfifo_wr_en/Op1]
connect_bd_net [get_bd_pins inv_tx_fifo_full/Res]               [get_bd_pins and_txfifo_wr_en/Op2]
connect_bd_net [get_bd_pins and_txfifo_wr_en/Res]               [get_bd_pins async_fifo_aurora_tx/wr_en]
connect_bd_net [get_bd_pins async_fifo_aurora_tx/dout]          [get_bd_pins slice_tx_tdata/Din]
connect_bd_net [get_bd_pins async_fifo_aurora_tx/dout]          [get_bd_pins slice_tx_tlast/Din]
connect_bd_net [get_bd_pins slice_tx_tdata/Dout]                [get_bd_pins aurora_data_channel_0/local_tx_tdata]
connect_bd_net [get_bd_pins slice_tx_tlast/Dout]                [get_bd_pins aurora_data_channel_0/local_tx_tlast]
connect_bd_net [get_bd_pins async_fifo_aurora_tx/valid]         [get_bd_pins aurora_data_channel_0/local_tx_tvalid]
connect_bd_net [get_bd_pins aurora_data_channel_0/local_tx_tready] [get_bd_pins async_fifo_aurora_tx/rd_en]
connect_bd_net [get_bd_pins async_fifo_aurora_tx/prog_full]     [get_bd_pins inv_tx_fifo_full/Op1]
connect_bd_net [get_bd_pins inv_tx_fifo_full/Res]               [get_bd_pins dispatcher_0/tx_tready]
# 實體 tx1/tx0 的 s_axi_tx_*/s_axi_tx_tkeep 已在上面「三個新 Aurora 多板協定
# 模組」區塊接好（經仲裁器/直接接 Layer3 ctrl_tx0），這裡不用再重複接

# -- reply TX relay path (aurora_reply_tx_0 -sys_clk-> async_fifo_reply_tx
#    -aurora_clk-> aurora_tx1_arbiter_0/reply_tx1_*) ---------------------------
# 2026-07-27 新增（統一讀取/寫入架構）：跟上面 dispatcher_0 的 TX relay
# path 是同一種 wr_en/tready 反壓慣例（wr_en = tvalid && !prog_full，
# tready 回饋給上游同一個 !prog_full 訊號），差別只在來源是
# aurora_reply_tx_0 而不是 dispatcher_0。
connect_bd_net [get_bd_pins aurora_reply_tx_0/reply_tdata]        [get_bd_pins concat_reply_fifo_din/In0]
connect_bd_net [get_bd_pins aurora_reply_tx_0/reply_tlast]        [get_bd_pins concat_reply_fifo_din/In1]
connect_bd_net [get_bd_pins concat_reply_fifo_din/dout]           [get_bd_pins async_fifo_reply_tx/din]
connect_bd_net [get_bd_pins aurora_reply_tx_0/reply_tvalid]       [get_bd_pins and_replyfifo_wr_en/Op1]
connect_bd_net [get_bd_pins inv_reply_fifo_full/Res]              [get_bd_pins and_replyfifo_wr_en/Op2]
connect_bd_net [get_bd_pins and_replyfifo_wr_en/Res]              [get_bd_pins async_fifo_reply_tx/wr_en]
connect_bd_net [get_bd_pins async_fifo_reply_tx/dout]             [get_bd_pins slice_reply_tdata/Din]
connect_bd_net [get_bd_pins async_fifo_reply_tx/dout]             [get_bd_pins slice_reply_tlast/Din]
connect_bd_net [get_bd_pins slice_reply_tdata/Dout]               [get_bd_pins aurora_tx1_arbiter_0/reply_tx1_tdata]
connect_bd_net [get_bd_pins slice_reply_tlast/Dout]               [get_bd_pins aurora_tx1_arbiter_0/reply_tx1_tlast]
connect_bd_net [get_bd_pins async_fifo_reply_tx/valid]            [get_bd_pins aurora_tx1_arbiter_0/reply_tx1_tvalid]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/reply_tx1_tready] [get_bd_pins async_fifo_reply_tx/rd_en]
connect_bd_net [get_bd_pins async_fifo_reply_tx/prog_full]        [get_bd_pins inv_reply_fifo_full/Op1]
connect_bd_net [get_bd_pins inv_reply_fifo_full/Res]              [get_bd_pins aurora_reply_tx_0/reply_tready]

# -- diag_cdc_0 -------------------------------------------------------------------
connect_bd_net [get_bd_pins dispatcher_0/diag_disp_fp_seen]    [get_bd_pins diag_cdc_0/in_fp_seen]
connect_bd_net [get_bd_pins dispatcher_0/diag_disp_local_seen] [get_bd_pins diag_cdc_0/in_local_seen]
connect_bd_net [get_bd_pins dispatcher_0/diag_disp_first_lo]   [get_bd_pins diag_cdc_0/in_first_lo]
connect_bd_net [get_bd_pins dispatcher_0/diag_disp_first_hi]   [get_bd_pins diag_cdc_0/in_first_hi]

# -- fp_ddr4_rw_1 (Read + Write: same architecture as step-12, TI bus direct-wired) --
# ti_cmd wired directly to fp0/ti40_ep_trigger (already connected in TI bus fan-out)
# bit[0]=start_wr -> host TI_BIT_DDR4_WR=0; bit[1]=start_rd -> host TI_BIT_DDR4_RD=1
# wi_addr <- WI 0x12 (already connected in WI section)
# wi_wdata_0-3 <- WI 0x14-0x17
connect_bd_net [get_bd_pins fp0/wi14_ep_dataout] [get_bd_pins fp_ddr4_rw_1/wi_wdata_0]
connect_bd_net [get_bd_pins fp0/wi15_ep_dataout] [get_bd_pins fp_ddr4_rw_1/wi_wdata_1]
connect_bd_net [get_bd_pins fp0/wi16_ep_dataout] [get_bd_pins fp_ddr4_rw_1/wi_wdata_2]
connect_bd_net [get_bd_pins fp0/wi17_ep_dataout] [get_bd_pins fp_ddr4_rw_1/wi_wdata_3]

# -- AXI / DDR4 interface -----------------------------------------------------------
# Step 14.3b playback port: smartconnect_0 is now on ddr4_ui_clk. Each sys_clk master
# (ddr_writer_0, fp_ddr4_rw_1) crosses via its own axi_clock_converter into the SC;
# the 8 stream readers are already on ui_clk and connect to SC directly; SC M00
# connects straight to DDR4 (same clock domain, no CC in between).
# NOTE: 2026-07-15 起這條路徑改成 ddr_writer_0/m_axi -> ddr_writer_axi_fifo
# （fifo_generator，AXI4/Write-Channels-only/Independent Clocks）->
# smartconnect_0/S00_AXI，取代原本的 axi_cc_ddr4（axi_clock_converter）。
connect_bd_intf_net [get_bd_intf_pins ddr_writer_0/m_axi]     [get_bd_intf_pins ddr_writer_axi_fifo/S_AXI]
# 2026-07-16 補漏：上面這行 connect_bd_intf_net 只自動配對成功 11 個子訊號
# （awvalid/awaddr/awburst/awlen/awsize/wvalid/wdata/wstrb/wlast/bready/
# bresp），漏了 awready/wready/bvalid 這 3 個（讀生成的網表
# awg_step16.gen/sources_1/bd/awg_step16_bd/synth/awg_step16_bd.v 直接
# 證實：ddr_writer_0 的 m_axi_awready/wready/bvalid 一直被 tie 常數
# 1'b0，跟 ILA 探測與否無關，從這條路徑 2026-07-15 建立起就沒接上）。
# 原因不確定（信心 <50%，猜測跟 ddr_writer_axi_fifo 的 AXI4/Write-
# Channels-only 模式的 IP-XACT interface 定義有關，沒有查證），但修法
# 不依賴查出原因：額外用 connect_bd_net 補這 3 個訊號，跟上面的
# interface 整包連接並存互補，見 PROJECT.md「T_WAVEFORM_STREAM」章節。
connect_bd_net [get_bd_pins ddr_writer_axi_fifo/s_axi_awready] [get_bd_pins ddr_writer_0/m_axi_awready]
connect_bd_net [get_bd_pins ddr_writer_axi_fifo/s_axi_wready]  [get_bd_pins ddr_writer_0/m_axi_wready]
connect_bd_net [get_bd_pins ddr_writer_axi_fifo/s_axi_bvalid]  [get_bd_pins ddr_writer_0/m_axi_bvalid]
connect_bd_intf_net [get_bd_intf_pins ddr_writer_axi_fifo/M_AXI] [get_bd_intf_pins smartconnect_0/S00_AXI]
connect_bd_intf_net [get_bd_intf_pins fp_ddr4_rw_1/m_axi]     [get_bd_intf_pins axi_cc_fpddr4/S_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_cc_fpddr4/M_AXI]    [get_bd_intf_pins smartconnect_0/S01_AXI]
connect_bd_intf_net [get_bd_intf_pins reader_b_0/m_axi]       [get_bd_intf_pins smartconnect_0/S02_AXI]
connect_bd_intf_net [get_bd_intf_pins reader_b_1/m_axi]       [get_bd_intf_pins smartconnect_0/S03_AXI]
connect_bd_intf_net [get_bd_intf_pins reader_b_2/m_axi]       [get_bd_intf_pins smartconnect_0/S04_AXI]
connect_bd_intf_net [get_bd_intf_pins reader_b_3/m_axi]       [get_bd_intf_pins smartconnect_0/S05_AXI]
connect_bd_intf_net [get_bd_intf_pins reader_a_0/m_axi]       [get_bd_intf_pins smartconnect_0/S06_AXI]
connect_bd_intf_net [get_bd_intf_pins reader_a_1/m_axi]       [get_bd_intf_pins smartconnect_0/S07_AXI]
connect_bd_intf_net [get_bd_intf_pins reader_a_2/m_axi]       [get_bd_intf_pins smartconnect_0/S08_AXI]
connect_bd_intf_net [get_bd_intf_pins reader_a_3/m_axi]       [get_bd_intf_pins smartconnect_0/S09_AXI]
# 2026-07-14：ddr_zero_writer_0（「初始化」指令的 DDR4 清空狀態機），
# 比照 ddr_writer_0/fp_ddr4_rw_1 的既有 pattern，自己的 upstream CC
connect_bd_intf_net [get_bd_intf_pins ddr_zero_writer_0/m_axi] [get_bd_intf_pins axi_cc_ddrzero/S_AXI]
connect_bd_intf_net [get_bd_intf_pins axi_cc_ddrzero/M_AXI]    [get_bd_intf_pins smartconnect_0/S10_AXI]
connect_bd_intf_net [get_bd_intf_pins smartconnect_0/M00_AXI] [get_bd_intf_pins ddr4_0/C0_DDR4_S_AXI]

# -- DDR4 address assignment ---------------------------------------------------------
assign_bd_address -offset 0x0000000000000000 -range 1G \
    -target_address_space [get_bd_addr_spaces ddr_writer_0/m_axi] \
    [get_bd_addr_segs {ddr4_0/C0_DDR4_MEMORY_MAP/C0_DDR4_ADDRESS_BLOCK}]
assign_bd_address -offset 0x0000000000000000 -range 1G \
    -target_address_space [get_bd_addr_spaces fp_ddr4_rw_1/m_axi] \
    [get_bd_addr_segs {ddr4_0/C0_DDR4_MEMORY_MAP/C0_DDR4_ADDRESS_BLOCK}]
assign_bd_address -offset 0x0000000000000000 -range 1G \
    -target_address_space [get_bd_addr_spaces ddr_zero_writer_0/m_axi] \
    [get_bd_addr_segs {ddr4_0/C0_DDR4_MEMORY_MAP/C0_DDR4_ADDRESS_BLOCK}]
foreach ch {0 1 2 3} {
    assign_bd_address -offset 0x0000000000000000 -range 1G \
        -target_address_space [get_bd_addr_spaces reader_a_$ch/m_axi] \
        [get_bd_addr_segs {ddr4_0/C0_DDR4_MEMORY_MAP/C0_DDR4_ADDRESS_BLOCK}]
    assign_bd_address -offset 0x0000000000000000 -range 1G \
        -target_address_space [get_bd_addr_spaces reader_b_$ch/m_axi] \
        [get_bd_addr_segs {ddr4_0/C0_DDR4_MEMORY_MAP/C0_DDR4_ADDRESS_BLOCK}]
}

# -- WO endpoints -------------------------------------------------------------------
# WO 0x20: ddr_writer_0 busy + ddr_zero_writer_0 busy (2026-07-14)
connect_bd_net [get_bd_pins ddr_writer_0/busy]      [get_bd_pins concat_ddr_busy/In0]
connect_bd_net [get_bd_pins ddr_zero_writer_0/busy] [get_bd_pins concat_ddr_busy/In1]
connect_bd_net [get_bd_pins const_30b0/dout]        [get_bd_pins concat_ddr_busy/In2]
connect_bd_net [get_bd_pins concat_ddr_busy/dout]   [get_bd_pins fp0/wo20_ep_datain]
# WO 0x21-0x24: DDR4 read data 128-bit (fp_ddr4_rw_1)
connect_bd_net [get_bd_pins fp_ddr4_rw_1/wo_rdata_0] [get_bd_pins fp0/wo21_ep_datain]
connect_bd_net [get_bd_pins fp_ddr4_rw_1/wo_rdata_1] [get_bd_pins fp0/wo22_ep_datain]
connect_bd_net [get_bd_pins fp_ddr4_rw_1/wo_rdata_2] [get_bd_pins fp0/wo23_ep_datain]
connect_bd_net [get_bd_pins fp_ddr4_rw_1/wo_rdata_3] [get_bd_pins fp0/wo24_ep_datain]
# WO 0x25: DDR4 read status (fp_ddr4_rw_1)
connect_bd_net [get_bd_pins fp_ddr4_rw_1/wo_status]  [get_bd_pins fp0/wo25_ep_datain]

# WO 0x27 AURORA_STATUS：2026-07-05 (step 15a) 復原，接真實 channel_up；
# 2026-07-06 bugfix：channel_up 是 aurora_clk domain，這裡（WO 讀回）是
# sys_clk domain，原本直接接線沒有 CDC，改經 channel_up0/1_cdc_0（同步後的
# 輸出也直接拿去餵下面的 LED，不需要重複做一次 CDC）
connect_bd_net [get_bd_pins aurora_64b66b_0/channel_up] [get_bd_pins channel_up0_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_64b66b_1/channel_up] [get_bd_pins channel_up1_cdc_0/src_in]
connect_bd_net [get_bd_pins channel_up0_cdc_0/dst_out]  [get_bd_pins concat_aurora_status/In0]
connect_bd_net [get_bd_pins channel_up1_cdc_0/dst_out]  [get_bd_pins concat_aurora_status/In1]
connect_bd_net [get_bd_pins const_30b0/dout]            [get_bd_pins concat_aurora_status/In2]
connect_bd_net [get_bd_pins concat_aurora_status/dout]  [get_bd_pins fp0/wo27_ep_datain]

# WO 0x26 ENUM 狀態（新，2026-07-05；2026-07-06 bugfix：init_ok/total_boards/
# board_index 都是 aurora_clk domain，原本直接接線沒有 CDC，改經
# init_ok_cdc_0/total_boards_cdc_0/board_index_cdc_0）
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/init_ok]      [get_bd_pins init_ok_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/total_boards] [get_bd_pins total_boards_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/board_index]  [get_bd_pins board_index_cdc_0/src_in]
# 2026-07-23 新增：au_trig_delay 全自動校準機制，per_hop_value_cdc_0
# 來源（見上方 create_bd_cell 處註解）
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/per_hop_value] [get_bd_pins per_hop_value_cdc_0/src_in]
# 2026-07-24 再追加：per_hop_value_eff（自動或手動，見 rtl/aurora_ctrl_
# channel.v per_hop_value_eff port 註解）讀回，CDC 回 sys_clk 後取低
# 30-bit 塞進 WO 0x2c（concat_hard_err/In2，原本是 const_30b0 常數 0
# 佔位，借用它的保留欄位，見下方 per_hop_eff_ro_slice 註解）
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/per_hop_value_eff] [get_bd_pins per_hop_value_eff_cdc_0/src_in]
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice per_hop_eff_ro_slice
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {29} CONFIG.DIN_TO {0}] \
    [get_bd_cells per_hop_eff_ro_slice]
connect_bd_net [get_bd_pins per_hop_value_eff_cdc_0/dst_out] [get_bd_pins per_hop_eff_ro_slice/Din]
# 2026-07-21 新增：aurora_data_channel_0 的 src_id 合理性檢查用（見
# rtl/aurora_data_channel_relayfifo.v port 宣告處註解）。跟上面
# total_boards_cdc_0 的來源是同一個 pin，這裡直接再接一條給
# aurora_data_channel_0——兩個模組都在 aurora_clk domain，不需要 CDC。
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/total_boards] [get_bd_pins aurora_data_channel_0/total_boards]
connect_bd_net [get_bd_pins init_ok_cdc_0/dst_out]      [get_bd_pins concat_enum_status/In0]
connect_bd_net [get_bd_pins total_boards_cdc_0/dst_out] [get_bd_pins concat_enum_status/In1]
connect_bd_net [get_bd_pins board_index_cdc_0/dst_out]  [get_bd_pins concat_enum_status/In2]
connect_bd_net [get_bd_pins const_21b0/dout]                    [get_bd_pins concat_enum_status/In3]
connect_bd_net [get_bd_pins concat_enum_status/dout]            [get_bd_pins fp0/wo26_ep_datain]

# WO 0x28 RESERVE 狀態（新，2026-07-05；2026-07-06 bugfix：同上，改經
# reserve_ok_cdc_0/reserve_busy_cdc_0）
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/reserve_ok]    [get_bd_pins reserve_ok_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/reserve_busy]  [get_bd_pins reserve_busy_cdc_0/src_in]
connect_bd_net [get_bd_pins reserve_ok_cdc_0/dst_out]    [get_bd_pins concat_reserve_status/In0]
connect_bd_net [get_bd_pins reserve_busy_cdc_0/dst_out]  [get_bd_pins concat_reserve_status/In1]
connect_bd_net [get_bd_pins const_30b0/dout]                     [get_bd_pins concat_reserve_status/In2]
connect_bd_net [get_bd_pins concat_reserve_status/dout]          [get_bd_pins fp0/wo28_ep_datain]

# WO 0x29 trig_delay readback（2026-07-07 step 15b 新增）：au_trig_delay 跟
# local_reg_handler_0/fp0 同在 sys_clk domain，不需要 CDC，直接接線
connect_bd_net [get_bd_pins local_reg_handler_0/au_trig_delay] [get_bd_pins concat_trig_delay/In0]
connect_bd_net [get_bd_pins const_16b0/dout]                    [get_bd_pins concat_trig_delay/In1]
connect_bd_net [get_bd_pins concat_trig_delay/dout]             [get_bd_pins fp0/wo29_ep_datain]

# WO 0x2A flash_status + WO 0x2B raw_magic（2026-07-08 step 15b 新增）
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/flash_status] \
    [get_bd_pins concat_flash_status/In0] \
    [get_bd_pins flash_status_cdc_0/src_in]
# 2026-07-31 新增：CDC 後（sys_clk）給 QT_FLASH_STATUS 用
connect_bd_net [get_bd_pins flash_status_cdc_0/dst_out] [get_bd_pins aurora_reply_tx_0/bi_flash_status]

# 2026-07-31 新增（同一天，擴充）：QT_FLASH_STATUS 加上 flash 實際
# 內容。init_scale_cfg 已經扇出給 board_cfg_reg_0/flash_scale_cfg
# （line ~2415）跟 awg_calib_regs_0/flash_scale_cfg（line ~4438），
# 這裡再多接一份給 CDC；init_coef_all 是這次新增的打包 output，只有
# 這一個目的地。
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/init_scale_cfg] [get_bd_pins flash_scale_cfg_cdc_0/src_in]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/init_coef_all]  [get_bd_pins flash_coef_all_cdc_0/src_in]
connect_bd_net [get_bd_pins flash_scale_cfg_cdc_0/dst_out] [get_bd_pins aurora_reply_tx_0/bi_flash_scale_cfg]
connect_bd_net [get_bd_pins flash_coef_all_cdc_0/dst_out]  [get_bd_pins aurora_reply_tx_0/bi_flash_coef_all]
connect_bd_net [get_bd_pins const_28b0/dout]                [get_bd_pins concat_flash_status/In1]
connect_bd_net [get_bd_pins concat_flash_status/dout]       [get_bd_pins fp0/wo2a_ep_datain]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/raw_magic]    [get_bd_pins fp0/wo2b_ep_datain]
connect_bd_net [get_bd_pins hard_err_sticky_0/sticky_out] [get_bd_pins concat_hard_err/In0]
connect_bd_net [get_bd_pins hard_err_sticky_1/sticky_out] [get_bd_pins concat_hard_err/In1]
# 2026-07-24：原本這裡接 const_30b0（常數 0 佔位），改接
# per_hop_eff_ro_slice/Dout（見上方 create_bd_cell 處註解），const_30b0
# 其餘 3 個 fan-out（concat_ddr_busy/concat_aurora_status/concat_
# reserve_status 的 In2）不受影響，仍然接常數 0
connect_bd_net [get_bd_pins per_hop_eff_ro_slice/Dout]    [get_bd_pins concat_hard_err/In2]
connect_bd_net [get_bd_pins concat_hard_err/dout]         [get_bd_pins fp0/wo2c_ep_datain]

connect_bd_net [get_bd_pins reserve_fwd_cdc_0/dst_out]     [get_bd_pins concat_reserve_diag/In0]
connect_bd_net [get_bd_pins reserve_timeout_cdc_0/dst_out] [get_bd_pins concat_reserve_diag/In1]
connect_bd_net [get_bd_pins reserve_reply_cdc_0/dst_out]   [get_bd_pins concat_reserve_diag/In2]
connect_bd_net [get_bd_pins reserve_pulse_cdc_0/dst_out]   [get_bd_pins concat_reserve_diag/In3]
connect_bd_net [get_bd_pins au_reserve_start_sticky_0/sticky_out] [get_bd_pins concat_reserve_diag/In4]
connect_bd_net [get_bd_pins reply_seen_cdc_0/dst_out]      [get_bd_pins concat_reserve_diag/In5]
connect_bd_net [get_bd_pins reply_granted_cdc_0/dst_out]   [get_bd_pins concat_reserve_diag/In6]
connect_bd_net [get_bd_pins local_reg_handler_0/au_reply_src]        [get_bd_pins concat_reserve_diag/In7]
connect_bd_net [get_bd_pins local_reg_handler_0/au_reply_query_type] [get_bd_pins concat_reserve_diag/In8]
connect_bd_net [get_bd_pins const_1b0/dout]                [get_bd_pins concat_reserve_diag/In9]
connect_bd_net [get_bd_pins concat_reserve_diag/dout]      [get_bd_pins fp0/wo2d_ep_datain]

# WO 0x2F DIAG
# 2026-07-27：In2/In3 原本接 local_reg_handler_0/diag_lrh_bcfg_wr、
# board_cfg_reg_0/diag_au_wr_seen 這兩個診斷 sticky latch，兩者都隨
# T_BOARD_CFG(0x10) 一併從 RTL 移除（見 PORTS.md 933 行、local_reg_
# handler.v/board_cfg_reg.v 對應註解），create_bd.tcl 這裡原本殘留對
# 已刪除 port 的引用，改成跟 In0/In1 一樣接 const_zero（這個診斷字組
# 本來就不是正式 host 協定的一部分，純粹拔掉失效的 bring-up 診斷位元）。
connect_bd_net [get_bd_pins const_zero/dout]                        [get_bd_pins concat_diag/In0]
connect_bd_net [get_bd_pins const_zero/dout]                        [get_bd_pins concat_diag/In1]
connect_bd_net [get_bd_pins const_zero/dout]                        [get_bd_pins concat_diag/In2]
connect_bd_net [get_bd_pins const_zero/dout]                        [get_bd_pins concat_diag/In3]
connect_bd_net [get_bd_pins const_28b0/dout]                        [get_bd_pins concat_diag/In4]
connect_bd_net [get_bd_pins concat_diag/dout]                       [get_bd_pins fp0/wo2f_ep_datain]

# WO 0x34 RT_BOARD_INFO
connect_bd_net [get_bd_pins board_cfg_reg_0/board_id]       [get_bd_pins concat_rt_board_info/In0]
connect_bd_net [get_bd_pins board_cfg_reg_0/is_master]      [get_bd_pins concat_rt_board_info/In1]
connect_bd_net [get_bd_pins board_cfg_reg_0/ext_clk_sel]    [get_bd_pins concat_rt_board_info/In2]
connect_bd_net [get_bd_pins dac_mode_ramp_concat_0/dout]  [get_bd_pins concat_rt_board_info/In3]
connect_bd_net [get_bd_pins const_2b0/dout]                 [get_bd_pins concat_rt_board_info/In4]
connect_bd_net [get_bd_pins concat_rt_board_info/dout] [get_bd_pins fp0/wo34_ep_datain]

# 2026-07-15 新增：WO 0x35/0x36/0x37 讀回
connect_bd_net [get_bd_pins board_cfg_reg_0/scale_cfg]       [get_bd_pins concat_scale_cfg_ro/In0]
connect_bd_net [get_bd_pins const_24b0/dout]                 [get_bd_pins concat_scale_cfg_ro/In1]
connect_bd_net [get_bd_pins concat_scale_cfg_ro/dout]         [get_bd_pins fp0/wo35_ep_datain]
connect_bd_net [get_bd_pins amp_ctrl_read_mux_0/amp_ctrl_sel_out] [get_bd_pins concat_amp_ctrl_ro/In0]
connect_bd_net [get_bd_pins const_14b0/dout]                 [get_bd_pins concat_amp_ctrl_ro/In1]
connect_bd_net [get_bd_pins concat_amp_ctrl_ro/dout]          [get_bd_pins fp0/wo36_ep_datain]
connect_bd_net [get_bd_pins awg_calib_regs_0/coef_readback]  [get_bd_pins concat_coef_ro/In0]
connect_bd_net [get_bd_pins const_14b0/dout]                 [get_bd_pins concat_coef_ro/In1]
connect_bd_net [get_bd_pins concat_coef_ro/dout]              [get_bd_pins fp0/wo37_ep_datain]

# -- Diag nodes (WO 0x3A-0x3F) --------------------------------------------------------
connect_bd_net [get_bd_pins local_reg_handler_0/diag_lrh_rx_seen] [get_bd_pins concat_diag_nodes/In0]
connect_bd_net [get_bd_pins fp_input_wr_0/diag_fp_wr_seen]       [get_bd_pins concat_diag_nodes/In1]
connect_bd_net [get_bd_pins diag_cdc_0/out_fp_seen]               [get_bd_pins concat_diag_nodes/In2]
connect_bd_net [get_bd_pins diag_cdc_0/out_local_seen]            [get_bd_pins concat_diag_nodes/In3]
connect_bd_net [get_bd_pins const_28b0/dout]                      [get_bd_pins concat_diag_nodes/In4]
connect_bd_net [get_bd_pins concat_diag_nodes/dout]               [get_bd_pins fp0/wo3a_ep_datain]

connect_bd_net [get_bd_pins fp_input_wr_0/diag_fp_first_lo] [get_bd_pins fp0/wo3b_ep_datain]
connect_bd_net [get_bd_pins fp_input_wr_0/diag_fp_first_hi] [get_bd_pins fp0/wo3c_ep_datain]
connect_bd_net [get_bd_pins diag_cdc_0/out_first_lo]     [get_bd_pins fp0/wo3d_ep_datain]
connect_bd_net [get_bd_pins local_reg_handler_0/diag_lrh_first_lo] [get_bd_pins fp0/wo3e_ep_datain]
connect_bd_net [get_bd_pins local_reg_handler_0/diag_lrh_first_hi] [get_bd_pins fp0/wo3f_ep_datain]

# -- diag_capture_0 (BTPipeOut 0xA0) ---------------------------------------------------
# 2026-07-29 CDC 修法 v2：diag_capture_0 內部用 xpm_cdc_handshake 把整張
# 表搬進 okClk 域自己的暫存器組（見 rtl/diag_capture.v 檔頭說明），
# po_ep_datain/po_ep_read 直接接 fp0/poa0_ep_*，不需要中間的 FIFO。
connect_bd_net [get_bd_pins fp_input_wr_0/diag_raw_data]       [get_bd_pins diag_capture_0/raw_data]
connect_bd_net [get_bd_pins local_reg_handler_0/diag_lrh_data] [get_bd_pins diag_capture_0/lrh_data]
connect_bd_net [get_bd_pins diag_capture_0/po_ep_datain]       [get_bd_pins fp0/poa0_ep_datain]
connect_bd_net [get_bd_pins fp0/poa0_ep_read]                  [get_bd_pins diag_capture_0/po_ep_read]

# -- status_reply_capture_0 (BTPipeOut 0xA1, PO_STATUS_REPLY) --------------------------
# 2026-07-27 新增（統一讀取/寫入架構）：au_reply_src/au_reply_query_type/
# au_reply_data 已在上面 aurora_reply_tx_0 那段接好，這裡只補 BTPipeOut
# 本身。2026-07-29 CDC 修法 v2：跟 diag_capture_0/PO_DIAG(0xA0) 同款
# 接線，po_ep_datain/po_ep_read 直接接 fp0/poa1_ep_*，不需要中間的 FIFO。
connect_bd_net [get_bd_pins status_reply_capture_0/po_ep_datain] [get_bd_pins fp0/poa1_ep_datain]
connect_bd_net [get_bd_pins fp0/poa1_ep_read]                    [get_bd_pins status_reply_capture_0/po_ep_read]

# -- LED --------------------------------------------------------------------------------
# 2026-07-05 (step 15a): 復原，channel_up LED 接回真實訊號
# 2026-07-06 bugfix：LED 是 sys_clk 域的輸出，channel_up 是 aurora_clk，沿用
# 上面 WO 0x27 已經做好的 channel_up0/1_cdc_0 同步輸出，不用重複 CDC
connect_bd_net [get_bd_pins clk_wiz_0/locked]          [get_bd_pins led_concat/In0]
connect_bd_net [get_bd_pins channel_up0_cdc_0/dst_out] [get_bd_pins led_concat/In1]
connect_bd_net [get_bd_pins channel_up1_cdc_0/dst_out] [get_bd_pins led_concat/In2]
# 2026-07-14 改版：led[3]/led[4] 不能只看選擇位元正反，要「選中 且 該
# 時脈源真的 locked」才亮，避免選了 ext 但沒鎖定時 led[3] 誤亮。
# clk_wiz_ext_0/locked、clk_wiz_0/locked 都是既有訊號，這裡是多接一條
# fan-out，不是新建訊號。
# 2026-07-24：LED 改讀 board_cfg_reg_0/ext_clk_sel（真正生效的
# TriggerIn 觸發式值），不再直接讀 wi_ext_clk_sel/Dout（那只是 host
# 還沒觸發確認寫入前的暫存區，可能跟實際生效值不同步）。
connect_bd_net [get_bd_pins board_cfg_reg_0/ext_clk_sel] [get_bd_pins led_ext_sel_and_locked/Op1]
connect_bd_net [get_bd_pins clk_wiz_ext_0/locked]      [get_bd_pins led_ext_sel_and_locked/Op2]
connect_bd_net [get_bd_pins led_ext_sel_and_locked/Res] [get_bd_pins led_concat/In3]

connect_bd_net [get_bd_pins board_cfg_reg_0/ext_clk_sel] [get_bd_pins led_int_sel_not/Op1]
connect_bd_net [get_bd_pins led_int_sel_not/Res]     [get_bd_pins led_int_sel_and_locked/Op1]
connect_bd_net [get_bd_pins clk_wiz_0/locked]        [get_bd_pins led_int_sel_and_locked/Op2]
connect_bd_net [get_bd_pins led_int_sel_and_locked/Res] [get_bd_pins led_concat/In4]

connect_bd_net [get_bd_pins zmod_awg_0/sInitDoneDAC] [get_bd_pins led_awg_init_and01/Op1]
connect_bd_net [get_bd_pins zmod_awg_1/sInitDoneDAC] [get_bd_pins led_awg_init_and01/Op2]
connect_bd_net [get_bd_pins led_awg_init_and01/Res]  [get_bd_pins led_awg_init_and012/Op1]
connect_bd_net [get_bd_pins zmod_awg_2/sInitDoneDAC] [get_bd_pins led_awg_init_and012/Op2]
connect_bd_net [get_bd_pins led_awg_init_and012/Res] [get_bd_pins led_awg_init_and0123/Op1]
connect_bd_net [get_bd_pins zmod_awg_3/sInitDoneDAC] [get_bd_pins led_awg_init_and0123/Op2]
connect_bd_net [get_bd_pins led_awg_init_and0123/Res] [get_bd_pins led_concat/In5]

connect_bd_net [get_bd_pins led_concat/dout]            [get_bd_ports led]

# 2026-07-10：整段刪除 disabled system_ila_0 block（原本 if{0} 包住、從未執行
# 的死程式碼，2026-07-06 停用後留著「供參考/還原」，但在追 flash-save 問題
# 時被誤認成現存的 ILA，浪費了兩輪診斷時間，確認純屬死碼後直接刪除。
# dispatcher_0/diag_state、diag_idle_route 這兩個原本只接在這裡的訊號，
# 已經改接到 dispatcher_ila_0/probe14、probe30（見下方 dispatcher_ila_0 段落）。

# ==============================================================================
#  Aurora ILA: enum-protocol CDC-fix verification (aurora_clk domain)
# ==============================================================================
# 2026-07-06: added alongside the init_start_pulse/trig_start_pulse/
# reserve_start_pulse/board_id/is_master/reserve_dest_id CDC bugfix (see
# au_init_pulse_cdc_0 etc. above) -- 3-board hardware test showed enum protocol
# never even made the first hop (board_index stuck at 0 on all 3 boards) despite
# channel_up=1 everywhere; root-caused to a missing CDC on the host TI pulses.
# This ILA verifies the fix on real hardware: does the CDC-synced pulse actually
# arrive in aurora_clk, does master's ctrl_tx1 actually transmit, does the next
# board's rx0 actually receive it. All probes are existing module ports (no new
# RTL debug ports added -- keeping this one change (CDC fix) isolated from a
# second one (new debug ports) until the fix itself is confirmed on hardware).
if {$ENABLE_ILA} {
create_bd_cell -type ip -vlnv xilinx.com:ip:system_ila aurora_ila_0
# 2026-07-29：原本 C_NUM_OF_PROBES/C_MON_TYPE/C_DATA_DEPTH 跟所有
# C_PROBEn_WIDTH 分成兩次獨立的 set_property 呼叫——這次從 89 加到
# 91（加 probe89/90）時，第二次呼叫執行時 IP 顯然還沒真的長出
# probe89/90 這兩個 pin（`get_bd_pins aurora_ila_0/probe89` 找不到，
# 導致 connect_bd_net 起連鎖失敗，BD 留在破碎狀態，wrapper 產生時
# 大量該內部接的訊號被錯誤升級成頂層 I/O port，place_design 因為
# 1052 個 port 遠超過晶片實際可用的 342 個 I/O 而失敗）。合併成單一
# set_property 呼叫，確保 probe 數量單次呼叫就跟所有 width 一起
# 生效，不要再分兩次。
set_property -dict [list \
    CONFIG.C_MON_TYPE      {NATIVE} \
    CONFIG.C_NUM_OF_PROBES {92} \
    CONFIG.C_DATA_DEPTH    {2048} \
    CONFIG.C_PROBE0_WIDTH  {1}  \
    CONFIG.C_PROBE1_WIDTH  {1}  \
    CONFIG.C_PROBE2_WIDTH  {16} \
    CONFIG.C_PROBE3_WIDTH  {64} \
    CONFIG.C_PROBE4_WIDTH  {1}  \
    CONFIG.C_PROBE5_WIDTH  {1}  \
    CONFIG.C_PROBE6_WIDTH  {64} \
    CONFIG.C_PROBE7_WIDTH  {1}  \
    CONFIG.C_PROBE8_WIDTH  {1}  \
    CONFIG.C_PROBE9_WIDTH  {1}  \
    CONFIG.C_PROBE10_WIDTH {1}  \
    CONFIG.C_PROBE11_WIDTH {1}  \
    CONFIG.C_PROBE12_WIDTH {1}  \
    CONFIG.C_PROBE13_WIDTH {5}  \
    CONFIG.C_PROBE14_WIDTH {1}  \
    CONFIG.C_PROBE15_WIDTH {64} \
    CONFIG.C_PROBE16_WIDTH {1}  \
    CONFIG.C_PROBE17_WIDTH {1}  \
    CONFIG.C_PROBE18_WIDTH {2}  \
    CONFIG.C_PROBE19_WIDTH {1}  \
    CONFIG.C_PROBE20_WIDTH {1}  \
    CONFIG.C_PROBE21_WIDTH {1}  \
    CONFIG.C_PROBE22_WIDTH {1}  \
    CONFIG.C_PROBE23_WIDTH {16} \
    CONFIG.C_PROBE24_WIDTH {1}  \
    CONFIG.C_PROBE25_WIDTH {1}  \
    CONFIG.C_PROBE26_WIDTH {1}  \
    CONFIG.C_PROBE27_WIDTH {64} \
    CONFIG.C_PROBE28_WIDTH {1}  \
    CONFIG.C_PROBE29_WIDTH {64} \
    CONFIG.C_PROBE30_WIDTH {1}  \
    CONFIG.C_PROBE31_WIDTH {1}  \
    CONFIG.C_PROBE32_WIDTH {16} \
    CONFIG.C_PROBE33_WIDTH {1}  \
    CONFIG.C_PROBE34_WIDTH {1}  \
    CONFIG.C_PROBE35_WIDTH {1}  \
    CONFIG.C_PROBE36_WIDTH {1}  \
    CONFIG.C_PROBE37_WIDTH {2}  \
    CONFIG.C_PROBE38_WIDTH {64} \
    CONFIG.C_PROBE39_WIDTH {1}  \
    CONFIG.C_PROBE40_WIDTH {1}  \
    CONFIG.C_PROBE41_WIDTH {1}  \
    CONFIG.C_PROBE42_WIDTH {64} \
    CONFIG.C_PROBE43_WIDTH {1}  \
    CONFIG.C_PROBE44_WIDTH {1}  \
    CONFIG.C_PROBE45_WIDTH {1}  \
    CONFIG.C_PROBE46_WIDTH {64} \
    CONFIG.C_PROBE47_WIDTH {1}  \
    CONFIG.C_PROBE48_WIDTH {1}  \
    CONFIG.C_PROBE49_WIDTH {1}  \
    CONFIG.C_PROBE50_WIDTH {64} \
    CONFIG.C_PROBE51_WIDTH {1}  \
    CONFIG.C_PROBE52_WIDTH {1}  \
    CONFIG.C_PROBE53_WIDTH {1}  \
    CONFIG.C_PROBE54_WIDTH {1}  \
    CONFIG.C_PROBE55_WIDTH {1}  \
    CONFIG.C_PROBE56_WIDTH {1}  \
    CONFIG.C_PROBE57_WIDTH {1}  \
    CONFIG.C_PROBE58_WIDTH {1}  \
    CONFIG.C_PROBE59_WIDTH {1}  \
    CONFIG.C_PROBE60_WIDTH {1}  \
    CONFIG.C_PROBE61_WIDTH {1}  \
    CONFIG.C_PROBE62_WIDTH {1}  \
    CONFIG.C_PROBE63_WIDTH {1}  \
    CONFIG.C_PROBE64_WIDTH {1}  \
    CONFIG.C_PROBE65_WIDTH {1}  \
    CONFIG.C_PROBE66_WIDTH {65} \
    CONFIG.C_PROBE67_WIDTH {65} \
    CONFIG.C_PROBE68_WIDTH {1}  \
    CONFIG.C_PROBE69_WIDTH {1}  \
    CONFIG.C_PROBE70_WIDTH {2}  \
    CONFIG.C_PROBE71_WIDTH {8}  \
    CONFIG.C_PROBE72_WIDTH {8}  \
    CONFIG.C_PROBE73_WIDTH {8}  \
    CONFIG.C_PROBE74_WIDTH {1}  \
    CONFIG.C_PROBE75_WIDTH {8}  \
    CONFIG.C_PROBE76_WIDTH {1}  \
    CONFIG.C_PROBE77_WIDTH {8}  \
    CONFIG.C_PROBE78_WIDTH {16} \
    CONFIG.C_PROBE79_WIDTH {16} \
    CONFIG.C_PROBE80_WIDTH {1}  \
    CONFIG.C_PROBE81_WIDTH {1}  \
    CONFIG.C_PROBE82_WIDTH {1}  \
    CONFIG.C_PROBE83_WIDTH {1}  \
    CONFIG.C_PROBE84_WIDTH {1}  \
    CONFIG.C_PROBE85_WIDTH {1}  \
    CONFIG.C_PROBE86_WIDTH {1}  \
    CONFIG.C_PROBE87_WIDTH {1}  \
    CONFIG.C_PROBE88_WIDTH {1}  \
    CONFIG.C_PROBE89_WIDTH {1}  \
    CONFIG.C_PROBE90_WIDTH {1}  \
    CONFIG.C_PROBE91_WIDTH {5}  \
] [get_bd_cells aurora_ila_0]

# 2026-07-17：來源改成 aurora_64b66b_0/user_clk_out，跟主幹 aurora_clk 用同一條淨（見上方 fan-out 說明）
connect_bd_net [get_bd_pins aurora_64b66b_0/user_clk_out] [get_bd_pins aurora_ila_0/clk]

# probe0-1: CDC-synced host trigger + is_master (confirms the CDC fix actually
# delivers the pulse into aurora_clk domain)
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/init_start_pulse] [get_bd_pins aurora_ila_0/probe0]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/is_master]        [get_bd_pins aurora_ila_0/probe1]
# probe2: CDC-synced board_id
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/board_id] [get_bd_pins aurora_ila_0/probe2]
# probe3-5: forward TX (master should transmit T_ENUM_COUNT here)
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx1_tdata]  [get_bd_pins aurora_ila_0/probe3]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx1_tvalid] [get_bd_pins aurora_ila_0/probe4]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx1_tready] [get_bd_pins aurora_ila_0/probe5]
# probe6-7: rx0 (this board's incoming ring traffic, own broadcast or relayed)
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/rx0_tdata]  [get_bd_pins aurora_ila_0/probe6]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/rx0_tvalid] [get_bd_pins aurora_ila_0/probe7]
# probe8-9: channel_up cross-check
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/channel_up_0] [get_bd_pins aurora_ila_0/probe8]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/channel_up_1] [get_bd_pins aurora_ila_0/probe9]
# probe10: synthesized TRIGGER-injection pulse (for the later trigger-protocol test)
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/local_inject_tvalid] [get_bd_pins aurora_ila_0/probe10]
# probe11-14 (2026-07-06, trigger-protocol debug): trig_start_pulse never seen
# to cause ctrl_tx1_tvalid on real hardware -- add the missing pulse itself plus
# lock_req/ack_count/go_sent to pinpoint which step of the PAUSE_REQ->ACK->GO
# sequence (if any) actually runs.
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/trig_start_pulse] [get_bd_pins aurora_ila_0/probe11]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/lock_req]         [get_bd_pins aurora_ila_0/probe12]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_ack_count]   [get_bd_pins aurora_ila_0/probe13]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_go_sent]     [get_bd_pins aurora_ila_0/probe14]
# probe15-16 (2026-07-06): aurora-side end of the dispatcher<->aurora boundary --
# aurora_rx_merge_0's output is what actually reaches async_fifo_aurora_rx ->
# dispatcher_0/rx_tdata,rx_tvalid (sys_clk side, see dispatcher_ila_0 below for
# the other end of this same boundary).
connect_bd_net [get_bd_pins aurora_rx_merge_0/local_out_tdata]  [get_bd_pins aurora_ila_0/probe15]
connect_bd_net [get_bd_pins aurora_rx_merge_0/local_out_tvalid] [get_bd_pins aurora_ila_0/probe16]
# probe17-18 (2026-07-06): ack_count/go_sent confirmed correct on real hardware,
# but local_inject_tvalid never fires -- two separate iverilog simulations
# (isolated aurora_ctrl_channel, and full Layer2+Layer3+arbiter) both show the
# RTL logic working correctly, so the discrepancy must be hardware-specific.
# These two probes let us see directly whether inject_pending ever gets set,
# and whether inj_state ever leaves INJ_IDLE, to pinpoint exactly where real
# hardware diverges from simulation.
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_inject_pending] [get_bd_pins aurora_ila_0/probe17]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_inj_state]      [get_bd_pins aurora_ila_0/probe18]
# probe19-20 (2026-07-06): inject_pending confirmed 1 on real hardware (both a
# clean first firing and a repeat firing), but inj_state never leaves INJ_IDLE
# despite the seemingly-unconditional "INJ_IDLE: if (inject_pending) inj_state
# <= INJ_BEAT0;" rule -- ruled out via code review and 2 iverilog simulations.
# The only two remaining inputs to this exact state machine we haven't directly
# probed are rst (would force inj_state back to IDLE every cycle if stuck
# asserted -- inferred healthy from ftx_state/frx_state working, but never
# directly confirmed) and local_inject_tready (doesn't gate the IDLE->BEAT0
# transition per the code, but included for completeness since it's the
# module's only other external input relevant to this always block).
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/rst]                 [get_bd_pins aurora_ila_0/probe19]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/local_inject_tready] [get_bd_pins aurora_ila_0/probe20]
# probe21: Layer 2's own local_tvalid (aurora_rx_merge_0's data_local_tvalid
# input) -- if this were stuck permanently high it would explain
# local_inject_tready staying low (aurora_rx_merge.v: inject_tready =
# !data_local_tvalid), though that specific mechanism doesn't gate the
# INJ_IDLE->INJ_BEAT0 transition we're actually stuck on -- included anyway
# for completeness per user request to rule out everything nearby.
connect_bd_net [get_bd_pins aurora_data_channel_0/local_out_tvalid] [get_bd_pins aurora_ila_0/probe21]

# probe22-32 (2026-07-08, step 15b): T_RESERVE_START 封包路徑上機測試持續
# 失敗（board B/board C 都是 reserve_ok=0，負向對照也是 reserve_ok=0），host 端
# 讀不到中間狀態，無法分辨 reserve_start_pulse 到底有沒有真的送到
# aurora_ctrl_channel_0。這批 probe 涵蓋整條 reserve 協定路徑：
#   probe22-23: CDC 送進來的 pulse/資料本身有沒有到、值對不對
#   probe24-25: 最終結果（跟 WO 0x28 讀到的是同一份訊號，但這裡是
#               aurora_clk domain 原始值，沒有 CDC 延遲）
#   probe26:    is_busy fail-fast 判斷用的輸入，本身有沒有卡在忙碌狀態
#   probe27-28: backward RX（ACK 應該從這裡進來，這條路完全沒探測過）
#   probe29-30: backward TX（reply/relay 應該從這裡送出去，這條路完全沒
#               探測過）
#   probe31-32: fwd_pending/reserve_dest_id_r 這兩個內部 reg（新增
#               diag_reserve_fwd_pending/diag_reserve_dest_id_r port），
#               確認 reserve_start_pulse 進來後有沒有真的組出 REQ 封包
#               排隊送出、鎖存的目的地是不是正確值
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/reserve_start_pulse]     [get_bd_pins aurora_ila_0/probe22]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/reserve_dest_id]        [get_bd_pins aurora_ila_0/probe23]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/reserve_busy]            [get_bd_pins aurora_ila_0/probe24]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/reserve_ok]              [get_bd_pins aurora_ila_0/probe25]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/is_busy]                 [get_bd_pins aurora_ila_0/probe26]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/rx1_tdata]               [get_bd_pins aurora_ila_0/probe27]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/rx1_tvalid]              [get_bd_pins aurora_ila_0/probe28]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx0_tdata]          [get_bd_pins aurora_ila_0/probe29]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/ctrl_tx0_tvalid]         [get_bd_pins aurora_ila_0/probe30]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_reserve_fwd_pending] [get_bd_pins aurora_ila_0/probe31]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_reserve_dest_id_r]   [get_bd_pins aurora_ila_0/probe32]
# probe33-37（2026-07-16 新增）：排查 T_DDR4_WRITE(0x04) 遠端寫入送不出去
# 的問題——dispatcher 本身已經確認正確判斷 RT_TX、封包內容正確寫進
# async_fifo_aurora_tx，但目標板子完全沒收到，懷疑卡在
# aurora_data_channel_0 的 TX_IDLE/TX_RELAY/TX_LOCAL 仲裁沒有進
# TX_LOCAL（可能 locked_r 卡在 1，或 pkt_busy 卡在 1 讓仲裁一直判給
# TX_RELAY）。locked/lock_req/is_busy 是既有 port（不是 bundled
# interface，直接 connect_bd_net 沒有風險），debug_pkt_busy/
# debug_tx_state 是這次新增的鏡像 port。
# probe33: aurora_data_channel_0/locked
# probe34: aurora_data_channel_0/lock_req
# probe35: aurora_data_channel_0/is_busy
# probe36: aurora_data_channel_0/debug_pkt_busy
# probe37: aurora_data_channel_0/debug_tx_state[1:0]（TX_IDLE=0/TX_RELAY=1/TX_LOCAL=2）
connect_bd_net [get_bd_pins aurora_data_channel_0/locked]         [get_bd_pins aurora_ila_0/probe33]
connect_bd_net [get_bd_pins aurora_data_channel_0/lock_req]       [get_bd_pins aurora_ila_0/probe34]
connect_bd_net [get_bd_pins aurora_data_channel_0/is_busy]        [get_bd_pins aurora_ila_0/probe35]
connect_bd_net [get_bd_pins aurora_data_channel_0/debug_pkt_busy] [get_bd_pins aurora_ila_0/probe36]
connect_bd_net [get_bd_pins aurora_data_channel_0/debug_tx_state] [get_bd_pins aurora_ila_0/probe37]
# probe38-50（2026-07-16 新增，第六輪）：完整涵蓋 T_DDR4_WRITE(0x04) 送出
# 之後、到實體 Aurora TX 之間的每一個節點，一次擷取同時看送端(TX)+收端(RX)
# 內容，不要再一輪一輪加 probe（重建一次要 1 小時）。這幾個訊號全部都是
# 各模組既有 port，不需要新增鏡像 port。
# probe38-41: aurora_data_channel_0 輸入端（來自 async_fifo_aurora_tx 讀出，
#             即 dispatcher_0/tx_tdata 跨完 CDC 之後）
connect_bd_net [get_bd_pins aurora_data_channel_0/local_tx_tdata]  [get_bd_pins aurora_ila_0/probe38]
connect_bd_net [get_bd_pins aurora_data_channel_0/local_tx_tvalid] [get_bd_pins aurora_ila_0/probe39]
connect_bd_net [get_bd_pins aurora_data_channel_0/local_tx_tready] [get_bd_pins aurora_ila_0/probe40]
connect_bd_net [get_bd_pins aurora_data_channel_0/local_tx_tlast]  [get_bd_pins aurora_ila_0/probe41]
# probe42-45: aurora_data_channel_0 輸出端（送給 aurora_tx1_arbiter_0）
connect_bd_net [get_bd_pins aurora_data_channel_0/data_tx1_tdata]  [get_bd_pins aurora_ila_0/probe42]
connect_bd_net [get_bd_pins aurora_data_channel_0/data_tx1_tvalid] [get_bd_pins aurora_ila_0/probe43]
connect_bd_net [get_bd_pins aurora_data_channel_0/data_tx1_tready] [get_bd_pins aurora_ila_0/probe44]
connect_bd_net [get_bd_pins aurora_data_channel_0/data_tx1_tlast]  [get_bd_pins aurora_ila_0/probe45]
# probe46-49: aurora_tx1_arbiter_0 輸出端（送給實體 aurora_64b66b_1/s_axi_tx_*，
#             這是離開這片板子前的最後一個觀測點）
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/tx1_tdata]  [get_bd_pins aurora_ila_0/probe46]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/tx1_tvalid] [get_bd_pins aurora_ila_0/probe47]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/tx1_tready] [get_bd_pins aurora_ila_0/probe48]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/tx1_tlast]  [get_bd_pins aurora_ila_0/probe49]

# 2026-07-28 新增：aurora_tx1_arbiter_0 內部仲裁狀態（ST_IDLE=0/ST_DATA=1/
# ST_CTRL=2/ST_REPLY=3），懷疑三路仲裁改版後 ctrl/reply 其中一路卡住不放
# 導致 data（relay 出去的封包）永遠輪不到，見 rtl/aurora_tx1_arbiter.v
# dbg_state port 註解、PROJECT.md/NOTES.md 2026-07-28 排查記錄。
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/dbg_state]  [get_bd_pins aurora_ila_0/probe70]

# 2026-07-28 新增，同一天改版（Opus 覆查抓到第一版 watchdog 的
# lockout 邏輯有致命 bug，見 rtl/aurora_tx1_arbiter.v 檔頭註解，這是
# 修正版）：ST_CTRL/ST_REPLY watchdog 逾時次數（飽和計數，不是單一
# sticky bit，才能分辨「發生過一次」跟「持續發生」）。正常運作應該
# 永遠是 0；如果上機後變非 0，代表 probe70 卡住的懷疑（reply FIFO
# reset race，見 NOTES.md「async_fifo_reply_tx reset race 用 xsim
# 真實 IP 重現」章節）是真的發生過，watchdog 只是防止 data 永久餓死，
# 不是根因已經修好。
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/dbg_ctrl_watchdog_count]  [get_bd_pins aurora_ila_0/probe71]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/dbg_reply_watchdog_count] [get_bd_pins aurora_ila_0/probe73]

# 2026-07-28 新增，同一天改版（Opus 覆查抓到第一版用純計時當
# watchdog 判據會誤傷合法長封包，見 rtl/aurora_data_channel_
# relayfifo.v 檔頭 local_pkt_budget 說明，這是修正版，改用封包自己
# 宣告的 pkt_len 當預算）：aurora_data_channel_0（=
# aurora_data_channel_relayfifo 模組）TX_LOCAL/TX_DRAIN(local_tx)
# watchdog 逾時次數（飽和計數）——async_fifo_aurora_tx 已經直接用
# xsim 真實 IP 證實跟 async_fifo_reply_tx 有一樣的 reset race，這裡
# 是對稱的第二道 watchdog 防線。跟 probe51-55（async_fifo_aurora_tx
# 自己的 full/empty/prog_full/wr_rst_busy/rd_rst_busy）一起看，可以
# 同時確認「FIFO 本身有沒有異常」跟「異常有沒有導致 TX_LOCAL 卡住、
# watchdog 有沒有救回來」。
connect_bd_net [get_bd_pins aurora_data_channel_0/dbg_tx_local_watchdog_count] [get_bd_pins aurora_ila_0/probe72]

# 2026-07-28 新增（Phase 10 T_QUERY/T_STATUS_REPORT 多種 query_type
# 上機失敗排查，見 PROJECT.md/NOTES.md 同日「Linux 機器接手：xsim
# 重現『跨查詢順序性』」章節）：這條路徑（aurora_reply_tx_0/
# async_fifo_reply_tx/aurora_tx1_arbiter_0 的 reply_tx1_*/local_reg_
# handler_0 的 T_QUERY 解碼跟 T_STATUS_REPORT 解碼）先前完全沒有接
# 任何 ILA probe——xsim 已經用兩支互補測試台（單板含真實雙向 async_
# fifo_reply_tx IP、完整 3 板環路含全部真實 RTL）證實數位邏輯層級
#沒有 bug（両支都 errors=0），這批新 probe 是為了直接在硬體上觀察
# xsim 模不到的部分（實體 GT/SFP 時序、真正雙時脈在多站環路下的
# 交互行為）。

# probe74-75: local_reg_handler_0 送出查詢那端（T_QUERY 解碼結果）——
# req_reply 有沒有正確 fire、req_query_type 對不對
connect_bd_net [get_bd_pins local_reg_handler_0/req_reply]      [get_bd_pins aurora_ila_0/probe74]
connect_bd_net [get_bd_pins local_reg_handler_0/req_query_type] [get_bd_pins aurora_ila_0/probe75]

# probe76-79: local_reg_handler_0 收回覆那端（T_STATUS_REPORT 解碼
# 結果）——au_reply_wr 到底有沒有 fire（區分「完全沒收到」vs「收到但
# 內容不對」）、query_type/src/reply_dest_id 用來確認路由對不對（跟
# 一開始誤判「dest_id 永遠是 0」那個已撤回的假設對照用）
connect_bd_net [get_bd_pins local_reg_handler_0/au_reply_wr]         [get_bd_pins aurora_ila_0/probe76]
connect_bd_net [get_bd_pins local_reg_handler_0/au_reply_query_type] [get_bd_pins aurora_ila_0/probe77]
connect_bd_net [get_bd_pins local_reg_handler_0/au_reply_src]        [get_bd_pins aurora_ila_0/probe78]
connect_bd_net [get_bd_pins local_reg_handler_0/reply_dest_id]       [get_bd_pins aurora_ila_0/probe79]

# probe80-82: aurora_reply_tx_0 的輸出握手（reply_tx1 FSM 有沒有卡住、
# 下游 FIFO 有沒有一直不 ready）
connect_bd_net [get_bd_pins aurora_reply_tx_0/reply_tvalid] [get_bd_pins aurora_ila_0/probe80]
connect_bd_net [get_bd_pins aurora_reply_tx_0/reply_tready] [get_bd_pins aurora_ila_0/probe81]
connect_bd_net [get_bd_pins aurora_reply_tx_0/reply_tlast]  [get_bd_pins aurora_ila_0/probe82]

# probe83-84: aurora_tx1_arbiter_0 的 reply_tx1 這組輸入（跟現有
# probe46-49 只看得到三路仲裁後的最終輸出互補——這組才看得到 reply
# 這條線單獨進來時，仲裁器有沒有實際選中它、FIFO 有沒有把資料交出來）
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/reply_tx1_tvalid] [get_bd_pins aurora_ila_0/probe83]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/reply_tx1_tready] [get_bd_pins aurora_ila_0/probe84]

# probe85-88: async_fifo_reply_tx 自己的狀態旗標——跟 probe51-55
# （async_fifo_aurora_tx，data/relay 用的那顆）同一套做法，鏡像
# 套用到 reply 專用的這顆 FIFO 上，直接看 full/empty 有沒有互相矛盾、
# reset busy 旗標有沒有卡住
connect_bd_net [get_bd_pins async_fifo_reply_tx/prog_full]   [get_bd_pins aurora_ila_0/probe85]
connect_bd_net [get_bd_pins async_fifo_reply_tx/empty]       [get_bd_pins aurora_ila_0/probe86]
connect_bd_net [get_bd_pins async_fifo_reply_tx/wr_rst_busy] [get_bd_pins aurora_ila_0/probe87]
connect_bd_net [get_bd_pins async_fifo_reply_tx/rd_rst_busy] [get_bd_pins aurora_ila_0/probe88]

# 2026-07-29 新增（垃圾封包 storm 根因修復，見 rtl/aurora_ctrl_channel.v
# dbg_rx_src_rejected/dbg_bwd_src_rejected port 註解、NOTES.md 2026-07-28/29
# 章節）：src_id 合理性檢查擋下垃圾封包時的偵測 pulse，直接接 ILA，
# 之後拉波形能直接看到「這一站有沒有擋下過垃圾封包」。⚠️ 第一版
# 誤放在 aurora_ila_0 這個 cell 真正被建立（下面 create_bd_cell）
# 之前的一段更早的程式碼位置，導致 get_bd_pins 找不到 probe89/90
# 這兩個 pin（那時候 aurora_ila_0 根本還沒被建立過），
# connect_bd_net 直接報錯、後面整個 BD 留在破碎狀態，place_design
# 因此爆出 1052 個頂層 I/O port 的離譜錯誤——已修正搬到這裡（跟其他
# probe74-88 同一個位置，aurora_ila_0 已經確定存在之後）。
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/dbg_rx_src_rejected]  [get_bd_pins aurora_ila_0/probe89]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/dbg_bwd_src_rejected] [get_bd_pins aurora_ila_0/probe90]

# 2026-07-29 新增：board_index_cdc_0 改用 xpm_cdc_handshake 對照實驗用
# （見 board_index_cdc_0 建立處註解、NOTES.md 對應章節）。這是 CDC 前
# （aurora_clk domain，跟這顆 ILA 同一個 clock，純接線不需要額外 CDC）
# 的原始值，拿來跟 CDC 後、WO 0x26（concat_enum_status）讀到的值比對。
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/board_index] [get_bd_pins aurora_ila_0/probe91]

# probe50: aurora_data_channel_0/local_out_tdata (receiver side, paired with the
#          existing probe21's local_tvalid -- probe21 only has valid, not
#          tdata itself; added this time to confirm whether the content is
#          correct after the receiving board's aurora_data_channel_0 decides
#          "this is mine", or whether it's already wrong before merging with
#          aurora_rx_merge_0)
connect_bd_net [get_bd_pins aurora_data_channel_0/local_out_tdata] [get_bd_pins aurora_ila_0/probe50]
# probe51-55 (added 2026-07-17, debugging a large T_WAVEFORM_STREAM local
# origination that deadlocks partway through): async_fifo_aurora_tx's own
# status flags. ILA already confirmed that when stuck,
# dispatcher_0/tx_tready=0 (= inverted async_fifo_aurora_tx/prog_full) while
# aurora_data_channel_0/local_tx_tvalid=1 with the data frozen -- the write
# side thinks it's full while the read side's data isn't advancing, both
# ends stuck simultaneously. Suspect this FIFO itself has an issue under a
# scenario it was never actually stress-tested at before: a transfer far
# exceeding depth 256, needing many rounds of "fill up -> drain" cycles.
# These probes look directly at the FIFO's internal flags instead of
# inferring from surrounding signals: full/empty being contradictory would
# directly show the valid/empty signals conflicting with each other;
# wr_rst_busy/rd_rst_busy rule out the possibility of "stuck in a
# post-reset busy state".
connect_bd_net [get_bd_pins async_fifo_aurora_tx/full]        [get_bd_pins aurora_ila_0/probe51]
connect_bd_net [get_bd_pins async_fifo_aurora_tx/empty]       [get_bd_pins aurora_ila_0/probe52]
connect_bd_net [get_bd_pins async_fifo_aurora_tx/prog_full]   [get_bd_pins aurora_ila_0/probe53]
connect_bd_net [get_bd_pins async_fifo_aurora_tx/wr_rst_busy] [get_bd_pins aurora_ila_0/probe54]
connect_bd_net [get_bd_pins async_fifo_aurora_tx/rd_rst_busy] [get_bd_pins aurora_ila_0/probe55]

# probe56-63 (added 2026-07-21, debugging aurora_data_channel_0
# [= aurora_data_channel_relayfifo module]'s TX_RELAY state being
# permanently stuck with debug_pkt_busy toggling at high frequency --
# suspect rx0 noise/idle characters being misclassified as legitimate
# small packets by the RX-side classification logic (no CRC/validity
# check, only excludes the 6 Layer-3-owned types), continuously feeding
# the relay FIFO. probe56-60 verify the classification decision itself
# (rx_state/cur_is_mine/cur_to_relay/cur_to_local are registered,
# beat0_to_relay is the combinational same-cycle decision); probe61-62
# verify the relay FIFO write side isn't overflowing (relay_wr_full is
# the external fifo_generator's full flag tapped directly off
# aurora_data_channel_0's input pin; dbg_overflow is the pre-existing
# but never-wired relay_wr_en&&relay_wr_full flag).
connect_bd_net [get_bd_pins aurora_data_channel_0/dbg_rx_state]       [get_bd_pins aurora_ila_0/probe56]
connect_bd_net [get_bd_pins aurora_data_channel_0/dbg_cur_is_mine]    [get_bd_pins aurora_ila_0/probe57]
connect_bd_net [get_bd_pins aurora_data_channel_0/dbg_cur_to_relay]   [get_bd_pins aurora_ila_0/probe58]
connect_bd_net [get_bd_pins aurora_data_channel_0/dbg_cur_to_local]   [get_bd_pins aurora_ila_0/probe59]
connect_bd_net [get_bd_pins aurora_data_channel_0/dbg_beat0_to_relay] [get_bd_pins aurora_ila_0/probe60]
connect_bd_net [get_bd_pins aurora_data_channel_0/relay_wr_full]      [get_bd_pins aurora_ila_0/probe61]
connect_bd_net [get_bd_pins aurora_data_channel_0/dbg_overflow]       [get_bd_pins aurora_ila_0/probe62]
connect_bd_net [get_bd_pins aurora_data_channel_0/relay_wr_en]        [get_bd_pins aurora_ila_0/probe63]

# probe64-65 (added 2026-07-21, verifying the src_id plausibility-check
# loop-termination fix -- see rtl/aurora_data_channel_relayfifo.v port
# declaration comments for the full rationale): probe64 is the RX-side
# rejection pulse (beat0 classified as data/other but src_id out of
# range -- never enters ST_DATA tracking); probe65 is the TX-side drain
# level (draining an already-queued invalid-src packet without ever
# asserting data_tx1_tvalid for it).
connect_bd_net [get_bd_pins aurora_data_channel_0/dbg_rx_src_rejected] [get_bd_pins aurora_ila_0/probe64]
connect_bd_net [get_bd_pins aurora_data_channel_0/dbg_tx_drain]        [get_bd_pins aurora_ila_0/probe65]

# probe66-69 (added 2026-07-22, debugging TX_RELAY getting permanently
# stuck with the relay FIFO empty -- aurora_data_channel_relayfifo.v's
# TX_RELAY state has no defense against relay_has_data unexpectedly
# dropping before a tlast beat is ever seen, so it waits forever reading
# the FIFO's stale/frozen dout register). These four let us directly
# compare what's written into the relay FIFO vs what's read back out,
# beat by beat, to find whether the write side ever produced a
# tlast-marked beat for whatever packet is stuck, and to identify (via
# the src_id field inside the captured tdata) which board actually sent
# it. probe66/probe67 tap the FIFO's own din/dout ports directly (not
# aurora_data_channel_0's renamed relay_rd_dout input) to match this
# file's existing convention for probing FIFO-status signals (see
# probe51-55).
connect_bd_net [get_bd_pins aurora_data_channel_0/relay_wr_din] [get_bd_pins aurora_ila_0/probe66]
connect_bd_net [get_bd_pins relay_fifo_0/dout]                  [get_bd_pins aurora_ila_0/probe67]
connect_bd_net [get_bd_pins relay_fifo_0/valid]                 [get_bd_pins aurora_ila_0/probe68]
connect_bd_net [get_bd_pins aurora_data_channel_0/relay_rd_en]  [get_bd_pins aurora_ila_0/probe69]
}

# 2026-07-29 新增：reply 寫入端 USB 可讀診斷（不需要 ILA）——
# dbg_reply_seen/dbg_reply_granted 是 aurora_tx1_arbiter.v 內部的
# sticky（aurora_clk domain），這裡跨到 sys_clk 餵進 concat_reserve_
# diag（WO 0x2d）目前空出來的 bit，見該 xlconcat 建立處的說明。
# 2026-08-05 移到這裡（原本誤放進 aurora_ila_0 的 if {$ENABLE_ILA} 區塊
# 內，明明註解自己都寫「不需要 ILA」卻被連坐關掉——這次 ENABLE_ILA
# 第一次真正 dry-run 測試 =0 才發現，reply_seen_cdc_0/reply_granted_
# cdc_0/src_in 變成 unconnected，CRITICAL WARNING 數比基準多 2 個。
# reply_seen_cdc_0/reply_granted_cdc_0 這兩個 cell 本身建立在 1632-1633
# 行，早於任何 ENABLE_ILA 判斷，本來就不該被这個開關影響）。
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/dbg_reply_seen]    [get_bd_pins reply_seen_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_tx1_arbiter_0/dbg_reply_granted] [get_bd_pins reply_granted_cdc_0/src_in]

# ==============================================================================
#  Dispatcher ILA: sys_clk-side end of the dispatcher<->aurora message boundary
# ==============================================================================
# 2026-07-06: dispatcher_0/rx_tdata,rx_tvalid,rx_tready is sys_clk domain (after
# the async_fifo_aurora_rx CDC FIFO), so it cannot share aurora_ila_0's clock --
# needs its own ILA. Added to see both ends of the same handoff: does whatever
# aurora_rx_merge_0 produces (aurora_ila_0/probe15-16) actually show up here
# with the same content, and does dispatcher_0 then correctly route it to
# local_reg_handler_0 (lrh_tdata/tvalid/tready) -- relevant to both the
# trigger-injection failure and the still-unexplained T_BOARD_CFG packet-path
# failure from earlier today.
if {$ENABLE_ILA} {
create_bd_cell -type ip -vlnv xilinx.com:ip:system_ila dispatcher_ila_0
# 2026-07-08：C_DATA_DEPTH 從 2048 改成 32768（原因見下方 git 歷史/舊註解：
# 比對 aurora_ila_0 的擷取窗口）。
# 2026-07-10：32768 改回 4096。這是全部 ILA 裡 depth 最大的一個，這次
# JTAG 讀取這顆時持續回報「Waveform data read from ILA core is
# corrupted」（連降 JTAG clock 到 5MHz 都沒用，其餘 depth 較小的 ILA
# 都正常），懷疑資料量過大是主因；也順便減少 BRAM 用量（見第 31 節
# system_ila_1 那次 DRC UTLZ-1）。4096 samples @ 100MHz ≈ 40.96us，
# 不再刻意比對 aurora_ila_0 的擷取窗口長度。
set_property -dict [list \
    CONFIG.C_MON_TYPE      {NATIVE} \
    CONFIG.C_NUM_OF_PROBES {35} \
    CONFIG.C_DATA_DEPTH    {1024} \
] [get_bd_cells dispatcher_ila_0]
set_property -dict [list \
    CONFIG.C_PROBE0_WIDTH  {64} \
    CONFIG.C_PROBE1_WIDTH  {1}  \
    CONFIG.C_PROBE2_WIDTH  {1}  \
    CONFIG.C_PROBE3_WIDTH  {64} \
    CONFIG.C_PROBE4_WIDTH  {1}  \
    CONFIG.C_PROBE5_WIDTH  {1}  \
    CONFIG.C_PROBE6_WIDTH  {16} \
    CONFIG.C_PROBE7_WIDTH  {64} \
    CONFIG.C_PROBE8_WIDTH  {1}  \
    CONFIG.C_PROBE9_WIDTH  {1}  \
    CONFIG.C_PROBE10_WIDTH {64} \
    CONFIG.C_PROBE11_WIDTH {1}  \
    CONFIG.C_PROBE12_WIDTH {1}  \
    CONFIG.C_PROBE13_WIDTH {16} \
    CONFIG.C_PROBE14_WIDTH {1}  \
    CONFIG.C_PROBE15_WIDTH {1}  \
    CONFIG.C_PROBE16_WIDTH {1}  \
    CONFIG.C_PROBE17_WIDTH {1}  \
    CONFIG.C_PROBE18_WIDTH {4}  \
    CONFIG.C_PROBE19_WIDTH {4}  \
    CONFIG.C_PROBE20_WIDTH {1}  \
    CONFIG.C_PROBE21_WIDTH {1}  \
    CONFIG.C_PROBE22_WIDTH {1}  \
    CONFIG.C_PROBE23_WIDTH {64} \
    CONFIG.C_PROBE24_WIDTH {1}  \
    CONFIG.C_PROBE25_WIDTH {1}  \
    CONFIG.C_PROBE26_WIDTH {1}  \
    CONFIG.C_PROBE27_WIDTH {1}  \
    CONFIG.C_PROBE28_WIDTH {1}  \
    CONFIG.C_PROBE29_WIDTH {1}  \
    CONFIG.C_PROBE30_WIDTH {3}  \
    CONFIG.C_PROBE31_WIDTH {64} \
    CONFIG.C_PROBE32_WIDTH {1}  \
    CONFIG.C_PROBE33_WIDTH {1}  \
    CONFIG.C_PROBE34_WIDTH {1}  \
] [get_bd_cells dispatcher_ila_0]
# probe23-29（2026-07-09，burn-in 發現 flash write 100% 失敗後新增，見
# PROJECT.md 第 27 節）：
#   probe23-25: dispatcher_0/flash_tdata,flash_tvalid,flash_tready --
#               確認 dispatcher 有沒有真的把 RT_FLASH 路由的資料送出去
#   probe26-29: flash_payload_cdc_0 內部除錯訊號（拆字狀態機/FIFO 寫側）
connect_bd_net [get_bd_pins dispatcher_0/flash_tdata]  [get_bd_pins dispatcher_ila_0/probe23]
connect_bd_net [get_bd_pins dispatcher_0/flash_tvalid] [get_bd_pins dispatcher_ila_0/probe24]
connect_bd_net [get_bd_pins dispatcher_0/flash_tready] [get_bd_pins dispatcher_ila_0/probe25]
connect_bd_net [get_bd_pins flash_payload_cdc_0/dbg_fifo_wr_en]       [get_bd_pins dispatcher_ila_0/probe26]
connect_bd_net [get_bd_pins flash_payload_cdc_0/dbg_unpack_half]      [get_bd_pins dispatcher_ila_0/probe27]
connect_bd_net [get_bd_pins flash_payload_cdc_0/dbg_fifo_prog_full]   [get_bd_pins dispatcher_ila_0/probe28]
connect_bd_net [get_bd_pins flash_payload_cdc_0/dbg_fifo_wr_rst_busy] [get_bd_pins dispatcher_ila_0/probe29]

connect_bd_net [get_bd_pins clk_wiz_0/clk_100] [get_bd_pins dispatcher_ila_0/clk]

connect_bd_net [get_bd_pins dispatcher_0/rx_tdata]  [get_bd_pins dispatcher_ila_0/probe0]
connect_bd_net [get_bd_pins dispatcher_0/rx_tvalid] [get_bd_pins dispatcher_ila_0/probe1]
connect_bd_net [get_bd_pins dispatcher_0/rx_tready] [get_bd_pins dispatcher_ila_0/probe2]
connect_bd_net [get_bd_pins dispatcher_0/lrh_tdata]  [get_bd_pins dispatcher_ila_0/probe3]
connect_bd_net [get_bd_pins dispatcher_0/lrh_tvalid] [get_bd_pins dispatcher_ila_0/probe4]
connect_bd_net [get_bd_pins dispatcher_0/lrh_tready] [get_bd_pins dispatcher_ila_0/probe5]
connect_bd_net [get_bd_pins dispatcher_0/board_id]   [get_bd_pins dispatcher_ila_0/probe6]
# probe7-9 (2026-07-06): host BTPipeIn side (fp_tdata/tvalid/tready) -- T_BOARD_CFG
# comes in via THIS input (not rx_tdata, which is Aurora-side only), and
# diag_lrh_rx_seen stayed 0 for it earlier today. This lets us see directly
# whether dispatcher_0 ever even sees the host packet at ST_IDLE, and if so,
# whether idle_fire/idle_route decode it correctly before it should reach
# lrh_tdata/tvalid above.
connect_bd_net [get_bd_pins dispatcher_0/fp_tdata]  [get_bd_pins dispatcher_ila_0/probe7]
connect_bd_net [get_bd_pins dispatcher_0/fp_tvalid] [get_bd_pins dispatcher_ila_0/probe8]
connect_bd_net [get_bd_pins dispatcher_0/fp_tready] [get_bd_pins dispatcher_ila_0/probe9]

# probe10-20（2026-07-08 第二輪，T_RESERVE_START/T_FLASH_ERASE 除錯用）：
# 上面 probe0-9 只看得到 dispatcher_0 本身的輸出（lrh_tdata/tvalid），完全
# 看不到 local_reg_handler_0 解碼封包後產生的 au_reserve_start/au_flash_erase
# 這兩個關鍵訊號本身 -- 這是 reserve 跟 flash_erase 兩個 bug 共同、目前完全
# 沒驗證過的第一個環節（封包到底有沒有解碼成功，CDC/mux/gate 都還沒介入）。
#   probe10-11: local_reg_handler_0 自己的 rx_tdata/tvalid（async_fifo_local
#               輸出端，確認 FIFO 沒有搞丟/弄壞封包內容）
#   probe12-13: au_reserve_start/au_reserve_dest_id -- CDC 之前，reserve 封包
#               有沒有真的解碼成功，數值對不對（跟 aurora_ila_0 現有的
#               probe22-23，即 CDC 之後的版本，前後對照）
#   probe14:    dispatcher_0/diag_state -- 2026-07-10 改接：原本接
#               au_flash_save，但那是已移除的舊 T_FLASH_SAVE 封包觸發路徑
#               （host 端 2026-07-09 起已不再送 0x0E 封包，見 PROJECT.md 第
#               29 節），此訊號在目前架構下永遠不會 fire，誤導了兩輪診斷，
#               已清掉（見 rtl/local_reg_handler.v）。騰出的 probe14 改看
#               dispatcher_0 自己的封包路由狀態機（ST_IDLE=0/ST_DATA=1），
#               用來確認 fp_tready 卡死時 dispatcher 是不是卡在 ST_DATA。
#   probe15:    au_flash_erase -- flash_erase 封包有沒有真的解碼成功
#   probe16:    aurora_ctrl_mux_0/out_flash_erase -- 有沒有正確穿過 mux
#               （純 assign，理論上該跟 probe15 一樣，若不同代表 mux 本身
#               或這段 BD 接線有問題）
#   probe17:    fpga_flash_ctrl_0/fp_flash_cfg_erase -- 有沒有正確接到頂層
#               模組（跟 probe16 之間是 BD 接線，可能漏接）
#   probe18:    fpga_flash_ctrl_0/flash_status[3:0] -- busy/done/err/
#               loader_done，跟上面訊號同步對照，確認 pulse 有沒有真的
#               觸發狀態機
#   probe19-20: flash_config_writer 內部 state/do_program（新增
#               diag_state/diag_do_program 純 assign 診斷 port，local copy，
#               見 rtl/flash_config_writer.v）-- 如果 probe17/18 都正常但
#               busy 沒被觸發，就要看內部狀態機真正卡在哪一個 state
connect_bd_net [get_bd_pins local_reg_handler_0/rx_tdata]         [get_bd_pins dispatcher_ila_0/probe10]
connect_bd_net [get_bd_pins local_reg_handler_0/rx_tvalid]        [get_bd_pins dispatcher_ila_0/probe11]
connect_bd_net [get_bd_pins local_reg_handler_0/au_reserve_start] [get_bd_pins dispatcher_ila_0/probe12]
connect_bd_net [get_bd_pins local_reg_handler_0/au_reserve_dest_id] [get_bd_pins dispatcher_ila_0/probe13]
connect_bd_net [get_bd_pins dispatcher_0/diag_state]              [get_bd_pins dispatcher_ila_0/probe14]
connect_bd_net [get_bd_pins dispatcher_0/diag_idle_route]         [get_bd_pins dispatcher_ila_0/probe30]
# probe31-34（2026-07-16 新增，第六輪）：排查 T_DDR4_WRITE(0x04) Aurora
# 遠端寫入送不出去的問題，dispatcher_0 自己的 TX 輸出（判斷完 RT_TX
# 之後，實際送進 async_fifo_aurora_tx 之前）完全沒被探測過，這是「dispatcher
# 判斷正確 -> 封包實際送出去」這條路徑上第一個節點。
connect_bd_net [get_bd_pins dispatcher_0/tx_tdata]  [get_bd_pins dispatcher_ila_0/probe31]
connect_bd_net [get_bd_pins dispatcher_0/tx_tvalid] [get_bd_pins dispatcher_ila_0/probe32]
connect_bd_net [get_bd_pins dispatcher_0/tx_tready] [get_bd_pins dispatcher_ila_0/probe33]
connect_bd_net [get_bd_pins dispatcher_0/tx_tlast]  [get_bd_pins dispatcher_ila_0/probe34]
# 2026-07-10：原本這裡有 probe31-35 看 fp_input_0 寫入端訊號，但那些是
# ok_clk domain、跟 dispatcher_ila_0(sys_clk)沒做 CDC，有 metastability
# 風險。改把 fp_input_wr_0/fp_fifo_0 這幾個 ok_clk 訊號接到同樣是 ok_clk
# 域的 flash_ctrl_ila_0（見下方該 ILA 的 probe14-18），不需要跨域採樣。
connect_bd_net [get_bd_pins local_reg_handler_0/au_flash_erase]   [get_bd_pins dispatcher_ila_0/probe15]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_flash_erase]    [get_bd_pins dispatcher_ila_0/probe16]
# 2026-07-09 已知缺口（維持接線，不拆掉，避免 probe port 懸空造成
# validate_bd_design/synthesis 報錯）：fpga_flash_ctrl_0 搬到 okClk 域
# 後，probe17-20 這 4 個訊號（fp_flash_cfg_erase/flash_status/
# diag_cfg_state/diag_cfg_do_program）都變成 okClk 域，但
# dispatcher_ila_0 本身是 sys_clk 域（clk_wiz_0/clk_100 -> clk），這 4
# 條變成沒做 CDC 的純觀察用接線——ILA 是被動採樣，不會造成功能性
# bug，但在時脈交界瞬間可能偶爾採到過渡態、capture 出來的波形不完全
# 可靠。之後如果要正式用這幾個 probe 判讀時序，應該先比照
# flash_load_valid_cdc_0 的做法過 trigger_cdc 再接，或另開一個 okClk
# 域的 ILA。見 PROJECT.md 第 25 節。
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/fp_flash_cfg_erase] [get_bd_pins dispatcher_ila_0/probe17]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/flash_status]       [get_bd_pins dispatcher_ila_0/probe18]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/diag_cfg_state]      [get_bd_pins dispatcher_ila_0/probe19]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/diag_cfg_do_program] [get_bd_pins dispatcher_ila_0/probe20]

# probe21-22（2026-07-08 第四輪，debug-only TI-direct 除錯路徑用）：
#   probe21: ti_au_reserve/Dout -- TI bit31 直接觸發的原始 pulse，這條路徑
#            完全繞過封包解碼，理論上應該一定會 fire
#   probe22: or_reserve_pulse/Res -- OR 過後、真正進 CDC 的合併 pulse，
#            跟 probe12（純封包路徑）對照，確認 OR gate 本身有沒有正常運作
connect_bd_net [get_bd_pins ti_au_reserve/Dout]   [get_bd_pins dispatcher_ila_0/probe21]
connect_bd_net [get_bd_pins or_reserve_pulse/Res] [get_bd_pins dispatcher_ila_0/probe22]
}

# ==============================================================================
#  Flash CTRL ILA (okClk domain) -- 2026-07-09 新增
# ==============================================================================
# burn-in 發現 flash write 100% 失敗（raw_magic 讀回全 0）後新增，見
# PROJECT.md 第 27 節。這是專案裡第一個 okClk 域的 ILA（既有 4 個分別是
# sys_clk/aurora_clk/ddr4_clk/dac_clk），用來確認 flash_payload_cdc_0
# 的 FIFO 讀側到 fpga_flash_ctrl_0 這一段，資料有沒有真的送達。
if {$ENABLE_ILA} {
create_bd_cell -type ip -vlnv xilinx.com:ip:system_ila flash_ctrl_ila_0
# 2026-07-10：新增 probe14-18（fp_input_wr_0/fp_fifo_0 寫入端訊號，見下方）
# 讓總 bit 數大幅增加，C_DATA_DEPTH 16384->8192 維持 BRAM 用量大致不變，
# 避免重踩第 31 節的 DRC UTLZ-1。
set_property -dict [list \
    CONFIG.C_MON_TYPE      {NATIVE} \
    CONFIG.C_NUM_OF_PROBES {19} \
    CONFIG.C_DATA_DEPTH    {1024} \
] [get_bd_cells flash_ctrl_ila_0]
# 2026-07-09（第 29 節）：拿掉 flash_save_gate 相關的 probe（該設計已
# 移除，見上方 cell 建立處註解）。改成直接驗證新架構：payload_complete
# 本身就是觸發訊號，直接接 fp_flash_cfg_wr，這裡再多探一個
# fp_flash_cfg_wr 本身確認自動觸發真的有到達 fpga_flash_ctrl_0。
# 2026-07-09（第 30 節）：CDC/觸發鏈路已確認正常（payload_complete 有
# 正確觸發），但寫入結果仍是空的，新增 probe5-13 直接觀察 SPI 實體
# 訊號跟 spi_flash_ctrl 內部狀態機，見 rtl/fpga_flash_ctrl.v 新增的
# dbg_spi_* 除錯輸出。C_DATA_DEPTH 拉大到 16384 增加涵蓋 WREN/SE 等
# 初始指令序列的機會。
set_property -dict [list \
    CONFIG.C_PROBE0_WIDTH {32} \
    CONFIG.C_PROBE1_WIDTH {1}  \
    CONFIG.C_PROBE2_WIDTH {32} \
    CONFIG.C_PROBE3_WIDTH {1}  \
    CONFIG.C_PROBE4_WIDTH {1}  \
    CONFIG.C_PROBE5_WIDTH {1}  \
    CONFIG.C_PROBE6_WIDTH {1}  \
    CONFIG.C_PROBE7_WIDTH {1}  \
    CONFIG.C_PROBE8_WIDTH {1}  \
    CONFIG.C_PROBE9_WIDTH {4}  \
    CONFIG.C_PROBE10_WIDTH {3} \
    CONFIG.C_PROBE11_WIDTH {1} \
    CONFIG.C_PROBE12_WIDTH {1} \
    CONFIG.C_PROBE13_WIDTH {1} \
    CONFIG.C_PROBE14_WIDTH {1}  \
    CONFIG.C_PROBE15_WIDTH {32} \
    CONFIG.C_PROBE16_WIDTH {1}  \
    CONFIG.C_PROBE17_WIDTH {1}  \
    CONFIG.C_PROBE18_WIDTH {64} \
] [get_bd_cells flash_ctrl_ila_0]
connect_bd_net [get_bd_pins fp0/okClk] [get_bd_pins flash_ctrl_ila_0/clk]
# probe0-1: flash_payload_cdc_0 的 FIFO 讀側輸出（CDC 之後、進 flash_ctrl 之前）
# probe2-3: fpga_flash_ctrl_0 實際收到的 pipe_wdata/pipe_wvalid（跟 probe0-1
#           應該完全一樣，因為中間是直接接線；如果不一樣代表 BD 接線本身有問題）
connect_bd_net [get_bd_pins flash_payload_cdc_0/out_data]   [get_bd_pins flash_ctrl_ila_0/probe0]
connect_bd_net [get_bd_pins flash_payload_cdc_0/out_valid]  [get_bd_pins flash_ctrl_ila_0/probe1]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/pipe_wdata]   [get_bd_pins flash_ctrl_ila_0/probe2]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/pipe_wvalid]  [get_bd_pins flash_ctrl_ila_0/probe3]
# probe4（2026-07-09，第 29 節新架構驗證用）：payload_complete 現在
# 直接就是 fp_flash_cfg_wr 的來源，這裡確認它收滿 64 個 word 時真的有
# 脈衝一次
connect_bd_net [get_bd_pins flash_payload_cdc_0/payload_complete] [get_bd_pins flash_ctrl_ila_0/probe4]
# probe5-13（2026-07-09，第 30 節）：SPI 實體訊號 + spi_flash_ctrl 內部
# 狀態機，直接觀察抹除/寫入指令有沒有正確送出、SPI clock 波形是否正常
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/dbg_spi_clk]   [get_bd_pins flash_ctrl_ila_0/probe5]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/dbg_spi_cs_n]  [get_bd_pins flash_ctrl_ila_0/probe6]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/dbg_spi_mosi]  [get_bd_pins flash_ctrl_ila_0/probe7]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/dbg_spi_miso]  [get_bd_pins flash_ctrl_ila_0/probe8]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/dbg_spi_state] [get_bd_pins flash_ctrl_ila_0/probe9]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/dbg_spi_op_cmd] [get_bd_pins flash_ctrl_ila_0/probe10]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/dbg_spi_busy]  [get_bd_pins flash_ctrl_ila_0/probe11]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/dbg_spi_done]  [get_bd_pins flash_ctrl_ila_0/probe12]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/dbg_spi_err]   [get_bd_pins flash_ctrl_ila_0/probe13]
# probe14-18（2026-07-10，第 32 節：追查 dispatcher 收到的 fp_tdata 讀到
# 全 0 的問題）：fp_input_wr_0/fp_fifo_0 寫入端訊號，跟 flash_ctrl_ila_0
# 本身同樣是 ok_clk domain（fp0/okClk），不需要跨域採樣，比原本誤放在
# sys_clk 域 dispatcher_ila_0 上更可靠。
connect_bd_net [get_bd_pins fp_input_wr_0/pi_write]    [get_bd_pins flash_ctrl_ila_0/probe14]
connect_bd_net [get_bd_pins fp_input_wr_0/pi_data]     [get_bd_pins flash_ctrl_ila_0/probe15]
connect_bd_net [get_bd_pins fp_input_wr_0/ep_ready]    [get_bd_pins flash_ctrl_ila_0/probe16]
connect_bd_net [get_bd_pins fp_input_wr_0/fifo_wr_en]  [get_bd_pins flash_ctrl_ila_0/probe17]
connect_bd_net [get_bd_pins fp_input_wr_0/fifo_din]    [get_bd_pins flash_ctrl_ila_0/probe18]
}

# ==============================================================================
#  ddr_writer ILA (sys_clk domain) -- 2026-07-15 新增
# ==============================================================================
# 排查 T_WAVEFORM_STREAM(0x13) 寫入 DDR4 內容錯位問題（2026-07-15 用
# test_waveform_stream_write.py 重新驗證，48/64 樣本錯誤，規律是每 4 個
# 樣本只有第 1 個正確——懷疑是 64->128 pair-acc 或 AXI burst-write 狀態機
# 的 wr_ptr/rd_ptr 沒有正確同步，見 PROJECT.md「T_WAVEFORM_STREAM」章節）。
# 獨立小型 ILA，不跟已經有 BRAM 壓力歷史的 dispatcher_ila_0 搶 probe，
# depth 刻意保守（2048，遠小於曾經造成 JTAG 讀取失敗的 32768/4096）。
if {$ENABLE_ILA} {
create_bd_cell -type ip -vlnv xilinx.com:ip:system_ila ddr_writer_ila_0
set_property -dict [list \
    CONFIG.C_MON_TYPE      {NATIVE} \
    CONFIG.C_NUM_OF_PROBES {24} \
    CONFIG.C_DATA_DEPTH    {1024} \
] [get_bd_cells ddr_writer_ila_0]
set_property -dict [list \
    CONFIG.C_PROBE0_WIDTH  {4}  \
    CONFIG.C_PROBE1_WIDTH  {2}  \
    CONFIG.C_PROBE2_WIDTH  {9}  \
    CONFIG.C_PROBE3_WIDTH  {1}  \
    CONFIG.C_PROBE4_WIDTH  {1}  \
    CONFIG.C_PROBE5_WIDTH  {8}  \
    CONFIG.C_PROBE6_WIDTH  {1}  \
    CONFIG.C_PROBE7_WIDTH  {32} \
    CONFIG.C_PROBE8_WIDTH  {32} \
    CONFIG.C_PROBE9_WIDTH  {1}  \
    CONFIG.C_PROBE10_WIDTH {1}  \
    CONFIG.C_PROBE11_WIDTH {32} \
    CONFIG.C_PROBE12_WIDTH {1}  \
    CONFIG.C_PROBE13_WIDTH {1}  \
    CONFIG.C_PROBE14_WIDTH {128} \
    CONFIG.C_PROBE15_WIDTH {1}  \
    CONFIG.C_PROBE16_WIDTH {1}  \
    CONFIG.C_PROBE17_WIDTH {1}  \
    CONFIG.C_PROBE18_WIDTH {1}  \
    CONFIG.C_PROBE19_WIDTH {1}  \
    CONFIG.C_PROBE20_WIDTH {2}  \
    CONFIG.C_PROBE21_WIDTH {64} \
    CONFIG.C_PROBE22_WIDTH {1}  \
    CONFIG.C_PROBE23_WIDTH {128} \
] [get_bd_cells ddr_writer_ila_0]
connect_bd_net [get_bd_pins clk_wiz_0/clk_100] [get_bd_pins ddr_writer_ila_0/clk]
# probe0: axi_state（ST_IDLE_A/ST_FILL/ST_AW/ST_W/ST_RESP）
# probe1: debug_bresp（AXI write response，非 OKAY 代表 DDR4 controller 拒絕寫入）
# probe2: debug_fifo_cnt（256-entry 128-bit pair-acc FIFO 目前深度）
# probe3: debug_pacc_wr_en（2026-07-16 第四輪改接，原本是 debug_wr_ptr，
#         但 xpm_fifo_sync 取代手寫指標後這個 port 已經寫死接 0、沒有
#         意義，改接 pacc_fifo_wr_en，qualify probe23/debug_pacc_din
#         哪一拍才是真正寫入的那筆）
# probe4: debug_in_tvalid（2026-07-16 第四輪改接，原本是 debug_rd_ptr，
#         同樣已死，改接 in_tvalid，qualify probe21/debug_in_tdata 哪一拍
#         有效）
# probe5: debug_rxbeatcnt（收到的 data beat 計數，確認封包 beat 數對不對）
# probe6: debug_half_valid（pair-acc 目前是不是還在等待第二個 64-bit beat）
# probe7: debug_wave_total_bytes（beat1 解析出的長度，確認 meta 解析正確）
# probe8: debug_remaining_beats（AXI 寫入端還剩多少 128-bit beat 要寫）
connect_bd_net [get_bd_pins ddr_writer_0/debug_state]            [get_bd_pins ddr_writer_ila_0/probe0]
connect_bd_net [get_bd_pins ddr_writer_0/debug_bresp]            [get_bd_pins ddr_writer_ila_0/probe1]
connect_bd_net [get_bd_pins ddr_writer_0/debug_fifo_cnt]         [get_bd_pins ddr_writer_ila_0/probe2]
connect_bd_net [get_bd_pins ddr_writer_0/debug_pacc_wr_en]       [get_bd_pins ddr_writer_ila_0/probe3]
connect_bd_net [get_bd_pins ddr_writer_0/debug_in_tvalid]        [get_bd_pins ddr_writer_ila_0/probe4]
connect_bd_net [get_bd_pins ddr_writer_0/debug_rxbeatcnt]        [get_bd_pins ddr_writer_ila_0/probe5]
connect_bd_net [get_bd_pins ddr_writer_0/debug_half_valid]       [get_bd_pins ddr_writer_ila_0/probe6]
connect_bd_net [get_bd_pins ddr_writer_0/debug_wave_total_bytes] [get_bd_pins ddr_writer_ila_0/probe7]
connect_bd_net [get_bd_pins ddr_writer_0/debug_remaining_beats]  [get_bd_pins ddr_writer_ila_0/probe8]
# probe9-17（2026-07-15 新增，第二輪）：ddr_writer_0 實際輸出的 m_axi_*
# AXI write channel（pair-acc 之後、跨進 ddr_writer_axi_fifo 非同步時脈域
# 之前——這條路徑後來又從 axi_cc_ddr4 換成 fifo_generator，見上方
# ddr_writer_axi_fifo 建立處的說明，這幾個 probe 沒有跟著改，接的還是
# ddr_writer_0 自己的輸出，不受下游換 cell 影響），用來分辨「連續第 3 次
# 寫入開始出錯」這個門檻是 ddr_writer_0 自己的邏輯問題，還是下游
# （現在是 ddr_writer_axi_fifo/smartconnect_0/ddr4_0，都是官方 IP）的
# 問題——如果這裡的 m_axi_wdata 已經是錯的，代表問題在 ddr_writer_0 這端；
# 如果這裡是對的，問題就在更下游。見 PROJECT.md「T_WAVEFORM_STREAM」章節。
# 2026-07-16 修正（第一次）：probe10/13/16（awready/wready/bvalid）原本接
# ddr_writer_0 自己的輸入 pin，這幾個訊號實際上是被 ddr_writer_axi_fifo
# 驅動、ddr_writer_0 只是接收端——ILA 的 probe 本身也是輸入端，兩個
# 「接收端」互相連接，Vivado synth 找不到真正的驅動來源，這幾個 port
# 判定成 unconnected（[Synth 8-7071]），implementation 階段 opt_design
# 又在 ddr_writer_ila_0 內部的 trigger matching unit 炸出 LUT 缺接腳的
# 錯誤（[Opt 31-65]）。改成直接接真正驅動這幾個訊號的一端
# （ddr_writer_axi_fifo 的 s_axi_awready/s_axi_wready/s_axi_bvalid，按
# AXI 協定這些訊號本來就是由 slave 端／這裡是 fifo_generator 驅動）。
#
# 2026-07-16 修正（第二次，同一天）：原本以為「ddr_writer_0 是驅動端的
# 訊號（awvalid/awaddr/wvalid/wdata/wlast/bready）維持直接接 ddr_writer_0
# 沒問題」，但上機重測 T_WAVEFORM_STREAM 仍卡死在 ST_AW，直接讀生成的
# 網表（awg_step16.gen/sources_1/bd/awg_step16_bd/synth/awg_step16_bd.v）
# 才發現：這 6 個訊號雖然是 m_axi interface 的成員 pin（ddr_writer.v 裡
# 有 X_INTERFACE_INFO 標註），額外對它們下 connect_bd_net 接 ILA，會讓
# Vivado 把這個 pin 從 m_axi 的 interface 整合網路裡排除——不管是這次的
# 輸出方向還是上次踩過的輸入方向，都會破壞 interface 連接，只是輸出方向
# 這次完全不會報錯，build 正常過，ddr_writer_axi_fifo 端收到的
# s_axi_awvalid/s_axi_awaddr/s_axi_wvalid/s_axi_wdata 全部被靜默 tie 常數
# 0（無聲無息，比輸入方向的報錯更難發現）。詳見 PROJECT.md
# 「T_WAVEFORM_STREAM」章節 2026-07-16 postmortem 跟
# [[feedback_ila_probe_interface_pin_driver_side]]。
# 修法：ddr_writer.v 新增 debug_axi_awvalid/awaddr/wvalid/wdata/wlast/
# bready 這 6 個鏡像輸出 port（純 assign 出來，跟 m_axi interface pin
# 完全分開），下面全部改接這幾個新 port，不再直接碰 m_axi_* 本身。
connect_bd_net [get_bd_pins ddr_writer_0/debug_axi_awvalid]  [get_bd_pins ddr_writer_ila_0/probe9]
connect_bd_net [get_bd_pins ddr_writer_axi_fifo/s_axi_awready] [get_bd_pins ddr_writer_ila_0/probe10]
connect_bd_net [get_bd_pins ddr_writer_0/debug_axi_awaddr]   [get_bd_pins ddr_writer_ila_0/probe11]
connect_bd_net [get_bd_pins ddr_writer_0/debug_axi_wvalid]   [get_bd_pins ddr_writer_ila_0/probe12]
connect_bd_net [get_bd_pins ddr_writer_axi_fifo/s_axi_wready]  [get_bd_pins ddr_writer_ila_0/probe13]
connect_bd_net [get_bd_pins ddr_writer_0/debug_axi_wdata]    [get_bd_pins ddr_writer_ila_0/probe14]
connect_bd_net [get_bd_pins ddr_writer_0/debug_axi_wlast]    [get_bd_pins ddr_writer_ila_0/probe15]
connect_bd_net [get_bd_pins ddr_writer_axi_fifo/s_axi_bvalid]  [get_bd_pins ddr_writer_ila_0/probe16]
connect_bd_net [get_bd_pins ddr_writer_0/debug_axi_bready]   [get_bd_pins ddr_writer_ila_0/probe17]
# probe18-20（2026-07-16 新增，第三輪）：pacc_fifo_inst（xpm_fifo_sync）
# 原本空接的進階 port，補上 ILA 可見度。起因：wr_data_count
# （debug_fifo_cnt/probe2）上機驗證懷疑不可靠（即使 USE_ADV_FEATURES
# 開對 bit[2]/EN_WDC，仍觀察到整個 capture window 都停在 0，即使
# rxbeatcnt/half_valid 顯示資料確實有收到、pair-acc 也確實 toggle
# 過），改把 ST_FILL 判斷式改成只看 !pacc_fifo_empty（不依賴
# wr_data_count），順便把 full/empty/wr_rst_busy/rd_rst_busy 也接出來，
# 之後如果還要排查這顆 FIFO，不用再等一輪重建才能看到這些訊號。
# probe18: debug_pacc_empty（pacc_fifo_inst 的 empty，標準 port，不受
#          USE_ADV_FEATURES 影響，理論上一定可靠）
# probe19: debug_pacc_full
# probe20: debug_pacc_rst_busy[1:0] = {wr_rst_busy, rd_rst_busy}（懷疑
#          方向之一：如果這兩個訊號重置後遲遲不降回0，代表pacc_fifo_
#          inst內部一直卡在reset序列，寫入會被silently擋掉）
connect_bd_net [get_bd_pins ddr_writer_0/debug_pacc_empty]    [get_bd_pins ddr_writer_ila_0/probe18]
connect_bd_net [get_bd_pins ddr_writer_0/debug_pacc_full]     [get_bd_pins ddr_writer_ila_0/probe19]
connect_bd_net [get_bd_pins ddr_writer_0/debug_pacc_rst_busy] [get_bd_pins ddr_writer_ila_0/probe20]
# probe21-23（2026-07-16 新增，第四輪）：把 dispatcher 送進來的原始
# in_tdata 跟 pair-acc 組出來、要塞進 pacc_fifo 的 128-bit 合併值同時接
# 上 ILA，直接比對「進來的兩個 64-bit beat」有沒有正確組成該有的
# 128-bit 值——不再只看下游輸出（probe14/debug_axi_wdata），把輸入端
# 也攤在同一個 capture window 裡對照。probe4(debug_in_tvalid)/
# probe3(debug_pacc_wr_en) 分別 qualify 這兩個訊號哪一拍才是有效值。
# probe21: debug_in_tdata（in_tdata，64-bit）
# probe22: debug_in_tready
# probe23: debug_pacc_din（pacc_fifo_din，128-bit，= {in_tdata, half_buf}）
connect_bd_net [get_bd_pins ddr_writer_0/debug_in_tdata]      [get_bd_pins ddr_writer_ila_0/probe21]
connect_bd_net [get_bd_pins ddr_writer_0/debug_in_tready]     [get_bd_pins ddr_writer_ila_0/probe22]
connect_bd_net [get_bd_pins ddr_writer_0/debug_pacc_din]      [get_bd_pins ddr_writer_ila_0/probe23]
}

# ==============================================================================
#  System ILA 2: DAC/consumer-side observation (dac_clk domain)
# ==============================================================================
# 2026-07-05: kept as its own ILA (not folded into system_ila_0) per user
# request -- dac_clk may later be switched to an external clock via
# dac_clk_mux_0 (currently I0=I1=clk_wiz_0/clk_100, S=const_zero, so dac_clk
# happens to equal sys_clk today, but that's a deviation, not a guarantee).
# Clocking this ILA from dac_clk_mux_0/O (the mux output all wctrl_$ch/clk
# actually use) instead of clk_wiz_0/clk_100 directly means no rewiring is
# needed once ext clock support is added. 4 channels x (dac_data[32b] +
# dac_valid[1b] + mux_sel[1b]) = 12 probes. Confirms the waveform actually
# feeding each ZmodAWG channel plays completely with no gaps/dropouts.
if {$ENABLE_ILA} {
create_bd_cell -type ip -vlnv xilinx.com:ip:system_ila system_ila_2
set_property -dict [list \
    CONFIG.C_MON_TYPE      {NATIVE} \
    CONFIG.C_NUM_OF_PROBES {12}     \
    CONFIG.C_DATA_DEPTH    {1024}   \
] [get_bd_cells system_ila_2]
set_property -dict [list \
    CONFIG.C_PROBE0_WIDTH  {32} CONFIG.C_PROBE1_WIDTH  {1}  CONFIG.C_PROBE2_WIDTH  {1} \
    CONFIG.C_PROBE3_WIDTH  {32} CONFIG.C_PROBE4_WIDTH  {1}  CONFIG.C_PROBE5_WIDTH  {1} \
    CONFIG.C_PROBE6_WIDTH  {32} CONFIG.C_PROBE7_WIDTH  {1}  CONFIG.C_PROBE8_WIDTH  {1} \
    CONFIG.C_PROBE9_WIDTH  {32} CONFIG.C_PROBE10_WIDTH {1}  CONFIG.C_PROBE11_WIDTH {1} \
] [get_bd_cells system_ila_2]

connect_bd_net [get_bd_pins dac_clk_mux_0/O] [get_bd_pins system_ila_2/clk]

foreach ch {0 1 2 3} {
    set base [expr {$ch * 3}]
    connect_bd_net [get_bd_pins wctrl_$ch/dac_data]  [get_bd_pins system_ila_2/probe$base]
    connect_bd_net [get_bd_pins wctrl_$ch/dac_valid] [get_bd_pins system_ila_2/probe[expr {$base+1}]]
    connect_bd_net [get_bd_pins wctrl_$ch/mux_sel]   [get_bd_pins system_ila_2/probe[expr {$base+2}]]
}
}

# ==============================================================================
#  System ILA 1: FIFO write-side observation (ddr4_ui_clk domain)
# ==============================================================================
# 2026-07-05: second, independent ILA per user request, to see actual FIFO
# fill/write behavior on the ddr4_ui_clk domain (separate clock from
# system_ila_0's clk_100) -- checks for FIFO underflow ("FIFO 用光") and lets
# the two ILAs together show the full data-flow picture: writer side (here)
# vs. DAC/consumer side (system_ila_2). 8 FIFOs (4ch x A/B) x
# (wr_en + din[31:0] + full + prog_full) = 32 probes. Resources ample on
# XEM8320 (xcau25p) per user confirmation -- not economizing on probe count.
if {$ENABLE_ILA} {
create_bd_cell -type ip -vlnv xilinx.com:ip:system_ila system_ila_1
# 2026-07-10: C_DATA_DEPTH 16384->4096 -- 這個 ILA 單獨用掉 250/600 顆
# RAMB18（41.7%），是 place_design DRC UTLZ-1（BRAM over-utilized，634
# 需要 vs 600 可用）的最大宗來源，跟目前查 SPI 4-byte address 問題無關，
# 縮小騰出空間（見 PROJECT.md 第 31 節）
set_property -dict [list \
    CONFIG.C_MON_TYPE      {NATIVE} \
    CONFIG.C_NUM_OF_PROBES {32}     \
    CONFIG.C_DATA_DEPTH    {1024}   \
] [get_bd_cells system_ila_1]
set_property -dict [list \
    CONFIG.C_PROBE0_WIDTH  {1}  CONFIG.C_PROBE1_WIDTH  {32} \
    CONFIG.C_PROBE2_WIDTH  {1}  CONFIG.C_PROBE3_WIDTH  {1}  \
    CONFIG.C_PROBE4_WIDTH  {1}  CONFIG.C_PROBE5_WIDTH  {32} \
    CONFIG.C_PROBE6_WIDTH  {1}  CONFIG.C_PROBE7_WIDTH  {1}  \
    CONFIG.C_PROBE8_WIDTH  {1}  CONFIG.C_PROBE9_WIDTH  {32} \
    CONFIG.C_PROBE10_WIDTH {1}  CONFIG.C_PROBE11_WIDTH {1}  \
    CONFIG.C_PROBE12_WIDTH {1}  CONFIG.C_PROBE13_WIDTH {32} \
    CONFIG.C_PROBE14_WIDTH {1}  CONFIG.C_PROBE15_WIDTH {1}  \
    CONFIG.C_PROBE16_WIDTH {1}  CONFIG.C_PROBE17_WIDTH {32} \
    CONFIG.C_PROBE18_WIDTH {1}  CONFIG.C_PROBE19_WIDTH {1}  \
    CONFIG.C_PROBE20_WIDTH {1}  CONFIG.C_PROBE21_WIDTH {32} \
    CONFIG.C_PROBE22_WIDTH {1}  CONFIG.C_PROBE23_WIDTH {1}  \
    CONFIG.C_PROBE24_WIDTH {1}  CONFIG.C_PROBE25_WIDTH {32} \
    CONFIG.C_PROBE26_WIDTH {1}  CONFIG.C_PROBE27_WIDTH {1}  \
    CONFIG.C_PROBE28_WIDTH {1}  CONFIG.C_PROBE29_WIDTH {32} \
    CONFIG.C_PROBE30_WIDTH {1}  CONFIG.C_PROBE31_WIDTH {1}  \
] [get_bd_cells system_ila_1]

connect_bd_net [get_bd_pins ddr4_0/c0_ddr4_ui_clk] [get_bd_pins system_ila_1/clk]

# probe layout per FIFO (4 bits/probes each): wr_en, din[31:0], full, prog_full
# order: fifo_a_0, fifo_b_0, fifo_a_1, fifo_b_1, fifo_a_2, fifo_b_2, fifo_a_3, fifo_b_3
set fifo_ila1_list {fifo_a_0 fifo_b_0 fifo_a_1 fifo_b_1 fifo_a_2 fifo_b_2 fifo_a_3 fifo_b_3}
set p 0
foreach fifo $fifo_ila1_list {
    connect_bd_net [get_bd_pins $fifo/wr_en]     [get_bd_pins system_ila_1/probe$p]
    incr p
    connect_bd_net [get_bd_pins $fifo/din]       [get_bd_pins system_ila_1/probe$p]
    incr p
    connect_bd_net [get_bd_pins $fifo/full]      [get_bd_pins system_ila_1/probe$p]
    incr p
    connect_bd_net [get_bd_pins $fifo/prog_full] [get_bd_pins system_ila_1/probe$p]
    incr p
}
}

# ==============================================================================
#  Step 14.3b playback path: connections (ported from awg-test-step-14)
# ==============================================================================

# -- DAC clock muxes --------------------------------------------------------------
# 2026-07-13（ext clock 階段 B）：I1 從原本 I0 的重複接線改成 clk_wiz_ext_0
# 的輸出。2026-07-24（USB-only 功能盤點第 2 項）：S 改接
# board_cfg_reg_0/ext_clk_sel（TriggerIn 觸發式暫存器，host WI 0x18 +
# 新 TI bit8 確認寫入，或 Aurora T_EXT_CLK_SEL=0x28 廣播覆寫，見該檔案
# port 註解），不再直接接 wi_ext_clk_sel/Dout（那是還沒確認寫入的
# 暫存區）。
connect_bd_net [get_bd_pins clk_wiz_0/clk_100]            [get_bd_pins dac_clk_mux_0/I0]
connect_bd_net [get_bd_pins clk_wiz_ext_0/clk_ext_100]    [get_bd_pins dac_clk_mux_0/I1]
connect_bd_net [get_bd_pins board_cfg_reg_0/ext_clk_sel]  [get_bd_pins dac_clk_mux_0/S]
connect_bd_net [get_bd_pins clk_wiz_0/clk_100_90]         [get_bd_pins dac_90_clk_mux_0/I0]
connect_bd_net [get_bd_pins clk_wiz_ext_0/clk_ext_100_90] [get_bd_pins dac_90_clk_mux_0/I1]
connect_bd_net [get_bd_pins board_cfg_reg_0/ext_clk_sel]  [get_bd_pins dac_90_clk_mux_0/S]

# dac_clk (mux output) fan-out: FIFO rd side, wctrl, sync_start, ZmodAWG DAC_InIO_Clk,
# and every trigger_cdc dst_clk
connect_bd_net [get_bd_pins dac_clk_mux_0/O] \
    [get_bd_pins sync_start_0/clk] \
    [get_bd_pins wctrl_0/clk] [get_bd_pins wctrl_1/clk] \
    [get_bd_pins wctrl_2/clk] [get_bd_pins wctrl_3/clk] \
    [get_bd_pins fifo_a_0/rd_clk] [get_bd_pins fifo_a_1/rd_clk] \
    [get_bd_pins fifo_a_2/rd_clk] [get_bd_pins fifo_a_3/rd_clk] \
    [get_bd_pins fifo_b_0/rd_clk] [get_bd_pins fifo_b_1/rd_clk] \
    [get_bd_pins fifo_b_2/rd_clk] [get_bd_pins fifo_b_3/rd_clk] \
    [get_bd_pins zmod_awg_0/DAC_InIO_Clk] [get_bd_pins zmod_awg_1/DAC_InIO_Clk] \
    [get_bd_pins zmod_awg_2/DAC_InIO_Clk] [get_bd_pins zmod_awg_3/DAC_InIO_Clk] \
    [get_bd_pins reinit_pulse_cdc_0/dst_clk] \
    [get_bd_pins native_trig_delay_cdc_0/dst_clk] \
    [get_bd_pins trig_fire_fifo_0/rd_clk]         \
    [get_bd_pins dac_trig_queue_0/dac_clk]        \
    [get_bd_pins trig_timer_0/clk] [get_bd_pins trig_timer_1/clk] \
    [get_bd_pins trig_timer_2/clk] [get_bd_pins trig_timer_3/clk] \
    [get_bd_pins group_trig_scheduler_0/clk] \
    [get_bd_pins gsc_slot_cdc_0/dst_clk]  [get_bd_pins gsc_intv_cdc_0/dst_clk]  \
    [get_bd_pins gsc_group_cdc_0/dst_clk] [get_bd_pins gsc_depth_cdc_0/dst_clk] \
    [get_bd_pins gsc_run_cdc_0/dst_clk]   [get_bd_pins gsc_loop_cdc_0/dst_clk]  \
    [get_bd_pins gsc_wr_cdc_0/dst_clk]    [get_bd_pins gsc_arm_mode_cdc_0/dst_clk] \
    [get_bd_pins gsc_fire_fifo_0/wr_clk]  \
    [get_bd_pins timer_slot_cdc_0/dst_clk]   [get_bd_pins timer_intv_cdc_0/dst_clk]  \
    [get_bd_pins timer_depth_cdc_0/dst_clk]  [get_bd_pins timer_run_cdc_0/dst_clk]   \
    [get_bd_pins timer_loop_cdc_0/dst_clk]   [get_bd_pins timer_wr_cdc_0/dst_clk]    \
    [get_bd_pins timer_slot_cdc_1/dst_clk]   [get_bd_pins timer_intv_cdc_1/dst_clk]  \
    [get_bd_pins timer_depth_cdc_1/dst_clk]  [get_bd_pins timer_run_cdc_1/dst_clk]   \
    [get_bd_pins timer_loop_cdc_1/dst_clk]   [get_bd_pins timer_wr_cdc_1/dst_clk]    \
    [get_bd_pins timer_slot_cdc_2/dst_clk]   [get_bd_pins timer_intv_cdc_2/dst_clk]  \
    [get_bd_pins timer_depth_cdc_2/dst_clk]  [get_bd_pins timer_run_cdc_2/dst_clk]   \
    [get_bd_pins timer_loop_cdc_2/dst_clk]   [get_bd_pins timer_wr_cdc_2/dst_clk]    \
    [get_bd_pins timer_slot_cdc_3/dst_clk]   [get_bd_pins timer_intv_cdc_3/dst_clk]  \
    [get_bd_pins timer_depth_cdc_3/dst_clk]  [get_bd_pins timer_run_cdc_3/dst_clk]   \
    [get_bd_pins timer_loop_cdc_3/dst_clk]   [get_bd_pins timer_wr_cdc_3/dst_clk]    \
    [get_bd_pins sine_sel_cdc_0/dst_clk]      [get_bd_pins sine_data_cdc_0/dst_clk]      \
    [get_bd_pins sine_wr_cdc_0/dst_clk]       \
    [get_bd_pins sine_list_sel_cdc_0/dst_clk] [get_bd_pins sine_list_data_cdc_0/dst_clk] \
    [get_bd_pins sine_list_wr_cdc_0/dst_clk]  \
    [get_bd_pins list_sel_cdc_0/dst_clk]  [get_bd_pins list_addr_cdc_0/dst_clk] \
    [get_bd_pins list_len_cdc_0/dst_clk]  [get_bd_pins list_wr_cdc_0/dst_clk]   \
    [get_bd_pins sine_ctrl_regs_0/dac_clk] \
    [get_bd_pins aurora_reply_tx_0/dac_clk] \
    [get_bd_pins sine_gen_0_1_a/dac_clk] [get_bd_pins sine_gen_0_1_b/dac_clk] \
    [get_bd_pins sine_gen_0_2_a/dac_clk] [get_bd_pins sine_gen_0_2_b/dac_clk] \
    [get_bd_pins sine_gen_1_1_a/dac_clk] [get_bd_pins sine_gen_1_1_b/dac_clk] \
    [get_bd_pins sine_gen_1_2_a/dac_clk] [get_bd_pins sine_gen_1_2_b/dac_clk] \
    [get_bd_pins sine_gen_2_1_a/dac_clk] [get_bd_pins sine_gen_2_1_b/dac_clk] \
    [get_bd_pins sine_gen_2_2_a/dac_clk] [get_bd_pins sine_gen_2_2_b/dac_clk] \
    [get_bd_pins sine_gen_3_1_a/dac_clk] [get_bd_pins sine_gen_3_1_b/dac_clk] \
    [get_bd_pins sine_gen_3_2_a/dac_clk] [get_bd_pins sine_gen_3_2_b/dac_clk] \
    [get_bd_pins amp_ramp_gen_0_1_a/dac_clk] [get_bd_pins amp_ramp_gen_0_1_b/dac_clk] \
    [get_bd_pins amp_ramp_gen_0_2_a/dac_clk] [get_bd_pins amp_ramp_gen_0_2_b/dac_clk] \
    [get_bd_pins amp_ramp_gen_1_1_a/dac_clk] [get_bd_pins amp_ramp_gen_1_1_b/dac_clk] \
    [get_bd_pins amp_ramp_gen_1_2_a/dac_clk] [get_bd_pins amp_ramp_gen_1_2_b/dac_clk] \
    [get_bd_pins amp_ramp_gen_2_1_a/dac_clk] [get_bd_pins amp_ramp_gen_2_1_b/dac_clk] \
    [get_bd_pins amp_ramp_gen_2_2_a/dac_clk] [get_bd_pins amp_ramp_gen_2_2_b/dac_clk] \
    [get_bd_pins amp_ramp_gen_3_1_a/dac_clk] [get_bd_pins amp_ramp_gen_3_1_b/dac_clk] \
    [get_bd_pins amp_ramp_gen_3_2_a/dac_clk] [get_bd_pins amp_ramp_gen_3_2_b/dac_clk] \
    [get_bd_pins ddr_status_cdc_0/src_clk] [get_bd_pins ddr_status_cdc_1/src_clk] \
    [get_bd_pins ddr_status_cdc_2/src_clk] [get_bd_pins ddr_status_cdc_3/src_clk]
connect_bd_net [get_bd_pins dac_90_clk_mux_0/O] \
    [get_bd_pins zmod_awg_0/DAC_Clk] [get_bd_pins zmod_awg_1/DAC_Clk] \
    [get_bd_pins zmod_awg_2/DAC_Clk] [get_bd_pins zmod_awg_3/DAC_Clk]

# -- syzygy_ready ------------------------------------------------------------------
# 2026-07-08 (step 15b): fpga_flash_ctrl_0 接回來了，eos 改接真正的
# flash-load-complete 訊號（取代原本 const_one 的 deviation，DELAY_CYCLES
# power-up wait 現在會等 flash 讀取流程真正跑完才開始）
connect_bd_net [get_bd_pins clk_wiz_0/locked]        [get_bd_pins syzygy_ready_0/locked]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/fpga_eos] [get_bd_pins syzygy_ready_0/eos]
connect_bd_net [get_bd_pins syzygy_ready_0/rst_n] \
    [get_bd_pins zmod_awg_0/aRst_n] [get_bd_pins zmod_awg_1/aRst_n] \
    [get_bd_pins zmod_awg_2/aRst_n] [get_bd_pins zmod_awg_3/aRst_n]

# -- aurora_ctrl_mux_0 : FP inputs ------------------------------------------------
connect_bd_net [get_bd_pins fp0/ti40_ep_trigger] [get_bd_pins aurora_ctrl_mux_0/fp_ti_cmd]
connect_bd_net [get_bd_pins fp0/wi00_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_ddr4_addr]
connect_bd_net [get_bd_pins fp0/wi01_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_ddr4_w0]
connect_bd_net [get_bd_pins fp0/wi02_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_ddr4_w1]
connect_bd_net [get_bd_pins fp0/wi03_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_ddr4_w2]
connect_bd_net [get_bd_pins fp0/wi04_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_ddr4_w3]
connect_bd_net [get_bd_pins fp0/wi05_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_list_sel]
connect_bd_net [get_bd_pins fp0/wi06_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_list_addr]
connect_bd_net [get_bd_pins fp0/wi07_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_list_len]
connect_bd_net [get_bd_pins fp0/wi08_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_depth]
connect_bd_net [get_bd_pins fp0/wi09_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_play_en]
connect_bd_net [get_bd_pins fp0/wi0a_ep_dataout] [get_bd_pins aurora_ctrl_mux_0/fp_scale_cfg]
# 2026-07-27：WI 0x0a → board_cfg_reg_0/fp_scale_cfg 這條已移除
# （board_cfg_reg_0 的 fp_scale_wr 本機直寫路徑拔除，統一讀取/寫入
# 架構收斂，只剩 au_scale_wr）。上面那條 WI 0x0a → aurora_ctrl_mux_0/
# fp_scale_cfg 是 2026-07-15 就已經是死路的既有遺留（沒有下游消費
# 者），跟今天的改動無關，不在這次範圍內，不動。
# 2026-08-04：WI 0x0e/0x0f → aurora_ctrl_mux_0/fp_trig_slot/fp_trig_intv
# 這兩條已移除——T_TRIG_SLOT 整條路徑退役（z0 合併進 T_TIMER_CTRL），
# `fp_trig_slot`/`fp_trig_intv` input port 本身也已從 aurora_ctrl_mux.v
# 拔除，WI 0x0e/0x0f 兩個 WireIn 變成沒有下游消費者（比照既有 WI
# 0x0b/0x0c 退役慣例）。
# 2026-07-27：fp_amp_ctrl_data（WI 0x13）/fp_amp_ctrl_wr（ti_amp_ctrl_
# wr）/fp_amp_ctrl_sel（WI 0x19 寫入端）已移除——aurora_ctrl_mux.v 本地
# fork 拔除這 3 個 input port，amp_ctrl 只剩 Aurora T_AMP_CTRL(0x09)
# 一條寫入路徑。amp_ctrl_read_sel_slice/amp_ctrl_read_mux_0 這組讀回
# 路徑是獨立的組合邏輯（讀 aurora_ctrl_mux_0/out_amp_ctrl_0..7 的即時
# 值，不經過寫入位址），完全不受影響，維持不動。
create_bd_cell -type ip -vlnv xilinx.com:ip:xlslice amp_ctrl_read_sel_slice
set_property -dict [list CONFIG.DIN_WIDTH {32} CONFIG.DIN_FROM {2} CONFIG.DIN_TO {0}] [get_bd_cells amp_ctrl_read_sel_slice]
connect_bd_net [get_bd_pins fp0/wi19_ep_dataout] [get_bd_pins amp_ctrl_read_sel_slice/Din]
connect_bd_net [get_bd_pins amp_ctrl_read_sel_slice/Dout] [get_bd_pins amp_ctrl_read_mux_0/sel]

# -- sine_ctrl_regs_0 selector/data/mode (added 2026-07-17, widened 2026-07-23
# for 8-channel independent output) --------------------------------------------
# WI 0x1B[5:0] = {param_sel[2:0], ch_sel[2:0]} (widened from [4:0] to [5:0]:
# ch_sel is now 3-bit, 0-7, encoding {inst[1:0],is_ch2} -- see
# sine_ctrl_regs.v header). WI 0x1C = 32-bit data.
# WI 0x1D[3:0] = per-MODULE DDR(0)/sine(1) mode (unchanged, 4 bits -- DDR/sine
# switching stays per-module, confirmed with user 2026-07-23, see
# dac_output_mux.v). WI 0x1D[11:4] = per-PHYSICAL-CHANNEL amp ramp enable
# (0=static amp_ctrl, 1=amp_ramp_gen), widened from 4 to 8 bits to match
# amp_ctrl_mux_0..7 (one per physical channel, matching the pre-existing
# amp_ctrl_0..7 numbering in aurora_ctrl_mux.v) -- WI 0x1D has plenty of free
# bits above [7:4], no conflict.
# 2026-07-27：sine_wi_sel_slice（WI 0x1B）跟 WI 0x1C → sine_ctrl_regs_0/
# wi_sel/wi_data 這兩條本機直寫路徑已移除——sine_ctrl_regs.v 本地拔除
# wi_sel/wi_data/wr_strobe，只剩 au_sine_* 這條 Aurora 路徑。
# 2026-08 改版（Sine mode N-slot 機制，比照 DDR waveform_controller.v 的
# 方式）：sine_ctrl_regs.v 整條寫入路徑（含既有 T_SINE_CTRL 單值寫入）從
# sys_clk 搬到 dac_clk domain（見該檔案檔頭說明），au_sine_sel/data/wr
# 不再直接接線，改成 sine_sel_cdc_0/sine_data_cdc_0（level_cdc）+
# sine_wr_cdc_0（trigger_cdc）正規 CDC，跟 T_SINE_LIST_CTRL 新封包用的
# sine_list_*_cdc_0 一起接線，比照 timer_wr_cdc_$port/timer_slot_cdc_$port
# 已驗證過的慣例（level_cdc 給準穩態 sel/data、trigger_cdc 給 wr pulse，
# host 端固定「先送 sel/data、才送 wr pulse」的封包序列保證 settling
# time，安全性論證跟 trig_timer 完全相同，見 rtl/sine_ctrl_regs.v 檔頭
# 「CDC 安全性論證」段落）。這 6 個 cell 本身在上方跟其他 CDC 一起建立
# （早於 src_clk/dst_clk fan-out），這裡只做實際接線。
connect_bd_net [get_bd_pins local_reg_handler_0/au_sine_sel]       [get_bd_pins sine_sel_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_sine_data]      [get_bd_pins sine_data_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_sine_wr]        [get_bd_pins sine_wr_cdc_0/src_pulse]
connect_bd_net [get_bd_pins local_reg_handler_0/au_sine_list_sel]  [get_bd_pins sine_list_sel_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_sine_list_data] [get_bd_pins sine_list_data_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_sine_list_wr]   [get_bd_pins sine_list_wr_cdc_0/src_pulse]

connect_bd_net [get_bd_pins sine_sel_cdc_0/dst_out]        [get_bd_pins sine_ctrl_regs_0/dac_sine_sel]
connect_bd_net [get_bd_pins sine_data_cdc_0/dst_out]       [get_bd_pins sine_ctrl_regs_0/dac_sine_data]
connect_bd_net [get_bd_pins sine_wr_cdc_0/dst_pulse]       [get_bd_pins sine_ctrl_regs_0/dac_sine_wr]
connect_bd_net [get_bd_pins sine_list_sel_cdc_0/dst_out]   [get_bd_pins sine_ctrl_regs_0/dac_sine_list_sel]
connect_bd_net [get_bd_pins sine_list_data_cdc_0/dst_out]  [get_bd_pins sine_ctrl_regs_0/dac_sine_list_data]
connect_bd_net [get_bd_pins sine_list_wr_cdc_0/dst_pulse]  [get_bd_pins sine_ctrl_regs_0/dac_sine_list_wr]
# 2026-08-05：dac_mode_ramp 併入 sine_ctrl_regs_0（idle/active 雙緩衝，
# 取代這裡原本 board_cfg_reg_0 的整包覆寫、無緩衝設計），au_dac_mode_ramp/
# au_dac_mode_ramp_wr 這組 Aurora 直連（連同 T_DAC_MODE_RAMP 封包本身）
# 已退役，不再接線。dac_output_mux_$ch/mode、amp_ctrl_mux_$i/ramp_en
# 改接 sine_ctrl_regs_0 的輸出，見下方 foreach 迴圈 + rtl/sine_ctrl_regs.v
# 檔頭說明。

# 2026-07-27 新增（Group-based Trigger 架構）：T_TRIG_MASK(0x0D) 新語意
# ——「這個 group 涵蓋哪些模組」，local_reg_handler_0 -> board_cfg_reg_0
# （都在 sys_clk domain，不需要 CDC）。沒有 fp_* 本機路徑（見「開工前
# 對齊」第2點：WI 位址已滿，且封包 dest_id 機制本來就已經支援本機）。
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_cfg_wr]       [get_bd_pins board_cfg_reg_0/au_group_cfg_wr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_cfg_group_id] [get_bd_pins board_cfg_reg_0/au_group_cfg_group_id]
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_cfg_mask]     [get_bd_pins board_cfg_reg_0/au_group_cfg_mask]

# 2026-07-24 再追加：per_hop_value 手動覆寫（WI 0x1F，32-bit 直接
# passthrough，不需要 slice，跟 fp_ddr4_addr(WI 0x00) 同一種全寬直連
# 模式）+ TI bit10（見上方 ti_per_hop_value_wr 註解）。這是第一個真正
# 用掉 0x1F 的 WI，`fp0` 的 `CONFIG.WI.COUNT` 需要從 31 擴充到 32（見
# 下方 fp0 CONFIG 區塊）。
connect_bd_net [get_bd_pins fp0/wi1f_ep_dataout]      [get_bd_pins board_cfg_reg_0/fp_per_hop_value]
connect_bd_net [get_bd_pins ti_per_hop_value_wr/Dout] [get_bd_pins board_cfg_reg_0/fp_per_hop_value_wr]
# 2026-08-05：sine_mode_slice_$ch/sine_ramp_en_slice_$i（xlslice）已移除
# ——sine_ctrl_regs_0 現在直接輸出個別 1-bit port（mode_active_$ch/
# ramp_en_active_$i，比照本模組既有 mux_sel_0..7 的慣例），不需要額外
# slice 才能接 1-bit 目的地。
foreach ch {0 1 2 3} {
    connect_bd_net [get_bd_pins sine_ctrl_regs_0/mode_active_$ch] [get_bd_pins dac_output_mux_$ch/mode]
}
foreach i {0 1 2 3 4 5 6 7} {
    connect_bd_net [get_bd_pins sine_ctrl_regs_0/ramp_en_active_$i] [get_bd_pins amp_ctrl_mux_$i/ramp_en]
}
# dac_mode_ramp_concat_0：重組回原本 12-bit 排列（bit[3:0]=mode,
# bit[11:4]=ramp_en），見該 cell 建立處註解。
foreach ch {0 1 2 3} {
    connect_bd_net [get_bd_pins sine_ctrl_regs_0/mode_active_$ch] [get_bd_pins dac_mode_ramp_concat_0/In$ch]
}
foreach i {0 1 2 3 4 5 6 7} {
    set concat_in [expr {4 + $i}]
    connect_bd_net [get_bd_pins sine_ctrl_regs_0/ramp_en_active_$i] [get_bd_pins dac_mode_ramp_concat_0/In$concat_in]
}
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_amp_ctrl_0] [get_bd_pins amp_ctrl_read_mux_0/amp_ctrl_0]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_amp_ctrl_1] [get_bd_pins amp_ctrl_read_mux_0/amp_ctrl_1]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_amp_ctrl_2] [get_bd_pins amp_ctrl_read_mux_0/amp_ctrl_2]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_amp_ctrl_3] [get_bd_pins amp_ctrl_read_mux_0/amp_ctrl_3]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_amp_ctrl_4] [get_bd_pins amp_ctrl_read_mux_0/amp_ctrl_4]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_amp_ctrl_5] [get_bd_pins amp_ctrl_read_mux_0/amp_ctrl_5]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_amp_ctrl_6] [get_bd_pins amp_ctrl_read_mux_0/amp_ctrl_6]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_amp_ctrl_7] [get_bd_pins amp_ctrl_read_mux_0/amp_ctrl_7]
# TODO(port): fp_timer_p1/p2/p3_intv had no WI in 14.3b -- step-14 fed them from
# WI 0x15/0x16/0x17, but those are used by fp_ddr4_rw_1 write-data here. Tied to
# const_32b0 (Aurora path still drives out_timer_pN_intv). Assign spare WIs if FP-direct
# per-port timer intervals are needed.
connect_bd_net [get_bd_pins const_32b0/dout] [get_bd_pins aurora_ctrl_mux_0/fp_timer_p1_intv]
connect_bd_net [get_bd_pins const_32b0/dout] [get_bd_pins aurora_ctrl_mux_0/fp_timer_p2_intv]
connect_bd_net [get_bd_pins const_32b0/dout] [get_bd_pins aurora_ctrl_mux_0/fp_timer_p3_intv]
# flash controller 接回來（2026-07-08 step 15b），取代原本 const tie-off
# 2026-07-15：amp_ctrl 現在存在獨立 sector（跟身份 sector 不同時間讀完），
# 改接專屬的 flash_amp_load_valid_cdc_0，不能再用共用的 flash_load_valid_cdc_0
# （那個現在只代表身份 sector 讀完，時機對 amp_ctrl 來說是錯的）
connect_bd_net [get_bd_pins flash_amp_load_valid_cdc_0/dst_pulse] [get_bd_pins aurora_ctrl_mux_0/flash_load_valid]
foreach i {0 1 2 3 4 5 6 7} {
    connect_bd_net [get_bd_pins fpga_flash_ctrl_0/init_amp_ctrl_$i] [get_bd_pins aurora_ctrl_mux_0/flash_amp_ctrl_$i]
}

# -- aurora_ctrl_mux_0 : Aurora (local_reg_handler decoded) inputs ----------------
connect_bd_net [get_bd_pins local_reg_handler_0/au_ddr4_wr]      [get_bd_pins aurora_ctrl_mux_0/au_ddr4_wr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_ddr4_addr]    [get_bd_pins aurora_ctrl_mux_0/au_ddr4_addr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_ddr4_w0]      [get_bd_pins aurora_ctrl_mux_0/au_ddr4_w0]
connect_bd_net [get_bd_pins local_reg_handler_0/au_ddr4_w1]      [get_bd_pins aurora_ctrl_mux_0/au_ddr4_w1]
connect_bd_net [get_bd_pins local_reg_handler_0/au_ddr4_w2]      [get_bd_pins aurora_ctrl_mux_0/au_ddr4_w2]
connect_bd_net [get_bd_pins local_reg_handler_0/au_ddr4_w3]      [get_bd_pins aurora_ctrl_mux_0/au_ddr4_w3]
connect_bd_net [get_bd_pins local_reg_handler_0/au_list_wr]      [get_bd_pins aurora_ctrl_mux_0/au_list_wr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_list_sel]     [get_bd_pins aurora_ctrl_mux_0/au_list_sel]
connect_bd_net [get_bd_pins local_reg_handler_0/au_list_addr]    [get_bd_pins aurora_ctrl_mux_0/au_list_addr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_list_len]     [get_bd_pins aurora_ctrl_mux_0/au_list_len]
connect_bd_net [get_bd_pins local_reg_handler_0/au_play_ctrl_wr] [get_bd_pins aurora_ctrl_mux_0/au_play_ctrl_wr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_play_ctrl]    [get_bd_pins aurora_ctrl_mux_0/au_play_ctrl]
connect_bd_net [get_bd_pins local_reg_handler_0/au_scale_cfg_wr] [get_bd_pins aurora_ctrl_mux_0/au_scale_cfg_wr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_scale_cfg]    [get_bd_pins aurora_ctrl_mux_0/au_scale_cfg]
connect_bd_net [get_bd_pins local_reg_handler_0/au_amp_ctrl_wr]  [get_bd_pins aurora_ctrl_mux_0/au_amp_ctrl_wr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_amp_ch_sel]   [get_bd_pins aurora_ctrl_mux_0/au_amp_ch_sel]
connect_bd_net [get_bd_pins local_reg_handler_0/au_amp_val]      [get_bd_pins aurora_ctrl_mux_0/au_amp_val]
connect_bd_net [get_bd_pins local_reg_handler_0/au_flush_standby]       [get_bd_pins aurora_ctrl_mux_0/au_flush_standby]
# 2026-07-24（trigger 統一化架構改版）：T_TRIG_PORT 機制已拿掉，
# local_reg_handler_0 不再有 au_trig_port_wr/au_trig_port_mask，改
# tie-off 常數（見上方 const_4b0 建立處註解）。
connect_bd_net [get_bd_pins const_zero/dout] [get_bd_pins aurora_ctrl_mux_0/au_trig_port_wr]
connect_bd_net [get_bd_pins const_4b0/dout]  [get_bd_pins aurora_ctrl_mux_0/au_trig_port_mask]
connect_bd_net [get_bd_pins local_reg_handler_0/au_timer_ctrl_wr]       [get_bd_pins aurora_ctrl_mux_0/au_timer_ctrl_wr]
connect_bd_net [get_bd_pins local_reg_handler_0/au_timer_ctrl_port]     [get_bd_pins aurora_ctrl_mux_0/au_timer_ctrl_port]
connect_bd_net [get_bd_pins local_reg_handler_0/au_timer_ctrl_run]      [get_bd_pins aurora_ctrl_mux_0/au_timer_ctrl_run]
connect_bd_net [get_bd_pins local_reg_handler_0/au_timer_ctrl_loop]     [get_bd_pins aurora_ctrl_mux_0/au_timer_ctrl_loop]
connect_bd_net [get_bd_pins local_reg_handler_0/au_timer_ctrl_depth]    [get_bd_pins aurora_ctrl_mux_0/au_timer_ctrl_depth]
connect_bd_net [get_bd_pins local_reg_handler_0/au_timer_ctrl_slot]     [get_bd_pins aurora_ctrl_mux_0/au_timer_ctrl_slot]
connect_bd_net [get_bd_pins local_reg_handler_0/au_timer_ctrl_intv]     [get_bd_pins aurora_ctrl_mux_0/au_timer_ctrl_intv]
# 2026-07-27（Group-based Trigger 架構）：local_reg_handler_0 不再有
# au_trig_mask_wr/au_trig_mask_val（T_TRIG_MASK 重新定義，見上方
# au_group_cfg_wr/group_id/mask 接線），改 tie-off 常數。aurora_ctrl_
# mux_0 是共用檔案（awg-test-step-14/rtl/，被多個 step 專案引用），這次
# 不需要新增邏輯，直接不用它的 trig_mask 功能即可，不改檔案本身（比照
# 「開工前對齊」第1點）。out_trig_mask_0..3 變成沒有消費者的孤兒，見
# 下方 wctrl_$ch/trigger_mask 接線移除處。
connect_bd_net [get_bd_pins const_zero/dout] [get_bd_pins aurora_ctrl_mux_0/au_trig_mask_wr]
connect_bd_net [get_bd_pins const_4b0/dout]  [get_bd_pins aurora_ctrl_mux_0/au_trig_mask_val]
# 2026-07-10：aurora_ctrl_mux_0/au_flash_save 是共用檔案（awg-test-step-14
# 還在用這個 port，見 PORTS.md），不能拿掉 port 本身，但 step-15b 這邊已經
# 移除 local_reg_handler_0 的 au_flash_save 輸出（見上方/rtl/
# local_reg_handler.v），改接常數 0，避免 unconnected pin 錯誤。
connect_bd_net [get_bd_pins const_zero/dout]                            [get_bd_pins aurora_ctrl_mux_0/au_flash_save]
connect_bd_net [get_bd_pins local_reg_handler_0/au_flash_erase]         [get_bd_pins aurora_ctrl_mux_0/au_flash_erase]
# 2026-07-27：au_board_cfg_wr/au_board_cfg_id/au_board_cfg_is_master 這
# 3 條已移除——local_reg_handler.v 的來源 port、aurora_ctrl_mux.v（本地
# fork）的目的 port 都已經拔掉了（這條 pass-through 本來就沒有下游
# 消費者，out_board_cfg_wr/id/is_master 從沒被接過，是純粹的死路，見
# PORTS.md「aurora_ctrl_mux」章節）。

# -- aurora_ctrl_mux_0 : outputs --------------------------------------------------
# out_ti_cmd -> ti_trig_list_wr (fp_ddr4_rw_1/ti_cmd kept on direct ti40)
# 2026-08-10：ti_list_wr/Din 這條已移除，見上方 ti_trig_list_wr 建立處說明。
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_ti_cmd] \
    [get_bd_pins ti_trig_list_wr/Din]
# 2026-08-10：out_list_sel/addr/len 不再直接接 wctrl_$ch（dac_clk
# domain）——這條 sys_clk->dac_clk 路徑原本完全沒有 CDC，改插入
# list_sel_cdc_0/list_addr_cdc_0/list_len_cdc_0（level_cdc），下游
# wctrl_$ch 改接這三個 CDC 的 dst_out（見下方「Waveform controllers」
# 段落），跟 out_ti_cmd 的 list_wr_en 一起改走 CDC，見 list_wr_cdc_0。
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_list_sel]  [get_bd_pins list_sel_slice/Din]
connect_bd_net [get_bd_pins list_sel_slice/Dout]             [get_bd_pins list_sel_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_list_addr] [get_bd_pins list_addr_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_list_len]  [get_bd_pins list_len_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_list_wr]   [get_bd_pins list_wr_cdc_0/src_pulse]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_depth] \
    [get_bd_pins depth_slice_0/Din] [get_bd_pins depth_slice_1/Din] \
    [get_bd_pins depth_slice_2/Din] [get_bd_pins depth_slice_3/Din]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_play_en] \
    [get_bd_pins play_en_slice_0/Din] [get_bd_pins play_en_slice_1/Din] \
    [get_bd_pins play_en_slice_2/Din] [get_bd_pins play_en_slice_3/Din]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_scale_cfg] [get_bd_pins scale_cfg_slice_calib/Din]
# out_trig_slot/out_trig_intv → trig_slot_slice/timer_intv_cdc_0 這兩條
# 2026-08-04 移除——z0 合併進 T_TIMER_CTRL，port 0 的 intv/slot 現在跟
# port 1-3 用同一套接線模式，見下方「Trigger timer plumbing」章節。
# 2026-07-17: amp_ctrl_$i for even $i (0,2,4,6) is the "active" Ch1 gain
# for channel $i/2 (amp_ctrl_0=z0_ch1, amp_ctrl_2=z1_ch1, ... see
# rtl/awg_calib_regs.v header comment), odd $i is Ch2 -- i = 2*inst +
# (sub==2 ? 1 : 0), exactly the same module-major encoding as sine_ctrl_
# regs_0's ch_sel. 2026-07-23: now ALL 8 (not just even/Ch1) get the ramp
# treatment via amp_ctrl_mux_$i, matching the 8-channel sine_gen expansion --
# each amp_ctrl_mux_$i selects between the static hold value and the
# CURRENTLY ACTIVE one of amp_ramp_gen_${inst}_${sub}_a/_b (mux_sel_$i,
# same signal sine_ctrl_regs_0 uses for the sine_gen active/idle selection
# on this physical channel).
foreach i {0 1 2 3 4 5 6 7} {
    set inst [expr {$i / 2}]
    set sub  [expr {($i % 2 == 0) ? 1 : 2}]
    connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_amp_ctrl_$i]        [get_bd_pins amp_ctrl_mux_$i/static_amp]
    connect_bd_net [get_bd_pins amp_ramp_gen_${inst}_${sub}_a/amp_out]    [get_bd_pins amp_ctrl_mux_$i/ramp_amp_a]
    connect_bd_net [get_bd_pins amp_ramp_gen_${inst}_${sub}_b/amp_out]    [get_bd_pins amp_ctrl_mux_$i/ramp_amp_b]
    connect_bd_net [get_bd_pins sine_ctrl_regs_0/mux_sel_$i]              [get_bd_pins amp_ctrl_mux_$i/mux_sel]
    connect_bd_net [get_bd_pins amp_ctrl_mux_$i/amp_ctrl_out]             [get_bd_pins awg_calib_regs_0/amp_ctrl_$i]
}
# 2026-07-24（trigger 統一化架構改版）：out_trig_port_$port 的下游
# （au_trig_port_cdc_$port）已移除，T_TRIG_PORT 機制拿掉，這 4 個
# aurora_ctrl_mux_0 output 變成沒有消費者的死路，比照既有 out_scale_cfg
# 的處理方式（見該處註解），留著不接即可，不需要另外 tie-off（這是
# module output，不像 input 那樣需要明確驅動源）。
# 2026-07-27（Group-based Trigger 架構）：out_trig_mask_0..3 -> wctrl_
# $ch/trigger_mask 這條既有接線整個移除——wctrl_$ch/trigger_mask port
# 本身也被移除（masking 邏輯上移到下方新增的 group_trig_select_$ch，見
# rtl/waveform_controller.v）。aurora_ctrl_mux_0/out_trig_mask_0..3 變成
# 沒有消費者的孤兒 output，比照上方 out_trig_port_$port 的處理方式，
# 留著不接即可。

# -- Trigger timer plumbing -------------------------------------------------------
# WI 0x10 packs per-port timer run/loop/depth -> the timer slices
foreach port {0 1 2 3} {
    connect_bd_net [get_bd_pins fp0/wi10_ep_dataout] [get_bd_pins timer_run_slice_$port/Din]
    connect_bd_net [get_bd_pins fp0/wi10_ep_dataout] [get_bd_pins timer_loop_slice_$port/Din]
    connect_bd_net [get_bd_pins fp0/wi10_ep_dataout] [get_bd_pins timer_depth_slice_$port/Din]
}
# 2026-07-24（trigger 統一化架構改版）：trig_timer_$port 搬到 dac_clk
# domain，list_wr_slot/list_wr_intv/list_depth/run/loop_en/list_wr_en
# 這 6 個訊號原本直接從 sys_clk 側接過去，現在全部改先進各自獨立的
# level_cdc/pulse CDC（timer_slot_cdc_$port 等，見上方 cell 建立處
# 註解），CDC 輸出才接 trig_timer_$port，最下面統一列出。
# port 0: 2026-08-04 起跟 port 1-3 用完全同一套模式（slot/intv/wr/run/
# loop/depth 全部先過 OR gate 再接 CDC）——z0 已合併進 T_TIMER_CTRL，
# 不再是獨立的 T_TRIG_SLOT/FP-direct-slot 路徑，見 aurora_ctrl_mux.v
# 同日修復記錄。跟 port 1-3 寫成獨立區塊（不是同一個 foreach）純粹是
# 因為 port 0 的 FP 直寫觸發位元沿用既有 `ti_trig_list_wr`（T_TRIG_SLOT
# 舊路徑留下的 TI bit 5 名稱），跟 `ti_timer_wr_$port` 命名不同，接線
# 順序/邏輯跟下面 port 1-3 完全一致。
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_p0_slot] [get_bd_pins timer_slot_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_p0_intv] [get_bd_pins timer_intv_cdc_0/src_in]
connect_bd_net [get_bd_pins ti_trig_list_wr/Dout]                [get_bd_pins or_timer_wr_0/Op1]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_p0_wr]   [get_bd_pins or_timer_wr_0/Op2]
connect_bd_net [get_bd_pins or_timer_wr_0/Res]                   [get_bd_pins timer_wr_cdc_0/src_pulse]
connect_bd_net [get_bd_pins timer_run_slice_0/Dout]              [get_bd_pins or_timer_run_0/Op1]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_run_0]   [get_bd_pins or_timer_run_0/Op2]
connect_bd_net [get_bd_pins or_timer_run_0/Res]                  [get_bd_pins timer_run_cdc_0/src_in]
connect_bd_net [get_bd_pins timer_loop_slice_0/Dout]             [get_bd_pins or_timer_loop_0/Op1]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_loop_0]  [get_bd_pins or_timer_loop_0/Op2]
connect_bd_net [get_bd_pins or_timer_loop_0/Res]                 [get_bd_pins timer_loop_cdc_0/src_in]
connect_bd_net [get_bd_pins timer_depth_slice_0/Dout]            [get_bd_pins or_timer_depth_0/Op1]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_depth_0] [get_bd_pins or_timer_depth_0/Op2]
connect_bd_net [get_bd_pins or_timer_depth_0/Res]                 [get_bd_pins timer_depth_cdc_0/src_in]
# ports 1-3: slot/intv from aurora_ctrl_mux per-port; wr/run/loop/depth via OR gates
foreach port {1 2 3} {
    connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_p${port}_slot] [get_bd_pins timer_slot_cdc_$port/src_in]
    connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_p${port}_intv] [get_bd_pins timer_intv_cdc_$port/src_in]
    connect_bd_net [get_bd_pins ti_timer_wr_$port/Dout]                    [get_bd_pins or_timer_wr_$port/Op1]
    connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_p${port}_wr]   [get_bd_pins or_timer_wr_$port/Op2]
    connect_bd_net [get_bd_pins or_timer_wr_$port/Res]                     [get_bd_pins timer_wr_cdc_$port/src_pulse]
    connect_bd_net [get_bd_pins timer_run_slice_$port/Dout]                [get_bd_pins or_timer_run_$port/Op1]
    connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_run_$port]     [get_bd_pins or_timer_run_$port/Op2]
    connect_bd_net [get_bd_pins or_timer_run_$port/Res]                    [get_bd_pins timer_run_cdc_$port/src_in]
    connect_bd_net [get_bd_pins timer_loop_slice_$port/Dout]               [get_bd_pins or_timer_loop_$port/Op1]
    connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_loop_$port]    [get_bd_pins or_timer_loop_$port/Op2]
    connect_bd_net [get_bd_pins or_timer_loop_$port/Res]                   [get_bd_pins timer_loop_cdc_$port/src_in]
    connect_bd_net [get_bd_pins timer_depth_slice_$port/Dout]              [get_bd_pins or_timer_depth_$port/Op1]
    connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_timer_depth_$port]   [get_bd_pins or_timer_depth_$port/Op2]
    connect_bd_net [get_bd_pins or_timer_depth_$port/Res]                  [get_bd_pins timer_depth_cdc_$port/src_in]
}
# CDC 輸出 -> trig_timer_$port（dac_clk domain 原生 port）
foreach port {0 1 2 3} {
    connect_bd_net [get_bd_pins timer_slot_cdc_$port/dst_out]  [get_bd_pins trig_timer_$port/list_wr_slot]
    connect_bd_net [get_bd_pins timer_intv_cdc_$port/dst_out]  [get_bd_pins trig_timer_$port/list_wr_intv]
    connect_bd_net [get_bd_pins timer_depth_cdc_$port/dst_out] [get_bd_pins trig_timer_$port/list_depth]
    connect_bd_net [get_bd_pins timer_run_cdc_$port/dst_out]   [get_bd_pins trig_timer_$port/run]
    connect_bd_net [get_bd_pins timer_loop_cdc_$port/dst_out]  [get_bd_pins trig_timer_$port/loop_en]
    connect_bd_net [get_bd_pins timer_wr_cdc_$port/dst_pulse]  [get_bd_pins trig_timer_$port/list_wr_en]
}

# -- group_trig_scheduler_0 plumbing（2026-08-04，trigger group 排程功能
# 三部曲「C」）------------------------------------------------------------
# 控制輸入：純 Aurora（local_reg_handler_0/au_group_sched_*），沒有 FP
# 直寫路徑，不需要 OR gate，直接進 CDC 再進模組
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_sched_slot]      [get_bd_pins gsc_slot_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_sched_intv]      [get_bd_pins gsc_intv_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_sched_group_sel] [get_bd_pins gsc_group_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_sched_depth]     [get_bd_pins gsc_depth_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_sched_run]       [get_bd_pins gsc_run_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_sched_loop]      [get_bd_pins gsc_loop_cdc_0/src_in]
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_sched_wr]        [get_bd_pins gsc_wr_cdc_0/src_pulse]
# 2026-08-20 新增（多板同步輪播 Architecture B）：arm_mode 同一套
# level_cdc 模式，跟 run/loop_en 並列。
connect_bd_net [get_bd_pins local_reg_handler_0/au_group_sched_arm_mode]  [get_bd_pins gsc_arm_mode_cdc_0/src_in]

connect_bd_net [get_bd_pins gsc_slot_cdc_0/dst_out]   [get_bd_pins group_trig_scheduler_0/list_wr_slot]
connect_bd_net [get_bd_pins gsc_intv_cdc_0/dst_out]   [get_bd_pins group_trig_scheduler_0/list_wr_intv]
connect_bd_net [get_bd_pins gsc_group_cdc_0/dst_out]  [get_bd_pins group_trig_scheduler_0/list_wr_group]
connect_bd_net [get_bd_pins gsc_depth_cdc_0/dst_out]  [get_bd_pins group_trig_scheduler_0/list_depth]
connect_bd_net [get_bd_pins gsc_run_cdc_0/dst_out]    [get_bd_pins group_trig_scheduler_0/run]
connect_bd_net [get_bd_pins gsc_loop_cdc_0/dst_out]   [get_bd_pins group_trig_scheduler_0/loop_en]
connect_bd_net [get_bd_pins gsc_wr_cdc_0/dst_pulse]   [get_bd_pins group_trig_scheduler_0/list_wr_en]
connect_bd_net [get_bd_pins gsc_arm_mode_cdc_0/dst_out] [get_bd_pins group_trig_scheduler_0/arm_mode]

# 2026-08-20 新增（多板同步輪播 Architecture B）：first_trigger 接既有
# dac_trig_queue_0/native_trig_out——這是這個專案唯一一套「先走一次完整
# 跨板 PAUSE/ACK/GO 廣播+補償延遲，同步起始點」的機制，本身已經是
# dac_clk domain（跟 group_trig_scheduler_0 同一個 clock），不需要新的
# CDC，純粹 fan-out 多接一條線。arm_mode=0（legacy）時這條線不會被用到
# （armed 狀態機只有 arm_mode=1 才會進入），接了也不影響現有行為。
connect_bd_net [get_bd_pins dac_trig_queue_0/native_trig_out] [get_bd_pins group_trig_scheduler_0/first_trigger]

# 輸出：fire（1-cycle pulse）+ group_select_out（同一拍有效）跨到
# sys_clk，OR 進既有 local_reg_handler_0/au_trig_start/au_trig_group_
# select（au_trig_pulse_cdc_0/au_trig_group_cdc_0 這兩顆既有、已驗證
# 的 Aurora 協定 CDC 完全不用動），重用整套已驗證的多板 PAUSE/ACK/GO
# 握手機制，不用碰 Aurora TX 傳送層。**這裡取代原本
# local_reg_handler_0/au_trig_start 直接接 au_trig_pulse_cdc_0/
# src_pulse、au_trig_group_select 直接接 au_trig_group_cdc_0/src_in
# 的兩條 connect_bd_net（下方兩行是新版本，舊的兩條直連已移除，不是
# 額外並聯——BD 一個 input pin 只能有一個驅動源）**。
#
# 2026-08-04 CDC 修法：group_select_out 走 gsc_fire_fifo_0（FIFO，取代
# 原本誤用的 level_cdc，見上方 cell 建立處完整說明）+
# fifo_event_reader_0（把 FWFT 讀出端轉成 pulse+data），不是單純的
# pulse_cdc/level_cdc 一對一跨域。
connect_bd_net [get_bd_pins group_trig_scheduler_0/fire]             [get_bd_pins gsc_fire_fifo_0/wr_en]
connect_bd_net [get_bd_pins group_trig_scheduler_0/group_select_out] [get_bd_pins gsc_fire_fifo_0/din]
connect_bd_net [get_bd_pins gsc_fire_fifo_0/dout]        [get_bd_pins fifo_event_reader_0/fifo_dout]
connect_bd_net [get_bd_pins gsc_fire_fifo_0/empty]       [get_bd_pins fifo_event_reader_0/fifo_empty]
connect_bd_net [get_bd_pins fifo_event_reader_0/fifo_rd_en] [get_bd_pins gsc_fire_fifo_0/rd_en]

# 2026-08-20 新增（多板同步輪播 Architecture B）：fire_local/group_
# select_out（arm_mode=1 時才會非 0，見 group_trig_scheduler.v 檔頭
# 說明）不走上面那條 legacy 路徑，改接 trig_merge_0——兩端都是 dac_clk，
# 不需要 CDC，見 rtl/trig_merge.v 檔頭說明。trig_merge_0 本身的另外兩個
# input（dac_trig_queue_0 的既有輸出）、輸出接 group_trig_select_$ch，
# 在下方 group_trig_select_$ch 建立處一起接（取代原本直接接
# dac_trig_queue_0 的兩條線）。
create_bd_cell -type module -reference trig_merge trig_merge_0
connect_bd_net [get_bd_pins group_trig_scheduler_0/fire_local]       [get_bd_pins trig_merge_0/sched_trig]
connect_bd_net [get_bd_pins group_trig_scheduler_0/group_select_out] [get_bd_pins trig_merge_0/sched_group]
connect_bd_net [get_bd_pins dac_trig_queue_0/native_trig_out]       [get_bd_pins trig_merge_0/queue_trig]
connect_bd_net [get_bd_pins dac_trig_queue_0/native_trig_group_out] [get_bd_pins trig_merge_0/queue_group]

connect_bd_net [get_bd_pins local_reg_handler_0/au_trig_start] [get_bd_pins or_group_sched_fire_0/Op1]
connect_bd_net [get_bd_pins fifo_event_reader_0/event_pulse]    [get_bd_pins or_group_sched_fire_0/Op2]
connect_bd_net [get_bd_pins or_group_sched_fire_0/Res]           [get_bd_pins au_trig_pulse_cdc_0/src_pulse]

connect_bd_net [get_bd_pins local_reg_handler_0/au_trig_group_select] [get_bd_pins or_group_sched_group_0/Op1]
connect_bd_net [get_bd_pins fifo_event_reader_0/event_data]            [get_bd_pins or_group_sched_group_0/Op2]
connect_bd_net [get_bd_pins or_group_sched_group_0/Res]                [get_bd_pins au_trig_group_cdc_0/src_in]

# 2026-07-14：ti_reinit（TI bit17，「初始化」指令）分配——ddr_zero_
# writer_0 在 sys_clk domain，跟 ti_reinit 本身同域，直接接不用 CDC；
# wctrl_$ch 在 dac_clk domain，走 reinit_pulse_cdc_0（見上方 trigger_cdc
# 章節）。2026-07-24：trig_timer_$ch 搬到 dac_clk 後，reinit_req 也改吃
# 這顆既有 CDC——原本直接接 sys_clk 側的 ti_reinit/Dout 會變成真正跨域
# 卻沒有同步器，必須改走 CDC（wctrl 已經在用同一顆，不用新建）。
foreach ch {0 1 2 3} {
    connect_bd_net [get_bd_pins reinit_pulse_cdc_0/dst_pulse] [get_bd_pins trig_timer_$ch/reinit_req]
}
# 2026-08-04 新增：group_trig_scheduler_0 比照 trig_timer_$ch 同一套
# reinit_req 機制（同一顆既有 CDC，不用新建）
connect_bd_net [get_bd_pins reinit_pulse_cdc_0/dst_pulse] [get_bd_pins group_trig_scheduler_0/reinit_req]
# 2026-07-24 新增：新增 Aurora 廣播路徑 T_REINIT(0x27)——USB-only 功能
# 盤點裡優先權最高的一項（安全層級，slave 真實部署下沒有 USB 就完全
# 沒辦法緊急歸零，見 NOTES.md「USB-only 功能盤點」章節）。
# local_reg_handler_0/au_reinit（sys_clk domain，跟 ti_reinit/Dout 同
# domain，不需要額外 CDC）跟既有的 ti_reinit/Dout 用 OR gate 合併，
# 兩條路徑都能觸發同一組 reinit_pulse_cdc_0/ddr_zero_writer_0 目的地。
create_bd_cell -type ip -vlnv xilinx.com:ip:util_vector_logic or_reinit_0
set_property -dict [list CONFIG.C_SIZE {1} CONFIG.C_OPERATION {or}] [get_bd_cells or_reinit_0]
connect_bd_net [get_bd_pins ti_reinit/Dout]              [get_bd_pins or_reinit_0/Op1]
connect_bd_net [get_bd_pins local_reg_handler_0/au_reinit] [get_bd_pins or_reinit_0/Op2]
connect_bd_net [get_bd_pins or_reinit_0/Res] [get_bd_pins reinit_pulse_cdc_0/src_pulse]
connect_bd_net [get_bd_pins or_reinit_0/Res] [get_bd_pins ddr_zero_writer_0/start]

# out_flash_erase（2026-07-08 step 15b 新增，2026-07-09 改接 CDC）：15b
# 沒有 legacy TI-direct flash 觸發路徑，erase 沒有 payload，維持獨立
# 封包觸發（T_FLASH_ERASE -> au_flash_erase -> aurora_ctrl_mux_0 ->
# trigger_cdc -> fp_flash_cfg_erase），不需要 OR gate。
# 2026-07-09（第 29 節）：out_flash_save/T_FLASH_SAVE 這條路徑已完全
# 移除（見 cell 建立處註解）——write 觸發改成 flash_payload_cdc_0/
# payload_complete 直接驅動 fp_flash_cfg_wr，payload 收滿即自動寫入，
# 不再需要 host 額外送 T_FLASH_SAVE 封包，從根本上消除「觸發跟 payload
# 兩條路徑競爭」的問題。
connect_bd_net [get_bd_pins flash_payload_cdc_0/payload_complete] [get_bd_pins fpga_flash_ctrl_0/fp_flash_cfg_wr]
connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_flash_erase]  [get_bd_pins flash_erase_pulse_cdc_0/src_pulse]
connect_bd_net [get_bd_pins flash_erase_pulse_cdc_0/dst_pulse]  [get_bd_pins fpga_flash_ctrl_0/fp_flash_cfg_erase]
# 2026-07-24（trigger 統一化架構改版），2026-07-30 改版：first_trigger
# 不再是 ti_global_trig | local_reg_handler_0/trigger_out，改接
# dac_trig_queue_0/native_trig_out（GO 完成、補償延遲倒數本身已經在
# dac_clk domain 的 dac_trig_queue_0 跑完的最終觸發脈衝，原生就是
# dac_clk，不需要再額外 CDC）。
foreach port {0 1 2 3} {
    connect_bd_net [get_bd_pins dac_trig_queue_0/native_trig_out] [get_bd_pins trig_timer_$port/first_trigger]
}

# -- TI slice sources -------------------------------------------------------------
# 2026-07-24：ti_global_trig/ti_fp_port_trig_1-3 已移除（本機 TI global
# trigger、FP port trigger 機制拿掉），從這個 fan-out 清單移除。
# 2026-07-27：ti_calib_wr/ti_amp_ctrl_wr/ti_sine_ctrl_wr 這 3 個 cell
# 已經整個移除（見上方 cell 宣告處），連同這裡也從 fan-out 清單移除；
# sine_ctrl_regs_0/wr_strobe 這個 port 本身也已經拔掉（見該檔案 fork）。
connect_bd_net [get_bd_pins fp0/ti40_ep_trigger] \
    [get_bd_pins ti_timer_wr_1/Din] [get_bd_pins ti_timer_wr_2/Din] [get_bd_pins ti_timer_wr_3/Din]

# -- Trigger CDCs (sys_clk -> dac_clk) --------------------------------------------
# 2026-07-24（trigger 統一化架構改版）：global_trig_cdc/timer_cdc_*/
# trig_ext_cdc_0/au_trig_port_cdc_*/fp_port_cdc_*/or_final_trig_*/
# or_au_fp_trig_*/or_total_trig_* 這整組舊 merge tree 全部移除。
# 2026-07-30 改版：aurora_ctrl_channel_0 不再自己倒數，「決定要 fire」
# 那一拍送出 trig_fire_req(pulse)+trig_fire_group(4-bit)，一起進
# trig_fire_fifo_0（官方 fifo_generator，depth 16，短間隔連續事件
# 用佇列排隊，不會互相蓋掉，取代原本單一 pulse CDC 的 native_trig_
# cdc_0）。補償延遲量（native_trig_delay_out，aurora_clk cycle 單位，
# quasi-static）另外用 native_trig_delay_cdc_0（level_cdc）跨過去。
# dac_trig_queue_0（dac_clk domain）把兩者組合起來：讀 FIFO 拿到
# group_select、讀 level_cdc 拿到延遲量換算成 dac_clk cycle 數，算出
# fire_at 塞進內部深度 8 的 fire-timestamp 佇列，真正倒數/輸出
# native_trig_out 都在 dac_clk domain 原生完成。詳見 rtl/dac_trig_
# queue.v 檔頭說明、NOTES.md 2026-07-30「sine wave mode 精確度討論」
# 章節。
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/trig_fire_req]   [get_bd_pins trig_fire_fifo_0/wr_en]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/trig_fire_group] [get_bd_pins trig_fire_fifo_0/din]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/native_trig_delay_out] [get_bd_pins native_trig_delay_cdc_0/src_in]

connect_bd_net [get_bd_pins trig_fire_fifo_0/dout]  [get_bd_pins dac_trig_queue_0/fifo_rd_data]
connect_bd_net [get_bd_pins trig_fire_fifo_0/empty] [get_bd_pins dac_trig_queue_0/fifo_rd_empty]
connect_bd_net [get_bd_pins dac_trig_queue_0/fifo_rd_en] [get_bd_pins trig_fire_fifo_0/rd_en]
connect_bd_net [get_bd_pins native_trig_delay_cdc_0/dst_out] [get_bd_pins dac_trig_queue_0/delay_aurora_cycles]

# 2026-07-27 新增（Group-based Trigger 架構）：4 個 group_trig_select
# instance（每個模組 A/B/C/D 各一個，ch=0..3 對應 wctrl_0..3），純組合
# 邏輯 4:1 mux，決定這次觸發時這個模組要不要真的動作（trig_out =
# trig_pulse & group_select[group_id]）。group_id 來自 board_cfg_reg_0
# （T_TRIG_MASK 設定的分組表）。
#
# 2026-08-20 改接（多板同步輪播 Architecture B）：trig_pulse/group_select
# 原本直接接 dac_trig_queue_0 的輸出，改接 trig_merge_0（合併了
# dac_trig_queue_0 既有輸出 + group_trig_scheduler_0/fire_local 本地
# 觸發，見 rtl/trig_merge.v 檔頭說明）——arm_mode=0（legacy）時
# fire_local 永遠是 0，trig_merge_0 的輸出等同直接透傳 dac_trig_queue_0
# 的值，行為完全不變；只有 arm_mode=1 才會多出本地觸發這條路。
set group_id_letters {a b c d}
foreach ch {0 1 2 3} {
    create_bd_cell -type module -reference group_trig_select group_trig_select_$ch
    set letter [lindex $group_id_letters $ch]
    connect_bd_net [get_bd_pins trig_merge_0/group_out] [get_bd_pins group_trig_select_$ch/group_select]
    connect_bd_net [get_bd_pins trig_merge_0/trig_out]  [get_bd_pins group_trig_select_$ch/trig_pulse]
    connect_bd_net [get_bd_pins board_cfg_reg_0/group_id_$letter] [get_bd_pins group_trig_select_$ch/group_id]
}

# wctrl_$ch/sw_trigger 的新來源：group_trig_select_$ch/trig_out（多板
# 同步/本機 Aurora trigger，已依這個模組的 group 分組過濾）OR trig_
# timer_$ch/trigger_out（該 channel 自主排程，不受 group 分組影響，見
# 上方 or_sw_trig_$ch 建立處註解）。
foreach ch {0 1 2 3} {
    connect_bd_net [get_bd_pins group_trig_select_$ch/trig_out]  [get_bd_pins or_sw_trig_$ch/Op1]
    connect_bd_net [get_bd_pins trig_timer_$ch/trigger_out]   [get_bd_pins or_sw_trig_$ch/Op2]
}

# sine_ctrl_regs_0/trig_start_0..3（2026-07-27 改版，Group-based Trigger
# 架構）：原本單一 trig_start 改成 4 個 per-module port（見 rtl/sine_
# ctrl_regs.v），各自接對應模組的 or_sw_trig_$ch/Res，跟下方 sine_gen_
# ${inst}_${sub}_${ab}/trig_start、amp_ramp_gen_${inst}_${sub}_${ab}/
# trig_start 用同一條線（inst==ch），維持「glitch-free swap 必須同一個
# edge」這個既有設計要求在模組內部成立。
#
# 2026-08-05 修正：這三處（這裡+下方兩處 sine_gen/amp_ramp_gen）原本都
# 直接接 group_trig_select_$ch/trig_out，沒有經過 or_sw_trig_$ch——這顆
# OR gate（見上方建立處）本來就是把 group_trig_select_$ch/trig_out（手
# 動/群組 trigger）跟 trig_timer_$ch/trigger_out（該 channel 自主排程）
# OR 在一起，wctrl_$ch/sw_trigger（DDR）本來就正確接的是 or_sw_trig_
# $ch/Res（見下方 wctrl_$ch/sw_trigger 接線處），但 sine 這三處漏接，導致 trig_timer 自動
# 排程對 Sine 完全沒有作用（DDR 正常、Sine 靜止），只有手動/群組
# trigger 才能讓 Sine 動作。這是既有 bug（trig_timer 從沒人實測搭配過
# Sine，缺口一直沒被踩到），跟這次 N-slot 改版本身無關，上機測試 N-slot
# 的「trig_timer 自動輪播 Sine」這個原始目標時才發現。三處必須一起改，
# 不能只改其中一個，否則同一模組內 mux_sel 翻轉跟 sine_gen/amp_ramp_gen
# 實際 reload 的 trigger 來源會不同拍，破壞 glitch-free 設計。
foreach ch {0 1 2 3} {
    connect_bd_net [get_bd_pins or_sw_trig_$ch/Res] [get_bd_pins sine_ctrl_regs_0/trig_start_$ch]
}

# -- calib_mux_0 + awg_calib_regs_0 -----------------------------------------------
# 2026-07-27：WI 0x0b/0x0c → calib_sel_slice/calib_data_slice → calib_
# mux_0/fp_sel,fp_data,fp_wr 這組本機直寫路徑已整組移除（calib_mux.v
# 本地 fork 拔除 fp_wr/fp_sel/fp_data，calib_sel 讀寫共用同一個 port
# 的問題用 awg_calib_regs_0/coef_all 解決，見 PORTS.md 表3 0x0b 條目）。
connect_bd_net [get_bd_pins local_reg_handler_0/calib_wr_en]   [get_bd_pins calib_mux_0/au_wr]
connect_bd_net [get_bd_pins local_reg_handler_0/calib_wr_addr] [get_bd_pins calib_mux_0/au_sel]
connect_bd_net [get_bd_pins local_reg_handler_0/calib_wr_data] [get_bd_pins calib_mux_0/au_data]
connect_bd_net [get_bd_pins local_reg_handler_0/calib_rst_out] [get_bd_pins calib_mux_0/au_rst]
connect_bd_net [get_bd_pins calib_mux_0/calib_rst]  [get_bd_pins awg_calib_regs_0/rst]
connect_bd_net [get_bd_pins calib_mux_0/calib_sel]  [get_bd_pins awg_calib_regs_0/calib_sel]
connect_bd_net [get_bd_pins calib_mux_0/calib_data] [get_bd_pins awg_calib_regs_0/calib_data]
connect_bd_net [get_bd_pins calib_mux_0/calib_wr]   [get_bd_pins awg_calib_regs_0/calib_wr]
# 2026-07-15：scale_cfg 改由 board_cfg_reg_0 管理（該模組已經處理好
# flash/fp/au 三路合併跟優先序），這裡直接接它算好的結果，取代原本
# aurora_ctrl_mux_0/out_scale_cfg -> scale_cfg_slice_calib 那條路徑
# （scale_cfg_slice_calib cell 保留在 BD 裡但不再使用，避免動到其他
# 可能還有引用的地方；out_scale_cfg 本身也變成沒有消費者的死路）
connect_bd_net [get_bd_pins board_cfg_reg_0/scale_cfg] [get_bd_pins awg_calib_regs_0/scale_cfg]
# flash controller 接回來（2026-07-08 step 15b），取代原本 const tie-off
# 2026-07-15：calib_coef 現在存在獨立 sector，改接專屬的
# flash_coef_load_valid_cdc_0（理由同上面 amp_ctrl 那條）
connect_bd_net [get_bd_pins flash_coef_load_valid_cdc_0/dst_pulse] [get_bd_pins awg_calib_regs_0/flash_load_valid]
connect_bd_net [get_bd_pins fpga_flash_ctrl_0/init_scale_cfg] [get_bd_pins awg_calib_regs_0/flash_scale_cfg]
foreach i {0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31} {
    connect_bd_net [get_bd_pins fpga_flash_ctrl_0/init_coef_$i] [get_bd_pins awg_calib_regs_0/flash_coef_$i]
}

# -- awg_calib_regs -> ZmodAWG coefficient/scale ----------------------------------
foreach inst {0 1 2 3} {
    foreach ch {1 2} {
        if {$ch == 1} {set ch_tag "Ch1"} else {set ch_tag "Ch2"}
        connect_bd_net \
            [get_bd_pins awg_calib_regs_0/z${inst}_ch${ch}_scale] \
            [get_bd_pins zmod_awg_${inst}/sExtCh${ch}Scale]
        foreach mode_info {{hg Hg} {lg Lg}} {
            set mode    [lindex $mode_info 0]
            set ModeStr [lindex $mode_info 1]
            foreach type_info {{mult Mult} {add Add}} {
                set type    [lindex $type_info 0]
                set TypeStr [lindex $type_info 1]
                connect_bd_net \
                    [get_bd_pins awg_calib_regs_0/z${inst}_ch${ch}_${mode}_${type}] \
                    [get_bd_pins zmod_awg_${inst}/cExt${ch_tag}${ModeStr}${TypeStr}Coef]
            }
        }
    }
}

# -- Waveform controllers + readers + FIFOs ---------------------------------------
foreach ch {0 1 2 3} {
    # 2026-08-10：list_wr_sel/addr/len/en 改接 list_*_cdc_0 的 dac_clk
    # 側輸出（見上方 aurora_ctrl_mux_0 outputs 段落新增的 CDC 接線），
    # 不再是 sys_clk 側訊號直接硬接進 dac_clk domain。
    connect_bd_net [get_bd_pins list_sel_cdc_0/dst_out]    [get_bd_pins wctrl_$ch/list_wr_sel]
    connect_bd_net [get_bd_pins list_addr_cdc_0/dst_out]   [get_bd_pins wctrl_$ch/list_wr_addr]
    connect_bd_net [get_bd_pins list_len_cdc_0/dst_out]    [get_bd_pins wctrl_$ch/list_wr_len]
    connect_bd_net [get_bd_pins list_wr_cdc_0/dst_pulse]   [get_bd_pins wctrl_$ch/list_wr_en]
    connect_bd_net [get_bd_pins depth_slice_$ch/Dout]      [get_bd_pins wctrl_$ch/list_depth]
    connect_bd_net [get_bd_pins play_en_slice_$ch/Dout]    [get_bd_pins wctrl_$ch/play_en]
    connect_bd_net [get_bd_pins or_sw_trig_$ch/Res]        [get_bd_pins wctrl_$ch/sw_trigger]
    # 2026-07-24 改版（trigger 統一化架構改版）：ext_trigger 直接接觸發
    # 來源，不用 OR gate——trig_ext_cdc_0 舊 sys_clk 補償路徑已移除，只
    # 剩這一個來源。2026-07-27 再改（Group-based Trigger 架構）：來源
    # 從 native_trig_cdc_0/dst_pulse 改成這個模組自己的 group_trig_
    # select_$ch/trig_out（已依 group 分組過濾）。
    connect_bd_net [get_bd_pins group_trig_select_$ch/trig_out] [get_bd_pins wctrl_$ch/ext_trigger]
    connect_bd_net [get_bd_pins sync_start_0/all_ready]    [get_bd_pins wctrl_$ch/all_ready]
    connect_bd_net [get_bd_pins reinit_pulse_cdc_0/dst_pulse]  [get_bd_pins wctrl_$ch/reinit_req]
    connect_bd_net [get_bd_pins wctrl_$ch/mux_sel]         [get_bd_pins concat_mux_sel_4b/In$ch]
    # wctrl -> readers
    connect_bd_net [get_bd_pins wctrl_$ch/ra_start_addr]  [get_bd_pins reader_a_$ch/start_addr]
    connect_bd_net [get_bd_pins wctrl_$ch/ra_wave_len]    [get_bd_pins reader_a_$ch/wave_len]
    connect_bd_net [get_bd_pins wctrl_$ch/ra_play_en]     [get_bd_pins reader_a_$ch/play_en]
    connect_bd_net [get_bd_pins wctrl_$ch/rb_start_addr]  [get_bd_pins reader_b_$ch/start_addr]
    connect_bd_net [get_bd_pins wctrl_$ch/rb_wave_len]    [get_bd_pins reader_b_$ch/wave_len]
    connect_bd_net [get_bd_pins wctrl_$ch/rb_play_en]     [get_bd_pins reader_b_$ch/play_en]
    # readers -> fifos
    connect_bd_net [get_bd_pins reader_a_$ch/fifo_din]    [get_bd_pins fifo_a_$ch/din]
    connect_bd_net [get_bd_pins reader_a_$ch/fifo_we]     [get_bd_pins fifo_a_$ch/wr_en]
    connect_bd_net [get_bd_pins fifo_a_$ch/prog_full]     [get_bd_pins reader_a_$ch/fifo_prog_full]
    connect_bd_net [get_bd_pins reader_b_$ch/fifo_din]    [get_bd_pins fifo_b_$ch/din]
    connect_bd_net [get_bd_pins reader_b_$ch/fifo_we]     [get_bd_pins fifo_b_$ch/wr_en]
    connect_bd_net [get_bd_pins fifo_b_$ch/prog_full]     [get_bd_pins reader_b_$ch/fifo_prog_full]
    # fifos -> wctrl
    connect_bd_net [get_bd_pins fifo_a_$ch/dout]          [get_bd_pins wctrl_$ch/fifo_a_dout]
    connect_bd_net [get_bd_pins fifo_a_$ch/empty]         [get_bd_pins wctrl_$ch/fifo_a_empty]
    connect_bd_net [get_bd_pins wctrl_$ch/fifo_a_rd_en]   [get_bd_pins fifo_a_$ch/rd_en]
    connect_bd_net [get_bd_pins fifo_b_$ch/dout]          [get_bd_pins wctrl_$ch/fifo_b_dout]
    connect_bd_net [get_bd_pins fifo_b_$ch/empty]         [get_bd_pins wctrl_$ch/fifo_b_empty]
    connect_bd_net [get_bd_pins wctrl_$ch/fifo_b_rd_en]   [get_bd_pins fifo_b_$ch/rd_en]
    # 2026-07-04: wctrl_$ch/fifo_a_flush/fifo_b_flush now feed or_fifo_a/b_rst_$ch
    # (combined with power-on reset) instead of a dedicated FIFO flush port -- see
    # the or_fifo_poweron_rst/or_fifo_a_rst_$ch/or_fifo_b_rst_$ch wiring earlier in
    # this file. New: dc_fifo_xpm's wr_rst_busy/rd_rst_busy -> wctrl_$ch, so the
    # flush/reconfigure state machine waits for the official IP's own reset-busy
    # handshake instead of trusting a fixed guessed cycle count alone.
    connect_bd_net [get_bd_pins fifo_a_$ch/wr_rst_busy]   [get_bd_pins wctrl_$ch/fifo_a_wr_rst_busy]
    connect_bd_net [get_bd_pins fifo_a_$ch/rd_rst_busy]   [get_bd_pins wctrl_$ch/fifo_a_rd_rst_busy]
    connect_bd_net [get_bd_pins fifo_b_$ch/wr_rst_busy]   [get_bd_pins wctrl_$ch/fifo_b_wr_rst_busy]
    connect_bd_net [get_bd_pins fifo_b_$ch/rd_rst_busy]   [get_bd_pins wctrl_$ch/fifo_b_rd_rst_busy]
    # 2026-07-26: reader_a/b's new `idle` output -> wctrl_$ch/ra_idle/rb_idle,
    # raw (ui_clk domain), 2-flop synced into dac_clk inside waveform_
    # controller.v itself (same pattern as fifo_a/b_wr_rst_busy above) -- see
    # rtl/ddr4_stream_reader.v / rtl/waveform_controller.v header comments.
    connect_bd_net [get_bd_pins reader_a_$ch/idle]        [get_bd_pins wctrl_$ch/ra_idle]
    connect_bd_net [get_bd_pins reader_b_$ch/idle]        [get_bd_pins wctrl_$ch/rb_idle]
    # fifo empty -> sync_start concats
    connect_bd_net [get_bd_pins fifo_a_$ch/empty]         [get_bd_pins concat_fifo_a_empty/In$ch]
    connect_bd_net [get_bd_pins fifo_b_$ch/empty]         [get_bd_pins concat_fifo_b_empty/In$ch]
    # play_en -> sync_start concat
    connect_bd_net [get_bd_pins play_en_slice_$ch/Dout]   [get_bd_pins concat_play_en_4b/In$ch]
    # wctrl -> ZmodAWG data
    # 2026-07-17: inserted dac_output_mux_$ch between wctrl_$ch and
    # zmod_awg_$ch to select between DDR4 playback and the new sine
    # generator (mode driven by sine_ctrl_regs_0/mode_active_$ch,
    # 2026-08-05 併入 sine_ctrl_regs_0 後改接，見該檔案檔頭說明).
    # 2026-07-23: dac_output_mux_$ch now takes the 2 ACTIVE sine sources
    # (ch1/ch2, one per physical channel) instead of 1 -- see the sine_gen/
    # amp_ramp_gen 8-channel wiring loop below for where sine_code_ch{1,2}_
    # {a,b}/sine_mux_sel_ch{1,2} actually come from.
    connect_bd_net [get_bd_pins wctrl_$ch/dac_data]       [get_bd_pins dac_output_mux_$ch/ddr_data]
    connect_bd_net [get_bd_pins wctrl_$ch/dac_valid]      [get_bd_pins dac_output_mux_$ch/ddr_valid]
    connect_bd_net [get_bd_pins dac_output_mux_$ch/out_data]  [get_bd_pins zmod_awg_$ch/cDataAxisTdata]
    connect_bd_net [get_bd_pins dac_output_mux_$ch/out_valid] [get_bd_pins zmod_awg_$ch/cDataAxisTvalid]
    connect_bd_net [get_bd_pins const_zero/dout]          [get_bd_pins zmod_awg_$ch/sTestMode]
    connect_bd_net [get_bd_pins const_one/dout]           [get_bd_pins zmod_awg_$ch/sDAC_EnIn]
}

# -- sine_gen/amp_ramp_gen 8-channel wiring (rewritten 2026-07-24, retriggered 2026-07-27) ---
# 2026-07-27（Group-based Trigger 架構）：原本 32 個 instance（16 sine_gen
# + 16 amp_ramp_gen）跟 sine_ctrl_regs_0/trig_start 共用同一條
# native_trig_cdc_0/dst_pulse，改成每個模組（inst=0..3，對應 A/B/C/D）
# 各自接自己的 or_sw_trig_$inst/Res（inst==ch，跟 wctrl_$ch 用同一顆
# OR gate）——同一個模組內的 sine_gen/amp_ramp_gen（sub=1/2、ab=a/b 共
# 4 個 instance）+ sine_ctrl_regs_0/trig_start_$inst 三者仍然是同一條
# 線同一拍生效，「glitch-free swap 必須同一個 edge」這個既有設計要求
# 在模組內部沒有被破壞（見 sine_ctrl_regs.v header 註解 2026-07-27 補
# 充說明），只是現在允許不同模組在不同拍各自動作，不再要求全部 4 個
# 模組永遠同步。
#
# 2026-08-05 修正：這兩處原本直接接 group_trig_select_$inst/trig_out，
# 沒有經過 or_sw_trig_$inst（跟上方 sine_ctrl_regs_0/trig_start_$ch 同
# 一個既有 bug，一起修，理由/根因見該處 2026-08-05 註解）。
foreach inst {0 1 2 3} {
    foreach sub {1 2} {
        set i [expr {2*$inst + ($sub == 2 ? 1 : 0)}]
        foreach ab {a b} {
            connect_bd_net [get_bd_pins or_sw_trig_${inst}/Res] [get_bd_pins sine_gen_${inst}_${sub}_${ab}/trig_start]
            connect_bd_net [get_bd_pins sine_ctrl_regs_0/tuning_word_stage_${i}_${ab}] [get_bd_pins sine_gen_${inst}_${sub}_${ab}/tuning_word_stage]
            connect_bd_net [get_bd_pins sine_ctrl_regs_0/phase_stage_${i}_${ab}]       [get_bd_pins sine_gen_${inst}_${sub}_${ab}/phase_stage]
            # 2026-07-27 新增（統一讀取/寫入架構，QT_SINE_STATUS 查詢用）
            connect_bd_net [get_bd_pins sine_gen_${inst}_${sub}_${ab}/phase_acc_out] [get_bd_pins sine_phase_acc_mux_0/phase_acc_${i}_${ab}]

            connect_bd_net [get_bd_pins or_sw_trig_${inst}/Res] [get_bd_pins amp_ramp_gen_${inst}_${sub}_${ab}/trig_start]
            connect_bd_net [get_bd_pins sine_ctrl_regs_0/start_amp_stage_${i}_${ab}]        [get_bd_pins amp_ramp_gen_${inst}_${sub}_${ab}/start_amp_stage]
            connect_bd_net [get_bd_pins sine_ctrl_regs_0/step_stage_${i}_${ab}]             [get_bd_pins amp_ramp_gen_${inst}_${sub}_${ab}/step_stage]
            connect_bd_net [get_bd_pins sine_ctrl_regs_0/duration_cycles_stage_${i}_${ab}]  [get_bd_pins amp_ramp_gen_${inst}_${sub}_${ab}/duration_cycles_stage]
            connect_bd_net [get_bd_pins sine_ctrl_regs_0/loop_mode_stage_${i}_${ab}]        [get_bd_pins amp_ramp_gen_${inst}_${sub}_${ab}/loop_mode_stage]
        }
    }
    # dac_output_mux_$inst: ch1 comes from sub=1's a/b pair, ch2 from sub=2's
    set i_ch1 [expr {2*$inst}]
    set i_ch2 [expr {2*$inst + 1}]
    connect_bd_net [get_bd_pins sine_gen_${inst}_1_a/dac_code] [get_bd_pins dac_output_mux_$inst/sine_code_ch1_a]
    connect_bd_net [get_bd_pins sine_gen_${inst}_1_b/dac_code] [get_bd_pins dac_output_mux_$inst/sine_code_ch1_b]
    connect_bd_net [get_bd_pins sine_ctrl_regs_0/mux_sel_$i_ch1]  [get_bd_pins dac_output_mux_$inst/sine_mux_sel_ch1]
    connect_bd_net [get_bd_pins sine_gen_${inst}_2_a/dac_code] [get_bd_pins dac_output_mux_$inst/sine_code_ch2_a]
    connect_bd_net [get_bd_pins sine_gen_${inst}_2_b/dac_code] [get_bd_pins dac_output_mux_$inst/sine_code_ch2_b]
    connect_bd_net [get_bd_pins sine_ctrl_regs_0/mux_sel_$i_ch2]  [get_bd_pins dac_output_mux_$inst/sine_mux_sel_ch2]
    # 2026-07-27 新增（統一讀取/寫入架構，QT_SINE_STATUS 查詢用）：
    # sine_phase_acc_mux_0 要用 dac_clk 原始版 mux_sel（不是 sys_clk
    # 已同步的 mux_sel_sync）選出目前 active 的 phase_acc，跟
    # dac_output_mux_$inst 用同一個來源訊號多 fan-out 一份。
    connect_bd_net [get_bd_pins sine_ctrl_regs_0/mux_sel_$i_ch1] [get_bd_pins sine_phase_acc_mux_0/mux_sel_$i_ch1]
    connect_bd_net [get_bd_pins sine_ctrl_regs_0/mux_sel_$i_ch2] [get_bd_pins sine_phase_acc_mux_0/mux_sel_$i_ch2]
}

# -- aurora_reply_tx_0（統一讀取/寫入架構，見 rtl/aurora_reply_tx.v /
#    PORTS.md「統一讀取/寫入架構」章節）-------------------------------------------

# T_QUERY 解碼觸發（來自 local_reg_handler_0，同 sys_clk domain）
connect_bd_net [get_bd_pins local_reg_handler_0/req_reply]      [get_bd_pins aurora_reply_tx_0/req_reply]
connect_bd_net [get_bd_pins local_reg_handler_0/reply_dest_id]  [get_bd_pins aurora_reply_tx_0/reply_dest_id]
connect_bd_net [get_bd_pins local_reg_handler_0/req_query_type] [get_bd_pins aurora_reply_tx_0/req_query_type]

# 2026-08 新增：T_QUERY 回覆本機捷徑（修 PROJECT.md 記錄的已知設計缺口
# ——查自己時原本一定要讓回覆封包真的繞完整個實體 Aurora 環路才能被
# 解碼，SFP 沒接/環路沒通時永遠讀不到，見 rtl/aurora_reply_tx.v 對應
# port 註解）。aurora_reply_tx_0 判斷 reply_dest_id==bi_board_id 時直接
# 產生這批訊號，local_reg_handler_0 用跟 ti_debug_reply_trig 一樣的
# 獨立注入方式接收，兩者都在 sys_clk domain，直接接線，不需要 CDC。
connect_bd_net [get_bd_pins aurora_reply_tx_0/local_reply_valid]      [get_bd_pins local_reg_handler_0/local_reply_valid]
connect_bd_net [get_bd_pins aurora_reply_tx_0/local_reply_src]        [get_bd_pins local_reg_handler_0/local_reply_src]
connect_bd_net [get_bd_pins aurora_reply_tx_0/local_reply_query_type] [get_bd_pins local_reg_handler_0/local_reply_query_type]
connect_bd_net [get_bd_pins aurora_reply_tx_0/local_reply_payload]    [get_bd_pins local_reg_handler_0/local_reply_payload]

# QT_BOARD_INFO 欄位（都是既有 sys_clk domain 準靜態值，多 fan-out 一份，
# 沿用既有 WO 0x26/0x27/0x2C/0x34 讀回用的同一批已同步訊號）
connect_bd_net [get_bd_pins board_cfg_reg_0/board_id]        [get_bd_pins aurora_reply_tx_0/bi_board_id]
connect_bd_net [get_bd_pins board_cfg_reg_0/is_master]       [get_bd_pins aurora_reply_tx_0/bi_is_master]
# 2026-07-30 修正：total_boards_cdc_0/init_ok_cdc_0 的來源是 aurora_
# ctrl_channel_0/total_boards|init_ok，這兩個 register 只有 master
# 執行 enum 才會被寫入，slave 上永遠是 reset 值 0（WO 0x26 本機查詢
# 天生只查自己，不會踩到這個限制，這兩顆 CDC 繼續留給 WO 0x26 用，
# 不要刪）。QT_BOARD_INFO 是跨板查詢，改接 local_reg_handler_0/
# bid_total_boards（T_BOARD_ID_ASSIGN 廣播解出來的值，每片板子都
# 收得到）+ 新增的 bid_init_ok（sticky，收過 T_BOARD_ID_ASSIGN 就是
# 1）。兩者都跟 aurora_reply_tx_0 同一個 sys_clk domain，不需要 CDC。
# 見 rtl/local_reg_handler.v bid_init_ok port 註解、NOTES.md 2026-07-30
# 「Windows 上機驗收：build12」章節根因分析。
connect_bd_net [get_bd_pins local_reg_handler_0/bid_total_boards] [get_bd_pins aurora_reply_tx_0/bi_total_boards]
connect_bd_net [get_bd_pins local_reg_handler_0/bid_init_ok]      [get_bd_pins aurora_reply_tx_0/bi_init_ok]
connect_bd_net [get_bd_pins board_index_cdc_0/dst_out]       [get_bd_pins aurora_reply_tx_0/bi_board_index]
connect_bd_net [get_bd_pins per_hop_value_eff_cdc_0/dst_out] [get_bd_pins aurora_reply_tx_0/bi_per_hop_value]
connect_bd_net [get_bd_pins channel_up0_cdc_0/dst_out]       [get_bd_pins aurora_reply_tx_0/bi_channel_up_0]
connect_bd_net [get_bd_pins channel_up1_cdc_0/dst_out]       [get_bd_pins aurora_reply_tx_0/bi_channel_up_1]
connect_bd_net [get_bd_pins board_cfg_reg_0/ext_clk_sel]     [get_bd_pins aurora_reply_tx_0/bi_ext_clk_sel]
# 2026-07-30 新增：trigger delay 手動覆寫狀態，local_reg_handler_0
# 跟 aurora_reply_tx_0 同一個 sys_clk domain，不需要 CDC，直接接。
connect_bd_net [get_bd_pins local_reg_handler_0/au_trig_delay]              [get_bd_pins aurora_reply_tx_0/bi_trig_delay]
connect_bd_net [get_bd_pins local_reg_handler_0/manual_delay_override_active] [get_bd_pins aurora_reply_tx_0/bi_manual_delay_active]
# 2026-07-30 新增：web 控制介面 master-only USB 需求補上的兩個欄位，
# 見 NOTES.md「QT_BOARD_INFO 擴充規格」。兩個來源 pin 本來就已經扇出
# 給其他既有目的地（dac_mode_ramp_concat_0/dout → concat_rt_board_
# info/In3，2026-08-05 改接，見該 cell 建立處註解；ext_clk_freq_
# counter_0/freq_count_sync → fp0/wo2e_ep_datain），這裡再多接一條到
# aurora_reply_tx_0，同一個 sys_clk domain 準靜態值，不需要額外 CDC。
connect_bd_net [get_bd_pins dac_mode_ramp_concat_0/dout]            [get_bd_pins aurora_reply_tx_0/bi_dac_mode_ramp]
connect_bd_net [get_bd_pins ext_clk_freq_counter_0/freq_count_sync] [get_bd_pins aurora_reply_tx_0/bi_ext_clk_freq_count]

# 2026-08-20 新增：dispatcher_0/diag_tx_timeout_seen -> aurora_reply_tx_0/
# bi_tx_timeout_seen，兩者都是 sys_clk domain，不需要 CDC（見
# rtl/aurora_reply_tx.v port 註解）。
connect_bd_net [get_bd_pins dispatcher_0/diag_tx_timeout_seen]      [get_bd_pins aurora_reply_tx_0/bi_tx_timeout_seen]

# 2026-08-20 Phase 7 新增：DIAG（enum 失敗斷點定位）sys_clk<->aurora_clk
# CDC，比照 reserve_* 的舊模式（每個功能各自一組專屬 level_cdc/
# trigger_cdc，使用者確認過的做法，見 PROJECT.md「Phase 7 DIAG」章節），
# 不比照 debug-only 那些已放棄的 OR-merge/sticky-latch 額外接線
# （DIAG 沒有舊的 TI-direct 除錯路徑要合併）。查詢結果透過 aurora_
# reply_tx_0 的 board_info 封包讀回 host（T_QUERY/QT_BOARD_INFO），
# 不佔用 WireOut（0x20-0x3F 32 個已全部用完，見設計討論）。
create_bd_cell -type module -reference level_cdc diag_dest_id_cdc_0
set_property -dict [list CONFIG.WIDTH {16}] [get_bd_cells diag_dest_id_cdc_0]
create_bd_cell -type module -reference trigger_cdc au_diag_pulse_cdc_0
create_bd_cell -type module -reference level_cdc diag_ok_cdc_0
create_bd_cell -type module -reference level_cdc diag_busy_cdc_0
create_bd_cell -type module -reference level_cdc diag_r_channel_up_0_cdc_0
create_bd_cell -type module -reference level_cdc diag_r_channel_up_1_cdc_0
create_bd_cell -type module -reference level_cdc diag_r_relay_blocked_cdc_0

# sys_clk -> aurora_clk 方向（host 觸發）
connect_bd_net [get_bd_pins clk_wiz_0/clk_100] \
    [get_bd_pins diag_dest_id_cdc_0/src_clk]  \
    [get_bd_pins au_diag_pulse_cdc_0/src_clk] \
    [get_bd_pins diag_ok_cdc_0/dst_clk]              \
    [get_bd_pins diag_busy_cdc_0/dst_clk]            \
    [get_bd_pins diag_r_channel_up_0_cdc_0/dst_clk]  \
    [get_bd_pins diag_r_channel_up_1_cdc_0/dst_clk]  \
    [get_bd_pins diag_r_relay_blocked_cdc_0/dst_clk]

# aurora_clk -> sys_clk 方向（查詢結果）
connect_bd_net [get_bd_pins aurora_64b66b_0/user_clk_out] \
    [get_bd_pins diag_dest_id_cdc_0/dst_clk]  \
    [get_bd_pins au_diag_pulse_cdc_0/dst_clk] \
    [get_bd_pins diag_ok_cdc_0/src_clk]              \
    [get_bd_pins diag_busy_cdc_0/src_clk]            \
    [get_bd_pins diag_r_channel_up_0_cdc_0/src_clk]  \
    [get_bd_pins diag_r_channel_up_1_cdc_0/src_clk]  \
    [get_bd_pins diag_r_relay_blocked_cdc_0/src_clk]

# reset：au_diag_pulse_cdc_0 是唯一需要 reset 的（trigger_cdc），跟
# au_reserve_pulse_cdc_0 完全同一組——src_rst 用 sys_clk domain 的
# rst_clk100/peripheral_reset，dst_rst 用 aurora_clk domain、跟
# aurora_ctrl_channel_0 共用同一組的 or_aurora_extra_rst2/Res。
# level_cdc 沒有 rst port（純 2-flop 同步器，不需要）。
connect_bd_net [get_bd_pins rst_clk100/peripheral_reset] [get_bd_pins au_diag_pulse_cdc_0/src_rst]
# 2026-08-20 Phase 1：同上（見 au_init_pulse_cdc_0 附近的說明），改接
# const_zero，不再接 or_aurora_extra_rst2/Res——避免違反 XPM_CDC_PULSE
# 的 src_rst/dest_rst 必須同時 assert 規範。
connect_bd_net [get_bd_pins const_zero/dout]     [get_bd_pins au_diag_pulse_cdc_0/dst_rst]

# host 觸發路徑：local_reg_handler_0（T_DIAG_START 0x1F 解碼）-> CDC ->
# aurora_ctrl_channel_0
connect_bd_net [get_bd_pins local_reg_handler_0/au_diag_dest_id] [get_bd_pins diag_dest_id_cdc_0/src_in]
connect_bd_net [get_bd_pins diag_dest_id_cdc_0/dst_out]          [get_bd_pins aurora_ctrl_channel_0/diag_dest_id]
connect_bd_net [get_bd_pins local_reg_handler_0/au_diag_start]   [get_bd_pins au_diag_pulse_cdc_0/src_pulse]
connect_bd_net [get_bd_pins au_diag_pulse_cdc_0/dst_pulse]       [get_bd_pins aurora_ctrl_channel_0/diag_start_pulse]

# 查詢結果路徑：aurora_ctrl_channel_0 -> CDC -> aurora_reply_tx_0
# （board_info_word2，見 rtl/aurora_reply_tx.v）
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_ok]              [get_bd_pins diag_ok_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_busy]            [get_bd_pins diag_busy_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_r_channel_up_0]  [get_bd_pins diag_r_channel_up_0_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_r_channel_up_1]  [get_bd_pins diag_r_channel_up_1_cdc_0/src_in]
connect_bd_net [get_bd_pins aurora_ctrl_channel_0/diag_r_relay_blocked] [get_bd_pins diag_r_relay_blocked_cdc_0/src_in]
connect_bd_net [get_bd_pins diag_ok_cdc_0/dst_out]              [get_bd_pins aurora_reply_tx_0/bi_diag_ok]
connect_bd_net [get_bd_pins diag_busy_cdc_0/dst_out]            [get_bd_pins aurora_reply_tx_0/bi_diag_busy]
connect_bd_net [get_bd_pins diag_r_channel_up_0_cdc_0/dst_out]  [get_bd_pins aurora_reply_tx_0/bi_diag_r_channel_up_0]
connect_bd_net [get_bd_pins diag_r_channel_up_1_cdc_0/dst_out]  [get_bd_pins aurora_reply_tx_0/bi_diag_r_channel_up_1]
connect_bd_net [get_bd_pins diag_r_relay_blocked_cdc_0/dst_out] [get_bd_pins aurora_reply_tx_0/bi_diag_r_relay_blocked]

# QT_SINE_STATUS：sine_stage_a/b 是 sine_ctrl_regs_0 的 96 個 stage
# output（8 channel x 6 param x a/b）攤平打包成的 1536-bit 匯流排。
# xlconcat 有 port 數上限，分兩階段：先每個 channel 6 個 param 併成
# 192-bit block（16 個 xlconcat），再把 8 個 block 併成 1536-bit
# （2 個 xlconcat，a/b 各一）。channel-major/param-minor 排列跟
# aurora_reply_tx.v/PORTS.md 文件一致，不要憑印象改順序。
foreach ab {a b} {
    foreach ch {0 1 2 3 4 5 6 7} {
        create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat sine_stage_block_${ch}_${ab}
        set_property -dict [list CONFIG.NUM_PORTS {6} \
            CONFIG.IN0_WIDTH {32} CONFIG.IN1_WIDTH {32} CONFIG.IN2_WIDTH {32} \
            CONFIG.IN3_WIDTH {32} CONFIG.IN4_WIDTH {32} CONFIG.IN5_WIDTH {32}] \
            [get_bd_cells sine_stage_block_${ch}_${ab}]
        connect_bd_net [get_bd_pins sine_ctrl_regs_0/tuning_word_stage_${ch}_${ab}]     [get_bd_pins sine_stage_block_${ch}_${ab}/In0]
        connect_bd_net [get_bd_pins sine_ctrl_regs_0/phase_stage_${ch}_${ab}]           [get_bd_pins sine_stage_block_${ch}_${ab}/In1]
        connect_bd_net [get_bd_pins sine_ctrl_regs_0/start_amp_stage_${ch}_${ab}]       [get_bd_pins sine_stage_block_${ch}_${ab}/In2]
        connect_bd_net [get_bd_pins sine_ctrl_regs_0/step_stage_${ch}_${ab}]            [get_bd_pins sine_stage_block_${ch}_${ab}/In3]
        connect_bd_net [get_bd_pins sine_ctrl_regs_0/duration_cycles_stage_${ch}_${ab}] [get_bd_pins sine_stage_block_${ch}_${ab}/In4]
        connect_bd_net [get_bd_pins sine_ctrl_regs_0/loop_mode_stage_${ch}_${ab}]       [get_bd_pins sine_stage_block_${ch}_${ab}/In5]
    }
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat sine_stage_full_${ab}
    set_property -dict [list CONFIG.NUM_PORTS {8} \
        CONFIG.IN0_WIDTH {192} CONFIG.IN1_WIDTH {192} CONFIG.IN2_WIDTH {192} CONFIG.IN3_WIDTH {192} \
        CONFIG.IN4_WIDTH {192} CONFIG.IN5_WIDTH {192} CONFIG.IN6_WIDTH {192} CONFIG.IN7_WIDTH {192}] \
        [get_bd_cells sine_stage_full_${ab}]
    foreach ch {0 1 2 3 4 5 6 7} {
        connect_bd_net [get_bd_pins sine_stage_block_${ch}_${ab}/dout] [get_bd_pins sine_stage_full_${ab}/In${ch}]
    }
    connect_bd_net [get_bd_pins sine_stage_full_${ab}/dout] [get_bd_pins aurora_reply_tx_0/sine_stage_${ab}]
}
connect_bd_net [get_bd_pins sine_ctrl_regs_0/mux_sel_sync]         [get_bd_pins aurora_reply_tx_0/sine_mux_sel_sync]
connect_bd_net [get_bd_pins sine_phase_acc_mux_0/phase_acc_active] [get_bd_pins aurora_reply_tx_0/phase_acc_active]

# QT_CALIB_STATUS：cal_amp_ctrl 是 aurora_ctrl_mux_0 的 8 個 out_amp_
# ctrl_0..7（18-bit each）攤平打包成 144-bit
create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat cal_amp_ctrl_concat
set_property -dict [list CONFIG.NUM_PORTS {8} \
    CONFIG.IN0_WIDTH {18} CONFIG.IN1_WIDTH {18} CONFIG.IN2_WIDTH {18} CONFIG.IN3_WIDTH {18} \
    CONFIG.IN4_WIDTH {18} CONFIG.IN5_WIDTH {18} CONFIG.IN6_WIDTH {18} CONFIG.IN7_WIDTH {18}] \
    [get_bd_cells cal_amp_ctrl_concat]
foreach ch {0 1 2 3 4 5 6 7} {
    connect_bd_net [get_bd_pins aurora_ctrl_mux_0/out_amp_ctrl_$ch] [get_bd_pins cal_amp_ctrl_concat/In$ch]
}
connect_bd_net [get_bd_pins cal_amp_ctrl_concat/dout]  [get_bd_pins aurora_reply_tx_0/cal_amp_ctrl]
connect_bd_net [get_bd_pins board_cfg_reg_0/scale_cfg] [get_bd_pins aurora_reply_tx_0/cal_scale_cfg]
connect_bd_net [get_bd_pins awg_calib_regs_0/coef_all] [get_bd_pins aurora_reply_tx_0/cal_coef_all]

# QT_TRIGGER_GROUP
connect_bd_net [get_bd_pins board_cfg_reg_0/group_id_a] [get_bd_pins aurora_reply_tx_0/grp_id_a]
connect_bd_net [get_bd_pins board_cfg_reg_0/group_id_b] [get_bd_pins aurora_reply_tx_0/grp_id_b]
connect_bd_net [get_bd_pins board_cfg_reg_0/group_id_c] [get_bd_pins aurora_reply_tx_0/grp_id_c]
connect_bd_net [get_bd_pins board_cfg_reg_0/group_id_d] [get_bd_pins aurora_reply_tx_0/grp_id_d]

# -- status_reply_capture_0（PO_STATUS_REPLY=0xA1，host 讀查詢回覆用，
#    見下方 fp0 IP 設定新增的 PO ADDR）------------------------------------------
connect_bd_net [get_bd_pins local_reg_handler_0/au_reply_src]        [get_bd_pins status_reply_capture_0/au_reply_src]
connect_bd_net [get_bd_pins local_reg_handler_0/au_reply_query_type] [get_bd_pins status_reply_capture_0/au_reply_query_type]
connect_bd_net [get_bd_pins local_reg_handler_0/au_reply_data]       [get_bd_pins status_reply_capture_0/au_reply_data]

# -- sync_start_0 -----------------------------------------------------------------
connect_bd_net [get_bd_pins concat_play_en_4b/dout]   [get_bd_pins sync_start_0/play_en]
connect_bd_net [get_bd_pins concat_fifo_a_empty/dout] [get_bd_pins sync_start_0/fifo_a_empty]
connect_bd_net [get_bd_pins concat_fifo_b_empty/dout] [get_bd_pins sync_start_0/fifo_b_empty]
connect_bd_net [get_bd_pins concat_mux_sel_4b/dout]   [get_bd_pins sync_start_0/mux_sel]

# -- ZmodAWG physical ports -------------------------------------------------------
foreach ch {0 1 2 3} {
    connect_bd_net [get_bd_pins zmod_awg_$ch/sZmodDAC_CS]     [get_bd_ports sZmodDAC_CS_$ch]
    connect_bd_net [get_bd_pins zmod_awg_$ch/sZmodDAC_SCLK]   [get_bd_ports sZmodDAC_SCLK_$ch]
    connect_bd_net [get_bd_pins zmod_awg_$ch/sZmodDAC_SDIO]   [get_bd_ports sZmodDAC_SDIO_$ch]
    connect_bd_net [get_bd_pins zmod_awg_$ch/sZmodDAC_Reset]  [get_bd_ports sZmodDAC_Reset_$ch]
    connect_bd_net [get_bd_pins zmod_awg_$ch/ZmodDAC_ClkIO]   [get_bd_ports ZmodDAC_ClkIO_$ch]
    connect_bd_net [get_bd_pins zmod_awg_$ch/ZmodDAC_ClkIn]   [get_bd_ports ZmodDAC_ClkIn_$ch]
    connect_bd_net [get_bd_pins zmod_awg_$ch/dZmodDAC_Data]   [get_bd_ports dZmodDAC_Data_$ch]
    connect_bd_net [get_bd_pins zmod_awg_$ch/sZmodDAC_SetFS1] [get_bd_ports sZmodDAC_SetFS1_$ch]
    connect_bd_net [get_bd_pins zmod_awg_$ch/sZmodDAC_SetFS2] [get_bd_ports sZmodDAC_SetFS2_$ch]
    connect_bd_net [get_bd_pins zmod_awg_$ch/sZmodDAC_EnOut]  [get_bd_ports sZmodDAC_EnOut_$ch]
}

# -- 2026-07-04: WO_PORT_STATUS diagnostics (0x30-0x33) --------------------------
# 每個 wctrl_$ch 的 current_idx[2:0]/next_idx[2:0]/mux_sel 即時狀態，之前移植時
# 沒有接（awg_common.py 的 WO_PORT_STATUS_0..3 = 0x30-0x33，格式：
# bits[2:0]=current_idx bits[5:3]=next_idx bit[6]=mux_sel）。這次補上，用來排查
# port D（channel 3）完全沒有輸出的問題——先確認 wctrl_3 的狀態機到底有沒有真的
#跑到 ST_RUNNING。0x30-0x33 之前完全沒接過，不會跟既有暫存器衝突（已查證）。

# 2026-07-14：新增 In4=wctrl_$ch/play_pos（4-bit signed，-1=已上膛未觸發，
# 0..depth-1=真正 trigger 生效次數 mod depth，見 waveform_controller.v
# 的 play_pos 章節）。bits[10:7]，補齊 bits[31:11] 用既有的
# const_21b0（line ~701，`concat_enum_status/In3` 已經在用，重用不用
# 新建，新建會撞名——2026-07-14 上機時真的撞過一次，教訓）。格式更新：
# bits[2:0]=current_idx bits[5:3]=next_idx bit[6]=mux_sel
# bits[10:7]=play_pos（signed）。
foreach ch {0 1 2 3} {
    create_bd_cell -type ip -vlnv xilinx.com:ip:xlconcat concat_port_status_$ch
    set_property -dict [list CONFIG.NUM_PORTS {5} \
        CONFIG.IN0_WIDTH {3} CONFIG.IN1_WIDTH {3} CONFIG.IN2_WIDTH {1} \
        CONFIG.IN3_WIDTH {4} CONFIG.IN4_WIDTH {21}] \
        [get_bd_cells concat_port_status_$ch]
    connect_bd_net [get_bd_pins wctrl_$ch/current_idx] [get_bd_pins concat_port_status_$ch/In0]
    connect_bd_net [get_bd_pins wctrl_$ch/next_idx]    [get_bd_pins concat_port_status_$ch/In1]
    connect_bd_net [get_bd_pins wctrl_$ch/mux_sel]     [get_bd_pins concat_port_status_$ch/In2]
    connect_bd_net [get_bd_pins wctrl_$ch/play_pos]    [get_bd_pins concat_port_status_$ch/In3]
    connect_bd_net [get_bd_pins const_21b0/dout]       [get_bd_pins concat_port_status_$ch/In4]
}
connect_bd_net [get_bd_pins concat_port_status_0/dout] [get_bd_pins fp0/wo30_ep_datain]
connect_bd_net [get_bd_pins concat_port_status_1/dout] [get_bd_pins fp0/wo31_ep_datain]
connect_bd_net [get_bd_pins concat_port_status_2/dout] [get_bd_pins fp0/wo32_ep_datain]
connect_bd_net [get_bd_pins concat_port_status_3/dout] [get_bd_pins fp0/wo33_ep_datain]

# 2026-07-30 新增：同一份 concat_port_status_$ch/dout 再接一份給
# ddr_status_slice_$ch（縮寬到 [10:0]）→ ddr_status_cdc_$ch（dac_clk→
# sys_clk）→ aurora_reply_tx_0，讓 QT_DDR_STATUS 可跨板讀取。跟既有
# WO 0x30-33 那條路徑（無 CDC）並存、互不影響，見 NOTES.md 對應章節。
foreach ch {0 1 2 3} {
    connect_bd_net [get_bd_pins concat_port_status_$ch/dout] [get_bd_pins ddr_status_slice_$ch/Din]
    connect_bd_net [get_bd_pins ddr_status_slice_$ch/Dout] [get_bd_pins ddr_status_cdc_$ch/src_in]
    connect_bd_net [get_bd_pins ddr_status_cdc_$ch/dst_out] [get_bd_pins aurora_reply_tx_0/di_ddr_status_ch$ch]
}

# ==============================================================================
#  Save, validate, wrap
# ==============================================================================
regenerate_bd_layout
save_bd_design
catch { validate_bd_design }
save_bd_design

generate_target all [get_files awg_step16_bd.bd]
set wrapper [make_wrapper -files [get_files awg_step16_bd.bd] -top]
add_files -norecurse $wrapper
update_compile_order -fileset sources_1
set_property top awg_step16_bd_wrapper [current_fileset]
update_compile_order -fileset sources_1

puts ""
puts "AWG Step-15b BD created (forked from 15a, host-triggered enum/trigger/reserve now via packet not TI bit)"
puts "  Clock domains: sys_clk (main logic) / ddr4_ui_clk / aurora_clk"
puts "  FP write: WI 0x12(addr) + WI 0x14-0x17(data) + TI bit0 -> fp_ddr4_rw_1 -> SmartConnect S01 -> DDR4"
puts "  FP read : WI 0x12(addr) + TI bit1 -> fp_ddr4_rw_1 -> DDR4 -> WO 0x21-0x24"
puts "  Disp wr : PipeIn(type=0x13) -> fp_input -> dispatcher -> ddr_writer_0 -> SmartConnect S00 -> DDR4"
puts "  Aurora  : bidirectional (aurora_64b66b_0/1), Layer2(data)+Layer3(ctrl protocol)+tx1 arbiter"
puts "  Aurora host trigger: packet-based (host->dispatcher->local_reg_handler_0),"
puts "                    T_ENUM_START=0x1A(1-beat), T_TRIG_START=0x1B(1-beat),"
puts "                    T_RESERVE_START=0x1C(2-beat, beat1 bits 15:0=dest_id);"
puts "                    TI bit29/30/31 + WI 0x11 removed in 15b."
puts "                    T_TRIG_DELAY_CFG=0x1D(2-beat, beat1 bits 15:0=delay in sys_clk cycles)"
puts "                    WO 0x26=enum status, WO 0x27=channel_up, WO 0x28=reserve status,"
puts "                    WO 0x29=trig_delay readback"
puts "  Flash   : fpga_flash_ctrl_0 restored (STARTUPE3-based, sector 200/0xC80000, 256B page)."
puts "                    2026-07-09: fpga_flash_ctrl_0 now on okClk (was sys_clk); PI 0x80 removed."
puts "                    Write payload: BTPI0x81 -> dispatcher(type 0x14/T_FLASH_WRITE_DATA)"
puts "                    -> flash_payload_cdc_0 -> pipe_wdata/pipe_wvalid."
puts "                    Write auto-triggers on payload_complete (64 words received) -- no T_FLASH_SAVE needed."
puts "                    Erase trigger: out_flash_erase (packet 0x0F) -> trigger_cdc -> fp_flash_cfg_erase."
puts "                    Persists board_id/is_master/scale_cfg/amp_ctrl x8/calib_coef x32/trig_delay"
puts "                    (load_valid -> flash_load_valid_cdc_0 -> fan-out, see PROJECT.md sec 25)."
puts "                    WO 0x2A=flash_status, WO 0x2B=raw_magic (diagnostic)."
puts "=== create_bd.tcl DONE -- awg_step16_bd ready to build ==="
