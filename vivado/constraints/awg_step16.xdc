# awg_step16.xdc
# XEM8320-AU25P: FrontPanel + DDR4 + Aurora 雙向多板同步（見 create_bd.tcl）
# Step 14.3b: 14.3 v9 基礎上移除 Aurora，排除 Aurora 干擾 dispatcher→DDR 的可能性
# Step 15a: 2026-07-05 復原 Aurora（雙向化，接新的 Layer2/3/tx1 仲裁器模組）
# Step 15b: 2026-07-07 從 15a 分支，host 觸發 enum/trigger/reserve 協定改成
# 走封包（host→dispatcher→local_reg_handler），拿掉 TI bit 29/30/31 + WI 0x11

set_property CFGBVS GND [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

set_property BITSTREAM.CONFIG.EXTMASTERCCLK_EN disable [current_design]
set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property BITSTREAM.CONFIG.SPI_FALL_EDGE YES [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 85.0 [current_design]

# ── FrontPanel USB ────────────────────────────────────────────────────
set_property PACKAGE_PIN U20 [get_ports {okHU[0]}]
set_property PACKAGE_PIN U26 [get_ports {okHU[1]}]
set_property PACKAGE_PIN T22 [get_ports {okHU[2]}]
set_property SLEW FAST [get_ports {okHU[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {okHU[*]}]

set_property PACKAGE_PIN V23 [get_ports {okUH[0]}]
set_property PACKAGE_PIN T23 [get_ports {okUH[1]}]
set_property PACKAGE_PIN U22 [get_ports {okUH[2]}]
set_property PACKAGE_PIN U25 [get_ports {okUH[3]}]
set_property PACKAGE_PIN U21 [get_ports {okUH[4]}]
set_property IOSTANDARD LVCMOS18 [get_ports {okUH[*]}]

set_property PACKAGE_PIN P26 [get_ports {okUHU[0]}]
set_property PACKAGE_PIN P25 [get_ports {okUHU[1]}]
set_property PACKAGE_PIN R26 [get_ports {okUHU[2]}]
set_property PACKAGE_PIN R25 [get_ports {okUHU[3]}]
set_property PACKAGE_PIN R23 [get_ports {okUHU[4]}]
set_property PACKAGE_PIN R22 [get_ports {okUHU[5]}]
set_property PACKAGE_PIN P21 [get_ports {okUHU[6]}]
set_property PACKAGE_PIN P20 [get_ports {okUHU[7]}]
set_property PACKAGE_PIN R21 [get_ports {okUHU[8]}]
set_property PACKAGE_PIN R20 [get_ports {okUHU[9]}]
set_property PACKAGE_PIN P23 [get_ports {okUHU[10]}]
set_property PACKAGE_PIN N23 [get_ports {okUHU[11]}]
set_property PACKAGE_PIN T25 [get_ports {okUHU[12]}]
set_property PACKAGE_PIN N24 [get_ports {okUHU[13]}]
set_property PACKAGE_PIN N22 [get_ports {okUHU[14]}]
set_property PACKAGE_PIN V26 [get_ports {okUHU[15]}]
set_property PACKAGE_PIN N19 [get_ports {okUHU[16]}]
set_property PACKAGE_PIN V21 [get_ports {okUHU[17]}]
set_property PACKAGE_PIN N21 [get_ports {okUHU[18]}]
set_property PACKAGE_PIN W20 [get_ports {okUHU[19]}]
set_property PACKAGE_PIN W26 [get_ports {okUHU[20]}]
set_property PACKAGE_PIN W19 [get_ports {okUHU[21]}]
set_property PACKAGE_PIN Y25 [get_ports {okUHU[22]}]
set_property PACKAGE_PIN Y26 [get_ports {okUHU[23]}]
set_property PACKAGE_PIN Y22 [get_ports {okUHU[24]}]
set_property PACKAGE_PIN V22 [get_ports {okUHU[25]}]
set_property PACKAGE_PIN W21 [get_ports {okUHU[26]}]
set_property PACKAGE_PIN AA23 [get_ports {okUHU[27]}]
set_property PACKAGE_PIN Y23 [get_ports {okUHU[28]}]
set_property PACKAGE_PIN AA24 [get_ports {okUHU[29]}]
set_property PACKAGE_PIN W25 [get_ports {okUHU[30]}]
set_property PACKAGE_PIN AA25 [get_ports {okUHU[31]}]
set_property SLEW FAST [get_ports {okUHU[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {okUHU[*]}]

set_property PACKAGE_PIN T19 [get_ports okAA]
set_property IOSTANDARD LVCMOS18 [get_ports okAA]

create_clock -period 9.920 -name okUH0 [get_ports {okUH[0]}]
set_input_delay -clock [get_clocks okUH0] -max -add_delay 8.000 [get_ports {okUH[*]}]
set_input_delay -clock [get_clocks okUH0] -min -add_delay 9.920 [get_ports {okUH[*]}]
set_input_delay -clock [get_clocks okUH0] -max -add_delay 7.000 [get_ports {okUHU[*]}]
set_input_delay -clock [get_clocks okUH0] -min -add_delay 2.000 [get_ports {okUHU[*]}]
set_output_delay -clock [get_clocks okUH0] -max -add_delay 2.000 [get_ports {okHU[*]}]
set_output_delay -clock [get_clocks okUH0] -min -add_delay -0.500 [get_ports {okHU[*]}]
set_output_delay -clock [get_clocks okUH0] -max -add_delay 2.000 [get_ports {okUHU[*]}]
set_output_delay -clock [get_clocks okUH0] -min -add_delay -0.500 [get_ports {okUHU[*]}]

# ── System clock (100 MHz LVDS) ───────────────────────────────────────
create_clock -period 10.000 -name sys_clk [get_ports sys_clk_p]
set_property IOSTANDARD LVDS [get_ports sys_clk_p]
set_property IOSTANDARD LVDS [get_ports sys_clk_n]
set_property PACKAGE_PIN T24 [get_ports sys_clk_p]
set_property PACKAGE_PIN U24 [get_ports sys_clk_n]

set_clock_groups -asynchronous -group [get_clocks sys_clk] -group [get_clocks okUH0]

# ── DDR4 reference clock (100 MHz via board interface fixed_ddr4_100mhz) ─
create_clock -period 9.996 -name ddr4_refclk [get_ports ddr4_sys_clk_clk_p]

set_clock_groups -name async_ddr4_sys -asynchronous \
    -group [get_clocks -include_generated_clocks sys_clk] \
    -group [get_clocks -include_generated_clocks ddr4_refclk]

# ══════════════════════════════════════════════════════════════════════
# Step 14.3b playback port: SYZYGY / ZmodAWG DAC pins (ported from awg-test-step-14)
#   S0->SetFS1 S2->SetFS2 S4->Reset S6->SCLK S8->SDIO S10->CS S12->EnOut
#   P2C_CLKP->ClkIO  C2P_CLKP->ClkIn
# ══════════════════════════════════════════════════════════════════════

# ── SYZYGY PORTA — ZmodAWG ch0 (Bank 66, LVCMOS18) ───────────────────
set_property PACKAGE_PIN L18 [get_ports sZmodDAC_SetFS1_0]
set_property PACKAGE_PIN K18 [get_ports sZmodDAC_SetFS2_0]
set_property PACKAGE_PIN M20 [get_ports sZmodDAC_Reset_0]
set_property PACKAGE_PIN M21 [get_ports sZmodDAC_SCLK_0]
set_property PACKAGE_PIN J19 [get_ports sZmodDAC_SDIO_0]
set_property PACKAGE_PIN J20 [get_ports sZmodDAC_CS_0]
set_property PACKAGE_PIN L22 [get_ports sZmodDAC_EnOut_0]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SetFS1_0]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SetFS2_0]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_Reset_0]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SCLK_0]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SDIO_0]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_CS_0]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_EnOut_0]
set_property PACKAGE_PIN M25 [get_ports {dZmodDAC_Data_0[13]}]
set_property PACKAGE_PIN M26 [get_ports {dZmodDAC_Data_0[12]}]
set_property PACKAGE_PIN L24 [get_ports {dZmodDAC_Data_0[11]}]
set_property PACKAGE_PIN L25 [get_ports {dZmodDAC_Data_0[10]}]
set_property PACKAGE_PIN K25 [get_ports {dZmodDAC_Data_0[9]}]
set_property PACKAGE_PIN K26 [get_ports {dZmodDAC_Data_0[8]}]
set_property PACKAGE_PIN K22 [get_ports {dZmodDAC_Data_0[7]}]
set_property PACKAGE_PIN K23 [get_ports {dZmodDAC_Data_0[6]}]
set_property PACKAGE_PIN L19 [get_ports {dZmodDAC_Data_0[5]}]
set_property PACKAGE_PIN H24 [get_ports {dZmodDAC_Data_0[4]}]
set_property PACKAGE_PIN H23 [get_ports {dZmodDAC_Data_0[3]}]
set_property PACKAGE_PIN L23 [get_ports {dZmodDAC_Data_0[2]}]
set_property PACKAGE_PIN J21 [get_ports {dZmodDAC_Data_0[1]}]
set_property PACKAGE_PIN M19 [get_ports {dZmodDAC_Data_0[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {dZmodDAC_Data_0[*]}]
set_property SLEW SLOW [get_ports {dZmodDAC_Data_0[*]}]
set_property PACKAGE_PIN J23 [get_ports ZmodDAC_ClkIO_0]
set_property PACKAGE_PIN H26 [get_ports ZmodDAC_ClkIn_0]
set_property IOSTANDARD LVCMOS18 [get_ports ZmodDAC_ClkIO_0]
set_property IOSTANDARD LVCMOS18 [get_ports ZmodDAC_ClkIn_0]
set_property SLEW FAST [get_ports ZmodDAC_ClkIO_0]
set_property SLEW FAST [get_ports ZmodDAC_ClkIn_0]

# ── SYZYGY PORTB — ZmodAWG ch1 (Bank 66/67, LVCMOS18) ────────────────
set_property PACKAGE_PIN A22 [get_ports sZmodDAC_SetFS1_1]
set_property PACKAGE_PIN A23 [get_ports sZmodDAC_SetFS2_1]
set_property PACKAGE_PIN E21 [get_ports sZmodDAC_Reset_1]
set_property PACKAGE_PIN D21 [get_ports sZmodDAC_SCLK_1]
set_property PACKAGE_PIN E25 [get_ports sZmodDAC_SDIO_1]
set_property PACKAGE_PIN E26 [get_ports sZmodDAC_CS_1]
set_property PACKAGE_PIN F23 [get_ports sZmodDAC_EnOut_1]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SetFS1_1]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SetFS2_1]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_Reset_1]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SCLK_1]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SDIO_1]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_CS_1]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_EnOut_1]
set_property PACKAGE_PIN A24 [get_ports {dZmodDAC_Data_1[13]}]
set_property PACKAGE_PIN A25 [get_ports {dZmodDAC_Data_1[12]}]
set_property PACKAGE_PIN D24 [get_ports {dZmodDAC_Data_1[11]}]
set_property PACKAGE_PIN D25 [get_ports {dZmodDAC_Data_1[10]}]
set_property PACKAGE_PIN C23 [get_ports {dZmodDAC_Data_1[9]}]
set_property PACKAGE_PIN B24 [get_ports {dZmodDAC_Data_1[8]}]
set_property PACKAGE_PIN C21 [get_ports {dZmodDAC_Data_1[7]}]
set_property PACKAGE_PIN B21 [get_ports {dZmodDAC_Data_1[6]}]
set_property PACKAGE_PIN C26 [get_ports {dZmodDAC_Data_1[5]}]
set_property PACKAGE_PIN D26 [get_ports {dZmodDAC_Data_1[4]}]
set_property PACKAGE_PIN D23 [get_ports {dZmodDAC_Data_1[3]}]
set_property PACKAGE_PIN E23 [get_ports {dZmodDAC_Data_1[2]}]
set_property PACKAGE_PIN B26 [get_ports {dZmodDAC_Data_1[1]}]
set_property PACKAGE_PIN B25 [get_ports {dZmodDAC_Data_1[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {dZmodDAC_Data_1[*]}]
set_property SLEW SLOW [get_ports {dZmodDAC_Data_1[*]}]
set_property PACKAGE_PIN G24 [get_ports ZmodDAC_ClkIO_1]
set_property PACKAGE_PIN H21 [get_ports ZmodDAC_ClkIn_1]
set_property IOSTANDARD LVCMOS18 [get_ports ZmodDAC_ClkIO_1]
set_property IOSTANDARD LVCMOS18 [get_ports ZmodDAC_ClkIn_1]
set_property SLEW FAST [get_ports ZmodDAC_ClkIO_1]
set_property SLEW FAST [get_ports ZmodDAC_ClkIn_1]

# ── SYZYGY PORTC — ZmodAWG ch2 (Bank 67, LVCMOS18) ───────────────────
set_property PACKAGE_PIN F20 [get_ports sZmodDAC_SetFS1_2]
set_property PACKAGE_PIN E20 [get_ports sZmodDAC_SetFS2_2]
set_property PACKAGE_PIN H18 [get_ports sZmodDAC_Reset_2]
set_property PACKAGE_PIN H19 [get_ports sZmodDAC_SCLK_2]
set_property PACKAGE_PIN F18 [get_ports sZmodDAC_SDIO_2]
set_property PACKAGE_PIN F19 [get_ports sZmodDAC_CS_2]
set_property PACKAGE_PIN E16 [get_ports sZmodDAC_EnOut_2]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SetFS1_2]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SetFS2_2]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_Reset_2]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SCLK_2]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SDIO_2]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_CS_2]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_EnOut_2]
set_property PACKAGE_PIN C18 [get_ports {dZmodDAC_Data_2[13]}]
set_property PACKAGE_PIN C19 [get_ports {dZmodDAC_Data_2[12]}]
set_property PACKAGE_PIN H17 [get_ports {dZmodDAC_Data_2[11]}]
set_property PACKAGE_PIN G17 [get_ports {dZmodDAC_Data_2[10]}]
set_property PACKAGE_PIN A17 [get_ports {dZmodDAC_Data_2[9]}]
set_property PACKAGE_PIN A18 [get_ports {dZmodDAC_Data_2[8]}]
set_property PACKAGE_PIN B15 [get_ports {dZmodDAC_Data_2[7]}]
set_property PACKAGE_PIN A15 [get_ports {dZmodDAC_Data_2[6]}]
set_property PACKAGE_PIN B19 [get_ports {dZmodDAC_Data_2[5]}]
set_property PACKAGE_PIN A19 [get_ports {dZmodDAC_Data_2[4]}]
set_property PACKAGE_PIN D19 [get_ports {dZmodDAC_Data_2[3]}]
set_property PACKAGE_PIN E17 [get_ports {dZmodDAC_Data_2[2]}]
set_property PACKAGE_PIN H16 [get_ports {dZmodDAC_Data_2[1]}]
set_property PACKAGE_PIN D16 [get_ports {dZmodDAC_Data_2[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {dZmodDAC_Data_2[*]}]
set_property SLEW SLOW [get_ports {dZmodDAC_Data_2[*]}]
set_property PACKAGE_PIN E18 [get_ports ZmodDAC_ClkIO_2]
set_property PACKAGE_PIN C17 [get_ports ZmodDAC_ClkIn_2]
set_property IOSTANDARD LVCMOS18 [get_ports ZmodDAC_ClkIO_2]
set_property IOSTANDARD LVCMOS18 [get_ports ZmodDAC_ClkIn_2]
set_property SLEW FAST [get_ports ZmodDAC_ClkIO_2]
set_property SLEW FAST [get_ports ZmodDAC_ClkIn_2]

# ── SYZYGY PORTD — ZmodAWG ch3 (Banks 84/87, LVCMOS18, VIO2) ─────────
set_property PACKAGE_PIN J12 [get_ports sZmodDAC_SetFS1_3]
set_property PACKAGE_PIN H12 [get_ports sZmodDAC_SetFS2_3]
set_property PACKAGE_PIN Y13 [get_ports sZmodDAC_Reset_3]
set_property PACKAGE_PIN AA13 [get_ports sZmodDAC_SCLK_3]
set_property PACKAGE_PIN J13 [get_ports sZmodDAC_SDIO_3]
set_property PACKAGE_PIN H13 [get_ports sZmodDAC_CS_3]
set_property PACKAGE_PIN AE13 [get_ports sZmodDAC_EnOut_3]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SetFS1_3]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SetFS2_3]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_Reset_3]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SCLK_3]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_SDIO_3]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_CS_3]
set_property IOSTANDARD LVCMOS18 [get_ports sZmodDAC_EnOut_3]
set_property PACKAGE_PIN W12 [get_ports {dZmodDAC_Data_3[13]}]
set_property PACKAGE_PIN W13 [get_ports {dZmodDAC_Data_3[12]}]
set_property PACKAGE_PIN H14 [get_ports {dZmodDAC_Data_3[11]}]
set_property PACKAGE_PIN G14 [get_ports {dZmodDAC_Data_3[10]}]
set_property PACKAGE_PIN AF14 [get_ports {dZmodDAC_Data_3[9]}]
set_property PACKAGE_PIN AF15 [get_ports {dZmodDAC_Data_3[8]}]
set_property PACKAGE_PIN AC13 [get_ports {dZmodDAC_Data_3[7]}]
set_property PACKAGE_PIN AC14 [get_ports {dZmodDAC_Data_3[6]}]
set_property PACKAGE_PIN J15 [get_ports {dZmodDAC_Data_3[5]}]
set_property PACKAGE_PIN J14 [get_ports {dZmodDAC_Data_3[4]}]
set_property PACKAGE_PIN AB16 [get_ports {dZmodDAC_Data_3[3]}]
set_property PACKAGE_PIN AF13 [get_ports {dZmodDAC_Data_3[2]}]
set_property PACKAGE_PIN W14 [get_ports {dZmodDAC_Data_3[1]}]
set_property PACKAGE_PIN Y15 [get_ports {dZmodDAC_Data_3[0]}]
set_property IOSTANDARD LVCMOS18 [get_ports {dZmodDAC_Data_3[*]}]
set_property SLEW SLOW [get_ports {dZmodDAC_Data_3[*]}]
set_property PACKAGE_PIN AA14 [get_ports ZmodDAC_ClkIO_3]
set_property PACKAGE_PIN AD13 [get_ports ZmodDAC_ClkIn_3]
set_property IOSTANDARD LVCMOS18 [get_ports ZmodDAC_ClkIO_3]
set_property IOSTANDARD LVCMOS18 [get_ports ZmodDAC_ClkIn_3]
set_property SLEW FAST [get_ports ZmodDAC_ClkIO_3]
set_property SLEW FAST [get_ports ZmodDAC_ClkIn_3]

# ── ZmodAWG SPI control / DAC data false paths + IOB placement ────────
set_false_path -to [get_ports sZmodDAC_Reset_*]
set_false_path -to [get_ports sZmodDAC_EnOut_*]
set_false_path -from [get_ports sZmodDAC_SDIO_*]
set_false_path -to [get_ports sZmodDAC_SDIO_*]
set_false_path -to [get_ports sZmodDAC_SetFS1_*]
set_false_path -to [get_ports sZmodDAC_SetFS2_*]
set_property IOB TRUE [get_cells -hier -filter {NAME =~ *InstDataODDR*}]
set_false_path -to [get_ports {dZmodDAC_Data_0[*]}]
set_false_path -to [get_ports {dZmodDAC_Data_1[*]}]
set_false_path -to [get_ports {dZmodDAC_Data_2[*]}]
set_false_path -to [get_ports {dZmodDAC_Data_3[*]}]
set_false_path -to [get_ports ZmodDAC_ClkIO_0]
set_false_path -to [get_ports ZmodDAC_ClkIn_0]
set_false_path -to [get_ports ZmodDAC_ClkIO_1]
set_false_path -to [get_ports ZmodDAC_ClkIn_1]
set_false_path -to [get_ports ZmodDAC_ClkIO_2]
set_false_path -to [get_ports ZmodDAC_ClkIn_2]
set_false_path -to [get_ports ZmodDAC_ClkIO_3]
set_false_path -to [get_ports ZmodDAC_ClkIn_3]

# ── User LEDs ─────────────────────────────────────────────────────────
set_property PACKAGE_PIN G19 [get_ports {led[0]}]
set_property PACKAGE_PIN B16 [get_ports {led[1]}]
set_property PACKAGE_PIN F22 [get_ports {led[2]}]
set_property PACKAGE_PIN E22 [get_ports {led[3]}]
set_property PACKAGE_PIN M24 [get_ports {led[4]}]
set_property PACKAGE_PIN G22 [get_ports {led[5]}]
set_property IOSTANDARD LVCMOS18 [get_ports {led[*]}]
set_false_path -to [get_ports {led[*]}]

# ── Timing false paths ────────────────────────────────────────────────
set_false_path -from [get_clocks mmcm0_clk0] -to [get_clocks okUH0]
set_false_path -from [get_clocks okUH0] -to [get_clocks mmcm0_clk0]

set_false_path -from [get_clocks -include_generated_clocks sys_clk] -to [get_clocks mmcm0_clk0]
set_false_path -from [get_clocks mmcm0_clk0] -to [get_clocks -include_generated_clocks sys_clk]

set_false_path -from [get_clocks mmcm_clkout0] -to [get_clocks mmcm0_clk0]
set_false_path -from [get_clocks mmcm0_clk0] -to [get_clocks mmcm_clkout0]

# ── Clock routing ─────────────────────────────────────────────────────
# 2026-07-06 修正（15a）：instance 名稱從 14.3b 沿用時忘記改，一直是
# awg_step14_3b_bd_i（對不上實際的 awg_step15a_bd_i，get_nets 找不到東西，
# 這三條約束一直是靜默失效的空操作），改成正確 instance 名稱後解決。
# 2026-07-07（15b）：從 15a 分支，這裡直接沿用正確作法，改成
# awg_step16_bd_i（跟這個專案的 create_bd_design "awg_step16_bd" 一致），
# 不要再犯同樣的沿用疏漏。
set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets awg_step16_bd_i/clk_wiz_0/inst/clkin1_ibufds/O]

set_property LOC MMCM_X0Y2 [get_cells -hierarchical -filter {NAME =~ *fp0*okHI*mmcm0}]

set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets awg_step16_bd_i/fp0/inst/okHI/hi_clk_bufg/O]
set_property CLOCK_DEDICATED_ROUTE BACKBONE [get_nets -quiet {awg_step16_bd_i/fp0/inst/okHI/okUH0_ibufg_BUFGCE}]

# ── Aurora 64B/66B — SFP1+SFP2 (Bank 226, Quad X0Y2, Lane X0Y8/X0Y9) ─
# 2026-07-05 (step 15a)：復原（14.3b 為排除 Aurora 干擾整段註解，pin/clock
# 定義不變；C_START_QUAD/C_START_LANE 已在 create_bd.tcl 的 aurora_64b66b_0/1
# CONFIG 指定，GT 序列腳位不需要這裡另外下 PACKAGE_PIN）
set_property PACKAGE_PIN P6 [get_ports aurora_refclk_n]
set_property PACKAGE_PIN P7 [get_ports aurora_refclk_p]
create_clock -period 8.000 -name aurora_refclk [get_ports aurora_refclk_p]

# 2026-08-05 新增：SFP TX_DISABLE（TDIS，active-high，拉低才會致能
# 雷射）——這個設計從一開始就沒有驅動過這兩根，過去用 DAC 銅纜測試
# 從沒踩到（銅纜不需要雷射、不受 TX_DISABLE 影響），換成真正的光纖
# SFP 才發現 Aurora 完全連不上（channel_up 全 FAIL）。腳位查 Opal
# Kelly 官方文件確認：SFP1/SFP2 控制腳位透過電平轉換器接到 FPGA
# Bank 87，電壓由 SYZYGY Port D 週邊模組決定（這個專案 Port D 接
# ZmodAWG ch3，既有 216-252 行 SYZYGY PORTD 接線確認 Bank 87 現在是
# LVCMOS18，且腳位不衝突），不是板子上硬接地常態致能。見 NOTES.md
# 2026-08-05「換 SFP+光纖後 Aurora 連不上」章節。
set_property PACKAGE_PIN C13 [get_ports TDIS_1]
set_property PACKAGE_PIN F13 [get_ports TDIS_2]
set_property IOSTANDARD LVCMOS18 [get_ports TDIS_1]
set_property IOSTANDARD LVCMOS18 [get_ports TDIS_2]

# ── ext DAC clock — MGTREFCLK1_226 (Bank 226, 跟 Aurora 的 MGTREFCLK0
#    是同 bank 不同組，不衝突) ────────────────────────────────────────
# 2026-07-11 新增，見 PROJECT.md「時脈路徑設計」章節。J19/J20（REFCLK+/-）
# -> M7/M6 -> MGTREFCLK1P/N_226。外部 Si5332-6eX-EVB 輸出 100MHz。
# 這裡只約束「進來的參考時脈」本身（100MHz，period=10ns），不約束
# ext_clk_out（IBUFDS_GTE4 ODIV2 + BUFG_GT 疊加後的實際頻率還沒上機
# 驗證過，見 rtl/ext_dac_clk_ibuf.v 註解，不能用猜的頻率下 create_clock）。
set_property PACKAGE_PIN M6 [get_ports ext_dac_refclk_n]
set_property PACKAGE_PIN M7 [get_ports ext_dac_refclk_p]
create_clock -period 10.000 -name ext_dac_refclk [get_ports ext_dac_refclk_p]

# 2026-07-13：clk_wiz_ext_0 的輸入定義（見 PROJECT.md「時脈路徑設計」階段 A）。
# PRIM_SOURCE=No_buffer 的 clk_wiz 輸入已經是 ext_dac_clk_ibuf_0 內
# BUFG_GT 的輸出（global buffer），IP 本身不會像 Differential_clock_
# capable_pin 模式自動產生輸入腳位約束，必須自己明確定義這個 clock。
# 實測 ext_clk_out ≈99.98MHz（1:1 直通，量測誤差來自軟體讀值間隔非硬體
# 時脈本身），用標稱 100MHz/10ns 定義，跟其他時脈統一用整數週期慣例。
# ⚠️ 這個 clock 定義必須放在下面所有 set_clock_groups 之前——
# awg-test-step-7 曾在同一種 BUFG_GT->No_buffer clk_wiz 場景因為順序寫
# 反，導致 TNS -14313ns 假失敗（見 feedback 記錄），修正順序後才變回
# 0 violations。
#
# 2026-07-13 上機測試發現 Critical Warning：用 create_clock（獨立 primary
# clock）約束 BUFG_GT（Vivado 認定的 Clock Modifying Block）輸出，Vivado
# 報「A primary clock ext_clk_out is created on the output pin or net
# ... of a Clock Modifying Block」（兩次，ext_dac_clk_ibuf_0/ext_clk_out
# 與 .../inst/ext_clk_out 是同一顆 net 的兩個 hierarchy 別名，都被
# -hierarchical -filter 抓到）。改成 create_generated_clock，明確關聯回
# 真正的來源時脈 ext_dac_refclk（-divide_by 1 對應實測 1:1 直通），而不是
# 憑空定義一個無父系的獨立 clock。順帶也讓 ext_clk_out 正式成為
# ext_dac_refclk 衍生出來的 generated clock，理論上能讓下面 async_extclk
# 的 -include_generated_clocks 更可靠地追溯到 clk_wiz_ext_0 的輸出
# （原本 60% 信心的疑慮，這個修正後待驗證）。
create_generated_clock -name ext_clk_out -source [get_ports ext_dac_refclk_p] -divide_by 1 \
    [get_pins -hierarchical -filter {NAME =~ *ext_dac_clk_ibuf_0*ext_clk_out}]

# ── Aurora clock groups (include ddr4_refclk) ─────────────────────────
set_clock_groups -name async_aurora -asynchronous \
    -group [get_clocks -include_generated_clocks aurora_refclk] \
    -group [get_clocks -include_generated_clocks sys_clk] \
    -group [get_clocks -include_generated_clocks ddr4_refclk]

set_clock_groups -name async_aurora_usr -asynchronous \
    -group [get_clocks -filter {NAME =~ *txoutclk*}] \
    -group [get_clocks -include_generated_clocks okUH0] \
    -group [get_clocks -include_generated_clocks sys_clk] \
    -group [get_clocks -include_generated_clocks ddr4_refclk]

# 2026-07-05 (step 15a)：舊版 aurora_packet_tx_0 的手工 toggle-sync 多位元
# latching CDC（cw_sel_lat/cw_data_lat/amp_ch_sel_lat/amp_val_lat）不復原
# ——那個模組已經被新的 aurora_data_channel/aurora_ctrl_channel 取代，新模組
# 的 sys_clk<->aurora_clk 交界全部走正規 CDC（async_fifo_aurora_rx/tx 這兩個
# FIFO、trigger_pulse 走 xpm_cdc_pulse 的 au_trigger_cdc_0），沒有對應的手工
# latch 暫存器需要這種 exception constraint。

# ── ext clock domain (clk_wiz_ext_0，2026-07-13 階段 A) ────────────────
# Si5332 是完全獨立的外部振盪器來源，跟 sys_clk/ddr4_refclk/aurora_refclk/
# okUH0 都沒有共同來源，必須明確標成 async，否則 Vivado 預設會把所有 clock
# 當成潛在同步、跑全域 inter-clock timing，可能出現 near-synchronous 假失敗
# （跟 awg-test-step-7 的 ext_100 vs okUH0 是同一類問題）。
#
# ⚠️ 2026-07-13 階段 B 上機實測踩到：第一版漏列了 mmcm0_clk0/mmcm_clkout0
# （見上面「Clock routing」章節，mmcm0_clk0 是 fp0/okHI 內部 USB host
# interface 自己的 MMCM，mmcm_clkout0 是另一個既有內部時脈，兩者都已經
# 用 set_false_path 互相排除，但都沒被列進這裡）。dac_clk_mux_0/I1 接上
# clk_wiz_ext_0 後，下游 wctrl_$ch 對 Vivado 來說變成可能被兩個不同時脈
# 驅動，implementation 卡在 route_design 的 rip-up-and-reroute，
# runme.log 出現「3704 pins with tight setup and hold constraints」、
# WNS=-3.534/TNS=-5667.041（跟 awg-test-step-7 當年一樣的災難性假失敗）。
# 補上 mmcm0_clk0/mmcm_clkout0 這兩組後才解決。
#
# ⚠️ 信心中等（~60%，這條還沒解除）：`-include_generated_clocks
# ext_clk_out` 理論上該自動涵蓋 clk_wiz_ext_0 的 CLKOUT1/2（它們是從
# ext_clk_out 衍生的 generated clock），但 step-7 的歷史記錄顯示
# No_buffer 模式下這個自動追溯不一定可靠。建完後請跑
# report_clock_networks 或看 timing summary，確認 clk_wiz_ext_0 的兩個
# 輸出有沒有被正確歸類、WNS/TNS 沒有異常。如果沒有，把下面註解掉的
# fallback 那行加回來（額外用 -filter 明確抓 *clk_wiz_ext_0*）。
set_clock_groups -name async_extclk -asynchronous \
    -group [get_clocks -include_generated_clocks ext_clk_out] \
    -group [get_clocks -include_generated_clocks sys_clk] \
    -group [get_clocks -include_generated_clocks ddr4_refclk] \
    -group [get_clocks -include_generated_clocks aurora_refclk] \
    -group [get_clocks mmcm0_clk0] \
    -group [get_clocks mmcm_clkout0] \
    -group [get_clocks okUH0]
# Fallback（如果上面那組沒抓到 clk_wiz_ext_0 輸出，改用這個版本 -- 注意
# 這個版本把 ext_clk_out 跟 clk_wiz_ext_0 輸出放在同一個 -group 裡，不能
# 分開，否則會把同一顆 MMCM 的輸入/輸出誤標成互相 async）：
# set_clock_groups -name async_extclk -asynchronous \
#     -group [concat [get_clocks ext_clk_out] [get_clocks -filter {NAME =~ *clk_wiz_ext_0*}]] \
#     -group [get_clocks -include_generated_clocks sys_clk] \
#     -group [get_clocks -include_generated_clocks ddr4_refclk] \
#     -group [get_clocks -include_generated_clocks aurora_refclk] \
#     -group [get_clocks mmcm0_clk0] \
#     -group [get_clocks mmcm_clkout0] \
#     -group [get_clocks okUH0]

# ── dac_clk_mux_0/dac_90_clk_mux_0 mux 輸入互斥關係（2026-07-14）────────
# 參考同系列舊專案 eclypse-z7-awg-dma 的 my_clk_mux.v（同樣是 BUFGMUX 家族，
# 跟本專案 dac_clk_mux.v 的 BUFGMUX_CTRL 同一類）在 timing_late_v2.xdc 的
# 寫法：clk_wiz_0 的 clk_100/clk_100_90（dac_clk_mux_0/dac_90_clk_mux_0 的
# I0）跟 clk_wiz_ext_0 的 clk_ext_100/clk_ext_100_90（I1）是同一個
# BUFGMUX_CTRL 的兩個輸入，任何時刻只有其中一個真正驅動下游（S 選擇），
# 用 -physically_exclusive 比上面的 -asynchronous 更精確描述這個關係
# （-asynchronous 只表示「沒有固定相位關係、可能同時並存」，但這裡兩者
# 其實是「不可能同時並存」，被同一個 mux 保證互斥）。跟上面 async_extclk
# 的 sys_clk/ext_clk_out group 有重疊（同一對 clock 被兩種不同 exception
# 各宣告一次）——eclypse-z7 的先例本身也是這樣疊加宣告且已驗證可行，
# 沿用同一模式，不特地把 sys_clk 從 async_extclk 裡摘出來。
set_clock_groups -name physexcl_dac_clk_mux -physically_exclusive \
    -group [get_clocks -include_generated_clocks sys_clk] \
    -group [get_clocks -include_generated_clocks ext_clk_out]
