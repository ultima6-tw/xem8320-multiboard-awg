# build.tcl -- Rebuild awg_step16 bitstream (incremental)
# Usage (in Vivado TCL console):
#   source {C:/path/to/awg-test-step-16/vivado/build.tcl}

set PROJ_DIR {D:/Vivado/awg_step16}

if {[catch {current_project}]} {
    open_project $PROJ_DIR/awg_step16.xpr
}

# catch: on a fresh project the run hasn't executed yet, so reset fails -- safe to ignore
catch { reset_run awg_step16_bd_fp_input_0_0_synth_1 }
catch { reset_run awg_step16_bd_fpga_flash_ctrl_0_0_synth_1 }
# 2026-07-15：ddr_writer_0/ddr_writer_ila_0 有自己專屬的 OOC synth run，
# 沒重設會用到修改前的舊 netlist，跟改過的 RTL/probe 對不上，implementation
# 階段報 LUT 缺接腳的錯（見 PROJECT.md「T_WAVEFORM_STREAM」章節這次踩的坑）
catch { reset_run awg_step16_bd_ddr_writer_0_0_synth_1 }
catch { reset_run awg_step16_bd_ddr_writer_ila_0_0_synth_1 }
catch { reset_run synth_1 }
catch { reset_run impl_1 }

launch_runs impl_1 -to_step write_bitstream -jobs 8
wait_on_run impl_1

set status [get_property STATUS [get_runs impl_1]]
puts "impl_1 STATUS: $status"
if {$status ne "write_bitstream Complete!"} {
    error "BUILD FAILED: $status"
}
puts "BUILD SUCCESS: $PROJ_DIR/awg_step16.runs/impl_1/awg_step16_bd_wrapper.bit"
