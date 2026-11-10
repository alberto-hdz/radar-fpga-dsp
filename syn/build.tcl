# Non-project Vivado build: synthesis + implementation + reports (no bitstream).
# Run from the syn/ folder:  vivado -mode batch -source build.tcl

set part   xc7a200tfbg484-1
set top    radar_top
set out    out
set period 5.0

file mkdir $out

read_vhdl -vhdl2008 {
  ../rtl/mf_coef_pkg.vhd
  ../rtl/mf_fir.vhd
  ../rtl/mag_sq.vhd
  ../rtl/cfar_ca.vhd
  ../rtl/radar_top.vhd
}
read_xdc radar.xdc

# Synthesis
synth_design -top $top -part $part
report_utilization    -file $out/util_synth.rpt
report_timing_summary -file $out/timing_synth.rpt

# Implementation
opt_design
place_design
phys_opt_design
route_design

# Reports
report_utilization               -file $out/util.rpt
report_utilization -hierarchical -file $out/util_hier.rpt
report_timing_summary -max_paths 10 -file $out/timing.rpt
write_checkpoint -force $out/post_route.dcp

set wns [get_property SLACK [get_timing_paths -setup -max_paths 1]]
puts "=============================================="
puts [format "WNS = %.3f ns  ->  Fmax ~ %.1f MHz" $wns [expr {1000.0 / ($period - $wns)}]]
puts "=============================================="
