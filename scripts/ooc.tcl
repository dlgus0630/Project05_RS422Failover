set root [file normalize [lindex $argv 0]]
set out [file normalize [lindex $argv 1]]
file mkdir $out
set_param general.maxThreads 4
read_verilog [glob $root/Project04_RS422Failover.srcs/sources_1/new/*.v]
set constraints [open $out/core_clock.xdc w]
puts $constraints {create_clock -name core_clk -period 10.000 [get_ports clk]}
close $constraints
read_xdc $out/core_clock.xdc
synth_design -top redundant_link_core -part xc7a35tcpg236-1 -mode out_of_context
opt_design
place_design
phys_opt_design
route_design
report_timing_summary -report_unconstrained -file $out/timing_summary.rpt
report_utilization -hierarchical -file $out/utilization_hierarchy.rpt
report_utilization -file $out/utilization.rpt
report_route_status -file $out/route_status.rpt
report_drc -file $out/drc.rpt
report_timing -max_paths 5 -file $out/critical_paths.rpt
