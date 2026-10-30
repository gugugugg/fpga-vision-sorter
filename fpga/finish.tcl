if {[catch {
set board_start_step opt_route
set alRun phy_1
set ADCList {"inputs/contest_board_pins.adc"}
set IpADCList { ddr2 vendor_ip/ddr2/src/adc/ddr2.adc }
set IpSDCList { ddr2 vendor_ip/ddr2/src/sdc/ddr2.sdc w128_d512_fifo vendor_ip/w128_d512_fifo/w128_d512_fifo.tcl }
set SDCList {"contest_board_top.sdc"}
set area_option -packarea
set arr_filter false
set device_name ph1_35p.db
set drHoldFix on
set end_step bitgen
set package_name PH1P35MDG324
set prj_name contest_board_system
set run_type phy
set speed 3
if {![info exists board_start_step]} {set board_start_step read_design}
set start_step $board_start_step
set top_model_name contest_board_top

puts "TD_PROJECT=contest_board_system.al"
# TD 6.2.178840 repeatedly exits 0xC0000409 in report_clock_utilization
# after placement. Omit only these optional diagnostic reports locally;
# keep place, route/fix_hold, clock/check_timing, area and bitgen unchanged.
set flow_file [open $::env(TD_ROOT)/scripts/DefaultFlow.tcl r]
set board_flow [read $flow_file]
close $flow_file
foreach diagnostic {place route} {
    set broken_command "report_clock_utilization -file ${diagnostic}_clock_utilization.txt"
    if {[string first $broken_command $board_flow] < 0} {
        error "Default flow changed: expected diagnostic command missing"
    }
    set board_flow [string map [list $broken_command "puts {Optional clock utilization report omitted: TD 6.2 crash}"] $board_flow]
}


set checked_route {
  set_param route fix_hold high
  route
  update_timing -mode final
  report_timing_status -file profile_routed.ts
  report_timing_summary -file profile_routed.timing
  report_area -io_info -file profile_routed.area
  export_db profile_routed.db
  report_timing_path -from [get_clocks {MIPI_RX_BYTE_CLK}] -to [get_clocks {MIPI_RX_BYTE_CLK}] -ep_num 10 -path_num 1 -file mipi_byte_domain.rpt
  report_timing_path -from [get_clocks {pll_clk_100m}] -to [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}] -ep_num 200 -path_num 1 -file ddr_calibration_crossing.rpt
  report_timing_path -from [get_clocks {u_ph1p35_324_ddr_wrapper/u_ddr2/usr_clk}] -to [get_clocks {pll_clk_100m}] -ep_num 200 -path_num 1 -file ddr_calibration_reverse.rpt
  set timing_file [open profile_routed.ts r]
  set timing_text [read $timing_file]
  close $timing_file
  foreach metric {SWNS STNS HWNS HTNS} {
    if {![regexp [format {%s:\s*([-+0-9.]+)ns} $metric] $timing_text _ value] || $value < 0} {
      error "Profile final timing failed: $metric; checkpoint retained; bitgen blocked"
    }
  }
}
set route_command "\n  route\n"
if {[string first $route_command $board_flow] < 0} {error "Native route command missing"}
set board_flow [string map [list $route_command "\n$checked_route\n"] $board_flow]
eval $board_flow
puts PROFILE_FULL_FLOW_FINISHED
close_project

} message]} {puts stderr $message;exit 1}
exit
