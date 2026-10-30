if {[catch {
set bkaADCList {inputs/ddr_calibration_locations.adc inputs/ddr_hold_receiver.adc inputs/ui_timing_locations.adc inputs/capture_ddr_locations.adc}
set alRun phy_1
set ADCList {"inputs/contest_board_pins.adc"}
set IpADCList { ddr2 vendor_ip/ddr2/src/adc/ddr2.adc }
set IpSDCList { ddr2 vendor_ip/ddr2/src/sdc/ddr2.sdc w128_d512_fifo vendor_ip/w128_d512_fifo/w128_d512_fifo.tcl }
set SDCList {"contest_board_top.sdc"}
set area_option -packarea
set arr_filter false
set device_name ph1_35p.db
set drHoldFix on
set end_step opt_place
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

set board_flow [string map [list {  elaborate -top $top_model_name} {  elaborate -top $top_model_name

  set check_file [open profile_instances.txt w]
  set selected [get_cells -hier -nowarn {*u_menu_overlay}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_menu_overlay} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_menu_overlay}}
  set selected [get_cells -hier -nowarn {*u_color_shading}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_color_shading} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_color_shading}}
  set selected [get_cells -hier -nowarn {*u_hs_fabric_sync}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_hs_fabric_sync} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_hs_fabric_sync}}
  set selected [get_cells -hier -nowarn {*u_target}]
  puts $check_file [format {%s expected=0 actual=%s} {*u_target} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *u_target}}
  set selected [get_cells -hier -nowarn {*u_inspection}]
  puts $check_file [format {%s expected=0 actual=%s} {*u_inspection} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *u_inspection}}
  set selected [get_cells -hier -nowarn {*blobs}]
  puts $check_file [format {%s expected=0 actual=%s} {*blobs} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *blobs}}
  set selected [get_cells -hier -nowarn {*hull}]
  puts $check_file [format {%s expected=0 actual=%s} {*hull} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *hull}}
  set selected [get_cells -hier -nowarn {*u_neighbourhood}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_neighbourhood} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_neighbourhood}}
  set selected [get_cells -hier -nowarn {*u_adaptive}]
  puts $check_file [format {%s expected=0 actual=%s} {*u_adaptive} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *u_adaptive}}
  set selected [get_cells -hier -nowarn {*u_equalizer}]
  puts $check_file [format {%s expected=0 actual=%s} {*u_equalizer} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *u_equalizer}}
  set selected [get_cells -hier -nowarn {*u_eva_map}]
  puts $check_file [format {%s expected=0 actual=%s} {*u_eva_map} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *u_eva_map}}
  set selected [get_cells -hier -nowarn {*u_cartoon}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_cartoon} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_cartoon}}
  set selected [get_cells -hier -nowarn {*u_sobel}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_sobel} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_sobel}}
  set selected [get_cells -hier -nowarn {g_cartoon_history$u_store}]
  puts $check_file [format {%s expected=1 actual=%s} {g_cartoon_history$u_store} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: g_cartoon_history$u_store}}
  set selected [get_cells -hier -nowarn {*u_motion}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_motion} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_motion}}
  set selected [get_cells -hier -nowarn {*u_arm_bridge}]
  puts $check_file [format {%s expected=0 actual=%s} {*u_arm_bridge} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *u_arm_bridge}}
  set selected [get_cells -hier -nowarn {*u_mapper}]
  puts $check_file [format {%s expected=0 actual=%s} {*u_mapper} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *u_mapper}}
  set selected [get_cells -hier -nowarn {*u_arm_control}]
  puts $check_file [format {%s expected=0 actual=%s} {*u_arm_control} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *u_arm_control}}
  set selected [get_cells -hier -nowarn {*u_ukiyo_v2}]
  puts $check_file [format {%s expected=0 actual=%s} {*u_ukiyo_v2} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *u_ukiyo_v2}}
  set selected [get_cells -hier -nowarn {*u_stylize}]
  puts $check_file [format {%s expected=0 actual=%s} {*u_stylize} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: *u_stylize}}
  set selected [get_cells -hier -nowarn {g_network$u_network}]
  puts $check_file [format {%s expected=0 actual=%s} {g_network$u_network} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: g_network$u_network}}
  set selected [get_cells -hier -nowarn {g_network$u_network_phy}]
  puts $check_file [format {%s expected=0 actual=%s} {g_network$u_network_phy} $selected]
  if {[llength $selected] != 0} {error {Profile instance mismatch: g_network$u_network_phy}}
  set selected [get_cells -hier -nowarn {*u_grip_feedback}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_grip_feedback} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_grip_feedback}}
  set selected [get_cells -hier -nowarn {*u_templates}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_templates} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_templates}}
  set selected [get_cells -hier -nowarn {*u_normalizer}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_normalizer} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_normalizer}}
  set selected [get_cells -hier -nowarn {*u_background}]
  puts $check_file [format {%s expected=1 actual=%s} {*u_background} $selected]
  if {[llength $selected] != 1} {error {Profile instance mismatch: *u_background}}
  close $check_file}] $board_flow]
set board_flow [string map [list {
  place
} {
  set_param place seed 2
  place
}] $board_flow]
eval $board_flow
puts PROFILE_PLACE_FINISHED
close_project

} message]} {puts stderr $message;exit 1}
exit
