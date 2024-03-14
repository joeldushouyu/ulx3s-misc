#-- Lattice Semiconductor Corporation Ltd.
#-- Synplify OEM project file

#device options
set_option -technology ECP5U
set_option -part LFE5U_45F
set_option -package BG381C
set_option -speed_grade -6

#compilation/mapping options
set_option -symbolic_fsm_compiler true
set_option -resource_sharing true

#use verilog 2001 standard option
set_option -vlog_std v2001

#map options
set_option -frequency auto
set_option -maxfan 1000
set_option -auto_constrain_io 0
set_option -disable_io_insertion false
set_option -retiming false; set_option -pipe true
set_option -force_gsr false
set_option -compiler_compatible 0
set_option -dup false

set_option -default_enum_encoding onehot

#simulation options


#timing analysis options



#automatic place and route (vendor) options
set_option -write_apr_constraint 1

#synplifyPro options
set_option -fix_gated_and_generated_clocks 1
set_option -update_models_cp 0
set_option -resolve_multiple_driver 0


set_option -seqshift_no_replicate 0

#-- add_file options
add_file -vhdl {/usr/local/diamond/3.13/cae_library/synthesis/vhdl/ecp5u.vhd}
add_file -vhdl -lib "work" {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/sdram/sdram_fpga/hdl/top/sdram_fpga_hex_oled.vhd}
add_file -vhdl -lib "work" {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/sdram/sdram_fpga/hdl/sdram_0bject.vhd}
add_file -vhdl -lib "work" {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/oled/proj/ulx3s_hex_vhdl/clock/clk_25M_100M_7M5_12M_60M.vhd}
add_file -vhdl -lib "work" {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/oled/hdl/ssd1331_hex_vhdl/oled_hex_decoder.vhd}
add_file -vhdl -lib "work" {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/oled/hdl/ssd1331_hex_vhdl/oled_init_pack.vhd}
add_file -vhdl -lib "work" {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/oled/hdl/ssd1331_hex_vhdl/oled_font_pack.vhd}

#-- top module name
set_option -top_module sdram_fpga_hex_oled

#-- set result format/file last
project -result_file {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/sdram/sdram_fpga/proj/ulx3s_sdram_hex_v/project/project_project.edi}

#-- error message log file
project -log_file {project_project.srf}

#-- set any command lines input by customer


#-- run Synplify with 'arrange HDL file'
project -run hdl_info_gen -fileorder
project -run
