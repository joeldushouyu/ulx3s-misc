#-- Lattice Semiconductor Corporation Ltd.
#-- Synplify OEM project file

#device options
set_option -technology ECP5U
set_option -part LFE5U_12F
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
set_option -include_path {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/sdram/sdram_16bit/proj/ulx3s_sdram_hex_v}
add_file -verilog -vlog_std v2001 {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/sdram/sdram_16bit/hdl/top/sdram_hex_oled.v}
add_file -verilog -vlog_std v2001 {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/sdram/sdram_16bit/hdl/top/clk_25_125_25_12_100.v}
add_file -verilog -vlog_std v2001 {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/sdram/sdram_16bit/hdl/sdram_16bit.v}
add_file -verilog -vlog_std v2001 {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/oled/hdl/ssd1331_video_verilog/oled_video.v}
add_file -verilog -vlog_std v2001 {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/oled/hdl/ssd1331_video_verilog/hex_decoder.v}

#-- top module name
set_option -top_module sdram_hex_oled

#-- set result format/file last
project -result_file {/home/shouyu/4.2/designProblem/ulx3s-misc/examples/sdram/sdram_16bit/proj/ulx3s_sdram_hex_v/project/project_project.edi}

#-- error message log file
project -log_file {project_project.srf}

#-- set any command lines input by customer


#-- run Synplify with 'arrange HDL file'
project -run
