add_file -type verilog "src/RISC-V_SBK_SoC.v"
add_file -type verilog "src/peripherals.v"
add_file -type cst "src/TangNano1k.cst"
add_file -type sdc "src/GW1NZ.sdc"
set_device GW1NZ-LV1QN48C6/I5 -device_version NA
set_csr NA
set_option -synthesis_tool gowinsynthesis
set_option -output_base_name RISC-V_SBK
set_option -global_freq default
set_option -top_module Soc
set_option -verilog_std v2001
set_option -vhdl_std vhd2008
set_option -print_all_synthesis_warning 1
set_option -disable_io_insertion 0
set_option -looplimit 2000
set_option -rw_check_on_ram 1
set_option -gen_sdf 0
set_option -gen_io_cst 0

set_option -gen_ibis 0
set_option -gen_posp 0
set_option -gen_text_timing_rpt 1
set_option -gen_verilog_sim_netlist 0
set_option -gen_vhdl_sim_netlist 0
set_option -show_init_in_vo 0
set_option -show_all_warn 1
set_option -timing_driven 1
set_option -ireg_in_iob 1
set_option -oreg_in_iob 1
set_option -ioreg_in_iob 1
set_option -replicate_resources 0
set_option -cst_warn_to_error 1
set_option -rpt_auto_place_io_info 0
set_option -correct_hold_violation 1
set_option -place_option 2
set_option -route_option 2
set_option -clock_route_order 1
set_option -route_maxfan 100
set_option -use_jtag_as_gpio 0
set_option -use_sspi_as_gpio 1
set_option -use_mspi_as_gpio 1
set_option -use_ready_as_gpio 1
set_option -use_done_as_gpio 1
set_option -use_reconfign_as_gpio 1
set_option -use_mode_as_gpio 0
set_option -use_i2c_as_gpio 0
set_option -use_cpu_as_gpio 0
set_option -power_on_reset_monitor 1
set_option -bit_format bin
set_option -bit_crc_check 1
set_option -bit_compress 1
set_option -bit_encrypt 0
set_option -bit_encrypt_key 00000000000000000000000000000000
set_option -bit_security 1
set_option -bit_incl_bsram_init 1
set_option -bg_programming off
set_option -hotboot 0
set_option -i2c_slave_addr 00
set_option -secure_mode 0
set_option -loading_rate default
set_option -program_done_bypass 0
set_option -wakeup_mode 0
set_option -user_code default
set_option -unused_pin default
set_option -multi_boot 1
set_option -multiboot_address_width 24
set_option -multiboot_mode single
set_option -multiboot_spi_flash_address 00000000
set_option -mspi_jump 0
set_option -turn_off_bg 0
set_option -vccx 3.3
set_option -vcc 1.2

run all
