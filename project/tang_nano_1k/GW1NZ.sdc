//Copyright (C)2014-2025 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//Tool Version: V1.9.11.03 Education 
//Created Time: 2025-11-28 07:51:24
create_clock -name clk_in -period 37.037 -waveform {0 18.5} [get_ports {clock}]
report_timing -setup -max_paths 300 -max_common_paths 1
set_operating_conditions -grade c -model slow -speed 6 -setup -hold -max -min -max_min
