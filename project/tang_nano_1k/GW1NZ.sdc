//Copyright (C)2014-2023 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//GOWIN Version: 1.9.8.09 Education
//Created Time: 2023-01-07 23:16:19
create_clock -name clk_in -period 20 -waveform {0 10} [get_ports {clock}]
report_timing -setup -max_paths 180 -max_common_paths 1
set_operating_conditions -grade c -model slow -speed 6 -setup -hold -max -min -max_min
