#!/bin/sh

~/Documents/iac/lab0-devtools/tools/attach_usb.sh

# cleanup
rm -rf obj_dir
rm -f *.vcd

# run Verilator to translate Verilog into C++, including C++ testbench
# verilator -Wall --cc --trace bin2bcd.sv clktick.sv delay.sv f1_fsm.sv f1.sv lfsr.sv mux.sv --exe f1_tb.cpp
verilator -Wall --cc --trace --top-module f1 bin2bcd.sv clktick.sv delay.sv f1_fsm.sv f1.sv lfsr.sv mux.sv --exe f1_tb.cpp

# build C++ project via make automatically generated by Verilator
make -j -C obj_dir/ -f Vf1.mk Vf1

# run executable simulation file
echo "\nRunning simulation"
obj_dir/Vf1
echo "\nSimulation completed"
