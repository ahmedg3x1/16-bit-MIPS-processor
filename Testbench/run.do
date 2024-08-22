vlib work
vlog MIPS_tb.v
vlog -f files.args
vsim -voptargs=+acc work.MIPS_tb
add wave clk
add wave rst
add wave -radix unsigned in_port
add wave -radix unsigned out_port
add wave -radix hexadecimal dut/datapath/instruction
add wave -radix unsigned dut/control/cs
add wave -radix unsigned dut/control/ns
add wave -radix unsigned dut/datapath/pc
add wave -radix unsigned dut/datapath/next_pc
add wave -radix unsigned dut/datapath/reg_file/reg_file
add wave dut/control/PCSrc
add wave dut/control/PCnext
add wave -radix unsigned dut/datapath/jump_addr
add wave dut/datapath/data_mem/DATA_MEM
run -all