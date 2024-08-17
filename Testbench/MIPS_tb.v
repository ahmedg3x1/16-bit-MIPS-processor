module MIPS_tb;
    parameter BUS_WIDTH = 16,
              REGFILE_DEPTH = 8;

    reg clk, rst;
    reg [BUS_WIDTH-1:0] in_port;
    wire [BUS_WIDTH-1:0] out_port;

    MIPS_top #(BUS_WIDTH, REGFILE_DEPTH) dut(clk, rst, in_port, out_port);

    initial begin
        clk = 0;
        forever
            #1 clk = ~clk;
    end

    initial begin
        $readmemh("mem.dat", dut.datapath.inst_mem.INS_MEM);
        in_port = 0;
        rst = 1;
        @(negedge clk);
        rst = 0;

        repeat(1000) begin
            in_port = $urandom_range(0,100);
            @(negedge clk);
        end
        $stop;
    end 
endmodule