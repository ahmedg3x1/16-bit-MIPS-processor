module MIPS_top (clk, rst, in_port, out_port);
    parameter BUS_WIDTH = 16,
              REGFILE_DEPTH = 8;

    input clk, rst;
    input [BUS_WIDTH-1:0] in_port;

    output [BUS_WIDTH-1:0] out_port;

    //----------------------internal wires---------------------------//
    wire InsRead, PCnext, MemRead, MemWrite, ALUSrc, RegWrite,zero_flag, outEn; 
    wire [1:0] RegDst, PCSrc, MemtoReg;
    wire [2:0] ALUControl, func, ALUOP; 
    wire [BUS_WIDTH-1:0] instruction;
    assign func = instruction[2:0];
    //---------------------------------------------------------------//

    MIPS_datapath #(BUS_WIDTH, REGFILE_DEPTH) datapath(clk, rst, in_port, out_port, instruction, InsRead, PCnext, RegDst, PCSrc
    , MemRead, MemtoReg, ALUControl, MemWrite, ALUSrc, RegWrite, zero_flag, outEn);

    MIPS_control #(BUS_WIDTH) control(clk, rst, MemWrite, MemRead, zero_flag, instruction, InsRead, PCSrc, PCnext, ALUOP, ALUSrc, RegDst, MemtoReg, RegWrite, outEn);
    ALU_cotrol alu_cotrol(ALUOP, func, ALUControl);

endmodule