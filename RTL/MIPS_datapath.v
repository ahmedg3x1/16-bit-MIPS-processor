module MIPS_datapath(clk, rst, in_port, out_port, instruction, InsRead, PCnext, RegDst, PCSrc
, MemRead, MemtoReg, ALUControl, MemWrite, ALUSrc, RegWrite, zero_flag, outEn);

    parameter BUS_WIDTH = 16,
              REGFILE_DEPTH = 8;
    localparam REGFILEA_ADDR_SIZE = $clog2(REGFILE_DEPTH);

    input clk, rst, InsRead, PCnext, MemRead, MemWrite, ALUSrc, RegWrite, outEn;
    input [1:0] RegDst, PCSrc, MemtoReg;
    input [2:0] ALUControl; 
    input [BUS_WIDTH-1:0] in_port;

    output [BUS_WIDTH-1:0] instruction;
    output zero_flag;
    output reg [BUS_WIDTH-1:0] out_port;

//----------------------internal wires---------------------------//
    //pc
    wire [BUS_WIDTH-1:0] sign_extend, sign_extend_shifted, pc_incr, pc_adder, jump_addr;
    reg [BUS_WIDTH-1:0] pc, next_pc;
    
    //regfile
    wire [REGFILEA_ADDR_SIZE-1:0] dst;
    reg [BUS_WIDTH-1:0] regfile_wr_data;
    wire [BUS_WIDTH-1:0] read_data_1, read_data_2, alu_B_mux, alus_result;

    //data memory
     wire [BUS_WIDTH-1:0] mem_read_data;
//---------------------------------------------------------------//

//----------------------programme counter---------------------------//
    assign pc_incr = pc + 2;
    assign sign_extend = {{(BUS_WIDTH - REGFILEA_ADDR_SIZE){1'b0}}, instruction[5:0]};
    assign sign_extend_shifted = sign_extend << 1;
    assign pc_adder = pc_incr + sign_extend_shifted;

    assign jump_addr = {pc[15:13], instruction[11:0], 1'b0};

    always @(*) begin
        case (PCSrc)
            2'b00: next_pc = pc_incr;
            2'b01: next_pc = pc_adder;
            2'b10: next_pc = jump_addr;
            2'b11: next_pc = read_data_1;
        endcase
    end
    
    always @(posedge clk) begin
        if(rst)
            pc <= 0;
        else
            if(PCnext)
                pc <= next_pc;
    end
//------------------------------------------------------------------//

//----------------------instruction memorey---------------------------//
    instruction_memory #(BUS_WIDTH) inst_mem(.clk(clk), .rst(rst), .rd_en(InsRead), .addr(pc), .instruction(instruction));
//--------------------------------------------------------------------//


//----------------------regFile---------------------------//
    assign dst = (RegDst == 2'b10) ? 3'b111 : ((RegDst == 2'b01) ? instruction[5:3] : instruction[8:6]); //regfile addr.

    register_file #(BUS_WIDTH, REGFILE_DEPTH) 
     reg_file(.clk(clk), .rst(rst), .rd_addr_1(instruction[11:9]), .rd_addr_2(instruction[8:6]), .wr_en(RegWrite), .wr_addr(dst), 
      .wr_data(regfile_wr_data), .rd_data_1(read_data_1), .rd_data_2(read_data_2));
//--------------------------------------------------------//


//----------------------ALSU---------------------------//
    assign alu_B_mux = (ALUSrc) ?  sign_extend : read_data_2; //source for alus input B
    ALSU #(BUS_WIDTH) alsu(.alu_Op(ALUControl), .alu_A(read_data_1), .alu_B(alu_B_mux), .alus_result(alus_result), .zero_flag(zero_flag));
//-----------------------------------------------------//


//----------------------data memory---------------------------//
    data_memory #(BUS_WIDTH) data_mem(.clk(clk), .rst(rst), .rd_en(MemRead), .wr_en(MemWrite), .addr(alus_result), .wr_data(read_data_2), .read_data(mem_read_data));
//------------------------------------------------------------//
   

//------------------regfile: write mux-----------------------//
    always @(*) begin
        case (MemtoReg)
            2'b00: regfile_wr_data = alus_result;
            2'b01: regfile_wr_data = mem_read_data;
            2'b10: regfile_wr_data = pc;
            2'b11: regfile_wr_data = in_port;
        endcase
    end
//-----------------------------------------------------------//


//--------------------Output - Port-------------------------//
        always @(posedge clk) begin
            if(rst)
                out_port <= 0;
            else 
                if(outEn) 
                    out_port <= read_data_1;
        end
//----------------------------------------------------------//   

endmodule