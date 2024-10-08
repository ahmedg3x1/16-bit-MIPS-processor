module instruction_memory(clk, rst, rd_en, addr, instruction);
    parameter BUS_WIDTH = 16;

    localparam MEM_WIDTH = 16, 
               DEPTH = 2 ** BUS_WIDTH; 

    input clk, rst, rd_en;
    input [BUS_WIDTH-1:0] addr;
    output reg [BUS_WIDTH-1:0] instruction;
    
    reg [MEM_WIDTH-1:0] INS_MEM [DEPTH-1:0];

    always @(posedge clk) begin
        if(rst)
            instruction <= 0;
        else
            if(rd_en)
                instruction <= INS_MEM[{1'b0, addr[15:1]}];
    end
endmodule