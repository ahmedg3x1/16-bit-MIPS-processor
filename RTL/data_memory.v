module data_memory(clk, rst, rd_en, wr_en, addr, wr_data, read_data);
    parameter BUS_WIDTH = 16;
    localparam MEM_WIDTH = 8,
               DEPTH = 2 ** BUS_WIDTH; 

    input clk, rst, rd_en, wr_en;
    input [BUS_WIDTH-1:0] addr, wr_data; 
    output reg [BUS_WIDTH-1:0] read_data;


    reg [MEM_WIDTH-1:0] DATA_MEM [DEPTH-1:0]; //byte accessible, word = 2 bytes and big endian

    
    always @(posedge clk) begin
        if(rst)
            read_data <= 0;
        else begin
            if(rd_en)
                read_data <= {DATA_MEM[addr], DATA_MEM[addr+1]};
            if(wr_en) 
                {DATA_MEM[addr], DATA_MEM[addr+1]} = wr_data;
        end
    end

endmodule