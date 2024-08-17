module register_file(clk, rst, rd_addr_1, rd_addr_2, wr_en, wr_addr, wr_data, rd_data_1, rd_data_2);
    parameter BUS_WIDTH = 16,
              DEPTH = 8;

    localparam ADDR_SIZE = $clog2(DEPTH);

    input clk, rst, wr_en;
    input [ADDR_SIZE-1:0] rd_addr_1, rd_addr_2, wr_addr;
    input [BUS_WIDTH-1:0] wr_data;

    output [BUS_WIDTH-1:0] rd_data_1, rd_data_2;


    reg [BUS_WIDTH-1:0] reg_file [DEPTH-1:0];
    
    integer i;
    always @(posedge clk) begin 
        if(rst) begin
            for (i = 0; i < DEPTH; i = i + 1) 
                reg_file[i] <= 0;
        end    
        else
            if(wr_en) reg_file[wr_addr] <= wr_data;  
    end

    assign rd_data_1 = reg_file[rd_addr_1];
    assign rd_data_2 = reg_file[rd_addr_2];

endmodule