module ALSU(alu_Op, alu_A, alu_B, alus_result, zero_flag);
    parameter BUS_WIDTH = 16;

    input [2:0] alu_Op;
    input [BUS_WIDTH-1:0] alu_A, alu_B;

    output reg [BUS_WIDTH-1:0] alus_result;
    output zero_flag;

    always @(*) begin
        case (alu_Op)
            3'b000: alus_result = alu_A + alu_B;    //ADD
            3'b001: alus_result = alu_A - alu_B;    //SUB
            3'b010: alus_result = alu_A & alu_B;    //AND
            3'b011: alus_result = alu_A | alu_B;    //OR 
            3'b100: alus_result = ~(alu_A | alu_B); //NOR
            3'b101: alus_result = alu_A ^ alu_B;    //XOR
            3'b110: alus_result = alu_A << 1;       //SLL
            3'b111: alus_result = alu_A >> 1;       //SRL
        endcase 
    end

    assign zero_flag = (alus_result) ? 1'b0 : 1'b1;
endmodule