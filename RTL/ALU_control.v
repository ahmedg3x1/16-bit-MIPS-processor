module ALU_cotrol (ALUOP, func, ALUControl);
    input [2:0] ALUOP, func;
    output reg [2:0] ALUControl;

    always @(*) begin
        case (ALUOP)
            3'b000: ALUControl = func;
            3'b001: ALUControl = 3'b000;
            3'b010: ALUControl = 3'b010;
            3'b011: ALUControl = 3'b011;
            3'b100: ALUControl = 3'b001;
        endcase
    end
endmodule