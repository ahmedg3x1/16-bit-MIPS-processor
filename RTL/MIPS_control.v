module MIPS_control(clk, rst, MemWrite, MemRead, zero_flag, instruction, InsRead, PCSrc, PCnext, ALUOP, ALUSrc, RegDst, MemtoReg, RegWrite, outEn);
    parameter BUS_WIDTH = 16;
    localparam Fetch   = 5'd0,
               DECODE  = 5'd1,
               ALU     = 5'd2,
               ADDI    = 5'd3,
               ANDI    = 5'd4,
               ORI     = 5'd5,
               Lw_data = 5'd6,
               LW_addr = 5'd7,
               SW      = 5'd8,
               JUMP    = 5'd9,
               JUMPAL  = 5'd10,
               JUMPR   = 5'd11,
               BEQ     = 5'd12,
               BNE     = 5'd13,
               IN      = 5'd14,
               OUT     = 5'd15,
               NOP     = 5'd16,
               HLT     = 5'd17;


    input clk, rst, zero_flag;
    input [BUS_WIDTH-1:0] instruction;

    output reg InsRead, PCnext, RegWrite, outEn, ALUSrc, MemWrite, MemRead;
    output reg [1:0] PCSrc;
    output reg [2:0] ALUOP;
    output reg [1:0] RegDst, MemtoReg;

    reg [3:0] cs, ns;

    always @(posedge clk) begin
        if(rst) 
            cs <= Fetch;
        else 
            cs <= ns;
    end

    always @(*) begin
        case (cs)
            Fetch: ns = DECODE;
            DECODE: begin
                case (instruction[15:12]) //0000
                    4'h0: ns = ALU;
                    4'h4: ns = ADDI;
                    4'h5: ns = ANDI; 
                    4'h6: ns = ORI;  
                    4'h7: ns = LW_addr; 
                    4'h8: ns = SW;
                    4'hb: ns = JUMP;
                    4'hc: ns = JUMPAL;
                    4'h3: ns = JUMPR; 
                    4'h9: ns = BEQ; 
                    4'ha: ns = BNE;
                    4'h1: ns = IN; 
                    4'h2: ns = OUT;
                    4'he: ns = NOP; 
                    4'hf: ns = HLT; 
                    default: ns = Fetch;     
                endcase
            end

            LW_addr: ns = Lw_data;
            HLT:     ns = HLT;
            default: ns = Fetch;
        endcase
    end

    always @(*) begin
        case (cs)
            Fetch: begin
                PCSrc    = 2'b00;
                InsRead  = 1'b1;
                PCnext   = 1'b1;
                {MemWrite, MemRead, ALUOP, RegDst, MemtoReg, RegWrite, outEn} = 0;
            end

            ALU: begin
                ALUOP    = 3'b000;
                ALUSrc   = 1'b0;
                RegDst   = 2'b01;
                MemtoReg = 2'b00;
                RegWrite = 1'b1;
            end

            ADDI: begin
                ALUOP    = 3'b001;
                ALUSrc   = 1'b1;
                RegDst   = 2'b00;
                MemtoReg = 2'b00;
                RegWrite = 1'b1;
            end

            ANDI: begin
                ALUOP    = 3'b010;
                ALUSrc   = 1'b1;
                RegDst   = 2'b00;
                MemtoReg = 2'b00;
                RegWrite = 1'b1;
            end

            ORI: begin
                ALUOP    = 3'b011;
                ALUSrc   = 1'b1;
                RegDst   = 2'b00;
                MemtoReg = 2'b00;
                RegWrite = 1'b1;
            end

            LW_addr: begin
                ALUOP    = 3'b001;
                ALUSrc   = 1'b1;
                MemRead  = 1'b1;
            end

            Lw_data: begin
                RegDst   = 2'b00;
                MemtoReg = 2'b01;
                RegWrite = 1'b1;
            end

            SW: begin
                ALUOP    = 3'b001;
                ALUSrc   = 1'b1;
                MemWrite  = 1'b1;
            end

            JUMP: begin 
                PCSrc = 2'b10;
                PCnext = 1'b1;
            end

            JUMPAL: begin
                PCSrc = 2'b10;
                PCnext = 1'b1;
                MemtoReg = 2'b10;
                RegDst = 2'b10;
                RegWrite = 1'b1;
            end

            JUMPR: begin
                PCSrc = 2'b11;
                PCnext = 1'b1;
            end

            BEQ: begin
                ALUOP = 3'b100;
                ALUSrc = 1'b0;
                if(zero_flag) begin
                    PCSrc = 2'b01;
                    PCnext = 1'b1;
                end
            end

            BNE: begin
                ALUOP = 3'b100;
                ALUSrc = 1'b0;
                if(~zero_flag) begin
                    PCSrc = 2'b01;
                    PCnext = 1'b1;
                end
            end

            IN: begin
                RegDst = 2'b01; 
                MemtoReg = 2'b11;
                RegWrite = 1'b1;  
            end

            OUT: outEn = 1'b1;

            default:
                {InsRead, PCSrc, PCnext, ALUOP, RegDst, MemtoReg, RegWrite, outEn} = 0;
        endcase
    end
endmodule