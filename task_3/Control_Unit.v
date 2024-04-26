module Control_Unit (
    input [6:0] Opcode,
    output reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, RegWrite,
    output reg [1:0] ALUOp 
);

    // logic block
    always @(*) begin
        // Opcode-based case statement
case (Opcode)
    7'b0110011: // R-type (add/sub)
        begin
            // outputs according to R-type instruction
            ALUSrc = 1'b0;
            MemtoReg = 1'b0;
            RegWrite = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b10;
        end
    7'b0000011: // I-type (ld types)
        begin
            // outputs according to I-type instruction
            ALUSrc = 1'b1;
            MemtoReg = 1'b1;
            RegWrite = 1'b1;
            MemRead = 1'b1;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b00;
        end
    7'b0100011: // S-type 
        begin
            // outputs according to S-type instruction
            ALUSrc = 1'b1;
            MemtoReg = 1'bx; // 'x' = value
            RegWrite = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b1;
            Branch = 1'b0;
            ALUOp = 2'b00;
        end
    7'b1100011: // Branch instruction
        begin
            // Set output signals for branch instruction
            ALUSrc = 0;
            MemtoReg = 1'bX; // X = unknown
            RegWrite = 0;
            MemRead = 0;
            MemWrite = 0;
            Branch = 1;
            ALUOp = 2'b01;
        end
    7'b0010011: // I-type (addi, slli etc.)
        begin
            // I-type instruction (other)
        Branch = 1'b0;
        MemRead = 1'b0;
        MemtoReg = 1'b0;
        MemWrite = 1'b0;
        ALUSrc = 1'b1;
        RegWrite = 1'b1;
        ALUOp = 2'b00;
        end
    default:
        begin
    // case for invalid type
    Branch = 1'b0;
    MemRead = 1'b0;
    MemtoReg = 1'b0;
    MemWrite = 1'b0;
    ALUSrc = 1'b0;
    RegWrite = 1'b0;
    ALUOp = 2'b00;
        end
endcase
    end

endmodule
