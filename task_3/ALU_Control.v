
module ALU_Control
(
    input [1:0] ALUOp,
    input [3:0] Funct,
    output reg [3:0] Operation
);
always @(*)
begin
case(ALUOp)
    2'b00: // ALUOp = 00, R-type instruction
    begin
        Operation = 4'b0010; // Add
    end
    
    2'b01: // ALUOp = 01, branch type instructions
    begin
        case(Funct[2:0])
            3'b000: // Funct = 000, beq
            begin
                Operation = 4'b0110; // Subtract
            end
            
            3'b100: // Funct = 100, blt
            begin
                Operation = 4'b0100; // Less than
            end
        endcase
    end

    2'b10: // ALUOp = 10, immediate type instructions
    begin
        case(Funct)
            4'b0000: // Funct
            begin
                Operation = 4'b0010; // Add
            end
            
            4'b1000: // Funct = 1000
            begin
                Operation = 4'b0110; // Subtract
            end
            
            4'b0111: // Funct = 0111
            begin
                Operation = 4'b0000; // AND
            end
            
            4'b0110: // Funct = 0110
            begin
                Operation = 4'b0001; // OR
            end
        endcase
    end
endcase
end

endmodule
