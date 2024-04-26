`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:22:43 PM
// Design Name: 
// Module Name: ALU_Control
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module ALU_Control
(
    input [1:0] ALUOp, // 2 bit ALUOPcode
    input [3:0] Funct, // 4 bit func to detemine what type of operation will be performed
    output reg [3:0] Operation // 4 bit Output operation
);
always @(*)
begin
    case(ALUOp)
        2'b00: // ALUOp = 00, R-type
        begin
            Operation = 4'b0010; // ALU operation: Add
        end
        
        2'b01: // ALUOp = 01, Branch type
        begin
            case(Funct[2:0])
                3'b000: // beq when func3 = 000
                begin
                    Operation = 4'b0110; // ALU operation: Subtract
                end
                
                3'b100: // blt whne func3 = 100
                begin
                    Operation = 4'b0100; // ALU operation Less than
                end
            endcase
        end

        2'b10: // ALUOp = 10, immediate type instructions
        begin
            case(Funct)
                4'b0000: // Funct = 0000
                begin
                    Operation = 4'b0010; // Immediate Add
                end
                
                4'b1000: // Funct = 1000
                begin
                    Operation = 4'b0110; // Immediate Subtract
                end
                
                4'b0111: // Funct = 0111
                begin
                    Operation = 4'b0000; // ALU operation: AND
                end
                
                4'b0110: // Funct = 0110
                begin
                    Operation = 4'b0001; // ALU operation: OR
                end
            endcase
        end
    endcase
end

endmodule

