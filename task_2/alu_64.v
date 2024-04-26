`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:18:19 PM
// Design Name: 
// Module Name: alu_64
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


module alu_64
(
    input [63:0] a, b,
    input [3:0] ALUOp,
    output reg [63:0] Result,
    output reg Zero,
    output reg is_greater
);
    localparam [3:0]
    AND = 4'b0000, // availibility of operands in the ALU
    OR	= 4'b0001,
    ADD	= 4'b0010,
    Sub	= 4'b0110,
    NOR = 4'b1100,
    Lesser=4'b0100,
    LeftShift=4'b0111;
    
    always @ (ALUOp, a, b) // put forward the output whenever aluop or the 2 inputs change
    begin
        case (ALUOp)
            AND: Result = a & b;
            OR:	 Result = a | b;
            ADD: Result = a + b;
            Sub: Result = a - b;
            NOR: Result = ~(a | b);
            Lesser: Result = ( a < b)? 0: 1;
            LeftShift: Result = a << b;
            default: Result = 0; // result will be zero if there is an invalid operand
        endcase
         // if result is zero, Zeero signal will be high for other uses
        Zero = (Result == 64'd0) ? 1'b1 : 1'b0;

        // Check if a is greater than b and update the extra signal
        is_greater = (a > b) ? 1'b1 : 1'b0;
    end
    
endmodule
