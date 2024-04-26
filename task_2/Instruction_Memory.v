`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:40:15 PM
// Design Name: 
// Module Name: Instruction_Memory
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


module Instruction_Memory
(
	input [63:0] Inst_Address,
	output reg [31:0] Instruction
);
	reg [7:0] inst_mem [63:0];
	
	initial begin
    inst_mem[0] = 8'b00110011; //add x10,x12,x13
    inst_mem[1] = 8'b00000101;
    inst_mem[2] = 8'b11010110;
    inst_mem[3] = 8'b0;
    
    inst_mem[4] = 8'b00110011; //add x8,x10,x12
    inst_mem[5] = 8'b00000100;
    inst_mem[6] = 8'b11000101;
    inst_mem[7] = 8'b0;
    
//    inst_mem[8] = 8'b10110011; //sub x1,x8,x10
//    inst_mem[9] = 8'b00000000;
//    inst_mem[10] = 8'b10010100;
//    inst_mem[11] = 8'b01000000;

    inst_mem[8] = 8'b10110011; //add x1,x8,x10
    inst_mem[9] = 8'b00000000;
    inst_mem[10] = 8'b10100100;
    inst_mem[11] = 8'b00000000;
    
    
    
	end
	
	always @(Inst_Address)
	begin
		Instruction={inst_mem[Inst_Address+3],inst_mem[Inst_Address+2],inst_mem[Inst_Address+1],inst_mem[Inst_Address]};
	end
endmodule