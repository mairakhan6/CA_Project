`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:44:40 PM
// Design Name: 
// Module Name: MEM_WB
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


module MEM_WB(
    input clk,                      // Clock signal
    input RegWrite,                 // Control signal for enabling register write
    input MemtoReg,                 // Control signal for selecting memory or ALU result for register write
    input [63:0] ReadData,          // Data read from memory or register file
    input [63:0] ALU_result,        // Result of the ALU operation
    input [4:0] destination_reg,    // Destination register for register write

    output reg RegWrite_out,        // Output signal for enabling register write
    output reg MemtoReg_out,        // Output signal for selecting memory or ALU result for register write
    output reg [63:0] ReadData_out, // Output signal for data read from memory or register file
    output reg [63:0] ALU_result_out, // Output signal for the ALU result
    output reg [4:0] destination_reg_out // Output signal for destination register for register write
);

    // Assign output values based on input signals
    always @(posedge clk) begin
        RegWrite_out = RegWrite;
        MemtoReg_out = MemtoReg;
        ReadData_out = ReadData;
        ALU_result_out = ALU_result;
        destination_reg_out = destination_reg;
    end

endmodule
