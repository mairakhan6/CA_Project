`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:28:16 PM
// Design Name: 
// Module Name: EX_MEM
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


module EX_MEM(
    input clk,                     // Clock 
    input Flush,                   // Flush control 
    input RegWrite,                
    input MemtoReg,                // Control signal for selecting memory or ALU result for register write
    input Branch,                  // Branch Control SIgnal
    input Zero,                    // Control singal indicating the ALU result is zero
    input MemWrite,                
    input MemRead,                 
    input is_greater,              // Control signal indicating the comparison result of the ALU operation
    input [63:0] immvalue_added_pc, // Immediate value added to the program counter
    input [63:0] ALU_result,       // Result of the ALU operation
    input [63:0] WriteData,        // Data to be written to memory or register file
    input [3:0] function_code,     // Function code for ALU operation
    input [4:0] destination_reg,   // Destination register for register write

    output reg RegWrite_out,       // register write
    output reg MemtoReg_out,       // MEM or ALU result for register write
    output reg Branch_out,         // branch signal
    output reg Zero_out,           // signal for ALU result is zero
    output reg MemWrite_out,       // MEM write
    output reg MemRead_out,        // MEM read
    output reg is_greater_out,     // when a greater than check for comparison
    output reg [63:0] immvalue_added_pc_out, // immediate value added to the program counter
    output reg [63:0] ALU_result_out,       // ALU result
    output reg [63:0] WriteData_out,        // Write data
    output reg [3:0] ALU_OP,     // ALU operation code
    output reg [4:0] dest_reg_out    // destination register for reg write operation
);

    // Assign output values based on control signals
    always @(posedge clk) begin
        if (Flush) begin
            // SET every signal and every output to 0 if flush signaled
            RegWrite_out = 0;
            MemtoReg_out = 0;
            Branch_out = 0;
            Zero_out = 0;
            is_greater_out = 0;
            MemWrite_out = 0;
            MemRead_out = 0;
            immvalue_added_pc_out = 0;
            ALU_result_out = 0;
            WriteData_out = 0;
            ALU_OP = 0;
            dest_reg_out = 0;
        end 
        else begin
            // Assign output values based on input signals
            RegWrite_out = RegWrite;
            MemtoReg_out = MemtoReg;
            Branch_out = Branch;
            Zero_out = Zero;
            is_greater_out = is_greater;
            MemWrite_out = MemWrite;
            MemRead_out = MemRead;
            immvalue_added_pc_out = immvalue_added_pc;
            ALU_result_out = ALU_result;
            WriteData_out = WriteData;
            ALU_OP = function_code;
            dest_reg_out = destination_reg;
        end
    end

endmodule
