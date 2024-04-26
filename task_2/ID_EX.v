`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:31:56 PM
// Design Name: 
// Module Name: ID_EX
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


module ID_EX(
    input        clk,                            // Clock signal
    input        Flush,                          // Flush control signal
    input [63:0] program_counter_addr,    // Program counter address input
    input [63:0] read_data1,               // Data 1 input
    input [63:0] read_data2,               // Data 2 input
    input [63:0] immediate_value,          // Immediate value input
    input [3:0]  function_code,             // Function code input
    input [4:0]  destination_reg,           // Destination register input
    input [4:0]  source_reg1,                // Source register 1 input
    input [4:0]  source_reg2,               // Source register 2 input
    input        MemtoReg,                        // Memory-to-register control signal
    input        RegWrite,                        // Register write control signal
    input        Branch,                           // Branch control signal
    input        MemWrite,                         // Memory write control signal
    input        MemRead,                          // Memory read control signal
    input        ALUSrc,                           // ALU source control signal
    input [1:0]  ALU_op,                     // ALU operation control signal

    output reg [63:0] program_counter_addr_out,    // Output: Stored program counter address
    output reg [63:0] read_data1_out,               // Output: Stored Data 1
    output reg [63:0] read_data2_out,               // Output: Stored Data 2
    output reg [63:0] immediate_value_out,          // Output: Stored Immediate value
    output reg [3:0] function_code_out,             // Output: Stored Function code
    output reg [4:0] destination_reg_out,           // Output: Stored Destination register
    output reg [4:0] source_reg1_out,                     // Output: Stored Source register 1
    output reg [4:0] source_reg2_out,                     // Output: Stored Source register 2
    output reg       MemtoReg_out,                        // Output: Stored Memory-to-register control
    output reg       RegWrite_out,                            // Output: Stored Register write control
    output reg Branch_out,                               // Output: Stored Branch control
    output reg MemWrite_out,                            // Output: Stored Memory write control
    output reg MemRead_out,                         // Output: Stored Memory read control
    output reg ALUSrc_out,                               // Output: Stored ALU source control
    output reg [1:0] ALU_op_out                     // Output: Stored ALU operation control
    
);

always @(posedge clk) begin
    if (Flush) 
    begin
    // Reset all output registers to 0
    program_counter_addr_out = 0;
    read_data1_out = 0;
    read_data2_out = 0;
    immediate_value_out = 0;
    function_code_out = 0;
    destination_reg_out = 0;
    source_reg1_out = 0;
    source_reg2_out = 0;
    MemtoReg_out = 0;
    RegWrite_out = 0;
    Branch_out = 0;
    MemWrite_out = 0;
    MemRead_out = 0;
    ALUSrc_out = 0;
    ALU_op_out = 0;
    end 
    
    else 
    begin
        // Pass input values to output registers
    program_counter_addr_out = program_counter_addr;
    read_data1_out = read_data1;
    read_data2_out = read_data2;
    immediate_value_out = immediate_value;
    function_code_out = function_code;
    destination_reg_out = destination_reg;
    source_reg1_out = source_reg1;
    source_reg2_out = source_reg2;
    RegWrite_out = RegWrite;
    MemtoReg_out = MemtoReg;
    Branch_out = Branch;
    MemWrite_out = MemWrite;
    MemRead_out = MemRead;
    ALUSrc_out = ALUSrc;
    ALU_op_out = ALU_op;
    end
end

endmodule 
