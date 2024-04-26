`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:30:40 PM
// Design Name: 
// Module Name: Hazard_Detection
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


module Hazard_Detection
(
    input [4:0] current_rd, previous_rs1, previous_rs2,
    input current_MemRead,
    output reg mux_out,
    output reg enable_Write, enable_PCWrite
);
initial begin
    if (current_MemRead && (current_rd == previous_rs1 || current_rd == previous_rs2)) begin
        // Hazard detected: Set control signals accordingly
        mux_out = 1;             // Disable the multiplexer output
        enable_Write = 1;       // Disable write to the next pipeline stage
        enable_PCWrite = 1;    // Disable PC write
    end else begin
        // No hazard detected: Set control signals accordingly
        mux_out = 1;             // Enable the multiplexer output
        enable_Write = 1;       // Enable write to the next pipeline stage
        enable_PCWrite = 1;    // Enable PC write
    end
end

endmodule
