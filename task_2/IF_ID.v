`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:32:54 PM
// Design Name: 
// Module Name: IF_ID
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


module IF_ID(
    input clk, IFID_Write, Flush,
    input [63:0] PC_addr,
    input [31:0] Instruc,
    output reg [63:0] PC_store,
    output reg [31:0] Instr_store
);

always @(posedge clk) begin
    // Check if Flush signal is active
    if (Flush) begin
        // Flush active: Reset stored values
        PC_store <= 0;
        Instr_store <= 0;
    end else if (!IFID_Write) begin
        // IFID_Write inactive: Preserve stored values
        PC_store <= PC_store;
        Instr_store <= Instr_store;
    end else begin
        // Store new values in IF/ID pipeline registers
        PC_store <= PC_addr;
        Instr_store <= Instruc;
    end
end

endmodule