`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:52:02 PM
// Design Name: 
// Module Name: registerfile
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


module registerFile
(
    input clk, reset, RegWrite,
    input [63:0] WriteData,
    input [4:0] RS1, RS2, RD,
    output reg [63:0] ReadData1, ReadData2,
    output [63:0] r1,
    output [63:0] r2,
    output [63:0] r3,
    output [63:0] r4,
    output [63:0] r5
);

reg [63:0] Registers [31:0];
integer i;

initial
begin
    for (i = 0; i < 32; i = i + 1)
            Registers[i] = 64'd0;
    Registers[12] = 64'd1;
    Registers[13] = 64'd2;
end


always @(negedge clk ) begin // doing regwrite on the negative clock edge
    
    if (RegWrite) begin
        Registers[RD] = WriteData;
    end
end

always@(*) begin
    ReadData1 = reset ? 0 : Registers[RS1];
    ReadData2 = reset ? 0 : Registers[RS2];
end

  assign r1 = Registers[12];
  assign r2 = Registers[13];
  assign r3 = Registers[10];
  assign r4 = Registers[8];
  assign r5 = Registers[1];

endmodule 
