`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:27:26 PM
// Design Name: 
// Module Name: Data_Memory
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


module Data_Memory
(
	input [63:0] Mem_Addr,
	input [63:0] Write_Data,
	input clk, MemWrite, MemRead,
	output reg [63:0] Read_Data,
	output [63:0] ele1,
    output [63:0] ele2,
    output [63:0] ele3,
    output [63:0] ele4,
    output [63:0] ele5
);

	reg [7:0] DataMemory [63:0];
	
	  assign ele1 = DataMemory[0]; // these 5 elemenets represent the first 5 values in the memory
      assign ele2 = DataMemory[8];
      assign ele3 = DataMemory[16];                      
      assign ele4 = DataMemory[24];
      assign ele5 = DataMemory[32];

      integer x; // loop variable
  
   initial  
    begin 
      for (x = 0; x < 64; x = x + 1)
      begin 
        DataMemory[x] = 8'd0; //reset all values
        end
     end    

	
	always @ (posedge clk)
	begin
		if (MemWrite) // on mem write signal high
		begin
			DataMemory[Mem_Addr] = Write_Data[7:0];
			DataMemory[Mem_Addr+1] = Write_Data[15:8];
			DataMemory[Mem_Addr+2] = Write_Data[23:16];
			DataMemory[Mem_Addr+3] = Write_Data[31:24];
			DataMemory[Mem_Addr+4] = Write_Data[39:32];
			DataMemory[Mem_Addr+5] = Write_Data[47:40];
			DataMemory[Mem_Addr+6] = Write_Data[55:48];
			DataMemory[Mem_Addr+7] = Write_Data[63:56];
		end
	end
	
	always @ (*)
	begin
		if (MemRead) // on mem read signal high
			Read_Data = {DataMemory[Mem_Addr+7],DataMemory[Mem_Addr+6],DataMemory[Mem_Addr+5],DataMemory[Mem_Addr+4],DataMemory[Mem_Addr+3],DataMemory[Mem_Addr+2],DataMemory[Mem_Addr+1],DataMemory[Mem_Addr]};
	end
endmodule

