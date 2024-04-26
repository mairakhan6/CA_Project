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

reg [7:0] DataMemory [512:0];

assign  ele1  = DataMemory[256];
assign  ele2 = DataMemory[264];
assign  ele3 = DataMemory[272];                      
assign  ele4 = DataMemory[280];
assign  ele5 = DataMemory[288];
integer i;

initial  
begin 
for (i = 0; i < 512; i = i + 1)
begin 
//DataMemory[256] = 8'd20;
// DataMemory[264]=  8'd10;
// DataMemory[272]= 8'd30;
// DataMemory[280] = 8'd40;
// DataMemory[288]= 8'd50;
// DataMemory[296]= 8'd100;

//DataMemory[256] = 8'd3;
// DataMemory[264]=  8'd2;
// DataMemory[272]= 8'd5;
// DataMemory[280] = 8'd4;
// DataMemory[288]= 8'd6;
DataMemory[256] = 8'd2;
 DataMemory[264]=  8'd5;
 DataMemory[272]= 8'd4;
 DataMemory[280] = 8'd6;
 DataMemory[288]= 8'd3;
 
end
end    

	
always @ (posedge clk)
begin
if (MemWrite)
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
		if (MemRead)
Read_Data = {DataMemory[Mem_Addr+7],DataMemory[Mem_Addr+6],DataMemory[Mem_Addr+5],DataMemory[Mem_Addr+4],DataMemory[Mem_Addr+3],DataMemory[Mem_Addr+2],DataMemory[Mem_Addr+1],DataMemory[Mem_Addr]};
	end
endmodule
