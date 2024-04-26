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
    Registers[12] = 64'd7;
    Registers[13] = 64'd8;
end


always @(negedge clk ) begin
    
    if (RegWrite) begin
        Registers[RD] = WriteData;
    end
end

always@(*) begin
    ReadData1 = reset ? 0 : Registers[RS1];
    ReadData2 = reset ? 0 : Registers[RS2];
end

  assign r1 = Registers[11];
  assign r2 = Registers[12];
  assign r3 = Registers[13];
  assign r4 = Registers[14];
  assign r5 = Registers[15];

endmodule
