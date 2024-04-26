`timescale 1ns / 1ps

module exmemreg(
  input clk,reset,
  input [63:0] adderout, //adder output
  input [63:0] resultinalu,//64bit alu output
  input zeroin,//64bit alu output
  input [63:0] writedatain, //2 bit mux2by1 output
  input [4:0] rdin, //IDEX output
  input branchin,memreadin, memtoregin, memwritein, regwritein, //IDEXX outputs
  input addermuxselectin,
  output reg [63:0] exmemadderout,
  output reg exmemzero,
  output reg [63:0] exmemresultoutalu,
  output reg [63:0] exmemwritedataout,
  output reg [4:0] exmemrd,
  output reg exmembranch, exmemmemread, exmemmemtoreg, exmemmemwrite, exmemregwrite,
  output reg exmemaddermuxselect);
  
  always @(posedge clk)
    begin
      if (reset == 1'b1)
        begin
          exmemadderout = 64'b0;
          exmemzero = 1'b0;
          exmemresultoutalu = 63'b0;
          exmemwritedataout = 64'b0;
          exmemrd = 5'b0;
          exmembranch = 1'b0;
          exmemmemread = 1'b0;
          exmemmemtoreg =1'b0;
          exmemmemwrite = 1'b0;
          exmemregwrite = 1'b0;
          exmemaddermuxselect = 1'b0;
        end
      else
        begin
          exmemadderout = adderout;
          exmemzero = zeroin;
          exmemresultoutalu = resultinalu;
          exmemwritedataout = writedatain;
          exmemrd = rdin;
          exmembranch = branchin;
          exmemmemread = memreadin;
          exmemmemtoreg = memtoregin;
          exmemmemwrite = memwritein;
          exmemregwrite = regwritein;
          exmemaddermuxselect = addermuxselectin;
        end
    end
endmodule