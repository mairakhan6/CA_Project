`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/20/2024 03:09:06 PM
// Design Name: 
// Module Name: tb
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


module tb();

  reg clk, reset;
//  wire [63:0] e1,e2,e3,e4,e5;
  wire [63:0] r1,r2,r3,r4,r5;
  wire [63:0] e1,e2,e3,e4,e5;
  wire [1:0]fwda,fwdb ;

  RISC_V_Processor t1 (
    .clk(clk),
    .reset(reset),
    .ele1(e1),
    .ele2(e2),
    .ele3(e3),
    .ele4(e4),
    .ele5(e5),
    .reg1(r1),
    .reg2(r2),
    .reg3(r3),
    .reg4(r4),
    .reg5(r5),
    .fwdA(fwda),
    .fwdB(fwdb)
    
    

  );
  
  initial begin
    clk = 1'b0;                      // Initialize clk signal to 0
    reset = 1'b1;                    // Set reset signal to active state
    #10;                             // Wait for 10 time units
    reset = 1'b0;                    // Deassert reset signal
  end
  
  always begin
    #5;                              // Toggle the clock signal every 5 time units
    clk = ~clk;                      // Negate the value of clk signal
  end
  
  initial begin
    $dumpfile("dump.vcd");           // Specify the waveform dump file
    $dumpvars();                     // Dump all variables for waveform visualization
    $monitor("Time = %d --> clk = %b, reset = %b", $time, clk, reset);  // Display time, clk, and reset values in the console
                          
  end
  
endmodule

