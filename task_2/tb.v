`timescale 1ns / 1ps


module Pipelined_RISC_V_tb();

    // Define parameters
    parameter CLOCK_PERIOD = 10; // Clock period in ns

    // Inputs
    reg clk;
    reg reset;

    // Outputs
    wire [63:0] pc_out;
    wire [63:0] adder1_out;
    wire [63:0] adder2_out;
    wire [63:0] pc_in;
    wire zero;
    wire [31:0] instruction;
    wire [6:0] opcode;
    wire [4:0] rd;
    wire [2:0] funct3;
    wire [4:0] rs1;
    wire [4:0] rs2;
    wire [6:0] funct7;
    wire [63:0] writedata;
    wire [63:0] readdata1;
    wire [63:0] readdata2;
    wire branch;
    wire memread;
    wire memtoreg;
    wire memwrite;
    wire alusrc;
    wire regwrite;
    wire [1:0] aluop;
    wire [63:0] immdata;
    wire [63:0] mux2out;
    wire [3:0] operation;
    wire [63:0] aluout;
    wire [63:0] datamemoryreaddata;
    wire [63:0] element1;
    wire [63:0] element2;
    wire [63:0] element3;
    wire [63:0] element4;
    wire [63:0] element5;
    wire [63:0] element6;
    wire [63:0] element7;
    wire [63:0] element8;

    // Instantiate the DUT
    RISC_V_processor dut(
        .clk(clk),
        .reset(reset),
        .pc_out(pc_out),
        .adder1_out(adder1_out),
        .adder2_out(adder2_out),
        .pc_in(pc_in),
        .zero(zero),
        .instruction(instruction),
        .opcode(opcode),
        .rd(rd),
        .funct3(funct3),
        .rs1(rs1),
        .rs2(rs2),
        .funct7(funct7),
        .writedata(writedata),
        .readdata1(readdata1),
        .readdata2(readdata2),
        .branch(branch),
        .memread(memread),
        .memtoreg(memtoreg),
        .memwrite(memwrite),
        .alusrc(alusrc),
        .regwrite(regwrite),
        .aluop(aluop),
        .immdata(immdata),
        .mux2out(mux2out),
        .operation(operation),
        .aluout(aluout),
        .datamemoryreaddata(datamemoryreaddata),
        .element1(element1),
        .element2(element2),
        .element3(element3),
        .element4(element4),
        .element5(element5),
        .element6(element6),
        .element7(element7),
        .element8(element8)
    );

    // Clock generation
    always #((CLOCK_PERIOD / 2)) clk <= ~clk;
 initial
    begin
      clk = 1'b0;
  end
 
  always
    begin
    #2
    clk = ~clk;
      end
  initial
    begin
      reset = 1'b1;
      #10
      reset = 1'b0;
      #4000
      reset = 1'b1;
      $finish;
      end
 
    // Output monitoring
    always @(posedge clk) begin
        // Monitor outputs here
    end

endmodule