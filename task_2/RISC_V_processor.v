`timescale 1ns / 1ps

module RISC_V_processor(
input clk,
input reset,
output wire [63:0] pc_out,
output wire [63:0] adder1_out,
output wire [63:0] adder2_out,
output wire [63:0] pc_in,
//output wire branch,
output wire zero,
output wire [31:0] instruction,
output wire [6:0] opcode,
output wire [4:0] rd,
output wire [2:0] funct3,
output wire [4:0] rs1,
output wire [4:0] rs2,
output wire [6:0] funct7,
output wire [63:0]writedata,
//output wire regwrite,
output wire [63:0]readdata1,
output wire [63:0]readdata2,
output wire branch, memread, memtoreg, memwrite, alusrc, regwrite,
output wire [1:0] aluop,
output wire [63:0] immdata,
output wire [63:0] mux2out,
output wire [3:0] operation,
output wire [63:0] aluout,
output wire [63:0] datamemoryreaddata,
output wire [63:0] element1,
  output wire [63:0] element2,
  output wire [63:0] element3,
  output wire [63:0] element4,
  output wire [63:0] element5,
  output wire [63:0] element6,
  output wire [63:0] element7,
  output wire [63:0] element8
    );

wire addermuxselect;
wire branch_finale;
wire [3:0] funct;    
    
wire [63:0] ifidpc_out;
wire [31:0] ifidinst;

wire [63:0]idexpc_out,idexreaddata1,idexreaddata2, ideximmdata;
wire [4:0] idexrs1, idexrs2, idexrd;
//wire [2:0] idexfunct3;
wire idexbranch, idexmemread, idexmemtoreg, idexmemwrite, idexregwrite, idexalusrc;
wire [1:0] idexaluop;

wire [63:0] exmemadderout;
wire exmemzero;
 wire [63:0]   exmemaluout;
  wire [63:0]  exmemwritedataout;
  wire [4:0]  exmemrd;
  wire  exmembranch, exmemmemread, exmemmemtoreg, exmemmemwrite, exmemregwrite;
  
wire [63:0] memwbreaddataout, memwbaluout;
wire [4:0] memwbrd; 
wire memwbmemtoreg, mwmwbregwrite;

wire [1:0] forwarda, forwardb;

wire [63:0] m3to1out1, m3to1out2;
    
wire [3:0] idexfunct;

Program_Counter pc(clk, reset, pc_in,pc_out);
Instruction_Memory im(pc_out, instruction);

Adder a1(pc_out, 64'd4, adder1_out);

ifidreg ifid(clk, reset, pc_out, instruction, ifidpc_out, ifidinst);

InsParser ip(ifidinst, opcode, rd, funct3, rs1, rs2, funct7);
Control_Unit cu(opcode, branch, memread, memtoreg, memwrite, alusrc, regwrite, aluop);
registerFile rf(writedata, rs1, rs2, memwbrd, memwbregwrite, clk, reset, readdata1, readdata2);
ImmGen imm_gen(ifidinst, immdata);

idexreg idex( clk, reset,  ifidpc_out,readdata1,readdata2, immdata,   rs1, rs2, rd,
                {ifidinst[30],ifidinst[14:12] },  branch, memread, memtoreg, memwrite, regwrite, alusrc, 
               aluop,  idexpc_out,idexreaddata1,idexreaddata2, ideximmdata,  
               idexrs1, idexrs2, idexrd,   idexfunct,   idexbranch, idexmemread, 
               idexmemtoreg, idexmemwrite, idexregwrite, idexalusrc,   idexaluop);


//assign funct = {ifidinst[30], ifidinst[14:12]};

Adder a2(idexpc_out, ideximmdata*2, adder2_out);

mux_3to1 mm(idexreaddata1, writedata, exmemaluout, forwarda, m3to1out1);

mux_3to1 mn(idexreaddata2, writedata, exmemaluout, forwardb, m3to1out2);

Mux m2(m3to1out2, ideximmdata, idexalusrc, mux2out);

ALU_64_bit alu(m3to1out1, mux2out, operation, aluout, zero);
ALU_Control alu_c(idexaluop, idexfunct, operation);


exmemreg exmem( clk,reset,
   adder2_out, 
    aluout, 
   zero,
   m3to1out2,
   idexrd, 
   idexbranch,idexmemread, idexmemtoreg, idexmemwrite, idexregwrite,
   addermuxselect,
     exmemadderout,
    exmemzero,
     exmemaluout,
     exmemwritedataout,
     exmemrd,
    exmembranch, exmemmemread, exmemmemtoreg, exmemmemwrite, exmemregwrite,
    branch_finale);

Data_Memory dm(exmemaluout, exmemwritedataout, clk, exmemmemwrite, exmemmemread, datamemoryreaddata,element1,element2,element3,element4,element5,element6,element7,element8);
Mux m1(adder1_out, exmemadderout,(exmembranch&&branch_finale) , pc_in);

memwbreg memwb(clk, reset, datamemoryreaddata, exmemaluout, exmemrd, exmemmemtoreg, exmemregwrite, 
                            memwbreaddataout, memwbaluout, memwbrd, memwbmemtoreg, memwbregwrite);

Mux m3(memwbaluout, memwbreaddataout, memwbmemtoreg, writedata);

forwardingunit fu(idexrs1, idexrs2, exmemrd, memwbrd, memwbregwrite, exmemregwrite, forwarda, forwardb);

branching_unit bu(idexfunct[2:0], idexreaddata1, mux2out, addermuxselect);
endmodule
