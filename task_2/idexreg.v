`timescale 1ns / 1ps

module idexreg(
input clk,
input reset,
input  [63:0]ifidpc_out,readdata1,readdata2, imm,
input  [4:0] rs1, rs2, rd,
input  [3:0] funct3,
input  branch, memread, memtoreg, memwrite, regwrite, alusrc,
input  [1:0] aluop,
output reg [63:0]idexpc_out,idexreaddata1,idexreaddata2, ideximm,
output reg [4:0] idexrs1, idexrs2, idexrd,
output reg [3:0] idexfunct3,
output reg idexbranch, idexmemread, idexmemtoreg, idexmemwrite, idexregwrite, idexalusrc,
output reg [1:0] idexaluop
    );
    
always @(posedge clk) begin
    if (reset == 1'b1)begin
        idexpc_out = 0;
        idexreaddata1 = 0;
        idexreaddata2 = 0;
        ideximm = 0;
        idexrs1 = 0;
        idexrs2 = 0;
        idexrd = 0;
        idexfunct3 = 0;
        idexbranch = 0;
        idexmemread = 0;
        idexmemtoreg = 0;
        idexmemwrite = 0;
        idexregwrite = 0;
        idexalusrc = 0;
        idexaluop = 0;
        end
    else begin
        idexpc_out = ifidpc_out;
        idexreaddata1 = readdata1;
        idexreaddata2 = readdata2;
        ideximm = imm;
        idexrs1 = rs1;
        idexrs2 = rs2;
        idexrd = rd;
        idexfunct3 = funct3;
        idexbranch = branch;
        idexmemread = memread;
        idexmemtoreg = memtoreg;
        idexmemwrite = memwrite;
        idexregwrite = memwrite;
        idexalusrc = alusrc;
        idexaluop = aluop;
        end
        
    end
endmodule
