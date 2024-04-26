`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:21:15 PM
// Design Name: 
// Module Name: Pipelined_Processor
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




















 






module RISC_V_Processor (
    input clk, reset,
    output wire [63:0] reg1, reg2, reg3, reg4, reg5,
    output wire [63:0] ele1, ele2, ele3, ele4, ele5,
    output [1:0] fwdA,fwdB
    
);
    
    // Instruction Memory (IM) to Instruction Fetch/Decode (IFID) wires
    wire [63:0] PC_to_IM;
    wire [31:0] IM_to_IFID;
    
    // Instruction Decode/Decode (IFID) stage outputs
    wire [6:0] opcode_out;
    wire [4:0] rd_out;
    wire [2:0] funct3_out;
    wire [6:0] funct7_out;
    wire [4:0] rs1_out, rs2_out;
    wire Branch_out, MemRead_out, MemtoReg_out, MemWrite_out, ALUSrc_out, RegWrite_out;
    wire Is_Greater_out;
    wire [1:0] ALUOp_out;
    wire [63:0] mux_to_reg;
    wire [63:0] mux_to_pc_in;
    wire [3:0] ALU_C_Operation;
    wire [63:0] ReadData1_out, ReadData2_out;
    wire [63:0] imm_data_out;
    
    // Fixed value wire
    wire [63:0] fixed_4 = 64'd4;
    
    // PC + 4 to Instruction Fetch/Decode (IFID) mux input wire
    wire [63:0] PC_plus_4_to_mux;
    
    // ALU mux output wire
    wire [63:0] alu_mux_out;
    
    // ALU result wire
    wire [63:0] alu_result_out;
    
    // Zero flag wire
    wire zero_out;
    
    // Immediate value wire for the adder
    wire [63:0] imm_to_adder;
    
    // Immediate adder output wire to mux input
    wire [63:0] imm_adder_to_mux;
    
    // Data Memory (DM) read data wire
    wire [63:0] DM_Read_Data_out;
    
    // Program Counter (PC) mux select wire
    wire pc_mux_sel_wire;
    
    // PC write enable wire
    wire PCWrite_out;
    
    // ID/EX stage outputs
    wire IDEX_Branch_out, IDEX_MemRead_out, IDEX_MemtoReg_out, IDEX_MemWrite_out, IDEX_ALUSrc_out, IDEX_RegWrite_out;
    wire [63:0] IDEX_PC_addr, IDEX_ReadData1_out, IDEX_ReadData2_out, IDEX_imm_data_out;
    wire [3:0] IDEX_funct_in;
    wire [4:0] IDEX_rd_out, IDEX_rs1_out, IDEX_rs2_out;
    wire [1:0] IDEX_ALUOp_out;
    
    // EX/MEM stage outputs
    wire EXMEM_Branch_out, EXMEM_MemRead_out, EXMEM_MemtoReg_out, EXMEM_MemWrite_out, EXMEM_RegWrite_out;
    wire EXMEM_zero_out, EXMEM_Is_Greater_out;
    wire [63:0] EXMEM_PC_plus_imm, EXMEM_alu_result_out, EXMEM_ReadData2_out;
    wire [3:0] EXMEM_funct_in;
    wire [4:0] EXMEM_rd_out;
    wire Flush_out;
    
    // MEM/WB stage outputs
    wire MEMWB_MemtoReg_out, MEMWB_RegWrite_out;
    wire [63:0] MEMWB_DM_Read_Data_out, MEMWB_alu_result_out;
    wire [4:0] MEMWB_rd_out;
    
    // IF/ID stage wires
    wire [63:0] IFID_PC_addr;
    wire [31:0] IFID_IM_to_parse;
    wire IFID_Write_out;
    
    // ID/EX stage wires
    wire [3:0] funct_in;
    assign funct_in = {IFID_IM_to_parse[30], IFID_IM_to_parse[14:12]};
    wire control_mux_sel;
    wire [1:0] ALUop_IDEXin;
    wire [63:0] pcplusimm_to_EXMEM;
    wire [1:0] fwd_A_out, fwd_B_out;
    
    // Forwarding multiplexer inputs
    wire [63:0] triplemux_to_a, triplemux_to_b;
    
    assign fwdA = fwd_A_out;
    assign fwdB = fwd_B_out;

assign MemtoReg_IDEXin = control_mux_sel ? MemtoReg_out : 0;    // Select MemtoReg_out if control_mux_sel is true, otherwise assign 0
assign RegWrite_IDEXin = control_mux_sel ? RegWrite_out : 0;    // Select RegWrite_out if control_mux_sel is true, otherwise assign 0
assign Branch_IDEXin = control_mux_sel ? Branch_out : 0;        // Select Branch_out if control_mux_sel is true, otherwise assign 0
assign MemWrite_IDEXin = control_mux_sel ? MemWrite_out : 0;    // Select MemWrite_out if control_mux_sel is true, otherwise assign 0
assign MemRead_IDEXin = control_mux_sel ? MemRead_out : 0;      // Select MemRead_out if control_mux_sel is true, otherwise assign 0
assign ALUSrc_IDEXin = control_mux_sel ? ALUSrc_out : 0;        // Select ALUSrc_out if control_mux_sel is true, otherwise assign 0
assign ALUop_IDEXin = control_mux_sel ? ALUOp_out : 2'b00;      // Select ALUOp_out if control_mux_sel is true, otherwise assign 2'b00

mux2x1 PCsrcmux
(
    .a(EXMEM_PC_plus_imm),  
    .b(PC_plus_4_to_mux),
    .sel(pc_mux_sel_wire),
    .data_out(mux_to_pc_in)
);

Program_Counter ProgC (
    .clk(clk),
    .reset(reset),
    .PCWrite(PCWrite_out),
    .PC_In(mux_to_pc_in),
    .PC_Out(PC_to_IM)
);

Adder PC_Adder
(
    .a(PC_to_IM),
    .b(fixed_4),
    .OUT(PC_plus_4_to_mux)
);

Instruction_Memory INST_Mem
(
    .Inst_Address(PC_to_IM),
    .Instruction(IM_to_IFID)
);

IF_ID IF_ID
(
    .clk(clk),
    .Flush(Flush_out),
    .IFID_Write(IFID_Write_out),
    .PC_addr(PC_to_IM),
    .Instruc(IM_to_IFID),
    .PC_store(IFID_PC_addr),
    .Instr_store(IFID_IM_to_parse)
);


Hazard_Detection HD_Unit
(
    .current_rd(IDEX_rd_out),
    .previous_rs1(rs1_out),
    .previous_rs2(rs2_out),
    .current_MemRead(IDEX_MemRead_out),
    .mux_out(control_mux_sel),
    .enable_Write(IFID_Write_out),
    .enable_PCWrite(PCWrite_out)
);



Instruction_Parser I_Parser
(
    .instruction(IFID_IM_to_parse),
    .opcode(opcode_out),
    .rd(rd_out),
    .funct3(funct3_out),
    .rs1(rs1_out),
    .rs2(rs2_out),
    .funct7(funct7_out)
);

Control_Unit Ctrl_unit
(
    .Opcode(opcode_out),
    .Branch(Branch_out), 
    .MemRead(MemRead_out), 
    .MemtoReg(MemtoReg_out),
    .MemWrite(MemWrite_out), 
    .ALUSrc(ALUSrc_out),
    .Regwrite(RegWrite_out),
    .ALUOp(ALUOp_out)
);

registerFile RegFile
(
    .clk(clk),
    .reset(reset),
    .RegWrite(MEMWB_RegWrite_out), 
    .WriteData(mux_to_reg),
    .RS1(rs1_out),
    .RS2(rs2_out),
    .RD(MEMWB_rd_out),    
    .ReadData1(ReadData1_out),
    .ReadData2(ReadData2_out),
    .r1(reg1),
    .r2(reg2),
    .r3(reg3),
    .r4(reg4),
    .r5(reg5)
    
);


data_extractor Imm_Gen
(
    .instruction(IFID_IM_to_parse),
    .imm_data(imm_data_out)
);


ID_EX ID_EX 
(
    .clk              (clk),
    .Flush              (Flush_out),
    .program_counter_addr (IFID_PC_addr),
    .read_data1          (ReadData1_out),
    .read_data2          (ReadData2_out),
    .immediate_value     (imm_data_out),
    .function_code       (funct_in),
    .destination_reg     (rd_out),
    .source_reg1         (rs1_out),
    .source_reg2         (rs2_out),
    .MemtoReg            (RegWrite_IDEXin),
    .RegWrite             (MemtoReg_IDEXin),
    .Branch                (Branch_IDEXin),
    .MemWrite            (MemWrite_IDEXin),
    .MemRead             (MemRead_IDEXin),
    .ALUSrc              (ALUSrc_IDEXin),
    .ALU_op              (ALUop_IDEXin),

    .program_counter_addr_out       (IDEX_PC_addr),
    .read_data1_out             (IDEX_ReadData1_out),
    .read_data2_out             (IDEX_ReadData2_out),
    .immediate_value_out     (IDEX_imm_data_out),
    .function_code_out    (IDEX_funct_in),
    .destination_reg_out        (IDEX_rd_out),
    .source_reg1_out           (IDEX_rs1_out),
    .source_reg2_out            (IDEX_rs2_out),
    .MemtoReg_out               (IDEX_RegWrite_out),
    .RegWrite_out               (IDEX_MemtoReg_out),
    .Branch_out                 (IDEX_Branch_out),
    .MemWrite_out               (IDEX_MemWrite_out),
    .MemRead_out               (IDEX_MemRead_out),
    .ALUSrc_out                 (IDEX_ALUSrc_out),
    .ALU_op_out                 (IDEX_ALUOp_out)

);

ALU_Control ALU_Control1
(
    .ALUOp(IDEX_ALUOp_out),
    .Funct(IDEX_funct_in),
    .Operation(ALU_C_Operation)
);


mux2x1 ALU_mux
(
    .a(IDEX_imm_data_out), //value when sel is 1
    .b(triplemux_to_b),
    .sel(IDEX_ALUSrc_out),
    .data_out(alu_mux_out)
);



mux3x1 mux_forw_a
(
    .a(IDEX_ReadData1_out), 
    .b(mux_to_reg), 
    .c(EXMEM_alu_result_out),   
    .sel(fwd_A_out),
    .data_out(triplemux_to_a)  
);

mux3x1 mux_forw_b
(
    .a(IDEX_ReadData2_out), 
    .b(mux_to_reg), 
    .c(EXMEM_alu_result_out),   
    .sel(fwd_B_out),
    .data_out(triplemux_to_b)  
);

alu_64 ALU64
(
    .a(triplemux_to_a),
    .b(alu_mux_out), 
    .ALUOp(ALU_C_Operation),
    .Result(alu_result_out),
    .Zero(zero_out),
    .is_greater(Is_Greater_out)
);



Forwarding_Unit FWD_Unit
(
    .EXMEM_rd(EXMEM_rd_out),
    .MEMWB_rd(MEMWB_rd_out),
    .IDEX_rs1(IDEX_rs1_out),
    .IDEX_rs2(IDEX_rs2_out),
    .EXMEM_RegWrite(EXMEM_RegWrite_out),
    .EXMEM_MemtoReg(EXMEM_MemtoReg_out),
    .MEMWB_RegWrite(MEMWB_RegWrite_out),
    .fwd_A(fwd_A_out),
    .fwd_B(fwd_B_out)
    
);


Adder ProgCounter_plus_imm
(
    .a(IDEX_PC_addr),
    .b(imm_to_adder),
    .OUT(pcplusimm_to_EXMEM)
);

EX_MEM EX_MEM
(
    .clk(clk),
    .Flush(Flush_out),
    .RegWrite(IDEX_RegWrite_out),
    .MemtoReg(IDEX_MemtoReg_out),
    .Branch(IDEX_Branch_out),
    .Zero(zero_out),
    .is_greater(Is_Greater_out),
    .MemWrite(IDEX_MemWrite_out),
    .MemRead(IDEX_MemRead_out),
    .immvalue_added_pc(pcplusimm_to_EXMEM),
    .ALU_result(alu_result_out),
    .WriteData(triplemux_to_b),
    .function_code(IDEX_funct_in),
    .destination_reg(IDEX_rd_out),

    .RegWrite_out(EXMEM_RegWrite_out),
    .MemtoReg_out(EXMEM_MemtoReg_out),
    .Branch_out(EXMEM_Branch_out),
    .Zero_out(EXMEM_zero_out),
    .is_greater_out(EXMEM_Is_Greater_out),
    .MemWrite_out(EXMEM_MemWrite_out),
    .MemRead_out(EXMEM_MemRead_out),
    .immvalue_added_pc_out(EXMEM_PC_plus_imm),
    .ALU_result_out(EXMEM_alu_result_out),
    .WriteData_out(EXMEM_ReadData2_out),
    .ALU_OP(EXMEM_funct_in),
    .dest_reg_out(EXMEM_rd_out)
);



Branch_Control Branchctrl_unit
(
    .Branch(EXMEM_Branch_out),
    .Flush(Flush_out),
    .Zero(EXMEM_zero_out),
    .greater_than(EXMEM_Is_Greater_out),
    .funct(EXMEM_funct_in),
    .Switch_Branch(pc_mux_sel_wire)
);


Data_Memory Datamem
(
	EXMEM_alu_result_out,
	EXMEM_ReadData2_out,
	clk,EXMEM_MemWrite_out,EXMEM_MemRead_out,
	DM_Read_Data_out
, 
ele1,
ele2,
ele3,
ele4,
ele5
);


MEM_WB Memory_WB
(
    .clk(clk),
    .RegWrite(EXMEM_RegWrite_out),
    .MemtoReg(EXMEM_MemtoReg_out),
    .ReadData(DM_Read_Data_out),
    .ALU_result(EXMEM_alu_result_out),
    .destination_reg(EXMEM_rd_out),

    .RegWrite_out(MEMWB_RegWrite_out),
    .MemtoReg_out(MEMWB_MemtoReg_out),
    .ReadData_out(MEMWB_DM_Read_Data_out),
    .ALU_result_out(MEMWB_alu_result_out),
    .destination_reg_out(MEMWB_rd_out)

);


mux2x1 mux2
(
    .a(MEMWB_DM_Read_Data_out), //value when sel is 1
    .b(MEMWB_alu_result_out),
    .sel(MEMWB_MemtoReg_out),
    .data_out(mux_to_reg)
);


endmodule
