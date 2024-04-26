module Adder(
    input [63:0] a, b,
    output reg [63:0] OUT
);
always@(*)
    OUT = a + b; // Whenever used add the 2 inputs a and b to the output
endmodule

module alu_64
(
    input [63:0] a, b,
    input [3:0] ALUOp,
    output reg [63:0] Result,
    output reg Zero,
    output reg is_greater
);
    localparam [3:0]
    AND = 4'b0000, // availibility of operands in the ALU
    OR	= 4'b0001,
    ADD	= 4'b0010,
    Sub	= 4'b0110,
    NOR = 4'b1100,
    Lesser=4'b0100,
    LeftShift=4'b0111;
    
    always @ (ALUOp, a, b) // put forward the output whenever aluop or the 2 inputs change
    begin
        case (ALUOp)
            AND: Result = a & b;
            OR:	 Result = a | b;
            ADD: Result = a + b;
            Sub: Result = a - b;
            NOR: Result = ~(a | b);
            Lesser: Result = ( a < b)? 0: 1;
            LeftShift: Result = a << b;
            default: Result = 0; // result will be zero if there is an invalid operand
        endcase
         // if result is zero, Zeero signal will be high for other uses
        Zero = (Result == 64'd0) ? 1'b1 : 1'b0;

        // Check if a is greater than b and update the extra signal
        is_greater = (a > b) ? 1'b1 : 1'b0;
    end
    
endmodule

module ALU_Control
(
    input [1:0] ALUOp, // 2 bit ALUOPcode
    input [3:0] Funct, // 4 bit func to detemine what type of operation will be performed
    output reg [3:0] Operation // 4 bit Output operation
);
always @(*)
begin
    case(ALUOp)
        2'b00: // ALUOp = 00, R-type
        begin
            Operation = 4'b0010; // ALU operation: Add
        end
        
        2'b01: // ALUOp = 01, Branch type
        begin
            case(Funct[2:0])
                3'b000: // beq when func3 = 000
                begin
                    Operation = 4'b0110; // ALU operation: Subtract
                end
                
                3'b100: // blt whne func3 = 100
                begin
                    Operation = 4'b0100; // ALU operation Less than
                end
            endcase
        end

        2'b10: // ALUOp = 10, immediate type instructions
        begin
            case(Funct)
                4'b0000: // Funct = 0000
                begin
                    Operation = 4'b0010; // Immediate Add
                end
                
                4'b1000: // Funct = 1000
                begin
                    Operation = 4'b0110; // Immediate Subtract
                end
                
                4'b0111: // Funct = 0111
                begin
                    Operation = 4'b0000; // ALU operation: AND
                end
                
                4'b0110: // Funct = 0110
                begin
                    Operation = 4'b0001; // ALU operation: OR
                end
            endcase
        end
    endcase
end

endmodule

module Branch_Control
(
    input Branch, Zero, greater_than,
    input [3:0] funct,
    output reg Switch_Branch, Flush
);


always @(*) begin
    // Check if Branch signal is active
    if (Branch) begin

        // Case on funct[2:0]
        case ({funct[2:0]})

            // when funct[2:0] is 000
            3'b000: begin
                // Check if Zero signal is active
                if (Zero)
                    Switch_Branch = 1;  // Set Switch_Branch_branch to 1
                else
                    Switch_Branch = 0;  // Set Switch_Branch_branch to 0
            end

            // when funct[2:0] is 001
            3'b001: begin
                // Check if Zero signal is active
                if (Zero)
                    Switch_Branch = 0;  // Set Switch_Branch_branch to 0
                else
                    Switch_Branch = 1;  // Set Switch_Branch_branch to 1
            end

            //  when funct[2:0] is 101
            3'b101: begin
                // Check if greater_than signal is active
                if (greater_than)
                    Switch_Branch = 1;  // Set Switch_Branch_branch to 1
                else
                    Switch_Branch = 0;  // Set Switch_Branch_branch to 0
            end

            // when funct[2:0] is 100
            3'b100: begin
                // Check if greater_than signal is active
                if (greater_than)
                    Switch_Branch = 0;  // Set Switch_Branch_branch to 0
                else
                    Switch_Branch = 1;  // Set Switch_Branch_branch to 1
            end

            // Default case
            default: Switch_Branch = 0;  // Set Switch_Branch_branch to 0

        endcase
    
    end
    
    else
        Switch_Branch = 0;  // Set Switch_Branch_branch to 0 if Branch signal is inactive
end

always @(Switch_Branch) begin
    // Synchronise switch branch value with flush signal : Flush is used to flush the instruction in all stages
    if (Switch_Branch)
        Flush <= 1;  
    else
        Flush <= 0; 
    
end

endmodule

module Control_Unit
(
    input [6:0] Opcode,
    output reg [1:0] ALUOp,
    output reg Branch, MemRead, MemtoReg, MemWrite, ALUSrc, Regwrite
);
always @(*)
begin
    case (Opcode)
    7'b0110011: // R-type (add/sub)
        begin
            ALUSrc = 1'b0;
            MemtoReg = 1'b0;
            Regwrite = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b10;
        end
    7'b0000011: // I-type (ld)
        begin
            ALUSrc = 1'b1;
            MemtoReg = 1'b1;
            Regwrite = 1'b1;
            MemRead = 1'b1;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b00;
        end
    7'b0100011: // S-type(sd)
        begin
            ALUSrc = 1'b1;
            MemtoReg = 1'bx;
            Regwrite = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b1;
            Branch = 1'b0;
            ALUOp = 2'b00;
        end
    7'b0010011: // I-type (addi)
        begin
            ALUSrc = 1'b1;
            MemtoReg = 1'b0;
            Regwrite = 1'b1;
            MemRead = 1'b1;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b00;
        end
    7'b1100011: // SB-type (beq/bne/bge)
        begin
            ALUSrc = 1'b0;
            MemtoReg = 1'bx;
            Regwrite = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            Branch = 1'b1;
            ALUOp = 2'b01;
        end
    default: begin
            ALUSrc = 1'b0;
            MemtoReg = 1'b0;
            Regwrite = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
            Branch = 1'b0;
            ALUOp = 2'b00;
    end
    endcase
end
endmodule

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

module EX_MEM(
    input clk,                     // Clock 
    input Flush,                   // Flush control 
    input RegWrite,                
    input MemtoReg,                // Control signal for selecting memory or ALU result for register write
    input Branch,                  // Branch Control SIgnal
    input Zero,                    // Control singal indicating the ALU result is zero
    input MemWrite,                
    input MemRead,                 
    input is_greater,              // Control signal indicating the comparison result of the ALU operation
    input [63:0] immvalue_added_pc, // Immediate value added to the program counter
    input [63:0] ALU_result,       // Result of the ALU operation
    input [63:0] WriteData,        // Data to be written to memory or register file
    input [3:0] function_code,     // Function code for ALU operation
    input [4:0] destination_reg,   // Destination register for register write

    output reg RegWrite_out,       // register write
    output reg MemtoReg_out,       // MEM or ALU result for register write
    output reg Branch_out,         // branch signal
    output reg Zero_out,           // signal for ALU result is zero
    output reg MemWrite_out,       // MEM write
    output reg MemRead_out,        // MEM read
    output reg is_greater_out,     // when a greater than check for comparison
    output reg [63:0] immvalue_added_pc_out, // immediate value added to the program counter
    output reg [63:0] ALU_result_out,       // ALU result
    output reg [63:0] WriteData_out,        // Write data
    output reg [3:0] ALU_OP,     // ALU operation code
    output reg [4:0] dest_reg_out    // destination register for reg write operation
);

    // Assign output values based on control signals
    always @(posedge clk) begin
        if (Flush) begin
            // SET every signal and every output to 0 if flush signaled
            RegWrite_out = 0;
            MemtoReg_out = 0;
            Branch_out = 0;
            Zero_out = 0;
            is_greater_out = 0;
            MemWrite_out = 0;
            MemRead_out = 0;
            immvalue_added_pc_out = 0;
            ALU_result_out = 0;
            WriteData_out = 0;
            ALU_OP = 0;
            dest_reg_out = 0;
        end 
        else begin
            // Assign output values based on input signals
            RegWrite_out = RegWrite;
            MemtoReg_out = MemtoReg;
            Branch_out = Branch;
            Zero_out = Zero;
            is_greater_out = is_greater;
            MemWrite_out = MemWrite;
            MemRead_out = MemRead;
            immvalue_added_pc_out = immvalue_added_pc;
            ALU_result_out = ALU_result;
            WriteData_out = WriteData;
            ALU_OP = function_code;
            dest_reg_out = destination_reg;
        end
    end

endmodule

module Forwarding_Unit
(
    input [4:0] EXMEM_rd, MEMWB_rd,
    input [4:0] IDEX_rs1, IDEX_rs2,
    input EXMEM_RegWrite, EXMEM_MemtoReg,
    input MEMWB_RegWrite,

    output reg [1:0] fwd_A, fwd_B
);

always @(*) begin
    // Forwarding logic for operand A
    if (EXMEM_rd == IDEX_rs1 /*&& EXMEM_RegWrite*/ && EXMEM_rd != 0) begin
        fwd_A = 2'b10;  // Forward value from the EX/MEM pipeline stage
    end else if ((MEMWB_rd == IDEX_rs1) && MEMWB_RegWrite && (MEMWB_rd != 0) &&
               !(EXMEM_RegWrite && (EXMEM_rd != 0) && (EXMEM_rd == IDEX_rs1))) begin
        fwd_A = 2'b01;  // Forward value from the MEM/WB pipeline stage
    end else begin
        fwd_A = 2'b00;  // No forwarding for operand A
    end
    
    // Forwarding logic for operand B
    if ((EXMEM_rd == IDEX_rs2) /*&& EXMEM_RegWrite*/ && EXMEM_rd != 0) begin
        fwd_B = 2'b10;  // Forward value from the EX/MEM pipeline stage
    end else if ((MEMWB_rd == IDEX_rs2) && (MEMWB_RegWrite == 1) && (MEMWB_rd != 0) &&
               !(EXMEM_RegWrite && (EXMEM_rd != 0) && (EXMEM_rd == IDEX_rs2))) begin
        fwd_B = 2'b01;  // Forward value from the MEM/WB pipeline stage
    end else begin
        fwd_B = 2'b00;  // No forwarding for operand B
    end
end

endmodule

module Hazard_Detection
(
    input [4:0] current_rd, previous_rs1, previous_rs2,
    input current_MemRead,
    output reg mux_out,
    output reg enable_Write, enable_PCWrite
);
initial begin
    if (current_MemRead && (current_rd == previous_rs1 || current_rd == previous_rs2)) begin
        // Hazard detected: Set control signals accordingly
        mux_out = 1;             // Disable the multiplexer output
        enable_Write = 1;       // Disable write to the next pipeline stage
        enable_PCWrite = 1;    // Disable PC write
    end else begin
        // No hazard detected: Set control signals accordingly
        mux_out = 1;             // Enable the multiplexer output
        enable_Write = 1;       // Enable write to the next pipeline stage
        enable_PCWrite = 1;    // Enable PC write
    end
end

endmodule

module ID_EX(
    input        clk,                            // Clock signal
    input        Flush,                          // Flush control signal
    input [63:0] program_counter_addr,    // Program counter address input
    input [63:0] read_data1,               // Data 1 input
    input [63:0] read_data2,               // Data 2 input
    input [63:0] immediate_value,          // Immediate value input
    input [3:0]  function_code,             // Function code input
    input [4:0]  destination_reg,           // Destination register input
    input [4:0]  source_reg1,                // Source register 1 input
    input [4:0]  source_reg2,               // Source register 2 input
    input        MemtoReg,                        // Memory-to-register control signal
    input        RegWrite,                        // Register write control signal
    input        Branch,                           // Branch control signal
    input        MemWrite,                         // Memory write control signal
    input        MemRead,                          // Memory read control signal
    input        ALUSrc,                           // ALU source control signal
    input [1:0]  ALU_op,                     // ALU operation control signal

    output reg [63:0] program_counter_addr_out,    // Output: Stored program counter address
    output reg [63:0] read_data1_out,               // Output: Stored Data 1
    output reg [63:0] read_data2_out,               // Output: Stored Data 2
    output reg [63:0] immediate_value_out,          // Output: Stored Immediate value
    output reg [3:0] function_code_out,             // Output: Stored Function code
    output reg [4:0] destination_reg_out,           // Output: Stored Destination register
    output reg [4:0] source_reg1_out,                     // Output: Stored Source register 1
    output reg [4:0] source_reg2_out,                     // Output: Stored Source register 2
    output reg       MemtoReg_out,                        // Output: Stored Memory-to-register control
    output reg       RegWrite_out,                            // Output: Stored Register write control
    output reg Branch_out,                               // Output: Stored Branch control
    output reg MemWrite_out,                            // Output: Stored Memory write control
    output reg MemRead_out,                         // Output: Stored Memory read control
    output reg ALUSrc_out,                               // Output: Stored ALU source control
    output reg [1:0] ALU_op_out                     // Output: Stored ALU operation control
    
);

always @(posedge clk) begin
    if (Flush) 
    begin
    // Reset all output registers to 0
    program_counter_addr_out = 0;
    read_data1_out = 0;
    read_data2_out = 0;
    immediate_value_out = 0;
    function_code_out = 0;
    destination_reg_out = 0;
    source_reg1_out = 0;
    source_reg2_out = 0;
    MemtoReg_out = 0;
    RegWrite_out = 0;
    Branch_out = 0;
    MemWrite_out = 0;
    MemRead_out = 0;
    ALUSrc_out = 0;
    ALU_op_out = 0;
    end 
    
    else 
    begin
        // Pass input values to output registers
    program_counter_addr_out = program_counter_addr;
    read_data1_out = read_data1;
    read_data2_out = read_data2;
    immediate_value_out = immediate_value;
    function_code_out = function_code;
    destination_reg_out = destination_reg;
    source_reg1_out = source_reg1;
    source_reg2_out = source_reg2;
    RegWrite_out = RegWrite;
    MemtoReg_out = MemtoReg;
    Branch_out = Branch;
    MemWrite_out = MemWrite;
    MemRead_out = MemRead;
    ALUSrc_out = ALUSrc;
    ALU_op_out = ALU_op;
    end
end

endmodule 

module IF_ID(
    input clk, IFID_Write, Flush,
    input [63:0] PC_addr,
    input [31:0] Instruc,
    output reg [63:0] PC_store,
    output reg [31:0] Instr_store
);

always @(posedge clk) begin
    // Check if Flush signal is active
    if (Flush) begin
        // Flush active: Reset stored values
        PC_store <= 0;
        Instr_store <= 0;
    end else if (!IFID_Write) begin
        // IFID_Write inactive: Preserve stored values
        PC_store <= PC_store;
        Instr_store <= Instr_store;
    end else begin
        // Store new values in IF/ID pipeline registers
        PC_store <= PC_addr;
        Instr_store <= Instruc;
    end
end

endmodule

module data_extractor (
    input [31:0] instruction,
    output reg [63:0] imm_data
);

  wire [6:0] opcode;  // Wire to hold the opcode extracted from the instruction
  assign opcode = instruction[6:0];  // Assign the lower 7 bits of the instruction to the opcode wire

  always @(*)  
  begin
      case (opcode)
          7'b0000011: imm_data =  {{52{instruction[31]}}, instruction[31:20]};  // I-type instruction with 12-bit immediate
          7'b0100011: imm_data = {{52{instruction[31]}}, instruction[31:25], instruction[11:7]};  // S-type instruction with 12-bit immediate
          7'b1100011: imm_data = {{52{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8]};  // B-type instruction with 13-bit immediate
          7'b0010011: imm_data = {{52{instruction[31]}}, instruction[31:20]};  // I-type instruction with 12-bit immediate
          default : imm_data = 64'd0;  // No immediate value for other opcode values
      endcase
  end

endmodule

module Instruction_Parser(
    input [31:0] instruction,
    output [6:0] opcode, funct7,
    output [4:0] rd , rs1 , rs2,
    output [2:0] funct3 

);

    assign opcode = instruction[6:0];
    assign rd = instruction[11:7];
    assign funct3 = instruction[14:12];
    assign rs1 = instruction[19:15];
    assign rs2 = instruction[24:20];
    assign funct7 = instruction[31:25];
    
    endmodule

module Instruction_Memory
(
	input [63:0] Inst_Address,
	output reg [31:0] Instruction
);
	reg [7:0] inst_mem [63:0];
	
	initial begin
    inst_mem[0] = 8'b00110011; //add x10,x12,x13
    inst_mem[1] = 8'b00000101;
    inst_mem[2] = 8'b11010110;
    inst_mem[3] = 8'b0;
    
    inst_mem[4] = 8'b00110011; //add x8,x10,x12
    inst_mem[5] = 8'b00000100;
    inst_mem[6] = 8'b11000101;
    inst_mem[7] = 8'b0;
    
//    inst_mem[8] = 8'b10110011; //sub x1,x8,x10
//    inst_mem[9] = 8'b00000000;
//    inst_mem[10] = 8'b10010100;
//    inst_mem[11] = 8'b01000000;

    inst_mem[8] = 8'b10110011; //add x1,x8,x10
    inst_mem[9] = 8'b00000000;
    inst_mem[10] = 8'b10100100;
    inst_mem[11] = 8'b00000000;
    
    
    
	end
	
	always @(Inst_Address)
	begin
		Instruction={inst_mem[Inst_Address+3],inst_mem[Inst_Address+2],inst_mem[Inst_Address+1],inst_mem[Inst_Address]};
	end
endmodule

module MEM_WB(
    input clk,                      // Clock signal
    input RegWrite,                 // Control signal for enabling register write
    input MemtoReg,                 // Control signal for selecting memory or ALU result for register write
    input [63:0] ReadData,          // Data read from memory or register file
    input [63:0] ALU_result,        // Result of the ALU operation
    input [4:0] destination_reg,    // Destination register for register write

    output reg RegWrite_out,        // Output signal for enabling register write
    output reg MemtoReg_out,        // Output signal for selecting memory or ALU result for register write
    output reg [63:0] ReadData_out, // Output signal for data read from memory or register file
    output reg [63:0] ALU_result_out, // Output signal for the ALU result
    output reg [4:0] destination_reg_out // Output signal for destination register for register write
);

    // Assign output values based on input signals
    always @(posedge clk) begin
        RegWrite_out = RegWrite;
        MemtoReg_out = MemtoReg;
        ReadData_out = ReadData;
        ALU_result_out = ALU_result;
        destination_reg_out = destination_reg;
    end

endmodule

module mux2x1
(
    input [63:0] a,b,
    input sel ,
    output [63:0] data_out
);

assign data_out = sel ? a : b; //select b or a based on the sel bit

endmodule 

module mux3x1(
    input [63:0] a, b, c,
    input [1:0] sel,
    output reg [63:0] data_out   
);

always @(*) begin
    if (sel == 2'b01) begin    // If sel is 01, select input B
        data_out = b;
    end
    else if (sel == 2'b00) begin    // If sel is 00, select input A
        data_out = a;
    end
    else if (sel == 2'b10) begin    // If sel is 10, select input C
        data_out = c;
    end
    else begin    // For all other cases, output X (undefined)
        data_out = 2'bX;
    end
end


endmodule

module Program_Counter 
(
    input clk, reset, PCWrite,
    input [63:0] PC_In,
    output reg [63:0] PC_Out
);

    reg reset_force; // Variable to force 0th value after reset

    initial
    begin
        PC_Out <= 64'd0; // Initialize PC_Out to 0
    end
    
    always @(negedge reset)
    begin
        reset_force <= 1; // Set reset_force to 1 on the falling edge of reset signal
    end
    
    always @(posedge clk or posedge reset)
    begin
        if (reset || reset_force)
        begin
            PC_Out <= 64'd0; // Reset PC_Out to 0 when reset or reset_force is active
            reset_force <= 0; // Reset the reset_force variable
        end
        else if (!PCWrite)
        begin
            PC_Out <= PC_Out; // Hold the current value of PC_Out if PCWrite is not enabled
        end
        else
        begin
            PC_Out <= PC_In; // Update PC_Out with the value from PC_In when PCWrite is enabled
        end
    end

endmodule

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

endmodule // registerFile

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