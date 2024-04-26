module MEM_WB(
    input clk,                      // Clock sig
    input RegWrite,                 // enabling register write
    input MemtoReg,                 //  ALU resullt for register write
    input [63:0] ReadData,          // Data read from memory or register file
    input [63:0] ALU_result,        // Result of the ALU operation
    input [4:0] destination_reg,    // Destination register for register write

    output reg RegWrite_out,        // enabling reg write
    output reg MemtoReg_out,        // ALU result for register write
    output reg [63:0] ReadData_out, // Output signal for memory or register file
    output reg [63:0] ALU_result_out, // Output signal for the ALU result
    output reg [4:0] destination_reg_out // Output signal for destination register for register write
    // dest reg will be the reg where the finakl result goes
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
