module data_extractor (
    input [31:0] instruction,
    output reg [63:0] imm_data
);

  wire [6:0] opcode;  // Wire to hold the opcode 
  assign opcode = instruction[6:0];  // Assign the lower 7 bits to opcode wire

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
