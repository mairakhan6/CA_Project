
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
        // Reset stored values
        PC_store <= 0;
        Instr_store <= 0;
    end else if (!IFID_Write) begin
        // Preserve stored values
        PC_store <= PC_store;
        Instr_store <= Instr_store;
    end else begin
        // IF/ID pipeline registers
        PC_store <= PC_addr;
        Instr_store <= Instruc;
    end
end

endmodule
