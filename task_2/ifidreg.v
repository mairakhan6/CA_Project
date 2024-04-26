`timescale 1ns / 1ps


module ifidreg(
input clk,
input reset,
input [63:0] pc_out,
input [31:0] instruction,
output reg [63:0] ifidpc_out,
output reg [31:0] ifidinst);

always @(posedge clk) begin
    if (reset == 1'b1) begin
        ifidpc_out = 0;
        ifidinst = 0;
        end
    else begin
        ifidpc_out = pc_out;
        ifidinst = instruction;
        end
end

endmodule
