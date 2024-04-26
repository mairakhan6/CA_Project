module mux2x1
(
    input [63:0] a,b,
    input sel ,
    output [63:0] data_out
);

assign data_out = sel ? a : b; //select b or a based on the sel bit

endmodule 