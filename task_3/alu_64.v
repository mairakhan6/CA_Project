
module alu_64
(
    input [63:0] a, b,
    input [3:0] ALUOp,
    output reg [63:0] Result,
    output reg Zero,
    output reg is_greater
);
    localparam [3:0]
    AND = 4'b0000,
    OR	= 4'b0001,
    ADD	= 4'b0010,
    Sub	= 4'b0110,
    NOR = 4'b1100,
    Lesser=4'b0100,
    LeftShift=4'b0111;
    assign ZERO = (Result == 0);
    
    always @ (ALUOp, a, b)
    begin
        case (ALUOp)
            AND: Result = a & b;
            OR:	 Result = a | b;
            ADD: Result = a + b;
            Sub: Result = a - b;
            NOR: Result = ~(a | b);
            Lesser: Result = ( a < b)? 0: 1;
            LeftShift: Result = a << b; // for left shifting all values
            default: Result = 0;
        endcase
         // Check if the Result is zero
        Zero = (Result == 64'd0) ? 1'b1 : 1'b0;

        // Check if a is greater than b
        is_greater = (a > b) ? 1'b1 : 1'b0;
    end
    
endmodule
