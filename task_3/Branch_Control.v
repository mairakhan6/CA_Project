module Branch_Control
(
input Branch, Zero, Is_Greater_Than,
input [3:0] funct,
output reg switch, Flush
);


always @(*) begin
// Check if Branch signal is 1 then we swithc branch and flush else not
if (Branch) begin

    // Use a case statement based on funct[2:0] value
    case ({funct[2:0]})

        // Case when funct[2:0] is 3'b000
        3'b000: begin
            // Check if Zero signal is active
            if (Zero)
                switch = 1;  // Set switch_branch to 1
            else
                switch = 0;  // Set switch_branch to 0
        end

        // Case when funct[2:0] is 3'b001
        3'b001: begin
            // Check if Zero signal is active
            if (Zero)
                switch = 0;  // Set switch_branch to 0
            else
                switch = 1;  // Set switch_branch to 1
        end

        // Case when funct[2:0] is 3'b101
        3'b101: begin
            // Check if Is_Greater_Than signal is active
            if (Is_Greater_Than)
                switch = 1;  // Set switch_branch to 1
            else
                switch = 0;  // Set switch_branch to 0
        end

        // Case when funct[2:0] is 3'b100
        3'b100: begin
            // Check if Is_Greater_Than signal is active
            if (Is_Greater_Than)
                switch = 0;  // Set switch_branch to 0
            else
                switch = 1;  // Set switch_branch to 1
        end

        // Default case
        default: switch = 0;  // Set switch_branch to 0

    endcase

end

else
    switch = 0;  // Set switch_branch to 0 if Branch signal is inactive
end

always @(switch) begin
    // Based on the switch_branch value
    if (switch)
        Flush = 1;  // switch_branch is 1
    else
        Flush = 0;  // switch_branch is 0
    
end

endmodule
