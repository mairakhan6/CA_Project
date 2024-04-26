`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/19/2024 12:24:50 PM
// Design Name: 
// Module Name: Branch_Control
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

