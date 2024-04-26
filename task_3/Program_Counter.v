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
        else if (!PCWrite) // holding the last instruction if PCwrite is low
        begin
            PC_Out <= PC_Out; // Hold the current value of PC_Out if PCWrite is not enabled
        end
        else
        begin
            PC_Out <= PC_In; // Update PC_Out with the value from PC_In when PCWrite is enabled
        end
    end

endmodule
