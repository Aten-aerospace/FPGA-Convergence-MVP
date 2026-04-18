// =============================================================================
// File        : antiwindup_clamp.sv
// Module      : antiwindup_clamp
// Description : Anti-windup integrator clamp module.
//               Freezes integration when output is saturated and error
//               would increase saturation (back-calculation method).
//               Per-axis clamp limits configurable from register file.
// =============================================================================

`timescale 1ns/1ps

module antiwindup_clamp #(
    parameter int AXES    = 8,
    parameter int DATA_W  = 16,  // error width (Q4.12)
    parameter int INTEG_W = 32   // integrator width (Q16.16)
)(
    input  logic clk,
    input  logic rst_n,

    // Per-axis integrator values
    input  logic signed [INTEG_W-1:0] integ_in  [0:AXES-1],

    // Anti-windup limits
    input  logic signed [INTEG_W-1:0] integ_max [0:AXES-1],

    // Saturation flags from sat_rate_limiter
    input  logic [AXES-1:0] output_sat,

    // Error polarity (used to decide freeze direction)
    input  logic signed [DATA_W-1:0]  error [0:AXES-1],

    // Clamped integrator out
    output logic signed [INTEG_W-1:0] integ_out [0:AXES-1],
    output logic [AXES-1:0]           clamped
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int a = 0; a < AXES; a++) begin
                integ_out[a] <= '0;
                clamped[a]   <= 1'b0;
            end
        end else begin
            for (int a = 0; a < AXES; a++) begin
                logic freeze;
                // Freeze: output saturated AND error pushes same direction
                freeze = output_sat[a] &&
                         (( (integ_in[a] > integ_max[a])  && (error[a] > 0) ) ||
                          ( (integ_in[a] < -integ_max[a]) && (error[a] < 0) ));

                if (freeze) begin
                    integ_out[a] <= integ_in[a]; // hold
                    clamped[a]   <= 1'b1;
                end else if (integ_in[a] > integ_max[a]) begin
                    integ_out[a] <= integ_max[a];
                    clamped[a]   <= 1'b1;
                end else if (integ_in[a] < -integ_max[a]) begin
                    integ_out[a] <= -integ_max[a];
                    clamped[a]   <= 1'b1;
                end else begin
                    integ_out[a] <= integ_in[a];
                    clamped[a]   <= 1'b0;
                end
            end
        end
    end

endmodule
