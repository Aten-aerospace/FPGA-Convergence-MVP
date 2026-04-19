// =============================================================================
// File        : sat_rate_limiter.sv
// Module      : sat_rate_limiter
// Description : Per-channel output saturation and rate limiter.
//               Clamps value to [out_min, out_max] and limits the delta
//               between consecutive values to max_delta per cycle.
// =============================================================================

`timescale 1ns/1ps

module sat_rate_limiter #(
    parameter int CHANNELS = 4,
    parameter int DATA_W   = 16  // signed
)(
    input  logic clk,
    input  logic rst_n,
    input  logic enable,

    input  logic signed [DATA_W-1:0] data_in    [0:CHANNELS-1],
    input  logic signed [DATA_W-1:0] out_max    [0:CHANNELS-1],
    input  logic signed [DATA_W-1:0] out_min    [0:CHANNELS-1],
    input  logic signed [DATA_W-1:0] max_delta  [0:CHANNELS-1],

    output logic signed [DATA_W-1:0] data_out   [0:CHANNELS-1],
    output logic [CHANNELS-1:0]      saturated
);

    logic signed [DATA_W-1:0] prev [0:CHANNELS-1];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int c = 0; c < CHANNELS; c++) begin
                data_out[c]  <= '0;
                prev[c]      <= '0;
                saturated[c] <= 1'b0;
            end
        end else if (enable) begin
            for (int c = 0; c < CHANNELS; c++) begin
                logic signed [DATA_W-1:0] delta, rate_limited, sat_val;
                logic sat_flag;

                // Rate limiting
                delta = data_in[c] - prev[c];
                if      (delta > max_delta[c])   rate_limited = prev[c] + max_delta[c];
                else if (delta < -max_delta[c])  rate_limited = prev[c] - max_delta[c];
                else                              rate_limited = data_in[c];

                // Saturation
                sat_flag = 1'b0;
                if      (rate_limited > out_max[c]) begin sat_val = out_max[c]; sat_flag = 1'b1; end
                else if (rate_limited < out_min[c]) begin sat_val = out_min[c]; sat_flag = 1'b1; end
                else                                 sat_val = rate_limited;

                data_out[c]  <= sat_val;
                prev[c]      <= sat_val;
                saturated[c] <= sat_flag;
            end
        end
    end

endmodule
