// =============================================================================
// LOW LUT MAG UPDATE
// =============================================================================

`timescale 1ns/1ps

module ekf_mag_update #(
    parameter int STATES  = 9,
    parameter int STATE_W = 32,
    parameter int P_W     = 32
)(
    input logic clk,
    input logic rst_n,
    input logic ce_10hz,

    input logic signed [STATE_W-1:0] mag_heading,
    input logic [P_W-1:0] mag_noise,

    input logic signed [STATE_W-1:0] state_in [0:STATES-1],
    input logic signed [P_W-1:0] p_diag_in [0:STATES-1],

    output logic signed [STATE_W-1:0] state_out [0:STATES-1],
    output logic signed [P_W-1:0] p_diag_out [0:STATES-1],
    output logic valid
);

localparam YAW_IDX = 2;
localparam GAIN_SHIFT = 3;

logic signed [STATE_W-1:0] innov;

integer i;

always_ff @(posedge clk or negedge rst_n) begin

    if(!rst_n) begin

        valid <= 0;

        for(i=0;i<STATES;i=i+1) begin
            state_out[i] <= 0;
            p_diag_out[i] <= 32'h00010000;
        end
    end
    else begin

        valid <= ce_10hz;

        for(i=0;i<STATES;i=i+1) begin
            state_out[i] <= state_in[i];
            p_diag_out[i] <= p_diag_in[i];
        end

        if(ce_10hz) begin

            innov = mag_heading - state_in[YAW_IDX];

            state_out[YAW_IDX]
                <= state_in[YAW_IDX] + (innov >>> GAIN_SHIFT);

            p_diag_out[YAW_IDX]
                <= p_diag_in[YAW_IDX]
                   - (p_diag_in[YAW_IDX] >>> GAIN_SHIFT);
        end
    end
end

endmodule
