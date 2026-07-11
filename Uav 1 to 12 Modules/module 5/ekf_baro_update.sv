// =============================================================================
// LOW LUT BARO UPDATE
// =============================================================================

`timescale 1ns/1ps

module ekf_baro_update #(
    parameter int STATES  = 9,
    parameter int STATE_W = 32,
    parameter int P_W     = 32
)(
    input logic clk,
    input logic rst_n,
    input logic ce_50hz,

    input logic signed [STATE_W-1:0] baro_alt,
    input logic [P_W-1:0] baro_noise,

    input logic signed [STATE_W-1:0] state_in [0:STATES-1],
    input logic signed [P_W-1:0] p_diag_in [0:STATES-1],

    output logic signed [STATE_W-1:0] state_out [0:STATES-1],
    output logic signed [P_W-1:0] p_diag_out [0:STATES-1],
    output logic valid
);

localparam ALT_IDX = 8;
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

        valid <= ce_50hz;

        for(i=0;i<STATES;i=i+1) begin
            state_out[i] <= state_in[i];
            p_diag_out[i] <= p_diag_in[i];
        end

        if(ce_50hz) begin

            innov = baro_alt - state_in[ALT_IDX];

            state_out[ALT_IDX]
                <= state_in[ALT_IDX] + (innov >>> GAIN_SHIFT);

            p_diag_out[ALT_IDX]
                <= p_diag_in[ALT_IDX]
                   - (p_diag_in[ALT_IDX] >>> GAIN_SHIFT);
        end
    end
end

endmodule
