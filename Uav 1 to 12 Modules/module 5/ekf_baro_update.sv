// =============================================================================
// File        : ekf_baro_update.sv
// Module      : ekf_baro_update
// Description : Barometer scalar altitude EKF measurement update (50 Hz).
//               H = [0,0,0,0,0,0,0,0,1] (observe alt directly)
//               Innovation: y = alt_meas - alt_predicted
//               Joseph form for numerical stability.
// =============================================================================

`timescale 1ns/1ps

module ekf_baro_update #(
    parameter int STATES  = 9,
    parameter int STATE_W = 32,  // Q4.28 (state vector format)
    parameter int P_W     = 32   // Q16.16 (covariance format)
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_50hz,

    // Measurement
    input  logic signed [STATE_W-1:0] baro_alt,   // altitude from barometer Q10.22
    input  logic [P_W-1:0]            baro_noise,  // R (measurement noise)

    // EKF state & covariance
    input  logic signed [STATE_W-1:0] state_in  [0:STATES-1],
    input  logic signed [P_W-1:0]     p_diag_in [0:STATES-1],

    // Updated state & covariance
    output logic signed [STATE_W-1:0] state_out  [0:STATES-1],
    output logic signed [P_W-1:0]     p_diag_out [0:STATES-1],
    output logic                       valid
);

    // H = [0…0,1] observes alt (state index 8)
    localparam int ALT_IDX = 8;

    logic signed [STATE_W-1:0] innov;     // y - H·x
    logic signed [P_W-1:0]     S;         // H·P·Hᵀ + R
    logic signed [P_W-1:0]     K_alt;
    logic signed [63:0]        K_innov;
    logic signed [P_W-1:0]     one_minus_K;
    logic accept_gate;
    logic gate_valid;

    // ---- Innovation ---------------------------------------------------------
    always_comb begin
        innov = baro_alt - state_in[ALT_IDX];
        S     = p_diag_in[ALT_IDX] + $signed({1'b0, baro_noise});
    end

    // ---- Innovation gate ----------------------------------------------------
    innovation_gate #(.DATA_W(STATE_W)) u_gate (
        .clk       (clk), .rst_n (rst_n),
        .valid_in  (ce_50hz),
        .innov     (innov),
        .innov_cov (S),
        .accept    (accept_gate),
        .valid_out (gate_valid)
    );

    // ---- Kalman gain: K_vec[i] = P[i] × H[i] / S  (H[8]=1, H[others]=0)
    //      Only state 8 couples to altitude observation: K[8] = P[8] / S
    // ---- State & covariance update ------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < STATES; s++) begin
                state_out[s]  <= '0;
                p_diag_out[s] <= 32'sh00010000; // 1.0
            end
            valid <= 1'b0;
        end else begin
            valid <= gate_valid;

            if (gate_valid && accept_gate && S != 0) begin
                K_alt = (p_diag_in[ALT_IDX] <<< 16) / S; // Q16.16 gain

                for (int s = 0; s < STATES; s++) begin
                    state_out[s]  <= state_in[s]; // pass-through
                    p_diag_out[s] <= p_diag_in[s];
                end

                // Update only altitude state
                K_innov = K_alt * innov;
                state_out[ALT_IDX]  <= state_in[ALT_IDX] + (K_innov >>> 16);

                // Joseph form: P_new[8] = (1 - K)² × P[8] + K² × R
                one_minus_K = 32'sh00010000 - K_alt; // (1 - K)
                p_diag_out[ALT_IDX] <= ((one_minus_K * one_minus_K) >>> 16) *
                                        (p_diag_in[ALT_IDX] >>> 16) +
                                       ((K_alt * K_alt) >>> 16) *
                                        ($signed({1'b0, baro_noise}) >>> 16);
            end else if (gate_valid) begin
                // Pass-through (gated out or S=0)
                for (int s = 0; s < STATES; s++) begin
                    state_out[s]  <= state_in[s];
                    p_diag_out[s] <= p_diag_in[s];
                end
            end
        end
    end

endmodule