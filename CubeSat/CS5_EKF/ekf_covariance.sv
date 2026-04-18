// =============================================================================
// Module: ekf_covariance
// Subsystem: CS5 - Extended Kalman Filter
// Description: Full 7×7 covariance matrix manager for the 7-state EKF.
//   Handles the predict-step covariance propagation:
//     P_pred[i][j] = P[i][j] + Q_noise[i] × δ(i==j)   (diagonal Q injection)
//   Full Joseph-form update is performed externally (ekf_joseph_update.sv).
//   This module exposes the P diagonal for monitoring and PD enforcement.
//
//   State vector: [q0, q1, q2, q3, bx, by, bz]
//   P dimensions: 7×7 = 49 elements stored in registers.
//
//   Process noise Q (diagonal only, Q15):
//     Q[0..3] = q_q_noise  (quaternion state noise)
//     Q[4..6] = q_b_noise  (gyroscope bias random-walk noise)
//
//   Positive-definite enforcement: P diagonal clamped to [PD_MIN, PD_MAX].
//
//   Pipeline: 1 registered stage (latency = 1 clock cycle).
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module ekf_covariance #(
    // Initial P diagonal values (Q15)
    parameter signed [15:0] P_INIT_Q    = 16'sh0CCC, // 0.1 in Q15
    parameter signed [15:0] P_INIT_BIAS = 16'sh0147, // 0.01 in Q15
    // Saturation limits
    parameter signed [15:0] SAT_MAX     = 16'sh7FFF,
    parameter signed [15:0] SAT_MIN     = 16'sh0001
)(
    input  logic        clk,
    input  logic        rst_n,

    // Trigger predict-step P update
    input  logic        update_valid,

    // Full 7×7 P matrix input (from Joseph update output, or initialization)
    input  logic signed [15:0] P_in    [0:6][0:6],
    input  logic               P_in_valid, // when 1, load P_in; else hold

    // Process noise diagonal (Q15): [q0-q3, bx-bz]
    input  logic signed [15:0] q_q_noise [0:3],  // quaternion process noise
    input  logic signed [15:0] q_b_noise [0:2],  // bias process noise

    // Full 7×7 P matrix output
    output logic signed [15:0] P_out   [0:6][0:6],

    // Diagonal elements for monitoring (Q15)
    output logic signed [15:0] P_diag  [0:6],

    // PD flag: 1 if all diagonal elements ≥ SAT_MIN after update
    output logic               pd_ok,

    output logic               cov_valid
);

    // =========================================================================
    // Internal 7×7 P register bank
    // =========================================================================
    logic signed [15:0] P_reg [0:6][0:6];

    // Predict-step noise injection: build 17-bit saturating sums for diagonal
    logic signed [16:0] p_diag_sum [0:6];
    // Module-scope temporary for pd_ok accumulation
    logic               cov_pd_all_ok;

    logic signed [15:0] q_n_tmp; // temporary for noise selection in comb block

    always_comb begin
        for (int i = 0; i < 7; i++) begin
            q_n_tmp = (i < 4) ? q_q_noise[i] : q_b_noise[i-4];
            p_diag_sum[i] = {P_reg[i][i][15], P_reg[i][i]} + {q_n_tmp[15], q_n_tmp};
        end
    end

    // =========================================================================
    // Registered update with saturation + PD enforcement
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset to initial P = diag([P_INIT_Q×4, P_INIT_BIAS×3])
            for (int i = 0; i < 7; i++)
                for (int j = 0; j < 7; j++)
                    P_reg[i][j] <= (i == j) ?
                        ((i < 4) ? P_INIT_Q : P_INIT_BIAS) : 16'sh0000;
            for (int i = 0; i < 7; i++) P_diag[i] <= (i < 4) ? P_INIT_Q : P_INIT_BIAS;
            for (int i = 0; i < 7; i++)
                for (int j = 0; j < 7; j++) P_out[i][j] <= '0;
            pd_ok     <= 1'b1;
            cov_valid <= 1'b0;
        end else begin
            cov_valid <= 1'b0;

            if (P_in_valid) begin
                // Load updated P from Joseph update result
                for (int i = 0; i < 7; i++)
                    for (int j = 0; j < 7; j++)
                        P_reg[i][j] <= P_in[i][j];
            end

            if (update_valid) begin
                cov_valid    <= 1'b1;
                cov_pd_all_ok = 1'b1;
                // Add process noise to diagonal + enforce PD
                for (int i = 0; i < 7; i++) begin
                    if      (p_diag_sum[i][16:15] == 2'b01)
                        P_reg[i][i] <= SAT_MAX;
                    else if ($signed(p_diag_sum[i][15:0]) > $signed(SAT_MAX))
                        P_reg[i][i] <= SAT_MAX;
                    else if ($signed(p_diag_sum[i][15:0]) < $signed(SAT_MIN)) begin
                        P_reg[i][i]   <= SAT_MIN;
                        cov_pd_all_ok  = 1'b0;
                    end else
                        P_reg[i][i] <= p_diag_sum[i][15:0];
                end
                pd_ok <= cov_pd_all_ok;

                // Latch outputs
                for (int i = 0; i < 7; i++) begin
                    P_diag[i] <= P_reg[i][i];
                    for (int j = 0; j < 7; j++)
                        P_out[i][j] <= P_reg[i][j];
                end
            end
        end
    end

endmodule