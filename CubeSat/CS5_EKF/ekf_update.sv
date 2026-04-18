// =============================================================================
// Module: ekf_update
// Subsystem: CS5 - Extended Kalman Filter
// Description: EKF measurement update step.
//   Orchestrates the full 7-state Kalman update by sequencing:
//     1. ekf_measurement_model  - computes h(q) and 6×7 H Jacobian (3 cycles)
//     2. ekf_joseph_update      - sequential scalar rank-1 P updates (FSM)
//
//   Inputs:  predicted state [q0-q3, bx-bz], P_in 7×7, raw measurements
//   Outputs: updated state x_est[0:6], P updated (via ekf_joseph_update),
//            P_diag[0:6], K_gain[0:6][0:5], innovation[0:5], valid
//
//   The module accepts pred_valid from ekf_predict, waits for meas_valid,
//   then fires ekf_joseph_update.start once measurements and h/H are ready.
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module ekf_update (
    input  logic        clk,
    input  logic        rst_n,

    // Predicted 7-state from ekf_predict (Q15)
    input  logic signed [15:0] x_pred   [0:6],  // [q0-q3, bx-bz]
    input  logic               pred_valid,

    // Predicted P matrix (7×7, Q15)
    input  logic signed [15:0] P_pred   [0:6][0:6],

    // Magnetometer measurement (Q15, normalised body-frame)
    input  logic signed [15:0] mag_meas  [0:2],
    input  logic               mag_valid,

    // Accelerometer measurement (Q15, normalised body-frame)
    input  logic signed [15:0] accel_meas [0:2],

    // Estimated 7-state output (Q15)
    output logic signed [15:0] x_est     [0:6],
    output logic               est_valid,

    // Covariance diagonal output (Q15)
    output logic signed [15:0] P_diag    [0:6],

    // Kalman gain matrix (Q15)
    output logic signed [15:0] K_gain    [0:6][0:5],

    // Innovation vector (Q15)
    output logic signed [15:0] innovation [0:5],

    // PD / sigma flags
    output logic               pd_ok,
    output logic               sigma_ok
);

    // =========================================================================
    // Build 6-element measurement vector z = [accel_meas, mag_meas] (Q15)
    // =========================================================================
    logic signed [15:0] z_meas [0:5];
    always_comb begin
        for (int i = 0; i < 3; i++) z_meas[i]   = accel_meas[i];
        for (int i = 0; i < 3; i++) z_meas[i+3] = mag_meas[i];
    end

    // =========================================================================
    // Measurement model: h(q) and H Jacobian
    // Uses only the quaternion part of x_pred (indices 0-3)
    // =========================================================================
    logic signed [15:0] q_in_for_model [0:3];
    always_comb begin
        for (int i = 0; i < 4; i++) q_in_for_model[i] = x_pred[i];
    end

    logic signed [15:0] h_meas   [0:5];
    logic signed [15:0] H_mat    [0:5][0:6];
    logic               hm_valid;

    // Fire measurement model when predicted state is valid
    logic               model_start;
    logic               meas_valid;

    assign meas_valid  = pred_valid && mag_valid;
    assign model_start = meas_valid;

    ekf_measurement_model u_meas_model (
        .clk       (clk),
        .rst_n     (rst_n),
        .valid_in  (model_start),
        .q_in      (q_in_for_model),
        .h_meas    (h_meas),
        .H_mat     (H_mat),
        .valid_out (hm_valid)
    );

    // =========================================================================
    // Latch x_pred and P_pred when measurement model fires (pipeline match)
    // Pipeline: 3 cycles in measurement model → need to hold x_pred/P_pred
    // =========================================================================
    logic signed [15:0] x_pred_d [0:6];
    logic signed [15:0] P_pred_d [0:6][0:6];
    logic signed [15:0] z_pred_d [0:5];
    logic [1:0]         delay_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            delay_cnt <= '0;
            for (int i = 0; i < 7; i++) x_pred_d[i] <= '0;
            for (int i = 0; i < 7; i++)
                for (int j = 0; j < 7; j++) P_pred_d[i][j] <= '0;
            for (int i = 0; i < 6; i++) z_pred_d[i] <= '0;
        end else begin
            // Delay inputs by 3 cycles to match measurement model pipeline
            if (meas_valid) begin
                x_pred_d  <= x_pred;
                P_pred_d  <= P_pred;
                z_pred_d  <= z_meas;
                delay_cnt <= 2'd0;
            end else if (delay_cnt < 2'd3) begin
                delay_cnt <= delay_cnt + 2'd1;
            end
        end
    end

    // =========================================================================
    // Sequential Kalman update FSM (ekf_joseph_update)
    // =========================================================================
    logic joseph_start;
    assign joseph_start = hm_valid; // fire when h/H ready (pipeline matched)

    ekf_joseph_update u_joseph (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (joseph_start),
        .x_in       (x_pred_d),
        .P_in       (P_pred_d),
        .h_meas     (h_meas),
        .H_mat      (H_mat),
        .z_meas     (z_pred_d),
        .x_est      (x_est),
        .P_diag     (P_diag),
        .K_gain     (K_gain),
        .innovation (innovation),
        .pd_ok      (pd_ok),
        .sigma_ok   (sigma_ok),
        .valid_out  (est_valid)
    );

endmodule