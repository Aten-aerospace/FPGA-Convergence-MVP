// =============================================================================
// Module: ekf_predict
// Subsystem: CS5 - Extended Kalman Filter
// Description: EKF prediction step. Propagates the full 7-state vector
//              [q0,q1,q2,q3, bx,by,bz] forward one 100 Hz epoch.
//
//   Algorithm:
//     omega_corr[i] = gyro[i] - bias_est[i]         (bias subtraction)
//     dq = [1, ω_x·dt/2, ω_y·dt/2, ω_z·dt/2]       (small-angle dq)
//     q_pred = normalize( q_state ⊗ dq )             (kinematic propagation)
//     bias_pred[i] = bias_est[i]                     (random-walk model: hold)
//
//   DT_HALF = round(0.005 × 32768) = 164 (for 100 Hz, dt = 0.01 s)
//
//   Pipeline: 8-cycle latency (dq compute → multiply 3cy → normalize 3cy → output reg)
//   Bias is passed through with a matching pipeline delay.
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module ekf_predict (
    input  logic        clk,
    input  logic        rst_n,

    // 100 Hz clock enable
    input  logic        ce_100hz,

    // Current quaternion state (Q15)
    input  logic signed [15:0] q_state   [0:3],

    // Gyro measurement from CS1/IMU (Q15 rad/s)
    input  logic signed [15:0] gyro      [0:2],
    input  logic               gyro_valid,

    // Gyro bias estimate (Q15 rad/s) - held constant during predict step
    input  logic signed [15:0] bias_est  [0:2],

    // Predicted state outputs
    output logic signed [15:0] q_pred    [0:3],
    output logic signed [15:0] bias_pred [0:2],   // pass-through with matched delay
    output logic               pred_valid
);

    localparam signed [15:0] DT_HALF = 16'sd164; // 0.005 × 32768

    // =========================================================================
    // Stage 0: bias-correct omega and compute dq components
    // =========================================================================
    logic signed [15:0] omega_corr   [0:2];
    logic signed [31:0] dq_raw       [0:2];
    logic signed [15:0] dq           [0:3];
    logic signed [15:0] q_state_r    [0:3];
    logic signed [15:0] bias_r       [0:2]; // registered bias for pipeline match
    logic               prop_valid_r;

    always_comb begin
        for (int i = 0; i < 3; i++) begin
            omega_corr[i] = gyro[i] - bias_est[i];
            dq_raw[i]     = omega_corr[i] * DT_HALF;
        end
        dq[0] = 16'sh7FFF;
        dq[1] = dq_raw[0][30:15];
        dq[2] = dq_raw[1][30:15];
        dq[3] = dq_raw[2][30:15];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) q_state_r[i] <= '0;
            for (int i = 0; i < 3; i++) bias_r[i]    <= '0;
            prop_valid_r <= 1'b0;
        end else begin
            prop_valid_r <= ce_100hz && gyro_valid;
            if (ce_100hz && gyro_valid) begin
                for (int i = 0; i < 4; i++) q_state_r[i] <= q_state[i];
                for (int i = 0; i < 3; i++) bias_r[i]    <= bias_est[i];
            end
        end
    end

    // =========================================================================
    // Registered dq
    // =========================================================================
    logic signed [15:0] dq_r [0:3];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) dq_r[i] <= '0;
            dq_r[0] <= 16'sh7FFF;
        end else if (ce_100hz && gyro_valid) begin
            for (int i = 0; i < 4; i++) dq_r[i] <= dq[i];
        end
    end

    // =========================================================================
    // Quaternion multiply: q_state ⊗ dq  (3-cycle pipeline)
    // =========================================================================
    logic signed [15:0] q_mul_out  [0:3];
    logic               mul_valid_out;

    quat_multiply u_mul (
        .clk       (clk),
        .rst_n     (rst_n),
        .qa        (q_state_r),
        .qb        (dq_r),
        .valid_in  (prop_valid_r),
        .qout      (q_mul_out),
        .valid_out (mul_valid_out)
    );

    // =========================================================================
    // Quaternion normalize  (3-cycle pipeline)
    // =========================================================================
    logic norm_error_unused;

    quat_normalize u_norm (
        .clk        (clk),
        .rst_n      (rst_n),
        .q_in       (q_mul_out),
        .valid_in   (mul_valid_out),
        .q_out      (q_pred),
        .valid_out  (pred_valid),
        .norm_error (norm_error_unused)
    );

    // =========================================================================
    // Bias pass-through: delay bias_r by 6 cycles to match quat pipeline
    // (mul: 3 cycles, norm: 3 cycles = 6 total after the first register)
    // =========================================================================
    logic signed [15:0] bias_pipe [0:5][0:2];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < 6; s++)
                for (int i = 0; i < 3; i++) bias_pipe[s][i] <= '0;
            for (int i = 0; i < 3; i++) bias_pred[i] <= '0;
        end else begin
            for (int i = 0; i < 3; i++) begin
                bias_pipe[0][i] <= bias_r[i];
                for (int s = 1; s < 6; s++)
                    bias_pipe[s][i] <= bias_pipe[s-1][i];
                if (pred_valid) bias_pred[i] <= bias_pipe[5][i];
            end
        end
    end

endmodule