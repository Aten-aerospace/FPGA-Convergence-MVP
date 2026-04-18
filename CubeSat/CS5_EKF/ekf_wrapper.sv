// =============================================================================
// Module: ekf_wrapper (CS5 top-level wrapper)
// Subsystem: CS5 - Extended Kalman Filter
// Description: 7-state EKF for CubeSat attitude estimation.
//              Thin wrapper around ekf_core, adding P_diag and innovation
//              diagnostic outputs for ground-station telemetry.
//
//   States: q[0..3] (unit quaternion) + bias[0..2] (gyro bias in rad/s).
//   Extended outputs: P_diag[0:6] (covariance diagonal) and
//                     innovation[0:6] (measurement residuals, index 6 unused).
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module ekf_wrapper #(
    parameter int CLK_HZ       = 100_000_000,
    parameter int FAULT_CYCLES = 200_000      // 2 ms watchdog
)(
    input  logic        clk,
    input  logic        rst_n,

    // 100 Hz clock enable from CS12
    input  logic        ce_100hz,

    // Sensor inputs (Q15)
    input  logic signed [15:0] accel    [0:2],
    input  logic signed [15:0] gyro     [0:2],
    input  logic signed [15:0] mag      [0:2],
    input  logic               meas_valid,

    // EKF primary outputs (Q15)
    output logic signed [15:0] q_est    [0:3],
    output logic signed [15:0] bias_est [0:2],
    output logic               ekf_valid,
    output logic               ekf_fault,

    // Extended diagnostic outputs (Q15)
    output logic signed [15:0] P_diag     [0:6],
    output logic signed [15:0] innovation [0:6]
);

    ekf_core #(
        .CLK_HZ      (CLK_HZ),
        .FAULT_CYCLES(FAULT_CYCLES)
    ) u_core (
        .clk        (clk),
        .rst_n      (rst_n),
        .ce_100hz   (ce_100hz),
        .accel      (accel),
        .gyro       (gyro),
        .mag        (mag),
        .meas_valid (meas_valid),
        .q_est      (q_est),
        .bias_est   (bias_est),
        .ekf_valid  (ekf_valid),
        .ekf_fault  (ekf_fault),
        .P_diag     (P_diag),
        .innovation (innovation)
    );

endmodule