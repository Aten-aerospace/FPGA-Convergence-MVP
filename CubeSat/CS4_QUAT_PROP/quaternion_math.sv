// =============================================================================
// Module: quaternion_math
// Subsystem: CS4 - Quaternion Propagator
// Description: Combined quaternion math utility. Performs a full Hamilton
//              product followed by normalization in a single module interface.
//              This is a reusable generic math primitive (no ADCS-specific
//              logic such as omega scaling or ce_100hz gating). Suitable for
//              use wherever an arbitrary normalized quaternion product is needed
//              (e.g., EKF attitude composition).
//
//   Data flow:
//     (qa ⊗ qb) → quat_multiply (3-cycle pipeline)
//               → quat_normalize (3-cycle pipeline)
//               → q_out (Q15, normalized)
//
//   Total latency: 6 clock cycles from valid_in to valid_out.
//   All values are signed Q15 (1.15, 16-bit).
//
//   q_norm:       squared-norm of q_out in Q15 (≈32767 for a unit quaternion).
//   q_norm_valid: strobes with valid_out (same cycle as q_out).
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module quaternion_math (
    input  logic        clk,
    input  logic        rst_n,

    // Input quaternions [w,x,y,z] (signed Q15)
    input  logic signed [15:0] qa      [0:3],
    input  logic signed [15:0] qb      [0:3],
    input  logic               valid_in,

    // Normalized product quaternion (signed Q15)
    output logic signed [15:0] q_out       [0:3],
    output logic               valid_out,
    output logic               norm_error,
    output logic        [15:0] q_norm,      // squared-norm in Q15 (≈32767 for unit q)
    output logic               q_norm_valid // strobes with valid_out
);

    // =========================================================================
    // Quaternion multiply: qa ⊗ qb  (3-cycle pipeline)
    // =========================================================================
    logic signed [15:0] q_mul    [0:3];
    logic               mul_valid;

    quat_multiply u_multiply (
        .clk       (clk),
        .rst_n     (rst_n),
        .qa        (qa),
        .qb        (qb),
        .valid_in  (valid_in),
        .qout      (q_mul),
        .valid_out (mul_valid)
    );

    // =========================================================================
    // Quaternion normalize: Newton-Raphson correction (3-cycle pipeline)
    // =========================================================================
    quat_normalize u_normalize (
        .clk         (clk),
        .rst_n       (rst_n),
        .q_in        (q_mul),
        .valid_in    (mul_valid),
        .q_out       (q_out),
        .valid_out   (valid_out),
        .norm_error  (norm_error),
        .q_norm      (q_norm),
        .q_norm_valid(q_norm_valid)
    );

endmodule