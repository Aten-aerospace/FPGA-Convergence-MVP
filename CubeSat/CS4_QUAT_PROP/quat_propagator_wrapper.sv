// =============================================================================
// Module: quat_propagator_wrapper (CS4 top-level wrapper)
// Subsystem: CS4 - Quaternion Propagator
// Description: Propagates the attitude quaternion forward one time step using
//              the angular velocity (omega) from the IMU.
//
//   Algorithm (small-angle approximation):
//     dq = [1, ω_x·dt/2, ω_y·dt/2, ω_z·dt/2]  (normalised before use)
//     q(k+1) = normalize( q(k) ⊗ dq )
//
//   omega is in Q15 rad/s, dt = 0.01 s (100 Hz update rate).
//   ω·dt/2 = ω × 0.005 → in Q15: multiply by DT_HALF = round(0.005 × 32768) = 164
//   dq is then [32767, ω_x_small, ω_y_small, ω_z_small] in Q15.
//
//   Internally instantiates:
//     quat_multiply  (3-cycle pipeline)  - Hamilton product  q_in ⊗ dq
//     quat_normalize (3-cycle pipeline)  - Newton-Raphson normalization
//     norm_checker   (2-cycle pipeline)  - post-normalization health check on q_out
//
//   Total latency from ce_100hz to q_valid_out:  ~7 clock cycles
//   Additional latency to norm_ok_valid:         +2 clock cycles (~9 total)
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module quat_propagator_wrapper (
    input  logic        clk,
    input  logic        rst_n,

    // 100 Hz clock enable (from CS12/clk_manager)
    input  logic        ce_100hz,

    // Current attitude quaternion (Q15)
    input  logic signed [15:0] q_in  [0:3],
    input  logic               q_valid_in,

    // Angular velocity (Q15 rad/s) from CS1/IMU
    input  logic signed [15:0] omega [0:2],

    // Propagated + normalised quaternion (Q15)
    output logic signed [15:0] q_out        [0:3],
    output logic               q_valid_out,
    output logic               norm_error,
    output logic        [15:0] q_norm,       // squared-norm in Q15 (≈32767 for unit q)
    output logic               q_norm_valid, // strobe: q_norm is valid (same cycle as q_valid_out)
    output logic               quat_ready,   // propagation complete (alias of q_valid_out)

    // Post-normalization health check outputs from norm_checker (2 cycles after q_valid_out)
    output logic               norm_ok,      // asserts when post-norm ‖q‖² is within threshold
    output logic        [15:0] norm_val,     // |‖q‖²-1| deviation in Q15 (for EKF drift monitor)
    output logic               norm_ok_valid // strobe: norm_ok / norm_val are valid
);

    // =========================================================================
    // Compute delta-quaternion dq from omega
    // ω·dt/2 in Q15 = omega × DT_HALF >> 15  where DT_HALF ≈ 164 (= 0.005 × 32768)
    // =========================================================================
    localparam signed [15:0] DT_HALF = 16'sd164; // 0.005 × 32768 rounded

    logic signed [15:0] dq [0:3];
    logic signed [31:0] omega_small_raw [0:2]; // Q30

    always_comb begin
        for (int i = 0; i < 3; i++)
            omega_small_raw[i] = omega[i] * DT_HALF; // Q15 × Q15 = Q30
        // dq[0] = 1.0 in Q15; dq[1..3] = omega_small scaled to Q15
        dq[0] = 16'sh7FFF;
        for (int i = 0; i < 3; i++)
            dq[i+1] = omega_small_raw[i][30:15]; // take Q15 slice
    end

    // =========================================================================
    // Pipeline input: register inputs on each ce_100hz + q_valid_in
    // =========================================================================
    logic signed [15:0] q_in_r  [0:3];
    logic signed [15:0] dq_r    [0:3];
    logic               mul_valid_in;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            q_in_r[0] <= 16'sh7FFF; // identity w = 1.0
            q_in_r[1] <= '0; q_in_r[2] <= '0; q_in_r[3] <= '0;
            dq_r[0]   <= 16'sh7FFF; // identity dq w = 1.0
            dq_r[1]   <= '0; dq_r[2]   <= '0; dq_r[3]   <= '0;
            mul_valid_in <= 1'b0;
        end else begin
            mul_valid_in <= ce_100hz && q_valid_in;
            if (ce_100hz && q_valid_in) begin
                for (int i = 0; i < 4; i++) begin
                    q_in_r[i] <= q_in[i];
                    dq_r[i]   <= dq[i];
                end
            end
        end
    end

    // =========================================================================
    // Quaternion multiply: q_in ⊗ dq  (3-cycle pipeline)
    // =========================================================================
    logic signed [15:0] q_mul_out [0:3];
    logic               mul_valid_out;

    quat_multiply u_mul (
        .clk       (clk),
        .rst_n     (rst_n),
        .qa        (q_in_r),
        .qb        (dq_r),
        .valid_in  (mul_valid_in),
        .qout      (q_mul_out),
        .valid_out (mul_valid_out)
    );

    // =========================================================================
    // Quaternion normalize: Newton-Raphson correction  (3-cycle pipeline)
    // =========================================================================
    quat_normalize u_norm (
        .clk         (clk),
        .rst_n       (rst_n),
        .q_in        (q_mul_out),
        .valid_in    (mul_valid_out),
        .q_out       (q_out),
        .valid_out   (q_valid_out),
        .norm_error  (norm_error),
        .q_norm      (q_norm),
        .q_norm_valid(q_norm_valid)
    );

    // quat_ready: asserts for one clock cycle when propagation is complete
    assign quat_ready = q_valid_out;

    // =========================================================================
    // Post-normalization health check: norm_checker on q_out
    // Verifies that quat_normalize produced a unit quaternion (|q_out|² ≈ 1).
    // This is separate from quat_normalize's internal norm_error which fires on
    // the PRE-normalization input. norm_checker here confirms the OUTPUT is
    // within tolerance - a sanity check that normalisation converged.
    // Latency: 2 clock cycles after q_valid_out → norm_ok / norm_val valid.
    // =========================================================================
    logic nc_norm_error_unused; // quat_normalize already exposes norm_error

    norm_checker u_norm_chk (
        .clk       (clk),
        .rst_n     (rst_n),
        .q_in      (q_out),
        .valid_in  (q_valid_out),
        .norm_ok   (norm_ok),
        .norm_error(nc_norm_error_unused),
        .norm_val  (norm_val)
    );

    // Delay q_valid_out by 2 cycles to produce norm_ok_valid aligned with norm_ok/norm_val
    logic norm_ok_valid_s1;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            norm_ok_valid_s1 <= 1'b0;
            norm_ok_valid    <= 1'b0;
        end else begin
            norm_ok_valid_s1 <= q_valid_out;
            norm_ok_valid    <= norm_ok_valid_s1;
        end
    end

endmodule