// =============================================================================
// Module: norm_checker
// Subsystem: CS4 - Quaternion Propagator
// Description: Checks quaternion unit-norm constraint and flags deviations.
//
//   Algorithm:
//     norm_sq   = q[0]² + q[1]² + q[2]² + q[3]²   (Q30, then scaled to Q15)
//     deviation = |norm_sq_q15 - Q15_ONE|
//     norm_ok   = 1 if deviation ≤ NORM_THRESH
//     norm_error = 1 if deviation > NORM_THRESH
//     norm_val  = deviation (Q15 unsigned, saturated to 16 bits)
//
//   1.0 in Q15 = 32767 (Q15_ONE). A perfect unit quaternion gives
//   norm_sq_q15 = 32767, hence deviation = 0.
//   NORM_THRESH = 33 = round(0.001 × 32767) → ±0.1% tolerance per CS-ADCS-004.
//
//   Pipeline: 2 registered stages.
//     Stage 1: square each component (Q30)
//     Stage 2: sum squares → norm_sq (Q15), compute deviation, assert flags
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module norm_checker #(
    parameter int NORM_THRESH = 33   // round(0.001 × 32767) = 33 (±0.1% tolerance per CS-ADCS-004)
)(
    input  logic        clk,
    input  logic        rst_n,

    // Input quaternion [w,x,y,z] (signed Q15)
    input  logic signed [15:0] q_in     [0:3],
    input  logic               valid_in,

    // Status outputs
    output logic               norm_ok,        // asserts when norm deviation ≤ threshold
    output logic               norm_error,     // asserts when norm deviation > threshold
    output logic        [15:0] norm_val        // |‖q‖²-1| deviation in Q15
);

    localparam [15:0] Q15_ONE = 16'h7FFF; // 1.0 in Q15

    // =========================================================================
    // Stage 1: register inputs, compute squared components (Q30)
    // =========================================================================
    logic signed [31:0] sq      [0:3]; // q_in[i]², Q30
    logic               valid_s1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) sq[i] <= '0;
            valid_s1 <= 1'b0;
        end else begin
            valid_s1 <= valid_in;
            if (valid_in) begin
                for (int i = 0; i < 4; i++)
                    sq[i] <= q_in[i] * q_in[i]; // Q15 × Q15 = Q30
            end
        end
    end

    // =========================================================================
    // Stage 2 combinatorial: accumulate norm_sq, compute deviation
    // =========================================================================
    logic signed [33:0] norm_sq_q30;  // sum of 4 Q30 terms
    logic        [15:0] norm_sq_q15;  // Q30 → Q15 (bits [30:15])
    logic        [15:0] deviation;    // |norm_sq_q15 - Q15_ONE|

    always_comb begin
        norm_sq_q30 = sq[0] + sq[1] + sq[2] + sq[3];
        norm_sq_q15 = norm_sq_q30[30:15];
        // Unsigned absolute difference
        if (norm_sq_q15 >= Q15_ONE)
            deviation = norm_sq_q15 - Q15_ONE;
        else
            deviation = Q15_ONE - norm_sq_q15;
    end

    // =========================================================================
    // Stage 2 registers: latch flags and norm_val
    // norm_ok is guaranteed to be the complement of norm_error.
    // =========================================================================
    logic norm_exceeded; // combinatorial threshold comparison

    always_comb begin
        norm_exceeded = (deviation > NORM_THRESH[15:0]);
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            norm_ok    <= 1'b1;
            norm_error <= 1'b0;
            norm_val   <= '0;
        end else begin
            if (valid_s1) begin
                norm_val   <= deviation;
                norm_error <=  norm_exceeded;
                norm_ok    <= ~norm_exceeded;
            end
        end
    end

endmodule