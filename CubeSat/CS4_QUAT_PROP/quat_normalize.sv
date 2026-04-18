// =============================================================================
// Module: quat_normalize
// Subsystem: CS4 - Quaternion Propagator
// Description: First-order quaternion normalization using a Newton-Raphson step.
//
//   norm_sq  = (q0² + q1² + q2² + q3²)  [computed in Q30, scaled back to Q15]
//   scale    = (3·Q15_ONE - norm_sq) >> 1  [≈ (3 - ||q||²)/2 in Q15]
//   q_out[i] = (q_in[i] · scale) >> 15   [corrected component]
//
//   For a unit quaternion: norm_sq_q15 ≈ 32767, scale ≈ 32767, q_out ≈ q_in.
//
//   norm_error asserts when |norm_sq_q15 - 32767| > NORM_THRESH (±0.1% deviation).
//   NORM_THRESH = 33: derived from 0.001 × 32767 ≈ 32.767, rounded to 33.
//
//   q_norm is the squared-norm value (norm_sq_q15) registered through the pipeline
//   and output aligned with q_out/valid_out.  For a unit quaternion q_norm ≈ 32767.
//
// Pipeline stages:
//   Stage 1: square each component (Q30 product)
//   Stage 2: sum squares → norm_sq_q30; compute norm_sq_q15 and scale
//   Stage 3: apply scale to each component → q_out (Q15); latch q_norm
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module quat_normalize #(
    parameter int NORM_THRESH = 33  // round(0.001 × 32767) = 33 (±0.1% tolerance per CS-ADCS-004)
)(
    input  logic        clk,
    input  logic        rst_n,

    // Input quaternion (signed Q15)
    input  logic signed [15:0] q_in  [0:3],
    input  logic               valid_in,

    // Normalized output (signed Q15)
    output logic signed [15:0] q_out       [0:3],
    output logic               valid_out,
    output logic               norm_error,  // set when ||q||² deviates > NORM_THRESH
    output logic        [15:0] q_norm,      // squared-norm in Q15 (≈32767 for unit q)
    output logic               q_norm_valid // strobes with valid_out
);

    // Q15 representation of 1.0 (approximately)
    localparam signed [15:0] Q15_ONE = 16'sh7FFF;

    // =========================================================================
    // Stage 1: register inputs, compute squared components (Q30)
    // =========================================================================
    logic signed [31:0] sq [0:3]; // Q30
    logic signed [15:0] q_s1 [0:3];
    logic valid_s1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) begin
                sq[i]   <= '0;
                q_s1[i] <= '0;
            end
            valid_s1 <= 1'b0;
        end else begin
            valid_s1 <= valid_in;
            if (valid_in) begin
                for (int i = 0; i < 4; i++) begin
                    q_s1[i] <= q_in[i];
                    sq[i]   <= q_in[i] * q_in[i]; // Q15 × Q15 = Q30
                end
            end
        end
    end

    // =========================================================================
    // Stage 2: compute norm_sq_q30, scale back to Q15, compute correction factor
    // =========================================================================
    logic signed [33:0] norm_sq_q30; // sum of 4 Q30 values (2 extra guard bits)
    logic [15:0]        norm_sq_q15; // norm_sq scaled to Q15 range
    logic signed [16:0] scale_raw;   // (3·Q15_ONE - norm_sq_q15) - 17-bit
    logic signed [15:0] scale;       // >> 1 → Q15 scale factor
    logic signed [15:0] q_s2 [0:3];
    logic        [15:0] norm_sq_s2; // norm_sq_q15 pipelined to Stage 3
    logic valid_s2;

    always_comb begin
        norm_sq_q30 = sq[0] + sq[1] + sq[2] + sq[3];
        // Q30 → Q15: drop lower 15 bits (equivalent to /32768)
        norm_sq_q15 = norm_sq_q30[30:15];
        // Newton-Raphson: scale = (3 - norm_sq) / 2  in Q15
        // Q15 represents 1.0 as 32767 (= 2^15 - 1), so 3.0 in Q15 units = 3×32767 = 98301.
        // Using 98301 (not 98304 = 3×32768) ensures that for a perfect unit quaternion
        // (norm_sq_q15 = 32767) the scale factor is exactly 32767 (≈ 1.0), avoiding
        // a one-LSB overflow in the 17-bit intermediate scale_raw.
        scale_raw = 17'sd98301 - {1'b0, norm_sq_q15};
        scale     = scale_raw[16:1]; // >> 1
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) q_s2[i] <= '0;
            norm_sq_s2 <= '0;
            valid_s2   <= 1'b0;
            norm_error <= 1'b0;
        end else begin
            valid_s2 <= valid_s1;
            if (valid_s1) begin
                for (int i = 0; i < 4; i++) q_s2[i] <= q_s1[i];
                norm_sq_s2 <= norm_sq_q15;
                // Flag if norm deviates from 1.0 by more than threshold
                norm_error <= (norm_sq_q15 > (Q15_ONE + NORM_THRESH[15:0])) ||
                              (norm_sq_q15 < (Q15_ONE - NORM_THRESH[15:0]));
            end
        end
    end

    // =========================================================================
    // Stage 3: apply scale factor  q_out[i] = (q_s2[i] × scale) >> 15
    // =========================================================================
    logic signed [31:0] scaled [0:3]; // Q30

    always_comb begin
        for (int i = 0; i < 4; i++)
            scaled[i] = q_s2[i] * scale; // Q15 × Q15 = Q30
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) q_out[i] <= '0;
            valid_out    <= 1'b0;
            q_norm       <= '0;
            q_norm_valid <= 1'b0;
        end else begin
            valid_out    <= valid_s2;
            q_norm_valid <= valid_s2;
            if (valid_s2) begin
                q_norm <= norm_sq_s2; // squared-norm in Q15 (≈32767 for unit quaternion)
                for (int i = 0; i < 4; i++) begin
                    // Saturate to Q15 range
                    if (scaled[i][31:30] == 2'b00 || scaled[i][31:30] == 2'b11)
                        q_out[i] <= scaled[i][30:15];
                    else
                        q_out[i] <= scaled[i][31] ? 16'sh8000 : 16'sh7FFF;
                end
            end
        end
    end

endmodule