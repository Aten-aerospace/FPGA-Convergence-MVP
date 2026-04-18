// =============================================================================
// Module: quat_multiply
// Subsystem: CS4 - Quaternion Propagator
// Description: 3-stage pipelined Hamilton quaternion product  q_out = qa ⊗ qb.
//              Convention: q = [w, x, y, z] = [q[0], q[1], q[2], q[3]].
//              All inputs and outputs are signed Q15 (1.15, 16-bit).
//              Multiplication: Q15 × Q15 → Q30 (32-bit); result scaled to Q15
//              by taking bits [30:15] of the 34-bit accumulated sum.
//              Pipeline stages:
//                Stage 1: register inputs
//                Stage 2: compute 16 cross-products (one DSP per multiply)
//                Stage 3: accumulate per-component sums and scale to Q15
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module quat_multiply (
    input  logic        clk,
    input  logic        rst_n,

    // Inputs: quaternion components [w, x, y, z], signed Q15
    input  logic signed [15:0] qa [0:3],
    input  logic signed [15:0] qb [0:3],
    input  logic               valid_in,

    // Outputs: quaternion product, signed Q15
    output logic signed [15:0] qout [0:3],
    output logic               valid_out
);

    // =========================================================================
    // Stage 1: register inputs
    // =========================================================================
    logic signed [15:0] qa_r [0:3];
    logic signed [15:0] qb_r [0:3];
    logic valid_s1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) begin
                qa_r[i] <= '0;
                qb_r[i] <= '0;
            end
            valid_s1 <= 1'b0;
        end else begin
            valid_s1 <= valid_in;
            if (valid_in) begin
                for (int i = 0; i < 4; i++) begin
                    qa_r[i] <= qa[i];
                    qb_r[i] <= qb[i];
                end
            end
        end
    end

    // =========================================================================
    // Stage 2: compute all 16 products  prod[i][j] = qa_r[i] * qb_r[j]
    // =========================================================================
    logic signed [31:0] prod [0:3][0:3]; // Q30
    logic valid_s2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    prod[i][j] <= '0;
            valid_s2 <= 1'b0;
        end else begin
            valid_s2 <= valid_s1;
            if (valid_s1) begin
                for (int i = 0; i < 4; i++)
                    for (int j = 0; j < 4; j++)
                        prod[i][j] <= qa_r[i] * qb_r[j];
            end
        end
    end

    // =========================================================================
    // Stage 3: accumulate Hamilton product sums, scale Q30 → Q15
    //
    //   q_out[0] (w) = + qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3]
    //   q_out[1] (x) = + qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2]
    //   q_out[2] (y) = + qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1]
    //   q_out[3] (z) = + qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0]
    //
    // Sum of 4 Q30 terms: 34-bit signed.  Q15 result = sum[30:15].
    // Saturate if overflow bits [33:31] are not all sign-equal.
    // =========================================================================
    logic signed [33:0] sum_pre [0:3];

    always_comb begin
        sum_pre[0] = prod[0][0] - prod[1][1] - prod[2][2] - prod[3][3];
        sum_pre[1] = prod[0][1] + prod[1][0] + prod[2][3] - prod[3][2];
        sum_pre[2] = prod[0][2] - prod[1][3] + prod[2][0] + prod[3][1];
        sum_pre[3] = prod[0][3] + prod[1][2] - prod[2][1] + prod[3][0];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) qout[i] <= '0;
            valid_out <= 1'b0;
        end else begin
            valid_out <= valid_s2;
            if (valid_s2) begin
                for (int i = 0; i < 4; i++) begin
                    // Saturate: check that bits [33:31] are uniform sign extension
                    if (sum_pre[i][33:31] == 3'b000 || sum_pre[i][33:31] == 3'b111)
                        qout[i] <= sum_pre[i][30:15]; // in-range: take Q15 slice
                    else
                        qout[i] <= sum_pre[i][33] ? 16'sh8000 : 16'sh7FFF; // saturate
                end
            end
        end
    end

endmodule