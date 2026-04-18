// =============================================================================
// Module: quat_propagator
// Subsystem: CS4 - Quaternion Propagator
// Description: Core quaternion kinematic propagator (standalone, no sub-instances).
//              Propagates attitude quaternion one 100 Hz epoch using body-frame
//              angular velocity in a fully-inline 3-stage pipeline.
//
//   Algorithm:
//     dq = [1, ω_x·dt/2, ω_y·dt/2, ω_z·dt/2]       (Q15, small-angle dq)
//     q_raw = q_in ⊗ dq                               (Hamilton product)
//     q_out = normalize(q_raw)                         (Newton-Raphson correction)
//
//   dt/2 → DT_HALF = round(0.005 × 32768) = 164 (Q15, 100 Hz epoch, dt = 0.01 s)
//   All arithmetic is Q15 signed fixed-point (1.15 format).
//
//   Pipeline (3 registered stages, no sub-module instantiations):
//     Stage 1 : register q_in and dq, assert valid_s1
//     Stage 2 : inline Hamilton product → q_mul (Q15), Newton-Raphson scale
//     Stage 3 : apply normalization scale → q_out (Q15), assert norm_error
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module quat_propagator (
    input  logic        clk,
    input  logic        rst_n,

    // 100 Hz clock enable
    input  logic        ce_100hz,

    // Input attitude quaternion [w,x,y,z] (signed Q15)
    input  logic signed [15:0] q_in   [0:3],

    // Angular velocity [ωx,ωy,ωz] (signed Q15, rad/s)
    input  logic signed [15:0] omega  [0:2],
    input  logic               valid_in,

    // Propagated + normalised quaternion (signed Q15)
    output logic signed [15:0] q_out  [0:3],
    output logic               valid_out,
    output logic               norm_error   // asserts when |‖q‖²-1| > threshold
);

    localparam signed [15:0] DT_HALF    = 16'sd164;   // 0.005 × 32768
    localparam signed [15:0] Q15_ONE    = 16'sh7FFF;
    localparam        [15:0] NORM_THRESH = 16'd33;     // round(0.001 × 32767) = 33 (±0.1% tolerance)

    // =========================================================================
    // Combinatorial: compute dq from omega (no registers yet)
    // =========================================================================
    logic signed [31:0] omega_half_raw [0:2]; // Q30
    logic signed [15:0] dq_comb        [0:3]; // Q15

    always_comb begin
        for (int i = 0; i < 3; i++)
            omega_half_raw[i] = omega[i] * DT_HALF; // Q15 × Q15 = Q30
        dq_comb[0] = Q15_ONE;
        for (int i = 0; i < 3; i++)
            dq_comb[i+1] = omega_half_raw[i][30:15]; // Q30 → Q15
    end

    // =========================================================================
    // Stage 1: register q_in and dq
    // =========================================================================
    logic signed [15:0] q_s1   [0:3];
    logic signed [15:0] dq_s1  [0:3];
    logic               valid_s1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) begin
                q_s1[i]  <= '0;
                dq_s1[i] <= '0;
            end
            q_s1[0]  <= Q15_ONE; // identity w = 1
            dq_s1[0] <= Q15_ONE;
            valid_s1 <= 1'b0;
        end else begin
            valid_s1 <= ce_100hz && valid_in;
            if (ce_100hz && valid_in) begin
                for (int i = 0; i < 4; i++) begin
                    q_s1[i]  <= q_in[i];
                    dq_s1[i] <= dq_comb[i];
                end
            end
        end
    end

    // =========================================================================
    // Stage 2 combinatorial: Hamilton product and Newton-Raphson scale
    //
    //   Hamilton product  q_in ⊗ dq  ([w,x,y,z] convention):
    //     w_out =  qa[0]*qb[0] - qa[1]*qb[1] - qa[2]*qb[2] - qa[3]*qb[3]
    //     x_out =  qa[0]*qb[1] + qa[1]*qb[0] + qa[2]*qb[3] - qa[3]*qb[2]
    //     y_out =  qa[0]*qb[2] - qa[1]*qb[3] + qa[2]*qb[0] + qa[3]*qb[1]
    //     z_out =  qa[0]*qb[3] + qa[1]*qb[2] - qa[2]*qb[1] + qa[3]*qb[0]
    //   Sum of 4 Q30 terms fits in 34 bits. Scale Q30 → Q15: take [30:15].
    //
    //   Newton-Raphson normalization:
    //     norm_sq_q15 = sum(q_mul[i]²) >> 15
    //     scale       = (3·32767 - norm_sq_q15) >> 1
    // =========================================================================
    logic signed [31:0] prod   [0:3][0:3]; // 16 cross-products Q30
    logic signed [33:0] ham    [0:3];      // Hamilton sums Q34
    logic signed [15:0] q_mul  [0:3];      // scaled to Q15
    logic signed [31:0] sq     [0:3];      // q_mul[i]² Q30
    logic signed [33:0] norm_sq_q30;
    logic        [15:0] norm_sq_q15;
    logic signed [16:0] scale_raw;
    logic signed [15:0] scale_comb;

    always_comb begin
        // 16 pairwise products
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                prod[i][j] = q_s1[i] * dq_s1[j];

        // Hamilton accumulation
        ham[0] = prod[0][0] - prod[1][1] - prod[2][2] - prod[3][3];
        ham[1] = prod[0][1] + prod[1][0] + prod[2][3] - prod[3][2];
        ham[2] = prod[0][2] - prod[1][3] + prod[2][0] + prod[3][1];
        ham[3] = prod[0][3] + prod[1][2] - prod[2][1] + prod[3][0];

        // Scale Q30 → Q15 (take bits [30:15] with saturation)
        for (int i = 0; i < 4; i++) begin
            if (ham[i][33:31] == 3'b000 || ham[i][33:31] == 3'b111)
                q_mul[i] = ham[i][30:15];
            else
                q_mul[i] = ham[i][33] ? 16'sh8000 : 16'sh7FFF;
        end

        // Squares for norm computation
        for (int i = 0; i < 4; i++)
            sq[i] = q_mul[i] * q_mul[i]; // Q30

        // Norm squared
        norm_sq_q30 = sq[0] + sq[1] + sq[2] + sq[3];
        norm_sq_q15 = norm_sq_q30[30:15];

        // Newton-Raphson scale factor: (3·Q15_ONE - norm_sq_q15) >> 1
        scale_raw  = 17'sd98301 - {1'b0, norm_sq_q15};
        scale_comb = scale_raw[16:1];
    end

    // =========================================================================
    // Stage 2 registers: latch q_mul, scale, norm deviation
    // =========================================================================
    logic signed [15:0] q_s2    [0:3];
    logic signed [15:0] scale_s2;
    logic               norm_err_s2;
    logic               valid_s2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) q_s2[i] <= '0;
            scale_s2    <= Q15_ONE;
            norm_err_s2 <= 1'b0;
            valid_s2    <= 1'b0;
        end else begin
            valid_s2 <= valid_s1;
            if (valid_s1) begin
                for (int i = 0; i < 4; i++) q_s2[i] <= q_mul[i];
                scale_s2    <= scale_comb;
                norm_err_s2 <= (norm_sq_q15 > (16'(Q15_ONE) + NORM_THRESH)) ||
                               (norm_sq_q15 < (16'(Q15_ONE) - NORM_THRESH));
            end
        end
    end

    // =========================================================================
    // Stage 3 combinatorial: apply normalization scale
    // q_out[i] = (q_s2[i] × scale_s2) >> 15  (Q30 → Q15)
    // =========================================================================
    logic signed [31:0] scaled [0:3]; // Q30

    always_comb begin
        for (int i = 0; i < 4; i++)
            scaled[i] = q_s2[i] * scale_s2;
    end

    // =========================================================================
    // Stage 3 registers: saturate and output
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) q_out[i] <= '0;
            q_out[0]   <= Q15_ONE; // identity
            valid_out  <= 1'b0;
            norm_error <= 1'b0;
        end else begin
            valid_out  <= valid_s2;
            norm_error <= 1'b0;
            if (valid_s2) begin
                norm_error <= norm_err_s2;
                for (int i = 0; i < 4; i++) begin
                    if (scaled[i][31:30] == 2'b00 || scaled[i][31:30] == 2'b11)
                        q_out[i] <= scaled[i][30:15];
                    else
                        q_out[i] <= scaled[i][31] ? 16'sh8000 : 16'sh7FFF;
                end
            end
        end
    end

endmodule