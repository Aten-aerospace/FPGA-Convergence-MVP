// =============================================================================
// File        : rotation_matrix.sv
// Module      : rotation_matrix
// Description : Computes 3×3 rotation matrix R from Euler angles (roll, pitch, yaw).
//               Uses CORDIC sin/cos engine (reused from cordic.sv).
//               Output format: Q2.14 (16-bit signed)
//               R = Rz(yaw) × Ry(pitch) × Rx(roll)
// =============================================================================

`timescale 1ns/1ps

module rotation_matrix #(
    parameter int ANGLE_W = 16,  // Q1.15 angle input
    parameter int OUT_W   = 16,  // Q1.15 trig outputs
    parameter int ITER    = 16   // CORDIC iterations
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start,

    // Euler angles (Q1.15, range -π to +π)
    input  logic [ANGLE_W-1:0] roll,
    input  logic [ANGLE_W-1:0] pitch,
    input  logic [ANGLE_W-1:0] yaw,

    // Rotation matrix elements (Q1.15)
    // R[row][col]
    output logic signed [OUT_W-1:0] R [0:2][0:2],
    output logic                     valid
);

    // -------------------------------------------------------------------------
    // CORDIC instances for roll, pitch, yaw
    // -------------------------------------------------------------------------
    logic [OUT_W-1:0] cos_r, sin_r;
    logic [OUT_W-1:0] cos_p, sin_p;
    logic [OUT_W-1:0] cos_y, sin_y;
    logic done_r, done_p, done_y;

    cordic #(.ITERATIONS(ITER)) u_cordic_roll (
        .clk     (clk), .rst_n (rst_n),
        .start   (start),
        .angle   (roll),
        .cos_out (cos_r), .sin_out (sin_r), .done (done_r)
    );

    cordic #(.ITERATIONS(ITER)) u_cordic_pitch (
        .clk     (clk), .rst_n (rst_n),
        .start   (start),
        .angle   (pitch),
        .cos_out (cos_p), .sin_out (sin_p), .done (done_p)
    );

    cordic #(.ITERATIONS(ITER)) u_cordic_yaw (
        .clk     (clk), .rst_n (rst_n),
        .start   (start),
        .angle   (yaw),
        .cos_out (cos_y), .sin_out (sin_y), .done (done_y)
    );

    // -------------------------------------------------------------------------
    // Compose rotation matrix from trig outputs (Q1.15 × Q1.15 = Q2.30 → Q1.15)
    // R = Rz(ψ)·Ry(θ)·Rx(φ)
    //
    // R = [ cψcθ,  cψsθsφ-sψcφ,  cψsθcφ+sψsφ ]
    //     [ sψcθ,  sψsθsφ+cψcφ,  sψsθcφ-cψsφ ]
    //     [ -sθ,   cθsφ,          cθcφ         ]
    // -------------------------------------------------------------------------
    logic all_done;
    assign all_done = done_r & done_p & done_y;

    // Multiply and shift helpers (Q1.15 × Q1.15 >> 15 = Q1.15)
    function automatic logic signed [OUT_W-1:0] mul15(
        input logic signed [OUT_W-1:0] a,
        input logic signed [OUT_W-1:0] b
    );
        logic signed [31:0] prod;
        prod = a * b;
        return prod >>> 15;
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int r = 0; r < 3; r++)
                for (int c = 0; c < 3; c++)
                    R[r][c] <= '0;
            valid <= 1'b0;
        end else if (all_done) begin
            valid <= 1'b1;

            // Row 0
            R[0][0] <= mul15($signed(cos_y), $signed(cos_p));
            R[0][1] <= mul15($signed(cos_y), mul15($signed(sin_p), $signed(sin_r))) -
                       mul15($signed(sin_y), $signed(cos_r));
            R[0][2] <= mul15($signed(cos_y), mul15($signed(sin_p), $signed(cos_r))) +
                       mul15($signed(sin_y), $signed(sin_r));

            // Row 1
            R[1][0] <= mul15($signed(sin_y), $signed(cos_p));
            R[1][1] <= mul15($signed(sin_y), mul15($signed(sin_p), $signed(sin_r))) +
                       mul15($signed(cos_y), $signed(cos_r));
            R[1][2] <= mul15($signed(sin_y), mul15($signed(sin_p), $signed(cos_r))) -
                       mul15($signed(cos_y), $signed(sin_r));

            // Row 2
            R[2][0] <= -$signed(sin_p);
            R[2][1] <= mul15($signed(cos_p), $signed(sin_r));
            R[2][2] <= mul15($signed(cos_p), $signed(cos_r));
        end else begin
            valid <= 1'b0;
        end
    end

endmodule
