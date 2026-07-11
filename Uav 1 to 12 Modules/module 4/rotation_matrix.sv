// =============================================================================
// File        : rotation_matrix.sv
// Module      : rotation_matrix
// Optimized   : Reduced LUT usage and routing complexity
// Changes:
//   - Fixed 2D array assignment in synthesizable format
//   - Removed mul15 function duplication
//   - Shared multiplier paths
//   - Forced DSP inference
//   - Reduced nested multipliers
//   - Simplified sequential logic
//   - FIXED: Force cordic_yaw synthesis via registered outputs
// =============================================================================

`timescale 1ns/1ps

module rotation_matrix #(
    parameter int ANGLE_W = 16,
    parameter int OUT_W   = 16,
    parameter int ITER    = 16
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start,

    input  logic signed [ANGLE_W-1:0] roll,
    input  logic signed [ANGLE_W-1:0] pitch,
    input  logic signed [ANGLE_W-1:0] yaw,

    output logic signed [OUT_W-1:0] R [0:2][0:2],
    output logic                    valid
);

    // -------------------------------------------------------------------------
    // Trig outputs
    // -------------------------------------------------------------------------
    logic signed [OUT_W-1:0] cos_r, sin_r;
    logic signed [OUT_W-1:0] cos_p, sin_p;
    logic signed [OUT_W-1:0] cos_y, sin_y;

    logic done_r, done_p, done_y;

    // =========================================================================
    // CRITICAL: Registered yaw outputs to force cordic_yaw retention
    // =========================================================================
    logic signed [OUT_W-1:0] cos_y_reg, sin_y_reg;
    logic done_y_reg;

    // -------------------------------------------------------------------------
    // CORDIC Instances (placeholders - assumes cordic.sv exists)
    // -------------------------------------------------------------------------
    cordic #(.ITERATIONS(ITER)) u_cordic_roll (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (start),
        .angle   (roll),
        .cos_out (cos_r),
        .sin_out (sin_r),
        .done    (done_r)
    );

    cordic #(.ITERATIONS(ITER)) u_cordic_pitch (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (start),
        .angle   (pitch),
        .cos_out (cos_p),
        .sin_out (sin_p),
        .done    (done_p)
    );

    cordic #(.ITERATIONS(ITER)) u_cordic_yaw (
        .clk     (clk),
        .rst_n   (rst_n),
        .start   (start),
        .angle   (yaw),
        .cos_out (cos_y),
        .sin_out (sin_y),
        .done    (done_y)
    );

    // =========================================================================
    // PIPELINE STAGE 1: Register all yaw outputs to force synthesis
    // =========================================================================
    (* keep = "true" *) always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cos_y_reg <= '0;
            sin_y_reg <= '0;
            done_y_reg <= 1'b0;
        end else begin
            cos_y_reg <= cos_y;      // ← Forces cos_y to be retained
            sin_y_reg <= sin_y;      // ← Forces sin_y to be retained
            done_y_reg <= done_y;    // ← Forces done_y to be retained
        end
    end

    // -------------------------------------------------------------------------
    // Shared Multiply Paths
    // -------------------------------------------------------------------------
    logic all_done;
    assign all_done = done_r & done_p & done_y_reg;

    logic signed [31:0] mult0, mult1, mult2, mult3;
    logic signed [31:0] mult4, mult5, mult6, mult7;
    logic signed [31:0] mult8, mult9, mult10, mult11;

    logic signed [OUT_W-1:0] sp_sr;
    logic signed [OUT_W-1:0] sp_cr;
    logic signed [OUT_W-1:0] cp_sr;
    logic signed [OUT_W-1:0] cp_cr;
    logic signed [OUT_W-1:0] cy_cp;
    logic signed [OUT_W-1:0] sy_cp;
    logic signed [OUT_W-1:0] cy_cr;
    logic signed [OUT_W-1:0] sy_sr;

    // -------------------------------------------------------------------------
    // PIPELINE STAGE 2: Shared DSP Multipliers (use registered yaw)
    // -------------------------------------------------------------------------
    (* use_dsp = "yes" *) always_comb begin

        // Roll-Pitch products
        mult0 = $signed(sin_p) * $signed(sin_r);
        mult1 = $signed(sin_p) * $signed(cos_r);
        mult2 = $signed(cos_p) * $signed(sin_r);
        mult3 = $signed(cos_p) * $signed(cos_r);

        sp_sr = mult0 >>> 15;
        sp_cr = mult1 >>> 15;
        cp_sr = mult2 >>> 15;
        cp_cr = mult3 >>> 15;

        // Cross-angle products (using registered yaw)
        mult4  = $signed(cos_y_reg) * $signed(sp_sr);
        mult5  = $signed(sin_y_reg) * $signed(cos_r);
        mult6  = $signed(sin_y_reg) * $signed(sp_cr);
        mult7  = $signed(cos_y_reg) * $signed(sin_r);

        // Yaw-specific products (EXPLICITLY USE REGISTERED YAW)
        mult8  = $signed(cos_y_reg) * $signed(cos_p);  // cos(yaw)*cos(pitch)
        mult9  = $signed(sin_y_reg) * $signed(cos_p);  // sin(yaw)*cos(pitch)
        mult10 = $signed(cos_y_reg) * $signed(cos_r);  // cos(yaw)*cos(roll)
        mult11 = $signed(sin_y_reg) * $signed(sin_r);  // sin(yaw)*sin(roll)

        cy_cp = mult8 >>> 15;
        sy_cp = mult9 >>> 15;
        cy_cr = mult10 >>> 15;
        sy_sr = mult11 >>> 15;

    end

    // =========================================================================
    // PIPELINE STAGE 3: Matrix Computation (Sequential)
    // =========================================================================
    (* use_dsp = "yes" *) always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin

            for (int r = 0; r < 3; r++) begin
                for (int c = 0; c < 3; c++) begin
                    R[r][c] <= '0;
                end
            end

            valid <= 1'b0;

        end
        else if (all_done) begin

            valid <= 1'b1;

            // -----------------------------------------------------------------
            // Row 0: Rotation matrix elements R[0][*]
            // R[0][0] = cos(yaw) * cos(pitch)
            // R[0][1] = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll)
            // R[0][2] = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll)
            // -----------------------------------------------------------------
            R[0][0] <= cy_cp;  // ← Uses registered yaw product

            R[0][1] <= ((mult4) >>> 15) - ((mult5) >>> 15);

            R[0][2] <= ((mult6) >>> 15) + (sy_sr);  // ← Uses registered yaw product

            // -----------------------------------------------------------------
            // Row 1: Rotation matrix elements R[1][*]
            // R[1][0] = sin(yaw) * cos(pitch)
            // R[1][1] = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll)
            // R[1][2] = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)
            // -----------------------------------------------------------------
            R[1][0] <= sy_cp;  // ← Uses registered yaw product

            R[1][1] <= ((mult4) >>> 15) + (cy_cr);  // ← Uses registered yaw product

            R[1][2] <= ((mult6) >>> 15) - (cy_cr);

            // -----------------------------------------------------------------
            // Row 2: Rotation matrix elements R[2][*]
            // R[2][0] = -sin(pitch)
            // R[2][1] = cos(pitch) * sin(roll)
            // R[2][2] = cos(pitch) * cos(roll)
            // -----------------------------------------------------------------
            R[2][0] <= -$signed(sin_p);

            R[2][1] <= cp_sr;

            R[2][2] <= cp_cr;

        end
        else begin
            valid <= 1'b0;
        end
    end

endmodule
