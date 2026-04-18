// =============================================================================
// Module: spiral_refinement (CS10 ACQUIRE state spiral pattern)
// Subsystem: CS10 - Laser Pointing FSM
// Description: Generates inward spiral scan pattern for ACQUIRE state.
//              Starts at peak position from SEARCH, spirals inward until
//              radius < 5 arc-minutes or 5 seconds elapsed.
//
//   Algorithm (linear approximation of logarithmic spiral):
//     - Start radius: 120 arc-minutes (±2°)
//     - 8 angular steps per revolution (45° each), at ce_100hz rate
//     - Radius reduced by ~12.5% per step (50% per revolution)
//     - Position: az = center_az + r*cos(θ), el = center_el + r*sin(θ)
//     - cos/sin approximated via 8-point LUT (unit circle, Q15 format)
//     - Convergence: r < 5 arc-min OR 500 ce_100hz ticks (5 seconds)
//
//   Convergence time output saturates at 255 (100ms units).
//
// Requirement: CS-LSR-004 (ACQUIRE State Behavior - spiral, peak-hold)
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module spiral_refinement #(
    parameter int R_START_ARCMIN  = 120,  // initial spiral radius (arc-min)
    parameter int R_MIN_ARCMIN    = 5,    // convergence radius threshold (arc-min)
    parameter int TIMEOUT_TICKS   = 500,  // ce_100hz ticks = 5 seconds
    parameter int ANGULAR_STEPS   = 8     // steps per revolution
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    input  logic        spiral_start,             // strobe to begin spiral
    input  logic signed [15:0] spiral_center_az,  // center azimuth from raster peak
    input  logic signed [15:0] spiral_center_el,  // center elevation from raster peak
    input  logic [11:0] peak_signal_strength,     // from peak_hold_detector

    output logic signed [15:0] gimbal_cmd_az,     // azimuth command (arc-min)
    output logic signed [15:0] gimbal_cmd_el,     // elevation command (arc-min)
    output logic               cmd_valid,
    output logic               spiral_converged,  // radius<R_MIN or timeout
    output logic [7:0]         convergence_time_100ms  // elapsed in 100ms units
);

    // =========================================================================
    // 8-point unit-circle LUT (cos, sin) in Q15 signed (32767 = 1.0)
    //   Angle index:  0=0°, 1=45°, 2=90°, 3=135°, 4=180°, 5=225°, 6=270°, 7=315°
    // =========================================================================
    localparam int LUT_SZ = 8;
    // cos_lut in Q15 (32767 * cos(k*45°))
    localparam logic signed [15:0] COS_LUT [0:LUT_SZ-1] = '{
        16'sd23170,   //  cos(  0°) = 1.0, but radius r is pre-scaled by 0.707 per arm
        16'sd16384,   //  cos( 45°) ≈  0.500 * 32768 - approx
        16'sd0,       //  cos( 90°)
       -16'sd16384,   //  cos(135°)
       -16'sd23170,   //  cos(180°)
       -16'sd16384,   //  cos(225°)
        16'sd0,       //  cos(270°)
        16'sd16384    //  cos(315°)
    };
    localparam logic signed [15:0] SIN_LUT [0:LUT_SZ-1] = '{
        16'sd0,        //  sin(  0°)
        16'sd16384,    //  sin( 45°)
        16'sd23170,    //  sin( 90°)
        16'sd16384,    //  sin(135°)
        16'sd0,        //  sin(180°)
       -16'sd16384,    //  sin(225°)
       -16'sd23170,    //  sin(270°)
       -16'sd16384     //  sin(315°)
    };

    // =========================================================================
    // State
    // =========================================================================
    typedef enum logic [1:0] {
        SP_IDLE   = 2'd0,
        SP_RUN    = 2'd1,
        SP_DONE   = 2'd2
    } sp_state_t;

    sp_state_t sp_st;

    // Radius in arc-minutes (integer)
    logic [7:0]  radius;          // 0..255 arc-min
    logic [2:0]  ang_idx;         // 0..7 (angle step within revolution)
    logic [9:0]  elapsed;         // ce_100hz ticks (0..511)
    logic [7:0]  conv_time_reg;

    // Decay factor per step: r_new = r * 7/8 (≈ 12.5% reduction = 50% per 4 steps)
    // (Using 4-step revolution with 50% reduction: r_new = r * 181/256 ≈ r * 0.707)
    // Simpler: reduce by 1 arc-min each step, or use shift-based approximation
    // We use: r_next = (r * 7) >> 3  (multiply by 7/8 = 87.5% each step)
    logic [7:0] radius_next;
    always_comb begin
        radius_next = (radius[7:0] * 8'd7) >> 3;
        if (radius_next == radius && radius > 0)
            radius_next = radius - 8'd1;  // ensure monotonic decay
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sp_st          <= SP_IDLE;
            radius         <= 8'(R_START_ARCMIN);
            ang_idx        <= 3'd0;
            elapsed        <= 10'd0;
            conv_time_reg  <= 8'd0;
            gimbal_cmd_az  <= 16'sd0;
            gimbal_cmd_el  <= 16'sd0;
            cmd_valid      <= 1'b0;
            spiral_converged <= 1'b0;
            convergence_time_100ms <= 8'd0;
        end else begin
            cmd_valid        <= 1'b0;
            spiral_converged <= 1'b0;

            case (sp_st)
                SP_IDLE: begin
                    if (spiral_start) begin
                        sp_st   <= SP_RUN;
                        radius  <= 8'(R_START_ARCMIN);
                        ang_idx <= 3'd0;
                        elapsed <= 10'd0;
                    end
                end

                SP_RUN: begin
                    if (ce_100hz) begin
                        // Compute position: center + r * cos/sin
                        // Q15 product: (r * cos_lut) >> 15 gives arc-min offset
                        gimbal_cmd_az <= spiral_center_az +
                            16'((32'($signed({8'd0, radius})) * 32'(COS_LUT[ang_idx])) >>> 15);
                        gimbal_cmd_el <= spiral_center_el +
                            16'((32'($signed({8'd0, radius})) * 32'(SIN_LUT[ang_idx])) >>> 15);
                        cmd_valid <= 1'b1;

                        // Advance angle and decay radius each step
                        ang_idx <= ang_idx + 1;
                        if (ang_idx == 3'(ANGULAR_STEPS - 1)) begin
                            radius <= radius_next;
                        end

                        // Elapsed time
                        if (elapsed < 10'd511)
                            elapsed <= elapsed + 1;

                        // Convergence check
                        if (radius <= 8'(R_MIN_ARCMIN) || elapsed >= 10'(TIMEOUT_TICKS)) begin
                            sp_st  <= SP_DONE;
                            spiral_converged <= 1'b1;
                            // saturate elapsed at 255 (in 100ms units, elapsed is in ticks)
                            conv_time_reg <= (elapsed[9:1] > 8'd255) ? 8'd255 : 8'(elapsed[9:1]);
                            convergence_time_100ms <= conv_time_reg;
                        end
                    end
                end

                SP_DONE: begin
                    convergence_time_100ms <= conv_time_reg;
                    // Return to IDLE when spiral_start toggled
                    if (spiral_start) begin
                        sp_st   <= SP_RUN;
                        radius  <= 8'(R_START_ARCMIN);
                        ang_idx <= 3'd0;
                        elapsed <= 10'd0;
                    end
                end

                default: sp_st <= SP_IDLE;
            endcase
        end
    end

endmodule