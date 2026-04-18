// =============================================================================
// Module: raster_scan_engine (CS10 SEARCH state boustrophedon raster)
// Subsystem: CS10 - Laser Pointing FSM
// Description: Generates back-and-forth (boustrophedon) raster scan pattern
//              for the SEARCH state.
//
//   Scan parameters (fixed, ±10° az × ±5° el, 0.5° steps):
//     Azimuth range:   -600 to +600 arc-minutes  (±10°)
//     Elevation range: -300 to +300 arc-minutes  (±5°)
//     Step size:        30 arc-minutes (0.5°)
//     Lines:           21  ((-300 to +300)/30 + 1)
//     Steps/line:      41  ((-600 to +600)/30 + 1)
//     Total steps:    ~861 steps; at 10 steps/s (ce_100hz) ≈ 86 s scan
//
//   Pattern:
//     Line 0: az = -600 → +600, el = -300, direction = positive
//     Line 1: az = +600 → -600, el = -270, direction = negative
//     ...
//     Line 20: az = ±600 → ∓600, el = +300
//
//   On scan_done: asserts for one ce_100hz tick, then scan_start needed to restart.
//
// Requirement: CS-LSR-003 (SEARCH State Behavior - raster ±10°az × ±5°el)
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module raster_scan_engine #(
    parameter int AZ_LIMIT    = 600,   // arc-minutes  (±10°)
    parameter int EL_LIMIT    = 300,   // arc-minutes  (±5°)
    parameter int STEP_SIZE   = 30,    // arc-minutes per step (0.5°)
    parameter int STEPS_PER_S = 10     // steps per second (limited by ce_100hz rate)
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_100hz,

    input  logic scan_start,       // strobe to begin/continue scan
    input  logic scan_reset,       // reset to home position

    output logic signed [15:0] gimbal_cmd_az,  // azimuth command (arc-min, signed)
    output logic signed [15:0] gimbal_cmd_el,  // elevation command (arc-min, signed)
    output logic               cmd_valid,      // new command strobe
    output logic               scan_done       // full scan pattern completed
);

    // =========================================================================
    // Derived constants
    // =========================================================================
    localparam int AZ_STEPS = (2 * AZ_LIMIT) / STEP_SIZE;   // 40 intervals → 41 positions
    localparam int EL_STEPS = (2 * EL_LIMIT) / STEP_SIZE;   // 20 intervals → 21 lines

    // =========================================================================
    // State
    // =========================================================================
    typedef enum logic [1:0] {
        IDLE    = 2'd0,
        RUNNING = 2'd1,
        DONE    = 2'd2
    } scan_state_t;

    scan_state_t scan_st;

    // Current scan position (az index and el line)
    logic signed [6:0]  az_idx;    // 0..AZ_STEPS (40), signed for direction
    logic signed [5:0]  el_line;   // 0..EL_STEPS (20)
    logic               az_dir;    // 0=negative, 1=positive
    logic               active;

    // Compute current az/el values in arc-minutes
    logic signed [15:0] cur_az;
    logic signed [15:0] cur_el;

    always_comb begin
        // az: -AZ_LIMIT + az_idx * STEP_SIZE
        cur_az = 16'(-AZ_LIMIT) + 16'(STEP_SIZE) * {9'b0, az_idx};
        // el: -EL_LIMIT + el_line * STEP_SIZE
        cur_el = 16'(-EL_LIMIT) + 16'(STEP_SIZE) * {10'b0, el_line};
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            scan_st      <= IDLE;
            az_idx       <= '0;
            el_line      <= '0;
            az_dir       <= 1'b1;   // start positive
            cmd_valid    <= 1'b0;
            scan_done    <= 1'b0;
            gimbal_cmd_az <= 16'sd0;
            gimbal_cmd_el <= 16'sd0;
        end else begin
            cmd_valid  <= 1'b0;
            scan_done  <= 1'b0;

            if (scan_reset) begin
                scan_st      <= IDLE;
                az_idx       <= '0;
                el_line      <= '0;
                az_dir       <= 1'b1;
                gimbal_cmd_az <= 16'sd0;
                gimbal_cmd_el <= 16'sd0;
            end else begin
                case (scan_st)
                    IDLE: begin
                        if (scan_start)
                            scan_st <= RUNNING;
                    end

                    RUNNING: begin
                        if (ce_100hz) begin
                            // Output current position
                            gimbal_cmd_az <= cur_az;
                            gimbal_cmd_el <= cur_el;
                            cmd_valid     <= 1'b1;

                            // Advance along current line
                            if (az_dir) begin
                                // positive direction: az_idx 0 → AZ_STEPS
                                if (az_idx == 7'(AZ_STEPS)) begin
                                    // end of line - step elevation
                                    if (el_line == 6'(EL_STEPS)) begin
                                        // scan complete
                                        scan_st  <= DONE;
                                        scan_done <= 1'b1;
                                    end else begin
                                        el_line <= el_line + 1;
                                        az_dir  <= 1'b0;
                                    end
                                end else begin
                                    az_idx <= az_idx + 1;
                                end
                            end else begin
                                // negative direction: az_idx AZ_STEPS → 0
                                if (az_idx == 7'd0) begin
                                    if (el_line == 6'(EL_STEPS)) begin
                                        scan_st   <= DONE;
                                        scan_done <= 1'b1;
                                    end else begin
                                        el_line <= el_line + 1;
                                        az_dir  <= 1'b1;
                                    end
                                end else begin
                                    az_idx <= az_idx - 1;
                                end
                            end
                        end
                    end

                    DONE: begin
                        // Wait for scan_start to restart
                        if (scan_start) begin
                            scan_st  <= RUNNING;
                            az_idx   <= '0;
                            el_line  <= '0;
                            az_dir   <= 1'b1;
                        end
                    end

                    default: scan_st <= IDLE;
                endcase
            end
        end
    end

endmodule