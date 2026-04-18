// =============================================================================
// Module: peak_hold_detector (CS10 signal peak tracking)
// Subsystem: CS10 - Laser Pointing FSM
// Description: Tracks and holds maximum signal strength during SEARCH/ACQUIRE.
//              Records gimbal position at peak signal.
//
//   On each ce_100hz tick when detector_en=1: compare signal_strength to
//   peak_strength.  If greater, update peak and record gimbal position.
//   Hold outputs until reset.
//
// Requirement: CS-LSR-004 (ACQUIRE State Behavior - peak-hold)
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module peak_hold_detector (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    input  logic [11:0] signal_strength,   // filtered signal from signal_monitor
    input  logic [15:0] gimbal_az,         // current azimuth position (arc-min Q15)
    input  logic [15:0] gimbal_el,         // current elevation position (arc-min Q15)
    input  logic        detector_en,       // enable peak tracking (SEARCH/ACQUIRE)
    input  logic        reset,             // clear peak on state transition

    output logic [11:0] peak_strength,     // strongest signal seen
    output logic [15:0] peak_az,           // gimbal az at peak
    output logic [15:0] peak_el,           // gimbal el at peak
    output logic        peak_valid         // asserts once a peak has been found
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            peak_strength <= 12'd0;
            peak_az       <= 16'd0;
            peak_el       <= 16'd0;
            peak_valid    <= 1'b0;
        end else if (reset) begin
            peak_strength <= 12'd0;
            peak_az       <= 16'd0;
            peak_el       <= 16'd0;
            peak_valid    <= 1'b0;
        end else if (ce_100hz && detector_en) begin
            if (signal_strength > peak_strength) begin
                peak_strength <= signal_strength;
                peak_az       <= gimbal_az;
                peak_el       <= gimbal_el;
                peak_valid    <= 1'b1;
            end
        end
    end

endmodule