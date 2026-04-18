// =============================================================================
// File        : preflight_checker.sv
// Module      : preflight_checker
// Description : Pre-flight checklist module.
//               Asserts preflight_ok only when all checks pass:
//                 1. EKF_HEALTHY
//                 2. GPS_FIX ≥ 3D (fix_type ≥ 3)
//                 3. HDOP ≤ 2.5 (hdop_q8 ≤ 640 at ×256 scaling)
//                 4. BARO_VALID
//                 5. No sensor faults
//               Individual fault flags for diagnostics.
// =============================================================================

`timescale 1ns/1ps

module preflight_checker (
    input  logic clk,
    input  logic rst_n,

    // Health inputs
    input  logic       ekf_healthy,
    input  logic [3:0] gps_fix_type,   // ≥3 required
    input  logic [15:0] gps_hdop_q8,   // HDOP × 256; ≤640 = HDOP ≤ 2.5
    input  logic       baro_valid,
    input  logic       mag_valid,
    input  logic       imu_fault,      // 1 = fault detected
    input  logic       gcs_present,    // optional GCS connection check

    // Pre-arm override (from AXI register, disables individual checks)
    input  logic [7:0] check_mask,     // 1 = check enabled

    // Outputs
    output logic preflight_ok,
    output logic [7:0] fault_flags     // individual fault bits
);

    // Fault bit assignments
    // [0]=EKF  [1]=GPS_FIX  [2]=HDOP  [3]=BARO  [4]=MAG  [5]=IMU  [6]=GCS  [7]=reserved
    localparam logic [7:0] BIT_EKF     = 8'h01;
    localparam logic [7:0] BIT_GPS_FIX = 8'h02;
    localparam logic [7:0] BIT_HDOP    = 8'h04;
    localparam logic [7:0] BIT_BARO    = 8'h08;
    localparam logic [7:0] BIT_MAG     = 8'h10;
    localparam logic [7:0] BIT_IMU     = 8'h20;
    localparam logic [7:0] BIT_GCS     = 8'h40;

    localparam int HDOP_THRESH = 640; // 2.5 × 256

    logic [7:0] raw_faults;

    always_comb begin
        raw_faults = 8'h00;
        if (!ekf_healthy)             raw_faults |= BIT_EKF;
        if (gps_fix_type < 4'd3)      raw_faults |= BIT_GPS_FIX;
        if (gps_hdop_q8 > HDOP_THRESH) raw_faults |= BIT_HDOP;
        if (!baro_valid)              raw_faults |= BIT_BARO;
        if (!mag_valid)               raw_faults |= BIT_MAG;
        if (imu_fault)                raw_faults |= BIT_IMU;
        if (!gcs_present)             raw_faults |= BIT_GCS;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            preflight_ok <= 1'b0;
            fault_flags  <= 8'hFF; // all faults at reset
        end else begin
            // Apply check mask: only enabled checks contribute to fault
            fault_flags  <= raw_faults & check_mask;
            preflight_ok <= ((raw_faults & check_mask) == 8'h00);
        end
    end

endmodule
