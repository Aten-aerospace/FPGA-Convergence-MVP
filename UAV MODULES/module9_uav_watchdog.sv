// =============================================================================
// File        : uav_watchdog.sv
// Module      : uav_watchdog
// Description : UAV MOD_9 - Watchdog Timer + Pre-Flight + Emergency top.
//               Integrates watchdog_timer, preflight_checker, emergency_descent.
// =============================================================================

`timescale 1ns/1ps

module module9_uav_watchdog #(
    parameter int CLK_HZ  = 50_000_000,
    parameter int DATA_W  = 32
)(
    input  logic clk,
    input  logic rst_n,

    // Watchdog kick (from MOD_8, 100Hz)
    input  logic wdt_kick,
    input  logic armed,

    // WDT configuration (from MOD_10 AXI)
    input  logic        wdt_config_en,
    input  logic [10:0] wdt_timeout_ms,

    // Pre-flight check inputs
    input  logic       ekf_healthy,
    input  logic [3:0] gps_fix_type,
    input  logic [15:0] gps_hdop_q8,
    input  logic       baro_valid,
    input  logic       mag_valid,
    input  logic       imu_fault,
    input  logic       gcs_present,
    input  logic [7:0] check_mask,

    // Emergency descent inputs
    input  logic signed [DATA_W-1:0] curr_lat,
    input  logic signed [DATA_W-1:0] curr_lon,
    input  logic signed [DATA_W-1:0] curr_alt,

    // Outputs
    output logic wdt_ok,
    output logic wdt_expired,
    output logic preflight_ok,
    output logic [7:0] fault_flags,
    output logic emerg_active,

    output logic signed [DATA_W-1:0] emerg_sp_lat,
    output logic signed [DATA_W-1:0] emerg_sp_lon,
    output logic signed [DATA_W-1:0] emerg_sp_alt,
    output logic signed [DATA_W-1:0] emerg_sp_thrust,
    output logic signed [DATA_W-1:0] emerg_sp_vd
);

    // -------------------------------------------------------------------------
    // Watchdog timer
    // -------------------------------------------------------------------------
    watchdog_timer #(.CLK_HZ(CLK_HZ)) u_wdt (
        .clk         (clk),
        .rst_n       (rst_n),
        .kick        (wdt_kick),
        .config_en   (wdt_config_en),
        .timeout_ms  (wdt_timeout_ms),
        .armed       (armed),
        .wdt_expired (wdt_expired),
        .wdt_cnt_out ()
    );

    assign wdt_ok = ~wdt_expired;

    // -------------------------------------------------------------------------
    // Pre-flight checker
    // -------------------------------------------------------------------------
    preflight_checker u_pfc (
        .clk          (clk),
        .rst_n        (rst_n),
        .ekf_healthy  (ekf_healthy),
        .gps_fix_type (gps_fix_type),
        .gps_hdop_q8  (gps_hdop_q8),
        .baro_valid   (baro_valid),
        .mag_valid    (mag_valid),
        .imu_fault    (imu_fault),
        .gcs_present  (gcs_present),
        .check_mask   (check_mask),
        .preflight_ok (preflight_ok),
        .fault_flags  (fault_flags)
    );

    // -------------------------------------------------------------------------
    // Emergency descent (activated on WDT expiry)
    // -------------------------------------------------------------------------
    emergency_descent #(.DATA_W(DATA_W)) u_emerg (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (wdt_expired),
        .curr_lat  (curr_lat),
        .curr_lon  (curr_lon),
        .curr_alt  (curr_alt),
        .sp_lat    (emerg_sp_lat),
        .sp_lon    (emerg_sp_lon),
        .sp_alt    (emerg_sp_alt),
        .sp_thrust (emerg_sp_thrust),
        .sp_vd     (emerg_sp_vd),
        .active    (emerg_active)
    );

endmodule