// =============================================================================
// File        : uav_nav_fsm.sv
// Module      : uav_nav_fsm
// Description : UAV MOD_8 - Navigation FSM top-level.
//               Integrates fsm_controller, waypoint_manager, geofence_checker.
//               32 MAVLink-compatible waypoints in BRAM.
//               Cylindrical geofence with CORDIC-based distance/bearing.
// =============================================================================

`timescale 1ns/1ps

module module8_uav_nav_fsm #(
    parameter int CLK_HZ  = 50_000_000,
    parameter int DATA_W  = 32,   // Q4.28
    parameter int MAX_WP  = 32
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_100hz,
    input  logic ce_10hz,

    // MAVLink commands
    input  logic cmd_arm,
    input  logic cmd_disarm,
    input  logic cmd_takeoff,
    input  logic cmd_land,
    input  logic cmd_rtl,
    input  logic cmd_waypoint,
    input  logic cmd_set_mode,
    input  logic [7:0] cmd_mode,

    // Mission upload data (from MAVLink dispatcher)
    input  logic              wp_wr_en,
    input  logic [4:0]        wp_wr_idx,
    input  logic signed [DATA_W-1:0] wp_wr_lat,
    input  logic signed [DATA_W-1:0] wp_wr_lon,
    input  logic signed [DATA_W-1:0] wp_wr_alt,
    input  logic [DATA_W-1:0] wp_wr_radius,
    input  logic [DATA_W-1:0] wp_wr_speed,
    input  logic [7:0]        wp_wr_cmd,

    // Health / sensor flags
    input  logic preflight_ok,
    input  logic ekf_healthy,
    input  logic gps_fix,
    input  logic baro_valid,
    input  logic wdt_ok,

    // EKF state
    input  logic signed [DATA_W-1:0] ekf_lat,
    input  logic signed [DATA_W-1:0] ekf_lon,
    input  logic signed [DATA_W-1:0] ekf_alt,
    input  logic signed [DATA_W-1:0] ekf_vd,

    // Geofence configuration
    input  logic signed [DATA_W-1:0] home_lat,
    input  logic signed [DATA_W-1:0] home_lon,
    input  logic signed [DATA_W-1:0] home_alt,
    input  logic [DATA_W-1:0]        geofence_radius_sq,
    input  logic [DATA_W-1:0]        geofence_max_alt,

    // PID setpoints to MOD_2
    output logic signed [DATA_W-1:0] sp_alt,
    output logic signed [DATA_W-1:0] sp_vn,
    output logic signed [DATA_W-1:0] sp_ve,
    output logic signed [DATA_W-1:0] sp_yaw,
    output logic signed [DATA_W-1:0] sp_thrust,
    output logic signed [DATA_W-1:0] sp_roll,
    output logic signed [DATA_W-1:0] sp_pitch,
    output logic signed [DATA_W-1:0] sp_vd,

    // Status
    output logic armed,
    output logic [7:0] flight_mode,
    output logic wdt_kick
);

    // -------------------------------------------------------------------------
    // Waypoint manager
    // -------------------------------------------------------------------------
    logic [4:0] wp_idx;
    logic signed [DATA_W-1:0] wp_lat, wp_lon, wp_alt;
    logic [DATA_W-1:0]         wp_radius, wp_speed;
    logic [7:0]                 wp_cmd;
    logic                       wp_valid;
    logic [4:0]                 wp_count;

    waypoint_manager #(.MAX_WP(MAX_WP), .WP_W(DATA_W)) u_wp (
        .clk       (clk),
        .rst_n     (rst_n),
        .wp_idx    (wp_idx),
        .wp_lat    (wp_lat), .wp_lon (wp_lon), .wp_alt (wp_alt),
        .wp_radius (wp_radius), .wp_speed (wp_speed), .wp_cmd (wp_cmd),
        .wp_valid  (wp_valid),
        .wr_en     (wp_wr_en),
        .wr_idx    (wp_wr_idx),
        .wr_lat    (wp_wr_lat), .wr_lon (wp_wr_lon), .wr_alt (wp_wr_alt),
        .wr_radius (wp_wr_radius), .wr_speed (wp_wr_speed), .wr_cmd (wp_wr_cmd),
        .wp_count  (wp_count)
    );

    // -------------------------------------------------------------------------
    // Geofence checker
    // -------------------------------------------------------------------------
    logic geofence_ok;
    logic fence_valid;

    logic geofence_violation;

    geofence_checker #(.POS_W(DATA_W)) u_fence (
        .clk               (clk),
        .rst_n             (rst_n),
        .enable            (ce_10hz),
        .curr_lat          (ekf_lat),
        .curr_lon          (ekf_lon),
        .curr_alt          (ekf_alt),
        .home_lat          (home_lat),
        .home_lon          (home_lon),
        .home_alt          (home_alt),
        .max_radius_sq     (geofence_radius_sq),
        .max_alt           (geofence_max_alt),
        .geofence_violation(geofence_violation),
        .valid             (fence_valid)
    );

    assign geofence_ok = ~geofence_violation;

    // -------------------------------------------------------------------------
    // FSM controller
    // -------------------------------------------------------------------------
    fsm_controller #(.DATA_W(DATA_W), .CLK_HZ(CLK_HZ)) u_fsm (
        .clk          (clk),
        .rst_n        (rst_n),
        .ce_100hz     (ce_100hz),
        .cmd_arm      (cmd_arm),
        .cmd_disarm   (cmd_disarm),
        .cmd_takeoff  (cmd_takeoff),
        .cmd_land     (cmd_land),
        .cmd_rtl      (cmd_rtl),
        .cmd_set_mode (cmd_set_mode),
        .cmd_mode     (cmd_mode),
        .preflight_ok (preflight_ok),
        .ekf_healthy  (ekf_healthy),
        .gps_fix      (gps_fix),
        .baro_valid   (baro_valid),
        .geofence_ok  (geofence_ok),
        .wdt_ok       (wdt_ok),
        .ekf_alt      (ekf_alt),
        .ekf_vd       (ekf_vd),
        .wp_alt       (wp_alt),
        .wp_speed     ($signed(wp_speed)),
        .wp_count     (wp_count),
        .wp_idx_out   (wp_idx),
        .home_lat     (home_lat),
        .home_lon     (home_lon),
        .home_alt     (home_alt),
        .sp_alt       (sp_alt),
        .sp_vn        (sp_vn),
        .sp_ve        (sp_ve),
        .sp_yaw       (sp_yaw),
        .sp_thrust    (sp_thrust),
        .sp_roll      (sp_roll),
        .sp_pitch     (sp_pitch),
        .sp_vd        (sp_vd),
        .armed        (armed),
        .flight_mode  (flight_mode),
        .wdt_kick     (wdt_kick)
    );

endmodule
