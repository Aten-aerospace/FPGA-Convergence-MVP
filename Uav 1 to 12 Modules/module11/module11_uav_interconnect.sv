// =============================================================================
// File        : uav_interconnect.sv
// Module      : uav_interconnect
// Description : UAV MOD_11 - Cross-Module Interconnect top-level.
//               Integrates ce_strobe_arbiter and bram_access_mux.
//               Routes signals between all modules:
//                 EKF → PID, FSM → PID, sensors → EKF, MAVLink → FSM
//               Provides centralized signal routing with proper buffering.
// =============================================================================

`timescale 1ns/1ps

module module11_uav_interconnect #(
    parameter int DATA_W = 32,
    parameter int ADDR_W = 10
)(
    input  logic clk,
    input  logic rst_n,

    // ---- CE strobes from MOD_1 ----------------------------------------------
    input  logic ce_1khz_raw,
    input  logic ce_100hz_raw,
    input  logic ce_50hz_raw,
    input  logic ce_10hz_raw,

    // ---- Distributed CE outputs ---------------------------------------------
    output logic ce_pid_inner,
    output logic ce_pid_outer,
    output logic ce_ekf_predict,
    output logic ce_baro_upd,
    output logic ce_gps_upd,
    output logic ce_mag_upd,
    output logic ce_nav,
    output logic ce_1hz,
    output logic ce_5hz,
    output logic ce_50hz,
    output logic ce_10hz,

    // ---- EKF State routing (MOD_4/5 → MOD_2/8) -----------------------------
    input  logic signed [DATA_W-1:0] ekf_state_in [0:8],   // from MOD_5 output
    output logic signed [DATA_W-1:0] ekf_state_pid[0:8],   // to MOD_2
    output logic signed [DATA_W-1:0] ekf_state_nav[0:8],   // to MOD_8

    // ---- BRAM arbitration ports ---------------------------------------------
    // EKF predict write
    input  logic               ekf_pred_wr_en,
    input  logic [ADDR_W-1:0]  ekf_pred_wr_addr,
    input  logic [DATA_W-1:0]  ekf_pred_wr_data,
    // EKF update read
    input  logic               ekf_upd_rd_en,
    input  logic [ADDR_W-1:0]  ekf_upd_rd_addr,
    output logic [DATA_W-1:0]  ekf_upd_rd_data,
    // NAV read
    input  logic               nav_rd_en,
    input  logic [ADDR_W-1:0]  nav_rd_addr,
    output logic [DATA_W-1:0]  nav_rd_data,
    // Waypoint NAV read
    input  logic               wp_nav_rd_en,
    input  logic [ADDR_W-1:0]  wp_nav_rd_addr,
    output logic [DATA_W-1:0]  wp_nav_rd_data,
    // Waypoint mission write
    input  logic               wp_mis_wr_en,
    input  logic [ADDR_W-1:0]  wp_mis_wr_addr,
    input  logic [DATA_W-1:0]  wp_mis_wr_data
);

    // -------------------------------------------------------------------------
    // CE strobe arbiter
    // -------------------------------------------------------------------------
    ce_strobe_arbiter u_ce_arb (
        .clk           (clk), .rst_n (rst_n),
        .ce_1khz_raw   (ce_1khz_raw),
        .ce_100hz_raw  (ce_100hz_raw),
        .ce_50hz_raw   (ce_50hz_raw),
        .ce_10hz_raw   (ce_10hz_raw),
        .ce_pid_inner  (ce_pid_inner),
        .ce_ekf_predict(ce_ekf_predict),
        .ce_pid_outer  (ce_pid_outer),
        .ce_baro_upd   (ce_baro_upd),
        .ce_gps_upd    (ce_gps_upd),
        .ce_mag_upd    (ce_mag_upd),
        .ce_1hz        (ce_1hz),
        .ce_5hz        (ce_5hz),
        .ce_nav        (ce_nav),
        .ce_50hz       (ce_50hz),
        .ce_10hz       (ce_10hz)
    );

    // -------------------------------------------------------------------------
    // BRAM access multiplexer
    // -------------------------------------------------------------------------
    bram_access_mux #(.DATA_W(DATA_W), .ADDR_W(ADDR_W)) u_bram_mux (
        .clk             (clk),
        .ekf_pred_wr_en  (ekf_pred_wr_en),
        .ekf_pred_wr_addr(ekf_pred_wr_addr),
        .ekf_pred_wr_data(ekf_pred_wr_data),
        .ekf_upd_rd_en   (ekf_upd_rd_en),
        .ekf_upd_rd_addr (ekf_upd_rd_addr),
        .ekf_upd_rd_data (ekf_upd_rd_data),
        .nav_rd_en       (nav_rd_en),
        .nav_rd_addr     (nav_rd_addr),
        .nav_rd_data     (nav_rd_data),
        .wp_nav_rd_en    (wp_nav_rd_en),
        .wp_nav_rd_addr  (wp_nav_rd_addr),
        .wp_nav_rd_data  (wp_nav_rd_data),
        .wp_mis_wr_en    (wp_mis_wr_en),
        .wp_mis_wr_addr  (wp_mis_wr_addr),
        .wp_mis_wr_data  (wp_mis_wr_data),
        .ekf_grant_pred  (),
        .ekf_grant_upd   (),
        .ekf_grant_nav   (),
        .wp_grant_nav    (),
        .wp_grant_mis    ()
    );

    // -------------------------------------------------------------------------
    // EKF state routing (registered for timing closure)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < 9; s++) begin
                ekf_state_pid[s] <= '0;
                ekf_state_nav[s] <= '0;
            end
        end else begin
            for (int s = 0; s < 9; s++) begin
                ekf_state_pid[s] <= ekf_state_in[s];
                ekf_state_nav[s] <= ekf_state_in[s];
            end
        end
    end

endmodule
