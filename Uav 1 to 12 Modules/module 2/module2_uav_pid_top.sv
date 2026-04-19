// =============================================================================
// File        : uav_pid_top.sv
// Module      : uav_pid_top
// Description : UAV MOD_2 - Dual-Loop PID Controller top-level.
//               Integrates angle limiting, velocity→angle conversion,
//               and time-multiplexed 8-axis PID scheduling.
//               Inner 1 kHz: Roll/Pitch/Yaw rate PIDs
//               Outer 100 Hz: Roll/Pitch/Alt attitude PIDs, VN/VE velocity PDs
//               Flight-mode presets from BRAM (4 pages × 18 gain words).
// =============================================================================

`timescale 1ns/1ps

module module2_uav_pid_top #(
    parameter int CLK_HZ  = 50_000_000,
    parameter int DATA_W  = 16,  // Q4.12
    parameter int INTEG_W = 32,  // Q16.16
    parameter int COEFF_W = 16   // Q4.12
)(
    input  logic clk,
    input  logic rst_n,

    // CE strobes
    input  logic ce_1khz,
    input  logic ce_100hz,

    // ---- EKF State inputs --------------------------------------------------
    input  logic signed [DATA_W-1:0] ekf_roll,
    input  logic signed [DATA_W-1:0] ekf_pitch,
    input  logic signed [DATA_W-1:0] ekf_yaw,
    input  logic signed [DATA_W-1:0] ekf_roll_rate,
    input  logic signed [DATA_W-1:0] ekf_pitch_rate,
    input  logic signed [DATA_W-1:0] ekf_yaw_rate,
    input  logic signed [DATA_W-1:0] ekf_alt,
    input  logic signed [DATA_W-1:0] ekf_vn,
    input  logic signed [DATA_W-1:0] ekf_ve,

    // ---- Navigation setpoints (from MOD_8) ---------------------------------
    input  logic signed [DATA_W-1:0] nav_roll_sp,
    input  logic signed [DATA_W-1:0] nav_pitch_sp,
    input  logic signed [DATA_W-1:0] nav_yaw_sp,
    input  logic signed [DATA_W-1:0] nav_alt_sp,
    input  logic signed [DATA_W-1:0] nav_vn_sp,
    input  logic signed [DATA_W-1:0] nav_ve_sp,
    input  logic signed [DATA_W-1:0] nav_thrust_sp,

    // ---- Gain registers (from MOD_10 AXI interface) -----------------------
    input  logic signed [COEFF_W-1:0] gains    [0:7][0:2],
    input  logic signed [INTEG_W-1:0] integ_max[0:7],
    input  logic signed [DATA_W-1:0]  out_max  [0:7],
    input  logic signed [DATA_W-1:0]  out_min  [0:7],

    // Velocity-to-angle gain
    input  logic signed [DATA_W-1:0] kv,

    // Clear integrators on mode change
    input  logic [7:0] clear_integ,

    // ---- Outputs to motor mixer (MOD_3) ------------------------------------
    output logic signed [DATA_W-1:0] roll_rate_cmd,
    output logic signed [DATA_W-1:0] pitch_rate_cmd,
    output logic signed [DATA_W-1:0] yaw_rate_cmd,
    output logic signed [DATA_W-1:0] thrust_cmd,

    // Debug / telemetry
    output logic signed [DATA_W-1:0] pid_out [0:7],
    output logic [7:0]               pid_valid
);

    // -------------------------------------------------------------------------
    // Velocity → angle conversion (100 Hz)
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] v2a_roll_sp, v2a_pitch_sp;

    velocity_to_angle #(.DATA_W(DATA_W)) u_v2a (
        .clk      (clk),
        .rst_n    (rst_n),
        .enable   (ce_100hz),
        .vn_error (nav_vn_sp - ekf_vn),
        .ve_error (nav_ve_sp - ekf_ve),
        .kv       (kv),
        .pitch_sp (v2a_pitch_sp),
        .roll_sp  (v2a_roll_sp)
    );

    // Blend: nav provides direct roll/pitch or velocity cascades
    // Here we use nav setpoints directly for simplicity (cascade optional)
    logic signed [DATA_W-1:0] blended_roll_sp, blended_pitch_sp;
    always_comb begin
        blended_roll_sp  = nav_roll_sp  + v2a_roll_sp;
        blended_pitch_sp = nav_pitch_sp + v2a_pitch_sp;
    end

    // -------------------------------------------------------------------------
    // Angle limiter
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] lim_roll, lim_pitch, lim_yaw, lim_alt, lim_vn, lim_ve;

    angle_limiter #(.DATA_W(DATA_W)) u_lim (
        .clk         (clk),
        .rst_n       (rst_n),
        .roll_sp_in  (blended_roll_sp),
        .pitch_sp_in (blended_pitch_sp),
        .yaw_sp_in   (nav_yaw_sp),
        .alt_sp_in   (nav_alt_sp),
        .vn_sp_in    (nav_vn_sp),
        .ve_sp_in    (nav_ve_sp),
        .roll_sp_out (lim_roll),
        .pitch_sp_out(lim_pitch),
        .yaw_sp_out  (lim_yaw),
        .alt_sp_out  (lim_alt),
        .vn_sp_out   (lim_vn),
        .ve_sp_out   (lim_ve)
    );

    // -------------------------------------------------------------------------
    // Outer-loop → inner-loop setpoints:
    //   Outer PID axes 3-5 (roll/pitch/alt) outputs drive inner roll/pitch/yaw-rate SPs
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] inner_roll_rate_sp;
    logic signed [DATA_W-1:0] inner_pitch_rate_sp;
    logic signed [DATA_W-1:0] inner_yaw_rate_sp;

    // Outer PID outputs feed inner setpoints
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            inner_roll_rate_sp  <= '0;
            inner_pitch_rate_sp <= '0;
            inner_yaw_rate_sp   <= '0;
        end else if (pid_valid[3]) begin
            inner_roll_rate_sp  <= pid_out[3];
        end else if (pid_valid[4]) begin
            inner_pitch_rate_sp <= pid_out[4];
        end
        // Yaw rate comes directly from navigation
        inner_yaw_rate_sp <= nav_yaw_sp;
    end

    // -------------------------------------------------------------------------
    // Time-multiplexed PID mux
    // -------------------------------------------------------------------------
    pid_loop_mux #(
        .DATA_W  (DATA_W),
        .INTEG_W (INTEG_W),
        .COEFF_W (COEFF_W)
    ) u_mux (
        .clk            (clk),
        .rst_n          (rst_n),
        .ce_1khz        (ce_1khz),
        .ce_100hz       (ce_100hz),
        .sp_roll_rate   (inner_roll_rate_sp),
        .sp_pitch_rate  (inner_pitch_rate_sp),
        .sp_yaw_rate    (inner_yaw_rate_sp),
        .sp_roll        (lim_roll),
        .sp_pitch       (lim_pitch),
        .sp_alt         (lim_alt),
        .sp_vn          (lim_vn),
        .sp_ve          (lim_ve),
        .meas_roll_rate (ekf_roll_rate),
        .meas_pitch_rate(ekf_pitch_rate),
        .meas_yaw_rate  (ekf_yaw_rate),
        .meas_roll      (ekf_roll),
        .meas_pitch     (ekf_pitch),
        .meas_alt       (ekf_alt),
        .meas_vn        (ekf_vn),
        .meas_ve        (ekf_ve),
        .gains          (gains),
        .integ_max      (integ_max),
        .out_max        (out_max),
        .out_min        (out_min),
        .clear_integ    (clear_integ),
        .pid_out        (pid_out),
        .pid_valid      (pid_valid),
        .roll_rate_cmd  (roll_rate_cmd),
        .pitch_rate_cmd (pitch_rate_cmd),
        .yaw_rate_cmd   (yaw_rate_cmd)
    );

    // Thrust passthrough (from navigation)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) thrust_cmd <= '0;
        else        thrust_cmd <= nav_thrust_sp;
    end

endmodule
