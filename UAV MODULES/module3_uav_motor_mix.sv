// =============================================================================
// File        : uav_motor_mix.sv
// Module      : uav_motor_mix
// Description : UAV MOD_3 - Motor Mixing, Output Saturation & ESC PWM top.
//               Instantiates mixing_matrix_4x4, sat_rate_limiter,
//               antiwindup_clamp, and pwm_sync.
//               Supports armed/disarmed interlock (PWM_MIN=1000µs when disarmed).
// =============================================================================

`timescale 1ns/1ps

module module3_uav_motor_mix #(
    parameter int CLK_HZ  = 50_000_000,
    parameter int PWM_HZ  = 400,
    parameter int DATA_W  = 16,   // Q4.12
    parameter int COEF_W  = 16,   // Q2.14 mixing coefficients
    parameter int INTEG_W = 32    // Q16.16 (for antiwindup)
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_1khz,    // Enable from MOD_1

    // Armed flag
    input  logic armed,

    // PID command inputs (Q4.12)
    input  logic signed [DATA_W-1:0] roll_cmd,
    input  logic signed [DATA_W-1:0] pitch_cmd,
    input  logic signed [DATA_W-1:0] yaw_cmd,
    input  logic signed [DATA_W-1:0] thrust_cmd,

    // Mixing matrix coefficients Q2.14 [motor][channel]
    input  logic signed [COEF_W-1:0] mix_coef [0:3][0:3],

    // Saturation and rate limits (per motor)
    input  logic signed [DATA_W-1:0]  out_max    [0:3],
    input  logic signed [DATA_W-1:0]  out_min    [0:3],
    input  logic signed [DATA_W-1:0]  max_delta  [0:3],

    // Anti-windup limits (per motor)
    input  logic signed [INTEG_W-1:0] integ_max [0:3],

    // ESC PWM outputs
    output logic [3:0] pwm_out,

    // Debug
    output logic signed [DATA_W-1:0] motor_cmd_sat [0:3]
);

    // -------------------------------------------------------------------------
    // Motor mixing matrix
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] mixed_cmd [0:3];
    logic                      mix_valid;

    mixing_matrix_4x4 #(
        .CMD_W  (DATA_W),
        .COEF_W (COEF_W),
        .OUT_W  (DATA_W)
    ) u_mix (
        .clk        (clk),
        .rst_n      (rst_n),
        .enable     (ce_1khz),
        .roll_cmd   (roll_cmd),
        .pitch_cmd  (pitch_cmd),
        .yaw_cmd    (yaw_cmd),
        .thrust_cmd (thrust_cmd),
        .mix_coef   (mix_coef),
        .motor_cmd  (mixed_cmd),
        .valid      (mix_valid)
    );

    // -------------------------------------------------------------------------
    // Per-axis output saturation and rate limiter
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] sat_cmd [0:3];
    logic [3:0]                sat_flags;

    sat_rate_limiter #(
        .CHANNELS (4),
        .DATA_W   (DATA_W)
    ) u_sat (
        .clk       (clk),
        .rst_n     (rst_n),
        .enable    (mix_valid),
        .data_in   (mixed_cmd),
        .out_max   (out_max),
        .out_min   (out_min),
        .max_delta (max_delta),
        .data_out  (sat_cmd),
        .saturated (sat_flags)
    );

    // Expose saturated motor commands
    always_comb begin
        for (int m = 0; m < 4; m++)
            motor_cmd_sat[m] = sat_cmd[m];
    end

    // -------------------------------------------------------------------------
    // Anti-windup clamp: prevent integrator windup on saturation
    // -------------------------------------------------------------------------
    logic signed [INTEG_W-1:0] integ_in [0:3];
    logic signed [INTEG_W-1:0] integ_clamped [0:3];
    logic [3:0]                clamped_flags;
    logic signed [DATA_W-1:0]  error_term [0:3];

    // Sign-extend mixed_cmd from DATA_W to INTEG_W for antiwindup input
    always_comb begin
        for (int m = 0; m < 4; m++) begin
            integ_in[m] = {{(INTEG_W-DATA_W){mixed_cmd[m][DATA_W-1]}}, mixed_cmd[m]};
            error_term[m] = mixed_cmd[m] - sat_cmd[m];
        end
    end

    antiwindup_clamp #(
        .AXES    (4),
        .DATA_W  (DATA_W),
        .INTEG_W (INTEG_W)
    ) u_antiwindup (
        .clk        (clk),
        .rst_n      (rst_n),
        .integ_in   (integ_in),
        .integ_max  (integ_max),
        .output_sat (sat_flags),
        .error      (error_term),
        .integ_out  (integ_clamped),
        .clamped    (clamped_flags)
    );

    // -------------------------------------------------------------------------
    // Synchronised ESC PWM generation (armed/disarmed interlock inside)
    // -------------------------------------------------------------------------
    logic [15:0] motor_u16 [0:3];
    always_comb begin
        for (int m = 0; m < 4; m++) begin
            // Map signed [-32768, 32767] → unsigned [0, 32767]
            motor_u16[m] = (sat_cmd[m][DATA_W-1]) ? 16'h0000 :
                                                      sat_cmd[m][14:0];
        end
    end

    pwm_sync #(
        .CLK_HZ   (CLK_HZ),
        .PWM_HZ   (PWM_HZ),
        .CHANNELS (4)
    ) u_pwm (
        .clk       (clk),
        .rst_n     (rst_n),
        .armed     (armed),
        .motor_cmd (motor_u16),
        .pwm_out   (pwm_out)
    );

endmodule