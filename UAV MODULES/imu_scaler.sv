// =============================================================================
// File        : imu_scaler.sv
// Module      : imu_scaler
// Description : Scales raw ICM-42688 SPI data to Q4.28 fixed-point.
//               Gyro: LSB = 1/131 deg/s (±250 dps full-scale)
//               Accel: LSB = 1/2048 g (±16g full-scale)
//               Converts raw 16-bit signed integers to Q4.28 radians/s and m/s².
// =============================================================================

`timescale 1ns/1ps

module imu_scaler #(
    parameter int RAW_W  = 16,   // raw sensor data width
    parameter int OUT_W  = 32    // Q4.28 output
)(
    input  logic clk,
    input  logic rst_n,
    input  logic valid_in,

    // Raw gyro (deg/s × 131)
    input  logic signed [RAW_W-1:0] gx_raw,
    input  logic signed [RAW_W-1:0] gy_raw,
    input  logic signed [RAW_W-1:0] gz_raw,

    // Raw accel (g × 2048)
    input  logic signed [RAW_W-1:0] ax_raw,
    input  logic signed [RAW_W-1:0] ay_raw,
    input  logic signed [RAW_W-1:0] az_raw,

    // Calibration biases (Q4.28)
    input  logic signed [OUT_W-1:0] gyro_bias_x,
    input  logic signed [OUT_W-1:0] gyro_bias_y,
    input  logic signed [OUT_W-1:0] gyro_bias_z,
    input  logic signed [OUT_W-1:0] accel_bias_x,
    input  logic signed [OUT_W-1:0] accel_bias_y,
    input  logic signed [OUT_W-1:0] accel_bias_z,

    // Scaled outputs (Q4.28 rad/s and m/s²)
    output logic signed [OUT_W-1:0] gx,
    output logic signed [OUT_W-1:0] gy,
    output logic signed [OUT_W-1:0] gz,
    output logic signed [OUT_W-1:0] ax,
    output logic signed [OUT_W-1:0] ay,
    output logic signed [OUT_W-1:0] az,
    output logic                    valid_out
);

    // -------------------------------------------------------------------------
    // Conversion factors (Q4.28):
    //   Gyro: 1/131 deg/s per LSB × π/180 = 0.0001332 rad/s per LSB
    //         × 2^28 = 35,778  ≈ 32'sd35778
    //   Accel: 1/2048 g per LSB × 9.80665 m/s² = 0.004789 m/s² per LSB
    //         × 2^28 = 1,288,490 ≈ 32'sd1288490
    // -------------------------------------------------------------------------
    localparam signed [47:0] GYRO_SCALE  = 48'sd35778;
    localparam signed [47:0] ACCEL_SCALE = 48'sd1288490;

    // Pipeline stage 1: multiply
    logic signed [47:0] gx_m, gy_m, gz_m;
    logic signed [47:0] ax_m, ay_m, az_m;
    logic v1;

    (* use_dsp = "yes" *)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gx_m <= '0; gy_m <= '0; gz_m <= '0;
            ax_m <= '0; ay_m <= '0; az_m <= '0;
            v1   <= 1'b0;
        end else begin
            v1   <= valid_in;
            gx_m <= gx_raw * GYRO_SCALE;
            gy_m <= gy_raw * GYRO_SCALE;
            gz_m <= gz_raw * GYRO_SCALE;
            ax_m <= ax_raw * ACCEL_SCALE;
            ay_m <= ay_raw * ACCEL_SCALE;
            az_m <= az_raw * ACCEL_SCALE;
        end
    end

    // Pipeline stage 2: scale to Q4.28 and subtract bias
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gx <= '0; gy <= '0; gz <= '0;
            ax <= '0; ay <= '0; az <= '0;
            valid_out <= 1'b0;
        end else begin
            valid_out <= v1;
            // Already scaled: raw × factor gives Q4.28 directly
            gx <= gx_m[OUT_W-1:0] - gyro_bias_x;
            gy <= gy_m[OUT_W-1:0] - gyro_bias_y;
            gz <= gz_m[OUT_W-1:0] - gyro_bias_z;
            ax <= ax_m[OUT_W-1:0] - accel_bias_x;
            ay <= ay_m[OUT_W-1:0] - accel_bias_y;
            az <= az_m[OUT_W-1:0] - accel_bias_z;
        end
    end

endmodule
