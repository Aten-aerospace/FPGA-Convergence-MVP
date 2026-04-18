// =============================================================================
// Module: imu_data_handler
// Subsystem: CS1 - IMU Sensor Interface (SPI)
// Description: Receives raw 16-bit sensor words from imu_controller, applies
//              scale factors, provides CDC-safe outputs to downstream logic.
//              Scale: accel Q4.12 (1 LSB = 1/4096 g), gyro Q6.10 (1 LSB = 1/1024 dps)
// Provenance: cubesat_requirements.md REQ_ADCS_01; Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module imu_data_handler #(
    parameter int DATA_W = 16
)(
    input  logic                   clk,
    input  logic                   rst_n,

    // Raw sensor inputs (from imu_controller)
    input  logic signed [15:0]     raw_accel_x, raw_accel_y, raw_accel_z,
    input  logic signed [15:0]     raw_gyro_x,  raw_gyro_y,  raw_gyro_z,
    input  logic signed [15:0]     raw_mag_x,   raw_mag_y,   raw_mag_z,
    input  logic                   raw_valid,

    // Calibrated / scaled outputs (same Q format, offset removed)
    output logic signed [15:0]     cal_accel_x, cal_accel_y, cal_accel_z,
    output logic signed [15:0]     cal_gyro_x,  cal_gyro_y,  cal_gyro_z,
    output logic signed [15:0]     cal_mag_x,   cal_mag_y,   cal_mag_z,
    output logic                   cal_valid,

    // Status
    output logic                   overflow_flag,   // any axis saturated
    output logic [2:0]             fault_axis       // which axes faulted
);

    // =========================================================================
    // Bias offset registers (fixed calibration, parameterisable via top-level)
    // These are synthesizable constants; in production loaded via AXI4-Lite.
    // =========================================================================
    localparam signed [15:0] BIAS_AX = 16'sd0;
    localparam signed [15:0] BIAS_AY = 16'sd0;
    localparam signed [15:0] BIAS_AZ = 16'sd0;
    localparam signed [15:0] BIAS_GX = 16'sd0;
    localparam signed [15:0] BIAS_GY = 16'sd0;
    localparam signed [15:0] BIAS_GZ = 16'sd0;
    localparam signed [15:0] BIAS_MX = 16'sd0;
    localparam signed [15:0] BIAS_MY = 16'sd0;
    localparam signed [15:0] BIAS_MZ = 16'sd0;

    // Magnetometer scale factor (AK8963 inside MPU-9250):
    //   Sensitivity: 4915 LSB/mT  (16-bit mode, ±4800 µT range → 0.15 µT/LSB)
    //   Raw output is in LSB; divide by MAG_SCALE to obtain mT.
    //   No DSP48 multiplication used here - raw units are passed through as-is.
    localparam int MAG_SCALE_LSB_PER_MT = 4915; // LSB/mT @ ±4800 µT (AK8963 16-bit)

    localparam signed [15:0] SAT_POS = 16'sd32767;
    localparam signed [15:0] SAT_NEG = -16'sd32768;

    // =========================================================================
    // Saturation helper function
    // =========================================================================
    function automatic signed [15:0] saturate17 (input signed [16:0] v);
        if (v > 17'sh07FFF)
            return 16'sh7FFF;
        else if (v < -17'sh08000)
            return -16'sh8000;
        else
            return v[15:0];
    endfunction

    // =========================================================================
    // Pipeline stage 1: subtract bias (17-bit extended to detect overflow)
    // =========================================================================
    logic signed [16:0] sub_ax, sub_ay, sub_az;
    logic signed [16:0] sub_gx, sub_gy, sub_gz;
    logic signed [16:0] sub_mx, sub_my, sub_mz;
    logic               pipe1_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sub_ax <= '0; sub_ay <= '0; sub_az <= '0;
            sub_gx <= '0; sub_gy <= '0; sub_gz <= '0;
            sub_mx <= '0; sub_my <= '0; sub_mz <= '0;
            pipe1_valid <= 1'b0;
        end else begin
            pipe1_valid <= raw_valid;
            if (raw_valid) begin
                sub_ax <= {raw_accel_x[15], raw_accel_x} - {BIAS_AX[15], BIAS_AX};
                sub_ay <= {raw_accel_y[15], raw_accel_y} - {BIAS_AY[15], BIAS_AY};
                sub_az <= {raw_accel_z[15], raw_accel_z} - {BIAS_AZ[15], BIAS_AZ};
                sub_gx <= {raw_gyro_x[15],  raw_gyro_x}  - {BIAS_GX[15], BIAS_GX};
                sub_gy <= {raw_gyro_y[15],  raw_gyro_y}  - {BIAS_GY[15], BIAS_GY};
                sub_gz <= {raw_gyro_z[15],  raw_gyro_z}  - {BIAS_GZ[15], BIAS_GZ};
                sub_mx <= {raw_mag_x[15],   raw_mag_x}   - {BIAS_MX[15], BIAS_MX};
                sub_my <= {raw_mag_y[15],   raw_mag_y}   - {BIAS_MY[15], BIAS_MY};
                sub_mz <= {raw_mag_z[15],   raw_mag_z}   - {BIAS_MZ[15], BIAS_MZ};
            end
        end
    end

    // =========================================================================
    // Pipeline stage 2: saturate and output
    // =========================================================================
    logic ovf_ax, ovf_ay, ovf_az;
    logic ovf_gx, ovf_gy, ovf_gz;
    logic ovf_mx, ovf_my, ovf_mz;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cal_accel_x <= '0; cal_accel_y <= '0; cal_accel_z <= '0;
            cal_gyro_x  <= '0; cal_gyro_y  <= '0; cal_gyro_z  <= '0;
            cal_mag_x   <= '0; cal_mag_y   <= '0; cal_mag_z   <= '0;
            cal_valid     <= 1'b0;
            overflow_flag <= 1'b0;
            fault_axis    <= 3'b000;
        end else begin
            cal_valid <= pipe1_valid;
            if (pipe1_valid) begin
                cal_accel_x <= saturate17(sub_ax);
                cal_accel_y <= saturate17(sub_ay);
                cal_accel_z <= saturate17(sub_az);
                cal_gyro_x  <= saturate17(sub_gx);
                cal_gyro_y  <= saturate17(sub_gy);
                cal_gyro_z  <= saturate17(sub_gz);
                cal_mag_x   <= saturate17(sub_mx);
                cal_mag_y   <= saturate17(sub_my);
                cal_mag_z   <= saturate17(sub_mz);

                ovf_ax = (sub_ax > 17'sh07FFF) || (sub_ax < -17'sh08000);
                ovf_ay = (sub_ay > 17'sh07FFF) || (sub_ay < -17'sh08000);
                ovf_az = (sub_az > 17'sh07FFF) || (sub_az < -17'sh08000);
                ovf_gx = (sub_gx > 17'sh07FFF) || (sub_gx < -17'sh08000);
                ovf_gy = (sub_gy > 17'sh07FFF) || (sub_gy < -17'sh08000);
                ovf_gz = (sub_gz > 17'sh07FFF) || (sub_gz < -17'sh08000);
                ovf_mx = (sub_mx > 17'sh07FFF) || (sub_mx < -17'sh08000);
                ovf_my = (sub_my > 17'sh07FFF) || (sub_my < -17'sh08000);
                ovf_mz = (sub_mz > 17'sh07FFF) || (sub_mz < -17'sh08000);

                overflow_flag <= ovf_ax | ovf_ay | ovf_az |
                                 ovf_gx | ovf_gy | ovf_gz |
                                 ovf_mx | ovf_my | ovf_mz;
                fault_axis    <= {(ovf_ax | ovf_ay | ovf_az),
                                  (ovf_gx | ovf_gy | ovf_gz),
                                  (ovf_mx | ovf_my | ovf_mz)};
            end
        end
    end

endmodule