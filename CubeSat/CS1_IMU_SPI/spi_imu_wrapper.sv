// =============================================================================
// Module: spi_imu_wrapper (CS1 top-level wrapper)
// Subsystem: CS1 - IMU Sensor Interface (SPI)
// Description: Top-level integration of imu_controller and imu_data_handler.
//              Exposes calibrated 9-axis sensor data to downstream ADCS logic.
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module spi_imu_wrapper #(
    parameter int CLK_HZ   = 100_000_000,
    parameter int SPI_HZ   = 10_000_000
)(
    input  logic        clk_100mhz,  // SPI controller clock domain
    input  logic        sys_clk,     // downstream system clock domain (CDC target)
    input  logic        rst_n,

    // Trigger: one pulse per sample period (e.g. 100 Hz CE)
    input  logic        imu_read_trigger,

    // SPI pins (connect to FPGA top-level I/O)
    output logic        spi_sclk,
    output logic        spi_mosi,
    input  logic        spi_miso,
    output logic        spi_cs_n,

    // Calibrated sensor outputs (Q15 fixed-point) - stable between valid pulses
    output logic signed [15:0] accel_x, accel_y, accel_z,
    output logic signed [15:0] gyro_x,  gyro_y,  gyro_z,
    output logic signed [15:0] mag_x,   mag_y,   mag_z,

    // Status (sys_clk domain via CDC synchronizer)
    output logic        imu_data_valid,  // synchronized strobe: all 9 axes updated
    output logic        crc_pass,        // synchronized strobe: frame received cleanly
    output logic        imu_busy,
    output logic        imu_fault,
    output logic        imu_overflow
);

    // =========================================================================
    // Internal wires between imu_controller and imu_data_handler
    // =========================================================================
    logic signed [15:0] raw_accel_x, raw_accel_y, raw_accel_z;
    logic signed [15:0] raw_gyro_x,  raw_gyro_y,  raw_gyro_z;
    logic signed [15:0] raw_mag_x,   raw_mag_y,   raw_mag_z;
    logic               raw_valid;
    logic               raw_crc_pass;  // crc_pass in clk_100mhz domain
    logic               cal_valid_r;   // cal_valid in clk_100mhz domain (pre-CDC)

    // =========================================================================
    // CS1 Sub-module: IMU SPI Controller
    // =========================================================================
    imu_controller #(
        .CLK_HZ   (CLK_HZ),
        .SPI_HZ   (SPI_HZ)
    ) u_imu_ctrl (
        .clk_100mhz   (clk_100mhz),
        .rst_n        (rst_n),
        .read_trigger (imu_read_trigger),
        .spi_sclk     (spi_sclk),
        .spi_mosi     (spi_mosi),
        .spi_miso     (spi_miso),
        .spi_cs_n     (spi_cs_n),
        .accel_x      (raw_accel_x),
        .accel_y      (raw_accel_y),
        .accel_z      (raw_accel_z),
        .gyro_x       (raw_gyro_x),
        .gyro_y       (raw_gyro_y),
        .gyro_z       (raw_gyro_z),
        .mag_x        (raw_mag_x),
        .mag_y        (raw_mag_y),
        .mag_z        (raw_mag_z),
        .data_valid   (raw_valid),
        .busy         (imu_busy),
        .fault        (imu_fault),
        .crc_pass     (raw_crc_pass)
    );

    // =========================================================================
    // CS1 Sub-module: IMU Data Handler (bias removal, saturation)
    // =========================================================================
    imu_data_handler u_imu_data (
        .clk         (clk_100mhz),
        .rst_n       (rst_n),
        .raw_accel_x (raw_accel_x),
        .raw_accel_y (raw_accel_y),
        .raw_accel_z (raw_accel_z),
        .raw_gyro_x  (raw_gyro_x),
        .raw_gyro_y  (raw_gyro_y),
        .raw_gyro_z  (raw_gyro_z),
        .raw_mag_x   (raw_mag_x),
        .raw_mag_y   (raw_mag_y),
        .raw_mag_z   (raw_mag_z),
        .raw_valid   (raw_valid),
        .cal_accel_x (accel_x),
        .cal_accel_y (accel_y),
        .cal_accel_z (accel_z),
        .cal_gyro_x  (gyro_x),
        .cal_gyro_y  (gyro_y),
        .cal_gyro_z  (gyro_z),
        .cal_mag_x   (mag_x),
        .cal_mag_y   (mag_y),
        .cal_mag_z   (mag_z),
        .cal_valid   (cal_valid_r),
        .overflow_flag (imu_overflow),
        .fault_axis    ()
    );

    // =========================================================================
    // CDC: clk_100mhz → sys_clk (2-FF synchronizers)
    // Both imu_data_valid and crc_pass are single-cycle pulses at 100 Hz.
    // At sys_clk ≤ clk_100mhz the pulse is always captured within 2 cycles.
    // Multi-bit sensor data registers are directly connected; they remain
    // stable for ≥ 9 ms between updates (100 Hz rate), so no FIFO is required.
    // =========================================================================
    synchronizer #(.STAGES(2), .WIDTH(1)) u_sync_valid (
        .dst_clk  (sys_clk),
        .async_in (cal_valid_r),
        .sync_out (imu_data_valid)
    );

    synchronizer #(.STAGES(2), .WIDTH(1)) u_sync_crc (
        .dst_clk  (sys_clk),
        .async_in (raw_crc_pass),
        .sync_out (crc_pass)
    );

endmodule