// =============================================================================
// Module: i2c_mag_wrapper (CS2 top-level)
// Subsystem: CS2 - Magnetometer Interface (I2C)
// Description: Top-level wrapper integrating i2c_mag_controller, mag_calibration,
//              and mag_fault_detector for the CS2 magnetometer interface.
//
//              i2c_mag_controller performs a burst read: writes register pointer
//              0x03 once then reads 6 consecutive bytes, exploiting HMC5883L /
//              IST8310 auto-increment. This gives ~350 µs latency at 400 kHz,
//              well within the ≤1 ms per 100 Hz cycle budget.
//
//              mag_age_ms tracks milliseconds since the last valid data frame.
//              mag_fault_detector monitors for saturation, stuck values, and
//              noise transients; any fault type is combined into mag_fault.
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module i2c_mag_wrapper #(
    parameter int CLK_HZ   = 100_000_000,
    parameter int I2C_HZ   = 400_000,       // fast-mode
    parameter int MAG_ADDR = 7'h0E          // HMC5883L / IST8310 default
)(
    input  logic        sys_clk,
    input  logic        rst_n,

    // Trigger: one pulse per sample period
    input  logic        mag_read_trigger,

    // I2C physical pins
    inout  wire         i2c_sda,
    output logic        i2c_scl,

    // Calibrated 3-axis outputs: mag_data[0]=X, [1]=Y, [2]=Z (Q15 format)
    output logic signed [15:0] mag_data [0:2],
    // µT outputs: mag_ut[0]=X, [1]=Y, [2]=Z (bit-shift approximation, DSP48-free)
    output logic signed [15:0] mag_ut   [0:2],

    // Status
    output logic        mag_valid,
    output logic        mag_busy,
    output logic        mag_fault,

    // Age counter: milliseconds since last valid data frame (resets on valid)
    output logic [31:0] mag_age_ms
);

    // =========================================================================
    // Millisecond clock-enable (1 kHz) and age counter
    // Counts CLK_HZ/1000 cycles between ticks
    // =========================================================================
    localparam int MS_DIV = CLK_HZ / 1000;

    logic [$clog2(MS_DIV)-1:0] ms_cnt;
    logic ce_1khz;

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_cnt  <= '0;
            ce_1khz <= 1'b0;
        end else begin
            if (ms_cnt == (MS_DIV - 1)) begin
                ms_cnt  <= '0;
                ce_1khz <= 1'b1;
            end else begin
                ms_cnt  <= ms_cnt + 1'b1;
                ce_1khz <= 1'b0;
            end
        end
    end

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            mag_age_ms <= '0;
        end else if (mag_valid) begin
            // mag_valid takes priority over ce_1khz: reset wins if both arrive
            // in the same cycle (extremely unlikely; prevents off-by-one count).
            mag_age_ms <= '0;
        end else if (ce_1khz) begin
            if (mag_age_ms != 32'hFFFF_FFFF)
                mag_age_ms <= mag_age_ms + 32'd1;
        end
    end

    // =========================================================================
    // 100 Hz clock-enable for mag_fault_detector stuck-value timer
    // =========================================================================
    localparam int HZ100_DIV = CLK_HZ / 100;

    logic [$clog2(HZ100_DIV)-1:0] hz100_cnt;
    logic ce_100hz;

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            hz100_cnt <= '0;
            ce_100hz  <= 1'b0;
        end else begin
            if (hz100_cnt == (HZ100_DIV - 1)) begin
                hz100_cnt <= '0;
                ce_100hz  <= 1'b1;
            end else begin
                hz100_cnt <= hz100_cnt + 1'b1;
                ce_100hz  <= 1'b0;
            end
        end
    end

    // =========================================================================
    // I2C mag controller sub-module
    // Drives the I2C bus and delivers raw_x/y/z + data_valid + busy + fault.
    // =========================================================================
    logic signed [15:0] raw_x, raw_y, raw_z;
    logic               ctrl_data_valid;
    logic               ctrl_fault;

    i2c_mag_controller #(
        .CLK_HZ   (CLK_HZ),
        .I2C_HZ   (I2C_HZ),
        .MAG_ADDR (MAG_ADDR)
    ) u_ctrl (
        .sys_clk      (sys_clk),
        .rst_n        (rst_n),
        .read_trigger (mag_read_trigger),
        .i2c_sda      (i2c_sda),
        .i2c_scl      (i2c_scl),
        .raw_x        (raw_x),
        .raw_y        (raw_y),
        .raw_z        (raw_z),
        .data_valid   (ctrl_data_valid),
        .busy         (mag_busy),
        .fault        (ctrl_fault)
    );

    // Latch I2C NACK fault until the next read cycle clears it
    logic fsm_fault;

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n)                fsm_fault <= 1'b0;
        else if (ctrl_fault)       fsm_fault <= 1'b1;
        else if (mag_read_trigger) fsm_fault <= 1'b0;
    end

    // =========================================================================
    // Calibration sub-module (DSP48-free, produces Q15 and µT)
    // Internal signals mag_cal_x/y/z feed the fault detector and are exposed
    // through the mag_data[0:2] and mag_ut[0:2] output arrays.
    // =========================================================================
    logic signed [15:0] mag_cal_x, mag_cal_y, mag_cal_z;
    logic signed [15:0] mag_ut_x,  mag_ut_y,  mag_ut_z;
    logic               cal_fault;

    mag_calibration u_cal (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .raw_x     (raw_x),
        .raw_y     (raw_y),
        .raw_z     (raw_z),
        .raw_valid (ctrl_data_valid),
        .cal_x     (mag_cal_x),
        .cal_y     (mag_cal_y),
        .cal_z     (mag_cal_z),
        .ut_x      (mag_ut_x),
        .ut_y      (mag_ut_y),
        .ut_z      (mag_ut_z),
        .cal_valid (mag_valid),
        .mag_fault (cal_fault)
    );

    // Map internal calibrated signals to the required output arrays
    assign mag_data[0] = mag_cal_x;
    assign mag_data[1] = mag_cal_y;
    assign mag_data[2] = mag_cal_z;

    assign mag_ut[0] = mag_ut_x;
    assign mag_ut[1] = mag_ut_y;
    assign mag_ut[2] = mag_ut_z;

    // =========================================================================
    // Fault detector sub-module
    // Monitors for saturation, stuck values, and noise transients.
    // any_fault is ORed with cal_fault (overflow) and fsm_fault (NACK).
    // =========================================================================
    logic det_sat, det_stuck, det_noise, det_any;

    mag_fault_detector #(
        .CLK_HZ (CLK_HZ)
    ) u_fault (
        .clk        (sys_clk),
        .rst_n      (rst_n),
        .ce_100hz   (ce_100hz),
        .mag_x      (mag_cal_x),
        .mag_y      (mag_cal_y),
        .mag_z      (mag_cal_z),
        .data_valid (mag_valid),
        .fault_sat  (det_sat),
        .fault_stuck(det_stuck),
        .fault_noise(det_noise),
        .any_fault  (det_any)
    );

    assign mag_fault = fsm_fault | cal_fault | det_any;

endmodule