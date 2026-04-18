// =============================================================================
// Module: mag_fault_detector
// Subsystem: CS2 - Magnetometer Interface (I2C)
// Description: Monitors magnetometer readings for three fault classes:
//              fault_sat  - any axis exceeds MAG_SAT_THRESH in absolute value.
//              fault_stuck - a reading has not changed for > STUCK_TIMEOUT
//                            ce_100hz ticks.
//              fault_noise - abrupt inter-sample change > NOISE_THRESH_P on any
//                            axis (default 8192 LSB ≈ 0.25 Q15, ~3-sigma of
//                            HMC5883L noise floor).
//              any_fault  - OR of all three.
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module mag_fault_detector #(
    parameter int CLK_HZ         = 100_000_000,
    parameter int MAG_SAT_THRESH = 32000,    // Q15 saturation threshold (~0.98 full-scale)
    parameter int STUCK_TIMEOUT  = 100,      // ce_100hz ticks before stuck fault
    // Noise threshold: 8192 LSBs ≈ 0.25 in Q15, chosen to exceed 3-sigma
    // sensor noise while remaining sensitive to real transients (per HMC5883L datasheet)
    parameter int NOISE_THRESH_P = 8192
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,           // 100 Hz clock enable

    input  logic signed [15:0] mag_x,
    input  logic signed [15:0] mag_y,
    input  logic signed [15:0] mag_z,
    input  logic        data_valid,

    output logic        fault_sat,
    output logic        fault_stuck,
    output logic        fault_noise,
    output logic        any_fault
);

    // =========================================================================
    // Saturation fault: |axis| > MAG_SAT_THRESH
    // =========================================================================
    logic signed [15:0] abs_x, abs_y, abs_z;

    always_comb begin
        abs_x = (mag_x[15]) ? -mag_x : mag_x;
        abs_y = (mag_y[15]) ? -mag_y : mag_y;
        abs_z = (mag_z[15]) ? -mag_z : mag_z;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fault_sat <= 1'b0;
        end else if (data_valid) begin
            fault_sat <= (abs_x > MAG_SAT_THRESH[15:0]) |
                         (abs_y > MAG_SAT_THRESH[15:0]) |
                         (abs_z > MAG_SAT_THRESH[15:0]);
        end
    end

    // =========================================================================
    // Stuck fault: value unchanged for > STUCK_TIMEOUT ce_100hz ticks
    // =========================================================================
    logic signed [15:0] prev_x, prev_y, prev_z;
    logic [$clog2(STUCK_TIMEOUT+2)-1:0] stuck_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_x    <= '0;
            prev_y    <= '0;
            prev_z    <= '0;
            stuck_cnt <= '0;
            fault_stuck <= 1'b0;
        end else if (data_valid && ce_100hz) begin
            if ((mag_x == prev_x) && (mag_y == prev_y) && (mag_z == prev_z)) begin
                if (stuck_cnt < STUCK_TIMEOUT[$clog2(STUCK_TIMEOUT+2)-1:0]) begin
                    stuck_cnt <= stuck_cnt + 1'b1;
                end
                fault_stuck <= (stuck_cnt >= STUCK_TIMEOUT[$clog2(STUCK_TIMEOUT+2)-1:0]);
            end else begin
                stuck_cnt   <= '0;
                fault_stuck <= 1'b0;
                prev_x      <= mag_x;
                prev_y      <= mag_y;
                prev_z      <= mag_z;
            end
        end
    end

    // =========================================================================
    // Noise fault: |delta| > 8192 between consecutive valid samples
    // =========================================================================
    // Noise threshold: parameterised via NOISE_THRESH_P
    localparam logic signed [15:0] NOISE_THRESH = NOISE_THRESH_P[15:0];

    logic signed [15:0] last_x, last_y, last_z;
    logic signed [15:0] delta_x, delta_y, delta_z;
    logic signed [15:0] abs_dx, abs_dy, abs_dz;

    always_comb begin
        delta_x = mag_x - last_x;
        delta_y = mag_y - last_y;
        delta_z = mag_z - last_z;
        abs_dx  = delta_x[15] ? -delta_x : delta_x;
        abs_dy  = delta_y[15] ? -delta_y : delta_y;
        abs_dz  = delta_z[15] ? -delta_z : delta_z;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_x      <= '0;
            last_y      <= '0;
            last_z      <= '0;
            fault_noise <= 1'b0;
        end else if (data_valid) begin
            fault_noise <= (abs_dx > NOISE_THRESH) |
                           (abs_dy > NOISE_THRESH) |
                           (abs_dz > NOISE_THRESH);
            last_x <= mag_x;
            last_y <= mag_y;
            last_z <= mag_z;
        end
    end

    // =========================================================================
    // any_fault: OR of all three
    // =========================================================================
    assign any_fault = fault_sat | fault_stuck | fault_noise;

endmodule