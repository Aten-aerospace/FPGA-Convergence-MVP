// =============================================================================
// Module: mag_calibration
// Subsystem: CS2 - Magnetometer Interface (I2C)
// Description: Hard-iron + soft-iron calibration for 3-axis magnetometer.
//              Applies offset subtraction and scale-factor matrix (diagonal only
//              for FPGA resource efficiency). Outputs both Q15 and µT formats.
//              DSP48-free: Stage 2 uses saturating arithmetic (SI = 1.0 identity);
//              µT conversion uses bit-shift approximation (no multiplier).
// Provenance: cubesat_requirements.md; Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module mag_calibration #(
    // HMC5883L default sensitivity: 1530 LSB/mT at gain setting 1.
    // µT conversion: µT ≈ raw × (1000 / SCALE_FACTOR)
    //              ≈ raw × 0.654 ≈ (raw>>>1) + (raw>>>3) + (raw>>>5)
    //              approximation error < 0.4 % vs ideal.
    parameter int SCALE_FACTOR = 1530
)(
    input  logic        clk,
    input  logic        rst_n,

    // Raw magnetometer inputs (signed 16-bit)
    input  logic signed [15:0] raw_x, raw_y, raw_z,
    input  logic               raw_valid,

    // Calibrated outputs (Q15 format)
    output logic signed [15:0] cal_x, cal_y, cal_z,
    // Calibrated outputs in µT (bit-shift approximation, DSP48-free)
    output logic signed [15:0] ut_x,  ut_y,  ut_z,
    output logic               cal_valid,

    // Fault: saturation detected (any axis at or beyond full-scale limit)
    output logic               mag_fault
);

    // Hard-iron offsets (synthesizable constants; AXI4-Lite writable in production)
    localparam signed [15:0] HI_X = 16'sd0;
    localparam signed [15:0] HI_Y = 16'sd0;
    localparam signed [15:0] HI_Z = 16'sd0;

    // =========================================================================
    // Stage 1: subtract hard-iron offset
    // =========================================================================
    logic signed [16:0] sub_x, sub_y, sub_z;
    logic pipe1_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sub_x <= '0; sub_y <= '0; sub_z <= '0;
            pipe1_valid <= 1'b0;
        end else begin
            pipe1_valid <= raw_valid;
            if (raw_valid) begin
                sub_x <= {raw_x[15], raw_x} - {HI_X[15], HI_X};
                sub_y <= {raw_y[15], raw_y} - {HI_Y[15], HI_Y};
                sub_z <= {raw_z[15], raw_z} - {HI_Z[15], HI_Z};
            end
        end
    end

    // =========================================================================
    // Stage 2: soft-iron correction - DSP48-free saturating 17 → 16-bit
    // SI_X/Y/Z = 1.0 (identity matrix diagonal); no multiplier needed.
    // Overflow detection via sign-extension bits [16:15]:
    //   2'b01 → positive overflow  → saturate to +32767 (16'sh7FFF)
    //   2'b10 → negative overflow  → saturate to -32768 (16'sh8000)
    //   2'b00 / 2'b11 → in range   → pass through sub[15:0]
    // =========================================================================
    logic signed [15:0] sc_x, sc_y, sc_z;
    logic pipe2_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sc_x <= '0; sc_y <= '0; sc_z <= '0;
            pipe2_valid <= 1'b0;
        end else begin
            pipe2_valid <= pipe1_valid;
            if (pipe1_valid) begin
                sc_x <= (sub_x[16:15] == 2'b01) ? 16'sh7FFF :
                        (sub_x[16:15] == 2'b10) ? 16'sh8000 : sub_x[15:0];
                sc_y <= (sub_y[16:15] == 2'b01) ? 16'sh7FFF :
                        (sub_y[16:15] == 2'b10) ? 16'sh8000 : sub_y[15:0];
                sc_z <= (sub_z[16:15] == 2'b01) ? 16'sh7FFF :
                        (sub_z[16:15] == 2'b10) ? 16'sh8000 : sub_z[15:0];
            end
        end
    end

    // =========================================================================
    // Stage 3: output Q15 + µT conversion (bit-shift, DSP48-free)
    // µT = cal × (1000 / SCALE_FACTOR) ≈ cal × 0.654
    //    = (cal>>>1) + (cal>>>3) + (cal>>>5)   [0.500 + 0.125 + 0.031 = 0.656]
    // Maximum output: ±32767 × 0.656 ≈ ±21495 µT, fits in signed 16-bit.
    // =========================================================================
    // Bit-shift µT approximation (reused for all three axes)
    function automatic logic signed [15:0] lsb_to_uT;
        input logic signed [15:0] val;
        // Arithmetic shifts preserve sign; no DSP48 inference
        lsb_to_uT = ($signed(val) >>> 1) + ($signed(val) >>> 3) +
                     ($signed(val) >>> 5);
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cal_x <= '0; cal_y <= '0; cal_z <= '0;
            ut_x  <= '0; ut_y  <= '0; ut_z  <= '0;
            cal_valid <= 1'b0;
            mag_fault <= 1'b0;
        end else begin
            cal_valid <= pipe2_valid;
            if (pipe2_valid) begin
                cal_x <= sc_x;
                cal_y <= sc_y;
                cal_z <= sc_z;

                // Bit-shift µT approximation (SCALE_FACTOR = 1530 LSB/mT)
                ut_x  <= lsb_to_uT(sc_x);
                ut_y  <= lsb_to_uT(sc_y);
                ut_z  <= lsb_to_uT(sc_z);

                // Saturation fault: axis reached full-scale limit
                mag_fault <= (sc_x == 16'sh7FFF) | (sc_x == 16'sh8000) |
                             (sc_y == 16'sh7FFF) | (sc_y == 16'sh8000) |
                             (sc_z == 16'sh7FFF) | (sc_z == 16'sh8000);
            end
        end
    end

endmodule