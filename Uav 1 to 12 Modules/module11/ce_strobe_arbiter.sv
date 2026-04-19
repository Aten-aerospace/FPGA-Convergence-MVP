// =============================================================================
// File        : ce_strobe_arbiter.sv
// Module      : ce_strobe_arbiter
// Description : Arbitrates and distributes CE strobes from MOD_1 to all modules.
//               Ensures strobes are properly sequenced to avoid race conditions.
//               Priority: EKF predict > GPS update > baro update > nav/PID.
// =============================================================================

`timescale 1ns/1ps

module ce_strobe_arbiter (
    input  logic clk,
    input  logic rst_n,

    // Raw CE inputs from MOD_1
    input  logic ce_1khz_raw,
    input  logic ce_100hz_raw,
    input  logic ce_50hz_raw,
    input  logic ce_10hz_raw,

    // Module-specific CE outputs with priority gating
    // MOD_2 PID inner loop (1 kHz)
    output logic ce_pid_inner,

    // MOD_4 EKF predict (100 Hz) - highest priority at 100Hz
    output logic ce_ekf_predict,

    // MOD_2 PID outer loop (100 Hz) - after EKF predict
    output logic ce_pid_outer,

    // MOD_5 Baro update (50 Hz)
    output logic ce_baro_upd,

    // MOD_5 GPS update (10 Hz)
    output logic ce_gps_upd,

    // MOD_5 Mag update (10 Hz)
    output logic ce_mag_upd,

    // MOD_7 MAVLink 1Hz strobe (derived from ce_10hz with /10 divider)
    output logic ce_1hz,
    output logic ce_5hz,

    // MOD_8 Navigation (100 Hz)
    output logic ce_nav,

    // Pass-through for sub-modules
    output logic ce_50hz,
    output logic ce_10hz
);

    // -------------------------------------------------------------------------
    // 1 Hz from 10 Hz (÷10)
    // -------------------------------------------------------------------------
    logic [3:0] cnt_1hz;
    logic [2:0] cnt_5hz;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_1hz <= '0; cnt_5hz <= '0;
            ce_1hz  <= 1'b0; ce_5hz <= 1'b0;
        end else begin
            ce_1hz <= 1'b0;
            ce_5hz <= 1'b0;
            if (ce_10hz_raw) begin
                if (cnt_1hz == 4'd9) begin
                    cnt_1hz <= '0;
                    ce_1hz  <= 1'b1;
                end else begin
                    cnt_1hz <= cnt_1hz + 1'b1;
                end
                if (cnt_5hz == 3'd1) begin
                    cnt_5hz <= '0;
                    ce_5hz  <= 1'b1;
                end else begin
                    cnt_5hz <= cnt_5hz + 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Priority gating at 100 Hz: EKF predict fires first, then PID outer,
    // then navigation. Use delayed versions to create ordered sub-slots.
    // -------------------------------------------------------------------------
    logic ce_100hz_d1, ce_100hz_d2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ce_100hz_d1 <= 1'b0;
            ce_100hz_d2 <= 1'b0;
        end else begin
            ce_100hz_d1 <= ce_100hz_raw;
            ce_100hz_d2 <= ce_100hz_d1;
        end
    end

    // EKF predict: fires on ce_100hz_raw
    // PID outer: fires 1 cycle after EKF predict
    // Navigation: fires 2 cycles after EKF predict
    assign ce_ekf_predict = ce_100hz_raw;
    assign ce_pid_outer   = ce_100hz_d1;
    assign ce_nav         = ce_100hz_d2;

    // PID inner: 1 kHz direct
    assign ce_pid_inner   = ce_1khz_raw;

    // Baro update: 50 Hz direct
    assign ce_baro_upd    = ce_50hz_raw;
    assign ce_50hz        = ce_50hz_raw;

    // GPS/Mag updates: 10 Hz direct
    assign ce_gps_upd     = ce_10hz_raw;
    assign ce_mag_upd     = ce_10hz_raw;
    assign ce_10hz        = ce_10hz_raw;

endmodule
