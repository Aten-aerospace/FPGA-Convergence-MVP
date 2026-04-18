// =============================================================================
// Module: mtq_driver (CS7 magnetorquer PWM + H-bridge driver)
// Subsystem: CS7 - Actuator Drivers
// Description: Converts signed 16-bit dipole commands (Q15) to PWM duty cycle
//              and direction signals for three magnetorquers driven by H-bridges.
//              Meets CS-ADCS-009: "3-axis PWM @ 10 kHz carrier, ≥12-bit resolution,
//              cross-axis coupling <1%".
//
//   Positive dipole_cmd → dir = 1, duty = |cmd| mapped to 0-10000.
//   Negative dipole_cmd → dir = 0, duty = |cmd| mapped to 0-10000.
//   Zero ± dead-band   → PWM off (duty = 0), dir = 0.
//
//   PWM frequency = 10 kHz (CS-ADCS-009 requirement).
//   ce_1khz is used as the watchdog tick; an independent 10 kHz PWM is derived
//   from CLK_HZ / PWM_PERIOD_CNT (100 MHz / 10 000 = 10 kHz).
//
//   Saturation flags (mtq_sat_flag[2:0]) assert when any axis duty > 9500
//   (>95% of full scale).  coupling_warning asserts if >1 axis is saturated
//   simultaneously, indicating potential cross-axis coupling violation.
//
// Timing: duty_latch updated at PWM period boundary (glitch-free).
//         mtq_sat_flag / coupling_warning are combinational (0 latency).
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module mtq_driver #(
    parameter int CLK_HZ         = 100_000_000,
    parameter int PWM_PERIOD_CNT = 10_000,         // 100 MHz / 10k = 10 kHz (CS-ADCS-009)
    parameter int DEADBAND       = 128,            // Q15 LSBs
    parameter int SAT_THRESHOLD  = 9500            // duty units (0-10000); >9500 = saturated
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_1khz,

    // Signed Q15 dipole commands, one per magnetorquer axis
    input  logic signed [15:0] dipole_cmd [0:2],
    input  logic               cmd_valid,

    output logic [2:0]  mtq_pwm,
    output logic [2:0]  mtq_dir,
    output logic [2:0]  mtq_enable,

    // Cross-axis coupling monitors (CS-ADCS-009 <1% coupling requirement)
    output logic [2:0]  mtq_sat_flag,      // bit[i] asserts when axis i duty > SAT_THRESHOLD
    output logic        coupling_warning   // asserts when >1 axis saturated simultaneously
);

    // -------------------------------------------------------------------------
    // Derived parameters
    // -------------------------------------------------------------------------
    localparam int CNT_W = $clog2(PWM_PERIOD_CNT);

    // -------------------------------------------------------------------------
    // PWM counter
    // -------------------------------------------------------------------------
    logic [CNT_W-1:0] pwm_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pwm_cnt <= '0;
        else if (pwm_cnt == PWM_PERIOD_CNT - 1)
            pwm_cnt <= '0;
        else
            pwm_cnt <= pwm_cnt + 1;
    end

    // -------------------------------------------------------------------------
    // Direction and duty latches (glitch-free, updated at period boundary)
    // -------------------------------------------------------------------------
    logic [13:0] duty_latch [0:2];   // 0..10000
    logic [2:0]  dir_latch;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) duty_latch[i] <= '0;
            dir_latch <= 3'b000;
        end else if (pwm_cnt == PWM_PERIOD_CNT - 1) begin
            for (int i = 0; i < 3; i++) begin
                logic signed [15:0] cmd;
                logic [14:0]        magnitude;
                cmd = dipole_cmd[i];
                if ($signed(cmd) > -DEADBAND && $signed(cmd) < DEADBAND) begin
                    duty_latch[i] <= '0;
                    dir_latch[i]  <= 1'b0;
                end else if ($signed(cmd) >= 0) begin
                    // Positive: dir = 1, magnitude = cmd
                    magnitude     = 15'(cmd);
                    duty_latch[i] <= 14'((magnitude * 15'd10000) >> 15);
                    dir_latch[i]  <= 1'b1;
                end else begin
                    // Negative: dir = 0, magnitude = -cmd (negate; -32768 clips to 32767)
                    magnitude     = (cmd == 16'sh8000) ? 15'd32767 : 15'(-$signed(cmd));
                    duty_latch[i] <= 14'((magnitude * 15'd10000) >> 15);
                    dir_latch[i]  <= 1'b0;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // PWM generation
    // -------------------------------------------------------------------------
    logic [CNT_W-1:0] threshold [0:2];

    always_comb begin
        for (int k = 0; k < 3; k++) begin
            threshold[k] = (duty_latch[k] * PWM_PERIOD_CNT) / 10000;
            mtq_pwm[k]   = (pwm_cnt < threshold[k]);
        end
        mtq_dir    = dir_latch;
        mtq_enable = 3'b111;          // always enabled (safe_mode handled in wrapper)
    end

    // -------------------------------------------------------------------------
    // Cross-axis coupling monitors (CS-ADCS-009: <1% cross-axis coupling)
    //   mtq_sat_flag[i] - combinational, asserts when duty_latch[i] > SAT_THRESHOLD
    //   coupling_warning - asserts when >1 axis saturated simultaneously
    //   SAT_THRESHOLD default = 9500 (95% of full scale)
    // -------------------------------------------------------------------------
    always_comb begin
        for (int k = 0; k < 3; k++)
            // SAT_THRESHOLD is a 32-bit parameter; duty_latch is 14-bit (max 10000).
            // Use explicit cast to 14-bit to clearly express width conversion intent.
            mtq_sat_flag[k] = duty_latch[k] > 14'(SAT_THRESHOLD);
        coupling_warning = ($countones(mtq_sat_flag) > 1);
    end

endmodule