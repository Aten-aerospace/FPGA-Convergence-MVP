// =============================================================================
// Module: rw_driver (CS7 reaction-wheel PWM driver)
// Subsystem: CS7 - Actuator Drivers
// Description: Converts signed 16-bit torque commands (Q15 format) into PWM
//              duty cycles for three reaction wheels.  A dead-band of ±64 LSB
//              suppresses noise-driven jitter at zero crossing.
//
//   Duty mapping:
//     torque_cmd = 0x7FFF (+1.0 Q15)  → duty 10000 (100 %)
//     torque_cmd = 0x0000              → duty 5000  ( 50 % - bi-directional centre)
//     torque_cmd = 0x8000 (-1.0 Q15)  → duty 0     (  0 %)
//
//   PWM frequency = CLK_HZ / PWM_PERIOD_CNT  (default ≈ 20 kHz at 100 MHz).
//   rw_fault asserts if cmd_valid is not seen for FAULT_TIMEOUT ce_1khz ticks.
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module rw_driver #(
    parameter int CLK_HZ         = 100_000_000,
    parameter int PWM_PERIOD_CNT = 5000,           // 100 MHz / 5000 = 20 kHz
    parameter int FAULT_TIMEOUT  = 200,            // ce_1khz ticks → 200 ms
    parameter int DEADBAND       = 64              // Q15 LSBs
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_1khz,

    // Signed Q15 torque commands, one per wheel
    input  logic signed [15:0] torque_cmd [0:2],
    input  logic               cmd_valid,

    output logic [2:0]  pwm_out,
    output logic [2:0]  rw_enable,
    output logic        rw_fault
);

    // -------------------------------------------------------------------------
    // Derived parameters
    // -------------------------------------------------------------------------
    localparam int CNT_W = $clog2(PWM_PERIOD_CNT);

    // -------------------------------------------------------------------------
    // PWM counters
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
    // Duty-cycle latches (update at period boundary for glitch-free PWM)
    // duty range 0-10000 = 0%-100%
    // -------------------------------------------------------------------------
    logic [13:0] duty [0:2];          // 0..10000

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) duty[i] <= 14'd5000;
        end else if (pwm_cnt == PWM_PERIOD_CNT - 1) begin
            for (int i = 0; i < 3; i++) begin
                // Map Q15 signed → duty 0-10000
                // formula: duty = (torque_cmd + 32768) * 10000 / 65536
                //        = (torque_cmd + 32768) * 5 / 32 (fixed-point approx)
                // Apply dead-band at zero crossing
                logic signed [15:0] cmd_clamped;
                logic signed [16:0] shifted;
                logic [16:0]        mag;
                if ($signed(torque_cmd[i]) > -DEADBAND &&
                    $signed(torque_cmd[i]) <  DEADBAND) begin
                    cmd_clamped = 16'sd0;
                end else begin
                    cmd_clamped = torque_cmd[i];
                end
                // Offset-binary: shift from signed to unsigned
                shifted = {cmd_clamped[15], cmd_clamped} + 17'sd32768;
                // Scale: *10000 / 65536  ≈  *5 / 32
                duty[i] <= 14'((shifted * 17'd10000) >> 16);
            end
        end
    end

    // -------------------------------------------------------------------------
    // PWM outputs
    // -------------------------------------------------------------------------
    logic [13:0] threshold [0:2];

    always_comb begin
        for (int k = 0; k < 3; k++) begin
            // threshold in CNT_W units: duty * PWM_PERIOD_CNT / 10000
            threshold[k] = (duty[k] * PWM_PERIOD_CNT) / 10000;
            pwm_out[k]   = (pwm_cnt < threshold[k]);
        end
    end

    // -------------------------------------------------------------------------
    // Enable: wheels are enabled when not in fault
    // -------------------------------------------------------------------------
    always_comb begin
        rw_enable = rw_fault ? 3'b000 : 3'b111;
    end

    // -------------------------------------------------------------------------
    // Fault watchdog: fires if cmd_valid absent for FAULT_TIMEOUT ce_1khz ticks
    // -------------------------------------------------------------------------
    localparam int WDOG_W = $clog2(FAULT_TIMEOUT + 1);

    logic [WDOG_W-1:0] wdog_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdog_cnt <= '0;
            rw_fault <= 1'b0;
        end else begin
            if (cmd_valid) begin
                wdog_cnt <= '0;
                rw_fault <= 1'b0;
            end else if (ce_1khz) begin
                if (wdog_cnt == FAULT_TIMEOUT[WDOG_W-1:0]) begin
                    rw_fault <= 1'b1;
                end else begin
                    wdog_cnt <= wdog_cnt + 1;
                end
            end
        end
    end

endmodule