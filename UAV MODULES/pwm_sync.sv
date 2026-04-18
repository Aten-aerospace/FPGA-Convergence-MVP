// =============================================================================
// File        : pwm_sync.sv
// Module      : pwm_sync
// Description : Synchronised 4-channel ESC PWM generator.
//               Wraps pwm_gen and ensures all 4 channels share a common
//               period counter (inter-channel skew ≤ 1 µs).
//               Motor commands (16-bit, 0-32767) are converted to
//               ESC pulse widths: 1000-2000 µs (armed), 1000 µs (disarmed).
//               Configurable PWM frequency 50-400 Hz.
// =============================================================================

`timescale 1ns/1ps

module pwm_sync #(
    parameter int CLK_HZ  = 50_000_000,
    parameter int PWM_HZ  = 400,  // 50-400 Hz
    parameter int CHANNELS = 4
)(
    input  logic clk,
    input  logic rst_n,

    input  logic armed,   // When low → all channels output 1000 µs (PWM_MIN)

    // Motor commands: 0 = min thrust, 32767 = max thrust
    input  logic [15:0] motor_cmd [0:CHANNELS-1],

    // PWM outputs
    output logic [CHANNELS-1:0] pwm_out
);

    // -------------------------------------------------------------------------
    // Convert motor command to duty cycle (0-10000 = 0%-100%)
    // ESC: 1000-2000 µs out of 1/PWM_HZ period
    // At 400 Hz: period = 2500 µs
    //   1000 µs → 40.0%, 2000 µs → 80.0% → duty range 4000-8000 (×0.01%)
    // -------------------------------------------------------------------------
    localparam int PERIOD_US  = 1_000_000 / PWM_HZ;  // e.g. 2500 µs at 400 Hz
    localparam int DUTY_MIN   = (1000 * 10000) / PERIOD_US; // 4000
    localparam int DUTY_MAX   = (2000 * 10000) / PERIOD_US; // 8000
    localparam int DUTY_RANGE = DUTY_MAX - DUTY_MIN;        // 4000

    logic [15:0] duty_pct [0:CHANNELS-1]; // duty × 0.01%

    always_comb begin
        for (int c = 0; c < CHANNELS; c++) begin
            if (!armed) begin
                duty_pct[c] = DUTY_MIN[15:0]; // 1000 µs when disarmed
            end else begin
                // Scale: 0→DUTY_MIN, 32767→DUTY_MAX
                duty_pct[c] = DUTY_MIN[15:0] +
                              (({16'b0, motor_cmd[c]} * DUTY_RANGE) >> 15);
            end
        end
    end

    // -------------------------------------------------------------------------
    // Shared period counter (guarantees zero inter-channel skew)
    // -------------------------------------------------------------------------
    pwm_gen #(
        .CHANNELS (CHANNELS),
        .CLK_HZ   (CLK_HZ),
        .PWM_HZ   (PWM_HZ)
    ) u_pwm (
        .clk     (clk),
        .rst_n   (rst_n),
        .duty    (duty_pct),
        .pwm_out (pwm_out)
    );

endmodule
