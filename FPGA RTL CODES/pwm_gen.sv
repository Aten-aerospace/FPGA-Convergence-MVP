`timescale 1ns/1ps

module pwm_gen #(
    parameter integer CHANNELS = 8,
    parameter integer CLK_HZ   = 50_000_000,
    parameter integer PWM_HZ   = 400   // 50-400 Hz typical
)(
    input  logic clk,
    input  logic rst_n,

    // Duty input: 0-10000 = 0.00%-100.00%
    input  logic [15:0] duty [CHANNELS-1:0],

    output logic [CHANNELS-1:0] pwm_out
);

    // ------------------------------------------------------------------------
    // Derived parameters
    // ------------------------------------------------------------------------
    localparam integer PERIOD_CNT = CLK_HZ / PWM_HZ;
    localparam integer CNT_W = $clog2(PERIOD_CNT);

    // ------------------------------------------------------------------------
    // Registers
    // ------------------------------------------------------------------------
    logic [CNT_W-1:0] counter;

    // Latched duty (glitch-free update)
    logic [15:0] duty_latched [CHANNELS-1:0];

    // Compare thresholds
    logic [CNT_W-1:0] threshold [CHANNELS-1:0];

    // ------------------------------------------------------------------------
    // Period Counter
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            counter <= '0;
        else if (counter == PERIOD_CNT - 1)
            counter <= '0;
        else
            counter <= counter + 1;
    end

    // ------------------------------------------------------------------------
    // Latch duty at period boundary (prevents mid-cycle glitch)
    // ------------------------------------------------------------------------
    integer i;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < CHANNELS; i++)
                duty_latched[i] <= 16'd0;
        end
        else if (counter == PERIOD_CNT - 1) begin
            for (i = 0; i < CHANNELS; i++)
                duty_latched[i] <= duty[i];
        end
    end

    // ------------------------------------------------------------------------
    // Compute thresholds
    // threshold = duty% * PERIOD
    // duty range: 0-10000 → scaled division by 10000
    // ------------------------------------------------------------------------
    always_comb begin
        for (int j = 0; j < CHANNELS; j++) begin
            threshold[j] = (duty_latched[j] * PERIOD_CNT) / 10000;
        end
    end

    // ------------------------------------------------------------------------
    // PWM generation
    // ------------------------------------------------------------------------
    always_comb begin
        for (int k = 0; k < CHANNELS; k++) begin
            pwm_out[k] = (counter < threshold[k]);
        end
    end

endmodule
