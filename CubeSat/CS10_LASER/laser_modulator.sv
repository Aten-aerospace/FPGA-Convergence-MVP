// =============================================================================
// Module: laser_modulator (CS10 ISL laser modulator)
// Subsystem: CS10 - Laser Pointing FSM
// Description: Drives laser modulator with high-frequency PWM and optional
//              ISL 8-bit data encoding using OOK (on-off keying).
//
//   PWM carrier at CLK_HZ / PWM_PERIOD (configurable).
//   When mod_en=1: output laser_pwm at configured duty cycle.
//   When isl_data_valid=1: encode 8-bit word via OOK on 10 MHz subcarrier
//     bit=1 → 90% duty cycle, bit=0 → 10% duty cycle.
//   Each bit transmitted over SUBCARRIER_DIV cycles of the PWM period.
//
// Requirement: CS-LSR-006 (COMM State Behavior - modulator + ISL data)
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module laser_modulator #(
    parameter int CLK_HZ        = 100_000_000,  // system clock (100 MHz)
    parameter int PWM_PERIOD    = 10,            // PWM period in clk cycles (10 MHz carrier)
    parameter int BITS_PER_WORD = 8              // ISL word width
)(
    input  logic        clk,
    input  logic        rst_n,

    input  logic        mod_en,              // enable modulator (COMM state)
    input  logic [11:0] mod_duty_cycle,      // nominal duty cycle (0-4095, default 2048)
    input  logic [7:0]  isl_data_in,         // 8-bit ISL data to transmit
    input  logic        isl_data_valid,      // strobe to load new word

    output logic        laser_pwm,           // PWM output to laser driver
    output logic [7:0]  isl_data_out,        // loopback for monitoring
    output logic        mod_active           // high when modulating
);

    // =========================================================================
    // PWM counter (0 .. PWM_PERIOD-1)
    // =========================================================================
    localparam int CNT_W = $clog2(PWM_PERIOD + 1);

    logic [CNT_W-1:0] pwm_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            pwm_cnt <= '0;
        else if (pwm_cnt == CNT_W'(PWM_PERIOD - 1))
            pwm_cnt <= '0;
        else
            pwm_cnt <= pwm_cnt + 1;
    end

    // =========================================================================
    // ISL shift register + bit counter
    // =========================================================================
    logic [7:0]  shift_reg;
    logic [2:0]  bit_idx;    // 0..7 (3 bits for 8 bits)
    logic        isl_active; // currently sending an ISL word

    // Bits-per-cycle counter: each bit occupies PWM_PERIOD clk cycles
    // Bit transition at the end of each PWM period
    logic bit_advance;
    assign bit_advance = (pwm_cnt == CNT_W'(PWM_PERIOD - 1));

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_reg   <= 8'd0;
            bit_idx     <= 3'd0;
            isl_active  <= 1'b0;
            isl_data_out <= 8'd0;
        end else begin
            if (isl_data_valid && !isl_active) begin
                // Load new word to transmit (MSB first)
                shift_reg  <= isl_data_in;
                isl_data_out <= isl_data_in;
                isl_active <= 1'b1;
                bit_idx    <= 3'd0;
            end else if (isl_active && bit_advance) begin
                if (bit_idx == 3'd7) begin
                    isl_active <= 1'b0;
                    bit_idx    <= 3'd0;
                end else begin
                    shift_reg <= {shift_reg[6:0], 1'b0}; // shift MSB out
                    bit_idx   <= bit_idx + 1;
                end
            end
        end
    end

    // =========================================================================
    // Duty cycle selection
    //   OOK: bit=1 → 90% duty (9 of 10 cycles), bit=0 → 10% duty (1 of 10)
    //   No ISL: use mod_duty_cycle scaled to PWM_PERIOD
    // =========================================================================
    localparam int DUTY_HIGH = (PWM_PERIOD * 9) / 10;  // 90%
    localparam int DUTY_LOW  = (PWM_PERIOD * 1) / 10;  // 10%

    logic [CNT_W-1:0] threshold;

    always_comb begin
        if (isl_active) begin
            // OOK: current bit is MSB of shift_reg
            threshold = shift_reg[7] ? CNT_W'(DUTY_HIGH) : CNT_W'(DUTY_LOW);
        end else begin
            // Nominal duty: scale mod_duty_cycle (12-bit, 0-4095) to PWM_PERIOD
            // threshold = (mod_duty_cycle * PWM_PERIOD) / 4096
            threshold = CNT_W'((mod_duty_cycle * PWM_PERIOD) >> 12);
        end
    end

    // =========================================================================
    // PWM output
    // =========================================================================
    always_comb begin
        laser_pwm  = mod_en && (pwm_cnt < threshold);
        mod_active = mod_en;
    end

endmodule