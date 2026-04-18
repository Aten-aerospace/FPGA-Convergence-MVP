// =============================================================================
// Module: sun_vector_compute
// Subsystem: CS3 - Sun Sensor ADC Interface
// Description: Computes a 2D sun-direction vector from four photodiode readings.
//              alpha = (ch0 - ch1) / (ch0 + ch1)   [azimuth proxy]
//              beta  = (ch2 - ch3) / (ch2 + ch3)   [elevation proxy]
//              Division is implemented as a 15-step shift-subtract (restoring
//              binary division) to produce a Q15 fixed-point result.
//              sun_present asserts when any channel exceeds THRESHOLD.
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module sun_vector_compute #(
    parameter int THRESHOLD = 12'd409   // 10 % FS: 0.10 × 4095 ≈ 409 counts
)(
    input  logic        clk,
    input  logic        rst_n,

    // 4-channel 12-bit photodiode readings
    input  logic [11:0] ch [0:3],
    input  logic        data_valid,

    // Q15 sun-vector components (signed)
    output logic signed [15:0] sun_alpha,  // (ch0-ch1)/(ch0+ch1)
    output logic signed [15:0] sun_beta,   // (ch2-ch3)/(ch2+ch3)

    // Status
    output logic        sun_present,  // at least one channel above threshold
    output logic        cal_valid     // one-cycle strobe: alpha & beta updated
);

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [3:0] {
        S_IDLE,
        S_SUMS,
        S_DIV_INIT_A,
        S_DIV_RUN_A,
        S_DIV_APPLY_A,
        S_DIV_INIT_B,
        S_DIV_RUN_B,
        S_DIV_APPLY_B,
        S_OUTPUT
    } state_t;

    state_t state;

    // =========================================================================
    // Sum / difference intermediate values
    // ch values are 12-bit unsigned; sum is 13-bit, diff is 13-bit signed
    // =========================================================================
    logic [12:0] sum01, sum23;
    logic signed [12:0] diff01, diff23;

    // =========================================================================
    // Divider state
    // Computes Q15 = (|diff| << 15) / sum  via 15-step restoring division.
    //   div_numer_b = {|diff|[11:0], 3'b000}   (15-bit shift register)
    //   div_rem     = partial remainder (16-bit, always < div_den)
    //   div_quot    = accumulating 15-bit unsigned quotient
    //   div_sign    = sign of diff → used to negate quotient afterwards
    // =========================================================================
    logic [11:0]  div_abs_num;    // |diff|, 12-bit
    logic [14:0]  div_numer_b;   // {|diff|[11:0], 3'b0}
    logic [12:0]  div_den;       // sum (13-bit denominator)
    logic [15:0]  div_rem;       // partial remainder
    logic [14:0]  div_quot;      // unsigned quotient (15-bit)
    logic [3:0]   div_step;      // step counter 0..14
    logic         div_sign;      // sign of numerator

    // Combinational trial subtraction for restoring division
    // trial = {div_rem[13:0], next_bit} - {0, div_den}
    logic         div_new_bit;
    logic [15:0]  div_trial;

    always_comb begin
        div_new_bit = (div_step <= 4'd14) ? div_numer_b[4'd14 - div_step] : 1'b0;
        div_trial   = {div_rem[13:0], div_new_bit} - {3'b000, div_den};
    end

    // =========================================================================
    // Latched Q15 intermediate results
    // =========================================================================
    logic signed [15:0] alpha_raw, beta_raw;

    // =========================================================================
    // FSM + datapath
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            sun_alpha   <= '0;
            sun_beta    <= '0;
            sun_present <= 1'b0;
            cal_valid   <= 1'b0;
            sum01       <= '0;  sum23  <= '0;
            diff01      <= '0;  diff23 <= '0;
            div_abs_num <= '0;  div_numer_b <= '0;
            div_den     <= '0;  div_rem <= '0;
            div_quot    <= '0;  div_step <= '0;
            div_sign    <= 1'b0;
            alpha_raw   <= '0;  beta_raw <= '0;
        end else begin
            cal_valid <= 1'b0; // default de-assert

            case (state)

                S_IDLE: begin
                    if (data_valid) begin
                        sun_present <= (ch[0] > THRESHOLD[11:0]) ||
                                       (ch[1] > THRESHOLD[11:0]) ||
                                       (ch[2] > THRESHOLD[11:0]) ||
                                       (ch[3] > THRESHOLD[11:0]);
                        state <= S_SUMS;
                    end
                end

                S_SUMS: begin
                    // 13-bit sums (no overflow: 12-bit + 12-bit)
                    sum01  <= {1'b0, ch[0]} + {1'b0, ch[1]};
                    sum23  <= {1'b0, ch[2]} + {1'b0, ch[3]};
                    // 13-bit signed differences
                    diff01 <= $signed({1'b0, ch[0]}) - $signed({1'b0, ch[1]});
                    diff23 <= $signed({1'b0, ch[2]}) - $signed({1'b0, ch[3]});
                    state  <= S_DIV_INIT_A;
                end

                // ---- Alpha division  ----------------------------------------
                S_DIV_INIT_A: begin
                    if (sum01 == '0) begin
                        alpha_raw <= '0;
                        state     <= S_DIV_INIT_B;
                    end else begin
                        div_sign    <= diff01[12];  // MSB = sign
                        div_abs_num <= diff01[12] ?
                                       (~diff01[11:0] + 12'd1) : diff01[11:0];
                        div_den     <= sum01;
                        div_rem     <= '0;
                        div_quot    <= '0;
                        div_step    <= '0;
                        state       <= S_DIV_RUN_A;
                    end
                end

                S_DIV_RUN_A: begin
                    // Update numer_b on first step
                    if (div_step == '0)
                        div_numer_b <= {div_abs_num, 3'b000};

                    if (!div_trial[15]) begin
                        // subtraction succeeded: update remainder, set quot bit
                        div_rem  <= {2'b00, div_trial[13:0]};
                        div_quot <= {div_quot[13:0], 1'b1};
                    end else begin
                        // restore: keep shifted remainder with new bit
                        div_rem  <= {2'b00, div_rem[13:0], div_new_bit};
                        div_quot <= {div_quot[13:0], 1'b0};
                    end

                    if (div_step == 4'd14)
                        state <= S_DIV_APPLY_A;
                    else
                        div_step <= div_step + 1'b1;
                end

                S_DIV_APPLY_A: begin
                    // Apply sign; clamp to Q15 max (2^15-1 = 32767)
                    if (div_quot >= 15'd32767)
                        alpha_raw <= div_sign ? -16'sd32767 : 16'sd32767;
                    else
                        alpha_raw <= div_sign ?
                                     -$signed({1'b0, div_quot[14:0]}) :
                                      $signed({1'b0, div_quot[14:0]});
                    state <= S_DIV_INIT_B;
                end

                // ---- Beta division  -----------------------------------------
                S_DIV_INIT_B: begin
                    if (sum23 == '0) begin
                        beta_raw <= '0;
                        state    <= S_OUTPUT;
                    end else begin
                        div_sign    <= diff23[12];
                        div_abs_num <= diff23[12] ?
                                       (~diff23[11:0] + 12'd1) : diff23[11:0];
                        div_den     <= sum23;
                        div_rem     <= '0;
                        div_quot    <= '0;
                        div_step    <= '0;
                        state       <= S_DIV_RUN_B;
                    end
                end

                S_DIV_RUN_B: begin
                    if (div_step == '0)
                        div_numer_b <= {div_abs_num, 3'b000};

                    if (!div_trial[15]) begin
                        div_rem  <= {2'b00, div_trial[13:0]};
                        div_quot <= {div_quot[13:0], 1'b1};
                    end else begin
                        div_rem  <= {2'b00, div_rem[13:0], div_new_bit};
                        div_quot <= {div_quot[13:0], 1'b0};
                    end

                    if (div_step == 4'd14)
                        state <= S_DIV_APPLY_B;
                    else
                        div_step <= div_step + 1'b1;
                end

                S_DIV_APPLY_B: begin
                    if (div_quot >= 15'd32767)
                        beta_raw <= div_sign ? -16'sd32767 : 16'sd32767;
                    else
                        beta_raw <= div_sign ?
                                    -$signed({1'b0, div_quot[14:0]}) :
                                     $signed({1'b0, div_quot[14:0]});
                    state <= S_OUTPUT;
                end

                S_OUTPUT: begin
                    sun_alpha <= alpha_raw;
                    sun_beta  <= beta_raw;
                    cal_valid <= 1'b1;
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule