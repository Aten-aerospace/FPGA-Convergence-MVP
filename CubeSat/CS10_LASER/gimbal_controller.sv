// =============================================================================
// Module: gimbal_controller (CS10 2-axis stepper gimbal controller)
// Subsystem: CS10 - Laser Pointing FSM
// Description: Drives two stepper motor axes (azimuth = axis 0, elevation = axis 1)
//              from signed arc-minute commands.
//
//   Command units: arc-minutes (Q15 signed, 1 LSB = 1/32768 arc-min).
//   Positive = CW / up, Negative = CCW / down.
//
//   Step rate: STEPS_PER_ARCMIN steps per arc-minute.  The step pulse
//   interval is derived from ce_100hz (10 ms per tick): at full rate
//   STEP_RATE_LIMIT steps per tick (default 20 → 200 kHz at 100 Hz tick).
//
//   Resolution: default 400 steps/arc-minute = 400 steps/degree which maps
//   to 400 steps/60 arc-min = 6.67 steps/arc-min... wait, 400 steps/degree =
//   400/60 steps/arc-min.  Parameter is steps per arc-minute; default 400/60
//   rounded to nearest integer.  Use STEPS_PER_ARCMIN=7 (≈400 steps/degree).
//
//   Trapezoidal acceleration: current_rate ramps from 1 to STEP_RATE_LIMIT
//   over ACCEL_STEPS ticks, coasts, then ramps down over ACCEL_STEPS ticks.
//
//   Homing: on rst_n de-asserted both axes drive negative until at_limit
//   is asserted, then zero the position counter.  at_limit input is active
//   high from limit switches.
//
//   Position feedback: gimbal_pos_az and gimbal_pos_el expose current step
//   count (signed 24-bit), useful for pointing error computation.
//
// Requirement: CS-LSR-008 (Gimbal Stepper Motor Control - 2-axis, 200kHz, 400 steps/°)
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module gimbal_controller #(
    parameter int STEPS_PER_ARCMIN = 7,      // ≈ 400 steps/degree (400/60 ≈ 7)
    parameter int MAX_POS_ARCMIN   = 1800,   // ±30° in arc-minutes
    parameter int STEP_RATE_LIMIT  = 20,     // max steps per ce_100hz tick (200 kHz)
    parameter int ACCEL_STEPS      = 4       // ramp length in ce_100hz ticks
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    // Signed Q15 arc-minute commands, one per axis [0]=az, [1]=el
    input  logic signed [15:0] gimbal_cmd [0:1],
    input  logic               cmd_valid,

    output logic [1:0]  step,           // step pulse (one cycle wide)
    output logic [1:0]  dir,            // direction (1 = positive)
    input  logic [1:0]  at_limit,       // limit-switch inputs (active-high)
    output logic        gimbal_busy,

    // Position feedback (current position in steps, signed 24-bit)
    output logic signed [23:0] gimbal_pos_az,
    output logic signed [23:0] gimbal_pos_el
);

    // =========================================================================
    // FSM: HOMING → IDLE → MOVE
    // =========================================================================
    typedef enum logic [1:0] {
        HOMING = 2'd0,
        IDLE   = 2'd1,
        MOVE   = 2'd2
    } gimbal_state_t;

    gimbal_state_t state;

    // =========================================================================
    // Position counters (arc-minutes × STEPS_PER_ARCMIN, 24-bit signed)
    // =========================================================================
    logic signed [23:0] pos_cnt    [0:1];  // current position in steps
    logic signed [23:0] target_cnt [0:1];  // target in steps

    // =========================================================================
    // Trapezoidal acceleration: current rate per axis
    // =========================================================================
    localparam int RATE_W = $clog2(STEP_RATE_LIMIT + 2);
    logic [RATE_W-1:0] current_rate [0:1];  // ramps 1..STEP_RATE_LIMIT per axis
    logic [RATE_W-1:0] move_tick_cnt [0:1]; // tick counter for ramp tracking

    // =========================================================================
    // Direction register and step output
    // =========================================================================
    logic [1:0] dir_reg;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= HOMING;
            for (int i = 0; i < 2; i++) begin
                pos_cnt[i]      <= '0;
                target_cnt[i]   <= '0;
                current_rate[i] <= RATE_W'(1);
                move_tick_cnt[i] <= '0;
            end
            step    <= 2'b00;
            dir_reg <= 2'b00;
        end else begin
            step <= 2'b00; // default: no step pulse

            case (state)
                // ----------------------------------------------------------
                HOMING: begin
                    dir_reg <= 2'b00; // negative direction
                    if (ce_100hz) begin
                        for (int i = 0; i < 2; i++) begin
                            if (!at_limit[i]) begin
                                step[i] <= 1'b1;  // drive toward limit
                            end else begin
                                pos_cnt[i]    <= '0;
                                target_cnt[i] <= '0;
                            end
                        end
                    end
                    if (at_limit == 2'b11)
                        state <= IDLE;
                end

                // ----------------------------------------------------------
                IDLE: begin
                    if (cmd_valid) begin
                        // Convert Q15 arc-minute command to step count
                        for (int i = 0; i < 2; i++) begin
                            logic signed [23:0] new_target;
                            new_target = pos_cnt[i] +
                                         24'($signed(gimbal_cmd[i]) *
                                             $signed(24'(STEPS_PER_ARCMIN)) >>> 15);
                            // Clamp to travel limits
                            if ($signed(new_target) > $signed(24'(MAX_POS_ARCMIN * STEPS_PER_ARCMIN)))
                                target_cnt[i] <= 24'(MAX_POS_ARCMIN * STEPS_PER_ARCMIN);
                            else if ($signed(new_target) < -$signed(24'(MAX_POS_ARCMIN * STEPS_PER_ARCMIN)))
                                target_cnt[i] <= -$signed(24'(MAX_POS_ARCMIN * STEPS_PER_ARCMIN));
                            else
                                target_cnt[i] <= new_target;
                            current_rate[i]  <= RATE_W'(1);
                            move_tick_cnt[i] <= '0;
                        end
                        state <= MOVE;
                    end
                end

                // ----------------------------------------------------------
                MOVE: begin
                    if (ce_100hz) begin
                        logic done0, done1;
                        done0 = 1'b1;
                        done1 = 1'b1;

                        for (int i = 0; i < 2; i++) begin
                            logic signed [23:0] error;
                            logic signed [23:0] abs_error;
                            logic [RATE_W-1:0]  rate_clamped;

                            error     = $signed(target_cnt[i]) - $signed(pos_cnt[i]);
                            abs_error = (error < 0) ? -error : error;

                            // Trapezoidal acceleration
                            // Ramp up: accel phase if tick_cnt < ACCEL_STEPS
                            // Ramp down: decel phase if abs_error <= ACCEL_STEPS
                            if ($signed(abs_error) <= $signed(24'(ACCEL_STEPS))) begin
                                // decel: rate decreases toward 1
                                rate_clamped = RATE_W'(1);
                            end else if (move_tick_cnt[i] < RATE_W'(ACCEL_STEPS)) begin
                                // accel ramp
                                rate_clamped = move_tick_cnt[i] + RATE_W'(1);
                                if (rate_clamped > RATE_W'(STEP_RATE_LIMIT))
                                    rate_clamped = RATE_W'(STEP_RATE_LIMIT);
                            end else begin
                                rate_clamped = RATE_W'(STEP_RATE_LIMIT);
                            end

                            current_rate[i] <= rate_clamped;
                            if (move_tick_cnt[i] < RATE_W'(ACCEL_STEPS))
                                move_tick_cnt[i] <= move_tick_cnt[i] + 1;

                            if ($signed(abs_error) > 0) begin
                                logic [RATE_W-1:0] steps_this_tick;
                                steps_this_tick = ($signed(abs_error) > $signed(24'(rate_clamped)))
                                                  ? rate_clamped : RATE_W'(abs_error);

                                if (error > 0) begin
                                    dir_reg[i] <= 1'b1;
                                    pos_cnt[i] <= pos_cnt[i] + 24'(steps_this_tick);
                                end else begin
                                    dir_reg[i] <= 1'b0;
                                    pos_cnt[i] <= pos_cnt[i] - 24'(steps_this_tick);
                                end
                                step[i] <= (steps_this_tick > 0);

                                if (i == 0) done0 = ($signed(abs_error) <= $signed(24'(rate_clamped)));
                                if (i == 1) done1 = ($signed(abs_error) <= $signed(24'(rate_clamped)));
                            end
                        end

                        if (done0 && done1) begin
                            for (int i = 0; i < 2; i++) begin
                                pos_cnt[i]       <= target_cnt[i];
                                current_rate[i]  <= RATE_W'(1);
                                move_tick_cnt[i] <= '0;
                            end
                            state <= IDLE;
                        end
                    end
                end

                default: state <= HOMING;
            endcase
        end
    end

    always_comb begin
        dir           = dir_reg;
        gimbal_busy   = (state == MOVE || state == HOMING);
        gimbal_pos_az = pos_cnt[0];
        gimbal_pos_el = pos_cnt[1];
    end

endmodule