// =============================================================================
// Module: laser_fsm_wrapper (CS10 laser pointing FSM + gimbal controller)
// Subsystem: CS10 - Laser Pointing FSM
// Description: 6-state FSM for laser inter-satellite link pointing.
//
//   States:
//     IDLE    : Laser off; wait for laser_enable from CS8.
//     SEARCH  : Boustrophedon raster scan ±10°az × ±5°el.
//     ACQUIRE : Spiral refinement at SEARCH peak location.
//     TRACK   : Closed-loop PD pointing control @ 100 Hz.
//     HOLD    : Signal briefly lost; maintain last position.
//     COMM    : Inter-satellite link active; modulate ISL data.
//     FAULT   : Signal lost > FAULT_TIMEOUT ticks; safe state.
//     SAFE    : laser_enable de-asserted; immediate shutdown.
//
//   Transitions:
//     IDLE    → SEARCH  : laser_enable & signal_valid
//     SEARCH  → ACQUIRE : peak_strength > SEARCH_THRESH
//     ACQUIRE → TRACK   : spiral_converged
//     TRACK   → COMM    : isl_data_valid (ISL data to transmit)
//     TRACK   → HOLD    : signal_strength < HOLD_THRESH
//     COMM    → HOLD    : signal_strength < HOLD_THRESH
//     HOLD    → TRACK   : signal_strength > HOLD_THRESH (no ISL pending)
//     HOLD    → COMM    : signal_strength > HOLD_THRESH & ISL active
//     HOLD    → FAULT   : hold_time > HOLD_TIMEOUT ticks
//     FAULT   → IDLE    : manual_clear & !gimbal_busy
//     any     → SAFE    : !laser_enable
//     SAFE    → IDLE    : laser_enable & !gimbal_busy
//
// Requirements: CS-LSR-001 through CS-LSR-010
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module laser_fsm_wrapper #(
    parameter int ACQ_THRESH    = 512,   // ADC counts (12-bit: 0-4095)
    parameter int HOLD_THRESH   = 256,   // ADC counts
    parameter int ACQ_CONFIRM   = 5,     // ce_100hz ticks (kept for TB compatibility)
    parameter int HOLD_TIMEOUT  = 20,    // ce_100hz ticks
    parameter int FAULT_TIMEOUT = 100,   // ce_100hz ticks
    parameter int SEARCH_THRESH = 300    // peak signal threshold to advance to ACQUIRE
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    // ADC / signal strength
    input  logic [11:0] signal_strength,  // raw ADC (12-bit, 0-4095)
    input  logic        signal_valid,

    // Enable from CS8 ADCS FSM
    input  logic        laser_enable,

    // Safe mode override from CS8
    input  logic        safe_mode,

    // External gimbal pointing commands (CS-LSR-009)
    input  logic signed [15:0] gimbal_cmd_abs [0:1], // absolute az/el command
    input  logic               gimbal_cmd_valid,      // strobe

    // ISL data interface (CS-LSR-006)
    input  logic [7:0]  isl_data_in,
    input  logic        isl_data_valid,

    // Fault management
    input  logic        manual_clear,       // uplink command to clear FAULT

    // Pointing error from external estimator (for TRACK PD)
    input  logic signed [15:0] pointing_error_az,
    input  logic signed [15:0] pointing_error_el,

    // Outputs
    output logic        laser_mod_en,
    output logic [1:0]  gimbal_step,
    output logic [1:0]  gimbal_dir,
    output logic [2:0]  laser_state,
    output logic        pointing_locked,
    output logic        laser_fault,
    output logic        laser_pwm,           // ISL PWM output (COMM state)
    output logic [7:0]  fault_code,          // detailed fault reason
    output logic signed [23:0] gimbal_pos_az,  // feedback position
    output logic signed [23:0] gimbal_pos_el,
    output logic [7:0]  convergence_time_100ms,  // spiral convergence time
    output logic [11:0] signal_strength_filtered // filtered signal output
);

    // =========================================================================
    // State encoding (8 states; laser_state output uses 3 bits)
    // =========================================================================
    typedef enum logic [2:0] {
        IDLE    = 3'd0,
        SEARCH  = 3'd1,
        ACQUIRE = 3'd2,
        TRACK   = 3'd3,
        HOLD    = 3'd4,
        COMM    = 3'd5,
        FAULT   = 3'd6,
        SAFE    = 3'd7
    } laser_state_t;

    laser_state_t state, next_state;

    // =========================================================================
    // Signal monitor (10 kHz ADC, 16-sample rolling average)
    // CS-LSR-010
    // =========================================================================
    logic [11:0] sig_filtered;
    logic        sig_filt_valid;

    signal_monitor #(
        .CLK_HZ    (100_000_000),
        .ADC_HZ    (10_000),
        .AVG_DEPTH (16)
    ) u_sig_mon (
        .clk                     (clk),
        .rst_n                   (rst_n),
        .adc_data                (signal_strength),
        .adc_valid               (1'b0),        // use internal divider
        .ce_100hz                (ce_100hz),
        .signal_strength_filtered(sig_filtered),
        .filter_valid            (sig_filt_valid)
    );

    assign signal_strength_filtered = sig_filtered;

    // =========================================================================
    // Peak hold detector
    // CS-LSR-004
    // =========================================================================
    // Convert gimbal step positions to arc-minutes for peak detector
    logic signed [15:0] pos_az_am, pos_el_am;  // arc-minutes (truncated from steps)

    // Divide steps by STEPS_PER_ARCMIN (7): approximate with >>3 (÷8) for synth
    assign pos_az_am = gimbal_pos_az[18:3];  // rough arc-min approximation
    assign pos_el_am = gimbal_pos_el[18:3];

    logic [11:0] peak_strength;
    logic [15:0] peak_az, peak_el;
    logic        peak_valid;
    logic        peak_detector_en;
    logic        peak_reset;

    assign peak_detector_en = (state == SEARCH || state == ACQUIRE);
    assign peak_reset       = (next_state == IDLE || next_state == SAFE);

    peak_hold_detector u_peak (
        .clk             (clk),
        .rst_n           (rst_n),
        .ce_100hz        (ce_100hz),
        .signal_strength (sig_filtered),
        .gimbal_az       (pos_az_am),
        .gimbal_el       (pos_el_am),
        .detector_en     (peak_detector_en),
        .reset           (peak_reset),
        .peak_strength   (peak_strength),
        .peak_az         (peak_az),
        .peak_el         (peak_el),
        .peak_valid      (peak_valid)
    );

    // =========================================================================
    // Raster scan engine - SEARCH state
    // CS-LSR-003
    // =========================================================================
    logic signed [15:0] raster_cmd_az, raster_cmd_el;
    logic               raster_cmd_valid;
    logic               scan_done;
    logic               scan_start_pulse;

    assign scan_start_pulse = (state == SEARCH);

    raster_scan_engine #(
        .AZ_LIMIT    (600),
        .EL_LIMIT    (300),
        .STEP_SIZE   (30)
    ) u_raster (
        .clk          (clk),
        .rst_n        (rst_n),
        .ce_100hz     (ce_100hz),
        .scan_start   (scan_start_pulse),
        .scan_reset   (state == IDLE || state == SAFE),
        .gimbal_cmd_az(raster_cmd_az),
        .gimbal_cmd_el(raster_cmd_el),
        .cmd_valid    (raster_cmd_valid),
        .scan_done    (scan_done)
    );

    // =========================================================================
    // Spiral refinement - ACQUIRE state
    // CS-LSR-004
    // =========================================================================
    logic signed [15:0] spiral_cmd_az, spiral_cmd_el;
    logic               spiral_cmd_valid;
    logic               spiral_converged;
    logic               spiral_start_pulse;

    assign spiral_start_pulse = (state == ACQUIRE);

    spiral_refinement #(
        .R_START_ARCMIN (120),
        .R_MIN_ARCMIN   (5),
        .TIMEOUT_TICKS  (500)
    ) u_spiral (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .ce_100hz              (ce_100hz),
        .spiral_start          (spiral_start_pulse),
        .spiral_center_az      ($signed(peak_az)),
        .spiral_center_el      ($signed(peak_el)),
        .peak_signal_strength  (peak_strength),
        .gimbal_cmd_az         (spiral_cmd_az),
        .gimbal_cmd_el         (spiral_cmd_el),
        .cmd_valid             (spiral_cmd_valid),
        .spiral_converged      (spiral_converged),
        .convergence_time_100ms(convergence_time_100ms)
    );

    // =========================================================================
    // PID controllers - TRACK/COMM state (2 instances: AZ & EL)
    // CS-LSR-005
    // =========================================================================
    localparam logic signed [15:0] KP_DEFAULT  = 16'sd1024; // Q4.12 gain
    localparam logic signed [15:0] KD_DEFAULT  = 16'sd256;
    localparam logic signed [15:0] OUT_LIMIT   = 16'sd4096;
    localparam logic signed [31:0] INTEG_LIMIT = 32'sd65536;

    logic signed [15:0] pid_out_az, pid_out_el;
    logic               pid_out_valid_az, pid_out_valid_el;
    logic               pid_en;

    assign pid_en = (state == TRACK || state == COMM);

    pid_controller #(
        .DATA_W  (16),
        .INTEG_W (32),
        .COEFF_W (16),
        .OUT_W   (16)
    ) u_pid_az (
        .clk               (clk),
        .rst_n             (rst_n),
        .enable            (pid_en && ce_100hz),
        .clear_integrator  (state == IDLE || state == SEARCH || state == ACQUIRE),
        .setpoint          (16'sd0),
        .measured          (pointing_error_az),
        .feedforward       (16'sd0),
        .kp                (KP_DEFAULT),
        .ki                (16'sd0),
        .kd                (KD_DEFAULT),
        .output_max        (OUT_LIMIT),
        .output_min        (-OUT_LIMIT),
        .integral_max      (INTEG_LIMIT),
        .control_output    (pid_out_az),
        .output_valid      (pid_out_valid_az),
        .error_out         (),
        .integral_out      (),
        .integral_saturated(),
        .output_saturated  ()
    );

    pid_controller #(
        .DATA_W  (16),
        .INTEG_W (32),
        .COEFF_W (16),
        .OUT_W   (16)
    ) u_pid_el (
        .clk               (clk),
        .rst_n             (rst_n),
        .enable            (pid_en && ce_100hz),
        .clear_integrator  (state == IDLE || state == SEARCH || state == ACQUIRE),
        .setpoint          (16'sd0),
        .measured          (pointing_error_el),
        .feedforward       (16'sd0),
        .kp                (KP_DEFAULT),
        .ki                (16'sd0),
        .kd                (KD_DEFAULT),
        .output_max        (OUT_LIMIT),
        .output_min        (-OUT_LIMIT),
        .integral_max      (INTEG_LIMIT),
        .control_output    (pid_out_el),
        .output_valid      (pid_out_valid_el),
        .error_out         (),
        .integral_out      (),
        .integral_saturated(),
        .output_saturated  ()
    );

    // =========================================================================
    // Laser modulator - COMM state
    // CS-LSR-006
    // =========================================================================
    logic mod_active;

    laser_modulator #(
        .CLK_HZ        (100_000_000),
        .PWM_PERIOD    (10),
        .BITS_PER_WORD (8)
    ) u_mod (
        .clk           (clk),
        .rst_n         (rst_n),
        .mod_en        (state == COMM),
        .mod_duty_cycle(12'd2048),
        .isl_data_in   (isl_data_in),
        .isl_data_valid(isl_data_valid),
        .laser_pwm     (laser_pwm),
        .isl_data_out  (),
        .mod_active    (mod_active)
    );

    // =========================================================================
    // Fault handler - FAULT state logging
    // CS-LSR-007
    // =========================================================================
    logic [15:0] fault_timestamp;
    logic [7:0]  fault_code_reg;
    logic        fault_trigger_internal;

    // Simple timestamp counter (ce_100hz ticks)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) fault_timestamp <= 16'd0;
        else if (ce_100hz && fault_timestamp != 16'hFFFF)
            fault_timestamp <= fault_timestamp + 1;
    end

    assign fault_trigger_internal = (next_state == FAULT && state != FAULT);

    always_comb begin
        if (state == HOLD && sig_filtered < HOLD_THRESH[11:0])
            fault_code_reg = 8'h00;  // signal loss
        else
            fault_code_reg = 8'h02;  // timeout
    end

    assign fault_code = fault_code_reg;

    laser_fault_handler u_fault_log (
        .clk            (clk),
        .rst_n          (rst_n),
        .ce_100hz       (ce_100hz),
        .fault_trigger  (fault_trigger_internal),
        .fault_code     (fault_code_reg),
        .timestamp      (fault_timestamp),
        .manual_clear   (manual_clear),
        .fault_fifo_data(),
        .fault_fifo_wr_en(),
        .fault_fifo_full(),
        .fault_logged   ()
    );

    // =========================================================================
    // Gimbal controller (enhanced - CS-LSR-008/009)
    // =========================================================================
    logic signed [15:0] gimbal_cmd_mux [0:1];
    logic               gimbal_cmd_valid_mux;
    logic [1:0]         at_limit;
    logic               gimbal_busy;
    logic [1:0]         step_raw;
    logic [1:0]         dir_raw;

    // Limit switches (tied low - no hardware limits in this design)
    assign at_limit = 2'b00;

    // Gimbal command multiplexer: select based on state
    always_comb begin
        gimbal_cmd_mux[0]   = 16'sd0;
        gimbal_cmd_mux[1]   = 16'sd0;
        gimbal_cmd_valid_mux = 1'b0;

        case (state)
            SEARCH: begin
                gimbal_cmd_mux[0]    = raster_cmd_az;
                gimbal_cmd_mux[1]    = raster_cmd_el;
                gimbal_cmd_valid_mux = raster_cmd_valid;
            end
            ACQUIRE: begin
                gimbal_cmd_mux[0]    = spiral_cmd_az;
                gimbal_cmd_mux[1]    = spiral_cmd_el;
                gimbal_cmd_valid_mux = spiral_cmd_valid;
            end
            TRACK, COMM: begin
                // PD control output
                gimbal_cmd_mux[0]    = pid_out_az;
                gimbal_cmd_mux[1]    = pid_out_el;
                gimbal_cmd_valid_mux = pid_out_valid_az;
            end
            default: begin
                // External command passthrough (absolute, for IDLE/HOLD/external)
                if (gimbal_cmd_valid) begin
                    gimbal_cmd_mux[0]    = gimbal_cmd_abs[0];
                    gimbal_cmd_mux[1]    = gimbal_cmd_abs[1];
                    gimbal_cmd_valid_mux = 1'b1;
                end
            end
        endcase
    end

    gimbal_controller #(
        .STEPS_PER_ARCMIN (7),
        .MAX_POS_ARCMIN   (1800),
        .STEP_RATE_LIMIT  (20),
        .ACCEL_STEPS      (4)
    ) u_gimbal (
        .clk          (clk),
        .rst_n        (rst_n),
        .ce_100hz     (ce_100hz),
        .gimbal_cmd   (gimbal_cmd_mux),
        .cmd_valid    (gimbal_cmd_valid_mux),
        .step         (step_raw),
        .dir          (dir_raw),
        .at_limit     (at_limit),
        .gimbal_busy  (gimbal_busy),
        .gimbal_pos_az(gimbal_pos_az),
        .gimbal_pos_el(gimbal_pos_el)
    );

    // =========================================================================
    // Counters for timeouts
    // =========================================================================
    localparam int CNT_W = $clog2(FAULT_TIMEOUT + 2);

    logic [CNT_W-1:0] hold_cnt;
    logic [CNT_W-1:0] fault_cnt;
    logic             isl_pending;  // ISL data was requested while in TRACK

    // =========================================================================
    // ISL pending latch
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            isl_pending <= 1'b0;
        else if (isl_data_valid)
            isl_pending <= 1'b1;
        else if (state == COMM && !isl_data_valid)
            isl_pending <= 1'b0;
    end

    // =========================================================================
    // FSM - state register
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else if (ce_100hz)
            state <= next_state;
    end

    // =========================================================================
    // Counter update
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hold_cnt  <= '0;
            fault_cnt <= '0;
        end else if (ce_100hz) begin
            case (next_state)
                HOLD: begin
                    hold_cnt  <= hold_cnt + 1;
                    fault_cnt <= '0;
                end
                FAULT: begin
                    fault_cnt <= fault_cnt + 1;
                end
                default: begin
                    hold_cnt  <= '0;
                    fault_cnt <= '0;
                end
            endcase
        end
    end

    // =========================================================================
    // FSM - next-state logic
    // =========================================================================
    always_comb begin
        next_state = state;

        // Global overrides
        if (!laser_enable || safe_mode) begin
            next_state = SAFE;
        end else begin
            case (state)
                IDLE: begin
                    if (laser_enable && signal_valid)
                        next_state = SEARCH;
                end

                SEARCH: begin
                    // Advance to ACQUIRE when peak found above threshold
                    if (peak_valid && peak_strength > SEARCH_THRESH[11:0])
                        next_state = ACQUIRE;
                    else if (scan_done)
                        // Rescan: restart from beginning (stay in SEARCH)
                        next_state = SEARCH;
                end

                ACQUIRE: begin
                    if (spiral_converged)
                        next_state = TRACK;
                end

                TRACK: begin
                    if (sig_filtered < HOLD_THRESH[11:0])
                        next_state = HOLD;
                    else if (isl_pending || isl_data_valid)
                        next_state = COMM;
                end

                COMM: begin
                    if (sig_filtered < HOLD_THRESH[11:0])
                        next_state = HOLD;
                    else if (!isl_pending && !isl_data_valid)
                        next_state = TRACK;
                end

                HOLD: begin
                    if (sig_filtered >= HOLD_THRESH[11:0]) begin
                        if (isl_pending)
                            next_state = COMM;
                        else
                            next_state = TRACK;
                    end else if (hold_cnt >= HOLD_TIMEOUT[CNT_W-1:0])
                        next_state = FAULT;
                end

                FAULT: begin
                    if (manual_clear && !gimbal_busy)
                        next_state = IDLE;
                end

                SAFE: begin
                    if (laser_enable && !safe_mode && !gimbal_busy)
                        next_state = IDLE;
                end

                default: next_state = IDLE;
            endcase
        end
    end

    // =========================================================================
    // Output assignments
    // =========================================================================
    always_comb begin
        laser_mod_en    = (state == TRACK || state == HOLD || state == COMM);
        gimbal_step     = step_raw;
        gimbal_dir      = dir_raw;
        laser_state     = state;
        pointing_locked = (state == TRACK || state == COMM);
        laser_fault     = (state == FAULT);
    end

endmodule