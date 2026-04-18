// =============================================================================
// Module: adcs_fsm_wrapper (CS8 ADCS mode FSM + health monitor)
// Subsystem: CS8 - ADCS FSM & Health Monitor
// Description: 6-state ADCS mode finite-state machine.
//
//   States:
//     BOOT        : Power-on initialisation, wait for sensor valid.
//     DETUMBLE    : High angular-rate regime; B-dot or spin-down mode.
//     COARSE_POINT: Moderate rate; coarse sun/mag pointing.
//     FINE_POINT  : Low rate + small pointing error; precision control.
//     SAFE        : health_ok de-asserted or uplink override; actuators off.
//     FAULT       : Persistent health failure; minimal operations.
//
//   Transition triggers:
//     BOOT       → DETUMBLE    : imu_valid & mag_valid
//     DETUMBLE   → COARSE_POINT: omega_mag < OMEGA_DET_THRESH
//     COARSE_POINT→ FINE_POINT : q_err_mag < QERR_COARSE_THRESH
//     FINE_POINT → COARSE_POINT: q_err_mag > QERR_FINE_THRESH  OR  !health_ok
//     any        → SAFE        : (uplink_mode = SAFE_CMD & ul_active) or !health_ok
//     any        → FAULT       : fault_flags[7] held for FAULT_PERSIST ticks
//     SAFE/FAULT → BOOT        : (uplink_mode = BOOT_CMD & ul_active) & health_ok
//     any non-SAFE/FAULT → SAFE: uplink command timeout (ul_active de-asserts)
//
//   adcs_mode[2:0] encoding matches state enum (binary-coded).
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md  |  CS-ADCS-010, CS-ADCS-012
// =============================================================================
`timescale 1ns/1ps

module adcs_fsm_wrapper #(
    parameter int OMEGA_DET_THRESH   = 1638,  // Q15 ≈ 0.05 rad/s
    parameter int QERR_COARSE_THRESH = 3277,  // Q15 ≈ 0.1 rad (≈5.7°)
    parameter int QERR_FINE_THRESH   = 6554,  // Q15 ≈ 0.2 rad (≈11.5°)
    parameter int FAULT_PERSIST      = 1,     // ce_100hz ticks (~10 ms; was 50)
    parameter int UL_CMD_TIMEOUT     = 10     // ce_100hz ticks (100 ms) before uplink stale
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    // Sensor heartbeats
    input  logic        imu_valid,
    input  logic        mag_valid,
    input  logic        ekf_valid,

    // State magnitudes (Q15 unsigned)
    input  logic [15:0] q_err_mag,   // quaternion error magnitude from CS5
    input  logic [15:0] omega_mag,   // angular rate magnitude from CS1

    // Uplink command (3-bit mode override; 3'd7 = no command)
    input  logic [2:0]  uplink_mode,
    // Uplink command valid strobe: pulse high when uplink_mode contains a fresh command
    // Tie to 1'b1 for always-valid (backward-compatible) operation.
    input  logic        uplink_cmd_valid,

    // External fault injection (8-bit): routed to health_monitor.fault_trigger
    input  logic [7:0]  fault_trigger,

    // ADCS telemetry inputs for 100 Hz BRAM logging
    input  logic [31:0] q_est,       // packed quaternion estimate (4×8-bit) from CS5
    input  logic [23:0] omega_in,    // packed angular rate (3×8-bit) from CS1

    // BRAM log read interface (for CS11 telemetry readout)
    input  logic [7:0]  bram_log_rd_addr,
    output logic [95:0] bram_log_rd_data,

    // BRAM log write interface (exposed for external monitoring)
    output logic [7:0]  bram_log_addr,
    output logic [95:0] bram_log_data,
    output logic        bram_log_wr_en,

    // Mode / health outputs
    output logic [2:0]  adcs_mode,   // current mode (matches state enum)
    output logic        mode_valid,
    output logic        health_ok,
    output logic [7:0]  fault_flags,
    output logic        adcs_fault,

    // Per-axis fault tracking from health_monitor
    output logic [23:0] per_axis_faults
);

    // =========================================================================
    // Uplink command constants
    // =========================================================================
    localparam logic [2:0] UL_BOOT  = 3'd0;
    localparam logic [2:0] UL_SAFE  = 3'd4;
    localparam logic [2:0] UL_NONE  = 3'd7;

    // =========================================================================
    // Uplink command timeout (CS-ADCS-010: validate / timeout uplink commands)
    // =========================================================================
    // ul_active: uplink_mode is considered fresh when this is high.
    // The counter resets on every uplink_cmd_valid pulse; after UL_CMD_TIMEOUT
    // ce_100hz ticks without a refresh the uplink is treated as stale.
    // Tie uplink_cmd_valid = 1'b1 to disable the timeout (always-valid mode).
    localparam int UL_W = $clog2(UL_CMD_TIMEOUT + 2);
    logic [UL_W-1:0] ul_cnt;
    logic ul_active;   // high when uplink is within timeout window

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ul_cnt <= '0;
        else if (uplink_cmd_valid)
            ul_cnt <= '0;
        else if (ce_100hz && ul_cnt != UL_W'(UL_CMD_TIMEOUT))
            ul_cnt <= ul_cnt + 1;
    end

    always_comb ul_active = (ul_cnt < UL_W'(UL_CMD_TIMEOUT));

    // =========================================================================
    // State encoding
    // =========================================================================
    typedef enum logic [2:0] {
        BOOT         = 3'd0,
        DETUMBLE     = 3'd1,
        COARSE_POINT = 3'd2,
        FINE_POINT   = 3'd3,
        SAFE         = 3'd4,
        FAULT        = 3'd5
    } adcs_state_t;

    adcs_state_t state, next_state;

    // =========================================================================
    // Health monitor instance
    // =========================================================================
    // omega_max / q_norm_err wired from top-level inputs (omega_mag / q_err_mag)
    health_monitor #(
        .HBEAT_TIMEOUT (1)    // was 10 - now ~10 ms (CS-ADCS-011: timeout >5 ms)
    ) u_health (
        .clk            (clk),
        .rst_n          (rst_n),
        .ce_100hz       (ce_100hz),
        .imu_valid      (imu_valid),
        .mag_valid      (mag_valid),
        .ekf_valid      (ekf_valid),
        .omega_max      (omega_mag),
        .q_norm_err     (q_err_mag),
        .fault_trigger  (fault_trigger),
        .health_ok      (health_ok),
        .fault_flags    (fault_flags),
        .per_axis_faults(per_axis_faults)
    );

    // =========================================================================
    // Fault persistence counter (CS-ADCS-010: FAULT transition < 2 ms)
    // =========================================================================
    localparam int PERS_W = $clog2(FAULT_PERSIST + 2);
    logic [PERS_W-1:0] fault_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            fault_cnt <= '0;
        else if (health_ok)
            fault_cnt <= '0;
        else if (ce_100hz && fault_cnt != FAULT_PERSIST[PERS_W-1:0])
            fault_cnt <= fault_cnt + 1;
    end

    logic fault_persistent;
    always_comb fault_persistent = (fault_cnt == FAULT_PERSIST[PERS_W-1:0]);

    // =========================================================================
    // FSM - state register
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= BOOT;
        else if (ce_100hz)
            state <= next_state;
    end

    // =========================================================================
    // FSM - next-state logic
    // =========================================================================
    always_comb begin
        next_state = state;

        // Highest priority: uplink SAFE override (only when command is fresh)
        if (ul_active && uplink_mode == UL_SAFE) begin
            next_state = SAFE;
        // Uplink timeout guard: if not in SAFE/FAULT/BOOT and uplink expired,
        // transition to SAFE to ensure safe operation
        end else if (!ul_active && state != SAFE && state != FAULT && state != BOOT) begin
            next_state = SAFE;
        end else if (fault_persistent) begin
            next_state = FAULT;
        end else begin
            case (state)
                BOOT: begin
                    if (imu_valid && mag_valid)
                        next_state = DETUMBLE;
                end

                DETUMBLE: begin
                    if (!health_ok)
                        next_state = SAFE;
                    else if (omega_mag < OMEGA_DET_THRESH[15:0])
                        next_state = COARSE_POINT;
                end

                COARSE_POINT: begin
                    if (!health_ok)
                        next_state = SAFE;
                    else if (q_err_mag < QERR_COARSE_THRESH[15:0])
                        next_state = FINE_POINT;
                end

                FINE_POINT: begin
                    if (!health_ok || q_err_mag > QERR_FINE_THRESH[15:0])
                        next_state = COARSE_POINT;
                end

                SAFE: begin
                    if (ul_active && uplink_mode == UL_BOOT && health_ok)
                        next_state = BOOT;
                end

                FAULT: begin
                    if (ul_active && uplink_mode == UL_BOOT && health_ok)
                        next_state = BOOT;
                end

                default: next_state = BOOT;
            endcase
        end
    end

    // =========================================================================
    // Output assignments
    // =========================================================================
    always_comb begin
        adcs_mode  = state;
        mode_valid = (state != BOOT);
        adcs_fault = (state == FAULT);
    end

    // =========================================================================
    // 8-bit tick timestamp counter (used by fault_logger)
    // =========================================================================
    logic [7:0] tick_cnt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            tick_cnt <= '0;
        else if (ce_100hz)
            tick_cnt <= tick_cnt + 1;
    end

    // =========================================================================
    // Fault edge detection for fault_logger strobe
    // =========================================================================
    logic [7:0] fault_flags_prev;
    logic        new_fault_event;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            fault_flags_prev <= '0;
        else if (ce_100hz)
            fault_flags_prev <= fault_flags;
    end

    // New fault: any bit newly asserted (rising edge on any fault bit)
    always_comb new_fault_event = |(fault_flags & ~fault_flags_prev);

    // =========================================================================
    // Fault logger instance (writes to BRAM addresses 248-255)
    // =========================================================================
    logic [7:0]  fl_bram_wr_addr;
    logic [95:0] fl_bram_wr_data;
    logic        fl_bram_wr_en;

    fault_logger #(
        .LOG_DEPTH (8),
        .BRAM_BASE (248)
    ) u_fault_logger (
        .clk          (clk),
        .rst_n        (rst_n),
        .fault_code   (fault_flags),
        .timestamp    (tick_cnt),
        .log_en       (new_fault_event),
        .bram_wr_addr (fl_bram_wr_addr),
        .bram_wr_data (fl_bram_wr_data),
        .bram_wr_en   (fl_bram_wr_en)
    );

    // =========================================================================
    // ADCS data logger instance (writes to BRAM addresses 0-247 @ 100 Hz)
    // =========================================================================
    logic [7:0]  dl_bram_wr_addr;
    logic [95:0] dl_bram_wr_data;
    logic        dl_bram_wr_en;

    adcs_data_logger #(
        .DEPTH (256)
    ) u_data_logger (
        .clk          (clk),
        .rst_n        (rst_n),
        .ce_100hz     (ce_100hz),
        .q_est        (q_est),
        .omega        (omega_in),
        .adcs_mode    (adcs_mode),
        .fault_flags  (fault_flags),
        .health_ok    (health_ok),
        .bram_wr_addr (dl_bram_wr_addr),
        .bram_wr_data (dl_bram_wr_data),
        .bram_wr_en   (dl_bram_wr_en),
        .wr_ptr       ()
    );

    // =========================================================================
    // BRAM write arbitration: fault_logger has priority over data_logger
    // =========================================================================
    logic [7:0]  bram_wr_addr_i;
    logic [95:0] bram_wr_data_i;
    logic        bram_wr_en_i;

    always_comb begin
        if (fl_bram_wr_en) begin
            bram_wr_addr_i = fl_bram_wr_addr;
            bram_wr_data_i = fl_bram_wr_data;
            bram_wr_en_i   = 1'b1;
        end else begin
            bram_wr_addr_i = dl_bram_wr_addr;
            bram_wr_data_i = dl_bram_wr_data;
            bram_wr_en_i   = dl_bram_wr_en;
        end
    end

    // =========================================================================
    // BRAM circular buffer instance (256 × 96-bit)
    // =========================================================================
    bram_circular_buffer #(
        .DEPTH (256),
        .WIDTH (96)
    ) u_bram (
        .clk     (clk),
        .wr_en   (bram_wr_en_i),
        .wr_addr (bram_wr_addr_i),
        .wr_data (bram_wr_data_i),
        .rd_addr (bram_log_rd_addr),
        .rd_data (bram_log_rd_data)
    );

    // =========================================================================
    // Expose BRAM write interface for external monitoring (CS11 telemetry)
    // =========================================================================
    always_comb begin
        bram_log_addr  = bram_wr_addr_i;
        bram_log_data  = bram_wr_data_i;
        bram_log_wr_en = bram_wr_en_i;
    end

endmodule