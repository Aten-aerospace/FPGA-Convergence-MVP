// =============================================================================
// Module: health_monitor (CS8 ADCS health monitor)
// Subsystem: CS8 - ADCS FSM & Health Monitor
// Description: Monitors ADCS sensor and estimator heartbeats and state
//              sanity checks. Asserts health_ok when all monitored quantities
//              are within acceptable bounds.
//
//   fault_flags[7:0] bit assignments:
//     [0] IMU heartbeat timeout    (imu_valid absent > HBEAT_TIMEOUT ce_100hz ticks)
//     [1] MAG heartbeat timeout    (mag_valid absent > HBEAT_TIMEOUT ce_100hz ticks)
//     [2] EKF heartbeat timeout    (ekf_valid absent > HBEAT_TIMEOUT ce_100hz ticks)
//     [3] Angular rate excess      (omega_max > OMEGA_THRESH)
//     [4] Quaternion norm error    (q_norm_err > QNORM_THRESH for >= QNORM_CYCLES ticks)
//     [5] External fault [3:0]     (OR of fault_trigger[3:0])
//     [6] External fault [7:4]     (OR of fault_trigger[7:4])
//     [7] Any fault                (OR of [0:6])
//
//   Thresholds (Q15 fixed-point, 1 = 32768):
//     OMEGA_THRESH  = 16'd3277  ≈ 0.1 rad/s in Q15 (10 deg/s limit)
//     QNORM_THRESH  = 16'd328   ≈ 0.01 in Q15 (1% quaternion norm error)
//     HBEAT_TIMEOUT = 1         (ce_100hz ticks ≈ 10 ms; CS-ADCS-011 req <5 ms)
//     QNORM_CYCLES  = 3         (consecutive ce_100hz ticks before latching fault)
//
//   per_axis_faults[23:0] layout (infrastructure for future per-axis expansion):
//     [2:0]   IMU gyro  X/Y/Z    - all = imu_timeout (pending per-axis inputs)
//     [5:3]   IMU accel X/Y/Z    - all = imu_timeout
//     [8:6]   MAG       X/Y/Z    - all = mag_timeout
//     [11:9]  EKF       X/Y/Z    - all = ekf_timeout
//     [14:12] Omega axis X/Y/Z   - all = omega_fault
//     [17:15] Qnorm axis X/Y/Z   - all = qnorm_fault
//     [23:18]                    - reserved (0)
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md  |  CS-ADCS-011
// =============================================================================
`timescale 1ns/1ps

module health_monitor #(
    parameter int HBEAT_TIMEOUT = 1,          // ce_100hz ticks (~10 ms; was 10)
    parameter int OMEGA_THRESH  = 3277,       // Q15 ≈ 0.1 rad/s
    parameter int QNORM_THRESH  = 328,        // Q15 ≈ 0.01
    parameter int QNORM_CYCLES  = 3           // consecutive ticks before qnorm fault
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    // Heartbeat strobes (one pulse when sensor/estimator produces a reading)
    input  logic        imu_valid,
    input  logic        mag_valid,
    input  logic        ekf_valid,

    // Magnitude quantities (Q15 unsigned)
    input  logic [15:0] omega_max,    // max |ω| across 3 axes from EKF/IMU
    input  logic [15:0] q_norm_err,  // |‖q‖ - 1| in Q15

    // External fault injection: bits[3:0] → fault_flags[5]; bits[7:4] → fault_flags[6]
    input  logic [7:0]  fault_trigger,

    output logic        health_ok,
    output logic [7:0]  fault_flags,
    output logic [23:0] per_axis_faults   // per-axis fault tracking (see header)
);

    // -------------------------------------------------------------------------
    // Heartbeat watchdog counters
    // -------------------------------------------------------------------------
    localparam int WDOG_W = $clog2(HBEAT_TIMEOUT + 2);

    logic [WDOG_W-1:0] imu_wdog;
    logic [WDOG_W-1:0] mag_wdog;
    logic [WDOG_W-1:0] ekf_wdog;

    logic imu_timeout, mag_timeout, ekf_timeout;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imu_wdog <= '0;
            mag_wdog <= '0;
            ekf_wdog <= '0;
        end else begin
            // IMU watchdog
            if (imu_valid)
                imu_wdog <= '0;
            else if (ce_100hz && imu_wdog != HBEAT_TIMEOUT[WDOG_W-1:0])
                imu_wdog <= imu_wdog + 1;

            // MAG watchdog
            if (mag_valid)
                mag_wdog <= '0;
            else if (ce_100hz && mag_wdog != HBEAT_TIMEOUT[WDOG_W-1:0])
                mag_wdog <= mag_wdog + 1;

            // EKF watchdog
            if (ekf_valid)
                ekf_wdog <= '0;
            else if (ce_100hz && ekf_wdog != HBEAT_TIMEOUT[WDOG_W-1:0])
                ekf_wdog <= ekf_wdog + 1;
        end
    end

    always_comb begin
        imu_timeout = (imu_wdog == HBEAT_TIMEOUT[WDOG_W-1:0]);
        mag_timeout = (mag_wdog == HBEAT_TIMEOUT[WDOG_W-1:0]);
        ekf_timeout = (ekf_wdog == HBEAT_TIMEOUT[WDOG_W-1:0]);
    end

    // -------------------------------------------------------------------------
    // Sanity checks
    // -------------------------------------------------------------------------
    logic omega_fault;
    logic qnorm_raw;           // immediate qnorm error
    logic qnorm_fault;         // latched after QNORM_CYCLES consecutive ticks

    always_comb begin
        omega_fault = (omega_max  > OMEGA_THRESH[15:0]);
        qnorm_raw   = (q_norm_err > QNORM_THRESH[15:0]);
    end

    // 3-cycle consecutive counter for qnorm fault (CS-ADCS-011 transient rejection)
    // Width = $clog2(QNORM_CYCLES+1) holds values 0..QNORM_CYCLES exactly.
    localparam int QNC_W = $clog2(QNORM_CYCLES + 1);
    logic [QNC_W-1:0] qnorm_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            qnorm_cnt <= '0;
        else if (ce_100hz) begin
            if (!qnorm_raw)
                qnorm_cnt <= '0;
            else if (qnorm_cnt != QNC_W'(QNORM_CYCLES))
                qnorm_cnt <= qnorm_cnt + 1;
        end
    end

    always_comb qnorm_fault = (qnorm_cnt == QNC_W'(QNORM_CYCLES));

    // -------------------------------------------------------------------------
    // Fault flag assembly (including external fault_trigger injection)
    // -------------------------------------------------------------------------
    always_comb begin
        fault_flags[0] = imu_timeout;
        fault_flags[1] = mag_timeout;
        fault_flags[2] = ekf_timeout;
        fault_flags[3] = omega_fault;
        fault_flags[4] = qnorm_fault;
        fault_flags[5] = |fault_trigger[3:0];  // external fault injection low nibble
        fault_flags[6] = |fault_trigger[7:4];  // external fault injection high nibble
        fault_flags[7] = imu_timeout | mag_timeout | ekf_timeout |
                         omega_fault | qnorm_fault |
                         fault_flags[5] | fault_flags[6];
        health_ok      = ~fault_flags[7];
    end

    // -------------------------------------------------------------------------
    // Per-axis fault tracking (infrastructure for future per-axis inputs)
    //   Currently all bits within a group reflect the per-sensor fault since
    //   individual axis inputs are not yet available at this interface level.
    // -------------------------------------------------------------------------
    always_comb begin
        per_axis_faults[ 2: 0] = {3{imu_timeout}};  // IMU gyro  X/Y/Z
        per_axis_faults[ 5: 3] = {3{imu_timeout}};  // IMU accel X/Y/Z
        per_axis_faults[ 8: 6] = {3{mag_timeout}};  // MAG X/Y/Z
        per_axis_faults[11: 9] = {3{ekf_timeout}};  // EKF X/Y/Z
        per_axis_faults[14:12] = {3{omega_fault}};  // Omega axis X/Y/Z
        per_axis_faults[17:15] = {3{qnorm_fault}};  // Qnorm axis X/Y/Z
        per_axis_faults[23:18] = '0;                // reserved
    end

endmodule