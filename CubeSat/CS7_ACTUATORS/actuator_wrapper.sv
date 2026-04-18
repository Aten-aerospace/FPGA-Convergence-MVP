// =============================================================================
// Module: actuator_wrapper (CS7 top-level actuator wrapper)
// Subsystem: CS7 - Actuator Drivers
// Requirements: CS-ADCS-008 (RW SPI commands + 10 Hz fault polling),
//               CS-ADCS-009 (MTQ 10 kHz PWM, cross-axis coupling <1%)
//
// Description:
//   Full-compliance wrapper integrating all CS7 sub-modules:
//     actuator_command_arbiter  - torque_cmd[3] → RW speed commands
//     rw_spi_driver             - SPI motor driver commands + fault polling
//     rw_driver                 - legacy PWM back-channel for RW gate enables
//     mtq_driver                - torque_cmd[3] → 10 kHz PWM + direction + sat flags
//     fault_status_monitor      - watchdog + SPI fault merge → rw_fault[3]
//
//   Specification interface (CS-ADCS-008/009):
//     Inputs : sys_clk, clk_100mhz, rst_n, ce_1khz, torque_cmd[3], safe_mode
//     Outputs: rw_sclk, rw_mosi[3], rw_cs_n[3], rw_miso[3], mtq_pwm[3],
//              rw_fault[3]
//
//   Extended outputs for hardware/telemetry (beyond spec minimum):
//     rw_enable, dir_mtq, mtq_enable, mtq_sat_flag, coupling_warning
//
// Safe-mode blanking: within 1 control cycle (combinational in arbiter +
//   output mux), meeting CS-ADCS-008/009 "1 cycle blanking" requirement.
//
// Timing: torque_cmd → SPI transaction < 6 µs; MTQ PWM update < 100 µs;
//         both well within the 1 ms execution budget.
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module actuator_wrapper #(
    parameter int CLK_HZ = 100_000_000
)(
    // Primary spec clocks (both 100 MHz; sys_clk drives control, clk_100mhz PWM)
    input  logic        sys_clk,
    input  logic        clk_100mhz,
    input  logic        rst_n,
    input  logic        ce_1khz,

    // Torque commands from CS6 (Q15 signed, one per axis); routed to both RW and MTQ
    input  logic signed [15:0] torque_cmd [0:2],
    input  logic               cmd_valid,

    // Safe mode from CS8: blanks all actuator outputs within 1 control cycle
    input  logic               safe_mode,

    // --- Spec-required outputs ---

    // Reaction-wheel SPI bus
    output logic        rw_sclk,
    output logic [2:0]  rw_mosi,
    output logic [2:0]  rw_cs_n,
    input  logic [2:0]  rw_miso,

    // Magnetorquer PWM (CS-ADCS-009: 10 kHz carrier)
    output logic [2:0]  mtq_pwm,

    // Per-axis RW fault: watchdog timeout OR SPI-reported motor fault
    output logic [2:0]  rw_fault,

    // --- Extended hardware/telemetry outputs (beyond spec minimum) ---

    // Reaction-wheel PWM enable (from rw_driver PWM watchdog)
    output logic [2:0]  rw_enable,

    // Magnetorquer H-bridge direction and enable
    output logic [2:0]  dir_mtq,
    output logic [2:0]  mtq_enable,

    // Cross-axis coupling monitors (CS-ADCS-009 <1% coupling)
    output logic [2:0]  mtq_sat_flag,
    output logic        coupling_warning
);

    // =========================================================================
    // Internal wires between sub-modules
    // =========================================================================

    // Arbiter outputs (RW path only)
    logic signed [15:0] rw_cmd      [0:2];
    logic               rw_cmd_valid;

    // SPI fault bits (rw_spi_driver → fault_status_monitor)
    logic [2:0]         rw_spi_fault;

    // MTQ driver raw outputs (gated with safe_mode before export)
    logic [2:0]         mtq_pwm_raw;
    logic [2:0]         mtq_dir_raw;
    logic [2:0]         mtq_en_raw;
    logic [2:0]         mtq_sat_raw;
    logic               coup_warn_raw;

    // =========================================================================
    // actuator_command_arbiter
    //   Splits torque_cmd[3] → RW speed (Q15); safe_mode and cmd_valid gating
    // =========================================================================
    actuator_command_arbiter u_arb (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .torque_cmd   (torque_cmd),
        .cmd_valid    (cmd_valid),
        .safe_mode    (safe_mode),
        .rw_cmd       (rw_cmd),
        .rw_cmd_valid (rw_cmd_valid)
    );

    // =========================================================================
    // rw_spi_driver
    //   Polls and sends RW speed commands @ 10 Hz; SPI clock @ ~8.33 MHz
    // =========================================================================
    rw_spi_driver #(
        .CLK_HZ (CLK_HZ)
    ) u_rw_spi (
        .clk        (sys_clk),
        .rst_n      (rst_n),
        .ce_1khz    (ce_1khz),
        .torque_cmd (rw_cmd),
        .cmd_valid  (rw_cmd_valid),
        .safe_mode  (safe_mode),
        .rw_sclk    (rw_sclk),
        .rw_mosi    (rw_mosi),
        .rw_cs_n    (rw_cs_n),
        .rw_miso    (rw_miso),
        .rw_fault   (rw_spi_fault)
    );

    // =========================================================================
    // rw_driver (legacy PWM path: bi-directional PWM + internal watchdog)
    //   Kept for hardware back-channel; rw_enable used to gate motor FETs
    // =========================================================================
    logic [2:0] rw_pwm_raw;
    logic [2:0] rw_en_raw;
    logic       rw_fault_pwm;   // PWM-path watchdog (not exported; FSM uses rw_fault)

    rw_driver #(
        .CLK_HZ (CLK_HZ)
    ) u_rw (
        .clk        (sys_clk),
        .rst_n      (rst_n),
        .ce_1khz    (ce_1khz),
        .torque_cmd (rw_cmd),
        .cmd_valid  (rw_cmd_valid),
        .pwm_out    (rw_pwm_raw),
        .rw_enable  (rw_en_raw),
        .rw_fault   (rw_fault_pwm)
    );

    assign rw_enable = safe_mode ? 3'b000 : rw_en_raw;

    // =========================================================================
    // mtq_driver
    //   Takes torque_cmd[3] (Q15) directly: computes magnitude + direction,
    //   generates 3× 10 kHz PWM, and reports saturation / coupling flags.
    //   Uses clk_100mhz for PWM counter (clock-domain separation from control).
    //   Safe-mode blanking is applied combinationally on the outputs below.
    // =========================================================================
    mtq_driver #(
        .CLK_HZ (CLK_HZ)
    ) u_mtq (
        .clk             (clk_100mhz),
        .rst_n           (rst_n),
        .ce_1khz         (ce_1khz),
        .dipole_cmd      (torque_cmd),
        .cmd_valid       (cmd_valid),
        .mtq_pwm         (mtq_pwm_raw),
        .mtq_dir         (mtq_dir_raw),
        .mtq_enable      (mtq_en_raw),
        .mtq_sat_flag    (mtq_sat_raw),
        .coupling_warning(coup_warn_raw)
    );

    // Safe-mode blanking: gate all MTQ outputs combinationally
    assign mtq_pwm        = safe_mode ? 3'b000 : mtq_pwm_raw;
    assign dir_mtq        = safe_mode ? 3'b000 : mtq_dir_raw;
    assign mtq_enable     = safe_mode ? 3'b000 : mtq_en_raw;
    assign mtq_sat_flag   = safe_mode ? 3'b000 : mtq_sat_raw;
    assign coupling_warning = safe_mode ? 1'b0  : coup_warn_raw;

    // =========================================================================
    // fault_status_monitor
    //   Merges watchdog fault with per-axis SPI fault → rw_fault[2:0].
    //   MTQ saturation monitoring is handled by mtq_driver above.
    // =========================================================================
    fault_status_monitor u_fault (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .ce_1khz      (ce_1khz),
        .cmd_valid    (cmd_valid),
        .rw_spi_fault (rw_spi_fault),
        .rw_fault     (rw_fault)
    );

endmodule