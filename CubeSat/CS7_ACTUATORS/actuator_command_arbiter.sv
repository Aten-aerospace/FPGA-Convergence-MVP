// =============================================================================
// Module: actuator_command_arbiter (CS7 torque command router - RW path)
// Subsystem: CS7 - Actuator Drivers
// Requirements: CS-ADCS-008 (RW speed commands @ 1 kHz),
//               CS-ADCS-009 (safe-mode blanking)
//
// Description:
//   Routes torque_cmd[3] (Q15 signed, from CS6 PD controller) to the reaction-
//   wheel actuator path.  MTQ commands are handled directly by mtq_driver.sv.
//
//     - Reaction wheels (RW): torque_cmd passed directly as Q15 speed
//       commands for the SPI motor driver.
//
//   Safe-mode blanking: when safe_mode is asserted, all output commands are
//   zeroed combinationally within the same control cycle (< 1 cycle latency).
//
// Command mapping:
//   RW  : rw_cmd[i]   = safe_mode ? 0 : torque_cmd[i]         (Q15)
//
// Timing: combinational.
//   rw_cmd is valid same cycle torque_cmd changes.
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module actuator_command_arbiter (
    input  logic        clk,
    input  logic        rst_n,

    // Torque commands from CS6 PD controller (Q15 signed, one per axis)
    input  logic signed [15:0] torque_cmd [0:2],
    input  logic               cmd_valid,

    // Safe mode from CS8: blanks all actuator commands within 1 cycle
    input  logic               safe_mode,

    // Reaction-wheel path: Q15 speed commands (direct pass-through, safe-gated)
    // rw_cmd_valid mirrors cmd_valid and is de-asserted in safe_mode
    output logic signed [15:0] rw_cmd       [0:2],
    output logic               rw_cmd_valid
);

    // =========================================================================
    // RW path: combinational pass-through gated by safe_mode
    // =========================================================================
    always_comb begin
        for (int i = 0; i < 3; i++)
            rw_cmd[i] = (safe_mode || !cmd_valid) ? 16'sd0 : torque_cmd[i];
        rw_cmd_valid = cmd_valid && !safe_mode;
    end

endmodule