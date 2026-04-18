// =============================================================================
// Module: system_monitor
// Subsystem: CS12 - System Integration
// Description: Monitors overall system health from all CubeSat subsystems.
//              Aggregates fault signals from ADCS, orbit, laser, actuators,
//              and telemetry subsystems.  Asserts sys_fault when any
//              individual fault persists for more than FAULT_PERSIST
//              ce_100hz ticks.
//
//   sys_fault_flags[4:0] bit assignments:
//     [0] adcs_fault
//     [1] orb_fault
//     [2] laser_fault
//     [3] actuator_fault
//     [4] tlm_fault
//
//   fault_cnt[7:0]: counts total fault-active ce_100hz ticks (saturates at 255)
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module system_monitor #(
    parameter int FAULT_PERSIST = 100  // ce_100hz ticks before sys_fault asserts
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    // Subsystem fault inputs
    input  logic        adcs_fault,
    input  logic        orb_fault,
    input  logic        laser_fault,
    input  logic        actuator_fault,
    input  logic        tlm_fault,

    // Aggregated outputs
    output logic        sys_ok,
    output logic [4:0]  sys_fault_flags,
    output logic        sys_fault,
    output logic [7:0]  fault_cnt
);

    // =========================================================================
    // Fault flag assembly (combinational)
    // =========================================================================
    always_comb begin
        sys_fault_flags[0] = adcs_fault;
        sys_fault_flags[1] = orb_fault;
        sys_fault_flags[2] = laser_fault;
        sys_fault_flags[3] = actuator_fault;
        sys_fault_flags[4] = tlm_fault;
    end

    logic any_fault;
    always_comb any_fault = |sys_fault_flags;

    // =========================================================================
    // Persistence counter
    // =========================================================================
    localparam int PERS_W = $clog2(FAULT_PERSIST + 2);
    logic [PERS_W-1:0] persist_cnt;
    logic              sys_fault_r;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            persist_cnt <= '0;
            sys_fault_r <= 1'b0;
        end else if (ce_100hz) begin
            if (!any_fault) begin
                persist_cnt <= '0;
                sys_fault_r <= 1'b0;
            end else if (persist_cnt == FAULT_PERSIST[PERS_W-1:0]) begin
                sys_fault_r <= 1'b1;
            end else begin
                persist_cnt <= persist_cnt + 1'b1;
            end
        end
    end

    // =========================================================================
    // Fault event counter (counts ce_100hz ticks while any fault active)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fault_cnt <= '0;
        end else if (ce_100hz && any_fault && fault_cnt != 8'hFF) begin
            fault_cnt <= fault_cnt + 1'b1;
        end
    end

    // =========================================================================
    // Outputs
    // =========================================================================
    always_comb begin
        sys_fault = sys_fault_r;
        sys_ok    = ~sys_fault_r;
    end

endmodule
