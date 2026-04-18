// =============================================================================
// Module: fault_status_monitor (CS7 RW fault aggregation)
// Subsystem: CS7 - Actuator Drivers
// Requirements: CS-ADCS-008 - "fault status read @ 10 Hz"
//
// Description:
//   Aggregates per-axis RW fault information from two sources and exports a
//   single rw_fault[2:0] array for use by CS8 (ADCS FSM) and CS11 (telemetry):
//
//   Source 1 - CMD_VALID watchdog:
//     Fires if cmd_valid is absent for FAULT_TIMEOUT consecutive ce_1khz ticks
//     (default 200 ms).  Applies to all three RW axes simultaneously.
//
//   Source 2 - SPI fault bits (from rw_spi_driver.sv):
//     Per-axis fault bit read back from each motor controller over SPI.
//     Stored in rw_spi_fault[2:0] input; latched per axis.
//
//   Combined: rw_fault[i] = watchdog_fault | rw_spi_fault[i]
//
//   MTQ saturation monitoring (CS-ADCS-009 <1% cross-axis coupling) is handled
//   directly by mtq_driver.sv (mtq_sat_flag, coupling_warning).
//
// Timing: rw_fault updated on each clk cycle (combinational merge with
//         registered watchdog and latched SPI faults).
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module fault_status_monitor #(
    parameter int FAULT_TIMEOUT = 200   // watchdog: N ce_1khz ticks (default = 200 ms)
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_1khz,

    // cmd_valid handshake from CS6; absence triggers watchdog fault
    input  logic        cmd_valid,

    // Per-axis SPI fault bits from rw_spi_driver (bit[i] = fault from wheel i)
    input  logic [2:0]  rw_spi_fault,

    // Per-axis RW fault: watchdog (broadcast) OR per-axis SPI fault
    output logic [2:0]  rw_fault
);

    // =========================================================================
    // CMD_VALID watchdog timer
    //   Counts ce_1khz ticks while cmd_valid is absent.
    //   Fires (wdog_fault = 1) after FAULT_TIMEOUT consecutive absent ticks.
    //   Clears immediately when cmd_valid returns.
    // =========================================================================
    localparam int WDOG_W = $clog2(FAULT_TIMEOUT + 1);

    logic [WDOG_W-1:0] wdog_cnt;
    logic              wdog_fault;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdog_cnt   <= '0;
            wdog_fault <= 1'b0;
        end else begin
            if (cmd_valid) begin
                // cmd_valid received: reset watchdog and clear fault
                wdog_cnt   <= '0;
                wdog_fault <= 1'b0;
            end else if (ce_1khz) begin
                if (wdog_cnt == WDOG_W'(FAULT_TIMEOUT)) begin
                    wdog_fault <= 1'b1;   // timeout: fire fault
                end else begin
                    wdog_cnt <= wdog_cnt + 1;
                end
            end
        end
    end

    // =========================================================================
    // Per-axis SPI fault latch
    //   Faults cleared when cmd_valid returns (implying motor driver recovered).
    //   Held until cleared to ensure CS8 can observe the fault.
    // =========================================================================
    logic [2:0] spi_fault_lat;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            spi_fault_lat <= 3'b000;
        end else begin
            if (cmd_valid) begin
                // Clear latched SPI faults when system is healthy again
                spi_fault_lat <= 3'b000;
            end else begin
                // Latch any new SPI faults (sticky until cmd_valid)
                spi_fault_lat <= spi_fault_lat | rw_spi_fault;
            end
        end
    end

    // =========================================================================
    // Combined per-axis RW fault output
    //   wdog_fault is broadcast to all axes (global timeout affects all wheels)
    //   spi_fault_lat is per-axis (each motor driver reports independently)
    // =========================================================================
    assign rw_fault = {3{wdog_fault}} | spi_fault_lat;

endmodule