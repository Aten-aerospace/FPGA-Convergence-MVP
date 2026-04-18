// =============================================================================
// Module: laser_fault_handler (CS10 fault logging FIFO)
// Subsystem: CS10 - Laser Pointing FSM
// Description: Logs laser faults (loss-of-signal, gimbal error) to an
//              8-entry circular FIFO and manages FAULT-state recovery.
//
//   FIFO is 8 entries of 24 bits: {fault_code[7:0], timestamp[15:0]}.
//   On fault_trigger: write {fault_code, timestamp} to FIFO if not full.
//   manual_clear allows FSM to retry after fault acknowledgement.
//
//   Fault codes (fault_code[7:0]):
//     8'h00 - Signal loss
//     8'h01 - Gimbal error
//     8'h02 - Timeout
//     8'hFF - Manual clear event
//
// Requirement: CS-LSR-007 (FAULT State - safe, log, manual clear)
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module laser_fault_handler (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    input  logic        fault_trigger,      // strobe when fault event occurs
    input  logic [7:0]  fault_code,         // fault reason
    input  logic [15:0] timestamp,          // mission time for fault logging
    input  logic        manual_clear,       // uplink command to clear FAULT

    output logic [23:0] fault_fifo_data,    // {fault_code[7:0], timestamp[15:0]}
    output logic        fault_fifo_wr_en,   // write strobe
    output logic        fault_fifo_full,    // 8-entry FIFO full
    output logic        fault_logged        // asserts when fault written to FIFO
);

    // =========================================================================
    // 8-entry circular FIFO (24-bit wide)
    // =========================================================================
    localparam int DEPTH = 8;
    localparam int ADDR_W = 3;

    logic [23:0] fifo_mem [0:DEPTH-1];
    logic [ADDR_W-1:0] wr_ptr;
    logic [ADDR_W-1:0] rd_ptr;
    logic [ADDR_W:0]   count;   // 0..8

    assign fault_fifo_full = (count == DEPTH[ADDR_W:0]);
    assign fault_fifo_data = fifo_mem[rd_ptr];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr          <= '0;
            rd_ptr          <= '0;
            count           <= '0;
            fault_fifo_wr_en <= 1'b0;
            fault_logged    <= 1'b0;
        end else begin
            fault_fifo_wr_en <= 1'b0;
            fault_logged     <= 1'b0;

            if (fault_trigger && !fault_fifo_full) begin
                fifo_mem[wr_ptr] <= {fault_code, timestamp};
                wr_ptr           <= wr_ptr + 1;
                count            <= count + 1;
                fault_fifo_wr_en <= 1'b1;
                fault_logged     <= 1'b1;
            end

            // manual_clear: log the clear event and drain oldest entry
            if (manual_clear && count > '0) begin
                rd_ptr <= rd_ptr + 1;
                count  <= count - 1;
            end
        end
    end

endmodule