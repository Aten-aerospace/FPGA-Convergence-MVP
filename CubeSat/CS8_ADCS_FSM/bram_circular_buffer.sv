// =============================================================================
// Module: bram_circular_buffer (CS8 ADCS data log BRAM)
// Subsystem: CS8 - ADCS FSM & Health Monitor
// Description: Simple-dual-port 256×96-bit BRAM abstraction for ADCS circular
//              telemetry logging.
//
//   Write port (Port A): synchronous write at caller-supplied address.
//   Read  port (Port B): registered (synchronous) read for BRAM inference.
//
//   Circular behaviour (pointer management) is the caller's responsibility.
//   See adcs_data_logger.sv and fault_logger.sv for write-pointer logic.
//
//   Resource: 256 × 96 bits = 24,576 bits = 3,072 bytes → fits in one 36Kb BRAM.
//
// Provenance: CS-ADCS-012 (Data Logging)
// =============================================================================
`timescale 1ns/1ps

module bram_circular_buffer #(
    parameter int DEPTH = 256,   // number of log entries
    parameter int WIDTH = 96     // bits per entry
)(
    input  logic                      clk,

    // Write port (Port A) - synchronous, single-cycle latency
    input  logic                      wr_en,
    input  logic [$clog2(DEPTH)-1:0]  wr_addr,
    input  logic [WIDTH-1:0]          wr_data,

    // Read port (Port B) - registered output for BRAM inference
    input  logic [$clog2(DEPTH)-1:0]  rd_addr,
    output logic [WIDTH-1:0]          rd_data
);

    // -------------------------------------------------------------------------
    // Memory array - synthesiser should infer this as block RAM
    // -------------------------------------------------------------------------
    logic [WIDTH-1:0] mem [0:DEPTH-1];

    // Port A: synchronous write
    always_ff @(posedge clk) begin
        if (wr_en)
            mem[wr_addr] <= wr_data;
    end

    // Port B: registered read (required for BRAM inference on most FPGAs)
    always_ff @(posedge clk) begin
        rd_data <= mem[rd_addr];
    end

endmodule