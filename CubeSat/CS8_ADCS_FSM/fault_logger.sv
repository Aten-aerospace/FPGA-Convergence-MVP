// =============================================================================
// Module: fault_logger (CS8 ADCS fault event logger)
// Subsystem: CS8 - ADCS FSM & Health Monitor
// Description: Logs discrete fault events (code + timestamp) into the shared
//              bram_circular_buffer.  Uses a dedicated 8-entry sub-space at
//              BRAM addresses [BRAM_BASE : BRAM_BASE+7] to avoid colliding
//              with the 100 Hz data log written by adcs_data_logger.
//
//   96-bit entry layout:
//     [95:88] fault_code [7:0]   - code of the fault event
//     [87:80] timestamp  [7:0]   - 8-bit tick counter at time of event
//     [79: 0] padding            - zero
//
//   log_en: pulse high for one clk cycle to latch and write the current
//           fault_code + timestamp.  Uses synchronous write, so the BRAM
//           write fires on the following posedge.
//
//   Write pointer wraps within [BRAM_BASE : BRAM_BASE + LOG_DEPTH - 1].
//
// Provenance: CS-ADCS-012 (Data Logging)
// =============================================================================
`timescale 1ns/1ps

module fault_logger #(
    parameter int LOG_DEPTH  = 8,    // number of fault event slots
    parameter int BRAM_BASE  = 248   // start address within shared BRAM (248-255)
)(
    input  logic       clk,
    input  logic       rst_n,

    // Fault event inputs
    input  logic [7:0] fault_code,   // fault flags / code at time of event
    input  logic [7:0] timestamp,    // 8-bit tick counter

    // Strobe: write this fault event to BRAM
    input  logic       log_en,

    // BRAM write interface → connect to bram_circular_buffer Port A
    output logic [7:0]  bram_wr_addr,
    output logic [95:0] bram_wr_data,
    output logic        bram_wr_en
);

    // -------------------------------------------------------------------------
    // Local write pointer (wraps within LOG_DEPTH)
    // -------------------------------------------------------------------------
    localparam int PTR_W = $clog2(LOG_DEPTH);
    logic [PTR_W-1:0] ptr;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            ptr <= '0;
        else if (log_en)
            ptr <= (ptr == PTR_W'(LOG_DEPTH - 1)) ? '0 : ptr + 1;
    end

    // -------------------------------------------------------------------------
    // BRAM write interface
    // -------------------------------------------------------------------------
    always_comb begin
        bram_wr_en   = log_en;
        bram_wr_addr = 8'(BRAM_BASE) + 8'(ptr);
        bram_wr_data = { fault_code,    // [95:88]
                         timestamp,     // [87:80]
                         80'b0 };       // [79:0] padding
    end

endmodule