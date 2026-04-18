// =============================================================================
// Module: adcs_data_logger (CS8 ADCS telemetry data logger)
// Subsystem: CS8 - ADCS FSM & Health Monitor
// Description: Samples ADCS state at 100 Hz and packs it into a 96-bit log
//              frame for storage in bram_circular_buffer.  Manages its own
//              circular write pointer (wraps at DEPTH).
//
//   96-bit frame layout:
//     [95:64] q_est [31:0]    - packed quaternion estimate (4 × 8-bit)
//     [63:40] omega [23:0]    - packed angular rate       (3 × 8-bit)
//     [39:37] adcs_mode [2:0] - current FSM mode
//     [36:29] fault_flags[7:0]- active fault bits
//     [28]    health_ok       - global health status
//     [27:20] wr_ptr [7:0]    - current write-pointer (for debug)
//     [19: 0] padding         - zero
//
//   Write rate: one entry per ce_100hz strobe.
//   Circular:   write pointer increments mod DEPTH on every write.
//
// Provenance: CS-ADCS-012 (Data Logging)
// =============================================================================
`timescale 1ns/1ps

module adcs_data_logger #(
    parameter int DEPTH = 256   // must match bram_circular_buffer DEPTH
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    // ADCS state inputs
    input  logic [31:0] q_est,        // packed quaternion estimate (4×8-bit)
    input  logic [23:0] omega,        // packed angular rate        (3×8-bit)
    input  logic [2:0]  adcs_mode,
    input  logic [7:0]  fault_flags,
    input  logic        health_ok,

    // BRAM write interface → connect directly to bram_circular_buffer Port A
    output logic [7:0]  bram_wr_addr,
    output logic [95:0] bram_wr_data,
    output logic        bram_wr_en,

    // Current write pointer (for debug / telemetry status)
    output logic [7:0]  wr_ptr
);

    // -------------------------------------------------------------------------
    // Circular write pointer - 8-bit unsigned, wraps naturally at 2^8 = 256
    // (matches DEPTH default of 256; no explicit modulo needed)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            wr_ptr <= '0;
        else if (ce_100hz)
            wr_ptr <= wr_ptr + 1;   // wraps naturally at 256
    end

    // -------------------------------------------------------------------------
    // Frame packer (combinatorial)
    // -------------------------------------------------------------------------
    always_comb begin
        bram_wr_data = { q_est,          // [95:64]
                         omega,          // [63:40]
                         adcs_mode,      // [39:37]
                         fault_flags,    // [36:29]
                         health_ok,      // [28]
                         wr_ptr,         // [27:20]
                         20'b0 };        // [19:0]  padding
        bram_wr_addr = wr_ptr;
        bram_wr_en   = ce_100hz;
    end

endmodule