// =============================================================================
// Module: orbit_state_manager (CS9 mission-elapsed-time and epoch tracker)
// Subsystem: CS9 - Orbit Propagator
// Description: Maintains a free-running mission elapsed time (MET) counter
//   incremented once per ce_1hz tick.  A loadable initial value allows the
//   counter to be synchronised to an on-board time source.  The epoch
//   (Modified Julian Date) is separately latchable so downstream modules
//   always have access to the reference epoch associated with the current TLE.
//
//   met_counter   : seconds since MET epoch, wraps at 2^32 (≈ 136 years)
//   epoch_tracked : MJD of current TLE epoch, Q15.16 (matches tle_parser output)
// =============================================================================
`timescale 1ns/1ps

module orbit_state_manager (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_1hz,          // 1-Hz enable pulse

    // MET load interface
    input  logic [31:0] met_load_value,  // value to preload into counter
    input  logic        met_write,       // strobe: latch met_load_value

    // Epoch interface
    input  logic [31:0] epoch_mjd,       // MJD Q15.16 from tle_parser
    input  logic        epoch_write,     // strobe: latch epoch_mjd

    output logic [31:0] met_counter,     // free-running MET (seconds)
    output logic [31:0] epoch_tracked    // most-recently latched epoch MJD
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            met_counter   <= '0;
            epoch_tracked <= '0;
        end else begin
            // MET counter: load takes priority over increment
            if (met_write)
                met_counter <= met_load_value;
            else if (ce_1hz)
                met_counter <= met_counter + 32'd1;

            // Epoch latch
            if (epoch_write)
                epoch_tracked <= epoch_mjd;
        end
    end

endmodule