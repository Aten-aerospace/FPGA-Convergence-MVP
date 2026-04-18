// =============================================================================
// Module: clk_manager (CS12 clock enable generator)
// Subsystem: CS12 - System Integration
// Description: Accepts a 100 MHz master clock and generates synchronous
//              clock-enable (CE) pulses for all subsystem rate domains.
//
//   sys_clk  = clk passthrough (combinational; no gating)
//   ce_1hz   : one pulse every 100_000_000 cycles  (1 Hz)
//   ce_100hz : one pulse every   1_000_000 cycles  (100 Hz)
//   ce_1khz  : one pulse every     100_000 cycles  (1 kHz)
//
//   All CE signals are single-cycle wide, synchronous to the rising edge of
//   clk.  No PLLs or clock primitives are instantiated — fully portable RTL.
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module clk_manager #(
    parameter int CLK_HZ   = 100_000_000,
    parameter int HZ_1HZ   = 1,
    parameter int HZ_100HZ = 100,
    parameter int HZ_1KHZ  = 1_000
)(
    input  logic clk,
    input  logic rst_n,

    // Passthrough
    output logic sys_clk,

    // Clock enables
    output logic ce_1hz,
    output logic ce_100hz,
    output logic ce_1khz
);

    // -------------------------------------------------------------------------
    // Derived counter limits (evaluated at elaboration time)
    // -------------------------------------------------------------------------
    localparam int CNT_1HZ   = CLK_HZ / HZ_1HZ   - 1;   // 99_999_999
    localparam int CNT_100HZ = CLK_HZ / HZ_100HZ  - 1;   //    999_999
    localparam int CNT_1KHZ  = CLK_HZ / HZ_1KHZ   - 1;   //     99_999

    localparam int W_1HZ   = $clog2(CNT_1HZ   + 1);
    localparam int W_100HZ = $clog2(CNT_100HZ + 1);
    localparam int W_1KHZ  = $clog2(CNT_1KHZ  + 1);

    // -------------------------------------------------------------------------
    // Counters
    // -------------------------------------------------------------------------
    logic [W_1HZ-1:0]   cnt_1hz;
    logic [W_100HZ-1:0] cnt_100hz;
    logic [W_1KHZ-1:0]  cnt_1khz;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_1hz   <= '0;
            cnt_100hz <= '0;
            cnt_1khz  <= '0;
            ce_1hz    <= 1'b0;
            ce_100hz  <= 1'b0;
            ce_1khz   <= 1'b0;
        end else begin
            // Default: de-assert all CEs
            ce_1hz   <= 1'b0;
            ce_100hz <= 1'b0;
            ce_1khz  <= 1'b0;

            // 1 kHz
            if (cnt_1khz == W_1KHZ'(CNT_1KHZ)) begin
                cnt_1khz <= '0;
                ce_1khz  <= 1'b1;
            end else begin
                cnt_1khz <= cnt_1khz + 1;
            end

            // 100 Hz
            if (cnt_100hz == W_100HZ'(CNT_100HZ)) begin
                cnt_100hz <= '0;
                ce_100hz  <= 1'b1;
            end else begin
                cnt_100hz <= cnt_100hz + 1;
            end

            // 1 Hz
            if (cnt_1hz == W_1HZ'(CNT_1HZ)) begin
                cnt_1hz <= '0;
                ce_1hz  <= 1'b1;
            end else begin
                cnt_1hz <= cnt_1hz + 1;
            end
        end
    end

    // -------------------------------------------------------------------------
    // sys_clk passthrough (combinational — connects directly to clk)
    // -------------------------------------------------------------------------
    assign sys_clk = clk;

endmodule
