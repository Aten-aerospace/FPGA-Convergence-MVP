// =============================================================================
// Module: reset_controller
// Subsystem: CS12 - System Integration
// Description: Reset synchronizer and power-on-reset (POR) sequencer.
//              Synchronises the asynchronous external reset to the system
//              clock using a 2-FF metastability filter, then holds the
//              synchronised active-low reset asserted for POR_CYCLES after
//              the last deassertion of rst_ext_n.
//
//   por_done asserts once POR_CYCLES have elapsed and remains asserted until
//   rst_ext_n is reasserted.
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module reset_controller #(
    parameter int CLK_HZ    = 100_000_000,
    parameter int POR_CYCLES = 1000          // cycles to hold rst_sys_n low after POR
)(
    input  logic clk,
    input  logic rst_ext_n,   // asynchronous external active-low reset

    output logic rst_sys_n,   // synchronised active-low reset for all downstream
    output logic por_done     // asserts after POR_CYCLES; de-asserts on rst_ext_n
);

    // =========================================================================
    // 2-FF synchroniser
    // =========================================================================
    logic sync_ff1, sync_ff2;

    always_ff @(posedge clk or negedge rst_ext_n) begin
        if (!rst_ext_n) begin
            sync_ff1 <= 1'b0;
            sync_ff2 <= 1'b0;
        end else begin
            sync_ff1 <= 1'b1;
            sync_ff2 <= sync_ff1;
        end
    end

    // sync_ff2 is now metastability-safe synchronised release of reset

    // =========================================================================
    // POR counter
    // =========================================================================
    localparam int CNT_W = $clog2(POR_CYCLES + 2);

    logic [CNT_W-1:0] por_cnt;
    logic              por_done_r;

    always_ff @(posedge clk or negedge rst_ext_n) begin
        if (!rst_ext_n) begin
            por_cnt   <= '0;
            por_done_r <= 1'b0;
        end else begin
            if (!por_done_r) begin
                if (por_cnt == POR_CYCLES[CNT_W-1:0]) begin
                    por_done_r <= 1'b1;
                end else begin
                    por_cnt <= por_cnt + 1'b1;
                end
            end
        end
    end

    // =========================================================================
    // Output assignments
    // =========================================================================
    // rst_sys_n is low until both sync_ff2 is high AND por is done
    always_comb begin
        rst_sys_n = sync_ff2 & por_done_r;
        por_done  = por_done_r;
    end

endmodule
