// =============================================================================
// Synchronizer (2-FF / 3-FF) for CDC
// ----------------------------------------------------------------------------
// - Metastability mitigation using FF chain
// - STAGES = 2 (standard) or 3 (higher MTBF)
// - WIDTH supports vector synchronization
// - No reset (recommended for CDC reliability)
// - Use ASYNC_REG attribute for Xilinx tools
// =============================================================================

`timescale 1ns / 1ps

module synchronizer #(
    parameter int STAGES = 2,   // 2 or 3
    parameter int WIDTH  = 1
)(
    input  logic                   dst_clk,
    input  logic [WIDTH-1:0]       async_in,
    output logic [WIDTH-1:0]       sync_out
);

    // -------------------------------------------------------------------------
    // Compile-time check
    // -------------------------------------------------------------------------
    initial begin
        if (STAGES < 2 || STAGES > 3) begin
            $error("synchronizer: STAGES must be 2 or 3");
        end
    end

    // -------------------------------------------------------------------------
    // Synchronizer registers
    // -------------------------------------------------------------------------
    (* ASYNC_REG = "TRUE" *) logic [WIDTH-1:0] sync_ff [STAGES-1:0];

    // -------------------------------------------------------------------------
    // FF chain
    // -------------------------------------------------------------------------
    always_ff @(posedge dst_clk) begin
        sync_ff[0] <= async_in;

        for (int i = 1; i < STAGES; i++) begin
            sync_ff[i] <= sync_ff[i-1];
        end
    end

    // -------------------------------------------------------------------------
    // Output
    // -------------------------------------------------------------------------
    assign sync_out = sync_ff[STAGES-1];

endmodule
