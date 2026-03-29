// =============================================================================
// File: tick_gen.sv
// Description: Periodic single-cycle tick generator
// =============================================================================

`timescale 1ns/1ps

module tick_gen #(
    parameter int CLK_HZ  = 50_000_000,
    parameter int TICK_HZ = 100
)(
    input  logic clk,
    input  logic rst_n,
    output logic tick
);

    // -------------------------------------------------------------------------
    // Derived constants
    // -------------------------------------------------------------------------
    localparam int DIVISOR = CLK_HZ / TICK_HZ;

    // Safety check (compile-time)
    initial begin
        if (TICK_HZ == 0) begin
            $error("TICK_HZ must be > 0");
        end
        if (DIVISOR == 0) begin
            $error("CLK_HZ must be >= TICK_HZ");
        end
    end

    // Counter width
    localparam int CNT_W = $clog2(DIVISOR);

    logic [CNT_W-1:0] cnt;

    // -------------------------------------------------------------------------
    // Counter + tick generation
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt  <= '0;
            tick <= 1'b0;
        end else begin
            if (cnt == DIVISOR-1) begin
                cnt  <= '0;
                tick <= 1'b1;   // single-cycle pulse
            end else begin
                cnt  <= cnt + 1'b1;
                tick <= 1'b0;
            end
        end
    end

endmodule
