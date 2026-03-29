`timescale 1ns / 1ps

module clk_divider #(
    parameter integer DIVIDE_BY = 2  // Must be >=2, even for exact 50% duty
)(
    input  logic clk_in,
    input  logic rst_n,
    output logic clk_out
);

    // ─────────────────────────────────────────────────────────────
    // Local parameters
    // ─────────────────────────────────────────────────────────────
    localparam integer HALF_DIV = DIVIDE_BY / 2;
    localparam integer CNT_W = $clog2(HALF_DIV);

    // ─────────────────────────────────────────────────────────────
    // Internal signals
    // ─────────────────────────────────────────────────────────────
    logic [CNT_W-1:0] counter;

    // ─────────────────────────────────────────────────────────────
    // Clock divider logic
    // ─────────────────────────────────────────────────────────────
    always_ff @(posedge clk_in or negedge rst_n) begin
        if (!rst_n) begin
            counter <= '0;
            clk_out <= 1'b0;
        end else begin
            if (counter == HALF_DIV - 1) begin
                counter <= '0;
                clk_out <= ~clk_out; // toggle output
            end else begin
                counter <= counter + 1'b1;
            end
        end
    end

    // ─────────────────────────────────────────────────────────────
    // Assertions (synthesis-safe if disabled)
    // ─────────────────────────────────────────────────────────────
`ifdef SIM
    initial begin
        if (DIVIDE_BY < 2) begin
            $error("DIVIDE_BY must be >= 2");
        end
        if (DIVIDE_BY % 2 != 0) begin
            $warning("DIVIDE_BY is odd → duty cycle will NOT be exactly 50%%");
        end
    end
`endif

endmodule
