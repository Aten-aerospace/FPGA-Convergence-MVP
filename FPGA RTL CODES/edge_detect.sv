`timescale 1ns/1ps

module edge_detect #(
    parameter string EDGE_TYPE = "RISING" // "RISING", "FALLING", "BOTH"
)(
    input  logic clk,
    input  logic rst_n,
    input  logic signal_in,
    output logic edge_pulse
);

    // ─────────────────────────────────────────────────────────────
    // Previous state register
    // ─────────────────────────────────────────────────────────────
    logic signal_d;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            signal_d <= 1'b0;
        else
            signal_d <= signal_in;
    end

    // ─────────────────────────────────────────────────────────────
    // Edge detection logic (combinational)
    // ─────────────────────────────────────────────────────────────
    logic rising_edge;
    logic falling_edge;

    assign rising_edge  =  signal_in & ~signal_d; // 0 → 1
    assign falling_edge = ~signal_in &  signal_d; // 1 → 0

    // ─────────────────────────────────────────────────────────────
    // Select edge type
    // ─────────────────────────────────────────────────────────────
    always_comb begin
        edge_pulse = 1'b0;

        case (EDGE_TYPE)
            "RISING":  edge_pulse = rising_edge;
            "FALLING": edge_pulse = falling_edge;
            "BOTH":    edge_pulse = rising_edge | falling_edge;
            default:   edge_pulse = 1'b0;
        endcase
    end

endmodule
