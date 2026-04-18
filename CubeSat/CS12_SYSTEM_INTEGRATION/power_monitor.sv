// =============================================================================
// Module: power_monitor
// Subsystem: CS12 - System Integration
// Description: Monitors CubeSat power bus voltage and current.
//              Compares measured vbus_mv and curr_ma against configurable
//              thresholds and outputs a power_state indicator.
//
//   power_state[1:0] encoding:
//     2'd0  NORMAL  : vbus and current within limits
//     2'd1  LOW     : vbus below VBUS_LOW_MV
//     2'd2  HIGH    : vbus above VBUS_HIGH_MV
//     2'd3  FAULT   : current above CURR_MAX_MA (over-current)
//
//   power_fault asserts in any non-NORMAL state.
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module power_monitor #(
    parameter int VBUS_LOW_MV  = 4500,   // under-voltage threshold (mV)
    parameter int VBUS_HIGH_MV = 5500,   // over-voltage threshold (mV)
    parameter int CURR_MAX_MA  = 2000    // over-current threshold (mA)
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    input  logic [15:0] vbus_mv,   // power bus voltage in mV (unsigned)
    input  logic [15:0] curr_ma,   // supply current in mA (unsigned)

    output logic        vbus_ok,
    output logic        curr_ok,
    output logic        power_ok,
    output logic        power_fault,
    output logic [1:0]  power_state  // NORMAL=0, LOW=1, HIGH=2, FAULT=3
);

    // =========================================================================
    // Threshold comparisons (combinational)
    // =========================================================================
    logic vbus_low, vbus_high, curr_over;

    always_comb begin
        vbus_low  = (vbus_mv < VBUS_LOW_MV[15:0]);
        vbus_high = (vbus_mv > VBUS_HIGH_MV[15:0]);
        curr_over = (curr_ma > CURR_MAX_MA[15:0]);
        vbus_ok   = ~vbus_low & ~vbus_high;
        curr_ok   = ~curr_over;
    end

    // =========================================================================
    // Power state machine (registered, samples each ce_100hz)
    // =========================================================================
    typedef enum logic [1:0] {
        NORMAL = 2'd0,
        LOW    = 2'd1,
        HIGH   = 2'd2,
        FAULT  = 2'd3
    } pwr_state_t;

    pwr_state_t state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= NORMAL;
        end else if (ce_100hz) begin
            if (curr_over) begin
                state <= FAULT;
            end else if (vbus_low) begin
                state <= LOW;
            end else if (vbus_high) begin
                state <= HIGH;
            end else begin
                state <= NORMAL;
            end
        end
    end

    // =========================================================================
    // Output assignments
    // =========================================================================
    always_comb begin
        power_state = state;
        power_fault = (state != NORMAL);
        power_ok    = ~power_fault;
    end

endmodule
