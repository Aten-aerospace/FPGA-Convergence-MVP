// =============================================================================
// Module: sun_presence_detector
// Subsystem: CS3 - Sun Sensor ADC Interface
// Description: Detects whether the sun is in the sensor field of view by
//              comparing each ADC channel against SUN_THRESH.
//              sun_present asserts when at least one channel exceeds the
//              threshold.  sun_ch_mask is a per-channel bitmask of lit
//              channels.  dominant_ch is the index of the highest reading.
//              NUM_CH is parameterizable (default 4 for the CS3 4-channel spec).
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module sun_presence_detector #(
    parameter int NUM_CH     = 4,   // number of ADC channels
    parameter int SUN_THRESH = 409  // 10 % FS: 0.10 × 4095 ≈ 409 counts (12-bit)
)(
    input  logic        clk,
    input  logic        rst_n,

    input  logic [11:0] adc_ch [0:NUM_CH-1],
    input  logic        data_valid,

    output logic                              sun_present,
    output logic [NUM_CH-1:0]                 sun_ch_mask,
    output logic [$clog2(NUM_CH > 1 ? NUM_CH : 2)-1:0] dominant_ch
);

    // =========================================================================
    // Derived width for dominant_ch index
    // =========================================================================
    localparam int DOM_W = $clog2(NUM_CH > 1 ? NUM_CH : 2);

    // =========================================================================
    // Combinational: build mask and find dominant channel
    // =========================================================================
    logic [NUM_CH-1:0] mask_c;
    logic [DOM_W-1:0]  dom_ch_c;
    logic [11:0]       dom_val_c;

    always_comb begin
        mask_c    = '0;
        dom_ch_c  = '0;
        dom_val_c = 12'h0;

        for (int i = 0; i < NUM_CH; i++) begin
            if (adc_ch[i] > SUN_THRESH[11:0])
                mask_c[i] = 1'b1;
            if (adc_ch[i] > dom_val_c) begin
                dom_val_c = adc_ch[i];
                dom_ch_c  = DOM_W'(unsigned'(i));
            end
        end
    end

    // =========================================================================
    // Register outputs on data_valid strobe
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sun_present <= 1'b0;
            sun_ch_mask <= '0;
            dominant_ch <= '0;
        end else if (data_valid) begin
            sun_ch_mask <= mask_c;
            sun_present <= |mask_c;
            dominant_ch <= dom_ch_c;
        end
    end

endmodule