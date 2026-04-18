// =============================================================================
// Module: signal_monitor (CS10 ADC signal monitor)
// Subsystem: CS10 - Laser Pointing FSM
// Description: Samples ADC at 10 kHz, applies 16-sample rolling average,
//              outputs filtered signal at 100 Hz.
//
//   Algorithm:
//     - 16-sample ring buffer (FIFO)
//     - On each adc_valid strobe: add new sample, drop oldest
//     - Accumulator: sum = sum - old_sample + new_sample
//     - Every 100 ADC samples (every 10 ms): output sum >> 4 (÷16)
//     - Output: 12-bit unsigned Q12 format
//
//   Clock domain:
//     - adc_valid strobe at 10 kHz generated internally from clk divider
//     - filter_valid output registered on clk, strobed at ce_100hz rate
//
// Requirement: CS-LSR-010 (Signal Strength Monitoring - 10kHz, 12-bit, rolling avg)
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module signal_monitor #(
    parameter int CLK_HZ     = 100_000_000,  // system clock frequency
    parameter int ADC_HZ     = 10_000,       // ADC sampling rate
    parameter int AVG_DEPTH  = 16            // rolling average window
)(
    input  logic        clk,
    input  logic        rst_n,

    input  logic [11:0] adc_data,            // raw ADC input (12-bit, 0-4095)
    input  logic        adc_valid,           // external strobe at ADC_HZ, or tie low to use internal
    input  logic        ce_100hz,            // 100 Hz output strobe

    output logic [11:0] signal_strength_filtered,  // rolling average (12-bit)
    output logic        filter_valid               // strobes @ 100 Hz
);

    // =========================================================================
    // Internal ADC clock divider (generates 10 kHz strobe if adc_valid not used)
    // =========================================================================
    localparam int ADC_DIV = CLK_HZ / ADC_HZ;   // 100M / 10k = 10000
    localparam int DIV_W   = $clog2(ADC_DIV + 1);

    logic [DIV_W-1:0] adc_div_cnt;
    logic             adc_strobe;   // internal strobe

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            adc_div_cnt <= '0;
            adc_strobe  <= 1'b0;
        end else begin
            adc_strobe <= 1'b0;
            if (adc_div_cnt == DIV_W'(ADC_DIV - 1)) begin
                adc_div_cnt <= '0;
                adc_strobe  <= 1'b1;
            end else begin
                adc_div_cnt <= adc_div_cnt + 1;
            end
        end
    end

    // Use external strobe if asserted, otherwise use internal divider strobe
    logic sample_strobe;
    assign sample_strobe = adc_valid | adc_strobe;

    // =========================================================================
    // 16-sample ring buffer and rolling accumulator
    // =========================================================================
    localparam int ADDR_W  = $clog2(AVG_DEPTH);
    localparam int SUM_W   = 12 + ADDR_W;   // 12 + 4 = 16 bits for AVG_DEPTH=16

    logic [11:0]    ring_buf [0:AVG_DEPTH-1];
    logic [ADDR_W-1:0] wr_idx;
    logic [SUM_W-1:0]  acc;          // rolling sum
    logic               buf_full;    // asserts after first full pass
    logic [ADDR_W:0]    sample_cnt;  // saturation counter up to AVG_DEPTH

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < AVG_DEPTH; i++)
                ring_buf[i] <= 12'd0;
            wr_idx          <= '0;
            acc             <= '0;
            buf_full        <= 1'b0;
            sample_cnt      <= '0;
        end else if (sample_strobe) begin
            // subtract oldest sample, add new sample
            acc     <= acc - {4'd0, ring_buf[wr_idx]} + {4'd0, adc_data};
            ring_buf[wr_idx] <= adc_data;
            wr_idx  <= wr_idx + 1;

            if (!buf_full) begin
                if (sample_cnt == ADDR_W'(AVG_DEPTH - 1))
                    buf_full <= 1'b1;
                sample_cnt <= sample_cnt + 1;
            end
        end
    end

    // =========================================================================
    // Output registration @ 100 Hz
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            signal_strength_filtered <= 12'd0;
            filter_valid             <= 1'b0;
        end else begin
            filter_valid <= 1'b0;
            if (ce_100hz) begin
                // Divide sum by AVG_DEPTH (right-shift by log2(AVG_DEPTH))
                signal_strength_filtered <= acc[SUM_W-1:ADDR_W];
                filter_valid             <= 1'b1;
            end
        end
    end

endmodule