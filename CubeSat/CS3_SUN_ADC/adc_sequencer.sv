// =============================================================================
// Module: adc_sequencer
// Subsystem: CS3 - Sun Sensor ADC Interface
// Description: SPI sequencer for ADS7952-compatible 4-channel 12-bit ADC.
//              Sequences all four photodiode channels sequentially, issuing a
//              16-bit SPI frame per channel (manual mode). SCLK is provided
//              externally by spi_mux_controller (shared with IMU SPI bus).
//              Bus access is arbitrated via spi_req / spi_grant handshake.
//              Result is presented as four 12-bit parallel outputs with a
//              single-cycle valid strobe.
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module adc_sequencer #(
    parameter int CLK_HZ  = 100_000_000,   // system clock Hz (for timeout only)
    parameter int NUM_CH  = 4,
    parameter int ADC_BITS = 12
)(
    input  logic        clk,       // clk_100mhz
    input  logic        rst_n,

    // Trigger: one pulse starts a full 4-channel acquisition
    input  logic        adc_trigger,

    // Shared SPI bus interface - SCLK driven by spi_mux_controller
    input  logic        spi_sclk,        // external SCLK from mux controller
    output logic        spi_mosi,        // MOSI to ADC
    input  logic        spi_miso,        // MISO from ADC
    output logic        spi_cs_adc_n,    // ADC chip select (active low)

    // Bus arbitration
    output logic        spi_req,         // request SPI bus
    input  logic        spi_grant,       // grant from spi_mux_controller

    // Parallel 12-bit ADC channel outputs
    output logic [11:0] adc_ch [0:3],

    // Status
    output logic        adc_valid,   // one-cycle strobe: all 4 channels updated
    output logic        adc_busy,
    output logic        adc_fault    // SPI timeout or grant timeout
);

    // =========================================================================
    // Timeout counter (system-clock cycles)
    // =========================================================================
    localparam int TIMEOUT_CYCLES = CLK_HZ / 100; // 10 ms at 100 MHz

    // =========================================================================
    // SCLK edge detection (spi_sclk is in the same clk_100mhz domain)
    // =========================================================================
    logic sclk_prev;
    logic sclk_rise, sclk_fall;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) sclk_prev <= 1'b0;
        else        sclk_prev <= spi_sclk;
    end

    assign sclk_rise = spi_sclk  & ~sclk_prev;
    assign sclk_fall = ~spi_sclk &  sclk_prev;

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [3:0] {
        S_IDLE,
        S_WAIT_GRANT,
        S_CS_ASSERT,
        S_SHIFT,
        S_CS_DEASSERT,
        S_STORE,
        S_NEXT_CH,
        S_DONE,
        S_FAULT
    } state_t;

    state_t state;

    // =========================================================================
    // Datapath registers
    // =========================================================================
    logic [1:0]  ch_idx;
    logic [4:0]  bit_cnt;
    logic [15:0] tx_shift;
    logic [15:0] rx_shift;
    logic [11:0] adc_buf [0:3];

    logic [$clog2(TIMEOUT_CYCLES)-1:0] timeout_cnt;

    // ADS7952 manual-mode command:
    // [15:12]=0001 (manual mode), [11:10]=2'b00, [9:8]=channel[1:0], [7:0]=8'h00
    function automatic logic [15:0] build_cmd(input logic [1:0] ch);
        return {4'b0001, 2'b00, ch, 8'h00};
    endfunction

    // MSB of ADS7952 manual-mode command is always 0 ({4'b0001,...}[15] == 0).
    // Declared as a localparam to avoid unsupported bit-select on function return.
    localparam logic CMD_MSB = 1'b0;

    // Last valid channel index (2-bit); avoids unsupported part-select on constant.
    localparam logic [1:0] LAST_CH = logic'(NUM_CH - 1);

    // =========================================================================
    // FSM + datapath
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            spi_cs_adc_n <= 1'b1;
            spi_mosi     <= 1'b0;
            spi_req      <= 1'b0;
            ch_idx       <= '0;
            bit_cnt      <= '0;
            tx_shift     <= '0;
            rx_shift     <= '0;
            adc_valid    <= 1'b0;
            adc_busy     <= 1'b0;
            adc_fault    <= 1'b0;
            timeout_cnt  <= '0;
            for (int i = 0; i < 4; i++) adc_buf[i] <= '0;
            for (int i = 0; i < 4; i++) adc_ch[i]  <= '0;
        end else begin
            adc_valid <= 1'b0; // default de-assert

            case (state)

                S_IDLE: begin
                    spi_cs_adc_n <= 1'b1;
                    spi_req      <= 1'b0;
                    adc_busy     <= 1'b0;
                    adc_fault    <= 1'b0;
                    timeout_cnt  <= '0;
                    if (adc_trigger) begin
                        ch_idx   <= '0;
                        adc_busy <= 1'b1;
                        spi_req  <= 1'b1;
                        state    <= S_WAIT_GRANT;
                    end
                end

                // Wait for spi_mux_controller to grant bus access
                S_WAIT_GRANT: begin
                    timeout_cnt <= timeout_cnt + 1'b1;
                    if (timeout_cnt == TIMEOUT_CYCLES[$clog2(TIMEOUT_CYCLES)-1:0] - 1) begin
                        spi_req <= 1'b0;
                        state   <= S_FAULT;
                    end else if (spi_grant) begin
                        timeout_cnt <= '0;
                        state       <= S_CS_ASSERT;
                    end
                end

                // Assert CS, pre-load TX word and drive MSB on MOSI
                S_CS_ASSERT: begin
                    spi_cs_adc_n <= 1'b0;
                    tx_shift     <= build_cmd(ch_idx);
                    spi_mosi     <= CMD_MSB; // MSB valid before first SCLK rise
                    bit_cnt      <= 5'd15;
                    rx_shift     <= '0;
                    state        <= S_SHIFT;
                end

                // Shift 16 bits: drive MOSI on falling edge, sample MISO on rising edge
                S_SHIFT: begin
                    timeout_cnt <= timeout_cnt + 1'b1;
                    if (timeout_cnt == TIMEOUT_CYCLES[$clog2(TIMEOUT_CYCLES)-1:0] - 1) begin
                        spi_cs_adc_n <= 1'b1;
                        spi_req      <= 1'b0;
                        state        <= S_FAULT;
                    end else begin
                        // Drive MOSI with next bit on falling edge
                        if (sclk_fall) begin
                            spi_mosi <= tx_shift[14];
                            tx_shift <= {tx_shift[13:0], 1'b0};
                        end
                        // Sample MISO on rising edge
                        if (sclk_rise) begin
                            rx_shift <= {rx_shift[14:0], spi_miso};
                            if (bit_cnt == '0)
                                state <= S_CS_DEASSERT;
                            else
                                bit_cnt <= bit_cnt - 1'b1;
                        end
                    end
                end

                S_CS_DEASSERT: begin
                    spi_cs_adc_n <= 1'b1;
                    timeout_cnt  <= '0;
                    state        <= S_STORE;
                end

                // ADS7952 response: [15:12]=channel addr, [11:0]=12-bit ADC data
                S_STORE: begin
                    adc_buf[ch_idx] <= rx_shift[11:0];
                    state           <= S_NEXT_CH;
                end

                S_NEXT_CH: begin
                    if (ch_idx == LAST_CH) begin
                        state <= S_DONE;
                    end else begin
                        ch_idx <= ch_idx + 1'b1;
                        state  <= S_CS_ASSERT; // retain bus grant for next channel
                    end
                end

                S_DONE: begin
                    for (int i = 0; i < 4; i++) adc_ch[i] <= adc_buf[i];
                    adc_valid <= 1'b1;
                    adc_busy  <= 1'b0;
                    spi_req   <= 1'b0; // release bus
                    state     <= S_IDLE;
                end

                S_FAULT: begin
                    spi_cs_adc_n <= 1'b1;
                    adc_fault    <= 1'b1;
                    adc_busy     <= 1'b0;
                    spi_req      <= 1'b0;
                    state        <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
