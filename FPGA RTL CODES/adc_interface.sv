`timescale 1ns/1ps

module adc_interface #(
    parameter integer CHANNELS = 8,
    parameter integer CLK_DIV  = 50   // SCLK = clk/(2*CLK_DIV)
)(
    input  logic clk,
    input  logic rst_n,

    input  logic sample_trigger,

    output logic [11:0] adc_data [CHANNELS-1:0],
    output logic data_valid,

    // SPI
    output logic sclk,
    output logic mosi,
    output logic cs_n,
    input  logic miso
);

    // ─────────────────────────────────────────────
    // PARAMETERS
    // ─────────────────────────────────────────────
    localparam integer TOTAL_BITS = 24;

    // FSM
    typedef enum logic [2:0] {
        IDLE,
        START,
        TRANSFER,
        NEXT_CH,
        DONE
    } state_t;

    state_t state, state_next;

    // ─────────────────────────────────────────────
    // CLOCK DIVIDER (generate SCLK)
    // ─────────────────────────────────────────────
    logic [$clog2(CLK_DIV)-1:0] clk_cnt;
    logic sclk_en;
    logic sclk_int;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt  <= 0;
            sclk_int <= 0;
        end else if (sclk_en) begin
            if (clk_cnt == CLK_DIV-1) begin
                clk_cnt  <= 0;
                sclk_int <= ~sclk_int;
            end else begin
                clk_cnt <= clk_cnt + 1;
            end
        end else begin
            clk_cnt  <= 0;
            sclk_int <= 0;
        end
    end

    assign sclk = sclk_int;

    // rising edge detect
    logic sclk_d;
    always_ff @(posedge clk) sclk_d <= sclk_int;

    wire sclk_rise = (sclk_int & ~sclk_d);
    wire sclk_fall = (~sclk_int & sclk_d);

    // ─────────────────────────────────────────────
    // CONTROL REGISTERS
    // ─────────────────────────────────────────────
    logic [4:0]  bit_cnt;
    logic [2:0]  ch_cnt;
    logic [23:0] shift_reg;
    logic [11:0] sample_reg;

    // ─────────────────────────────────────────────
    // FSM SEQUENTIAL
    // ─────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= state_next;
    end

    // ─────────────────────────────────────────────
    // FSM NEXT STATE
    // ─────────────────────────────────────────────
    always_comb begin
        state_next = state;

        case (state)
            IDLE:
                if (sample_trigger)
                    state_next = START;

            START:
                state_next = TRANSFER;

            TRANSFER:
                if (bit_cnt == TOTAL_BITS && sclk_rise)
                    state_next = NEXT_CH;

            NEXT_CH:
                if (ch_cnt == CHANNELS-1)
                    state_next = DONE;
                else
                    state_next = START;

            DONE:
                state_next = IDLE;
        endcase
    end

    // ─────────────────────────────────────────────
    // MAIN LOGIC
    // ─────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cs_n       <= 1;
            mosi       <= 0;
            sclk_en    <= 0;
            bit_cnt    <= 0;
            ch_cnt     <= 0;
            data_valid <= 0;
            shift_reg  <= 0;
        end else begin
            data_valid <= 0;

            case (state)

                // ───────────── IDLE ─────────────
                IDLE: begin
                    cs_n    <= 1;
                    sclk_en <= 0;
                    bit_cnt <= 0;
                    ch_cnt  <= 0;
                end

                // ───────────── START ─────────────
                START: begin
                    cs_n    <= 0;
                    sclk_en <= 1;
                    bit_cnt <= 0;

                    // MCP3208 command: 1 (start), 1 (single), channel
                    shift_reg <= {
                        1'b1,           // start
                        1'b1,           // single-ended
                        ch_cnt[2:0],    // channel
                        19'b0           // padding
                    };
                end

                // ───────────── TRANSFER ─────────────
                TRANSFER: begin
                    // MOSI on falling edge
                    if (sclk_fall) begin
                        mosi <= shift_reg[23];
                        shift_reg <= {shift_reg[22:0], 1'b0};
                    end

                    // MISO sampling on rising edge
                    if (sclk_rise) begin
                        bit_cnt <= bit_cnt + 1;

                        // Capture 12-bit result (bits 13-24)
                        if (bit_cnt >= 12 && bit_cnt < 24) begin
                            sample_reg <= {sample_reg[10:0], miso};
                        end
                    end
                end

                // ───────────── NEXT CHANNEL ─────────────
                NEXT_CH: begin
                    sclk_en <= 0;
                    cs_n    <= 1;

                    adc_data[ch_cnt] <= sample_reg;
                    ch_cnt <= ch_cnt + 1;
                end

                // ───────────── DONE ─────────────
                DONE: begin
                    data_valid <= 1;
                end

            endcase
        end
    end

endmodule
