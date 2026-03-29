`timescale 1ns / 1ps


module spi_master #(
    parameter integer CLK_DIV = 5,   // sclk = clk/(2*CLK_DIV)
    parameter integer DATA_W  = 16,
    parameter integer CPOL    = 0,
    parameter integer CPHA    = 0
)(
    input  logic              clk,
    input  logic              rst_n,

    // TX interface
    input  logic [DATA_W-1:0] tx_data,
    input  logic              tx_valid,
    output logic              tx_ready,

    // RX interface
    output logic [DATA_W-1:0] rx_data,
    output logic              rx_valid,

    // SPI signals
    output logic              sclk,
    output logic              mosi,
    output logic              cs_n,
    input  logic              miso
);

    // =========================================================================
    // Internal signals
    // =========================================================================
    typedef enum logic [1:0] {
        IDLE,
        TRANSFER,
        DONE
    } state_t;

    state_t state, next_state;

    logic [$clog2(CLK_DIV)-1:0] clk_cnt;
    logic                       sclk_en;
    logic                       sclk_int;
    logic                       sclk_edge;

    logic [$clog2(DATA_W):0]    bit_cnt;

    logic [DATA_W-1:0]          tx_shift;
    logic [DATA_W-1:0]          rx_shift;

    logic                       sample_edge;
    logic                       shift_edge;

    // =========================================================================
    // Clock Divider (Generates SCLK enable)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= '0;
            sclk_en <= 1'b0;
        end else begin
            if (state == TRANSFER) begin
                if (clk_cnt == CLK_DIV-1) begin
                    clk_cnt <= '0;
                    sclk_en <= 1'b1;
                end else begin
                    clk_cnt <= clk_cnt + 1'b1;
                    sclk_en <= 1'b0;
                end
            end else begin
                clk_cnt <= '0;
                sclk_en <= 1'b0;
            end
        end
    end

    // =========================================================================
    // SCLK generation
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sclk_int <= CPOL;
        else if (state == TRANSFER && sclk_en)
            sclk_int <= ~sclk_int;
        else if (state == IDLE)
            sclk_int <= CPOL;
    end

    assign sclk = sclk_int;

    // Detect edge
    assign sclk_edge = sclk_en;

    // =========================================================================
    // CPOL / CPHA edge selection
    // =========================================================================
    always_comb begin
        // Default
        sample_edge = 1'b0;
        shift_edge  = 1'b0;

        if (CPHA == 0) begin
            // Sample on first edge
            if (sclk_edge && (sclk_int != CPOL))
                sample_edge = 1'b1;

            if (sclk_edge && (sclk_int == CPOL))
                shift_edge = 1'b1;

        end else begin
            // Sample on second edge
            if (sclk_edge && (sclk_int == CPOL))
                sample_edge = 1'b1;

            if (sclk_edge && (sclk_int != CPOL))
                shift_edge = 1'b1;
        end
    end

    // =========================================================================
    // FSM
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            state <= IDLE;
        else
            state <= next_state;
    end

    always_comb begin
        next_state = state;

        case (state)
            IDLE: begin
                if (tx_valid)
                    next_state = TRANSFER;
            end

            TRANSFER: begin
                if ((bit_cnt == DATA_W) && sample_edge)
                    next_state = DONE;
            end

            DONE: begin
                next_state = IDLE;
            end
        endcase
    end

    // =========================================================================
    // Shift Register & Bit Counter
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_shift <= '0;
            rx_shift <= '0;
            bit_cnt  <= '0;
        end else begin
            case (state)
                IDLE: begin
                    if (tx_valid) begin
                        tx_shift <= tx_data;
                        rx_shift <= '0;
                        bit_cnt  <= 0;
                    end
                end

                TRANSFER: begin
                    // Shift out (MSB first)
                    if (shift_edge) begin
                        tx_shift <= {tx_shift[DATA_W-2:0], 1'b0};
                    end

                    // Sample MISO
                    if (sample_edge) begin
                        rx_shift <= {rx_shift[DATA_W-2:0], miso};
                        bit_cnt  <= bit_cnt + 1'b1;
                    end
                end

                default: ;
            endcase
        end
    end

    // =========================================================================
    // MOSI output (MSB first)
    // =========================================================================
    assign mosi = tx_shift[DATA_W-1];

    // =========================================================================
    // Chip Select (active low during transfer)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cs_n <= 1'b1;
        else begin
            case (state)
                IDLE:     cs_n <= 1'b1;
                TRANSFER: cs_n <= 1'b0;
                DONE:     cs_n <= 1'b1;
            endcase
        end
    end

    // =========================================================================
    // Output control
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_data  <= '0;
            rx_valid <= 1'b0;
        end else begin
            rx_valid <= 1'b0;

            if (state == DONE) begin
                rx_data  <= rx_shift;
                rx_valid <= 1'b1; // done pulse
            end
        end
    end

    assign tx_ready = (state == IDLE);

endmodule

