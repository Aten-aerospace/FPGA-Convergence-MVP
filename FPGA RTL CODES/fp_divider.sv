`timescale 1ns / 1ps

module fp_divider #(
    parameter DATA_W = 32
)(
    input  logic clk,
    input  logic rst_n,

    input  logic start,
    input  logic signed [DATA_W-1:0] dividend,
    input  logic signed [DATA_W-1:0] divisor,

    output logic signed [DATA_W-1:0] quotient,
    output logic done
);

    // ─────────────────────────────────────────────
    // State machine
    // ─────────────────────────────────────────────
    typedef enum logic [1:0] {
        IDLE,
        RUN,
        DONE
    } state_t;

    state_t state;

    // ─────────────────────────────────────────────
    // Internal signals
    // ─────────────────────────────────────────────
    logic [5:0] count;

    logic signed [DATA_W-1:0] dividend_abs;
    logic signed [DATA_W-1:0] divisor_abs;
    logic sign_q;

    logic signed [DATA_W:0] remainder, remainder_next;
    logic [DATA_W-1:0] quotient_reg, quotient_next;

    // ─────────────────────────────────────────────
    // Absolute value + sign
    // ─────────────────────────────────────────────
    always_comb begin
        dividend_abs = dividend[DATA_W-1] ? -dividend : dividend;
        divisor_abs  = divisor [DATA_W-1] ? -divisor  : divisor;
        sign_q       = dividend[DATA_W-1] ^ divisor[DATA_W-1];
    end

    // ─────────────────────────────────────────────
    // Main FSM
    // ─────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= IDLE;
            count        <= '0;
            remainder    <= '0;
            quotient_reg <= '0;
            quotient     <= '0;
            done         <= 1'b0;
        end else begin
            case (state)

            // ───────────── IDLE
            IDLE: begin
                done <= 1'b0;
                if (start) begin
                    remainder    <= '0;
                    quotient_reg <= dividend_abs;
                    count        <= DATA_W;
                    state        <= RUN;
                end
            end

            // ───────────── RUN (non-restoring division)
            RUN: begin
                // STEP 1: shift left
                remainder_next = {remainder[DATA_W-1:0], quotient_reg[DATA_W-1]};
                quotient_next  = {quotient_reg[DATA_W-2:0], 1'b0};

                // STEP 2: add/subtract
                if (remainder_next[DATA_W]) begin
                    remainder_next = remainder_next + divisor_abs;
                    quotient_next[0] = 1'b0;
                end else begin
                    remainder_next = remainder_next - divisor_abs;
                    quotient_next[0] = 1'b1;
                end

                // STEP 3: register update
                remainder    <= remainder_next;
                quotient_reg <= quotient_next;

                count <= count - 1;

                if (count == 1) begin
                    state <= DONE;
                end
            end

            // ───────────── DONE
            DONE: begin
                // final correction
                if (remainder[DATA_W]) begin
                    remainder <= remainder + divisor_abs;
                end

                // apply sign
                quotient <= sign_q ? -quotient_reg : quotient_reg;

                done  <= 1'b1;
                state <= IDLE;
            end

            endcase
        end
    end

endmodule

