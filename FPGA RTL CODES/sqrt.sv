`timescale 1ns / 1ps


module sqrt #(
    parameter DATA_W = 32  // Q15.16
)(
    input  logic clk,
    input  logic rst_n,
    input  logic start,
    input  logic [DATA_W-1:0] x,   // input Q15.16

    output logic [DATA_W-1:0] sqrt_out,
    output logic done
);

    // ─────────────────────────────────────────────
    // Internal parameters
    // ─────────────────────────────────────────────
    localparam ITER = 4;
    localparam FRAC = 16;

    typedef enum logic [2:0] {
        IDLE,
        INIT,
        ITERATE,
        DONE
    } state_t;

    state_t state;

    // Registers
    logic [DATA_W-1:0] N;
    logic [DATA_W-1:0] y;          // current estimate
    logic [DATA_W-1:0] y_next;
    logic [2:0] iter_cnt;

    // Intermediate signals
    logic [63:0] mult_full;
    logic [DATA_W-1:0] div_result;
    logic [DATA_W-1:0] reciprocal;

    // ─────────────────────────────────────────────
    // Initial estimate (critical for convergence)
    // Simple: y0 = x >> 1 + 1
    // ─────────────────────────────────────────────
    function automatic [DATA_W-1:0] init_guess(input [DATA_W-1:0] val);
        begin
            if (val == 0)
                init_guess = 0;
            else
                init_guess = (val >> 1) + (1 << FRAC); // ~x/2 + 1
        end
    endfunction

    // ─────────────────────────────────────────────
    // Reciprocal using 1 iteration NR:
    // r = r * (2 - y*r)
    // good enough for sqrt loop
    // ─────────────────────────────────────────────
    function automatic [DATA_W-1:0] reciprocal_nr(input [DATA_W-1:0] val);
        logic [DATA_W-1:0] r;
        logic [63:0] t;
        begin
            // crude initial guess
            r = 32'h00010000; // 1.0 in Q15.16

            // r = r*(2 - y*r)
            t = (val * r);                 // Q15.16 * Q15.16 = Q30.32
            t = t >> FRAC;                 // back to Q15.16

            t = (32'h00020000 - t);        // 2 - y*r

            t = (r * t);                  // multiply again
            reciprocal_nr = t >> FRAC;
        end
    endfunction

    // ─────────────────────────────────────────────
    // FSM
    // ─────────────────────────────────────────────
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= IDLE;
            done       <= 0;
            sqrt_out   <= 0;
            iter_cnt   <= 0;
            y          <= 0;
            N          <= 0;
        end else begin
            case (state)

                IDLE: begin
                    done <= 0;
                    if (start) begin
                        N        <= x;
                        y        <= init_guess(x);
                        iter_cnt <= 0;
                        state    <= INIT;
                    end
                end

                INIT: begin
                    state <= ITERATE;
                end

                ITERATE: begin
                    // Compute reciprocal of y
                    reciprocal = reciprocal_nr(y);

                    // N / y = N * (1/y)
                    mult_full  = N * reciprocal;
                    div_result = mult_full >> FRAC;

                    // y_next = 0.5*(y + N/y)
                    y_next = (y + div_result) >> 1;

                    y <= y_next;
                    iter_cnt <= iter_cnt + 1;

                    if (iter_cnt == ITER-1)
                        state <= DONE;
                end

                DONE: begin
                    sqrt_out <= y;
                    done     <= 1;
                    state    <= IDLE;
                end

            endcase
        end
    end

endmodule

