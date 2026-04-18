// =============================================================================
// Module: multi_satellite_tracker (CS9 relative-motion computer)
// Subsystem: CS9 - Orbit Propagator
// Description: Computes the relative position and velocity between any two
//   of three tracked satellites, and derives separation distance and closure
//   rate.
//
//   sat_id selects the pair:
//     2'b00 → sat1 vs sat2
//     2'b01 → sat1 vs sat3
//     2'b10 → sat2 vs sat3
//
//   Separation distance (sqrt.sv):
//     dr_int[i] = delta_r[i] >> 16       (integer km)
//     dr_sq     = Σ dr_int[i]²           (km²)
//     sqrt input = dr_sq (raw)
//       → sqrt_out = |Δr|/256 in Q15.16
//     separation_km = sqrt_out << 8       (Q15.16 km)
//
//   Closure rate (fp_divider.sv):
//     dot_rv = Σ delta_r[i] × delta_v[i] >> 32   (Q15.16)
//     closure_rate = -dot_rv / separation_km       (Q15.16 km/s)
//     Negative = approaching; exposed unsigned as absolute value.
// =============================================================================
`timescale 1ns/1ps

module multi_satellite_tracker (
    input  logic        clk,
    input  logic        rst_n,

    input  logic [31:0] sat1_pos [0:2],  // Q15.16 km
    input  logic [31:0] sat1_vel [0:2],  // Q15.16 km/s
    input  logic [31:0] sat2_pos [0:2],
    input  logic [31:0] sat2_vel [0:2],
    input  logic [31:0] sat3_pos [0:2],
    input  logic [31:0] sat3_vel [0:2],
    input  logic [1:0]  sat_id,          // 0=1v2, 1=1v3, 2=2v3
    input  logic        compute_valid,   // strobe: begin computation

    output logic [31:0] delta_r_eci [0:2],  // Q15.16 km
    output logic [31:0] delta_v_eci [0:2],  // Q15.16 km/s
    output logic [31:0] separation_km,      // Q15.16 km
    output logic [31:0] closure_rate_kmps,  // Q15.16 km/s (magnitude)
    output logic        relative_valid
);

    // =========================================================================
    // Mux: select satellite pair based on sat_id
    // =========================================================================
    logic [31:0] posA [0:2], posB [0:2];
    logic [31:0] velA [0:2], velB [0:2];

    always_comb begin
        case (sat_id)
            2'b00: begin
                for (int i=0;i<3;i++) posA[i] = sat1_pos[i];
                for (int i=0;i<3;i++) posB[i] = sat2_pos[i];
                for (int i=0;i<3;i++) velA[i] = sat1_vel[i];
                for (int i=0;i<3;i++) velB[i] = sat2_vel[i];
            end
            2'b01: begin
                for (int i=0;i<3;i++) posA[i] = sat1_pos[i];
                for (int i=0;i<3;i++) posB[i] = sat3_pos[i];
                for (int i=0;i<3;i++) velA[i] = sat1_vel[i];
                for (int i=0;i<3;i++) velB[i] = sat3_vel[i];
            end
            default: begin
                for (int i=0;i<3;i++) posA[i] = sat2_pos[i];
                for (int i=0;i<3;i++) posB[i] = sat3_pos[i];
                for (int i=0;i<3;i++) velA[i] = sat2_vel[i];
                for (int i=0;i<3;i++) velB[i] = sat3_vel[i];
            end
        endcase
    end

    // =========================================================================
    // Math library instances
    // =========================================================================
    logic        sqrt_start;
    logic [31:0] sqrt_in;
    logic [31:0] sqrt_out;
    logic        sqrt_done;

    sqrt u_sqrt (
        .clk(clk), .rst_n(rst_n),
        .start(sqrt_start), .x(sqrt_in),
        .sqrt_out(sqrt_out), .done(sqrt_done)
    );

    logic        div_start;
    logic signed [31:0] div_dividend, div_divisor, div_quotient;
    logic        div_done;

    fp_divider u_div (
        .clk(clk), .rst_n(rst_n),
        .start(div_start),
        .dividend(div_dividend), .divisor(div_divisor),
        .quotient(div_quotient), .done(div_done)
    );

    // =========================================================================
    // Internal registers
    // =========================================================================
    logic signed [31:0] r_dr [0:2]; // delta_r (Q15.16)
    logic signed [31:0] r_dv [0:2]; // delta_v (Q15.16)
    logic [31:0]        r_dr_int [0:2]; // integer-km parts
    logic [31:0]        dr_sq;          // Σ dr_int² (km²)
    logic signed [63:0] dot_acc;        // dot product accumulator (Q31.32)
    logic signed [31:0] dot_q1516;      // dot product in Q15.16
    logic [31:0]        sep_q;          // separation_km latch

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE,
        S_DIFF,
        S_SQRT_START,
        S_SQRT_WAIT,
        S_DOT,
        S_DIV_START,
        S_DIV_WAIT,
        S_DONE
    } state_t;

    state_t state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            relative_valid  <= 1'b0;
            separation_km   <= '0;
            closure_rate_kmps <= '0;
            sqrt_start      <= 1'b0;
            div_start       <= 1'b0;
            div_dividend    <= '0;
            div_divisor     <= '0;
            sqrt_in         <= '0;
            dot_acc         <= '0;
            dot_q1516       <= '0;
            sep_q           <= '0;
            for (int i=0;i<3;i++) begin
                r_dr[i]     <= '0;
                r_dv[i]     <= '0;
                delta_r_eci[i] <= '0;
                delta_v_eci[i] <= '0;
            end
        end else begin
            sqrt_start <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                S_IDLE: begin
                    relative_valid <= 1'b0;
                    if (compute_valid) state <= S_DIFF;
                end

                S_DIFF: begin
                    // Compute delta vectors and extract integer km
                    for (int i = 0; i < 3; i++) begin
                        r_dr[i]     <= $signed(posA[i]) - $signed(posB[i]);
                        r_dv[i]     <= $signed(velA[i]) - $signed(velB[i]);
                        r_dr_int[i] <= {16'b0, posA[i][31:16] - posB[i][31:16]};
                    end
                    state <= S_SQRT_START;
                end

                S_SQRT_START: begin
                    // dr_sq = Σ dr_int[i]²  (km², integer)
                    dr_sq    <= (r_dr_int[0] * r_dr_int[0]) +
                                (r_dr_int[1] * r_dr_int[1]) +
                                (r_dr_int[2] * r_dr_int[2]);
                    sqrt_in  <= (r_dr_int[0] * r_dr_int[0]) +
                                (r_dr_int[1] * r_dr_int[1]) +
                                (r_dr_int[2] * r_dr_int[2]);
                    sqrt_start <= 1'b1;
                    state    <= S_SQRT_WAIT;
                end

                S_SQRT_WAIT: begin
                    if (sqrt_done) begin
                        // sqrt_out = |Δr|/256 in Q15.16 → scale back × 256
                        sep_q <= {sqrt_out[23:0], 8'b0};
                        state <= S_DOT;
                    end
                end

                S_DOT: begin
                    // dot_rv = Σ delta_r[i] × delta_v[i], result in Q15.16
                    dot_acc   <= ($signed(r_dr[0]) * $signed(r_dv[0]) +
                                  $signed(r_dr[1]) * $signed(r_dv[1]) +
                                  $signed(r_dr[2]) * $signed(r_dv[2])) >>> 32;
                    dot_q1516 <= 32'(($signed(r_dr[0]) * $signed(r_dv[0]) +
                                      $signed(r_dr[1]) * $signed(r_dv[1]) +
                                      $signed(r_dr[2]) * $signed(r_dv[2])) >>> 32);
                    state <= S_DIV_START;
                end

                S_DIV_START: begin
                    // closure_rate = dot_rv / separation_km  (Q15.16 / Q15.16)
                    // fp_divider performs signed integer division so scale:
                    // dividend = dot_q1516 (Q15.16), divisor = sep_q (Q15.16)
                    // result = dot_q1516 / sep_q (dimensionless ratio ×65536 = Q15.16 km/s)
                    div_dividend <= dot_q1516;
                    div_divisor  <= $signed(sep_q);
                    div_start    <= 1'b1;
                    state        <= S_DIV_WAIT;
                end

                S_DIV_WAIT: begin
                    if (div_done) begin
                        state <= S_DONE;
                    end
                end

                S_DONE: begin
                    for (int i = 0; i < 3; i++) begin
                        delta_r_eci[i] <= r_dr[i];
                        delta_v_eci[i] <= r_dv[i];
                    end
                    separation_km     <= sep_q;
                    // Closure rate is -dot/|dr|; report magnitude
                    closure_rate_kmps <= div_quotient[31] ?
                                         32'(-$signed(div_quotient)) :
                                         div_quotient;
                    relative_valid    <= 1'b1;
                    state             <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule