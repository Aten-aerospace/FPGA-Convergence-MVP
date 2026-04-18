// =============================================================================
// Module: tle_parser (CS9 TLE ASCII parser)
// Subsystem: CS9 - Orbit Propagator
// Description: Accepts raw 69-byte TLE Line-1 and Line-2 ASCII arrays,
//   validates both checksums, and extracts the six Keplerian elements plus
//   epoch in Modified Julian Date (Q15.16).
//
//   Angle scale: Q0.15 where 32768 = π radians
//     degrees → Q0.15:  angle = round(deg × 32768 / 180)
//     = (deg_1e4 × 19114) >> 20  (deg_1e4 = degrees × 10000)
//
//   Eccentricity Q0.15: e_Q015 = (ecc_1e7 × 4296) >> 20
//     ecc_1e7 = 7 ASCII digits (no decimal point; TLE implies "0.")
//
//   Mean motion Q15.16 rad/s: n_Q1516 = (rev_1e8 × 52407) >> 40
//     rev_1e8 = 10 significant digits of "dd.xxxxxxxx" rev/day
//
// Line-2 byte map (0-indexed):
//   [ 8:15] inclination   "ddd.dddd"
//   [17:24] RAAN          "ddd.dddd"
//   [26:32] eccentricity  "xxxxxxx"  (7 digits)
//   [34:41] arg of perigee "ddd.dddd"
//   [43:50] mean anomaly   "ddd.dddd"
//   [52:62] mean motion    "dd.xxxxxxxx"
//   [68]    checksum digit
//
// Line-1 byte map:
//   [18:19] 2-digit epoch year
//   [20:22] epoch day hundreds/tens/units
//   [68]    checksum digit
// =============================================================================
`timescale 1ns/1ps

module tle_parser (
    input  logic        clk,
    input  logic        rst_n,

    input  logic [7:0]  tle_line1 [0:68],
    input  logic [7:0]  tle_line2 [0:68],
    input  logic        tle_write,

    output logic [31:0] tle_n0,           // mean motion  Q15.16 rad/s
    output logic [15:0] tle_e0,           // eccentricity Q0.15
    output logic [15:0] tle_i0,           // inclination  Q0.15 rad
    output logic [15:0] tle_raan,         // RAAN         Q0.15 rad
    output logic [15:0] tle_argp,         // arg perigee  Q0.15 rad
    output logic [15:0] tle_m0,           // mean anomaly Q0.15 rad
    output logic [31:0] tle_epoch_mjd,    // epoch MJD    Q15.16
    output logic        tle_checksum_ok,
    output logic        tle_parsed_valid
);

    // =========================================================================
    // Registered byte arrays
    // =========================================================================
    logic [7:0] r_l1 [0:68];
    logic [7:0] r_l2 [0:68];

    // =========================================================================
    // State machine
    // =========================================================================
    typedef enum logic [1:0] { S_IDLE, S_WAIT, S_DONE } state_t;
    state_t state;

    // =========================================================================
    // Angle conversion: ddd.dddd (8 bytes with '.' at position +3)
    //   → Q0.15 radians (scale: 32768 = π)
    //   angle_Q015 = (deg_1e4 × 19114) >> 20
    //   For d0..d2 = integer digits, d4..d7 = fractional digits (byte+3 = '.')
    // =========================================================================
    function automatic logic [15:0] parse_angle8(
        input logic [7:0] b0, b1, b2,   // integer-digit bytes d0 d1 d2
        input logic [7:0] b4, b5, b6, b7 // fractional-digit bytes
    );
        logic [22:0] deg_1e4;
        logic [37:0] prod;
        deg_1e4 = ({19'b0, b0[3:0]} * 23'd1000000) +
                  ({19'b0, b1[3:0]} * 23'd100000)  +
                  ({19'b0, b2[3:0]} * 23'd10000)   +
                  ({19'b0, b4[3:0]} * 23'd1000)    +
                  ({19'b0, b5[3:0]} * 23'd100)     +
                  ({19'b0, b6[3:0]} * 23'd10)      +
                   {19'b0, b7[3:0]};
        prod = {15'b0, deg_1e4} * 38'd19114;
        parse_angle8 = prod[35:20];
    endfunction

    // =========================================================================
    // Combinational signals
    // =========================================================================
    logic [9:0]  comb_sum1, comb_sum2;
    logic        comb_chk1, comb_chk2;
    logic [15:0] comb_i0, comb_raan, comb_argp, comb_m0, comb_e0;
    logic [31:0] comb_n0, comb_epoch;

    always_comb begin : extract_blk
        // ---- Line-1 checksum ----
        comb_sum1 = 10'b0;
        for (int i = 0; i < 68; i++) begin
            if (r_l1[i] >= 8'h30 && r_l1[i] <= 8'h39)
                comb_sum1 = comb_sum1 + 10'(r_l1[i] - 8'h30);
            else if (r_l1[i] == 8'h2B || r_l1[i] == 8'h2D) // '+' or '-'
                comb_sum1 = comb_sum1 + 10'd1;
        end
        comb_chk1 = ((comb_sum1 % 10) == 10'(r_l1[68] - 8'h30));

        // ---- Line-2 checksum ----
        comb_sum2 = 10'b0;
        for (int i = 0; i < 68; i++) begin
            if (r_l2[i] >= 8'h30 && r_l2[i] <= 8'h39)
                comb_sum2 = comb_sum2 + 10'(r_l2[i] - 8'h30);
            else if (r_l2[i] == 8'h2B || r_l2[i] == 8'h2D)
                comb_sum2 = comb_sum2 + 10'd1;
        end
        comb_chk2 = ((comb_sum2 % 10) == 10'(r_l2[68] - 8'h30));

        // ---- Orbital element angles (ddd.dddd) ----
        // Inclination: bytes 8-15; '.' at byte 11
        comb_i0   = parse_angle8(r_l2[8],  r_l2[9],  r_l2[10],
                                 r_l2[12], r_l2[13], r_l2[14], r_l2[15]);
        // RAAN: bytes 17-24; '.' at byte 20
        comb_raan = parse_angle8(r_l2[17], r_l2[18], r_l2[19],
                                 r_l2[21], r_l2[22], r_l2[23], r_l2[24]);
        // Arg of perigee: bytes 34-41; '.' at byte 37
        comb_argp = parse_angle8(r_l2[34], r_l2[35], r_l2[36],
                                 r_l2[38], r_l2[39], r_l2[40], r_l2[41]);
        // Mean anomaly: bytes 43-50; '.' at byte 46
        comb_m0   = parse_angle8(r_l2[43], r_l2[44], r_l2[45],
                                 r_l2[47], r_l2[48], r_l2[49], r_l2[50]);

        // ---- Eccentricity: bytes 26-32 (7 digits, implied "0.") ----
        // e_Q015 = (ecc_1e7 × 4296) >> 20
        begin
            logic [22:0] ecc_1e7;
            logic [38:0] ecc_prod;
            ecc_1e7  = ({19'b0, r_l2[26][3:0]} * 23'd1000000) +
                       ({19'b0, r_l2[27][3:0]} * 23'd100000)  +
                       ({19'b0, r_l2[28][3:0]} * 23'd10000)   +
                       ({19'b0, r_l2[29][3:0]} * 23'd1000)    +
                       ({19'b0, r_l2[30][3:0]} * 23'd100)     +
                       ({19'b0, r_l2[31][3:0]} * 23'd10)      +
                        {19'b0, r_l2[32][3:0]};
            ecc_prod = {16'b0, ecc_1e7} * 39'd4296;
            comb_e0  = ecc_prod[35:20];
        end

        // ---- Mean motion: bytes 52-62 ("dd.xxxxxxxx") ----
        // n_Q1516 = (rev_1e8 × 52407) >> 40
        begin
            logic [33:0] rev_1e8;
            logic [49:0] n_prod;
            rev_1e8 = ({30'b0, r_l2[52][3:0]} * 34'd1000000000) +
                      ({30'b0, r_l2[53][3:0]} * 34'd100000000)  +
                      // byte 54 = '.'
                      ({30'b0, r_l2[55][3:0]} * 34'd10000000)   +
                      ({30'b0, r_l2[56][3:0]} * 34'd1000000)    +
                      ({30'b0, r_l2[57][3:0]} * 34'd100000)     +
                      ({30'b0, r_l2[58][3:0]} * 34'd10000)      +
                      ({30'b0, r_l2[59][3:0]} * 34'd1000)       +
                      ({30'b0, r_l2[60][3:0]} * 34'd100)        +
                      ({30'b0, r_l2[61][3:0]} * 34'd10)         +
                       {30'b0, r_l2[62][3:0]};
            n_prod  = {16'b0, rev_1e8} * 50'd52407;
            // >> 40: take bits [49:40] padded to 32 bits
            comb_n0 = {22'b0, n_prod[49:40]};
        end

        // ---- Epoch MJD (Q15.16): year from line-1 bytes 18-19, day 20-22 ----
        begin
            logic [6:0]  yr2;
            logic [13:0] full_yr;
            logic [9:0]  day_int;
            logic [21:0] mjd_int;
            yr2      = 7'({3'b0, r_l1[18][3:0]} * 7'd10 + {3'b0, r_l1[19][3:0]});
            full_yr  = (yr2 >= 7'd57) ? (14'd1900 + 14'(yr2)) : (14'd2000 + 14'(yr2));
            day_int  = 10'({6'b0, r_l1[20][3:0]} * 10'd100 +
                            {6'b0, r_l1[21][3:0]} * 10'd10  +
                            {6'b0, r_l1[22][3:0]});
            mjd_int  = 22'd51544 + 22'((full_yr - 14'd2000) * 14'd365) +
                       22'(day_int) - 22'd1;
            comb_epoch = {mjd_int, 10'b0}; // integer MJD in Q15.16 upper bits
        end
    end

    // =========================================================================
    // State machine: latch bytes → wait 1 cycle → register outputs
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state           <= S_IDLE;
            tle_parsed_valid <= 1'b0;
            tle_checksum_ok  <= 1'b0;
            tle_n0           <= '0;
            tle_e0           <= '0;
            tle_i0           <= '0;
            tle_raan         <= '0;
            tle_argp         <= '0;
            tle_m0           <= '0;
            tle_epoch_mjd    <= '0;
            for (int i = 0; i <= 68; i++) begin
                r_l1[i] <= '0;
                r_l2[i] <= '0;
            end
        end else begin
            case (state)
                S_IDLE: begin
                    tle_parsed_valid <= 1'b0;
                    if (tle_write) begin
                        for (int i = 0; i <= 68; i++) begin
                            r_l1[i] <= tle_line1[i];
                            r_l2[i] <= tle_line2[i];
                        end
                        state <= S_WAIT;
                    end
                end

                S_WAIT: begin
                    // Allow one cycle for combinational paths to settle
                    state <= S_DONE;
                end

                S_DONE: begin
                    tle_n0           <= comb_n0;
                    tle_e0           <= comb_e0;
                    tle_i0           <= comb_i0;
                    tle_raan         <= comb_raan;
                    tle_argp         <= comb_argp;
                    tle_m0           <= comb_m0;
                    tle_epoch_mjd    <= comb_epoch;
                    tle_checksum_ok  <= comb_chk1 && comb_chk2;
                    tle_parsed_valid <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule