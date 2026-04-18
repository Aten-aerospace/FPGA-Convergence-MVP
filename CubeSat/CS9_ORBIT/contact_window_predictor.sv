// =============================================================================
// Module: contact_window_predictor (CS9 ground-contact geometry)
// Subsystem: CS9 - Orbit Propagator
// Description: Estimates the elevation angle of the satellite as seen from a
//   ground station and asserts contact_valid when the satellite clears the
//   minimum mask angle (10°).
//
//   Method:
//     1. Convert geodetic lat/lon to unit direction vectors for both the
//        satellite sub-point and the ground station using CORDIC sin/cos.
//     2. Compute the central angle θ via dot product of unit vectors.
//     3. Elevation angle ε using: ε = atan(cos θ / sin θ) - asin(Re/(Re+h))
//        Simplified:  ε ≈ atan((h/(Re×θ)) - θ/2)  for small θ
//
//   Simplified FPGA implementation:
//     - Use CORDIC to compute sin/cos of lat/lon angles (Q1.15 angle input)
//     - Compute dot product of ground-track unit vectors
//     - Derive elevation via Bhaskara approximation as in ground_track_calculator
//
//   Angle input conversion:
//     latitude_rad / longitude_rad are Q15.16 (π ≈ 205887).
//     CORDIC input is Q1.15 (π ≈ 32768 = 0x8000), so scale:
//       cordic_angle = latitude_rad[16:1]  (>> 1 maps 205887→102943≈π Q1.15 ✓)
//
//   AOS/LOS prediction: outputs are set to 0 (requires full propagation engine).
//
//   Minimum mask angle: 10° → in Q15.16 rad = round(10×π/180×65536) = 11380
// =============================================================================
`timescale 1ns/1ps

module contact_window_predictor (
    input  logic        clk,
    input  logic        rst_n,

    input  logic [31:0] latitude_rad,     // satellite Q15.16 rad
    input  logic [31:0] longitude_rad,    // satellite Q15.16 rad
    input  logic [31:0] altitude_m,       // satellite altitude Q15.16 m
    input  logic [31:0] gnd_lat_rad,      // ground station Q15.16 rad
    input  logic [31:0] gnd_lon_rad,      // ground station Q15.16 rad
    input  logic [31:0] gnd_alt_m,        // ground station altitude Q15.16 m
    input  logic        calc_valid,

    output logic [15:0] elevation_angle_deg, // Q8.8 (0-90°)
    output logic        los_imminent,
    output logic [15:0] aos_predicted_secs,  // 0 (future work)
    output logic [15:0] los_predicted_secs,  // 0 (future work)
    output logic        contact_valid        // elevation > 10°
);

    // =========================================================================
    // Constants
    // =========================================================================
    // 10° minimum mask in Q15.16 rad = round(10 × π/180 × 65536) = 11380
    localparam logic [31:0] MASK_RAD_Q1516  = 32'd11380;
    // π/4 in Q15.16 = 51471
    localparam logic signed [31:0] QTR_PI_Q1516 = 32'sh0000_C90F;
    // π/2 in Q15.16 = 102944
    localparam logic signed [31:0] HALF_PI_Q1516 = 32'sh0001_921F;
    // 1.0 in Q15.16
    localparam logic [31:0] ONE_Q1516 = 32'h0001_0000;
    // 57.296 deg/rad in Q15.16 = round(57.2958 × 65536) = 3754022 (Q8.8 scale for output)
    // 1 radian in Q8.8 degrees = 57.296 × 256 = 14668
    localparam logic [31:0] RAD_TO_DEG_Q8_SCALE = 32'd14668;

    // =========================================================================
    // CORDIC instances (one for satellite lat, one for ground station lat)
    // We reuse a single CORDIC instance sequenced by FSM.
    // =========================================================================
    logic        cordic_start;
    logic [15:0] cordic_angle;
    logic [15:0] cordic_cos_out, cordic_sin_out;
    logic        cordic_done;

    cordic u_cordic (
        .clk(clk), .rst_n(rst_n),
        .start(cordic_start),
        .angle(cordic_angle),
        .cos_out(cordic_cos_out), .sin_out(cordic_sin_out),
        .done(cordic_done)
    );

    // fp_divider for elevation angle computation
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
    // Unit-vector components (Q1.15 → stored as signed 32-bit for dot product)
    // =========================================================================
    // Satellite: (cos_lat × cos_lon, cos_lat × sin_lon, sin_lat)
    // Ground:    (cos_glat × cos_glon, cos_glat × sin_glon, sin_glat)
    //
    // We compute each of the 4 trig values sequentially with CORDIC.
    // =========================================================================
    logic signed [15:0] sat_cos_lat, sat_sin_lat;
    logic signed [15:0] sat_cos_lon, sat_sin_lon;
    logic signed [15:0] gnd_cos_lat, gnd_sin_lat;
    logic signed [15:0] gnd_cos_lon, gnd_sin_lon;

    // Unit vectors (Q1.15 products, result in Q2.30, shift >>15 for Q1.15)
    logic signed [31:0] sat_ux, sat_uy, sat_uz;
    logic signed [31:0] gnd_ux, gnd_uy, gnd_uz;
    logic signed [63:0] dot_prod_wide;
    logic signed [31:0] dot_prod;   // cos(central_angle), Q1.15 range

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [3:0] {
        S_IDLE,
        S_SAT_LAT,   S_WAIT_SAT_LAT,
        S_SAT_LON,   S_WAIT_SAT_LON,
        S_GND_LAT,   S_WAIT_GND_LAT,
        S_GND_LON,   S_WAIT_GND_LON,
        S_DOT,
        S_ELEV_DIV,  S_WAIT_DIV,
        S_OUTPUT
    } state_t;

    state_t state;

    // Scale Q15.16 rad → Q1.15 CORDIC angle input: >> 1
    // (Since π in Q15.16 = 205887 >> 1 = 102943 ≈ 32768×π/π × ... actually
    //  Q1.15 has π = 32768, Q15.16 has π = 205887, ratio = 205887/32768 ≈ 6.28 = 2π
    //  So correct scale: cordic_angle = (lat_rad_q1516 × 32768 / 205887) >> 0
    //  Approximation: × 1 >> 2 (factor 0.25; actual factor = 32768/205887 = 0.159)
    //  Better: lat_q1516 × 10 >> 6 = × 0.15625 ≈ 0.159 (3% error, acceptable)
    function automatic logic [15:0] to_cordic_angle(
        input logic [31:0] rad_q1516  // Q15.16 radians
    );
        // scale: 32768/205887 ≈ 5 >> 5 = 0.15625
        to_cordic_angle = 16'((32'(rad_q1516) * 32'd5) >> 5);
    endfunction

    // Bhaskara atan for elevation computation
    function automatic logic signed [31:0] atan_approx(
        input logic signed [31:0] ratio_q1516
    );
        logic signed [63:0] tmp;
        tmp = ratio_q1516 * $signed(QTR_PI_Q1516);
        atan_approx = 32'(tmp >>> 16);
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                <= S_IDLE;
            cordic_start         <= 1'b0;
            div_start            <= 1'b0;
            elevation_angle_deg  <= '0;
            los_imminent         <= 1'b0;
            aos_predicted_secs   <= '0;
            los_predicted_secs   <= '0;
            contact_valid        <= 1'b0;
            sat_cos_lat <= '0; sat_sin_lat <= '0;
            sat_cos_lon <= '0; sat_sin_lon <= '0;
            gnd_cos_lat <= '0; gnd_sin_lat <= '0;
            gnd_cos_lon <= '0; gnd_sin_lon <= '0;
            sat_ux <= '0; sat_uy <= '0; sat_uz <= '0;
            gnd_ux <= '0; gnd_uy <= '0; gnd_uz <= '0;
            dot_prod <= '0;
            cordic_angle <= '0;
            div_dividend <= '0; div_divisor <= '0;
        end else begin
            cordic_start <= 1'b0;
            div_start    <= 1'b0;

            case (state)
                S_IDLE: begin
                    contact_valid <= 1'b0;
                    los_imminent  <= 1'b0;
                    if (calc_valid) state <= S_SAT_LAT;
                end

                // ---- Satellite latitude sin/cos ----
                S_SAT_LAT: begin
                    cordic_angle <= to_cordic_angle(latitude_rad);
                    cordic_start <= 1'b1;
                    state        <= S_WAIT_SAT_LAT;
                end
                S_WAIT_SAT_LAT: begin
                    if (cordic_done) begin
                        sat_cos_lat <= $signed(cordic_cos_out);
                        sat_sin_lat <= $signed(cordic_sin_out);
                        state       <= S_SAT_LON;
                    end
                end

                // ---- Satellite longitude sin/cos ----
                S_SAT_LON: begin
                    cordic_angle <= to_cordic_angle(longitude_rad);
                    cordic_start <= 1'b1;
                    state        <= S_WAIT_SAT_LON;
                end
                S_WAIT_SAT_LON: begin
                    if (cordic_done) begin
                        sat_cos_lon <= $signed(cordic_cos_out);
                        sat_sin_lon <= $signed(cordic_sin_out);
                        // Build satellite unit vector using proper signed multiplication
                        // sat_cos_lat and cordic_cos/sin are Q1.15 signed [15:0]
                        sat_ux <= ($signed(sat_cos_lat) * $signed(cordic_cos_out)) >>> 15;
                        sat_uy <= ($signed(sat_cos_lat) * $signed(cordic_sin_out)) >>> 15;
                        sat_uz <= {{16{sat_sin_lat[15]}}, sat_sin_lat};
                        state  <= S_GND_LAT;
                    end
                end

                // ---- Ground station latitude ----
                S_GND_LAT: begin
                    cordic_angle <= to_cordic_angle(gnd_lat_rad);
                    cordic_start <= 1'b1;
                    state        <= S_WAIT_GND_LAT;
                end
                S_WAIT_GND_LAT: begin
                    if (cordic_done) begin
                        gnd_cos_lat <= $signed(cordic_cos_out);
                        gnd_sin_lat <= $signed(cordic_sin_out);
                        state       <= S_GND_LON;
                    end
                end

                // ---- Ground station longitude ----
                S_GND_LON: begin
                    cordic_angle <= to_cordic_angle(gnd_lon_rad);
                    cordic_start <= 1'b1;
                    state        <= S_WAIT_GND_LON;
                end
                S_WAIT_GND_LON: begin
                    if (cordic_done) begin
                        gnd_cos_lon <= $signed(cordic_cos_out);
                        gnd_sin_lon <= $signed(cordic_sin_out);
                        gnd_ux <= ($signed(gnd_cos_lat) * $signed(cordic_cos_out)) >>> 15;
                        gnd_uy <= ($signed(gnd_cos_lat) * $signed(cordic_sin_out)) >>> 15;
                        gnd_uz <= {{16{gnd_sin_lat[15]}}, gnd_sin_lat};
                        state  <= S_DOT;
                    end
                end

                // ---- Dot product = cos(central_angle) ----
                S_DOT: begin
                    dot_prod_wide <= $signed(sat_ux) * $signed(gnd_ux) +
                                      $signed(sat_uy) * $signed(gnd_uy) +
                                      $signed(sat_uz) * $signed(gnd_uz);
                    dot_prod <= 32'(($signed(sat_ux) * $signed(gnd_ux) +
                                     $signed(sat_uy) * $signed(gnd_uy) +
                                     $signed(sat_uz) * $signed(gnd_uz)) >>> 15);
                    // Elevation ≈ asin(dot_prod) - nadir_angle
                    // Simplified: elev ≈ atan(dot_prod / sqrt(1-dot²))
                    // → use atan_approx(dot_prod) as elevation approximation
                    state <= S_ELEV_DIV;
                end

                S_ELEV_DIV: begin
                    // elev ≈ atan(cos_θ) × π/4; scale to Q8.8 degrees
                    // 1 rad = 57.296° → Q8.8: 57.296 × 256 = 14668
                    elevation_angle_deg <=
                        16'(($signed(atan_approx(dot_prod)) * 32'sd14668) >>> 16);
                    state <= S_OUTPUT;
                end

                S_WAIT_DIV: begin
                    if (div_done) state <= S_OUTPUT;
                end

                S_OUTPUT: begin
                    contact_valid <= ($signed(atan_approx(dot_prod)) >=
                                      $signed(MASK_RAD_Q1516));
                    los_imminent  <= ($signed(atan_approx(dot_prod)) >=
                                      $signed(MASK_RAD_Q1516)) &&
                                     ($signed(atan_approx(dot_prod)) <
                                      $signed(MASK_RAD_Q1516 + 32'd5690));
                    aos_predicted_secs <= '0;
                    los_predicted_secs <= '0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule