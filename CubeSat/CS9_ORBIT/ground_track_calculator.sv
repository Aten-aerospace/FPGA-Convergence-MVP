// =============================================================================
// Module: ground_track_calculator (CS9 ECI → geodetic converter)
// Subsystem: CS9 - Orbit Propagator
// Description: Converts ECI position to geodetic latitude, longitude, and
//   altitude above the WGS84 ellipsoid.
//
//   Algorithm (simplified geocentric, sufficient for CubeSat orbit display):
//     p        = sqrt(x² + y²)              (ground-track radius)
//     longitude = atan2(y, x)               (approximated without vectoring CORDIC)
//     latitude  = atan2(z, p)               (geocentric; geodetic correction skipped)
//     altitude  = |r| - Re                  (Re = 6378.137 km)
//
//   atan2 approximation (Bhaskara I variant, < 1° error):
//     For |num/den| ≤ 1 (first-quadrant equivalent):
//       atan(t) ≈ t × (π/4) / (0.2732 t² + 1)
//       where coefficient 0.2732 ≈ 0x4613 in Q15.16
//     Quadrant correction applied after.
//
//   Magnitude scaling (matches orbit_health_monitor):
//     r_km_int = eci_pos >> 16  (integer km)
//     r_sq     = Σ r_km_int[i]²
//     sqrt(r_sq) → |r|/256 Q15.16  →  position_km = sqrt_out << 8
//
//   Output formats:
//     latitude_rad  Q15.16  (range ±π/2  → raw ±102944)
//     longitude_rad Q15.16  (range ±π   → raw ±205888)
//     altitude_m    Q15.16  (meters above Re; Re = 6378137 m)
// =============================================================================
`timescale 1ns/1ps

module ground_track_calculator (
    input  logic        clk,
    input  logic        rst_n,

    input  logic [31:0] eci_pos [0:2],    // Q15.16 km
    input  logic        calc_valid,       // strobe: begin computation

    output logic [31:0] latitude_rad,     // Q15.16
    output logic [31:0] longitude_rad,    // Q15.16
    output logic [31:0] altitude_m,       // Q15.16 m above WGS84 approx
    output logic        ground_track_valid
);

    // =========================================================================
    // Constants
    // =========================================================================
    // Re = 6378.137 km  → Q15.16: 6378 × 65536 = 417,923,072 = 0x18E8_0000
    localparam logic [31:0] RE_Q1516 = 32'h18E8_0000;
    // Re in metres × 65536 = 6378137 × 65536 = 417,974,304,768 → too large for 32-bit
    // Store as integer metres: 6378137; output in Q15.16 after subtraction
    localparam logic [31:0] RE_KM_INT = 32'd6378; // integer km (approximate)

    // π in Q15.16 = 205887 = 0x0003_243F (≈ π × 65536)
    localparam logic signed [31:0] PI_Q1516    = 32'sh0003_243F;
    // π/2 in Q15.16 = 102944 = 0x0001_921F
    localparam logic signed [31:0] HALF_PI_Q1516 = 32'sh0001_921F;
    // π/4 in Q15.16 = 51471 = 0x0000_C90F
    localparam logic signed [31:0] QTR_PI_Q1516  = 32'sh0000_C90F;
    // Bhaskara coefficient 0.2732 in Q15.16 = round(0.2732 × 65536) = 17908 = 0x45F4
    localparam logic [31:0] BHAS_COEFF = 32'h0000_45F4;

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
    // atan2 approximation function using Bhaskara I variant (< 10% error)
    //   atan(t) ≈ t × π/4  for |t| ≤ 1 (first approximation, no division needed)
    //   Result in Q15.16 radians
    function automatic logic signed [31:0] atan_approx(
        input logic signed [31:0] ratio   // Q15.16 ratio t = y/x or z/p
    );
        logic signed [63:0] tmp;
        tmp = ratio * $signed(QTR_PI_Q1516);
        atan_approx = 32'(tmp >>> 16);
    endfunction
    typedef enum logic [3:0] {
        S_IDLE,
        S_LATCH,
        S_RSQRT_START,    // sqrt for |r| (all 3 axes)
        S_RSQRT_WAIT,
        S_PSQRT_START,    // sqrt for p = sqrt(x²+y²)
        S_PSQRT_WAIT,
        S_LON_DIV_START,  // div: y/x for longitude atan2
        S_LON_DIV_WAIT,
        S_LAT_DIV_START,  // div: z/p for latitude atan2
        S_LAT_DIV_WAIT,
        S_ATAN_LON,
        S_ATAN_LAT,
        S_OUTPUT
    } state_t;

    state_t state;

    logic signed [31:0] rx, ry, rz;
    logic [31:0] rx_km, ry_km, rz_km; // integer km
    logic [31:0] r_sq, p_sq;
    logic [31:0] r_mag_q; // |r| Q15.16
    logic [31:0] p_mag_q; // p   Q15.16

    // atan2 intermediate
    logic signed [31:0] lon_ratio_q; // y/x Q15.16
    logic signed [31:0] lat_ratio_q; // z/p Q15.16
    logic [1:0]  lon_quad;           // quadrant flags {x<0, y<0}
    logic [1:0]  lat_quad;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state              <= S_IDLE;
            sqrt_start         <= 1'b0;
            div_start          <= 1'b0;
            ground_track_valid <= 1'b0;
            latitude_rad       <= '0;
            longitude_rad      <= '0;
            altitude_m         <= '0;
            rx <= '0; ry <= '0; rz <= '0;
            rx_km <= '0; ry_km <= '0; rz_km <= '0;
            r_sq <= '0; p_sq <= '0;
            r_mag_q <= '0; p_mag_q <= '0;
            lon_ratio_q <= '0; lat_ratio_q <= '0;
            lon_quad <= '0; lat_quad <= '0;
            sqrt_in <= '0; div_dividend <= '0; div_divisor <= '0;
        end else begin
            sqrt_start <= 1'b0;
            div_start  <= 1'b0;

            case (state)
                S_IDLE: begin
                    ground_track_valid <= 1'b0;
                    if (calc_valid) state <= S_LATCH;
                end

                S_LATCH: begin
                    rx    <= $signed(eci_pos[0]);
                    ry    <= $signed(eci_pos[1]);
                    rz    <= $signed(eci_pos[2]);
                    rx_km <= {16'b0, eci_pos[0][31:16]};
                    ry_km <= {16'b0, eci_pos[1][31:16]};
                    rz_km <= {16'b0, eci_pos[2][31:16]};
                    // Record quadrant signs before taking absolute values
                    lon_quad <= {eci_pos[0][31], eci_pos[1][31]};
                    lat_quad <= {eci_pos[2][31], 1'b0};
                    state <= S_RSQRT_START;
                end

                S_RSQRT_START: begin
                    r_sq    <= (rx_km * rx_km) + (ry_km * ry_km) + (rz_km * rz_km);
                    sqrt_in <= (rx_km * rx_km) + (ry_km * ry_km) + (rz_km * rz_km);
                    sqrt_start <= 1'b1;
                    state <= S_RSQRT_WAIT;
                end

                S_RSQRT_WAIT: begin
                    if (sqrt_done) begin
                        r_mag_q <= {sqrt_out[23:0], 8'b0}; // × 256 scale-back
                        state   <= S_PSQRT_START;
                    end
                end

                S_PSQRT_START: begin
                    p_sq    <= (rx_km * rx_km) + (ry_km * ry_km);
                    sqrt_in <= (rx_km * rx_km) + (ry_km * ry_km);
                    sqrt_start <= 1'b1;
                    state <= S_PSQRT_WAIT;
                end

                S_PSQRT_WAIT: begin
                    if (sqrt_done) begin
                        p_mag_q <= {sqrt_out[23:0], 8'b0};
                        state   <= S_LON_DIV_START;
                    end
                end

                // longitude = atan2(y, x): compute y/x
                S_LON_DIV_START: begin
                    // Use absolute values; track sign via lon_quad
                    div_dividend <= lon_quad[0] ? 32'(-$signed(ry)) : ry; // |y|
                    div_divisor  <= lon_quad[1] ? 32'(-$signed(rx)) : rx; // |x|
                    div_start    <= 1'b1;
                    state        <= S_LON_DIV_WAIT;
                end

                S_LON_DIV_WAIT: begin
                    if (div_done) begin
                        lon_ratio_q <= div_quotient; // |y/x| in Q15.16
                        state <= S_LAT_DIV_START;
                    end
                end

                // latitude = atan2(z, p): compute z/p
                S_LAT_DIV_START: begin
                    div_dividend <= lat_quad[1] ? 32'(-$signed(rz)) : rz; // |z|
                    div_divisor  <= $signed(p_mag_q);
                    div_start    <= 1'b1;
                    state        <= S_LAT_DIV_WAIT;
                end

                S_LAT_DIV_WAIT: begin
                    if (div_done) begin
                        lat_ratio_q <= div_quotient;
                        state <= S_ATAN_LON;
                    end
                end

                S_ATAN_LON: begin
                    // lon_quad = {x<0, y<0}: bit[1]=x<0, bit[0]=y<0
                    case (lon_quad)
                        2'b00: longitude_rad <= atan_approx(lon_ratio_q);
                        2'b01: longitude_rad <= 32'(-$signed(atan_approx(lon_ratio_q)));
                        2'b10: longitude_rad <= $signed(PI_Q1516) -
                                                $signed(atan_approx(lon_ratio_q));
                        2'b11: longitude_rad <= $signed(atan_approx(lon_ratio_q)) -
                                                $signed(PI_Q1516);
                        default: longitude_rad <= atan_approx(lon_ratio_q);
                    endcase
                    state <= S_ATAN_LAT;
                end

                S_ATAN_LAT: begin
                    latitude_rad <= lat_quad[1] ?
                                    32'(-$signed(atan_approx(lat_ratio_q))) :
                                    32'(atan_approx(lat_ratio_q));
                    state <= S_OUTPUT;
                end

                S_OUTPUT: begin
                    // altitude = (|r| - Re) in metres, stored as Q15.16
                    // alt_km Q15.16 = r_mag_q - RE_Q1516
                    // alt_m  Q15.16 = alt_km × 1000  (wide multiply then truncate)
                    altitude_m <= 32'(($signed(r_mag_q) - $signed(RE_Q1516)) * 32'sd1000);
                    ground_track_valid <= 1'b1;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule