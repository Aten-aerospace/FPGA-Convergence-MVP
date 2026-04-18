// =============================================================================
// Module: orbit_health_monitor (CS9 orbital health watchdog)
// Subsystem: CS9 - Orbit Propagator
// Description: Monitors position and velocity magnitudes for out-of-bounds
//   conditions consistent with a nominal LEO CubeSat orbit, and tracks the
//   age of the current TLE set.
//
//   Magnitude computation method:
//     r_km[i] = eci_pos[i] >> 16         (integer-km part of Q15.16)
//     r_sq    = Σ r_km[i]²               (km², ≤ 3×7500² = 168.75 M < 2²⁸)
//     sqrt input = r_sq (treated as raw Q15.16 by sqrt.sv)
//       → sqrt output = |r|/256 in Q15.16
//     position_magnitude_km = sqrt_out << 8  (Q15.16 km)
//
//     For velocity, scale by 256 first (v_q8 = eci_vel >> 8 = v × 256):
//       v_q8² = v² × 65536  → sqrt input = Σ v_q8[i]²
//       → sqrt output = |v| in Q15.16 directly
//
//   Bounds (Q15.16):
//     Position: 0x188D_0000 (6300 km) ≤ |r| ≤ 0x1D4C_0000 (7500 km)
//     Velocity: 0x0006_0000 (6 km/s) ≤ |v| ≤ 0x0009_0000 (9 km/s)
//     TLE stale: tle_age_hours > 168 (7 days)
// =============================================================================
`timescale 1ns/1ps

module orbit_health_monitor (
    input  logic        clk,
    input  logic        rst_n,

    input  logic [31:0] eci_pos [0:2],   // Q15.16 km
    input  logic [31:0] eci_vel [0:2],   // Q15.16 km/s
    input  logic        tle_write,       // strobe: new TLE loaded (resets age)
    input  logic        ce_1hz,          // 1-Hz enable

    output logic        propagator_valid,        // magnitude within bounds
    output logic        overflow_flag,           // magnitude out of bounds
    output logic [15:0] tle_age_hours,           // hours since last TLE write
    output logic        tle_stale,               // age > 168 h (7 days)
    output logic [31:0] position_magnitude_km,   // Q15.16
    output logic [31:0] velocity_magnitude_kmps  // Q15.16
);

    // =========================================================================
    // Bounds constants
    // =========================================================================
    localparam logic [31:0] POS_LO = 32'h188D_0000; // 6300 km Q15.16
    localparam logic [31:0] POS_HI = 32'h1D4C_0000; // 7500 km Q15.16
    localparam logic [31:0] VEL_LO = 32'h0006_0000; // 6 km/s  Q15.16
    localparam logic [31:0] VEL_HI = 32'h0009_0000; // 9 km/s  Q15.16
    localparam logic [15:0] AGE_STALE = 16'd168;     // 7 days in hours

    // =========================================================================
    // TLE age counter (in hours, incremented every 3600 ce_1hz pulses)
    // =========================================================================
    logic [11:0] sec_cnt;   // counts 0..3599

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sec_cnt      <= '0;
            tle_age_hours <= '0;
            tle_stale     <= 1'b0;
        end else begin
            if (tle_write) begin
                sec_cnt       <= '0;
                tle_age_hours <= '0;
                tle_stale     <= 1'b0;
            end else if (ce_1hz) begin
                if (sec_cnt == 12'd3599) begin
                    sec_cnt       <= '0;
                    tle_age_hours <= tle_age_hours + 16'd1;
                end else begin
                    sec_cnt <= sec_cnt + 12'd1;
                end
                tle_stale <= (tle_age_hours >= AGE_STALE);
            end
        end
    end

    // =========================================================================
    // sqrt.sv instance (shared between position and velocity computations)
    // =========================================================================
    logic        sqrt_start;
    logic [31:0] sqrt_in;
    logic [31:0] sqrt_out;
    logic        sqrt_done;

    sqrt u_sqrt (
        .clk      (clk),
        .rst_n    (rst_n),
        .start    (sqrt_start),
        .x        (sqrt_in),
        .sqrt_out (sqrt_out),
        .done     (sqrt_done)
    );

    // =========================================================================
    // Magnitude computation FSM
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE,
        S_POS_START,
        S_POS_WAIT,
        S_POS_LATCH,
        S_VEL_START,
        S_VEL_WAIT,
        S_VEL_LATCH
    } state_t;

    state_t state;

    logic [31:0] r_km   [0:2];  // integer km extracted from Q15.16
    logic [31:0] v_q8   [0:2];  // velocity × 256 (v_q8 = eci_vel >> 8)
    logic [31:0] r_sq;
    logic [31:0] v_sq;

    // Latch inputs combinationally
    always_comb begin
        for (int i = 0; i < 3; i++) begin
            r_km[i] = {16'b0, eci_pos[i][31:16]};  // integer km
            v_q8[i] = {8'b0,  eci_vel[i][31:8]};   // v × 256
        end
        // r_sq: fits in 32-bit unsigned for LEO (max ≈ 168 M)
        r_sq = (r_km[0] * r_km[0]) + (r_km[1] * r_km[1]) + (r_km[2] * r_km[2]);
        // v_sq: v_q8² = v² × 65536 → directly usable by sqrt (max ≈ 16 M)
        v_sq = (v_q8[0] * v_q8[0]) + (v_q8[1] * v_q8[1]) + (v_q8[2] * v_q8[2]);
    end

    logic [31:0] pos_mag_q, vel_mag_q;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state                  <= S_IDLE;
            sqrt_start             <= 1'b0;
            sqrt_in                <= '0;
            pos_mag_q              <= '0;
            vel_mag_q              <= '0;
            position_magnitude_km  <= '0;
            velocity_magnitude_kmps <= '0;
            propagator_valid       <= 1'b0;
            overflow_flag          <= 1'b0;
        end else begin
            sqrt_start <= 1'b0;

            case (state)
                S_IDLE: begin
                    // Start a new computation every time ce_1hz fires
                    if (ce_1hz) begin
                        state <= S_POS_START;
                    end
                end

                S_POS_START: begin
                    sqrt_in    <= r_sq;   // raw km², sqrt sees as Q15.16
                    sqrt_start <= 1'b1;
                    state      <= S_POS_WAIT;
                end

                S_POS_WAIT: begin
                    if (sqrt_done)
                        state <= S_POS_LATCH;
                end

                S_POS_LATCH: begin
                    // sqrt_out represents |r|/256 in Q15.16; scale back ×256
                    pos_mag_q <= {sqrt_out[23:0], 8'b0}; // << 8
                    state     <= S_VEL_START;
                end

                S_VEL_START: begin
                    sqrt_in    <= v_sq;   // v_sq = v²×65536 → sqrt gives |v| in Q15.16
                    sqrt_start <= 1'b1;
                    state      <= S_VEL_WAIT;
                end

                S_VEL_WAIT: begin
                    if (sqrt_done)
                        state <= S_VEL_LATCH;
                end

                S_VEL_LATCH: begin
                    vel_mag_q <= sqrt_out; // directly Q15.16 km/s

                    // Commit outputs and check bounds
                    position_magnitude_km   <= pos_mag_q;
                    velocity_magnitude_kmps <= sqrt_out;

                    propagator_valid <= (pos_mag_q >= POS_LO) && (pos_mag_q <= POS_HI) &&
                                        (sqrt_out  >= VEL_LO) && (sqrt_out  <= VEL_HI);
                    overflow_flag    <= (pos_mag_q < POS_LO) || (pos_mag_q > POS_HI) ||
                                        (sqrt_out  < VEL_LO) || (sqrt_out  > VEL_HI);
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule