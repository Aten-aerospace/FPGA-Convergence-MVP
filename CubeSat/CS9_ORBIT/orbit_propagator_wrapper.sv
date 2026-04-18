// =============================================================================
// Module: orbit_propagator_wrapper (CS9 top-level orbit wrapper) - enhanced
// Subsystem: CS9 - Orbit Propagator
// Description: Integrates all CS9 orbit subsystem modules.
//   Backward-compatible with the original testbench interface.
//
//   TLE element bus: tle_data[0..5] maps to [n0, e0, i0, raan, argp, m0].
//   n0 is 32-bit Q15.16; the others are 16-bit Q0.15, zero-extended.
//   Alternatively, tle_raw_write + tle_line1/2 triggers the tle_parser path.
//
//   New modules integrated:
//     tle_parser            → parses raw ASCII TLE bytes
//     orbit_state_manager   → MET counter and epoch tracking
//     orbit_health_monitor  → position/velocity bounds checking
//     multi_satellite_tracker → relative motion between 3 satellites
//     ground_track_calculator → ECI→lat/lon/alt conversion
//     contact_window_predictor → elevation angle and contact detection
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module orbit_propagator_wrapper (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_1hz,

    // -------------------------------------------------------------------------
    // Existing (backward-compatible) interface
    // -------------------------------------------------------------------------
    input  logic [31:0] tle_data [0:5],    // [n0, e0, i0, raan, argp, m0]
    input  logic        tle_write,         // pre-parsed element write strobe

    output logic [31:0] eci_pos [0:2],     // Q15.16 km
    output logic [31:0] eci_vel [0:2],     // Q15.16 km/s

    output logic [31:0] lvlh_x,            // r_hat components (Q15.16)
    output logic [31:0] lvlh_y,
    output logic [31:0] lvlh_z,

    output logic        orb_valid,
    output logic        orb_fault,

    // -------------------------------------------------------------------------
    // New inputs
    // -------------------------------------------------------------------------
    input  logic [7:0]  tle_line1 [0:68],  // raw ASCII TLE line-1
    input  logic [7:0]  tle_line2 [0:68],  // raw ASCII TLE line-2
    input  logic        tle_raw_write,     // triggers tle_parser

    input  logic [31:0] met_load_value,    // MET preload value
    input  logic        met_write,         // MET load strobe

    input  logic [31:0] gnd_lat_rad,       // ground station lat Q15.16
    input  logic [31:0] gnd_lon_rad,       // ground station lon Q15.16
    input  logic [31:0] gnd_alt_m,         // ground station altitude Q15.16 m

    input  logic [1:0]  sat_id,            // 0=1v2, 1=1v3, 2=2v3
    input  logic [31:0] sat2_pos [0:2],    // Q15.16 km
    input  logic [31:0] sat2_vel [0:2],    // Q15.16 km/s
    input  logic [31:0] sat3_pos [0:2],
    input  logic [31:0] sat3_vel [0:2],

    // -------------------------------------------------------------------------
    // New outputs
    // -------------------------------------------------------------------------
    output logic [31:0] orbital_elements [0:5],   // [SMA,e,i,Ω,ω,ν] Q15.16
    output logic [15:0] true_anomaly,              // Q0.15 rad

    output logic [31:0] lvlh_matrix [0:8],         // full LVLH rotation matrix

    output logic [31:0] met_counter,               // seconds since MET epoch
    output logic [31:0] epoch_tracked,             // current TLE epoch MJD

    output logic [31:0] latitude_rad,              // Q15.16
    output logic [31:0] longitude_rad,             // Q15.16
    output logic [31:0] altitude_m,                // Q15.16 m
    output logic        ground_track_valid,

    output logic [15:0] elevation_angle_deg,       // Q8.8
    output logic        contact_valid,
    output logic [15:0] aos_predicted_secs,
    output logic [15:0] los_predicted_secs,

    output logic [31:0] delta_r_eci [0:2],         // Q15.16 km
    output logic [31:0] delta_v_eci [0:2],         // Q15.16 km/s
    output logic [31:0] separation_km,             // Q15.16 km

    output logic        propagator_valid,
    output logic        overflow_flag,
    output logic [15:0] tle_age_hours,
    output logic        tle_stale,
    output logic        tle_checksum_ok,
    output logic [31:0] position_magnitude_km,
    output logic [31:0] velocity_magnitude_kmps
);

    // =========================================================================
    // TLE parser (raw ASCII path)
    // =========================================================================
    logic [31:0] parsed_n0;
    logic [15:0] parsed_e0, parsed_i0, parsed_raan, parsed_argp, parsed_m0;
    logic [31:0] parsed_epoch_mjd;
    logic        parsed_valid;

    tle_parser u_tle_parser (
        .clk             (clk),
        .rst_n           (rst_n),
        .tle_line1       (tle_line1),
        .tle_line2       (tle_line2),
        .tle_write       (tle_raw_write),
        .tle_n0          (parsed_n0),
        .tle_e0          (parsed_e0),
        .tle_i0          (parsed_i0),
        .tle_raan        (parsed_raan),
        .tle_argp        (parsed_argp),
        .tle_m0          (parsed_m0),
        .tle_epoch_mjd   (parsed_epoch_mjd),
        .tle_checksum_ok (tle_checksum_ok),
        .tle_parsed_valid(parsed_valid)
    );

    // =========================================================================
    // TLE mux: raw-parsed path takes priority over pre-parsed bus
    // =========================================================================
    logic [31:0] sgp4_n0;
    logic [15:0] sgp4_e0, sgp4_i0, sgp4_raan, sgp4_argp, sgp4_m0;
    logic        sgp4_write;

    always_comb begin
        if (parsed_valid) begin
            sgp4_n0    = parsed_n0;
            sgp4_e0    = parsed_e0;
            sgp4_i0    = parsed_i0;
            sgp4_raan  = parsed_raan;
            sgp4_argp  = parsed_argp;
            sgp4_m0    = parsed_m0;
            sgp4_write = 1'b1;
        end else begin
            sgp4_n0    = tle_data[0];
            sgp4_e0    = tle_data[1][15:0];
            sgp4_i0    = tle_data[2][15:0];
            sgp4_raan  = tle_data[3][15:0];
            sgp4_argp  = tle_data[4][15:0];
            sgp4_m0    = tle_data[5][15:0];
            sgp4_write = tle_write;
        end
    end

    // =========================================================================
    // SGP4-lite instance
    // =========================================================================
    logic [31:0] eci_pos_int [0:2];
    logic [31:0] eci_vel_int [0:2];
    logic        orb_valid_int;

    sgp4_lite u_sgp4 (
        .clk            (clk),
        .rst_n          (rst_n),
        .ce_1hz         (ce_1hz),
        .tle_n0         (sgp4_n0),
        .tle_e0         (sgp4_e0),
        .tle_i0         (sgp4_i0),
        .tle_raan       (sgp4_raan),
        .tle_argp       (sgp4_argp),
        .tle_m0         (sgp4_m0),
        .tle_write      (sgp4_write),
        .eci_pos        (eci_pos_int),
        .eci_vel        (eci_vel_int),
        .orb_valid      (orb_valid_int),
        .true_anomaly   (true_anomaly),
        .orbital_elements(orbital_elements)
    );

    // =========================================================================
    // LVLH converter (enhanced)
    // =========================================================================
    logic [31:0] h_hat_x_w, h_hat_y_w, h_hat_z_w;
    logic [31:0] t_hat_x_w, t_hat_y_w, t_hat_z_w;

    lvlh_converter u_lvlh (
        .clk         (clk),
        .rst_n       (rst_n),
        .eci_pos     (eci_pos_int),
        .eci_vel     (eci_vel_int),
        .eci_valid   (orb_valid_int),
        .lvlh_x      (lvlh_x),
        .lvlh_y      (lvlh_y),
        .lvlh_z      (lvlh_z),
        .lvlh_valid  (orb_valid),
        .h_hat_x     (h_hat_x_w),
        .h_hat_y     (h_hat_y_w),
        .h_hat_z     (h_hat_z_w),
        .t_hat_x     (t_hat_x_w),
        .t_hat_y     (t_hat_y_w),
        .t_hat_z     (t_hat_z_w),
        .lvlh_matrix (lvlh_matrix)
    );

    // =========================================================================
    // Drive ECI outputs
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) begin
                eci_pos[i] <= '0;
                eci_vel[i] <= '0;
            end
        end else if (orb_valid_int) begin
            for (int i = 0; i < 3; i++) begin
                eci_pos[i] <= eci_pos_int[i];
                eci_vel[i] <= eci_vel_int[i];
            end
        end
    end

    // =========================================================================
    // Orbit state manager
    // =========================================================================
    orbit_state_manager u_osm (
        .clk           (clk),
        .rst_n         (rst_n),
        .ce_1hz        (ce_1hz),
        .met_load_value(met_load_value),
        .met_write     (met_write),
        .epoch_mjd     (parsed_epoch_mjd),
        .epoch_write   (parsed_valid),
        .met_counter   (met_counter),
        .epoch_tracked (epoch_tracked)
    );

    // =========================================================================
    // Orbit health monitor
    // =========================================================================
    logic tle_any_write;
    assign tle_any_write = tle_write | parsed_valid;

    orbit_health_monitor u_ohm (
        .clk                   (clk),
        .rst_n                 (rst_n),
        .eci_pos               (eci_pos_int),
        .eci_vel               (eci_vel_int),
        .tle_write             (tle_any_write),
        .ce_1hz                (ce_1hz),
        .propagator_valid      (propagator_valid),
        .overflow_flag         (overflow_flag),
        .tle_age_hours         (tle_age_hours),
        .tle_stale             (tle_stale),
        .position_magnitude_km (position_magnitude_km),
        .velocity_magnitude_kmps(velocity_magnitude_kmps)
    );

    // =========================================================================
    // Multi-satellite tracker (sat1 = locally propagated)
    // =========================================================================
    multi_satellite_tracker u_mst (
        .clk              (clk),
        .rst_n            (rst_n),
        .sat1_pos         (eci_pos_int),
        .sat1_vel         (eci_vel_int),
        .sat2_pos         (sat2_pos),
        .sat2_vel         (sat2_vel),
        .sat3_pos         (sat3_pos),
        .sat3_vel         (sat3_vel),
        .sat_id           (sat_id),
        .compute_valid    (orb_valid_int),
        .delta_r_eci      (delta_r_eci),
        .delta_v_eci      (delta_v_eci),
        .separation_km    (separation_km),
        .closure_rate_kmps(), // not exposed at top level
        .relative_valid   ()
    );

    // =========================================================================
    // Ground track calculator
    // =========================================================================
    ground_track_calculator u_gtc (
        .clk               (clk),
        .rst_n             (rst_n),
        .eci_pos           (eci_pos_int),
        .calc_valid        (orb_valid_int),
        .latitude_rad      (latitude_rad),
        .longitude_rad     (longitude_rad),
        .altitude_m        (altitude_m),
        .ground_track_valid(ground_track_valid)
    );

    // =========================================================================
    // Contact window predictor
    // =========================================================================
    contact_window_predictor u_cwp (
        .clk                (clk),
        .rst_n              (rst_n),
        .latitude_rad       (latitude_rad),
        .longitude_rad      (longitude_rad),
        .altitude_m         (altitude_m),
        .gnd_lat_rad        (gnd_lat_rad),
        .gnd_lon_rad        (gnd_lon_rad),
        .gnd_alt_m          (gnd_alt_m),
        .calc_valid         (ground_track_valid),
        .elevation_angle_deg(elevation_angle_deg),
        .los_imminent       (),
        .aos_predicted_secs (aos_predicted_secs),
        .los_predicted_secs (los_predicted_secs),
        .contact_valid      (contact_valid)
    );

    // =========================================================================
    // Fault: orb_valid not seen within 3 seconds after tle_write
    // =========================================================================
    logic [1:0] fault_cnt;
    logic       tle_written;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fault_cnt   <= '0;
            orb_fault   <= 1'b0;
            tle_written <= 1'b0;
        end else begin
            if (tle_write || parsed_valid) begin
                tle_written <= 1'b1;
                fault_cnt   <= '0;
                orb_fault   <= 1'b0;
            end else if (orb_valid_int) begin
                fault_cnt   <= '0;
                orb_fault   <= 1'b0;
            end else if (tle_written && ce_1hz) begin
                if (fault_cnt == 2'd3)
                    orb_fault <= 1'b1;
                else
                    fault_cnt <= fault_cnt + 1;
            end
        end
    end

endmodule