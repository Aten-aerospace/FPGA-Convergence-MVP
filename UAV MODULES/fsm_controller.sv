// =============================================================================
// File        : fsm_controller.sv
// Module      : fsm_controller
// Description : 8-state one-hot navigation FSM.
//               States: DISARMED / ARMED / TAKEOFF / EN_ROUTE / LOITER /
//                        LAND / RTL / EMERGENCY
//               Transition guards enforce safety conditions.
//               Generates PID setpoints and mode control signals.
// =============================================================================

`timescale 1ns/1ps

module fsm_controller #(
    parameter int DATA_W  = 32,  // Q4.28 setpoints
    parameter int CLK_HZ  = 50_000_000
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_100hz,   // FSM ticks at 100 Hz

    // MAVLink commands
    input  logic cmd_arm,
    input  logic cmd_disarm,
    input  logic cmd_takeoff,
    input  logic cmd_land,
    input  logic cmd_rtl,
    input  logic cmd_set_mode,
    input  logic [7:0] cmd_mode,

    // Pre-flight / health checks
    input  logic preflight_ok,    // all pre-flight checks passed
    input  logic ekf_healthy,
    input  logic gps_fix,
    input  logic baro_valid,
    input  logic geofence_ok,     // 1 = inside fence
    input  logic wdt_ok,          // 1 = watchdog not expired

    // EKF state (Q4.28)
    input  logic signed [DATA_W-1:0] ekf_alt,
    input  logic signed [DATA_W-1:0] ekf_vd,   // down velocity

    // Waypoint data
    input  logic signed [DATA_W-1:0] wp_alt,
    input  logic signed [DATA_W-1:0] wp_speed,
    input  logic [4:0]               wp_count,
    output logic [4:0]               wp_idx_out,

    // Home position (captured at arm)
    input  logic signed [DATA_W-1:0] home_lat,
    input  logic signed [DATA_W-1:0] home_lon,
    input  logic signed [DATA_W-1:0] home_alt,

    // PID setpoints out (Q4.28)
    output logic signed [DATA_W-1:0] sp_alt,
    output logic signed [DATA_W-1:0] sp_vn,
    output logic signed [DATA_W-1:0] sp_ve,
    output logic signed [DATA_W-1:0] sp_yaw,
    output logic signed [DATA_W-1:0] sp_thrust,
    output logic signed [DATA_W-1:0] sp_roll,
    output logic signed [DATA_W-1:0] sp_pitch,
    output logic signed [DATA_W-1:0] sp_vd,      // descent rate setpoint (m/s Q4.28)

    // Mode / status outputs
    output logic armed,
    output logic [7:0] flight_mode,  // DISARMED=0, ARMED=1, TAKEOFF=2, EN_ROUTE=3,...

    // Watchdog kick (100 Hz)
    output logic wdt_kick
);

    // -------------------------------------------------------------------------
    // One-hot state encoding
    // -------------------------------------------------------------------------
    typedef enum logic [7:0] {
        ST_DISARMED  = 8'b00000001,
        ST_ARMED     = 8'b00000010,
        ST_TAKEOFF   = 8'b00000100,
        ST_EN_ROUTE  = 8'b00001000,
        ST_LOITER    = 8'b00010000,
        ST_LAND      = 8'b00100000,
        ST_RTL       = 8'b01000000,
        ST_EMERGENCY = 8'b10000000
    } fsm_state_t;

    fsm_state_t state;

    localparam signed [DATA_W-1:0] TAKEOFF_ALT = 32'sh05000000; // ~5m Q4.28 approx
    // Ground detect: vD < 0.1 m/s and alt < 0.5 m above home
    localparam signed [DATA_W-1:0] GROUND_VD_THRESH  = 32'sh00199999; // 0.1 m/s Q4.28
    localparam signed [DATA_W-1:0] GROUND_ALT_THRESH = 32'sh00800000; // 0.5 m Q4.28
    // Emergency descent constants
    // 30% thrust Q4.28: 0.30 × 2^28 = 80,530,636 = 32'h04CCCCCD
    localparam signed [DATA_W-1:0] THRUST_30PCT_SP   = 32'sh04CCCCCD;
    // 2 m/s descent rate Q4.28: 2 × 2^28 = 536,870,912 = 32'h20000000
    localparam signed [DATA_W-1:0] DESCENT_RATE_2MPS = 32'sh20000000;

    logic [4:0] wp_idx;

    // -------------------------------------------------------------------------
    // State transitions
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_DISARMED;
            wp_idx     <= '0;
            wdt_kick   <= 1'b0;
        end else if (ce_100hz) begin
            wdt_kick <= 1'b1; // kick watchdog every 100 Hz

            // Emergency override (highest priority)
            if (!wdt_ok || !geofence_ok) begin
                state <= ST_EMERGENCY;
            end else begin
                case (state)
                    ST_DISARMED: begin
                        if (cmd_arm && preflight_ok)
                            state <= ST_ARMED;
                    end
                    ST_ARMED: begin
                        if (cmd_disarm)         state <= ST_DISARMED;
                        else if (cmd_takeoff)   state <= ST_TAKEOFF;
                    end
                    ST_TAKEOFF: begin
                        if (cmd_disarm)         state <= ST_DISARMED;
                        else if (cmd_land)      state <= ST_LAND;
                        // Transition to EN_ROUTE when altitude ramp reached
                        else if (ekf_alt >= (home_alt + TAKEOFF_ALT))
                            state <= (wp_count > 0) ? ST_EN_ROUTE : ST_LOITER;
                    end
                    ST_EN_ROUTE: begin
                        if (cmd_land)           state <= ST_LAND;
                        else if (cmd_rtl)       state <= ST_RTL;
                        else if (cmd_disarm)    state <= ST_DISARMED;
                        else if (wp_idx >= wp_count - 1) state <= ST_LOITER;
                    end
                    ST_LOITER: begin
                        if (cmd_land)           state <= ST_LAND;
                        else if (cmd_rtl)       state <= ST_RTL;
                        else if (cmd_disarm)    state <= ST_DISARMED;
                    end
                    ST_LAND: begin
                        if (cmd_disarm)         state <= ST_DISARMED;
                        // Ground contact: vD ≈ 0 and alt ≈ home
                        else if ((ekf_vd < GROUND_VD_THRESH) &&
                                 ((ekf_alt - home_alt) < GROUND_ALT_THRESH))
                            state <= ST_DISARMED;
                    end
                    ST_RTL: begin
                        if (cmd_disarm)         state <= ST_DISARMED;
                        else if ((ekf_alt - home_alt) < GROUND_ALT_THRESH)
                            state <= ST_LAND;
                    end
                    ST_EMERGENCY: begin
                        // Stay until manually reset
                        if (cmd_disarm)         state <= ST_DISARMED;
                    end
                    default: state <= ST_DISARMED;
                endcase
            end
        end else begin
            wdt_kick <= 1'b0;
        end
    end

    // -------------------------------------------------------------------------
    // Waypoint indexing (advance in EN_ROUTE - simplified: tick every 10s)
    // -------------------------------------------------------------------------
    logic [9:0] wp_dwell_cnt; // 10s × 100Hz = 1000 ticks
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || state != ST_EN_ROUTE) begin
            wp_dwell_cnt <= '0;
            if (state == ST_ARMED || state == ST_DISARMED) wp_idx <= '0;
        end else if (ce_100hz) begin
            if (wp_dwell_cnt == 10'd999) begin
                wp_dwell_cnt <= '0;
                if (wp_idx < wp_count - 1) wp_idx <= wp_idx + 1'b1;
            end else begin
                wp_dwell_cnt <= wp_dwell_cnt + 1'b1;
            end
        end
    end

    assign wp_idx_out = wp_idx;

    // -------------------------------------------------------------------------
    // Setpoint generation per state
    // -------------------------------------------------------------------------
    always_comb begin
        // Default: hold position (all zeroes = hover)
        sp_alt    = home_alt + TAKEOFF_ALT;
        sp_vn     = '0;
        sp_ve     = '0;
        sp_yaw    = '0;
        sp_thrust = 32'sh20000000; // 50% thrust Q4.28
        sp_roll   = '0;
        sp_pitch  = '0;
        armed     = (state != ST_DISARMED);
        flight_mode = 8'h00;

        case (state)
            ST_DISARMED:  begin armed = 1'b0; sp_thrust = '0; flight_mode = 8'd0; end
            ST_ARMED:     begin armed = 1'b1; sp_thrust = '0; flight_mode = 8'd1; end
            ST_TAKEOFF:   begin sp_alt = home_alt + TAKEOFF_ALT; sp_thrust = 32'sh28000000; flight_mode = 8'd2; end
            ST_EN_ROUTE:  begin sp_alt = wp_alt; sp_vn = wp_speed; flight_mode = 8'd3; end
            ST_LOITER:    begin sp_alt = home_alt + TAKEOFF_ALT; flight_mode = 8'd4; end
            ST_LAND:      begin sp_alt = home_alt; sp_thrust = 32'sh10000000; sp_vd = DESCENT_RATE_2MPS; flight_mode = 8'd5; end
            ST_RTL:       begin sp_alt = home_alt + TAKEOFF_ALT; flight_mode = 8'd6; end
            ST_EMERGENCY: begin sp_thrust = THRUST_30PCT_SP; sp_vd = DESCENT_RATE_2MPS; flight_mode = 8'd7; end // 30% thrust, 2 m/s descent
            default: ;
        endcase
    end

    // Unused sp_vd in port list - now an output port

endmodule
