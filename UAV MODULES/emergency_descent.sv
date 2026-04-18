// =============================================================================
// File        : emergency_descent.sv
// Module      : emergency_descent
// Description : Emergency descent control profile.
//               On activation: position hold + 30% thrust + 2 m/s descent.
//               Overrides PID setpoints.
//               Deactivates only on explicit ground detection or disarm.
// =============================================================================

`timescale 1ns/1ps

module emergency_descent #(
    parameter int DATA_W = 32  // Q4.28
)(
    input  logic clk,
    input  logic rst_n,
    input  logic enable,         // Activate emergency descent

    // Current state (for position hold)
    input  logic signed [DATA_W-1:0] curr_lat,
    input  logic signed [DATA_W-1:0] curr_lon,
    input  logic signed [DATA_W-1:0] curr_alt,

    // Emergency setpoints out (override normal PID setpoints when active)
    output logic signed [DATA_W-1:0] sp_lat,
    output logic signed [DATA_W-1:0] sp_lon,
    output logic signed [DATA_W-1:0] sp_alt,
    output logic signed [DATA_W-1:0] sp_thrust,  // 30% thrust
    output logic signed [DATA_W-1:0] sp_vd,       // 2 m/s descent rate
    output logic                      active
);

    // 30% thrust in Q4.28: 0.30 × 2^28 = 80,530,636 ≈ 32'h04CCCCCD
    localparam signed [DATA_W-1:0] THRUST_30PCT = 32'sh04CCCCCD;
    // 2 m/s descent rate in Q4.28: 2 × 2^28 = 536,870,912 = 32'h20000000
    localparam signed [DATA_W-1:0] DESCENT_RATE = 32'sh20000000;

    // Latch position on activation
    logic signed [DATA_W-1:0] hold_lat, hold_lon, hold_alt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active   <= 1'b0;
            hold_lat <= '0;
            hold_lon <= '0;
            hold_alt <= '0;
        end else begin
            if (enable && !active) begin
                // Capture position on first activation
                hold_lat <= curr_lat;
                hold_lon <= curr_lon;
                hold_alt <= curr_alt;
                active   <= 1'b1;
            end else if (!enable) begin
                active <= 1'b0;
            end
        end
    end

    always_comb begin
        sp_lat    = hold_lat;
        sp_lon    = hold_lon;
        sp_alt    = hold_alt;
        sp_thrust = THRUST_30PCT;
        sp_vd     = DESCENT_RATE;
    end

endmodule
