// =============================================================================
// File        : geofence_checker.sv
// Module      : geofence_checker
// Description : Cylindrical geofence check.
//               Asserts geofence_violation when UAV exceeds:
//                 - horizontal radius from home (configurable)
//                 - altitude limit (configurable)
//               Uses squared distance to avoid sqrt (compare dist² vs radius²).
// =============================================================================

`timescale 1ns/1ps

module geofence_checker #(
    parameter int POS_W = 32   // Q10.22 position
)(
    input  logic clk,
    input  logic rst_n,
    input  logic enable,

    // Current position (Q10.22)
    input  logic signed [POS_W-1:0] curr_lat,
    input  logic signed [POS_W-1:0] curr_lon,
    input  logic signed [POS_W-1:0] curr_alt,

    // Home / geofence origin (Q10.22)
    input  logic signed [POS_W-1:0] home_lat,
    input  logic signed [POS_W-1:0] home_lon,
    input  logic signed [POS_W-1:0] home_alt,

    // Limits
    input  logic [POS_W-1:0] max_radius_sq,  // max horizontal dist² (Q10.22²)
    input  logic [POS_W-1:0] max_alt,         // max altitude above home (Q10.22)

    // Status
    output logic geofence_violation,
    output logic valid
);

    logic signed [POS_W-1:0] dlat, dlon, dalt;
    logic signed [63:0]       dist_sq;
    logic                     v1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            dlat <= '0; dlon <= '0; dalt <= '0;
            dist_sq <= '0; v1 <= 1'b0;
        end else begin
            v1   <= enable;
            dlat <= curr_lat - home_lat;
            dlon <= curr_lon - home_lon;
            dalt <= curr_alt - home_alt;
            dist_sq <= (dlat * dlat) + (dlon * dlon);
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            geofence_violation <= 1'b0;
            valid              <= 1'b0;
        end else begin
            valid              <= v1;
            geofence_violation <= v1 &&
                ((dist_sq > $signed({1'b0, max_radius_sq})) ||
                 (dalt    > $signed({1'b0, max_alt})));
        end
    end

endmodule
