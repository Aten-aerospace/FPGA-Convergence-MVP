// =============================================================================
// File        : waypoint_manager.sv
// Module      : waypoint_manager
// Description : 32-waypoint BRAM manager for MAVLink-compatible mission.
//               Each waypoint: 34 bytes (lat Q10.22, lon Q10.22, alt Q10.22,
//                 acceptance_radius, speed_ms, command, frame, autocontinue, etc.)
//               Supports: WP read (FSM), mission write (MAVLink).
//               Priority: WP read > mission write.
// =============================================================================

`timescale 1ns/1ps

module waypoint_manager #(
    parameter int MAX_WP    = 32,
    parameter int WP_W      = 32,   // width per field (Q10.22)
    parameter int ADDR_W    = $clog2(MAX_WP)
)(
    input  logic clk,
    input  logic rst_n,

    // Navigation read interface
    input  logic [ADDR_W-1:0]  wp_idx,       // active waypoint index
    output logic signed [WP_W-1:0] wp_lat,
    output logic signed [WP_W-1:0] wp_lon,
    output logic signed [WP_W-1:0] wp_alt,
    output logic [WP_W-1:0]    wp_radius,    // acceptance radius (m Q16.16)
    output logic [WP_W-1:0]    wp_speed,     // target speed (m/s Q4.28)
    output logic [7:0]         wp_cmd,       // MAVLink command
    output logic               wp_valid,

    // Mission write interface (from MAVLink dispatcher)
    input  logic               wr_en,
    input  logic [ADDR_W-1:0]  wr_idx,
    input  logic signed [WP_W-1:0] wr_lat,
    input  logic signed [WP_W-1:0] wr_lon,
    input  logic signed [WP_W-1:0] wr_alt,
    input  logic [WP_W-1:0]    wr_radius,
    input  logic [WP_W-1:0]    wr_speed,
    input  logic [7:0]         wr_cmd,

    // Total waypoints in mission
    output logic [ADDR_W-1:0]  wp_count
);

    // -------------------------------------------------------------------------
    // BRAM storage (inferred dual-port)
    // -------------------------------------------------------------------------
    logic signed [WP_W-1:0] bram_lat    [0:MAX_WP-1];
    logic signed [WP_W-1:0] bram_lon    [0:MAX_WP-1];
    logic signed [WP_W-1:0] bram_alt    [0:MAX_WP-1];
    logic [WP_W-1:0]         bram_radius [0:MAX_WP-1];
    logic [WP_W-1:0]         bram_speed  [0:MAX_WP-1];
    logic [7:0]              bram_cmd    [0:MAX_WP-1];

    logic [ADDR_W-1:0] wp_cnt_reg;

    // Write port
    always_ff @(posedge clk) begin
        if (wr_en) begin
            bram_lat[wr_idx]    <= wr_lat;
            bram_lon[wr_idx]    <= wr_lon;
            bram_alt[wr_idx]    <= wr_alt;
            bram_radius[wr_idx] <= wr_radius;
            bram_speed[wr_idx]  <= wr_speed;
            bram_cmd[wr_idx]    <= wr_cmd;
            if (wr_idx >= wp_cnt_reg)
                wp_cnt_reg <= wr_idx + 1'b1;
        end
    end

    // Read port (registered for BRAM timing)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wp_lat    <= '0; wp_lon    <= '0; wp_alt    <= '0;
            wp_radius <= '0; wp_speed  <= '0; wp_cmd    <= '0;
            wp_valid  <= 1'b0; wp_cnt_reg <= '0;
        end else begin
            wp_lat    <= bram_lat[wp_idx];
            wp_lon    <= bram_lon[wp_idx];
            wp_alt    <= bram_alt[wp_idx];
            wp_radius <= bram_radius[wp_idx];
            wp_speed  <= bram_speed[wp_idx];
            wp_cmd    <= bram_cmd[wp_idx];
            wp_valid  <= 1'b1;
        end
    end

    assign wp_count = wp_cnt_reg;

    // Initialise BRAM to safe values (synthesis: will be mapped to init data)
    initial begin
        for (int i = 0; i < MAX_WP; i++) begin
            bram_lat[i]    = '0;
            bram_lon[i]    = '0;
            bram_alt[i]    = 32'sh00001900; // 25 m default altitude
            bram_radius[i] = 32'sh00050000; // 5 m acceptance radius
            bram_speed[i]  = 32'sh01000000; // 1 m/s
            bram_cmd[i]    = 8'd16;         // MAV_CMD_NAV_WAYPOINT
        end
        wp_cnt_reg = '0;
    end

endmodule
