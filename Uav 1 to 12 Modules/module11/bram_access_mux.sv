// =============================================================================
// File        : bram_access_mux.sv
// Module      : bram_access_mux
// Description : BRAM dual-port arbitration for shared resources.
//               Arbitrates access to:
//                 - EKF state/covariance BRAM (EKF predict > GPS update > baro)
//                 - Waypoint BRAM (WP read > mission write)
//               Uses round-robin with fixed priority for time-critical paths.
// =============================================================================

`timescale 1ns/1ps

module bram_access_mux #(
    parameter int DATA_W = 32,
    parameter int ADDR_W = 10   // covers 9 states × 2 (state+cov) + margins
)(
    input  logic clk,
    input  logic rst_n,

    // ---- EKF state BRAM ports -----------------------------------------------
    // Port A: EKF predict writer (highest priority)
    input  logic               ekf_pred_wr_en,
    input  logic [ADDR_W-1:0]  ekf_pred_wr_addr,
    input  logic [DATA_W-1:0]  ekf_pred_wr_data,

    // Port B: EKF update reader (GPS/baro/mag)
    input  logic               ekf_upd_rd_en,
    input  logic [ADDR_W-1:0]  ekf_upd_rd_addr,
    output logic [DATA_W-1:0]  ekf_upd_rd_data,

    // Port C: PID/NAV reader (lowest priority)
    input  logic               nav_rd_en,
    input  logic [ADDR_W-1:0]  nav_rd_addr,
    output logic [DATA_W-1:0]  nav_rd_data,

    // ---- Waypoint BRAM ports -------------------------------------------------
    // Port A: Navigation reader (high priority)
    input  logic               wp_nav_rd_en,
    input  logic [ADDR_W-1:0]  wp_nav_rd_addr,
    output logic [DATA_W-1:0]  wp_nav_rd_data,

    // Port B: Mission writer (low priority)
    input  logic               wp_mis_wr_en,
    input  logic [ADDR_W-1:0]  wp_mis_wr_addr,
    input  logic [DATA_W-1:0]  wp_mis_wr_data,

    // Grant signals (for diagnostics)
    output logic ekf_grant_pred,
    output logic ekf_grant_upd,
    output logic ekf_grant_nav,
    output logic wp_grant_nav,
    output logic wp_grant_mis
);

    // -------------------------------------------------------------------------
    // EKF state BRAM (simple dual-port, 1K × 32b)
    // -------------------------------------------------------------------------
    logic [DATA_W-1:0] ekf_bram [0:(1<<ADDR_W)-1];

    // Priority arbitration: predict > update > nav
    // This is synchronous access; NAV read delayed by 1 cycle if conflict
    logic ekf_pred_busy;
    assign ekf_pred_busy    = ekf_pred_wr_en;
    assign ekf_grant_pred   = ekf_pred_wr_en;
    assign ekf_grant_upd    = ekf_upd_rd_en && !ekf_pred_busy;
    assign ekf_grant_nav    = nav_rd_en      && !ekf_pred_busy && !ekf_upd_rd_en;

    always_ff @(posedge clk) begin
        if (ekf_pred_wr_en)
            ekf_bram[ekf_pred_wr_addr] <= ekf_pred_wr_data;
        if (ekf_grant_upd)
            ekf_upd_rd_data <= ekf_bram[ekf_upd_rd_addr];
        if (ekf_grant_nav)
            nav_rd_data     <= ekf_bram[nav_rd_addr];
    end

    // -------------------------------------------------------------------------
    // Waypoint BRAM (simple dual-port, 1K × 32b)
    // -------------------------------------------------------------------------
    logic [DATA_W-1:0] wp_bram [0:(1<<ADDR_W)-1];

    // Priority: nav_read > mission_write
    assign wp_grant_nav = wp_nav_rd_en;
    assign wp_grant_mis = wp_mis_wr_en && !wp_nav_rd_en;

    always_ff @(posedge clk) begin
        if (wp_grant_mis)
            wp_bram[wp_mis_wr_addr] <= wp_mis_wr_data;
        if (wp_grant_nav)
            wp_nav_rd_data <= wp_bram[wp_nav_rd_addr];
    end

    // -------------------------------------------------------------------------
    // Initialise BRAMs to known state
    // -------------------------------------------------------------------------
    initial begin
        for (int i = 0; i < (1<<ADDR_W); i++) begin
            ekf_bram[i] = '0;
            wp_bram[i]  = '0;
        end
        ekf_upd_rd_data = '0;
        nav_rd_data     = '0;
        wp_nav_rd_data  = '0;
    end

endmodule
