// =============================================================================
// File        : uav_gps_interface.sv
// Module      : uav_gps_interface
// Description : UAV MOD_6 - GPS Receiver Interface top.
//               UART RX at 57600 bps (NEO-M9N).
//               512-byte async FIFO for frame buffering.
//               NMEA 0183 parser for GGA + VTG sentences.
//               Outputs position, velocity, fix flag (lat/lon/alt Q10.22,
//               vel Q4.28, fix_type, HDOP, gps_fix).
// =============================================================================

`timescale 1ns/1ps

module module6_uav_gps_interface #(
    parameter int CLK_HZ   = 50_000_000,
    parameter int GPS_BAUD = 57600,
    parameter int FIFO_DEPTH = 512
)(
    input  logic clk,
    input  logic rst_n,

    // UART physical interface (NEO-M9N GPS)
    input  logic uart_rx,

    // Parsed GPS outputs
    output logic signed [31:0] gps_lat,
    output logic signed [31:0] gps_lon,
    output logic signed [31:0] gps_alt,
    output logic signed [31:0] gps_vel_n,
    output logic signed [31:0] gps_vel_e,
    output logic [3:0]         gps_fix_type,
    output logic [15:0]        gps_hdop,
    output logic               gps_fix,
    output logic               gps_data_valid
);

    // -------------------------------------------------------------------------
    // UART receiver (reuse uart_controller.sv, TX unused for GPS-only)
    // -------------------------------------------------------------------------
    logic [7:0] uart_rx_data;
    logic       uart_rx_valid;
    logic       uart_rx_ready;
    logic       uart_frame_err, uart_overflow;
    logic       uart_tx_dummy;

    uart_controller #(
        .CLK_HZ    (CLK_HZ),
        .BAUD      (GPS_BAUD),
        .DATA_BITS (8),
        .STOP_BITS (1),
        .PARITY    (0)
    ) u_uart (
        .clk          (clk),
        .rst_n        (rst_n),
        .tx_data      (8'h00),
        .tx_valid     (1'b0),
        .tx_ready     (),
        .uart_tx      (uart_tx_dummy),
        .rx_data      (uart_rx_data),
        .rx_valid     (uart_rx_valid),
        .rx_ready     (uart_rx_ready),
        .uart_rx      (uart_rx),
        .rx_frame_err (uart_frame_err),
        .rx_overflow  (uart_overflow)
    );

    // -------------------------------------------------------------------------
    // 512-byte async FIFO (clk domain is same here; single-clock FIFO usage)
    // -------------------------------------------------------------------------
    logic [7:0] fifo_rd_data;
    logic       fifo_rd_en;
    logic       fifo_rd_empty;
    logic       fifo_wr_full;

    async_fifo #(
        .DATA_W (8),
        .DEPTH  (FIFO_DEPTH)
    ) u_fifo (
        .wr_clk   (clk),
        .wr_rst_n (rst_n),
        .wr_data  (uart_rx_data),
        .wr_en    (uart_rx_valid && !fifo_wr_full),
        .wr_full  (fifo_wr_full),
        .rd_clk   (clk),
        .rd_rst_n (rst_n),
        .rd_data  (fifo_rd_data),
        .rd_en    (fifo_rd_en),
        .rd_empty (fifo_rd_empty)
    );

    // Feed UART output to FIFO; drain FIFO into parser
    assign uart_rx_ready = !fifo_wr_full;
    assign fifo_rd_en    = !fifo_rd_empty;

    // -------------------------------------------------------------------------
    // NMEA sentence parser
    // -------------------------------------------------------------------------
    nmea_sentence_parser u_parser (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx_byte    (fifo_rd_data),
        .rx_valid   (!fifo_rd_empty),
        .rx_ready   (),  // not used (FIFO manages flow)
        .lat        (gps_lat),
        .lon        (gps_lon),
        .alt_msl    (gps_alt),
        .vel_n      (gps_vel_n),
        .vel_e      (gps_vel_e),
        .fix_type   (gps_fix_type),
        .hdop_q8    (gps_hdop),
        .gps_fix    (gps_fix),
        .data_valid (gps_data_valid)
    );

endmodule
