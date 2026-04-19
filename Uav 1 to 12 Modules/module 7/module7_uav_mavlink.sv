// =============================================================================
// File        : uav_mavlink.sv
// Module      : uav_mavlink
// Description : UAV MOD_7 - MAVLink Protocol top-level.
//               UART RX/TX at 57600 bps from/to ground station.
//               512B async FIFO for RX buffering.
//               MAVLink v2 frame state machine with CRC-16/MCRF4XX.
//               HEARTBEAT monitoring, command dispatching, telemetry mux.
// =============================================================================

`timescale 1ns/1ps

module module7_uav_mavlink #(
    parameter int CLK_HZ   = 50_000_000,
    parameter int MAV_BAUD = 57600,
    parameter int FIFO_DEPTH = 512,
    parameter int DATA_W  = 32
)(
    input  logic clk,
    input  logic rst_n,

    // CE strobes
    input  logic ce_1hz,
    input  logic ce_5hz,
    input  logic ce_10hz,
    input  logic ce_50hz,

    // Physical UART
    input  logic uart_rx,
    output logic uart_tx,

    // EKF state (for telemetry)
    input  logic signed [DATA_W-1:0] ekf_roll,
    input  logic signed [DATA_W-1:0] ekf_pitch,
    input  logic signed [DATA_W-1:0] ekf_yaw,
    input  logic signed [DATA_W-1:0] ekf_lat,
    input  logic signed [DATA_W-1:0] ekf_lon,
    input  logic signed [DATA_W-1:0] ekf_alt,
    input  logic                      ekf_healthy,

    // System status
    input  logic [7:0]  sys_id,
    input  logic [7:0]  comp_id,
    input  logic [7:0]  base_mode,
    input  logic [7:0]  custom_mode,
    input  logic [15:0] sensor_status,

    // Decoded commands (to MOD_8/MOD_9)
    output logic cmd_arm,
    output logic cmd_disarm,
    output logic cmd_takeoff,
    output logic cmd_land,
    output logic cmd_rtl,
    output logic cmd_waypoint,
    output logic cmd_set_mode,
    output logic cmd_reboot,
    output logic [7:0] cmd_mode_val,
    output logic gcs_present,
    output logic gcs_lost
);

    // -------------------------------------------------------------------------
    // UART controller (57600 bps RX/TX)
    // -------------------------------------------------------------------------
    logic [7:0] uart_rx_data;
    logic       uart_rx_valid;
    logic       uart_rx_ready;
    logic [7:0] uart_tx_data;
    logic       uart_tx_valid;
    logic       uart_tx_ready;

    uart_controller #(
        .CLK_HZ    (CLK_HZ),
        .BAUD      (MAV_BAUD),
        .DATA_BITS (8),
        .STOP_BITS (1),
        .PARITY    (0)
    ) u_uart (
        .clk          (clk),
        .rst_n        (rst_n),
        .tx_data      (uart_tx_data),
        .tx_valid     (uart_tx_valid),
        .tx_ready     (uart_tx_ready),
        .uart_tx      (uart_tx),
        .rx_data      (uart_rx_data),
        .rx_valid     (uart_rx_valid),
        .rx_ready     (uart_rx_ready),
        .uart_rx      (uart_rx),
        .rx_frame_err (),
        .rx_overflow  ()
    );

    // -------------------------------------------------------------------------
    // 512-byte RX FIFO
    // -------------------------------------------------------------------------
    logic [7:0] fifo_data;
    logic       fifo_empty, fifo_full;

    async_fifo #(.DATA_W(8), .DEPTH(FIFO_DEPTH)) u_rx_fifo (
        .wr_clk   (clk), .wr_rst_n (rst_n),
        .wr_data  (uart_rx_data),
        .wr_en    (uart_rx_valid && !fifo_full),
        .wr_full  (fifo_full),
        .rd_clk   (clk), .rd_rst_n (rst_n),
        .rd_data  (fifo_data),
        .rd_en    (!fifo_empty),
        .rd_empty (fifo_empty)
    );

    assign uart_rx_ready = !fifo_full;

    // -------------------------------------------------------------------------
    // MAVLink frame parser
    // -------------------------------------------------------------------------
    logic [23:0] msg_id;
    logic [7:0]  payload [0:254];
    logic [7:0]  payload_len;
    logic [7:0]  seq_num;
    logic [7:0]  rx_sys, rx_comp;
    logic        frame_valid;
    logic        crc_error;

    mavlink_frame_parser #(.MAX_PAYLOAD(255), .CLK_HZ(CLK_HZ)) u_parser (
        .clk         (clk),
        .rst_n       (rst_n),
        .rx_byte     (fifo_data),
        .rx_valid    (!fifo_empty),
        .msg_id      (msg_id),
        .payload     (payload),
        .payload_len (payload_len),
        .seq_num     (seq_num),
        .sys_id      (rx_sys),
        .comp_id     (rx_comp),
        .frame_valid (frame_valid),
        .crc_error   (crc_error)
    );

    // -------------------------------------------------------------------------
    // Command dispatcher
    // -------------------------------------------------------------------------
    mavlink_command_dispatcher #(.CLK_HZ(CLK_HZ)) u_dispatch (
        .clk          (clk),
        .rst_n        (rst_n),
        .msg_id       (msg_id),
        .payload      (payload),
        .payload_len  (payload_len),
        .frame_valid  (frame_valid),
        .cmd_arm      (cmd_arm),
        .cmd_disarm   (cmd_disarm),
        .cmd_takeoff  (cmd_takeoff),
        .cmd_land     (cmd_land),
        .cmd_rtl      (cmd_rtl),
        .cmd_waypoint (cmd_waypoint),
        .cmd_set_mode (cmd_set_mode),
        .cmd_reboot   (cmd_reboot),
        .cmd_mode_val (cmd_mode_val),
        .gcs_present  (gcs_present),
        .gcs_lost     (gcs_lost)
    );

    // -------------------------------------------------------------------------
    // Telemetry multiplexer
    // -------------------------------------------------------------------------
    mavlink_telemetry_mux #(.CLK_HZ(CLK_HZ), .DATA_W(DATA_W)) u_tel (
        .clk           (clk),
        .rst_n         (rst_n),
        .ce_1hz        (ce_1hz),
        .ce_5hz        (ce_5hz),
        .ce_10hz       (ce_10hz),
        .ce_50hz       (ce_50hz),
        .sys_id        (sys_id),
        .comp_id       (comp_id),
        .base_mode     (base_mode),
        .custom_mode   (custom_mode),
        .ekf_roll      (ekf_roll),
        .ekf_pitch     (ekf_pitch),
        .ekf_yaw       (ekf_yaw),
        .ekf_lat       (ekf_lat),
        .ekf_lon       (ekf_lon),
        .ekf_alt       (ekf_alt),
        .sensor_status (sensor_status),
        .ekf_healthy   (ekf_healthy),
        .tx_data       (uart_tx_data),
        .tx_valid      (uart_tx_valid),
        .tx_ready      (uart_tx_ready)
    );

endmodule
