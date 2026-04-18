// =============================================================================
// File        : uav_axi_regif.sv
// Module      : uav_axi_regif
// Description : UAV MOD_10 - Sensor Calibration + AXI4-Lite Register Interface top.
//               AXI4-Lite slave (32-bit, 256-byte space).
//               50+ control/status registers.
//               Integrates axi_slave, register_file, system_orchestrator.
// =============================================================================

`timescale 1ns/1ps

module module10_uav_axi_regif #(
    parameter int CLK_HZ  = 50_000_000,
    parameter int DATA_W  = 32,
    parameter int ADDR_W  = 8   // 256 bytes
)(
    input  logic clk,
    input  logic rst_n,

    // AXI4-Lite slave port
    input  logic [ADDR_W-1:0] s_awaddr,  input  logic s_awvalid, output logic s_awready,
    input  logic [DATA_W-1:0] s_wdata,   input  logic [3:0] s_wstrb, input  logic s_wvalid,  output logic s_wready,
    output logic [1:0] s_bresp,           output logic s_bvalid,  input  logic s_bready,
    input  logic [ADDR_W-1:0] s_araddr,  input  logic s_arvalid, output logic s_arready,
    output logic [DATA_W-1:0] s_rdata,   output logic [1:0] s_rresp,  output logic s_rvalid,  input  logic s_rready,

    // Status inputs from subsystems
    input  logic [31:0] status_ekf,
    input  logic [31:0] status_gps,
    input  logic [31:0] status_imu,

    // FSM state for orchestrator
    input  logic [7:0] flight_mode,
    input  logic       armed,

    // ---- Decoded register outputs -------------------------------------------
    output logic signed [15:0] pid_gains    [0:7][0:2],
    output logic signed [31:0] pid_integ_max[0:7],
    output logic signed [15:0] mix_coef     [0:3][0:3],
    output logic [31:0]        ekf_q_diag   [0:8],
    output logic signed [31:0] gyro_bias    [0:2],
    output logic signed [31:0] accel_bias   [0:2],
    output logic signed [31:0] mag_offset   [0:2],
    output logic [10:0]        wdt_timeout_ms,
    output logic               wdt_config_en,
    output logic [31:0]        geofence_radius_sq,
    output logic [31:0]        geofence_max_alt,
    output logic [7:0]         check_mask,
    output logic [7:0]         sys_arm_cmd,
    output logic [7:0]         sys_mode_cmd,

    // LED status
    output logic [3:0] led_status
);

    // -------------------------------------------------------------------------
    // AXI slave
    // -------------------------------------------------------------------------
    logic [ADDR_W-1:0] reg_waddr;
    logic [DATA_W-1:0] reg_wdata;
    logic               reg_wen;
    logic [ADDR_W-1:0] reg_raddr;
    logic [DATA_W-1:0] reg_rdata;
    logic               reg_ren;

    axi_slave #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_axi (
        .clk      (clk), .rst_n (rst_n),
        .awaddr   (s_awaddr),  .awvalid (s_awvalid), .awready (s_awready),
        .wdata    (s_wdata),   .wstrb   (s_wstrb),   .wvalid  (s_wvalid),  .wready  (s_wready),
        .bresp    (s_bresp),   .bvalid  (s_bvalid),  .bready  (s_bready),
        .araddr   (s_araddr),  .arvalid (s_arvalid), .arready (s_arready),
        .rdata    (s_rdata),   .rresp   (s_rresp),   .rvalid  (s_rvalid),  .rready  (s_rready),
        .reg_waddr(reg_waddr), .reg_wdata(reg_wdata), .reg_wen(reg_wen),
        .reg_raddr(reg_raddr), .reg_rdata(reg_rdata), .reg_ren(reg_ren)
    );

    // -------------------------------------------------------------------------
    // Register file
    // -------------------------------------------------------------------------
    register_file #(.ADDR_W(ADDR_W), .DATA_W(DATA_W)) u_regs (
        .clk             (clk), .rst_n (rst_n),
        .waddr           (reg_waddr), .wdata (reg_wdata), .wen (reg_wen),
        .raddr           (reg_raddr), .rdata (reg_rdata),
        .sys_arm_cmd     (sys_arm_cmd), .sys_mode_cmd (sys_mode_cmd),
        .pid_gains       (pid_gains),
        .pid_integ_max   (pid_integ_max),
        .ekf_q_diag      (ekf_q_diag),
        .gyro_bias       (gyro_bias),
        .accel_bias      (accel_bias),
        .mag_offset      (mag_offset),
        .wdt_timeout_ms  (wdt_timeout_ms),
        .wdt_config_en   (wdt_config_en),
        .geofence_radius_sq (geofence_radius_sq),
        .geofence_max_alt   (geofence_max_alt),
        .check_mask      (check_mask),
        .mix_coef        (mix_coef),
        .status_ekf      (status_ekf),
        .status_gps      (status_gps),
        .status_imu      (status_imu)
    );

    // -------------------------------------------------------------------------
    // System orchestrator
    // -------------------------------------------------------------------------
    logic [5:0]  preset_addr;
    logic [1:0]  preset_page;
    logic        preset_rd_en;
    logic [31:0] preset_gain_data;
    logic        apply_preset;

    system_orchestrator #(.DATA_W(DATA_W)) u_orch (
        .clk             (clk), .rst_n (rst_n),
        .sys_arm_cmd     (sys_arm_cmd), .sys_mode_cmd (sys_mode_cmd),
        .flight_mode     (flight_mode), .armed (armed),
        .preset_addr     (preset_addr), .preset_page (preset_page),
        .preset_rd_en    (preset_rd_en),
        .preset_gain_data(preset_gain_data),
        .apply_preset    (apply_preset),
        .rst_ekf         (), .rst_pid (), .rst_nav (), .rst_mavlink (),
        .led_status      (led_status)
    );

endmodule
