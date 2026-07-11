// =============================================================================
// File        : uav_top.sv
// Module      : uav_top
// Description : UAV MOD_12 - Top-Level System Integration 
//               MOD_4 EKF → MOD_5 sensor fusion (active dataflow)
//               MOD_2 PID → MOD_3 motor commands (active dataflow)
// =============================================================================

`timescale 1ns/1ps

module module12_uav_top #(
    parameter int CLK_HZ   = 50_000_000,
    parameter int DATA_W   = 32,   
    parameter int STATE_W  = 32,   
    parameter int P_W      = 32,   
    parameter int STATES   = 9
)(
    input  logic        clk_in,
    input  logic        pll_locked,
    input  logic        ext_rst_n,
    input  logic        arm_btn,
    input  logic        mode_btn,
    output logic        imu_sclk,
    output logic        imu_mosi,
    input  logic        imu_miso,
    output logic        imu_cs_n,
    output logic        i2c_scl,
    inout  wire         i2c_sda,
    input  logic        gps_rx,
    input  logic        mav_rx,
    output logic        mav_tx,
    output logic [3:0]  pwm_out,
    input  logic [7:0]  s_axi_awaddr,
    input  logic        s_axi_awvalid,
    output logic        s_axi_awready,
    input  logic [31:0] s_axi_wdata,
    input  logic [3:0]  s_axi_wstrb,
    input  logic        s_axi_wvalid,
    output logic        s_axi_wready,
    output logic [1:0]  s_axi_bresp,
    output logic        s_axi_bvalid,
    input  logic        s_axi_bready,
    input  logic [7:0]  s_axi_araddr,
    input  logic        s_axi_arvalid,
    output logic        s_axi_arready,
    output logic [31:0] s_axi_rdata,
    output logic [1:0]  s_axi_rresp,
    output logic        s_axi_rvalid,
    input  logic        s_axi_rready,
    output logic [3:0]  led
);

    // =========================================================================
    // Internal wires / buses
    // =========================================================================
    
    logic clk;
    assign clk = clk_in;

    logic rst_n;
    logic pll_lock_stable;
    logic arm_btn_sync, mode_sync;

    logic ce_1khz_raw, ce_100hz_raw, ce_50hz_raw, ce_10hz_raw;
    logic ce_pid_inner, ce_pid_outer, ce_ekf_predict;
    logic ce_baro_upd, ce_gps_upd, ce_mag_upd;
    logic ce_nav, ce_1hz, ce_5hz, ce_50hz, ce_10hz;

    logic signed [STATE_W-1:0] ekf_state_pred [0:STATES-1];
    logic signed [STATE_W-1:0] ekf_state_upd  [0:STATES-1];
    logic signed [STATE_W-1:0] ekf_state_pid  [0:STATES-1];
    logic signed [STATE_W-1:0] ekf_state_nav  [0:STATES-1];
    logic signed [P_W-1:0]     p_diag_pred    [0:STATES-1];
    logic signed [P_W-1:0]     p_diag_upd     [0:STATES-1];
    logic                       ekf_pred_valid;
    logic                       ekf_upd_valid;

    logic signed [DATA_W-1:0] sp_roll, sp_pitch, sp_yaw;
    logic signed [DATA_W-1:0] sp_alt, sp_vn, sp_ve, sp_thrust;

    logic signed [DATA_W-1:0] roll_rate_cmd, pitch_rate_cmd, yaw_rate_cmd, thrust_cmd;
    logic signed [15:0] roll_rate_cmd_16, pitch_rate_cmd_16, yaw_rate_cmd_16, thrust_cmd_16;

    logic signed [DATA_W-1:0] nav_sp_roll, nav_sp_pitch, nav_sp_yaw;
    logic signed [DATA_W-1:0] nav_sp_alt, nav_sp_vn, nav_sp_ve, nav_sp_thrust;
    logic        armed, wdt_kick;
    logic [7:0]  flight_mode;

    logic signed [31:0] gps_lat, gps_lon, gps_alt, gps_vel_n, gps_vel_e;
    logic [3:0]          gps_fix_type;
    logic [15:0]         gps_hdop;
    logic                gps_fix, gps_data_valid;

    logic cmd_arm, cmd_disarm, cmd_takeoff, cmd_land, cmd_rtl;
    logic cmd_waypoint, cmd_set_mode, cmd_reboot;
    logic [7:0] cmd_mode_val;
    logic gcs_present, gcs_lost;

    logic wdt_ok, wdt_expired, preflight_ok;
    logic [7:0] fault_flags;
    logic emerg_active;
    logic signed [DATA_W-1:0] emerg_sp_lat, emerg_sp_lon, emerg_sp_alt;
    logic signed [DATA_W-1:0] emerg_sp_thrust, emerg_sp_vd;
    logic baro_valid, mag_valid_flag;
    logic geofence_violation;
    logic emergency;

    // Register file outputs (from MOD_10)
    logic signed [15:0] pid_gains     [0:7][0:2];
    logic signed [31:0] pid_integ_max [0:7];
    logic signed [15:0] mix_coef      [0:3][0:3];
    logic [31:0]         ekf_q_diag   [0:8];
    logic signed [31:0] gyro_bias     [0:2];
    logic signed [31:0] accel_bias    [0:2];
    logic signed [31:0] mag_offset    [0:2];
    logic [10:0]         wdt_timeout_ms;
    logic                wdt_config_en;
    logic [31:0]         geofence_radius_sq, geofence_max_alt;
    logic [7:0]          check_mask;
    logic [7:0]          sys_arm_cmd, sys_mode_cmd;

    logic rst_n_clk, rst_n_sync, rst_n_ekf, rst_n_sensors;
    logic rst_n_gps, rst_n_mavlink, rst_n_nav, rst_n_wdt, rst_n_axi;

    logic signed [DATA_W-1:0] home_lat, home_lon, home_alt;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            home_lat <= '0; home_lon <= '0; home_alt <= '0;
        end else if (cmd_arm && preflight_ok && !armed) begin
            home_lat <= ekf_state_upd[6];
            home_lon <= ekf_state_upd[7];
            home_alt <= ekf_state_upd[8];
        end
    end

    // =========================================================================
    // Conversion signals for MOD_7 (16-bit)
    // =========================================================================
    logic signed [15:0] mav_ekf_roll, mav_ekf_pitch, mav_ekf_yaw;
    logic signed [15:0] mav_ekf_lat, mav_ekf_lon, mav_ekf_alt;

    always_comb begin
        mav_ekf_roll  = ekf_state_upd[0][15:0];
        mav_ekf_pitch = ekf_state_upd[1][15:0];
        mav_ekf_yaw   = ekf_state_upd[2][15:0];
        mav_ekf_lat   = ekf_state_upd[6][15:0];
        mav_ekf_lon   = ekf_state_upd[7][15:0];
        mav_ekf_alt   = ekf_state_upd[8][15:0];
    end

    // =========================================================================
    // MOD_1: System Clock & Control Strobes
    // =========================================================================
    module1_uav_clk_ctrl #(.CLK_HZ(CLK_HZ)) u_mod1 (
        .clk             (clk),
        .pll_locked      (pll_locked),
        .ext_rst_n       (ext_rst_n),
        .arm_btn_async   (arm_btn),
        .mode_async      (mode_btn),
        .rst_n           (rst_n),
        .arm_btn_sync    (arm_btn_sync),
        .mode_sync       (mode_sync),
        .ce_1khz         (ce_1khz_raw),
        .ce_100hz        (ce_100hz_raw),
        .ce_50hz         (ce_50hz_raw),
        .ce_10hz         (ce_10hz_raw),
        .pll_lock_stable (pll_lock_stable)
    );

    // =========================================================================
    // MOD_11: Interconnect (CE arbiter + BRAM mux)
    // =========================================================================
    module11_uav_interconnect #(.DATA_W(DATA_W)) u_mod11 (
        .clk            (clk), .rst_n (rst_n),
        .ce_1khz_raw    (ce_1khz_raw),
        .ce_100hz_raw   (ce_100hz_raw),
        .ce_50hz_raw    (ce_50hz_raw),
        .ce_10hz_raw    (ce_10hz_raw),
        .ce_pid_inner   (ce_pid_inner),
        .ce_pid_outer   (ce_pid_outer),
        .ce_ekf_predict (ce_ekf_predict),
        .ce_baro_upd    (ce_baro_upd),
        .ce_gps_upd     (ce_gps_upd),
        .ce_mag_upd     (ce_mag_upd),
        .ce_nav         (ce_nav),
        .ce_1hz         (ce_1hz),
        .ce_5hz         (ce_5hz),
        .ce_50hz        (ce_50hz),
        .ce_10hz        (ce_10hz),
        .ekf_state_in   (ekf_state_upd),
        .ekf_state_pid  (ekf_state_pid),
        .ekf_state_nav  (ekf_state_nav),
        .ekf_pred_wr_en   (1'b0), .ekf_pred_wr_addr('0), .ekf_pred_wr_data('0),
        .ekf_upd_rd_en    (1'b0), .ekf_upd_rd_addr('0),  .ekf_upd_rd_data(),
        .nav_rd_en        (1'b0), .nav_rd_addr('0),       .nav_rd_data(),
        .wp_nav_rd_en     (1'b0), .wp_nav_rd_addr('0),    .wp_nav_rd_data(),
        .wp_mis_wr_en     (1'b0), .wp_mis_wr_addr('0),    .wp_mis_wr_data('0)
    );

    // =========================================================================
    // MOD_10: AXI4-Lite Register Interface
    // =========================================================================
    module10_uav_axi_regif #(.CLK_HZ(CLK_HZ)) u_mod10 (
        .clk              (clk), .rst_n (rst_n & rst_n_axi),
        .s_awaddr  (s_axi_awaddr),  .s_awvalid (s_axi_awvalid), .s_awready (s_axi_awready),
        .s_wdata   (s_axi_wdata),   .s_wstrb   (s_axi_wstrb),   .s_wvalid  (s_axi_wvalid),  .s_wready  (s_axi_wready),
        .s_bresp   (s_axi_bresp),   .s_bvalid  (s_axi_bvalid),  .s_bready  (s_axi_bready),
        .s_araddr  (s_axi_araddr),  .s_arvalid (s_axi_arvalid), .s_arready (s_axi_arready),
        .s_rdata   (s_axi_rdata),   .s_rresp   (s_axi_rresp),   .s_rvalid  (s_axi_rvalid),  .s_rready  (s_axi_rready),
        .status_ekf  ({31'h0, ekf_upd_valid}),
        .status_gps  ({31'h0, gps_fix}),
        .status_imu  (32'h00000001),
        .flight_mode (flight_mode),
        .armed       (armed),
        .pid_gains   (pid_gains),
        .pid_integ_max(pid_integ_max),
        .mix_coef    (mix_coef),
        .ekf_q_diag  (ekf_q_diag),
        .gyro_bias   (gyro_bias),
        .accel_bias  (accel_bias),
        .mag_offset  (mag_offset),
        .wdt_timeout_ms   (wdt_timeout_ms),
        .wdt_config_en    (wdt_config_en),
        .geofence_radius_sq(geofence_radius_sq),
        .geofence_max_alt  (geofence_max_alt),
        .check_mask       (check_mask),
        .sys_arm_cmd      (sys_arm_cmd),
        .sys_mode_cmd     (sys_mode_cmd),
        .led_status       (led)
    );

    // =========================================================================
    // MOD_4: EKF Prediction (CRITICAL: Output feeds MOD_5 & MOD_2)
    // =========================================================================
    logic [STATES*STATE_W-1:0] mod4_state_out_bus;
    logic [STATES*P_W-1:0]     mod4_p_diag_out_bus;
    
    module4_uav_ekf_predict #(
        .CLK_HZ(CLK_HZ),
        .STATES(STATES),
        .STATE_W(STATE_W),
        .P_W(P_W),
        .EXPOSE_PARALLEL_IO(1'b1),
        .EXPOSE_DEBUG(1'b0)
    ) u_mod4 (
        .clk        (clk), .rst_n (rst_n & rst_n_ekf),
        .ce_100hz   (ce_ekf_predict),
        .spi_sclk   (imu_sclk),
        .spi_mosi   (imu_mosi),
        .spi_miso   (imu_miso),
        .spi_cs_n   (imu_cs_n),
        .gyro_bias  ({gyro_bias[2], gyro_bias[1], gyro_bias[0]}),
        .accel_bias ({accel_bias[2], accel_bias[1], accel_bias[0]}),
        .q_diag     ({ekf_q_diag[8], ekf_q_diag[7], ekf_q_diag[6], ekf_q_diag[5], 
                      ekf_q_diag[4], ekf_q_diag[3], ekf_q_diag[2], ekf_q_diag[1], ekf_q_diag[0]}),
        .state_in   ({ekf_state_pred[8], ekf_state_pred[7], ekf_state_pred[6], ekf_state_pred[5], 
                      ekf_state_pred[4], ekf_state_pred[3], ekf_state_pred[2], ekf_state_pred[1], ekf_state_pred[0]}),
        .p_diag_in  ({p_diag_pred[8], p_diag_pred[7], p_diag_pred[6], p_diag_pred[5], 
                      p_diag_pred[4], p_diag_pred[3], p_diag_pred[2], p_diag_pred[1], p_diag_pred[0]}),
        .state_out  (mod4_state_out_bus),
        .p_diag_out (mod4_p_diag_out_bus),
        .ekf_valid  (ekf_pred_valid),
        .debug_raw_imu(),
        .sqrt_result(),
        .sqrt_valid_out(),
        .p_pred_valid_out()
    );
    
    // Unpack MOD_4 outputs (feed to MOD_2 & MOD_5)
    generate
        for (genvar s = 0; s < STATES; s++) begin : g_mod4_unpack
            assign ekf_state_pred[s] = mod4_state_out_bus[((s+1)*STATE_W)-1 -: STATE_W];
            assign p_diag_pred[s]    = mod4_p_diag_out_bus[((s+1)*P_W)-1 -: P_W];
        end
    endgenerate

    // =========================================================================
    // MOD_5: Sensor Interface + EKF Updates (state stored internally)
    // =========================================================================
    // ✅ MOD_5 receives MOD_4 predictions via feedback loop
    logic [7:0]  mod5_dbg_status;
    logic [31:0] mod5_dbg_state;
    
    module5_uav_sensor_ekf #(
        .STATES(STATES),
        .STATE_W(STATE_W),
        .P_W(P_W)
    ) u_mod5 (
        .clk        (clk), 
        .rst_n      (rst_n & rst_n_sensors),
        .ce_100hz   (ce_100hz_raw),
        .ce_50hz    (ce_50hz),
        .ce_10hz    (ce_10hz),
        .gps_lat    (gps_lat),  
        .gps_lon    (gps_lon), 
        .gps_alt    (gps_alt),
        .gps_vn     (gps_vel_n), 
        .gps_ve     (gps_vel_e),
        .gps_valid  (gps_fix),
        .imu_ax     (32'h0),
        .imu_ay     (32'h0),
        .imu_az     (32'h0),
        .R_baro     (32'h00010000),
        .R_mag      (32'h00010000),
        .R_lat      (32'h00010000), 
        .R_lon      (32'h00010000),
        .R_alt_gps  (32'h00010000), 
        .R_vn       (32'h00010000), 
        .R_ve       (32'h00010000),
        .i2c_scl    (i2c_scl),
        .i2c_sda    (i2c_sda),
        .ekf_valid  (ekf_upd_valid),
        .baro_valid (baro_valid),
        .mag_valid_flag(mag_valid_flag),
        .dbg_status (mod5_dbg_status),
        .dbg_state  (mod5_dbg_state)
    );
    
    // ✅ Distribute MOD_4 predictions to all downstream modules
    always_comb begin
        for (int s = 0; s < STATES; s++) begin
            ekf_state_upd[s] = ekf_state_pred[s];  // MOD_4 output → system state
            p_diag_upd[s]    = p_diag_pred[s];     // MOD_4 covariance → system state
        end
    end

    // =========================================================================
    // MOD_6: GPS Interface
    // =========================================================================
    module6_uav_gps_interface #(.CLK_HZ(CLK_HZ)) u_mod6 (
        .clk          (clk), .rst_n (rst_n & rst_n_gps),
        .uart_rx      (gps_rx),
        .gps_lat      (gps_lat),
        .gps_lon      (gps_lon),
        .gps_alt      (gps_alt),
        .gps_vel_n    (gps_vel_n),
        .gps_vel_e    (gps_vel_e),
        .gps_fix_type (gps_fix_type),
        .gps_hdop     (gps_hdop),
        .gps_fix      (gps_fix),
        .gps_data_valid(gps_data_valid)
    );

    // =========================================================================
    // MOD_7: MAVLink Protocol
    // =========================================================================
    module7_uav_mavlink #(.CLK_HZ(CLK_HZ)) u_mod7 (
        .clk           (clk), .rst_n (rst_n & rst_n_mavlink),
        .ce_1hz        (ce_1hz), .ce_5hz (ce_5hz),
        .ce_10hz       (ce_10hz), .ce_50hz (ce_50hz),
        .uart_rx       (mav_rx),
        .uart_tx       (mav_tx),
        .ekf_roll      (mav_ekf_roll),
        .ekf_pitch     (mav_ekf_pitch),
        .ekf_yaw       (mav_ekf_yaw),
        .ekf_lat       (mav_ekf_lat),
        .ekf_lon       (mav_ekf_lon),
        .ekf_alt       (mav_ekf_alt),
        .ekf_healthy   (ekf_upd_valid),
        .sys_id        (8'd1),
        .comp_id       (8'd1),
        .base_mode     ({1'b0, 6'h00, armed}),
        .custom_mode   (flight_mode),
        .sensor_status (16'h3FFF),
        .cmd_arm       (cmd_arm),
        .cmd_disarm    (cmd_disarm),
        .cmd_takeoff   (cmd_takeoff),
        .cmd_land      (cmd_land),
        .cmd_rtl       (cmd_rtl),
        .cmd_waypoint  (cmd_waypoint),
        .cmd_set_mode  (cmd_set_mode),
        .cmd_reboot    (cmd_reboot),
        .cmd_mode_val  (cmd_mode_val),
        .gcs_present   (gcs_present),
        .gcs_lost      (gcs_lost)
    );

    // =========================================================================
    // MOD_9: Watchdog + Pre-Flight + Emergency
    // =========================================================================
    module9_uav_watchdog #(.CLK_HZ(CLK_HZ)) u_mod9 (
        .clk           (clk), .rst_n (rst_n & rst_n_wdt),
        .wdt_kick      (wdt_kick),
        .armed         (armed),
        .wdt_config_en (wdt_config_en),
        .wdt_timeout_ms(wdt_timeout_ms),
        .ekf_healthy   (ekf_upd_valid),
        .gps_fix_type  (gps_fix_type),
        .gps_hdop_q8   (gps_hdop),
        .baro_valid    (baro_valid),
        .mag_valid     (mag_valid_flag),
        .imu_fault     (1'b0),
        .gcs_present   (gcs_present),
        .check_mask    (check_mask),
        .curr_lat      (ekf_state_upd[6]),
        .curr_lon      (ekf_state_upd[7]),
        .curr_alt      (ekf_state_upd[8]),
        .wdt_ok        (wdt_ok),
        .wdt_expired   (wdt_expired),
        .preflight_ok  (preflight_ok),
        .fault_flags   (fault_flags),
        .emerg_active  (emerg_active),
        .emerg_sp_lat  (emerg_sp_lat),
        .emerg_sp_lon  (emerg_sp_lon),
        .emerg_sp_alt  (emerg_sp_alt),
        .emerg_sp_thrust(emerg_sp_thrust),
        .emerg_sp_vd   (emerg_sp_vd)
    );

    // =========================================================================
    // MOD_8: Navigation FSM
    // =========================================================================
    module8_uav_nav_fsm #(.CLK_HZ(CLK_HZ)) u_mod8 (
        .clk           (clk), .rst_n (rst_n & rst_n_nav),
        .ce_100hz      (ce_nav),
        .ce_10hz       (ce_10hz),
        .cmd_arm       (cmd_arm | arm_btn_sync),
        .cmd_disarm    (cmd_disarm),
        .cmd_takeoff   (cmd_takeoff),
        .cmd_land      (cmd_land),
        .cmd_rtl       (cmd_rtl),
        .cmd_waypoint  (cmd_waypoint),
        .cmd_set_mode  (cmd_set_mode),
        .cmd_mode      (cmd_mode_val),
        .wp_wr_en      (cmd_waypoint),
        .wp_wr_idx     (5'd0),
        .wp_wr_lat     (gps_lat),
        .wp_wr_lon     (gps_lon),
        .wp_wr_alt     (gps_alt),
        .wp_wr_radius  (32'h00050000),
        .wp_wr_speed   (32'h01000000),
        .wp_wr_cmd     (8'd16),
        .preflight_ok  (preflight_ok),
        .ekf_healthy   (ekf_upd_valid),
        .gps_fix       (gps_fix),
        .baro_valid    (baro_valid),
        .wdt_ok        (wdt_ok),
        .ekf_lat       (ekf_state_nav[6]),
        .ekf_lon       (ekf_state_nav[7]),
        .ekf_alt       (ekf_state_nav[8]),
        .ekf_vd        (ekf_state_nav[5]),
        .home_lat      (home_lat), .home_lon (home_lon), .home_alt (home_alt),
        .geofence_radius_sq(geofence_radius_sq),
        .geofence_max_alt  (geofence_max_alt),
        .sp_alt        (nav_sp_alt),
        .sp_vn         (nav_sp_vn),
        .sp_ve         (nav_sp_ve),
        .sp_yaw        (nav_sp_yaw),
        .sp_thrust     (nav_sp_thrust),
        .sp_roll       (nav_sp_roll),
        .sp_pitch      (nav_sp_pitch),
        .sp_vd         (),
        .armed         (armed),
        .flight_mode   (flight_mode),
        .wdt_kick      (wdt_kick)
    );

    // Mux emergency setpoints
    always_comb begin
        if (emerg_active) begin
            sp_alt    = emerg_sp_alt;
            sp_thrust = emerg_sp_thrust;
            sp_vn     = '0; sp_ve  = '0;
            sp_roll   = '0; sp_pitch = '0; sp_yaw = '0;
        end else begin
            sp_alt    = nav_sp_alt;
            sp_thrust = nav_sp_thrust;
            sp_vn     = nav_sp_vn;
            sp_ve     = nav_sp_ve;
            sp_roll   = nav_sp_roll;
            sp_pitch  = nav_sp_pitch;
            sp_yaw    = nav_sp_yaw;
        end
    end

    // =========================================================================
    // MOD_2: Dual-Loop PID Controller (CRITICAL: Input from MOD_4 EKF)
    // =========================================================================
    logic signed [15:0] pid_gains_trunc [0:7][0:2];
    logic signed [15:0] pid_out_max [0:7];
    logic signed [15:0] pid_out_min [0:7];
    logic signed [15:0] pid_out [0:7];
    logic [7:0]         pid_valid;
    
    always_comb begin
        for (int a = 0; a < 8; a++) begin
            for (int c = 0; c < 3; c++) begin
                pid_gains_trunc[a][c] = pid_gains[a][c];
            end
            pid_out_max[a]   = 16'sh7FFF;
            pid_out_min[a]   = -16'sh8000;
        end
    end

    // ✅ MOD_2 receives MOD_4 EKF predictions (ekf_state_pred = MOD_4 output)
    module2_uav_pid_top #(.CLK_HZ(CLK_HZ)) u_mod2 (
        .clk            (clk), .rst_n (rst_n),
        .ce_1khz        (ce_pid_inner),
        .ce_100hz       (ce_pid_outer),
        .ekf_roll       (ekf_state_pred[0][15:0]),      // ✅ FROM MOD_4
        .ekf_pitch      (ekf_state_pred[1][15:0]),      // ✅ FROM MOD_4
        .ekf_yaw        (ekf_state_pred[2][15:0]),      // ✅ FROM MOD_4
        .ekf_roll_rate  (ekf_state_pred[0][15:0]),      // ✅ FROM MOD_4
        .ekf_pitch_rate (ekf_state_pred[1][15:0]),      // ✅ FROM MOD_4
        .ekf_yaw_rate   (ekf_state_pred[2][15:0]),      // ✅ FROM MOD_4
        .ekf_alt        (ekf_state_pred[8][15:0]),      // ✅ FROM MOD_4
        .ekf_vn         (ekf_state_pred[3][15:0]),      // ✅ FROM MOD_4
        .ekf_ve         (ekf_state_pred[4][15:0]),      // ✅ FROM MOD_4
        .nav_roll_sp    (sp_roll[15:0]),
        .nav_pitch_sp   (sp_pitch[15:0]),
        .nav_yaw_sp     (sp_yaw[15:0]),
        .nav_alt_sp     (sp_alt[15:0]),
        .nav_vn_sp      (sp_vn[15:0]),
        .nav_ve_sp      (sp_ve[15:0]),
        .nav_thrust_sp  (sp_thrust[15:0]),
        .gains          (pid_gains_trunc),
        .integ_max      (pid_integ_max),
        .out_max        (pid_out_max),
        .out_min        (pid_out_min),
        .kv             (16'sd410),
        .clear_integ    (8'h00),
        .roll_rate_cmd  (roll_rate_cmd_16),
        .pitch_rate_cmd (pitch_rate_cmd_16),
        .yaw_rate_cmd   (yaw_rate_cmd_16),
        .thrust_cmd     (thrust_cmd_16),
        .pid_out        (pid_out),
        .pid_valid      (pid_valid)
    );
    
    // ✅ Extend MOD_2 16-bit outputs to 32-bit for MOD_3
    always_comb begin
        roll_rate_cmd  = {{16{roll_rate_cmd_16[15]}}, roll_rate_cmd_16};
        pitch_rate_cmd = {{16{pitch_rate_cmd_16[15]}}, pitch_rate_cmd_16};
        yaw_rate_cmd   = {{16{yaw_rate_cmd_16[15]}}, yaw_rate_cmd_16};
        thrust_cmd     = {{16{thrust_cmd_16[15]}}, thrust_cmd_16};
    end

    // =========================================================================
    // MOD_3: Motor Mixing + ESC PWM (CRITICAL: Input from MOD_2 PID)
    // =========================================================================
    logic signed [15:0] motor_cmd_sat [0:3];
    logic signed [15:0] motor_out_max [0:3];
    logic signed [15:0] motor_out_min [0:3];
    logic signed [15:0] motor_max_delta[0:3];
    
    always_comb begin
        for (int m = 0; m < 4; m++) begin
            motor_out_max[m]   = 16'sh7FFF;
            motor_out_min[m]   = 16'sh0000;
            motor_max_delta[m] = 16'sh1000;
        end
    end
    
    // ✅ MOD_3 receives MOD_2 PID commands (critical dataflow)
    module3_uav_motor_mix #(.CLK_HZ(CLK_HZ)) u_mod3 (
        .clk           (clk), 
        .rst_n         (rst_n),
        .ce_1khz       (ce_pid_inner),
        .armed         (armed),
        .roll_cmd      (roll_rate_cmd_16),         // ✅ FROM MOD_2
        .pitch_cmd     (pitch_rate_cmd_16),        // ✅ FROM MOD_2
        .yaw_cmd       (yaw_rate_cmd_16),          // ✅ FROM MOD_2
        .thrust_cmd    (thrust_cmd_16),            // ✅ FROM MOD_2
        .mix_coef      (mix_coef),
        .out_max       (motor_out_max),
        .out_min       (motor_out_min),
        .max_delta     (motor_max_delta),
        .integ_max     (pid_integ_max[0:3]),
        .pwm_out       (pwm_out),
        .motor_cmd_sat (motor_cmd_sat)
    );
    
    // =========================================================================
    // System controller (reset sequencer + emergency)
    // =========================================================================
    uav_system_controller #(.CLK_HZ(CLK_HZ)) u_sysctrl (
        .clk                (clk), .rst_n (rst_n),
        .pll_lock_stable    (pll_lock_stable),
        .rst_n_clk          (rst_n_clk),
        .rst_n_sync         (rst_n_sync),
        .rst_n_ekf          (rst_n_ekf),
        .rst_n_sensors      (rst_n_sensors),
        .rst_n_gps          (rst_n_gps),
        .rst_n_mavlink      (rst_n_mavlink),
        .rst_n_nav          (rst_n_nav),
        .rst_n_wdt          (rst_n_wdt),
        .rst_n_axi          (rst_n_axi),
        .wdt_expired        (wdt_expired),
        .geofence_violation (1'b0),
        .emergency          (emergency)
    );

endmodule
