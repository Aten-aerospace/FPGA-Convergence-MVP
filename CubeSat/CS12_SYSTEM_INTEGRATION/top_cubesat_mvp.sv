// =============================================================================
// Module: top_cubesat_mvp (CS12 system integration top-level)
// Subsystem: CS12 - System Integration
// Description: Top-level CubeSat FPGA integration.  Instantiates:
//
//   CS1  spi_imu_wrapper     - IMU SPI interface
//   CS2  i2c_mag_wrapper     - Magnetometer I2C interface
//   CS3  sun_sensor_wrapper  - Sun sensor ADC interface
//   CS4  quat_propagator_wrapper - Quaternion kinematic propagator
//   CS5  ekf_wrapper         - Extended Kalman Filter
//   CS6  pd_control_wrapper  - PD attitude controller
//   CS7  actuator_wrapper    - Reaction wheel + magnetorquer drivers
//   CS8  adcs_fsm_wrapper    - ADCS mode FSM + health monitor
//   CS9  orbit_propagator_wrapper - SGP4-lite orbit propagator
//   CS10 laser_fsm_wrapper   - Laser pointing FSM + gimbal controller
//   CS11 telemetry_wrapper   - CCSDS telemetry encoder + UART TX
//   CS12 clk_manager         - Clock enable generation
//
//   Signal routing summary:
//     CS1  → CS4 : gyro/accel (omega feed to propagator)
//     CS1  → CS5 : accel, gyro, imu_data_valid
//     CS2  → CS5 : mag_x/y/z, mag_data_valid
//     CS5  → CS4 : q_est (feedback state to propagator)
//     CS5  → CS6 : q_est → q_err (via bias subtraction, simplified here)
//     CS5  → CS8 : ekf_valid, q_err_mag
//     CS1  → CS6 : omega → angular rate for PD-D term
//     CS1  → CS8 : imu_data_valid
//     CS2  → CS8 : mag_data_valid
//     CS6  → CS7 : torque_cmd, cmd_valid
//     CS8  → CS7 : safe_mode (state == SAFE || state == FAULT)
//     CS8  → CS10: laser_enable (state == FINE_POINT)
//     CS9  → CS11: orbit_tlm bytes (packed from eci_pos/vel)
//     CS10 → CS11: laser_tlm bytes (packed from laser_state/pointing_locked)
//     CS5  → CS11: adcs_tlm bytes (packed from q_est)
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module top_cubesat_mvp #(
    parameter int CLK_HZ   = 100_000_000,
    parameter int SPI_HZ   = 8_000_000,
    parameter int I2C_HZ   = 400_000
)(
    // -------------------------------------------------------------------------
    // Physical pins
    // -------------------------------------------------------------------------
    input  logic        clk_100mhz,
    input  logic        rst_ext_n,       // external active-low reset

    // IMU SPI (CS1)
    output logic        imu_spi_sclk,
    output logic        imu_spi_mosi,
    input  logic        imu_spi_miso,
    output logic        imu_spi_cs_n,

    // Magnetometer I2C (CS2)
    inout  wire         mag_i2c_sda,
    output logic        mag_i2c_scl,

    // Sun sensor SPI (CS3)
    output logic        sun_spi_sclk,
    output logic        sun_spi_mosi,
    input  logic        sun_spi_miso,
    output logic        sun_spi_cs_n,

    // Reaction-wheel PWM + enable (CS7)
    output logic [2:0]  pwm_rw,
    output logic [2:0]  rw_enable,

    // Magnetorquer PWM + direction (CS7)
    output logic [2:0]  pwm_mtq,
    output logic [2:0]  dir_mtq,
    output logic [2:0]  mtq_enable,

    // Gimbal step/direction (CS10)
    output logic [1:0]  gimbal_step,
    output logic [1:0]  gimbal_dir,

    // Laser modulator enable (CS10)
    output logic        laser_mod_en,

    // Telemetry UART TX (CS11)
    output logic        tlm_uart_tx,

    // Status / debug outputs
    output logic [2:0]  adcs_mode,
    output logic        adcs_fault,
    output logic        actuator_fault,
    output logic        orb_valid,
    output logic        pointing_locked,
    output logic        tlm_valid
);

    // =========================================================================
    // Internal reset synchroniser (2-FF)
    // =========================================================================
    logic rst_sync_ff1, rst_sync_ff2;
    logic rst_n; // synchronised active-low reset

    always_ff @(posedge clk_100mhz or negedge rst_ext_n) begin
        if (!rst_ext_n) begin
            rst_sync_ff1 <= 1'b0;
            rst_sync_ff2 <= 1'b0;
        end else begin
            rst_sync_ff1 <= 1'b1;
            rst_sync_ff2 <= rst_sync_ff1;
        end
    end

    assign rst_n = rst_sync_ff2;

    // =========================================================================
    // CS12 — Clock manager
    // =========================================================================
    logic sys_clk;
    logic ce_1hz;
    logic ce_100hz;
    logic ce_1khz;

    clk_manager #(
        .CLK_HZ   (CLK_HZ)
    ) u_clk (
        .clk      (clk_100mhz),
        .rst_n    (rst_n),
        .sys_clk  (sys_clk),
        .ce_1hz   (ce_1hz),
        .ce_100hz (ce_100hz),
        .ce_1khz  (ce_1khz)
    );

    // =========================================================================
    // CS1 — IMU SPI wrapper
    // =========================================================================
    logic signed [15:0] imu_accel_x, imu_accel_y, imu_accel_z;
    logic signed [15:0] imu_gyro_x,  imu_gyro_y,  imu_gyro_z;
    logic signed [15:0] imu_mag_x,   imu_mag_y,   imu_mag_z;
    logic               imu_data_valid;
    logic               imu_fault;

    spi_imu_wrapper #(
        .CLK_HZ (CLK_HZ),
        .SPI_HZ (SPI_HZ)
    ) u_cs1 (
        .clk              (clk_100mhz),
        .rst_n            (rst_n),
        .imu_read_trigger (ce_100hz),
        .spi_sclk         (imu_spi_sclk),
        .spi_mosi         (imu_spi_mosi),
        .spi_miso         (imu_spi_miso),
        .spi_cs_n         (imu_spi_cs_n),
        .accel_x          (imu_accel_x),
        .accel_y          (imu_accel_y),
        .accel_z          (imu_accel_z),
        .gyro_x           (imu_gyro_x),
        .gyro_y           (imu_gyro_y),
        .gyro_z           (imu_gyro_z),
        .mag_x            (imu_mag_x),
        .mag_y            (imu_mag_y),
        .mag_z            (imu_mag_z),
        .imu_data_valid   (imu_data_valid),
        .imu_busy         (),
        .imu_fault        (imu_fault),
        .imu_overflow     ()
    );

    // =========================================================================
    // CS2 — Magnetometer I2C wrapper
    // =========================================================================
    logic signed [15:0] mag_x, mag_y, mag_z;
    logic               mag_data_valid;
    logic               mag_fault;

    i2c_mag_wrapper #(
        .CLK_HZ (CLK_HZ)
    ) u_cs2 (
        .clk              (clk_100mhz),
        .rst_n            (rst_n),
        .mag_read_trigger (ce_100hz),
        .i2c_sda          (mag_i2c_sda),
        .i2c_scl          (mag_i2c_scl),
        .mag_x            (mag_x),
        .mag_y            (mag_y),
        .mag_z            (mag_z),
        .mag_data_valid   (mag_data_valid),
        .mag_busy         (),
        .mag_fault        (mag_fault)
    );

    // =========================================================================
    // CS3 — Sun sensor ADC wrapper
    // =========================================================================
    logic signed [15:0] sun_alpha, sun_beta;
    logic               sun_present;
    logic               sun_data_valid;

    sun_sensor_wrapper #(
        .CLK_HZ (CLK_HZ)
    ) u_cs3 (
        .clk            (clk_100mhz),
        .rst_n          (rst_n),
        .sun_trigger    (ce_100hz),
        .spi_sclk       (sun_spi_sclk),
        .spi_mosi       (sun_spi_mosi),
        .spi_miso       (sun_spi_miso),
        .spi_cs_n       (sun_spi_cs_n),
        .sun_alpha      (sun_alpha),
        .sun_beta       (sun_beta),
        .sun_present    (sun_present),
        .sun_data_valid (sun_data_valid),
        .sun_busy       (),
        .sun_fault      ()
    );

    // =========================================================================
    // CS5 — EKF wrapper
    // =========================================================================
    logic signed [15:0] ekf_accel [0:2];
    logic signed [15:0] ekf_gyro  [0:2];
    logic signed [15:0] ekf_mag   [0:2];
    logic signed [15:0] q_est     [0:3];
    logic signed [15:0] bias_est  [0:2];
    logic               ekf_valid;
    logic               ekf_fault;

    // Pack scalar outputs of CS1 into arrays for EKF
    always_comb begin
        ekf_accel[0] = imu_accel_x;
        ekf_accel[1] = imu_accel_y;
        ekf_accel[2] = imu_accel_z;
        ekf_gyro[0]  = imu_gyro_x;
        ekf_gyro[1]  = imu_gyro_y;
        ekf_gyro[2]  = imu_gyro_z;
        ekf_mag[0]   = mag_x;
        ekf_mag[1]   = mag_y;
        ekf_mag[2]   = mag_z;
    end

    ekf_wrapper #(
        .CLK_HZ (CLK_HZ)
    ) u_cs5 (
        .clk        (clk_100mhz),
        .rst_n      (rst_n),
        .ce_100hz   (ce_100hz),
        .accel      (ekf_accel),
        .gyro       (ekf_gyro),
        .mag        (ekf_mag),
        .meas_valid (imu_data_valid),
        .q_est      (q_est),
        .bias_est   (bias_est),
        .ekf_valid  (ekf_valid),
        .ekf_fault  (ekf_fault)
    );

    // =========================================================================
    // CS4 — Quaternion propagator wrapper
    // =========================================================================
    logic signed [15:0] omega_prop [0:2]; // angular rate for propagator
    logic signed [15:0] q_prop_out [0:3];
    logic               q_prop_valid;

    always_comb begin
        omega_prop[0] = imu_gyro_x;
        omega_prop[1] = imu_gyro_y;
        omega_prop[2] = imu_gyro_z;
    end

    quat_propagator_wrapper u_cs4 (
        .clk         (clk_100mhz),
        .rst_n       (rst_n),
        .ce_100hz    (ce_100hz),
        .q_in        (q_est),
        .q_valid_in  (ekf_valid),
        .omega       (omega_prop),
        .q_out       (q_prop_out),
        .q_valid_out (q_prop_valid),
        .norm_error  ()
    );

    // =========================================================================
    // CS6 — PD control wrapper
    // Quaternion error = q_est (target = identity, so q_err ≈ q_est[1:3])
    // =========================================================================
    logic signed [15:0] pd_q_err     [0:2];
    logic signed [15:0] pd_omega     [0:2];
    logic signed [15:0] torque_cmd   [0:2];
    logic               pd_cmd_valid;

    always_comb begin
        pd_q_err[0] = q_est[1];  // vector part of quaternion error
        pd_q_err[1] = q_est[2];
        pd_q_err[2] = q_est[3];
        pd_omega[0] = imu_gyro_x;
        pd_omega[1] = imu_gyro_y;
        pd_omega[2] = imu_gyro_z;
    end

    pd_control_wrapper u_cs6 (
        .clk        (clk_100mhz),
        .rst_n      (rst_n),
        .ce_1khz    (ce_1khz),
        .q_err      (pd_q_err),
        .omega      (pd_omega),
        .ctrl_valid (ekf_valid),
        .torque_cmd (torque_cmd),
        .sat_flag   (),
        .cmd_valid  (pd_cmd_valid)
    );

    // =========================================================================
    // CS8 — ADCS FSM wrapper
    // =========================================================================
    // omega_mag: max of |gyro| components (simplified as |gyro_x| for MVP)
    logic [15:0] omega_mag_unsigned;
    logic [15:0] q_err_mag_unsigned;
    logic        health_ok_int;
    logic [7:0]  fault_flags_int;
    logic        mode_valid_int;

    always_comb begin
        // omega_mag = unsigned max of |gyro_x|, |gyro_y|, |gyro_z|
        logic [15:0] ax, ay, az;
        ax = imu_gyro_x[15] ? 16'(-$signed(imu_gyro_x)) : 16'(imu_gyro_x);
        ay = imu_gyro_y[15] ? 16'(-$signed(imu_gyro_y)) : 16'(imu_gyro_y);
        az = imu_gyro_z[15] ? 16'(-$signed(imu_gyro_z)) : 16'(imu_gyro_z);
        omega_mag_unsigned = (ax > ay && ax > az) ? ax :
                             (ay > az)            ? ay : az;
        // q_err_mag = |q_est[1]|
        q_err_mag_unsigned = q_est[1][15] ? 16'(-$signed(q_est[1])) : 16'(q_est[1]);
    end

    logic [2:0] adcs_mode_int;
    logic       adcs_fault_int;
    logic       safe_mode_int;

    adcs_fsm_wrapper u_cs8 (
        .clk         (clk_100mhz),
        .rst_n       (rst_n),
        .ce_100hz    (ce_100hz),
        .imu_valid   (imu_data_valid),
        .mag_valid   (mag_data_valid),
        .ekf_valid   (ekf_valid),
        .q_err_mag   (q_err_mag_unsigned),
        .omega_mag   (omega_mag_unsigned),
        .uplink_mode (3'd7),            // UL_NONE (connect to command decoder for flight)
        .adcs_mode   (adcs_mode_int),
        .mode_valid  (mode_valid_int),
        .health_ok   (health_ok_int),
        .fault_flags (fault_flags_int),
        .adcs_fault  (adcs_fault_int)
    );

    // safe_mode: assert when FSM is in SAFE (4) or FAULT (5)
    always_comb safe_mode_int = (adcs_mode_int == 3'd4) || (adcs_mode_int == 3'd5);

    // =========================================================================
    // CS7 — Actuator wrapper
    // =========================================================================
    // dipole_cmd: zero for MVP (connect mag torque controller for flight)
    logic signed [15:0] dipole_cmd [0:2];
    always_comb for (int i = 0; i < 3; i++) dipole_cmd[i] = 16'sd0;

    logic actuator_fault_int;

    actuator_wrapper #(
        .CLK_HZ (CLK_HZ)
    ) u_cs7 (
        .clk            (clk_100mhz),
        .rst_n          (rst_n),
        .ce_1khz        (ce_1khz),
        .torque_cmd     (torque_cmd),
        .dipole_cmd     (dipole_cmd),
        .cmd_valid      (pd_cmd_valid),
        .safe_mode      (safe_mode_int),
        .pwm_rw         (pwm_rw),
        .rw_enable      (rw_enable),
        .pwm_mtq        (pwm_mtq),
        .dir_mtq        (dir_mtq),
        .mtq_enable     (mtq_enable),
        .actuator_fault (actuator_fault_int)
    );

    // =========================================================================
    // CS9 — Orbit propagator wrapper
    // =========================================================================
    // TLE write not connected to a live command decoder in MVP — tied off
    logic [31:0] tle_data [0:5];
    logic        tle_write;
    logic [31:0] eci_pos  [0:2];
    logic [31:0] eci_vel  [0:2];
    logic [31:0] lvlh_x, lvlh_y, lvlh_z;
    logic        orb_valid_int;
    logic        orb_fault;

    always_comb begin
        // Default TLE: nominal ISS-like orbit
        tle_data[0] = 32'h0001_6800;   // n0 LEO
        tle_data[1] = 32'h0000_0010;   // e0
        tle_data[2] = 32'h0000_6488;   // i0 = 90°
        tle_data[3] = 32'h0000_0000;
        tle_data[4] = 32'h0000_0000;
        tle_data[5] = 32'h0000_0000;
        tle_write   = 1'b0;            // tie off until command decoder connected
    end

    orbit_propagator_wrapper u_cs9 (
        .clk       (clk_100mhz),
        .rst_n     (rst_n),
        .ce_1hz    (ce_1hz),
        .tle_data  (tle_data),
        .tle_write (tle_write),
        .eci_pos   (eci_pos),
        .eci_vel   (eci_vel),
        .lvlh_x    (lvlh_x),
        .lvlh_y    (lvlh_y),
        .lvlh_z    (lvlh_z),
        .orb_valid (orb_valid_int),
        .orb_fault (orb_fault)
    );

    // =========================================================================
    // CS10 — Laser FSM wrapper
    // laser_enable: asserted when ADCS is in FINE_POINT (mode = 3)
    // =========================================================================
    logic        laser_enable_int;
    logic [2:0]  laser_state_int;
    logic        pointing_locked_int;
    logic        laser_fault_int;

    always_comb laser_enable_int = (adcs_mode_int == 3'd3);

    laser_fsm_wrapper u_cs10 (
        .clk             (clk_100mhz),
        .rst_n           (rst_n),
        .ce_100hz        (ce_100hz),
        .signal_strength (12'h000),    // connect to ADC in hardware build
        .signal_valid    (1'b0),       // tie off until ADC connected
        .laser_enable    (laser_enable_int),
        .laser_mod_en    (laser_mod_en),
        .gimbal_step     (gimbal_step),
        .gimbal_dir      (gimbal_dir),
        .laser_state     (laser_state_int),
        .pointing_locked (pointing_locked_int),
        .laser_fault     (laser_fault_int)
    );

    // =========================================================================
    // CS11 — Telemetry wrapper
    // Pack subsystem data into TLM byte arrays
    // =========================================================================

    // ADCS TLM (44 bytes): [q_est × 4 × 2 bytes] + [omega × 3 × 2 bytes] + padding
    logic [7:0]  adcs_tlm  [0:43];
    logic        adcs_tlm_valid;

    always_ff @(posedge clk_100mhz or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 44; i++) adcs_tlm[i] <= 8'h00;
            adcs_tlm_valid <= 1'b0;
        end else begin
            adcs_tlm_valid <= ekf_valid;
            if (ekf_valid) begin
                // Quaternion estimate (8 bytes, big-endian)
                for (int i = 0; i < 4; i++) begin
                    adcs_tlm[i*2]     <= q_est[i][15:8];
                    adcs_tlm[i*2 + 1] <= q_est[i][7:0];
                end
                // Gyro rates (6 bytes)
                adcs_tlm[8]  <= imu_gyro_x[15:8];
                adcs_tlm[9]  <= imu_gyro_x[7:0];
                adcs_tlm[10] <= imu_gyro_y[15:8];
                adcs_tlm[11] <= imu_gyro_y[7:0];
                adcs_tlm[12] <= imu_gyro_z[15:8];
                adcs_tlm[13] <= imu_gyro_z[7:0];
                // ADCS mode + fault flags (2 bytes)
                adcs_tlm[14] <= {5'h00, adcs_mode_int};
                adcs_tlm[15] <= fault_flags_int;
                // Padding
                for (int i = 16; i < 44; i++) adcs_tlm[i] <= 8'h00;
            end
        end
    end

    // Orbit TLM (47 bytes): ECI pos (12 bytes) + ECI vel (12 bytes) + LVLH (12 bytes) + orb_valid
    logic [7:0]  orbit_tlm [0:46];
    logic        orbit_tlm_valid;

    always_ff @(posedge clk_100mhz or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 47; i++) orbit_tlm[i] <= 8'h00;
            orbit_tlm_valid <= 1'b0;
        end else begin
            orbit_tlm_valid <= orb_valid_int;
            if (orb_valid_int) begin
                for (int i = 0; i < 3; i++) begin
                    orbit_tlm[i*4]     <= eci_pos[i][31:24];
                    orbit_tlm[i*4 + 1] <= eci_pos[i][23:16];
                    orbit_tlm[i*4 + 2] <= eci_pos[i][15:8];
                    orbit_tlm[i*4 + 3] <= eci_pos[i][7:0];
                end
                for (int i = 0; i < 3; i++) begin
                    orbit_tlm[12 + i*4]     <= eci_vel[i][31:24];
                    orbit_tlm[12 + i*4 + 1] <= eci_vel[i][23:16];
                    orbit_tlm[12 + i*4 + 2] <= eci_vel[i][15:8];
                    orbit_tlm[12 + i*4 + 3] <= eci_vel[i][7:0];
                end
                orbit_tlm[24] <= lvlh_x[31:24];
                orbit_tlm[25] <= lvlh_x[23:16];
                orbit_tlm[26] <= lvlh_x[15:8];
                orbit_tlm[27] <= lvlh_x[7:0];
                orbit_tlm[28] <= lvlh_y[31:24];
                orbit_tlm[29] <= lvlh_y[23:16];
                orbit_tlm[30] <= lvlh_y[15:8];
                orbit_tlm[31] <= lvlh_y[7:0];
                orbit_tlm[32] <= lvlh_z[31:24];
                orbit_tlm[33] <= lvlh_z[23:16];
                orbit_tlm[34] <= lvlh_z[15:8];
                orbit_tlm[35] <= lvlh_z[7:0];
                orbit_tlm[36] <= {7'h00, orb_valid_int};
                for (int i = 37; i < 47; i++) orbit_tlm[i] <= 8'h00;
            end
        end
    end

    // Laser TLM (20 bytes): state, pointing_locked, fault
    logic [7:0]  laser_tlm [0:19];
    logic        laser_tlm_valid;

    always_ff @(posedge clk_100mhz or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 20; i++) laser_tlm[i] <= 8'h00;
            laser_tlm_valid <= 1'b0;
        end else begin
            laser_tlm_valid <= ce_100hz;
            if (ce_100hz) begin
                laser_tlm[0] <= {5'h00, laser_state_int};
                laser_tlm[1] <= {6'h00, pointing_locked_int, laser_fault_int};
                for (int i = 2; i < 20; i++) laser_tlm[i] <= 8'h00;
            end
        end
    end

    // HK TLM (16 bytes): health flags
    logic [7:0]  hk_tlm [0:15];
    logic        hk_tlm_valid;

    always_ff @(posedge clk_100mhz or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 16; i++) hk_tlm[i] <= 8'h00;
            hk_tlm_valid <= 1'b0;
        end else begin
            hk_tlm_valid <= ce_1hz;
            if (ce_1hz) begin
                hk_tlm[0]  <= fault_flags_int;
                hk_tlm[1]  <= {5'h00, adcs_mode_int};
                hk_tlm[2]  <= {6'h00, ekf_fault, imu_fault};
                hk_tlm[3]  <= {6'h00, orb_fault, actuator_fault_int};
                for (int i = 4; i < 16; i++) hk_tlm[i] <= 8'h00;
            end
        end
    end

    telemetry_wrapper #(
        .CLK_HZ  (CLK_HZ),
        .BAUD_HZ (115_200)
    ) u_cs11 (
        .clk         (clk_100mhz),
        .rst_n       (rst_n),
        .ce_1hz      (ce_1hz),
        .adcs_tlm    (adcs_tlm),
        .adcs_valid  (adcs_tlm_valid),
        .orbit_tlm   (orbit_tlm),
        .orbit_valid (orbit_tlm_valid),
        .laser_tlm   (laser_tlm),
        .laser_valid (laser_tlm_valid),
        .hk_tlm      (hk_tlm),
        .hk_valid    (hk_tlm_valid),
        .uart_tx     (tlm_uart_tx),
        .tlm_valid   (tlm_valid),
        .tlm_busy    ()
    );

    // =========================================================================
    // Top-level output assignments
    // =========================================================================
    always_comb begin
        adcs_mode      = adcs_mode_int;
        adcs_fault     = adcs_fault_int;
        actuator_fault = actuator_fault_int;
        orb_valid      = orb_valid_int;
        pointing_locked = pointing_locked_int;
    end

endmodule
