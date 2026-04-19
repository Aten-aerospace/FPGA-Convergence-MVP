// =============================================================================
// File        : uav_sensor_ekf.sv
// Module      : uav_sensor_ekf
// Description : UAV MOD_5 - Sensor Interface + EKF Measurement Updates top.
//               Interfaces BMP388 (I2C), Magnetometer (I2C).
//               Cascaded IIR LPF: Baro 10 Hz, Mag 2 Hz.
//               EKF measurement updates: GPS (10 Hz), Baro (50 Hz), Mag (10 Hz).
// =============================================================================

`timescale 1ns/1ps

module module5_uav_sensor_ekf #(
    parameter int CLK_HZ  = 50_000_000,
    parameter int STATES  = 9,
    parameter int STATE_W = 32,  // Q4.28
    parameter int P_W     = 32   // Q16.16
)(
    input  logic clk,
    input  logic rst_n,

    // CE strobes
    input  logic ce_100hz,
    input  logic ce_50hz,
    input  logic ce_10hz,

    // I2C interfaces (BMP388 + Magnetometer)
    output logic i2c_scl,
    inout  wire  i2c_sda,

    // GPS measurements (from MOD_6)
    input  logic signed [STATE_W-1:0] gps_lat,
    input  logic signed [STATE_W-1:0] gps_lon,
    input  logic signed [STATE_W-1:0] gps_alt,
    input  logic signed [STATE_W-1:0] gps_vn,
    input  logic signed [STATE_W-1:0] gps_ve,
    input  logic                       gps_valid,

    // Measurement noise registers (Q16.16, from MOD_10)
    input  logic [P_W-1:0] R_baro,
    input  logic [P_W-1:0] R_mag,
    input  logic [P_W-1:0] R_lat, R_lon, R_alt_gps, R_vn, R_ve,

    // Mag hard-iron calibration (Q4.28, from MOD_10)
    input  logic signed [STATE_W-1:0] mag_offset_x,
    input  logic signed [STATE_W-1:0] mag_offset_y,
    input  logic signed [STATE_W-1:0] mag_offset_z,

    // EKF state & covariance in (predicted, from MOD_4)
    input  logic signed [STATE_W-1:0] state_in  [0:STATES-1],
    input  logic signed [P_W-1:0]     p_diag_in [0:STATES-1],

    // Updated EKF state & covariance out
    output logic signed [STATE_W-1:0] state_out  [0:STATES-1],
    output logic signed [P_W-1:0]     p_diag_out [0:STATES-1],
    output logic                       ekf_valid,

    // Sensor health flags
    output logic baro_valid,
    output logic mag_valid_flag
);

    // -------------------------------------------------------------------------
    // I2C master (shared bus - time-division for BMP388 and magnetometer)
    // -------------------------------------------------------------------------
    logic       i2c_start;
    logic [6:0] i2c_slave_addr;
    logic       i2c_rw;
    logic [7:0] i2c_write_data;
    logic [7:0] i2c_read_data;
    logic       i2c_busy;
    logic       i2c_ack_error;

    i2c_master #(
        .CLK_HZ  (CLK_HZ),
        .I2C_HZ  (400_000)
    ) u_i2c (
        .clk        (clk),
        .rst_n      (rst_n),
        .start      (i2c_start),
        .slave_addr (i2c_slave_addr),
        .rw         (i2c_rw),
        .write_data (i2c_write_data),
        .read_data  (i2c_read_data),
        .busy       (i2c_busy),
        .ack_error  (i2c_ack_error),
        .sda        (i2c_sda),
        .scl        (i2c_scl)
    );

    // -------------------------------------------------------------------------
    // Placeholder baro / mag measurement registers (would be filled by I2C FSM)
    // -------------------------------------------------------------------------
    logic signed [STATE_W-1:0] baro_alt_raw;
    logic signed [STATE_W-1:0] mag_heading_raw;
    logic baro_meas_valid, mag_meas_valid;

    // Simplified: assert valid on strobes (real design would read from I2C)
    assign baro_meas_valid = ce_50hz;
    assign mag_meas_valid  = ce_10hz;
    assign baro_alt_raw    = state_in[8]; // feedback when no real sensor
    assign mag_heading_raw = state_in[2];

    assign baro_valid    = 1'b1; // healthy flag (tied to I2C ACK in full impl)
    assign mag_valid_flag = 1'b1;

    // -------------------------------------------------------------------------
    // Cascaded IIR LPF (reuse lpf.sv)
    // -------------------------------------------------------------------------
    logic signed [STATE_W-1:0] baro_lpf, mag_heading_lpf;

    lpf #(.DATA_W(STATE_W), .ORDER(2)) u_lpf_baro (
        .clk         (clk), .rst_n (rst_n),
        .enable      (baro_meas_valid),
        .data_in     (baro_alt_raw),
        .cutoff_freq (16'd10),
        .data_out    (baro_lpf)
    );

    lpf #(.DATA_W(STATE_W), .ORDER(2)) u_lpf_mag (
        .clk         (clk), .rst_n (rst_n),
        .enable      (mag_meas_valid),
        .data_in     (mag_heading_raw),
        .cutoff_freq (16'd2),
        .data_out    (mag_heading_lpf)
    );

    // -------------------------------------------------------------------------
    // Innovation Gate
    // -------------------------------------------------------------------------
    logic        gate_valid_gps, gate_valid_baro, gate_valid_mag;
    logic        gate_accept_gps, gate_accept_baro, gate_accept_mag;
    logic signed [P_W-1:0] innov_cov_gps, innov_cov_baro, innov_cov_mag;
    logic signed [STATE_W-1:0] innov_gps, innov_baro, innov_mag;

    innovation_gate #(.DATA_W(STATE_W), .THRESH(589824)) u_gate_gps (
        .clk       (clk), .rst_n (rst_n),
        .valid_in  (gps_valid & ce_10hz),
        .innov     (innov_gps),
        .innov_cov (innov_cov_gps),
        .accept    (gate_accept_gps),
        .valid_out (gate_valid_gps)
    );

    innovation_gate #(.DATA_W(STATE_W), .THRESH(589824)) u_gate_baro (
        .clk       (clk), .rst_n (rst_n),
        .valid_in  (ce_50hz & baro_valid),
        .innov     (innov_baro),
        .innov_cov (innov_cov_baro),
        .accept    (gate_accept_baro),
        .valid_out (gate_valid_baro)
    );

    innovation_gate #(.DATA_W(STATE_W), .THRESH(589824)) u_gate_mag (
        .clk       (clk), .rst_n (rst_n),
        .valid_in  (ce_10hz & mag_valid_flag),
        .innov     (innov_mag),
        .innov_cov (innov_cov_mag),
        .accept    (gate_accept_mag),
        .valid_out (gate_valid_mag)
    );

    // -------------------------------------------------------------------------
    // Kalman 1V (Alternative scalar filter - optional integration)
    // -------------------------------------------------------------------------
    logic signed [STATE_W-1:0] kal_est_px, kal_est_py, kal_est_pz;
    logic signed [STATE_W-1:0] kal_est_vx, kal_est_vy, kal_est_vz;
    logic kal_data_valid;

    kalman_1v #(.POS_W(STATE_W), .VEL_W(STATE_W), .MEAS_W(STATE_W), .VID(0)) u_kalman (
        .clk       (clk), .rst_n (rst_n),
        .predict_en(ce_100hz),
        .update_en (ce_10hz),
        .meas_px   (gps_lat), .meas_py (gps_lon), .meas_pz (gps_alt),
        .accel_x   (state_in[3]), .accel_y (state_in[4]), .accel_z (state_in[5]),
        .est_px    (kal_est_px), .est_py (kal_est_py), .est_pz (kal_est_pz),
        .est_vx    (kal_est_vx), .est_vy (kal_est_vy), .est_vz (kal_est_vz),
        .data_valid(kal_data_valid)
    );

    // -------------------------------------------------------------------------
    // Floating Point Divider (for covariance updates)
    // -------------------------------------------------------------------------
    logic fp_div_start;
    logic signed [STATE_W-1:0] fp_div_dividend, fp_div_divisor;
    logic signed [STATE_W-1:0] fp_div_quotient;
    logic fp_div_done;

    fp_divider #(.DATA_W(STATE_W)) u_fp_div (
        .clk      (clk), .rst_n (rst_n),
        .start    (fp_div_start),
        .dividend (fp_div_dividend),
        .divisor  (fp_div_divisor),
        .quotient (fp_div_quotient),
        .done     (fp_div_done)
    );

    // -------------------------------------------------------------------------
    // GPS EKF update
    // -------------------------------------------------------------------------
    logic signed [STATE_W-1:0] state_gps  [0:STATES-1];
    logic signed [P_W-1:0]     p_gps      [0:STATES-1];
    logic                       gps_upd_valid;

    ekf_gps_update #(.STATES(STATES), .STATE_W(STATE_W), .P_W(P_W)) u_gps_upd (
        .clk       (clk), .rst_n (rst_n),
        .ce_10hz   (ce_10hz & gps_valid),
        .gps_lat   (gps_lat), .gps_lon (gps_lon), .gps_alt (gps_alt),
        .gps_vn    (gps_vn), .gps_ve  (gps_ve),
        .R_lat     (R_lat), .R_lon (R_lon), .R_alt (R_alt_gps), .R_vn (R_vn), .R_ve (R_ve),
        .state_in  (state_in),  .p_diag_in  (p_diag_in),
        .state_out (state_gps), .p_diag_out (p_gps),
        .valid     (gps_upd_valid)
    );

    // Mux: use GPS update output or pass-through
    logic signed [STATE_W-1:0] state_after_gps [0:STATES-1];
    logic signed [P_W-1:0]     p_after_gps     [0:STATES-1];

    always_comb begin
        for (int s = 0; s < STATES; s++) begin
            state_after_gps[s] = gps_upd_valid ? state_gps[s] : state_in[s];
            p_after_gps[s]     = gps_upd_valid ? p_gps[s]     : p_diag_in[s];
        end
    end

    // -------------------------------------------------------------------------
    // Baro EKF update
    // -------------------------------------------------------------------------
    logic signed [STATE_W-1:0] state_baro [0:STATES-1];
    logic signed [P_W-1:0]     p_baro     [0:STATES-1];
    logic                       baro_upd_valid;

    ekf_baro_update #(.STATES(STATES), .STATE_W(STATE_W), .P_W(P_W)) u_baro_upd (
        .clk       (clk), .rst_n (rst_n),
        .ce_50hz   (ce_50hz & baro_valid),
        .baro_alt  (baro_lpf),
        .baro_noise(R_baro),
        .state_in  (state_after_gps), .p_diag_in  (p_after_gps),
        .state_out (state_baro),      .p_diag_out (p_baro),
        .valid     (baro_upd_valid)
    );

    logic signed [STATE_W-1:0] state_after_baro [0:STATES-1];
    logic signed [P_W-1:0]     p_after_baro     [0:STATES-1];

    always_comb begin
        for (int s = 0; s < STATES; s++) begin
            state_after_baro[s] = baro_upd_valid ? state_baro[s] : state_after_gps[s];
            p_after_baro[s]     = baro_upd_valid ? p_baro[s]     : p_after_gps[s];
        end
    end

    // -------------------------------------------------------------------------
    // Magnetometer EKF update
    // -------------------------------------------------------------------------
    logic signed [STATE_W-1:0] state_mag [0:STATES-1];
    logic signed [P_W-1:0]     p_mag     [0:STATES-1];
    logic                       mag_upd_valid;

    ekf_mag_update #(.STATES(STATES), .STATE_W(STATE_W), .P_W(P_W)) u_mag_upd (
        .clk         (clk), .rst_n (rst_n),
        .ce_10hz     (ce_10hz & mag_valid_flag),
        .mag_heading (mag_heading_lpf),
        .mag_noise   (R_mag),
        .state_in    (state_after_baro), .p_diag_in  (p_after_baro),
        .state_out   (state_mag),        .p_diag_out (p_mag),
        .valid       (mag_upd_valid)
    );

    // -------------------------------------------------------------------------
    // Final output
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < STATES; s++) begin
                state_out[s]  <= '0;
                p_diag_out[s] <= 32'sh00010000;
            end
            ekf_valid <= 1'b0;
        end else begin
            ekf_valid <= mag_upd_valid | baro_upd_valid | gps_upd_valid;
            for (int s = 0; s < STATES; s++) begin
                state_out[s]  <= mag_upd_valid  ? state_mag[s]       :
                                  baro_upd_valid ? state_baro[s]      :
                                  gps_upd_valid  ? state_after_gps[s] :
                                                   state_in[s];
                p_diag_out[s] <= mag_upd_valid  ? p_mag[s]           :
                                  baro_upd_valid ? p_baro[s]          :
                                  gps_upd_valid  ? p_after_gps[s]     :
                                                   p_diag_in[s];
            end
        end
    end

endmodule