// =============================================================================
// File        : uav_ekf_predict.sv
// Module      : uav_ekf_predict
// Description : UAV MOD_4 - EKF Prediction top-level.
//               Integrates imu_scaler, rotation_matrix, ekf_state_predict,
//               and p_update_predict.
//               Runs 9-state EKF prediction at 100 Hz.
//               SPI interface to ICM-42688 for raw IMU data.
// =============================================================================

`timescale 1ns/1ps

module module4_uav_ekf_predict #(
    parameter int CLK_HZ  = 50_000_000,
    parameter int STATES  = 9,
    parameter int STATE_W = 32,  // Q4.28
    parameter int P_W     = 32,  // Q16.16
    parameter int SPI_DIV = 10   // SPI clock divider
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_100hz,

    // SPI interface (ICM-42688)
    output logic spi_sclk,
    output logic spi_mosi,
    input  logic spi_miso,
    output logic spi_cs_n,

    // Calibration biases (Q4.28, from MOD_10)
    input  logic signed [STATE_W-1:0] gyro_bias [0:2],
    input  logic signed [STATE_W-1:0] accel_bias[0:2],

    // Process noise Q diagonal (Q16.16, from MOD_10)
    input  logic [P_W-1:0] q_diag [0:STATES-1],

    // Current state & covariance in (from BRAM / MOD_11)
    input  logic signed [STATE_W-1:0] state_in  [0:STATES-1],
    input  logic signed [P_W-1:0]     p_diag_in [0:STATES-1],

    // Predicted state & covariance out
    output logic signed [STATE_W-1:0] state_out [0:STATES-1],
    output logic signed [P_W-1:0]     p_diag_out[0:STATES-1],
    output logic                       ekf_valid
);

    // -------------------------------------------------------------------------
    // SPI master for ICM-42688 (reuse spi_master.sv)
    // We read 12 bytes: GX_H/L GY_H/L GZ_H/L AX_H/L AY_H/L AZ_H/L
    // -------------------------------------------------------------------------
    localparam int SPI_BYTES = 13; // 1 addr + 12 data
    logic [7:0]  spi_tx_data;
    logic        spi_tx_valid;
    logic        spi_tx_ready;
    logic [7:0]  spi_rx_data;
    logic        spi_rx_valid;

        spi_master #(
        .CLK_DIV  (SPI_DIV),
        .CPOL     (0),
        .CPHA     (0),
        .DATA_W   (8)
    ) u_spi (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_data  (spi_tx_data),
        .tx_valid (spi_tx_valid),
        .tx_ready (spi_tx_ready),
        .rx_data  (spi_rx_data),
        .rx_valid (spi_rx_valid),
        .sclk     (spi_sclk),
        .mosi     (spi_mosi),
        .miso     (spi_miso),
        .cs_n     (spi_cs_n)
    );

    // -------------------------------------------------------------------------
    // IMU read state machine (triggered at 100 Hz)
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        IMU_IDLE, IMU_ADDR, IMU_READ_WAIT, IMU_DONE
    } imu_state_t;

    imu_state_t imu_state;
    logic [3:0]  byte_cnt;
    logic [7:0]  raw_bytes [0:11];
    logic        raw_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            imu_state    <= IMU_IDLE;
            byte_cnt     <= '0;
            spi_tx_valid <= 1'b0;
            spi_tx_data  <= 8'h00;
            raw_valid    <= 1'b0;
        end else begin
            spi_tx_valid <= 1'b0;
            raw_valid    <= 1'b0;
            case (imu_state)
                IMU_IDLE: begin
                    if (ce_100hz) begin
                        // ICM-42688 gyro/accel base address 0x1D, read (MSB=1)
                        spi_tx_data  <= 8'h9D;
                        spi_tx_valid <= 1'b1;
                        byte_cnt     <= 4'd0;
                        imu_state    <= IMU_ADDR;
                    end
                end
                IMU_ADDR: begin
                    if (spi_tx_ready) begin
                        spi_tx_data  <= 8'h00; // dummy byte
                        spi_tx_valid <= 1'b1;
                        imu_state    <= IMU_READ_WAIT;
                    end
                end
                IMU_READ_WAIT: begin
                    if (spi_rx_valid && byte_cnt < 12) begin
                        raw_bytes[byte_cnt] <= spi_rx_data;
                        byte_cnt <= byte_cnt + 1'b1;
                        if (byte_cnt < 11) begin
                            spi_tx_valid <= 1'b1;
                            spi_tx_data  <= 8'h00;
                        end else begin
                            imu_state <= IMU_DONE;
                        end
                    end
                end
                IMU_DONE: begin
                    raw_valid <= 1'b1;
                    imu_state <= IMU_IDLE;
                end
                default: imu_state <= IMU_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Reconstruct 16-bit signed raw values from byte pairs
    // -------------------------------------------------------------------------
    logic signed [15:0] gx_r, gy_r, gz_r, ax_r, ay_r, az_r;
    always_comb begin
        gx_r = {raw_bytes[0],  raw_bytes[1]};
        gy_r = {raw_bytes[2],  raw_bytes[3]};
        gz_r = {raw_bytes[4],  raw_bytes[5]};
        ax_r = {raw_bytes[6],  raw_bytes[7]};
        ay_r = {raw_bytes[8],  raw_bytes[9]};
        az_r = {raw_bytes[10], raw_bytes[11]};
    end

    // -------------------------------------------------------------------------
    // IMU scaler
    // -------------------------------------------------------------------------
    logic signed [STATE_W-1:0] gx, gy, gz, a_x, a_y, a_z;
    logic imu_scaled_valid;

    imu_scaler #(.RAW_W(16), .OUT_W(STATE_W)) u_scale (
        .clk          (clk),
        .rst_n        (rst_n),
        .valid_in     (raw_valid),
        .gx_raw       (gx_r), .gy_raw (gy_r), .gz_raw (gz_r),
        .ax_raw       (ax_r), .ay_raw (ay_r), .az_raw (az_r),
        .gyro_bias_x  (gyro_bias[0]),
        .gyro_bias_y  (gyro_bias[1]),
        .gyro_bias_z  (gyro_bias[2]),
        .accel_bias_x (accel_bias[0]),
        .accel_bias_y (accel_bias[1]),
        .accel_bias_z (accel_bias[2]),
        .gx (gx), .gy (gy), .gz (gz),
        .ax (a_x), .ay (a_y), .az (a_z),
        .valid_out (imu_scaled_valid)
    );

    // -------------------------------------------------------------------------
    // CORDIC for sin/cos computation
    // -------------------------------------------------------------------------
    logic [15:0] cordic_angle;
    logic cordic_start;
    logic [15:0] cordic_cos, cordic_sin;
    logic cordic_done;

    cordic #(.ITERATIONS(16)) u_cordic (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (cordic_start),
        .angle     (cordic_angle),
        .cos_out   (cordic_cos),
        .sin_out   (cordic_sin),
        .done      (cordic_done)
    );

    // -------------------------------------------------------------------------
    // Rotation matrix
    // -------------------------------------------------------------------------
    logic signed [15:0] R [0:2][0:2];
    logic rot_valid;

    rotation_matrix #(.ANGLE_W(16), .OUT_W(16), .ITER(16)) u_rot (
        .clk   (clk), .rst_n (rst_n),
        .start (ce_100hz),
        .roll  (state_in[0][15:0]),
        .pitch (state_in[1][15:0]),
        .yaw   (state_in[2][15:0]),
        .R     (R),
        .valid (rot_valid)
    );

    // Extract trig terms from rotation matrix
    logic signed [15:0] sin_roll_q, cos_roll_q, sin_pitch_q, cos_pitch_q, tan_pitch_q;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sin_roll_q <= '0; cos_roll_q <= '0;
            sin_pitch_q <= '0; cos_pitch_q <= '0; tan_pitch_q <= '0;
        end else if (rot_valid) begin
            sin_roll_q  <= R[2][1]; // sinφ·cosθ / cosθ = sinφ
            cos_roll_q  <= R[2][2]; // cosφ·cosθ / cosθ = cosφ
            sin_pitch_q <= -R[2][0]; // -sinθ → sinθ (negated)
            cos_pitch_q <= R[0][0];  // cosψ·cosθ → extract cosθ
            // tanθ approximation: sinθ/cosθ
            tan_pitch_q <= (R[0][0] != 0) ? ((-R[2][0]) <<< 12) / R[0][0] : 16'sd0;
        end
    end

    // -------------------------------------------------------------------------
    // EKF state prediction
    // -------------------------------------------------------------------------
    logic signed [STATE_W-1:0] pred_state [0:STATES-1];
    logic pred_valid;

    ekf_state_predict #(.STATE_W(STATE_W)) u_pred (
        .clk       (clk), .rst_n (rst_n), .ce_100hz (ce_100hz & imu_scaled_valid),
        .p         (gx), .q (gy), .r (gz),
        .ax        (a_x), .ay (a_y), .az (a_z),
        .sin_roll  (sin_roll_q), .cos_roll  (cos_roll_q),
        .sin_pitch (sin_pitch_q),.cos_pitch (cos_pitch_q),
        .tan_pitch (tan_pitch_q),
        .roll_in   (state_in[0]), .pitch_in (state_in[1]), .yaw_in (state_in[2]),
        .vN_in     (state_in[3]), .vE_in    (state_in[4]), .vD_in  (state_in[5]),
        .lat_in    (state_in[6]), .lon_in   (state_in[7]), .alt_in (state_in[8]),
        .roll_out  (pred_state[0]), .pitch_out (pred_state[1]), .yaw_out (pred_state[2]),
        .vN_out    (pred_state[3]), .vE_out   (pred_state[4]), .vD_out  (pred_state[5]),
        .lat_out   (pred_state[6]), .lon_out  (pred_state[7]), .alt_out (pred_state[8]),
        .valid     (pred_valid)
    );

    // -------------------------------------------------------------------------
    // SQRT for covariance operations
    // -------------------------------------------------------------------------
    logic [P_W-1:0] sqrt_in;
    logic sqrt_start;
    logic [P_W-1:0] sqrt_out_val;
    logic sqrt_done;

    sqrt #(.DATA_W(P_W)) u_sqrt (
        .clk       (clk),
        .rst_n     (rst_n),
        .start     (sqrt_start),
        .x         (sqrt_in),
        .sqrt_out  (sqrt_out_val),
        .done      (sqrt_done)
    );

    assign sqrt_start = pred_valid;
    assign sqrt_in = p_diag_out[0];

    // -------------------------------------------------------------------------
    // Covariance prediction
    // -------------------------------------------------------------------------
    logic signed [P_W-1:0] f_diag_ones [0:STATES-1];
    always_comb begin
        for (int s = 0; s < STATES; s++) f_diag_ones[s] = 32'sh00010000; // 1.0 Q16.16
    end

    p_update_predict #(.STATES(STATES), .P_W(P_W), .Q_W(P_W)) u_p_pred (
        .clk         (clk), .rst_n (rst_n), .ce_100hz (ce_100hz),
        .q_diag      (q_diag),
        .p_diag_in   (p_diag_in),
        .f_diag      (f_diag_ones),
        .p_diag_out  (p_diag_out),
        .valid       ()
    );

    // -------------------------------------------------------------------------
    // Output assignments
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < STATES; s++) state_out[s] <= '0;
            ekf_valid <= 1'b0;
        end else begin
            ekf_valid <= pred_valid;
            if (pred_valid) begin
                for (int s = 0; s < STATES; s++)
                    state_out[s] <= pred_state[s];
            end
        end
    end

endmodule