`timescale 1ns/1ps

module module5_uav_sensor_ekf #(
    parameter int STATES  = 9,
    parameter int STATE_W = 32,
    parameter int P_W     = 32
)(
    input logic clk,
    input logic rst_n,

    input logic ce_100hz,
    input logic ce_50hz,
    input logic ce_10hz,

    // GPS
    input logic signed [31:0] gps_lat,
    input logic signed [31:0] gps_lon,
    input logic signed [31:0] gps_alt,
    input logic signed [31:0] gps_vn,
    input logic signed [31:0] gps_ve,
    input logic gps_valid,

    // IMU
    input logic signed [31:0] imu_ax,
    input logic signed [31:0] imu_ay,
    input logic signed [31:0] imu_az,

    // CONFIG
    input logic [31:0] R_baro,
    input logic [31:0] R_mag,
    input logic [31:0] R_lat,
    input logic [31:0] R_lon,
    input logic [31:0] R_alt_gps,
    input logic [31:0] R_vn,
    input logic [31:0] R_ve,

    // I2C
    output logic i2c_scl,
    inout wire i2c_sda,

    // STATUS
    output logic ekf_valid,
    output logic baro_valid,
    output logic mag_valid_flag,

    output logic [7:0] dbg_status,
    output logic [15:0] dbg_state
);

    // =========================================================
    // INTERNAL STORAGE
    // =========================================================

    logic signed [31:0] state_reg [0:STATES-1];
    logic signed [31:0] p_reg [0:STATES-1];

    // =========================================================
    // I2C MASTER SIGNALS
    // =========================================================

    logic       i2c_start;
    logic [6:0] i2c_slave_addr;
    logic       i2c_rw;
    logic [7:0] i2c_write_data;
    logic [7:0] i2c_read_data;
    logic       i2c_busy;
    logic       i2c_ack_error;

    logic [23:0] i2c_timer;

    typedef enum logic [1:0] {
        I2C_IDLE,
        I2C_BARO,
        I2C_MAG,
        I2C_WAIT
    } i2c_state_t;

    i2c_state_t i2c_state;

    logic signed [31:0] baro_alt_raw;
    logic signed [31:0] mag_heading_raw;

    // =========================================================
    // I2C FSM
    // =========================================================

    always_ff @(posedge clk or negedge rst_n) begin

        if(!rst_n) begin

            i2c_timer       <= 0;
            i2c_state       <= I2C_IDLE;

            i2c_start       <= 0;
            i2c_slave_addr  <= 0;
            i2c_rw          <= 1;
            i2c_write_data  <= 0;

            baro_alt_raw    <= 0;
            mag_heading_raw <= 0;

        end
        else begin

            i2c_timer <= i2c_timer + 1;
            i2c_start <= 0;

            case(i2c_state)

                I2C_IDLE: begin

                    if(i2c_timer == 24'd1000000) begin

                        i2c_start      <= 1'b1;
                        i2c_slave_addr <= 7'h76;
                        i2c_rw         <= 1'b1;

                        i2c_state <= I2C_BARO;
                    end
                end

                I2C_BARO: begin

                    if(!i2c_busy) begin

                        baro_alt_raw <= {
                            24'd0,
                            i2c_read_data
                        };

                        i2c_start      <= 1'b1;
                        i2c_slave_addr <= 7'h1E;
                        i2c_rw         <= 1'b1;

                        i2c_state <= I2C_MAG;
                    end
                end

                I2C_MAG: begin

                    if(!i2c_busy) begin

                        mag_heading_raw <= {
                            24'd0,
                            i2c_read_data
                        };

                        i2c_state <= I2C_WAIT;
                    end
                end

                I2C_WAIT: begin

                    if(i2c_timer == 24'd2000000) begin

                        i2c_timer <= 0;
                        i2c_state <= I2C_IDLE;

                    end
                end

            endcase
        end
    end

    // =========================================================
    // I2C INSTANCE
    // =========================================================

    (* dont_touch = "true" *)
    (* keep_hierarchy = "yes" *)
    (* keep = "true" *)
    i2c_master #(
        .CLK_HZ(50000000),
        .I2C_HZ(100000)
    ) u_i2c (
        .clk(clk),
        .rst_n(rst_n),
        .start(i2c_start),
        .slave_addr(i2c_slave_addr),
        .rw(i2c_rw),
        .write_data(i2c_write_data),
        .read_data(i2c_read_data),
        .busy(i2c_busy),
        .ack_error(i2c_ack_error),
        .sda(i2c_sda),
        .scl(i2c_scl)
    );

    assign baro_valid     = ~i2c_ack_error;
    assign mag_valid_flag = ~i2c_ack_error;

    // =========================================================
    // LPF
    // =========================================================

    logic signed [31:0] baro_lpf;
    logic signed [31:0] mag_heading_lpf;

    (* dont_touch = "true" *)
    lpf #(
        .DATA_W(32),
        .ORDER(1)
    ) u_lpf_baro (
        .clk(clk),
        .rst_n(rst_n),
        .enable(ce_50hz),
        .data_in(baro_alt_raw),
        .cutoff_freq(16'd8),
        .data_out(baro_lpf)
    );

    (* dont_touch = "true" *)
    lpf #(
        .DATA_W(32),
        .ORDER(1)
    ) u_lpf_mag (
        .clk(clk),
        .rst_n(rst_n),
        .enable(ce_10hz),
        .data_in(mag_heading_raw),
        .cutoff_freq(16'd2),
        .data_out(mag_heading_lpf)
    );

    // =========================================================
    // KALMAN
    // =========================================================

    logic signed [31:0] kal_est_px;
    logic signed [31:0] kal_est_vx;
    logic kal_data_valid;

    (* dont_touch = "true" *)
    kalman_1v u_kalman (
        .clk(clk),
        .rst_n(rst_n),

        .predict_en(ce_100hz),
        .update_en(ce_10hz & gps_valid),

        .meas_px(gps_lat),
        .meas_py(gps_lon),
        .meas_pz(gps_alt),

        .accel_x(imu_ax),
        .accel_y(imu_ay),
        .accel_z(imu_az),

        .est_px(kal_est_px),
        .est_py(),
        .est_pz(),

        .est_vx(kal_est_vx),
        .est_vy(),
        .est_vz(),

        .data_valid(kal_data_valid)
    );

    // =========================================================
    // FP DIVIDER
    // =========================================================

    logic signed [31:0] fp_div_q;
    logic fp_div_done;

    (* dont_touch = "true" *)
    fp_divider #(
        .DATA_W(16)
    ) u_fp_div (
        .clk(clk),
        .rst_n(rst_n),
        .start(ce_10hz),
        .dividend(gps_alt[15:0]),
        .divisor(16'd8),
        .quotient(fp_div_q[15:0]),
        .done(fp_div_done)
    );

    // =========================================================
    // EKF WIRES
    // =========================================================

    logic signed [31:0] gps_state [0:STATES-1];
    logic signed [31:0] gps_p [0:STATES-1];

    logic signed [31:0] baro_state [0:STATES-1];
    logic signed [31:0] baro_p [0:STATES-1];

    logic signed [31:0] mag_state [0:STATES-1];
    logic signed [31:0] mag_p [0:STATES-1];

    logic gps_valid_upd;
    logic baro_valid_upd;
    logic mag_valid_upd;

    // =========================================================
    // EKF INSTANCES
    // =========================================================

    ekf_gps_update u_gps_upd (
        .clk(clk),
        .rst_n(rst_n),
        .ce_10hz(ce_10hz & gps_valid),

        .gps_lat(gps_lat),
        .gps_lon(gps_lon),
        .gps_alt(gps_alt),
        .gps_vn(gps_vn),
        .gps_ve(gps_ve),

        .R_lat(R_lat),
        .R_lon(R_lon),
        .R_alt(R_alt_gps),
        .R_vn(R_vn),
        .R_ve(R_ve),

        .state_in(state_reg),
        .p_diag_in(p_reg),

        .state_out(gps_state),
        .p_diag_out(gps_p),

        .valid(gps_valid_upd)
    );

    ekf_baro_update u_baro_upd (
        .clk(clk),
        .rst_n(rst_n),
        .ce_50hz(ce_50hz),

        .baro_alt(baro_lpf),
        .baro_noise(R_baro),

        .state_in(state_reg),
        .p_diag_in(p_reg),

        .state_out(baro_state),
        .p_diag_out(baro_p),

        .valid(baro_valid_upd)
    );

    ekf_mag_update u_mag_upd (
        .clk(clk),
        .rst_n(rst_n),
        .ce_10hz(ce_10hz),

        .mag_heading(mag_heading_lpf),
        .mag_noise(R_mag),

        .state_in(state_reg),
        .p_diag_in(p_reg),

        .state_out(mag_state),
        .p_diag_out(mag_p),

        .valid(mag_valid_upd)
    );

    // =========================================================
    // CENTRAL STATE UPDATE
    // SINGLE DRIVER ONLY
    // =========================================================

    integer i;

    always_ff @(posedge clk or negedge rst_n) begin

        if(!rst_n) begin

            ekf_valid <= 0;

            for(i=0;i<STATES;i=i+1) begin

                state_reg[i] <= 0;
                p_reg[i] <= 32'h00010000;

            end
        end
        else begin

            ekf_valid <= 0;

            // =================================================
            // PREDICTION STEP
            // =================================================

            if(ce_100hz) begin

                state_reg[3] <= state_reg[3] + (imu_ax >>> 4);
                state_reg[4] <= state_reg[4] + (imu_ay >>> 4);
                state_reg[5] <= state_reg[5] + (imu_az >>> 4);

                state_reg[6] <= state_reg[6] + (state_reg[3] >>> 6);
                state_reg[7] <= state_reg[7] + (state_reg[4] >>> 6);
                state_reg[8] <= state_reg[8] + (state_reg[5] >>> 6);

            end

            // =================================================
            // GPS UPDATE
            // =================================================

            if(gps_valid_upd) begin

                ekf_valid <= 1;

                for(i=0;i<STATES;i=i+1) begin

                    state_reg[i] <= gps_state[i];
                    p_reg[i] <= gps_p[i];

                end
            end

            // =================================================
            // BARO UPDATE
            // =================================================

            else if(baro_valid_upd) begin

                ekf_valid <= 1;

                for(i=0;i<STATES;i=i+1) begin

                    state_reg[i] <= baro_state[i];
                    p_reg[i] <= baro_p[i];

                end
            end

            // =================================================
            // MAG UPDATE
            // =================================================

            else if(mag_valid_upd) begin

                ekf_valid <= 1;

                for(i=0;i<STATES;i=i+1) begin

                    state_reg[i] <= mag_state[i];
                    p_reg[i] <= mag_p[i];

                end
            end
        end
    end

    // =========================================================
    // DEBUG
    // =========================================================

    assign dbg_status = {
        i2c_busy,
        i2c_ack_error,
        fp_div_done,
        kal_data_valid,
        gps_valid_upd,
        baro_valid_upd,
        mag_valid_upd,
        ekf_valid
    };

    assign dbg_state =
          state_reg[8][15:0]
        ^ baro_lpf[15:0]
        ^ mag_heading_lpf[15:0]
        ^ fp_div_q[15:0]
        ^ {8'd0,i2c_read_data};

endmodule
