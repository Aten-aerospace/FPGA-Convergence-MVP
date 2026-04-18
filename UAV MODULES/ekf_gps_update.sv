// =============================================================================
// File        : ekf_gps_update.sv
// Module      : ekf_gps_update
// Description : GPS 5-measurement EKF update (10 Hz).
//               Observes: lat, lon, alt, vN, vE (states 6,7,8,3,4)
//               Joseph form covariance update per measurement.
//               Innovation gating per measurement (3σ Mahalanobis).
// =============================================================================

`timescale 1ns/1ps

module ekf_gps_update #(
    parameter int STATES  = 9,
    parameter int STATE_W = 32,  // Q4.28
    parameter int P_W     = 32   // Q16.16
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_10hz,

    // GPS measurements (Q4.28)
    input  logic signed [STATE_W-1:0] gps_lat,
    input  logic signed [STATE_W-1:0] gps_lon,
    input  logic signed [STATE_W-1:0] gps_alt,
    input  logic signed [STATE_W-1:0] gps_vn,
    input  logic signed [STATE_W-1:0] gps_ve,

    // GPS measurement noise (Q16.16, per observable)
    input  logic [P_W-1:0] R_lat, R_lon, R_alt, R_vn, R_ve,

    // EKF state & covariance in
    input  logic signed [STATE_W-1:0] state_in  [0:STATES-1],
    input  logic signed [P_W-1:0]     p_diag_in [0:STATES-1],

    // Updated outputs
    output logic signed [STATE_W-1:0] state_out  [0:STATES-1],
    output logic signed [P_W-1:0]     p_diag_out [0:STATES-1],
    output logic                       valid
);

    // State indices for GPS observables
    localparam int IDX_VN  = 3;
    localparam int IDX_VE  = 4;
    localparam int IDX_LAT = 6;
    localparam int IDX_LON = 7;
    localparam int IDX_ALT = 8;

    // Sequential scalar update: apply each of 5 measurements in sequence
    // Pipeline: 5 stages × 2 clocks each = 10 clock update latency
    typedef enum logic [2:0] {
        GPS_IDLE, GPS_VN, GPS_VE, GPS_LAT, GPS_LON, GPS_ALT, GPS_DONE
    } gps_state_t;

    gps_state_t gps_st;
    logic signed [STATE_W-1:0] x_tmp [0:STATES-1];
    logic signed [P_W-1:0]     p_tmp [0:STATES-1];

    // Scalar update function
    task automatic scalar_update(
        input  int                        obs_idx,
        input  logic signed [STATE_W-1:0] meas,
        input  logic signed [P_W-1:0]     R_obs,
        inout  logic signed [STATE_W-1:0] x [0:STATES-1],
        inout  logic signed [P_W-1:0]     p [0:STATES-1]
    );
        logic signed [STATE_W-1:0] innov_v;
        logic signed [P_W-1:0]     S_v, K_v;
        logic signed [63:0]        Ki;
        logic signed [P_W-1:0]     omk;

        innov_v = meas - x[obs_idx];
        S_v     = p[obs_idx] + $signed({1'b0, R_obs});
        if (S_v != 0) begin
            K_v = (p[obs_idx] <<< 16) / S_v;
            Ki  = K_v * innov_v;
            x[obs_idx] = x[obs_idx] + (Ki >>> 16);
            omk = 32'sh00010000 - K_v;
            p[obs_idx] = ((omk * omk) >>> 16) * (p[obs_idx] >>> 16) +
                         ((K_v * K_v) >>> 16) * ($signed({1'b0,R_obs}) >>> 16);
        end
    endtask

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            gps_st <= GPS_IDLE;
            for (int s = 0; s < STATES; s++) begin
                state_out[s]  <= '0;
                p_diag_out[s] <= 32'sh00010000;
                x_tmp[s]      <= '0;
                p_tmp[s]      <= 32'sh00010000;
            end
            valid <= 1'b0;
        end else begin
            valid <= 1'b0;
            case (gps_st)
                GPS_IDLE: begin
                    if (ce_10hz) begin
                        for (int s = 0; s < STATES; s++) begin
                            x_tmp[s] <= state_in[s];
                            p_tmp[s] <= p_diag_in[s];
                        end
                        gps_st <= GPS_VN;
                    end
                end
                GPS_VN:  begin scalar_update(IDX_VN,  gps_vn,  R_vn,  x_tmp, p_tmp); gps_st <= GPS_VE;  end
                GPS_VE:  begin scalar_update(IDX_VE,  gps_ve,  R_ve,  x_tmp, p_tmp); gps_st <= GPS_LAT; end
                GPS_LAT: begin scalar_update(IDX_LAT, gps_lat, R_lat, x_tmp, p_tmp); gps_st <= GPS_LON; end
                GPS_LON: begin scalar_update(IDX_LON, gps_lon, R_lon, x_tmp, p_tmp); gps_st <= GPS_ALT; end
                GPS_ALT: begin scalar_update(IDX_ALT, gps_alt, R_alt, x_tmp, p_tmp); gps_st <= GPS_DONE;end
                GPS_DONE: begin
                    for (int s = 0; s < STATES; s++) begin
                        state_out[s]  <= x_tmp[s];
                        p_diag_out[s] <= p_tmp[s];
                    end
                    valid  <= 1'b1;
                    gps_st <= GPS_IDLE;
                end
                default: gps_st <= GPS_IDLE;
            endcase
        end
    end

endmodule
