// =============================================================================
// File        : ekf_mag_update.sv
// Module      : ekf_mag_update
// Description : Magnetometer scalar heading EKF update (10 Hz).
//               Observes yaw (state index 2) from magnetometer heading.
//               Gimbal lock guard: reject update when |pitch| > 70°.
// =============================================================================

`timescale 1ns/1ps

module ekf_mag_update #(
    parameter int STATES  = 9,
    parameter int STATE_W = 32,  // Q4.28
    parameter int P_W     = 32   // Q16.16
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_10hz,

    // Magnetometer heading (Q4.28 radians)
    input  logic signed [STATE_W-1:0] mag_heading,
    input  logic [P_W-1:0]            mag_noise,

    // EKF state & covariance
    input  logic signed [STATE_W-1:0] state_in  [0:STATES-1],
    input  logic signed [P_W-1:0]     p_diag_in [0:STATES-1],

    // Updated outputs
    output logic signed [STATE_W-1:0] state_out  [0:STATES-1],
    output logic signed [P_W-1:0]     p_diag_out [0:STATES-1],
    output logic                       valid
);

    localparam int YAW_IDX   = 2;
    localparam int PITCH_IDX = 1;
    // 70° in Q4.28 radians: 70 × π/180 × 2^28 = 329,246,899 ≈ 32'sh139D2B33
    localparam logic signed [STATE_W-1:0] GIMBAL_THRESH = 32'sh139D2B33;

    // Gimbal lock guard: |pitch| > 70°
    logic gimbal_lock;
    logic signed [STATE_W-1:0] abs_pitch;

    always_comb begin
        abs_pitch   = (state_in[PITCH_IDX][STATE_W-1]) ?
                      -state_in[PITCH_IDX] : state_in[PITCH_IDX];
        gimbal_lock = (abs_pitch > GIMBAL_THRESH);
    end

    logic signed [STATE_W-1:0] innov;
    logic signed [P_W-1:0]     S;
    logic accept_gate;
    logic gate_valid;

    always_comb begin
        innov = mag_heading - state_in[YAW_IDX];
        // Wrap to ±π (Q4.28: π = 32'sh06487ED5)
        if      (innov >  32'sh06487ED5) innov = innov - 32'sh0C90FDAA;
        else if (innov < -32'sh06487ED5) innov = innov + 32'sh0C90FDAA;
        S = p_diag_in[YAW_IDX] + $signed({1'b0, mag_noise});
    end

    innovation_gate #(.DATA_W(STATE_W)) u_gate (
        .clk       (clk), .rst_n (rst_n),
        .valid_in  (ce_10hz & ~gimbal_lock),
        .innov     (innov),
        .innov_cov (S),
        .accept    (accept_gate),
        .valid_out (gate_valid)
    );

    logic signed [P_W-1:0] K_yaw;
    logic signed [63:0]    K_innov;
    logic signed [P_W-1:0] omk;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < STATES; s++) begin
                state_out[s]  <= '0;
                p_diag_out[s] <= 32'sh00010000;
            end
            valid <= 1'b0;
        end else begin
            valid <= gate_valid | (ce_10hz & gimbal_lock);

            for (int s = 0; s < STATES; s++) begin
                state_out[s]  <= state_in[s];
                p_diag_out[s] <= p_diag_in[s];
            end

            if (gate_valid && accept_gate && S != 0) begin
                K_yaw    = (p_diag_in[YAW_IDX] <<< 16) / S;
                K_innov  = K_yaw * innov;
                state_out[YAW_IDX]  <= state_in[YAW_IDX] + (K_innov >>> 16);

                omk = 32'sh00010000 - K_yaw;
                p_diag_out[YAW_IDX] <= ((omk * omk) >>> 16) *
                                        (p_diag_in[YAW_IDX] >>> 16) +
                                       ((K_yaw * K_yaw) >>> 16) *
                                        ($signed({1'b0, mag_noise}) >>> 16);
            end
        end
    end

endmodule