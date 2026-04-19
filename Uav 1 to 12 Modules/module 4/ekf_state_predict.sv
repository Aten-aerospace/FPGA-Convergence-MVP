// =============================================================================
// File        : ekf_state_predict.sv
// Module      : ekf_state_predict
// Description : 9-state EKF prediction step at 100 Hz.
//               State vector: [roll, pitch, yaw, vN, vE, vD, lat, lon, alt]
//               Euler kinematic equations for attitude rates:
//                 φ̇ = p + q·sinφ·tanθ + r·cosφ·tanθ
//                 θ̇ = q·cosφ - r·sinφ
//                 ψ̇ = (q·sinφ + r·cosφ)/cosθ
//               Velocity update with gravity compensation.
//               Position integration via NED velocity.
//               All arithmetic in Q4.28 fixed-point.
// =============================================================================

`timescale 1ns/1ps

module ekf_state_predict #(
    parameter int STATE_W = 32,  // Q4.28 state variables
    parameter int DT_Q    = 28,  // fractional bits of dt
    // dt = 0.01 s at 100 Hz → Q4.28: 0.01 × 2^28 = 2,684,355 ≈ 32'sd2684355
    parameter int DT_VAL  = 2684355
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_100hz,

    // IMU inputs (Q4.28, from imu_scaler)
    input  logic signed [STATE_W-1:0] p,   // roll  rate (rad/s)
    input  logic signed [STATE_W-1:0] q,   // pitch rate (rad/s)
    input  logic signed [STATE_W-1:0] r,   // yaw   rate (rad/s)
    input  logic signed [STATE_W-1:0] ax,  // accel NED X (m/s²)
    input  logic signed [STATE_W-1:0] ay,  // accel NED Y
    input  logic signed [STATE_W-1:0] az,  // accel NED Z

    // Trig terms from rotation_matrix (Q1.15 as Q4.28-compatible)
    input  logic signed [15:0] sin_roll,
    input  logic signed [15:0] cos_roll,
    input  logic signed [15:0] sin_pitch,
    input  logic signed [15:0] cos_pitch,
    input  logic signed [15:0] tan_pitch,

    // Current state in
    input  logic signed [STATE_W-1:0] roll_in,
    input  logic signed [STATE_W-1:0] pitch_in,
    input  logic signed [STATE_W-1:0] yaw_in,
    input  logic signed [STATE_W-1:0] vN_in,
    input  logic signed [STATE_W-1:0] vE_in,
    input  logic signed [STATE_W-1:0] vD_in,
    input  logic signed [STATE_W-1:0] lat_in,
    input  logic signed [STATE_W-1:0] lon_in,
    input  logic signed [STATE_W-1:0] alt_in,

    // Predicted state out
    output logic signed [STATE_W-1:0] roll_out,
    output logic signed [STATE_W-1:0] pitch_out,
    output logic signed [STATE_W-1:0] yaw_out,
    output logic signed [STATE_W-1:0] vN_out,
    output logic signed [STATE_W-1:0] vE_out,
    output logic signed [STATE_W-1:0] vD_out,
    output logic signed [STATE_W-1:0] lat_out,
    output logic signed [STATE_W-1:0] lon_out,
    output logic signed [STATE_W-1:0] alt_out,
    output logic                       valid
);

    // Gravity constant (Q4.28): 9.80665 × 2^28 = 2,635,041,751 ≈ 32'sd2635041751 (overflow 32-bit!)
    // Use Q4.20 internally: 9.80665 × 2^20 = 10,281,717
    localparam logic signed [STATE_W-1:0] GRAVITY_Q28 = 32'sh09D81B00; // 9.80665 × 2^24 shifted

    // Helpers: Q1.15 × Q4.28 >> 15 = Q4.28
    function automatic logic signed [STATE_W-1:0] mul_trig_state(
        input logic signed [15:0]      trig,
        input logic signed [STATE_W-1:0] val
    );
        logic signed [47:0] prod;
        prod = trig * val;
        return prod >>> 15;
    endfunction

    // dt multiply: val × DT_VAL >> 28 = val × 0.01
    function automatic logic signed [STATE_W-1:0] dt_mult(
        input logic signed [STATE_W-1:0] val
    );
        logic signed [63:0] prod;
        prod = val * DT_VAL;
        return prod >>> DT_Q;
    endfunction

    // Internal signals declared outside always block
    logic signed [STATE_W-1:0] phi_dot, theta_dot, psi_dot;
    logic signed [STATE_W-1:0] q_sinphi, r_cosphi;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            roll_out  <= '0; pitch_out <= '0; yaw_out  <= '0;
            vN_out    <= '0; vE_out    <= '0; vD_out   <= '0;
            lat_out   <= '0; lon_out   <= '0; alt_out  <= '0;
            valid     <= 1'b0;
        end else if (ce_100hz) begin
            valid <= 1'b1;

            // ---- Attitude kinematics ----------------------------------------
            // φ̇ = p + (q·sinφ + r·cosφ)·tanθ
            q_sinphi  = mul_trig_state(sin_roll, q);
            r_cosphi  = mul_trig_state(cos_roll, r);
            phi_dot   = p + mul_trig_state(tan_pitch, q_sinphi + r_cosphi);
            theta_dot = mul_trig_state(cos_roll, q) -
                        mul_trig_state(sin_roll, r);
            // ψ̇ = (q·sinφ + r·cosφ)/cosθ  (simplified: multiply by 1/cosθ)
            // Approximate 1/cosθ as cos_pitch^{-1}: use division approximation
            // For small angles use identity; here we use cos_pitch directly (reciprocal approximated)
            psi_dot   = (cos_pitch != 0) ?
                        ((q_sinphi + r_cosphi) <<< 15) / cos_pitch :
                        (q_sinphi + r_cosphi);

            roll_out  <= roll_in  + dt_mult(phi_dot);
            pitch_out <= pitch_in + dt_mult(theta_dot);
            yaw_out   <= yaw_in   + dt_mult(psi_dot);

            // ---- Velocity update with gravity --------------------------------
            // aN = Rbody→NED × accel + [0,0,g]
            // Approximate: vN += ax·dt, vE += ay·dt, vD += (az+g)·dt
            vN_out <= vN_in + dt_mult(ax);
            vE_out <= vE_in + dt_mult(ay);
            vD_out <= vD_in + dt_mult(az + GRAVITY_Q28);

            // ---- Position integration ----------------------------------------
            lat_out <= lat_in + dt_mult(vN_in);
            lon_out <= lon_in + dt_mult(vE_in);
            alt_out <= alt_in - dt_mult(vD_in); // NED: down positive
        end else begin
            valid <= 1'b0;
        end
    end

endmodule