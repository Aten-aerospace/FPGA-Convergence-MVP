// =============================================================================
// Module: ekf_measurement_model
// Subsystem: CS5 - Extended Kalman Filter
// Description: Computes the measurement function h(q) and the 6×7 measurement
//              Jacobian H(q) for a 7-state EKF fusing accelerometer + magnetometer.
//
//   Measurement vector (6 elements, Q15):
//     h_accel = R(q)·[0,0,1]  (gravity reference in body frame)
//     h_mag   = R(q)·[1,0,0]  (NED mag reference in body frame)
//
//   h_accel[0] = 2(q1·q3 + q0·q2)
//   h_accel[1] = 2(q2·q3 - q0·q1)
//   h_accel[2] = q0²-q1²-q2²+q3²
//   h_mag[0]   = q0²+q1²-q2²-q3²
//   h_mag[1]   = 2(q1·q2 + q0·q3)
//   h_mag[2]   = 2(q1·q3 - q0·q2)
//
//   H Jacobian rows (∂h/∂x, bias columns are zero):
//     H[0,:] = [2q2,  2q3,  2q0,  2q1, 0,0,0]  (∂h_a[0]/∂q)
//     H[1,:] = [-2q1,-2q0,  2q3,  2q2, 0,0,0]  (∂h_a[1]/∂q)
//     H[2,:] = [2q0, -2q1, -2q2,  2q3, 0,0,0]  (∂h_a[2]/∂q)
//     H[3,:] = [2q0,  2q1, -2q2, -2q3, 0,0,0]  (∂h_m[0]/∂q)
//     H[4,:] = [2q3,  2q2,  2q1,  2q0, 0,0,0]  (∂h_m[1]/∂q)
//     H[5,:] = [-2q2, 2q3, -2q0,  2q1, 0,0,0]  (∂h_m[2]/∂q)
//
//   Pipeline: 3 clock cycles (2 product stages + 1 combine stage).
//   Peak DSP48: 6 (Stage 1 uses 6, Stage 2 uses 4).
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module ekf_measurement_model (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        valid_in,

    // Unit quaternion input [w,x,y,z] in Q15
    input  logic signed [15:0] q_in [0:3],

    // Predicted measurement vector h(q) in Q15
    output logic signed [15:0] h_meas [0:5],

    // H Jacobian matrix (6 rows × 7 cols) in Q15
    output logic signed [15:0] H_mat  [0:5][0:6],

    output logic               valid_out
);

    // =========================================================================
    // Stage 1: register inputs, compute first 6 products (6 DSP48)
    //   p00 = q0², p11 = q1², p22 = q2², p33 = q3², p01 = q0·q1, p02 = q0·q2
    // =========================================================================
    logic signed [31:0] p00_s1, p11_s1, p22_s1, p33_s1, p01_s1, p02_s1;
    logic signed [15:0] q_s1 [0:3];
    logic               valid_s1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s1 <= 1'b0;
            for (int i = 0; i < 4; i++) q_s1[i] <= '0;
            p00_s1 <= '0; p11_s1 <= '0; p22_s1 <= '0;
            p33_s1 <= '0; p01_s1 <= '0; p02_s1 <= '0;
        end else begin
            valid_s1 <= valid_in;
            if (valid_in) begin
                for (int i = 0; i < 4; i++) q_s1[i] <= q_in[i];
                p00_s1 <= q_in[0] * q_in[0]; // DSP0
                p11_s1 <= q_in[1] * q_in[1]; // DSP1
                p22_s1 <= q_in[2] * q_in[2]; // DSP2
                p33_s1 <= q_in[3] * q_in[3]; // DSP3
                p01_s1 <= q_in[0] * q_in[1]; // DSP4
                p02_s1 <= q_in[0] * q_in[2]; // DSP5
            end
        end
    end

    // =========================================================================
    // Stage 2: compute remaining 4 products (4 DSP48), carry stage-1 products
    //   p03 = q0·q3, p12 = q1·q2, p13 = q1·q3, p23 = q2·q3
    // =========================================================================
    logic signed [31:0] p00_s2, p11_s2, p22_s2, p33_s2, p01_s2, p02_s2;
    logic signed [31:0] p03_s2, p12_s2, p13_s2, p23_s2;
    logic signed [15:0] q_s2 [0:3];
    logic               valid_s2;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_s2 <= 1'b0;
            for (int i = 0; i < 4; i++) q_s2[i] <= '0;
            p00_s2 <= '0; p11_s2 <= '0; p22_s2 <= '0; p33_s2 <= '0;
            p01_s2 <= '0; p02_s2 <= '0;
            p03_s2 <= '0; p12_s2 <= '0; p13_s2 <= '0; p23_s2 <= '0;
        end else begin
            valid_s2 <= valid_s1;
            if (valid_s1) begin
                for (int i = 0; i < 4; i++) q_s2[i] <= q_s1[i];
                // Carry stage-1 products
                p00_s2 <= p00_s1; p11_s2 <= p11_s1;
                p22_s2 <= p22_s1; p33_s2 <= p33_s1;
                p01_s2 <= p01_s1; p02_s2 <= p02_s1;
                // New products (4 DSP48)
                p03_s2 <= q_s1[0] * q_s1[3]; // DSP0
                p12_s2 <= q_s1[1] * q_s1[2]; // DSP1
                p13_s2 <= q_s1[1] * q_s1[3]; // DSP2
                p23_s2 <= q_s1[2] * q_s1[3]; // DSP3
            end
        end
    end

    // =========================================================================
    // Helper: saturate 33-bit sum >> shift_bits to Q15
    // =========================================================================
    function automatic logic signed [15:0] sat_shift14;
        input logic signed [32:0] x;
        logic signed [18:0] s;
        s = x[32:14]; // arithmetic >> 14
        if (s > 19'sh7FFF)       sat_shift14 = 16'sh7FFF;
        else if (s < -19'sh8000) sat_shift14 = 16'sh8000;
        else                     sat_shift14 = s[15:0];
    endfunction

    function automatic logic signed [15:0] sat_shift15;
        input logic signed [32:0] x;
        logic signed [17:0] s;
        s = x[32:15]; // arithmetic >> 15
        if (s > 18'sh7FFF)       sat_shift15 = 16'sh7FFF;
        else if (s < -18'sh8000) sat_shift15 = 16'sh8000;
        else                     sat_shift15 = s[15:0];
    endfunction

    // Saturating 2×q: 2*q_in (stored as Q15, H elements ±2q → may saturate to ±32767)
    function automatic logic signed [15:0] sat2;
        input logic signed [15:0] x;
        logic signed [16:0] d;
        d = {x[15], x} + {x[15], x};
        if (d > 17'sh7FFF)       sat2 = 16'sh7FFF;
        else if (d < -17'sh8000) sat2 = 16'sh8000;
        else                     sat2 = d[15:0];
    endfunction

    // =========================================================================
    // Stage 3: combine products into h_meas and H_mat
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            valid_out <= 1'b0;
            for (int i = 0; i < 6; i++) h_meas[i] <= '0;
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 7; j++) H_mat[i][j] <= '0;
        end else begin
            valid_out <= valid_s2;
            if (valid_s2) begin
                // --- h_accel ---
                // h_a[0] = 2*(q1·q3 + q0·q2): sum in Q30, need Q15 → >> 14
                h_meas[0] <= sat_shift14({p13_s2[31], p13_s2} + {p02_s2[31], p02_s2});
                // h_a[1] = 2*(q2·q3 - q0·q1)
                h_meas[1] <= sat_shift14({p23_s2[31], p23_s2} - {p01_s2[31], p01_s2});
                // h_a[2] = q0²-q1²-q2²+q3² (Q30) >> 15
                h_meas[2] <= sat_shift15(
                    {p00_s2[31], p00_s2} - {p11_s2[31], p11_s2}
                  - {p22_s2[31], p22_s2} + {p33_s2[31], p33_s2});

                // --- h_mag ---
                // h_m[0] = q0²+q1²-q2²-q3²
                h_meas[3] <= sat_shift15(
                    {p00_s2[31], p00_s2} + {p11_s2[31], p11_s2}
                  - {p22_s2[31], p22_s2} - {p33_s2[31], p33_s2});
                // h_m[1] = 2*(q1·q2 + q0·q3)
                h_meas[4] <= sat_shift14({p12_s2[31], p12_s2} + {p03_s2[31], p03_s2});
                // h_m[2] = 2*(q1·q3 - q0·q2)
                h_meas[5] <= sat_shift14({p13_s2[31], p13_s2} - {p02_s2[31], p02_s2});

                // --- H Jacobian (6 rows × 7 cols; cols 4..6 = 0 for bias) ---
                // Row 0: ∂h_a[0]/∂q = [2q2, 2q3, 2q0, 2q1, 0, 0, 0]
                H_mat[0][0] <= sat2(q_s2[2]); H_mat[0][1] <= sat2(q_s2[3]);
                H_mat[0][2] <= sat2(q_s2[0]); H_mat[0][3] <= sat2(q_s2[1]);
                H_mat[0][4] <= '0; H_mat[0][5] <= '0; H_mat[0][6] <= '0;

                // Row 1: ∂h_a[1]/∂q = [-2q1, -2q0, 2q3, 2q2, 0, 0, 0]
                H_mat[1][0] <= -sat2(q_s2[1]); H_mat[1][1] <= -sat2(q_s2[0]);
                H_mat[1][2] <= sat2(q_s2[3]);  H_mat[1][3] <= sat2(q_s2[2]);
                H_mat[1][4] <= '0; H_mat[1][5] <= '0; H_mat[1][6] <= '0;

                // Row 2: ∂h_a[2]/∂q = [2q0, -2q1, -2q2, 2q3, 0, 0, 0]
                H_mat[2][0] <= sat2(q_s2[0]);  H_mat[2][1] <= -sat2(q_s2[1]);
                H_mat[2][2] <= -sat2(q_s2[2]); H_mat[2][3] <= sat2(q_s2[3]);
                H_mat[2][4] <= '0; H_mat[2][5] <= '0; H_mat[2][6] <= '0;

                // Row 3: ∂h_m[0]/∂q = [2q0, 2q1, -2q2, -2q3, 0, 0, 0]
                H_mat[3][0] <= sat2(q_s2[0]);  H_mat[3][1] <= sat2(q_s2[1]);
                H_mat[3][2] <= -sat2(q_s2[2]); H_mat[3][3] <= -sat2(q_s2[3]);
                H_mat[3][4] <= '0; H_mat[3][5] <= '0; H_mat[3][6] <= '0;

                // Row 4: ∂h_m[1]/∂q = [2q3, 2q2, 2q1, 2q0, 0, 0, 0]
                H_mat[4][0] <= sat2(q_s2[3]); H_mat[4][1] <= sat2(q_s2[2]);
                H_mat[4][2] <= sat2(q_s2[1]); H_mat[4][3] <= sat2(q_s2[0]);
                H_mat[4][4] <= '0; H_mat[4][5] <= '0; H_mat[4][6] <= '0;

                // Row 5: ∂h_m[2]/∂q = [-2q2, 2q3, -2q0, 2q1, 0, 0, 0]
                H_mat[5][0] <= -sat2(q_s2[2]); H_mat[5][1] <= sat2(q_s2[3]);
                H_mat[5][2] <= -sat2(q_s2[0]); H_mat[5][3] <= sat2(q_s2[1]);
                H_mat[5][4] <= '0; H_mat[5][5] <= '0; H_mat[5][6] <= '0;
            end
        end
    end

endmodule