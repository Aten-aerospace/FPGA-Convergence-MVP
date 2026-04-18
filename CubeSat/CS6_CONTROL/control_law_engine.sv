// =============================================================================
// Module: control_law_engine
// Subsystem: CS6 - Attitude Control
// Description: Control law top-level. Computes quaternion attitude error using
//              proper quaternion product, then dispatches to pd_law to generate
//              actuator torque commands.
//
//   Quaternion error:  q_error = q_ref ⊗ conj(q_cur)
//
//   conj(q) negates the vector part: conj([w,x,y,z]) = [w,-x,-y,-z]
//
//   Letting rw=q_ref[0], rx=q_ref[1], ry=q_ref[2], rz=q_ref[3]
//   and     cw=q_cur[0], cx=q_cur[1], cy=q_cur[2], cz=q_cur[3]:
//
//     q_e_x = -rw·cx + rx·cw - ry·cz + rz·cy   (vector part x)
//     q_e_y = -rw·cy + rx·cz + ry·cw - rz·cx   (vector part y)
//     q_e_z = -rw·cz - rx·cy + ry·cx + rz·cw   (vector part z)
//
//   Each term is a Q15×Q15 = Q30 product; four Q30 terms are summed in a 34-bit
//   accumulator (Q30 + 2 guard bits), then scaled back to Q15 (bits[30:15]).
//   q_err_sat[i] asserts when the Q30 sum overflows the Q15 representable range
//   (|value| ≥ 1.0), which can occur if input quaternions are not normalised.
//
//   Mapping to 12 registered products (Stage 1):
//     p[0]=rw·cx  p[1]=rx·cw  p[2]=ry·cz  p[3]=rz·cy   → q_e_x
//     p[4]=rw·cy  p[5]=rx·cz  p[6]=ry·cw  p[7]=rz·cx   → q_e_y
//     p[8]=rw·cz  p[9]=rx·cy  p[10]=ry·cx p[11]=rz·cw  → q_e_z
//
//   Data flow (4 registered pipeline stages total):
//     Stage 1 : 12 Q15×Q15 products registered
//     Stage 2 : sum products (34-bit Q30), scale Q30→Q15, clamp, q_err_sat
//     Stage 3-4 : pd_law computes torque (2-cycle pipeline)
//
// Provenance: cubesat_requirements.md (CS-ADCS-007)
// =============================================================================
`timescale 1ns/1ps

module control_law_engine (
    input  logic        clk,
    input  logic        rst_n,

    // 1 kHz clock enable
    input  logic        ce_1khz,

    // Reference quaternion [w,x,y,z] (signed Q15)
    input  logic signed [15:0] q_ref      [0:3],

    // Current attitude quaternion [w,x,y,z] (signed Q15) from EKF
    input  logic signed [15:0] q_cur      [0:3],

    // Angular rate [ωx,ωy,ωz] (signed Q15, rad/s)
    input  logic signed [15:0] omega      [0:2],
    input  logic               meas_valid,

    // Runtime-programmable gains (Q15); default 0.1 and 0.05 when tied to constants
    input  logic signed [15:0] Kp,
    input  logic signed [15:0] Kd,

    // Torque command output (signed Q15)
    output logic signed [15:0] torque_cmd [0:2],
    output logic               sat_flag,
    output logic               cmd_valid,

    // Per-axis quaternion error saturation flags (asserts when |q_err| >= 1.0)
    output logic        [2:0]  q_err_sat
);

    // =========================================================================
    // Stage 1: register 12 Q30 products and propagate omega
    // =========================================================================
    logic signed [31:0] p_s1 [0:11];    // Q30 products (Q15 × Q15)
    logic signed [15:0] omega_s1 [0:2]; // registered omega for Stage 2
    logic               valid_s1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 12; i++) p_s1[i]    <= '0;
            for (int i = 0; i < 3;  i++) omega_s1[i] <= '0;
            valid_s1 <= 1'b0;
        end else begin
            valid_s1 <= ce_1khz && meas_valid;
            if (ce_1khz && meas_valid) begin
                // Products for q_e_x = -p[0] + p[1] - p[2] + p[3]
                p_s1[0]  <= q_ref[0] * q_cur[1]; // rw·cx
                p_s1[1]  <= q_ref[1] * q_cur[0]; // rx·cw
                p_s1[2]  <= q_ref[2] * q_cur[3]; // ry·cz
                p_s1[3]  <= q_ref[3] * q_cur[2]; // rz·cy
                // Products for q_e_y = -p[4] + p[5] + p[6] - p[7]
                p_s1[4]  <= q_ref[0] * q_cur[2]; // rw·cy
                p_s1[5]  <= q_ref[1] * q_cur[3]; // rx·cz
                p_s1[6]  <= q_ref[2] * q_cur[0]; // ry·cw
                p_s1[7]  <= q_ref[3] * q_cur[1]; // rz·cx
                // Products for q_e_z = -p[8] - p[9] + p[10] + p[11]
                p_s1[8]  <= q_ref[0] * q_cur[3]; // rw·cz
                p_s1[9]  <= q_ref[1] * q_cur[2]; // rx·cy
                p_s1[10] <= q_ref[2] * q_cur[1]; // ry·cx
                p_s1[11] <= q_ref[3] * q_cur[0]; // rz·cw
                for (int i = 0; i < 3; i++) omega_s1[i] <= omega[i];
            end
        end
    end

    // =========================================================================
    // Stage 2 combinatorial: sign-extend products to 34 bits and sum
    //
    //   34-bit signed accumulator holds a Q30 value with 2 guard bits.
    //   No overflow expected for normalised unit quaternions; q_err_sat flags
    //   corner cases where |q_e| >= 1.0 (bits[33:30] != 0000 or 1111).
    // =========================================================================
    logic signed [33:0] p_ext [0:11]; // sign-extended to 34 bits
    logic signed [33:0] q_sum [0:2];  // 34-bit Q30 sums

    always_comb begin
        for (int i = 0; i < 12; i++)
            p_ext[i] = {{2{p_s1[i][31]}}, p_s1[i]}; // sign-extend 32→34 bits

        // q_e_x = -rw·cx + rx·cw - ry·cz + rz·cy
        q_sum[0] = -p_ext[0] + p_ext[1] - p_ext[2] + p_ext[3];
        // q_e_y = -rw·cy + rx·cz + ry·cw - rz·cx
        q_sum[1] = -p_ext[4] + p_ext[5] + p_ext[6] - p_ext[7];
        // q_e_z = -rw·cz - rx·cy + ry·cx + rz·cw
        q_sum[2] = -p_ext[8] - p_ext[9] + p_ext[10] + p_ext[11];
    end

    // =========================================================================
    // Stage 2 registered: scale Q30→Q15, clamp on overflow, assert q_err_sat
    //
    //   Q30→Q15 scale: take bits[30:15] of the 34-bit Q30 sum.
    //   Overflow check: bits[33:30] must all be 0000 (positive) or 1111
    //   (negative) for the value to fit in Q15 [-1, +1).
    //   Positive overflow → clamp to +0x7FFF; negative → clamp to -0x8000.
    // =========================================================================
    logic signed [15:0] q_err_r   [0:2]; // Stage 2 Q15 quaternion error
    logic signed [15:0] omega_s2  [0:2]; // Stage 2 omega pass-through
    logic        [2:0]  q_err_sat_r;     // per-axis overflow flags
    logic               ctrl_valid_r;    // valid strobe to pd_law
    // sign_bits[i] = q_sum[i][33:30]; used for overflow detection in always_ff
    logic        [3:0]  sign_bits [0:2];
    always_comb begin
        for (int i = 0; i < 3; i++)
            sign_bits[i] = q_sum[i][33:30];
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) begin
                q_err_r[i]  <= '0;
                omega_s2[i] <= '0;
            end
            q_err_sat_r  <= 3'b000;
            ctrl_valid_r <= 1'b0;
        end else begin
            ctrl_valid_r <= valid_s1;
            if (valid_s1) begin
                for (int i = 0; i < 3; i++) begin
                    omega_s2[i]    <= omega_s1[i];
                    // Detect overflow: bits[33:30] must be all-0 or all-1
                    q_err_sat_r[i] <= (sign_bits[i] != 4'b0000) &&
                                      (sign_bits[i] != 4'b1111);
                    // Scale Q30→Q15 and clamp
                    if (sign_bits[i] == 4'b0000 || sign_bits[i] == 4'b1111)
                        q_err_r[i] <= q_sum[i][30:15]; // in range: take Q15 slice
                    else if (!q_sum[i][33])
                        q_err_r[i] <= 16'sh7FFF;       // positive overflow
                    else
                        q_err_r[i] <= 16'sh8000;       // negative overflow
                end
            end
        end
    end

    assign q_err_sat = q_err_sat_r;

    // =========================================================================
    // pd_law instance (Stage 3 + Stage 4)
    // =========================================================================
    pd_law u_pd_law (
        .clk        (clk),
        .rst_n      (rst_n),
        .ce_1khz    (ce_1khz),
        .q_err      (q_err_r),
        .omega      (omega_s2),
        .ctrl_valid (ctrl_valid_r),
        .Kp         (Kp),
        .Kd         (Kd),
        .torque_cmd (torque_cmd),
        .sat_flag   (sat_flag),
        .cmd_valid  (cmd_valid)
    );

endmodule