// =============================================================================
// Module: pd_law
// Subsystem: CS6 - PD Control Law
// Description: PD attitude control law. Computes reaction-wheel torque commands:
//
//   torque[i] = -Kp × q_err[i] - Kd × omega[i]    for i ∈ {0,1,2}
//
//   The negative sign matches CS-ADCS-007: τ = -Kp×q_err(1:3) - Kd×ω.
//   Kp and Kd are positive Q15 gain values; the restoring sign is applied here.
//
//   q_err  is the vector part of the quaternion error [q1, q2, q3] (Q15).
//   omega  is the angular rate [ωx, ωy, ωz] in Q15 rad/s.
//   Kp, Kd are runtime-programmable Q15 gains (AXI4-Lite in production).
//
//   Anti-windup saturation clamps each torque output to [-SAT, +SAT].
//   sat_flag asserts for any saturated axis.
//
//   Pipeline: 2-clock-cycle latency from ctrl_valid to cmd_valid.
//     Stage 1: compute Kp×q_err and Kd×omega products (Q30)
//     Stage 2: accumulate sum, scale to Q15, saturate, register output
//
//   Q-Format reference:
//     Input  q_err[i], omega[i] : Q15 signed 16-bit  (value = int × 2⁻¹⁵)
//     Input  Kp, Kd             : Q15 signed 16-bit  (value = int × 2⁻¹⁵)
//     Stage 1 products          : Q30 signed 32-bit  (value = int × 2⁻³⁰, negated)
//                                 kp_prod[i] = -q_err[i]_int × Kp_int  (-Q30)
//                                 kd_prod[i] = -omega[i]_int  × Kd_int (-Q30)
//     Stage 2 sum               : Q30 signed 33-bit  (guard bit for add)
//                                 torque_raw[i] = kp_prod[i] + kd_prod[i]
//     Stage 2 output            : Q15 signed 16-bit  (bits[30:15] of Q30 sum)
//                                 torque_cmd[i] = torque_raw[i][30:15]
//     Saturation                : clamp to [-SAT_LIMIT, +SAT_LIMIT] in Q15
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module pd_law #(
    parameter signed [15:0] DEFAULT_KP  = 16'sh0CCD, // 0.1 in Q15  (round(0.1  × 32768) = 3277 = 0x0CCD)
    parameter signed [15:0] DEFAULT_KD  = 16'sh0666, // 0.05 in Q15 (round(0.05 × 32768) = 1638 = 0x0666)
    parameter signed [15:0] SAT_LIMIT   = 16'sh7FFF  // full Q15 range
)(
    input  logic        clk,
    input  logic        rst_n,

    // 1 kHz clock enable from CS12 (control loop rate)
    input  logic        ce_1khz,

    // Quaternion error vector part [q1, q2, q3] from EKF (Q15)
    input  logic signed [15:0] q_err   [0:2],

    // Angular rate [ωx, ωy, ωz] from IMU/EKF (Q15 rad/s)
    input  logic signed [15:0] omega   [0:2],

    // Handshake
    input  logic               ctrl_valid,

    // Runtime-programmable gains (Q15)
    input  logic signed [15:0] Kp,
    input  logic signed [15:0] Kd,

    // Torque command output (Q15) for each axis
    output logic signed [15:0] torque_cmd [0:2],

    // Status
    output logic               sat_flag,   // any axis saturated
    output logic               cmd_valid   // strobe: torque_cmd valid
);

    // =========================================================================
    // Stage 1: register inputs, compute products Q30
    //   Q-format: -Q15 × Q15 → -Q30 (value = int × 2⁻³⁰, negated)
    // =========================================================================
    logic signed [31:0] kp_prod [0:2]; // -Kp × q_err  (-Q30)
    logic signed [31:0] kd_prod [0:2]; // -Kd × omega  (-Q30)
    logic               valid_s1;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) begin
                kp_prod[i] <= '0;
                kd_prod[i] <= '0;
            end
            valid_s1 <= 1'b0;
        end else begin
            valid_s1 <= ce_1khz && ctrl_valid;
            if (ce_1khz && ctrl_valid) begin
                for (int i = 0; i < 3; i++) begin
                    kp_prod[i] <= -(q_err[i] * Kp); // -Q15 × Q15 = -Q30
                    kd_prod[i] <= -(omega[i]  * Kd); // -Q15 × Q15 = -Q30
                end
            end
        end
    end

    // =========================================================================
    // Stage 2: sum, scale Q30 → Q15, saturate
    //   torque_raw = kp_prod + kd_prod  (33-bit Q30, sign-guard bit added)
    //   Q30 → Q15 : take bits[30:15] (arithmetic right-shift by 15)
    //   Saturation: clamp to [-SAT_LIMIT, +SAT_LIMIT] in Q15
    // =========================================================================
    logic signed [32:0] torque_raw [0:2]; // 33-bit: Q30 sum + sign guard
    logic               any_sat;

    always_comb begin
        any_sat = 1'b0;
        for (int i = 0; i < 3; i++) begin
            torque_raw[i] = {kp_prod[i][31], kp_prod[i]} +
                            {kd_prod[i][31], kd_prod[i]};
        end
        for (int i = 0; i < 3; i++) begin
            // Overflow if bits[32:31] are not all sign-equal
            if (torque_raw[i][32:31] != 2'b00 && torque_raw[i][32:31] != 2'b11)
                any_sat = 1'b1;
            // Also saturate if in-range Q15 slice would exceed SAT_LIMIT
            if (torque_raw[i][32:31] == 2'b00 &&
                torque_raw[i][30:15] > SAT_LIMIT)
                any_sat = 1'b1;
            if (torque_raw[i][32:31] == 2'b11 &&
                $signed(torque_raw[i][30:15]) < -SAT_LIMIT)
                any_sat = 1'b1;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) torque_cmd[i] <= '0;
            sat_flag  <= 1'b0;
            cmd_valid <= 1'b0;
        end else begin
            cmd_valid <= 1'b0; // default de-assert
            if (valid_s1) begin
                cmd_valid <= 1'b1;
                sat_flag  <= any_sat;
                for (int i = 0; i < 3; i++) begin
                    // Overflow saturation
                    if (torque_raw[i][32:31] == 2'b01)
                        torque_cmd[i] <= SAT_LIMIT;
                    else if (torque_raw[i][32:31] == 2'b10)
                        torque_cmd[i] <= -SAT_LIMIT;
                    // In-range: scale Q30 → Q15, then clamp to SAT_LIMIT
                    else if ($signed(torque_raw[i][30:15]) > $signed(SAT_LIMIT))
                        torque_cmd[i] <= SAT_LIMIT;
                    else if ($signed(torque_raw[i][30:15]) < $signed(-SAT_LIMIT))
                        torque_cmd[i] <= -SAT_LIMIT;
                    else
                        torque_cmd[i] <= torque_raw[i][30:15];
                end
            end
        end
    end

endmodule