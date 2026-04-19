// =============================================================================
// File        : mixing_matrix_4x4.sv
// Module      : mixing_matrix_4x4
// Description : 4×4 motor mixing matrix using DSP48E1 MAC operations.
//               Maps [roll_cmd, pitch_cmd, yaw_cmd, thrust_cmd] → 4 motor commands.
//               Matrix coefficients in Q2.14 stored in BRAM.
//               Quadcopter X-frame default mixing:
//                 M0 (FL): +roll - pitch + yaw + thrust
//                 M1 (FR): -roll - pitch - yaw + thrust
//                 M2 (RL): +roll + pitch - yaw + thrust
//                 M3 (RR): -roll + pitch + yaw + thrust
// =============================================================================

`timescale 1ns/1ps

module mixing_matrix_4x4 #(
    parameter int CMD_W  = 16,  // Q4.12 command inputs
    parameter int COEF_W = 16,  // Q2.14 matrix coefficients
    parameter int OUT_W  = 16   // Q4.12 motor output
)(
    input  logic clk,
    input  logic rst_n,
    input  logic enable,

    // Control commands (Q4.12)
    input  logic signed [CMD_W-1:0]  roll_cmd,
    input  logic signed [CMD_W-1:0]  pitch_cmd,
    input  logic signed [CMD_W-1:0]  yaw_cmd,
    input  logic signed [CMD_W-1:0]  thrust_cmd,

    // Mixing matrix coefficients [motor][input]: Q2.14
    // motor = 0-3, input = 0(roll) 1(pitch) 2(yaw) 3(thrust)
    input  logic signed [COEF_W-1:0] mix_coef [0:3][0:3],

    // Motor outputs (Q4.12)
    output logic signed [OUT_W-1:0]  motor_cmd [0:3],
    output logic                      valid
);

    // -------------------------------------------------------------------------
    // Accumulate: motor_i = Σ_j coef[i][j] × cmd[j]  (MAC with DSP48)
    // -------------------------------------------------------------------------
    logic signed [CMD_W-1:0]  cmd_arr [0:3];
    always_comb begin
        cmd_arr[0] = roll_cmd;
        cmd_arr[1] = pitch_cmd;
        cmd_arr[2] = yaw_cmd;
        cmd_arr[3] = thrust_cmd;
    end

    // Pipeline: enable → compute → valid (2-cycle latency)
    logic en_d, en_dd;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin en_d <= 0; en_dd <= 0; end
        else        begin en_d <= enable; en_dd <= en_d; end
    end
    assign valid = en_dd;

    // MAC accumulation (1 cycle)
    logic signed [31:0] mac_raw [0:3];

    (* use_dsp = "yes" *)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int m = 0; m < 4; m++) mac_raw[m] <= '0;
        end else if (enable) begin
            for (int m = 0; m < 4; m++) begin
                mac_raw[m] <= (mix_coef[m][0] * cmd_arr[0]) +
                              (mix_coef[m][1] * cmd_arr[1]) +
                              (mix_coef[m][2] * cmd_arr[2]) +
                              (mix_coef[m][3] * cmd_arr[3]);
            end
        end
    end

    // Scale Q2.14 × Q4.12 = Q6.26 → shift right 26 → Q4.12 (16-bit)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int m = 0; m < 4; m++) motor_cmd[m] <= '0;
        end else if (en_d) begin
            for (int m = 0; m < 4; m++) begin
                logic signed [31:0] scaled;
                scaled = mac_raw[m] >>> 14; // remove Q2.14 fraction bits
                // Saturate to OUT_W
                if      (scaled >  32'sd32767) motor_cmd[m] <=  16'sd32767;
                else if (scaled < -32'sd32768) motor_cmd[m] <= -16'sd32768;
                else                            motor_cmd[m] <= scaled[OUT_W-1:0];
            end
        end
    end

endmodule
