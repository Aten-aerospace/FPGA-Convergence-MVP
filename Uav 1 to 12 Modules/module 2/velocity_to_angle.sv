// =============================================================================
// File        : velocity_to_angle.sv
// Module      : velocity_to_angle
// Description : Converts North/East velocity error (m/s Q4.12) to
//               roll/pitch angle setpoints (rad Q4.12).
//               Uses proportional mapping:
//                 pitch_sp = Kv × vN_error   (pitch forward for North vel)
//                 roll_sp  = Kv × vE_error   (roll right for East vel)
//               Default Kv = 0.1 (Q4.12: 410)
// =============================================================================

`timescale 1ns/1ps

module velocity_to_angle #(
    parameter int DATA_W = 16,          // Q4.12
    parameter int KV_DEFAULT = 16'd410  // 0.1 × 2^12
)(
    input  logic clk,
    input  logic rst_n,
    input  logic enable,

    // Velocity errors (m/s Q4.12)
    input  logic signed [DATA_W-1:0] vn_error,
    input  logic signed [DATA_W-1:0] ve_error,

    // Velocity-to-angle gain (Q4.12, programmable)
    input  logic signed [DATA_W-1:0] kv,

    // Angle setpoints out (rad Q4.12)
    output logic signed [DATA_W-1:0] pitch_sp,  // pitch forward = positive vN
    output logic signed [DATA_W-1:0] roll_sp    // roll right = positive vE
);

    // -------------------------------------------------------------------------
    // Multiply and scale back to Q4.12
    // -------------------------------------------------------------------------
    logic signed [31:0] pitch_raw, roll_raw;

    (* use_dsp = "yes" *)
    always_comb begin
        pitch_raw = (kv * vn_error) >>> 12;
        roll_raw  = (kv * ve_error) >>> 12;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pitch_sp <= '0;
            roll_sp  <= '0;
        end else if (enable) begin
            // Clamp to 16-bit signed
            pitch_sp <= (pitch_raw > 32'sd32767) ? 16'sd32767 :
                        (pitch_raw < -32'sd32768) ? -16'sd32768 :
                        pitch_raw[DATA_W-1:0];
            roll_sp  <= (roll_raw  > 32'sd32767) ? 16'sd32767 :
                        (roll_raw  < -32'sd32768) ? -16'sd32768 :
                        roll_raw[DATA_W-1:0];
        end
    end

endmodule
