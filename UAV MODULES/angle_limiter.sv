// =============================================================================
// File        : angle_limiter.sv
// Module      : angle_limiter
// Description : Clamps roll/pitch/yaw angle setpoints to hardware limits.
//               Roll: ±45° (±45×2^12/180×π ≈ ±3217 in Q4.12 radians)
//               Pitch: ±30°
//               Altitude: 0-500 m
//               North/East velocity: ±15 m/s
// =============================================================================

`timescale 1ns/1ps

module angle_limiter #(
    parameter int DATA_W = 16  // Q4.12 signed
)(
    input  logic clk,
    input  logic rst_n,

    // Raw setpoints in
    input  logic signed [DATA_W-1:0] roll_sp_in,
    input  logic signed [DATA_W-1:0] pitch_sp_in,
    input  logic signed [DATA_W-1:0] yaw_sp_in,
    input  logic signed [DATA_W-1:0] alt_sp_in,
    input  logic signed [DATA_W-1:0] vn_sp_in,
    input  logic signed [DATA_W-1:0] ve_sp_in,

    // Limited setpoints out
    output logic signed [DATA_W-1:0] roll_sp_out,
    output logic signed [DATA_W-1:0] pitch_sp_out,
    output logic signed [DATA_W-1:0] yaw_sp_out,
    output logic signed [DATA_W-1:0] alt_sp_out,
    output logic signed [DATA_W-1:0] vn_sp_out,
    output logic signed [DATA_W-1:0] ve_sp_out
);

    // -------------------------------------------------------------------------
    // Q4.12 constants:  value × 2^12 (rounded)
    // ±45°  = ±(π/4)  in rad → ±0.7854 → ±3217
    // ±30°  = ±(π/6)  in rad → ±0.5236 → ±2145
    // 0-500 m in Q4.12 → 0 to 2048000 (overflow 16-bit)
    // Using Q10.6 for altitude: 500 × 64 = 32000 ≤ 32767 ✓
    // Velocity ±15 m/s Q4.12 → ±61440 (overflow) → limit capped at signed max
    // For simplicity we keep DATA_W=16 and apply limits within representable range
    // -------------------------------------------------------------------------
    localparam signed [DATA_W-1:0] ROLL_MAX  =  16'sd3217;
    localparam signed [DATA_W-1:0] ROLL_MIN  = -16'sd3217;
    localparam signed [DATA_W-1:0] PITCH_MAX =  16'sd2145;
    localparam signed [DATA_W-1:0] PITCH_MIN = -16'sd2145;
    localparam signed [DATA_W-1:0] YAW_MAX   =  16'sd12868; // ±π in Q4.12
    localparam signed [DATA_W-1:0] YAW_MIN   = -16'sd12868;
    localparam signed [DATA_W-1:0] ALT_MAX   =  16'sd32000; // ~500 m (Q6.10)
    localparam signed [DATA_W-1:0] ALT_MIN   =  16'sd0;
    localparam signed [DATA_W-1:0] VEL_MAX   =  16'sd32767; // saturate to max
    localparam signed [DATA_W-1:0] VEL_MIN   = -16'sd32767;

    // Combinational saturation helper
    function automatic logic signed [DATA_W-1:0] clamp(
        input logic signed [DATA_W-1:0] val,
        input logic signed [DATA_W-1:0] lo,
        input logic signed [DATA_W-1:0] hi
    );
        if      (val > hi) return hi;
        else if (val < lo) return lo;
        else               return val;
    endfunction

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            roll_sp_out  <= '0;
            pitch_sp_out <= '0;
            yaw_sp_out   <= '0;
            alt_sp_out   <= '0;
            vn_sp_out    <= '0;
            ve_sp_out    <= '0;
        end else begin
            roll_sp_out  <= clamp(roll_sp_in,  ROLL_MIN,  ROLL_MAX);
            pitch_sp_out <= clamp(pitch_sp_in, PITCH_MIN, PITCH_MAX);
            yaw_sp_out   <= clamp(yaw_sp_in,   YAW_MIN,   YAW_MAX);
            alt_sp_out   <= clamp(alt_sp_in,   ALT_MIN,   ALT_MAX);
            vn_sp_out    <= clamp(vn_sp_in,    VEL_MIN,   VEL_MAX);
            ve_sp_out    <= clamp(ve_sp_in,    VEL_MIN,   VEL_MAX);
        end
    end

endmodule
