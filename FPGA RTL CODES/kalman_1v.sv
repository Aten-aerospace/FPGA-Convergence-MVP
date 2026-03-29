`timescale 1ns/1ps

module kalman_1v #(
  parameter POS_W  = 32,   // Q15.16
  parameter VEL_W  = 32,   // Q8.24
  parameter MEAS_W = 32,   // Q15.16
  parameter VID    = 0
)(
  input  logic clk, rst_n,

  input  logic predict_en,     // 100 Hz
  input  logic update_en,      // 10 Hz

  input  logic signed [POS_W-1:0] meas_px, meas_py, meas_pz,
  input  logic signed [VEL_W-1:0] accel_x, accel_y, accel_z,

  output logic signed [POS_W-1:0] est_px, est_py, est_pz,
  output logic signed [VEL_W-1:0] est_vx, est_vy, est_vz,
  output logic data_valid
);

  // ============================================================================
  // CONSTANTS (Fixed-point)
  // ============================================================================

  // Ts = 0.01 → Q8.24
  localparam signed [31:0] TS_FP = 32'sd167772; // ~0.01 * 2^24

  // Kalman gains (from spec)
  localparam signed [31:0] KP = 32'sd858993459; // ~0.8 (Q1.30)
  localparam signed [31:0] KV = 32'sd838861;    // ~0.05 (Q8.24)

  // ============================================================================
  // STATE REGISTERS
  // ============================================================================

  logic signed [POS_W-1:0] px, py, pz;
  logic signed [VEL_W-1:0] vx, vy, vz;

  logic signed [POS_W-1:0] px_pred, py_pred, pz_pred;
  logic signed [VEL_W-1:0] vx_pred, vy_pred, vz_pred;

  logic signed [POS_W-1:0] innov_x, innov_y, innov_z;

  // ============================================================================
  // FSM
  // ============================================================================

  typedef enum logic [1:0] {
    IDLE,
    PREDICT,
    UPDATE,
    OUTPUT
  } state_t;

  state_t state, next_state;

  // ============================================================================
  // FIXED-POINT MULTIPLY FUNCTIONS
  // ============================================================================

  function automatic signed [31:0] mult_q8_24;
    input signed [31:0] a, b;
    logic signed [63:0] tmp;
    begin
      tmp = a * b;
      mult_q8_24 = tmp >>> 24;
    end
  endfunction

  function automatic signed [31:0] mult_kp;
    input signed [31:0] kp;
    input signed [31:0] innov;
    logic signed [63:0] tmp;
    begin
      tmp = kp * innov;
      mult_kp = tmp >>> 30;
    end
  endfunction

  function automatic signed [31:0] mult_kv;
    input signed [31:0] kv;
    input signed [31:0] innov;
    logic signed [63:0] tmp;
    begin
      tmp = kv * innov;
      mult_kv = tmp >>> 24;
    end
  endfunction

  // ============================================================================
  // FSM SEQUENTIAL
  // ============================================================================

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= IDLE;
    else
      state <= next_state;
  end

  // ============================================================================
  // FSM NEXT STATE
  // ============================================================================

  always_comb begin
    next_state = state;
    case (state)
      IDLE:    if (predict_en) next_state = PREDICT;
      PREDICT: next_state = update_en ? UPDATE : OUTPUT;
      UPDATE:  next_state = OUTPUT;
      OUTPUT:  next_state = IDLE;
    endcase
  end

  // ============================================================================
  // CORE LOGIC
  // ============================================================================

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      px <= '0; py <= '0; pz <= '0;
      vx <= '0; vy <= '0; vz <= '0;
      data_valid <= 1'b0;
    end else begin

      data_valid <= 1'b0;

      case (state)

        // ----------------------------------------------------------------------
        // PREDICT: x = F*x + B*u
        // ----------------------------------------------------------------------
        PREDICT: begin

          // Position: p + v*Ts
          px_pred <= px + mult_q8_24(vx, TS_FP);
          py_pred <= py + mult_q8_24(vy, TS_FP);
          pz_pred <= pz + mult_q8_24(vz, TS_FP);

          // Velocity: v + a*Ts
          vx_pred <= vx + mult_q8_24(accel_x, TS_FP);
          vy_pred <= vy + mult_q8_24(accel_y, TS_FP);
          vz_pred <= vz + mult_q8_24(accel_z, TS_FP);
        end

        // ----------------------------------------------------------------------
        // UPDATE: x = x + K*(z - Hx)
        // ----------------------------------------------------------------------
        UPDATE: begin

          // Innovation
          innov_x <= meas_px - px_pred;
          innov_y <= meas_py - py_pred;
          innov_z <= meas_pz - pz_pred;

          // Position update
          px <= px_pred + mult_kp(KP, innov_x);
          py <= py_pred + mult_kp(KP, innov_y);
          pz <= pz_pred + mult_kp(KP, innov_z);

          // Velocity update
          vx <= vx_pred + mult_kv(KV, innov_x);
          vy <= vy_pred + mult_kv(KV, innov_y);
          vz <= vz_pred + mult_kv(KV, innov_z);
        end

        // ----------------------------------------------------------------------
        // OUTPUT
        // ----------------------------------------------------------------------
        OUTPUT: begin
          est_px <= px;
          est_py <= py;
          est_pz <= pz;

          est_vx <= vx;
          est_vy <= vy;
          est_vz <= vz;

          data_valid <= 1'b1;
        end

        default: ;
      endcase
    end
  end

endmodule
