`timescale 1ns / 1ps


module lpf #(
  parameter int DATA_W = 16,   // Q4.12
  parameter int ORDER  = 2     // 1 or 2
)(
  input  logic                     clk,
  input  logic                     rst_n,
  input  logic                     enable,

  input  logic signed [DATA_W-1:0] data_in,
  input  logic [15:0]              cutoff_freq,  // Hz

  output logic signed [DATA_W-1:0] data_out
);

  // ============================================================
  // CONSTANTS
  // ============================================================
  localparam int FRAC = 12;
  localparam int SCALE = (1 << FRAC);

  // Assume Fs = 1000 Hz (Ts = 1 ms)
  localparam int FS = 1000;

  // ============================================================
  // INTERNAL REGISTERS
  // ============================================================
  logic signed [DATA_W-1:0] x1, x2;
  logic signed [DATA_W-1:0] y1, y2;

  // Coefficients (Q4.12)
  logic signed [DATA_W-1:0] a0, a1, a2;
  logic signed [DATA_W-1:0] b1, b2;

  // MAC intermediates
  logic signed [31:0] mac;

  // ============================================================
  // COEFFICIENT COMPUTATION (RUNTIME)
  // ============================================================
  // alpha = fc / Fs (Q4.12)
  logic signed [31:0] alpha_q;

  always_comb begin
    // alpha = cutoff_freq / Fs
    alpha_q = (cutoff_freq << FRAC) / FS;

    if (ORDER == 1) begin
      // First-order Butterworth
      // Simple form: y = a*x + (1-a)*y_prev

      a0 = alpha_q[FRAC +: DATA_W];   // alpha
      a1 = '0;
      a2 = '0;

      b1 = (SCALE - a0);  // (1 - alpha)
      b2 = '0;

    end else begin
      // Second-order Butterworth approximation

      // Precompute terms
      logic signed [31:0] alpha2;
      logic signed [31:0] denom;

      alpha2 = (alpha_q * alpha_q) >>> FRAC;

      // denom = 1 + sqrt(2)*alpha + alpha^2
      // sqrt(2) ≈ 1.414 → Q4.12 = 5793
      denom = SCALE + ((alpha_q * 16'sd5793) >>> FRAC) + alpha2;

      // a0 = alpha^2 / denom
      a0 = ((alpha2 << FRAC) / denom);

      // a1 = 2*a0
      a1 = a0 <<< 1;

      // a2 = a0
      a2 = a0;

      // b1 = (2*(alpha^2 - 1)) / denom
      b1 = (( (alpha2 - SCALE) <<< 1 ) << FRAC) / denom;

      // b2 = (1 - sqrt(2)*alpha + alpha^2) / denom
      b2 = ((SCALE - ((alpha_q * 16'sd5793) >>> FRAC) + alpha2) << FRAC) / denom;
    end
  end

  // ============================================================
  // FILTER COMPUTATION
  // ============================================================
  logic signed [DATA_W-1:0] y_new;
  logic signed [31:0] acc;

  always_comb begin
    acc = 0;

    if (ORDER == 1) begin
      // y[n] = a0*x[n] + b1*y[n-1]
      acc = (a0 * data_in) +
            (b1 * y1);

    end else begin
      // y[n] = a0*x[n] + a1*x[n-1] + a2*x[n-2]
      //      - b1*y[n-1] - b2*y[n-2]

      acc = (a0 * data_in) +
            (a1 * x1) +
            (a2 * x2) -
            (b1 * y1) -
            (b2 * y2);
    end
  end

  // ============================================================
  // SCALING + SATURATION
  // ============================================================
  logic signed [DATA_W-1:0] y_sat;

  always_comb begin
    // Convert Q8.24 -> Q4.12
    logic signed [31:0] shifted;
    shifted = acc >>> FRAC;

    // Saturation
    if (shifted > 16'sh7FFF)
      y_sat = 16'sh7FFF;
    else if (shifted < -16'sh8000)
      y_sat = -16'sh8000;
    else
      y_sat = shifted[DATA_W-1:0];
  end

  // ============================================================
  // STATE UPDATE
  // ============================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x1 <= 0;
      x2 <= 0;
      y1 <= 0;
      y2 <= 0;
      data_out <= 0;
    end else if (enable) begin
      data_out <= y_sat;

      // Shift registers
      x2 <= x1;
      x1 <= data_in;

      y2 <= y1;
      y1 <= y_sat;
    end
  end

endmodule

