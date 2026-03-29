// =============================================================================
// CORDIC Engine (Rotation Mode) - 16-stage Pipeline
// Computes sin/cos from angle input (Q1.15)
// Fully pipelined: throughput = 1 sample/cycle, latency = ITERATIONS+1
// =============================================================================

`timescale 1ns/1ps


module cordic #(
  parameter ITERATIONS = 16
)(
  input  logic         clk,
  input  logic         rst_n,

  input  logic         start,
  input  logic [15:0]  angle,      // Q1.15 (-π to +π)

  output logic [15:0]  cos_out,    // Q1.15
  output logic [15:0]  sin_out,    // Q1.15
  output logic         done
);

  // ===========================================================================
  // Internal format:
  // x,y → Q2.14 (18-bit signed)
  // z   → Q1.15 (16-bit signed)
  // ===========================================================================

  typedef logic signed [17:0] data_t; // x,y
  typedef logic signed [15:0] angle_t;

  // Pipeline registers
  data_t  x   [0:ITERATIONS];
  data_t  y   [0:ITERATIONS];
  angle_t z   [0:ITERATIONS];

  logic   valid [0:ITERATIONS];

  // ===========================================================================
  // Precomputed constants (atan(2^-i)) in Q1.15 (scaled by π)
  // Values aligned with spec table :contentReference[oaicite:1]{index=1}
  // ===========================================================================

  localparam angle_t atan_table [0:15] = '{
    16'sd8192,   // atan(2^0)
    16'sd4836,
    16'sd2555,
    16'sd1297,
    16'sd651,
    16'sd326,
    16'sd163,
    16'sd81,
    16'sd41,
    16'sd20,
    16'sd10,
    16'sd5,
    16'sd3,
    16'sd1,
    16'sd1,
    16'sd0
  };

  // CORDIC gain compensation: Kn = 0.607252935
  // Kn * 2^15 ≈ 19898
  localparam data_t X_INIT = 18'sd19898;
  localparam data_t Y_INIT = 18'sd0;

  // ===========================================================================
  // Stage 0 (input latch)
  // ===========================================================================

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x[0]     <= '0;
      y[0]     <= '0;
      z[0]     <= '0;
      valid[0] <= 1'b0;
    end else begin
      if (start) begin
        x[0]     <= X_INIT;
        y[0]     <= Y_INIT;
        z[0]     <= angle;
        valid[0] <= 1'b1;
      end else begin
        valid[0] <= 1'b0;
      end
    end
  end

  // ===========================================================================
  // CORDIC Iteration Pipeline
  // x_{i+1} = x_i - d_i * (y_i >> i)
  // y_{i+1} = y_i + d_i * (x_i >> i)
  // z_{i+1} = z_i - d_i * atan(2^-i)
  // d_i = sign(z_i)
  // ===========================================================================

  genvar i;
  generate
    for (i = 0; i < ITERATIONS; i++) begin : cordic_stage

      logic signed [17:0] x_shift;
      logic signed [17:0] y_shift;
      logic               d;

      always_comb begin
        x_shift = x[i] >>> i;
        y_shift = y[i] >>> i;
        d       = (z[i] >= 0);
      end

      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
          x[i+1]     <= '0;
          y[i+1]     <= '0;
          z[i+1]     <= '0;
          valid[i+1] <= 1'b0;
        end else begin
          valid[i+1] <= valid[i];

          if (valid[i]) begin
            if (d) begin
              x[i+1] <= x[i] - y_shift;
              y[i+1] <= y[i] + x_shift;
              z[i+1] <= z[i] - atan_table[i];
            end else begin
              x[i+1] <= x[i] + y_shift;
              y[i+1] <= y[i] - x_shift;
              z[i+1] <= z[i] + atan_table[i];
            end
          end
        end
      end

    end
  endgenerate

  // ===========================================================================
  // Output stage
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cos_out <= '0;
      sin_out <= '0;
      done    <= 1'b0;
    end else begin
      cos_out <= x[ITERATIONS][15:0]; // Q1.15
      sin_out <= y[ITERATIONS][15:0];
      done    <= valid[ITERATIONS];
    end
  end

endmodule

