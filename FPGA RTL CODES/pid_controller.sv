`timescale 1ns/1ps

module pid_controller #(
    parameter DATA_W  = 16, // Q4.12
    parameter INTEG_W = 32, // Q16.16
    parameter COEFF_W = 16, // Q4.12
    parameter OUT_W   = 16  // Q4.12
)(
    input  logic clk,
    input  logic rst_n,
    input  logic enable,
    input  logic clear_integrator,

    input  logic signed [DATA_W-1:0]  setpoint,
    input  logic signed [DATA_W-1:0]  measured,
    input  logic signed [DATA_W-1:0]  feedforward,

    input  logic signed [COEFF_W-1:0] kp,
    input  logic signed [COEFF_W-1:0] ki,
    input  logic signed [COEFF_W-1:0] kd,

    input  logic signed [DATA_W-1:0]  output_max,
    input  logic signed [DATA_W-1:0]  output_min,
    input  logic signed [INTEG_W-1:0] integral_max,

    output logic signed [OUT_W-1:0]   control_output,
    output logic                     output_valid,
    output logic signed [DATA_W-1:0] error_out,
    output logic signed [INTEG_W-1:0] integral_out,
    output logic                     integral_saturated,
    output logic                     output_saturated
);

    // -------------------------------------------------------------------------
    // INTERNAL SIGNALS
    // -------------------------------------------------------------------------

    logic signed [DATA_W-1:0] error;
    logic signed [DATA_W-1:0] error_prev;
    logic signed [DATA_W-1:0] derivative;

    logic signed [INTEG_W-1:0] integral;
    logic signed [INTEG_W-1:0] integral_next;

    logic signed [31:0] p_mult;
    logic signed [31:0] i_mult;
    logic signed [31:0] d_mult;

    logic signed [DATA_W-1:0] p_term;
    logic signed [DATA_W-1:0] i_term;
    logic signed [DATA_W-1:0] d_term;

    logic signed [DATA_W+2:0] raw_sum;
    logic signed [DATA_W-1:0] raw_output;

    logic enable_d;

    // -------------------------------------------------------------------------
    // ERROR COMPUTATION
    // -------------------------------------------------------------------------

    always_comb begin
        error = setpoint - measured;
    end

    // -------------------------------------------------------------------------
    // DERIVATIVE
    // -------------------------------------------------------------------------

    always_comb begin
        derivative = error - error_prev;
    end

    // -------------------------------------------------------------------------
    // INTEGRATOR WITH SATURATION
    // -------------------------------------------------------------------------

    always_comb begin
        integral_next = integral;

        if (clear_integrator) begin
            integral_next = '0;
        end else if (enable) begin
            integral_next = integral + {{(INTEG_W-DATA_W){error[DATA_W-1]}}, error};

            if (integral_next > integral_max)
                integral_next = integral_max;
            else if (integral_next < -integral_max)
                integral_next = -integral_max;
        end
    end

    // -------------------------------------------------------------------------
    // MULTIPLIERS (DSP48)
    // -------------------------------------------------------------------------

    (* use_dsp = "yes" *)
    always_comb begin
        p_mult = kp * error;
    end

    (* use_dsp = "yes" *)
    always_comb begin
        i_mult = ki * integral[DATA_W-1:0];
    end

    (* use_dsp = "yes" *)
    always_comb begin
        d_mult = kd * derivative;
    end

    // -------------------------------------------------------------------------
    // SCALE BACK TO Q4.12
    // -------------------------------------------------------------------------

    always_comb begin
        p_term = p_mult >>> 12;
        i_term = i_mult >>> 12;
        d_term = d_mult >>> 12;
    end

    // -------------------------------------------------------------------------
    // SUM
    // -------------------------------------------------------------------------

    always_comb begin
        raw_sum = p_term + i_term + d_term + feedforward;
        raw_output = raw_sum[DATA_W-1:0];
    end

    // -------------------------------------------------------------------------
    // OUTPUT SATURATION
    // -------------------------------------------------------------------------

    always_comb begin
        control_output = raw_output;
        output_saturated = 1'b0;

        if (raw_output > output_max) begin
            control_output = output_max;
            output_saturated = 1'b1;
        end else if (raw_output < output_min) begin
            control_output = output_min;
            output_saturated = 1'b1;
        end
    end

    // -------------------------------------------------------------------------
    // STATUS FLAGS
    // -------------------------------------------------------------------------

    always_comb begin
        integral_saturated = (integral_next == integral_max) ||
                             (integral_next == -integral_max);
    end

    // -------------------------------------------------------------------------
    // REGISTERS
    // -------------------------------------------------------------------------

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            error_prev        <= '0;
            integral          <= '0;
            control_output    <= '0;
            error_out         <= '0;
            integral_out      <= '0;
            output_valid      <= 1'b0;
            enable_d          <= 1'b0;
        end else begin
            enable_d <= enable;

            if (enable) begin
                error_prev   <= error;
                integral     <= integral_next;

                error_out    <= error;
                integral_out <= integral_next;
            end

            // output valid delayed by 1 cycle
            output_valid <= enable_d;
        end
    end

endmodule

