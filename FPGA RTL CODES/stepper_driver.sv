`timescale 1ns/1ps

module stepper_driver #(
    parameter CLK_HZ = 50_000_000
)(
    input  logic        clk,
    input  logic        rst_n,

    input  logic        enable,
    input  logic [15:0] target_position,
    input  logic [15:0] max_velocity,     // steps/sec
    input  logic [15:0] acceleration,     // steps/sec^2

    output logic [15:0] current_position,
    output logic        step,
    output logic        dir,
    output logic        moving
);

    // ------------------------------------------------------------------------
    // INTERNAL PARAMETERS
    // ------------------------------------------------------------------------
    localparam ACC_W = 32;   // velocity accumulator width

    // ------------------------------------------------------------------------
    // INTERNAL SIGNALS
    // ------------------------------------------------------------------------
    logic signed [16:0] pos_error;
    logic signed [16:0] distance_abs;

    logic [31:0] velocity;          // steps/sec scaled
    logic [31:0] velocity_next;

    logic [31:0] accel_step;        // scaled accel per cycle
    logic [31:0] vel_accum;         // phase accumulator

    logic step_pulse;

    // ------------------------------------------------------------------------
    // ERROR & DIRECTION
    // ------------------------------------------------------------------------
    always_comb begin
        pos_error   = $signed({1'b0,target_position}) - $signed({1'b0,current_position});
        dir         = (pos_error >= 0);
        distance_abs = (pos_error >= 0) ? pos_error : -pos_error;
    end

    // ------------------------------------------------------------------------
    // ACCELERATION SCALING
    // accel_step = acceleration / CLK_HZ
    // scaled using shift approximation (fixed-point)
    // ------------------------------------------------------------------------
    always_comb begin
        accel_step = (acceleration << 16) / CLK_HZ; // Q16 scaling
    end

    // ------------------------------------------------------------------------
    // DECELERATION CONDITION
    // v^2 >= 2*a*d
    // ------------------------------------------------------------------------
    logic decel;

    always_comb begin
        decel = (velocity * velocity) >= (distance_abs * acceleration * 2);
    end

    // ------------------------------------------------------------------------
    // VELOCITY PROFILE (TRAPEZOIDAL)
    // ------------------------------------------------------------------------
    always_comb begin
        velocity_next = velocity;

        if (!enable || distance_abs == 0) begin
            velocity_next = 0;
        end
        else begin
            if (decel) begin
                // deceleration
                if (velocity > accel_step)
                    velocity_next = velocity - accel_step;
                else
                    velocity_next = 0;
            end
            else begin
                // acceleration
                if (velocity < (max_velocity << 16))
                    velocity_next = velocity + accel_step;
                else
                    velocity_next = (max_velocity << 16);
            end
        end
    end

    // ------------------------------------------------------------------------
    // STEP GENERATION (DDS)
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vel_accum <= 0;
            step_pulse <= 0;
        end else begin
            vel_accum <= vel_accum + velocity;

            // MSB overflow generates step
            step_pulse <= vel_accum[31];
            vel_accum[31] <= 1'b0; // clear MSB after pulse
        end
    end

    // ------------------------------------------------------------------------
    // POSITION UPDATE
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_position <= 0;
        end else if (step_pulse && enable) begin
            if (dir)
                current_position <= current_position + 1;
            else
                current_position <= current_position - 1;
        end
    end

    // ------------------------------------------------------------------------
    // VELOCITY REGISTER
    // ------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            velocity <= 0;
        else
            velocity <= velocity_next;
    end

    // ------------------------------------------------------------------------
    // OUTPUTS
    // ------------------------------------------------------------------------
    assign step   = step_pulse;
    assign moving = (distance_abs != 0) || (velocity != 0);

endmodule
