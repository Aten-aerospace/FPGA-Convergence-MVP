// =============================================================================
// File        : pid_loop_mux.sv
// Module      : pid_loop_mux
// Description : Time-multiplexed dual-loop PID scheduler for UAV MOD_2.
//               Sequences 8 PID axes across a single pid_controller instance:
//                 Inner (1 kHz):  Roll-rate, Pitch-rate, Yaw-rate
//                 Outer (100 Hz): Roll, Pitch, Altitude, VN, VE
//               Gain and limit sets are fetched from the register file.
// =============================================================================

`timescale 1ns/1ps

module pid_loop_mux #(
    parameter int DATA_W  = 16, // Q4.12
    parameter int INTEG_W = 32, // Q16.16
    parameter int COEFF_W = 16  // Q4.12
)(
    input  logic clk,
    input  logic rst_n,

    // CE strobes from MOD_1
    input  logic ce_1khz,
    input  logic ce_100hz,

    // Setpoints (from outer loop / navigation)
    input  logic signed [DATA_W-1:0] sp_roll_rate,   // rad/s Q4.12
    input  logic signed [DATA_W-1:0] sp_pitch_rate,
    input  logic signed [DATA_W-1:0] sp_yaw_rate,
    input  logic signed [DATA_W-1:0] sp_roll,        // rad Q4.12
    input  logic signed [DATA_W-1:0] sp_pitch,
    input  logic signed [DATA_W-1:0] sp_alt,         // m Q4.12
    input  logic signed [DATA_W-1:0] sp_vn,          // m/s Q4.12
    input  logic signed [DATA_W-1:0] sp_ve,

    // Measured values (from EKF)
    input  logic signed [DATA_W-1:0] meas_roll_rate,
    input  logic signed [DATA_W-1:0] meas_pitch_rate,
    input  logic signed [DATA_W-1:0] meas_yaw_rate,
    input  logic signed [DATA_W-1:0] meas_roll,
    input  logic signed [DATA_W-1:0] meas_pitch,
    input  logic signed [DATA_W-1:0] meas_alt,
    input  logic signed [DATA_W-1:0] meas_vn,
    input  logic signed [DATA_W-1:0] meas_ve,

    // Gain registers (18 total: 3 inner × 3 + 5 outer × 3 = Kp/Ki/Kd per axis)
    // Indexed [axis][0]=Kp [1]=Ki [2]=Kd
    input  logic signed [COEFF_W-1:0] gains [0:7][0:2],

    // Anti-windup clamps per axis
    input  logic signed [INTEG_W-1:0] integ_max [0:7],

    // Output saturation limits
    input  logic signed [DATA_W-1:0]  out_max [0:7],
    input  logic signed [DATA_W-1:0]  out_min [0:7],

    // Clear integrators (e.g., on mode switch)
    input  logic [7:0] clear_integ,

    // Outputs per axis
    output logic signed [DATA_W-1:0] pid_out [0:7],
    output logic [7:0]               pid_valid,

    // Inner-loop outputs (roll/pitch/yaw rate commands → motor mixer)
    output logic signed [DATA_W-1:0] roll_rate_cmd,
    output logic signed [DATA_W-1:0] pitch_rate_cmd,
    output logic signed [DATA_W-1:0] yaw_rate_cmd
);

    // -------------------------------------------------------------------------
    // Axis index encoding
    // Axes 0-2: inner (1 kHz rate PIDs)
    // Axes 3-7: outer (100 Hz attitude/alt/vel PIDs)
    // -------------------------------------------------------------------------
    localparam int AXES = 8;
    localparam int INNER_AXES = 3;

    // -------------------------------------------------------------------------
    // Setpoint / measurement mux arrays
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0] sp_arr   [0:AXES-1];
    logic signed [DATA_W-1:0] meas_arr [0:AXES-1];

    always_comb begin
        sp_arr[0] = sp_roll_rate;   meas_arr[0] = meas_roll_rate;
        sp_arr[1] = sp_pitch_rate;  meas_arr[1] = meas_pitch_rate;
        sp_arr[2] = sp_yaw_rate;    meas_arr[2] = meas_yaw_rate;
        sp_arr[3] = sp_roll;        meas_arr[3] = meas_roll;
        sp_arr[4] = sp_pitch;       meas_arr[4] = meas_pitch;
        sp_arr[5] = sp_alt;         meas_arr[5] = meas_alt;
        sp_arr[6] = sp_vn;          meas_arr[6] = meas_vn;
        sp_arr[7] = sp_ve;          meas_arr[7] = meas_ve;
    end

    // -------------------------------------------------------------------------
    // Scheduling FSM: cycle through enabled axes each CE
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        S_IDLE, S_AX0, S_AX1, S_AX2, S_AX3, S_AX4, S_AX5, S_AX6, S_AX7
    } state_t;

    state_t state;

    logic [2:0] cur_axis;
    logic       pid_en;
    logic       run_inner;  // 1 = 1 kHz axes, 0 = 100 Hz axes

    // Round-robin per CE
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            cur_axis  <= '0;
            pid_en    <= 1'b0;
            run_inner <= 1'b0;
        end else begin
            pid_en <= 1'b0;
            case (state)
                S_IDLE: begin
                    if (ce_1khz) begin
                        cur_axis  <= 3'd0;
                        run_inner <= 1'b1;
                        pid_en    <= 1'b1;
                        state     <= S_AX0;
                    end else if (ce_100hz) begin
                        cur_axis  <= 3'd3;
                        run_inner <= 1'b0;
                        pid_en    <= 1'b1;
                        state     <= S_AX3;
                    end
                end
                // Inner loop sequencing (axes 0-2)
                S_AX0: begin pid_en <= 1'b1; cur_axis <= 3'd1; state <= S_AX1; end
                S_AX1: begin pid_en <= 1'b1; cur_axis <= 3'd2; state <= S_AX2; end
                S_AX2: begin pid_en <= 1'b0; state <= S_IDLE; end
                // Outer loop sequencing (axes 3-7)
                S_AX3: begin pid_en <= 1'b1; cur_axis <= 3'd4; state <= S_AX4; end
                S_AX4: begin pid_en <= 1'b1; cur_axis <= 3'd5; state <= S_AX5; end
                S_AX5: begin pid_en <= 1'b1; cur_axis <= 3'd6; state <= S_AX6; end
                S_AX6: begin pid_en <= 1'b1; cur_axis <= 3'd7; state <= S_AX7; end
                S_AX7: begin pid_en <= 1'b0; state <= S_IDLE; end
                default: state <= S_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // PID controller instance (shared / time-multiplexed)
    // -------------------------------------------------------------------------
    logic signed [DATA_W-1:0]  pid_sp, pid_meas;
    logic signed [COEFF_W-1:0] pid_kp, pid_ki, pid_kd;
    logic signed [DATA_W-1:0]  pid_out_max, pid_out_min;
    logic signed [INTEG_W-1:0] pid_integ_max;
    logic                       pid_clr_integ;
    logic signed [DATA_W-1:0]  pid_result;
    logic                       pid_result_valid;

    always_comb begin
        pid_sp        = sp_arr[cur_axis];
        pid_meas      = meas_arr[cur_axis];
        pid_kp        = gains[cur_axis][0];
        pid_ki        = gains[cur_axis][1];
        pid_kd        = gains[cur_axis][2];
        pid_out_max   = out_max[cur_axis];
        pid_out_min   = out_min[cur_axis];
        pid_integ_max = integ_max[cur_axis];
        pid_clr_integ = clear_integ[cur_axis];
    end

    pid_controller #(
        .DATA_W  (DATA_W),
        .INTEG_W (INTEG_W),
        .COEFF_W (COEFF_W),
        .OUT_W   (DATA_W)
    ) u_pid (
        .clk              (clk),
        .rst_n            (rst_n),
        .enable           (pid_en),
        .clear_integrator (pid_clr_integ),
        .setpoint         (pid_sp),
        .measured         (pid_meas),
        .feedforward      ('0),
        .kp               (pid_kp),
        .ki               (pid_ki),
        .kd               (pid_kd),
        .output_max       (pid_out_max),
        .output_min       (pid_out_min),
        .integral_max     (pid_integ_max),
        .control_output   (pid_result),
        .output_valid     (pid_result_valid),
        .error_out        (),
        .integral_out     (),
        .integral_saturated(),
        .output_saturated ()
    );

    // -------------------------------------------------------------------------
    // Capture results into per-axis output registers
    // -------------------------------------------------------------------------
    logic [2:0] cur_axis_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) cur_axis_d <= '0;
        else        cur_axis_d <= cur_axis;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int a = 0; a < AXES; a++) begin
                pid_out[a]   <= '0;
                pid_valid[a] <= 1'b0;
            end
            roll_rate_cmd  <= '0;
            pitch_rate_cmd <= '0;
            yaw_rate_cmd   <= '0;
        end else begin
            for (int a = 0; a < AXES; a++)
                pid_valid[a] <= 1'b0;

            if (pid_result_valid) begin
                pid_out[cur_axis_d]   <= pid_result;
                pid_valid[cur_axis_d] <= 1'b1;

                case (cur_axis_d)
                    3'd0: roll_rate_cmd  <= pid_result;
                    3'd1: pitch_rate_cmd <= pid_result;
                    3'd2: yaw_rate_cmd   <= pid_result;
                    default: ;
                endcase
            end
        end
    end

endmodule
