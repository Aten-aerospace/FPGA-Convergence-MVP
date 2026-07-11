// =============================================================================
// LOW-LUT EKF GPS UPDATE
// NO DIVISION OPERATORS
// SHIFT-BASED KALMAN GAIN
// =============================================================================

`timescale 1ns/1ps

module ekf_gps_update #(
    parameter int STATES  = 9,
    parameter int STATE_W = 32,
    parameter int P_W     = 32
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_10hz,

    input  logic signed [STATE_W-1:0] gps_lat,
    input  logic signed [STATE_W-1:0] gps_lon,
    input  logic signed [STATE_W-1:0] gps_alt,
    input  logic signed [STATE_W-1:0] gps_vn,
    input  logic signed [STATE_W-1:0] gps_ve,

    input  logic [P_W-1:0] R_lat,
    input  logic [P_W-1:0] R_lon,
    input  logic [P_W-1:0] R_alt,
    input  logic [P_W-1:0] R_vn,
    input  logic [P_W-1:0] R_ve,

    input  logic signed [STATE_W-1:0] state_in [0:STATES-1],
    input  logic signed [P_W-1:0]     p_diag_in [0:STATES-1],

    output logic signed [STATE_W-1:0] state_out [0:STATES-1],
    output logic signed [P_W-1:0]     p_diag_out [0:STATES-1],
    output logic valid
);

localparam IDX_VN  = 3;
localparam IDX_VE  = 4;
localparam IDX_LAT = 6;
localparam IDX_LON = 7;
localparam IDX_ALT = 8;

// FIXED SHIFT GAIN ≈ 1/8
localparam GAIN_SHIFT = 3;

typedef enum logic [2:0] {
    IDLE,
    VN,
    VE,
    LAT,
    LON,
    ALT,
    DONE
} st_t;

st_t st;

logic signed [STATE_W-1:0] x [0:STATES-1];
logic signed [P_W-1:0]     p [0:STATES-1];

logic signed [STATE_W-1:0] innov;

integer i;

task automatic update_state(
    input int idx,
    input logic signed [STATE_W-1:0] meas
);
begin
    innov = meas - x[idx];

    // x = x + innov/8
    x[idx] = x[idx] + (innov >>> GAIN_SHIFT);

    // P = P - P/8
    p[idx] = p[idx] - (p[idx] >>> GAIN_SHIFT);
end
endtask

always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin

        st <= IDLE;
        valid <= 1'b0;

        for(i=0;i<STATES;i=i+1) begin
            x[i] <= 0;
            p[i] <= 32'h00010000;

            state_out[i] <= 0;
            p_diag_out[i] <= 32'h00010000;
        end
    end
    else begin

        valid <= 1'b0;

        case(st)

        IDLE: begin

            if(ce_10hz) begin

                for(i=0;i<STATES;i=i+1) begin
                    x[i] <= state_in[i];
                    p[i] <= p_diag_in[i];
                end

                st <= VN;
            end
        end

        VN: begin
            update_state(IDX_VN,gps_vn);
            st <= VE;
        end

        VE: begin
            update_state(IDX_VE,gps_ve);
            st <= LAT;
        end

        LAT: begin
            update_state(IDX_LAT,gps_lat);
            st <= LON;
        end

        LON: begin
            update_state(IDX_LON,gps_lon);
            st <= ALT;
        end

        ALT: begin
            update_state(IDX_ALT,gps_alt);
            st <= DONE;
        end

        DONE: begin

            for(i=0;i<STATES;i=i+1) begin
                state_out[i] <= x[i];
                p_diag_out[i] <= p[i];
            end

            valid <= 1'b1;
            st <= IDLE;
        end

        endcase
    end
end

endmodule
