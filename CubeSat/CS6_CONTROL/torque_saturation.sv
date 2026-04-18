// =============================================================================
// Module: torque_saturation
// Subsystem: CS6 - Attitude Control
// Description: Saturates torque commands to physical actuator limits.
//              Each axis is independently clamped to [-SAT_LIMIT, +SAT_LIMIT].
//              A per-axis sat_flag bit asserts when the corresponding channel
//              was clamped; valid_out pulses one cycle after valid_in.
//
//              When ENABLE_SAT_COUNT=1 (default), sat_count[15:0] increments
//              once per valid_in cycle in which any axis is saturated (CS-ADCS-007).
//              The counter saturates at 0xFFFF and does not wrap.
//              Set ENABLE_SAT_COUNT=0 to disable the counter for area-critical
//              instances; sat_count will read as 0x0000 in that case.
//
//   Algorithm (per axis i ∈ {0,1,2}):
//     if torque_in[i] > +SAT_LIMIT : torque_out[i] = +SAT_LIMIT, sat_flag[i]=1
//     if torque_in[i] < -SAT_LIMIT : torque_out[i] = -SAT_LIMIT, sat_flag[i]=1
//     else                         : torque_out[i] = torque_in[i], sat_flag[i]=0
//
//   Pipeline: 1 registered stage (latency = 1 clock cycle).
//
// Provenance: cubesat_requirements.md (CS-ADCS-007)
// =============================================================================
`timescale 1ns/1ps

module torque_saturation #(
    parameter signed [15:0] SAT_LIMIT        = 16'sh3FFF, // 16383
    parameter bit           ENABLE_SAT_COUNT = 1'b1        // 1=enable saturation event counter
)(
    input  logic        clk,
    input  logic        rst_n,

    // Input torque commands (signed Q15)
    input  logic signed [15:0] torque_in  [0:2],
    input  logic               valid_in,
    input  logic               sat_count_clear, // pulse high to clear sat_count

    // Saturated output (signed Q15)
    output logic signed [15:0] torque_out [0:2],
    output logic        [2:0]  sat_flag,   // per-axis saturation flags
    output logic               valid_out,
    output logic        [15:0] sat_count   // CS-ADCS-007: saturation event counter (saturates at 0xFFFF)
);

    // Combinatorial any-axis saturation detection on raw inputs
    logic any_sat_in;
    always_comb begin
        any_sat_in = ($signed(torque_in[0]) >  $signed(SAT_LIMIT))  ||
                     ($signed(torque_in[0]) <  $signed(-SAT_LIMIT)) ||
                     ($signed(torque_in[1]) >  $signed(SAT_LIMIT))  ||
                     ($signed(torque_in[1]) <  $signed(-SAT_LIMIT)) ||
                     ($signed(torque_in[2]) >  $signed(SAT_LIMIT))  ||
                     ($signed(torque_in[2]) <  $signed(-SAT_LIMIT));
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) torque_out[i] <= '0;
            sat_flag  <= 3'b000;
            valid_out <= 1'b0;
            sat_count <= 16'h0000;
        end else begin
            valid_out <= valid_in;
            if (valid_in) begin
                for (int i = 0; i < 3; i++) begin
                    if ($signed(torque_in[i]) > $signed(SAT_LIMIT)) begin
                        torque_out[i] <= SAT_LIMIT;
                        sat_flag[i]   <= 1'b1;
                    end else if ($signed(torque_in[i]) < $signed(-SAT_LIMIT)) begin
                        torque_out[i] <= -SAT_LIMIT;
                        sat_flag[i]   <= 1'b1;
                    end else begin
                        torque_out[i] <= torque_in[i];
                        sat_flag[i]   <= 1'b0;
                    end
                end
                // Increment sat_count when any axis is clamped (ENABLE_SAT_COUNT=1).
                // any_sat_in is evaluated here, gated by valid_in, so only stable
                // input values are used for counting.
                if (ENABLE_SAT_COUNT && any_sat_in)
                    sat_count <= (sat_count == 16'hFFFF) ? 16'hFFFF : sat_count + 16'h1;
            end
            // sat_count_clear has priority - clear wins even if valid_in fires same cycle.
            if (sat_count_clear)
                sat_count <= 16'h0000;
        end
    end

endmodule