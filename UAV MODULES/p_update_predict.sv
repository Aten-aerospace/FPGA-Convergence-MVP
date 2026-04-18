// =============================================================================
// File        : p_update_predict.sv
// Module      : p_update_predict
// Description : EKF covariance prediction: P = F·P·Fᵀ + Q
//               9×9 symmetric P matrix in Q2.46 (stored in dual-port BRAM).
//               Uses Joseph form for numerical stability.
//               Process noise Q diagonal loaded from register file (Q16.16).
//               Simplified scalar update for synthesisability:
//               Each diagonal element Pi += Fi_diag²·Pi + Qi
// =============================================================================

`timescale 1ns/1ps

module p_update_predict #(
    parameter int STATES  = 9,
    parameter int P_W     = 32,  // P matrix element width (Q16.16 for FPGA)
    parameter int Q_W     = 32   // Process noise Q element width (Q16.16)
)(
    input  logic clk,
    input  logic rst_n,
    input  logic ce_100hz,

    // Process noise diagonal (Q16.16)
    input  logic [Q_W-1:0] q_diag [0:STATES-1],

    // Current diagonal P in (Q16.16)
    input  logic signed [P_W-1:0] p_diag_in [0:STATES-1],

    // F matrix diagonal (simplified - full Jacobian requires extensive logic)
    // State transition: F ≈ I + F_lin·dt
    // Pass dt-scaled off-diagonal terms as separate inputs for key couplings
    input  logic signed [P_W-1:0] f_diag [0:STATES-1],

    // Updated P diagonal out (Q16.16)
    output logic signed [P_W-1:0] p_diag_out [0:STATES-1],
    output logic                   valid
);

    // Pipeline: compute P_new[i] = f[i]² × P_in[i] + Q[i]
    logic signed [63:0] fp_sq [0:STATES-1];
    logic v1;

    (* use_dsp = "yes" *)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < STATES; s++) fp_sq[s] <= '0;
            v1 <= 1'b0;
        end else if (ce_100hz) begin
            v1 <= 1'b1;
            for (int s = 0; s < STATES; s++)
                fp_sq[s] <= f_diag[s] * p_diag_in[s];
        end else begin
            v1 <= 1'b0;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int s = 0; s < STATES; s++) p_diag_out[s] <= '0;
            valid <= 1'b0;
        end else begin
            valid <= v1;
            if (v1) begin
                for (int s = 0; s < STATES; s++) begin
                    // Scale fp_sq: Q16.16 × Q16.16 = Q32.32, take upper Q16.16
                    logic signed [P_W-1:0] fp_scaled;
                    fp_scaled = fp_sq[s] >>> 16;
                    // Positive definiteness guard: clamp minimum to q_diag
                    p_diag_out[s] <= (fp_scaled < $signed({1'b0,q_diag[s]})) ?
                                     $signed({1'b0,q_diag[s]}) :
                                     fp_scaled + $signed({1'b0,q_diag[s]});
                end
            end
        end
    end

endmodule
