    // =============================================================================
    // File        : innovation_gate.sv
    // Module      : innovation_gate
    // Description : Mahalanobis distance gate for EKF measurement innovation.
    //               Rejects outlier measurements when innovation² > threshold × S
    //               where S = H·P·Hᵀ + R (innovation covariance, scalar case).
    //               Threshold corresponds to 3σ (chi-squared 99.7%).
    // =============================================================================
    
    `timescale 1ns/1ps
    
    module innovation_gate #(
        parameter int DATA_W  = 32,  // Q16.16 innovation and covariance
        // 3σ threshold: chi²(1, 99.7%) ≈ 9.0 → Q16.16: 9 × 65536 = 589824
        parameter int THRESH  = 589824
    )(
        input  logic clk,
        input  logic rst_n,
        input  logic valid_in,
    
        // Scalar innovation and innovation covariance
        input  logic signed [DATA_W-1:0] innov,    // y - H·x̂ (Q16.16)
        input  logic signed [DATA_W-1:0] innov_cov, // S = H·P·Hᵀ + R (Q16.16)
    
        output logic accept,    // 1 = within gate, 0 = rejected
        output logic valid_out
    );
    
        // Compute innov² (Q16.16 × Q16.16 = Q32.32, take upper Q16.16)
        logic signed [63:0] innov_sq;
        logic signed [63:0] thresh_times_s;
        logic v1;
    
        (* use_dsp = "yes" *)
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                innov_sq       <= '0;
                thresh_times_s <= '0;
                v1             <= 1'b0;
            end else begin
                v1             <= valid_in;
                innov_sq       <= innov * innov;
                thresh_times_s <= innov_cov * THRESH;
            end
        end
    
        // Gate decision: accept if innov² ≤ THRESH × S (Q32.32 comparison)
        // innov_sq is Q32.32; thresh_times_s is Q16.16 × const → need common scaling
        // Both scaled to Q32.32 for comparison
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                accept    <= 1'b0;
                valid_out <= 1'b0;
            end else begin
                valid_out <= v1;
                accept    <= v1 && (innov_sq <= (thresh_times_s >>> 16));
            end
        end
    
    endmodule
