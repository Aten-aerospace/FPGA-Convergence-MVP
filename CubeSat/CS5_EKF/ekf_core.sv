// =============================================================================
// Module: ekf_core
// Subsystem: CS5 - Extended Kalman Filter
// Description: Top-level EKF core integrating predict and full Kalman update.
//              Implements 5σ divergence watchdog with 3-consecutive-strike
//              policy and last-known-good (LKG) snapshot recovery.
//
//   Data flow per 100 Hz epoch:
//     1. ekf_predict    : [q_state, bias] + gyro → x_pred  (8-cycle pipeline)
//     2. ekf_covariance : P_state + Q → P_pred              (1-cycle)
//     3. ekf_update     : x_pred + P_pred + meas → x_est, P_diag, innovation
//     4. Feedback       : x_state ← x_est (registered)
//
//   Divergence watchdog (5σ rule):
//     Any |innovation[i]| > INNOV_THRESH triggers a divergence count.
//     After DIV_STRIKE_MAX consecutive divergent updates, the state rolls back
//     to the last-known-good (LKG) snapshot. ekf_fault asserts.
//     The LKG snapshot is updated on every healthy epoch.
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module ekf_core #(
    parameter int CLK_HZ        = 100_000_000,
    parameter int FAULT_CYCLES  = 200_000,     // watchdog timeout at 100 MHz
    parameter signed [15:0] INNOV_THRESH = 16'sh4000, // 0.5 in Q15
    parameter int DIV_STRIKE_MAX = 3
)(
    input  logic        clk,
    input  logic        rst_n,

    // 100 Hz clock enable
    input  logic        ce_100hz,

    // Sensor measurements (signed Q15)
    input  logic signed [15:0] accel     [0:2],
    input  logic signed [15:0] gyro      [0:2],
    input  logic signed [15:0] mag       [0:2],
    input  logic               meas_valid,

    // EKF primary outputs
    output logic signed [15:0] q_est     [0:3],
    output logic signed [15:0] bias_est  [0:2],
    output logic               ekf_valid,
    output logic               ekf_fault,

    // Extended diagnostic outputs
    output logic signed [15:0] P_diag    [0:6],
    output logic signed [15:0] innovation [0:6]
);

    // =========================================================================
    // Local parameters
    // =========================================================================
    localparam signed [15:0] P_RESET_Q    = 16'sh0CCC; // 0.1  in Q15
    localparam signed [15:0] P_RESET_BIAS = 16'sh0147; // 0.01 in Q15
    localparam signed [15:0] Q_Q_NOISE    = 16'sh0014;
    localparam signed [15:0] Q_B_NOISE    = 16'sh0003;
    localparam [$clog2(FAULT_CYCLES)-1:0] WDOG_MAX = FAULT_CYCLES - 1;

    // =========================================================================
    // 7-state register: x_state = [q0, q1, q2, q3, bx, by, bz]
    // =========================================================================
    logic signed [15:0] x_state [0:6];
    logic signed [15:0] P_state [0:6][0:6];

    // Last-known-good snapshot
    logic signed [15:0] x_lkg   [0:6];
    logic signed [15:0] P_lkg   [0:6][0:6];

    // Divergence strike counter
    logic [1:0] div_strikes;

    // =========================================================================
    // Process-noise input arrays
    // =========================================================================
    logic signed [15:0] q_q_noise [0:3];
    logic signed [15:0] q_b_noise [0:2];

    always_comb begin
        for (int i = 0; i < 4; i++) q_q_noise[i] = Q_Q_NOISE;
        for (int i = 0; i < 3; i++) q_b_noise[i] = Q_B_NOISE;
    end

    // =========================================================================
    // ekf_predict
    // =========================================================================
    logic signed [15:0] q_in_for_predict  [0:3];
    logic signed [15:0] bias_in_for_predict [0:2];
    logic signed [15:0] q_pred_raw  [0:3];
    logic signed [15:0] bias_pred   [0:2];
    logic               pred_valid;

    always_comb begin
        for (int i = 0; i < 4; i++) q_in_for_predict[i]    = x_state[i];
        for (int i = 0; i < 3; i++) bias_in_for_predict[i] = x_state[i+4];
    end

    ekf_predict u_predict (
        .clk        (clk),
        .rst_n      (rst_n),
        .ce_100hz   (ce_100hz),
        .q_state    (q_in_for_predict),
        .gyro       (gyro),
        .gyro_valid (meas_valid),
        .bias_est   (bias_in_for_predict),
        .q_pred     (q_pred_raw),
        .bias_pred  (bias_pred),
        .pred_valid (pred_valid)
    );

    // Build 7-element x_pred (register to match pipeline)
    logic signed [15:0] x_pred [0:6];
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_pred[0] <= 16'sh7FFF;
            for (int i = 1; i < 7; i++) x_pred[i] <= '0;
        end else if (pred_valid) begin
            for (int i = 0; i < 4; i++) x_pred[i]   <= q_pred_raw[i];
            for (int i = 0; i < 3; i++) x_pred[i+4] <= bias_pred[i];
        end
    end

    // =========================================================================
    // ekf_covariance (predict step: add Q noise to diagonal)
    // =========================================================================
    logic signed [15:0] P_pred_out [0:6][0:6];
    logic signed [15:0] P_diag_cov [0:6];
    logic               cov_valid;
    logic               P_load_valid;
    logic signed [15:0] P_joseph_in [0:6][0:6]; // updated P fed back from update
    logic               pd_ok_cov;

    ekf_covariance u_cov (
        .clk          (clk),
        .rst_n        (rst_n),
        .update_valid (pred_valid),
        .P_in         (P_joseph_in),
        .P_in_valid   (P_load_valid),
        .q_q_noise    (q_q_noise),
        .q_b_noise    (q_b_noise),
        .P_out        (P_pred_out),
        .P_diag       (P_diag_cov),
        .pd_ok        (pd_ok_cov),
        .cov_valid    (cov_valid)
    );

    // =========================================================================
    // ekf_update
    // =========================================================================
    logic signed [15:0] x_est_raw   [0:6];
    logic signed [15:0] P_diag_upd  [0:6];
    logic signed [15:0] K_gain      [0:6][0:5];
    logic signed [15:0] innov_upd   [0:5];
    logic               pd_ok_upd;
    logic               sigma_ok_upd;
    logic               est_valid;

    ekf_update u_update (
        .clk        (clk),
        .rst_n      (rst_n),
        .x_pred     (x_pred),
        .pred_valid (pred_valid),
        .P_pred     (P_pred_out),
        .mag_meas   (mag),
        .mag_valid  (pred_valid),
        .accel_meas (accel),
        .x_est      (x_est_raw),
        .est_valid  (est_valid),
        .P_diag     (P_diag_upd),
        .K_gain     (K_gain),
        .innovation (innov_upd),
        .pd_ok      (pd_ok_upd),
        .sigma_ok   (sigma_ok_upd)
    );

    // =========================================================================
    // Build P_joseph_in from P_diag_upd + previous off-diagonals from P_state
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            P_load_valid <= 1'b0;
            for (int i = 0; i < 7; i++)
                for (int j = 0; j < 7; j++)
                    P_joseph_in[i][j] <= (i == j) ?
                        ((i < 4) ? P_RESET_Q : P_RESET_BIAS) : 16'sh0000;
        end else begin
            P_load_valid <= 1'b0;
            if (est_valid) begin
                P_load_valid <= 1'b1;
                for (int i = 0; i < 7; i++)
                    for (int j = 0; j < 7; j++)
                        P_joseph_in[i][j] <= (i == j) ? P_diag_upd[i] : P_state[i][j];
            end
        end
    end

    // =========================================================================
    // Divergence monitor: any |innovation| > threshold → strike
    // =========================================================================
    logic innov_diverged;
    always_comb begin
        innov_diverged = 1'b0;
        for (int i = 0; i < 6; i++) begin
            if ($signed(innov_upd[i]) > $signed(INNOV_THRESH) ||
                $signed(innov_upd[i]) < -$signed(INNOV_THRESH))
                innov_diverged = 1'b1;
        end
    end

    // =========================================================================
    // State feedback + LKG snapshot + divergence recovery
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_state[0] <= 16'sh7FFF;
            for (int i = 1; i < 7; i++) x_state[i] <= '0;
            for (int i = 0; i < 7; i++)
                for (int j = 0; j < 7; j++)
                    P_state[i][j] <= (i == j) ?
                        ((i < 4) ? P_RESET_Q : P_RESET_BIAS) : 16'sh0000;
            x_lkg[0] <= 16'sh7FFF;
            for (int i = 1; i < 7; i++) x_lkg[i] <= '0;
            for (int i = 0; i < 7; i++)
                for (int j = 0; j < 7; j++)
                    P_lkg[i][j] <= (i == j) ?
                        ((i < 4) ? P_RESET_Q : P_RESET_BIAS) : 16'sh0000;
            q_est[0]   <= 16'sh7FFF;
            for (int i = 1; i < 4; i++) q_est[i]   <= '0;
            for (int i = 0; i < 3; i++) bias_est[i] <= '0;
            for (int i = 0; i < 7; i++) P_diag[i]  <= (i < 4) ? P_RESET_Q : P_RESET_BIAS;
            for (int i = 0; i < 7; i++) innovation[i] <= '0;
            ekf_valid   <= 1'b0;
            div_strikes <= 2'd0;
        end else begin
            ekf_valid <= 1'b0;

            if (est_valid) begin
                if (innov_diverged) begin
                    if (div_strikes == 2'(DIV_STRIKE_MAX - 1)) begin
                        // Recovery: restore LKG
                        for (int i = 0; i < 7; i++) x_state[i] <= x_lkg[i];
                        for (int i = 0; i < 7; i++)
                            for (int j = 0; j < 7; j++) P_state[i][j] <= P_lkg[i][j];
                        div_strikes <= 2'd0;
                    end else begin
                        div_strikes <= div_strikes + 2'd1;
                        for (int i = 0; i < 7; i++) x_state[i] <= x_est_raw[i];
                        for (int i = 0; i < 7; i++)
                            for (int j = 0; j < 7; j++) P_state[i][j] <= P_joseph_in[i][j];
                    end
                end else begin
                    // Healthy: latch state + snapshot LKG
                    for (int i = 0; i < 7; i++) x_state[i] <= x_est_raw[i];
                    for (int i = 0; i < 7; i++) x_lkg[i]   <= x_est_raw[i];
                    for (int i = 0; i < 7; i++) begin
                        for (int j = 0; j < 7; j++) begin
                            P_state[i][j] <= P_joseph_in[i][j];
                            P_lkg[i][j]   <= P_joseph_in[i][j];
                        end
                    end
                    div_strikes <= 2'd0;
                end

                // Latch outputs
                for (int i = 0; i < 4; i++) q_est[i]    <= x_est_raw[i];
                for (int i = 0; i < 3; i++) bias_est[i] <= x_est_raw[i+4];
                for (int i = 0; i < 7; i++) P_diag[i]   <= P_diag_upd[i];
                for (int i = 0; i < 6; i++) innovation[i] <= innov_upd[i];
                innovation[6] <= '0;
                ekf_valid <= 1'b1;
            end
        end
    end

    // =========================================================================
    // Fault flags
    // =========================================================================
    // Divergence fault: registered flag; asserts on 3rd-strike recovery,
    // de-asserts once a healthy (non-divergent) epoch is processed.
    logic div_fault;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            div_fault <= 1'b0;
        end else if (est_valid) begin
            if (innov_diverged && div_strikes == 2'(DIV_STRIKE_MAX - 1))
                div_fault <= 1'b1;   // 3rd consecutive strike: recovery triggered
            else if (!innov_diverged)
                div_fault <= 1'b0;   // healthy epoch: clear fault
        end
    end

    // Watchdog timeout fault
    logic [$clog2(FAULT_CYCLES)-1:0] wdog_cnt;
    logic wdog_fault;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdog_cnt   <= '0;
            wdog_fault <= 1'b0;
        end else begin
            if (est_valid) begin
                wdog_cnt   <= '0;
                wdog_fault <= 1'b0;
            end else begin
                if (wdog_cnt == WDOG_MAX)
                    wdog_fault <= 1'b1;
                else
                    wdog_cnt <= wdog_cnt + 1'b1;
            end
        end
    end

    assign ekf_fault = div_fault | wdog_fault;

endmodule