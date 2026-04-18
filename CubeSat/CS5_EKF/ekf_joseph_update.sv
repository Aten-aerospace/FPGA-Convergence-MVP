// =============================================================================
// Module: ekf_joseph_update
// Subsystem: CS5 - Extended Kalman Filter
// Description: Sequential scalar Kalman update for a 7-state EKF.
//   Processes 6 measurements (3 accel + 3 mag) one at a time via rank-1
//   P-matrix updates (equivalent to the Joseph form for scalar measurements).
//
//   Stores the full 7×7 covariance matrix P (49 registers, Q15).
//   Stores the 7-element state vector x = [q0,q1,q2,q3,bx,by,bz] (Q15).
//
//   Per-measurement update algorithm:
//     HP[j]   = sum_k H[i][k] * P[k][j]   (row × P columns, 6 MACs, 14 cycles)
//     s_i     = sum_j HP[j] * H[i][j] + R  (Q15, 2 cycles + saturation)
//     K[j]    = HP[j] * 2^15 / s_i         (15-cycle serial restoring divider × 7)
//     innov_i = z[i] - h_meas[i]           (1 subtract)
//     x[j]   += K[j] * innov_i >> 15       (7 MACs, 2 cycles)
//     P[j][k] -= K[j] * HP[k] >> 15        (49 updates, 1 per cycle)
//
//   After 6 measurements: positive-definite check on P diagonal.
//   Peak DSP48: 6 (during HP computation; divider uses adder logic only).
//
//   Total latency: ~7 ms budget.  Typical: ~1300 cycles = 13 µs @ 100 MHz.
//
//   Output signals:
//     x_est[0:6]      - updated 7-state vector (Q15)
//     P_diag[0:6]     - diagonal covariance elements (Q15)
//     K_gain[0:6][0:5]- Kalman gain for each measurement (Q15)
//     innovation[0:5] - measurement residuals z - h(x) (Q15)
//     pd_ok           - 1 if all P diagonal elements > PD_MIN
//     valid_out       - pulses 1 cycle when update complete
//
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module ekf_joseph_update #(
    // Measurement noise (Q15): accel channels 0-2, mag channels 3-5
    parameter signed [15:0] R_ACCEL  = 16'sh0148, // ~0.01 in Q15
    parameter signed [15:0] R_MAG    = 16'sh0148,
    // Positive-definite minimum threshold for P diagonal
    parameter signed [15:0] PD_MIN   = 16'sh0001,
    // Max bias clamp (Q15)
    parameter signed [15:0] BIAS_MAX = 16'sh4000,
    // Steady-state σ_q threshold: flag if P_diag[0:3] > SIGMA_Q_THRESH
    // ~0.5° equivalent in quaternion space (262 / 32768 ≈ 0.008 rad ≈ 0.46°)
    parameter signed [15:0] SIGMA_Q_THRESH = 16'sh0106
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        start,               // pulse to begin update

    // Initial state (from predict step)
    input  logic signed [15:0] x_in    [0:6],   // [q0-q3, bx-bz] Q15
    input  logic signed [15:0] P_in    [0:6][0:6], // initial P (Q15)

    // Measurement function and H Jacobian (from ekf_measurement_model)
    input  logic signed [15:0] h_meas  [0:5],   // predicted measurements Q15
    input  logic signed [15:0] H_mat   [0:5][0:6], // H Jacobian (6×7) Q15

    // Raw measurements (Q15)
    input  logic signed [15:0] z_meas  [0:5],   // [ax,ay,az,mx,my,mz] Q15

    // Outputs
    output logic signed [15:0] x_est   [0:6],   // updated state Q15
    output logic signed [15:0] P_diag  [0:6],   // P diagonal Q15
    output logic signed [15:0] K_gain  [0:6][0:5], // Kalman gain Q15
    output logic signed [15:0] innovation [0:5],   // z - h(x) Q15
    output logic               pd_ok,              // all P[i,i] > PD_MIN
    output logic               sigma_ok,            // all |q| variances ≤ threshold
    output logic               valid_out
);

    // =========================================================================
    // Internal P matrix and state registers
    // =========================================================================
    logic signed [15:0] P_reg [0:6][0:6];
    logic signed [15:0] x_reg [0:6];

    // Working registers for current measurement
    logic signed [33:0] HP_acc [0:5];          // HP[0..5] accumulators (Q30)
    logic signed [33:0] HP6_acc;               // HP[6] accumulator
    logic signed [15:0] HP    [0:6];           // HP results (Q15)
    logic signed [33:0] s_acc;                 // s accumulator (Q30 sum)
    logic signed [15:0] s_reg;                 // s (Q15, saturated to [1,32767])
    logic signed [15:0] K_reg [0:6];           // K[j] for current measurement
    logic signed [15:0] innov_i;               // z[i] - h_meas[i]
    logic [2:0]         meas_idx;              // current measurement (0..5)
    logic [5:0]         k_cnt;                 // general accumulation counter
    logic [5:0]         p_cnt;                 // P update pair counter (0..48)
    logic [3:0]         j_k_cnt;              // K division j-index + bit counter
    // Row/column counters for P rank-1 update (replaces division by 7)
    logic [2:0]         p_row;                 // 0..6
    logic [2:0]         p_col;                 // 0..6

    // =========================================================================
    // Serial restoring divider state (for K[j] = HP[j]<<15 / s)
    // =========================================================================
    logic [30:0] div_num;   // unsigned numerator (|HP[j]|<<15, 31-bit)
    logic [15:0] div_den;   // unsigned denominator (|s_reg|, 16-bit)
    logic [30:0] div_rem;   // running remainder
    logic [14:0] div_quot;  // current quotient bits
    logic [3:0]  div_bit;   // bit counter 14 downto 0
    logic        div_neg;   // sign of K[j] (HP[j][15] XOR s_reg[15])
    logic [2:0]  k_j_idx;   // which j we are dividing for (0..6)

    // =========================================================================
    // Module-scope temporaries used inside FSM states (synthesis portability)
    // =========================================================================
    logic signed [17:0] fsm_s_tmp;        // S_S_STORE: s accumulator shift
    logic signed [15:0] fsm_k_val;        // S_K_STORE: computed K[j]
    logic signed [31:0] fsm_prod;         // shared multiply result
    logic signed [15:0] fsm_delta;        // shared Q30→Q15 result
    logic signed [16:0] fsm_sum;          // shared saturating add/sub
    logic signed [16:0] fsm_newP;         // S_P_UPD: new P element
    logic               fsm_pd_all_ok;    // S_PD_CHECK accumulator
    logic               fsm_sigma_all_ok; // S_PD_CHECK sigma accumulator
    logic [30:0]        fsm_shifted_rem;  // S_K_DIV: shifted remainder
    logic [30:0]        fsm_diff;         // S_K_DIV: trial subtraction

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [4:0] {
        S_IDLE,
        S_LOAD,         // Load P_in and x_in (1 cycle)
        S_INNOV,        // Compute innovation for current meas (1 cycle)
        S_HP_INIT,      // Clear HP accumulators (1 cycle)
        S_HP05,         // Accumulate HP[0..5]: k_cnt = 0..6 (7 cycles)
        S_HP6,          // Accumulate HP[6]:    k_cnt = 0..6 (7 cycles)
        S_HP_STORE,     // Convert Q30→Q15, store HP[0..6] (1 cycle)
        S_S_INIT,       // Clear s accumulator (1 cycle)
        S_S_COMP0,      // s += HP[0..5]*H[i][0..5]: 6 MACs (1 cycle)
        S_S_COMP1,      // s += HP[6]*H[i][6] + R_i  (1 cycle)
        S_S_STORE,      // Saturate s, store s_reg (1 cycle)
        S_K_INIT,       // Load divider for K[0] (1 cycle)
        S_K_DIV,        // 15 restoring-division steps (15 cycles per K value)
        S_K_STORE,      // Store K[j], advance to next j (1 cycle)
        S_X_UPD0,       // x[0..5] += K[0..5]*innov_i >> 15 (6 MACs, 1 cycle)
        S_X_UPD1,       // x[6]    += K[6]   *innov_i >> 15 (1 MAC,  1 cycle)
        S_P_UPD,        // P[j][k] -= K[j]*HP[k]>>15, 1 pair/cycle (49 cycles)
        S_NEXT_MEAS,    // Advance meas_idx or goto PD_CHECK (1 cycle)
        S_PD_CHECK,     // Verify P diagonal > PD_MIN (1 cycle)
        S_DONE          // Assert valid_out for 1 cycle
    } fsm_t;

    fsm_t state;

    // Convenience: R for current measurement index
    function automatic logic signed [15:0] get_R;
        input logic [2:0] idx;
        get_R = (idx < 3'd3) ? R_ACCEL : R_MAG;
    endfunction

    // Saturating Q30→Q15 (34-bit → 16-bit)
    function automatic logic signed [15:0] q30_to_q15;
        input logic signed [33:0] v;
        logic signed [17:0] s;
        s = v[33:16];
        if (s > 18'sh7FFF)       q30_to_q15 = 16'sh7FFF;
        else if (s < -18'sh8000) q30_to_q15 = 16'sh8000;
        else                     q30_to_q15 = s[15:0];
    endfunction

    // Saturating add Q15
    function automatic logic signed [15:0] sat_add;
        input logic signed [15:0] a, b;
        logic signed [16:0] s;
        s = {a[15], a} + {b[15], b};
        if (s > 17'sh7FFF)       sat_add = 16'sh7FFF;
        else if (s < -17'sh8000) sat_add = 16'sh8000;
        else                     sat_add = s[15:0];
    endfunction

    // Saturating sub Q15
    function automatic logic signed [15:0] sat_sub;
        input logic signed [15:0] a, b;
        logic signed [16:0] s;
        s = {a[15], a} - {b[15], b};
        if (s > 17'sh7FFF)       sat_sub = 16'sh7FFF;
        else if (s < -17'sh8000) sat_sub = 16'sh8000;
        else                     sat_sub = s[15:0];
    endfunction

    // =========================================================================
    // Main FSM
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= S_IDLE;
            valid_out <= 1'b0;
            pd_ok     <= 1'b1;
            sigma_ok  <= 1'b1;
            meas_idx  <= '0;
            k_cnt     <= '0;
            p_cnt     <= '0;
            p_row     <= '0;
            p_col     <= '0;
            k_j_idx   <= '0;
            div_bit   <= '0;
            for (int i = 0; i < 6; i++) innovation[i] <= '0;
            for (int i = 0; i < 7; i++) begin
                x_est[i]  <= '0;
                P_diag[i] <= '0;
                for (int j = 0; j < 6; j++) K_gain[i][j] <= '0;
            end
            for (int i = 0; i < 7; i++) begin
                x_reg[i] <= '0;
                for (int j = 0; j < 7; j++) P_reg[i][j] <= '0;
            end
        end else begin
            valid_out <= 1'b0;

            case (state)

                // ----------------------------------------------------------
                S_IDLE: begin
                    if (start) state <= S_LOAD;
                end

                // ----------------------------------------------------------
                S_LOAD: begin
                    // Load initial P and x from inputs
                    for (int i = 0; i < 7; i++) begin
                        x_reg[i] <= x_in[i];
                        for (int j = 0; j < 7; j++)
                            P_reg[i][j] <= P_in[i][j];
                    end
                    meas_idx <= 3'd0;
                    state    <= S_INNOV;
                end

                // ----------------------------------------------------------
                S_INNOV: begin
                    // Compute innovation for current measurement
                    innov_i  <= sat_sub(z_meas[meas_idx], h_meas[meas_idx]);
                    state    <= S_HP_INIT;
                end

                // ----------------------------------------------------------
                S_HP_INIT: begin
                    for (int j = 0; j < 6; j++) HP_acc[j] <= '0;
                    HP6_acc <= '0;
                    k_cnt   <= 6'd0;
                    state   <= S_HP05;
                end

                // ----------------------------------------------------------
                // Compute HP[0..5]: for j=0..5, HP_acc[j] += H[i][k]*P[k][j]
                // Uses 6 MACs simultaneously (one per j), k_cnt steps k=0..6
                // ----------------------------------------------------------
                S_HP05: begin
                    for (int j = 0; j < 6; j++)
                        HP_acc[j] <= HP_acc[j] + H_mat[meas_idx][k_cnt] * P_reg[k_cnt][j];
                    if (k_cnt == 6'd6) begin
                        k_cnt <= 6'd0;
                        state <= S_HP6;
                    end else
                        k_cnt <= k_cnt + 6'd1;
                end

                // ----------------------------------------------------------
                // Compute HP[6]: HP6_acc += H[i][k]*P[k][6], 1 MAC per step
                // ----------------------------------------------------------
                S_HP6: begin
                    HP6_acc <= HP6_acc + H_mat[meas_idx][k_cnt] * P_reg[k_cnt][6];
                    if (k_cnt == 6'd6)
                        state <= S_HP_STORE;
                    else
                        k_cnt <= k_cnt + 6'd1;
                end

                // ----------------------------------------------------------
                S_HP_STORE: begin
                    for (int j = 0; j < 6; j++)
                        HP[j] <= q30_to_q15(HP_acc[j]);
                    HP[6]  <= q30_to_q15(HP6_acc);
                    state  <= S_S_INIT;
                end

                // ----------------------------------------------------------
                S_S_INIT: begin
                    s_acc <= '0;
                    state <= S_S_COMP0;
                end

                // ----------------------------------------------------------
                // s += HP[0..5]*H[i][0..5]: 6 MACs (1 cycle)
                // ----------------------------------------------------------
                S_S_COMP0: begin
                    s_acc <= s_acc
                           + HP[0] * H_mat[meas_idx][0]
                           + HP[1] * H_mat[meas_idx][1]
                           + HP[2] * H_mat[meas_idx][2]
                           + HP[3] * H_mat[meas_idx][3]
                           + HP[4] * H_mat[meas_idx][4]
                           + HP[5] * H_mat[meas_idx][5];
                    state <= S_S_COMP1;
                end

                // ----------------------------------------------------------
                // s += HP[6]*H[i][6] + R_i
                // ----------------------------------------------------------
                S_S_COMP1: begin
                    s_acc <= s_acc
                           + HP[6] * H_mat[meas_idx][6]
                           + (34'(get_R(meas_idx)) << 15); // R in Q30 (zero-extend to 34-bit then shift)
                    state <= S_S_STORE;
                end

                // ----------------------------------------------------------
                S_S_STORE: begin
                    // s_acc is Q30; convert to Q15 and clamp to [1, 32767]
                    fsm_s_tmp = s_acc[33:16]; // Q30 >> 15 = Q15 (18-bit signed)
                    if      (fsm_s_tmp <= 18'sh0001) s_reg <= 16'sh0001;
                    else if (fsm_s_tmp >= 18'sh7FFF) s_reg <= 16'sh7FFF;
                    else                              s_reg <= fsm_s_tmp[15:0];
                    k_j_idx <= 3'd0;
                    state   <= S_K_INIT;
                end

                // ----------------------------------------------------------
                S_K_INIT: begin
                    div_neg  <= HP[k_j_idx][15] ^ s_reg[15];
                    div_num  <= HP[k_j_idx][15]
                                ? ({1'b0, ~HP[k_j_idx][14:0] + 15'd1}) << 15
                                : {1'b0, HP[k_j_idx][14:0]} << 15;
                    div_den  <= s_reg[15] ? (~s_reg + 1'b1) : s_reg;
                    div_rem  <= '0;
                    div_quot <= '0;
                    div_bit  <= 4'd14;
                    state    <= S_K_DIV;
                end

                // ----------------------------------------------------------
                // Restoring binary division: 15 steps (bit 14 downto 0)
                // ----------------------------------------------------------
                S_K_DIV: begin
                    fsm_shifted_rem = {div_rem[29:0], div_num[30]};
                    div_num         <= {div_num[29:0], 1'b0};
                    fsm_diff        = fsm_shifted_rem - {15'h0, div_den};
                    if (!fsm_diff[30]) begin
                        div_rem           <= fsm_diff;
                        div_quot[div_bit] <= 1'b1;
                    end else begin
                        div_rem           <= fsm_shifted_rem;
                        div_quot[div_bit] <= 1'b0;
                    end
                    if (div_bit == 4'd0)
                        state <= S_K_STORE;
                    else
                        div_bit <= div_bit - 4'd1;
                end

                // ----------------------------------------------------------
                S_K_STORE: begin
                    // div_quot is 15-bit unsigned quotient; apply sign
                    fsm_k_val                  = div_neg ? -(16'(div_quot)) : 16'(div_quot);
                    K_reg[k_j_idx]          <= fsm_k_val;
                    K_gain[k_j_idx][meas_idx] <= fsm_k_val;
                    if (k_j_idx == 3'd6) begin
                        state <= S_X_UPD0;
                    end else begin
                        k_j_idx <= k_j_idx + 3'd1;
                        state   <= S_K_INIT;
                    end
                end

                // ----------------------------------------------------------
                // State update x[0..5] += K[0..5]*innov_i >> 15 (6 MACs)
                // ----------------------------------------------------------
                S_X_UPD0: begin
                    for (int j = 0; j < 6; j++) begin
                        fsm_prod  = K_reg[j] * innov_i;
                        fsm_delta = fsm_prod[30:15];
                        fsm_sum   = {x_reg[j][15], x_reg[j]} + {fsm_delta[15], fsm_delta};
                        x_reg[j] <= (fsm_sum > 17'sh7FFF)  ? 16'sh7FFF :
                                    (fsm_sum < -17'sh8000) ? 16'sh8000 :
                                    fsm_sum[15:0];
                    end
                    state <= S_X_UPD1;
                end

                // ----------------------------------------------------------
                // State update x[6] += K[6]*innov_i >> 15 (1 MAC)
                // Also store innovation
                // ----------------------------------------------------------
                S_X_UPD1: begin
                    fsm_prod  = K_reg[6] * innov_i;
                    fsm_delta = fsm_prod[30:15];
                    fsm_sum   = {x_reg[6][15], x_reg[6]} + {fsm_delta[15], fsm_delta};
                    // Clamp bias states to ±BIAS_MAX
                    x_reg[6] <= (fsm_sum > $signed({1'b0, BIAS_MAX}))             ?  BIAS_MAX :
                                (fsm_sum < -$signed({1'b0, BIAS_MAX}))            ? -BIAS_MAX :
                                fsm_sum[15:0];
                    innovation[meas_idx] <= innov_i;
                    p_cnt  <= 6'd0;
                    p_row  <= 3'd0;
                    p_col  <= 3'd0;
                    state  <= S_P_UPD;
                end

                // ----------------------------------------------------------
                // P rank-1 update: P[j][k] -= K[j]*HP[k]>>15
                // Row/column counters p_row, p_col replace division-by-7
                // 49 cycles total
                // ----------------------------------------------------------
                S_P_UPD: begin
                    fsm_prod  = K_reg[p_row] * HP[p_col];
                    fsm_delta = fsm_prod[30:15];
                    fsm_newP  = {P_reg[p_row][p_col][15], P_reg[p_row][p_col]} - {fsm_delta[15], fsm_delta};
                    P_reg[p_row][p_col] <= (fsm_newP > 17'sh7FFF)  ? 16'sh7FFF :
                                           (fsm_newP < -17'sh8000) ? 16'sh8000 :
                                           fsm_newP[15:0];
                    if (p_cnt == 6'd48) begin
                        state <= S_NEXT_MEAS;
                    end else begin
                        p_cnt <= p_cnt + 6'd1;
                        if (p_col == 3'd6) begin
                            p_col <= 3'd0;
                            p_row <= p_row + 3'd1;
                        end else begin
                            p_col <= p_col + 3'd1;
                        end
                    end
                end

                // ----------------------------------------------------------
                S_NEXT_MEAS: begin
                    if (meas_idx == 3'd5)
                        state <= S_PD_CHECK;
                    else begin
                        meas_idx <= meas_idx + 3'd1;
                        state    <= S_INNOV;
                    end
                end

                // ----------------------------------------------------------
                // PD check: verify all P[i][i] > PD_MIN
                // Also compute sigma_ok flag (q-state variances)
                // ----------------------------------------------------------
                S_PD_CHECK: begin
                    fsm_pd_all_ok    = 1'b1;
                    fsm_sigma_all_ok = 1'b1;
                    for (int i = 0; i < 7; i++) begin
                        if ($signed(P_reg[i][i]) < $signed(PD_MIN))
                            fsm_pd_all_ok = 1'b0;
                    end
                    for (int i = 0; i < 4; i++) begin
                        if ($signed(P_reg[i][i]) > $signed(SIGMA_Q_THRESH))
                            fsm_sigma_all_ok = 1'b0;
                    end
                    pd_ok    <= fsm_pd_all_ok;
                    sigma_ok <= fsm_sigma_all_ok;
                    // Floor the diagonal at PD_MIN to keep P positive-definite
                    for (int i = 0; i < 7; i++) begin
                        if ($signed(P_reg[i][i]) < $signed(PD_MIN))
                            P_reg[i][i] <= PD_MIN;
                    end
                    state <= S_DONE;
                end

                // ----------------------------------------------------------
                S_DONE: begin
                    // Latch outputs
                    for (int i = 0; i < 7; i++) begin
                        x_est[i]  <= x_reg[i];
                        P_diag[i] <= P_reg[i][i];
                    end
                    valid_out <= 1'b1;
                    state     <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule