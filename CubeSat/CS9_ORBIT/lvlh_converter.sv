// =============================================================================
// Module: lvlh_converter (CS9 ECI → LVLH frame converter) - enhanced
// Subsystem: CS9 - Orbit Propagator
// Description: Converts ECI position and velocity vectors into the full
//              Local Vertical Local Horizontal (LVLH) frame:
//
//   r_hat = r / |r|           (nadir / radial    - LVLH Z-axis)
//   h_hat = (r × v) / |r×v|  (orbit normal       - LVLH Y-axis)
//   t_hat = h_hat × r_hat     (along-track        - LVLH X-axis)
//
//   All inputs/outputs are Q15.16 fixed-point.  Magnitudes use the same
//   leading-zero-count bit-shift normalisation as the original r_hat path.
//
//   Enhancement (v2) adds:
//     lvlh_matrix[0:8] - full 3×3 rotation matrix packed as:
//       [0..2] = t_hat (X), [3..5] = h_hat (Y), [6..8] = r_hat (Z)
//     h_hat_x/y/z - orbit-normal unit vector (LVLH Y)
//     t_hat_x/y/z - along-track unit vector  (LVLH X)
//
//   Existing ports lvlh_x/y/z (= r_hat) are unchanged for backward compat.
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module lvlh_converter (
    input  logic        clk,
    input  logic        rst_n,

    // ECI position (km Q15.16) and velocity (km/s Q15.16)
    input  logic [31:0] eci_pos [0:2],
    input  logic [31:0] eci_vel [0:2],
    input  logic        eci_valid,

    // LVLH radial unit vector (r_hat) Q15.16 - backward-compatible
    output logic [31:0] lvlh_x,
    output logic [31:0] lvlh_y,
    output logic [31:0] lvlh_z,
    output logic        lvlh_valid,

    // Enhanced outputs (v2)
    output logic [31:0] h_hat_x,           // orbit-normal (LVLH Y)
    output logic [31:0] h_hat_y,
    output logic [31:0] h_hat_z,
    output logic [31:0] t_hat_x,           // along-track  (LVLH X)
    output logic [31:0] t_hat_y,
    output logic [31:0] t_hat_z,
    output logic [31:0] lvlh_matrix [0:8]  // [t_hat|h_hat|r_hat] row-major
);

    // =========================================================================
    // Registered pipeline stage 1: capture inputs and compute r² = rx²+ry²+rz²
    // =========================================================================
    logic [31:0] pos_r [0:2];
    logic        s1_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) pos_r[i] <= '0;
            s1_valid <= 1'b0;
        end else begin
            s1_valid <= eci_valid;
            if (eci_valid) begin
                for (int i = 0; i < 3; i++) pos_r[i] <= eci_pos[i];
            end
        end
    end

    // =========================================================================
    // Stage 2: magnitude squared |r|²  (Q31.32, truncated to 48-bit)
    // =========================================================================
    logic [47:0] r_sq;
    logic [31:0] pos_s2 [0:2];
    logic        s2_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_sq    <= '0;
            s2_valid <= 1'b0;
        end else begin
            s2_valid <= s1_valid;
            if (s1_valid) begin
                for (int i = 0; i < 3; i++) pos_s2[i] <= pos_r[i];
                r_sq <= ($signed(pos_r[0]) * $signed(pos_r[0]) +
                         $signed(pos_r[1]) * $signed(pos_r[1]) +
                         $signed(pos_r[2]) * $signed(pos_r[2])) >>> 16;
            end
        end
    end

    // =========================================================================
    // Stage 3: approximate |r| via bit-length normalisation
    //   |r| ≈ 2^(floor(log2(r_sq)/2)) using leading-zero count
    //   Then r_hat[i] = pos[i] / |r|   ≈  pos[i] >> shift
    // =========================================================================
    logic [5:0]  r_shift;   // half the bit position of the leading '1' in r_sq
    logic [31:0] r_hat_out [0:2];
    logic        s3_valid;

    // Leading-zero count on r_sq (48-bit) - synthesisable priority encoder
    always_comb begin
        r_shift = 6'd0;
        for (int b = 47; b >= 0; b--) begin
            if (r_sq[b]) begin
                r_shift = 6'(b >> 1); // floor(log2(r_sq)/2) gives |r| exponent
                break;
            end
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) r_hat_out[i] <= '0;
            s3_valid <= 1'b0;
        end else begin
            s3_valid <= s2_valid;
            if (s2_valid) begin
                // Normalise each component: divide by approx |r| via right shift
                // Result is in Q15.16 unit range (-1 … +1)
                for (int i = 0; i < 3; i++) begin
                    if (r_shift > 6'd16)
                        r_hat_out[i] <= 32'(signed'(pos_s2[i]) >>> (r_shift - 6'd16));
                    else
                        r_hat_out[i] <= 32'(signed'(pos_s2[i]) <<< (6'd16 - r_shift));
                end
            end
        end
    end

    // =========================================================================
    // Output registration
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lvlh_x     <= '0;
            lvlh_y     <= '0;
            lvlh_z     <= '0;
            lvlh_valid <= 1'b0;
        end else begin
            lvlh_valid <= s3_valid;
            if (s3_valid) begin
                lvlh_x <= r_hat_out[0];
                lvlh_y <= r_hat_out[1];
                lvlh_z <= r_hat_out[2];
            end
        end
    end

    // =========================================================================
    // Stage 4: pipeline velocity alongside position (s1→s2) for cross product
    // =========================================================================
    logic [31:0] vel_s1 [0:2];
    logic [31:0] vel_s2 [0:2];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) begin
                vel_s1[i] <= '0;
                vel_s2[i] <= '0;
            end
        end else begin
            if (eci_valid)
                for (int i = 0; i < 3; i++) vel_s1[i] <= eci_vel[i];
            if (s1_valid)
                for (int i = 0; i < 3; i++) vel_s2[i] <= vel_s1[i];
        end
    end

    // =========================================================================
    // Stage 4b: compute h_vec = pos_s2 × vel_s2  (both Q15.16 → result Q15.16)
    //   hx = py×vz - pz×vy
    //   hy = pz×vx - px×vz
    //   hz = px×vy - py×vx
    // =========================================================================
    logic signed [31:0] h_vec [0:2];
    logic [47:0]        h_mag_sq;   // combinational, driven by always_comb below
    logic [5:0]         h_shift;
    logic [31:0]        h_hat_out [0:2];
    logic               s5_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) begin
                h_vec[i]     <= '0;
                h_hat_out[i] <= '0;
            end
            s5_valid <= 1'b0;
        end else begin
            s5_valid <= s2_valid;
            if (s2_valid) begin
                h_vec[0] <= 32'(($signed(pos_s2[1]) * $signed(vel_s2[2]) -
                                  $signed(pos_s2[2]) * $signed(vel_s2[1])) >>> 16);
                h_vec[1] <= 32'(($signed(pos_s2[2]) * $signed(vel_s2[0]) -
                                  $signed(pos_s2[0]) * $signed(vel_s2[2])) >>> 16);
                h_vec[2] <= 32'(($signed(pos_s2[0]) * $signed(vel_s2[1]) -
                                  $signed(pos_s2[1]) * $signed(vel_s2[0])) >>> 16);
            end
        end
    end

    // =========================================================================
    // Stage 5: compute |h|² from registered h_vec, then normalise h_hat
    // =========================================================================
    logic [5:0]  h_shift_lzc;

    // h_mag_sq driven combinationally from registered h_vec (always_comb)
    always_comb begin
        h_mag_sq = ($signed(h_vec[0]) * $signed(h_vec[0]) +
                    $signed(h_vec[1]) * $signed(h_vec[1]) +
                    $signed(h_vec[2]) * $signed(h_vec[2])) >>> 16;
    end

    always_comb begin
        h_shift_lzc = 6'd0;
        for (int b = 47; b >= 0; b--) begin
            if (h_mag_sq[b]) begin
                h_shift_lzc = 6'(b >> 1);
                break;
            end
        end
    end

    logic [31:0] h_hat_reg [0:2];
    logic        s6_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) h_hat_reg[i] <= '0;
            s6_valid <= 1'b0;
        end else begin
            s6_valid <= s5_valid;
            if (s5_valid) begin
                for (int i = 0; i < 3; i++) begin
                    if (h_shift_lzc > 6'd16)
                        h_hat_reg[i] <= 32'(signed'(h_vec[i]) >>> (h_shift_lzc - 6'd16));
                    else
                        h_hat_reg[i] <= 32'(signed'(h_vec[i]) <<< (6'd16 - h_shift_lzc));
                end
            end
        end
    end

    // =========================================================================
    // Stage 6: t_hat = h_hat × r_hat
    //   tx = hy×rz - hz×ry
    //   ty = hz×rx - hx×rz
    //   tz = hx×ry - hy×rx
    // r_hat_out is from stage 3; h_hat_reg is from stage 5.
    // Both are available at s6_valid (h pipeline is one cycle behind r pipeline
    // at s3 since it diverges from s2 not s3, but s3_valid and s5_valid both
    // derive from s2_valid+1 and s2_valid+0 respectively.  Align by registering
    // r_hat through one extra register.
    // =========================================================================
    logic [31:0] r_hat_s6 [0:2];

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) r_hat_s6[i] <= '0;
        end else begin
            if (s3_valid)
                for (int i = 0; i < 3; i++) r_hat_s6[i] <= r_hat_out[i];
        end
    end

    logic [31:0] t_hat_reg [0:2];
    logic        s7_valid;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 3; i++) t_hat_reg[i] <= '0;
            s7_valid <= 1'b0;
        end else begin
            s7_valid <= s6_valid;
            if (s6_valid) begin
                t_hat_reg[0] <= 32'(($signed(h_hat_reg[1]) * $signed(r_hat_s6[2]) -
                                      $signed(h_hat_reg[2]) * $signed(r_hat_s6[1])) >>> 16);
                t_hat_reg[1] <= 32'(($signed(h_hat_reg[2]) * $signed(r_hat_s6[0]) -
                                      $signed(h_hat_reg[0]) * $signed(r_hat_s6[2])) >>> 16);
                t_hat_reg[2] <= 32'(($signed(h_hat_reg[0]) * $signed(r_hat_s6[1]) -
                                      $signed(h_hat_reg[1]) * $signed(r_hat_s6[0])) >>> 16);
            end
        end
    end

    // =========================================================================
    // Stage 7: output registration for enhanced signals
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            h_hat_x <= '0; h_hat_y <= '0; h_hat_z <= '0;
            t_hat_x <= '0; t_hat_y <= '0; t_hat_z <= '0;
            for (int i = 0; i < 9; i++) lvlh_matrix[i] <= '0;
        end else if (s7_valid) begin
            // h_hat (LVLH Y)
            h_hat_x <= h_hat_reg[0];
            h_hat_y <= h_hat_reg[1];
            h_hat_z <= h_hat_reg[2];
            // t_hat (LVLH X)
            t_hat_x <= t_hat_reg[0];
            t_hat_y <= t_hat_reg[1];
            t_hat_z <= t_hat_reg[2];
            // Full matrix [t|h|r] row-major
            lvlh_matrix[0] <= t_hat_reg[0];
            lvlh_matrix[1] <= t_hat_reg[1];
            lvlh_matrix[2] <= t_hat_reg[2];
            lvlh_matrix[3] <= h_hat_reg[0];
            lvlh_matrix[4] <= h_hat_reg[1];
            lvlh_matrix[5] <= h_hat_reg[2];
            lvlh_matrix[6] <= r_hat_s6[0];
            lvlh_matrix[7] <= r_hat_s6[1];
            lvlh_matrix[8] <= r_hat_s6[2];
        end
    end

endmodule