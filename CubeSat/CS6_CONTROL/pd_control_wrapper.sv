// =============================================================================
// Module: pd_control_wrapper (CS6 top-level wrapper)
// Subsystem: CS6 - PD Control Law (CS-ADCS-007)
// Description: Top-level wrapper connecting the CS6 RTL pipeline:
//
//   q_err[0:3] (4-element error quaternion from CS5/EKF)
//      │
//      │  extract vector part q_err[1:3] → q_err_vec[0:2]
//      ▼
//   pd_law (dynamic Kp/Kd via kp_reg/kd_reg; SAT_LIMIT=max - no internal clamp)
//      │  torque_raw[0:2]   (2-cycle pipeline)
//      ▼
//   torque_saturation (SAT_LIMIT=16'sh3FFF ≈ ±10 mNm; ENABLE_SAT_COUNT=1)
//      │  torque_cmd[0:2]   (1-cycle pipeline)
//      │  sat_flag[2:0]     → saturation_flag = |sat_flag
//      │  sat_count[15:0]
//      │  valid_out         → ctrl_valid
//
//   Total pipeline latency: 3 clock cycles.
//
//   Gain programmability (spec: Kp_coeff, Kd_coeff, axi_gain_write):
//     Option A - simple strobe: assert axi_gain_write for one cycle; Kp_coeff
//                and Kd_coeff are latched into kp_reg and kd_reg.
//     Option B - AXI4-Lite:    write to the register map (overrides Option A
//                if both occur in the same cycle).
//
//   AXI4-Lite Register Map (word-addressed, 4-byte aligned):
//     Offset 0x0 - KP_REG  : Kp gain   (RW, Q15, default 0x0CCD ≈ 0.1)
//     Offset 0x4 - KD_REG  : Kd gain   (RW, Q15, default 0x0666 ≈ 0.05)
//     Offset 0x8 - SAT_CNT : saturation event counter (RW: write any value to clear)
//     Offset 0xC - STATUS  : bit[0]=saturation_flag (RO)
//
// Spec interface (CS-ADCS-007):
//   Input : sys_clk, rst_n, ce_1khz, q_err[4], omega[3],
//           Kp_coeff, Kd_coeff, axi_gain_write
//   Output: torque_cmd[3], saturation_flag, sat_count, ctrl_valid
//
// Provenance: cubesat_requirements.md (CS-ADCS-007)
// =============================================================================
`timescale 1ns/1ps

module pd_control_wrapper (
    input  logic        clk,      // sys_clk: 100 MHz system clock
    input  logic        rst_n,    // active-low synchronous reset

    // 1 kHz clock enable from CS12
    input  logic        ce_1khz,

    // Full error quaternion [w,x,y,z] (signed Q15) from CS5/EKF (spec: q_err[4])
    // Vector part q_err[1:3] is extracted and fed to the PD law core.
    input  logic signed [15:0] q_err  [0:3],

    // Angular rate [ωx, ωy, ωz] (signed Q15, rad/s) from CS1/IMU (spec: omega[3])
    input  logic signed [15:0] omega  [0:2],

    // Measurement-valid handshake (assert when q_err/omega are stable)
    input  logic               meas_valid,

    // =========================================================================
    // Gain loading - simple strobe interface (spec: Kp_coeff, Kd_coeff, axi_gain_write)
    // Assert axi_gain_write for one cycle to latch Kp_coeff and Kd_coeff.
    // AXI4-Lite writes (below) take priority when both occur in the same cycle.
    // =========================================================================
    input  logic signed [15:0] Kp_coeff,
    input  logic signed [15:0] Kd_coeff,
    input  logic               axi_gain_write,

    // =========================================================================
    // Spec outputs (CS-ADCS-007)
    // =========================================================================
    output logic signed [15:0] torque_cmd [0:2], // torque_cmd[3]
    output logic               saturation_flag,   // any axis clamped this cycle
    output logic        [15:0] sat_count,          // saturation event counter
    output logic               ctrl_valid,         // output valid strobe

    // =========================================================================
    // Optional full AXI4-Lite slave interface for CS12 production integration
    // (superset of axi_gain_write; tie all inputs to 0 when not used)
    // =========================================================================
    // Write address channel
    input  logic [3:0]  axi_awaddr,
    input  logic        axi_awvalid,
    output logic        axi_awready,
    // Write data channel
    input  logic [15:0] axi_wdata,
    input  logic        axi_wvalid,
    output logic        axi_wready,
    // Write response channel
    output logic [1:0]  axi_bresp,
    output logic        axi_bvalid,
    input  logic        axi_bready,
    // Read address channel
    input  logic [3:0]  axi_araddr,
    input  logic        axi_arvalid,
    output logic        axi_arready,
    // Read data channel
    output logic [15:0] axi_rdata,
    output logic [1:0]  axi_rresp,
    output logic        axi_rvalid,
    input  logic        axi_rready
);

    // Default gains (synthesizable constants; used as reset values for registers)
    localparam signed [15:0] KP_DEFAULT = 16'sh0CCD; // round(0.1  × 32768) = 3277
    localparam signed [15:0] KD_DEFAULT = 16'sh0666; // round(0.05 × 32768) = 1638

    // =========================================================================
    // Gain registers - loaded by axi_gain_write strobe or AXI4-Lite writes.
    // AXI4-Lite write takes explicit priority over the simple strobe.
    // =========================================================================
    logic [15:0] kp_reg;      // Offset 0x0 - Kp gain (Q15, RW)
    logic [15:0] kd_reg;      // Offset 0x4 - Kd gain (Q15, RW)

    logic        aw_captured; // AXI write-address captured
    logic [3:0]  aw_addr_r;   // latched AXI write address
    logic        sat_clear_r; // one-cycle strobe to clear sat_count

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kp_reg      <= KP_DEFAULT;
            kd_reg      <= KD_DEFAULT;
            aw_captured <= 1'b0;
            aw_addr_r   <= 4'h0;
            sat_clear_r <= 1'b0;
            axi_awready <= 1'b1;
            axi_wready  <= 1'b1;
            axi_bresp   <= 2'b00; // OKAY
            axi_bvalid  <= 1'b0;
            axi_arready <= 1'b1;
            axi_rdata   <= 16'h0;
            axi_rresp   <= 2'b00;
            axi_rvalid  <= 1'b0;
        end else begin
            sat_clear_r <= 1'b0; // default: deassert each cycle

            // ---- Simple gain-write strobe (spec: axi_gain_write) ----
            // Lowest priority - overridden by AXI write below if simultaneous
            if (axi_gain_write) begin
                kp_reg <= Kp_coeff;
                kd_reg <= Kd_coeff;
            end

            // ---- AXI write address channel ----
            if (axi_awvalid && axi_awready) begin
                aw_addr_r   <= axi_awaddr;
                aw_captured <= 1'b1;
                axi_awready <= 1'b0;
            end

            // ---- AXI write data + register update (higher priority than strobe) ----
            if (axi_wvalid && axi_wready && aw_captured) begin
                case (aw_addr_r[3:2])
                    2'h0: begin kp_reg      <= axi_wdata; axi_bresp <= 2'b00; end  // Kp gain
                    2'h1: begin kd_reg      <= axi_wdata; axi_bresp <= 2'b00; end  // Kd gain
                    2'h2: begin sat_clear_r <= 1'b1;      axi_bresp <= 2'b00; end  // SAT_CNT: write to clear
                    // 0xC (STATUS) is read-only
                    default: axi_bresp <= 2'b10; // SLVERR - write to RO or reserved address
                endcase
                axi_wready  <= 1'b0;
                aw_captured <= 1'b0;
                axi_bvalid  <= 1'b1;
            end

            // ---- AXI write response ----
            if (axi_bvalid && axi_bready) begin
                axi_bvalid  <= 1'b0;
                axi_awready <= 1'b1;
                axi_wready  <= 1'b1;
            end

            // ---- AXI read ----
            if (axi_arvalid && axi_arready) begin
                axi_arready <= 1'b0;
                axi_rvalid  <= 1'b1;
                case (axi_araddr[3:2])
                    2'h0: axi_rdata <= kp_reg;
                    2'h1: axi_rdata <= kd_reg;
                    2'h2: axi_rdata <= sat_count;             // from torque_saturation
                    2'h3: axi_rdata <= {15'h0, saturation_flag}; // STATUS
                    default: axi_rdata <= 16'h0;
                endcase
            end

            if (axi_rvalid && axi_rready) begin
                axi_rvalid  <= 1'b0;
                axi_arready <= 1'b1;
            end
        end
    end

    // =========================================================================
    // Extract vector part of error quaternion
    // q_err[0] = scalar w (unused by PD law)
    // q_err[1:3] = vector part [x, y, z] → passed to pd_law as q_err_vec[0:2]
    // =========================================================================
    logic signed [15:0] q_err_vec [0:2];
    assign q_err_vec[0] = q_err[1]; // x-axis error component
    assign q_err_vec[1] = q_err[2]; // y-axis error component
    assign q_err_vec[2] = q_err[3]; // z-axis error component

    // =========================================================================
    // pd_law: PD control law with AXI-programmable gains
    // SAT_LIMIT set to max to pass raw PD output to torque_saturation below.
    // torque_saturation performs the physical ±10 mNm clamp.
    // =========================================================================
    logic signed [15:0] pd_torque  [0:2]; // raw torque from pd_law (pre-saturation)
    logic               pd_valid;          // pd_law output valid (2-cycle latency)

    pd_law #(
        .DEFAULT_KP (KP_DEFAULT),
        .DEFAULT_KD (KD_DEFAULT),
        .SAT_LIMIT  (16'sh7FFF)   // max: disable internal clamp, let torque_saturation do it
    ) u_pd (
        .clk        (clk),
        .rst_n      (rst_n),
        .ce_1khz    (ce_1khz),
        .q_err      (q_err_vec),
        .omega      (omega),
        .ctrl_valid (meas_valid),
        .Kp         (kp_reg),
        .Kd         (kd_reg),
        .torque_cmd (pd_torque),
        .sat_flag   (),           // internal overflow flag; not exposed (use saturation_flag)
        .cmd_valid  (pd_valid)
    );

    // =========================================================================
    // torque_saturation: physical ±10 mNm clamp + saturation event counter
    // Provides: torque_cmd, sat_flag[2:0], sat_count, valid_out (→ ctrl_valid)
    // =========================================================================
    logic [2:0] sat_flag_3bit; // per-axis saturation bits from torque_saturation

    torque_saturation #(
        .SAT_LIMIT        (16'sh3FFF), // ±16383 ≈ ±10 mNm in Q15
        .ENABLE_SAT_COUNT (1'b1)       // enable saturation event counter
    ) u_sat (
        .clk             (clk),
        .rst_n           (rst_n),
        .torque_in       (pd_torque),
        .valid_in        (pd_valid),
        .sat_count_clear (sat_clear_r),
        .torque_out      (torque_cmd),
        .sat_flag        (sat_flag_3bit),
        .valid_out       (ctrl_valid),
        .sat_count       (sat_count)
    );

    // Collapse per-axis flags to a single any-axis saturation indicator (spec: saturation_flag)
    assign saturation_flag = |sat_flag_3bit;

endmodule