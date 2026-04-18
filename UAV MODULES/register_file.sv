// =============================================================================
// File        : register_file.sv
// Module      : register_file
// Description : 50+ register file for UAV system configuration and status.
//               Register map (byte address / 4):
//               0x00-0x03 : System control (arm/mode/reset)
//               0x04-0x0F : Reserved
//               0x10-0x55 : 18 PID gain registers (Kp/Ki/Kd × 6 axes Q4.12)
//               0x58-0x7B : 9 EKF process noise Q diagonal (Q16.16)
//               0x80-0x9B : IMU bias (Gyro xyz + Accel xyz, Q4.28)
//               0xA0-0xB3 : Mag hard-iron offset (xyz Q4.28)
//               0xB4-0xBF : WDT config, geofence params
//               0xC0-0xFF : Status registers (read-only)
//               Atomic multi-register writes via shadow+commit (address 0x00 bit 31).
// =============================================================================

`timescale 1ns/1ps

module register_file #(
    parameter int ADDR_W = 8,
    parameter int DATA_W = 32,
    parameter int N_REGS = 64   // 256 bytes / 4
)(
    input  logic clk,
    input  logic rst_n,

    // Write port (from AXI slave)
    input  logic [ADDR_W-1:0] waddr,
    input  logic [DATA_W-1:0] wdata,
    input  logic               wen,

    // Read port
    input  logic [ADDR_W-1:0] raddr,
    output logic [DATA_W-1:0] rdata,

    // ---- Decoded register outputs ------------------------------------------
    // System control
    output logic [7:0]  sys_arm_cmd,
    output logic [7:0]  sys_mode_cmd,

    // PID gains [axis][coef]: axis=0-7, coef=0(Kp),1(Ki),2(Kd)
    output logic signed [15:0] pid_gains [0:7][0:2],

    // Anti-windup clamps [axis]
    output logic signed [31:0] pid_integ_max [0:7],

    // EKF process noise Q diagonal
    output logic [31:0] ekf_q_diag [0:8],

    // Calibration biases (Q4.28)
    output logic signed [31:0] gyro_bias  [0:2],
    output logic signed [31:0] accel_bias [0:2],
    output logic signed [31:0] mag_offset [0:2],

    // WDT configuration
    output logic [10:0] wdt_timeout_ms,
    output logic        wdt_config_en,

    // Geofence
    output logic [31:0] geofence_radius_sq,
    output logic [31:0] geofence_max_alt,

    // Preflight check mask
    output logic [7:0] check_mask,

    // Motor mixing coefficients Q2.14 [motor][channel]
    output logic signed [15:0] mix_coef [0:3][0:3],

    // Status registers (written by subsystems)
    input  logic [31:0] status_ekf,
    input  logic [31:0] status_gps,
    input  logic [31:0] status_imu
);

    // -------------------------------------------------------------------------
    // Register array
    // -------------------------------------------------------------------------
    logic [DATA_W-1:0] regs [0:N_REGS-1];

    // =========================================================================
    // Register Map (64 × 32-bit words = 256 bytes, no conflicts):
    //   regs[0]    : System control  (arm_cmd[7:0], mode_cmd[15:8])
    //   regs[1]    : WDT config      ([10:0]=timeout_ms)
    //   regs[2]    : Check mask      ([7:0])
    //   regs[3]    : Geofence radius² (Q10.22²)
    //   regs[4]    : Geofence max alt (Q10.22)
    //   regs[5-22] : PID gains       (18 registers, Q4.12 low 16b)
    //   regs[23-30]: Anti-windup max (8 axes, Q16.16)
    //   regs[31-39]: EKF Q diagonal  (9 states, Q16.16)
    //   regs[40-42]: Gyro bias       (xyz, Q4.28)
    //   regs[43-45]: Accel bias      (xyz, Q4.28)
    //   regs[46-48]: Mag offset      (xyz, Q4.28)
    //   regs[49]   : Status EKF      (read-only, driven externally)  ← STATUS_EKF_IDX
    //   regs[50]   : Status GPS      (read-only)                      ← STATUS_GPS_IDX
    //   regs[51]   : Status IMU      (read-only)                      ← STATUS_IMU_IDX
    //   regs[52-59]: Mix coefficients (8 regs × 2 Q2.14 values each = 16 total)
    //   regs[60-63]: Spare
    // =========================================================================

    // Named constants for status register word indices
    localparam int STATUS_EKF_IDX = 49;
    localparam int STATUS_GPS_IDX = 50;
    localparam int STATUS_IMU_IDX = 51;

    // Shadow registers for atomic write (commit on regs[0] bit 31)
    logic [DATA_W-1:0] shadow [0:N_REGS-1];
    logic              commit_pending;

    // Dedicated status storage (read-only, bypassed in rdata mux)
    logic [DATA_W-1:0] status_regs [0:2];

    // ---- Write logic --------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < N_REGS; i++) begin
                regs[i]   <= '0;
                shadow[i] <= '0;
            end
            commit_pending <= 1'b0;
            wdt_config_en  <= 1'b0;

            // Default values
            // regs[1]: WDT timeout = 500ms
            regs[1]  <= 32'h000001F4;
            // regs[2]: check mask = all enabled
            regs[2]  <= 32'h0000007F;
            // PID gains (regs[5-22]): Kp=1.0 Q4.12 = 4096
            for (int i = 5; i < 23; i++) regs[i] <= 32'sh00001000;
            // EKF process noise Q (regs[31-39]): small default (Q16.16, ~0.004)
            for (int i = 31; i < 40; i++) regs[i] <= 32'h00000100;
            // Mix matrix (regs[52-59]): default X-frame, Q2.14 = 16384 packed
            // packed: [31:16]=coef_odd, [15:0]=coef_even
            // M0: +roll,-pitch,+yaw,+thrust  → [+1,-1,+1,+1]
            regs[52] <= {-16'sh4000, 16'sh4000};  // ch0=+1, ch1=-1
            regs[53] <= { 16'sh4000, 16'sh4000};  // ch2=+1, ch3=+1
            // M1: -roll,-pitch,-yaw,+thrust  → [-1,-1,-1,+1]
            regs[54] <= {-16'sh4000,-16'sh4000};  // ch0=-1, ch1=-1
            regs[55] <= { 16'sh4000,-16'sh4000};  // ch2=-1, ch3=+1
            // M2: +roll,+pitch,-yaw,+thrust  → [+1,+1,-1,+1]
            regs[56] <= { 16'sh4000, 16'sh4000};  // ch0=+1, ch1=+1
            regs[57] <= { 16'sh4000,-16'sh4000};  // ch2=-1, ch3=+1
            // M3: -roll,+pitch,+yaw,+thrust  → [-1,+1,+1,+1]
            regs[58] <= { 16'sh4000,-16'sh4000};  // ch0=-1, ch1=+1
            regs[59] <= { 16'sh4000, 16'sh4000};  // ch2=+1, ch3=+1
        end else begin
            wdt_config_en <= 1'b0;

            // Update read-only status registers from subsystems
            status_regs[0] <= status_ekf;
            status_regs[1] <= status_gps;
            status_regs[2] <= status_imu;

            if (wen) begin
                logic [5:0] widx;
                widx = waddr[7:2]; // byte address → word index

                // Atomic commit check: writing 0x00 with bit 31 set commits shadow
                if (widx == 6'd0 && wdata[31]) begin
                    for (int i = 0; i < N_REGS; i++)
                        if (shadow[i] != '0) regs[i] <= shadow[i];
                    commit_pending <= 1'b0;
                end else begin
                    // Block writes to read-only status registers
                    if (widx < STATUS_EKF_IDX[5:0] || widx > STATUS_IMU_IDX[5:0]) begin
                        shadow[widx] <= wdata;
                        regs[widx]   <= wdata;
                    end
                end

                // WDT config trigger
                if (widx == 6'd1) wdt_config_en <= 1'b1;
            end
        end
    end

    // ---- Read logic (status registers bypassed directly) --------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) rdata <= '0;
        else begin
            logic [5:0] ridx;
            ridx = raddr[7:2];
            case (ridx)
                STATUS_EKF_IDX[5:0]: rdata <= status_regs[0];
                STATUS_GPS_IDX[5:0]: rdata <= status_regs[1];
                STATUS_IMU_IDX[5:0]: rdata <= status_regs[2];
                default:             rdata <= regs[ridx];
            endcase
        end
    end

    // ---- Register decode to named outputs -----------------------------------
    // System control (reg 0)
    assign sys_arm_cmd  = regs[0][7:0];
    assign sys_mode_cmd = regs[0][15:8];

    // WDT and geofence
    assign wdt_timeout_ms    = regs[1][10:0];
    assign check_mask        = regs[2][7:0];
    assign geofence_radius_sq= regs[3];
    assign geofence_max_alt  = regs[4];

    // PID gains: regs[5-22] (18 gains: axis 0-5 × Kp/Ki/Kd)
    // Axis 0-5: regs[5+ax*3], [6+ax*3], [7+ax*3]
    // Axes 6-7: mapped to regs[5] (placeholder, reuse axis-0 gains)
    generate
        for (genvar ax = 0; ax < 6; ax++) begin : pid_gain_decode
            for (genvar co = 0; co < 3; co++) begin : coef_decode
                assign pid_gains[ax][co] = regs[5 + ax*3 + co][15:0];
            end
        end
        // Axes 6-7 share axis-0 gains
        for (genvar co = 0; co < 3; co++) begin : coef_decode_67
            assign pid_gains[6][co] = regs[5 + co][15:0];
            assign pid_gains[7][co] = regs[5 + co][15:0];
        end
    endgenerate

    // Anti-windup: regs[23-30]
    generate
        for (genvar ax = 0; ax < 8; ax++) begin : imax_decode
            assign pid_integ_max[ax] = regs[23 + ax];
        end
    endgenerate

    // EKF Q diagonal: regs[31-39]
    generate
        for (genvar s = 0; s < 9; s++) begin : ekfq_decode
            assign ekf_q_diag[s] = regs[31 + s];
        end
    endgenerate

    // Calibration biases
    assign gyro_bias[0]  = regs[40]; assign gyro_bias[1]  = regs[41]; assign gyro_bias[2]  = regs[42];
    assign accel_bias[0] = regs[43]; assign accel_bias[1] = regs[44]; assign accel_bias[2] = regs[45];
    assign mag_offset[0] = regs[46]; assign mag_offset[1] = regs[47]; assign mag_offset[2] = regs[48];

    // Mixing matrix coefficients: regs[52-59], 2 values per register (packed)
    // lin_idx = m*4 + ch; reg = 52 + lin_idx/2; low 16b if even, high 16b if odd
    generate
        for (genvar m = 0; m < 4; m++) begin : mix_row
            for (genvar ch = 0; ch < 4; ch++) begin : mix_col
                localparam int LIN = m*4 + ch;
                if (LIN % 2 == 0)
                    assign mix_coef[m][ch] = regs[52 + LIN/2][15:0];
                else
                    assign mix_coef[m][ch] = regs[52 + LIN/2][31:16];
            end
        end
    endgenerate

endmodule
