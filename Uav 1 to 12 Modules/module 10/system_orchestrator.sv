// =============================================================================
// File        : system_orchestrator.sv
// Module      : system_orchestrator
// Description : System-level orchestrator for UAV MOD_10.
//               Coordinates register updates, mode changes and subsystem
//               resets from AXI register writes.
//               Handles flight mode preset BRAM pages (4 modes × 18 gains).
// =============================================================================

`timescale 1ns/1ps

module system_orchestrator #(
    parameter int DATA_W = 32
)(
    input  logic clk,
    input  logic rst_n,

    // From register file
    input  logic [7:0] sys_arm_cmd,
    input  logic [7:0] sys_mode_cmd,

    // From FSM
    input  logic [7:0] flight_mode,
    input  logic       armed,

    // Flight mode preset BRAM interface
    // 4 pages × 18 gain words × 32 bits = 72 words
    // Page select uses sys_mode_cmd[1:0]
    output logic [5:0]  preset_addr,  // 0-17 = gain index within page
    output logic [1:0]  preset_page,  // 0-3 = flight mode page
    output logic        preset_rd_en,

    // Override PID gains from preset BRAM
    input  logic [DATA_W-1:0] preset_gain_data,  // from BRAM read
    output logic              apply_preset,       // pulse: use preset_gain_data

    // Subsystem soft-resets (active-high pulse)
    output logic rst_ekf,
    output logic rst_pid,
    output logic rst_nav,
    output logic rst_mavlink,

    // Status LED encoding (to top-level)
    output logic [3:0] led_status
);

    // -------------------------------------------------------------------------
    // Flight mode preset BRAM (inferred dual-port, 4 pages × 18 words)
    // -------------------------------------------------------------------------
    localparam int PRESET_DEPTH = 4 * 18; // 72 entries

    logic [DATA_W-1:0] preset_bram [0:PRESET_DEPTH-1];
    logic [6:0]        bram_raddr;
    logic [DATA_W-1:0] bram_rdata;

    // Initialise presets with safe defaults (all gains = 1.0 in Q4.12 = 4096)
    initial begin
        for (int i = 0; i < PRESET_DEPTH; i++)
            preset_bram[i] = 32'sh00001000; // Kp=1.0, Ki=Ki, Kd=0
    end

    always_ff @(posedge clk) begin
        if (preset_rd_en)
            bram_rdata <= preset_bram[bram_raddr];
    end

    assign preset_gain_data = bram_rdata;
    assign bram_raddr       = {preset_page, preset_addr};

    // -------------------------------------------------------------------------
    // Mode change detection → trigger preset load
    // -------------------------------------------------------------------------
    logic [7:0] mode_prev;
    logic       mode_changed;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mode_prev    <= 8'hFF;
            mode_changed <= 1'b0;
        end else begin
            mode_changed <= (flight_mode != mode_prev);
            mode_prev    <= flight_mode;
        end
    end

    // -------------------------------------------------------------------------
    // Preset load state machine
    // -------------------------------------------------------------------------
    typedef enum logic [1:0] { PST_IDLE, PST_READ, PST_APPLY } pst_state_t;
    pst_state_t pst_st;
    logic [4:0] gain_idx;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pst_st       <= PST_IDLE;
            preset_rd_en <= 1'b0;
            apply_preset <= 1'b0;
            preset_addr  <= '0;
            preset_page  <= '0;
            gain_idx     <= '0;
        end else begin
            preset_rd_en <= 1'b0;
            apply_preset <= 1'b0;

            case (pst_st)
                PST_IDLE: begin
                    if (mode_changed) begin
                        preset_page  <= sys_mode_cmd[1:0];
                        gain_idx     <= 5'd0;
                        pst_st       <= PST_READ;
                    end
                end
                PST_READ: begin
                    preset_addr  <= gain_idx;
                    preset_rd_en <= 1'b1;
                    pst_st       <= PST_APPLY;
                end
                PST_APPLY: begin
                    apply_preset <= 1'b1;
                    gain_idx     <= gain_idx + 1'b1;
                    if (gain_idx == 5'd17) pst_st <= PST_IDLE;
                    else                   pst_st <= PST_READ;
                end
                default: pst_st <= PST_IDLE;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // Subsystem resets (from AXI write to sys_arm_cmd bit fields)
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rst_ekf     <= 1'b0;
            rst_pid     <= 1'b0;
            rst_nav     <= 1'b0;
            rst_mavlink <= 1'b0;
        end else begin
            rst_ekf     <= sys_arm_cmd[4];
            rst_pid     <= sys_arm_cmd[5];
            rst_nav     <= sys_arm_cmd[6];
            rst_mavlink <= sys_arm_cmd[7];
        end
    end

    // -------------------------------------------------------------------------
    // LED status encoding
    // -------------------------------------------------------------------------
    always_comb begin
        case (flight_mode)
            8'd0:    led_status = 4'b0001; // DISARMED: LED 0 slow blink
            8'd1:    led_status = 4'b0011; // ARMED: LEDs 0-1
            8'd2:    led_status = 4'b0111; // TAKEOFF: LEDs 0-2
            8'd3:    led_status = 4'b1111; // EN_ROUTE: all LEDs
            8'd7:    led_status = 4'b1010; // EMERGENCY: alternating
            default: led_status = 4'b0101;
        endcase
    end

endmodule