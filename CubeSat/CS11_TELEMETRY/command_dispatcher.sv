// =============================================================================
// Module: command_dispatcher (CS11 ground command AXI4-Lite router)
// Subsystem: CS11 - Telemetry Encoder
// Description: Routes decoded ground commands to target subsystems via
//              AXI4-Lite write transactions.
//
//   Command routing table (CS-TLM-008):
//     APID 0x0200 → CS6  PD Controller   (Kp, Kd gains)
//     APID 0x0201 → CS7  Actuators       (RW speed, MTQ duty)
//     APID 0x0202 → CS8  ADCS FSM        (mode transition, reset)
//     APID 0x0203 → CS9  Orbit           (TLE update, MET load)
//     APID 0x0204 → CS10 Laser           (enable/disable, gimbal)
//     APID 0x0205 → CS11 Telemetry       (baud rate, packet enable)
//     APID 0x0206 → CS12 System          (reset, power control)
//
//   AXI4-Lite register map:
//     Base address = subsystem_base[APID - 0x0200] (parameterised)
//     Write address = base + cmd_code × 4
//     Write data    = cmd_data[0:3] (first 4 bytes of command parameters)
//
// Provenance: CS11 Telemetry Encoder specification CS-TLM-008
// =============================================================================
`timescale 1ns/1ps

module command_dispatcher (
    input  logic        clk,
    input  logic        rst_n,

    // Command input (from command_decoder)
    input  logic        cmd_valid,
    input  logic [15:0] cmd_apid,
    input  logic [7:0]  cmd_code,
    input  logic [7:0]  cmd_data [0:15],
    input  logic [4:0]  cmd_data_len,

    // AXI4-Lite master write channel
    output logic [31:0] axi_awaddr,
    output logic        axi_awvalid,
    input  logic        axi_awready,
    output logic [31:0] axi_wdata,
    output logic        axi_wvalid,
    input  logic        axi_wready,
    input  logic [1:0]  axi_bresp,
    input  logic        axi_bvalid,
    output logic        axi_bready,

    // Status
    output logic        cmd_ack,   // command accepted and dispatched
    output logic        cmd_nak    // command rejected (APID out of range)
);

    // =========================================================================
    // Subsystem base addresses (one per APID offset 0x0200 + n)
    // =========================================================================
    logic [31:0] base_addr [0:6];

    always_comb begin
        base_addr[0] = 32'h0200_0000; // CS6  PD Controller
        base_addr[1] = 32'h0201_0000; // CS7  Actuators
        base_addr[2] = 32'h0202_0000; // CS8  ADCS FSM
        base_addr[3] = 32'h0203_0000; // CS9  Orbit
        base_addr[4] = 32'h0204_0000; // CS10 Laser
        base_addr[5] = 32'h0205_0000; // CS11 Telemetry
        base_addr[6] = 32'h0206_0000; // CS12 System
    end

    // =========================================================================
    // AXI4-Lite write FSM
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE    = 3'd0,
        S_ADDR    = 3'd1,   // drive AW channel
        S_DATA    = 3'd2,   // drive W channel
        S_RESP    = 3'd3,   // wait for B channel
        S_ACK     = 3'd4,   // pulse cmd_ack
        S_NAK     = 3'd5    // pulse cmd_nak
    } disp_state_t;

    disp_state_t state;

    logic [2:0]  apid_idx;   // index into base_addr (APID - 0x0200, clamped)
    logic        apid_valid; // APID is in valid range

    always_comb begin
        apid_valid = (cmd_apid >= 16'h0200) && (cmd_apid <= 16'h0206);
        apid_idx   = apid_valid ? cmd_apid[2:0] : 3'd0;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= S_IDLE;
            axi_awaddr <= '0;
            axi_awvalid <= 1'b0;
            axi_wdata  <= '0;
            axi_wvalid <= 1'b0;
            axi_bready <= 1'b0;
            cmd_ack    <= 1'b0;
            cmd_nak    <= 1'b0;
        end else begin
            cmd_ack <= 1'b0;
            cmd_nak <= 1'b0;

            case (state)
                // -----------------------------------------------------------------
                S_IDLE: begin
                    axi_awvalid <= 1'b0;
                    axi_wvalid  <= 1'b0;
                    axi_bready  <= 1'b0;
                    if (cmd_valid) begin
                        if (apid_valid) begin
                            // Compute AXI write address
                            axi_awaddr  <= base_addr[apid_idx] + {22'h0, cmd_code, 2'b00};
                            axi_wdata   <= {cmd_data[3], cmd_data[2],
                                            cmd_data[1], cmd_data[0]};
                            axi_awvalid <= 1'b1;
                            axi_wvalid  <= 1'b1;
                            state       <= S_ADDR;
                        end else begin
                            state <= S_NAK;
                        end
                    end
                end

                // -----------------------------------------------------------------
                S_ADDR: begin
                    if (axi_awready) begin
                        axi_awvalid <= 1'b0;
                        state       <= S_DATA;
                    end
                end

                // -----------------------------------------------------------------
                S_DATA: begin
                    if (axi_wready) begin
                        axi_wvalid <= 1'b0;
                        axi_bready <= 1'b1;
                        state      <= S_RESP;
                    end
                end

                // -----------------------------------------------------------------
                S_RESP: begin
                    if (axi_bvalid) begin
                        axi_bready <= 1'b0;
                        state      <= S_ACK;
                    end
                end

                // -----------------------------------------------------------------
                S_ACK: begin
                    cmd_ack <= 1'b1;
                    state   <= S_IDLE;
                end

                // -----------------------------------------------------------------
                S_NAK: begin
                    cmd_nak <= 1'b1;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule