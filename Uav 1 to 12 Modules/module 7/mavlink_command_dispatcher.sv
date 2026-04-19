// =============================================================================
// File        : mavlink_command_dispatcher.sv
// Module      : mavlink_command_dispatcher
// Description : MAVLink command dispatcher and HEARTBEAT monitor.
//               Decodes received frames and routes:
//                 MSG_ID=0   (HEARTBEAT): GCS presence detection
//                 CMD: ARM/DISARM, TAKEOFF, LAND, RTL, WAYPOINT, SET_MODE, REBOOT
//               Monitors HEARTBEAT at 1 Hz; asserts gcs_lost if absent for 3s.
// =============================================================================

`timescale 1ns/1ps

module mavlink_command_dispatcher #(
    parameter int CLK_HZ       = 50_000_000,
    parameter int MAX_PAYLOAD  = 255,
    // HEARTBEAT timeout: 3 seconds
    parameter int HB_TIMEOUT   = 3 * CLK_HZ
)(
    input  logic clk,
    input  logic rst_n,

    // From frame parser
    input  logic [23:0] msg_id,
    input  logic [7:0]  payload [0:MAX_PAYLOAD-1],
    input  logic [7:0]  payload_len,
    input  logic        frame_valid,

    // Command outputs (to MOD_8 FSM)
    output logic cmd_arm,
    output logic cmd_disarm,
    output logic cmd_takeoff,
    output logic cmd_land,
    output logic cmd_rtl,
    output logic cmd_waypoint,
    output logic cmd_set_mode,
    output logic cmd_reboot,
    output logic [7:0] cmd_mode_val,

    // GCS status
    output logic gcs_present,
    output logic gcs_lost
);

    // MAVLink message IDs
    localparam logic [23:0] MSGID_HEARTBEAT      = 24'd0;
    localparam logic [23:0] MSGID_CMD_LONG       = 24'd76;
    localparam logic [23:0] MSGID_MISSION_ITEM   = 24'd39;
    localparam logic [23:0] MSGID_SET_MODE       = 24'd11;

    // MAVLink commands
    localparam logic [15:0] MAV_CMD_COMPONENT_ARM_DISARM = 16'd400;
    localparam logic [15:0] MAV_CMD_NAV_TAKEOFF          = 16'd22;
    localparam logic [15:0] MAV_CMD_NAV_LAND             = 16'd21;
    localparam logic [15:0] MAV_CMD_NAV_RETURN_TO_LAUNCH = 16'd20;
    localparam logic [15:0] MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN = 16'd246;

    // -------------------------------------------------------------------------
    // HEARTBEAT watchdog
    // -------------------------------------------------------------------------
    logic [$clog2(HB_TIMEOUT+1)-1:0] hb_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hb_cnt      <= '0;
            gcs_present <= 1'b0;
            gcs_lost    <= 1'b0;
        end else begin
            if (frame_valid && msg_id == MSGID_HEARTBEAT) begin
                hb_cnt      <= '0;
                gcs_present <= 1'b1;
                gcs_lost    <= 1'b0;
            end else begin
                if (hb_cnt < HB_TIMEOUT) begin
                    hb_cnt <= hb_cnt + 1'b1;
                end else begin
                    gcs_present <= 1'b0;
                    gcs_lost    <= 1'b1;
                end
            end
        end
    end

    // -------------------------------------------------------------------------
    // Command decoder
    // -------------------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_arm      <= 1'b0; cmd_disarm   <= 1'b0;
            cmd_takeoff  <= 1'b0; cmd_land     <= 1'b0;
            cmd_rtl      <= 1'b0; cmd_waypoint <= 1'b0;
            cmd_set_mode <= 1'b0; cmd_reboot   <= 1'b0;
            cmd_mode_val <= 8'h00;
        end else begin
            // Clear pulse outputs each cycle
            cmd_arm      <= 1'b0; cmd_disarm   <= 1'b0;
            cmd_takeoff  <= 1'b0; cmd_land     <= 1'b0;
            cmd_rtl      <= 1'b0; cmd_waypoint <= 1'b0;
            cmd_set_mode <= 1'b0; cmd_reboot   <= 1'b0;

            if (frame_valid) begin
                case (msg_id)
                    MSGID_CMD_LONG: begin
                        // Payload[0:1] = command ID (little-endian)
                        logic [15:0] cmd_id;
                        cmd_id = {payload[1], payload[0]};
                        // payload[28] = param1 (arm/disarm: 1=arm, 0=disarm)
                        case (cmd_id)
                            MAV_CMD_COMPONENT_ARM_DISARM: begin
                                if (payload[28] == 8'h01) cmd_arm    <= 1'b1;
                                else                      cmd_disarm <= 1'b1;
                            end
                            MAV_CMD_NAV_TAKEOFF:          cmd_takeoff <= 1'b1;
                            MAV_CMD_NAV_LAND:             cmd_land    <= 1'b1;
                            MAV_CMD_NAV_RETURN_TO_LAUNCH: cmd_rtl     <= 1'b1;
                            MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN: cmd_reboot <= 1'b1;
                            default: ;
                        endcase
                    end
                    MSGID_MISSION_ITEM: cmd_waypoint <= 1'b1;
                    MSGID_SET_MODE: begin
                        cmd_set_mode <= 1'b1;
                        cmd_mode_val <= payload[4]; // custom_mode byte
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule
