// =============================================================================
// Module: tlm_arbiter (CS11 telemetry superframe scheduler)
// Subsystem: CS11 - Telemetry Encoder
// Description: 1-second superframe scheduler for fixed-slot telemetry.
//
//   Superframe schedule (1000 ms total, CS-TLM-006):
//     0 ms   : HK    packet (18 B) - APID 0x0100
//     100 ms : ADCS  packet (44 B) - APID 0x0101
//     200 ms : Orbit packet (47 B) - APID 0x0102
//     300 ms : Laser packet (20 B) - APID 0x0103
//     400-999 ms : Idle (or retransmit overflow)
//
//   On ce_1hz  : frame_time_ms resets to 0; slot_active resets to 0.
//   On ce_1ms  : frame_time_ms increments by 1.
//   At each slot boundary: packet_ready pulses for one clock if the
//     corresponding *_data_valid input is asserted; packet_select indicates
//     which channel to transmit.
//
// Provenance: CS11 Telemetry Encoder specification CS-TLM-006
// =============================================================================
`timescale 1ns/1ps

module tlm_arbiter (
    input  logic        clk,
    input  logic        rst_n,

    // Clock enables
    input  logic        ce_1ms,           // 1 kHz clock enable (1 ms tick)
    input  logic        ce_1hz,           // 1 Hz clock enable  (superframe start)

    // Source readiness
    input  logic        hk_data_valid,    // HK packet ready
    input  logic        adcs_data_valid,  // ADCS packet ready
    input  logic        orbit_data_valid, // Orbit packet ready
    input  logic        laser_data_valid, // Laser packet ready

    // Outputs
    output logic [3:0]  slot_active,      // current slot ID (0-3)
    output logic [1:0]  packet_select,    // mux select: 0=HK,1=ADCS,2=Orbit,3=Laser
    output logic        packet_ready,     // pulse when selected packet should transmit
    output logic [9:0]  frame_time_ms     // elapsed ms since superframe start (0-999)
);

    // Slot boundary times in milliseconds
    localparam int HK_SLOT_MS    = 0;
    localparam int ADCS_SLOT_MS  = 100;
    localparam int ORBIT_SLOT_MS = 200;
    localparam int LASER_SLOT_MS = 300;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_time_ms <= '0;
            slot_active   <= 4'd0;
            packet_select <= 2'd0;
            packet_ready  <= 1'b0;
        end else begin
            packet_ready <= 1'b0;  // default: de-assert

            if (ce_1hz) begin
                // New superframe: reset timer and slot
                frame_time_ms <= '0;
                slot_active   <= 4'd0;
                // Trigger HK at slot 0 if ready
                if (hk_data_valid) begin
                    packet_select <= 2'd0;
                    packet_ready  <= 1'b1;
                    slot_active   <= 4'd0;
                end
            end else if (ce_1ms) begin
                // Advance millisecond counter (wraps at 999)
                if (frame_time_ms == 10'd999)
                    frame_time_ms <= '0;
                else
                    frame_time_ms <= frame_time_ms + 10'd1;

                // Trigger slot transitions
                case (frame_time_ms)
                    10'(ADCS_SLOT_MS): begin
                        slot_active   <= 4'd1;
                        if (adcs_data_valid) begin
                            packet_select <= 2'd1;
                            packet_ready  <= 1'b1;
                        end
                    end
                    10'(ORBIT_SLOT_MS): begin
                        slot_active   <= 4'd2;
                        if (orbit_data_valid) begin
                            packet_select <= 2'd2;
                            packet_ready  <= 1'b1;
                        end
                    end
                    10'(LASER_SLOT_MS): begin
                        slot_active   <= 4'd3;
                        if (laser_data_valid) begin
                            packet_select <= 2'd3;
                            packet_ready  <= 1'b1;
                        end
                    end
                    default: ;
                endcase
            end
        end
    end

endmodule