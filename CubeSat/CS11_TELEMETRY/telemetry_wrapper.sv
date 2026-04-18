// =============================================================================
// Module: telemetry_wrapper (CS11 top-level telemetry wrapper - v2)
// Subsystem: CS11 - Telemetry Encoder
// Description: Integrates tlm_arbiter (superframe scheduling), tlm_frame_builder
//              (CCSDS-aligned variable-length framing), command_decoder (UART RX
//              command reception), and command_dispatcher (AXI4-Lite routing).
//
//   Downlink path:
//     TLM byte arrays → tlm_arbiter → payload mux → tlm_frame_builder
//                                                  → 8N1 UART TX
//
//   Uplink path:
//     UART RX → command_decoder → command_dispatcher → AXI4-Lite write
//
//   Frame format (new, CS-TLM-001):
//     SYNC(4) | APID(2) | SEQ(2) | LEN(2) | TS(4) | PAYLOAD(N) | CRC(2)
//     Total: 16 + N bytes
//     HK=34B, ADCS=60B, Orbit=63B, Laser=36B
//
//   Baud rate: BAUD_HZ = 115200 (default)
//   CLK_HZ   = 100_000_000
//   Bit period = CLK_HZ / BAUD_HZ = 868 cycles
//
//   Superframe schedule (CS-TLM-006):
//     0   ms → HK    (APID 0x0100, 18 B payload)
//     100 ms → ADCS  (APID 0x0101, 44 B payload)
//     200 ms → Orbit (APID 0x0102, 47 B payload)
//     300 ms → Laser (APID 0x0103, 20 B payload)
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module telemetry_wrapper #(
    parameter int CLK_HZ  = 100_000_000,
    parameter int BAUD_HZ = 115_200
)(
    input  logic        clk,
    input  logic        rst_n,

    // Clock enables
    input  logic        ce_1hz,    // 1 Hz superframe trigger
    input  logic        ce_1ms,    // 1 kHz arbiter tick

    // Telemetry source inputs (pre-packed byte arrays)
    input  logic [7:0]  adcs_tlm  [0:43],   // 44-byte ADCS packet (CS-TLM-003)
    input  logic        adcs_valid,
    input  logic [7:0]  orbit_tlm [0:46],   // 47-byte Orbit packet (CS-TLM-004)
    input  logic        orbit_valid,
    input  logic [7:0]  laser_tlm [0:19],   // 20-byte Laser packet (CS-TLM-005)
    input  logic        laser_valid,
    input  logic [7:0]  hk_tlm    [0:17],   // 18-byte HK packet (CS-TLM-007)
    input  logic        hk_valid,

    // Timestamp (mission elapsed time in seconds, for frame header)
    input  logic [31:0] uptime_sec,

    // UART downlink TX
    output logic        uart_tx,

    // UART uplink RX (command reception, CS-TLM-008)
    input  logic        uart_rx,

    // Decoded command outputs (from command_decoder)
    output logic        cmd_valid,
    output logic [15:0] cmd_apid,
    output logic [7:0]  cmd_code,
    output logic [7:0]  cmd_data [0:15],
    output logic [4:0]  cmd_data_len,
    output logic        cmd_crc_error,
    output logic        cmd_sync_error,

    // AXI4-Lite master (from command_dispatcher)
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
    output logic        tlm_valid,   // pulses on frame_done
    output logic        tlm_busy     // asserted while frame is being transmitted
);

    // =========================================================================
    // Sequence counter (incremented per transmitted frame)
    // =========================================================================
    logic [15:0] seq_cnt;

    // =========================================================================
    // Superframe arbiter
    // =========================================================================
    logic [3:0]  arb_slot_active;
    logic [1:0]  arb_packet_select;
    logic        arb_packet_ready;
    logic [9:0]  arb_frame_time_ms;

    tlm_arbiter u_arbiter (
        .clk             (clk),
        .rst_n           (rst_n),
        .ce_1ms          (ce_1ms),
        .ce_1hz          (ce_1hz),
        .hk_data_valid   (hk_valid),
        .adcs_data_valid (adcs_valid),
        .orbit_data_valid(orbit_valid),
        .laser_data_valid(laser_valid),
        .slot_active     (arb_slot_active),
        .packet_select   (arb_packet_select),
        .packet_ready    (arb_packet_ready),
        .frame_time_ms   (arb_frame_time_ms)
    );

    // =========================================================================
    // Payload mux: select byte array and metadata based on arbiter output
    // =========================================================================
    localparam int MAX_PAYLOAD = 128;

    logic [7:0]  mux_payload [0:MAX_PAYLOAD-1];
    logic [7:0]  mux_payload_len;
    logic [15:0] mux_apid;
    logic        mux_frame_start;

    // Payload mux (combinational)
    always_comb begin
        for (int i = 0; i < MAX_PAYLOAD; i++) mux_payload[i] = 8'h00;
        mux_payload_len = 8'd18;
        mux_apid        = 16'h0100;

        case (arb_packet_select)
            2'd0: begin  // HK
                for (int i = 0; i < 18; i++) mux_payload[i] = hk_tlm[i];
                mux_payload_len = 8'd18;
                mux_apid        = 16'h0100;
            end
            2'd1: begin  // ADCS
                for (int i = 0; i < 44; i++) mux_payload[i] = adcs_tlm[i];
                mux_payload_len = 8'd44;
                mux_apid        = 16'h0101;
            end
            2'd2: begin  // Orbit
                for (int i = 0; i < 47; i++) mux_payload[i] = orbit_tlm[i];
                mux_payload_len = 8'd47;
                mux_apid        = 16'h0102;
            end
            2'd3: begin  // Laser
                for (int i = 0; i < 20; i++) mux_payload[i] = laser_tlm[i];
                mux_payload_len = 8'd20;
                mux_apid        = 16'h0103;
            end
            default: ;
        endcase
    end

    // =========================================================================
    // Gate frame_start: only start when arbiter fires and frame_builder is idle
    // =========================================================================
    logic fb_busy;
    logic fb_frame_done;

    // Sequence counter update on each completed frame
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            seq_cnt <= '0;
        else if (fb_frame_done)
            seq_cnt <= seq_cnt + 16'd1;
    end

    assign mux_frame_start = arb_packet_ready & ~fb_busy;

    // =========================================================================
    // Frame builder
    // =========================================================================
    logic [7:0] fb_byte;
    logic       fb_valid;
    logic       fb_ready;
    logic [9:0] fb_frame_len;

    tlm_frame_builder #(
        .MAX_PAYLOAD (MAX_PAYLOAD)
    ) u_frame_builder (
        .clk             (clk),
        .rst_n           (rst_n),
        .payload         (mux_payload),
        .payload_len     (mux_payload_len),
        .apid            (mux_apid),
        .seq             (seq_cnt),
        .timestamp       (uptime_sec),
        .frame_start     (mux_frame_start),
        .frame_out       (fb_byte),
        .frame_out_valid (fb_valid),
        .frame_out_ready (fb_ready),
        .frame_out_len   (fb_frame_len),
        .frame_done      (fb_frame_done),
        .frame_busy      (fb_busy)
    );

    // =========================================================================
    // 8N1 UART serialiser (BAUD_HZ, same logic as original telemetry_wrapper)
    // =========================================================================
    localparam int BIT_PERIOD = CLK_HZ / BAUD_HZ;
    localparam int BIT_CNT_W  = $clog2(BIT_PERIOD);

    typedef enum logic [1:0] {
        UART_IDLE  = 2'd0,
        UART_START = 2'd1,
        UART_DATA  = 2'd2,
        UART_STOP  = 2'd3
    } uart_state_t;

    uart_state_t uart_st;

    logic [BIT_CNT_W-1:0] baud_cnt;
    logic [7:0]  shift_reg;
    logic [3:0]  bit_cnt;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            uart_st   <= UART_IDLE;
            baud_cnt  <= '0;
            shift_reg <= 8'hFF;
            bit_cnt   <= '0;
            uart_tx   <= 1'b1;
            fb_ready  <= 1'b1;
        end else begin
            fb_ready <= 1'b0;

            case (uart_st)
                UART_IDLE: begin
                    uart_tx  <= 1'b1;
                    fb_ready <= 1'b1;
                    if (fb_valid) begin
                        shift_reg <= fb_byte;
                        baud_cnt  <= '0;
                        bit_cnt   <= '0;
                        uart_tx   <= 1'b0;   // start bit
                        fb_ready  <= 1'b0;
                        uart_st   <= UART_START;
                    end
                end

                UART_START: begin
                    if (baud_cnt == BIT_PERIOD[BIT_CNT_W-1:0] - 1) begin
                        baud_cnt <= '0;
                        uart_tx  <= shift_reg[0];
                        uart_st  <= UART_DATA;
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end

                UART_DATA: begin
                    if (baud_cnt == BIT_PERIOD[BIT_CNT_W-1:0] - 1) begin
                        baud_cnt  <= '0;
                        shift_reg <= {1'b1, shift_reg[7:1]};
                        if (bit_cnt == 4'd7) begin
                            uart_tx <= 1'b1;  // stop bit
                            uart_st <= UART_STOP;
                        end else begin
                            bit_cnt <= bit_cnt + 1;
                            uart_tx <= shift_reg[1];
                        end
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end

                UART_STOP: begin
                    if (baud_cnt == BIT_PERIOD[BIT_CNT_W-1:0] - 1) begin
                        baud_cnt <= '0;
                        fb_ready <= 1'b1;
                        uart_st  <= UART_IDLE;
                    end else begin
                        baud_cnt <= baud_cnt + 1;
                    end
                end

                default: uart_st <= UART_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Command decoder (UART RX path)
    // =========================================================================
    command_decoder #(
        .CLK_HZ  (CLK_HZ),
        .BAUD_HZ (BAUD_HZ)
    ) u_cmd_dec (
        .clk           (clk),
        .rst_n         (rst_n),
        .uart_rx       (uart_rx),
        .cmd_valid     (cmd_valid),
        .cmd_apid      (cmd_apid),
        .cmd_code      (cmd_code),
        .cmd_data      (cmd_data),
        .cmd_data_len  (cmd_data_len),
        .cmd_crc_error (cmd_crc_error),
        .cmd_sync_error(cmd_sync_error)
    );

    // =========================================================================
    // Command dispatcher (AXI4-Lite routing)
    // =========================================================================
    command_dispatcher u_cmd_disp (
        .clk         (clk),
        .rst_n       (rst_n),
        .cmd_valid   (cmd_valid),
        .cmd_apid    (cmd_apid),
        .cmd_code    (cmd_code),
        .cmd_data    (cmd_data),
        .cmd_data_len(cmd_data_len),
        .axi_awaddr  (axi_awaddr),
        .axi_awvalid (axi_awvalid),
        .axi_awready (axi_awready),
        .axi_wdata   (axi_wdata),
        .axi_wvalid  (axi_wvalid),
        .axi_wready  (axi_wready),
        .axi_bresp   (axi_bresp),
        .axi_bvalid  (axi_bvalid),
        .axi_bready  (axi_bready),
        .cmd_ack     (),
        .cmd_nak     ()
    );

    // =========================================================================
    // Status outputs
    // =========================================================================
    always_comb begin
        tlm_valid = fb_frame_done;
        tlm_busy  = fb_busy;
    end

endmodule