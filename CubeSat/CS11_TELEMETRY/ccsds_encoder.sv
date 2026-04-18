// =============================================================================
// Module: ccsds_encoder (CS11 CCSDS telemetry frame encoder)
// Subsystem: CS11 - Telemetry Encoder
// Description: Assembles a CCSDS-compatible Virtual Channel Data Unit (VCDU)
//              frame from a 32-byte payload and outputs it as a byte stream.
//
//   Frame structure (total 48 bytes):
//     [0:3]   Sync word       0x1A CF FC 1D  (CCSDS ASM)
//     [4:7]   VCDU header     version|SCID|VCID|counter[7:0]
//     [8:39]  Data field      32-byte payload
//     [40:41] CRC-16 CCITT    (x^16+x^12+x^5+1 over bytes [0:39])
//     [42:47] Fill            0x00
//
//   Byte-serial output via uart_tx_byte / uart_tx_valid handshake.
//   Downstream UART serialiser samples uart_tx_byte when uart_tx_ready & enc_busy.
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module ccsds_encoder #(
    parameter logic [15:0] SPACECRAFT_ID = 16'h0001,
    parameter logic [5:0]  VCID          = 6'd0,
    parameter int          FRAME_LEN     = 48           // total frame bytes
)(
    input  logic        clk,
    input  logic        rst_n,

    // Payload input (32 bytes)
    input  logic [7:0]  payload [0:31],
    input  logic        payload_valid,   // strobe: start encoding

    // UART byte-stream output
    output logic [7:0]  uart_tx_byte,
    output logic        uart_tx_valid,
    input  logic        uart_tx_ready,   // handshake from UART TX module

    output logic        frame_done,
    output logic        enc_busy
);

    // =========================================================================
    // Frame buffer (48 bytes)
    // =========================================================================
    logic [7:0] frame_buf [0:FRAME_LEN-1];

    // =========================================================================
    // CRC-16 CCITT (poly 0x1021, init 0xFFFF)
    // =========================================================================
    function automatic logic [15:0] crc16_byte(input logic [15:0] crc_in,
                                                input logic [7:0]  data_in);
        logic [15:0] crc;
        logic [7:0]  d;
        logic [3:0]  i;
        begin
            crc = crc_in;
            d   = data_in;
            for (int k = 0; k < 8; k++) begin
                if ((crc[15] ^ d[7]) == 1'b1)
                    crc = (crc << 1) ^ 16'h1021;
                else
                    crc = crc << 1;
                d = d << 1;
            end
            crc16_byte = crc;
        end
    endfunction

    // =========================================================================
    // FSM
    // =========================================================================
    typedef enum logic [1:0] {
        IDLE      = 2'd0,
        ASSEMBLE  = 2'd1,
        TRANSMIT  = 2'd2,
        DONE      = 2'd3
    } enc_state_t;

    enc_state_t state;

    logic [5:0]  byte_idx;      // current transmit index
    logic [7:0]  vcdu_cnt;      // frame counter (wraps)
    logic [15:0] crc_accum;

    // Sync word bytes
    localparam logic [7:0] SYNC [0:3] = '{8'h1A, 8'hCF, 8'hFC, 8'h1D};

    // =========================================================================
    // Frame assembly (registered; takes 1 clock after payload_valid)
    // =========================================================================
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= IDLE;
            byte_idx    <= '0;
            vcdu_cnt    <= '0;
            enc_busy    <= 1'b0;
            frame_done  <= 1'b0;
            uart_tx_valid <= 1'b0;
            uart_tx_byte  <= 8'h00;
            crc_accum   <= 16'hFFFF;
        end else begin
            frame_done    <= 1'b0;
            uart_tx_valid <= 1'b0;

            case (state)
                // --------------------------------------------------------------
                IDLE: begin
                    enc_busy <= 1'b0;
                    if (payload_valid) begin
                        // Build sync word
                        for (int s = 0; s < 4; s++) frame_buf[s] <= SYNC[s];
                        // VCDU primary header (4 bytes)
                        frame_buf[4] <= 8'h01;                      // version=0, SCID MSB
                        frame_buf[5] <= SPACECRAFT_ID[7:0];
                        frame_buf[6] <= {VCID, 2'b00};
                        frame_buf[7] <= vcdu_cnt;
                        // Data field
                        for (int p = 0; p < 32; p++)
                            frame_buf[8 + p] <= payload[p];
                        // Fill bytes
                        for (int f = 42; f < FRAME_LEN; f++)
                            frame_buf[f] <= 8'h00;
                        enc_busy  <= 1'b1;
                        state     <= ASSEMBLE;
                        crc_accum <= 16'hFFFF;
                    end
                end

                // --------------------------------------------------------------
                ASSEMBLE: begin
                    // Compute CRC over bytes [0:39] combinationally in one clock
                    // (40 iterations unrolled - synthesiser will pipeline this)
                    logic [15:0] c;
                    c = 16'hFFFF;
                    for (int b = 0; b < 40; b++)
                        c = crc16_byte(c, frame_buf[b]);
                    frame_buf[40] <= c[15:8];
                    frame_buf[41] <= c[7:0];
                    crc_accum     <= c;
                    byte_idx      <= '0;
                    state         <= TRANSMIT;
                end

                // --------------------------------------------------------------
                TRANSMIT: begin
                    if (uart_tx_ready) begin
                        uart_tx_byte  <= frame_buf[byte_idx];
                        uart_tx_valid <= 1'b1;
                        if (byte_idx == 6'(FRAME_LEN - 1)) begin
                            vcdu_cnt <= vcdu_cnt + 1;
                            state    <= DONE;
                        end else begin
                            byte_idx <= byte_idx + 1;
                        end
                    end
                end

                // --------------------------------------------------------------
                DONE: begin
                    enc_busy   <= 1'b0;
                    frame_done <= 1'b1;
                    state      <= IDLE;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule