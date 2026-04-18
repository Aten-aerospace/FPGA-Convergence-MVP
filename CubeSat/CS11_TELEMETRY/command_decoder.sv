// =============================================================================
// Module: command_decoder (CS11 ground command UART RX parser)
// Subsystem: CS11 - Telemetry Encoder
// Description: Receives and decodes ground commands from the UART RX line.
//              Validates CRC-16 CCITT and extracts command fields.
//
//   Uplink command frame format (CS-TLM-008):
//     [0:3]         SYNC     0x1A CF FC 1D (4 bytes)
//     [4:5]         APID     target subsystem (2 bytes)
//     [6]           CMD_CODE command opcode (1 byte)
//     [7]           CMD_LEN  command data length (0-16 bytes)
//     [8:8+LEN-1]   CMD_DATA command parameters (variable)
//     [8+LEN:9+LEN] CRC-16   CCITT-FALSE over all preceding bytes (2 bytes)
//
//   CRC is computed over bytes 0 through 7+LEN (SYNC+APID+CMD_CODE+CMD_LEN+DATA).
//
// Reuses: uart_controller.sv (RX byte stream), crc_calc.sv (CRC validation)
// Provenance: CS11 Telemetry Encoder specification CS-TLM-008
// =============================================================================
`timescale 1ns/1ps

module command_decoder #(
    parameter int CLK_HZ  = 100_000_000,
    parameter int BAUD_HZ = 115_200
)(
    input  logic        clk,
    input  logic        rst_n,

    // UART RX
    input  logic        uart_rx,

    // Decoded command output
    output logic        cmd_valid,          // one-cycle pulse when valid command decoded
    output logic [15:0] cmd_apid,           // target APID
    output logic [7:0]  cmd_code,           // command opcode
    output logic [7:0]  cmd_data [0:15],    // command parameters (up to 16 bytes)
    output logic [4:0]  cmd_data_len,       // actual data length (0-16)
    output logic        cmd_crc_error,      // CRC validation failed (one-cycle pulse)
    output logic        cmd_sync_error      // sync word not found (one-cycle pulse)
);

    // =========================================================================
    // UART controller (RX only; TX tied off)
    // =========================================================================
    logic [7:0] rx_byte;
    logic       rx_valid;
    logic       rx_ready;
    logic       rx_frame_err;
    logic       rx_overflow;

    assign rx_ready = 1'b1;  // always ready to consume bytes

    uart_controller #(
        .CLK_HZ    (CLK_HZ),
        .BAUD      (BAUD_HZ),
        .DATA_BITS (8),
        .STOP_BITS (1),
        .PARITY    (0)
    ) u_uart_rx (
        .clk          (clk),
        .rst_n        (rst_n),
        // TX (not used)
        .tx_data      (8'h00),
        .tx_valid     (1'b0),
        .tx_ready     (),
        .uart_tx      (),
        // RX
        .rx_data      (rx_byte),
        .rx_valid     (rx_valid),
        .rx_ready     (rx_ready),
        .uart_rx      (uart_rx),
        .rx_frame_err (rx_frame_err),
        .rx_overflow  (rx_overflow)
    );

    // =========================================================================
    // CRC-16 CCITT accumulator
    // =========================================================================
    logic        crc_reset;
    logic        crc_en;
    logic [7:0]  crc_data_in;
    logic [15:0] crc_out;

    crc_calc #(.CRC_TYPE(16)) u_crc (
        .clk        (clk),
        .rst_n      (rst_n),
        .data_in    (crc_data_in),
        .data_valid (crc_en),
        .crc_reset  (crc_reset),
        .crc_out    (crc_out)
    );

    // =========================================================================
    // Command frame parse FSM
    // =========================================================================
    typedef enum logic [3:0] {
        S_SYNC0    = 4'd0,   // waiting for 0x1A
        S_SYNC1    = 4'd1,   // waiting for 0xCF
        S_SYNC2    = 4'd2,   // waiting for 0xFC
        S_SYNC3    = 4'd3,   // waiting for 0x1D
        S_APID0    = 4'd4,   // APID high byte
        S_APID1    = 4'd5,   // APID low  byte
        S_CMDCODE  = 4'd6,   // CMD_CODE byte
        S_CMDLEN   = 4'd7,   // CMD_LEN byte
        S_DATA     = 4'd8,   // CMD_DATA bytes (0..CMD_LEN-1)
        S_CRC0     = 4'd9,   // received CRC high byte
        S_CRC1     = 4'd10,  // received CRC low  byte
        S_VALIDATE = 4'd11   // compare received vs computed CRC
    } cmd_state_t;

    cmd_state_t  cmd_st;

    // Receive buffers
    logic [15:0] rx_apid;
    logic [7:0]  rx_cmd_code;
    logic [4:0]  rx_cmd_len;
    logic [7:0]  rx_data_buf [0:15];
    logic [4:0]  data_cnt;
    logic [7:0]  rx_crc_hi;

    // Sync word constants
    localparam logic [7:0] SYNC [0:3] = '{8'h1A, 8'hCF, 8'hFC, 8'h1D};

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cmd_st       <= S_SYNC0;
            cmd_valid    <= 1'b0;
            cmd_crc_error <= 1'b0;
            cmd_sync_error <= 1'b0;
            cmd_apid     <= '0;
            cmd_code     <= '0;
            cmd_data_len <= '0;
            rx_apid      <= '0;
            rx_cmd_code  <= '0;
            rx_cmd_len   <= '0;
            rx_crc_hi    <= '0;
            data_cnt     <= '0;
            crc_reset    <= 1'b1;
            crc_en       <= 1'b0;
            crc_data_in  <= 8'h00;
            for (int i = 0; i < 16; i++) begin
                cmd_data[i]    <= 8'h00;
                rx_data_buf[i] <= 8'h00;
            end
        end else begin
            // Default: clear single-cycle strobes
            cmd_valid     <= 1'b0;
            cmd_crc_error <= 1'b0;
            cmd_sync_error <= 1'b0;
            crc_reset     <= 1'b0;
            crc_en        <= 1'b0;

            if (rx_valid) begin
                // Feed byte into CRC accumulator (all bytes before CRC bytes)
                case (cmd_st)
                    S_SYNC0, S_SYNC1, S_SYNC2, S_SYNC3,
                    S_APID0, S_APID1, S_CMDCODE, S_CMDLEN, S_DATA: begin
                        crc_data_in <= rx_byte;
                        crc_en      <= 1'b1;
                    end
                    default: ;
                endcase

                case (cmd_st)
                    // ---------------------------------------------------------
                    S_SYNC0: begin
                        if (rx_byte == SYNC[0]) begin
                            crc_reset <= 1'b1;    // reset CRC for new frame
                            crc_en    <= 1'b0;    // reset overrides data this cycle
                            cmd_st    <= S_SYNC1;
                        end
                        // else stay in SYNC0 hunting
                    end

                    S_SYNC1: begin
                        if (rx_byte == SYNC[1])
                            cmd_st <= S_SYNC2;
                        else begin
                            cmd_sync_error <= 1'b1;
                            cmd_st         <= S_SYNC0;
                        end
                    end

                    S_SYNC2: begin
                        if (rx_byte == SYNC[2])
                            cmd_st <= S_SYNC3;
                        else begin
                            cmd_sync_error <= 1'b1;
                            cmd_st         <= S_SYNC0;
                        end
                    end

                    S_SYNC3: begin
                        if (rx_byte == SYNC[3])
                            cmd_st <= S_APID0;
                        else begin
                            cmd_sync_error <= 1'b1;
                            cmd_st         <= S_SYNC0;
                        end
                    end

                    // ---------------------------------------------------------
                    S_APID0: begin
                        rx_apid[15:8] <= rx_byte;
                        cmd_st        <= S_APID1;
                    end

                    S_APID1: begin
                        rx_apid[7:0] <= rx_byte;
                        cmd_st       <= S_CMDCODE;
                    end

                    // ---------------------------------------------------------
                    S_CMDCODE: begin
                        rx_cmd_code <= rx_byte;
                        cmd_st      <= S_CMDLEN;
                    end

                    S_CMDLEN: begin
                        // Clamp to 16 bytes maximum
                        if (rx_byte > 8'd16)
                            rx_cmd_len <= 5'd16;
                        else
                            rx_cmd_len <= rx_byte[4:0];
                        data_cnt <= '0;
                        // If no data bytes, go straight to CRC
                        if (rx_byte == 8'h00)
                            cmd_st <= S_CRC0;
                        else
                            cmd_st <= S_DATA;
                    end

                    // ---------------------------------------------------------
                    S_DATA: begin
                        if (data_cnt < rx_cmd_len) begin
                            rx_data_buf[data_cnt] <= rx_byte;
                            data_cnt              <= data_cnt + 5'd1;
                            if (data_cnt == rx_cmd_len - 5'd1)
                                cmd_st <= S_CRC0;
                        end
                    end

                    // ---------------------------------------------------------
                    S_CRC0: begin
                        rx_crc_hi <= rx_byte;
                        cmd_st    <= S_CRC1;
                    end

                    S_CRC1: begin
                        // Validate: {rx_crc_hi, rx_byte} must equal crc_out
                        // Note: crc_out is the accumulated CRC over all bytes
                        //       up to and including the last data byte.
                        if ({rx_crc_hi, rx_byte} == crc_out) begin
                            // CRC valid - latch command outputs
                            cmd_valid    <= 1'b1;
                            cmd_apid     <= rx_apid;
                            cmd_code     <= rx_cmd_code;
                            cmd_data_len <= rx_cmd_len;
                            for (int i = 0; i < 16; i++)
                                cmd_data[i] <= rx_data_buf[i];
                        end else begin
                            cmd_crc_error <= 1'b1;
                        end
                        cmd_st <= S_SYNC0;
                    end

                    default: cmd_st <= S_SYNC0;
                endcase
            end
        end
    end

endmodule