// =============================================================================
// File        : mavlink_frame_parser.sv
// Module      : mavlink_frame_parser
// Description : MAVLink v2 frame state machine.
//               Parses: STX(0xFD) → LEN → FLAGS → SEQ → SYS_ID → COMP_ID
//                       → MSG_ID(3B) → PAYLOAD(len B) → CRC(2B)
//               CRC-16/MCRF4XX validation using crc_calc.sv.
//               Emits: msg_id, payload, frame_valid on successful frames.
// =============================================================================

`timescale 1ns/1ps

module mavlink_frame_parser #(
    parameter int MAX_PAYLOAD = 255,
    parameter int CLK_HZ      = 50_000_000
)(
    input  logic clk,
    input  logic rst_n,

    // Byte stream (from async FIFO)
    input  logic [7:0] rx_byte,
    input  logic       rx_valid,

    // Parsed frame output
    output logic [23:0]              msg_id,
    output logic [7:0]               payload [0:MAX_PAYLOAD-1],
    output logic [7:0]               payload_len,
    output logic [7:0]               seq_num,
    output logic [7:0]               sys_id,
    output logic [7:0]               comp_id,
    output logic                     frame_valid,
    output logic                     crc_error
);

    localparam logic [7:0] MAVLINK_STX = 8'hFD;

    typedef enum logic [3:0] {
        S_STX, S_LEN, S_FLAGS, S_INCOMPAT, S_COMPAT, S_SEQ,
        S_SYS, S_COMP, S_MSGID0, S_MSGID1, S_MSGID2,
        S_PAYLOAD, S_CRC_L, S_CRC_H
    } mav_state_t;

    mav_state_t mav_st;

    logic [7:0]  rx_len;
    logic [7:0]  rx_flags;
    logic [7:0]  rx_seq;
    logic [7:0]  rx_sys, rx_comp;
    logic [23:0] rx_msg_id;
    logic [7:0]  payload_cnt;
    logic [15:0] rx_crc;
    logic [15:0] calc_crc;

    // CRC accumulator (reuse crc_calc.sv)
    logic        crc_init, crc_en;
    logic [7:0]  crc_data_in;
    logic [15:0] crc_out;

        crc_calc #(.CRC_TYPE(16)) u_crc (
        .clk       (clk),
        .rst_n     (rst_n),
        .data_in   (crc_data_in),
        .data_valid(crc_en),
        .crc_reset (crc_init),
        .crc_out   (crc_out)
    );

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mav_st      <= S_STX;
            frame_valid <= 1'b0;
            crc_error   <= 1'b0;
            crc_init    <= 1'b0;
            crc_en      <= 1'b0;
            payload_cnt <= 8'h00;
        end else begin
            frame_valid <= 1'b0;
            crc_error   <= 1'b0;
            crc_init    <= 1'b0;
            crc_en      <= 1'b0;

            if (rx_valid) begin
                case (mav_st)
                    S_STX: begin
                        if (rx_byte == MAVLINK_STX) begin
                            crc_init <= 1'b1;
                            mav_st   <= S_LEN;
                        end
                    end
                    S_LEN: begin
                        rx_len      <= rx_byte;
                        crc_en      <= 1'b1;
                        crc_data_in <= rx_byte;
                        mav_st      <= S_FLAGS;
                    end
                    S_FLAGS: begin
                        rx_flags    <= rx_byte;
                        crc_en      <= 1'b1;
                        crc_data_in <= rx_byte;
                        mav_st      <= S_INCOMPAT;
                    end
                    S_INCOMPAT: begin
                        crc_en <= 1'b1; crc_data_in <= rx_byte;
                        mav_st <= S_COMPAT;
                    end
                    S_COMPAT: begin
                        crc_en <= 1'b1; crc_data_in <= rx_byte;
                        mav_st <= S_SEQ;
                    end
                    S_SEQ: begin
                        rx_seq      <= rx_byte;
                        crc_en      <= 1'b1;
                        crc_data_in <= rx_byte;
                        mav_st      <= S_SYS;
                    end
                    S_SYS: begin
                        rx_sys      <= rx_byte;
                        crc_en      <= 1'b1;
                        crc_data_in <= rx_byte;
                        mav_st      <= S_COMP;
                    end
                    S_COMP: begin
                        rx_comp     <= rx_byte;
                        crc_en      <= 1'b1;
                        crc_data_in <= rx_byte;
                        mav_st      <= S_MSGID0;
                    end
                    S_MSGID0: begin
                        rx_msg_id[7:0]   <= rx_byte;
                        crc_en <= 1'b1; crc_data_in <= rx_byte;
                        mav_st <= S_MSGID1;
                    end
                    S_MSGID1: begin
                        rx_msg_id[15:8]  <= rx_byte;
                        crc_en <= 1'b1; crc_data_in <= rx_byte;
                        mav_st <= S_MSGID2;
                    end
                    S_MSGID2: begin
                        rx_msg_id[23:16] <= rx_byte;
                        crc_en      <= 1'b1;
                        crc_data_in <= rx_byte;
                        payload_cnt <= 8'h00;
                        if (rx_len == 8'h00)
                            mav_st <= S_CRC_L;
                        else
                            mav_st <= S_PAYLOAD;
                    end
                    S_PAYLOAD: begin
                        payload[payload_cnt] <= rx_byte;
                        crc_en      <= 1'b1;
                        crc_data_in <= rx_byte;
                        payload_cnt <= payload_cnt + 1'b1;
                        if (payload_cnt == rx_len - 1'b1)
                            mav_st <= S_CRC_L;
                    end
                    S_CRC_L: begin
                        rx_crc[7:0] <= rx_byte;
                        mav_st      <= S_CRC_H;
                    end
                    S_CRC_H: begin
                        rx_crc[15:8] <= rx_byte;
                        // Validate: compare crc_out with received CRC
                        if ({rx_byte, rx_crc[7:0]} == crc_out) begin
                            msg_id      <= rx_msg_id;
                            payload_len <= rx_len;
                            seq_num     <= rx_seq;
                            sys_id      <= rx_sys;
                            comp_id     <= rx_comp;
                            frame_valid <= 1'b1;
                        end else begin
                            crc_error <= 1'b1;
                        end
                        mav_st <= S_STX;
                    end
                    default: mav_st <= S_STX;
                endcase
            end
        end
    end

endmodule
