`timescale 1ns/1ps


module mavlink_parser #(
  parameter MAX_PAYLOAD = 64
)(
  input  logic       clk, rst_n,

  // RX interface
  input  logic [7:0] rx_byte,
  input  logic       rx_valid,

  // Decoded output
  output logic [23:0] msg_id,
  output logic [7:0]  sys_id, comp_id,
  output logic [7:0]  payload [0:MAX_PAYLOAD-1],
  output logic [7:0]  payload_len,
  output logic        frame_valid,
  output logic        crc_error,

  // TX interface
  input  logic [23:0] tx_msg_id,
  input  logic [7:0]  tx_payload [0:MAX_PAYLOAD-1],
  input  logic [7:0]  tx_len,
  input  logic        tx_send,
  output logic [7:0]  tx_byte,
  output logic        tx_byte_valid,
  input  logic        tx_byte_ready
);

  // ============================================================
  // MAVLink constants
  // ============================================================
  localparam STX = 8'hFD;

  typedef enum logic [2:0] {
    RX_IDLE,
    RX_START,
    RX_HEADER,
    RX_PAYLOAD,
    RX_CRC,
    RX_DECODE
  } rx_state_t;

  rx_state_t state;

  // ============================================================
  // RX registers
  // ============================================================
  logic [7:0] header [0:7];
  logic [7:0] crc_l, crc_h;

  logic [7:0] count;
  logic [7:0] payload_cnt;

  logic [15:0] crc_calc;

  // ============================================================
  // CRC-16/MCRF4XX function (poly 0x1021, init 0xFFFF)
  // ============================================================
  function automatic [15:0] crc16_update(
    input [15:0] crc,
    input [7:0]  data
  );
    logic [15:0] c;
    integer i;
    begin
      c = crc ^ (data << 8);
      for (i = 0; i < 8; i++) begin
        if (c[15])
          c = (c << 1) ^ 16'h1021;
        else
          c = (c << 1);
      end
      return c;
    end
  endfunction

  // ============================================================
  // RX FSM
  // ============================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state        <= RX_IDLE;
      count        <= 0;
      payload_cnt  <= 0;
      frame_valid  <= 0;
      crc_error    <= 0;
      crc_calc     <= 16'hFFFF;
    end else begin
      frame_valid <= 0;

      if (rx_valid) begin
        case (state)

          // ----------------------------------------
          RX_IDLE: begin
            if (rx_byte == STX) begin
              state    <= RX_START;
              crc_calc <= 16'hFFFF;
            end
          end

          // ----------------------------------------
          RX_START: begin
            payload_len <= rx_byte;
            crc_calc    <= crc16_update(16'hFFFF, rx_byte);
            count       <= 0;
            state       <= RX_HEADER;
          end

          // ----------------------------------------
          RX_HEADER: begin
            header[count] <= rx_byte;
            crc_calc      <= crc16_update(crc_calc, rx_byte);

            count <= count + 1;

            if (count == 7) begin
              payload_cnt <= 0;
              state       <= RX_PAYLOAD;
            end
          end

          // ----------------------------------------
          RX_PAYLOAD: begin
            if (payload_cnt < MAX_PAYLOAD)
              payload[payload_cnt] <= rx_byte;

            crc_calc <= crc16_update(crc_calc, rx_byte);
            payload_cnt <= payload_cnt + 1;

            if (payload_cnt == payload_len - 1)
              state <= RX_CRC;
          end

          // ----------------------------------------
          RX_CRC: begin
            if (count == 0) begin
              crc_l <= rx_byte;
              count <= 1;
            end else begin
              crc_h <= rx_byte;
              state <= RX_DECODE;
            end
          end

          // ----------------------------------------
          RX_DECODE: begin
            // Extract fields
            sys_id  <= header[3];
            comp_id <= header[4];
            msg_id  <= {header[7], header[6], header[5]};

            // CRC check
            if ({crc_h, crc_l} == crc_calc) begin
              frame_valid <= 1;
              crc_error   <= 0;
            end else begin
              frame_valid <= 0;
              crc_error   <= 1;
            end

            state <= RX_IDLE;
          end

        endcase
      end
    end
  end

  // ============================================================
  // TX FSM (linear sequencer)
  // ============================================================
  typedef enum logic [2:0] {
    TX_IDLE,
    TX_SEND,
    TX_CRC_L,
    TX_CRC_H
  } tx_state_t;

  tx_state_t tx_state;

  logic [7:0] tx_cnt;
  logic [15:0] tx_crc;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_state       <= TX_IDLE;
      tx_byte_valid  <= 0;
      tx_cnt         <= 0;
      tx_crc         <= 16'hFFFF;
    end else begin
      tx_byte_valid <= 0;

      case (tx_state)

        // ----------------------------------------
        TX_IDLE: begin
          if (tx_send) begin
            tx_state <= TX_SEND;
            tx_cnt   <= 0;
            tx_crc   <= 16'hFFFF;
          end
        end

        // ----------------------------------------
        TX_SEND: begin
          if (tx_byte_ready) begin
            tx_byte_valid <= 1;

            case (tx_cnt)
              0: tx_byte <= STX;
              1: begin
                   tx_byte <= tx_len;
                   tx_crc  <= crc16_update(tx_crc, tx_len);
                 end

              // header
              2: begin tx_byte <= 8'h00; tx_crc <= crc16_update(tx_crc, 8'h00); end // incompat flags
              3: begin tx_byte <= 8'h00; tx_crc <= crc16_update(tx_crc, 8'h00); end // compat flags
              4: begin tx_byte <= 8'h00; tx_crc <= crc16_update(tx_crc, 8'h00); end // seq
              5: begin tx_byte <= 8'h01; tx_crc <= crc16_update(tx_crc, 8'h01); end // sys_id
              6: begin tx_byte <= 8'h01; tx_crc <= crc16_update(tx_crc, 8'h01); end // comp_id
              7: begin tx_byte <= tx_msg_id[7:0];   tx_crc <= crc16_update(tx_crc, tx_msg_id[7:0]); end
              8: begin tx_byte <= tx_msg_id[15:8];  tx_crc <= crc16_update(tx_crc, tx_msg_id[15:8]); end
              9: begin tx_byte <= tx_msg_id[23:16]; tx_crc <= crc16_update(tx_crc, tx_msg_id[23:16]); end

              default: begin
                if (tx_cnt < (10 + tx_len)) begin
                  tx_byte <= tx_payload[tx_cnt-10];
                  tx_crc  <= crc16_update(tx_crc, tx_payload[tx_cnt-10]);
                end
              end
            endcase

            tx_cnt <= tx_cnt + 1;

            if (tx_cnt == (10 + tx_len))
              tx_state <= TX_CRC_L;
          end
        end

        // ----------------------------------------
        TX_CRC_L: begin
          if (tx_byte_ready) begin
            tx_byte       <= tx_crc[7:0];
            tx_byte_valid <= 1;
            tx_state      <= TX_CRC_H;
          end
        end

        // ----------------------------------------
        TX_CRC_H: begin
          if (tx_byte_ready) begin
            tx_byte       <= tx_crc[15:8];
            tx_byte_valid <= 1;
            tx_state      <= TX_IDLE;
          end
        end

      endcase
    end
  end

endmodule