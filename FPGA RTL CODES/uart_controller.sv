`timescale 1ns/1ps


module uart_controller #(
  parameter CLK_HZ     = 50_000_000,
  parameter BAUD       = 115200,
  parameter DATA_BITS  = 8,
  parameter STOP_BITS  = 1,
  parameter PARITY     = 0   // 0 = none
)(
  input  logic       clk,
  input  logic       rst_n,

  // TX
  input  logic [7:0] tx_data,
  input  logic       tx_valid,
  output logic       tx_ready,
  output logic       uart_tx,

  // RX
  output logic [7:0] rx_data,
  output logic       rx_valid,
  input  logic       rx_ready,
  input  logic       uart_rx,

  // Status
  output logic       rx_frame_err,
  output logic       rx_overflow
);

  // ============================================================
  // BAUD GENERATOR
  // ============================================================
  localparam integer BAUD_DIV     = CLK_HZ / BAUD;
  localparam integer BAUD_DIV_16  = CLK_HZ / (BAUD * 16);

  logic [$clog2(BAUD_DIV)-1:0] baud_cnt;
  logic baud_tick;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      baud_cnt  <= 0;
      baud_tick <= 0;
    end else begin
      if (baud_cnt == BAUD_DIV-1) begin
        baud_cnt  <= 0;
        baud_tick <= 1;
      end else begin
        baud_cnt  <= baud_cnt + 1;
        baud_tick <= 0;
      end
    end
  end

  // 16x oversample tick
  logic [$clog2(BAUD_DIV_16)-1:0] baud16_cnt;
  logic baud16_tick;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      baud16_cnt  <= 0;
      baud16_tick <= 0;
    end else begin
      if (baud16_cnt == BAUD_DIV_16-1) begin
        baud16_cnt  <= 0;
        baud16_tick <= 1;
      end else begin
        baud16_cnt  <= baud16_cnt + 1;
        baud16_tick <= 0;
      end
    end
  end

  // ============================================================
  // TX FSM
  // ============================================================
  typedef enum logic [1:0] {
    TX_IDLE,
    TX_START,
    TX_DATA,
    TX_STOP
  } tx_state_t;

  tx_state_t tx_state;
  logic [7:0] tx_shift;
  logic [3:0] tx_bit_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      tx_state   <= TX_IDLE;
      uart_tx    <= 1'b1;
      tx_ready   <= 1'b1;
      tx_bit_cnt <= 0;
    end else begin
      if (baud_tick) begin
        case (tx_state)

          TX_IDLE: begin
            uart_tx  <= 1'b1;
            tx_ready <= 1'b1;
            if (tx_valid) begin
              tx_shift <= tx_data;
              tx_state <= TX_START;
              tx_ready <= 1'b0;
            end
          end

          TX_START: begin
            uart_tx  <= 1'b0;
            tx_state <= TX_DATA;
            tx_bit_cnt <= 0;
          end

          TX_DATA: begin
            uart_tx  <= tx_shift[0];
            tx_shift <= tx_shift >> 1;
            tx_bit_cnt <= tx_bit_cnt + 1;

            if (tx_bit_cnt == DATA_BITS-1)
              tx_state <= TX_STOP;
          end

          TX_STOP: begin
            uart_tx <= 1'b1;
            tx_state <= TX_IDLE;
          end

        endcase
      end
    end
  end

  // ============================================================
  // RX FIFO (8-deep)
  // ============================================================
  localparam FIFO_DEPTH = 8;
  logic [7:0] fifo_mem [FIFO_DEPTH];
  logic [2:0] wr_ptr, rd_ptr;
  logic [3:0] fifo_count;

  logic fifo_full  = (fifo_count == FIFO_DEPTH);
  logic fifo_empty = (fifo_count == 0);

  // Write
  logic fifo_wr;
  logic [7:0] fifo_din;

  // Read
  logic fifo_rd;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr <= 0;
      rd_ptr <= 0;
      fifo_count <= 0;
      rx_overflow <= 0;
    end else begin
      fifo_wr <= 0;
      fifo_rd <= 0;

      // Write
      if (fifo_din_valid) begin
        if (!fifo_full) begin
          fifo_mem[wr_ptr] <= fifo_din;
          wr_ptr <= wr_ptr + 1;
          fifo_count <= fifo_count + 1;
        end else begin
          rx_overflow <= 1;
        end
      end

      // Read
      if (rx_ready && !fifo_empty) begin
        rx_data <= fifo_mem[rd_ptr];
        rd_ptr <= rd_ptr + 1;
        fifo_count <= fifo_count - 1;
      end
    end
  end

  assign rx_valid = !fifo_empty;

  // ============================================================
  // RX FSM (16x oversampling + majority vote)
  // ============================================================
  typedef enum logic [1:0] {
    RX_IDLE,
    RX_START,
    RX_DATA,
    RX_STOP
  } rx_state_t;

  rx_state_t rx_state;

  logic [3:0] sample_cnt;
  logic [2:0] bit_cnt;
  logic [7:0] rx_shift;

  // sampling buffer for majority vote
  logic [2:0] sample_buf;

  logic sample_mid;
  logic fifo_din_valid;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rx_state <= RX_IDLE;
      sample_cnt <= 0;
      bit_cnt <= 0;
      fifo_din_valid <= 0;
      rx_frame_err <= 0;
    end else begin
      fifo_din_valid <= 0;

      if (baud16_tick) begin
        case (rx_state)

          RX_IDLE: begin
            if (!uart_rx) begin
              rx_state <= RX_START;
              sample_cnt <= 0;
            end
          end

          RX_START: begin
            sample_cnt <= sample_cnt + 1;

            if (sample_cnt == 7) begin
              if (!uart_rx)
                rx_state <= RX_DATA;
              else
                rx_state <= RX_IDLE;
              sample_cnt <= 0;
              bit_cnt <= 0;
            end
          end

          RX_DATA: begin
            sample_cnt <= sample_cnt + 1;

            // take 3 middle samples (7,8,9)
            if (sample_cnt >= 7 && sample_cnt <= 9)
              sample_buf[sample_cnt-7] <= uart_rx;

            if (sample_cnt == 15) begin
              // majority vote
              sample_mid = (sample_buf[0] + sample_buf[1] + sample_buf[2]) >= 2;
              rx_shift <= {sample_mid, rx_shift[7:1]};
              bit_cnt <= bit_cnt + 1;
              sample_cnt <= 0;

              if (bit_cnt == DATA_BITS-1)
                rx_state <= RX_STOP;
            end
          end

          RX_STOP: begin
            sample_cnt <= sample_cnt + 1;

            if (sample_cnt == 15) begin
              if (uart_rx) begin
                fifo_din <= rx_shift;
                fifo_din_valid <= 1;
              end else begin
                rx_frame_err <= 1;
              end
              rx_state <= RX_IDLE;
            end
          end

        endcase
      end
    end
  end

endmodule

