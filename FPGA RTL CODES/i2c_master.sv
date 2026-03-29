`timescale 1ns/1ps


module i2c_master #(
  parameter int CLK_HZ = 50_000_000,
  parameter int I2C_HZ = 100_000   // 100k or 400k
)(
  input  logic       clk,
  input  logic       rst_n,

  input  logic       start,
  input  logic [6:0] slave_addr,
  input  logic       rw,           // 0=write, 1=read
  input  logic [7:0] write_data,

  output logic [7:0] read_data,
  output logic       busy,
  output logic       ack_error,

  inout  wire        sda,
  output logic       scl
);

  // ===========================================================================
  // CLOCK DIVIDER (generate SCL ticks)
  // ===========================================================================
  localparam int DIV = CLK_HZ / (I2C_HZ * 4);  // 4 phases per bit

  logic [$clog2(DIV)-1:0] clk_cnt;
  logic tick;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      clk_cnt <= 0;
      tick    <= 0;
    end else begin
      if (clk_cnt == DIV-1) begin
        clk_cnt <= 0;
        tick    <= 1;
      end else begin
        clk_cnt <= clk_cnt + 1;
        tick    <= 0;
      end
    end
  end

  // ===========================================================================
  // OPEN-DRAIN SDA (tristate)
  // ===========================================================================
  logic sda_out_en;   // 1 = drive low, 0 = release
  assign sda = (sda_out_en) ? 1'b0 : 1'bz;
  wire sda_in = sda;

  // ===========================================================================
  // FSM STATES
  // ===========================================================================
  typedef enum logic [3:0] {
    IDLE,
    START,
    ADDR,
    ADDR_ACK,
    DATA,
    DATA_ACK,
    STOP
  } state_t;

  state_t state, next_state;

  // ===========================================================================
  // INTERNAL REGISTERS
  // ===========================================================================
  logic [7:0] shift_reg;
  logic [3:0] bit_cnt;
  logic       scl_en;
  logic [1:0] phase;   // 4-phase clocking

  // ===========================================================================
  // SCL GENERATION (with clock stretching support)
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      scl   <= 1;
      phase <= 0;
    end else if (tick) begin
      if (scl_en) begin
        phase <= phase + 1;

        // SCL low/high phases
        case (phase)
          2'd0: scl <= 0;
          2'd1: scl <= 0;
          2'd2: begin
            scl <= 1;
            // CLOCK STRETCHING: wait until slave releases SCL
            if (scl == 0) phase <= phase; 
          end
          2'd3: scl <= 1;
        endcase
      end else begin
        scl   <= 1;
        phase <= 0;
      end
    end
  end

  // ===========================================================================
  // FSM SEQUENTIAL
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n)
      state <= IDLE;
    else if (tick)
      state <= next_state;
  end

  // ===========================================================================
  // FSM COMBINATIONAL
  // ===========================================================================
  always_comb begin
    next_state = state;

    case (state)
      IDLE:      if (start) next_state = START;

      START:     next_state = ADDR;

      ADDR:      if (bit_cnt == 0 && phase == 3) next_state = ADDR_ACK;

      ADDR_ACK:  next_state = DATA;

      DATA:      if (bit_cnt == 0 && phase == 3) next_state = DATA_ACK;

      DATA_ACK:  next_state = STOP;

      STOP:      next_state = IDLE;
    endcase
  end

  // ===========================================================================
  // DATA PATH + CONTROL
  // ===========================================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy      <= 0;
      ack_error <= 0;
      sda_out_en<= 0;
      shift_reg <= 0;
      bit_cnt   <= 0;
      scl_en    <= 0;
      read_data <= 0;
    end else if (tick) begin

      case (state)

        // --------------------------------------------------
        IDLE: begin
          busy       <= 0;
          scl_en     <= 0;
          sda_out_en <= 0;
          ack_error  <= 0;
        end

        // --------------------------------------------------
        START: begin
          busy       <= 1;
          scl_en     <= 1;
          sda_out_en <= 1; // pull SDA low while SCL high
          shift_reg  <= {slave_addr, rw};
          bit_cnt    <= 7;
        end

        // --------------------------------------------------
        ADDR: begin
          // drive bits on SDA during SCL low
          if (phase == 0) begin
            sda_out_en <= ~shift_reg[bit_cnt];
          end

          // shift on falling edge
          if (phase == 3) begin
            if (bit_cnt != 0)
              bit_cnt <= bit_cnt - 1;
          end
        end

        // --------------------------------------------------
        ADDR_ACK: begin
          if (phase == 0)
            sda_out_en <= 0; // release SDA

          if (phase == 2) begin
            if (sda_in != 0)
              ack_error <= 1;
          end

          shift_reg <= write_data;
          bit_cnt   <= 7;
        end

        // --------------------------------------------------
        DATA: begin
          if (rw == 0) begin
            // WRITE
            if (phase == 0)
              sda_out_en <= ~shift_reg[bit_cnt];

          end else begin
            // READ
            if (phase == 2)
              read_data[bit_cnt] <= sda_in;
          end

          if (phase == 3) begin
            if (bit_cnt != 0)
              bit_cnt <= bit_cnt - 1;
          end
        end

        // --------------------------------------------------
        DATA_ACK: begin
          if (rw == 0) begin
            // check ACK
            if (phase == 0)
              sda_out_en <= 0;

            if (phase == 2 && sda_in != 0)
              ack_error <= 1;

          end else begin
            // send NACK after read
            if (phase == 0)
              sda_out_en <= 0;
          end
        end

        // --------------------------------------------------
        STOP: begin
          if (phase == 0)
            sda_out_en <= 1; // pull low

          if (phase == 2)
            sda_out_en <= 0; // release -> STOP

          busy   <= 0;
          scl_en <= 0;
        end

      endcase
    end
  end

endmodule

