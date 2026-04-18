// =============================================================================
// Module: i2c_mag_controller
// Subsystem: CS2 - Magnetometer Interface (I2C)
// Description: I2C burst-read sequencer for HMC5883L / IST8310 magnetometer.
//              Writes register pointer 0x03 once, then performs 6 consecutive
//              single-byte reads exploiting the device's internal auto-increment.
//              Assembles signed 16-bit raw_x/y/z outputs.
//              Instantiated as a sub-module by i2c_mag_wrapper.
// Provenance: cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module i2c_mag_controller #(
    parameter int CLK_HZ   = 100_000_000,
    parameter int I2C_HZ   = 400_000,
    parameter int MAG_ADDR = 7'h0E        // HMC5883L / IST8310 default
)(
    input  logic       sys_clk,
    input  logic       rst_n,

    // One-pulse trigger to initiate a full burst read
    input  logic       read_trigger,

    // I2C physical pins
    inout  wire        i2c_sda,
    output logic       i2c_scl,

    // Raw assembled outputs (signed 16-bit)
    output logic signed [15:0] raw_x,
    output logic signed [15:0] raw_y,
    output logic signed [15:0] raw_z,

    output logic       data_valid,
    output logic       busy,
    output logic       fault
);

    // =========================================================================
    // i2c_master interface signals
    // =========================================================================
    logic       m_start;
    logic [6:0] m_addr;
    logic       m_rw;
    logic [7:0] m_write_data;
    logic [7:0] m_read_data;
    logic       m_busy;
    logic       m_ack_error;

    i2c_master #(
        .CLK_HZ (CLK_HZ),
        .I2C_HZ (I2C_HZ)
    ) u_i2c (
        .clk        (sys_clk),
        .rst_n      (rst_n),
        .start      (m_start),
        .slave_addr (m_addr),
        .rw         (m_rw),
        .write_data (m_write_data),
        .read_data  (m_read_data),
        .busy       (m_busy),
        .ack_error  (m_ack_error),
        .sda        (i2c_sda),
        .scl        (i2c_scl)
    );

    // =========================================================================
    // Falling-edge detect on i2c_master busy → transaction complete
    // =========================================================================
    logic prev_busy;
    logic i2c_done;

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) prev_busy <= 1'b0;
        else        prev_busy <= m_busy;
    end

    assign i2c_done = prev_busy & ~m_busy;

    // =========================================================================
    // FSM - burst read: write register pointer 0x03 once, then read 6 bytes.
    // HMC5883L / IST8310 auto-increments the internal register address on each
    // read, so no re-write of the pointer is needed between bytes.
    // Timing: 1 write (~50 µs) + 6 reads (~50 µs each) ≈ 350 µs @ 400 kHz
    // =========================================================================
    typedef enum logic [3:0] {
        S_IDLE,
        S_WRITE_PTR,   // write starting register address 0x03
        S_WAIT_WRITE,  // wait for write transaction to finish
        S_READ_BYTE,   // issue single-byte read (device auto-increments ptr)
        S_WAIT_READ,   // wait for read transaction to finish
        S_NEXT_BYTE,   // capture byte, advance index or finish
        S_ASSEMBLE,    // build signed 16-bit words
        S_DONE,
        S_FAULT
    } state_t;

    state_t state;

    localparam logic [7:0] MAG_REG_START = 8'h03;

    logic [2:0] byte_idx;
    logic [7:0] raw_bytes [0:5];

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            m_start      <= 1'b0;
            m_rw         <= 1'b0;
            m_addr       <= MAG_ADDR[6:0];
            m_write_data <= 8'h00;
            byte_idx     <= '0;
            data_valid   <= 1'b0;
            busy         <= 1'b0;
            fault        <= 1'b0;
            raw_x        <= '0;
            raw_y        <= '0;
            raw_z        <= '0;
            for (int i = 0; i < 6; i++) raw_bytes[i] <= 8'h00;
        end else begin
            m_start    <= 1'b0;
            data_valid <= 1'b0;

            case (state)

                S_IDLE: begin
                    busy  <= 1'b0;
                    fault <= 1'b0;
                    if (read_trigger) begin
                        byte_idx <= '0;
                        busy     <= 1'b1;
                        state    <= S_WRITE_PTR;
                    end
                end

                // Write register pointer 0x03 to set device read address
                S_WRITE_PTR: begin
                    m_addr       <= MAG_ADDR[6:0];
                    m_write_data <= MAG_REG_START;
                    m_rw         <= 1'b0;
                    m_start      <= 1'b1;
                    state        <= S_WAIT_WRITE;
                end

                S_WAIT_WRITE: begin
                    if (i2c_done) begin
                        if (m_ack_error) state <= S_FAULT;
                        else begin
                            m_rw  <= 1'b1;
                            state <= S_READ_BYTE;
                        end
                    end
                end

                // Issue a single-byte read; device auto-increments register ptr
                S_READ_BYTE: begin
                    m_addr  <= MAG_ADDR[6:0];
                    m_start <= 1'b1;
                    state   <= S_WAIT_READ;
                end

                S_WAIT_READ: begin
                    if (i2c_done) begin
                        if (m_ack_error) state <= S_FAULT;
                        else             state <= S_NEXT_BYTE;
                    end
                end

                S_NEXT_BYTE: begin
                    raw_bytes[byte_idx] <= m_read_data;
                    if (byte_idx == 3'd5) begin
                        state <= S_ASSEMBLE;
                    end else begin
                        byte_idx <= byte_idx + 3'd1;
                        state    <= S_READ_BYTE;
                    end
                end

                S_ASSEMBLE: begin
                    raw_x <= signed'({raw_bytes[0], raw_bytes[1]});
                    raw_y <= signed'({raw_bytes[2], raw_bytes[3]});
                    raw_z <= signed'({raw_bytes[4], raw_bytes[5]});
                    state <= S_DONE;
                end

                S_DONE: begin
                    data_valid <= 1'b1;
                    busy       <= 1'b0;
                    state      <= S_IDLE;
                end

                S_FAULT: begin
                    fault <= 1'b1;
                    busy  <= 1'b0;
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule