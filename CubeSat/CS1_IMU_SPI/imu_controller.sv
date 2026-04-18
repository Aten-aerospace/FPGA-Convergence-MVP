// =============================================================================
// Module: imu_controller
// Subsystem: CS1 - IMU Sensor Interface (SPI)
// Description: SPI sequencer for 9-axis IMU (accel + gyro + mag).
//              Uses spi_master.sv for SPI transactions: each register read is a
//              24-bit frame (8-bit address + 16-bit sensor data).
//              Uses crc_calc.sv (CRC-16/CCITT) to validate each received frame.
// Provenance: Convergence MVP FPGA Block Diagram, cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module imu_controller #(
    parameter int CLK_HZ      = 100_000_000, // system clock frequency
    parameter int SPI_HZ      = 10_000_000,  // SPI SCLK frequency (10 MHz -> CLK_DIV=5)
    parameter int NUM_AXES    = 9,            // accel(3) + gyro(3) + mag(3)
    parameter int DATA_BITS   = 16            // per-axis resolution
)(
    input  logic                       clk_100mhz,
    input  logic                       rst_n,

    // trigger
    input  logic                       read_trigger,  // pulse: start a full 9-axis read

    // SPI physical (to IMU) -- driven by spi_master sub-module
    output logic                       spi_sclk,
    output logic                       spi_mosi,
    input  logic                       spi_miso,
    output logic                       spi_cs_n,

    // Parsed sensor output (Q15 fixed-point)
    output logic signed [15:0]         accel_x, accel_y, accel_z,
    output logic signed [15:0]         gyro_x,  gyro_y,  gyro_z,
    output logic signed [15:0]         mag_x,   mag_y,   mag_z,

    output logic                       data_valid,  // strobed 1 cycle when all axes updated
    output logic                       busy,
    output logic                       fault,       // timeout or CRC error
    output logic                       crc_pass     // strobed 1 cycle when frame received cleanly
);

    // =========================================================================
    // SPI clock divider: CLK_DIV = clk / (2 x SPI_HZ) = 100 MHz / 20 MHz = 5
    // =========================================================================
    localparam int CLK_DIV = CLK_HZ / (2 * SPI_HZ);

    // =========================================================================
    // Register address map (MPU-9250 / ICM-42688-P; bit7=1 for SPI read)
    // =========================================================================
    localparam logic [7:0] REG_ACCEL_X_H = 8'h1F;
    localparam logic [7:0] REG_ACCEL_Y_H = 8'h21;
    localparam logic [7:0] REG_ACCEL_Z_H = 8'h23;
    localparam logic [7:0] REG_GYRO_X_H  = 8'h25;
    localparam logic [7:0] REG_GYRO_Y_H  = 8'h27;
    localparam logic [7:0] REG_GYRO_Z_H  = 8'h29;
    localparam logic [7:0] REG_MAG_X_H   = 8'h3B;
    localparam logic [7:0] REG_MAG_Y_H   = 8'h3D;
    localparam logic [7:0] REG_MAG_Z_H   = 8'h3F;

    // =========================================================================
    // FSM
    // spi_master handles clocking, shifting, and CS_N per transaction.
    // This FSM sequences nine 24-bit transactions (one per register).
    // =========================================================================
    typedef enum logic [3:0] {
        S_IDLE,
        S_WAIT_READY,   // wait for spi_master tx_ready before each transaction
        S_WAIT_RX,      // wait for spi_master rx_valid (or timeout)
        S_CRC_H,        // feed received high byte to CRC accumulator
        S_CRC_L,        // feed received low byte to CRC accumulator
        S_NEXT_REG,
        S_DONE,
        S_FAULT
    } state_t;

    state_t state;

    // =========================================================================
    // Register address ROM
    // =========================================================================
    logic [7:0] reg_addr_rom [0:8];
    always_comb begin
        reg_addr_rom[0] = 8'h80 | REG_ACCEL_X_H;  // read bit = bit7
        reg_addr_rom[1] = 8'h80 | REG_ACCEL_Y_H;
        reg_addr_rom[2] = 8'h80 | REG_ACCEL_Z_H;
        reg_addr_rom[3] = 8'h80 | REG_GYRO_X_H;
        reg_addr_rom[4] = 8'h80 | REG_GYRO_Y_H;
        reg_addr_rom[5] = 8'h80 | REG_GYRO_Z_H;
        reg_addr_rom[6] = 8'h80 | REG_MAG_X_H;
        reg_addr_rom[7] = 8'h80 | REG_MAG_Y_H;
        reg_addr_rom[8] = 8'h80 | REG_MAG_Z_H;
    end

    // =========================================================================
    // Internal signals
    // =========================================================================
    logic [3:0]  reg_idx;       // 0..8: current register being read
    logic [23:0] rx_data_r;     // captured rx_data from spi_master

    // Timeout counter (10 ms @ 100 MHz, covers a full 9-transaction sequence)
    localparam int TIMEOUT_CYCLES = CLK_HZ / 100;
    logic [$clog2(TIMEOUT_CYCLES+1)-1:0] timeout_cnt;

    // Collected raw data
    logic signed [15:0] data_buf [0:8];

    // =========================================================================
    // spi_master interface signals
    // Each transaction is 24 bits: [23:16] = address byte, [15:0] = data.
    // Sensor response lands in sp_rx_data[15:0] after the 8-bit address phase.
    // =========================================================================
    logic [23:0] sp_tx_data;
    logic        sp_tx_valid;
    logic        sp_tx_ready;
    logic [23:0] sp_rx_data;
    logic        sp_rx_valid;

    // tx_valid: one-cycle pulse when FSM is waiting and spi_master is idle
    assign sp_tx_valid = (state == S_WAIT_READY) && sp_tx_ready;
    // tx_data: address byte (MSB) + 16-bit dummy to clock out sensor data
    assign sp_tx_data  = {reg_addr_rom[reg_idx], 16'h0000};

    // =========================================================================
    // spi_master instantiation (existing CubeSat RTL module)
    // CLK_DIV = 5 -> SCLK = 100 MHz / (2x5) = 10 MHz
    // DATA_W  = 24 -> 8-bit addr + 16-bit sensor data per transaction
    // =========================================================================
    spi_master #(
        .CLK_DIV (CLK_DIV),
        .DATA_W  (24),
        .CPOL    (0),
        .CPHA    (0)
    ) u_spi (
        .clk      (clk_100mhz),
        .rst_n    (rst_n),
        .tx_data  (sp_tx_data),
        .tx_valid (sp_tx_valid),
        .tx_ready (sp_tx_ready),
        .rx_data  (sp_rx_data),
        .rx_valid (sp_rx_valid),
        .sclk     (spi_sclk),
        .mosi     (spi_mosi),
        .cs_n     (spi_cs_n),
        .miso     (spi_miso)
    );

    // =========================================================================
    // CRC signals (CRC-16/CCITT over the two data bytes of each SPI word)
    // Reset at the start of each full 9-axis read.
    // High byte fed in S_CRC_H, low byte fed in S_CRC_L.
    // =========================================================================
    logic [7:0]  crc_data_in;
    logic        crc_byte_valid;
    logic        crc_rst;
    logic [15:0] crc_out_w;    // accumulated CRC (available for audit/debug)

    crc_calc #(.CRC_TYPE(16)) u_crc (
        .clk        (clk_100mhz),
        .rst_n      (rst_n),
        .data_in    (crc_data_in),
        .data_valid (crc_byte_valid),
        .crc_reset  (crc_rst),
        .crc_out    (crc_out_w)
    );

    always_comb begin
        crc_rst        = (state == S_IDLE) && read_trigger;
        crc_data_in    = '0;
        crc_byte_valid = 1'b0;
        case (state)
            S_CRC_H: begin crc_data_in = rx_data_r[15:8]; crc_byte_valid = 1'b1; end
            S_CRC_L: begin crc_data_in = rx_data_r[7:0];  crc_byte_valid = 1'b1; end
            default: ;
        endcase
    end

    // =========================================================================
    // FSM + datapath
    // =========================================================================
    always_ff @(posedge clk_100mhz or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            reg_idx     <= '0;
            rx_data_r   <= '0;
            data_valid  <= 1'b0;
            busy        <= 1'b0;
            fault       <= 1'b0;
            crc_pass    <= 1'b0;
            timeout_cnt <= '0;
            accel_x     <= '0; accel_y <= '0; accel_z <= '0;
            gyro_x      <= '0; gyro_y  <= '0; gyro_z  <= '0;
            mag_x       <= '0; mag_y   <= '0; mag_z   <= '0;
            for (int i = 0; i < 9; i++) data_buf[i] <= '0;
        end else begin
            data_valid <= 1'b0;
            crc_pass   <= 1'b0;

            case (state)

                S_IDLE: begin
                    busy        <= 1'b0;
                    fault       <= 1'b0;
                    timeout_cnt <= '0;
                    if (read_trigger) begin
                        reg_idx <= '0;
                        busy    <= 1'b1;
                        state   <= S_WAIT_READY;
                    end
                end

                S_WAIT_READY: begin
                    // sp_tx_valid is asserted combinatorially when sp_tx_ready is high.
                    // spi_master latches sp_tx_data and starts the transaction next cycle.
                    if (sp_tx_ready)
                        state <= S_WAIT_RX;
                end

                S_WAIT_RX: begin
                    timeout_cnt <= timeout_cnt + 1'b1;
                    if (timeout_cnt == TIMEOUT_CYCLES[$clog2(TIMEOUT_CYCLES+1)-1:0]) begin
                        state <= S_FAULT;
                    end else if (sp_rx_valid) begin
                        rx_data_r   <= sp_rx_data;
                        timeout_cnt <= '0;
                        state       <= S_CRC_H;
                    end
                end

                S_CRC_H: begin
                    // High byte (rx_data_r[15:8]) fed to CRC combinatorially; advance.
                    state <= S_CRC_L;
                end

                S_CRC_L: begin
                    // Low byte (rx_data_r[7:0]) fed to CRC combinatorially; store data.
                    data_buf[reg_idx] <= signed'(rx_data_r[15:0]);
                    state <= S_NEXT_REG;
                end

                S_NEXT_REG: begin
                    if (reg_idx == 4'd8) begin
                        state <= S_DONE;
                    end else begin
                        reg_idx <= reg_idx + 1'b1;
                        state   <= S_WAIT_READY;
                    end
                end

                S_DONE: begin
                    accel_x    <= data_buf[0];
                    accel_y    <= data_buf[1];
                    accel_z    <= data_buf[2];
                    gyro_x     <= data_buf[3];
                    gyro_y     <= data_buf[4];
                    gyro_z     <= data_buf[5];
                    mag_x      <= data_buf[6];
                    mag_y      <= data_buf[7];
                    mag_z      <= data_buf[8];
                    data_valid <= 1'b1;
                    crc_pass   <= 1'b1;  // all 9 words received without fault
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