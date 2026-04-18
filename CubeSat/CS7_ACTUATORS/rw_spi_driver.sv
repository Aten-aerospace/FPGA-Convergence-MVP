// =============================================================================
// Module: rw_spi_driver (CS7 reaction-wheel SPI command & fault polling driver)
// Subsystem: CS7 - Actuator Drivers
// Requirement: CS-ADCS-008 - "fault status read @ 10 Hz"
//
// Description:
//   Translates PD control torque commands (Q15) into 3 independent SPI motor
//   driver commands at each polling interval.  Polls fault status from each
//   reaction-wheel motor driver at POLL_PERIOD × 1 kHz ≈ 10 Hz.  Merges
//   per-axis SPI fault feedback with an internal watchdog (cmd_valid timeout)
//   to produce rw_fault[2:0].
//
//   Uses a single shared spi_master instance (time-multiplexed across 3 RW
//   chips).  MOSI is broadcast to all axes; only the active wheel's CS_N is
//   asserted.  MISO is selected from the active wheel.
//
// SPI data format (16-bit full-duplex, CS-ADCS-008):
//   TX (MOSI): torque_cmd[15:0] - 16-bit signed Q15 speed command
//              (1 LSB ≈ 0.2 RPM; range ±6000 RPM = ±30000 LSB @ Q15 mapping)
//   RX (MISO): {fault_bit[15], status[14:0]} - bit[15]=1 indicates motor fault
//
// Timing pipeline (CS-ADCS-008: "1 ms execution"):
//   SPI clock = CLK_HZ / (2 × SPI_CLK_DIV) = 100 MHz / 12 ≈ 8.33 MHz
//   SPI transaction: 16 bits × 12 clk/bit = 192 clk ≈ 1.92 µs per wheel
//   3 wheels sequentially: 3 × 1.92 µs = 5.76 µs ≪ 1 ms deadline ✓
//   Fault polling rate: POLL_PERIOD × 1 kHz → 10 Hz (every 100 ce_1khz ticks)
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md
// =============================================================================
`timescale 1ns/1ps

module rw_spi_driver #(
    parameter int CLK_HZ        = 100_000_000,
    parameter int SPI_CLK_DIV   = 6,              // sclk ≈ CLK_HZ/(2×CLK_DIV) ≈ 8.33 MHz
    parameter int POLL_PERIOD   = 100,            // poll every N ce_1khz ticks → 10 Hz
    parameter int FAULT_TIMEOUT = 200             // watchdog: N ce_1khz ticks before fault
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_1khz,

    // Torque commands from CS6 (Q15 signed)
    input  logic signed [15:0] torque_cmd [0:2],
    input  logic               cmd_valid,
    input  logic               safe_mode,

    // SPI bus: shared clock + per-axis data lines and chip-selects
    // rw_sclk    - shared SPI clock to all 3 motor drivers
    // rw_mosi[i] - MOSI to motor driver i (same data, only active CS selected)
    // rw_cs_n[i] - active-low chip select for motor driver i
    // rw_miso[i] - MISO from motor driver i
    output logic        rw_sclk,
    output logic [2:0]  rw_mosi,
    output logic [2:0]  rw_cs_n,
    input  logic [2:0]  rw_miso,

    // Per-axis fault: internal watchdog OR SPI fault bit
    output logic [2:0]  rw_fault
);

    // =========================================================================
    // SPI master (shared, time-multiplexed across 3 wheels)
    // =========================================================================
    logic [15:0] spi_tx_data;
    logic        spi_tx_valid;
    logic        spi_tx_ready;
    logic [15:0] spi_rx_data;
    logic        spi_rx_valid;
    logic        spi_sclk;
    logic        spi_mosi;
    logic        spi_cs_n;
    logic        spi_miso;

    spi_master #(
        .CLK_DIV (SPI_CLK_DIV),
        .DATA_W  (16),
        .CPOL    (0),
        .CPHA    (0)
    ) u_spi (
        .clk      (clk),
        .rst_n    (rst_n),
        .tx_data  (spi_tx_data),
        .tx_valid (spi_tx_valid),
        .tx_ready (spi_tx_ready),
        .rx_data  (spi_rx_data),
        .rx_valid (spi_rx_valid),
        .sclk     (spi_sclk),
        .mosi     (spi_mosi),
        .cs_n     (spi_cs_n),
        .miso     (spi_miso)
    );

    // =========================================================================
    // 10 Hz poll counter (counts ce_1khz ticks, fires every POLL_PERIOD ticks)
    // =========================================================================
    localparam int POLL_W = $clog2(POLL_PERIOD + 1);

    logic [POLL_W-1:0] poll_cnt;
    logic              poll_tick;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            poll_cnt  <= '0;
            poll_tick <= 1'b0;
        end else begin
            poll_tick <= 1'b0;
            if (ce_1khz) begin
                if (poll_cnt == POLL_W'(POLL_PERIOD - 1)) begin
                    poll_cnt  <= '0;
                    poll_tick <= 1'b1;
                end else begin
                    poll_cnt <= poll_cnt + 1;
                end
            end
        end
    end

    // =========================================================================
    // Polling FSM: cycles through 3 RW chips per poll trigger
    //   S_IDLE  : wait for poll_tick (and !safe_mode)
    //   S_START : assert tx_valid to begin SPI transfer for current wheel
    //   S_WAIT  : wait for SPI rx_valid; capture fault; advance wheel index
    // =========================================================================
    typedef enum logic [1:0] {
        S_IDLE,
        S_START,
        S_WAIT
    } poll_state_t;

    poll_state_t         poll_state;
    logic [1:0]          wheel_idx;         // currently active wheel (0, 1, 2)
    logic [2:0]          spi_fault;         // per-axis fault from last SPI read
    logic signed [15:0]  torque_latch [0:2]; // snapshot at poll boundary

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            poll_state   <= S_IDLE;
            wheel_idx    <= 2'd0;
            spi_tx_valid <= 1'b0;
            spi_tx_data  <= '0;
            spi_fault    <= 3'b000;
            for (int i = 0; i < 3; i++) torque_latch[i] <= '0;
        end else begin
            spi_tx_valid <= 1'b0;   // default: de-asserted each cycle

            case (poll_state)
                // -----------------------------------------------------------------
                // Wait for 10 Hz poll trigger; skip if safe_mode is active
                // -----------------------------------------------------------------
                S_IDLE: begin
                    if (poll_tick && !safe_mode) begin
                        // Snapshot torque commands at poll boundary for stable read
                        for (int i = 0; i < 3; i++)
                            torque_latch[i] <= torque_cmd[i];
                        wheel_idx  <= 2'd0;
                        poll_state <= S_START;
                    end
                end

                // -----------------------------------------------------------------
                // Assert tx_valid once SPI master is ready (IDLE state)
                // -----------------------------------------------------------------
                S_START: begin
                    if (spi_tx_ready) begin
                        spi_tx_data  <= torque_latch[wheel_idx];
                        spi_tx_valid <= 1'b1;
                        poll_state   <= S_WAIT;
                    end
                end

                // -----------------------------------------------------------------
                // Wait for SPI completion; capture per-axis fault bit
                // -----------------------------------------------------------------
                S_WAIT: begin
                    if (spi_rx_valid) begin
                        // SPI rx_data[15] = motor driver fault indicator
                        spi_fault[wheel_idx] <= spi_rx_data[15];

                        if (wheel_idx == 2'd2) begin
                            // All 3 wheels serviced - return to idle
                            poll_state <= S_IDLE;
                        end else begin
                            wheel_idx  <= wheel_idx + 1;
                            poll_state <= S_START;
                        end
                    end
                end

                default: poll_state <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Internal watchdog: fires if cmd_valid absent for FAULT_TIMEOUT ce_1khz ticks
    // Mirrors rw_driver.sv watchdog but combined with SPI fault for per-axis output
    // =========================================================================
    localparam int WDOG_W = $clog2(FAULT_TIMEOUT + 1);

    logic [WDOG_W-1:0] wdog_cnt;
    logic              wdog_fault;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdog_cnt   <= '0;
            wdog_fault <= 1'b0;
        end else begin
            if (cmd_valid) begin
                wdog_cnt   <= '0;
                wdog_fault <= 1'b0;
            end else if (ce_1khz) begin
                if (wdog_cnt == WDOG_W'(FAULT_TIMEOUT)) begin
                    wdog_fault <= 1'b1;
                end else begin
                    wdog_cnt <= wdog_cnt + 1;
                end
            end
        end
    end

    // =========================================================================
    // SPI signal routing
    //   rw_sclk      - shared SPI clock from master
    //   rw_mosi[2:0] - same MOSI data broadcast to all axes; CS_N selects target
    //   rw_cs_n[i]   - only asserted for currently active wheel during transfer
    //   spi_miso     - muxed from selected wheel's MISO line
    // =========================================================================
    assign rw_sclk = spi_sclk;
    assign rw_mosi = {3{spi_mosi}};  // broadcast; only active CS decodes it

    always_comb begin
        rw_cs_n = 3'b111;  // default: all deasserted
        case (wheel_idx)
            2'd0: rw_cs_n[0] = spi_cs_n;
            2'd1: rw_cs_n[1] = spi_cs_n;
            2'd2: rw_cs_n[2] = spi_cs_n;
            default: ;  // wheel_idx values 0-2 are the only valid states; rw_cs_n already defaulted to 3'b111 above
        endcase
    end

    always_comb begin
        case (wheel_idx)
            2'd0:    spi_miso = rw_miso[0];
            2'd1:    spi_miso = rw_miso[1];
            2'd2:    spi_miso = rw_miso[2];
            default: spi_miso = rw_miso[0];
        endcase
    end

    // =========================================================================
    // Combined per-axis fault: internal watchdog (all axes) OR SPI fault (per axis)
    // =========================================================================
    assign rw_fault = {3{wdog_fault}} | spi_fault;

endmodule