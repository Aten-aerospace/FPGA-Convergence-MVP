// =============================================================================
// Module: spi_mux_controller
// Subsystem: CS3 - Sun Sensor ADC Interface
// Description: Arbitrates shared SPI bus between the IMU (CS1) and the
//              sun-sensor ADC (CS3).  The IMU has higher priority.
//
//              A single SCLK is generated from clk_100mhz at SPI_HZ.
//              The SCLK runs only when the ADC has the bus and its CS_N is
//              asserted, ensuring CPOL=0 idle-low operation with no spurious
//              edges on the shared bus.
//
//              Physical SCLK / MOSI are routed from the active master.
//              Each device has its own independent CS_N line, giving <0.1% FS
//              crosstalk because no two CS_N lines are asserted simultaneously.
//
//              IMU passthrough: the IMU controller keeps its own SCLK; this
//              module routes those signals to the physical pins when the IMU
//              holds the grant.
//
// CDC Notes:
//   - All signals are in the clk_100mhz domain.
//   - imu_miso / adc_miso are simply aliases of spi_miso (no logic).
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module spi_mux_controller #(
    parameter int CLK_HZ = 100_000_000,   // clk_100mhz frequency
    parameter int SPI_HZ = 10_000_000     // shared SPI clock frequency
)(
    input  logic clk,       // clk_100mhz
    input  logic rst_n,

    // -------------------------------------------------------------------------
    // IMU side (CS1 IMU controller signals - passthrough)
    // -------------------------------------------------------------------------
    input  logic imu_req,       // IMU requesting bus
    input  logic imu_sclk,      // IMU-generated SCLK
    input  logic imu_mosi,      // IMU MOSI
    input  logic imu_cs_n,      // IMU CS_N (direct from IMU controller)
    output logic imu_grant,     // Grant to IMU
    output logic imu_miso,      // MISO returned to IMU (= spi_miso)

    // -------------------------------------------------------------------------
    // ADC side (adc_sequencer signals)
    // -------------------------------------------------------------------------
    input  logic adc_req,       // ADC requesting bus
    input  logic adc_mosi,      // ADC MOSI
    input  logic adc_cs_n,      // ADC CS_N (from adc_sequencer; drives SCLK gate)
    output logic adc_sclk,      // Gated SCLK provided to adc_sequencer
    output logic adc_grant,     // Grant to ADC
    output logic adc_miso,      // MISO returned to ADC (= spi_miso)

    // -------------------------------------------------------------------------
    // Physical shared SPI bus (connect to FPGA top-level I/O)
    // -------------------------------------------------------------------------
    output logic spi_sclk,      // Shared SCLK to both devices
    output logic spi_mosi,      // Shared MOSI to both devices
    input  logic spi_miso,      // Shared MISO from both devices
    output logic spi_cs_imu_n,  // Chip-select for IMU device
    output logic spi_cs_adc_n   // Chip-select for ADC device
);

    // =========================================================================
    // SCLK generator for ADC (CPOL=0, idle low)
    // Runs only while adc_grant is held AND adc_cs_n is asserted (low).
    // This prevents spurious SCLK edges when the bus is idle.
    // =========================================================================
    localparam int CLK_DIV = CLK_HZ / (2 * SPI_HZ);
    localparam int DIV_W   = $clog2(CLK_DIV > 1 ? CLK_DIV : 2);

    logic [DIV_W-1:0] sclk_cnt;
    logic             sclk_r;        // generated SCLK register
    logic             adc_active;    // ADC holds bus AND CS is asserted

    assign adc_active = adc_grant & ~adc_cs_n;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sclk_cnt <= '0;
            sclk_r   <= 1'b0;
        end else if (!adc_active) begin
            // Reset to idle-low when not in an ADC transaction
            sclk_cnt <= '0;
            sclk_r   <= 1'b0;
        end else begin
            if (sclk_cnt == DIV_W'(CLK_DIV - 1)) begin
                sclk_cnt <= '0;
                sclk_r   <= ~sclk_r;
            end else begin
                sclk_cnt <= sclk_cnt + 1'b1;
            end
        end
    end

    // =========================================================================
    // Arbitration FSM
    // IMU has priority over ADC.  ADC can be politely preempted between
    // SPI frames (when adc_cs_n is high) if IMU asserts imu_req.
    // =========================================================================
    typedef enum logic [1:0] {
        ARB_IDLE,
        ARB_IMU,
        ARB_ADC
    } arb_t;

    arb_t arb_state;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arb_state <= ARB_IDLE;
            imu_grant <= 1'b0;
            adc_grant <= 1'b0;
        end else begin
            case (arb_state)

                ARB_IDLE: begin
                    if (imu_req) begin
                        imu_grant <= 1'b1;
                        arb_state <= ARB_IMU;
                    end else if (adc_req) begin
                        adc_grant <= 1'b1;
                        arb_state <= ARB_ADC;
                    end
                end

                ARB_IMU: begin
                    if (!imu_req) begin
                        imu_grant <= 1'b0;
                        if (adc_req) begin
                            adc_grant <= 1'b1;
                            arb_state <= ARB_ADC;
                        end else begin
                            arb_state <= ARB_IDLE;
                        end
                    end
                end

                ARB_ADC: begin
                    if (!adc_req) begin
                        // ADC released the bus
                        adc_grant <= 1'b0;
                        arb_state <= ARB_IDLE;
                    end else if (imu_req && adc_cs_n) begin
                        // IMU preempts ADC only when no ADC frame is in flight
                        adc_grant <= 1'b0;
                        imu_grant <= 1'b1;
                        arb_state <= ARB_IMU;
                    end
                end

                default: arb_state <= ARB_IDLE;
            endcase
        end
    end

    // =========================================================================
    // SPI bus routing
    // =========================================================================

    // SCLK: IMU provides its own; ADC uses the generated sclk_r
    always_comb begin
        if (imu_grant)
            spi_sclk = imu_sclk;
        else if (adc_grant)
            spi_sclk = sclk_r;
        else
            spi_sclk = 1'b0;
    end

    // MOSI routing
    always_comb begin
        if (imu_grant)
            spi_mosi = imu_mosi;
        else if (adc_grant)
            spi_mosi = adc_mosi;
        else
            spi_mosi = 1'b0;
    end

    // CS_N: gated by grant so the inactive device's CS stays deasserted
    assign spi_cs_imu_n = imu_grant ? imu_cs_n : 1'b1;
    assign spi_cs_adc_n = adc_grant ? adc_cs_n : 1'b1;

    // MISO is shared: both controllers receive it; only the one with active
    // CS_N will obtain valid data.
    assign imu_miso = spi_miso;
    assign adc_miso = spi_miso;

    // SCLK feed-back to adc_sequencer (gated: zero when ADC does not hold grant)
    assign adc_sclk = adc_grant ? sclk_r : 1'b0;

endmodule