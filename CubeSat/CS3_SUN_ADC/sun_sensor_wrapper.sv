// =============================================================================
// Module: sun_sensor_wrapper (CS3 top-level wrapper)
// Subsystem: CS3 - Sun Sensor ADC Interface
// Description: Top-level integration of spi_mux_controller, adc_sequencer,
//              sun_vector_compute, and sun_presence_detector.
//
//              Two clock domains:
//                clk_100mhz - SPI/ADC acquisition (adc_sequencer, mux)
//                sys_clk    - Downstream processing (sun_vector_compute,
//                             sun_presence_detector, age counter)
//
//              CDC:
//                sun_trigger  sys_clk  → clk_100mhz  (2-FF sync + edge detect)
//                adc_valid    clk_100mhz → sys_clk    (2-FF sync + edge detect)
//                adc_ch[0:3]  captured in sys_clk on synchronized adc_valid
//                sun_present  clk_100mhz → sys_clk    (2-FF sync)
//                sun_valid    clk_100mhz → sys_clk    (2-FF sync + edge detect)
//
//              SPI bus sharing: spi_mux_controller arbitrates between the IMU
//              (CS1) passthrough and the sun-sensor ADC (CS3) sequencer.
//              Independent spi_cs_imu_n / spi_cs_adc_n guarantee <0.1% FS
//              crosstalk.
//
//              sun_presence_age_ms: 32-bit millisecond counter that increments
//              every ms when sun is absent, resets to 0 on each valid frame.
//              Runs in sys_clk domain.
//
// Provenance: Architecture/SUBSYSTEM_MODULE_MAPPING.md; cubesat_requirements.md
// =============================================================================
`timescale 1ns/1ps

module sun_sensor_wrapper #(
    parameter int CLK_HZ = 100_000_000,    // clk_100mhz frequency
    parameter int SPI_HZ = 10_000_000      // shared SPI clock (matches IMU)
)(
    // Clock domains
    input  logic        clk_100mhz,        // SPI / ADC domain
    input  logic        sys_clk,           // downstream processing domain
    input  logic        rst_n,

    // Measurement trigger (sys_clk domain)
    input  logic        sun_trigger,

    // -------------------------------------------------------------------------
    // IMU SPI passthrough (clk_100mhz domain)
    // Connect to CS1 IMU controller outputs / inputs
    // -------------------------------------------------------------------------
    input  logic        imu_spi_req,       // IMU requesting bus
    input  logic        imu_sclk,          // IMU-generated SCLK
    input  logic        imu_mosi,          // IMU MOSI
    input  logic        imu_cs_n,          // IMU CS_N
    output logic        imu_grant,         // Grant back to IMU controller
    output logic        imu_miso,          // MISO routed to IMU (= spi_miso)

    // -------------------------------------------------------------------------
    // Shared physical SPI bus (connect to FPGA top-level I/O)
    // -------------------------------------------------------------------------
    output logic        spi_sclk,
    output logic        spi_mosi,
    input  logic        spi_miso,
    output logic        spi_cs_imu_n,      // CS for IMU device (active low)
    output logic        spi_cs_adc_n,      // CS for ADC device (active low)

    // -------------------------------------------------------------------------
    // Sun sensor outputs (sys_clk domain)
    // -------------------------------------------------------------------------
    output logic [11:0] sun_channel [0:3], // raw 12-bit photodiode readings
    output logic signed [15:0] sun_alpha,  // Q15 azimuth  = (ch0-ch1)/(ch0+ch1)
    output logic signed [15:0] sun_beta,   // Q15 elevation = (ch2-ch3)/(ch2+ch3)
    output logic        sun_valid,         // one-cycle strobe: outputs updated
    output logic        sun_present,       // any channel above threshold
    output logic [31:0] sun_presence_age_ms, // ms since last valid measurement
    output logic        sun_busy,          // acquisition in progress
    output logic        sun_fault          // SPI fault
);

    // =========================================================================
    // CDC: sun_trigger (sys_clk → clk_100mhz)
    // Use 2-FF synchronizer then rising-edge detect to produce a one-cycle
    // pulse in the clk_100mhz domain.
    // =========================================================================
    logic trig_sync_100m;
    logic trig_prev_100m;
    logic adc_trigger_100m;

    synchronizer #(.STAGES(2), .WIDTH(1)) u_trig_cdc (
        .dst_clk  (clk_100mhz),
        .async_in (sun_trigger),
        .sync_out (trig_sync_100m)
    );

    always_ff @(posedge clk_100mhz or negedge rst_n) begin
        if (!rst_n) trig_prev_100m <= 1'b0;
        else        trig_prev_100m <= trig_sync_100m;
    end

    assign adc_trigger_100m = trig_sync_100m & ~trig_prev_100m; // rising-edge pulse

    // =========================================================================
    // Internal wires (clk_100mhz domain)
    // =========================================================================
    logic [11:0] adc_ch_100m [0:3];
    logic        adc_valid_100m;
    logic        adc_busy_100m;
    logic        adc_fault_100m;

    // ADC sequencer ↔ SPI mux controller
    logic adc_req_w;
    logic adc_grant_w;
    logic adc_sclk_w;
    logic adc_mosi_w;
    logic adc_miso_w;
    logic adc_cs_n_w;

    // =========================================================================
    // SPI Mux Controller
    // =========================================================================
    spi_mux_controller #(
        .CLK_HZ (CLK_HZ),
        .SPI_HZ (SPI_HZ)
    ) u_mux (
        .clk         (clk_100mhz),
        .rst_n       (rst_n),
        // IMU side
        .imu_req     (imu_spi_req),
        .imu_sclk    (imu_sclk),
        .imu_mosi    (imu_mosi),
        .imu_cs_n    (imu_cs_n),
        .imu_grant   (imu_grant),
        .imu_miso    (imu_miso),
        // ADC side
        .adc_req     (adc_req_w),
        .adc_mosi    (adc_mosi_w),
        .adc_cs_n    (adc_cs_n_w),
        .adc_sclk    (adc_sclk_w),
        .adc_grant   (adc_grant_w),
        .adc_miso    (adc_miso_w),
        // Physical bus
        .spi_sclk    (spi_sclk),
        .spi_mosi    (spi_mosi),
        .spi_miso    (spi_miso),
        .spi_cs_imu_n(spi_cs_imu_n),
        .spi_cs_adc_n(spi_cs_adc_n)
    );

    // =========================================================================
    // ADC Sequencer (runs in clk_100mhz domain)
    // =========================================================================
    adc_sequencer #(
        .CLK_HZ (CLK_HZ),
        .NUM_CH (4)
    ) u_adc_seq (
        .clk         (clk_100mhz),
        .rst_n       (rst_n),
        .adc_trigger (adc_trigger_100m),
        .spi_sclk    (adc_sclk_w),
        .spi_mosi    (adc_mosi_w),
        .spi_miso    (adc_miso_w),
        .spi_cs_adc_n(adc_cs_n_w),
        .spi_req     (adc_req_w),
        .spi_grant   (adc_grant_w),
        .adc_ch      (adc_ch_100m),
        .adc_valid   (adc_valid_100m),
        .adc_busy    (adc_busy_100m),
        .adc_fault   (adc_fault_100m)
    );

    // =========================================================================
    // CDC: adc_valid (clk_100mhz → sys_clk)
    // Synchronize the valid strobe; use rising-edge detect in sys_clk domain.
    // =========================================================================
    logic adc_valid_sync;
    logic adc_valid_prev_sys;
    logic adc_valid_pulse_sys;   // one-cycle pulse in sys_clk

    synchronizer #(.STAGES(2), .WIDTH(1)) u_valid_cdc (
        .dst_clk  (sys_clk),
        .async_in (adc_valid_100m),
        .sync_out (adc_valid_sync)
    );

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) adc_valid_prev_sys <= 1'b0;
        else        adc_valid_prev_sys <= adc_valid_sync;
    end

    assign adc_valid_pulse_sys = adc_valid_sync & ~adc_valid_prev_sys;

    // =========================================================================
    // CDC: adc_ch[0:3] capture (clk_100mhz → sys_clk)
    // adc_ch_100m is stable for the entire measurement epoch (held until the
    // next acquisition).  Capture it in sys_clk on adc_valid_pulse_sys.
    // =========================================================================
    logic [11:0] adc_ch_sys [0:3];

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 4; i++) adc_ch_sys[i] <= '0;
        end else if (adc_valid_pulse_sys) begin
            for (int i = 0; i < 4; i++) adc_ch_sys[i] <= adc_ch_100m[i];
        end
    end

    assign sun_channel = adc_ch_sys;

    // =========================================================================
    // CDC: adc_busy / adc_fault (clk_100mhz → sys_clk)
    // =========================================================================
    logic busy_sync_sys;
    logic fault_sync_sys;

    synchronizer #(.STAGES(2), .WIDTH(1)) u_busy_cdc (
        .dst_clk  (sys_clk),
        .async_in (adc_busy_100m),
        .sync_out (busy_sync_sys)
    );

    synchronizer #(.STAGES(2), .WIDTH(1)) u_fault_cdc (
        .dst_clk  (sys_clk),
        .async_in (adc_fault_100m),
        .sync_out (fault_sync_sys)
    );

    assign sun_busy  = busy_sync_sys;
    assign sun_fault = fault_sync_sys;

    // =========================================================================
    // Sun Vector Compute (sys_clk domain)
    // =========================================================================
    logic        sv_cal_valid_sys;
    logic signed [15:0] sv_alpha_sys;
    logic signed [15:0] sv_beta_sys;
    logic        sv_present_sys;

    sun_vector_compute u_sun_vec (
        .clk         (sys_clk),
        .rst_n       (rst_n),
        .ch          (adc_ch_sys),
        .data_valid  (adc_valid_pulse_sys),
        .sun_alpha   (sv_alpha_sys),
        .sun_beta    (sv_beta_sys),
        .sun_present (sv_present_sys),
        .cal_valid   (sv_cal_valid_sys)
    );

    assign sun_alpha = sv_alpha_sys;
    assign sun_beta  = sv_beta_sys;
    assign sun_valid = sv_cal_valid_sys;

    // =========================================================================
    // Sun Presence Detector (sys_clk domain, 4-channel, threshold = 409)
    // Instantiated for per-channel fault masking; sun_present is taken from
    // this detector (consistent threshold with sun_vector_compute).
    // =========================================================================
    logic [3:0] spd_ch_mask;
    logic [1:0] spd_dominant;

    sun_presence_detector #(
        .NUM_CH     (4),
        .SUN_THRESH (409)
    ) u_spd (
        .clk         (sys_clk),
        .rst_n       (rst_n),
        .adc_ch      (adc_ch_sys),
        .data_valid  (adc_valid_pulse_sys),
        .sun_present (sun_present),
        .sun_ch_mask (spd_ch_mask),
        .dominant_ch (spd_dominant)
    );

    // =========================================================================
    // Sun Presence Age Counter (sys_clk domain)
    // Millisecond tick generator: 1 kHz from sys_clk (CLK_HZ counts per ms).
    // Counter resets to 0 whenever a valid frame arrives.
    // Counter saturates at 32'hFFFF_FFFF (no wrap-around).
    // =========================================================================
    localparam int MS_TICKS = CLK_HZ / 1000; // clk cycles per 1 ms

    logic [$clog2(MS_TICKS)-1:0] ms_tick_cnt;
    logic                         ms_tick;

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            ms_tick_cnt <= '0;
            ms_tick     <= 1'b0;
        end else begin
            ms_tick <= 1'b0;
            if (ms_tick_cnt == $clog2(MS_TICKS)'(MS_TICKS - 1)) begin
                ms_tick_cnt <= '0;
                ms_tick     <= 1'b1;
            end else begin
                ms_tick_cnt <= ms_tick_cnt + 1'b1;
            end
        end
    end

    always_ff @(posedge sys_clk or negedge rst_n) begin
        if (!rst_n) begin
            sun_presence_age_ms <= '0;
        end else if (adc_valid_pulse_sys) begin
            // Valid frame received - reset age
            sun_presence_age_ms <= '0;
        end else if (ms_tick && !sun_present && sun_presence_age_ms != 32'hFFFF_FFFF) begin
            // Increment while sun is absent (saturate at max)
            sun_presence_age_ms <= sun_presence_age_ms + 32'd1;
        end
    end

endmodule