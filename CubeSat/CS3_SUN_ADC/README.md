# CS3 — Sun Sensor ADC Interface

> Acquires 4-channel photodiode signals via a 12-bit SPI ADC shared with the IMU bus, computes normalised sun-vector angles (α, β) in Q15, and asserts a presence flag when any channel exceeds the configurable threshold.

---

## Overview

| Attribute | Value |
|---|---|
| Requirement | CS-ADCS-003 |
| Top module | `sun_sensor_wrapper` |
| Clock domain | `clk_100mhz` (SPI/ADC) → `sys_clk` (processing) |
| CE strobe | 100 Hz `sun_trigger` from CS12 |
| SPI frequency | 10 MHz (shared with IMU — arbitrated by `spi_mux_controller`) |
| Channel budget | 4 channels within 10 ms |
| ADC resolution | 12 bits (0 – 4095 counts) |
| Sun threshold | 409 counts (≈ 10 % FS, configurable via `SUN_THRESH` parameter) |
| BRAM | 64 B |
| DSP48 | 0 |

---

## File Structure

| File | Purpose |
|---|---|
| `sun_sensor_wrapper.sv` | Top-level — integrates all sub-modules, owns CDC |
| `spi_mux_controller.sv` | Arbitrates shared SPI bus between IMU (CS1) and ADC (CS3) |
| `adc_sequencer.sv` | 4-channel SPI ADC read sequencer with per-channel CS_N |
| `sun_vector_compute.sv` | Computes α = (ch0−ch1)/(ch0+ch1), β = (ch2−ch3)/(ch2+ch3) in Q15 |
| `sun_presence_detector.sv` | Per-channel threshold compare; asserts `sun_present` |
| `tb_sun_sensor_wrapper.sv` | Self-checking directed testbench |

Shared primitives used: `spi_master.sv`, `adc_interface.sv`, `synchronizer.sv`, `edge_detect.sv`.

---

## Module Interface

```systemverilog
module sun_sensor_wrapper #(
    parameter int CLK_HZ = 100_000_000,
    parameter int SPI_HZ = 10_000_000
)(
    input  logic        clk_100mhz,
    input  logic        sys_clk,
    input  logic        rst_n,
    input  logic        sun_trigger,          // 100 Hz CE (sys_clk domain)

    // IMU SPI passthrough (arbitration)
    input  logic        imu_spi_req,
    input  logic        imu_sclk, imu_mosi, imu_cs_n,
    output logic        imu_grant,
    output logic        imu_miso,

    // Shared physical SPI bus
    output logic        spi_sclk, spi_mosi,
    input  logic        spi_miso,
    output logic        spi_cs_imu_n,         // IMU chip-select
    output logic        spi_cs_adc_n,         // ADC chip-select

    // Outputs (sys_clk domain)
    output logic [11:0] sun_channel [0:3],    // raw 12-bit per channel
    output logic signed [15:0] sun_alpha,     // Q15 azimuth ratio
    output logic signed [15:0] sun_beta,      // Q15 elevation ratio
    output logic        sun_valid,            // one-cycle strobe: outputs updated
    output logic        sun_present,          // any channel > SUN_THRESH
    output logic [31:0] sun_presence_age_ms,  // ms since last valid measurement
    output logic        sun_busy,
    output logic        sun_fault
);
```

---

## Functionality

1. **Trigger CDC** — `sun_trigger` (sys_clk) is synchronised into `clk_100mhz` domain via 2-FF synchroniser + rising-edge detect.
2. **Bus arbitration** — `spi_mux_controller` grants the shared SPI bus to the ADC sequencer; IMU passthrough is held off during acquisition.
3. **ADC acquisition** — `adc_sequencer` drives `spi_cs_adc_n` low for each of the 4 channels, clocking 12-bit results. Total sweep < 10 ms.
4. **CDC data capture** — `adc_valid` pulse synchronised back to `sys_clk`; `adc_ch[0:3]` captured in sys_clk domain on the pulse (data stable until next acquisition).
5. **Vector compute** — `sun_vector_compute` computes differential ratios α and β in Q15 using integer division.
6. **Presence detection** — `sun_presence_detector` compares each channel against `SUN_THRESH = 409`; asserts `sun_present` if any exceed threshold.
7. **Age counter** — 32-bit ms counter increments when `sun_present` is false; resets to 0 on each valid frame.

---

## Simulation Instructions

```bash
iverilog -g2012 -o sim_cs3 \
  CS3_SUN_ADC/tb_sun_sensor_wrapper.sv \
  CS3_SUN_ADC/sun_sensor_wrapper.sv \
  CS3_SUN_ADC/spi_mux_controller.sv \
  CS3_SUN_ADC/adc_sequencer.sv \
  CS3_SUN_ADC/sun_vector_compute.sv \
  CS3_SUN_ADC/sun_presence_detector.sv \
  spi_master.sv synchronizer.sv edge_detect.sv adc_interface.sv

vvp sim_cs3
```

```tcl
vlog -sv CS3_SUN_ADC/tb_sun_sensor_wrapper.sv CS3_SUN_ADC/*.sv spi_master.sv synchronizer.sv adc_interface.sv edge_detect.sv
vsim -t 1ps tb_sun_sensor_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 100 MHz `clk_100mhz`, 50 MHz `sys_clk` |
| Stimulus | Procedural SPI-ADC MISO model returning configurable 12-bit values per channel |
| Checking | Asserts on α/β mismatch; verifies `sun_present` logic against threshold |
| Coverage | Nominal 4-channel sweep, presence threshold boundary, bus arbitration with IMU, age counter |

---

## Expected Behaviour

```
sun_trigger (sys_clk)   __|‾|________________________________________
adc_sequencer (100m):      bus grant → 4× SPI reads (~N µs each)
adc_valid (clk_100mhz): ______________________________|‾|___________
adc_valid (sys_clk CDC):__________________________________|‾|_________
sun_valid:              ____________________________________|‾|_______
sun_alpha / sun_beta:   ====================================|STABLE==
sun_present:            _____________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾______  (when any ch > 409)
```

---

## Limitations

- Threshold (`SUN_THRESH`) is a compile-time parameter; no runtime command path in MVP.
- Photodiode transfer function assumed linear with no temperature model.
- No per-channel gain/offset calibration; systematic bias accumulates with component tolerance.
- `sun_vector_compute` uses integer division; resolution degrades when sum `(ch0+ch1)` is small.

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] 4-channel ADC sweep verified; raw values match MISO stimulus
- [x] Sun-vector ratio (α, β) within ±1 LSB of expected Q15 value
- [x] `sun_present` threshold boundary verified
- [x] CDC path (trigger and valid) confirmed across clock domains
- [x] SPI bus arbitration with IMU passthrough tested
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] Per-channel calibration register interface
- [ ] Synthesis timing closure in Vivado
