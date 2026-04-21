# CS1 — IMU Sensor Interface (SPI)

## 1. Module Title & ID

**Module:** CS1 — IMU Sensor Interface (SPI)
**Subsystem ID:** CS1
**Requirement:** CS-ADCS-001

---

## 2. Overview

CS1 provides the FPGA interface to a 9-axis Inertial Measurement Unit (MPU-9250 / ICM-42688) via SPI at up to 10 MHz SCLK. It sequentially reads accelerometer (3-axis), gyroscope (3-axis), and magnetometer (3-axis) registers, applies compile-time bias removal and saturation, and delivers calibrated Q15 fixed-point values to the ADCS pipeline at 100 Hz. A CRC/parity check flags any corrupted frame. Clock-domain crossing from the SPI acquisition clock (`clk_100mhz`) to the downstream system clock (`sys_clk`) is handled by 2-FF synchronisers.

**Target Platform:** Xilinx Artix-7 XC7A35T (50 MHz system clock; 100 MHz SPI domain)

---

## 3. Criticality

**CRITICAL** — CS-ADCS-001. IMU data is the primary input to the attitude estimation (CS4, CS5) and control (CS6) pipelines. Loss of IMU data for > 5 ms triggers a FAULT transition in CS8.

---

## 4. Key Functionality

- Drives an SPI master at up to 10 MHz SCLK (configurable via `SPI_HZ` parameter).
- Sequentially reads 9 IMU registers per `imu_read_trigger` pulse (one 100 Hz acquisition cycle).
- Converts raw 16-bit two's-complement ADC words to calibrated Q15 signed values via `imu_data_handler`.
- Applies per-axis bias subtraction (compile-time constants; hardened for orbit).
- Saturates output to prevent downstream overflow; sets `imu_overflow` per overrange event.
- Asserts `crc_pass` when the frame passes CRC/parity; ≥ 99.9 % frame-success target.
- Crosses `imu_data_valid` and `crc_pass` from `clk_100mhz` to `sys_clk` via 2-FF synchronisers.
- Holds `imu_busy` high during an active SPI transaction (prevents spurious re-triggers).
- Sets `imu_fault` on SPI timeout or protocol error; clears on next successful transaction.

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `clk_100mhz` | input | 1 | `clk_100mhz` | SPI controller and data-handler clock |
| `sys_clk` | input | 1 | `sys_clk` | Downstream system clock (CDC destination) |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `imu_read_trigger` | input | 1 | `clk_100mhz` | One-cycle pulse to start a 9-axis acquisition (connect to 100 Hz CE) |
| `spi_miso` | input | 1 | `clk_100mhz` | SPI data from IMU |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `spi_sclk` | output | 1 | `clk_100mhz` | SPI clock to IMU (≤ 10 MHz) |
| `spi_mosi` | output | 1 | `clk_100mhz` | SPI data to IMU |
| `spi_cs_n` | output | 1 | `clk_100mhz` | SPI chip select (active low; de-asserted between acquisitions) |
| `accel_x/y/z` | output | 16 each | `sys_clk` | Calibrated acceleration, signed Q15 (16384 LSB/g @ ±2 g) |
| `gyro_x/y/z` | output | 16 each | `sys_clk` | Calibrated angular rate, signed Q15 (131 LSB/°/s @ ±250 °/s) |
| `mag_x/y/z` | output | 16 each | `sys_clk` | Calibrated magnetic field, signed Q15 |
| `imu_data_valid` | output | 1 | `sys_clk` | One-cycle strobe — all 9 axes updated (CDC-synchronised) |
| `crc_pass` | output | 1 | `sys_clk` | One-cycle strobe — frame received cleanly (CDC-synchronised) |
| `imu_busy` | output | 1 | `clk_100mhz` | SPI transaction in progress |
| `imu_fault` | output | 1 | `clk_100mhz` | SPI timeout or protocol error |
| `imu_overflow` | output | 1 | `clk_100mhz` | Any axis exceeded calibration range |

---

## 7. Architecture

```
imu_read_trigger
      │
      ▼
┌─────────────────────┐  raw 9×16-bit   ┌──────────────────────┐
│  imu_controller     │─────────────────▶│  imu_data_handler    │
│  (SPI sequencer)    │  raw_valid       │  (bias removal,      │
│  spi_master (reused)│  raw_crc_pass    │   saturation, Q15)   │
└─────────────────────┘                  └──────────┬───────────┘
                                                     │ cal_valid_r (clk_100mhz)
                                                     │ accel/gyro/mag (Q15)
                                         ┌───────────▼───────────┐
                                         │  synchronizer ×2      │
                                         │  (2-FF, clk_100mhz    │
                                         │   → sys_clk)          │
                                         └───────────────────────┘
                                                     │
                                              imu_data_valid, crc_pass
                                              (sys_clk domain)
```

**Reused Helper IPs (from `CubeSat/`):**
- `spi_master.sv` — SPI protocol engine (instantiated inside `imu_controller`)
- `synchronizer.sv` — 2-FF CDC synchroniser (2 instances)
- `tick_gen.sv` — 100 Hz strobe generation (used in `CS12/clk_manager`)
- `edge_detect.sv` — Event detection for trigger pulses

---

## 8. Data Formats

| Signal Group | Format | LSB Weight | Range |
|---|---|---|---|
| `accel_{x,y,z}` | Q15 signed (16-bit) | 16384 LSB/g | ±2 g |
| `gyro_{x,y,z}` | Q15 signed (16-bit) | 131 LSB/°/s | ±250 °/s |
| `mag_{x,y,z}` | Q15 signed (16-bit) | device-dependent | ±4900 µT |
| Raw ADC (internal) | 16-bit two's complement | — | ±32767 |

All downstream consumers (CS4, CS5, CS6) expect Q15 signed 16-bit vectors.

---

## 9. Register Interface

CS1 has **no AXI4-Lite register interface** in the current implementation. Bias offset constants are compile-time `localparam` values in `imu_data_handler.sv`. To support in-orbit calibration updates, these constants should be promoted to AXI4-Lite writable registers within `spi_imu_wrapper` (see Design Considerations).

**Parameters (synthesised):**

| Parameter | Default | Description |
|---|---|---|
| `CLK_HZ` | 100_000_000 | `clk_100mhz` frequency in Hz |
| `SPI_HZ` | 10_000_000 | SPI SCLK frequency in Hz |

---

## 10. File Structure

```
CubeSat/CS1_IMU_SPI/
├── spi_imu_wrapper.sv       ← Top-level wrapper; CS12 integration point
├── imu_controller.sv        ← SPI sequencer; reads 9 registers per trigger
├── imu_data_handler.sv      ← Bias removal, saturation, Q15 conversion
├── tb_spi_imu_wrapper.sv    ← Integration testbench
└── README.md                ← This file

CubeSat/ (shared helper IPs used by CS1):
├── spi_master.sv            ← SPI master engine
├── synchronizer.sv          ← 2-FF CDC synchroniser
├── tick_gen.sv              ← CE strobe generator
└── edge_detect.sv           ← Edge/event detector
```

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `imu_read_trigger` | CS1 ← CS12 | `clk_manager` (CS12) | 100 Hz acquisition trigger |
| `accel_{x,y,z}`, `gyro_{x,y,z}` | CS1 → CS5 | `ekf_wrapper` (CS5) | Sensor fusion inputs |
| `gyro_{x,y,z}` | CS1 → CS4 | `quat_propagator_wrapper` (CS4) | Kinematic propagation |
| `gyro_{x,y,z}`, `omega` | CS1 → CS6 | `pd_control_wrapper` (CS6) | PD derivative term |
| `imu_data_valid` | CS1 → CS8 | `adcs_fsm_wrapper` (CS8) | Health heartbeat |
| `mag_{x,y,z}` | CS1 → CS5 | `ekf_wrapper` (CS5) | Magnetometer measurement |
| SPI bus | CS1 ↔ CS3 | `sun_sensor_wrapper` (CS3) | Shared SPI bus (muxed via CS3 `spi_mux_controller`) |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- SPI transaction budget: 9 registers × 2 bytes × 8 bits / 10 MHz = ~14.4 µs; within the 20 µs spec (CS-ADCS-001).
- 100 Hz update rate leaves 9.98 ms idle between acquisitions; no pipeline stall risk.

**Resource:**
- `imu_controller` is a small FSM; estimated < 80 LUTs, 0 DSP48E1, 0 BRAM.
- `imu_data_handler` uses fixed-point subtract/compare; < 40 LUTs.
- Bias registers are compile-time constants; no BRAM required.

**Power:**
- SPI clock is gated between acquisitions (CS_N held high); dynamic power is minimal.

**Optimization Opportunities:**
1. Promote bias constants to AXI4-Lite registers for in-orbit calibration without re-synthesis.
2. Add FIFO buffering if acquisition rate is raised above 1 kHz to prevent CDC data loss.
3. Consider DMA-style burst read for higher-rate applications (> 1 kHz).

**Timing:**
- Critical path: SPI shift register → data capture FF. Ensure `SPI_HZ ≤ CLK_HZ/4` for correct sampling.
- CDC path (`cal_valid_r → imu_data_valid`) is a single-bit synchroniser; no false-path constraints required beyond standard 2-FF.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS1_IMU_SPI/tb_spi_imu_wrapper.sv`

**Test Scenarios:**
- Drive `imu_read_trigger` at 100 Hz; verify all 9 axes update on each `imu_data_valid` pulse.
- Inject known raw SPI byte sequences; verify correct Q15 calibrated output values.
- Assert `spi_miso` with CRC error pattern; verify `crc_pass` de-asserts and `imu_fault` asserts.
- Apply maximum-range gyro values; verify `imu_overflow` asserts and output is saturated.
- Verify `imu_data_valid` arrives in `sys_clk` domain within 2 clock cycles of `cal_valid_r`.

**Simulation Notes:**
- Compile with `iverilog -g2012` including all shared helper IP files from `CubeSat/`.
- Simulated SPI slave model should return realistic MPU-9250 register data.
- Simulation timestep: 1 ns (`timescale 1ns/1ps`).

**Requirements Coverage:**
- CS-ADCS-001: SPI @ 10 MHz, ≤ 20 µs transaction, ≥ 99.9 % CRC pass rate.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
