# CS3 — Sun Sensor ADC Interface

## 1. Module Title & ID

**Module:** CS3 — Sun Sensor ADC Interface
**Subsystem ID:** CS3
**Requirement:** CS-ADCS-003

---

## 2. Overview

CS3 provides the FPGA interface to a 4-channel 12-bit SPI ADC (ADS7952-compatible) connected to four photodiode sun sensors. It sequentially reads all four channels, computes a 2D sun-direction vector (azimuth `sun_alpha`, elevation `sun_beta`) via Q15 ratio arithmetic, and asserts a sun-presence flag when any channel exceeds a configurable threshold. The module shares the SPI bus with CS1 (IMU) via a priority mux controller and bridges two clock domains: `clk_100mhz` for SPI/ADC acquisition and `sys_clk` for downstream processing.

**Target Platform:** Xilinx Artix-7 XC7A35T

---

## 3. Criticality

**HIGH** — CS-ADCS-003. Sun-sensor data provides the coarse attitude reference required during SEARCH and COARSE_POINT modes (CS8). Loss of sun-sensor data degrades attitude determination accuracy but does not immediately trigger FAULT.

---

## 4. Key Functionality

- Drives a shared SPI master at 10 MHz SCLK (same bus as CS1 IMU, muxed via `spi_mux_controller`).
- Reads 4 ADC channels per `sun_trigger` pulse; all channels complete within 10 ms.
- Independent `spi_cs_imu_n` and `spi_cs_adc_n` chip-selects guarantee < 0.1 % FS crosstalk.
- Computes `sun_alpha = (ch0 − ch1) / (ch0 + ch1)` and `sun_beta = (ch2 − ch3) / (ch2 + ch3)` in Q15.
- `sun_presence_detector` asserts `sun_present` when any channel exceeds `SUN_THRESH` (default 409 ≈ 10 % FS).
- `sun_presence_age_ms` counter tracks milliseconds since last valid measurement; saturates at 0xFFFF_FFFF.
- Full CDC coverage: trigger (sys_clk → clk_100mhz) and valid strobe (clk_100mhz → sys_clk) via 2-FF synchronisers.
- Multi-bit ADC data captured in sys_clk domain on synchronized valid pulse.

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `clk_100mhz` | input | 1 | `clk_100mhz` | SPI/ADC acquisition clock |
| `sys_clk` | input | 1 | `sys_clk` | Downstream processing clock (CDC destination) |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `sun_trigger` | input | 1 | `sys_clk` | Pulse to start 4-channel acquisition (connect to 100 Hz CE) |
| `imu_spi_req` | input | 1 | `clk_100mhz` | IMU requesting SPI bus (from CS1) |
| `imu_sclk` | input | 1 | `clk_100mhz` | IMU SCLK passthrough |
| `imu_mosi` | input | 1 | `clk_100mhz` | IMU MOSI passthrough |
| `imu_cs_n` | input | 1 | `clk_100mhz` | IMU CS_N passthrough |
| `spi_miso` | input | 1 | `clk_100mhz` | Shared SPI MISO from physical bus |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `spi_sclk` | output | 1 | `clk_100mhz` | Shared SPI clock to physical bus |
| `spi_mosi` | output | 1 | `clk_100mhz` | Shared SPI MOSI to physical bus |
| `spi_cs_imu_n` | output | 1 | `clk_100mhz` | IMU chip-select (active low) |
| `spi_cs_adc_n` | output | 1 | `clk_100mhz` | ADC chip-select (active low) |
| `imu_grant` | output | 1 | `clk_100mhz` | Bus grant back to IMU controller |
| `imu_miso` | output | 1 | `clk_100mhz` | MISO routed to IMU (= spi_miso) |
| `sun_channel[0:3]` | output | 12 each | `sys_clk` | Raw 12-bit photodiode ADC readings |
| `sun_alpha` | output | 16 | `sys_clk` | (ch0−ch1)/(ch0+ch1) in Q15 |
| `sun_beta` | output | 16 | `sys_clk` | (ch2−ch3)/(ch2+ch3) in Q15 |
| `sun_valid` | output | 1 | `sys_clk` | One-cycle strobe — sun_alpha/beta updated |
| `sun_present` | output | 1 | `sys_clk` | Any channel above threshold |
| `sun_presence_age_ms` | output | 32 | `sys_clk` | ms since last valid measurement |
| `sun_busy` | output | 1 | `sys_clk` | Acquisition in progress (CDC-synchronised) |
| `sun_fault` | output | 1 | `sys_clk` | SPI fault (CDC-synchronised) |

---

## 7. Architecture

```
sun_trigger (sys_clk)
      │ CDC (2-FF + edge detect)
      ▼ adc_trigger_100m (clk_100mhz)
┌──────────────────┐  adc_req/grant/sclk/mosi/cs  ┌─────────────────────────┐
│  adc_sequencer   │────────────────────────────▶ │  spi_mux_controller     │
│  (4-ch sequencer,│ adc_ch[0:3] (clk_100mhz)    │  (bus arbiter: IMU vs   │
│   SPI protocol)  │ adc_valid_100m               │   ADC; independent CS_N)│
└──────────────────┘                              └─────────────────────────┘
          │ CDC (2-FF + edge detect)                        │
          ▼ adc_valid_pulse_sys (sys_clk)                   │ spi_sclk/mosi/miso
  adc_ch_sys capture                                        ▼ physical SPI bus
          │
          ├──▶ sun_vector_compute       ──▶ sun_alpha, sun_beta, sun_valid
          │    (Q15 ratio: diff/sum)
          └──▶ sun_presence_detector    ──▶ sun_present
               (threshold compare,
                per-channel mask)
          └──▶ Age Counter (ms tick)    ──▶ sun_presence_age_ms
```

**Reused Helper IPs (from `CubeSat/`):**
- `spi_master.sv` — SPI master engine (used inside `adc_sequencer`)
- `adc_interface.sv` — ADC read protocol (12-bit)
- `synchronizer.sv` — 2-FF CDC synchroniser (multiple instances)
- `edge_detect.sv` — Rising-edge pulse generation for trigger/valid CDC

---

## 8. Data Formats

| Signal | Format | Notes |
|---|---|---|
| `sun_channel[0:3]` | 12-bit unsigned | Raw ADC counts; 0 = dark, 4095 = full-scale |
| `sun_alpha` | Q15 signed (16-bit) | α = (ch0−ch1)/(ch0+ch1); range [−1, +1) |
| `sun_beta` | Q15 signed (16-bit) | β = (ch2−ch3)/(ch2+ch3); range [−1, +1) |
| `sun_presence_age_ms` | 32-bit unsigned | Saturates at 0xFFFF_FFFF |

---

## 9. Register Interface

CS3 has **no AXI4-Lite register interface** in the current implementation. The `SUN_THRESH` value (default 409) is a compile-time parameter in `sun_presence_detector`.

**Parameters (synthesised):**

| Parameter | Default | Description |
|---|---|---|
| `CLK_HZ` | 100_000_000 | `clk_100mhz` frequency in Hz |
| `SPI_HZ` | 10_000_000 | SPI SCLK frequency (matches CS1 IMU) |
| `SUN_THRESH` (in `sun_presence_detector`) | 409 | Minimum ADC count to assert `sun_present` (≈ 10 % FS) |

---

## 10. File Structure

```
CubeSat/CS3_SUN_ADC/
├── sun_sensor_wrapper.sv      ← Top-level wrapper; CS12 integration point
├── spi_mux_controller.sv      ← SPI bus arbiter (IMU vs ADC, independent CS_N)
├── adc_sequencer.sv           ← 4-channel SPI ADC sequencer
├── sun_vector_compute.sv      ← Q15 ratio computation (sun_alpha, sun_beta)
├── sun_presence_detector.sv   ← Threshold comparator + per-channel mask
├── tb_sun_sensor_wrapper.sv   ← Integration testbench with SPI slave model
└── README.md                  ← This file

CubeSat/ (shared helper IPs used by CS3):
├── spi_master.sv              ← SPI master engine
├── adc_interface.sv           ← ADC read protocol (12-bit)
├── synchronizer.sv            ← 2-FF CDC synchroniser
└── edge_detect.sv             ← Rising-edge detector
```

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `sun_trigger` | CS3 ← CS12 | `clk_manager` (CS12) | 100 Hz acquisition trigger |
| `imu_spi_req/sclk/mosi/cs_n` | CS3 ← CS1 | `spi_imu_wrapper` (CS1) | Shared SPI bus passthrough |
| `imu_grant`, `imu_miso` | CS3 → CS1 | `spi_imu_wrapper` (CS1) | Grant and MISO back to IMU |
| `sun_alpha`, `sun_beta`, `sun_valid` | CS3 → CS5 | `ekf_wrapper` (CS5) | Sun-vector measurement for EKF (TBD integration) |
| `sun_present` | CS3 → CS8 | `adcs_fsm_wrapper` (CS8) | Sun-presence health indicator |
| `spi_sclk/mosi/cs_imu_n/cs_adc_n` | CS3 → Physical | FPGA I/O pins | SPI bus to IMU and ADC devices |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- 4-channel acquisition: 4 × SPI frame time + CS_N overhead < 10 ms at 10 MHz SCLK.
- CDC introduces ≤ 2 sys_clk cycles of latency on valid strobe; negligible vs 10 ms budget.

**Resource:**
- `spi_mux_controller`: combinational arbitration; < 20 LUTs, 0 DSP48E1, 0 BRAM.
- `sun_vector_compute`: 15-step binary restoring divider; < 120 LUTs, 0 DSP48E1.
- `sun_presence_detector`: comparator and mask logic; < 30 LUTs.

**Power:**
- SPI bus is gated to idle (CS_N high) between acquisitions.

**Optimization Opportunities:**
1. Promote `SUN_THRESH` to an AXI4-Lite writable register for runtime tuning.
2. If sun-vector accuracy must improve, add per-channel gain correction coefficients.
3. DSP48-based divider can improve latency of `sun_vector_compute` at the cost of resource usage.

**Timing:**
- Shared SPI bus arbitration is combinational; ensure no glitch on SCLK during handover.
- CDC for multi-bit `adc_ch_sys`: stable for full measurement epoch; no FIFO required.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS3_SUN_ADC/tb_sun_sensor_wrapper.sv`

**Test Scenarios:**
- Apply `sun_trigger` at 100 Hz; verify all 4 channels update and `sun_valid` pulses each cycle.
- Feed known ADC values; verify `sun_alpha` and `sun_beta` match expected Q15 ratio.
- Set channel values below `SUN_THRESH`; verify `sun_present` de-asserts.
- Inject SPI fault (no MISO response); verify `sun_fault` asserts and `sun_valid` does not pulse.
- Simultaneously assert `imu_spi_req` and `sun_trigger`; verify bus arbiter grants without collision.
- Verify `sun_presence_age_ms` increments when `sun_present` is low.

**Simulation Notes:**
- Include SPI slave model for ADC and IMU in testbench to exercise arbitration.
- Compile with `iverilog -g2012` including `spi_master.sv`, `adc_interface.sv`, `synchronizer.sv`, `edge_detect.sv`.
- Timescale: 1 ns / 1 ps.

**Requirements Coverage:**
- CS-ADCS-003: 4 channels within 10 ms, ≥ 12 bits, < 0.1 % crosstalk, sun-presence flag.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
