# CS1 — IMU Sensor Interface (SPI)

> Acquires 9-axis inertial data (accel / gyro / mag) from an MPU-9250 over SPI @ 10 MHz and delivers calibrated Q15 vectors to the ADCS estimation pipeline at 100 Hz.

---

## Overview

| Attribute | Value |
|---|---|
| Requirement | CS-ADCS-001 |
| Top module | `spi_imu_wrapper` |
| Clock domain | `clk_100mhz` (SPI) → `sys_clk` (downstream) |
| CE strobe | 100 Hz `imu_read_trigger` from CS12 |
| SPI frequency | 10 MHz (`SPI_HZ` parameter) |
| Transaction budget | 20 µs |
| BRAM | 256 B |
| DSP48 | 0 |

Reads 6-byte burst for each sensor register block. Applies compile-time bias removal and saturation detection in `imu_data_handler`. A 2-FF `synchronizer` crosses the `imu_data_valid` strobe into `sys_clk` domain.

---

## File Structure

| File | Purpose |
|---|---|
| `spi_imu_wrapper.sv` | Top-level wrapper — integrates controller, handler, CDC |
| `imu_controller.sv` | SPI FSM — drives SCLK/MOSI/CS_N; decodes 9-axis raw frames |
| `imu_data_handler.sv` | Bias removal, Q15 calibration, saturation / overflow detection |
| `tb_spi_imu_wrapper.sv` | Self-checking directed testbench |

Shared primitives used: `spi_master.sv`, `synchronizer.sv`, `tick_gen.sv`.

---

## Module Interface

```systemverilog
module spi_imu_wrapper #(
    parameter int CLK_HZ = 100_000_000,
    parameter int SPI_HZ = 10_000_000
)(
    input  logic        clk_100mhz,       // SPI controller clock
    input  logic        sys_clk,          // downstream clock (CDC target)
    input  logic        rst_n,            // active-low synchronous reset
    input  logic        imu_read_trigger, // 100 Hz CE strobe from CS12

    // SPI physical pins
    output logic        spi_sclk,
    output logic        spi_mosi,
    input  logic        spi_miso,
    output logic        spi_cs_n,

    // Calibrated 9-axis outputs (Q15, stable between valid pulses)
    output logic signed [15:0] accel_x, accel_y, accel_z,  // 16384 LSB/g
    output logic signed [15:0] gyro_x,  gyro_y,  gyro_z,   // 131 LSB/°/s
    output logic signed [15:0] mag_x,   mag_y,   mag_z,    // µT-scaled

    // Status (sys_clk domain via 2-FF synchronizer)
    output logic        imu_data_valid,   // one-cycle strobe: all 9 axes updated
    output logic        crc_pass,         // frame received without error
    output logic        imu_busy,
    output logic        imu_fault,
    output logic        imu_overflow
);
```

---

## Functionality

1. **Trigger** — `imu_read_trigger` (100 Hz CE) initiates a multi-register SPI burst read.
2. **Acquisition** — `imu_controller` drives CS_N low, clocks 48 bytes across MOSI/MISO at SPI_HZ, asserts `raw_valid` on completion within 20 µs.
3. **Calibration** — `imu_data_handler` subtracts compile-time bias constants, scales to Q15 (gyro: 131 LSB/°/s @ ±250 °/s; accel: 16384 LSB/g @ ±2 g), sets `imu_overflow` if any axis saturates.
4. **CDC** — `synchronizer #(.STAGES(2))` crosses `imu_data_valid` and `crc_pass` from `clk_100mhz` to `sys_clk`. Multi-bit data registers are stable for ~9 ms (100 Hz) — no FIFO required.

**Pipeline latency:** `imu_read_trigger` → `imu_data_valid` ≈ 20 µs SPI + 2 clock cycles CDC.

---

## Simulation Instructions

```bash
# From CubeSat/ directory
iverilog -g2012 -o sim_cs1 \
  CS1_IMU_SPI/tb_spi_imu_wrapper.sv \
  CS1_IMU_SPI/spi_imu_wrapper.sv \
  CS1_IMU_SPI/imu_controller.sv \
  CS1_IMU_SPI/imu_data_handler.sv \
  synchronizer.sv spi_master.sv

vvp sim_cs1
```

```tcl
# QuestaSim
vlog -sv CS1_IMU_SPI/tb_spi_imu_wrapper.sv CS1_IMU_SPI/*.sv synchronizer.sv spi_master.sv
vsim -t 1ps tb_spi_imu_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 100 MHz `clk_100mhz`, 50 MHz `sys_clk` |
| Reset | Active-low, deasserted after 5 cycles |
| Stimulus | Procedural SPI MISO model; injects known 9-axis byte patterns |
| Checking | `$error` on output mismatch vs. expected calibrated values |
| Coverage | Nominal read, CRC error injection, overflow axis, reset mid-transaction |

---

## Expected Behaviour

```
imu_read_trigger  __|‾|_______________________________________________
clk_100mhz domain:  SPI transaction (~20 µs) → raw_valid pulse
sys_clk domain:   ____________________________|‾| imu_data_valid (CDC)
accel_x/y/z:     ====================================|==STABLE Q15==
imu_fault:        _________________________________________  (stays low on clean frames)
```

On `imu_overflow`: the affected axis clamps to `16'sh7FFF` / `16'sh8000` and `imu_overflow` asserts for one cycle.

---

## Limitations

- Bias and scale constants are compile-time parameters; no runtime AXI4-Lite tuning path in MVP.
- No per-axis health confidence score; only binary `imu_fault` / `imu_overflow`.
- Shared SPI bus with CS3 (sun sensor) not yet arbitrated at CS1 level — muxing is handled in `CS3_SUN_ADC/spi_mux_controller.sv`.
- CRC check is a simplified parity — not full MPU-9250 frame CRC.

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] Testbench all tests pass (`vvp sim_cs1`)
- [x] Nominal 9-axis read verified against expected calibrated values
- [x] Overflow detection tested
- [x] Reset mid-transaction verified
- [x] CDC strobe timing confirmed across clock domains
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] Synthesis timing closure in Vivado
- [ ] Per-axis quality metrics for flight qualification

## Mission Context
CS1 is the primary ADCS sensor ingress block. It acquires inertial data used by attitude estimation/control (CS4–CS8), supports orbit/laser mission safety through fault visibility, and feeds telemetry through downstream packing in CS11.

## Requirements Traceability
| REQ_ID | Priority | CS1 Contribution | Acceptance Excerpt |
|---|---|---|---|
| CS-ADCS-001 | CRITICAL | Direct implementation (IMU SPI acquisition) | SPI transaction within 20 µs, ≥99.9% frame integrity |
| CS-ADCS-011 | CRITICAL | Provides heartbeat/overflow/fault inputs used by health monitor (CS8) | Sensor timeout/overflow must raise fault handling path |
| CS-ADCS-012 | HIGH | Source data for ADCS BRAM logging through CS5/CS8 | 100 Hz snapshots include attitude-rate-related sensor content |

## Data Flow & I/O
**Primary chain:** `CS1 -> CS4/CS5/CS8 -> CS6 -> CS7 -> CS11`

| Signal | Dir | Format | Downstream Use |
|---|---|---|---|
| `accel_x/y/z` | out | Q4.12 | EKF measurement update (CS5) |
| `gyro_x/y/z` | out | Q6.10 | Quaternion propagation (CS4), PD damping term (CS6) |
| `mag_x/y/z` | out | Q4.12 | EKF magnetic correction (CS5) |
| `imu_data_valid` | out | pulse @100 Hz | EKF update strobe + CS8 heartbeat |
| `imu_fault`, `imu_overflow` | out | status bits | CS8 fault escalation -> SAFE mode |

## Integration Notes
- Upstream: `clk_manager` trigger (`ce_100hz`) from CS12.
- Downstream dependencies: CS4/CS5 sample on valid strobe; CS8 monitors heartbeat and overflow.
- Safe/fault handling: persistent CS1 faults are expected to propagate via CS8 (`fault_flags`) and force actuator blanking in CS7.
- Hard-coded today: SPI frequency and calibration constants in `imu_data_handler`.
- Needed for in-orbit adaptation: runtime register control for sensor scale, bias, and sample-rate profile.

## Implementation Status
- MVP implemented in `spi_imu_wrapper.sv`, `imu_controller.sv`, `imu_data_handler.sv`.
- Current interface supports required 100 Hz pipeline integration.

## Limitations & Future Work
- Bias/scale are compile-time constants; move to AXI4-Lite runtime tuning.
- No per-axis health confidence output (only fault/overflow); add quality metrics for flight qualification.
- Add explicit parameter table in telemetry uplink guide for on-orbit retuning.

## Performance / Resource Targets (Plan)
| Metric | Target |
|---|---|
| Timing | 20 µs SPI + 2 µs CDC |
| BRAM | 256 B |
| DSP48 | 0 |
