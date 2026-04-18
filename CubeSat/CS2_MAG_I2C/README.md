# CS2 — Magnetometer Interface (I2C)

> Reads 3-axis magnetic field data from an HMC5883L / IST8310 magnetometer via I2C @ 400 kHz Fast Mode, applies calibration, and delivers Q15 vectors plus µT-scaled output to the EKF at 100 Hz.

---

## Overview

| Attribute | Value |
|---|---|
| Requirement | CS-ADCS-002 |
| Top module | `i2c_mag_wrapper` |
| Clock domain | `sys_clk` (50 MHz) |
| CE strobe | 100 Hz `mag_read_trigger` from CS12 |
| I2C frequency | 400 kHz Fast Mode (`I2C_HZ` parameter) |
| Transaction budget | ≤1 ms (6-byte burst ≈ 350 µs @ 400 kHz) |
| BRAM | 128 B |
| DSP48 | 0 |

Issues a single register pointer write to `0x03`, then burst-reads 6 bytes exploiting HMC5883L/IST8310 auto-increment. Detects NACK, saturation, stuck values, and noise transients. Tracks millisecond age of the last valid frame.

---

## File Structure

| File | Purpose |
|---|---|
| `i2c_mag_wrapper.sv` | Top-level wrapper — integrates controller, calibration, fault detector, age counter |
| `i2c_mag_controller.sv` | I2C FSM — START/ADDR/DATA/STOP sequencer; 6-byte burst read |
| `mag_calibration.sv` | Hard-iron offset removal, Q15 scaling, µT conversion |
| `mag_fault_detector.sv` | Saturation, stuck-value, noise-transient detection |
| `tb_i2c_mag_wrapper.sv` | Self-checking directed testbench (wrapper integration) |

Shared primitives used: `i2c_master.sv`, `debouncer.sv`, `synchronizer.sv`.

---

## Module Interface

```systemverilog
module i2c_mag_wrapper #(
    parameter int CLK_HZ   = 100_000_000,
    parameter int I2C_HZ   = 400_000,
    parameter int MAG_ADDR = 7'h0E        // HMC5883L / IST8310 default
)(
    input  logic        sys_clk,
    input  logic        rst_n,
    input  logic        mag_read_trigger, // 100 Hz CE from CS12

    // I2C physical pins
    inout  wire         i2c_sda,
    output logic        i2c_scl,

    // Calibrated outputs
    output logic signed [15:0] mag_data [0:2],  // Q15: [X, Y, Z]
    output logic signed [15:0] mag_ut   [0:2],  // µT approximation (DSP48-free)

    // Status
    output logic        mag_valid,              // one-cycle strobe @ 100 Hz
    output logic        mag_busy,
    output logic        mag_fault,              // NACK | saturation | stuck | noise
    output logic [31:0] mag_age_ms              // ms since last valid frame
);
```

---

## Functionality

1. **Trigger** — `mag_read_trigger` (100 Hz) starts an I2C burst transaction.
2. **Acquisition** — `i2c_mag_controller` writes register pointer `0x03`, then reads 6 bytes (2 bytes × 3 axes) in one bus transaction (~350 µs). NACK within 1 I2C clock period latches `fsm_fault`.
3. **Calibration** — `mag_calibration` subtracts hard-iron offsets, scales to Q15, and produces a DSP48-free µT approximation via bit-shift coefficients.
4. **Fault detection** — `mag_fault_detector` monitors for:
   - **Saturation:** any axis at ±FS for 1 cycle
   - **Stuck value:** identical reading for >10 consecutive 100 Hz cycles
   - **Noise transient:** sample-to-sample delta exceeds threshold
5. **Age counter** — 32-bit millisecond counter increments every ms while `mag_valid` is absent; resets to `0` on each valid frame (saturates at `0xFFFF_FFFF`).

`mag_fault` = `fsm_fault | cal_fault | det_any` — single-bit combined indication.

---

## Simulation Instructions

```bash
iverilog -g2012 -o sim_cs2 \
  CS2_MAG_I2C/tb_i2c_mag_wrapper.sv \
  CS2_MAG_I2C/i2c_mag_wrapper.sv \
  CS2_MAG_I2C/i2c_mag_controller.sv \
  CS2_MAG_I2C/mag_calibration.sv \
  CS2_MAG_I2C/mag_fault_detector.sv \
  i2c_master.sv debouncer.sv synchronizer.sv

vvp sim_cs2
```

```tcl
vlog -sv CS2_MAG_I2C/tb_i2c_mag_wrapper.sv CS2_MAG_I2C/*.sv i2c_master.sv debouncer.sv synchronizer.sv
vsim -t 1ps tb_i2c_mag_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 50 MHz `sys_clk` |
| Stimulus | Procedural I2C slave model; returns known 6-byte payloads |
| Checking | `$error` on calibrated Q15 value mismatch; fault flag assertions |
| Coverage | Nominal read, NACK injection, saturation axis, stuck-value window, age counter roll |

---

## Expected Behaviour

```
mag_read_trigger  __|‾|__________________________________________________
i2c transaction:     START─ADDR─DATA(6B)─STOP  (~350 µs)
mag_valid:        __________________________________|‾|__________________
mag_data[0..2]:   ==================================|==STABLE Q15 XYZ==
mag_fault:        ______________________________________________  (clear on nominal)
mag_age_ms:       000000000000000000000000000000000000|reset to 0 on valid
```

NACK scenario: `mag_fault` asserts on the cycle following the NACK; clears on next `mag_read_trigger`.

---

## Limitations

- Calibration coefficients (hard-iron offsets, scale factors) are compile-time constants — no runtime update path.
- Single I2C master; no redundant sensor path.
- No adaptive bus recovery for intermittent line contention (glitch or bus hang).
- µT conversion uses bit-shift approximation; accuracy is ±5% vs. exact floating-point.

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] Wrapper testbench passes — nominal I2C burst read
- [x] NACK fault injection tested
- [x] Saturation and stuck-value fault paths verified
- [x] Age counter reset-on-valid verified
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] Runtime calibration register interface
- [ ] Synthesis timing closure in Vivado
