# CS2 — Magnetometer Interface (I2C)

## 1. Module Title & ID

**Module:** CS2 — Magnetometer Interface (I2C)
**Subsystem ID:** CS2
**Requirement:** CS-ADCS-002

---

## 2. Overview

CS2 provides the FPGA interface to a 3-axis magnetometer (HMC5883L / IST8310) via I2C at 400 kHz Fast Mode. It performs a burst register read (6 consecutive bytes via auto-increment) to retrieve X, Y, Z field data, assembles signed 16-bit words, and applies hard-iron and soft-iron calibration to yield Q15 fixed-point outputs. A dedicated fault detector monitors for saturation, stuck values, and noise transients. An age counter tracks milliseconds since the last valid frame.

**Target Platform:** Xilinx Artix-7 XC7A35T

---

## 3. Criticality

**CRITICAL** — CS-ADCS-002. Magnetometer data is the primary attitude reference measurement for the EKF (CS5) and the magnetic-dipole actuation model (CS7). Loss of magnetometer data for > 5 ms triggers a FAULT flag in CS8.

---

## 4. Key Functionality

- Drives an I2C master at 400 kHz (Fast Mode); configurable via `I2C_HZ` parameter.
- Burst-reads 6 consecutive bytes (X_H, X_L, Z_H, Z_L, Y_H, Y_L per HMC5883L / IST8310 map) starting from register 0x03.
- Assembles raw 16-bit signed values; total transaction ≈ 350 µs (within ≤ 1 ms per 100 Hz cycle).
- Applies hard-iron offset subtraction and scale normalization in `mag_calibration.sv` (DSP48-free).
- Outputs both Q15 normalised values (`mag_data[0:2]`) and µT-scaled values (`mag_ut[0:2]`).
- `mag_fault_detector` monitors saturation, stuck-value (> configurable timeout), and noise burst conditions.
- `mag_age_ms` 32-bit counter increments every ms; resets to 0 on each valid frame.
- NACK detection: I2C NACK within one clock period sets `mag_fault`; clears on next `mag_read_trigger`.

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `sys_clk` | input | 1 | `sys_clk` | System clock (100 MHz) |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `mag_read_trigger` | input | 1 | `sys_clk` | One-cycle pulse to start a 3-axis read (connect to 100 Hz CE) |
| `i2c_sda` | inout | 1 | `sys_clk` | Open-drain SDA; requires 4.7 kΩ pull-up |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `i2c_scl` | output | 1 | `sys_clk` | I2C clock (up to 400 kHz) |
| `mag_data[0]` | output | 16 | `sys_clk` | Calibrated X-field, signed Q15 |
| `mag_data[1]` | output | 16 | `sys_clk` | Calibrated Y-field, signed Q15 |
| `mag_data[2]` | output | 16 | `sys_clk` | Calibrated Z-field, signed Q15 |
| `mag_ut[0:2]` | output | 16 each | `sys_clk` | µT-scaled X/Y/Z (bit-shift approximation) |
| `mag_valid` | output | 1 | `sys_clk` | One-cycle strobe — calibrated data updated |
| `mag_busy` | output | 1 | `sys_clk` | I2C transaction in progress |
| `mag_fault` | output | 1 | `sys_clk` | I2C NACK, saturation, stuck-value, or noise fault |
| `mag_age_ms` | output | 32 | `sys_clk` | Milliseconds since last valid frame (saturates at 0xFFFF_FFFF) |

---

## 7. Architecture

```
mag_read_trigger
      │
      ▼
┌───────────────────────┐  raw_x/y/z (16-bit)  ┌────────────────────┐
│  i2c_mag_controller   │──────────────────────▶│  mag_calibration   │
│  (I2C sequencer,      │  ctrl_data_valid       │  (hard-iron offset │
│   burst read 6 B)     │  ctrl_fault            │   + µT conversion) │
│  i2c_master (reused)  │                        └────────┬───────────┘
└───────────────────────┘                                 │ mag_cal_x/y/z (Q15)
                                                          │ mag_valid
                                              ┌───────────▼───────────────┐
                                              │  mag_fault_detector       │
                                              │  (saturation, stuck-value,│
                                              │   noise burst monitor)    │
                                              └───────────────────────────┘
                                                          │
                                                     mag_fault (combined)
                                         ┌────────────────────────────────┐
                                         │  Age Counter (ms tick, 1 kHz)  │
                                         │  resets on mag_valid           │
                                         └────────────────────────────────┘
```

**Reused Helper IPs (from `CubeSat/`):**
- `i2c_master.sv` — I2C protocol engine (ADDR_ACK, DATA_ACK, STOP each run 4 SCL ticks)
- `debouncer.sv` — Input line debouncing for SDA
- `synchronizer.sv` — CDC synchroniser for fault signals

---

## 8. Data Formats

| Signal Group | Format | Notes |
|---|---|---|
| `mag_data[0:2]` | Q15 signed (16-bit) | Normalised magnetic field; 1.0 = full scale |
| `mag_ut[0:2]` | Q15 signed (16-bit) | µT-scaled via bit-shift; no DSP48 required |
| Raw ADC (internal) | 16-bit two's complement | 13-bit ADC output zero-padded to 16 bits |
| `mag_age_ms` | 32-bit unsigned | Millisecond count; saturates at 0xFFFF_FFFF |

---

## 9. Register Interface

CS2 has **no AXI4-Lite register interface** in the current implementation. Hard-iron calibration constants are compile-time `localparam` values in `mag_calibration.sv`. For in-orbit calibration updates these should be promoted to AXI4-Lite writable registers.

**Parameters (synthesised):**

| Parameter | Default | Description |
|---|---|---|
| `CLK_HZ` | 100_000_000 | System clock in Hz |
| `I2C_HZ` | 400_000 | I2C Fast Mode clock in Hz |
| `MAG_ADDR` | `7'h0E` | 7-bit I2C slave address (HMC5883L / IST8310 default) |

---

## 10. File Structure

```
CubeSat/CS2_MAG_I2C/
├── i2c_mag_wrapper.sv       ← Top-level wrapper; CS12 integration point
├── i2c_mag_controller.sv    ← I2C read sequencer; burst-reads 6 bytes
├── mag_calibration.sv       ← Hard-iron offset + µT conversion (DSP48-free)
├── mag_fault_detector.sv    ← Saturation / stuck-value / noise monitor
├── tb_i2c_mag_wrapper.sv    ← Integration testbench
└── README.md                ← This file

CubeSat/ (shared helper IPs used by CS2):
├── i2c_master.sv            ← I2C master protocol engine
├── debouncer.sv             ← Input debouncer
└── synchronizer.sv          ← 2-FF CDC synchroniser
```

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `mag_read_trigger` | CS2 ← CS12 | `clk_manager` (CS12) | 100 Hz read trigger |
| `mag_data[0:2]` | CS2 → CS5 | `ekf_wrapper` (CS5) | Magnetometer measurement for EKF update step |
| `mag_valid` | CS2 → CS5 | `ekf_wrapper` (CS5) | Measurement-valid handshake |
| `mag_valid` | CS2 → CS8 | `adcs_fsm_wrapper` (CS8) | Health heartbeat |
| `mag_fault` | CS2 → CS8 | `adcs_fsm_wrapper` (CS8) | Fault reporting |
| `mag_data[0:2]` | CS2 → CS12 | `top_cubesat_mvp` (CS12) | Telemetry packing for CS11 |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- Burst I2C transaction: 6 bytes × 9 bits / 400 kHz ≈ 135 µs. Well within ≤ 1 ms cycle budget.
- Two separate CE generators (1 kHz for age counter, 100 Hz for fault detector) are embedded in the wrapper.

**Resource:**
- `i2c_mag_controller` FSM: estimated < 60 LUTs, 0 DSP48E1, 0 BRAM.
- `mag_calibration`: bit-shift operations only; 0 DSP48E1.
- `mag_fault_detector`: counter + comparator logic; < 30 LUTs.

**Power:**
- I2C SCL clock runs only during active transactions; minimal switching activity between acquisitions.

**Optimization Opportunities:**
1. Promote hard-iron and scale constants to AXI4-Lite registers for in-orbit calibration.
2. Add soft-iron (cross-axis) correction matrix (3×3 Q15 multiply) — requires 9 DSP48E1 cycles.
3. Optionally pipeline `mag_fault_detector` to reduce combinational depth.

**Timing:**
- I2C SDA/SCL are open-drain; external pull-up value must be chosen for 400 kHz rise time.
- No CDC required (single `sys_clk` domain throughout).

---

## 13. Testing & Verification

**Testbenches:**
- `CubeSat/CS2_MAG_I2C/tb_i2c_mag_wrapper.sv` — wrapper integration test
- `CubeSat/CS2_MAG_I2C/tb_i2c_mag.sv` (legacy) — I2C controller unit test

**Test Scenarios:**
- Drive `mag_read_trigger` at 100 Hz; verify `mag_valid` and all three axes update each cycle.
- Simulate I2C NACK on second byte; verify `mag_fault` asserts and `mag_valid` does not pulse.
- Apply saturating raw values; verify `fault_sat` asserts within `mag_fault_detector`.
- Hold raw values constant for > stuck-value timeout; verify `fault_stuck` asserts.
- Verify `mag_age_ms` increments correctly and resets to 0 on `mag_valid`.

**Simulation Notes:**
- Use an I2C slave model that responds with known HMC5883L register sequences.
- Compile with `iverilog -g2012` including `i2c_master.sv`, `debouncer.sv`, `synchronizer.sv`.
- Timescale: 1 ns / 1 ps.

**Requirements Coverage:**
- CS-ADCS-002: I2C @ 400 kHz, ≤ 150 µs for 3 axes, ACK received within 1 clock period.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
