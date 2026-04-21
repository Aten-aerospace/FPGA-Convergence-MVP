# CS10 — Laser Pointing FSM

## 1. Module Title & ID

**Module:** CS10 — Laser Inter-Satellite Link Pointing FSM
**Subsystem ID:** CS10
**Requirements:** CS-LSR-001 through CS-LSR-010

---

## 2. Overview

CS10 controls a 2-axis stepper gimbal for inter-satellite laser link (ISL) pointing. It implements an 8-state FSM that sequences through a boustrophedon SEARCH scan (± 10° az × ± 5° el), spiral ACQUIRE refinement (< 0.1° in ≤ 5 s), closed-loop PD TRACK at 100 Hz (RMS error < 0.05°), full-duplex COMM (ISL modulation active), HOLD (signal dropout recovery), and FAULT/SAFE shutdown. The gimbal stepper driver generates STEP/DIR pulses at up to 200 kHz with trapezoidal acceleration.

**Target Platform:** Xilinx Artix-7 XC7A35T

---

## 3. Criticality

**HIGH** — CS-LSR-001 to CS-LSR-010. CS10 is required for inter-satellite laser communication. Loss of CS10 does not affect attitude control (ADCS) but disables the ISL data path. `safe_mode` from CS8 causes immediate SAFE state entry.

---

## 4. Key Functionality

- **8-State FSM:** IDLE → SEARCH → ACQUIRE → TRACK → HOLD ↔ COMM → FAULT; any state → SAFE on `!laser_enable`.
- **SEARCH:** Boustrophedon raster scan ± 10° az × ± 5° el, ≤ 0.5°/step, via `raster_scan_engine`.
- **ACQUIRE:** Spiral refinement ± 2°; `peak_hold_detector` tracks peak signal; `spiral_refinement` converges < 0.1° in ≤ 5 s (FAULT_TIMEOUT = 100 ticks).
- **TRACK:** Closed-loop PD pointing at 100 Hz using `pid_controller` (reused) with `pointing_error_az/el` inputs; RMS < 0.05°.
- **COMM:** ISL modulation enabled via `laser_modulator`; 8-bit ISL data at 100 Mb/s.
- **HOLD:** Signal lost < HOLD_THRESH; maintains last gimbal position; transitions to FAULT if held > HOLD_TIMEOUT = 20 ticks (200 ms).
- **Signal Monitoring:** `signal_monitor` computes 16-sample rolling average; exposes `signal_strength_filtered`.
- **Fault Handling:** `laser_fault_handler` detects signal loss, gimbal stall, and modulator fault; `fault_code[7:0]` encodes detailed reason.
- **Gimbal PD:** 2 `pid_controller` instances for az and el axes.

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `clk` | input | 1 | `sys_clk` | 100 MHz system clock |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `ce_100hz` | input | 1 | `sys_clk` | 100 Hz clock-enable from CS12 |
| `signal_strength[11:0]` | input | 12 | `sys_clk` | Raw ADC from laser receiver (0–4095) |
| `signal_valid` | input | 1 | `sys_clk` | ADC reading valid |
| `laser_enable` | input | 1 | `sys_clk` | Enable from CS8 (asserted in FINE_POINT state) |
| `safe_mode` | input | 1 | `sys_clk` | Safe mode from CS8 (immediate shutdown) |
| `gimbal_cmd_abs[0:1]` | input | 2 × 16 | `sys_clk` | Absolute az/el gimbal command (Q15) from external |
| `gimbal_cmd_valid` | input | 1 | `sys_clk` | External gimbal command valid strobe |
| `isl_data_in[7:0]` | input | 8 | `sys_clk` | ISL data byte to transmit in COMM state |
| `isl_data_valid` | input | 1 | `sys_clk` | ISL data valid |
| `manual_clear` | input | 1 | `sys_clk` | Uplink command to clear FAULT state |
| `pointing_error_az` | input | 16 | `sys_clk` | Azimuth pointing error for TRACK PD (Q15) |
| `pointing_error_el` | input | 16 | `sys_clk` | Elevation pointing error for TRACK PD (Q15) |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `laser_mod_en` | output | 1 | `sys_clk` | Laser modulator enable (COMM state) |
| `gimbal_step[1:0]` | output | 2 | `sys_clk` | STEP pulses for AZ/EL stepper motors |
| `gimbal_dir[1:0]` | output | 2 | `sys_clk` | Direction signals (1 = positive / CW) |
| `laser_state[2:0]` | output | 3 | `sys_clk` | Current FSM state (3-bit encoded) |
| `pointing_locked` | output | 1 | `sys_clk` | Asserted in TRACK or COMM state |
| `laser_fault` | output | 1 | `sys_clk` | Asserted in FAULT state |
| `laser_pwm` | output | 1 | `sys_clk` | ISL PWM output during COMM state |
| `fault_code[7:0]` | output | 8 | `sys_clk` | Detailed fault reason code |
| `gimbal_pos_az[23:0]` | output | 24 | `sys_clk` | Current azimuth position feedback |
| `gimbal_pos_el[23:0]` | output | 24 | `sys_clk` | Current elevation position feedback |
| `convergence_time_100ms` | output | 8 | `sys_clk` | Spiral convergence time in 100 ms ticks |
| `signal_strength_filtered[11:0]` | output | 12 | `sys_clk` | 16-sample rolling average of signal_strength |

---

## 7. Architecture

```
laser_enable, safe_mode, ce_100hz
      │
      ▼
┌──────────────────────────────────────────────────────────────────┐
│  laser_fsm_wrapper (8-state FSM)                                 │
│  IDLE→SEARCH→ACQUIRE→TRACK→COMM→HOLD→FAULT; any→SAFE            │
│                                                                  │
│  ┌───────────────────┐  az/el increments  ┌──────────────────┐  │
│  │  raster_scan_engine│─────────────────▶ │  gimbal_controller│  │
│  │  (SEARCH boustro.  │                   │  (STEP/DIR pulses,│  │
│  │   ±10°az ±5°el)   │  ┌──────────────┐ │   position accum, │  │
│  └───────────────────┘  │spiral_refinement│ trapezoidal accel)│  │
│  ┌───────────────────┐  │(ACQUIRE ±2°) │  └──────────────────┘  │
│  │  peak_hold_detector│  └──────────────┘                       │
│  │  (peak tracking)  │                                           │
│  └───────────────────┘                                           │
│  ┌─────────────────────┐  ┌────────────────────────────────┐    │
│  │  signal_monitor      │  │  pid_controller × 2            │    │
│  │  (16-sample rolling  │  │  (TRACK closed-loop az, el)    │    │
│  │   average, 10 kHz)   │  └────────────────────────────────┘    │
│  └─────────────────────┘                                         │
│  ┌───────────────────┐  ┌────────────────┐                       │
│  │  laser_modulator  │  │ laser_fault_    │                      │
│  │  (COMM: ISL mux,  │  │ handler        │──▶ fault_code         │
│  │   laser_pwm)      │  └────────────────┘                       │
│  └───────────────────┘                                           │
└──────────────────────────────────────────────────────────────────┘
```

**State Encoding (`laser_state[2:0]`):**

| Value | State |
|---|---|
| 0 | IDLE |
| 1 | SEARCH |
| 2 | ACQUIRE |
| 3 | TRACK |
| 4 | HOLD |
| 5 | COMM |
| 6 | FAULT |
| 7 | SAFE |

**Reused Helper IPs (from `CubeSat/`):**
- `pid_controller.sv` — PD closed-loop pointing control (2 instances, az + el)
- `pwm_gen.sv` — Laser modulator PWM generation
- `stepper_driver.sv` — Gimbal STEP/DIR pulse generation
- `lpf.sv` — Low-pass filter on signal strength input

---

## 8. Data Formats

| Signal | Format | Notes |
|---|---|---|
| `signal_strength[11:0]` | 12-bit unsigned | 0 = no signal, 4095 = full-scale ADC |
| `signal_strength_filtered[11:0]` | 12-bit unsigned | 16-sample rolling average |
| `gimbal_cmd_abs[0:1]` | Q15 signed (16-bit) | Pointing angle; 1.0 = max travel |
| `pointing_error_az/el` | Q15 signed (16-bit) | Error in radians (or angle units) |
| `gimbal_pos_az/el[23:0]` | 24-bit signed | Step count × step_resolution |
| `convergence_time_100ms` | 8-bit unsigned | Ticks of ce_100hz to convergence |

**FSM Thresholds:**

| Parameter | Default | Description |
|---|---|---|
| `ACQ_THRESH` | 512 | ADC counts: SEARCH → ACQUIRE |
| `HOLD_THRESH` | 256 | ADC counts: below = dropout |
| `ACQ_CONFIRM` | 5 | Consecutive ticks above ACQ_THRESH |
| `HOLD_TIMEOUT` | 20 | ce_100hz ticks before HOLD → FAULT (200 ms) |
| `FAULT_TIMEOUT` | 100 | ce_100hz ticks before FAULT (1 s) |
| `SEARCH_THRESH` | 300 | Peak to advance from SEARCH → ACQUIRE |

---

## 9. Register Interface

CS10 has **no AXI4-Lite register interface** in the current implementation. All thresholds are compile-time parameters. For ground-configurable thresholds (e.g., different signal environments), these should be promoted to AXI4-Lite registers.

---

## 10. File Structure

```
CubeSat/CS10_LASER/
├── laser_fsm_wrapper.sv       ← Top-level 8-state FSM wrapper; CS12 integration point
├── raster_scan_engine.sv      ← SEARCH boustrophedon ±10°az ×±5°el pattern generator
├── spiral_refinement.sv       ← ACQUIRE spiral convergence pattern
├── peak_hold_detector.sv      ← Signal strength peak tracking during ACQUIRE
├── signal_monitor.sv          ← 16-sample rolling average; 10 kHz sampling
├── gimbal_controller.sv       ← 2-axis STEP/DIR generation; position accumulator
├── laser_modulator.sv         ← ISL data mux; laser_pwm generation (COMM)
├── laser_fault_handler.sv     ← Fault detection; fault_code encoding
├── tb_laser_fsm_wrapper.sv    ← Integration testbench
└── README.md                  ← This file

CubeSat/ (shared helper IPs used by CS10):
├── pid_controller.sv          ← PD tracking controller (2 instances)
├── pwm_gen.sv                 ← PWM generator for laser modulator
├── stepper_driver.sv          ← STEP/DIR pulse generator
└── lpf.sv                     ← Low-pass filter (signal noise rejection)
```

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `ce_100hz` | CS10 ← CS12 | `clk_manager` (CS12) | 100 Hz FSM tick |
| `laser_enable` | CS10 ← CS8 | `adcs_fsm_wrapper` (CS8) | Enable when FINE_POINT state |
| `safe_mode` | CS10 ← CS8 | `adcs_fsm_wrapper` (CS8) | Immediate SAFE shutdown |
| `laser_state`, `pointing_locked` | CS10 → CS11 | `telemetry_wrapper` (CS11) | Laser status telemetry (APID 0x0103) |
| `laser_fault` | CS10 → CS11 | `telemetry_wrapper` (CS11) | Fault status telemetry |
| `gimbal_step/dir` | CS10 → Physical | FPGA I/O | Stepper motor driver ICs |
| `laser_mod_en`, `laser_pwm` | CS10 → Physical | FPGA I/O | Laser modulator enable and PWM |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- SEARCH scan covers 20° × 10° in (20/0.5) × (10/0.5) / 100 Hz = 80 s worst-case.
- ACQUIRE convergence requirement: < 0.1° in ≤ 5 s (500 ce_100hz ticks).
- TRACK RMS error: < 0.05° requires PID gains tuned to gimbal inertia and ADC noise.

**Resource:**
- DSP48E1: 2 (per plan) for PID multiply-accumulate (az + el).
- BRAM: 512 B (per plan) for scan pattern tables and peak history.

**Optimization Opportunities:**
1. Promote FSM thresholds to AXI4-Lite registers for ground-configurable tuning.
2. Increase ADC sampling rate above 10 kHz for faster signal detection in acquisition.
3. Add velocity feed-forward to gimbal controller for faster transient response during TRACK.
4. Implement look-ahead prediction using CS9 orbit geometry for SEARCH starting angle.

**Timing:**
- STEP/DIR pulses up to 200 kHz must be generated deterministically; use a dedicated counter in `stepper_driver`.
- `signal_monitor` rolling average: 16 samples at 10 kHz = 1.6 ms response time.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS10_LASER/tb_laser_fsm_wrapper.sv`

**Test Scenarios:**
- Assert `laser_enable = 0`; verify FSM remains in IDLE/SAFE.
- Assert `laser_enable = 1`, `signal_valid = 1`, `signal_strength = 0`; verify SEARCH scan begins (gimbal_step pulses observed).
- Assert `signal_strength > ACQ_THRESH` for 5 consecutive ticks; verify SEARCH → ACQUIRE → TRACK.
- In TRACK state, drop `signal_strength` below `HOLD_THRESH`; verify TRACK → HOLD.
- Hold in HOLD for > 20 ticks; verify HOLD → FAULT.
- Assert `manual_clear` in FAULT; verify FAULT → IDLE.
- Assert `safe_mode = 1` from any state; verify immediate → SAFE.
- Verify `signal_strength_filtered` is stable 16-sample average of `signal_strength`.

**Simulation Notes:**
- Compile with `iverilog -g2012` including `pid_controller.sv`, `pwm_gen.sv`, `stepper_driver.sv`, `lpf.sv`.
- Timescale: 1 ns / 1 ps.

**Requirements Coverage:**
- CS-LSR-001 to CS-LSR-010: 8-state FSM, boustrophedon SEARCH ±10°az×±5°el, spiral ACQUIRE < 0.1° in 5 s, TRACK RMS < 0.05°, ISL modulation, HOLD/FAULT recovery.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
