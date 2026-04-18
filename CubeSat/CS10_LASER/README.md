# CS10 — Laser Pointing FSM

> 8-state FSM (IDLE → SEARCH → ACQUIRE → TRACK → HOLD → COMM → FAULT → SAFE) for laser inter-satellite link pointing, with boustrophedon raster scan, spiral refinement, PD closed-loop tracking, ISL modulation, and 2-axis gimbal stepper control.

---

## Overview

| Attribute | Value |
|---|---|
| Requirements | CS-LSR-001 through CS-LSR-010 |
| Top module | `laser_fsm_wrapper` |
| Clock domain | `sys_clk` (50 MHz) |
| CE strobe | 100 Hz `ce_100hz` from CS12 |
| States | IDLE, SEARCH, ACQUIRE, TRACK, HOLD, COMM, FAULT, SAFE |
| Tracking rate | 100 Hz closed-loop PD |
| Gimbal axes | 2 (azimuth, elevation), stepper steps |
| ADC signal | 12-bit, 0–4095 counts |
| Signal threshold | `ACQ_THRESH = 512`, `HOLD_THRESH = 256` |
| ISL data rate | 8-bit parallel @ 100 MHz |
| BRAM | 512 B |
| DSP48 | 2 |

---

## File Structure

| File | Purpose |
|---|---|
| `laser_fsm_wrapper.sv` | Top-level — state machine, signal routing, output mux |
| `raster_scan_engine.sv` | Boustrophedon ±10° az × ±5° el scan pattern, ≤0.5° step |
| `spiral_refinement.sv` | Spiral ±2° reducing 50 %/pass; reports `spiral_converged` |
| `peak_hold_detector.sv` | Tracks signal peak across acquisition window |
| `signal_monitor.sv` | 10 kHz sampling, rolling average, threshold compare |
| `laser_modulator.sv` | ISL 8-bit data modulation in COMM state |
| `laser_fault_handler.sv` | 8-entry fault FIFO; safe-state enforcement |
| `gimbal_controller.sv` | Absolute/relative stepper commands; position tracking |
| `tb_laser_fsm_wrapper.sv` | Self-checking directed testbench |

Shared primitives used: `pid_controller.sv`, `pwm_gen.sv`, `lpf.sv`, `stepper_driver.sv`.

---

## Module Interface

```systemverilog
module laser_fsm_wrapper #(
    parameter int ACQ_THRESH    = 512,
    parameter int HOLD_THRESH   = 256,
    parameter int HOLD_TIMEOUT  = 20,   // ce_100hz ticks
    parameter int FAULT_TIMEOUT = 100,  // ce_100hz ticks
    parameter int SEARCH_THRESH = 300
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    input  logic [11:0] signal_strength,        // 12-bit optical ADC
    input  logic        signal_valid,

    input  logic        laser_enable,            // from CS8 (FINE_POINT mode)
    input  logic        safe_mode,               // from CS8 (immediate shutdown)

    input  logic signed [15:0] gimbal_cmd_abs [0:1],  // abs az/el command Q8.8°
    input  logic               gimbal_cmd_valid,

    input  logic [7:0]  isl_data_in,
    input  logic        isl_data_valid,

    input  logic        manual_clear,            // uplink: clear FAULT state

    input  logic signed [15:0] pointing_error_az,  // track error input
    input  logic signed [15:0] pointing_error_el,

    output logic        laser_mod_en,
    output logic [1:0]  gimbal_step,
    output logic [1:0]  gimbal_dir,
    output logic [2:0]  laser_state,             // 3-bit state encoding
    output logic        pointing_locked,
    output logic        laser_fault,
    output logic        laser_pwm,               // ISL carrier (COMM state)
    output logic [7:0]  fault_code,
    output logic signed [23:0] gimbal_pos_az,
    output logic signed [23:0] gimbal_pos_el,
    output logic [7:0]  convergence_time_100ms,
    output logic [11:0] signal_strength_filtered
);
```

---

## Functionality

### State Machine

| State | Behaviour |
|---|---|
| IDLE | Home position (0°, 0°); modulator disabled; awaits `laser_enable` |
| SEARCH | Boustrophedon raster ±10° az × ±5° el; `raster_scan_engine` steps gimbal; advances to ACQUIRE when `signal_strength > SEARCH_THRESH` |
| ACQUIRE | `spiral_refinement` centres on peak; reduces spiral radius 50 %/pass; advances to TRACK on `spiral_converged` |
| TRACK | PD closed-loop (`pid_controller`) @ 100 Hz on `pointing_error_az/el`; transitions to COMM on `isl_data_valid` |
| HOLD | Maintains last gimbal position; returns to TRACK/COMM when signal recovers; → FAULT after `HOLD_TIMEOUT` ticks |
| COMM | Modulates ISL data via `laser_modulator`; maintains track; → HOLD on signal loss |
| FAULT | Disables emission and motion; logs fault code to 8-entry FIFO; clears only on `manual_clear & !gimbal_busy` |
| SAFE | Immediate shutdown on `!laser_enable`; returns to IDLE on `laser_enable & !gimbal_busy` |

### Signal monitoring
`signal_monitor` samples at 10 kHz, applies IIR LPF (`lpf.sv`), and exports `signal_strength_filtered`.

---

## Simulation Instructions

```bash
iverilog -g2012 -o sim_cs10 \
  CS10_LASER/tb_laser_fsm_wrapper.sv \
  CS10_LASER/laser_fsm_wrapper.sv \
  CS10_LASER/raster_scan_engine.sv \
  CS10_LASER/spiral_refinement.sv \
  CS10_LASER/peak_hold_detector.sv \
  CS10_LASER/signal_monitor.sv \
  CS10_LASER/laser_modulator.sv \
  CS10_LASER/laser_fault_handler.sv \
  CS10_LASER/gimbal_controller.sv \
  pid_controller.sv pwm_gen.sv lpf.sv stepper_driver.sv

vvp sim_cs10
```

```tcl
vlog -sv CS10_LASER/tb_laser_fsm_wrapper.sv CS10_LASER/*.sv pid_controller.sv pwm_gen.sv lpf.sv stepper_driver.sv
vsim -t 1ps tb_laser_fsm_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 50 MHz; CE at 100 Hz internally generated |
| Stimulus | Scripted signal-strength waveform stepping through SEARCH → ACQUIRE → TRACK → COMM → FAULT |
| Checking | State encoding at each transition; `pointing_locked` timing; fault-code FIFO after FAULT |
| Coverage | Full state graph traversal, safe-mode immediate shutdown, `manual_clear` recovery, hold timeout, signal recovery |

---

## Expected Behaviour

```
laser_enable       _____|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
safe_mode          ‾‾‾‾‾|_____________________________|‾_
laser_state[2:0]:  IDLE → SEARCH → ACQUIRE → TRACK → COMM
signal_strength:   0 → ramp up past SEARCH_THRESH → peak → tracking oscillation
pointing_locked:   _____________________________|‾‾‾‾‾‾‾‾
gimbal_step[1:0]:  stepper pulses visible during SEARCH/ACQUIRE
laser_pwm:         ___________________________|‾‾carrier‾
fault_code:        0x00 until signal lost > HOLD_TIMEOUT
```

---

## Limitations

- 2-axis gimbal model only; does not represent full 3-axis mechanical system.
- Optical channel effects (beam divergence, atmospheric turbulence) not modelled.
- PD gain (`pid_controller` Kp/Kd) is a compile-time parameter; no runtime command path.
- `manual_clear` has no command authentication in MVP.

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] All 8 states reached; transitions verified
- [x] `safe_mode` immediate shutdown within 1 cycle confirmed
- [x] Raster scan step pattern verified
- [x] Spiral convergence assertion within `HOLD_TIMEOUT`
- [x] Fault FIFO captures correct fault code
- [x] `manual_clear` recovery from FAULT tested
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] PD gain runtime command interface
- [ ] Synthesis timing closure; DSP48 count confirmed (target: 2)
