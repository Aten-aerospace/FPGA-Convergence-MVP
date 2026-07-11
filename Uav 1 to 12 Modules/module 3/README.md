# UAV Module 3: Motor Mixing + Output Saturation + ESC PWM Generation

## Overview

UAV Module 3 translates the four control outputs from the PID controller (roll, pitch, yaw, and collective thrust) into four synchronized ESC PWM signals. A 4×4 motor mixing matrix (implemented with DSP48E1 MAC blocks) distributes control authority, output saturation logic prevents over-driving any motor, and a synchronized PWM generator produces 1 µs-resolution pulses for all four ESCs simultaneously.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🔴 CRITICAL — Directly drives motors; failure causes loss of vehicle

---

## Key Functionality

- **4×4 motor mixing matrix:** Maps roll/pitch/yaw/thrust → 4 individual motor commands using DSP48E1 MAC units
- **Q2.14 mixing coefficients** stored in BRAM (MOD_12); updateable at 100 Hz via AXI
- **Per-axis output saturation:** 16-bit signed ±32 767 hard clamps
- **Anti-windup integrator feedback:** Saturation active flags fed back to MOD_2 PID
- **Rate limiting:** Maximum ΔThrust / ΔAngle change per 100 Hz cycle
- **4 synchronized ESC PWM outputs** at 50–400 Hz configurable frequency (default 50 Hz)
- **1 µs pulse-width resolution** across all channels
- **Inter-channel skew ≤ 1 µs** — all four PWM edges start simultaneously
- **Armed/Disarmed interlock:** All PWM outputs held at `PWM_MIN = 1000 µs` when disarmed
- **Saturation active flags** reported in STATUS register

---

## Input Signals

| Signal Name       | Width  | Format  | Source     | Description                                     |
|-------------------|--------|---------|------------|-------------------------------------------------|
| `clk`             | 1-bit  | —       | MOD_1      | 50 MHz system clock                             |
| `rst_n`           | 1-bit  | —       | MOD_1      | Active-low synchronous reset                    |
| `ce_100hz`        | 1-bit  | —       | MOD_1      | Motor mix and PWM update strobe                 |
| `u_roll`          | 32-bit | Q4.28   | MOD_2      | Roll rate command from PID                      |
| `u_pitch`         | 32-bit | Q4.28   | MOD_2      | Pitch rate command from PID                     |
| `u_yaw`           | 32-bit | Q4.28   | MOD_2      | Yaw rate command from PID                       |
| `u_thrust`        | 16-bit | unsigned| MOD_2      | Collective thrust (0–32 767)                    |
| `arm_disarm`      | 1-bit  | —       | MOD_8 FSM  | 1 = Armed (normal PWM); 0 = Disarmed (PWM_MIN)  |
| `mix_matrix[3:0][3:0]` | 16-bit | Q2.14 | MOD_10 AXI | 4×4 mixing matrix coefficients (BRAM)      |
| `pwm_freq_sel`    | 8-bit  | unsigned| MOD_10 AXI | ESC PWM frequency selector (50–400 Hz)          |

---

## Output Signals

| Signal Name          | Width  | Format     | Destination         | Description                              |
|----------------------|--------|------------|---------------------|------------------------------------------|
| `pwm[0]`             | 1-bit  | PWM        | ESC Motor 1         | 1000–2000 µs PWM pulse                   |
| `pwm[1]`             | 1-bit  | PWM        | ESC Motor 2         | 1000–2000 µs PWM pulse                   |
| `pwm[2]`             | 1-bit  | PWM        | ESC Motor 3         | 1000–2000 µs PWM pulse                   |
| `pwm[3]`             | 1-bit  | PWM        | ESC Motor 4         | 1000–2000 µs PWM pulse                   |
| `saturation_active`  | 4-bit  | status     | MOD_12 AXI status   | Bitmask: bit[n]=1 when motor n saturated  |

---

## Architecture

### High-Level Components

```
            ┌──────────────────────────────────────────────────────┐
            │              module3_uav_motor_mix.sv                │
            │                                                      │
  u_roll  ─►│                                                      │
  u_pitch ─►│  mixing_matrix_4x4.sv  ──► sat_rate_limiter.sv     ├─► motor_cmd[0:3]
  u_yaw   ─►│  (DSP48E1 MACs, Q2.14)      (±32767 clamp + rate)  │
  u_thrust►│                                                       │
            │  antiwindup_clamp.sv                                 │
            │  (saturation flags → MOD_2)                         │
            │                                                      │
            │  pwm_sync.sv ─────────────────────────────────────►  ├─► pwm[3:0]
            │  (pwm_gen.sv ×4 synchronized, 50-400Hz, 1µs res)    │
            └──────────────────────────────────────────────────────┘
```

### Reused Modules from RTL_20

| Module       | Role                                                      |
|--------------|-----------------------------------------------------------|
| `pwm_gen.sv` | Base PWM pulse generator (instantiated ×4)                |

### New Modules Created

| Module                  | Role                                                    |
|-------------------------|---------------------------------------------------------|
| `mixing_matrix_4x4.sv`  | 4×4 matrix multiply using DSP48E1 MACs, Q2.14 coefficients |
| `pwm_sync.sv`           | Synchronizer: ensures all 4 PWM rising edges align within 1 µs |
| `sat_rate_limiter.sv`   | Output saturation ±32 767 + per-cycle rate limiting      |
| `antiwindup_clamp.sv`   | Saturation flag generation for MOD_2 anti-windup feedback|

---

## Data Formats

| Parameter                | Format    | Range           | Notes                                      |
|--------------------------|-----------|-----------------|--------------------------------------------|
| Mixing matrix coefficients | Q2.14   | ±1.999 84       | 16 values stored in BRAM, updateable 100 Hz|
| Motor commands (pre-sat)  | 16-bit signed | ±32 767    | After matrix multiply, before clamping     |
| Motor commands (post-sat) | 16-bit signed | ±32 767    | After saturation and rate limiting          |
| PWM pulse width           | 16-bit counter | 1000–2000 µs | 1 µs resolution at 50 MHz clock             |
| PWM frequency             | 8-bit unsigned | 50–400 Hz  | Default 50 Hz for servo-type ESCs          |

---

## Register Interface (AXI4-Lite via MOD_10)

| Register Name          | Address (MOD_10) | Access | Format | Description                          |
|------------------------|------------------|--------|--------|--------------------------------------|
| `MOTOR_MIX_M[0..15]`  | 0x00–0x3C        | R/W    | Q2.14  | 4×4 mixing matrix (16 coefficients)  |
| `PWM_FREQUENCY`        | 0x40             | R/W    | unsigned| ESC PWM frequency (50–400 Hz)        |
| `SATURATION_STATUS`    | 0x44             | R/O    | bitmask| Per-motor saturation active flags     |

---

## File Structure

```
Uav 1 to 12 Modules/module 3/
├── module3_uav_motor_mix.sv    # Top-level motor mix + PWM wrapper
├── mixing_matrix_4x4.sv        # 4×4 DSP48E1 MAC mixing engine
├── pwm_sync.sv                 # Synchronized PWM edge generator
├── sat_rate_limiter.sv         # Output saturation + rate limiter
└── antiwindup_clamp.sv         # Saturation feedback for PID anti-windup
```

### Dependencies (from RTL_20 shared library)

```
pwm_gen.sv   # Base PWM pulse generator (reused ×4 channels)
```

---

## Module Interconnections

```
MOD_2 ── u_roll/u_pitch/u_yaw/u_thrust ─────────► MOD_3 (control commands)
MOD_8 ── arm_disarm ─────────────────────────────► MOD_3 (safety interlock)
MOD_10 ─ mix_matrix coefficients (BRAM) ─────────► MOD_3 (mixing coefficients)
MOD_3 ── pwm[3:0] ───────────────────────────────► ESC Motor 1–4 (hardware)
MOD_3 ── saturation_active ──────────────────────► MOD_12 AXI status register
```

---

## Design Considerations

- **DSP48E1 usage:** Each of the 4 motor outputs requires a dot-product of 4 inputs with 4 coefficients = 16 multiplications. Using DSP48E1 MACs with cascaded accumulation reduces the critical path and saves LUTs. Estimated 4–8 DSP48E1 blocks depending on pipelining depth.
- **PWM synchronization:** All four PWM counters reset simultaneously at the start of each PWM period. This ensures inter-channel skew ≤ 1 µs even with asynchronous service operations.
- **Armed/Disarmed interlock:** In disarmed state the PWM comparator value is forced to `PWM_MIN = 1000 µs` regardless of PID output. This provides positive motor-stop safety.
- **Rate limiting:** Prevents abrupt thrust jumps that could cause structural stress or loss of attitude control.
- **Resource estimate:**
  - DSP48E1: 4–8 (MAC for 4×4 matrix)
  - BRAM18: 1 (mixing matrix + saturation limits)
  - LUTs: ~400
  - FFs: ~350

---

## Testing & Verification

| Test Point                         | Verification Method                                        |
|------------------------------------|------------------------------------------------------------|
| Matrix multiply correctness        | Inject known inputs; compare output against MATLAB model   |
| Output saturation at ±32 767       | Drive input above limit; verify output clamped             |
| PWM pulse width = 1000 µs (min)    | Inject minimum command; measure pulse with counter          |
| PWM pulse width = 2000 µs (max)    | Inject maximum command; measure pulse                       |
| Inter-channel skew ≤ 1 µs          | Simultaneous 4-channel edge detection in simulation         |
| Disarmed → PWM_MIN                 | Toggle arm_disarm=0; verify all channels → 1000 µs          |
| Saturation flag assertion          | Saturate one motor; verify STATUS bit asserts               |
| Rate limiting step response        | Apply large step; verify ΔThrust ≤ rate limit per cycle     |

---

## Optimization Scope

| Area            | Opportunity                                                              | Impact  |
|-----------------|--------------------------------------------------------------------------|---------|
| **Resource**    | Use DSP48E1 cascaded accumulators for all 4 motor dot-products            | High    |
| **Performance** | Fully pipeline matrix multiply to achieve 1-cycle latency                | Medium  |
| **Power**       | Clock-gate PWM counters during disarmed state                            | Medium  |
| **Timing**      | Register matrix outputs before feeding PWM comparators                   | Low     |
| **Area**        | Share saturation/rate-limit logic across 4 channels via time-mux         | Low     |

---

*Module 3 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
