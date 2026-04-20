# UAV Module 2: Dual-Loop PID Controller

## Overview

UAV Module 2 implements a complete dual-loop PID control architecture for 6-DOF (degrees of freedom) UAV flight control. A single `pid_controller.sv` core is time-multiplexed across 8 control axes to save FPGA resources while meeting all real-time deadlines. The inner loop runs at 1 kHz for fast rate stabilisation; the outer loop runs at 100 Hz for attitude and velocity control.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🔴 CRITICAL — Primary flight control law; failure causes loss of vehicle

---

## Key Functionality

- **Inner loop @ 1 kHz:** Roll rate, Pitch rate, Yaw rate PIDs with gyro feedback
- **Outer loop @ 100 Hz:** Roll attitude, Pitch attitude, Altitude PIDs (±45°/±30°/0–500 m limits)
- **Outer loop @ 100 Hz:** North-velocity PD and East-velocity PD (±15 m/s range)
- Time-multiplexing of a **single `pid_controller.sv`** across all 8 axes (saves ~7× LUTs)
- **18 gain registers** (Kp, Ki, Kd × 6 axes) in Q4.12 format (resolution 0.000244)
- **4 flight-mode preset pages** stored in dual-port BRAM
- **Anti-windup integrator clamps** per axis: ±10 000 (roll/pitch) | ±8 000 (yaw)
- **Rate limiting** on ΔThrust and ΔAngle per 100 Hz cycle

---

## Input Signals

| Signal Name              | Width  | Format  | Source           | Description                              |
|--------------------------|--------|---------|------------------|------------------------------------------|
| `clk`                    | 1-bit  | —       | MOD_1            | 50 MHz system clock                      |
| `rst_n`                  | 1-bit  | —       | MOD_1            | Active-low synchronous reset             |
| `ce_1khz`                | 1-bit  | —       | MOD_1            | Inner-loop enable strobe                 |
| `ce_100hz`               | 1-bit  | —       | MOD_1            | Outer-loop enable strobe                 |
| `gyro_roll_rate`         | 32-bit | Q4.28   | MOD_5 (IMU SPI)  | Roll angular rate from ICM-42688         |
| `gyro_pitch_rate`        | 32-bit | Q4.28   | MOD_5 (IMU SPI)  | Pitch angular rate                       |
| `gyro_yaw_rate`          | 32-bit | Q4.28   | MOD_5 (IMU SPI)  | Yaw angular rate                         |
| `ekf_roll`               | 32-bit | Q3.29   | MOD_4/5/6        | Estimated roll angle from EKF            |
| `ekf_pitch`              | 32-bit | Q3.29   | MOD_4/5/6        | Estimated pitch angle from EKF           |
| `ekf_altitude`           | 32-bit | Q10.22  | MOD_4/5/6        | Estimated altitude from EKF              |
| `ekf_vel_n`              | 32-bit | Q4.28   | MOD_4/5/6        | North velocity estimate                  |
| `ekf_vel_e`              | 32-bit | Q4.28   | MOD_4/5/6        | East velocity estimate                   |
| `setpoint_roll`          | 32-bit | Q4.28   | MOD_8 FSM        | Roll attitude target                     |
| `setpoint_pitch`         | 32-bit | Q4.28   | MOD_8 FSM        | Pitch attitude target                    |
| `setpoint_altitude`      | 32-bit | Q10.22  | MOD_8 FSM        | Altitude target                          |
| `setpoint_vn`            | 32-bit | Q4.28   | MOD_8 FSM        | North-velocity target                    |
| `setpoint_ve`            | 32-bit | Q4.28   | MOD_8 FSM        | East-velocity target                     |
| `kp[17:0]`/`ki`/`kd`    | 16-bit | Q4.12   | MOD_10 AXI regs  | PID gain registers (18 total)            |

---

## Output Signals

| Signal Name  | Width  | Format  | Destination  | Description                                 |
|--------------|--------|---------|--------------|---------------------------------------------|
| `u_roll`     | 32-bit | Q4.28   | MOD_3        | Roll rate command (inner-loop output)        |
| `u_pitch`    | 32-bit | Q4.28   | MOD_3        | Pitch rate command                           |
| `u_yaw`      | 32-bit | Q4.28   | MOD_3        | Yaw rate command                             |
| `u_thrust`   | 16-bit | unsigned| MOD_3        | Collective thrust (0–32 767)                 |
| `pid_valid`  | 1-bit  | —       | MOD_3        | Output data valid strobe                     |

---

## Architecture

### High-Level Components

```
                    ┌─────────────────────────────────────────────┐
                    │            module2_uav_pid_top.sv            │
                    │                                              │
  ce_1khz ────────► │  pid_loop_mux.sv   ──► pid_controller.sv   │──► u_roll/pitch/yaw
  ce_100hz ───────► │  (8-axis time mux)     (RTL_20 reuse)      │──► u_thrust
  gyro_* ─────────► │                                              │
  ekf_* ──────────► │  velocity_to_angle.sv ──► angle_limiter.sv │
  setpoints ──────► │                                              │
  gains (AXI) ────► │  (BRAM: 4 flight mode presets)              │
                    └─────────────────────────────────────────────┘
```

### Reused Modules from RTL_20

| Module               | Role                                                    |
|----------------------|---------------------------------------------------------|
| `pid_controller.sv`  | Core PID computation engine (time-multiplexed ×8)      |

### New Modules Created

| Module                  | Role                                                  |
|-------------------------|-------------------------------------------------------|
| `pid_loop_mux.sv`       | Selects active axis; routes gains, error, integrator  |
| `angle_limiter.sv`      | Clamps outer-loop angle setpoints to safe limits      |
| `velocity_to_angle.sv`  | Converts velocity PD output to roll/pitch angles      |
| `antiwindup_clamp.sv`   | Integrator saturation limiter per axis                |

---

## Data Formats

| Parameter              | Format   | Range                     | Notes                           |
|------------------------|----------|---------------------------|---------------------------------|
| PID Gains (Kp/Ki/Kd)  | Q4.12    | 0–15.999 (resolution 0.000244) | 18 registers via AXI       |
| Angles (roll/pitch)    | Q3.29    | ±π rad                    | From EKF                        |
| Angular rates          | Q4.28    | ±4.36 rad/s               | From IMU gyro                   |
| Velocity               | Q4.28    | ±100 m/s                  | From EKF                        |
| Altitude               | Q10.22   | 0–10 000 m                | From EKF                        |
| Integrator accumulator | Q4.28    | ±10 000 (roll/pitch), ±8 000 (yaw) | Anti-windup clamped  |
| Thrust output          | 16-bit unsigned | 0–32 767           | To MOD_3 motor mixing           |

---

## Register Interface (AXI4-Lite via MOD_10)

All PID gains are written through MOD_10's AXI4-Lite register file and broadcast to MOD_2.

| Register Name      | Address (MOD_10) | Format | Description                        |
|--------------------|------------------|--------|------------------------------------|
| `KP_ROLL_RATE`     | 0x00             | Q4.12  | Roll rate proportional gain        |
| `KI_ROLL_RATE`     | 0x04             | Q4.12  | Roll rate integral gain            |
| `KD_ROLL_RATE`     | 0x08             | Q4.12  | Roll rate derivative gain          |
| `KP_PITCH_RATE`    | 0x0C             | Q4.12  | Pitch rate proportional gain       |
| `KI_PITCH_RATE`    | 0x10             | Q4.12  | Pitch rate integral gain           |
| `KD_PITCH_RATE`    | 0x14             | Q4.12  | Pitch rate derivative gain         |
| `KP_YAW_RATE`      | 0x18             | Q4.12  | Yaw rate proportional gain         |
| `KI_YAW_RATE`      | 0x1C             | Q4.12  | Yaw rate integral gain             |
| `KD_YAW_RATE`      | 0x20             | Q4.12  | Yaw rate derivative gain           |
| `KP_ROLL_ATT`      | 0x24             | Q4.12  | Roll attitude proportional gain    |
| `KI_ROLL_ATT`      | 0x28             | Q4.12  | Roll attitude integral gain        |
| `KD_ROLL_ATT`      | 0x2C             | Q4.12  | Roll attitude derivative gain      |
| `KP_PITCH_ATT`     | 0x30             | Q4.12  | Pitch attitude proportional gain   |
| `KI_PITCH_ATT`     | 0x34             | Q4.12  | Pitch attitude integral gain       |
| `KD_PITCH_ATT`     | 0x38             | Q4.12  | Pitch attitude derivative gain     |
| `KP_ALT`           | 0x3C             | Q4.12  | Altitude proportional gain         |
| `KI_ALT`           | 0x40             | Q4.12  | Altitude integral gain             |
| `KD_ALT`           | 0x44             | Q4.12  | Altitude derivative gain           |

---

## File Structure

```
Uav 1 to 12 Modules/module 2/
├── module2_uav_pid_top.sv      # Top-level dual-loop PID wrapper
├── pid_loop_mux.sv             # 8-axis time-multiplexer
├── angle_limiter.sv            # Setpoint angle clamp (±45°/±30°)
├── velocity_to_angle.sv        # Velocity PD → angle command conversion
└── antiwindup_clamp.sv         # Per-axis integrator saturation
```

### Dependencies (from RTL_20 shared library)

```
pid_controller.sv   # Core PID engine (time-multiplexed ×1 for 8 axes)
```

---

## Module Interconnections

```
MOD_1 ─── ce_1khz  ──────────────────────────────► MOD_2 (inner loop trigger)
MOD_1 ─── ce_100hz ──────────────────────────────► MOD_2 (outer loop trigger)
MOD_5 ─── gyro_scaled ───────────────────────────► MOD_2 (gyro feedback)
MOD_4/5/6 ─── ekf_state ─────────────────────────► MOD_2 (attitude/velocity/altitude)
MOD_8 ─── setpoints ─────────────────────────────► MOD_2 (roll/pitch/alt/vN/vE targets)
MOD_10 ── gain_registers ────────────────────────► MOD_2 (18 Kp/Ki/Kd values)
MOD_2 ─── u_roll/u_pitch/u_yaw/u_thrust ────────► MOD_3 (motor mixing)
```

---

## Design Considerations

- **Time-multiplexing:** Sharing a single PID core across 8 axes reduces DSP usage from ~56 to ~7 DSP48E1 blocks, at the cost of requiring 8 pipeline slots within each 1 kHz window (50 000 / 8 = 6 250 cycles available per axis — well within budget).
- **Anti-windup:** Integral terms are clamped per axis to prevent windup during actuator saturation. Roll/Pitch limits ±10 000, Yaw limit ±8 000 (Q4.28 LSB units).
- **Gain resolution:** Q4.12 provides 0.000244 per LSB, sufficient for autopilot tuning.
- **Flight mode presets:** 4 mode BRAM pages allow instant gain switching without AXI writes.
- **Resource estimate:**
  - DSP48E1: ~7 (shared PID multiplier)
  - BRAM18: 1 (4 gain pages × 18 registers)
  - LUTs: ~600
  - FFs: ~400

---

## Testing & Verification

| Test Point                          | Verification Method                                              |
|-------------------------------------|------------------------------------------------------------------|
| Step response roll rate @ 1 kHz     | Inject step gyro error; verify convergence within 50 ms         |
| Step response altitude @ 100 Hz     | Inject altitude error; verify settling within 3 s               |
| Anti-windup clamp activation        | Saturate setpoint; verify integrator freezes at limit            |
| Gain register update                | Write new Kp via AXI; verify output changes within 1 cycle       |
| Time-mux scheduling                 | Check u_roll updated exactly once per 1 ms                       |
| Flight-mode preset switch           | Toggle mode; verify all 18 gains switch atomically               |

---

## Optimization Scope

| Area            | Opportunity                                                                  | Impact  |
|-----------------|------------------------------------------------------------------------------|---------|
| **Resource**    | Time-mux across 8 axes already implemented; further share with MOD_6 filter  | Medium  |
| **Performance** | Pipeline PID stages to increase throughput if >8 axes required               | Low     |
| **Power**       | Clock-gate PID core when ce_1khz/ce_100hz is not asserted                    | Medium  |
| **Timing**      | Add one pipeline register between PID output and motor mixer if path >20 ns  | Low     |
| **Area**        | Store gain BRAM as 16×4 instead of 32×2 pages to save half a BRAM18          | Low     |

---

*Module 2 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
