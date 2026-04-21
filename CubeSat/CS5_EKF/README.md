# CS5 — Extended Kalman Filter (Attitude Estimation)

## 1. Module Title & ID

**Module:** CS5 — Extended Kalman Filter (7-State Attitude Estimation)
**Subsystem ID:** CS5
**Requirements:** CS-ADCS-005, CS-ADCS-006

---

## 2. Overview

CS5 implements a 7-state Extended Kalman Filter for CubeSat attitude determination. The state vector is `[q0, q1, q2, q3, bx, by, bz]`: unit quaternion plus 3-axis gyroscope bias in Q15 fixed-point. It fuses IMU gyroscope (CS1), accelerometer (CS1), and magnetometer (CS2) measurements at 100 Hz to produce a filtered attitude estimate with steady-state error ≤ 0.5° (3σ). A divergence watchdog resets to the last-known-good state after 3 consecutive 5σ innovations. Covariance is maintained via the Joseph-form update to guarantee positive-definiteness.

**Target Platform:** Xilinx Artix-7 XC7A35T

---

## 3. Criticality

**CRITICAL** — CS-ADCS-005, CS-ADCS-006. The EKF provides the primary attitude estimate for the PD controller (CS6) and the mode FSM (CS8). EKF fault triggers COARSE_POINT → DETUMBLE regression in CS8.

---

## 4. Key Functionality

- Predict step (`ekf_predict`): gyro-bias-corrected quaternion propagation using CS4 kinematic model.
- Update step (`ekf_update`): fuses accelerometer (gravity reference) and magnetometer (field reference) measurements.
- Joseph-form covariance update (`ekf_joseph_update`) guarantees symmetric positive-definite `P` matrix.
- Measurement model (`ekf_measurement_model`): computes predicted sensor readings from current state.
- Divergence watchdog: 2 ms timeout (`FAULT_CYCLES = 200_000` at 100 MHz); sets `ekf_fault` on timeout.
- Diagonal `P_diag[0:6]` and `innovation[0:6]` outputs for ground-station telemetry diagnostics.
- Extended outputs: `P_diag` and `innovation` passed through thin `ekf_wrapper` → `ekf_core`.
- Kalman gain K is fixed at `0.1` (Q15 ≈ 3277); promote to AXI4-Lite register for in-orbit tuning.

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `clk` | input | 1 | `sys_clk` | 100 MHz system clock |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `ce_100hz` | input | 1 | `sys_clk` | 100 Hz clock-enable from CS12 |
| `accel[0:2]` | input | 3 × 16 | `sys_clk` | Accelerometer [x,y,z] signed Q15 (normalised to unit vector) |
| `gyro[0:2]` | input | 3 × 16 | `sys_clk` | Gyroscope [x,y,z] signed Q15 rad/s (from CS1) |
| `mag[0:2]` | input | 3 × 16 | `sys_clk` | Magnetometer [x,y,z] signed Q15 (normalised; from CS2) |
| `meas_valid` | input | 1 | `sys_clk` | Sensor data valid; triggers update step |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `q_est[0:3]` | output | 4 × 16 | `sys_clk` | Estimated attitude quaternion [w,x,y,z] signed Q15 |
| `bias_est[0:2]` | output | 3 × 16 | `sys_clk` | Estimated gyro bias [x,y,z] signed Q15 rad/s |
| `ekf_valid` | output | 1 | `sys_clk` | One-cycle strobe — `q_est` / `bias_est` updated |
| `ekf_fault` | output | 1 | `sys_clk` | No valid update within watchdog window (2 ms) |
| `P_diag[0:6]` | output | 7 × 16 | `sys_clk` | Diagonal elements of covariance matrix P (Q15) |
| `innovation[0:6]` | output | 7 × 16 | `sys_clk` | Measurement residuals (index 6 unused) (Q15) |

---

## 7. Architecture

```
accel, gyro, mag, meas_valid
      │
      ▼
┌──────────────────────────────────────────────────────────────────────────┐
│  ekf_core                                                                │
│  ┌─────────────┐   q_pred    ┌─────────────────┐  δq_vec  ┌───────────┐│
│  │  ekf_predict│────────────▶│ ekf_measurement  │─────────▶│ekf_update ││
│  │ (bias-correct│            │      _model      │          │(gain ×    ││
│  │  kinematics) │            │  (gravity/field  │          │ innov.)   ││
│  └─────────────┘            │   prediction)    │          └───────────┘│
│                              └─────────────────┘                q_corr  │
│  ┌──────────────────────────────────────────────────┐               │   │
│  │  ekf_joseph_update                               │◀──────────────┘   │
│  │  (P_new = (I-KH)P(I-KH)ᵀ + KRKᵀ; PD check)    │                   │
│  └──────────────────────────────────────────────────┘                   │
│         q_est, bias_est, P_diag, innovation, ekf_valid, ekf_fault        │
└──────────────────────────────────────────────────────────────────────────┘
         │
  ekf_wrapper (thin pass-through; exposes P_diag and innovation)
```

**Reused Helper IPs (from `CubeSat/`):**
- `kalman_1v.sv` — Single-variable Kalman filter template
- `cordic.sv` — Trigonometric functions for H-matrix computation
- `sqrt.sv` — Covariance square root for Joseph form
- `fp_divider.sv` — Matrix inversion / Kalman gain division

**CS4 sub-modules shared:**
- `quat_multiply.sv` — Hamilton product (from CS4; referenced in predict step)
- `quat_normalize.sv` — Normalization (from CS4; referenced in predict step)

---

## 8. Data Formats

| Signal | Format | Notes |
|---|---|---|
| `q_est[0:3]`, `bias_est[0:2]` | Q15 signed (16-bit) | Quaternion: 1.0 = 32767; bias in rad/s |
| `accel`, `gyro`, `mag` inputs | Q15 signed (16-bit) | Sensors must be unit-vector-normalised |
| `P_diag[0:6]` | Q15 signed (16-bit) | Covariance diagonal; must remain positive |
| `innovation[0:6]` | Q15 signed (16-bit) | Measurement residuals |
| Kalman gain K (internal) | Q15 = 3277 | `0.1 × 32768`; compile-time constant |
| EKF state vector (internal) | Q15 (7 states × 16 bits = 112 bits) | — |

---

## 9. Register Interface

CS5 has **no AXI4-Lite register interface** in the current implementation. The Kalman gain `K` and fault timeout `FAULT_CYCLES` are synthesized constants.

**Parameters (synthesised):**

| Parameter | Default | Description |
|---|---|---|
| `CLK_HZ` | 100_000_000 | System clock frequency in Hz |
| `FAULT_CYCLES` | 200_000 | Watchdog timeout cycles (2 ms at 100 MHz) |

---

## 10. File Structure

```
CubeSat/CS5_EKF/
├── ekf_wrapper.sv           ← Top-level thin wrapper; CS12 integration point
├── ekf_core.sv              ← Main EKF orchestrator and cycle manager
├── ekf_predict.sv           ← Prediction step: bias-corrected kinematics
├── ekf_update.sv            ← Measurement update: gain-weighted correction
├── ekf_joseph_update.sv     ← Joseph-form covariance update + PD check
├── ekf_measurement_model.sv ← Predicted sensor measurements from state
├── ekf_covariance.sv        ← Covariance matrix storage and management
├── tb_ekf_wrapper.sv        ← Integration testbench
└── README.md                ← This file

CubeSat/ (shared helper IPs used by CS5):
├── kalman_1v.sv             ← Single-variable Kalman template
├── cordic.sv                ← CORDIC engine
├── sqrt.sv                  ← Square root
└── fp_divider.sv            ← Fixed-point divider
```

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `ce_100hz` | CS5 ← CS12 | `clk_manager` (CS12) | 100 Hz update trigger |
| `gyro[0:2]`, `accel[0:2]` | CS5 ← CS1 | `spi_imu_wrapper` (CS1) | IMU sensor inputs |
| `mag[0:2]` | CS5 ← CS2 | `i2c_mag_wrapper` (CS2) | Magnetometer measurement |
| `meas_valid` | CS5 ← CS1/CS2 | Combined valid | IMU + mag data valid |
| `q_est[0:3]` | CS5 → CS4 | `quat_propagator_wrapper` (CS4) | EKF-corrected quaternion feedback |
| `q_est[0:3]` | CS5 → CS6 | `pd_control_wrapper` (CS6) | Attitude estimate → error computation |
| `ekf_valid` | CS5 → CS8 | `adcs_fsm_wrapper` (CS8) | Health heartbeat |
| `ekf_fault` | CS5 → CS8 | `adcs_fsm_wrapper` (CS8) | EKF divergence fault |
| `P_diag`, `innovation` | CS5 → CS11 | `telemetry_wrapper` (CS11) | Diagnostic telemetry (ADCS packet) |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- Must complete predict + update within 8 ms (CS-ADCS-005). At 100 MHz, 8 ms = 800,000 cycles.
- Joseph-form covariance update is the most compute-intensive step; pipelining is essential.
- Gyro bias convergence ≤ 0.01 °/s within 60 s requires K ≥ 0.05.

**Resource:**
- BRAM: 512 B for P-matrix storage (7×7 × 16 bits = 784 bits ≈ 98 B per row; double-buffered).
- DSP48E1: 6 instances (per plan) for matrix multiply in Joseph-form and gain application.

**Optimization Opportunities:**
1. Make Kalman gain K runtime-configurable via AXI4-Lite for in-orbit tuning.
2. Use time-multiplexed DSP48E1 across the 7-state multiplications to reduce from 7 to 1 DSP per step.
3. Store P-matrix off-chip in BRAM vs distributed LUT RAM to free LUT resources.
4. Add square-root information filter (SRIF) variant to improve numerical stability at the cost of area.

**Timing:**
- P-matrix operations must be completed before the next `ce_100hz` (10 ms window).
- Critical path: multiply-accumulate chain in Joseph-form update; pipeline to ≥ 4 stages.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS5_EKF/tb_ekf_wrapper.sv`

**Test Scenarios:**
- Initialise to identity quaternion; apply constant gyro input; verify `q_est` rotates smoothly.
- Apply magnetometer correction; verify bias estimate converges within 60 simulated seconds.
- Assert `meas_valid = 0` for > `FAULT_CYCLES`; verify `ekf_fault` asserts.
- Apply 5σ innovation (large magnetometer anomaly); verify divergence watchdog triggers.
- Verify `P_diag` remains positive on all elements after each Joseph-form update.
- Compare `q_est` vs MATLAB/Python EKF golden reference for numerical accuracy ≤ 0.5°.

**Simulation Notes:**
- Compile with `iverilog -g2012` including `kalman_1v.sv`, `cordic.sv`, `sqrt.sv`, `fp_divider.sv`, CS4 sub-modules.
- Timescale: 1 ns / 1 ps.
- Full 30 ms system simulation: `vvp` (approximately 3 minutes runtime).

**Requirements Coverage:**
- CS-ADCS-005: 7-state EKF, predict + update within 8 ms, error ≤ 0.5° (3σ).
- CS-ADCS-006: 7×7 P matrix, Joseph-form update, positive-definite check.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
