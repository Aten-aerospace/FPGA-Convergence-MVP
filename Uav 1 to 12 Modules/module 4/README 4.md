# UAV Module 4: EKF Predict (IMU 100 Hz) + Covariance Manager

## Overview

UAV Module 4 implements the **prediction step** of a 9-state Extended Kalman Filter (EKF) for UAV state estimation. Running at 100 Hz driven by IMU data, it propagates the aircraft state (roll, pitch, yaw, 3D velocity, 3D position) forward in time using Euler kinematic equations and Newton's second law with gravity compensation. A CORDIC-based rotation matrix engine converts Euler angles to the body-to-NED direction cosine matrix, and a Joseph-form covariance propagation block maintains the 9×9 P matrix in BRAM.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🔴 CRITICAL — Foundation of all state estimates; EKF failure triggers EMERGENCY

---

## Key Functionality

- 9-state EKF prediction at **100 Hz** (every 10 ms)
- **Euler angle kinematics:**
  - φ̇ = p + q·sin(φ)·tan(θ) + r·cos(φ)·tan(θ)
  - θ̇ = q·cos(φ) − r·sin(φ)
  - ψ̇ = q·sin(φ)/cos(θ) + r·cos(φ)/cos(θ)
- **Velocity propagation** with gravity compensation in NED frame
- **Position integration** from NED velocity (Euler integration at 100 Hz)
- **CORDIC rotation matrix** R(φ,θ,ψ) for body-to-NED acceleration transformation
- **9×9 symmetric P-matrix** maintenance in BRAM (Q2.46, 486 bytes)
- **Joseph form** covariance update: P = F·P·Fᵀ + Q (numerically stable)
- **Positive-definiteness** checking with fault-triggered reset

---

## Input Signals

| Signal Name           | Width  | Format  | Source                 | Description                              |
|-----------------------|--------|---------|------------------------|------------------------------------------|
| `clk`                 | 1-bit  | —       | MOD_1                  | 50 MHz system clock                      |
| `rst_n`               | 1-bit  | —       | MOD_1                  | Active-low synchronous reset             |
| `ce_100hz`            | 1-bit  | —       | MOD_1                  | Prediction trigger strobe                |
| `gyro_x`              | 32-bit | Q4.28   | MOD_5 (ICM-42688 SPI)  | Body roll rate p (rad/s)                 |
| `gyro_y`              | 32-bit | Q4.28   | MOD_5                  | Body pitch rate q (rad/s)                |
| `gyro_z`              | 32-bit | Q4.28   | MOD_5                  | Body yaw rate r (rad/s)                  |
| `accel_x`             | 32-bit | Q4.28   | MOD_5 (ICM-42688 SPI)  | Body X acceleration (m/s²)               |
| `accel_y`             | 32-bit | Q4.28   | MOD_5                  | Body Y acceleration (m/s²)               |
| `accel_z`             | 32-bit | Q4.28   | MOD_5                  | Body Z acceleration (m/s²)               |
| `state_in[8:0]`       | 32-bit | mixed   | MOD_5 (measurement update) | Previous updated state vector         |
| `cov_p_in[80:0]`      | 48-bit | Q2.46   | BRAM (MOD_12)          | Previous 9×9 covariance matrix (upper Δ)|
| `q_diag[8:0]`         | 32-bit | Q16.16  | MOD_10 AXI             | Process noise diagonal (9 values)        |

---

## Output Signals

| Signal Name               | Width  | Format  | Destination          | Description                               |
|---------------------------|--------|---------|----------------------|-------------------------------------------|
| `state_predicted[8:0]`    | 32-bit | mixed   | MOD_5 (EKF updates)  | Predicted 9-state vector                  |
| `cov_p_predicted[80:0]`   | 48-bit | Q2.46   | BRAM (MOD_12)        | Predicted covariance P = F·P·Fᵀ + Q       |
| `ekf_predict_valid`       | 1-bit  | —       | MOD_5, MOD_9         | Asserted when prediction output is ready  |
| `ekf_predict_halted`      | 1-bit  | —       | MOD_9 watchdog       | Asserted on IMU fault or P non-PD fault   |

---

## Architecture

### High-Level Components

```
                ┌─────────────────────────────────────────────────────┐
                │             module4_uav_ekf_predict.sv               │
                │                                                     │
  gyro_xyz ───► │                                                     │
  accel_xyz ──► │  imu_scaler.sv ─────► ekf_state_predict.sv        ├─► state_predicted
  state_in ───► │  (LSB→Q4.28)          (Euler kinematics)           │
                │                                                     │
                │  rotation_matrix.sv ─► (CORDIC R matrix)           │
                │  (uses cordic.sv)                                   │
                │                                                     │
  cov_p_in ───► │  p_update_predict.sv ──────────────────────────── ├─► cov_p_predicted
                │  (Joseph form P = F·P·Fᵀ + Q)                      │
                │  (positive-definite check + fault reset)            │
                └─────────────────────────────────────────────────────┘
```

### Reused Modules from RTL_20

| Module       | Role                                                          |
|--------------|---------------------------------------------------------------|
| `cordic.sv`  | Computes sin/cos of roll/pitch/yaw for rotation matrix        |
| `sqrt.sv`    | Standard deviation for innovation gates (MOD_5 uses output)   |
| `spi_master.sv` | IMU SPI interface (ICM-42688); used in MOD_5, results fed here |

### New Modules Created

| Module                  | Role                                                       |
|-------------------------|------------------------------------------------------------|
| `ekf_state_predict.sv`  | Euler kinematics + velocity + position integration at 100 Hz|
| `rotation_matrix.sv`    | 3×3 DCM from CORDIC sin/cos, body → NED transform          |
| `imu_scaler.sv`         | IMU raw LSB to Q4.28 rad/s and m/s²                        |
| `p_update_predict.sv`   | Joseph form P propagation + positive-definiteness checker   |

---

## Data Formats

| State Variable    | Symbol | Format  | Range           | Description                  |
|-------------------|--------|---------|-----------------|------------------------------|
| Roll angle        | φ      | Q3.29   | ±π rad          | Euler roll in NED frame       |
| Pitch angle       | θ      | Q3.29   | ±π/2 rad        | Euler pitch                   |
| Yaw angle         | ψ      | Q3.29   | ±π rad          | Euler yaw (heading)           |
| North velocity    | vN     | Q4.28   | ±100 m/s        | NED North velocity            |
| East velocity     | vE     | Q4.28   | ±100 m/s        | NED East velocity             |
| Down velocity     | vD     | Q4.28   | ±50 m/s         | NED Down velocity             |
| Latitude          | lat    | Q10.22  | ±180° (µ°)      | WGS-84 latitude in µ°         |
| Longitude         | lon    | Q10.22  | ±360° (µ°)      | WGS-84 longitude in µ°        |
| Altitude MSL      | alt    | Q10.22  | 0–10 000 m      | Altitude above mean sea level |
| Covariance matrix | P      | Q2.46   | positive-definite | 9×9 symmetric, stored in BRAM |
| Process noise Q   | Q_diag | Q16.16  | positive         | Diagonal noise tuning params  |

---

## Register Interface (AXI4-Lite via MOD_10)

| Register Name   | Address (MOD_10) | Access | Format  | Description                        |
|-----------------|------------------|--------|---------|------------------------------------|
| `Q_ROLL`        | 0x48             | R/W    | Q16.16  | Process noise for roll             |
| `Q_PITCH`       | 0x4C             | R/W    | Q16.16  | Process noise for pitch            |
| `Q_YAW`         | 0x50             | R/W    | Q16.16  | Process noise for yaw              |
| `Q_VN`          | 0x54             | R/W    | Q16.16  | Process noise for North velocity   |
| `Q_VE`          | 0x58             | R/W    | Q16.16  | Process noise for East velocity    |
| `Q_VD`          | 0x5C             | R/W    | Q16.16  | Process noise for Down velocity    |
| `Q_LAT`         | 0x60             | R/W    | Q16.16  | Process noise for latitude         |
| `Q_LON`         | 0x64             | R/W    | Q16.16  | Process noise for longitude        |
| `Q_ALT`         | 0x68             | R/W    | Q16.16  | Process noise for altitude         |

---

## File Structure

```
Uav 1 to 12 Modules/module 4/
├── module4_uav_ekf_predict.sv  # Top-level EKF prediction wrapper
├── ekf_state_predict.sv        # 9-state kinematic predictor
├── rotation_matrix.sv          # CORDIC-based DCM builder
├── imu_scaler.sv               # IMU raw → Q-format converter
└── p_update_predict.sv         # Joseph-form covariance propagation
```

### Dependencies (from RTL_20 shared library)

```
cordic.sv       # CORDIC for sin/cos computation in rotation matrix
sqrt.sv         # Square root for innovation gates (used in MOD_5)
spi_master.sv   # ICM-42688 SPI interface (instantiated in MOD_5)
```

---

## Module Interconnections

```
MOD_1 ── ce_100hz ─────────────────────────────► MOD_4 (prediction trigger)
MOD_5 ── gyro_scaled / accel_scaled ───────────► MOD_4 (IMU measurements)
MOD_5 ── state_updated (from EKF update) ──────► MOD_4 (previous state)
MOD_12 (BRAM) ─── cov_p_in ──────────────────► MOD_4 (covariance P)
MOD_10 ── q_diag (process noise) ─────────────► MOD_4 (noise parameters)
MOD_4 ── state_predicted ──────────────────────► MOD_5 (EKF update engine)
MOD_4 ── cov_p_predicted ──────────────────────► BRAM → MOD_5
MOD_4 ── ekf_predict_halted ───────────────────► MOD_9 (watchdog / emergency)
```

---

## Design Considerations

- **Numerical stability:** Joseph form P = (I−K·H)·P·(I−K·H)ᵀ + K·R·Kᵀ preserves symmetric positive-definiteness better than the standard P = (I−K·H)·P form.
- **CORDIC latency:** The rotation matrix requires 3 CORDIC computations (φ, θ, ψ). CORDIC core latency is ~16 cycles at 50 MHz. Prediction runs every 500 000 cycles, giving plenty of margin.
- **BRAM P storage:** The 9×9 symmetric matrix requires 45 unique Q2.46 (48-bit) values = 216 bytes. Stored in a BRAM18 with single-port access sequenced by the prediction FSM.
- **Positive-definiteness check:** Diagonal elements of P are checked to be > 0 after each prediction. If any diagonal goes negative (numerical fault), P is reset to a safe diagonal initial value and `ekf_predict_halted` is asserted.
- **Resource estimate:**
  - DSP48E1: ~12 (matrix multiply for F·P·Fᵀ)
  - BRAM18: 1 (9×9 covariance P = 486 bytes)
  - LUTs: ~900
  - FFs: ~700

---

## Testing & Verification

| Test Point                       | Verification Method                                                  |
|----------------------------------|----------------------------------------------------------------------|
| Kinematic equations correctness  | Compare against MATLAB/Python EKF reference at known flight segments |
| Rotation matrix orthogonality    | Verify R·Rᵀ ≈ I to within Q4.28 precision                          |
| Covariance positive-definiteness | Check all P diagonal > 0 after 1000 prediction steps                |
| IMU scale factor accuracy        | Inject raw LSB; verify output matches ±4 g and ±250°/s bounds       |
| Reset on PD fault                | Force P diagonal negative; verify halted flag and P reset           |
| Joseph form vs standard form     | Compare with MATLAB: Joseph form should stay PD after 10 000 steps  |

---

## Optimization Scope

| Area            | Opportunity                                                                  | Impact  |
|-----------------|------------------------------------------------------------------------------|---------|
| **Resource**    | Only store upper triangle of symmetric P (45 elements instead of 81)         | High    |
| **Performance** | Pipeline CORDIC calls for φ, θ, ψ in parallel (3× throughput)               | Medium  |
| **Timing**      | Register BRAM outputs before matrix ALU to break long combinational paths     | Medium  |
| **Power**       | Gate DSP blocks between ce_100hz cycles                                       | Medium  |
| **Area**        | Combine imu_scaler with MOD_5 to share shift-register scaling logic           | Low     |

---

*Module 4 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
