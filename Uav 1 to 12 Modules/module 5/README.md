# UAV Module 5: Sensor Interface + EKF Measurement Updates

## Overview

UAV Module 5 is the primary sensor front-end and EKF measurement update engine. It interfaces with three physical sensors (ICM-42688 IMU via SPI, BMP388 barometer via I2C, and a HMC5883L-compatible magnetometer via I2C), applies cascaded IIR low-pass filtering and calibration, and runs the three EKF measurement update steps (GPS, barometer, magnetometer) using the Joseph form Kalman gain. Innovation gating rejects outlier measurements before they corrupt the state estimate.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🔴 CRITICAL — Only module that corrects EKF state from physical measurements

---

## Key Functionality

### Layer 1 — Physical Sensor Communication
- **ICM-42688 SPI** (100 Hz): 6-axis gyro/accel readout, ±250°/s, ±4 g → Q4.28
- **BMP388 I2C** (50 Hz, addr 0x76): Pressure + temperature → altitude via 44330 × (1 − (P/P₀)^0.1903) → Q10.22
- **Magnetometer I2C** (10 Hz, addr 0x1E): 3-axis field → hard-iron offset correction → heading atan2(Y,X) → Q4.28 rad
- **Cascaded IIR LPF** per sensor: 30 Hz cutoff (IMU), 10 Hz cutoff (Baro), 2 Hz cutoff (Mag)

### Layer 2 — EKF Measurement Updates
- **GPS update** (10 Hz): 5-measurement Joseph form — lat/lon/alt/vN/vE simultaneously
- **Barometer update** (50 Hz): Scalar Joseph form — altitude only
- **Magnetometer update** (10 Hz): Scalar Joseph form — heading, with gimbal lock guard at |θ| > 80°
- **Innovation gating**: Reject measurements > 3σ Mahalanobis distance
- **3-consecutive-outlier isolation**: Per-sensor (IMU / GPS / BARO / MAG) fault flagging

---

## Input Signals

| Signal Name            | Width   | Format  | Source               | Description                                    |
|------------------------|---------|---------|----------------------|------------------------------------------------|
| `clk`                  | 1-bit   | —       | MOD_1                | 50 MHz system clock                            |
| `rst_n`                | 1-bit   | —       | MOD_1                | Active-low synchronous reset                   |
| `ce_100hz`             | 1-bit   | —       | MOD_1                | IMU sampling strobe                            |
| `ce_50hz`              | 1-bit   | —       | MOD_1                | Barometer sampling strobe                      |
| `ce_10hz`              | 1-bit   | —       | MOD_1                | GPS/magnetometer strobe                        |
| `imu_miso`             | 1-bit   | SPI     | ICM-42688 (hardware) | SPI MISO from IMU                              |
| `baro_sda` / `baro_scl`| 1-bit ea| I2C     | BMP388 (hardware)    | I2C bus to barometer at 0x76                   |
| `mag_sda` / `mag_scl`  | 1-bit ea| I2C     | Mag sensor (hardware)| I2C bus to magnetometer at 0x1E                |
| `gps_lat_raw`          | 32-bit  | Q10.22  | MOD_6 NMEA parser    | GPS latitude measurement                       |
| `gps_lon_raw`          | 32-bit  | Q10.22  | MOD_6 NMEA parser    | GPS longitude measurement                      |
| `gps_alt_raw`          | 32-bit  | Q10.22  | MOD_6 NMEA parser    | GPS altitude measurement (MSL)                 |
| `gps_vn_raw`           | 32-bit  | Q4.28   | MOD_6 NMEA parser    | GPS North velocity                             |
| `gps_ve_raw`           | 32-bit  | Q4.28   | MOD_6 NMEA parser    | GPS East velocity                              |
| `gps_fix_valid`        | 1-bit   | —       | MOD_6               | GPS fix quality gate (3D fix AND HDOP ≤ 2.5)   |
| `state_predicted[8:0]` | 32-bit  | mixed   | MOD_4               | Predicted 9-state vector from EKF predict       |
| `cov_p_predicted[80:0]`| 48-bit  | Q2.46   | MOD_4/BRAM          | Predicted covariance P                         |
| `gyro_bias[2:0]`       | 32-bit  | Q4.28   | MOD_10 AXI          | IMU gyro bias calibration (x/y/z)              |
| `accel_bias[2:0]`      | 32-bit  | Q4.28   | MOD_10 AXI          | IMU accel bias calibration                     |
| `mag_hard_iron[2:0]`   | 32-bit  | Q4.28   | MOD_10 AXI          | Magnetometer hard-iron offset (x/y/z)          |
| `baro_sea_level_p0`    | 32-bit  | Pa      | MOD_10 AXI          | Reference sea-level pressure for altitude calc  |

---

## Output Signals

| Signal Name              | Width  | Format  | Destination             | Description                              |
|--------------------------|--------|---------|-------------------------|------------------------------------------|
| `imu_sclk`               | 1-bit  | SPI     | ICM-42688 (hardware)    | SPI clock to IMU                         |
| `imu_mosi`               | 1-bit  | SPI     | ICM-42688 (hardware)    | SPI MOSI to IMU                          |
| `imu_cs_n`               | 1-bit  | SPI     | ICM-42688 (hardware)    | SPI chip-select (active-low)             |
| `gyro_scaled[2:0]`       | 32-bit | Q4.28   | MOD_4 (predict), MOD_2  | Calibrated gyro rates (rad/s)            |
| `accel_scaled[2:0]`      | 32-bit | Q4.28   | MOD_4 (predict)         | Calibrated accelerations (m/s²)          |
| `altitude_scaled`        | 32-bit | Q10.22  | MOD_4 (baro update)     | Barometric altitude (m)                  |
| `heading_scaled`         | 32-bit | Q4.28   | MOD_4 (mag update)      | Magnetic heading (rad)                   |
| `state_updated[8:0]`     | 32-bit | mixed   | MOD_4 (next prediction) | EKF-corrected 9-state vector             |
| `cov_p_updated[80:0]`    | 48-bit | Q2.46   | BRAM → MOD_4            | Updated covariance matrix                |
| `ekf_measurement_outliers`| 8-bit | bitmask | MOD_9 (health)          | Per-sensor outlier count / fault flags   |
| `baro_valid`             | 1-bit  | —       | MOD_9 (preflight)       | Barometer data valid flag                |

---

## Architecture

### High-Level Components

```
         ┌───────────────────────────────────────────────────────┐
         │             module5_uav_sensor_ekf.sv                 │
         │                                                       │
IMU SPI ►│  spi_master.sv ─► imu_scaler ─► lpf.sv (30Hz)       ├──► gyro/accel_scaled
Baro I2C►│  i2c_master.sv ─► baro_calc  ─► lpf.sv (10Hz)       ├──► altitude_scaled
Mag I2C ►│  i2c_master.sv ─► atan2_hdg ─► lpf.sv (2Hz)         ├──► heading_scaled
         │                                                       │
GPS data►│  innovation_gate.sv ──────────────────────────────── │
         │  (3σ Mahalanobis rejection)                          │
         │                                                       │
state_in►│  ekf_gps_update.sv   (5-meas, 10Hz, Joseph form)    ├──► state_updated
cov_p_in►│  ekf_baro_update.sv  (1-meas, 50Hz, Joseph form)    ├──► cov_p_updated
         │  ekf_mag_update.sv   (1-meas, 10Hz, Joseph form)    │
         │  (gimbal lock guard at |θ|>80°)                      │
         └───────────────────────────────────────────────────────┘
```

### Reused Modules from RTL_20

| Module           | Role                                                         |
|------------------|--------------------------------------------------------------|
| `spi_master.sv`  | SPI Mode 0, 24 MHz, for ICM-42688 IMU reads at 100 Hz        |
| `i2c_master.sv`  | I2C master for BMP388 (0x76) and magnetometer (0x1E)         |
| `lpf.sv`         | Cascaded IIR LPF: 30 Hz (IMU), 10 Hz (Baro), 2 Hz (Mag)     |
| `kalman_1v.sv`   | Single scalar Kalman update template (used in baro/mag steps)|
| `fp_divider.sv`  | Fixed-point divider for H·P·Hᵀ inversion in GPS update       |

### New Modules Created

| Module                | Role                                                          |
|-----------------------|---------------------------------------------------------------|
| `ekf_gps_update.sv`   | 5-measurement Joseph form GPS update at 10 Hz                 |
| `ekf_baro_update.sv`  | Scalar barometric altitude Joseph form update at 50 Hz        |
| `ekf_mag_update.sv`   | Scalar heading Joseph form update, gimbal lock guard at 10 Hz |
| `innovation_gate.sv`  | Mahalanobis distance gating: reject > 3σ, count consecutive  |

---

## Data Formats

| Sensor / Output         | Format   | Range            | Notes                                     |
|-------------------------|----------|------------------|-------------------------------------------|
| Gyro (raw LSB)          | 16-bit int | ±32 768 LSB    | 131 LSB/°/s (±250°/s range ICM-42688)     |
| Gyro (scaled Q4.28)     | Q4.28    | ±4.36 rad/s      | After calibration and LPF                 |
| Accel (raw LSB)         | 16-bit int | ±32 768 LSB    | 2048 LSB/g (±4 g range ICM-42688)         |
| Accel (scaled Q4.28)    | Q4.28    | ±39.24 m/s²      | After calibration and LPF                 |
| Baro altitude           | Q10.22   | 0–500 m AGL      | 44330×(1−(P/P₀)^0.1903) formula           |
| Mag heading             | Q4.28    | 0–2π rad         | atan2(Y_corr, X_corr) after hard-iron     |
| GPS position            | Q10.22   | ±180°/±360° µ°   | DDMM.MMMM → µ° conversion                |
| GPS velocity            | Q4.28    | ±100 m/s         | Knots × 0.51444                           |
| Covariance P            | Q2.46    | positive-definite| 9×9 symmetric, BRAM stored                |
| Innovation gate threshold | Q4.12  | 0–3σ             | Configurable from MOD_10                  |

---

## Register Interface (AXI4-Lite via MOD_10)

| Register Name          | Address (MOD_10) | Access | Format  | Description                          |
|------------------------|------------------|--------|---------|--------------------------------------|
| `IMU_GYRO_BIAS_X`      | 0x88             | R/W    | Q4.28   | Gyro X bias correction               |
| `IMU_GYRO_BIAS_Y`      | 0x8C             | R/W    | Q4.28   | Gyro Y bias correction               |
| `IMU_GYRO_BIAS_Z`      | 0x90             | R/W    | Q4.28   | Gyro Z bias correction               |
| `IMU_ACCEL_BIAS_X`     | 0x94             | R/W    | Q4.28   | Accel X bias correction              |
| `IMU_ACCEL_BIAS_Y`     | 0x98             | R/W    | Q4.28   | Accel Y bias correction              |
| `IMU_ACCEL_BIAS_Z`     | 0x9C             | R/W    | Q4.28   | Accel Z bias correction              |
| `MAG_HARD_IRON_X`      | 0xA0             | R/W    | Q4.28   | Magnetometer hard-iron X offset      |
| `MAG_HARD_IRON_Y`      | 0xA4             | R/W    | Q4.28   | Magnetometer hard-iron Y offset      |
| `MAG_HARD_IRON_Z`      | 0xA8             | R/W    | Q4.28   | Magnetometer hard-iron Z offset      |
| `BARO_SEA_LEVEL_P0`    | 0xAC             | R/W    | 32-bit Pa | Reference pressure (default 101325 Pa)|

---

## File Structure

```
Uav 1 to 12 Modules/module 5/
├── module5_uav_sensor_ekf.sv    # Top-level sensor + EKF update wrapper
├── ekf_gps_update.sv            # GPS 5-measurement Joseph form update
├── ekf_baro_update.sv           # Barometric altitude scalar update
├── ekf_mag_update.sv            # Magnetometer heading scalar update
└── innovation_gate.sv           # Mahalanobis distance outlier gate
```

### Dependencies (from RTL_20 shared library)

```
spi_master.sv   # ICM-42688 SPI interface (24 MHz, Mode 0)
i2c_master.sv   # BMP388 + magnetometer I2C (100/400 kHz)
lpf.sv          # Cascaded IIR low-pass filter
kalman_1v.sv    # Scalar Kalman update template
fp_divider.sv   # Fixed-point divider for S = H·P·Hᵀ + R inversion
```

---

## Module Interconnections

```
ICM-42688 (hardware) ── SPI ────────────────────► MOD_5 (gyro/accel data)
BMP388 (hardware) ───── I2C ────────────────────► MOD_5 (pressure/temperature)
Mag sensor (hardware) ── I2C ───────────────────► MOD_5 (magnetic field)
MOD_6 ── gps_data ──────────────────────────────► MOD_5 (GPS measurements)
MOD_4 ── state_predicted ───────────────────────► MOD_5 (prior state)
MOD_10 ── calibration registers ────────────────► MOD_5 (bias/offset/P₀)
MOD_5 ── gyro/accel_scaled ─────────────────────► MOD_4 (EKF predict input)
MOD_5 ── state_updated / cov_p_updated ─────────► MOD_4 (feedback for next predict)
MOD_5 ── ekf_measurement_outliers ──────────────► MOD_9 (health watchdog)
MOD_5 ── baro_valid ────────────────────────────► MOD_9 (preflight checker)
```

---

## Design Considerations

- **I2C bus sharing:** BMP388 (0x76) and magnetometer (0x1E) share a single I2C bus. The `i2c_master` sequencer ensures non-overlapping transactions at each respective CE strobe.
- **SPI speed:** ICM-42688 supports 24 MHz SPI Mode 0. The 50 MHz system clock requires a ÷2 divider inside `spi_master.sv` to achieve valid 24 MHz.
- **Altitude formula:** The barometric altitude uses the ISA formula; temperature compensation requires the BMP388 compensation registers read during initialization.
- **Gimbal lock guard:** The magnetometer heading update is disabled when |pitch| > 80° to avoid singularities in the spherical heading computation.
- **Resource estimate:**
  - DSP48E1: ~10 (Kalman gain computation, matrix operations)
  - BRAM18: 1 (measurement noise R matrix, innovation history)
  - LUTs: ~1 200
  - FFs: ~900

---

## Testing & Verification

| Test Point                         | Verification Method                                              |
|------------------------------------|------------------------------------------------------------------|
| IMU scale factor accuracy          | Inject ±32 768 LSB; verify ±4.36 rad/s and ±39.24 m/s²         |
| Baro altitude at 101325 Pa         | Inject P=101325, T=25°C; verify alt=0 m                          |
| GPS update convergence             | Simulate 1 m position error; verify EKF corrects within 5 cycles |
| Innovation gate rejection          | Inject 5σ outlier GPS; verify ignored and fault flag asserts     |
| 3-consecutive-outlier isolation    | Send 3 consecutive outliers; verify sensor_fault flag asserts    |
| Gimbal lock guard (pitch=85°)      | Set EKF pitch >80°; verify mag update disabled                   |
| LPF cutoff @ 30 Hz                 | Inject 50 Hz sine on gyro; verify ≥40 dB attenuation            |

---

## Optimization Scope

| Area            | Opportunity                                                                  | Impact  |
|-----------------|------------------------------------------------------------------------------|---------|
| **Resource**    | Share single Joseph-form Kalman engine across GPS/baro/mag via state machine | High    |
| **Performance** | Overlap GPS and baro updates on separate pipeline stages                     | Medium  |
| **Power**       | Disable I2C master state machine between ce_50hz and ce_10hz strobes         | Medium  |
| **Timing**      | Break long path from fp_divider output to Kalman gain register               | Low     |
| **Area**        | Combine baro temperature compensation with LPF into single pipeline          | Low     |

---

*Module 5 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
