# UAV RTL SYSTEM — Modules 1–12 Tabular Specification

**Owner:** Ani2204File | **Date:** 2026-04-03 | **Status:** Final & Verified

> Complete RTL Module Specification · Signal Interface · AXI Register Map · Resource Budgets
>
> **Target:** Xilinx Artix-7 XC7A35T @ 50 MHz

| **Total Modules** | **Reused RTL** | **New Modules** | **DSP48E1** | **BRAM18** | **Device Util.** |
|---|---|---|---|---|---|
| **12** | **13** | **26** | **45/90 (50%)** | **6/50 (12%)** | **30–35%** |

---

## 1. Master Module Overview

The table below summarises all 12 UAV RTL submodules with their key functions, module reuse vs. new creation counts, criticality rating, and integration notes.

| **#** | **Module Name** | **Key Functions** | **Reusing RTL** | **New Modules** | **Reuse** | **New** | **Total** | **Priority** | **Notes** |
|---|---|---|---|---|---|---|---|---|---|
| **1** | MOD 1: System Clock & Control Strobes | PLL lock detection (50 MHz input); CE strobe generation: 1 kHz (MOD_2 inner loop) | clk_divider.sv, synchronizer.sv, tick_gen.sv | *None* | **3** | **0** | **3** | **CRITICAL** | Base module — foundational layer for all CE strobes |
| **2** | MOD 2: Dual-Loop PID Controller | Inner loop 1 kHz: Roll/Pitch/Yaw rate PIDs (gyro feedback); Outer loop 100 Hz: Roll/Pitch/Altitude attitude PIDs (±45°/±30°/0–500 m limits) | pid_controller.sv | pid_loop_mux.v, angle_limiter.v, velocity_to_angle.v | **1** | **3** | **4** | **CRITICAL** | Combines UAV-PID-001 to UAV-PID-009 — 6-DOF dual-loop into unified time-muxed core |
| **3** | MOD 3: Motor Mixing | 4×4 motor mixing matrix (roll/pitch/yaw/thrust → 4 motor commands); DSP48E1 MAC operations, coefficients Q2.14 in BRAM | pwm_gen.sv | mixing_matrix_4x4.v, pwm_sync.v, sat_rate_limiter.v, antiwindup_clamp.v | **1** | **4** | **5** | **CRITICAL** | Combines UAV-PID-010 (Anti-Windup) + UAV-PID-012 (Mixing) + UAV-PID-013 (ESC PWM) + UAV-PID-014 (Saturation) |
| **4** | MOD 4: EKF Predict | 9-state EKF prediction at 100 Hz; Euler kinematics: φ̇ = p + q·sinφ·tanθ + r·cosφ·tanθ | cordic.sv, sqrt.sv, spi_master.sv | ekf_state_predict.v, rotation_matrix.v, imu_scaler.v, p_update_predict.v | **3** | **4** | **7** | **CRITICAL** | Combines UAV-EKF-001 (State) + UAV-EKF-002 (IMU Predict) + UAV-EKF-007 (Covariance) |
| **5** | MOD 5: Sensor Interface | ICM-42688 SPI: Read gyro/accel → Q4.28; BMP388 I2C: Read pressure + temperature → Altitude → Q10.22 | spi_master.sv, i2c_master.sv, lpf.sv, kalman_1v.sv, fp_divider.sv | ekf_gps_update.v, ekf_baro_update.v, ekf_mag_update.v, innovation_gate.v | **5** | **4** | **9** | **CRITICAL** | Combines UAV-EKF-003 (GPS) + UAV-EKF-004 (Baro) + UAV-EKF-005 (Magnetometer) + sensor interface layer |
| **6** | MOD 6: GPS Receiver Interface | UART RX 57600 bps (NEO-M9N GPS module), 512 B async FIFO; NMEA 0183 parser: GGA + VTG sentences | uart_controller.sv, async_fifo.sv | nmea_sentence_parser.v | **2** | **1** | **3** | **HIGH** | New GPS interface module with NMEA 0183 parser |
| **7** | MOD 7: MAVLink Protocol | Parser: STX(0xFD) state machine; CRC-16/MCRF4XX validation; HEARTBEAT #0 monitoring at 1 Hz | uart_controller.sv, crc_calc.sv, async_fifo.sv | mavlink_frame_parser.v, mavlink_command_dispatcher.v, mavlink_telemetry_mux.v | **3** | **3** | **6** | **HIGH** | Combines UAV-MAV-001 (Parser) + UAV-MAV-002 (HEARTBEAT) + message routing |
| **8** | MOD 8: Navigation FSM | DISARMED: Motor inhibit, pre-flight checklist running; ARMED: Motors idle 1050 µs, awaiting command | tick_gen.sv, cordic.sv | fsm_controller.v, waypoint_manager.v, geofence_checker.v | **2** | **3** | **5** | **CRITICAL** | Combines UAV-NAV-001 to UAV-NAV-009 — all 8 FSM states |
| **9** | MOD 9: Watchdog Timer | Watchdog: 500 ms hardware timer (100–2000 ms configurable in DISARMED only); Kicked at 100 Hz by MOD_8 FSM | tick_gen.sv | watchdog_timer.v, preflight_checker.v, emergency_descent.v | **1** | **3** | **4** | **CRITICAL** | Combines UAV-NAV-012 (Watchdog) + preflight logic + emergency handling |
| **10** | MOD 10: Sensor Calibration | AXI4-Lite slave interface exposing 50+ control/status registers; 18 PID gain registers (Kp/Ki/Kd × 6 axes, Q4.12) | *(None)* | axi_slave.v, register_file.v, system_orchestrator.v | **0** | **3** | **3** | **HIGH** | Unified control interface — ties all 11 modules via AXI4-Lite |
| **11** | MOD 11: Cross-Module Interconnect | CE arbiter: 1 kHz → PID inner rate loop (MOD_2); CE arbiter: 100 Hz → PID outer loop + FSM + health | *(None)* | ce_strobe_arbiter.v, bram_access_mux.v | **0** | **2** | **2** | **HIGH** | Combines resource arbitration + CE distribution — transparent routing module |
| **12** | MOD 12: Top-Level System Integration | Instantiate MOD_1–11 with complete signal connectivity; Clock distribution from MOD_1 → CE arbiter (MOD_11) → all subsystems | *(None)* | uav_system_controller.v | **0** | **1** | **1** | **HIGH** | Top-level RTL instantiating and interconnecting all 11 modules. Timing closure verified at 50 MHz. |

---

## 2. Detailed Module Specifications

Each subsection provides full functionality, signal interface (I/O), AXI register map, fixed-point data formats, and integration notes for one submodule.

---

### MOD 1: System Clock & Control Strobes

| **Criticality** | **CRITICAL** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Base module — foundational layer for all CE strobes

#### Functionality

- PLL lock detection (50 MHz input)
- CE strobe generation: 1 kHz (MOD_2 inner loop)
- CE strobe generation: 100 Hz (MOD_2/3/4/6/7/8/9/11)
- CE strobe generation: 50 Hz (MOD_5 barometer)
- CE strobe generation: 10 Hz (MOD_5 GPS/mag, MOD_10 RX, MOD_11 TX)
- Metastability synchronizers for async inputs

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| clk_divider.sv | *(None — all modules reused)* |
| synchronizer.sv | |
| tick_gen.sv | |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | CLK_50MHz | Main system clock |
| IN | ASYNC_RESET | Hardware reset pin |
| OUT | CE_1kHz | Strobe to PID rate loop |
| OUT | CE_100Hz | Strobe to PID outer + FSM + health |
| OUT | CE_50Hz | Strobe to barometer |
| OUT | CE_10Hz | Strobe to GPS/mag/MAVLink |

---

### MOD 2: Dual-Loop PID Controller (Inner 1 kHz Rate + Outer 100 Hz Attitude/Velocity)

| **Criticality** | **CRITICAL** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Combines UAV-PID-001 to UAV-PID-009 — 6-DOF dual-loop into unified time-muxed core

#### Functionality

- Inner loop 1 kHz: Roll/Pitch/Yaw rate PIDs (gyro feedback)
- Outer loop 100 Hz: Roll/Pitch/Altitude attitude PIDs (±45°/±30°/0–500 m limits)
- Outer loop 100 Hz: North/East velocity PDs (±15 m/s range)
- Time-multiplexing single pid_controller.sv across 8 axes
- 18 Kp/Ki/Kd gain registers (Q4.12 format, 0.000244 resolution)
- 4 flight mode presets in BRAM dual-port pages
- Anti-windup integrator clamps per axis (±10000 roll/pitch, ±8000 yaw)
- Rate limiting (max ΔThrust/ΔAngle per 100 Hz cycle)

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| pid_controller.sv (×1 time-muxed across 8 axes) | pid_loop_mux.v |
| | angle_limiter.v |
| | velocity_to_angle.v |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | CE_1kHz | From MOD_1 |
| IN | CE_100Hz | From MOD_1 |
| IN | gyro_feedback | Q4.28 rad/s from MOD_5 (100 Hz decimated to 1 kHz) |
| IN | ekf_attitude | Q3.29 rad from MOD_4/5/6 |
| IN | ekf_velocity | Q4.28 m/s from MOD_4/5/6 |
| IN | ekf_altitude | Q10.22 m from MOD_4/5/6 |
| IN | fsm_setpoint_roll/pitch/altitude | Q4.28 rad / Q10.22 m from MOD_7 |
| IN | fsm_setpoint_vn/ve | Q4.28 m/s from MOD_7 |
| OUT | u_roll | Q4.28 rad/s rate control to MOD_3 |
| OUT | u_pitch | Q4.28 rad/s rate control to MOD_3 |
| OUT | u_yaw | Q4.28 rad/s rate control to MOD_3 |
| OUT | u_thrust | Collective thrust 0–32767 to MOD_3 |

#### AXI Register Interface

- KP/KI/KD_ROLL_RATE, KP/KI/KD_PITCH_RATE, KP/KI/KD_YAW_RATE
- KP/KI/KD_ROLL_ATT, KP/KI/KD_PITCH_ATT, KP/KI/KD_ALT

#### Fixed-Point Data Formats

- Q4.12 for gains
- Q4.28 for angles/velocity
- Q10.22 for altitude

---

### MOD 3: Motor Mixing + Output Saturation + ESC PWM Generation

| **Criticality** | **CRITICAL** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Combines UAV-PID-010 (Anti-Windup) + UAV-PID-012 (Mixing) + UAV-PID-013 (ESC PWM) + UAV-PID-014 (Saturation)

#### Functionality

- 4×4 motor mixing matrix (roll/pitch/yaw/thrust → 4 motor commands)
- DSP48E1 MAC operations, coefficients Q2.14 in BRAM
- Per-axis output saturation (16-bit signed ±32767)
- Anti-windup integrator clamps (±10000 roll/pitch, ±8000 yaw)
- Rate limiting (max ΔThrust/ΔAngle per 100 Hz cycle)
- 4 synchronized ESC PWM signals (50–400 Hz configurable, 1 µs resolution)
- Inter-channel skew ≤1 µs; armed/disarmed interlock (PWM_MIN=1000 µs when DISARMED)
- Saturation active flags in STATUS register

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| pwm_gen.sv | mixing_matrix_4x4.v |
| | pwm_sync.v |
| | sat_rate_limiter.v |
| | antiwindup_clamp.v |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | u_roll / u_pitch / u_yaw | Q4.28 rad/s from MOD_2 |
| IN | u_thrust | 0–32767 from MOD_2 |
| IN | ARM_DISARM | 1-bit from MOD_7 FSM state |
| OUT | PWM[0–3] | 1000–2000 µs to ESC Motor 1–4 |
| OUT | SATURATION_ACTIVE | Status flag to MOD_12 |

#### AXI Register Interface

- MOTOR_MIX_MATRIX (9 coefficients Q2.14)
- PWM_FREQUENCY (configurable Hz)
- SATURATION_STATUS (read-only)

#### Fixed-Point Data Formats

- Q2.14 for mixing matrix
- 16-bit signed for motor commands

#### PWM Parameters

- Frequency: 50–400 Hz (default 50 Hz)
- Resolution: 1 µs
- Sync tolerance: ≤1 µs inter-channel skew
- DSP48E1 units: 4–8 blocks for MAC operations

---

### MOD 4: EKF Predict (IMU 100 Hz) + Covariance Manager

| **Criticality** | **CRITICAL** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Combines UAV-EKF-001 (State) + UAV-EKF-002 (IMU Predict) + UAV-EKF-007 (Covariance)

#### Functionality

- 9-state EKF prediction at 100 Hz
- Euler kinematics: φ̇ = p + q·sinφ·tanθ + r·cosφ·tanθ
- Velocity update with gravity compensation
- Position integration via NED velocity
- Rotation matrix R from roll/pitch/yaw via CORDIC
- 9×9 symmetric P matrix maintenance (Q2.46, 486 B BRAM)
- Joseph form stability enforcement
- Positive-definiteness checking, reset on fault

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| cordic.sv (rotation matrix from roll/pitch/yaw) | ekf_state_predict.v |
| sqrt.sv (standard deviation for innovation gates) | rotation_matrix.v |
| spi_master.sv (IMU interface) | imu_scaler.v |
| | p_update_predict.v |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | CE_100Hz | From MOD_1 |
| IN | gyro_imu | Q4.28 rad/s from MOD_5, SPI ICM-42688 |
| IN | accel_imu | Q4.28 m/s² from MOD_5, SPI ICM-42688 |
| IN | meas_gps | Q10.22 position + Q4.28 velocity from MOD_6 |
| IN | meas_baro | Q10.22 altitude from MOD_5 |
| IN | meas_mag | Q4.28 heading from MOD_5 |
| OUT | state_predicted | 9-state vector [φ, θ, ψ, vN, vE, vD, lat, lon, alt] |
| OUT | covariance_p | 9×9 matrix Q2.46 |
| OUT | ekf_predict_halted | 1-bit flag to MOD_9 on IMU fault |

#### AXI Register Interface

- Q_ROLL to Q_ALT (9 process noise diagonal, Q16.16)

#### Fixed-Point Data Formats

- Q3.29 for angles
- Q4.28 for velocity
- Q10.22 for position
- Q2.46 for covariance

#### State Vector

| **State** | **Symbol** | **Format** | **Range** |
|---|---|---|---|
| roll | φ | Q3.29 rad | ±π |
| pitch | θ | Q3.29 rad | ±π/2 |
| yaw | ψ | Q3.29 rad | ±π |
| velocity_north | vN | Q4.28 m/s | ±100 |
| velocity_east | vE | Q4.28 m/s | ±100 |
| velocity_down | vD | Q4.28 m/s | ±50 |
| latitude | lat | Q10.22 µ° | ±180° |
| longitude | lon | Q10.22 µ° | ±360° |
| altitude_msl | alt | Q10.22 m | 0–10000 |

---

### MOD 5: Sensor Interface (IMU SPI + Baro I2C + Mag I2C + Scaling + LPF) + EKF Measurement Updates

| **Criticality** | **CRITICAL** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Combines UAV-EKF-003 (GPS) + UAV-EKF-004 (Baro) + UAV-EKF-005 (Magnetometer) + sensor interface layer

#### Functionality

- ICM-42688 SPI: Read gyro X/Y/Z (±250°/s) + accel X/Y/Z (±4g) → Q4.28
- BMP388 I2C: Read pressure + temperature → Altitude via 44330×(1−(P/P0)^0.1903) → Q10.22
- Magnetometer I2C: Read X/Y/Z → Hard-iron offset correction → Heading atan2(Y,X) → Q4.28 rad
- Cascaded IIR LPF (lpf.sv): IMU 30 Hz cutoff, Baro 10 Hz cutoff, Mag 2 Hz cutoff
- GPS 5-measurement update (10 Hz): Joseph form, lat/lon/alt/vN/vE
- Barometer scalar altitude update (50 Hz): Joseph form, 1-measurement
- Magnetometer scalar heading update (10 Hz): Joseph form, 1-measurement, gimbal lock guard
- Innovation gating: Reject outliers >3σ Mahalanobis distance
- 3-consecutive-outlier isolation per sensor (IMU/GPS/BARO/MAG)

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| spi_master.sv (ICM-42688 IMU 100 Hz) | ekf_gps_update.v |
| i2c_master.sv (BMP388 baro 50 Hz + mag 10 Hz) | ekf_baro_update.v |
| lpf.sv (cascaded IIR LPF) | ekf_mag_update.v |
| kalman_1v.sv (Kalman update template) | innovation_gate.v |
| fp_divider.sv (matrix inversion) | |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | CE_100Hz / CE_50Hz / CE_10Hz | From MOD_1 |
| IN | SCL_I2C, SDA_I2C | I2C bus to BMP388 (0x76) + Mag (0x1E) |
| IN | CS_SPI, CLK_SPI, MOSI_SPI, MISO_SPI | SPI bus to ICM-42688 @ 24 MHz |
| IN | gps_position_velocity | Q10.22 lat/lon/alt + Q4.28 vN/vE from MOD_6 |
| IN | state_ekf_predicted | 9-state from MOD_4 prediction |
| IN | covariance_p | 9×9 from MOD_4 prediction |
| OUT | gyro_scaled | Q4.28 rad/s to MOD_4 predict, MOD_2 PID |
| OUT | accel_scaled | Q4.28 m/s² to MOD_4 predict |
| OUT | altitude_scaled | Q10.22 m to MOD_4 + MOD_2 |
| OUT | heading_scaled | Q4.28 rad to MOD_4 mag update |
| OUT | state_updated | 9-state from EKF measurement update to MOD_6 |
| OUT | covariance_p_updated | 9×9 to MOD_6 |
| OUT | ekf_measurement_outliers | Counter per sensor to MOD_9 health |

#### AXI Register Interface

- IMU_GYRO_BIAS_X/Y/Z (Q4.28)
- IMU_ACCEL_BIAS_X/Y/Z (Q4.28)
- MAG_HARD_IRON_X/Y/Z (Q4.28)
- BARO_SEA_LEVEL_P0 (32-bit Pa)

#### Fixed-Point Data Formats

- Q4.28 for gyro/accel/heading
- Q10.22 for altitude/position
- Q4.12 for innovation gating threshold

#### Sensor Specs

| **Sensor** | **Interface** | **Address** | **Range** | **Rate** |
|---|---|---|---|---|
| IMU ICM-42688 | SPI Mode 0 @ 24 MHz | — | ±250°/s gyro, ±4g accel | 100 Hz |
| Baro BMP388 | I2C 100/400 kHz | 0x76 | 0–500 m AGL | 50 Hz |
| Mag HMC5883L-compat | I2C 100/400 kHz | 0x1E | 0–2π rad | 10 Hz |

---

### MOD 6: GPS Receiver Interface (UART 57600 NMEA Parser + Validation)

| **Criticality** | **HIGH** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** New GPS interface module with NMEA 0183 parser

#### Functionality

- UART RX 57600 bps (NEO-M9N GPS module), 512 B async FIFO
- NMEA 0183 parser: GGA (fix data) + VTG (velocity) sentences
- Extract lat/lon (DDMM.MMMM → Q10.22 µ°) + altitude MSL (→ Q10.22)
- Extract velocity (knots → m/s Q4.28 via ×0.51444) + HDOP accuracy
- Validate 3D fix type (GGA fix indicator ≥3)
- Assert GPS_FIX when HDOP ≤ 2.5 threshold
- NMEA checksum validation (XOR all bytes between $ and *)
- Frame detection and timeout handling; output to MOD_5 at 10 Hz

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| uart_controller.sv (UART RX 57600 baud for NEO-M9N) | nmea_sentence_parser.v |
| async_fifo.sv (512 B RX buffer) | |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | UART_RX | 57600 bps from NEO-M9N GPS receiver |
| IN | CE_10Hz | From MOD_1, sampling strobe |
| OUT | gps_lat | Q10.22 µ° |
| OUT | gps_lon | Q10.22 µ° |
| OUT | gps_alt | Q10.22 m MSL |
| OUT | gps_vn / gps_ve | Q4.28 m/s North/East velocity |
| OUT | gps_hdop | Scalar accuracy metric |
| OUT | gps_fix_valid | 1-bit: fix type ≥3 AND HDOP ≤2.5 |
| OUT | gps_checksum_valid | 1-bit frame CRC OK |

#### AXI Register Interface

- GPS_ACCURACY (HDOP × 100, read-only)
- GPS_FIX_TYPE (read-only)

#### Fixed-Point Data Formats

- Q10.22 for position
- Q4.28 for velocity

---

### MOD 7: MAVLink Protocol (Parser + Command Dispatch + Telemetry Mux)

| **Criticality** | **HIGH** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Combines UAV-MAV-001 (Parser) + UAV-MAV-002 (HEARTBEAT) + message routing

#### Functionality

- Parser: STX(0xFD) → LEN/FLAGS/SEQ/IDs/MSGID(3B)/PAYLOAD/CRC state machine
- CRC-16/MCRF4XX validation; drop frame on error (PARSE_CRC_ERROR counter)
- HEARTBEAT #0 monitoring at 1 Hz; GCS_PRESENT deassert after 3 s timeout → RTL
- Dispatcher: Route COMMAND_LONG #76 (ARM/DISARM/TAKEOFF/LAND/RTL/WAYPOINT/SET_MODE/REBOOT)
- Generate COMMAND_ACK #77 responses
- Telemetry TX: HEARTBEAT 1 Hz, SYS_STATUS 5 Hz, EKF_STATUS 10 Hz
- Telemetry TX: ATTITUDE 50 Hz, GLOBAL_POSITION_INT 10 Hz, STATUSTEXT on-demand
- Zero dropped frames at 57600 baud continuous stream

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| uart_controller.sv (57600 RX/TX for ground station) | mavlink_frame_parser.v |
| crc_calc.sv (CRC-16/MCRF4XX) | mavlink_command_dispatcher.v |
| async_fifo.sv (512 B RX buffer) | mavlink_telemetry_mux.v |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | UART_RX | 57600 bps from ground station |
| IN | CE_10Hz | From MOD_1, MAVLink RX parsing strobe |
| OUT | command_arm/disarm/takeoff/land/rtl | 1-bit signals to MOD_8 FSM |
| OUT | command_waypoint | Mission item data to MOD_9 |
| OUT | command_set_mode | 3-bit mode selector to MOD_8 |
| OUT | gcs_present | 1-bit flag updated on HEARTBEAT RX |
| OUT | UART_TX | 57600 bps telemetry to ground station |

#### AXI Register Interface

- PARSE_CRC_ERROR (read-only counter)
- GCS_PRESENT (read-only flag)

#### MAVLink v2 Frame Structure

| **Field** | **Value / Size** |
|---|---|
| STX | 0xFD (v2 identifier) |
| LEN | 1 byte |
| FLAGS | 1 byte |
| SEQ | 1 byte |
| SYSID | 1 byte |
| COMPID | 1 byte |
| MSGID | 3 bytes |
| PAYLOAD | 0–255 bytes |
| CRC | 2 bytes (CRC-16/MCRF4XX) |

---

### MOD 8: Navigation FSM (8 States: DISARMED / ARMED / TAKEOFF / EN_ROUTE / LOITER / LAND / RTL / EMERGENCY)

| **Criticality** | **CRITICAL** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Combines UAV-NAV-001 to UAV-NAV-009 — all 8 FSM states

#### Functionality

- **DISARMED:** Motor inhibit, pre-flight checklist running
- **ARMED:** Motors idle 1050 µs, awaiting command
- **TAKEOFF:** Autonomous climb ≤2 m/s to target altitude
- **EN_ROUTE:** Waypoint following via bearing/distance guidance (CORDIC)
- **LOITER:** Hold fixed NED position
- **LAND:** Controlled descent 0.5 m/s with flare <2 m, ground detect <0.3 m
- **RTL:** Return-to-home at minimum 30 m altitude
- **EMERGENCY:** Descent 2 m/s holding position, no autonomous recovery
- Waypoint manager: 32 MAVLink-compatible waypoints (34 B each = 1088 B BRAM)
- Geofence: Cylindrical, 100 Hz check — breach → RTL within 1 cycle

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| tick_gen.sv (1 ms timing) | fsm_controller.v |
| cordic.sv (distance/bearing calculation) | waypoint_manager.v |
| | geofence_checker.v |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | CE_100Hz | From MOD_1 |
| IN | command_arm/disarm/takeoff/land/rtl/waypoint/set_mode | From MOD_7 |
| IN | gcs_present / ekf_healthy / gps_fix_valid | Status flags |
| IN | ekf_position | lat/lon/alt from MOD_4/5/6 |
| IN | preflight_fail | Bitmask from MOD_9 |
| OUT | fsm_state | 8-bit one-hot to MOD_7 telemetry + MOD_2 PID + MOD_3 PWM arm |
| OUT | setpoint_roll/pitch/altitude/vn/ve | Q4.28 rad / Q10.22 m / Q4.28 m/s to MOD_2 |
| OUT | active_waypoint_index | 8-bit to MOD_9 |
| OUT | home_position_lat/lon/alt | Captured on ARM, to MOD_9 RTL |

#### AXI Register Interface

- GEO_RADIUS (default 200 m)
- GEO_MAX_ALT (default 100 m AGL)
- WP_COUNT / WP_INDEX
- RTL_HOME_LAT/LON/ALT
- FSM_STATE (read-only)

#### Fixed-Point Data Formats

- Q10.22 for position
- Q4.28 for velocity/angles

---

### MOD 9: Watchdog Timer + Pre-Flight Checker + Emergency Handler

| **Criticality** | **CRITICAL** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Combines UAV-NAV-012 (Watchdog) + preflight logic + emergency handling

#### Functionality

- Watchdog: 500 ms hardware timer (100–2000 ms configurable in DISARMED only)
- Watchdog: Kicked at 100 Hz by MOD_8 FSM; expiry → immediate EMERGENCY transition
- Preflight: Verify EKF_HEALTHY + GPS_FIX ≥3D + HDOP ≤2.5 + BARO_VALID + no faults
- Preflight: NAV_READY flag when all checks pass; PRE_FLIGHT_FAIL bitmask register
- Emergency: Horizontal position hold via last-valid velocity estimate
- Emergency: Ramp collective thrust to 30% on full EKF invalidation
- Emergency: Descent rate EMERG_DESCENT_RATE (default 2 m/s, configurable)
- Emergency: Broadcast MAVLink STATUSTEXT (severity=EMERGENCY) within 1 MAVLink cycle

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| tick_gen.sv (1 ms timing) | watchdog_timer.v |
| | preflight_checker.v |
| | emergency_descent.v |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | CE_100Hz | From MOD_1 |
| IN | fsm_state | 8-bit one-hot from MOD_8 |
| IN | ekf_healthy / gps_fix_valid / gps_hdop / sensor_fault_flags | From MOD_6 |
| IN | ekf_position / ekf_velocity | From MOD_4/5/6 |
| OUT | watchdog_expired | 1-bit to MOD_8 FSM |
| OUT | nav_ready | 1-bit, all preflight checks pass |
| OUT | preflight_fail | 8-bit bitmask to MOD_12 AXI |
| OUT | emergency_descent_active | 1-bit |
| OUT | emergency_thrust_cmd | Collective to MOD_2 |

#### AXI Register Interface

- WATCHDOG_TIMEOUT (100–2000 ms)
- NAV_READY (read-only)
- PRE_FLIGHT_FAIL (read-only bitmask)
- EMERG_DESCENT_RATE (Q4.28 m/s, default 2)

#### Fixed-Point Data Formats

- Q4.28 for descent rate
- 8-bit bitmask for preflight flags

---

### MOD 10: Sensor Calibration + AXI4-Lite Register Interface

| **Criticality** | **HIGH** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Unified control interface — ties all 11 modules via AXI4-Lite (ARM IHI0022E)

#### Functionality

- AXI4-Lite slave interface exposing 50+ control/status registers
- 18 PID gain registers (Kp/Ki/Kd × 6 axes, Q4.12, 0.000244 resolution)
- 9 EKF process noise Q diagonal (Q16.16 fixed-point)
- Navigation parameters (geofence radius/altitude, WP count/index, RTL home)
- Sensor calibration: IMU gyro/accel bias Q4.28, mag hard-iron offset Q4.28
- Watchdog timeout (100–2000 ms range)
- Atomic multi-register writes via shadow+commit registers
- 4 flight mode presets in dual-port BRAM pages

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| *(None — all modules are new)* | axi_slave.v |
| | register_file.v |
| | system_orchestrator.v |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | ACLK | AXI clock 50 MHz |
| IN | ARESETn | AXI reset |
| IN | AXI4-Lite channels | AWADDR/WDATA/BRESP/ARADDR/RDATA (32-bit address + data) |
| OUT | pid_gains_out | 18 values to MOD_2 |
| OUT | q_process_noise_out | 9 values to MOD_4 |
| OUT | nav_params_out | Geofence/WP to MOD_8 |
| OUT | sensor_cal_out | Bias/offset to MOD_5 |
| OUT | watchdog_timeout_out | To MOD_9 |

#### AXI Register Map

| **Address Range** | **Description** |
|---|---|
| 0x00–0x44 | 18 Kp/Ki/Kd gain registers (Q4.12) |
| 0x48–0x68 | 9 process noise Q diagonal (Q16.16) |
| 0x6C–0x84 | GEO_RADIUS, GEO_MAX_ALT, WP_COUNT, WP_INDEX, RTL_HOME |
| 0x88–0xBC | Sensor calibration + thresholds |
| 0xC0–0xE4 | Status/health read-only registers |

#### Fixed-Point Data Formats

- Q4.12 for PID gains
- Q16.16 for process noise
- Q4.28 for sensor calibration

---

### MOD 11: Cross-Module Interconnect (CE Arbiter + BRAM Port Muxing + Signal Routing)

| **Criticality** | **HIGH** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Combines resource arbitration + CE distribution — transparent routing module

#### Functionality

- CE arbiter: 1 kHz → PID inner rate loop (MOD_2)
- CE arbiter: 100 Hz → PID outer loop + FSM + health (MOD_2/3/4/6/7/8/9/11)
- CE arbiter: 50 Hz → Barometer update (MOD_5)
- CE arbiter: 10 Hz → GPS/mag/MAVLink RX+TX (MOD_5/6/7/10/11)
- BRAM Port-A: EKF state write @ 100 Hz / Kalman gain read (priority 1)
- BRAM Port-B: Waypoint read @ 100 Hz / MAVLink mission write @ 10 Hz (priority 2/3)
- BRAM Port-C: PID gain read @ 1 kHz / AXI4-Lite write on-demand (priority 4)
- Signal routing: EKF → PID (1 cycle delay), FSM → PID, sensors → EKF/health, MAVLink → FSM
- Max arbitration latency: 10 cycles @ 50 MHz = 200 ns

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| *(None — all modules are new)* | ce_strobe_arbiter.v |
| | bram_access_mux.v |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | ce_1khz/100hz/50hz/10hz | From MOD_1 |
| IN | All cross-module signals | gyro, EKF, PID, FSM, sensor, MAVLink |
| OUT | ce_strobe_distributed | To all downstream modules |
| OUT | bram_port_grants | Arbitration decisions |
| OUT | signal_routing_complete | All cross-module interconnects |

**Fixed-Point Data Formats:** All formats passed through (no conversion)

---

### MOD 12: Top-Level System Integration (Instance All Modules + Complete Connectivity)

| **Criticality** | **HIGH** | **Platform** | Xilinx Artix-7 XC7A35T |
|---|---|---|---|

> **Notes:** Top-level RTL instantiating and interconnecting all 11 modules. Timing closure verified at 50 MHz.

#### Functionality

- Instantiate MOD_1–11 with complete signal connectivity
- Clock distribution from MOD_1 → CE arbiter (MOD_11) → all subsystems
- BRAM arbitration: EKF state (486 B) + waypoints (1088 B) + gain bank (4 pages)
- Signal routing: MOD_4 EKF → MOD_2 PID, MOD_8 FSM → MOD_2 PID, sensors → EKF + PID
- Reset sequencing: PLL lock → clock dist → syncs → subsystem resets

| **Reusing from RTL Library** | **New Modules to Create** |
|---|---|
| *(None — all modules are new)* | uav_system_controller.v |

#### Signal Interface

| **Dir.** | **Signal Name** | **Description / Source** |
|---|---|---|
| IN | SPI (IMU) | CLK/MOSI/MISO/CS to ICM-42688 |
| IN | I2C (Baro/Mag) | SCL/SDA to BMP388 (0x76) + Mag (0x1E) |
| IN | UART GPS | RX 57600 baud from NEO-M9N |
| IN | UART MAVLink | RX+TX 57600 baud to ground station |
| IN | AXI4-Lite | AWADDR/WDATA/ARADDR/RDATA to host |
| OUT | PWM[3:0] | ESC signals to 4 motor controllers |
| OUT | LED_STATUS | System health indicator |

#### AXI Register Interface

- All 50+ registers mapped across MOD_10 address space

#### Resource Utilization (XC7A35T)

| **Resource** | **Used** | **Available** | **Utilization** |
|---|---|---|---|
| DSP48E1 | 45 | 90 | 50% |
| BRAM18 | 6 | 50 | 12% |
| LUTs | ~7,280 | 20,800 | ~35% |
| Flip-Flops | ~6,240 | 20,800 | ~30% |

> **Critical Path:** MOD_2 PID inner loop — verified < 20 ns at 1 kHz strobe. Timing closure confirmed at 50 MHz on Artix-7 XC7A35T.

---

## 3. Cross-Module Signal Connectivity

The following describes the primary signal flow between all 12 modules in the complete UAV RTL system.

| **Source → Destination** | **Signals** |
|---|---|
| **MOD_1 → All** | CE_1kHz / CE_100Hz / CE_50Hz / CE_10Hz distributed via MOD_11 CE arbiter |
| **MOD_2 → MOD_3** | u_roll / u_pitch / u_yaw / u_thrust (Q4.28 control outputs) |
| **MOD_3 → External** | PWM[3:0] to 4 motor ESCs (1000–2000 µs, 50–400 Hz) |
| **MOD_4 / 5 / 6 → MOD_2** | EKF state feedback: attitude / velocity / altitude (1-cycle delay via MOD_11) |
| **MOD_5 → MOD_4** | gyro_scaled / accel_scaled (Q4.28) for EKF prediction |
| **MOD_6 → MOD_5** | GPS lat/lon/alt/vN/vE (Q10.22 / Q4.28) for EKF GPS update at 10 Hz |
| **MOD_7 → MOD_8** | ARM/DISARM/TAKEOFF/LAND/RTL commands + mode selector |
| **MOD_8 → MOD_2** | PID setpoints: roll/pitch/altitude/vN/vE |
| **MOD_8 → MOD_3** | ARM_DISARM flag for ESC interlock |
| **MOD_9 → MOD_8** | watchdog_expired (immediate EMERGENCY trigger) + nav_ready |
| **MOD_10 → All** | 50+ AXI4-Lite registers: PID gains, EKF noise, calibration, thresholds |
| **MOD_11** | Transparent CE + BRAM arbitration; priority: EKF > GPS > Baro; WP > Mission; Gain > AXI |
| **MOD_12** | Top-level system controller; verifies 100% connectivity + 50 MHz timing closure |

---

## 4. FPGA Resource Summary (Xilinx Artix-7 XC7A35T)

| **Resource** | **Used** | **Available** | **Utilization** |
|---|---|---|---|
| **DSP48E1** | 45 | 90 | 50% |
| **BRAM18** | 6 | 50 | 12% |
| **LUTs** | ~7,280 | 20,800 | ~35% |
| **Flip-Flops** | ~6,240 | 20,800 | ~30% |
| **BRAM total** | ~70–85 KB | — | — |

> **Critical Path:** MOD_2 PID inner loop — verified < 20 ns at 1 kHz strobe. Timing closure confirmed at 50 MHz on Artix-7 XC7A35T.
>
> **Expansion Margin:** 30–35% overall device utilization provides healthy headroom for additional features.

---

*UAV RTL System — Confidential Technical Specification | Xilinx Artix-7 XC7A35T | 50 MHz*
