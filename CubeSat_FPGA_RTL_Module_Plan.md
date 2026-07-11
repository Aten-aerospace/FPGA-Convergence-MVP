# CUBESAT FPGA CONVERGENCE

## 12-Subsystem Complete Mapping

Comprehensive Module Division, Requirements Traceability & Development Workflow

**Project:** Ani2204/FPGA-Convergence | **Status:** Ready for Implementation | Revision: 1.0 | Date: 2026-04-02

---

## PART 1: Quick Reference Tables

### Table 1A — Subsystem Overview (12 Subsystems)

| ID | Subsystem Name | Category | Req Count | New RTL Files | Clock Domain | BRAM (B) | DSP48 | Status |
|---|---|---|---|---|---|---|---|---|
| **CS1** | IMU Sensor Interface (SPI) | Sensor | 1 | 3 | clk_100mhz | 256 | 0 | Design Phase |
| **CS2** | Magnetometer Interface (I2C) | Sensor | 1 | 3 | sys_clk | 128 | 0 | Design Phase |
| **CS3** | Sun Sensor ADC Interface | Sensor | 1 | 3 | clk_100mhz | 64 | 0 | Design Phase |
| **CS4** | Quaternion Propagator | Estimation | 1 | 3 | sys_clk | 96 | 3 | Design Phase |
| **CS5** | Extended Kalman Filter (7-State) | Estimation | 2 | 5 | sys_clk | 512 | 6 | Design Phase |
| **CS6** | PD Attitude Control Law | Control | 1 | 3 | sys_clk | 64 | 2 | Design Phase |
| **CS7** | RW & MTQ Drivers | Actuation | 2 | 4 | sys_clk | 128 | 1 | Design Phase |
| **CS8** | ADCS Mode FSM & Health Monitor | Management | 3 | 6 | sys_clk | 3,072 | 0 | Design Phase |
| **CS9** | Orbit Propagator (SGP4-Lite) | Estimation | 7 | 7 | sys_clk | 1,536 | 8 | Design Phase |
| **CS10** | Laser Pointing FSM | Management | 10 | 10 | sys_clk | 512 | 2 | Design Phase |
| **CS11** | Telemetry Encoder & Multiplexer | Communication | 8 | 9 | sys_clk | 768 | 0 | Design Phase |
| **CS12** | System Integration & Clock Dist. | System | — | 7 | sys_clk / clk_100mhz | 256 | 0 | Design Phase |
| **TOTAL** | | | **37** | **63** | — | **7,392** | **22** | — |

---

### Table 1B — Existing RTL Files Allocation (20 Files)

| RTL File | Purpose | Primary Subsystem(s) | Secondary Users | Reuse Factor |
|---|---|---|---|---|
| **spi_master.sv** | SPI protocol master | CS1, CS3, CS7 | IMU, sun sensor, RW | 3x |
| **i2c_master.sv** | I2C protocol master | CS2 | Magnetometer | 1x |
| **adc_interface.sv** | ADC controller | CS3 | Sun sensor | 1x |
| **cordic.sv** | CORDIC trigonometric | CS4, CS5, CS9 | Math library | 3x |
| **sqrt.sv** | Square root calculator | CS4, CS5, CS9 | Math library | 3x |
| **fp_divider.sv** | Fixed-point divider | CS4, CS5, CS9 | Math library | 3x |
| **pid_controller.sv** | PID control | CS6, CS10 | Attitude & laser | 2x |
| **pwm_gen.sv** | PWM generator | CS7, CS10 | MTQ & laser mod | 2x |
| **stepper_driver.sv** | Stepper motor | CS10 | Laser gimbal | 1x |
| **lpf.sv** | Low-pass filter | CS10 | Signal filtering | 1x |
| **uart_controller.sv** | UART interface | CS11 | Telemetry downlink | 1x |
| **crc_calc.sv** | CRC calculator | CS11 | Frame validation | 1x |
| **mavlink_parser.sv** | MAVLink protocol | CS11 | Command decoding | 1x |
| **async_fifo.sv** | Async FIFO buffer | CS11 | Clock domain bridging | 1x |
| **clk_divider.sv** | Clock divider | CS12 | System clock tree | 1x |
| **tick_gen.sv** | Tick/strobe generator | CS1, CS2, CS12 | CE generation | 3x |
| **synchronizer.sv** | CDC synchronizer | CS1, CS2, CS8, CS12 | Clock crossing | 4x |
| **debouncer.sv** | Input debouncer | CS2, CS8 | Signal conditioning | 2x |
| **edge_detect.sv** | Edge detector | CS1, CS3, CS8 | Event detection | 3x |
| **kalman_1v.sv** | Kalman filter template | CS5 | State estimation | 1x |

---

### Table 1C — Requirements-to-Subsystem Mapping (37 Requirements)

#### ADCS Requirements (12 total) → CS1–CS8

| Req ID | Requirement Title | Subsystem | Acceptance Criteria (Brief) |
|---|---|---|---|
| **CS-ADCS-001** | IMU Sensor Interface — SPI Read | CS1 | SPI @ 10MHz, 20µs transaction, ≥99.9% CRC pass, raw output LSB/°/s |
| **CS-ADCS-002** | Magnetometer Interface — I2C Read | CS2 | I2C @ 400kHz, 50µs per axis, ACK received within 1 clock period |
| **CS-ADCS-003** | Sun Sensor ADC Interface — SPI Read | CS3 | 4 channels within 10ms, ≥12 bits, <0.1% crosstalk, presence flag |
| **CS-ADCS-004** | Attitude Quaternion Propagation | CS4 | q(k+1) = q(k) + 0.5×q(k)⊗ω×Δt, norm 1±0.001, drift <0.01° over 60s |
| **CS-ADCS-005** | Extended Kalman Filter — State Update | CS5 | 7-state (q, bx, by, bz), predict+update within 8ms, error ≤0.5° (3σ) |
| **CS-ADCS-006** | EKF Covariance Matrix Management | CS5 | 7×7 P matrix, Joseph-form update, positive-definite check |
| **CS-ADCS-007** | PD Attitude Control Law | CS6 | τ = -Kp×q_err - Kd×ω @ 1kHz, saturation ±10 mNm, error <0.5° |
| **CS-ADCS-008** | Reaction Wheel Motor Driver Command | CS7 | SPI commands (±6000 RPM), @ 1kHz, fault status read @ 10Hz |
| **CS-ADCS-009** | Magnetorquer PWM Drive | CS7 | 3-axis PWM @ 10kHz, ≥12-bit resolution, <1% coupling, SAFE blanking |
| **CS-ADCS-010** | ADCS Mode Management — State Machine | CS8 | 5-state FSM: BOOT→DETUMBLE→COARSE_POINT→FINE_POINT→SAFE |
| **CS-ADCS-011** | ADCS Health Monitoring & Fault Detection | CS8 | SPI/I2C timeout >5ms, IMU overflow >500°/s, EKF divergence flags |
| **CS-ADCS-012** | ADCS Data Logging to BRAM | CS8 | 256×96B circular buffer @ 100Hz, dual-port for telemetry |

#### Orbit Requirements (7 total) → CS9

| Req ID | Requirement Title | Subsystem | Acceptance Criteria (Brief) |
|---|---|---|---|
| **CS-ORB-001** | TLE Input & Storage | CS9 | 69-char TLE lines, checksum validation, survives soft reset |
| **CS-ORB-002** | SGP4-Lite Propagation Engine | CS9 | ECI pos/vel, ±8000km / ±10km/s, ≤5km accuracy over 24h |
| **CS-ORB-003** | Position Output Format & LVLH Conversion | CS9 | LVLH matrix from pos+vel, orbital elements (a,e,i,RAAN,ω,ν), altitude |
| **CS-ORB-004** | Time Reference & Epoch Management | CS9 | MET counter @ 1Hz, 32-bit, loadable via AXI4-Lite, GPS sync |
| **CS-ORB-005** | Inter-Satellite Relative Position | CS9 | ΔR_12, ΔR_13 (ECI), ΔV_12, ΔV_13, separation distance |
| **CS-ORB-006** | Ground Track & Contact Window Prediction | CS9 | Geodetic lat/lon ±0.1°, AOS prediction ≤30s accuracy |
| **CS-ORB-007** | Orbit Propagator Health & Validity | CS9 | PROPAGATOR_VALID flag, overflow detection, TLE age tracking |

#### Laser FSM Requirements (10 total) → CS10

| Req ID | Requirement Title | Subsystem | Acceptance Criteria (Brief) |
|---|---|---|---|
| **CS-LSR-001** | Laser Pointing FSM — State Definition | CS10 | 6 states: IDLE, SEARCH, ACQUIRE, TRACK, COMM, FAULT; no dead states |
| **CS-LSR-002** | IDLE State Behavior | CS10 | Home position (0°,0°), modulator disabled, await START or trigger |
| **CS-LSR-003** | SEARCH State Behavior | CS10 | Raster scan ±10°az×±5°el, boustrophedon pattern, ≤0.5° step |
| **CS-LSR-004** | ACQUIRE State Behavior | CS10 | Spiral ±2° reducing 50%/pass, peak-hold, convergence <0.1° in ≤5s |
| **CS-LSR-005** | TRACK State Behavior | CS10 | PD control @ 100Hz, RMS error <0.05°, transition when error <0.02° |
| **CS-LSR-006** | COMM State Behavior | CS10 | Modulator enable, ISL 8-bit data @ 100MHz, maintain <0.05° RMS |
| **CS-LSR-007** | FAULT State Behavior & Recovery | CS10 | Safe all outputs, log fault (8 FIFO), manual CLEAR command only |
| **CS-LSR-008** | Gimbal Stepper Motor Control | CS10 | 2-axis (az/el), ≥2µs pulse, 200kHz max, 400 steps/°, position tracking |
| **CS-LSR-009** | Gimbal Pointing Command Interface | CS10 | Absolute (Q8.8°) + relative (steps), command priority, ≤1ms latency |
| **CS-LSR-010** | Optical Signal Strength Monitoring | CS10 | 10kHz sampling, ≥12-bit, configurable threshold, rolling average |

#### Telemetry Requirements (8 total) → CS11

| Req ID | Requirement Title | Subsystem | Acceptance Criteria (Brief) |
|---|---|---|---|
| **CS-TLM-001** | Telemetry Frame Format | CS11 | SYNC(4B)\|APID(2B)\|SEQ(2B)\|LEN(2B)\|TS(4B)\|PAYLOAD(≤128B)\|CRC16(2B) |
| **CS-TLM-002** | Telemetry Encoding — UART Downlink | CS11 | Baud 9600–115200, 8N1, ≤frame_length×10/baud + 500µs latency |
| **CS-TLM-003** | Telemetry Payload — ADCS Packet | CS11 | q[4], ω[3], mode, faults, τ[3], RW speeds[3], MTQ duty[3] = 44B |
| **CS-TLM-004** | Telemetry Payload — Orbit Packet | CS11 | ECI pos[3], vel[3], alt, lat, lon, eclipse, contact, AOS = 47B |
| **CS-TLM-005** | Telemetry Payload — Laser/ISL Packet | CS11 | FSM state, gimbal az/el, signal strength, pointing error, fault = 20B |
| **CS-TLM-006** | Telemetry Multiplexer & Arbiter | CS11 | 1s superframe: HK@0ms, ADCS@100ms, ORBIT@200ms, LASER@300ms |
| **CS-TLM-007** | Housekeeping Telemetry Packet | CS11 | uptime, FPGA temp (XADC), VCCINT, VCCO, subsystem health = 18B |
| **CS-TLM-008** | Ground Command Reception & Decoding | CS11 | SYNC(4B)\|APID(2B)\|CMD_CODE(1B)\|DATA(≤16B)\|CRC16(2B) |

---

## PART 2: Detailed Subsystem Specifications

### CS1: IMU Sensor Interface (SPI Read) [Sensor]

**Requirements:** CS-ADCS-001 | **Clock Domain:** clk_100mhz → sys_clk | **BRAM:** 256 B | **DSP48:** 0 | **Timing:** 20 µs (SPI) + 2 µs (CDC)

**Functional Description:**
Reads 3-axis accelerometer, gyroscope, and magnetometer data from MPU-9250 over SPI @ 10 MHz SCLK. Implements CRC/parity checking (≥99.9% frame success). Transaction must complete within 20 µs. Raw units: gyro 131 LSB/°/s @ ±250°/s, accel 16384 LSB/g @ ±2g.

**Existing RTL Files:**
- `spi_master.sv` — SPI protocol engine
- `tick_gen.sv` — 100 Hz strobe generation
- `synchronizer.sv` — CDC from clk_100mhz to sys_clk

**New RTL to Create:**
- `imu_controller.sv` — IMU SPI transaction sequencer, state machine
- `imu_data_handler.sv` — Raw data extraction, 9-axis buffering
- `spi_imu_wrapper.sv` — Top-level wrapper

**Interface Signals:**
- Input: `clk_100mhz`, `sys_clk`, `rst_n`, `spi_sclk`, `spi_mosi`, `spi_miso`
- Output: `imu_data_valid`, `imu_accel[3]`, `imu_gyro[3]`, `imu_mag[3]`, `crc_pass`

---

### CS2: Magnetometer Interface (I2C Read) [Sensor]

**Requirements:** CS-ADCS-002 | **Clock Domain:** sys_clk | **BRAM:** 128 B | **DSP48:** 0 | **Timing:** 50 µs per axis × 3 = 150 µs

**Functional Description:**
Reads 3-axis magnetic field data from magnetometer via I2C @ 400 kHz Fast Mode. Implements ACK monitoring; NACK within 1 I2C clock period triggers FAULT_MAG flag. 3-byte per-axis read, ≤1 ms total per 100 Hz cycle. 13-bit ADC output scalable to µT via coefficient register.

**Existing RTL Files:**
- `i2c_master.sv` — I2C protocol engine, 400 kHz
- `debouncer.sv` — Input line debouncing
- `synchronizer.sv` — CDC for fault signals

**New RTL to Create:**
- `i2c_mag_controller.sv` — I2C read sequencing, address/data handling
- `mag_fault_detector.sv` — ACK/NACK timeout detection
- `i2c_mag_wrapper.sv` — Top-level wrapper

**Interface Signals:**
- Input: `sys_clk`, `rst_n`, `i2c_sda`, `i2c_scl`
- Output: `mag_data[3]`, `mag_valid`, `mag_fault`, `mag_age_ms`

---

### CS3: Sun Sensor ADC Interface [Sensor]

**Requirements:** CS-ADCS-003 | **Clock Domain:** clk_100mhz → sys_clk | **BRAM:** 64 B | **DSP48:** 0 | **Timing:** 10 ms for all 4 channels

**Functional Description:**
Reads 4-channel sun sensor photodiode signals via SPI-connected 12-bit ADC, sharing IMU SPI bus (multiplexed via CS_N). Resolution ≥12 bits, no missing codes across 0–3.3V. <0.1% FS crosstalk via independent CS_N sequencing. Sun-presence flag asserted when any channel > configurable threshold (default 10% FS = ~409 counts).

**Existing RTL Files:**
- `spi_master.sv` — Shared SPI, same 10 MHz as IMU
- `adc_interface.sv` — ADC read protocol, 12-bit
- `edge_detect.sv` — Channel change detection

**New RTL to Create:**
- `sun_sensor_adc.sv` — 4-channel sequencer, CS_N multiplexing
- `adc_multiplexer.sv` — Channel selection logic, timing
- `sun_presence_detector.sv` — Threshold compare, flag generation

**Interface Signals:**
- Input: `clk_100mhz`, `sys_clk`, `rst_n`, `spi_mosi`, `spi_miso`, `adc_sclk`, `adc_cs_n`
- Output: `sun_channel[4]`, `sun_valid`, `sun_present`, `sun_presence_age_ms`

---

### CS4: Quaternion Propagator [Estimation]

**Requirements:** CS-ADCS-004 | **Clock Domain:** sys_clk | **BRAM:** 96 B | **DSP48:** 3 | **Timing:** 5 ms execution

**Functional Description:**
Propagates spacecraft attitude quaternion using gyroscope angular rate integration. Implements q(k+1) = q(k) + 0.5 × q(k) ⊗ ω × Δt in Q2.46 fixed-point arithmetic. Norm constraint: |q| = 1 ± 0.001 maintained via normalization each cycle. Drift <0.01° over 60 seconds free-run (no EKF correction).

**Existing RTL Files:**
- `cordic.sv` — Quaternion magnitude computation
- `sqrt.sv` — Norm normalization via square root
- `fp_divider.sv` — Fixed-point division for normalization

**New RTL to Create:**
- `quat_propagator.sv` — Main propagation pipeline: multiply, accumulate
- `quaternion_math.sv` — Quaternion multiplication, conjugate operations
- `norm_checker.sv` — Magnitude check, out-of-range detection, flags

**Interface Signals:**
- Input: `sys_clk`, `rst_n`, `ce_100hz`, `omega_x`, `omega_y`, `omega_z`, `gyro_valid`
- Output: `q_out[4]`, `q_norm`, `q_norm_valid`, `norm_err_flag`, `quat_ready`

---

### CS5: Extended Kalman Filter (7-State) [Estimation]

**Requirements:** CS-ADCS-005, CS-ADCS-006 | **Clock Domain:** sys_clk | **BRAM:** 512 B | **DSP48:** 6 | **Timing:** 8 ms execution

**Functional Description:**
Implements 7-state EKF fusing IMU and magnetometer for attitude estimation. State vector: [q0, q1, q2, q3, bx, by, bz] where q = quaternion, b = gyro bias. Predict step: constant-velocity model. Update step via Joseph-form covariance update. Steady-state error ≤0.5° (3σ). Gyro bias convergence to ±0.01°/s within 60s. Divergence watchdog: resets to last-known-good on 3 consecutive 5σ innovations.

**Existing RTL Files:**
- `kalman_1v.sv` — Single-vehicle EKF template
- `cordic.sv` — Trigonometric functions for H matrix
- `sqrt.sv` — Covariance square root for Joseph form
- `fp_divider.sv` — Matrix inversion, Kalman gain division

**New RTL to Create:**
- `ekf_core.sv` — Main EKF orchestrator, cycle manager
- `ekf_predict.sv` — State + covariance prediction step
- `ekf_update.sv` — Measurement update, innovation calculation
- `ekf_covariance.sv` — Joseph-form covariance update, PD check
- `ekf_matrix_ops.sv` — Utility: matrix multiply, transpose, inversion

**Interface Signals:**
- Input: `sys_clk`, `rst_n`, `ce_100hz`, `q_prior[4]`, `gyro_bias[3]`, `accel[3]`, `mag[3]`, `measurement_valid`
- Output: `q_est[4]`, `bias_est[3]`, `P_diag[7]`, `ekf_valid`, `divergence_flag`, `innovation[7]`

---

### CS6: PD Attitude Control Law [Control]

**Requirements:** CS-ADCS-007 | **Clock Domain:** sys_clk | **BRAM:** 64 B | **DSP48:** 2 | **Timing:** 1 ms execution

**Functional Description:**
Implements PD control law: τ = −Kp × q_err(1:3) − Kd × ω, executing @ 1 kHz. Output torque saturated to ±10 mNm per axis. Steady-state pointing error <0.5° (3σ) under 1e-6 N·m disturbance. Kp and Kd gains loadable via AXI4-Lite; changes effective within 1 control cycle.

**Existing RTL Files:**
- `pid_controller.sv` — PID template, reusable for PD subset

**New RTL to Create:**
- `pd_controller_quaternion.sv` — Quaternion error computation, PD law
- `torque_saturation.sv` — ±10 mNm saturation + flag generation
- `control_law_engine.sv` — Integrates PD + saturation, output multiplexer

**Interface Signals:**
- Input: `sys_clk`, `rst_n`, `ce_1khz`, `q_err[4]`, `omega[3]`, `Kp_coeff`, `Kd_coeff`, `axi_gain_write`
- Output: `torque_cmd[3]`, `saturation_flag`, `sat_count`, `ctrl_valid`

---

### CS7: Reaction Wheel & Magnetorquer Drivers [Actuation]

**Requirements:** CS-ADCS-008, CS-ADCS-009 | **Clock Domain:** sys_clk (1 kHz), clk_100mhz (PWM) | **BRAM:** 128 B | **DSP48:** 1 | **Timing:** 1 ms (SPI RW + MTQ PWM update)

**Functional Description:**
RW SPI Driver: Translates PD control torque commands into 3 independent SPI motor driver commands @ 1 kHz. Speed range: ±6000 RPM (16-bit signed, 1 LSB = 0.2 RPM). MTQ PWM Drive: 3-axis PWM @ 10 kHz carrier, ≥12-bit resolution. Cross-axis coupling <1%. SAFE mode blanking within 1 control cycle.

**Existing RTL Files:**
- `spi_master.sv` — SPI for 3× RW drivers
- `pwm_gen.sv` — PWM generators, 3× instances

**New RTL to Create:**
- `rw_spi_driver.sv` — 3× SPI RW sequencer, CS_N multiplexing
- `magnetorquer_pwm.sv` — 3× PWM instances, 10 kHz carrier
- `actuator_command_arbiter.sv` — Routes τ_cmd → RW speed + MTQ duty
- `fault_status_monitor.sv` — RW fault polling, MTQ current monitoring

**Interface Signals:**
- Input: `sys_clk`, `clk_100mhz`, `rst_n`, `ce_1khz`, `torque_cmd[3]`, `safe_mode`
- Output: `rw_sclk`, `rw_mosi[3]`, `rw_cs_n[3]`, `rw_miso[3]`, `mtq_pwm[3]`, `rw_fault[3]`

---

### CS8: ADCS Mode FSM & Health Monitor [Management]

**Requirements:** CS-ADCS-010, CS-ADCS-011, CS-ADCS-012 | **Clock Domain:** sys_clk | **BRAM:** 3,072 B | **DSP48:** 0 | **Timing:** <1 ms FSM + <1 ms logging

**Functional Description:**
Mode FSM: 5-state machine BOOT → DETUMBLE → COARSE_POINT → FINE_POINT → SAFE. Any FAULT transitions to SAFE within 2 ms. Health Monitor: continuously monitors SPI/I2C timeout >5ms, IMU rate >500°/s, quaternion norm outside 1±0.01 for >3 cycles. Data Logging: 256×96B circular BRAM buffer @ 100 Hz, dual-port for simultaneous telemetry read.

**Existing RTL Files:**
- `edge_detect.sv` — State transition edge detection
- `synchronizer.sv` — CDC for fault signals
- `debouncer.sv` — Fault signal debouncing

**New RTL to Create:**
- `adcs_fsm_controller.sv` — Main FSM orchestrator
- `adcs_state_machine.sv` — State transition logic, timer counters
- `fault_detector.sv` — Multi-fault detection: timeout, overflow, divergence
- `fault_logger.sv` — FIFO fault log: code + timestamp
- `adcs_health_monitor.sv` — Aggregates health signals
- `bram_circular_buffer.sv` — Dual-port 256×96B circular log

**Interface Signals:**
- Input: `sys_clk`, `rst_n`, `ce_100hz`, `imu_valid`, `mag_valid`, `ekf_valid`, `q_norm`, `omega_max`, `fault_trigger[8]`
- Output: `adcs_mode[3]`, `mode_valid`, `health_flag`, `fault_flags[8]`, `bram_log_addr`, `bram_log_data`

---

### CS9: Orbit Propagator (SGP4-Lite) [Estimation]

**Requirements:** CS-ORB-001 through CS-ORB-007 | **Clock Domain:** sys_clk | **BRAM:** 1,536 B | **DSP48:** 8 | **Timing:** 50ms propagation + 10ms LVLH + 5ms ground track = 65ms

**Functional Description:**
Full 7-requirement orbit stack. TLE Input: accepts 69-char TLE lines via AXI4-Lite, validates checksums, epoch parsed to MJD. SGP4-Lite Engine: computes ECI position + velocity @ 1 Hz, ≤5 km accuracy over 24h vs STK. LVLH Conversion: R_ECI_to_LVLH matrix, classical orbital elements, WGS84 altitude. MET Counter: 32-bit @ 1 Hz, GPS-syncable. Relative Position: ΔR_ij, ΔV_ij per ISL. Ground Track: geodetic lat/lon ±0.1°, AOS prediction ≤30s.

**Existing RTL Files:**
- `cordic.sv` — Sine/cosine for orbital mechanics
- `sqrt.sv` — Square root for distance calculations
- `fp_divider.sv` — Fixed-point division for orbital elements

**New RTL to Create:**
- `tle_parser.sv` — TLE validation, checksum, epoch extraction
- `sgp4_propagator_core.sv` — SGP4-Lite engine: ECI propagation
- `lvlh_converter.sv` — ECI → LVLH frame transformation
- `ground_track_calculator.sv` — WGS84 lat/lon computation
- `contact_window_predictor.sv` — AOS/LOS prediction, elevation check
- `orbit_health_monitor.sv` — Validity flags, overflow detection
- `orbit_state_manager.sv` — BRAM output bank, MET counter, epoch manager

**Interface Signals:**
- Input: `sys_clk`, `rst_n`, `ce_1hz`, `tle_input[138]`, `tle_write`, `met_load_value`, `gps_lla_input[3]`
- Output: `eci_pos[3]`, `eci_vel[3]`, `orbital_elements[6]`, `altitude_m`, `latitude_rad`, `longitude_rad`, `eclipse_flag`, `propagator_valid`

---

### CS10: Laser Pointing FSM [Management]

**Requirements:** CS-LSR-001 through CS-LSR-010 | **Clock Domain:** sys_clk (100 Hz), clk_100mhz (modulator) | **BRAM:** 512 B | **DSP48:** 2 | **Timing:** 10 ms per 100 Hz cycle

**Functional Description:**
6-state laser acquisition and tracking FSM: IDLE → SEARCH (boustrophedon raster ±10°az×±5°el, ≤0.5° step) → ACQUIRE (spiral ±2°, peak-hold, convergence <0.1° in ≤5s) → TRACK (PD @ 100Hz, RMS <0.05°) → COMM (modulator enabled, ISL 8-bit @ 100MHz) → FAULT. Gimbal stepper: 2-axis STEP/DIR, 400 steps/°, 200 kHz max, trapezoidal acceleration. Signal monitoring: 10 kHz ADC, 12-bit, rolling 16-sample average.

**Existing RTL Files:**
- `pid_controller.sv` — PD for TRACK/ACQUIRE pointing control
- `pwm_gen.sv` — Laser modulator PWM
- `stepper_driver.sv` — Gimbal STEP/DIR pulses
- `lpf.sv` — Low-pass filter on signal strength input

**New RTL to Create:**
- `laser_fsm.sv` — 6-state machine top-level orchestrator
- `gimbal_controller.sv` — Gimbal command generation, position calc
- `laser_state_machine.sv` — State transitions, guards, timers
- `raster_scan_engine.sv` — SEARCH boustrophedon raster generator
- `spiral_refinement.sv` — ACQUIRE spiral pattern, peak-hold
- `peak_hold_detector.sv` — Signal strength peak tracking
- `gimbal_stepper_control.sv` — STEP/DIR pulse generation, homing
- `signal_monitor.sv` — 10 kHz signal sampling, rolling average
- `laser_modulator.sv` — Modulator enable, ISL data mux
- `laser_fault_handler.sv` — Fault detection and FAULT state handler

**Interface Signals:**
- Input: `sys_clk`, `clk_100mhz`, `rst_n`, `ce_100hz`, `signal_strength_adc`, `gimbal_cmd_abs`, `gimbal_cmd_valid`, `safe_mode`
- Output: `laser_fsm_state[3]`, `gimbal_step[2]`, `gimbal_dir[2]`, `laser_modulator_en`, `isl_data[8]`, `signal_strength`, `fault_code`

---

### CS11: Telemetry Encoder & Multiplexer [Communication]

**Requirements:** CS-TLM-001 through CS-TLM-008 | **Clock Domain:** sys_clk | **BRAM:** 768 B | **DSP48:** 0 | **Timing:** <100 ms per frame transmission

**Functional Description:**
Full CCSDS-aligned frame builder: SYNC(4B)|APID(2B)|SEQ(2B)|LEN(2B)|TS(4B)|PAYLOAD(≤128B)|CRC16(2B). UART TX: 9600–115200 bps, 8N1, configurable inter-frame gap. 1-second superframe scheduler: HK@0ms, ADCS@100ms, ORBIT@200ms, LASER@300ms. ADCS packet 44B (APID 0x101), Orbit packet 47B (APID 0x102), Laser packet 20B (APID 0x103), HK packet 18B (APID 0x100). Command reception: UART RX → CRC check → route by APID to target subsystem via AXI4-Lite.

**Existing RTL Files:**
- `uart_controller.sv` — UART TX/RX @ configurable baud
- `crc_calc.sv` — CRC-16/CCITT-FALSE
- `mavlink_parser.sv` — MAVLink command parsing
- `async_fifo.sv` — UART RX → sys_clk FIFO, CDC

**New RTL to Create:**
- `tlm_frame_builder.sv` — Frame assembly: SYNC + APID + CRC
- `tlm_arbiter.sv` — 1 Hz superframe scheduling, slot arbiter
- `tlm_encoder_adcs.sv` — ADCS packet mux from log buffer
- `tlm_encoder_orbit.sv` — Orbit packet mux from propagator
- `tlm_encoder_laser.sv` — Laser packet mux from FSM
- `tlm_encoder_hk.sv` — HK packet: XADC, uptime, health
- `command_decoder.sv` — Command frame parsing, CRC check
- `uart_tx_scheduler.sv` — Baud rate FSM, inter-frame gap
- `command_dispatcher.sv` — Route commands to target subsystems

**Interface Signals:**
- Input: `sys_clk`, `rst_n`, `ce_1hz`, `uart_rx`, `adcs_tlm_data[44]`, `orbit_tlm_data[47]`, `laser_tlm_data[20]`
- Output: `uart_tx`, `tlm_valid`, `tlm_apid`, `tlm_seq`, `cmd_valid`, `cmd_apid`, `cmd_code`, `cmd_data[16]`

---

### CS12: System Integration & Clock Distribution [System]

**Requirements:** System-level (all subsystems) | **Clock Domain:** sys_clk, clk_100mhz | **BRAM:** 256 B | **DSP48:** 0 | **Timing:** N/A (infrastructure layer)

**Functional Description:**
Central clock manager: derives sys_clk, clk_100mhz, ce_1hz, ce_100hz, ce_1khz from 100 MHz reference. AXI4-Lite slave: full register map 0x0000–0x0BFF across all 12 subsystems. Reset controller: POR sequencing, configurable warm/cold soft reset (cold clears BRAM, warm preserves TLE/MET). XADC system monitor: 1 Hz temperature, VCCINT, VCCO readings; thermal throttle at >110°C. Top-level `top_cubesat_mvp.sv` instantiates all CS1–CS11 and all interconnects.

**Existing RTL Files:**
- `clk_divider.sv` — Frequency division for clock generation
- `tick_gen.sv` — CE strobe generation at multiple rates
- `synchronizer.sv` — CDC for cross-domain coordination

**New RTL to Create:**
- `clk_manager.sv` — Central clock tree, multiplexer, gating
- `axi4_lite_slave.sv` — AXI4-Lite protocol handler for all registers
- `reset_controller.sv` — POR sequencing, soft/cold/warm reset logic
- `system_monitor.sv` — XADC controller, temperature/voltage tracking
- `power_monitor.sv` — Per-subsystem power budget tracking, alarms
- `resource_arbiter.sv` — BRAM/DSP48 access coordination
- `top_cubesat_mvp.sv` — Top-level instantiation + interconnect

**Interface Signals:**
- Input: `clk_ref_100mhz`, `rst_pbl_n`, `axi_clk`, `axi_rst_n`, `axi_awaddr`, `axi_wdata`, `axi_araddr`
- Output: `sys_clk`, `clk_100mhz`, `ce_1hz`, `ce_100hz`, `ce_1khz`, `fpga_temp`, `vccint`, `vcco`, `thermal_alert`

---

## PART 3: Development Workflow & Integration Checklist

### Phase 1: Per-Subsystem Development Workflow

#### 1.1 Specification Review
- Read corresponding requirement(s) from specification files
- Extract key metrics: timing budget, accuracy, resource limits
- Identify input/output signals and protocols
- Document clock domains and CDC requirements
- List existing RTL files to reuse

#### 1.2 RTL Architecture Design
- Create subsystem-level block diagram
- Define internal module interfaces (signals, bus widths)
- Allocate BRAM and DSP48 resources
- Identify testable boundaries
- Document fixed-point number formats

#### 1.3 RTL Implementation
- Create new RTL files (list per subsystem)
- Implement each module with synthesizable SystemVerilog
- Add comments: timing constraints, state diagrams, algorithms
- Ensure consistent naming: `subsystem_function.sv`
- Include internal assertions for debugging

#### 1.4 Simulation & Unit Testing
- Write standalone testbench for each new module
- Run functional simulation (ModelSim/VCS)
- Verify timing: setup/hold, clock-to-out
- Check fixed-point accuracy against golden reference
- Generate waveforms for manual inspection

#### 1.5 Integration into CS12
- Add subsystem instantiation in `top_cubesat_mvp.sv`
- Connect clock/reset signals
- Connect AXI4-Lite register interface
- Route data signals to/from neighboring subsystems
- Add CDC synchronizers where needed

#### 1.6 Synthesis & Place & Route
- Synthesize subsystem in isolation (Vivado)
- Check resource utilization
- Verify timing closure (worst-case path)
- Inspect synthesized RTL for optimization issues
- Generate utilization report

#### 1.7 System-Level Simulation
- Instantiate full top module with all CS1–CS11
- Apply realistic input stimuli (sensor data, commands)
- Run for functional cycles (100–1000 steps)
- Verify data flow between subsystems
- Check for timing violations in cross-module paths

#### 1.8 Documentation
- Create subsystem README: purpose, interfaces, usage
- Document register map (AXI4-Lite addresses)
- Add performance metrics: latency, throughput, resource usage
- Include troubleshooting guide for common issues

---

### Phase 2: Integration Checklist (System-Level)

#### 2.1 Pre-Integration Review
- All 12 subsystems (CS1–CS11) synthesize individually without errors
- No timing violations in isolation
- BRAM utilization: sum of all subsystems < total available
- DSP48 utilization: sum of all subsystems < total available
- All external I/O (SPI, I2C, UART, PWM) mapped to FPGA pins

#### 2.2 Clock Domain Integration
- Clock tree synthesizes without glitches
- All CDC paths protected by synchronizers
- Convergence engine strobes (1 Hz, 100 Hz, 1 kHz) verified
- Jitter on ce_1hz, ce_100hz measured <5% of cycle period
- Timing closure achieved on sys_clk and clk_100mhz

#### 2.3 Interconnect Verification
- Data pathways: sensors → estimation → control → actuators verified
- Bus arbitration (if any shared buses) validated
- No combinatorial loops in high-speed paths
- All required handshake signals (valid/ready) implemented
- Backpressure handling tested (e.g., UART FIFO full)

#### 2.4 AXI4-Lite Register Map Validation
- Register addresses assigned without conflicts
- All subsystems accessible via AXI4-Lite
- Read/write permissions correctly enforced
- Register reset values set appropriately
- Ground station software uses correct address offsets

#### 2.5 Memory (BRAM) Allocation Verification
- Total BRAM used < device capacity (Xilinx Artix-7 XC7A35T = 45 KB available)
- Dual-port configurations for simultaneous access correctly instantiated
- Circular buffers (logging) overflow flags tested
- Write/read collisions prevented by CDC or arbitration

#### 2.6 Arithmetic & Precision Validation
- Fixed-point overflow/underflow cases identified
- Q-format (signed/fractional bits) consistent across subsystems
- Saturation behavior documented and tested
- CORDIC / SQRT / DIV outputs compared against golden reference

#### 2.7 Fault Injection & Recovery Testing
- Inject sensor faults (timeout, bad CRC) → verify FSM transitions to SAFE
- Inject command errors → verify error flags set, no system hang
- Power supply brown-out → verify graceful degradation
- Thermal threshold → verify throttling or shutdown
- Recovery procedures tested

#### 2.8 Telemetry & Logging Validation
- All packets transmitted at correct intervals
- CRC-16 verified on ground
- Sequence counter increments properly
- Data alignment (no byte shifts)
- Ground station receives all packets without gaps

#### 2.9 End-to-End Functional Test
- Apply simulated mission profile: pre-flight → orbital insertion → science ops
- Monitor all telemetry packets; verify no anomalies
- Inject simulated events: solar eclipse, contact window, laser acquisition
- Verify responses: mode transitions, control law outputs, ISL data
- Total simulation runtime ≥1 simulated mission orbit (~90 minutes)

#### 2.10 Resource & Performance Report
- Measure worst-case execution time per subsystem
- Verify no deadline misses (100 Hz ADCS, 1 kHz control)
- Report BRAM utilization as % of total
- Report DSP48 utilization as % of total
- Identify performance bottlenecks for future optimization

---

## PART 4: Module Quick Reference Table

| CS ID | New Files | Key New File | Existing Files | Timing | BRAM (B) | DSP48 | Interfaces |
|---|---|---|---|---|---|---|---|
| **CS1** | 3 | imu_controller.sv | 3 | 20µs | 256 | 0 | SPI 10MHz |
| **CS2** | 3 | i2c_mag_controller.sv | 3 | 150µs | 128 | 0 | I2C 400kHz |
| **CS3** | 3 | sun_sensor_adc.sv | 3 | 10ms | 64 | 0 | SPI 10MHz |
| **CS4** | 3 | quat_propagator.sv | 3 | 5ms | 96 | 3 | 100Hz strobe |
| **CS5** | 5 | ekf_core.sv | 4 | 8ms | 512 | 6 | 100Hz strobe |
| **CS6** | 3 | pd_controller_quaternion.sv | 1 | 1ms | 64 | 2 | 1kHz strobe |
| **CS7** | 4 | rw_spi_driver.sv | 2 | 1ms | 128 | 1 | SPI, PWM |
| **CS8** | 6 | adcs_fsm_controller.sv | 3 | 1ms | 3,072 | 0 | 100Hz strobe |
| **CS9** | 7 | sgp4_propagator_core.sv | 3 | 50ms | 1,536 | 8 | 1Hz strobe |
| **CS10** | 10 | laser_fsm.sv | 4 | 10ms | 512 | 2 | 100Hz strobe |
| **CS11** | 9 | tlm_frame_builder.sv | 4 | — | 768 | 0 | UART, AXI4-Lite |
| **CS12** | 7 | top_cubesat_mvp.sv | 3 | — | 256 | 0 | Global interconnect |
| **TOTAL** | **63 new files** | | **20** | | **7,392 B** | **22** | |

---

## PART 5: Implementation Priority & Timeline

### Phase A: Sensor & Estimation Layer (Weeks 1–4)

Foundation for all ADCS functionality. Sensor inputs and state estimation must be in place before control can execute.

- **CS1** — IMU SPI: Foundation for all ADCS (gyro, accel, mag)
- **CS2** — MAG I2C: Parallel to CS1
- **CS3** — SUN ADC: Parallel to CS1/CS2
- **CS4** — Quaternion Propagator: After CS1 (depends on gyro data)
- **CS9** — Orbit Propagator: Independent of ADCS, can start in Phase A
- **CS10** — Laser FSM: Independent, can start in Phase A
- **CS5** — EKF: After CS2/CS3/CS4

### Phase B: Control & Actuation Layer (Weeks 5–6)

- **CS6** — PD Control Law: After CS5 (uses EKF state estimate)
- **CS7** — RW & MTQ Actuators: After CS6

### Phase C: Management & Science (Weeks 7–8)

- **CS8** — ADCS FSM & Health Monitor: After CS1–CS7

### Phase D: Communication & Integration (Weeks 9–10)

- **CS11** — Telemetry Encoder & Multiplexer: After CS1–CS10 ready
- **CS12** — System Top-Level Integration: After all subsystems ready

---

## PART 6: Success Criteria

- All 37 requirements mapped to unique subsystems
- All 63 new RTL files created and synthesized
- All 20 existing RTL files reused without modification
- Total BRAM ≤ 8 KB (target: XC7A35T = 45 KB available)
- Total DSP48 ≤ 50 (XC7A35T = 90 available)
- All timing constraints met (100 Hz ADCS, 1 kHz control, 1 Hz telemetry)
- End-to-end system simulation passes for 1 mission orbit (~90 min simulated)
- Register map complete and ground station compatible
- All modules documented with testbenches and README files

---

*Generated for: Ani2204/FPGA-Convergence | Date: 2026-04-02 | Status: Ready for Implementation | Revision: 1.0*
