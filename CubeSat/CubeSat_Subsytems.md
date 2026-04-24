# CubeSat FPGA — Detailed Sub-Module Breakdown (All 12 Subsystems)

**With Purposes & Architectural Reasoning**

---

## CS1: IMU Sensor Interface (SPI) — 3 Sub-Modules

### Module Hierarchy

```
CS1: IMU Sensor Interface
│
├─ spi_imu_wrapper.sv          (TOP-LEVEL INTEGRATION)
│  │
│  ├─ imu_controller.sv        (SPI SEQUENCER)
│  │
│  ├─ imu_data_handler.sv      (CALIBRATION & DATA CONDITIONING)
│  │
│  └─ synchronizer.sv × 2      (CDC BRIDGES) [REUSED]
│
└─ Shared Dependencies:
   ├─ spi_master.sv             (SPI PROTOCOL) [REUSED]
   ├─ tick_gen.sv               (TIMING STROBES) [REUSED]
   └─ edge_detect.sv            (EVENT DETECTION) [REUSED]
```

---

### CS1.1: imu_controller.sv

**Purpose:** SPI transaction orchestrator for 9-axis sensor reads

**What It Does:**
- Implements a finite state machine (FSM) that sequences reads from MPU-9250 IMU sensor
- Reads 9 registers: 3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer
- Each register read = 24-bit SPI frame (8-bit address + 16-bit data)
- Executes entire 9-register sequence in ~18 µs (within 20 µs budget per CS-ADCS-001)
- Generates SPI clock signals: SCLK (10 MHz), MOSI, MISO, CS_N
- Validates CRC/parity on received frames (≥99.9% success target)

**Why Created:**
- **Abstraction Layer:** SPI protocol is complex (clock generation, bit shifting, CS management). Rather than inline all this logic, we encapsulate it into a reusable sequencer
- **Timing Budget:** 100 Hz IMU update rate = 10 ms between acquisitions. We must complete all 9 reads in < 20 µs to leave margin for downstream processing
- **Fault Tolerance:** CRC checking catches corrupted sensor reads; FSM timeout detection catches stuck or unresponsive sensor
- **Reusability:** Can be parameterized to work with different SPI devices (MPU-9250, ICM-42688-P)

**Inputs:**

| Signal | Description |
|--------|-------------|
| `clk_100mhz` | SPI controller clock |
| `rst_n` | Async reset |
| `read_trigger` | Pulse to start 9-axis acquisition |
| `spi_miso` | Data from IMU |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `spi_sclk`, `spi_mosi`, `spi_cs_n` | SPI signals to IMU |
| `accel_x/y/z`, `gyro_x/y/z`, `mag_x/y/z` | Raw 16-bit sensor values |
| `data_valid`, `crc_pass` | Strobe signals |
| `busy`, `fault` | Status flags |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~80 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS1.2: imu_data_handler.sv

**Purpose:** Transform raw sensor bits into calibrated, Q15 fixed-point values

**What It Does:**
- Receives raw 16-bit two's-complement ADC words from `imu_controller`
- **Bias Subtraction:** Each axis has a factory-calibrated bias offset (e.g., accelerometer zero-g offset ~0.1 g). Subtracts it: `cal_axis = raw_axis - BIAS_CONST`
- **Overflow Detection:** Checks if any axis hits ±32767 (saturation limit). Sets `overflow_flag` if so
- **Scale Factor Application:** Converts ADC counts to physical units:
  - Accelerometer: Q4.12 format (1 LSB = 1/4096 g)
  - Gyroscope: Q6.10 format (1 LSB = 1/1024 °/s)
  - Magnetometer: Device-dependent (AK8963 inside MPU-9250: 0.15 µT/LSB)

**Why Created:**
- **Data Conditioning:** Raw ADC values are meaningless without calibration. Factory tests characterize each sensor; we apply those offsets in FPGA
- **Consistency:** Downstream modules (CS4, CS5, CS6) expect Q15 fixed-point inputs. This module provides a single point of conversion, avoiding scattered format conversions
- **Future Extensibility:** Bias constants can be promoted to AXI4-Lite registers for in-orbit recalibration without re-synthesizing FPGA
- **Fault Detection:** Overflow flag allows CS8 health monitor to detect saturated sensor measurements

**Inputs:**

| Signal | Description |
|--------|-------------|
| `raw_accel[3]`, `raw_gyro[3]`, `raw_mag[3]` | Raw 16-bit ADC values from `imu_controller` |
| `raw_valid` | Pulse indicating new data |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `cal_accel[3]`, `cal_gyro[3]`, `cal_mag[3]` | Calibrated Q15 signed values |
| `cal_valid` | Synchronized strobe |
| `overflow_flag`, `fault_axis[2:0]` | Overflow + which axes saturated |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~40 |
| DSP48 | 0 |
| BRAM | ~256 B |

---

### CS1.3: spi_imu_wrapper.sv

**Purpose:** Top-level integration point; bridges clock domains and exposes clean interface

**What It Does:**
- Instantiates `imu_controller` (clk_100mhz domain) and `imu_data_handler` (same domain)
- Applies Clock Domain Crossing (CDC) synchronizers:
  - `imu_data_valid` strobed in clk_100mhz → 2-FF synchronizer → `imu_data_valid` in sys_clk
  - `crc_pass` strobed in clk_100mhz → 2-FF synchronizer → `crc_pass` in sys_clk
- Registers calibrated sensor outputs for stability: outputs held constant between valid pulses
- Presents unified interface to CS12 system integrator

**Why Created:**
- **Clock Domain Safety:** SPI operates at clk_100mhz (100 MHz), but rest of ADCS runs at sys_clk (50 MHz typically, per requirement). Naive cross-domain handshakes cause metastability. 2-FF synchronizers are proven CDC pattern
- **Encapsulation:** CS12 doesn't care about internal clk_100mhz complexity; just sees a black-box sensor with strobes and outputs
- **Output Stability:** Calibrated data held stable until next `imu_data_valid` strobe. Prevents CS5 EKF from seeing partial updates

**Inputs:**

| Signal | Description |
|--------|-------------|
| `clk_100mhz`, `sys_clk`, `rst_n` | Clock and reset |
| `imu_read_trigger` | 100 Hz CE from CS12 |
| `spi_miso` | Data from IMU |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `spi_sclk`, `spi_mosi`, `spi_cs_n` | SPI signals |
| `accel_x/y/z`, `gyro_x/y/z`, `mag_x/y/z` | Sensor data (sys_clk domain) |
| `imu_data_valid`, `crc_pass`, `imu_busy`, `imu_fault`, `imu_overflow` | Status (sys_clk domain) |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~8 |
| DSP48 | 0 |
| BRAM | 0 |

---

## CS2: Magnetometer Interface (I2C) — 3 Sub-Modules

### Module Hierarchy

```
CS2: Magnetometer Interface
│
├─ i2c_mag_wrapper.sv          (TOP-LEVEL INTEGRATION)
│  │
│  ├─ i2c_mag_controller.sv    (I2C SEQUENCER)
│  │
│  ├─ mag_fault_detector.sv    (WATCHDOG & FAULT DETECTION)
│  │
│  └─ synchronizer.sv          (CDC BRIDGE) [REUSED]
│
└─ Shared Dependencies:
   ├─ i2c_master.sv             (I2C PROTOCOL) [REUSED]
   ├─ debouncer.sv              (GLITCH REJECTION) [REUSED]
   └─ edge_detect.sv            (EVENT DETECTION) [REUSED]
```

---

### CS2.1: i2c_mag_controller.sv

**Purpose:** I2C protocol sequencer for AK8963 magnetometer reads

**What It Does:**
- Implements I2C state machine for 3-axis magnetic field readout (AK8963 @ address 0x0C)
- I2C speed: 400 kHz Fast Mode
- Reads 3 bytes per axis (x, y, z) = 3 I2C transactions
- Generates START condition, device address, data phases, STOP condition
- Monitors ACK/NACK signals: expects ACK from slave on each byte
- Timeout: If ACK not received within 1 I2C clock cycle, sets ACK_FAIL flag

**Why Created:**
- **Protocol Abstraction:** I2C is more complex than SPI (open-drain lines, clock stretching, START/STOP conditions). Encapsulating this prevents scattered logic
- **Fault Detection:** NACK detection is critical. If magnetometer is unresponsive, we must know immediately (triggers CS8 health monitor)
- **Realistic Timing:** 3 bytes × 400 kHz ≈ 150 µs total read time. Fits within 1 ms (1000 Hz) ADCS loop
- **Reusability:** Can work with different I2C magnetometers by changing address constant

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk` | System clock |
| `rst_n` | Reset |
| `mag_read_trigger` | Pulse to start read |
| `i2c_sda`, `i2c_scl` | I2C open-drain bus |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `mag_x[15:0]`, `mag_y[15:0]`, `mag_z[15:0]` | 3-axis mag field (Q15) |
| `mag_valid` | Strobe on valid read |
| `mag_fault` | NACK or timeout |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~60 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS2.2: mag_fault_detector.sv

**Purpose:** Real-time watchdog for magnetometer communication health

**What It Does:**
- Monitors `mag_valid` and `mag_fault` signals from `i2c_mag_controller`
- **Timeout Watchdog:** If no successful read for > TIMEOUT (tunable, e.g., 10 ms), asserts `mag_timeout`
- **Debouncing:** Glitchy ACK/NACK signals can cause false faults. Requires 2–3 consecutive ACK failures before asserting fault
- **Age Tracking:** Counts ms since last successful read; outputs `mag_age_ms`
- **NACK Logging:** Records NACK events for telemetry (ground can analyze sensor health degradation)

**Why Created:**
- **Health Monitoring:** CS8 FSM must know if magnetometer is dead or malfunctioning. Timeout detection is the mechanism
- **Debouncing:** I2C lines can be noisy; one bad bit doesn't mean sensor failure. Debouncer prevents false positives
- **Age Tracking:** Ground station wants to know: "How long since we last heard from the mag?" Useful for anomaly analysis
- **Separation of Concerns:** `i2c_mag_controller` handles protocol; `mag_fault_detector` handles reliability

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n` | Clock and reset |
| `mag_valid`, `mag_fault` | From `i2c_mag_controller` |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `mag_timeout` | Timeout flag |
| `mag_age_ms[15:0]` | Time since last valid read |
| `debounced_fault` | Debounced NACK |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~30 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS2.3: i2c_mag_wrapper.sv

**Purpose:** Top-level I2C integration; exposes clean sensor interface to CS12

**What It Does:**
- Instantiates `i2c_mag_controller` and `mag_fault_detector`
- CDC Synchronization: Routes `mag_valid` and `mag_fault` through 2-FF synchronizers
- Registers magnetic field data stable between transfers
- Aggregates all fault flags

**Why Created:**
- **Consistency with CS1:** Just as CS1 has `spi_imu_wrapper`, CS2 needs `i2c_mag_wrapper` for uniform integration pattern
- **Future Extensibility:** Can add register interface (AXI4-Lite) for gain/range adjustment without modifying `i2c_mag_controller`

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n` | Clock and reset |
| `mag_read_trigger` | 100 Hz or 10 Hz strobe |
| `i2c_sda`, `i2c_scl` | I2C bus |

**Outputs:** `mag_x/y/z`, `mag_valid`, `mag_fault`, `mag_age_ms` (all sys_clk domain)

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~8 |
| DSP48 | 0 |
| BRAM | 0 |

---

## CS3: Sun Sensor ADC Interface — 3 Sub-Modules

### Module Hierarchy

```
CS3: Sun Sensor ADC Interface
│
├─ sun_sensor_wrapper.sv       (TOP-LEVEL INTEGRATION)
│  │
│  ├─ sun_sensor_adc.sv        (4-CHANNEL SEQUENCER)
│  │
│  ├─ adc_multiplexer.sv       (CHANNEL SELECTION & TIMING)
│  │
│  ├─ sun_presence_detector.sv (THRESHOLD & PRESENCE FLAG)
│  │
│  └─ synchronizer.sv          (CDC BRIDGE) [REUSED]
│
└─ Shared Dependencies:
   ├─ spi_master.sv             (SPI PROTOCOL, SHARED WITH CS1) [REUSED]
   ├─ adc_interface.sv          (12-BIT ADC CONTROLLER) [REUSED]
   └─ edge_detect.sv            (EVENT DETECTION) [REUSED]
```

---

### CS3.1: sun_sensor_adc.sv

**Purpose:** Multi-channel ADC sequencer for 4-element sun sensor photodiode array

**What It Does:**
- Reads 4 independent photodiode channels (±X, ±Y sun vectors in body frame)
- Each channel = 12-bit SPI read @ 10 MHz SCLK (shares SPI bus with CS1 IMU via CS_N multiplexing)
- Sequence: Channel 0 → wait 100 µs → Channel 1 → Channel 2 → Channel 3 → cycle repeats
- All 4 channels sampled within 10 ms (100 Hz update rate)
- Validates < 0.1% full-scale crosstalk between channels

**Why Created:**
- **Multiplexing:** Rather than dedicate 4 separate SPI transactions, we multiplex on one shared bus via CS_N decoder. Saves FPGA I/O pins
- **Deterministic Timing:** Settling time between channel changes must be precise to meet crosstalk spec. Dedicated state machine ensures this
- **Reusability:** Can adapt to N-channel ADCs by parameterizing `NUM_CHANNELS`

**Inputs:**

| Signal | Description |
|--------|-------------|
| `clk_100mhz`, `sys_clk`, `rst_n` | Clock and reset |
| `ce_100hz` | Strobe for sequence start |
| `spi_mosi`, `spi_miso`, `spi_sclk` | Shared SPI lines |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `sun_channel[4][11:0]` | One 12-bit value per channel |
| `sun_valid` | All 4 channels valid |
| `channel_select[2:0]` | Currently active channel |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~50 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS3.2: adc_multiplexer.sv

**Purpose:** Channel multiplexing logic with setup/hold timing validation

**What It Does:**
- Generates `channel_select[2:0]` to route photodiode inputs to ADC
- Enforces settling time: after channel change, waits 100 µs before SPI read begins
- Prevents crosstalk by ensuring previous channel's charge is bled off
- Tracks which channel is currently active

**Why Created:**
- **Crosstalk Prevention:** Analog charge bleedoff from capacitive coupling takes time. Dedicated timing logic ensures spec compliance
- **Abstraction:** Keeps sequencer clean; multiplexer handles low-level timing details
- **Testability:** Can independently verify timing without running full sequence

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n` | Clock and reset |
| `channel_request[2:0]` | From `sun_sensor_adc` |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `channel_select[2:0]` | To external analog mux |
| `mux_settled` | Settling time expired; SPI read safe |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~20 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS3.3: sun_presence_detector.sv

**Purpose:** Detect solar eclipse / sun visibility for orbit determination & mode management

**What It Does:**
- Monitors all 4 sun sensor channels
- **Threshold Compare:** If any channel > `SUN_THRESHOLD` (tunable, e.g., 512 ADC counts), sun is visible
- **Debouncing:** Rolling 8-sample average (80 ms window) prevents flickering from clouds/brief eclipses
- **Presence Flag:** Asserts `sun_present = 1` when sun is visible, `0` during eclipse
- **Age Tracking:** Outputs `sun_presence_age_ms` (time since state change)

**Why Created:**
- **Eclipse Detection:** CubeSat periodically enters eclipse. EKF covariance should increase when outside sun-lit geometry
- **Mode Transitions:** CS8 FSM uses `sun_present` to decide: "Can we do sun-pointing?" During eclipse, must switch to magnetic-only attitude control
- **Rolling Average:** Raw ADC is noisy. Averaging over 80 ms smooths transients while remaining fast enough for 90-minute orbit
- **Telemetry:** Ground wants to know: "Is the spacecraft in eclipse?" Useful for mission analysis

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n` | Clock and reset |
| `sun_channel[4][11:0]` | From `sun_sensor_adc` |
| `sun_valid` | New channels ready |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `sun_present` | Boolean: sun visible? |
| `sun_presence_age_ms[15:0]` | Time since last state change |
| `signal_strength_avg[11:0]` | Filtered average (for CS8 telemetry) |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~30 |
| DSP48 | 0 |
| BRAM | ~64 B |

---

## CS4: Quaternion Propagator — 3 Sub-Modules

### Module Hierarchy

```
CS4: Quaternion Propagator
│
├─ quat_propagator_wrapper.sv  (TOP-LEVEL INTEGRATION)
│  │
│  ├─ quat_propagator.sv       (KINEMATIC INTEGRATION)
│  │
│  ├─ quaternion_math.sv       (ALGEBRA: PRODUCT, CONJUGATE, INVERSE)
│  │
│  └─ norm_checker.sv          (UNIT NORM VALIDATION & DRIFT DETECTION)
│
└─ Shared Dependencies:
   ├─ cordic.sv                 (TRIGONOMETRIC FUNCTIONS) [REUSED]
   ├─ sqrt.sv                   (SQUARE ROOT CALCULATOR) [REUSED]
   └─ fp_divider.sv             (FIXED-POINT DIVISION) [REUSED]
```

---

### CS4.1: quat_propagator.sv

**Purpose:** Kinematic integration of attitude quaternion from angular rate

**What It Does:**
- **Algorithm:** `q(k+1) = q(k) + 0.5 × q(k) ⊗ ω × Δt`
  - Takes current quaternion q(k) from EKF feedback (CS5)
  - Takes angular rate ω from IMU (CS1)
  - Computes quaternion rate: `q_rate = 0.5 × q ⊗ ω` (where ⊗ is Hamilton product)
  - Accumulates: `q(k+1) = q(k) + q_rate × Δt`
- Uses Q2.46 fixed-point (48-bit intermediate) for precision
- Executes in 5 ms pipeline (within 10 ms ADCS epoch)
- Uses 3 DSP48 blocks (one per axis of quaternion product)

**Why Created:**
- **Fast Attitude Prediction:** Between EKF updates (10 ms apart), gyro measurements directly yield attitude change. This module provides "quick-look" attitude for CS6 PD controller
- **Feedback Loop:** EKF corrects propagator drift over long term; propagator provides gyro-rate extrapolation for short term. They form complementary pair
- **Separation of Concerns:** EKF is complex (covariance, innovations). Propagator is just kinematics; isolated, testable, reusable
- **DSP Efficiency:** Uses 3 DSP48 blocks (fast multiply-accumulate) rather than >100 LUTs of combinational logic

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n` | Clock and reset |
| `ce_100hz` | 100 Hz strobe |
| `q_prior[4]` | Prior quaternion (from EKF) |
| `omega[3]` | Angular rate from IMU (rad/s, Q15) |
| `gyro_valid` | IMU data valid |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `q_out[4]` | Propagated quaternion (Q15) |
| `q_norm` | Magnitude (should be 1.0 ±0.001) |
| `q_norm_valid` | Norm valid strobe |
| `norm_err_flag` | Norm out of spec |
| `quat_ready` | q_out ready |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~120 |
| DSP48 | 3 |
| BRAM | ~96 B |

---

### CS4.2: quaternion_math.sv

**Purpose:** Core quaternion algebra operations

**What It Does:**
- **quat_multiply(q1, q2):** Hamilton product
  ```
  q1 ⊗ q2 = [w1*w2 - x1*x2 - y1*y2 - z1*z2,
             w1*x2 + x1*w2 + y1*z2 - z1*y2,
             w1*y2 - x1*z2 + y1*w2 + z1*x2,
             w1*z2 + x1*y2 - y1*x2 + z1*w2]
  ```
- **quat_conjugate(q):** `q* = [w, -x, -y, -z]` (inverse for unit quat)
- **quat_inverse(q):** `q^(-1) = conj(q) / |q|^2` (for attitude error computation)
- **quat_normalize(q):** `q_norm = q / |q|` (unit constraint)

**Why Created:**
- **Modularity:** Quaternion ops are used in multiple subsystems (CS4 propagator, CS5 EKF, CS6 control). Central library prevents code duplication
- **Testability:** Can verify quaternion math independently of propagator/controller logic
- **Hardware Efficiency:** Quaternion products are 4-multiply-16-add operations. Dedicated module can pipeline these for speed

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~80 |
| DSP48 | 1–2 |
| BRAM | 0 |

---

### CS4.3: norm_checker.sv

**Purpose:** Unit norm validation; detect quaternion divergence

**What It Does:**
- Computes quaternion magnitude: `|q| = √(q₀² + q₁² + q₂² + q₃²)`
- Compares against unit norm constraint: `|q| = 1 ± TOLERANCE` (e.g., ±0.001)
- If `|q_norm - 1| > TOLERANCE`, asserts `norm_err_flag = 1`
- Outputs `q_norm` for ground telemetry (diagnostic)
- Tracks drift over 60-second window: requirement is drift < 0.01° (≈ 0.0002 radians)

**Why Created:**
- **Numerical Stability:** Fixed-point quaternion arithmetic can accumulate round-off errors. Norm check detects divergence before it corrupts attitude estimate
- **Fault Detection:** If norm drifts significantly, propagator or EKF has a bug. Flag alerts CS8 health monitor
- **Diagnostic Telemetry:** Ground station can monitor `q_norm` to assess FPGA numerical health over weeks of operation

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n` | Clock and reset |
| `q[4]` | Quaternion to check |
| `q_valid` | Strobe |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `q_norm[15:0]` | Magnitude (Q15, should be ≈32767) |
| `q_norm_valid` | Norm valid |
| `norm_err_flag` | Out-of-spec flag |
| `norm_drift_rate` | Rate of drift (dq/dt) |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~60 |
| DSP48 | 1 |
| BRAM | 0 |

---

## CS5: Extended Kalman Filter (7-State) — 6 Sub-Modules

### Module Hierarchy

```
CS5: Extended Kalman Filter
│
├─ ekf_wrapper.sv              (TOP-LEVEL INTEGRATION)
│  │
│  ├─ ekf_core.sv              (MAIN ORCHESTRATOR & CYCLE MANAGER)
│  │  ├─ ekf_predict.sv        (PREDICTION STEP)
│  │  ├─ ekf_update.sv         (MEASUREMENT UPDATE & INNOVATION)
│  │  ├─ ekf_joseph_update.sv  (COVARIANCE UPDATE)
│  │  ├─ ekf_measurement_model.sv (PREDICTED MEASUREMENTS)
│  │  └─ ekf_covariance.sv     (STATE STORAGE & PD CHECK)
│  │
│  └─ synchronizer.sv          (CDC BRIDGE) [REUSED]
│
└─ Shared Dependencies:
   ├─ kalman_1v.sv              (1-VAR KALMAN TEMPLATE) [REUSED]
   ├─ cordic.sv                 (TRIGONOMETRIC FUNCTIONS) [REUSED]
   ├─ sqrt.sv                   (SQUARE ROOT) [REUSED]
   ├─ fp_divider.sv             (FIXED-POINT DIVISION) [REUSED]
   └─ quaternion_math.sv        (FROM CS4) [REUSED]
```

---

### CS5.1: ekf_core.sv

**Purpose:** Main EKF orchestrator; manages predict/update cycle; divergence watchdog

**What It Does:**
- Orchestrates 100 Hz EKF update cycle (10 ms period):
  - **Cycles 0–7 ms:** Call `ekf_predict()` (state + covariance predict)
  - **Cycles 8–9 ms:** Call `ekf_update()` (measurement update + gain application)
  - **Cycle 10 ms:** Divergence watchdog check (5σ innovation threshold)
- **7-State Vector:** `x = [q₀, q₁, q₂, q₃, bₓ, bᵧ, bᵤ]` (quaternion + gyro bias)
- **Divergence Watchdog:** 3-strike rule — if any innovation `|ν[i]| > 5σ` for 3 consecutive epochs → divergence detected → Last-Known-Good (LKG) snapshot recovery
- **Timeout Protection:** If update doesn't complete within 2 ms → `ekf_fault`

**Why Created:**
- **Complexity Management:** EKF is the most mathematically intensive subsystem. Breaking it into predict/update/covariance modules makes it testable
- **Robustness:** Divergence watchdog catches bugs (e.g., matrix inversion failure, measurement model mismatch) before they corrupt entire state
- **LKG Recovery:** Rather than hard-reset to identity quaternion on error, we keep last-good state. Maintains attitude smoothness
- **Real-Time Constraints:** 100 Hz ADCS loop requires predictable timing. FSM cycle manager ensures no overruns

**Inputs:**

| Signal | Description |
|--------|-------------|
| `clk`, `rst_n` | Clock and reset |
| `ce_100hz` | 100 Hz strobe (starts prediction phase) |
| `accel[3]`, `gyro[3]`, `mag[3]` | Sensor measurements (Q15) |
| `meas_valid` | Strobe when sensors ready |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `q_est[4]` | Estimated quaternion |
| `bias_est[3]` | Estimated gyro bias |
| `ekf_valid` | State updated strobe |
| `ekf_fault` | Divergence or timeout flag |
| `P_diag[7]` | Covariance diagonal (diagnostic) |
| `innovation[7]` | Measurement residuals (diagnostic) |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~200 |
| DSP48 | 6 |
| BRAM | 512 B |

---

### CS5.2: ekf_predict.sv

**Purpose:** Prediction step — advance state & covariance using motion model

**What It Does:**
- **State Predict:** `x_pred = f(x_state, gyro)`
  - Quaternion propagation using `quat_propagator` from CS4: `q_pred = q + 0.5 × q ⊗ (gyro - bias) × Δt`
  - Bias remains constant (no external force model): `b_pred = b`
- **Covariance Predict:** `P_pred = F × P × F^T + Q`
  - F = Jacobian (∂f/∂x) — how state changes under motion
  - Q = process noise covariance (gyro noise model)
- Executes in 8 clock cycles (pipelined)

**Why Created:**
- **Separation:** Predict step is distinct from update. Breaking them apart allows independent tuning/testing
- **Efficiency:** 8-cycle pipeline fits easily in 10 ms epoch
- **Modularity:** Can swap motion model without touching update logic

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n`, `ce_100hz` | Clock, reset, strobe |
| `x_state[7]` | Prior state |
| `P_state[7×7]` | Prior covariance |
| `gyro[3]` | Current angular rate |
| `dt` | Time step (10 ms) |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `x_pred[7]` | Predicted state |
| `P_pred[7×7]` | Predicted covariance |
| `predict_valid` | Strobe |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~150 |
| DSP48 | 2 |
| BRAM | 0 |

---

### CS5.3: ekf_update.sv

**Purpose:** Measurement update step — correct state using sensor observations

**What It Does:**
- **Measurement Model:** `z_pred = h(x_pred)`
  - Predict what accel/mag sensors should read given current quaternion
  - Accel: gravity [0, 0, g] rotated to body frame
  - Mag: Earth field rotated to body frame
- **Innovation:** `ν = z_actual - z_pred`
- **Kalman Gain:** `K = P_pred × H^T × (H × P_pred × H^T + R)^(-1)` (fixed at 0.1, tunable)
- **State Correction:** `x_est = x_pred + K × ν`

**Why Created:**
- **Clean Separation:** Update is conceptually distinct from predict. Isolating it allows swapping measurement models (e.g., sun sensor, star tracker)
- **Reusability:** Generic update equations; only measurement model h() changes per sensor type
- **Testability:** Can verify innovation computation independently

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n` | Clock and reset |
| `x_pred[7]`, `P_pred[7×7]` | From predict step |
| `accel[3]`, `mag[3]` | Measured sensor data (normalized) |
| `meas_valid` | Sensors ready |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `x_est[7]` | Corrected state |
| `innovation[7]` | Residuals (for diagnostics/ground telemetry) |
| `update_valid` | Strobe |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~180 |
| DSP48 | 3 |
| BRAM | 0 |

---

### CS5.4: ekf_joseph_update.sv

**Purpose:** Joseph-form covariance update — numerically stable & guarantees positive-definiteness

**What It Does:**
- **Standard Update:** `P = (I - K×H) × P × (I - K×H)^T + K×R×K^T` (suffers from round-off errors)
- **Joseph Form:** `P = (I - K×H) × P_pred × (I - K×H)^T + K×R×K^T`
  - Explicitly symmetric
  - Proven to remain positive-definite even under finite precision
- **PD Check:** Verifies diagonal elements `P[i,i] > 0` after update. If any diagonal < 0, signals `covariance_error`

**Why Created:**
- **Numerical Robustness:** Fixed-point arithmetic accumulates round-off. Joseph form is the gold standard for FP EKF implementations
- **Fault Detection:** Non-positive-definite covariance indicates broken algorithm. Detecting this allows graceful recovery vs silent divergence
- **Space Heritage:** Proven in practice on many spacecraft

**Inputs:**

| Signal | Description |
|--------|-------------|
| `P_pred[7×7]` | Predicted covariance |
| `K[7×7]` | Kalman gain |
| `H[7×3]` | Measurement Jacobian |
| `R[3×3]` | Measurement noise covariance |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `P_est[7×7]` | Updated covariance |
| `P_diag[7]` | Diagonal (for telemetry) |
| `covariance_error` | Non-PD flag |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~200 |
| DSP48 | 1 |
| BRAM | 0 |

---

### CS5.5: ekf_measurement_model.sv

**Purpose:** Compute predicted sensor readings given current state

**What It Does:**
- **Accelerometer Model:** `a_body = R(q) × [0, 0, -g]` — Rotate gravity vector to body frame using quaternion q
- **Magnetometer Model:** `m_body = R(q) × B_ecef` — Rotate Earth's magnetic field to body frame
- Uses `cordic.sv` for sine/cosine of attitude angles
- Output: predicted [accel, mag] vectors in body frame

**Why Created:**
- **Core EKF Equation:** innovation = measured - predicted. This module computes "predicted"
- **Reusability:** Measurement model is often the first thing to change in EKF tuning. Isolated module simplifies modifications
- **Testability:** Can verify that quaternion rotations match expected gravity/field vectors

**Inputs:**

| Signal | Description |
|--------|-------------|
| `q_est[4]` | Estimated quaternion |
| `bias_est[3]` | Estimated gyro bias |
| `ecef_position[3]` | Current position (for mag field lookup) |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `accel_pred[3]` | Predicted accel in body frame |
| `mag_pred[3]` | Predicted mag in body frame |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~100 |
| DSP48 | 2 |
| BRAM | 0 |

---

### CS5.6: ekf_covariance.sv

**Purpose:** Covariance matrix storage, access, and numerical checks

**What It Does:**
- Stores 7×7 symmetric covariance matrix P as 28 unique elements (exploiting symmetry)
- Dual-port BRAM allows simultaneous read/write
- **Positive-Definite Check:** After update, verify all diagonal elements > EPSILON (e.g., 1e-5)
- **Overflow Detection:** If any covariance element > 32767 (Q15 limit), flag overflow

**Why Created:**
- **Memory Efficiency:** Full 7×7×16-bit = 784 bits. Storing in distributed LUTs wastes logic. BRAM is designed for this
- **Dual-Port Access:** Allows EKF to read P for gain computation while writing updated P
- **Numerical Safety:** PD checks catch bugs before they propagate
- **Flexibility:** Can swap BRAM implementation for different FPGA families

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~20 |
| DSP48 | 0 |
| BRAM | 512 B |

---

## CS6: PD Attitude Control Law — 3 Sub-Modules

### Module Hierarchy

```
CS6: PD Attitude Control Law
│
├─ pd_control_wrapper.sv           (TOP-LEVEL INTEGRATION)
│  │
│  ├─ pd_controller_quaternion.sv  (QUATERNION ERROR & PD LAW)
│  │
│  ├─ torque_saturation.sv         (±10 mNm LIMIT & FLAG)
│  │
│  └─ control_law_engine.sv        (LAW INTEGRATION & OUTPUT MUX)
│
└─ Shared Dependencies:
   ├─ pid_controller.sv             (PID TEMPLATE) [REUSED]
   └─ fp_divider.sv                 (FIXED-POINT DIVISION) [REUSED]
```

---

### CS6.1: pd_controller_quaternion.sv

**Purpose:** Quaternion error computation + PD control law

**What It Does:**
- **Quaternion Error:** `q_err = q_desired ⊗ q_est^(-1)`
  - Vector part `q_err[1:3]` is the 3-axis error axis scaled by sin(error/2)
- **PD Law:** `τ = -Kp × q_err[1:3] - Kd × ω`
  - **P Term:** `-Kp × q_err[1:3]` — Proportional to attitude error; restores to desired quaternion
  - **D Term:** `-Kd × ω` — Proportional to angular rate; damps oscillations
  - Gains Kp, Kd tunable (future AXI4-Lite)
- Executes @ 1 kHz (1 ms clock-enable from CS12)

**Why Created:**
- **Core Control:** Attitude feedback control is the heart of ADCS. This module converts state estimate (q_est from CS5) into actuator commands (τ)
- **Quaternion Formulation:** Using quaternion error avoids gimbal lock and handles large attitude errors gracefully
- **Separation:** Quaternion math isolated from saturation logic; easier to modify gains

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n`, `ce_1khz` | Clock, reset, strobe |
| `q_est[4]` | Estimated attitude (from CS5) |
| `q_desired[4]` | Commanded attitude |
| `omega[3]` | Angular rate (from CS1) |
| `Kp_coeff`, `Kd_coeff` | Control gains |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `torque_unsaturated[3]` | Raw τ before saturation (mNm) |
| `ctrl_valid` | Strobe |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~100 |
| DSP48 | 2 |
| BRAM | 0 |

---

### CS6.2: torque_saturation.sv

**Purpose:** Hard saturation to physical actuator limits

**What It Does:**
- **Limit:** ±10 mNm per axis (based on reaction wheel and magnetorquer capabilities)
  ```
  if (tau_x > +10) τ_x_sat = +10, sat_flag_x = 1
  else if (tau_x < -10) τ_x_sat = -10, sat_flag_x = 1
  else τ_x_sat = tau_x, sat_flag_x = 0
  ```
- Outputs `saturation_flag[3]` and `sat_count` (number of axes saturated)

**Why Created:**
- **Actuator Realism:** Control laws assume unlimited authority. Real actuators have hard limits. Saturation prevents over-commanding
- **Diagnostics:** Saturated axes indicate disturbance environment or insufficient control authority
- **Anti-Windup:** Saturation prevents integrator from accumulating excessive error

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~40 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS6.3: control_law_engine.sv

**Purpose:** Top-level integration; routes PD law + saturation → CS7 actuators

**What It Does:**
- Instantiates `pd_controller_quaternion` and `torque_saturation`
- Passes `tau_saturated` to CS7 (Reaction Wheel & Magnetorquer Drivers)
- Exports `sat_flags` and `sat_count` to CS8 (ADCS FSM for logging)

**Why Created:**
- **Consistency:** Matches pattern of other subsystems (wrapper + core logic)
- **Testability:** Can independently verify control law without actuator driver logic

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~8 |
| DSP48 | 0 |
| BRAM | 0 |

---

## CS7: Reaction Wheel & Magnetorquer Drivers — 4 Sub-Modules

### Module Hierarchy

```
CS7: RW & MTQ Actuators
│
├─ actuator_wrapper.sv             (TOP-LEVEL INTEGRATION)
│  │
│  ├─ rw_spi_driver.sv             (RW MOTOR COMMANDS VIA SPI)
│  │
│  ├─ magnetorquer_pwm.sv          (MTQ COIL PWM @ 10 kHz)
│  │
│  ├─ actuator_command_arbiter.sv  (ROUTE τ → RW SPEED + MTQ DUTY)
│  │
│  └─ fault_status_monitor.sv      (RW FAULT POLLING, MTQ CURRENT)
│
└─ Shared Dependencies:
   ├─ spi_master.sv                 (SPI FOR RW COMMANDS) [REUSED]
   └─ pwm_gen.sv                    (PWM FOR MTQ COILS) [REUSED]
```

---

### CS7.1: rw_spi_driver.sv

**Purpose:** Convert torque commands into SPI motor driver commands

**What It Does:**
- **Mapping:** τ → motor speed command
  - Speed range: ±6000 RPM (16-bit signed, 1 LSB = 0.2 RPM)
  - Linear: `ω_rpm = τ_mnm / (moment of inertia) × conversion factor`
- **SPI Protocol:** 3 independent SPI writes @ 1 kHz (one per RW, CS_N multiplexed)
  - Each write: 16-bit speed command + fault status read
  - SPI clock: 10 MHz (shared with CS1, CS3)

**Why Created:**
- **Hardware Interface:** RW drivers (DRV8302 or equivalent) are accessed via SPI. Need dedicated sequencer to format commands
- **Real-Time:** Commands must update @ 1 kHz (synchronized with CS6 control law)
- **Fault Polling:** Read back RW status (over-temp, over-current) for health monitoring

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `clk_100mhz`, `rst_n` | Clocks and reset |
| `ce_1khz` | 1 kHz command rate |
| `tau_saturated[3]` | From CS6 |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `rw_sclk`, `rw_mosi[3]`, `rw_miso[3]`, `rw_cs_n[3]` | SPI lines |
| `rw_fault[3]` | Fault flags from each wheel |
| `rw_speed_actual[3][15:0]` | Feedback speeds |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~90 |
| DSP48 | 1 |
| BRAM | 128 B |

---

### CS7.2: magnetorquer_pwm.sv

**Purpose:** Drive magnetic coils via PWM for momentum management

**What It Does:**
- **PWM @ 10 kHz:** 3-axis PWM outputs (one per coil)
  - ≥12-bit resolution (4096 duty levels)
  - Polarity: ±duty controls coil direction
- **SAFE Blanking:** During SAFE mode (CS8), all MTQ PWM set to 0 (coils off)
- When RW is saturated, magnetorquers provide momentum dumping

**Why Created:**
- **Dual Actuation:** RWs are fast but can saturate. Magnetorquers are slow but have infinite authority (via Earth's field)
- **Momentum Dumping:** RWs accumulate momentum from external torques. Magnetorquers periodically "dump" momentum
- **PWM Efficiency:** Digital PWM is efficient for solenoid drives; no linear regulator losses

**Inputs:**

| Signal | Description |
|--------|-------------|
| `clk_100mhz`, `rst_n`, `ce_1khz` | Clock, reset, strobe |
| `tau_saturated[3]` | Torque commands |
| `safe_mode` | Force PWM to 0 |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `mtq_pwm[3]` | PWM outputs to driver |
| `mtq_duty[3][11:0]` | Actual duty values (for telemetry) |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~60 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS7.3: actuator_command_arbiter.sv

**Purpose:** Distribute torque commands to RW and MTQ subsystems

**What It Does:**
- Routes `τ[3]` from CS6 to both RW and MTQ
- **RW Priority:** First tries to satisfy torque with reaction wheels (faster, more precise)
- **MTQ Fallback:** If RW saturated, magnetorquers engage
- **Blending:** Can simultaneously command both (e.g., RW for attitude, MTQ for momentum management)

**Why Created:**
- **Abstraction:** CS6 doesn't know about RW vs MTQ. Arbiter maps abstract torque to hardware
- **Flexibility:** Can change actuation strategy (RW-only, MTQ-only, blended) without modifying CS6

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~30 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS7.4: fault_status_monitor.sv

**Purpose:** Monitor actuator health (RW faults, MTQ currents)

**What It Does:**
- **RW Monitoring:** Polls fault status from SPI reads (over-temperature, over-current, stalled rotor)
- **MTQ Monitoring:** Reads coil current via ADC — detects opens (no current) and shorts (excessive current)
- **Watchdog:** If RW reports fault, de-assert speed command (coast RW safely)

**Why Created:**
- **Robustness:** Actuator failures (motor fault, coil break) must be detected quickly to avoid compounding failures
- **Telemetry:** Ground wants to know RW health; helps predict maintenance needs

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~50 |
| DSP48 | 0 |
| BRAM | 0 |

---

## CS8: ADCS Mode FSM & Health Monitor — 6 Sub-Modules

### Module Hierarchy

```
CS8: ADCS Mode FSM & Health Monitor
│
├─ adcs_fsm_wrapper.sv         (TOP-LEVEL INTEGRATION)
│  │
│  ├─ adcs_fsm_controller.sv   (5-STATE FSM ORCHESTRATOR)
│  │
│  ├─ adcs_state_machine.sv    (STATE TRANSITION LOGIC)
│  │
│  ├─ fault_detector.sv        (MULTI-FAULT DETECTION)
│  │
│  ├─ adcs_health_monitor.sv   (AGGREGATES HEALTH SIGNALS)
│  │
│  ├─ fault_logger.sv          (FIFO FAULT LOG)
│  │
│  └─ bram_circular_buffer.sv  (256×96B DUAL-PORT BUFFER)
│
└─ Shared Dependencies:
   ├─ edge_detect.sv            (EDGE DETECTION) [REUSED]
   ├─ synchronizer.sv           (CDC BRIDGES) [REUSED]
   └─ debouncer.sv              (GLITCH REJECTION) [REUSED]
```

---

### CS8.1: adcs_fsm_controller.sv

**Purpose:** Main FSM orchestrator for ADCS mode management

**What It Does:**
- **5 States:**

| State | Description |
|-------|-------------|
| BOOT | Initialization; validates sensors; transitions to DETUMBLE |
| DETUMBLE | High angular rate damping using magnetorquers |
| COARSE_POINT | Coarse attitude acquisition using sun + mag sensors |
| FINE_POINT | High-precision pointing; EKF active; PD control; laser enabled (CS10) |
| SAFE | All actuators off; wait for ground command recovery |

- **Transitions:**
  - BOOT → DETUMBLE: Automatic on startup
  - DETUMBLE → COARSE_POINT: When `|ω| < threshold`
  - COARSE_POINT → FINE_POINT: When `q_error < threshold & EKF converged`
  - Any → SAFE: On critical fault or ground command
  - SAFE → IDLE: Manual ground command
- **Timer Logic:** Enforces state dwell times (e.g., min 5 s in DETUMBLE to ensure momentum damping)

**Why Created:**
- **Autonomous Mission Ops:** CubeSat orbits 90 minutes; can't constantly contact ground. FSM autonomously manages safe operation
- **Robustness:** States degrade gracefully. If EKF fails, drop back to COARSE_POINT
- **Fail-Safe:** SAFE mode is lowest-power; all commands zeroed; spacecraft tumble-stabilizes passively

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n`, `ce_100hz` | Clock, reset, FSM tick |
| `imu_valid`, `mag_valid`, `sun_valid` | Sensor heartbeats |
| `q_error_mag` | Magnitude of attitude error |
| `omega_max` | Max angular rate observed |
| `ekf_valid` | EKF converged |
| `imu_fault`, `mag_fault`, `ekf_fault` | Fault flags |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `adcs_mode[2:0]` | Current state (0=BOOT ... 4=SAFE) |
| `mode_valid` | Strobe on state change |
| `safe_mode` | To CS7, CS10 (kill actuators) |
| `laser_enable` | To CS10 (assert in FINE_POINT only) |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~120 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS8.2: adcs_state_machine.sv

**Purpose:** State transition logic and guards

**What It Does:**
- Evaluates guard conditions for each transition:
  - DETUMBLE → COARSE_POINT: `omega_max < 10°/s & dt_in_state > 5s`
  - COARSE_POINT → FINE_POINT: `q_error < 5° & ekf_valid & P_diag < threshold`
  - Any → SAFE: `any_critical_fault | !heartbeat[imu/mag] > 100ms`
- Enforces exclusive state occupancy (only one state active at a time)
- Provides next-state logic and state-to-output mappings

**Why Created:**
- **Clarity:** Guards decoupled from state FSM itself; easier to modify conditions
- **Debuggability:** Can trace guard evaluation in simulation

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~100 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS8.3: fault_detector.sv

**Purpose:** Multi-fault detection; aggregates sensor/algorithm failures

**What It Does:**
- **Timeout Faults:**
  - IMU: no heartbeat > 5 ms → `imu_timeout`
  - Magnetometer: no heartbeat > 10 ms → `mag_timeout`
  - EKF: no update > 2 ms → `ekf_timeout`
- **Range Faults:**
  - Gyro overflow: `|ω| > 500°/s` → `gyro_overflow`
  - Quaternion norm drift: `|q_norm - 1| > 0.001` → `quat_norm_error`
- **Algorithm Faults:**
  - EKF divergence: innovation > 5σ → `ekf_divergence`
  - Covariance non-positive-definite → `covariance_error`
- **Aggregation:** ORs all fault signals → `any_fault`

**Why Created:**
- **Early Detection:** Catching faults early (timeouts, overflows) prevents data corruption
- **Modularity:** Fault detection isolated from FSM logic; can test independently
- **Extensibility:** Easy to add new fault types (e.g., actuator faults from CS7)

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~80 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS8.4: adcs_health_monitor.sv

**Purpose:** Aggregate all health signals; compute overall subsystem health

**What It Does:**
- Tracks "health score" (0–100%):

| Condition | Score Impact |
|-----------|-------------|
| All sensors heartbeating, EKF converged, no faults | 100% |
| IMU timeout | -25% |
| Magnetometer fault | -50% |
| EKF divergence | -75% |
| Multiple critical failures | 0% |

- **Health Flag:** `health_flag = (score >= 50%)` → subsystem operational
- Outputs `health_score` to telemetry (CS11)

**Why Created:**
- **Mission Assessment:** Ground operator wants single metric: "Is ADCS working?"
- **Telemetry Compression:** One byte health score instead of 8 separate fault flags
- **Decision Support:** Ground can decide: "Should I command mode change?" based on health

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~50 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS8.5: fault_logger.sv

**Purpose:** Capture fault events with timestamps for post-mission analysis

**What It Does:**
- **FIFO:** 8-entry circular buffer — Entry: `[fault_code (8-bit), timestamp (32-bit)]`
  - fault_code encodes: 001=IMU_timeout, 010=MAG_timeout, 100=EKF_divergence, etc.
- **Write:** On fault detection, append `[fault_code, MET_timestamp]` to FIFO
- **Read:** CS11 telemetry reads out FIFO entries on demand
- **Overflow:** If FIFO full, oldest entry overwritten

**Why Created:**
- **Historical Record:** Instead of only current fault state, keep history of when faults occurred
- **Root Cause Analysis:** Ground can correlate fault sequence with mission events (eclipse entry, high solar wind)
- **Anomaly Detection:** Increasing fault rate indicates degrading hardware

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~60 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS8.6: bram_circular_buffer.sv

**Purpose:** High-speed, dual-port circular logging buffer for ADCS telemetry

**What It Does:**
- **Circular Buffer:** 256 entries, 96 bits per entry
  - Entry format: `[timestamp (32-bit), q_est (32-bit), omega (32-bit)]`
  - Total: 256×96 = 24,576 bits ≈ 3 KB
- **Dual Port:** Write port: CS8 writes at 100 Hz; Read port: CS11 reads asynchronously
- **Wrap-Around:** When write pointer reaches address 255, wraps to 0 (oldest data overwritten)

**Why Created:**
- **Real-Time Logging:** Captures 10 seconds of ADCS history (256 entries @ 100 Hz)
- **Dual-Port Performance:** Write and read simultaneously without stalling
- **BRAM Efficiency:** Uses actual BRAM18 blocks rather than LUT-based FIFO

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~30 |
| DSP48 | 0 |
| BRAM | 512 B (dedicated BRAM18) |

---

## CS9: Orbit Propagator (SGP4-Lite) — 7 Sub-Modules

### Module Hierarchy

```
CS9: Orbit Propagator
│
├─ orbit_propagator_wrapper.sv     (TOP-LEVEL INTEGRATION)
│  │
│  ├─ tle_parser.sv                (TLE VALIDATION & PARSING)
│  │
│  ├─ sgp4_propagator_core.sv      (MAIN SGP4 ENGINE)
│  │
│  ├─ lvlh_converter.sv            (ECI → LVLH FRAME TRANSFORMATION)
│  │
│  ├─ ground_track_calculator.sv   (GEODETIC LAT/LON)
│  │
│  ├─ contact_window_predictor.sv  (AOS/LOS PREDICTION)
│  │
│  ├─ orbit_health_monitor.sv      (VALIDITY & OVERFLOW FLAGS)
│  │
│  └─ orbit_state_manager.sv       (MET COUNTER, EPOCH MANAGEMENT)
│
└─ Shared Dependencies:
   ├─ cordic.sv                     (TRIGONOMETRY) [REUSED]
   ├─ sqrt.sv                       (DISTANCE CALCULATIONS) [REUSED]
   └─ fp_divider.sv                 (FIXED-POINT DIVISION) [REUSED]
```

---

### CS9.1: tle_parser.sv

**Purpose:** Ingest TLE (Two-Line Element) data; validate and extract orbital elements

**What It Does:**
- **TLE Format:** 2 lines, 69 characters each — Line 1: NORAD ID, Epoch, Inclination, RAAN, Eccentricity, Argument of Perigee, Mean Anomaly, Mean Motion; Line 2: Full element set + Checksum
- **Validation:** Checksum verification (mod-10), format and range checks
- **Parsing:** Extracts: semi-major axis (a), eccentricity (e), inclination (i), RAAN (Ω), argument of perigee (ω), mean anomaly (M), mean motion (n)
- **Epoch:** Parses TLE epoch (YYDDD.FFFFFFFF) → Modified Julian Date (MJD)

**Why Created:**
- **Input Validation:** Ground sends TLE updates via uplink. Checksums catch corruption
- **Data Extraction:** TLE is dense; parser produces readable orbital elements
- **Epoch Management:** MJD is standard for orbital propagation; conversion happens here

**Inputs:**

| Signal | Description |
|--------|-------------|
| `tle_input[138]` | 2×69-character TLE data (via AXI4-Lite from ground) |
| `tle_write` | Strobe to accept new TLE |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `orbital_elements[6][31:0]` | [a, e, i, RAAN, ω, M] |
| `epoch_mjd[31:0]` | Modified Julian Date |
| `tle_valid` | Checksum passed |
| `tle_fault[7:0]` | Error code (if invalid) |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~100 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS9.2: sgp4_propagator_core.sv

**Purpose:** SGP4-Lite orbit propagation engine

**What It Does:**
- **SGP4-Lite Algorithm:**
  - Includes J2 perturbation (oblateness of Earth) and sun-synchronous orbit correction
  - Excludes higher harmonics (J3, J4) for FPGA efficiency
  - ≤5 km accuracy over 24 hours
- **Execution:** @ 1 Hz (50 ms propagation time)
  - Kepler's equation solver (Newton iteration)
  - Eccentric anomaly → true anomaly → ECI position/velocity

**Why Created:**
- **Onboard Autonomy:** Ground can load TLE; spacecraft computes its own position in real-time
- **ISL & Contact Prediction:** Relative positions (CS9.5) and AOS predictions require accurate orbit
- **Simplified Algorithm:** Full SGP4 is complex. Lite version captures 99% accuracy with <1/10 the resources

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n`, `ce_1hz` | Clock, reset, strobe |
| `orbital_elements[6]` | From `tle_parser` |
| `epoch_mjd` | TLE epoch |
| `current_mjd` | Current time (from `orbit_state_manager`) |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `eci_pos[3][31:0]` | Position in ECI frame (km) |
| `eci_vel[3][31:0]` | Velocity in ECI frame (km/s) |
| `propagator_valid` | Results valid (no overflow) |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~300 |
| DSP48 | 8 |
| BRAM | 256 B |

---

### CS9.3: lvlh_converter.sv

**Purpose:** Transform ECI position/velocity into LVLH (Local Vertical Local Horizontal) frame

**What It Does:**
- **LVLH Definition:**
  - Local Vertical (Z): radially outward from Earth center
  - Local Horizontal (X): in direction of velocity (along-track)
  - Local Horizontal (Y): perpendicular to orbit plane (cross-track)
- **Transformation:**
  - Compute unit vectors: `r̂ = r/|r|`, `ĥ = r × v / |r × v|`
  - Construct rotation matrix: `R_ECI→LVLH = [ĥ, r̂ × ĥ, r̂]^T`

**Why Created:**
- **ISL Geometry:** To point laser at another satellite, need LVLH relative position
- **Ground Contact:** Ground-track calculation needs geodetic coords (derived from LVLH Z-component)
- **Relative Navigation:** LVLH is standard for proximity operations

**Inputs:** `eci_pos[3]`, `eci_vel[3]` — From `sgp4_propagator_core`

**Outputs:** `lvlh_pos[3]`, `lvlh_vel[3]`, `R_eci_to_lvlh[3×3]`

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~150 |
| DSP48 | 2 |
| BRAM | 0 |

---

### CS9.4: ground_track_calculator.sv

**Purpose:** Compute geodetic latitude/longitude and altitude above Earth

**What It Does:**
- **ECI → ECEF:** Apply time-dependent Earth rotation matrix (sidereal time): `r_ECEF = R(t) × r_ECI`
- **ECEF → Geodetic:** Iterative Newton's method — computes latitude (φ), longitude (λ), altitude (h above WGS84)
  - Tolerance: ±0.1° (per CS-ORB-006)

**Why Created:**
- **Ground Passes:** To predict AOS at ground station, need spacecraft lat/lon
- **Eclipse Detection:** Altitude + latitude determine if in Earth's shadow
- **Mission Planning:** Ground wants to know: "Where is my CubeSat?"

**Inputs:** `eci_pos[3]`, `jd` (Julian day for Earth rotation)

**Outputs:**

| Signal | Description |
|--------|-------------|
| `latitude[15:0]` | Geodetic latitude (Q15, rad) |
| `longitude[15:0]` | Geodetic longitude (Q15, rad) |
| `altitude[31:0]` | Height above WGS84 (km) |
| `eclipse_flag` | Asserted if in Earth's shadow |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~180 |
| DSP48 | 2 |
| BRAM | 256 B |

---

### CS9.5: contact_window_predictor.sv

**Purpose:** Predict AOS/LOS (Acquisition/Loss of Signal) at ground station

**What It Does:**
- **AOS Prediction:** Find time when spacecraft elevation angle exceeds horizon threshold (e.g., 5°)
  - Binary search forward in time for elevation crossing
  - Accuracy: ≤30 seconds (CS-ORB-006)
- **LOS Prediction:** Find time when elevation drops below threshold
- **Contact Duration:** AOS to LOS time

**Why Created:**
- **Ground Station Ops:** Ground doesn't have realtime satellite tracks. Spacecraft predicts own AOS so ground knows when to expect downlink
- **Link Budget:** Only transmit when elevation > threshold (stronger signals, less path loss)
- **Autonomous Scheduling:** Spacecraft can autonomously time telemetry downlinks

**Inputs:**

| Signal | Description |
|--------|-------------|
| `latitude[15:0]`, `longitude[15:0]` | Spacecraft position |
| `ground_lat[15:0]`, `ground_lon[15:0]` | Ground station location (via AXI4-Lite) |
| `horizon_elevation[15:0]` | Elevation threshold (typically 5°–10°) |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `aos_time_sec[31:0]` | Time to next AOS |
| `los_time_sec[31:0]` | Time to next LOS |
| `elevation_angle[15:0]` | Current elevation |
| `los_valid` | Predictions valid |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~200 |
| DSP48 | 2 |
| BRAM | 0 |

---

### CS9.6: orbit_health_monitor.sv

**Purpose:** Validate propagation results; detect overflow/errors

**What It Does:**
- **Validity Checks:**
  - Position range: `|r| = 6,371–8,371 km`
  - Velocity range: `|v| = 7–8 km/s`
- **TLE Age:** Track days since TLE epoch
  - After 30 days: `tle_age_warning = 1`
  - After 60 days: `tle_age_error = 1` (accuracy degraded significantly)
- **Propagation Time:** If propagation takes > 50 ms: `timeout_flag = 1`

**Why Created:**
- **Sanity Checks:** Catch algorithmic bugs (e.g., overflow in Kepler solver) before bad data propagates
- **TLE Management:** Spacecraft reminds ground when TLE is stale
- **Operational Awareness:** Ground knows accuracy limits of onboard orbit solution

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~80 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS9.7: orbit_state_manager.sv

**Purpose:** Manage MET (Mission Elapsed Time) counter; maintain current epoch

**What It Does:**
- **MET Counter:** 32-bit counter, increments @ 1 Hz
  - Loaded via AXI4-Lite command from ground (GPS time sync)
  - Survives soft reset (holds value)
- **Current Time:** Converts MET → Modified Julian Date (MJD): `MJD = (MET_sec / 86400) + epoch_mjd`
- **Epoch Management:** Stores reference epoch (launch date)

**Why Created:**
- **Autonomous Timekeeping:** CubeSat doesn't have GPS receiver. Ground periodically sends time updates via command
- **Orbit Propagation:** SGP4 needs current MJD as input
- **Telemetry Timestamping:** All telemetry packets timestamped with MET

**Inputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk`, `rst_n`, `ce_1hz` | Clock, reset, strobe |
| `met_load_value[31:0]` | Ground sync command (via AXI4-Lite) |
| `met_load_en` | Strobe to load |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `met_seconds[31:0]` | Current mission elapsed time |
| `current_mjd[31:0]` | Current Modified Julian Date |
| `met_valid` | Time reference valid |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~60 |
| DSP48 | 0 |
| BRAM | 256 B |

---

## CS12: System Integration & Clock Distribution — 7 Sub-Modules

### Module Hierarchy

```
CS12: System Integration & Clock Distribution
│
├─ top_cubesat_mvp.sv          (SYSTEM TOP-LEVEL)
│  │
│  ├─ clk_manager.sv           (CLOCK TREE & ENABLE GENERATION)
│  │
│  ├─ reset_controller.sv      (POR & RESET SEQUENCING)
│  │
│  ├─ system_monitor.sv        (XADC TEMPERATURE/VOLTAGE)
│  │
│  ├─ power_monitor.sv         (PER-SUBSYSTEM POWER TRACKING)
│  │
│  ├─ resource_arbiter.sv      (BRAM/DSP48 ARBITRATION)
│  │
│  └─ [Instantiates all CS1–CS11]
│
└─ Shared Dependencies:
   ├─ clk_divider.sv            (FREQUENCY DIVISION) [REUSED]
   ├─ tick_gen.sv               (STROBE GENERATION) [REUSED]
   └─ synchronizer.sv           (CDC BRIDGES) [REUSED]
```

---

### CS12.1: clk_manager.sv

**Purpose:** Central clock tree; generates strobes for all subsystems

**What It Does:**
- **Input:** `clk_ref_100mhz` (100 MHz external reference)
- **Outputs:**

| Signal | Description |
|--------|-------------|
| `sys_clk` | Main system clock (50 MHz or 100 MHz pass-through) |
| `clk_100mhz` | SPI/ADC clock (always 100 MHz) |
| `ce_1hz` | 1 Hz clock enable (for orbit propagator, telemetry) |
| `ce_100hz` | 100 Hz clock enable (for ADCS control loop) |
| `ce_1khz` | 1 kHz clock enable (for PD controller) |

- **Jitter Requirement:** < 5% of cycle period

**Why Created:**
- **Centralized Timing:** All 12 subsystems need synchronized strobes. Central manager prevents clock skew/desynchronization
- **Multi-Rate Control:** Different subsystems run at different rates (1 Hz, 100 Hz, 1 kHz). Clock enables provide deterministic partitioning

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~40 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS12.2: reset_controller.sv

**Purpose:** Reset sequencing; ensure deterministic startup

**What It Does:**
- **POR (Power-On Reset):** External button-press → `rst_pbl_n` → synchronized through debouncer → system-wide reset
- **Soft Reset:** Ground command via AXI4-Lite — Resets most logic except MET counter, TLE storage
- **Warm Reset:** Selective reset (e.g., reset EKF only without resetting RW history)
- **Reset Sequencing:** Ensures dependencies are met (clocks available before reset release)

**Why Created:**
- **Reliability:** Proper reset prevents metastability and stuck states
- **In-Orbit Recovery:** Ground can command soft reset if system hangs
- **Startup:** Deterministic boot sequence sets initial state

**Inputs:**

| Signal | Description |
|--------|-------------|
| `rst_pbl_n` | External POR button |
| `soft_reset_cmd` | Ground command (via AXI4-Lite) |
| `warm_reset_mask[11:0]` | Selective subsystem reset mask |

**Outputs:**

| Signal | Description |
|--------|-------------|
| `rst_n` | Global reset (to all subsystems) |
| `rst_subsystem[11:0]` | Per-subsystem reset (if warm reset active) |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~30 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS12.3: system_monitor.sv

**Purpose:** Monitor FPGA die temperature and power supply voltages

**What It Does:**
- **XADC (Xilinx ADC):** Built-in analog-to-digital converter
  - Measures die temperature (°C)
  - Measures VCCINT (core voltage, nominal 1.0 V)
  - Measures VCCO (I/O voltage, nominal 3.3 V)
- **Threshold Monitoring:**
  - Thermal alert: if T > 80°C → throttle (reduce clock, disable non-critical ops)
  - Voltage alert: if VCCINT < 0.95 V or VCCO < 3.15 V → `power_fault = 1`

**Why Created:**
- **On-Orbit Health:** Space environment has extreme temperature variation (sun-lit: +80°C, eclipse: -40°C)
- **Power Budget:** Solar panel output varies with eclipse/sun angle. Monitoring voltages helps diagnose power system faults
- **Graceful Degradation:** If FPGA overheating, can reduce processing vs shutdown entirely

**Outputs:**

| Signal | Description |
|--------|-------------|
| `fpga_temp[15:0]` | Temperature (Q15, °C) |
| `vccint[15:0]`, `vcco[15:0]` | Voltages |
| `thermal_alert`, `power_fault` | Threshold flags |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~50 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS12.4: power_monitor.sv

**Purpose:** Track per-subsystem power consumption estimates

**What It Does:**
- **Power Model:** Estimate power for each subsystem based on activity:
  - CS1 (IMU SPI): P ∝ SPI transaction rate × clock frequency
  - CS5 (EKF): P ∝ DSP48 utilization × cycle count
  - CS10 (Laser gimbal): P ∝ motor stepping rate
- **Budget Check:** Sum per-subsystem estimates; compare against total power budget (e.g., 5 W)
  - If sum > budget: `power_budget_exceeded = 1`

**Why Created:**
- **Power-Constrained:** CubeSat runs on solar panel + battery. Ground needs visibility into power consumption
- **Mission Planning:** Decide: "Can I enable laser comms or does that exceed power budget?"
- **Anomaly Detection:** If power suddenly spikes → possible fault (stuck motor, runaway DSP)

**Outputs:**

| Signal | Description |
|--------|-------------|
| `power_per_subsystem[11:0][15:0]` | Watts per subsystem |
| `total_power` | Sum of all subsystems |
| `power_budget_exceeded` | Budget exceeded flag |

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~100 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS12.5: resource_arbiter.sv

**Purpose:** Arbitrate shared BRAM and DSP48 resources among subsystems

**What It Does:**
- **BRAM Arbitration:** FPGA has fixed BRAM18 blocks (e.g., 45 KB total on XC7A35T)
  - CS8 circular buffer: 3 KB; CS5 covariance matrix: 512 B; CS9 orbit propagator: 256 B
  - Arbiter prevents double-allocation
- **DSP48 Arbitration:** Time-multiplex if necessary (swap which subsystem uses a DSP per cycle)
- **Conflict Resolution:** If 2 subsystems need same resource at same time, prioritize by criticality (CS5 EKF > CS10 laser)

**Why Created:**
- **Resource Constraints:** Real FPGAs have finite silicon. Can't have unlimited BRAM/DSP
- **Integration:** System integrator must verify total resource usage < available before synthesis
- **Transparency:** Arbiter makes resource sharing explicit and debuggable

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~50 |
| DSP48 | 0 |
| BRAM | 0 |

---

### CS12.6: top_cubesat_mvp.sv

**Purpose:** System top-level; instantiate all CS1–CS11 + interconnect

**What It Does:**
- **Module Hierarchy:**
  ```
  top_cubesat_mvp
  ├─ clk_manager
  ├─ reset_controller
  ├─ system_monitor
  ├─ power_monitor
  ├─ resource_arbiter
  ├─ spi_imu_wrapper        (CS1)
  ├─ i2c_mag_wrapper        (CS2)
  ├─ sun_sensor_wrapper     (CS3)
  ├─ quat_propagator_wrapper(CS4)
  ├─ ekf_wrapper            (CS5)
  ├─ pd_control_wrapper     (CS6)
  ├─ actuator_wrapper       (CS7)
  ├─ adcs_fsm_wrapper       (CS8)
  ├─ orbit_propagator_wrapper(CS9)
  ├─ laser_fsm_wrapper      (CS10)
  ├─ telemetry_wrapper      (CS11)
  └─ [Signal routing, AXI4-Lite muxing, CDC bridges]
  ```
- **Signal Routing:**
  - CS1 → CS4, CS5, CS6, CS8
  - CS5 → CS6, CS8
  - CS8 → CS7, CS10
- **AXI4-Lite Master:** Exposes control/configuration registers for ground station (Kp, Kd gains, TLE, mode commands)

**Why Created:**
- **Single Integration Point:** All 12 subsystems in one module; makes hierarchy clear
- **Testability:** Can instantiate `top_cubesat_mvp` for full-system simulation
- **Maintainability:** Signal routing explicit and not scattered across 12 files

**Resource Estimate:**

| Resource | Count |
|----------|-------|
| LUTs | ~100 |
| DSP48 | 0 |
| BRAM | 0 |

---

## Summary Table: All 63 New Modules

| CS ID | Module Name | Purpose | Reason Created | LOC Est. |
|-------|-------------|---------|----------------|----------|
| **CS1** | `imu_controller` | SPI sequencer | Abstraction of SPI protocol; 20 µs timing constraint | 200 |
| | `imu_data_handler` | Calibration | Convert ADC → Q15; removable bias offsets | 150 |
| | `spi_imu_wrapper` | Integration | CDC synchronization; uniform interface | 80 |
| **CS2** | `i2c_mag_controller` | I2C sequencer | Protocol abstraction; NACK detection | 180 |
| | `mag_fault_detector` | Watchdog | Real-time health monitoring; age tracking | 120 |
| | `i2c_mag_wrapper` | Integration | CDC + interface consistency | 70 |
| **CS3** | `sun_sensor_adc` | 4-channel sequencer | Multi-channel muxing; setup/hold timing | 160 |
| | `adc_multiplexer` | Timing controller | Crosstalk prevention; channel settling | 100 |
| | `sun_presence_detector` | Threshold logic | Eclipse detection; rolling average | 110 |
| **CS4** | `quat_propagator` | Kinematics | Fast attitude extrapolation; gyro-only backup | 250 |
| | `quaternion_math` | Algebra | Reusable quaternion ops (product, conjugate, inverse) | 180 |
| | `norm_checker` | Validation | Numerical stability; divergence detection | 100 |
| **CS5** | `ekf_core` | Orchestrator | Predict/update cycle manager; divergence watchdog | 300 |
| | `ekf_predict` | Prediction step | State + covariance advance | 220 |
| | `ekf_update` | Measurement update | Innovation + gain application | 200 |
| | `ekf_joseph_update` | Covariance update | Numerical stability (Joseph form) | 250 |
| | `ekf_measurement_model` | Sensor prediction | Quaternion-based rotations | 180 |
| | `ekf_covariance` | Matrix storage | BRAM-based dual-port access | 120 |
| **CS6** | `pd_controller_quaternion` | Control law | Quaternion-based attitude feedback | 200 |
| | `torque_saturation` | Limit logic | Physical actuator hard limits ±10 mNm | 90 |
| | `control_law_engine` | Integration | Unified control law interface | 60 |
| **CS7** | `rw_spi_driver` | RW commands | Format torque → RPM commands (SPI) | 180 |
| | `magnetorquer_pwm` | MTQ coils | 3-axis PWM @ 10 kHz | 120 |
| | `actuator_command_arbiter` | Routing | RW vs MTQ priority logic | 100 |
| | `fault_status_monitor` | Health | RW fault polling; MTQ current monitoring | 110 |
| **CS8** | `adcs_fsm_controller` | FSM orchestrator | 5-state mode management | 250 |
| | `adcs_state_machine` | State logic | Guard conditions + transitions | 200 |
| | `fault_detector` | Multi-fault aggregation | Timeout + range + algorithm faults | 160 |
| | `adcs_health_monitor` | Health scoring | Aggregate health metric | 100 |
| | `fault_logger` | FIFO logging | Fault history for post-mission analysis | 120 |
| | `bram_circular_buffer` | Dual-port BRAM | 256×96B telemetry log | 140 |
| **CS9** | `tle_parser` | TLE parsing | Validation + element extraction | 200 |
| | `sgp4_propagator_core` | Orbit engine | SGP4-Lite with J2 perturbations | 500 |
| | `lvlh_converter` | Frame transform | ECI → LVLH (ISL, ground tracking) | 180 |
| | `ground_track_calculator` | Geodetic coords | ECI → lat/lon/altitude (WGS84) | 200 |
| | `contact_window_predictor` | AOS/LOS prediction | Elevation angle search for ground contacts | 220 |
| | `orbit_health_monitor` | Validity checks | Overflow + TLE age + timeout | 110 |
| | `orbit_state_manager` | MET + epoch | Mission elapsed time; time management | 130 |
| **CS10** | `raster_scan_engine` | SEARCH pattern | Boustrophedon ±10°az × ±5°el | 180 |
| | `spiral_refinement` | ACQUIRE pattern | Spiral convergence; peak-hold | 160 |
| | `peak_hold_detector` | Peak tracking | Signal strength peak detection | 100 |
| | `signal_monitor` | Rolling average | 16-sample low-pass filter | 90 |
| | `gimbal_controller` | 2-axis gimbal | STEP/DIR generation; trapezoidal accel | 200 |
| | `laser_modulator` | ISL + PWM | Modulator enable; ISL data mux | 110 |
| | `laser_fault_handler` | Fault detection | Signal loss + gimbal stall | 100 |
| | `laser_fsm_wrapper` | FSM orchestrator | 8-state machine (IDLE→SEARCH→ACQUIRE→TRACK→COMM→HOLD→FAULT→SAFE) | 350 |
| | PID instances ×2 | Tracking control | Azimuth & elevation tracking control | 2×100 |
| **CS11** | `tlm_frame_builder` | Frame assembly | CCSDS packet format (SYNC+APID+CRC) | 180 |
| | `tlm_arbiter` | Superframe scheduler | 1s schedule; 4-slot multiplexer | 160 |
| | `tlm_encoder_adcs` | ADCS packet | Quaternion + rates + mode + torques | 120 |
| | `tlm_encoder_orbit` | Orbit packet | ECI pos/vel + geodetic coords | 130 |
| | `tlm_encoder_laser` | Laser packet | FSM state + pointing error + signal strength | 100 |
| | `tlm_encoder_hk` | Housekeeping packet | Uptime + temp + voltage + health | 110 |
| | `command_decoder` | RX parsing | UART frame parsing + CRC | 140 |
| | `uart_tx_scheduler` | UART rate control | Baud-rate FSM + inter-frame gap | 100 |
| | `command_dispatcher` | Command routing | Route to target subsystem via AXI4-Lite | 120 |
| **CS12** | `clk_manager` | Clock tree | ce_1hz, ce_100hz, ce_1khz generation | 120 |
| | `reset_controller` | Reset logic | POR sequencing; soft/warm resets | 100 |
| | `system_monitor` | XADC interface | Die temp, VCCINT, VCCO monitoring | 90 |
| | `power_monitor` | Power tracking | Per-subsystem power estimates | 110 |
| | `resource_arbiter` | BRAM/DSP arb. | Prevent resource double-allocation | 100 |
| | `top_cubesat_mvp` | System top | All 12 subsystems + signal routing | 400 |
| | TBD ×2 | AXI4-Lite controller etc. | Extended integration support | 2×80 |
| | | | **TOTAL NEW MODULES: 63** | **~12,000 LOC** |

---

## Conclusion

Each of the **63 new RTL modules** was created for a specific reason:

1. **Abstraction** — Isolate protocol complexity (SPI, I2C, PWM, CRC) from application logic
2. **Separation of Concerns** — Break complex algorithms (EKF, SGP4) into testable, reusable pieces
3. **Reusability** — Quaternion math, CORDIC, synchronizers used across multiple subsystems
4. **Real-Time Constraints** — Meet strict timing budgets (20 µs for SPI, 1 ms for control, 10 ms for EKF)
5. **Fault Tolerance** — Health monitoring, divergence detection, timeout watchdogs
6. **Scalability** — Parameterizable modules (frequency, thresholds, vector sizes) adapt to different missions
7. **Testability** — Modular decomposition allows unit testing before integration
8. **Performance** — Dedicated DSP48 blocks and BRAM for computation-heavy operations

The architecture balances **complexity management** with **resource efficiency** — achieving full ADCS + orbit + laser autonomy in an FPGA with <8 KB BRAM and <50 DSP48 slices.
