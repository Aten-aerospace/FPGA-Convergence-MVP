# CS12 — System Integration & Clock Distribution

## 1. Module Title & ID

**Module:** CS12 — CubeSat ADCS System Integration (Top Level)
**Subsystem ID:** CS12
**Requirements:** System-level (all subsystems CS1–CS11)

---

## 2. Overview

CS12 is the top-level FPGA integration module for the CubeSat ADCS MVP. It instantiates all eleven subsystem wrappers (CS1–CS11), generates system-wide clock enables at four rates (1 Hz, 100 Hz, 1 kHz, and 100 MHz PWM domain), synchronises the external reset, routes all inter-subsystem signals, and exposes physical FPGA I/O pins. Additionally it instantiates `power_monitor` (per-subsystem power budget tracking) and `system_monitor` (XADC temperature/voltage). A 2-FF reset synchroniser ensures metastability-safe initialisation.

**Target Platform:** Xilinx Artix-7 XC7A35T (CLK_HZ = 100 MHz, SPI_HZ = 8 MHz, I2C_HZ = 400 kHz)

---

## 3. Criticality

**SYSTEM** — All subsystems. CS12 is the root integration point; any synthesis or implementation error here propagates to all subsystems. Timing closure and CDC compliance at this level are prerequisites for flight qualification.

---

## 4. Key Functionality

- **Clock Distribution (`clk_manager`):** Derives `ce_1hz`, `ce_100hz`, and `ce_1khz` from `clk_100mhz` reference; separate `sys_clk` alias used for control logic.
- **Reset Synchronisation:** External `rst_ext_n` is passed through a 2-FF synchroniser chain to produce `rst_n`; prevents metastability at power-on.
- **Subsystem Instantiation:** CS1–CS11 wrappers each have a dedicated instantiation block with all signals connected or tied off.
- **Signal Routing:** All cross-subsystem data paths (sensor → estimator → controller → actuator → telemetry) are wired at this level; no combinational loops.
- **Telemetry Packing:** HK telemetry (`hk_tlm[0:17]`) is assembled at this level from `power_monitor` and system health signals.
- **Power Monitor:** Tracks per-subsystem activity; exposes thermal alert (`thermal_alert`) and power budget flags.
- **System Monitor (`system_monitor`):** XADC-based 1 Hz temperature, VCCINT, VCCO readings; `thermal_alert` at > 110 °C.
- **I/O Assignment:** Maps all physical package pins (SPI, I2C, PWM, UART, gimbal, laser) to the appropriate sub-module ports.

---

## 5. Inputs

| Port | Direction | Width | Description |
|---|---|---|---|
| `clk_100mhz` | input | 1 | 100 MHz external oscillator (FPGA primary clock) |
| `rst_ext_n` | input | 1 | Active-low external reset (board push-button or supervisor) |
| `imu_spi_miso` | input | 1 | SPI MISO from IMU (CS1) |
| `mag_i2c_sda` | inout | 1 | I2C SDA to/from magnetometer (CS2; open-drain) |
| `sun_spi_miso` | input | 1 | SPI MISO from sun sensor ADC (CS3) |
| `rw_miso[2:0]` | input | 3 | SPI MISO from reaction wheel motor drivers (CS7) |

---

## 6. Outputs

| Port | Direction | Width | Description |
|---|---|---|---|
| `imu_spi_sclk` | output | 1 | SPI SCLK to IMU |
| `imu_spi_mosi` | output | 1 | SPI MOSI to IMU |
| `imu_spi_cs_n` | output | 1 | SPI CS to IMU (active low) |
| `mag_i2c_scl` | output | 1 | I2C SCL to magnetometer |
| `sun_spi_sclk` | output | 1 | SPI SCLK to sun sensor ADC |
| `sun_spi_mosi` | output | 1 | SPI MOSI to sun sensor ADC |
| `sun_spi_cs_n` | output | 1 | SPI CS to sun sensor ADC (active low) |
| `pwm_rw[2:0]` | output | 3 | Reaction wheel PWM signals |
| `rw_enable[2:0]` | output | 3 | Reaction wheel enable |
| `pwm_mtq[2:0]` | output | 3 | Magnetorquer PWM signals |
| `dir_mtq[2:0]` | output | 3 | Magnetorquer H-bridge direction |
| `mtq_enable[2:0]` | output | 3 | Magnetorquer enable |
| `gimbal_step[1:0]` | output | 2 | Laser gimbal step pulses (CS10) |
| `gimbal_dir[1:0]` | output | 2 | Laser gimbal direction (CS10) |
| `laser_mod_en` | output | 1 | Laser modulator enable (CS10) |
| `tlm_uart_tx` | output | 1 | Telemetry UART TX at 115,200 bps (CS11) |
| `adcs_mode[2:0]` | output | 3 | Current ADCS mode (CS8) |
| `adcs_fault` | output | 1 | ADCS fault indicator (CS8) |
| `actuator_fault` | output | 1 | Actuator fault indicator (CS7 `rw_fault` OR) |
| `orb_valid` | output | 1 | Orbit solution valid (CS9) |
| `pointing_locked` | output | 1 | Laser pointing locked (CS10) |
| `tlm_valid` | output | 1 | Telemetry frame sent (CS11) |

---

## 7. Architecture

```
clk_100mhz ──▶ clk_manager ──▶ ce_1hz, ce_100hz, ce_1khz
rst_ext_n  ──▶ 2-FF sync   ──▶ rst_n

top_cubesat_mvp
├── reset_controller         ← POR sequencing, warm/cold soft reset
├── clk_manager              ← CE strobe generator (1 Hz, 100 Hz, 1 kHz)
├── CS1: spi_imu_wrapper     ← IMU SPI + calibrated sensor data
├── CS2: i2c_mag_wrapper     ← Magnetometer I2C + calibrated mag data
├── CS3: sun_sensor_wrapper  ← Sun sensor SPI + sun vector
├── CS4: quat_propagator_wrapper ← Quaternion kinematic propagator
├── CS5: ekf_wrapper         ← 7-state EKF attitude estimator
├── CS6: pd_control_wrapper  ← PD attitude controller (1 kHz)
├── CS7: actuator_wrapper    ← RW SPI + MTQ PWM drivers
├── CS8: adcs_fsm_wrapper    ← ADCS mode FSM + health monitor
├── CS9: orbit_propagator_wrapper ← SGP4-lite orbit propagator
├── CS10: laser_fsm_wrapper  ← Laser pointing FSM + gimbal
├── CS11: telemetry_wrapper  ← CCSDS frame encoder + UART TX
├── power_monitor            ← Per-subsystem power tracking
└── system_monitor           ← XADC: temperature, VCCINT, VCCO
```

**Clock Enable Hierarchy:**

| Signal | Rate | Period | Consumers |
|---|---|---|---|
| `ce_1hz` | 1 Hz | 100 M cycles | CS9 (orbit), CS11 (superframe) |
| `ce_100hz` | 100 Hz | 1 M cycles | CS3 (sun trigger), CS4, CS5, CS8, CS10 |
| `ce_1khz` | 1 kHz | 100 k cycles | CS6 (PD control), CS7 (fault watchdog), CS11 (arbiter tick) |
| `clk_100mhz` | 100 MHz | 10 ns | CS7 MTQ PWM domain |

---

## 8. Data Formats

All inter-subsystem signals adhere to the formats defined in each sub-module. Key routing types at CS12 level:

| Signal Group | Format | Notes |
|---|---|---|
| Quaternion `q_est[0:3]` | Q15 signed (16-bit) | CS5 → CS4, CS6 |
| Angular rate `omega[0:2]` | Q15 signed (16-bit) | CS1 → CS4, CS5, CS6 |
| Torque `torque_cmd[0:2]` | Q15 signed (16-bit) | CS6 → CS7 |
| ECI position/velocity | Q15.16 (32-bit) | CS9 → CS11 |
| TLM packets | uint8 arrays | CS5/CS8/CS9/CS10 → CS11 |
| HK telemetry | `hk_tlm[0:17]` (18 × uint8) | CS12 → CS11 |

---

## 9. Register Interface

CS12 itself (the top-level) has **no AXI4-Lite slave register interface** in the current MVP implementation. Individual subsystems (CS6) expose AXI4-Lite ports that are routed to `top_cubesat_mvp` for external access; the full address map is TBD pending AXI4-Lite slave (`axi4_lite_slave.sv`) integration.

**Plan AXI4-Lite Address Space (Offset 0x0000–0x0BFF, 256 words per subsystem):**

| Range | Subsystem |
|---|---|
| 0x0000–0x00FF | CS1 (IMU bias calibration — TBD) |
| 0x0100–0x01FF | CS2 (MAG calibration — TBD) |
| 0x0200–0x02FF | CS3 (Sun threshold — TBD) |
| 0x0300–0x03FF | CS4 (reserved) |
| 0x0400–0x04FF | CS5 (Kalman gain K — TBD) |
| 0x0500–0x05FF | CS6 (KP_REG, KD_REG, SAT_CNT, STATUS) |
| 0x0600–0x06FF | CS7 (RW limits — TBD) |
| 0x0700–0x07FF | CS8 (FSM thresholds — TBD) |
| 0x0800–0x08FF | CS9 (TLE write port — TBD) |
| 0x0900–0x09FF | CS10 (FSM thresholds — TBD) |
| 0x0A00–0x0AFF | CS11 (baud rate — TBD) |
| 0x0B00–0x0BFF | CS12 system monitor |

**Parameters (synthesised):**

| Parameter | Default | Description |
|---|---|---|
| `CLK_HZ` | 100_000_000 | Reference clock frequency in Hz |
| `SPI_HZ` | 8_000_000 | IMU SPI clock frequency |
| `I2C_HZ` | 400_000 | Magnetometer I2C clock frequency |

---

## 10. File Structure

```
CubeSat/CS12_SYSTEM_INTEGRATION/
├── top_cubesat_mvp.sv         ← Top-level; instantiates all CS1–CS11 + monitors
├── clk_manager.sv             ← Clock enable generator (1 Hz, 100 Hz, 1 kHz)
├── reset_controller.sv        ← POR sequencing, warm/cold soft reset logic
├── system_monitor.sv          ← XADC controller: temperature, VCCINT, VCCO
├── power_monitor.sv           ← Per-subsystem power budget tracking + alarms
├── tb_top_cubesat_mvp.sv      ← Full-system integration testbench
└── README.md                  ← This file

CubeSat/CS12_SYSTEM_INTEGRATION/ (instantiated sub-modules, located in CubeSat/):
├── CubeSat/CS1_IMU_SPI/spi_imu_wrapper.sv
├── CubeSat/CS2_MAG_I2C/i2c_mag_wrapper.sv
├── CubeSat/CS3_SUN_ADC/sun_sensor_wrapper.sv
├── CubeSat/CS4_QUAT_PROP/quat_propagator_wrapper.sv
├── CubeSat/CS5_EKF/ekf_wrapper.sv
├── CubeSat/CS6_CONTROL/pd_control_wrapper.sv
├── CubeSat/CS7_ACTUATORS/actuator_wrapper.sv
├── CubeSat/CS8_ADCS_FSM/adcs_fsm_wrapper.sv
├── CubeSat/CS9_ORBIT/orbit_propagator_wrapper.sv
├── CubeSat/CS10_LASER/laser_fsm_wrapper.sv
└── CubeSat/CS11_TELEMETRY/telemetry_wrapper.sv

CubeSat/ (shared helper IPs compiled together):
  sqrt.sv, fp_divider.sv, cordic.sv, uart_controller.sv, spi_master.sv,
  crc_calc.sv, synchronizer.sv, i2c_master.sv, lpf.sv, pid_controller.sv,
  pwm_gen.sv, stepper_driver.sv, ...
```

---

## 11. Interconnections

All inter-subsystem signal routing is centralised in `top_cubesat_mvp.sv`. Key paths:

| Source | Signal | Destination | Purpose |
|---|---|---|---|
| CS1 | `gyro[0:2]` | CS4, CS6 | Angular rate feed to propagator and PD-D term |
| CS1 | `accel[0:2]`, `gyro[0:2]`, `imu_data_valid` | CS5 | EKF sensor inputs + heartbeat |
| CS2 | `mag_data[0:2]`, `mag_valid` | CS5 | EKF magnetometer measurement + heartbeat |
| CS3 | `sun_valid` | CS8 | Sun-presence health indicator |
| CS5 | `q_est[0:3]` | CS4, CS6 | Quaternion feedback to propagator and controller |
| CS5 | `ekf_valid` | CS4, CS6, CS8 | EKF heartbeat |
| CS6 | `torque_cmd[0:2]`, `ctrl_valid` | CS7 | Control torques to actuators |
| CS8 | `adcs_mode == SAFE\|FAULT` | CS7 `safe_mode` | Actuator blanking |
| CS8 | `adcs_mode == FINE_POINT` | CS10 `laser_enable` | Enable laser pointing |
| CS9 | `eci_pos/vel`, `met_counter` | CS11 | Orbit telemetry and timestamp |
| CS10 | `laser_state`, `pointing_locked`, `laser_fault` | CS11 | Laser telemetry |
| CS5, CS8 | `q_est` (packed), `adcs_mode`, `fault_flags` | CS11 `adcs_tlm` | ADCS telemetry |
| CS12 | `hk_tlm[0:17]` | CS11 | HK telemetry from power/system monitor |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- Control loop: CS6 @ 1 kHz (1 ms deadline); pipeline latency = 3 cycles.
- ADCS estimation: CS5 @ 100 Hz (10 ms deadline); 8 ms compute budget.
- Orbit propagation: CS9 @ 1 Hz (1 s window); 50 ms compute budget.

**Resource (Plan Totals):**
- BRAM: 7,392 B total (CS1–CS12); XC7A35T has 45 KB available (16 % utilisation).
- DSP48E1: 22 total (CS1–CS12); XC7A35T has 90 available (24 % utilisation).
- LUT/FF: TBD; estimated < 15,000 LUTs for full system.

**Optimization Opportunities:**
1. Implement full `axi4_lite_slave.sv` to enable runtime register access to all subsystems.
2. Replace 2-FF reset synchroniser with a POR sequence controller supporting cold/warm reset modes.
3. Add CDC monitoring assertions (`$setup`/`$hold` equivalents for cross-domain signals).
4. Use Xilinx MMCM/PLL primitive in `clk_manager` for reduced jitter on `ce_1hz/ce_100hz`.
5. Implement XADC thermal throttle in `system_monitor` to reduce clock rates at > 110 °C.

**Timing:**
- All subsystems share a single `sys_clk` (100 MHz); no CDC within the control data path.
- `mag_i2c_sda` is open-drain inout; `top_cubesat_mvp` passes it directly to CS2 with no additional buffering.
- MTQ PWM domain (`clk_100mhz`) is the same physical clock as `sys_clk`; safe to treat as same domain for logic.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS12_SYSTEM_INTEGRATION/tb_top_cubesat_mvp.sv`

**Test Scenarios:**
- Full-system power-on: verify CS8 starts in BOOT, clocks begin at correct rates.
- Apply simulated SPI IMU data; verify CS1 → CS5 → CS6 → CS7 data flow.
- Verify BOOT → DETUMBLE → COARSE_POINT → FINE_POINT FSM transitions under simulated conditions.
- Assert `adcs_mode == FINE_POINT`; verify CS10 `laser_enable` is asserted.
- Trigger safe mode via uplink command; verify CS7 blanks all actuator outputs within 1 ms.
- Verify CS11 transmits all 4 telemetry packet types in sequence within the 1-second superframe.
- Inject FAULT into CS8 (`fault_trigger`); verify `adcs_fault` pin goes high and `safe_mode` to CS7.
- Verify `rst_ext_n` assertion brings all subsystems to reset state cleanly.

**Simulation Notes:**
- Full system compilation: `iverilog -g2012` with all module files from CS1–CS12 and all shared helper IPs.
- Simulation runtime: approximately 3 minutes for a 30 ms simulated time window.
- Timescale: 1 ns / 1 ps.

**Requirements Coverage:**
- System-level timing: 1 kHz control, 100 Hz ADCS, 1 Hz orbit, 1-sec telemetry superframe.
- BRAM ≤ 8 KB (target), DSP48 ≤ 50, all timing closure @ 100 MHz.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
- Plan: `CubeSat_FPGA_RTL_Module_Plan.md` — Part 3 (Integration Checklist)
