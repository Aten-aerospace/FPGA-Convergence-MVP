# CubeSat FPGA Firmware

> 12-subsystem SystemVerilog FPGA firmware for a 3U CubeSat — covering full ADCS sensor-to-actuator pipeline, SGP4-lite orbit propagation, laser inter-satellite link (ISL) pointing, and CCSDS telemetry on a single Artix-7 image.

---

## Overview

| Attribute | Value |
|---|---|
| Target device | Xilinx Artix-7 XC7A100T |
| Primary clock | 100 MHz (`clk_100mhz`) |
| System clock | 50 MHz (`sys_clk`) derived by `clk_manager` |
| CE strobes | 1 Hz · 100 Hz · 1 kHz |
| Language | SystemVerilog IEEE 1800-2012 |
| Simulation | Icarus Verilog 12 (`iverilog -g2012`), QuestaSim / ModelSim |
| Synthesis | Vivado 2024.x |
| Total subsystems | 12 (CS1 – CS12) |
| New RTL files | 63 subsystem + 20 shared primitives |
| BRAM budget | ~7,392 B |
| DSP48 budget | ~22 |

---

## Subsystems

| ID | Folder | Top Module | Description | Clock | BRAM | DSP48 |
|---|---|---|---|---|---|---|
| CS1 | `CS1_IMU_SPI/` | `spi_imu_wrapper` | IMU (MPU-9250) 9-axis SPI read + calibration | clk_100mhz → sys_clk | 256 B | 0 |
| CS2 | `CS2_MAG_I2C/` | `i2c_mag_wrapper` | Magnetometer I2C 400 kHz read + fault detect | sys_clk | 128 B | 0 |
| CS3 | `CS3_SUN_ADC/` | `sun_sensor_wrapper` | 4-channel SPI ADC sun sensor + vector compute | clk_100mhz → sys_clk | 64 B | 0 |
| CS4 | `CS4_QUAT_PROP/` | `quat_propagator_wrapper` | Quaternion kinematic propagator (Hamilton product + normalise) | sys_clk | 96 B | 3 |
| CS5 | `CS5_EKF/` | `ekf_wrapper` | 7-state Extended Kalman Filter (q + gyro bias) | sys_clk | 512 B | 6 |
| CS6 | `CS6_CONTROL/` | `pd_control_wrapper` | PD attitude control law, torque saturation, AXI4-Lite gains | sys_clk | 64 B | 2 |
| CS7 | `CS7_ACTUATORS/` | `actuator_wrapper` | Reaction-wheel SPI driver + 3-axis MTQ 10 kHz PWM | sys_clk / clk_100mhz | 128 B | 1 |
| CS8 | `CS8_ADCS_FSM/` | `adcs_fsm_wrapper` | 6-state ADCS mode FSM, health monitor, BRAM circular log | sys_clk | 3,072 B | 0 |
| CS9 | `CS9_ORBIT/` | `orbit_propagator_wrapper` | SGP4-lite propagator, LVLH, ground-track, contact windows | sys_clk | 1,536 B | 8 |
| CS10 | `CS10_LASER/` | `laser_fsm_wrapper` | 8-state laser ISL pointing FSM, gimbal stepper, PD track | sys_clk | 512 B | 2 |
| CS11 | `CS11_TELEMETRY/` | `telemetry_wrapper` | CCSDS telemetry encoder, UART downlink/uplink, command dispatch | sys_clk | 768 B | 0 |
| CS12 | `CS12_SYSTEM_INTEGRATION/` | `top_cubesat_mvp` | Top-level integration, clock/reset distribution, signal routing | both | 256 B | 0 |

---

## Shared RTL Primitives

These 20 modules live directly in `CubeSat/` and are reused across subsystems.

| File | Purpose | Used by |
|---|---|---|
| `spi_master.sv` | SPI protocol engine | CS1, CS3, CS7 |
| `i2c_master.sv` | I2C 400 kHz Fast Mode | CS2 |
| `cordic.sv` | CORDIC trig / vector magnitude | CS4, CS5, CS9 |
| `sqrt.sv` | Fixed-point square root | CS4, CS5, CS9 |
| `fp_divider.sv` | Fixed-point division | CS4, CS5, CS9 |
| `pid_controller.sv` | Generic PID | CS6, CS10 |
| `pwm_gen.sv` | PWM generator | CS7, CS10 |
| `uart_controller.sv` | UART TX/RX | CS11 |
| `crc_calc.sv` | CRC-16/CCITT | CS11 |
| `synchronizer.sv` | 2-FF CDC synchroniser | CS1, CS2, CS8, CS12 |
| `tick_gen.sv` | CE strobe generator | CS1, CS2, CS12 |
| `lpf.sv` | IIR low-pass filter | CS10 |
| `async_fifo.sv` | Async FIFO | CS11 |
| `stepper_driver.sv` | Stepper motor pulse | CS10 |
| `clk_divider.sv` | Integer clock divider | CS12 |
| `mavlink_parser.sv` | MAVLink command parser | CS11 |
| `adc_interface.sv` | SPI ADC read | CS3 |
| `debouncer.sv` | Input debouncer | CS2, CS8 |
| `edge_detect.sv` | Rising/falling edge detect | CS1, CS3, CS8 |
| `kalman_1v.sv` | Scalar Kalman template | CS5 |

---

## Directory Structure

```
CubeSat/
├── README.md                    ← this file
├── CS1_IMU_SPI/
│   ├── imu_controller.sv
│   ├── imu_data_handler.sv
│   ├── spi_imu_wrapper.sv       ← CS1 top
│   ├── tb_spi_imu_wrapper.sv
│   └── README.md
├── CS2_MAG_I2C/
│   ├── i2c_mag_controller.sv
│   ├── mag_calibration.sv
│   ├── mag_fault_detector.sv
│   ├── i2c_mag_wrapper.sv       ← CS2 top
│   ├── tb_i2c_mag_wrapper.sv
│   └── README.md
├── CS3_SUN_ADC/
│   ├── adc_sequencer.sv
│   ├── spi_mux_controller.sv
│   ├── sun_vector_compute.sv
│   ├── sun_presence_detector.sv
│   ├── sun_sensor_wrapper.sv    ← CS3 top
│   ├── tb_sun_sensor_wrapper.sv
│   └── README.md
├── CS4_QUAT_PROP/
│   ├── quat_multiply.sv
│   ├── quat_normalize.sv
│   ├── norm_checker.sv
│   ├── quaternion_math.sv
│   ├── quat_propagator.sv
│   ├── quat_propagator_wrapper.sv  ← CS4 top
│   ├── tb_quat_propagator_wrapper.sv
│   └── README.md
├── CS5_EKF/
│   ├── ekf_core.sv
│   ├── ekf_predict.sv
│   ├── ekf_update.sv
│   ├── ekf_covariance.sv
│   ├── ekf_joseph_update.sv
│   ├── ekf_measurement_model.sv
│   ├── ekf_wrapper.sv           ← CS5 top
│   ├── tb_ekf_wrapper.sv
│   └── README.md
├── CS6_CONTROL/
│   ├── pd_law.sv
│   ├── control_law_engine.sv
│   ├── torque_saturation.sv
│   ├── pd_control_wrapper.sv    ← CS6 top
│   ├── tb_pd_control_wrapper.sv
│   └── README.md
├── CS7_ACTUATORS/
│   ├── actuator_command_arbiter.sv
│   ├── rw_spi_driver.sv
│   ├── rw_driver.sv
│   ├── mtq_driver.sv
│   ├── fault_status_monitor.sv
│   ├── actuator_wrapper.sv      ← CS7 top
│   ├── tb_actuator_wrapper.sv
│   └── README.md
├── CS8_ADCS_FSM/
│   ├── health_monitor.sv
│   ├── adcs_data_logger.sv
│   ├── fault_logger.sv
│   ├── bram_circular_buffer.sv
│   ├── adcs_fsm_wrapper.sv      ← CS8 top
│   ├── tb_adcs_fsm_wrapper.sv
│   └── README.md
├── CS9_ORBIT/
│   ├── sgp4_lite.sv
│   ├── lvlh_converter.sv
│   ├── orbit_state_manager.sv
│   ├── orbit_health_monitor.sv
│   ├── ground_track_calculator.sv
│   ├── contact_window_predictor.sv
│   ├── multi_satellite_tracker.sv
│   ├── orbit_propagator_wrapper.sv  ← CS9 top
│   ├── tb_orbit_propagator_wrapper.sv
│   └── README.md
├── CS10_LASER/
│   ├── raster_scan_engine.sv
│   ├── spiral_refinement.sv
│   ├── peak_hold_detector.sv
│   ├── signal_monitor.sv
│   ├── laser_modulator.sv
│   ├── laser_fault_handler.sv
│   ├── gimbal_controller.sv
│   ├── laser_fsm_wrapper.sv     ← CS10 top
│   ├── tb_laser_fsm_wrapper.sv
│   └── README.md
├── CS11_TELEMETRY/
│   ├── tlm_arbiter.sv
│   ├── ccsds_encoder.sv
│   ├── command_decoder.sv
│   ├── command_dispatcher.sv
│   ├── tle_parser.sv
│   ├── telemetry_wrapper.sv     ← CS11 top
│   ├── tb_telemetry_wrapper.sv
│   └── README.md
├── CS12_SYSTEM_INTEGRATION/
│   ├── top_cubesat_mvp.sv       ← system top
│   ├── clk_manager.sv
│   ├── reset_controller.sv
│   ├── system_monitor.sv
│   ├── power_monitor.sv
│   ├── resource_arbiter.sv
│   ├── tb_top_cubesat_mvp.sv
│   └── README.md
└── [shared primitives — see table above]
```

---

## Signal Flow

```
CS1 (IMU SPI) ──┬──▶ CS4 (Quat Prop) ─▶ CS5 (EKF)
                └──▶ CS5 (EKF) ◀── CS2 (Mag I2C)
                         │
                    CS3 (Sun ADC) ─▶ (CS5 context)
                         │
                    CS5 ─┬──▶ CS6 (PD Control) ─▶ CS7 (Actuators)
                         └──▶ CS8 (ADCS FSM) ──▶ CS7 (safe_mode)
                                   │             ▶ CS10 (laser_enable)
CS9 (Orbit) ──────────────────────▶│
CS10 (Laser FSM) ─────────────────▶└──▶ CS11 (Telemetry) ─▶ UART TX
```

---

## Simulation

### Full system (CS1–CS12)
```bash
iverilog -g2012 -o sim_top \
  CS12_SYSTEM_INTEGRATION/tb_top_cubesat_mvp.sv \
  CS12_SYSTEM_INTEGRATION/top_cubesat_mvp.sv \
  CS12_SYSTEM_INTEGRATION/clk_manager.sv \
  CS12_SYSTEM_INTEGRATION/reset_controller.sv \
  CS12_SYSTEM_INTEGRATION/system_monitor.sv \
  CS12_SYSTEM_INTEGRATION/power_monitor.sv \
  CS1_IMU_SPI/spi_imu_wrapper.sv CS1_IMU_SPI/imu_controller.sv CS1_IMU_SPI/imu_data_handler.sv \
  CS2_MAG_I2C/i2c_mag_wrapper.sv CS2_MAG_I2C/i2c_mag_controller.sv \
    CS2_MAG_I2C/mag_calibration.sv CS2_MAG_I2C/mag_fault_detector.sv \
  CS3_SUN_ADC/sun_sensor_wrapper.sv CS3_SUN_ADC/adc_sequencer.sv \
    CS3_SUN_ADC/spi_mux_controller.sv CS3_SUN_ADC/sun_vector_compute.sv \
    CS3_SUN_ADC/sun_presence_detector.sv \
  CS4_QUAT_PROP/quat_propagator_wrapper.sv CS4_QUAT_PROP/quat_multiply.sv \
    CS4_QUAT_PROP/quat_normalize.sv CS4_QUAT_PROP/norm_checker.sv \
  CS5_EKF/ekf_wrapper.sv CS5_EKF/ekf_core.sv CS5_EKF/ekf_predict.sv \
    CS5_EKF/ekf_update.sv CS5_EKF/ekf_covariance.sv \
    CS5_EKF/ekf_joseph_update.sv CS5_EKF/ekf_measurement_model.sv \
  CS6_CONTROL/pd_control_wrapper.sv CS6_CONTROL/pd_law.sv \
    CS6_CONTROL/control_law_engine.sv CS6_CONTROL/torque_saturation.sv \
  CS7_ACTUATORS/actuator_wrapper.sv CS7_ACTUATORS/actuator_command_arbiter.sv \
    CS7_ACTUATORS/rw_spi_driver.sv CS7_ACTUATORS/rw_driver.sv \
    CS7_ACTUATORS/mtq_driver.sv CS7_ACTUATORS/fault_status_monitor.sv \
  CS8_ADCS_FSM/adcs_fsm_wrapper.sv CS8_ADCS_FSM/health_monitor.sv \
    CS8_ADCS_FSM/adcs_data_logger.sv CS8_ADCS_FSM/fault_logger.sv \
    CS8_ADCS_FSM/bram_circular_buffer.sv \
  CS9_ORBIT/orbit_propagator_wrapper.sv CS9_ORBIT/sgp4_lite.sv \
    CS9_ORBIT/lvlh_converter.sv CS9_ORBIT/orbit_state_manager.sv \
    CS9_ORBIT/orbit_health_monitor.sv CS9_ORBIT/ground_track_calculator.sv \
    CS9_ORBIT/contact_window_predictor.sv CS9_ORBIT/multi_satellite_tracker.sv \
  CS10_LASER/laser_fsm_wrapper.sv CS10_LASER/raster_scan_engine.sv \
    CS10_LASER/spiral_refinement.sv CS10_LASER/peak_hold_detector.sv \
    CS10_LASER/signal_monitor.sv CS10_LASER/laser_modulator.sv \
    CS10_LASER/laser_fault_handler.sv CS10_LASER/gimbal_controller.sv \
  CS11_TELEMETRY/telemetry_wrapper.sv CS11_TELEMETRY/tlm_arbiter.sv \
    CS11_TELEMETRY/ccsds_encoder.sv CS11_TELEMETRY/command_decoder.sv \
    CS11_TELEMETRY/command_dispatcher.sv CS11_TELEMETRY/tle_parser.sv \
  sqrt.sv fp_divider.sv cordic.sv uart_controller.sv spi_master.sv \
  crc_calc.sv synchronizer.sv i2c_master.sv lpf.sv pid_controller.sv \
  pwm_gen.sv stepper_driver.sv

vvp sim_top          # ~3 min for 30 ms simulated time
```

### Per-subsystem (example: CS5 EKF)
```bash
iverilog -g2012 -o sim_cs5 \
  CS5_EKF/tb_ekf_wrapper.sv CS5_EKF/ekf_wrapper.sv CS5_EKF/ekf_core.sv \
  CS5_EKF/ekf_predict.sv CS5_EKF/ekf_update.sv CS5_EKF/ekf_covariance.sv \
  CS5_EKF/ekf_joseph_update.sv CS5_EKF/ekf_measurement_model.sv \
  sqrt.sv fp_divider.sv cordic.sv

vvp sim_cs5
```

### QuestaSim / ModelSim
```tcl
vlog -sv CS5_EKF/tb_ekf_wrapper.sv CS5_EKF/*.sv cordic.sv fp_divider.sv sqrt.sv
vsim -t 1ps tb_ekf_wrapper -do "run -all; quit"
```

---

## RTL Coding Conventions

| Rule | Detail |
|---|---|
| Sequential | `always_ff @(posedge clk or negedge rst_n)` |
| Combinatorial | `always_comb` only |
| FSM encoding | `typedef enum logic [N:0]` |
| Signal type | `logic` throughout (no `wire`/`reg`) |
| Reset | Active-low synchronous (`rst_n`) |
| Arrays | `logic [W-1:0] name [0:N-1]` |
| Timescale | `` `timescale 1ns/1ps `` |
| Initial blocks | Not used in synthesisable RTL |

---

## Design Notes

- **Fixed-point formats:** Q15 for quaternion/angular-rate; Q15.16 for orbit km/km/s; Q8.8 for gimbal degrees.
- **Clock crossing:** all CDC handled inside the subsystem wrapper using `synchronizer.sv` 2-FF cells. Multi-bit data uses captured-on-valid pattern.
- **Safe-mode path:** `CS8.adcs_fsm_wrapper` drives `safe_mode` to CS7 and `laser_enable` to CS10. Both are combinational blanking paths.
- **iverilog limitations:** localparam array literals, `break` in for-loops, whole-array assignments, and `16'(...)` casts are not supported — use function-based LUTs and element-wise loops.
