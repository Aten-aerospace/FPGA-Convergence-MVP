# CS12 — System Integration & Clock Distribution

> Top-level FPGA integration layer instantiating all CS1–CS11 wrappers together with clock management, reset synchronisation, power monitoring, and system health aggregation into a single synthesisable image on the Artix-7.

---

## Overview

| Attribute | Value |
|---|---|
| Top module | `top_cubesat_mvp` |
| Target device | Xilinx Artix-7 XC7A100T |
| Primary clock input | `clk_100mhz` (100 MHz, external pin) |
| `sys_clk` | Derived by `clk_manager` (50 MHz or passthrough, parameterisable) |
| CE outputs | 1 Hz, 100 Hz, 1 kHz — distributed to all subsystems |
| Reset | 2-FF synchroniser on `rst_ext_n`; active-low `rst_n` broadcast |
| BRAM | 256 B (infrastructure only) |
| DSP48 | 0 |

---

## File Structure

| File | Purpose |
|---|---|
| `top_cubesat_mvp.sv` | Top-level — instantiates CS1–CS11 + infrastructure; owns signal routing |
| `clk_manager.sv` | CE strobe generation (1 Hz, 100 Hz, 1 kHz) from `clk_100mhz` |
| `reset_controller.sv` | POR + external reset debounce; generates synchronised `rst_n` |
| `system_monitor.sv` | Aggregates subsystem health outputs; drives `adcs_fault` / `tlm_valid` top ports |
| `power_monitor.sv` | XADC wrapper: FPGA temperature, VCCINT, VCCO reading for HK telemetry |
| `resource_arbiter.sv` | Reserved for future inter-subsystem bus arbitration |
| `tb_top_cubesat_mvp.sv` | Full-system directed testbench (30 ms simulated time) |

---

## Module Interface (top-level FPGA boundary)

```systemverilog
module top_cubesat_mvp #(
    parameter int CLK_HZ  = 100_000_000,
    parameter int SPI_HZ  = 8_000_000,
    parameter int I2C_HZ  = 400_000
)(
    input  logic        clk_100mhz,
    input  logic        rst_ext_n,       // external active-low reset

    // IMU SPI (CS1)
    output logic        imu_spi_sclk,
    output logic        imu_spi_mosi,
    input  logic        imu_spi_miso,
    output logic        imu_spi_cs_n,

    // Magnetometer I2C (CS2)
    inout  wire         mag_i2c_sda,
    output logic        mag_i2c_scl,

    // Sun sensor SPI (CS3)
    output logic        sun_spi_sclk,
    output logic        sun_spi_mosi,
    input  logic        sun_spi_miso,
    output logic        sun_spi_cs_n,

    // Reaction-wheel PWM + enable (CS7)
    output logic [2:0]  pwm_rw,
    output logic [2:0]  rw_enable,

    // Magnetorquer PWM + direction (CS7)
    output logic [2:0]  pwm_mtq,
    output logic [2:0]  dir_mtq,
    output logic [2:0]  mtq_enable,

    // Gimbal step/direction (CS10)
    output logic [1:0]  gimbal_step,
    output logic [1:0]  gimbal_dir,

    // Laser modulator (CS10)
    output logic        laser_mod_en,

    // Telemetry UART TX (CS11)
    output logic        tlm_uart_tx,

    // Status / debug
    output logic [2:0]  adcs_mode,
    output logic        adcs_fault,
    output logic        actuator_fault,
    output logic        orb_valid,
    output logic        pointing_locked,
    output logic        tlm_valid
);
```

---

## Signal Routing

| Route | Purpose |
|---|---|
| CS1 → CS4/CS5/CS8 | IMU data + `imu_data_valid` heartbeat |
| CS2 → CS5/CS8 | `mag_data[0:2]` + `mag_valid` heartbeat |
| CS3 → CS5 (context) | `sun_valid`, `sun_present` |
| CS5 → CS6 | `q_est` as `q_err` input (via subtraction placeholder) |
| CS5 → CS8 | `ekf_valid`, `q_err_mag` |
| CS1 → CS6 | `gyro_x/y/z` → `omega` for PD-D term |
| CS6 → CS7 | `torque_cmd[0:2]`, `ctrl_valid` |
| CS8 → CS7 | `safe_mode` (combinational blanking) |
| CS8 → CS10 | `laser_enable` (FINE_POINT mode gate) |
| CS9 → CS11 | `orbit_tlm` byte array (packed ECI pos/vel) |
| CS10 → CS11 | `laser_tlm` byte array (state + pointing + fault) |
| CS5 → CS11 | `adcs_tlm` byte array (packed q_est) |
| `system_monitor` → top ports | `adcs_fault`, `tlm_valid`, `orb_valid`, `pointing_locked` |

---

## Functionality

1. **Reset path** — `reset_controller` debounces `rst_ext_n` and feeds a synchronised `rst_n` to all subsystems.
2. **Clock tree** — `clk_manager` divides `clk_100mhz` to produce CE strobes used by all subsystems. No PLL in MVP; `sys_clk` = `clk_100mhz` or divided output.
3. **Subsystem instantiation** — `top_cubesat_mvp` directly instantiates `spi_imu_wrapper`, `i2c_mag_wrapper`, …, `telemetry_wrapper` and connects them via named signals following the routing table above.
4. **Extended port tie-off** — CS9 extended inputs (`tle_line1/2`, `met_*`, `gnd_*`, `sat2/3_*`) are tied to zero via intermediate `cs9_*` signals in MVP; CS11 HK payload is 18 bytes (`hk_tlm[0:17]`).
5. **System monitor** — `system_monitor` ORs subsystem fault flags and drives the top-level `adcs_fault`, `orb_valid`, etc. status outputs.
6. **Power monitor** — `power_monitor` reads FPGA XADC channels; temperature and supply voltages are packed into the HK telemetry byte array.

---

## Simulation Instructions

```bash
# From CubeSat/ directory
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
  [... all remaining subsystem files] \
  sqrt.sv fp_divider.sv cordic.sv uart_controller.sv spi_master.sv \
  crc_calc.sv synchronizer.sv i2c_master.sv lpf.sv pid_controller.sv \
  pwm_gen.sv stepper_driver.sv

vvp sim_top          # ~3 minutes for 30 ms simulated time
```

> See `CubeSat/README.md` for the full one-line compile command listing all source files.

```tcl
# QuestaSim
vlog -sv CS12_SYSTEM_INTEGRATION/tb_top_cubesat_mvp.sv \
         CS12_SYSTEM_INTEGRATION/*.sv [subsystem files] [shared files]
vsim -t 1ps tb_top_cubesat_mvp -do "run 30ms; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed system-level (non-UVM) |
| Clock | 100 MHz `clk_100mhz` |
| Simulated time | 30 ms (covers 3 ADCS cycles, 1 telemetry superframe) |
| Stimulus | Clock + reset; sensor SPI/I2C models for CS1/CS2/CS3; `laser_enable` sequencing |
| Checking | `adcs_mode` progression, `tlm_valid` assertion, `pointing_locked` after FINE_POINT, no fault flags on clean run |
| Coverage | Reset startup, ADCS boot sequence, telemetry frame output on UART |

---

## Expected Behaviour

```
rst_ext_n          _____|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾
ce_100hz           regular 100 Hz ticks (10 ms period)
adcs_mode[2:0]:    BOOT → DETUMBLE → COARSE_POINT (after ~10 ms)
tlm_uart_tx:       UART frames visible at 115200 bps after first ce_1hz
orb_valid:         ‾‾‾ after CS9 receives TLE and completes first propagation
adcs_fault:        ___ (low on nominal startup)
```

---

## Limitations

- Several CS9 extended interfaces (`tle_line1/2`, ground-station, inter-satellite) are tied off in MVP.
- `sys_clk = clk_100mhz` in MVP (no PLL); production build should add MMCM for 50 MHz system clock.
- No AXI4-Lite arbitration fabric; each subsystem's register interface is connected point-to-point.
- HK packet populated from `power_monitor` stub values until XADC is wired on target hardware.

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] 30 ms system testbench completes without `$error`
- [x] `adcs_mode` progresses through BOOT → DETUMBLE
- [x] UART frame bytes visible on `tlm_uart_tx`
- [x] `rst_n` synchroniser verified (no metastability false-positive)
- [ ] Full synthesis in Vivado — timing closure at 100 MHz
- [ ] Post-implementation timing simulation
- [ ] Resource utilisation report (LUT / FF / BRAM / DSP48)
- [ ] Bitstream loaded and verified on Nexys A7 hardware
