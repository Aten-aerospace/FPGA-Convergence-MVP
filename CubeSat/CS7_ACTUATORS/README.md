# CS7 — Reaction Wheel & Magnetorquer Actuators

> Converts 3-axis Q15 torque commands from CS6 into SPI-based reaction-wheel speed commands and 10 kHz PWM magnetorquer drive signals, with safe-mode combinational blanking within one control cycle.

---

## Overview

| Attribute | Value |
|---|---|
| Requirements | CS-ADCS-008, CS-ADCS-009 |
| Top module | `actuator_wrapper` |
| Clock domains | `sys_clk` (control logic) and `clk_100mhz` (MTQ PWM) |
| CE strobe | 1 kHz `ce_1khz` from CS12 |
| RW SPI rate | 10 MHz; transaction < 6 µs |
| MTQ PWM frequency | 10 kHz (via `pwm_gen` @ `clk_100mhz`) |
| MTQ resolution | ≥ 12-bit duty cycle |
| Safe-mode blanking | Combinational; < 1 control cycle |
| BRAM | 128 B |
| DSP48 | 1 |

---

## File Structure

| File | Purpose |
|---|---|
| `actuator_wrapper.sv` | Top-level — integrates all sub-modules; owns safe-mode mux |
| `actuator_command_arbiter.sv` | Maps torque_cmd[3] → per-axis RW speed commands |
| `rw_spi_driver.sv` | SPI motor driver command sequencer + 10 Hz fault polling |
| `rw_driver.sv` | PWM back-channel watchdog for RW gate enables |
| `mtq_driver.sv` | torque_cmd → 10 kHz PWM duty + direction + saturation flags |
| `fault_status_monitor.sv` | Watchdog timeout + SPI fault merge → `rw_fault[2:0]` |
| `tb_actuator_wrapper.sv` | Self-checking directed testbench |

Shared primitives used: `spi_master.sv`, `pwm_gen.sv`.

---

## Module Interface

```systemverilog
module actuator_wrapper #(
    parameter int CLK_HZ = 100_000_000
)(
    input  logic        sys_clk,
    input  logic        clk_100mhz,
    input  logic        rst_n,
    input  logic        ce_1khz,

    input  logic signed [15:0] torque_cmd [0:2],  // Q15 from CS6
    input  logic               cmd_valid,
    input  logic               safe_mode,          // from CS8; blanks all outputs

    // Reaction-wheel SPI bus
    output logic        rw_sclk,
    output logic [2:0]  rw_mosi,
    output logic [2:0]  rw_cs_n,
    input  logic [2:0]  rw_miso,

    // Magnetorquer PWM (10 kHz)
    output logic [2:0]  mtq_pwm,
    output logic [2:0]  dir_mtq,
    output logic [2:0]  mtq_enable,

    // Status
    output logic [2:0]  rw_fault,          // per-axis: watchdog | SPI fault
    output logic [2:0]  rw_enable,         // RW gate enables
    output logic [2:0]  mtq_sat_flag,      // MTQ axis saturated
    output logic        coupling_warning   // cross-axis coupling > 1 %
);
```

---

## Functionality

1. **Safe-mode** — `safe_mode` from CS8 is routed combinationally through `actuator_command_arbiter`; all RW commands and MTQ duty zeroed within the same control cycle.
2. **RW path** — `actuator_command_arbiter` scales `torque_cmd` to ±6000 RPM command. `rw_spi_driver` encodes commands into SPI frames and polls fault registers at 10 Hz.
3. **MTQ path** — `mtq_driver` is instantiated directly on `clk_100mhz`; maps `torque_cmd[i] → dipole_cmd`; drives `pwm_gen` at 10 kHz with `≥12-bit` duty resolution. Direction bit (`dir_mtq`) encodes polarity. `mtq_sat_flag` asserts if command exceeds hardware max.
4. **Fault monitor** — `fault_status_monitor` merges SPI-reported motor faults with a command watchdog timer into per-axis `rw_fault[2:0]`. Watchdog resets on each `cmd_valid`.
5. **Coupling warning** — computed by `fault_status_monitor` when cross-axis magnetic coupling > 1 %.

---

## Simulation Instructions

```bash
iverilog -g2012 -o sim_cs7 \
  CS7_ACTUATORS/tb_actuator_wrapper.sv \
  CS7_ACTUATORS/actuator_wrapper.sv \
  CS7_ACTUATORS/actuator_command_arbiter.sv \
  CS7_ACTUATORS/rw_spi_driver.sv \
  CS7_ACTUATORS/rw_driver.sv \
  CS7_ACTUATORS/mtq_driver.sv \
  CS7_ACTUATORS/fault_status_monitor.sv \
  spi_master.sv pwm_gen.sv

vvp sim_cs7
```

```tcl
vlog -sv CS7_ACTUATORS/tb_actuator_wrapper.sv CS7_ACTUATORS/*.sv spi_master.sv pwm_gen.sv
vsim -t 1ps tb_actuator_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 100 MHz `clk_100mhz` and `sys_clk` |
| Stimulus | Step torque commands; SPI slave model for RW fault injection |
| Checking | SPI frame decode matches expected speed command; MTQ PWM duty vs. torque_cmd |
| Coverage | Nominal torque, safe-mode blanking, RW SPI fault, MTQ saturation, watchdog timeout |

---

## Expected Behaviour

```
ce_1khz         __|‾|______________|‾|_______________
cmd_valid        ___|‾|______________|‾|_____________
safe_mode        ________|‾‾‾‾‾‾‾‾‾‾|_______________  (blanks all outputs)
rw_sclk          _________|‾‾‾‾‾‾‾‾‾‾‾‾‾|___________  (SPI transaction)
mtq_pwm[0]:      ~~~~~~~~~~~~~~~~~~~~~~~~  10 kHz carrier (duty ∝ torque_cmd)
mtq_pwm[0] during safe_mode: _______________ (zeroed)
rw_fault[i]:     _____________________ (latched if SPI reports fault)
```

---

## Limitations

- `magnetorquer_pwm.sv` exists but is not instantiated; `mtq_driver` directly interfaces `pwm_gen`.
- No per-actuator fault recovery policy; `rw_fault` latches until next successful poll.
- Watchdog timeout and RW scaling constants are compile-time parameters.
- Single SPI master shared across 3 RW axes (time-multiplexed via `rw_cs_n`).

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] SPI command frame matches expected speed encoding
- [x] MTQ PWM duty cycle proportional to torque_cmd
- [x] Safe-mode blanking within 1 cycle verified
- [x] RW fault injection and `rw_fault` latch tested
- [x] Watchdog timeout → `rw_fault` verified
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] Per-axis fault recovery state machine
- [ ] Synthesis timing closure in Vivado
