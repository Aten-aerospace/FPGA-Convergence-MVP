# CS6 — PD Attitude Control Law

> Computes three-axis torque commands from attitude and rate error using τ = −Kp·q_err − Kd·ω at 1 kHz, with ±10 mNm saturation clamping and AXI4-Lite runtime gain programming.

---

## Overview

| Attribute | Value |
|---|---|
| Requirement | CS-ADCS-007 |
| Top module | `pd_control_wrapper` |
| Clock domain | `sys_clk` (50 MHz) |
| CE strobe | 1 kHz `ce_1khz` from CS12 |
| Control law | τ = −Kp·q_err_vec − Kd·ω (Q15 gains, negation internal) |
| Saturation limit | `SAT_LIMIT = 0x3FFF` (≈ ±10 mNm) |
| Pipeline latency | 3 clock cycles |
| Default Kp | `0x0CCD` (≈ 0.1 in Q15) |
| Default Kd | `0x0666` (≈ 0.05 in Q15) |
| BRAM | 64 B |
| DSP48 | 2 |

---

## File Structure

| File | Purpose |
|---|---|
| `pd_control_wrapper.sv` | Top-level — gain registers, AXI4-Lite interface, pipeline staging |
| `pd_law.sv` | Stage 1: τ_raw = −Kp·q_err_vec − Kd·ω (2-cycle pipeline) |
| `control_law_engine.sv` | Stage 2: parameterised engine; takes Kp/Kd as input ports |
| `torque_saturation.sv` | Stage 3: clamp to ±SAT_LIMIT, count saturation events |
| `tb_pd_control_wrapper.sv` | Self-checking directed testbench |

---

## Module Interface

```systemverilog
module pd_control_wrapper (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_1khz,                        // 1 kHz clock enable

    input  logic signed [15:0] q_err  [0:3],            // error quaternion [w,x,y,z] Q15 from CS5
    input  logic signed [15:0] omega  [0:2],            // angular rate Q15 rad/s from CS1
    input  logic               meas_valid,

    // Gain loading — simple strobe
    input  logic signed [15:0] Kp_coeff,
    input  logic signed [15:0] Kd_coeff,
    input  logic               axi_gain_write,          // latch Kp/Kd on rising edge

    // Outputs
    output logic signed [15:0] torque_cmd [0:2],        // Q15, ±SAT_LIMIT
    output logic               saturation_flag,          // any axis clamped this cycle
    output logic        [15:0] sat_count,                // write-to-clear counter
    output logic               ctrl_valid                // 3-cycle delayed valid strobe
);
```

**AXI4-Lite register map** (optional; AXI writes take priority over strobe):

| Offset | Name | Access | Default | Description |
|---|---|---|---|---|
| `0x0` | KP_REG | RW | `0x0CCD` | Kp gain (Q15) |
| `0x4` | KD_REG | RW | `0x0666` | Kd gain (Q15) |
| `0x8` | SAT_CNT | RW | 0 | Saturation counter; write-any to clear |
| `0xC` | STATUS | RO | — | Bit[0] = `saturation_flag` |

---

## Functionality

1. **Gain load** — `Kp_coeff`/`Kd_coeff` are latched into `kp_reg`/`kd_reg` on `axi_gain_write`. AXI4-Lite writes override if both occur in the same cycle.
2. **PD law** (`pd_law`, 2 cycles) — extracts vector part `q_err[1:3]`; computes `τ_raw[i] = −kp_reg × q_err[i+1] − kd_reg × omega[i]`. Negation applied in Stage 1.
3. **Saturation** (`torque_saturation`, 1 cycle) — clamps each axis to `±0x3FFF`; asserts `sat_flag[2:0]`; increments `sat_count` (write-to-clear via `sat_count_clear`).
4. **Output** — `ctrl_valid` asserts 3 cycles after `meas_valid & ce_1khz`; `torque_cmd[0:2]` stable until next cycle.

---

## Simulation Instructions

```bash
iverilog -g2012 -o sim_cs6 \
  CS6_CONTROL/tb_pd_control_wrapper.sv \
  CS6_CONTROL/pd_control_wrapper.sv \
  CS6_CONTROL/pd_law.sv \
  CS6_CONTROL/control_law_engine.sv \
  CS6_CONTROL/torque_saturation.sv

vvp sim_cs6
```

```tcl
vlog -sv CS6_CONTROL/tb_pd_control_wrapper.sv CS6_CONTROL/*.sv
vsim -t 1ps tb_pd_control_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 50 MHz |
| CE strobe | Internally generated 1 kHz |
| Stimulus | Known `q_err` and `omega` vectors; expected `torque_cmd` computed offline |
| Checking | Output within ±1 LSB of expected; saturation counter increments correctly |
| Coverage | Nominal torque, saturation path (±FS input), gain update, write-to-clear counter, reset mid-cycle |

---

## Expected Behaviour

```
ce_1khz          __|‾|________|‾|________|‾|_____
meas_valid        ___|‾|________|‾|________|‾|___
[3-cycle pipeline]
ctrl_valid        __________|‾|__________|‾|_____
torque_cmd[0..2]: ==========|ΤORQUE Q15========
saturation_flag:  _________________________  (only when any axis > SAT_LIMIT)
sat_count:        increments when saturation_flag asserts
```

---

## Limitations

- No gain-scheduled or adaptive controller; single Kp/Kd pair for all ADCS modes.
- No anti-windup — integral term not present in PD law.
- Gain upload does not include range validation; out-of-range values accepted.
- Saturation limit is a compile-time parameter (cannot be changed in-flight).

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] Torque output matches expected Q15 value ±1 LSB
- [x] Saturation clamping and counter verified
- [x] AXI4-Lite gain write path tested
- [x] Write-to-clear sat_count verified
- [x] 3-cycle pipeline latency confirmed
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] Gain validation / range-check logic
- [ ] Synthesis DSP48 count confirmed (target: 2)
