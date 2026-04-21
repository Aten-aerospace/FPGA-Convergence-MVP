# CS6 — PD Attitude Control Law

## 1. Module Title & ID

**Module:** CS6 — PD Attitude Control Law
**Subsystem ID:** CS6
**Requirement:** CS-ADCS-007

---

## 2. Overview

CS6 implements a Proportional-Derivative (PD) attitude control law that maps the quaternion attitude error and angular rate to 3-axis reaction-wheel torque commands. It operates at 1 kHz via `ce_1khz`, applies independent per-axis saturation to ± 10 mNm, and exposes an AXI4-Lite register interface for runtime Kp/Kd gain updates. The 3-cycle pipeline (PD law + saturation) delivers torque commands with deterministic latency.

**Target Platform:** Xilinx Artix-7 XC7A35T

---

## 3. Criticality

**CRITICAL** — CS-ADCS-007. CS6 is on the critical control path. Any failure to output torque commands within 1 ms causes the reaction wheels to lose tracking authority and triggers CS8 FINE_POINT → COARSE_POINT regression.

---

## 4. Key Functionality

- Implements control law: `τ[i] = −Kp × q_err[i] − Kd × ω[i]` for axes {x, y, z}.
- Accepts full 4-element error quaternion `q_err[0:3]`; extracts vector part `q_err[1:3]` internally.
- 2-cycle `pd_law` pipeline: Stage 1 multiply (Kp×q_err_vec, Kd×ω), Stage 2 accumulate + negate.
- `torque_saturation` clamps each axis to ± `SAT_LIMIT` (16'sh3FFF ≈ ± 10 mNm in Q15); 1-cycle latency.
- Total pipeline: 3 clock cycles from `ce_1khz && meas_valid` to `ctrl_valid`.
- `saturation_flag` asserts when any axis is clamped (OR of `sat_flag[2:0]`).
- `sat_count` counts saturation events since last clear; write-to-clear via AXI4-Lite offset 0x8.
- AXI4-Lite slave: Kp/Kd gains updatable in-orbit within 1 control cycle; write priority over `axi_gain_write` strobe.

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `clk` | input | 1 | `sys_clk` | 100 MHz system clock |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `ce_1khz` | input | 1 | `sys_clk` | 1 kHz clock-enable from CS12 |
| `q_err[0:3]` | input | 4 × 16 | `sys_clk` | Error quaternion [w,x,y,z] signed Q15 from CS5 |
| `omega[0:2]` | input | 3 × 16 | `sys_clk` | Angular rate [x,y,z] signed Q15 rad/s from CS1 |
| `meas_valid` | input | 1 | `sys_clk` | Inputs valid handshake |
| `Kp_coeff` | input | 16 | `sys_clk` | Proportional gain (Q15); used with `axi_gain_write` |
| `Kd_coeff` | input | 16 | `sys_clk` | Derivative gain (Q15); used with `axi_gain_write` |
| `axi_gain_write` | input | 1 | `sys_clk` | One-cycle strobe to latch Kp_coeff/Kd_coeff (lower priority than AXI4-Lite) |
| `axi_awaddr[3:0]` | input | 4 | `sys_clk` | AXI4-Lite write address |
| `axi_awvalid` | input | 1 | `sys_clk` | AXI4-Lite write address valid |
| `axi_wdata[15:0]` | input | 16 | `sys_clk` | AXI4-Lite write data |
| `axi_wvalid` | input | 1 | `sys_clk` | AXI4-Lite write data valid |
| `axi_bready` | input | 1 | `sys_clk` | AXI4-Lite write response ready |
| `axi_araddr[3:0]` | input | 4 | `sys_clk` | AXI4-Lite read address |
| `axi_arvalid` | input | 1 | `sys_clk` | AXI4-Lite read address valid |
| `axi_rready` | input | 1 | `sys_clk` | AXI4-Lite read data ready |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `torque_cmd[0:2]` | output | 3 × 16 | `sys_clk` | Torque commands [x,y,z] signed Q15 (after saturation) |
| `ctrl_valid` | output | 1 | `sys_clk` | One-cycle strobe — `torque_cmd` updated |
| `saturation_flag` | output | 1 | `sys_clk` | Any axis clamped this cycle |
| `sat_count` | output | 16 | `sys_clk` | Saturation event counter (write-to-clear) |
| `axi_awready` | output | 1 | `sys_clk` | AXI4-Lite write address ready |
| `axi_wready` | output | 1 | `sys_clk` | AXI4-Lite write data ready |
| `axi_bresp[1:0]` | output | 2 | `sys_clk` | AXI4-Lite write response (00=OK, 10=SLVERR) |
| `axi_bvalid` | output | 1 | `sys_clk` | AXI4-Lite write response valid |
| `axi_arready` | output | 1 | `sys_clk` | AXI4-Lite read address ready |
| `axi_rdata[15:0]` | output | 16 | `sys_clk` | AXI4-Lite read data |
| `axi_rresp[1:0]` | output | 2 | `sys_clk` | AXI4-Lite read response |
| `axi_rvalid` | output | 1 | `sys_clk` | AXI4-Lite read data valid |

---

## 7. Architecture

```
q_err[0:3]  omega[0:2]  ce_1khz
      │           │          │
      │ extract   │          │
      │ q_err[1:3]│          │
      ▼           ▼          │
┌──────────────────────────────┐ pd_torque[0:2]  ┌──────────────────────────┐
│  pd_law                      │─────────────────▶│  torque_saturation       │
│  (Stage 1: Kp×q_err + Kd×ω  │  pd_valid        │  SAT_LIMIT=16'sh3FFF     │
│   Stage 2: negate & sum)     │                  │  ENABLE_SAT_COUNT=1      │
│  gains: kp_reg, kd_reg       │                  └──────────────────────────┘
└──────────────────────────────┘                          │
        ▲                                                  │ torque_cmd, sat_flag[2:0]
        │                                                  │ ctrl_valid, sat_count
┌───────────────────────────────────┐
│  AXI4-Lite gain register file     │
│  0x0: KP_REG (RW, Q15, def 0x0CCD│
│  0x4: KD_REG (RW, Q15, def 0x0666│
│  0x8: SAT_CNT (RW, write-to-clear)│
│  0xC: STATUS (RO, bit[0]=sat_flag)│
└───────────────────────────────────┘
```

**Reused Helper IPs (from `CubeSat/`):**
- `pid_controller.sv` — PID template; reused for PD subset

**CS6-specific sub-modules:**
- `pd_law.sv` — PD control law core (2-cycle pipeline)
- `torque_saturation.sv` — Per-axis ± 10 mNm saturation + event counter
- `control_law_engine.sv` — Integrates pd_law + saturation with configurable Kp/Kd

---

## 8. Data Formats

| Signal | Format | Notes |
|---|---|---|
| `q_err[0:3]`, `omega[0:2]` | Q15 signed (16-bit) | 1.0 = 32767; q_err is full quaternion |
| `torque_cmd[0:2]` | Q15 signed (16-bit) | ± 10 mNm mapped to ± 16383 (SAT_LIMIT = 0x3FFF) |
| `Kp_coeff` (default) | Q15 = 0x0CCD | `round(0.1 × 32768) = 3277` |
| `Kd_coeff` (default) | Q15 = 0x0666 | `round(0.05 × 32768) = 1638` |
| `sat_count` | 16-bit unsigned | Counts saturation events; wraps at 65535 |

---

## 9. Register Interface

**AXI4-Lite Slave Register Map (4-byte aligned, word-addressed):**

| Offset | Name | Access | Reset Value | Description |
|---|---|---|---|---|
| `0x0` | `KP_REG` | RW | `0x0CCD` | Proportional gain (Q15; `0.1 × 32768`) |
| `0x4` | `KD_REG` | RW | `0x0666` | Derivative gain (Q15; `0.05 × 32768`) |
| `0x8` | `SAT_CNT` | RW | `0x0000` | Saturation event counter; any write clears it |
| `0xC` | `STATUS` | RO | `0x0000` | Bit[0] = `saturation_flag` (current cycle) |

Write to address `0xC` returns `SLVERR` (read-only register).

---

## 10. File Structure

```
CubeSat/CS6_CONTROL/
├── pd_control_wrapper.sv    ← Top-level wrapper + AXI4-Lite slave; CS12 integration point
├── pd_law.sv                ← PD law core: 2-cycle multiply-accumulate pipeline
├── control_law_engine.sv    ← PD + saturation integrator with Kp/Kd as input ports
├── torque_saturation.sv     ← Per-axis ± 10 mNm clamp + saturation event counter
├── tb_pd_control_wrapper.sv ← Testbench: zero-error, non-zero-error, saturation
└── README.md                ← This file

CubeSat/ (shared helper IPs used by CS6):
└── pid_controller.sv        ← PID template (PD subset reused)
```

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `ce_1khz` | CS6 ← CS12 | `clk_manager` (CS12) | 1 kHz control trigger |
| `q_err[0:3]` | CS6 ← CS5 | `ekf_wrapper` (CS5) | Attitude error from EKF |
| `omega[0:2]` | CS6 ← CS1 | `spi_imu_wrapper` (CS1) | Angular rate (D-term) |
| `torque_cmd[0:2]` | CS6 → CS7 | `actuator_wrapper` (CS7) | Torque commands to actuators |
| `ctrl_valid` | CS6 → CS7 | `actuator_wrapper` (CS7) | Command valid handshake |
| `saturation_flag` | CS6 → CS8 | `adcs_fsm_wrapper` (CS8) | Saturation health flag |
| AXI4-Lite | CS6 ← CS12 | `top_cubesat_mvp` (CS12) | Gain register access |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- Total pipeline: 3 cycles (70 ns @ 100 MHz) << 1 ms execution budget.
- Gain changes take effect within 1 control cycle (AXI4-Lite write latency ≤ 2 cycles).

**Resource:**
- `pd_law`: 2 multipliers per axis × 3 axes = 6 multiplications → ~2 DSP48E1 (TDM across axes).
- `torque_saturation`: comparator logic; 0 DSP48E1.
- BRAM: 64 B (register file and saturation counter storage).

**Optimization Opportunities:**
1. Time-multiplex a single DSP48E1 across all 3 axes to reduce from 2 to 1 DSP.
2. Add integral term (PID) for steady-state disturbance rejection; requires anti-windup for saturation.
3. Extend AXI4-Lite interface to allow runtime saturation limit adjustment.
4. Add feed-forward term from orbit model (CS9) to improve disturbance rejection.

**Timing:**
- Critical path: DSP48E1 multiply in `pd_law` Stage 1 → accumulate Stage 2. Register between stages.
- AXI4-Lite read/write paths are on `sys_clk`; no CDC required.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS6_CONTROL/tb_pd_control_wrapper.sv`

**Test Scenarios:**
- Apply zero `q_err` and zero `omega`; verify `torque_cmd` outputs are all zero.
- Apply known `q_err = [0, 0.1, 0, 0]` (Q15); verify `torque_cmd[0] ≈ −Kp × 3277`.
- Apply large `q_err` exceeding saturation; verify `torque_cmd` clamps at `SAT_LIMIT` and `saturation_flag` asserts.
- Write new Kp/Kd via AXI4-Lite; verify gain takes effect on next `ce_1khz`.
- Verify `sat_count` increments on saturation; verify it clears on AXI write to 0x8.
- Confirm 3-cycle pipeline latency: `ctrl_valid` arrives exactly 3 cycles after `ce_1khz && meas_valid`.

**Simulation Notes:**
- Compile with `iverilog -g2012` including `pid_controller.sv`.
- Timescale: 1 ns / 1 ps.

**Requirements Coverage:**
- CS-ADCS-007: τ = −Kp×q_err − Kd×ω @ 1 kHz, saturation ± 10 mNm, pointing error < 0.5°.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
