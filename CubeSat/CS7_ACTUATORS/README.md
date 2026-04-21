# CS7 — Reaction Wheel & Magnetorquer Actuator Drivers

## 1. Module Title & ID

**Module:** CS7 — Reaction Wheel & Magnetorquer Actuator Drivers
**Subsystem ID:** CS7
**Requirements:** CS-ADCS-008, CS-ADCS-009

---

## 2. Overview

CS7 provides the low-level drive interface for two families of CubeSat attitude actuators: three reaction wheels (SPI-commanded motor controllers) and three magnetorquers (3-axis PWM H-bridge). It receives signed Q15 torque commands from the PD controller (CS6), routes them to the appropriate actuator path via `actuator_command_arbiter`, and enforces safe-mode blanking within one control cycle when commanded by the ADCS FSM (CS8). SPI commands are sent to reaction wheel drivers at 1 kHz; MTQ PWM runs at 10 kHz.

**Target Platform:** Xilinx Artix-7 XC7A35T

---

## 3. Criticality

**CRITICAL** — CS-ADCS-008 (RW), CS-ADCS-009 (MTQ). Actuator output failures directly impact attitude control authority. Safe-mode blanking must occur within 1 control cycle (1 ms) to prevent runaway during fault conditions.

---

## 4. Key Functionality

- **Reaction Wheel (RW) SPI Path:** `actuator_command_arbiter` maps `torque_cmd[0:2]` → `rw_cmd[0:2]`. `rw_spi_driver` sends SPI motor commands @ 1 kHz (± 6000 RPM range; 1 LSB = 0.2 RPM); polls fault status @ 10 Hz.
- **Legacy RW PWM Path:** `rw_driver` generates bi-directional PWM back-channel for RW gate enables; `rw_enable[2:0]` output gated by `safe_mode`.
- **Magnetorquer (MTQ) PWM:** `mtq_driver` converts `torque_cmd[0:2]` (Q15) → 3× 10 kHz PWM + H-bridge direction + enable signals. Cross-axis coupling < 1 % guaranteed by independent per-channel PWM counters.
- **Safe-Mode Blanking:** All MTQ and RW outputs forced to 0 combinationally within 1 control cycle when `safe_mode` asserts.
- **Fault Monitoring:** `fault_status_monitor` merges SPI watchdog with per-axis SPI faults → `rw_fault[2:0]`.
- **MTQ Saturation:** `mtq_sat_flag[2:0]` asserts per-axis when dipole command exceeds maximum drive; `coupling_warning` asserts when cross-axis interaction exceeds 1 %.

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `sys_clk` | input | 1 | `sys_clk` | Control-path system clock (100 MHz) |
| `clk_100mhz` | input | 1 | `clk_100mhz` | PWM counter clock (100 MHz; for MTQ) |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `ce_1khz` | input | 1 | `sys_clk` | 1 kHz clock-enable from CS12 |
| `torque_cmd[0:2]` | input | 3 × 16 | `sys_clk` | Signed Q15 torque commands [x,y,z] from CS6 |
| `cmd_valid` | input | 1 | `sys_clk` | Torque command valid handshake |
| `safe_mode` | input | 1 | `sys_clk` | Safe-mode from CS8 (blanks all outputs) |
| `rw_miso[2:0]` | input | 3 | `sys_clk` | SPI MISO from reaction wheel motor drivers |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `rw_sclk` | output | 1 | `sys_clk` | Shared SPI SCLK to RW drivers |
| `rw_mosi[2:0]` | output | 3 | `sys_clk` | SPI MOSI per RW axis |
| `rw_cs_n[2:0]` | output | 3 | `sys_clk` | SPI CS_N per RW axis (active low) |
| `rw_enable[2:0]` | output | 3 | `sys_clk` | RW gate enable (gated by safe_mode) |
| `mtq_pwm[2:0]` | output | 3 | `clk_100mhz` | MTQ PWM at 10 kHz (gated by safe_mode) |
| `dir_mtq[2:0]` | output | 3 | `clk_100mhz` | MTQ H-bridge direction (gated by safe_mode) |
| `mtq_enable[2:0]` | output | 3 | `clk_100mhz` | MTQ H-bridge enable (gated by safe_mode) |
| `rw_fault[2:0]` | output | 3 | `sys_clk` | Per-axis RW fault (watchdog OR SPI fault) |
| `mtq_sat_flag[2:0]` | output | 3 | `clk_100mhz` | Per-axis MTQ saturation flag |
| `coupling_warning` | output | 1 | `clk_100mhz` | Cross-axis MTQ coupling > 1 % |

---

## 7. Architecture

```
torque_cmd[0:2], cmd_valid, safe_mode
      │
      ▼
┌───────────────────────────┐  rw_cmd[0:2], rw_cmd_valid
│  actuator_command_arbiter │──────────────────────────┐
│  (torque → RW speed;      │                          │
│   safe_mode gating)       │                          │
└───────────────────────────┘                          │
                                                        ▼
                                         ┌─────────────────────────┐
                                         │  rw_spi_driver          │
                                         │  (SPI cmds @ 1kHz;      │
                                         │   fault poll @ 10Hz)    │
                                         │  rw_sclk/mosi/cs_n/miso │
                                         └─────────────────────────┘
                                                        │
                                               rw_spi_fault[2:0]
                                                        │
                                         ┌──────────────▼──────────┐
                                         │  fault_status_monitor   │
                                         │  (watchdog + SPI merge) │──▶ rw_fault[2:0]
                                         └─────────────────────────┘

torque_cmd[0:2] (direct)
      │
      ▼ (clk_100mhz domain)
┌────────────────────────┐
│  mtq_driver            │──▶ mtq_pwm[2:0], dir_mtq[2:0]
│  (10 kHz PWM, sat flag │──▶ mtq_enable[2:0], mtq_sat_flag[2:0]
│   coupling monitor)    │──▶ coupling_warning
└────────────────────────┘

rw_driver (legacy PWM back-channel) ──▶ rw_enable[2:0] (gated by safe_mode)
```

**Reused Helper IPs (from `CubeSat/`):**
- `spi_master.sv` — SPI master engine (used inside `rw_spi_driver`)
- `pwm_gen.sv` — PWM generator (used inside `mtq_driver`; 10 kHz)

---

## 8. Data Formats

| Signal | Format | Notes |
|---|---|---|
| `torque_cmd[0:2]` | Q15 signed (16-bit) | ± 1.0 maps to ± 10 mNm via SAT_LIMIT in CS6 |
| `rw_cmd[0:2]` (internal) | Q15 signed (16-bit) | Arbiter maps torque → RW speed |
| RW speed (SPI payload) | 16-bit signed | ± 6000 RPM; 1 LSB = 0.2 RPM |
| MTQ PWM duty | 12-bit unsigned (internal) | Derived from |torque_cmd| / 32768; 10 kHz carrier |

---

## 9. Register Interface

CS7 has **no AXI4-Lite register interface** in the current implementation. The PWM carrier frequency (10 kHz for MTQ) and SPI clock rate are compile-time parameters.

**Parameters (synthesised):**

| Parameter | Default | Description |
|---|---|---|
| `CLK_HZ` | 100_000_000 | System clock in Hz |

---

## 10. File Structure

```
CubeSat/CS7_ACTUATORS/
├── actuator_wrapper.sv          ← Top-level wrapper; CS12 integration point
├── actuator_command_arbiter.sv  ← Routes torque_cmd → RW speed; safe_mode gating
├── rw_spi_driver.sv             ← SPI motor commands @ 1 kHz + fault polling
├── rw_driver.sv                 ← Legacy PWM back-channel for RW gate enables
├── mtq_driver.sv                ← 3-axis 10 kHz PWM + direction + saturation
├── fault_status_monitor.sv      ← Watchdog + SPI fault merge → rw_fault[2:0]
├── tb_actuator_wrapper.sv       ← Integration testbench
└── README.md                    ← This file

CubeSat/ (shared helper IPs used by CS7):
├── spi_master.sv                ← SPI master engine
└── pwm_gen.sv                   ← PWM generator (10 kHz)
```

Note: `magnetorquer_pwm.sv` exists in the directory but is **not instantiated** in `actuator_wrapper.sv`; MTQ PWM functionality is provided directly by `mtq_driver.sv`.

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `ce_1khz` | CS7 ← CS12 | `clk_manager` (CS12) | 1 kHz control trigger |
| `torque_cmd[0:2]`, `cmd_valid` | CS7 ← CS6 | `pd_control_wrapper` (CS6) | Torque commands from PD controller |
| `safe_mode` | CS7 ← CS8 | `adcs_fsm_wrapper` (CS8) | Safe-mode blanking (SAFE or FAULT state) |
| `rw_miso[2:0]` | CS7 ← Physical | FPGA I/O | SPI MISO from RW motor controllers |
| `rw_sclk/mosi/cs_n` | CS7 → Physical | FPGA I/O | SPI bus to RW motor controllers |
| `mtq_pwm[2:0]`, `dir_mtq[2:0]` | CS7 → Physical | FPGA I/O | PWM + direction to MTQ H-bridges |
| `rw_fault[2:0]` | CS7 → CS8 | `adcs_fsm_wrapper` (CS8) | Actuator fault reporting |
| `mtq_sat_flag[2:0]` | CS7 → CS11 | `telemetry_wrapper` (CS11) | Saturation telemetry |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- SPI command latency: < 6 µs per RW (8-bit command at ~8.33 MHz SCLK); well within 1 ms budget.
- MTQ PWM update: combinational duty calculation; < 100 µs to first PWM edge.
- Safe-mode blanking: combinational `assign`; zero latency.

**Resource:**
- `rw_spi_driver`: SPI FSM + speed register; < 80 LUTs, 1 DSP48E1 (speed conversion).
- `mtq_driver`: 3× PWM counters + comparators; < 60 LUTs, 0 DSP48E1.
- `fault_status_monitor`: watchdog counters + OR logic; < 30 LUTs.
- Total CS7 DSP48 estimate: 1 (per plan); BRAM: 128 B.

**Optimization Opportunities:**
1. Add AXI4-Lite interface to `rw_spi_driver` for runtime speed limit and fault threshold configuration.
2. Extend `mtq_driver` with trapezoidal current limiting to reduce electromagnetic interference.
3. Share a single `pwm_gen` instance across 3 MTQ axes via time-division multiplexing.
4. Add reaction wheel speed telemetry read-back from SPI fault poll response.

**Timing:**
- MTQ PWM counter runs on `clk_100mhz`; separate domain from `sys_clk` control path. No CDC needed for safe_mode (combinational blanking).
- SPI clock: ≤ CLK_HZ / 12 for reliable MISO sampling.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS7_ACTUATORS/tb_actuator_wrapper.sv`

**Test Scenarios:**
- Apply `torque_cmd = [0.5, −0.25, 0.1]` (Q15) at 1 kHz; verify SPI commands are sent to all 3 RW axes.
- Assert `safe_mode = 1`; verify all `mtq_pwm`, `dir_mtq`, `mtq_enable`, `rw_enable` go to 0 within 1 cycle.
- De-assert `cmd_valid` for > 200 ms; verify `rw_fault` asserts from watchdog.
- Apply MTQ `torque_cmd` near saturation limit; verify `mtq_sat_flag` asserts.
- Apply equal torque on all 3 axes; verify `coupling_warning` reflects cross-axis interaction.
- Simulate SPI MISO fault response; verify per-axis `rw_fault` asserts.

**Simulation Notes:**
- Compile with `iverilog -g2012` including `spi_master.sv`, `pwm_gen.sv`.
- Timescale: 1 ns / 1 ps.

**Requirements Coverage:**
- CS-ADCS-008: SPI commands ± 6000 RPM @ 1 kHz, fault status @ 10 Hz.
- CS-ADCS-009: 3-axis PWM @ 10 kHz, ≥ 12-bit resolution, < 1 % coupling, SAFE blanking.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
