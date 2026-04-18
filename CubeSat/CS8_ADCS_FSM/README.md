# CS8 — ADCS Mode FSM & Health Monitor

> 6-state supervisory FSM (BOOT → DETUMBLE → COARSE_POINT → FINE_POINT → SAFE → FAULT) with integrated health monitoring, BRAM circular telemetry log, and per-axis fault tracking.

---

## Overview

| Attribute | Value |
|---|---|
| Requirements | CS-ADCS-010, CS-ADCS-011, CS-ADCS-012 |
| Top module | `adcs_fsm_wrapper` |
| Clock domain | `sys_clk` (50 MHz) |
| CE strobe | 100 Hz `ce_100hz` from CS12 |
| States | BOOT, DETUMBLE, COARSE_POINT, FINE_POINT, SAFE, FAULT |
| Health watchdog | `HBEAT_TIMEOUT = 1` ce_100hz ticks (≈10 ms) |
| BRAM log | 256 × 96-bit circular buffer @ 100 Hz |
| BRAM total | 3,072 B |
| DSP48 | 0 |

---

## File Structure

| File | Purpose |
|---|---|
| `adcs_fsm_wrapper.sv` | Top-level — FSM, transition logic, output driving |
| `health_monitor.sv` | Heartbeat watchdog, norm check, fault flag aggregation |
| `adcs_data_logger.sv` | Packs 96-bit snapshot (q + ω + mode + faults) per CE tick |
| `bram_circular_buffer.sv` | 256-deep 96-bit dual-port BRAM; write pointer wraps |
| `fault_logger.sv` | Timestamped fault-event ring buffer |
| `tb_adcs_fsm_wrapper.sv` | Self-checking directed testbench |

Shared primitives used: `synchronizer.sv`, `debouncer.sv`.

---

## Module Interface

```systemverilog
module adcs_fsm_wrapper #(
    parameter int OMEGA_DET_THRESH   = 1638,  // Q15 ≈ 0.05 rad/s
    parameter int QERR_COARSE_THRESH = 3277,  // Q15 ≈ 0.1 rad (~5.7°)
    parameter int QERR_FINE_THRESH   = 6554,  // Q15 ≈ 0.2 rad (~11.5°)
    parameter int FAULT_PERSIST      = 1,     // ce_100hz ticks
    parameter int UL_CMD_TIMEOUT     = 10     // ce_100hz ticks (100 ms)
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,

    // Sensor heartbeats
    input  logic        imu_valid, mag_valid, ekf_valid,

    // State magnitudes (Q15)
    input  logic [15:0] q_err_mag,   // from CS5
    input  logic [15:0] omega_mag,   // from CS1

    // Uplink command
    input  logic [2:0]  uplink_mode,
    input  logic        uplink_cmd_valid,

    // External fault injection
    input  logic [7:0]  fault_trigger,

    // BRAM logging inputs
    input  logic [31:0] q_est,       // packed 4×8-bit quaternion from CS5
    input  logic [23:0] omega_in,    // packed 3×8-bit angular rate from CS1
    input  logic [7:0]  bram_log_rd_addr,

    // Outputs
    output logic [2:0]  adcs_mode,
    output logic        mode_valid,
    output logic        health_ok,
    output logic [7:0]  fault_flags,
    output logic        adcs_fault,
    output logic [23:0] per_axis_faults,

    // BRAM read-back (for CS11 telemetry)
    output logic [95:0] bram_log_rd_data,
    output logic [7:0]  bram_log_addr,
    output logic [95:0] bram_log_data,
    output logic        bram_log_wr_en
);
```

---

## Functionality

### FSM State Transitions

| From | To | Condition |
|---|---|---|
| BOOT | DETUMBLE | `imu_valid & mag_valid` |
| DETUMBLE | COARSE_POINT | `omega_mag < OMEGA_DET_THRESH` |
| COARSE_POINT | FINE_POINT | `q_err_mag < QERR_COARSE_THRESH` |
| FINE_POINT | COARSE_POINT | `q_err_mag > QERR_FINE_THRESH` or `!health_ok` |
| any | SAFE | uplink `SAFE_CMD` or `!health_ok` or uplink timeout |
| any | FAULT | `fault_flags[7]` held `FAULT_PERSIST` ticks |
| SAFE/FAULT | BOOT | uplink `BOOT_CMD & health_ok` |

### Health Monitor (`health_monitor`)
- Heartbeat watchdog: asserts fault if `imu_valid`/`mag_valid`/`ekf_valid` absent > `HBEAT_TIMEOUT` ticks.
- `fault_trigger[7:0]`: low nibble → `fault_flags[5]`; high nibble → `fault_flags[6]`.
- `fault_flags[7]` = OR of all faults including external triggers.
- `per_axis_faults[23:0]`: 8-bit fault accumulator per axis.

### BRAM Logging
- `adcs_data_logger` writes a 96-bit record (q_est + omega_in + adcs_mode + fault_flags) every `ce_100hz` tick.
- `bram_circular_buffer` wraps write pointer at depth 256; dual-port for simultaneous write and telemetry read.
- CS11 reads via `bram_log_rd_addr` / `bram_log_rd_data`.

---

## Simulation Instructions

```bash
iverilog -g2012 -o sim_cs8 \
  CS8_ADCS_FSM/tb_adcs_fsm_wrapper.sv \
  CS8_ADCS_FSM/adcs_fsm_wrapper.sv \
  CS8_ADCS_FSM/health_monitor.sv \
  CS8_ADCS_FSM/adcs_data_logger.sv \
  CS8_ADCS_FSM/bram_circular_buffer.sv \
  CS8_ADCS_FSM/fault_logger.sv \
  synchronizer.sv debouncer.sv

vvp sim_cs8
```

```tcl
vlog -sv CS8_ADCS_FSM/tb_adcs_fsm_wrapper.sv CS8_ADCS_FSM/*.sv synchronizer.sv debouncer.sv
vsim -t 1ps tb_adcs_fsm_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 50 MHz `sys_clk` |
| CE | 100 Hz internally generated |
| Stimulus | Scripted magnitude sequences driving state transitions; fault injection via `fault_trigger` |
| Checking | `adcs_mode` encoding at each transition; `fault_flags` set/clear; BRAM read-back matches written record |
| Coverage | All 6 states, uplink override, heartbeat dropout, fault persistence, BRAM wrap-around |

---

## Expected Behaviour

```
ce_100hz        __|‾|__|‾|__|‾|__|‾|__|‾|__
omega_mag:      HIGH────────LOW (detumble complete)
adcs_mode[2:0]: BOOT(0)──DETUMBLE(1)──COARSE_POINT(2)──FINE_POINT(3)
health_ok:      ‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾  (stays high when sensors valid)
adcs_fault:     ____________  (asserts on fault_flags[7] persistence)
safe_mode out → CS7 blanking asserted when adcs_mode = SAFE or FAULT
```

---

## Limitations

- Transition thresholds (`OMEGA_DET_THRESH`, `QERR_*`) are compile-time parameters; no runtime command path.
- Fault policy is simple OR-based threshold; no fault classification or priority weighting.
- BRAM log is 256 records deep; at 100 Hz this covers 2.56 s before overwrite.
- No command authentication / rollback for mode overrides.

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] All 6 FSM states reachable and transitions verified
- [x] Health monitor heartbeat dropout → SAFE transition
- [x] `fault_trigger` bit mapping verified
- [x] BRAM write/read-back verified; circular wrap-around confirmed
- [x] Uplink command timeout → SAFE verified
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] Runtime-configurable threshold registers
- [ ] Synthesis BRAM inference confirmed (target: 3,072 B)
