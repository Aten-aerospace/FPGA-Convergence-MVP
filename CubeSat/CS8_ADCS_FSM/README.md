# CS8 — ADCS Mode FSM & Health Monitor

## 1. Module Title & ID

**Module:** CS8 — ADCS Mode FSM & Health Monitor
**Subsystem ID:** CS8
**Requirements:** CS-ADCS-010, CS-ADCS-011, CS-ADCS-012

---

## 2. Overview

CS8 implements the top-level ADCS operational mode finite-state machine (6 states), a sensor health monitor, a BRAM-based circular data logger, and a fault logger. It determines the current pointing regime, drives safe-mode signalling to actuators (CS7) and laser (CS10), and logs ADCS state at 100 Hz into a 256×96-bit dual-port circular buffer accessible by the telemetry subsystem (CS11). Uplink commands are validated with a configurable timeout to guard against stale ground commands.

**Target Platform:** Xilinx Artix-7 XC7A35T

---

## 3. Criticality

**CRITICAL** — CS-ADCS-010 (FSM), CS-ADCS-011 (Health Monitor), CS-ADCS-012 (Data Logging). CS8 is the system-level management layer; its `safe_mode` output gates all actuator outputs in CS7. Incorrect FSM transitions or missed health events can result in loss of attitude control.

---

## 4. Key Functionality

**Mode FSM (adcs_fsm_wrapper):**
- 6-state machine: BOOT → DETUMBLE → COARSE_POINT → FINE_POINT → SAFE ↔ FAULT.
- Transition guards: angular rate magnitude (`omega_mag`) vs `OMEGA_DET_THRESH`; quaternion error magnitude (`q_err_mag`) vs `QERR_COARSE_THRESH` / `QERR_FINE_THRESH`.
- Uplink command timeout: `UL_CMD_TIMEOUT = 10` ce_100hz ticks (100 ms); stale uplink triggers SAFE.
- Any fault → SAFE within `FAULT_PERSIST = 1` ce_100hz ticks (≈ 10 ms).
- SAFE/FAULT → BOOT on uplink BOOT_CMD with `health_ok`.

**Health Monitor (health_monitor):**
- Monitors IMU, MAG, and EKF heartbeat timeouts (1-tick watchdog).
- Quaternion norm check: `q_norm` counter trips after 3 consecutive out-of-tolerance cycles.
- External fault injection via `fault_trigger[7:0]`: low nibble → `fault_flags[5]`; high nibble → `fault_flags[6]`.
- Per-axis fault tracking exposed via `per_axis_faults[23:0]`.
- `fault_flags[7]` = OR of all internal plus external faults.

**Data Logger (adcs_data_logger + bram_circular_buffer):**
- Logs `q_est[31:0]` + `omega_in[23:0]` + mode + fault_flags @ 100 Hz.
- 256-entry × 96-bit circular BRAM buffer; dual-port (write @ 100 Hz, read by CS11 telemetry).
- Exposed read interface: `bram_log_rd_addr[7:0]` → `bram_log_rd_data[95:0]`.

**Fault Logger (fault_logger):**
- Logs fault codes + timestamps to FIFO; accessible via telemetry (CS11).

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `clk` | input | 1 | `sys_clk` | 100 MHz system clock |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `ce_100hz` | input | 1 | `sys_clk` | 100 Hz clock-enable from CS12 |
| `imu_valid` | input | 1 | `sys_clk` | IMU data heartbeat (from CS1) |
| `mag_valid` | input | 1 | `sys_clk` | Magnetometer data heartbeat (from CS2) |
| `ekf_valid` | input | 1 | `sys_clk` | EKF output heartbeat (from CS5) |
| `q_err_mag` | input | 16 | `sys_clk` | Quaternion error magnitude, unsigned Q15 (from CS5) |
| `omega_mag` | input | 16 | `sys_clk` | Angular rate magnitude, unsigned Q15 (from CS1) |
| `uplink_mode[2:0]` | input | 3 | `sys_clk` | Ground command mode override (3'd7 = no command) |
| `uplink_cmd_valid` | input | 1 | `sys_clk` | Pulse when `uplink_mode` contains a fresh command |
| `fault_trigger[7:0]` | input | 8 | `sys_clk` | External fault injection |
| `q_est[31:0]` | input | 32 | `sys_clk` | Packed quaternion estimate (4×8-bit) for BRAM log |
| `omega_in[23:0]` | input | 24 | `sys_clk` | Packed angular rate (3×8-bit) for BRAM log |
| `bram_log_rd_addr[7:0]` | input | 8 | `sys_clk` | BRAM circular buffer read address (for CS11) |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `adcs_mode[2:0]` | output | 3 | `sys_clk` | Current ADCS operational mode (binary-coded) |
| `mode_valid` | output | 1 | `sys_clk` | FSM has progressed past BOOT |
| `health_ok` | output | 1 | `sys_clk` | All health checks passing |
| `fault_flags[7:0]` | output | 8 | `sys_clk` | Active fault bitfield |
| `adcs_fault` | output | 1 | `sys_clk` | In FAULT state |
| `per_axis_faults[23:0]` | output | 24 | `sys_clk` | Per-axis fault tracking from health_monitor |
| `bram_log_rd_data[95:0]` | output | 96 | `sys_clk` | BRAM circular buffer read data |
| `bram_log_addr[7:0]` | output | 8 | `sys_clk` | BRAM current write address (for external monitoring) |
| `bram_log_data[95:0]` | output | 96 | `sys_clk` | Current BRAM write data |
| `bram_log_wr_en` | output | 1 | `sys_clk` | BRAM write enable strobe |

---

## 7. Architecture

```
imu_valid, mag_valid, ekf_valid, q_err_mag, omega_mag
uplink_mode, uplink_cmd_valid, fault_trigger
      │
      ▼
┌──────────────────────────────────────────────────────────────┐
│  adcs_fsm_wrapper                                            │
│  ┌────────────────────┐      ┌──────────────────────────────┐│
│  │  health_monitor    │      │  ADCS Mode FSM (6-state)     ││
│  │  (heartbeat wdogs, │─────▶│  BOOT→DETUMBLE→COARSE_POINT ││
│  │   q_norm check,    │      │  →FINE_POINT↔SAFE↔FAULT      ││
│  │   external fault   │      │  uplink timeout guard        ││
│  │   injection)       │      └──────────────────────────────┘│
│  └────────────────────┘                     │ adcs_mode       │
│  ┌───────────────────────────────────────── │ ───────────────┐│
│  │  adcs_data_logger                        │                ││
│  │  (packs q_est + omega + mode + faults    │                ││
│  │   → bram_circular_buffer @ 100 Hz)       │                ││
│  └──────────────────────────────────────────│ ───────────────┘│
│  ┌────────────────┐                         │                 │
│  │  fault_logger  │ (fault FIFO + timestamps)│                │
│  └────────────────┘                                           │
└──────────────────────────────────────────────────────────────┘
```

**Mode Encoding:**

| Value | State |
|---|---|
| 0 | BOOT |
| 1 | DETUMBLE |
| 2 | COARSE_POINT |
| 3 | FINE_POINT |
| 4 | SAFE |
| 5 | FAULT |

**Reused Helper IPs (from `CubeSat/`):**
- `synchronizer.sv` — CDC for fault signals
- `debouncer.sv` — Fault signal debouncing
- `edge_detect.sv` — State transition edge detection

---

## 8. Data Formats

| Signal | Format | Notes |
|---|---|---|
| `q_err_mag`, `omega_mag` | Q15 unsigned (16-bit) | Magnitude; 1.0 = 32767 |
| `q_est[31:0]` | 4 × 8-bit packed | Compressed quaternion for BRAM log |
| `omega_in[23:0]` | 3 × 8-bit packed | Compressed angular rate for BRAM log |
| BRAM log entry | 96-bit word | q_est(32) + omega(24) + mode(3) + faults(8) + padding |
| `fault_flags[7:0]` | 8-bit bitmask | See bit definitions below |
| `per_axis_faults[23:0]` | 3 × 8-bit | Per-axis fault accumulation |

**`fault_flags` bit definitions:**

| Bit | Source | Condition |
|---|---|---|
| [0] | IMU heartbeat timeout | `imu_valid` absent for ≥ HBEAT_TIMEOUT ticks |
| [1] | MAG heartbeat timeout | `mag_valid` absent for ≥ HBEAT_TIMEOUT ticks |
| [2] | EKF heartbeat timeout | `ekf_valid` absent for ≥ HBEAT_TIMEOUT ticks |
| [3] | Angular rate overflow | `omega_mag` > threshold |
| [4] | Quaternion norm error | q_norm deviates for ≥ QNORM_CYCLES cycles |
| [5] | External fault (low nibble) | `fault_trigger[3:0]` non-zero |
| [6] | External fault (high nibble) | `fault_trigger[7:4]` non-zero |
| [7] | Any fault | OR of bits [0:6] |

---

## 9. Register Interface

CS8 has **no AXI4-Lite register interface** in the current implementation. Threshold constants (`OMEGA_DET_THRESH`, `QERR_COARSE_THRESH`, etc.) are compile-time parameters. BRAM log can be read via the `bram_log_rd_addr / bram_log_rd_data` port exposed to CS11.

**Parameters (synthesised):**

| Parameter | Default | Description |
|---|---|---|
| `OMEGA_DET_THRESH` | 1638 | Q15 ≈ 0.05 rad/s; DETUMBLE→COARSE_POINT threshold |
| `QERR_COARSE_THRESH` | 3277 | Q15 ≈ 0.1 rad; COARSE→FINE threshold |
| `QERR_FINE_THRESH` | 6554 | Q15 ≈ 0.2 rad; FINE→COARSE regression threshold |
| `FAULT_PERSIST` | 1 | ce_100hz ticks before FAULT transition (≈ 10 ms) |
| `UL_CMD_TIMEOUT` | 10 | ce_100hz ticks before uplink treated as stale (100 ms) |

---

## 10. File Structure

```
CubeSat/CS8_ADCS_FSM/
├── adcs_fsm_wrapper.sv      ← Top-level FSM + health + logger wrapper; CS12 integration point
├── health_monitor.sv        ← Heartbeat watchdogs, q-norm check, fault injection
├── adcs_data_logger.sv      ← Packs telemetry data → BRAM write port
├── bram_circular_buffer.sv  ← 256×96-bit dual-port circular buffer
├── fault_logger.sv          ← Fault code + timestamp FIFO
├── tb_adcs_fsm_wrapper.sv   ← Integration testbench
└── README.md                ← This file

CubeSat/ (shared helper IPs used by CS8):
├── synchronizer.sv          ← CDC synchroniser
├── debouncer.sv             ← Input debouncer
└── edge_detect.sv           ← Edge detector
```

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `ce_100hz` | CS8 ← CS12 | `clk_manager` (CS12) | 100 Hz FSM + logger tick |
| `imu_valid` | CS8 ← CS1 | `spi_imu_wrapper` (CS1) | IMU heartbeat |
| `mag_valid` | CS8 ← CS2 | `i2c_mag_wrapper` (CS2) | Magnetometer heartbeat |
| `ekf_valid` | CS8 ← CS5 | `ekf_wrapper` (CS5) | EKF heartbeat |
| `q_err_mag` | CS8 ← CS5 | `ekf_wrapper` (CS5) | Pointing error for FSM transitions |
| `omega_mag` | CS8 ← CS1 | `spi_imu_wrapper` (CS1) | Angular rate for detumble detection |
| `adcs_mode[2:0]` | CS8 → CS7 | `actuator_wrapper` (CS7) | `safe_mode = (mode == SAFE || mode == FAULT)` |
| `adcs_mode[2:0]` | CS8 → CS10 | `laser_fsm_wrapper` (CS10) | `laser_enable = (mode == FINE_POINT)` |
| `bram_log_rd_data` | CS8 → CS11 | `telemetry_wrapper` (CS11) | ADCS telemetry readout |
| `fault_flags` | CS8 → CS11 | `telemetry_wrapper` (CS11) | Fault status for HK telemetry |
| `uplink_mode` | CS8 ← CS11 | `telemetry_wrapper` (CS11) | Decoded ground command |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- FSM transitions are evaluated every `ce_100hz` tick (10 ms); deterministic latency.
- BRAM write completes in 1 clock cycle (synchronous dual-port BRAM).
- Uplink timeout watchdog runs on `ce_100hz`; 100 ms resolution.

**Resource:**
- BRAM: 256 × 96 bits = 24,576 bits ≈ 3 KB (3,072 B per plan) → 1.5 BRAM18 blocks.
- FSM + health_monitor: < 200 LUTs, 0 DSP48E1.
- fault_logger FIFO: < 64 entries; fits in distributed RAM.

**Optimization Opportunities:**
1. Add AXI4-Lite registers for runtime threshold adjustment (OMEGA_DET_THRESH, QERR_*).
2. Expose fault logger FIFO via AXI4-Lite for direct ground-station diagnostic access.
3. Increase BRAM circular buffer depth (currently 256 × 96 B) if longer logging duration is needed.
4. Add mode dwell timer to prevent chattering at COARSE/FINE boundary.

**Timing:**
- Single clock domain (`sys_clk`); no CDC within CS8 except for fault signals from sensors (handled by `synchronizer.sv`).
- BRAM timing: write latency 1 cycle; read latency 1–2 cycles.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS8_ADCS_FSM/tb_adcs_fsm_wrapper.sv`

**Test Scenarios:**
- Power-on reset; verify FSM starts in BOOT state.
- Assert `imu_valid + mag_valid`; verify transition BOOT → DETUMBLE.
- Drive `omega_mag` below `OMEGA_DET_THRESH`; verify DETUMBLE → COARSE_POINT.
- Drive `q_err_mag` below `QERR_COARSE_THRESH`; verify COARSE_POINT → FINE_POINT.
- Assert `uplink_mode = 3'd4` (SAFE); verify immediate transition to SAFE.
- Hold `imu_valid = 0` for > HBEAT_TIMEOUT ticks; verify `fault_flags[0]` asserts and FSM enters SAFE.
- Assert `fault_trigger = 8'hF0`; verify `fault_flags[6]` asserts.
- Verify BRAM log: confirm `bram_log_rd_data` matches expected packed q_est/omega/mode values at the corresponding read address.

**Simulation Notes:**
- Compile with `iverilog -g2012` including `synchronizer.sv`, `debouncer.sv`, `edge_detect.sv`.
- Timescale: 1 ns / 1 ps.

**Requirements Coverage:**
- CS-ADCS-010: 5-state FSM (BOOT→DETUMBLE→COARSE→FINE→SAFE), fault→SAFE within 2 ms.
- CS-ADCS-011: SPI/I2C timeout > 5 ms, IMU overflow > 500°/s, EKF divergence flags.
- CS-ADCS-012: 256×96 B circular BRAM buffer @ 100 Hz, dual-port for telemetry.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
