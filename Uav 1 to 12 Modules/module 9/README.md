# UAV Module 9: Watchdog Timer + Pre-Flight Checker + Emergency Handler

## Overview

UAV Module 9 provides the safety and health supervision layer for the UAV FPGA autopilot. It combines three safety-critical functions: a configurable hardware watchdog timer that triggers emergency mode on processor hang, a pre-flight checklist that gates the ARM command until all systems are nominal, and an emergency descent controller that safely brings the vehicle down when faults occur. All parameters are observable and configurable via the AXI4-Lite register interface.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🔴 CRITICAL — Last line of defense against software/hardware faults

---

## Key Functionality

### Watchdog Timer
- **500 ms default timeout** (configurable 100–2000 ms via AXI register)
- Configuration changes allowed **only in DISARMED state** (safety lock)
- **Kicked at 100 Hz** by MOD_8 FSM (must kick every 10 ms to prevent expiry)
- Expiry → `WDT_EXPIRED` → immediate EMERGENCY state transition in MOD_8

### Pre-Flight Checker
Verifies all ARM preconditions; outputs `NAV_READY` only when all pass:

| Check          | Source  | Condition                              |
|----------------|---------|----------------------------------------|
| EKF healthy    | MOD_5/6 | `ekf_healthy = 1`                      |
| GPS 3D fix     | MOD_6   | Fix type ≥ 3                           |
| HDOP ≤ 2.5     | MOD_6   | `gps_hdop` ≤ 250 (×100 representation)|
| BARO valid     | MOD_5   | `baro_valid = 1` (altitude available)  |
| Sensor faults  | MOD_5/6 | All sensor fault flags = 0 (IMU/GPS/BARO/MAG) |

Outputs:
- `NAV_READY` — all checks pass; enables ARM
- `PRE_FLIGHT_FAIL` — 8-bit bitmask of individual check failures

### Emergency Handler
Actions triggered on EMERGENCY entry:
- **Horizontal position hold** via last valid EKF velocity estimate
- **Ramp collective thrust to 30%** on full EKF invalidation
- **Configurable descent rate** (`EMERG_DESCENT_RATE`, default 2 m/s, Q4.28)
- **MAVLink STATUSTEXT** broadcast (severity=EMERGENCY) within 1 MAVLink cycle

---

## Input Signals

| Signal Name           | Width  | Format  | Source      | Description                                     |
|-----------------------|--------|---------|-------------|-------------------------------------------------|
| `clk`                 | 1-bit  | —       | MOD_1       | 50 MHz system clock                             |
| `rst_n`               | 1-bit  | —       | MOD_1       | Active-low synchronous reset                    |
| `ce_100hz`            | 1-bit  | —       | MOD_1       | Watchdog kick / preflight update strobe         |
| `fsm_state`           | 8-bit  | one-hot | MOD_8       | Current flight state for watchdog & config gate |
| `wdt_kick`            | 1-bit  | —       | MOD_8       | Watchdog kick pulse (asserted each ce_100hz)    |
| `ekf_healthy`         | 1-bit  | —       | MOD_5/6     | EKF health flag                                 |
| `gps_fix_valid`       | 1-bit  | —       | MOD_6       | GPS 3D fix AND HDOP ≤ 2.5                       |
| `gps_hdop`            | 16-bit | ×100    | MOD_6       | HDOP accuracy (250 = 2.50)                      |
| `baro_valid`          | 1-bit  | —       | MOD_5       | Barometric altitude valid                       |
| `sensor_fault_flags`  | 8-bit  | bitmask | MOD_5/6     | IMU/GPS/BARO/MAG fault flags                    |
| `ekf_vel_n`           | 32-bit | Q4.28   | MOD_5/6     | North velocity for emergency hold               |
| `ekf_vel_e`           | 32-bit | Q4.28   | MOD_5/6     | East velocity for emergency hold                |
| `ekf_alt`             | 32-bit | Q10.22  | MOD_5/6     | Altitude for emergency descent reference        |
| `wdt_timeout_cfg`     | 16-bit | ms      | MOD_10 AXI  | Watchdog timeout setting (100–2000 ms)          |
| `emerg_descent_rate`  | 32-bit | Q4.28   | MOD_10 AXI  | Emergency descent rate (default 2.0 m/s)        |

---

## Output Signals

| Signal Name                | Width  | Format  | Destination     | Description                                   |
|----------------------------|--------|---------|-----------------|-----------------------------------------------|
| `watchdog_expired`         | 1-bit  | —       | MOD_8 FSM       | Watchdog timeout → trigger EMERGENCY          |
| `nav_ready`                | 1-bit  | —       | MOD_8 FSM       | All pre-flight checks passed → enables ARM    |
| `preflight_fail`           | 8-bit  | bitmask | MOD_10 AXI, MOD_8 | Individual check failure bits               |
| `emergency_descent_active` | 1-bit  | —       | MOD_7 (status)  | EMERGENCY descent in progress                 |
| `emergency_thrust_cmd`     | 16-bit | unsigned| MOD_2 (backup)  | 30% collective thrust override during fault   |
| `emergency_pos_hold_en`    | 1-bit  | —       | MOD_2           | Enable position-hold velocity setpoint        |
| `statustext_req`           | 1-bit  | —       | MOD_7 MAVLink   | Request STATUSTEXT broadcast                  |
| `statustext_severity`      | 8-bit  | —       | MOD_7 MAVLink   | Message severity (6=EMERGENCY)                |

---

## Architecture

### High-Level Components

```
          ┌──────────────────────────────────────────────────────────┐
          │             module9_uav_watchdog.sv                      │
          │                                                          │
wdt_kick ►│  watchdog_timer.sv                                      ├──► watchdog_expired
fsm_state►│  (500ms default, 100-2000ms configurable)               │
          │  (config locked outside DISARMED)                        │
          │                                                          │
ekf/gps/ ►│  preflight_checker.sv                                   ├──► nav_ready
baro/     │  (all checks: EKF + GPS + HDOP + BARO + faults)         ├──► preflight_fail
faults   ►│                                                          │
          │                                                          │
EMERGENCY►│  emergency_descent.sv                                   ├──► emergency_thrust_cmd
          │  (position hold, 30% thrust, configurable descent rate)  ├──► emergency_pos_hold_en
          │  (STATUSTEXT broadcast trigger)                          ├──► statustext_req
          └──────────────────────────────────────────────────────────┘
```

### Reused Modules from RTL_20

| Module        | Role                                             |
|---------------|--------------------------------------------------|
| `tick_gen.sv` | 1 ms tick for sub-100 Hz watchdog counting       |

### New Modules Created

| Module                  | Role                                                    |
|-------------------------|---------------------------------------------------------|
| `watchdog_timer.sv`     | Configurable countdown timer, kick-based reset          |
| `preflight_checker.sv`  | Multi-source health check aggregator + NAV_READY output |
| `emergency_descent.sv`  | Position hold, thrust ramp, descent rate control        |

---

## Data Formats

| Parameter                | Format    | Range          | Notes                                     |
|--------------------------|-----------|----------------|-------------------------------------------|
| Watchdog timeout         | 16-bit ms | 100–2 000 ms   | Configurable in DISARMED state only        |
| Pre-flight fail bitmask  | 8-bit     | 0x00–0xFF      | Bit[n]=1 means check n failed             |
| Emergency descent rate   | Q4.28     | 0–10 m/s       | Default 2.0 m/s                           |
| Emergency thrust         | 16-bit unsigned | 0–32767  | 30% = ~9 830 counts                      |
| HDOP threshold           | ×100 int  | 250 = HDOP 2.5 | From GGA field via MOD_6                  |

### Pre-Flight Fail Bitmask

| Bit | Check                   | Condition for Failure               |
|-----|-------------------------|-------------------------------------|
| [0] | EKF_HEALTHY             | `ekf_healthy = 0`                   |
| [1] | GPS_FIX                 | Fix type < 3                        |
| [2] | GPS_HDOP                | HDOP > 2.5                          |
| [3] | BARO_VALID              | Barometric altitude not available   |
| [4] | IMU_FAULT               | IMU fault flag asserted             |
| [5] | GPS_FAULT               | GPS fault flag asserted             |
| [6] | BARO_FAULT              | Barometer fault flag asserted       |
| [7] | MAG_FAULT               | Magnetometer fault flag asserted    |

---

## Register Interface (AXI4-Lite via MOD_10)

| Register Name          | Address (MOD_10) | Access | Format      | Description                              |
|------------------------|------------------|--------|-------------|------------------------------------------|
| `WATCHDOG_TIMEOUT`     | 0xB0             | R/W    | 16-bit ms   | Watchdog period (100–2000 ms); DISARMED only |
| `NAV_READY`            | 0xBC             | R/O    | 1-bit       | All pre-flight checks passed             |
| `PRE_FLIGHT_FAIL`      | 0xC0             | R/O    | 8-bit mask  | Individual check failure bits            |
| `EMERG_DESCENT_RATE`   | 0xB4             | R/W    | Q4.28       | Emergency descent rate (default 2.0 m/s) |

---

## File Structure

```
Uav 1 to 12 Modules/module 9/
├── module9_uav_watchdog.sv    # Top-level safety supervisor wrapper
├── watchdog_timer.sv          # Configurable countdown watchdog
├── preflight_checker.sv       # Multi-source pre-flight check aggregator
└── emergency_descent.sv       # Emergency position hold + descent controller
```

### Dependencies (from RTL_20 shared library)

```
tick_gen.sv   # 1 ms tick for watchdog countdown resolution
```

---

## Module Interconnections

```
MOD_1 ── ce_100hz ──────────────────────────────► MOD_9 (watchdog/preflight strobe)
MOD_8 ── wdt_kick / fsm_state ──────────────────► MOD_9 (watchdog kick + mode lock)
MOD_5/6 ── ekf_healthy / baro_valid ────────────► MOD_9 (preflight health sources)
MOD_6 ── gps_fix_valid / gps_hdop ──────────────► MOD_9 (GPS preflight checks)
MOD_5/6 ── sensor_fault_flags ──────────────────► MOD_9 (fault flag aggregation)
MOD_5/6 ── ekf_velocity / ekf_alt ──────────────► MOD_9 (emergency hold reference)
MOD_10 ── wdt_timeout_cfg / emerg_rate ─────────► MOD_9 (configuration registers)
MOD_9 ── watchdog_expired ──────────────────────► MOD_8 FSM (emergency trigger)
MOD_9 ── nav_ready ─────────────────────────────► MOD_8 FSM (ARM enable)
MOD_9 ── preflight_fail ─────────────────────────► MOD_8 FSM + MOD_10 AXI status
MOD_9 ── emergency_thrust_cmd ──────────────────► MOD_2 (override thrust)
MOD_9 ── statustext_req ─────────────────────────► MOD_7 (STATUSTEXT broadcast)
```

---

## Design Considerations

- **Watchdog config lock:** The timeout register is shadow-latched; writes to `WATCHDOG_TIMEOUT` are committed only when `fsm_state == DISARMED`. This prevents runtime timeout changes from causing false expiry.
- **Emergency descent authority:** In EMERGENCY state, MOD_9 broadcasts a collective thrust override of 30% (fallback when EKF fully invalid) and the position-hold velocity setpoint to MOD_2. The PID controller still runs but uses the emergency setpoints.
- **STATUSTEXT timing:** The `statustext_req` pulse is asserted for exactly one ce_10hz cycle, giving the MAVLink TX mux a window to schedule the message within its next 100 ms slot.
- **No autonomous recovery:** EMERGENCY is a terminal state in hardware. Recovery requires a manual DISARM → rearm cycle from the GCS.
- **Resource estimate:**
  - DSP48E1: 0
  - BRAM18: 0
  - LUTs: ~300
  - FFs: ~250

---

## Testing & Verification

| Test Point                           | Verification Method                                           |
|--------------------------------------|---------------------------------------------------------------|
| Watchdog expiry at 500 ms            | Stop wdt_kick; count ce_100hz cycles; verify expiry at 50     |
| Watchdog kick resets timer           | Kick at 490 ms; verify timer resets, no expiry               |
| Config lock (non-DISARMED)           | Write WATCHDOG_TIMEOUT in ARMED state; verify register unchanged |
| PRE_FLIGHT_FAIL bit[0] on ekf_healthy=0 | Deassert ekf_healthy; verify bit[0]=1 and NAV_READY=0     |
| All checks pass → NAV_READY=1        | Assert all health signals; verify NAV_READY asserts          |
| Emergency thrust = 30%               | Trigger EMERGENCY; verify emergency_thrust_cmd ≈ 9 830       |
| STATUSTEXT broadcast                 | Trigger EMERGENCY; verify statustext_req pulses within 100 ms |
| Descent rate from register           | Set EMERG_DESCENT_RATE=4 m/s; verify velocity setpoint = −4  |

---

## Optimization Scope

| Area            | Opportunity                                                                | Impact  |
|-----------------|----------------------------------------------------------------------------|---------|
| **Resource**    | Combine preflight_checker and emergency_descent into single always_comb   | Low     |
| **Performance** | Evaluate all preflight checks in single cycle (parallel comparators)       | Low     |
| **Power**       | Gate watchdog counter increment when in DISARMED (no expiry risk)          | Low     |
| **Timing**      | All paths are simple counters and comparators; no critical paths expected  | None    |
| **Area**        | Share watchdog counter with tick_gen counter via clock divider chain        | Low     |

---

*Module 9 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
