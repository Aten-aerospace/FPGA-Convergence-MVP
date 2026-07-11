# UAV Module 8: Navigation FSM (8 States)

## Overview

UAV Module 8 implements the complete autonomous flight state machine (FSM) for the UAV. It manages all eight flight states from ground power-on through mission execution to emergency handling, dispatches PID setpoints to the control loop, supervises waypoint sequencing from a 32-waypoint BRAM mission, enforces a cylindrical geofence boundary, and outputs the current FSM state for telemetry and actuator arming/disarming.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🔴 CRITICAL — Controls all flight modes; incorrect transitions can cause loss of vehicle

---

## Key Functionality

### Flight State Machine (One-Hot, 8-bit)

| State        | One-Hot | Behavior                                                             |
|--------------|---------|----------------------------------------------------------------------|
| `DISARMED`   | 0x01    | Motor inhibit; pre-flight checklist running                          |
| `ARMED`      | 0x02    | Motors idle at 1 050 µs; awaiting command                           |
| `TAKEOFF`    | 0x04    | Autonomous climb ≤ 2 m/s to target altitude                         |
| `EN_ROUTE`   | 0x08    | Waypoint following via bearing/distance guidance                     |
| `LOITER`     | 0x10    | Hold fixed NED position                                              |
| `LAND`       | 0x20    | Controlled descent 0.5 m/s; flare < 2 m; ground detect < 0.3 m     |
| `RTL`        | 0x40    | Return-to-home at minimum 30 m altitude                              |
| `EMERGENCY`  | 0x80    | Descent 2 m/s holding position; no autonomous recovery              |

### Transition Guards
- `DISARMED → ARMED`: requires `NAV_READY` (pre-flight pass from MOD_9)
- `ARMED → TAKEOFF`: requires `cmd_takeoff` from MAVLink
- `TAKEOFF → EN_ROUTE`: altitude ramp complete (EKF alt ≥ target − 0.5 m)
- `EN_ROUTE → LOITER`: last waypoint reached
- `ANY → LAND`: `cmd_land` from MAVLink
- `ANY → RTL`: `cmd_rtl` or GCS loss while EN_ROUTE or geofence breach
- `ANY → EMERGENCY`: `watchdog_expired` or sensor fault

### Waypoint Manager
- **32 MAVLink-compatible waypoints** in BRAM (34 B each = 1 088 B total)
- Fields: lat/lon/alt (Q10.22), command, param1–4, frame, autocontinue
- Mission upload via `MISSION_COUNT + MISSION_ITEM_INT`
- In-flight waypoint modification without restart
- 3D distance/bearing to active WP via CORDIC

### Geofence
- **Cylindrical boundary** checked at 100 Hz
- Center: home position (lat/lon captured on ARM)
- Radius: `GEO_RADIUS` AXI register (default 200 m)
- Max altitude: `GEO_MAX_ALT` AXI register (default 100 m AGL)
- Breach → immediate RTL within 1 cycle

---

## Input Signals

| Signal Name          | Width  | Format  | Source           | Description                              |
|----------------------|--------|---------|------------------|------------------------------------------|
| `clk`                | 1-bit  | —       | MOD_1            | 50 MHz system clock                      |
| `rst_n`              | 1-bit  | —       | MOD_1            | Active-low synchronous reset             |
| `ce_100hz`           | 1-bit  | —       | MOD_1            | FSM update strobe                        |
| `cmd_arm`            | 1-bit  | —       | MOD_7            | ARM command from MAVLink dispatcher      |
| `cmd_disarm`         | 1-bit  | —       | MOD_7            | DISARM command                           |
| `cmd_takeoff`        | 1-bit  | —       | MOD_7            | TAKEOFF command                          |
| `altitude_target`    | 32-bit | Q10.22  | MOD_7            | Target altitude for TAKEOFF              |
| `cmd_land`           | 1-bit  | —       | MOD_7            | LAND command                             |
| `cmd_rtl`            | 1-bit  | —       | MOD_7            | RTL command                              |
| `cmd_set_mode`       | 3-bit  | —       | MOD_7            | Flight mode selector                     |
| `cmd_waypoint_valid` | 1-bit  | —       | MOD_7            | New waypoint data available              |
| `cmd_waypoint_data`  | 272-bit| mixed   | MOD_7            | MISSION_ITEM_INT fields                  |
| `gcs_present`        | 1-bit  | —       | MOD_7            | GCS HEARTBEAT present within 3 s         |
| `ekf_healthy`        | 1-bit  | —       | MOD_5/6          | EKF health flag                          |
| `gps_fix_valid`      | 1-bit  | —       | MOD_6            | GPS 3D fix with HDOP ≤ 2.5               |
| `ekf_lat`            | 32-bit | Q10.22  | MOD_5/6          | Current estimated latitude               |
| `ekf_lon`            | 32-bit | Q10.22  | MOD_5/6          | Current estimated longitude              |
| `ekf_alt`            | 32-bit | Q10.22  | MOD_5/6          | Current estimated altitude               |
| `nav_ready`          | 1-bit  | —       | MOD_9            | Pre-flight all checks passed             |
| `watchdog_expired`   | 1-bit  | —       | MOD_9            | Watchdog timeout → force EMERGENCY       |
| `preflight_fail`     | 8-bit  | bitmask | MOD_9            | Pre-flight check failure bitmask         |

---

## Output Signals

| Signal Name            | Width  | Format  | Destination     | Description                                     |
|------------------------|--------|---------|-----------------|--------------------------------------------------|
| `fsm_state`            | 8-bit  | one-hot | MOD_7, MOD_2, MOD_3 | Current flight state                        |
| `setpoint_roll`        | 32-bit | Q4.28   | MOD_2           | Roll attitude target (rad)                       |
| `setpoint_pitch`       | 32-bit | Q4.28   | MOD_2           | Pitch attitude target (rad)                      |
| `setpoint_altitude`    | 32-bit | Q10.22  | MOD_2           | Altitude target (m)                              |
| `setpoint_vn`          | 32-bit | Q4.28   | MOD_2           | North-velocity target (m/s)                      |
| `setpoint_ve`          | 32-bit | Q4.28   | MOD_2           | East-velocity target (m/s)                       |
| `arm_motors`           | 1-bit  | —       | MOD_3           | 1 = Armed (normal PWM enabled)                   |
| `active_wp_index`      | 8-bit  | unsigned| MOD_9           | Current active waypoint index                    |
| `home_lat`             | 32-bit | Q10.22  | MOD_9 (RTL)     | Home latitude captured on ARM                    |
| `home_lon`             | 32-bit | Q10.22  | MOD_9 (RTL)     | Home longitude captured on ARM                   |
| `home_alt`             | 32-bit | Q10.22  | MOD_9 (RTL)     | Home altitude captured on ARM                    |

---

## Architecture

### High-Level Components

```
          ┌──────────────────────────────────────────────────────────┐
          │              module8_uav_nav_fsm.sv                      │
          │                                                          │
cmds ────►│  fsm_controller.sv                                      ├──► fsm_state
guards ──►│  (one-hot 8-state, 100Hz, transition guards)            ├──► setpoints
          │                                                          │
wp_data ►│  waypoint_manager.sv                                     │
          │  (32×34B BRAM, MISSION_ITEM_INT upload, CORDIC distance) │
          │                                                          │
position►│  geofence_checker.sv                                     ├──► fsm_rtl_trigger
          │  (cylindrical boundary, home center, 100Hz check)        │
          └──────────────────────────────────────────────────────────┘
```

### Reused Modules from RTL_20

| Module        | Role                                                              |
|---------------|-------------------------------------------------------------------|
| `tick_gen.sv` | 1 ms timing for sub-100 Hz timing (e.g., landing flare detection) |
| `cordic.sv`   | Distance/bearing calculation to waypoints                         |

### New Modules Created

| Module                 | Role                                                      |
|------------------------|-----------------------------------------------------------|
| `fsm_controller.sv`    | One-hot 8-state FSM with all transition guards            |
| `waypoint_manager.sv`  | 32-WP BRAM manager, MISSION_ITEM_INT upload, sequencer    |
| `geofence_checker.sv`  | Cylindrical boundary check, configurable radius/altitude  |

---

## Data Formats

| Parameter         | Format   | Range           | Notes                                 |
|-------------------|----------|-----------------|---------------------------------------|
| FSM state         | 8-bit one-hot | 8 valid values | One-hot encoding, single bit asserted |
| Setpoint angles   | Q4.28    | ±π rad          | Roll/pitch attitude targets           |
| Setpoint altitude | Q10.22   | 0–10 000 m      | Altitude target for TAKEOFF / LOITER  |
| Setpoint velocity | Q4.28    | ±15 m/s         | N/E velocity targets for EN_ROUTE     |
| WP lat/lon/alt    | Q10.22   | ±180°/±360°/0–10000 m | BRAM-stored waypoint fields    |
| Geofence radius   | 32-bit unsigned | 0–65 535 m | Default 200 m, AXI configurable    |
| Transition log    | 96-bit entry  | 32 entries   | 8-bit state + 64-bit timestamp (× 32) |

---

## Register Interface (AXI4-Lite via MOD_10)

| Register Name        | Address (MOD_10) | Access | Format      | Description                         |
|----------------------|------------------|--------|-------------|-------------------------------------|
| `GEO_RADIUS`         | 0x6C             | R/W    | 32-bit uint | Geofence radius (default 200 m)     |
| `GEO_MAX_ALT`        | 0x70             | R/W    | 32-bit uint | Max altitude AGL (default 100 m)    |
| `WP_COUNT`           | 0x74             | R/W    | 8-bit uint  | Number of waypoints in mission      |
| `WP_INDEX`           | 0x78             | R/O    | 8-bit uint  | Current active waypoint index       |
| `RTL_HOME_LAT`       | 0x7C             | R/O    | Q10.22      | Home latitude (captured on ARM)     |
| `RTL_HOME_LON`       | 0x80             | R/O    | Q10.22      | Home longitude (captured on ARM)    |
| `RTL_HOME_ALT`       | 0x84             | R/O    | Q10.22      | Home altitude (captured on ARM)     |
| `FSM_STATE`          | 0xC0             | R/O    | 8-bit one-hot | Current FSM state                 |

---

## File Structure

```
Uav 1 to 12 Modules/module 8/
├── module8_uav_nav_fsm.sv    # Top-level navigation FSM wrapper
├── fsm_controller.sv         # 8-state one-hot FSM logic
├── waypoint_manager.sv       # 32-WP BRAM manager + mission sequencer
└── geofence_checker.sv       # Cylindrical boundary enforcement
```

### Dependencies (from RTL_20 shared library)

```
tick_gen.sv    # 1 ms tick for sub-100 Hz timing (landing flare)
cordic.sv      # Distance/bearing to waypoint computation
```

---

## Module Interconnections

```
MOD_7 ── cmd_arm/disarm/takeoff/land/rtl/mode ──► MOD_8 (flight commands)
MOD_7 ── gcs_present ───────────────────────────► MOD_8 (failsafe trigger)
MOD_9 ── nav_ready / watchdog_expired ──────────► MOD_8 (ARM guard / emergency)
MOD_5/6 ── ekf_position / ekf_healthy ──────────► MOD_8 (state feedback)
MOD_6 ── gps_fix_valid ─────────────────────────► MOD_8 (ARM guard)
MOD_8 ── fsm_state ─────────────────────────────► MOD_7 (telemetry)
MOD_8 ── fsm_state ─────────────────────────────► MOD_3 (arm/disarm interlock)
MOD_8 ── setpoints ─────────────────────────────► MOD_2 (PID targets)
MOD_8 ── active_wp_index ───────────────────────► MOD_9 (health monitor)
MOD_8 ── home_lat/lon/alt ──────────────────────► MOD_9 (RTL target)
MOD_12 (BRAM) ─── wp_bram ──────────────────────► MOD_8 (waypoint storage)
```

---

## Design Considerations

- **One-hot encoding:** Chosen for fast state decode in hardware. Any two bits asserted simultaneously indicates a fault (detectable in synthesis).
- **BRAM waypoint storage:** 32 × 34 bytes = 1 088 bytes fits in one BRAM18 (2 048 byte capacity). Single-cycle read latency.
- **CORDIC distance computation:** Euclidean distance from (lat, lon, alt) to waypoint (lat_wp, lon_wp, alt_wp) in NED. Bearing angle computed for velocity setpoint generation in EN_ROUTE.
- **Transition log:** Last 32 state transitions with millisecond timestamps stored in circular BRAM buffer for post-mortem analysis.
- **Minimum RTL altitude:** RTL climbs to max(current_alt, 30 m) before heading home to avoid obstacles.
- **Resource estimate:**
  - DSP48E1: 2–4 (CORDIC distance metric)
  - BRAM18: 2 (1 × waypoints + 1 × transition log)
  - LUTs: ~700
  - FFs: ~500

---

## Testing & Verification

| Test Point                           | Verification Method                                          |
|--------------------------------------|--------------------------------------------------------------|
| DISARMED → ARMED transition          | Assert nav_ready=1, cmd_arm=1; verify fsm_state=0x02         |
| ARMED → TAKEOFF guard                | cmd_takeoff without nav_ready; verify state stays ARMED      |
| TAKEOFF altitude ramp                | Simulate altitude climb; verify EN_ROUTE when target reached |
| Geofence breach → RTL                | Set position outside GEO_RADIUS; verify fsm_state=0x40       |
| GCS loss → RTL while EN_ROUTE        | Deassert gcs_present; verify RTL within 1 ce_100hz cycle     |
| Waypoint upload and sequencing       | Upload 3 WPs; verify WP_INDEX advances on each reach         |
| EMERGENCY on watchdog expiry         | Assert watchdog_expired; verify fsm_state=0x80 in 1 cycle    |
| Home position capture on ARM         | Verify home_lat/lon/alt matches EKF position at ARM event    |

---

## Optimization Scope

| Area            | Opportunity                                                                | Impact  |
|-----------------|----------------------------------------------------------------------------|---------|
| **Resource**    | Store only upper triangle of CORDIC bearing matrix (2 angles vs 9)         | Medium  |
| **Performance** | Pre-compute bearing/distance at 100 Hz concurrently with FSM update        | Medium  |
| **Power**       | Gate geofence_checker when in DISARMED/EMERGENCY state                     | Low     |
| **Timing**      | Register geofence check output before FSM transition logic                  | Low     |
| **Area**        | Share single BRAM port between waypoints and transition log via time-mux   | Medium  |

---

*Module 8 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
