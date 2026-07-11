# UAV Module 10: Sensor Calibration + AXI4-Lite Register Interface

## Overview

UAV Module 10 provides the unified host control and configuration interface for the entire UAV FPGA system. It implements a fully compliant AXI4-Lite slave that exposes 50+ control and status registers, covering PID gains, EKF tuning parameters, navigation settings, sensor calibration offsets, health status, telemetry thresholds, and watchdog configuration. All runtime-tunable parameters in all other modules are set and read through this module.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🟡 HIGH — Configuration interface for all subsystems; incorrect settings could degrade performance

---

## Key Functionality

- **AXI4-Lite slave** (ARM IHI0022E compliant), 50+ registers, 256-byte address space
- **18 PID gain registers** (Kp/Ki/Kd × 6 axes, Q4.12, resolution 0.000244)
- **9 EKF process noise Q diagonal** parameters (Q16.16 fixed-point)
- **Navigation parameters:** geofence radius/altitude, waypoint count/index, RTL home position
- **Sensor calibration:** IMU gyro/accel bias (Q4.28), magnetometer hard-iron offset (Q4.28)
- **Telemetry thresholds:** HDOP limit, velocity/position variance limits
- **Watchdog timeout** (100–2000 ms range)
- **Status/health flags** (read-only): EKF_HEALTHY, GPS_FIX, sensor faults, FSM state, pre-flight fail
- **Health metrics** (read-only): GPS_ACCURACY (HDOP×100), velocity/position variance
- **Atomic multi-register writes:** shadow registers latched on COMMIT write
- **4 flight-mode preset pages** in dual-port BRAM for instant gain switching

---

## Input Signals (AXI4-Lite)

| Signal Name   | Width  | Channel          | Description                              |
|---------------|--------|------------------|------------------------------------------|
| `ACLK`        | 1-bit  | —                | AXI/system clock (50 MHz)               |
| `ARESETn`     | 1-bit  | —                | AXI active-low reset                    |
| `AWADDR`      | 32-bit | Write Address    | Target register address                  |
| `AWVALID`     | 1-bit  | Write Address    | Write address valid                      |
| `AWREADY`     | 1-bit  | Write Address    | Slave ready for address                  |
| `WDATA`       | 32-bit | Write Data       | Register write value                     |
| `WSTRB`       | 4-bit  | Write Data       | Byte-enable strobes                      |
| `WVALID`      | 1-bit  | Write Data       | Write data valid                         |
| `WREADY`      | 1-bit  | Write Data       | Slave ready for data                     |
| `BRESP`       | 2-bit  | Write Response   | Response (00=OKAY, 10=SLVERR)            |
| `BVALID`      | 1-bit  | Write Response   | Response valid                           |
| `BREADY`      | 1-bit  | Write Response   | Master ready for response                |
| `ARADDR`      | 32-bit | Read Address     | Target register address                  |
| `ARVALID`     | 1-bit  | Read Address     | Read address valid                       |
| `ARREADY`     | 1-bit  | Read Address     | Slave ready for address                  |
| `RDATA`       | 32-bit | Read Data        | Register read value                      |
| `RRESP`       | 2-bit  | Read Data        | Response (00=OKAY)                       |
| `RVALID`      | 1-bit  | Read Data        | Read data valid                          |
| `RREADY`      | 1-bit  | Read Data        | Master ready for data                    |

### Status Inputs (from subsystems)

| Signal Name         | Width  | Source   | Description                          |
|---------------------|--------|----------|--------------------------------------|
| `ekf_healthy_in`    | 1-bit  | MOD_5/6  | EKF health status                    |
| `gps_fix_type_in`   | 4-bit  | MOD_6    | GPS fix type                         |
| `gps_hdop_in`       | 16-bit | MOD_6    | HDOP × 100                           |
| `fsm_state_in`      | 8-bit  | MOD_8    | Current FSM state (one-hot)          |
| `preflight_fail_in` | 8-bit  | MOD_9    | Pre-flight check failure bitmask     |
| `sensor_faults_in`  | 8-bit  | MOD_5/6  | Per-sensor fault flags               |
| `parse_crc_error_in`| 16-bit | MOD_7    | MAVLink CRC error counter            |
| `gcs_present_in`    | 1-bit  | MOD_7    | GCS HEARTBEAT present                |
| `sat_active_in`     | 4-bit  | MOD_3    | Motor output saturation flags        |

---

## Output Signals (to subsystems)

| Signal Name              | Width    | Format  | Destination  | Description                          |
|--------------------------|----------|---------|--------------|--------------------------------------|
| `pid_gains[17:0]`        | 16-bit ea| Q4.12   | MOD_2        | 18 PID gain registers                |
| `q_process_noise[8:0]`   | 32-bit ea| Q16.16  | MOD_4        | EKF process noise diagonal           |
| `geo_radius`             | 32-bit   | uint    | MOD_8        | Geofence radius (m)                  |
| `geo_max_alt`            | 32-bit   | uint    | MOD_8        | Geofence max altitude (m AGL)        |
| `wp_count`               | 8-bit    | uint    | MOD_8        | Mission waypoint count               |
| `imu_gyro_bias[2:0]`     | 32-bit ea| Q4.28   | MOD_5        | Gyro X/Y/Z bias correction           |
| `imu_accel_bias[2:0]`    | 32-bit ea| Q4.28   | MOD_5        | Accel X/Y/Z bias correction          |
| `mag_hard_iron[2:0]`     | 32-bit ea| Q4.28   | MOD_5        | Mag hard-iron offset X/Y/Z           |
| `baro_sea_level_p0`      | 32-bit   | Pa      | MOD_5        | Reference sea-level pressure         |
| `wdt_timeout`            | 16-bit   | ms      | MOD_9        | Watchdog timeout value               |
| `emerg_descent_rate`     | 32-bit   | Q4.28   | MOD_9        | Emergency descent rate               |

---

## Architecture

### High-Level Components

```
          ┌──────────────────────────────────────────────────────────┐
          │             module10_uav_axi_regif.sv                     │
          │                                                          │
AXI4-Lite►│  axi_slave.sv                                           │
          │  (AWADDR/WDATA/ARADDR/RDATA channels, full handshake)   │
          │           │                                              │
          │           ▼                                              │
          │  register_file.sv                                        │──► pid_gains
          │  (50+ registers: PID gains, EKF params, nav,            │──► q_process_noise
          │   sensor cal, thresholds, watchdog, status)              │──► sensor_cal
          │  (shadow register + COMMIT mechanism for atomic writes)  │──► nav_params
          │                                                          │
          │  system_orchestrator.sv                                  │
          │  (arbitrate multi-module access, BRAM dual-port mux)    │──► BRAM port grants
          │  (4 flight mode BRAM pages)                              │
          └──────────────────────────────────────────────────────────┘
```

### Reused Modules from RTL_20

*None — Module 10 is entirely new custom logic.*

### New Modules Created

| Module                    | Role                                                       |
|---------------------------|------------------------------------------------------------|
| `axi_slave.sv`            | AXI4-Lite protocol handler (all 5 channels)                |
| `register_file.sv`        | 50+ register array with read/write decode and shadow logic |
| `system_orchestrator.sv`  | Cross-module BRAM arbitration and register broadcast       |

---

## Data Formats

| Category           | Format   | Count  | Notes                                              |
|--------------------|----------|--------|----------------------------------------------------|
| PID gains          | Q4.12    | 18     | 0.000244 resolution, 0–15.999 range                |
| EKF process noise  | Q16.16   | 9      | Diagonal Q matrix values                           |
| Sensor calibration | Q4.28    | 9 + 1  | Gyro bias (3), Accel bias (3), Mag offset (3), P₀  |
| Navigation params  | 32-bit uint | 3   | GEO_RADIUS, GEO_MAX_ALT, WP_COUNT                  |
| Status/health      | logic/bitmask | 8+ | Read-only from subsystems                          |
| Thresholds         | Q4.28 / uint | 2  | HDOP limit, watchdog timeout                       |

---

## Register Map (AXI4-Lite, Base: 0x40000000)

### PID Gain Registers (0x00–0x44)

| Offset | Register Name    | Format | Description                    |
|--------|------------------|--------|--------------------------------|
| 0x00   | `KP_ROLL_RATE`   | Q4.12  | Roll rate Kp                   |
| 0x04   | `KI_ROLL_RATE`   | Q4.12  | Roll rate Ki                   |
| 0x08   | `KD_ROLL_RATE`   | Q4.12  | Roll rate Kd                   |
| 0x0C   | `KP_PITCH_RATE`  | Q4.12  | Pitch rate Kp                  |
| 0x10   | `KI_PITCH_RATE`  | Q4.12  | Pitch rate Ki                  |
| 0x14   | `KD_PITCH_RATE`  | Q4.12  | Pitch rate Kd                  |
| 0x18   | `KP_YAW_RATE`    | Q4.12  | Yaw rate Kp                    |
| 0x1C   | `KI_YAW_RATE`    | Q4.12  | Yaw rate Ki                    |
| 0x20   | `KD_YAW_RATE`    | Q4.12  | Yaw rate Kd                    |
| 0x24   | `KP_ROLL_ATT`    | Q4.12  | Roll attitude Kp               |
| 0x28   | `KI_ROLL_ATT`    | Q4.12  | Roll attitude Ki               |
| 0x2C   | `KD_ROLL_ATT`    | Q4.12  | Roll attitude Kd               |
| 0x30   | `KP_PITCH_ATT`   | Q4.12  | Pitch attitude Kp              |
| 0x34   | `KI_PITCH_ATT`   | Q4.12  | Pitch attitude Ki              |
| 0x38   | `KD_PITCH_ATT`   | Q4.12  | Pitch attitude Kd              |
| 0x3C   | `KP_ALT`         | Q4.12  | Altitude Kp                    |
| 0x40   | `KI_ALT`         | Q4.12  | Altitude Ki                    |
| 0x44   | `KD_ALT`         | Q4.12  | Altitude Kd                    |

### EKF Process Noise (0x48–0x68)

| Offset | Register    | Format  | Description             |
|--------|-------------|---------|-------------------------|
| 0x48   | `Q_ROLL`    | Q16.16  | Roll process noise      |
| 0x4C   | `Q_PITCH`   | Q16.16  | Pitch process noise     |
| 0x50   | `Q_YAW`     | Q16.16  | Yaw process noise       |
| 0x54   | `Q_VN`      | Q16.16  | North velocity noise    |
| 0x58   | `Q_VE`      | Q16.16  | East velocity noise     |
| 0x5C   | `Q_VD`      | Q16.16  | Down velocity noise     |
| 0x60   | `Q_LAT`     | Q16.16  | Latitude noise          |
| 0x64   | `Q_LON`     | Q16.16  | Longitude noise         |
| 0x68   | `Q_ALT`     | Q16.16  | Altitude noise          |

### Navigation Parameters (0x6C–0x84)

| Offset | Register         | Format  | Description                      |
|--------|------------------|---------|----------------------------------|
| 0x6C   | `GEO_RADIUS`     | uint32  | Geofence radius (m)              |
| 0x70   | `GEO_MAX_ALT`    | uint32  | Geofence max alt AGL (m)         |
| 0x74   | `WP_COUNT`       | uint8   | Mission waypoint count           |
| 0x78   | `WP_INDEX`       | uint8   | Active WP index (R/O)            |
| 0x7C   | `RTL_HOME_LAT`   | Q10.22  | Home latitude (R/O)              |
| 0x80   | `RTL_HOME_LON`   | Q10.22  | Home longitude (R/O)             |
| 0x84   | `RTL_HOME_ALT`   | Q10.22  | Home altitude (R/O)              |

### Sensor Calibration (0x88–0xBC)

| Offset | Register             | Format | Description                    |
|--------|----------------------|--------|--------------------------------|
| 0x88   | `IMU_GYRO_BIAS_X`    | Q4.28  | Gyro X bias                    |
| 0x8C   | `IMU_GYRO_BIAS_Y`    | Q4.28  | Gyro Y bias                    |
| 0x90   | `IMU_GYRO_BIAS_Z`    | Q4.28  | Gyro Z bias                    |
| 0x94   | `IMU_ACCEL_BIAS_X`   | Q4.28  | Accel X bias                   |
| 0x98   | `IMU_ACCEL_BIAS_Y`   | Q4.28  | Accel Y bias                   |
| 0x9C   | `IMU_ACCEL_BIAS_Z`   | Q4.28  | Accel Z bias                   |
| 0xA0   | `MAG_HARD_IRON_X`    | Q4.28  | Mag hard-iron X offset         |
| 0xA4   | `MAG_HARD_IRON_Y`    | Q4.28  | Mag hard-iron Y offset         |
| 0xA8   | `MAG_HARD_IRON_Z`    | Q4.28  | Mag hard-iron Z offset         |
| 0xAC   | `BARO_SEA_LEVEL_P0`  | uint32 Pa | Reference pressure            |
| 0xB0   | `WATCHDOG_TIMEOUT`   | uint16 ms | WDT period (100–2000 ms)     |
| 0xB4   | `EMERG_DESCENT_RATE` | Q4.28  | Emergency descent rate         |
| 0xB8   | `PID_GAINS_COMMIT`   | W/O    | Write to latch shadow gains    |
| 0xBC   | `FLIGHT_MODE_PRESET` | uint8  | Select gain preset (0–3)       |

### Status / Health Registers (0xC0–0xE4) — Read-Only

| Offset | Register            | Format  | Description                        |
|--------|---------------------|---------|------------------------------------|
| 0xC0   | `FSM_STATE`         | 8-bit OH| Current FSM state (one-hot)        |
| 0xC4   | `EKF_HEALTHY`       | 1-bit   | EKF health status                  |
| 0xC8   | `GPS_FIX_TYPE`      | 4-bit   | GPS fix type                       |
| 0xCC   | `SENSOR_FAULTS`     | 8-bit   | Per-sensor fault bitmask           |
| 0xD0   | `PREFLIGHT_FAIL`    | 8-bit   | Pre-flight check failure bits      |
| 0xD4   | `PARSE_CRC_ERROR`   | 16-bit  | MAVLink CRC error counter          |
| 0xD8   | `GCS_PRESENT`       | 1-bit   | GCS HEARTBEAT present              |
| 0xDC   | `NAV_READY`         | 1-bit   | All pre-flight checks passed       |
| 0xE0   | `GPS_ACCURACY`      | uint16  | HDOP × 100                         |
| 0xE4   | `SAT_ACTIVE`        | 4-bit   | Motor output saturation flags      |

---

## File Structure

```
Uav 1 to 12 Modules/module 10/
├── module10_uav_axi_regif.sv    # Top-level AXI register interface wrapper
├── axi_slave.sv                 # AXI4-Lite protocol handler (5 channels)
├── register_file.sv             # 50+ register array + shadow/commit logic
└── system_orchestrator.sv       # BRAM arbitration + multi-module broadcast
```

---

## Module Interconnections

```
Host (processor/MicroBlaze) ── AXI4-Lite ──────► MOD_10 (configuration)
MOD_10 ── pid_gains ─────────────────────────────► MOD_2 (PID controller)
MOD_10 ── q_process_noise ──────────────────────► MOD_4 (EKF predict)
MOD_10 ── sensor_calibration ────────────────────► MOD_5 (sensor interface)
MOD_10 ── nav_params / geo_* ────────────────────► MOD_8 (navigation FSM)
MOD_10 ── wdt_timeout / emerg_rate ─────────────► MOD_9 (safety supervisor)
All modules ── status/health ────────────────────► MOD_10 (status registers)
MOD_10 ── BRAM grants ───────────────────────────► MOD_11 (arbitration)
```

---

## Design Considerations

- **Atomic writes:** Shadow register pattern — individual gain writes update shadow copies; a write to `PID_GAINS_COMMIT` (0xB8) simultaneously latches all 18 shadows into the active registers in a single clock edge.
- **AXI4-Lite compliance:** Single outstanding transaction. AWVALID and WVALID may arrive in any order; the slave accepts both and responds only after both are valid per ARM IHI0022E.
- **Flight mode presets:** 4 pages in a dual-port BRAM18 (18 × 4 × 2 bytes = 144 bytes per page, 576 bytes total). Writing `FLIGHT_MODE_PRESET` triggers a bulk BRAM-to-register copy in ~20 cycles.
- **Write protection:** Status registers (0xC0–0xE4) return `SLVERR` on write attempts.
- **Resource estimate:**
  - DSP48E1: 0
  - BRAM18: 1 (flight mode gain presets)
  - LUTs: ~500
  - FFs: ~450

---

## Testing & Verification

| Test Point                          | Verification Method                                          |
|-------------------------------------|--------------------------------------------------------------|
| AXI write/read round-trip           | Write KP_ROLL_RATE, read back; verify same value             |
| Shadow/commit atomic update         | Write 18 gains individually; write COMMIT; verify all latch  |
| Write to R/O status register        | Write to FSM_STATE; verify BRESP=SLVERR                      |
| Flight mode preset switch           | Write FLIGHT_MODE_PRESET=2; verify all 18 gains change       |
| WDT timeout write lock (non-DISARMED)| Write WATCHDOG_TIMEOUT in ARMED; verify not updated         |
| HDOP status reflects MOD_6 input   | Toggle gps_hdop_in; read GPS_ACCURACY; verify match          |
| All 50+ registers accessible       | Sweep all valid addresses; verify OKAY response               |

---

## Optimization Scope

| Area            | Opportunity                                                                | Impact  |
|-----------------|----------------------------------------------------------------------------|---------|
| **Resource**    | Encode register file as BRAM instead of distributed LUTs for >40 registers | High    |
| **Performance** | Pipelining read path: address decode → BRAM output in 1 cycle              | Medium  |
| **Area**        | Merge system_orchestrator BRAM mux into register_file FSM                  | Medium  |
| **Power**       | Use BRAM for registers to reduce FF toggle activity on idle registers       | Medium  |
| **Timing**      | AXI RDATA registered to meet output timing requirement                     | Low     |

---

*Module 10 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
