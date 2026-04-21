# CS9 — Orbit Propagator (SGP4-Lite)

## 1. Module Title & ID

**Module:** CS9 — Orbit Propagator (SGP4-Lite)
**Subsystem ID:** CS9
**Requirements:** CS-ORB-001 through CS-ORB-007

---

## 2. Overview

CS9 provides a full 7-requirement orbit stack for a CubeSat in LEO. It accepts Two-Line Element (TLE) set data (either pre-parsed 32-bit elements or raw 69-character ASCII lines via a built-in TLE parser), propagates the orbital state at 1 Hz using a simplified SGP4 engine, and exports ECI position/velocity, LVLH reference frame, WGS84 geodetic coordinates, contact window predictions, and relative separation to secondary satellites. Position accuracy is ≤ 5 km over 24 h vs STK reference.

**Target Platform:** Xilinx Artix-7 XC7A35T

---

## 3. Criticality

**HIGH** — CS-ORB-001 to CS-ORB-007. Orbit data feeds telemetry (CS11), contact window scheduling, and provides the reference frame for ground track calculations. Loss of orbit data degrades ground contact prediction but does not trigger an ADCS FAULT state.

---

## 4. Key Functionality

- **TLE Input:** Accepts pre-parsed elements via `tle_data[0:5]` + `tle_write` strobe, or raw ASCII bytes via `tle_line1/2[0:68]` + `tle_raw_write` → `tle_parser`.
- **TLE Validation:** CRC checksum verification; `tle_checksum_ok` flag; `tle_stale` asserts when age > threshold.
- **SGP4-Lite Engine (`sgp4_lite`):** Propagates mean anomaly M += n0×dt each 1 Hz tick; true anomaly via first-order eccentricity correction; computes ECI position/velocity. ≤ 5 km error over 24 h for circular LEO.
- **LVLH Conversion (`lvlh_converter`):** ECI → LVLH rotation matrix; R_hat (radial), T_hat (transverse), N_hat (normal).
- **Ground Track (`ground_track_calculator`):** WGS84 lat/lon/alt from ECI; geodetic accuracy ± 0.1°.
- **Contact Window Prediction (`contact_window_predictor`):** Elevation angle; AOS/LOS prediction ≤ 30 s ahead.
- **MET Counter (`orbit_state_manager`):** 32-bit Mission Elapsed Time counter @ 1 Hz; GPS-syncable via `met_load_value + met_write`.
- **Multi-Satellite Tracker (`multi_satellite_tracker`):** Δr, Δv, and separation km between up to 3 satellites.
- **Orbit Health Monitor (`orbit_health_monitor`):** Position/velocity bounds checking; `overflow_flag` and `propagator_valid`.

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `clk` | input | 1 | `sys_clk` | 100 MHz system clock |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `ce_1hz` | input | 1 | `sys_clk` | 1 Hz clock-enable from CS12 |
| `tle_data[0:5]` | input | 6 × 32 | `sys_clk` | Pre-parsed TLE: [n0, e0, i0, RAAN, argp, M0] |
| `tle_write` | input | 1 | `sys_clk` | Strobe to latch pre-parsed TLE elements |
| `tle_line1[0:68]` | input | 69 × 8 | `sys_clk` | Raw ASCII TLE line 1 (69 chars) |
| `tle_line2[0:68]` | input | 69 × 8 | `sys_clk` | Raw ASCII TLE line 2 (69 chars) |
| `tle_raw_write` | input | 1 | `sys_clk` | Triggers `tle_parser` on raw ASCII lines |
| `met_load_value[31:0]` | input | 32 | `sys_clk` | MET preload value (seconds) |
| `met_write` | input | 1 | `sys_clk` | MET load strobe (GPS sync) |
| `gnd_lat_rad[31:0]` | input | 32 | `sys_clk` | Ground station latitude (Q15.16 rad) |
| `gnd_lon_rad[31:0]` | input | 32 | `sys_clk` | Ground station longitude (Q15.16 rad) |
| `gnd_alt_m[31:0]` | input | 32 | `sys_clk` | Ground station altitude (Q15.16 m) |
| `sat_id[1:0]` | input | 2 | `sys_clk` | Satellite pair ID: 0=1v2, 1=1v3, 2=2v3 |
| `sat2_pos[0:2]`, `sat2_vel[0:2]` | input | 6 × 32 | `sys_clk` | Secondary satellite 2 ECI state (Q15.16) |
| `sat3_pos[0:2]`, `sat3_vel[0:2]` | input | 6 × 32 | `sys_clk` | Secondary satellite 3 ECI state (Q15.16) |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `eci_pos[0:2]` | output | 3 × 32 | `sys_clk` | ECI position [X,Y,Z] in Q15.16 km |
| `eci_vel[0:2]` | output | 3 × 32 | `sys_clk` | ECI velocity [Vx,Vy,Vz] in Q15.16 km/s |
| `lvlh_x/y/z` | output | 32 each | `sys_clk` | LVLH R_hat components (Q15.16) |
| `lvlh_matrix[0:8]` | output | 9 × 32 | `sys_clk` | Full LVLH rotation matrix |
| `orbital_elements[0:5]` | output | 6 × 32 | `sys_clk` | [SMA, e, i, Ω, ω, ν] in Q15.16 |
| `true_anomaly` | output | 16 | `sys_clk` | True anomaly in Q0.15 rad |
| `met_counter` | output | 32 | `sys_clk` | Seconds since MET epoch |
| `epoch_tracked` | output | 32 | `sys_clk` | Current TLE epoch MJD |
| `latitude_rad` | output | 32 | `sys_clk` | Geodetic latitude (Q15.16 rad) |
| `longitude_rad` | output | 32 | `sys_clk` | Geodetic longitude (Q15.16 rad) |
| `altitude_m` | output | 32 | `sys_clk` | Altitude above WGS84 ellipsoid (Q15.16 m) |
| `ground_track_valid` | output | 1 | `sys_clk` | Ground track outputs valid |
| `elevation_angle_deg` | output | 16 | `sys_clk` | Elevation to ground station (Q8.8 degrees) |
| `contact_valid` | output | 1 | `sys_clk` | Contact window active |
| `aos_predicted_secs` | output | 16 | `sys_clk` | Seconds to AOS |
| `los_predicted_secs` | output | 16 | `sys_clk` | Seconds to LOS |
| `delta_r_eci[0:2]` | output | 3 × 32 | `sys_clk` | Δr to secondary satellite (Q15.16 km) |
| `delta_v_eci[0:2]` | output | 3 × 32 | `sys_clk` | Δv to secondary satellite (Q15.16 km/s) |
| `separation_km` | output | 32 | `sys_clk` | Separation distance (Q15.16 km) |
| `orb_valid` | output | 1 | `sys_clk` | ECI position/velocity valid |
| `orb_fault` | output | 1 | `sys_clk` | Propagation fault |
| `propagator_valid` | output | 1 | `sys_clk` | Full propagator stack valid |
| `overflow_flag` | output | 1 | `sys_clk` | Arithmetic overflow in propagation |
| `tle_age_hours` | output | 16 | `sys_clk` | TLE age in hours |
| `tle_stale` | output | 1 | `sys_clk` | TLE age exceeded maximum |
| `tle_checksum_ok` | output | 1 | `sys_clk` | TLE checksum valid |
| `position_magnitude_km` | output | 32 | `sys_clk` | |r| (Q15.16 km) |
| `velocity_magnitude_kmps` | output | 32 | `sys_clk` | |v| (Q15.16 km/s) |

---

## 7. Architecture

```
tle_line1/2 + tle_raw_write
      │
      ▼
┌─────────────┐  tle_elements  ┌───────────────────────────────────────┐
│  tle_parser │───────────────▶│  sgp4_lite                            │
└─────────────┘                │  (mean anomaly propagation,           │
                               │   true anomaly correction,            │
tle_data + tle_write ──────────│   ECI r + v, orbital elements)        │
                               └──────────────┬────────────────────────┘
                                              │ eci_pos, eci_vel
                              ┌───────────────▼────────────┐
                              │  lvlh_converter             │
                              │  (ECI → LVLH rotation matrix│
                              └─────────────────────────────┘
                              ┌───────────────────────────────────┐
                              │  ground_track_calculator           │
                              │  (WGS84 lat/lon/alt, elevation)   │
                              └───────────────────────────────────┘
                              ┌───────────────────────────────────┐
                              │  contact_window_predictor          │
                              │  (AOS/LOS prediction, contact det.)│
                              └───────────────────────────────────┘
                              ┌───────────────────────────────────┐
                              │  orbit_state_manager               │
                              │  (MET counter, epoch tracking)    │
                              └───────────────────────────────────┘
                              ┌───────────────────────────────────┐
                              │  multi_satellite_tracker           │
                              │  (Δr, Δv, separation km)          │
                              └───────────────────────────────────┘
                              ┌───────────────────────────────────┐
                              │  orbit_health_monitor              │
                              │  (bounds check, overflow flag)    │
                              └───────────────────────────────────┘
```

**Reused Helper IPs (from `CubeSat/`):**
- `cordic.sv` — Sine/cosine for orbital mechanics (anomaly conversion, LVLH rotation)
- `sqrt.sv` — Square root for distance and magnitude calculations
- `fp_divider.sv` — Fixed-point division for eccentricity correction and altitude

---

## 8. Data Formats

| Signal | Format | Notes |
|---|---|---|
| `tle_data[0]` (n0) | Q15.16 rad/s | Mean motion; LEO ≈ 0x00016800 |
| `tle_data[1]` (e0) | Q0.15 (16-bit in 32-bit field) | Eccentricity 0–1 |
| `tle_data[2:5]` (i0, RAAN, argp, M0) | Q0.15 rad | Angles 0–π → 0–32767 |
| `eci_pos[0:2]` | Q15.16 km (32-bit) | ≈ 6778 km for 400 km LEO |
| `eci_vel[0:2]` | Q15.16 km/s (32-bit) | ≈ 7.5 km/s LEO |
| `lvlh_x/y/z`, `lvlh_matrix` | Q15.16 (32-bit) | Unit vectors; ≈ ±1.0 |
| `latitude_rad`, `longitude_rad` | Q15.16 rad (32-bit) | ±π/2, ±π |
| `altitude_m` | Q15.16 m (32-bit) | Typically 350 000–500 000 m |
| `elevation_angle_deg` | Q8.8 (16-bit) | 0–90 degrees |
| `separation_km` | Q15.16 km (32-bit) | — |

---

## 9. Register Interface

CS9 has **no AXI4-Lite register interface** in the current implementation. TLE data is written via the `tle_write` or `tle_raw_write` strobe interface. MET is loaded via `met_write`. Ground station coordinates and satellite IDs are direct port inputs.

CS9 has no user-configurable top-level parameters; all constants are embedded as `localparam` inside the respective sub-modules (`sgp4_lite`, `lvlh_converter`, etc.).

---

## 10. File Structure

```
CubeSat/CS9_ORBIT/
├── orbit_propagator_wrapper.sv    ← Top-level wrapper; CS12 integration point
├── sgp4_lite.sv                   ← SGP4-Lite engine: ECI r + v @ 1 Hz
├── lvlh_converter.sv              ← ECI → LVLH frame transformation
├── ground_track_calculator.sv     ← WGS84 geodetic lat/lon/alt
├── contact_window_predictor.sv    ← AOS/LOS prediction, elevation angle
├── orbit_state_manager.sv         ← MET counter, epoch management
├── multi_satellite_tracker.sv     ← Relative ECI motion for 3-sat formation
├── orbit_health_monitor.sv        ← Bounds checking, overflow detection
├── tle_parser.sv                  ← Raw ASCII TLE validation and parsing
├── tb_orbit_propagator_wrapper.sv ← Integration testbench
└── README.md                      ← This file

CubeSat/ (shared helper IPs used by CS9):
├── cordic.sv                      ← CORDIC engine
├── sqrt.sv                        ← Square root calculator
└── fp_divider.sv                  ← Fixed-point divider
```

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `ce_1hz` | CS9 ← CS12 | `clk_manager` (CS12) | 1 Hz propagation trigger |
| `tle_data`, `tle_write` | CS9 ← CS11 | `telemetry_wrapper` (CS11) | TLE uplink via command decoder |
| `eci_pos/vel` | CS9 → CS11 | `telemetry_wrapper` (CS11) | Orbit telemetry (APID 0x0102) |
| `lvlh_x/y/z` | CS9 → CS11 | `telemetry_wrapper` (CS11) | LVLH reference frame telemetry |
| `met_counter` | CS9 → CS11 | `telemetry_wrapper` (CS11) | Frame timestamp source |
| `orb_valid` | CS9 → CS12 | `top_cubesat_mvp` (CS12) | System-level orbit valid flag |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- Propagation budget: 50 ms (plan); at 1 Hz the full 1-second window is available.
- LVLH conversion: 10 ms (plan). CORDIC iterations take ~16 cycles each.
- Ground track + AOS: 5 ms each. All within 1-second cycle budget.

**Resource:**
- DSP48E1: 8 (per plan) for CORDIC multiply-accumulate and SGP4 polynomial evaluations.
- BRAM: 1,536 B (per plan) for TLE storage, MET buffer, orbital element tables.

**Optimization Opportunities:**
1. Add higher-order SGP4 perturbation terms (J2–J4 drag, solar pressure) for reduced error.
2. Increase CORDIC iteration count from 16 to 24 for improved angular accuracy.
3. Store full 3D rotation (RAAN/inclination) instead of equatorial-plane simplification.
4. Use AXI4-Lite registers to expose TLE write interface for standardized command routing.

**Timing:**
- Critical path: CORDIC pipeline chain in `lvlh_converter`. Multi-cycle constraint recommended.
- `fp_divider` operations may require multi-cycle timing constraints.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS9_ORBIT/tb_orbit_propagator_wrapper.sv`

**Test Scenarios:**
- Load ISS TLE; advance 90 minutes at 1 Hz; verify `orb_valid` and position magnitude ≈ 6,785 km.
- Verify `lvlh_x/y/z` unit vectors have magnitude ≈ 1.0 (within Q15.16 resolution).
- Write raw ASCII TLE via `tle_raw_write`; verify `tle_checksum_ok` and correct element parsing.
- Set `gnd_lat_rad/lon_rad`; verify `contact_valid` during pass and `elevation_angle_deg` > 0 when above horizon.
- Write `met_load_value = 1000`; verify `met_counter` starts at 1000 and increments at 1 Hz.
- Apply two satellite positions; verify `separation_km` matches manual calculation.
- Verify `tle_stale` asserts after TLE age exceeds threshold.

**Simulation Notes:**
- Compile with `iverilog -g2012` including `cordic.sv`, `sqrt.sv`, `fp_divider.sv`.
- Timescale: 1 ns / 1 ps.
- Use STK-generated reference trajectory for numerical accuracy comparison (≤ 5 km).

**Requirements Coverage:**
- CS-ORB-001: TLE input (69-char ASCII), checksum validation, epoch to MJD.
- CS-ORB-002: SGP4-Lite ECI propagation, ≤ 5 km error over 24 h vs STK.
- CS-ORB-003: LVLH conversion, classical orbital elements.
- CS-ORB-004: WGS84 altitude, geodetic lat/lon ± 0.1°.
- CS-ORB-005: 32-bit MET counter @ 1 Hz, GPS-syncable.
- CS-ORB-006: Relative position/velocity for ISL (Δr, Δv).
- CS-ORB-007: AOS/LOS prediction ≤ 30 s.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
