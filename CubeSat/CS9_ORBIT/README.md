# CS9 — Orbit Propagator (SGP4-lite)

> Propagates spacecraft position and velocity in ECI frame at 1 Hz using an SGP4-lite algorithm, with TLE parsing, LVLH frame conversion, ground-track computation, contact-window prediction, and inter-satellite relative motion tracking.

---

## Overview

| Attribute | Value |
|---|---|
| Requirements | CS-ORB-001 through CS-ORB-007 |
| Top module | `orbit_propagator_wrapper` |
| Clock domain | `sys_clk` (50 MHz) |
| CE strobe | 1 Hz `ce_1hz` from CS12 |
| Position format | Q15.16 km (ECI) |
| Velocity format | Q15.16 km/s (ECI) |
| MET counter | 32-bit seconds since epoch |
| BRAM | 1,536 B |
| DSP48 | 8 |

---

## File Structure

| File | Purpose |
|---|---|
| `orbit_propagator_wrapper.sv` | Top-level — integrates all CS9 modules |
| `sgp4_lite.sv` | SGP4-lite propagation core: ECI pos/vel from mean elements |
| `lvlh_converter.sv` | ECI → LVLH frame rotation matrix (r̂, θ̂, ĥ) |
| `orbit_state_manager.sv` | MET counter, epoch tracking, TLE age monitoring |
| `orbit_health_monitor.sv` | Position/velocity bounds check; `orb_valid` / `orb_fault` / `overflow_flag` |
| `ground_track_calculator.sv` | ECI → geodetic lat / lon / altitude |
| `contact_window_predictor.sv` | Elevation angle to ground station; AOS/LOS detection |
| `multi_satellite_tracker.sv` | ΔR, ΔV between sat-1/2/3 pairs |
| `tb_orbit_propagator_wrapper.sv` | Self-checking directed testbench |

Shared primitives used: `cordic.sv`, `sqrt.sv`, `fp_divider.sv`.

---

## Module Interface

```systemverilog
module orbit_propagator_wrapper (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_1hz,                // 1 Hz propagation tick

    // Pre-parsed TLE element bus
    input  logic [31:0] tle_data [0:5],        // [n0(Q15.16), e0, i0, raan, argp, m0]
    input  logic        tle_write,             // write strobe for parsed elements

    // Primary ECI outputs (Q15.16)
    output logic [31:0] eci_pos [0:2],         // km
    output logic [31:0] eci_vel [0:2],         // km/s

    // LVLH frame
    output logic [31:0] lvlh_x, lvlh_y, lvlh_z,        // unit vectors Q15.16
    output logic [31:0] lvlh_matrix [0:8],              // full rotation matrix

    // Orbital elements
    output logic [31:0] orbital_elements [0:5],         // [SMA,e,i,Ω,ω,ν] Q15.16
    output logic [15:0] true_anomaly,

    // State management
    output logic [31:0] met_counter,           // seconds since MET epoch
    output logic [31:0] epoch_tracked,

    // Ground track
    output logic [31:0] latitude_rad,          // Q15.16
    output logic [31:0] longitude_rad,
    output logic [31:0] altitude_m,
    output logic        ground_track_valid,

    // Contact window
    output logic [15:0] elevation_angle_deg,   // Q8.8
    output logic        contact_active,
    output logic [31:0] aos_time_sec,

    // Status
    output logic        orb_valid,
    output logic        orb_fault,
    output logic        propagator_valid,
    output logic        overflow_flag,

    // Raw ASCII TLE path
    input  logic [7:0]  tle_line1 [0:68],
    input  logic [7:0]  tle_line2 [0:68],
    input  logic        tle_raw_write,

    // MET load
    input  logic [31:0] met_load_value,
    input  logic        met_write,

    // Ground station
    input  logic [31:0] gnd_lat_rad, gnd_lon_rad, gnd_alt_m,

    // Inter-satellite tracking
    input  logic [1:0]  sat_id,
    input  logic [31:0] sat2_pos [0:2], sat2_vel [0:2],
    input  logic [31:0] sat3_pos [0:2], sat3_vel [0:2]
);
```

---

## Functionality

1. **TLE ingestion** — `tle_raw_write` triggers `tle_parser` to decode 69-char ASCII TLE lines into mean elements and write them into `orbit_state_manager`. Alternatively, pre-parsed elements can be written directly via `tle_write`.
2. **Propagation** (`sgp4_lite`) — on each `ce_1hz`, integrates mean motion `n0`, eccentricity, inclination, RAAN, argument of perigee, and mean anomaly to produce ECI position and velocity in Q15.16 km / km/s.
3. **LVLH** (`lvlh_converter`) — computes radial, tangential, and orbit-normal unit vectors; builds full 3×3 rotation matrix.
4. **Ground track** (`ground_track_calculator`) — converts ECI to geodetic lat/lon/altitude using iterative Bowring method.
5. **Contact windows** (`contact_window_predictor`) — computes elevation angle to the configured ground station; asserts `contact_active` above the horizon mask angle.
6. **Inter-satellite geometry** (`multi_satellite_tracker`) — computes ΔR and ΔV for up to 3 satellite pairs.
7. **Health** (`orbit_health_monitor`) — bounds-checks `|eci_pos|` (target ≤ 8,000 km) and `|eci_vel|` (≤ 10 km/s); asserts `overflow_flag` on violation, `orb_valid` when propagation is current.

---

## Simulation Instructions

```bash
iverilog -g2012 -o sim_cs9 \
  CS9_ORBIT/tb_orbit_propagator_wrapper.sv \
  CS9_ORBIT/orbit_propagator_wrapper.sv \
  CS9_ORBIT/sgp4_lite.sv \
  CS9_ORBIT/lvlh_converter.sv \
  CS9_ORBIT/orbit_state_manager.sv \
  CS9_ORBIT/orbit_health_monitor.sv \
  CS9_ORBIT/ground_track_calculator.sv \
  CS9_ORBIT/contact_window_predictor.sv \
  CS9_ORBIT/multi_satellite_tracker.sv \
  cordic.sv sqrt.sv fp_divider.sv

vvp sim_cs9
```

```tcl
vlog -sv CS9_ORBIT/tb_orbit_propagator_wrapper.sv CS9_ORBIT/*.sv cordic.sv sqrt.sv fp_divider.sv
vsim -t 1ps tb_orbit_propagator_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 50 MHz; CE at 1 Hz internally generated |
| Stimulus | Pre-computed TLE elements from a reference ISS-like orbit |
| Checking | ECI position within ±5 km of STK/Vallado SGP4 reference after 24 h |
| Coverage | TLE load, propagation step, LVLH construction, ground-track validity, overflow detection, contact detection |

---

## Expected Behaviour

```
ce_1hz          __|‾|______________|‾|_______
tle_write        |‾| (one-shot at init)
orb_valid        ________|‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾  (asserts once TLE loaded and first propagation done)
eci_pos[0..2]:   updated each second; |eci_pos| ≈ 6,370–8,000 km LEO
met_counter:     0, 1, 2, … incrementing each ce_1hz
contact_active:  ___|‾‾‾‾‾‾‾|___  (elevation > mask when pass overhead)
overflow_flag:   ___________ (stays low in nominal LEO)
```

---

## Limitations

- SGP4-lite omits high-order drag and deep-space resonance terms; positional accuracy degrades beyond ~24 h.
- Assumes near-circular LEO; highly elliptical orbits (e > 0.1) may exceed fixed-point dynamic range.
- TLE parser validates checksum but does not enforce epoch age limit.
- Ground track uses a simplified spherical Earth model; WGS-84 oblateness not fully modelled.

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] ECI position within ±5 km of reference after one orbit period
- [x] LVLH unit vectors orthonormal (dot-product test)
- [x] Ground track latitude within ±0.1° of reference
- [x] Contact-window elevation angle verified for known pass
- [x] `overflow_flag` asserts on out-of-range injection
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] 24-hour drift characterisation vs. Vallado SGP4
- [ ] Synthesis DSP48 count confirmed (target: 8)
