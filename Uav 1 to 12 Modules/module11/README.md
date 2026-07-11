# UAV Module 11: Cross-Module Interconnect

## Overview

UAV Module 11 is the structural interconnect and resource arbitration layer of the UAV FPGA system. It distributes the four CE clock strobes from MOD_1 to all downstream modules, arbitrates shared dual-port BRAM resources among the EKF engine, waypoint manager, and AXI register interface, and routes all cross-module data signals with the correct latency. Module 11 has no registers exposed to the AXI interface; it is a transparent routing and arbitration fabric.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🟡 HIGH — Incorrect arbitration could cause BRAM read corruption or missed CE strobes

---

## Key Functionality

### CE Strobe Arbitration
Distributes MOD_1 CE strobes with per-module enable gating:

| Strobe     | Recipients                                   | Rate   |
|------------|----------------------------------------------|--------|
| `ce_1khz`  | MOD_2 (PID inner rate loop)                  | 1 kHz  |
| `ce_100hz` | MOD_2, MOD_3, MOD_4, MOD_6, MOD_7, MOD_8, MOD_9, MOD_11 | 100 Hz |
| `ce_50hz`  | MOD_5 (barometer sampling)                   | 50 Hz  |
| `ce_10hz`  | MOD_5, MOD_6, MOD_7, MOD_10, MOD_11         | 10 Hz  |

### BRAM Port Arbitration
Multiplexes three shared BRAM resource pools with fixed-priority scheduling:

| Port   | Priority 1 (highest)            | Priority 2               | Priority 3       |
|--------|---------------------------------|--------------------------|------------------|
| Port-A | EKF P matrix write @ 100 Hz     | EKF Kalman gain read     | —                |
| Port-B | Waypoint read @ 100 Hz          | MAVLink mission write @ 10 Hz | —           |
| Port-C | PID gain read @ 1 kHz           | AXI4-Lite on-demand      | —                |

- Maximum arbitration latency: **10 cycles @ 50 MHz = 200 ns**
- Conflict resolution: Higher-priority requester granted; lower-priority stalled for 1 cycle

### Signal Routing
- **EKF → PID feedback path:** 1-cycle pipeline register on state_updated → MOD_2 to break the combinational loop
- **FSM → PID setpoints:** Direct combinational or 1-FF registered pass-through
- **Sensor → EKF:** gyro/accel (MOD_5) → MOD_4 predict; GPS/baro/mag meas (MOD_5/6) → MOD_5 update
- **Health aggregation:** All sensor faults + EKF health → MOD_9 watchdog/preflight
- **MAVLink → FSM:** Parsed commands (MOD_7) → MOD_8 dispatcher
- **Telemetry → UART TX:** Telemetry mux output (MOD_7) → UART TX (already internal to MOD_7)

---

## Input Signals

| Signal Name              | Width  | Format  | Source     | Description                                  |
|--------------------------|--------|---------|------------|----------------------------------------------|
| `clk`                    | 1-bit  | —       | MOD_1      | 50 MHz system clock                          |
| `rst_n`                  | 1-bit  | —       | MOD_1      | Active-low synchronous reset                 |
| `ce_1khz`                | 1-bit  | —       | MOD_1      | 1 kHz source strobe                          |
| `ce_100hz`               | 1-bit  | —       | MOD_1      | 100 Hz source strobe                         |
| `ce_50hz`                | 1-bit  | —       | MOD_1      | 50 Hz source strobe                          |
| `ce_10hz`                | 1-bit  | —       | MOD_1      | 10 Hz source strobe                          |
| `ekf_bram_req`           | 1-bit  | —       | MOD_4/5    | Port-A BRAM request from EKF engine          |
| `ekf_bram_addr`          | 10-bit | —       | MOD_4/5    | Port-A address (P matrix)                    |
| `ekf_bram_wdata`         | 48-bit | Q2.46   | MOD_4/5    | Port-A write data (covariance)               |
| `ekf_bram_wen`           | 1-bit  | —       | MOD_4/5    | Port-A write enable                          |
| `wp_bram_req`            | 1-bit  | —       | MOD_8      | Port-B BRAM read request from waypoint mgr   |
| `wp_bram_addr`           | 10-bit | —       | MOD_8      | Port-B waypoint address                      |
| `mission_bram_req`       | 1-bit  | —       | MOD_7      | Port-B BRAM write request (mission upload)   |
| `mission_bram_addr`      | 10-bit | —       | MOD_7      | Port-B mission write address                 |
| `mission_bram_wdata`     | 272-bit| mixed   | MOD_7      | Waypoint fields to write                     |
| `gain_bram_req`          | 1-bit  | —       | MOD_2      | Port-C BRAM read (PID gains @ 1 kHz)         |
| `axi_bram_req`           | 1-bit  | —       | MOD_10     | Port-C BRAM access (on-demand)               |
| All cross-module signals |  various | various | All mods | Routed pass-through signals                 |

---

## Output Signals

| Signal Name                 | Width  | Format  | Destination    | Description                               |
|-----------------------------|--------|---------|----------------|-------------------------------------------|
| `ce_1khz_to_pid`            | 1-bit  | —       | MOD_2          | Gated 1 kHz strobe to PID inner loop      |
| `ce_100hz_to_[mod]`         | 1-bit ea| —      | Each module    | Gated 100 Hz strobe per destination       |
| `ce_50hz_to_baro`           | 1-bit  | —       | MOD_5          | 50 Hz strobe to barometer                 |
| `ce_10hz_to_[mod]`          | 1-bit ea| —      | Each module    | Gated 10 Hz strobe per destination        |
| `bram_porta_grant`          | 2-bit  | —       | MOD_4/5        | Port-A grant: 0=EKF write, 1=Kalman read  |
| `bram_portb_grant`          | 2-bit  | —       | MOD_7/8        | Port-B grant: 0=WP read, 1=mission write  |
| `bram_portc_grant`          | 2-bit  | —       | MOD_2/10       | Port-C grant: 0=gain read, 1=AXI access   |
| `ekf_state_to_pid`          | various| mixed   | MOD_2          | EKF state (1-cycle delayed)               |
| `fsm_setpoints_to_pid`      | various| mixed   | MOD_2          | FSM setpoints pass-through                |
| `health_to_watchdog`        | various| mixed   | MOD_9          | Aggregated health signals                 |

---

## Architecture

### High-Level Components

```
         ┌──────────────────────────────────────────────────────────┐
         │            module11_uav_interconnect.sv                   │
         │                                                          │
CE (×4)─►│  ce_strobe_arbiter.sv                                   ├──► ce_*_to_[mod]
         │  (fan-out + per-module gating logic)                     │
         │                                                          │
BRAM    ►│  bram_access_mux.sv                                     ├──► BRAM port grants
requests │  (3-port priority arbiter: EKF>GPS>baro, WP>mission,    │──► BRAM rdata outputs
         │   gain>AXI; max 10-cycle latency at 50MHz)               │
         │                                                          │
signals ►│  Signal routing registers                                ├──► ekf→pid (1 FF)
         │  (1-cycle delay FFs on EKF→PID feedback path)            ├──► fsm→pid (pass)
         └──────────────────────────────────────────────────────────┘
```

### Reused Modules from RTL_20

*None — Module 11 is entirely new interconnect fabric.*

### New Modules Created

| Module                  | Role                                                         |
|-------------------------|--------------------------------------------------------------|
| `ce_strobe_arbiter.sv`  | Fan-out CE strobes with per-module gating and synchronization|
| `bram_access_mux.sv`    | 3-port BRAM priority arbiter with stall logic                |

---

## Data Formats

*All data formats are passed through without conversion. Module 11 is a structural routing layer only.*

| Path                         | Format   | Notes                              |
|------------------------------|----------|------------------------------------|
| CE strobes                   | 1-bit    | Single-cycle active-high pulses    |
| EKF state (passed through)   | Q3.29 / Q4.28 / Q10.22 | Delayed by 1 FF before MOD_2 |
| BRAM address/data            | Various  | Width depends on BRAM usage        |
| BRAM grants                  | 2-bit    | Priority encoding                  |

---

## Register Interface

Module 11 has **no AXI4-Lite registers**. It is a transparent routing and arbitration module. Configuration is implicit (compile-time parameter).

---

## File Structure

```
Uav 1 to 12 Modules/module11/
├── module11_uav_interconnect.sv    # Top-level interconnect fabric
├── ce_strobe_arbiter.sv            # CE strobe fan-out and gating
└── bram_access_mux.sv              # Dual-port BRAM priority arbiter
```

---

## Module Interconnections

```
MOD_1 ── ce_1khz/100hz/50hz/10hz ──────────────► MOD_11 (strobe source)
MOD_11 ── ce_1khz_to_pid ──────────────────────► MOD_2 (inner PID)
MOD_11 ── ce_100hz_to_* ───────────────────────► MOD_2/3/4/6/7/8/9
MOD_11 ── ce_50hz_to_baro ─────────────────────► MOD_5
MOD_11 ── ce_10hz_to_* ────────────────────────► MOD_5/6/7/10

MOD_4/5 ── ekf_bram_req ───────────────────────► MOD_11 (Port-A arbiter)
MOD_8 ── wp_bram_req ──────────────────────────► MOD_11 (Port-B arbiter)
MOD_7 ── mission_bram_req ─────────────────────► MOD_11 (Port-B arbiter)
MOD_2 ── gain_bram_req ────────────────────────► MOD_11 (Port-C arbiter)
MOD_10 ── axi_bram_req ────────────────────────► MOD_11 (Port-C arbiter)
MOD_11 ── bram_port_grants ─────────────────────► All BRAM requestors

MOD_5/6 ── ekf_state_updated ──────────────────► MOD_11 (1-cycle delay)
MOD_11 ── ekf_state_to_pid ────────────────────► MOD_2 (PID feedback)
MOD_8 ── fsm_setpoints ────────────────────────► MOD_11 (pass-through)
MOD_11 ── fsm_setpoints_to_pid ────────────────► MOD_2 (PID setpoints)
```

---

## BRAM Arbitration Logic

### Port Priority Table

| Port   | Request Sources (in priority order)  | Max Latency Added |
|--------|--------------------------------------|-------------------|
| Port-A | 1. EKF P matrix write (100 Hz)       | 0 cycles          |
|        | 2. Kalman gain read (on demand)      | ≤ 10 cycles       |
| Port-B | 1. Waypoint read (100 Hz)            | 0 cycles          |
|        | 2. Mission upload write (10 Hz)      | ≤ 10 cycles       |
| Port-C | 1. PID gain read (1 kHz)             | 0 cycles          |
|        | 2. AXI4-Lite on-demand               | ≤ 10 cycles       |

### Conflict Resolution
1. Higher-priority requester receives immediate grant
2. Lower-priority requester receives a stall signal for up to 10 cycles
3. At 50 MHz, 10 cycles = 200 ns maximum latency
4. AXI4-Lite can use `RVALID`/`BVALID` wait-state extension for stall absorption

---

## Design Considerations

- **EKF→PID delay:** A single pipeline register on the EKF state output breaks the timing path between EKF update completion and PID computation. The 1-cycle latency (20 ns) is negligible relative to the 10 ms outer-loop period.
- **CE fan-out:** Each CE strobe drives 4–8 downstream modules. The `ce_strobe_arbiter` uses BUFG-style fan-out to ensure hold times are met at all destinations without over-constraining the clock-enable path.
- **BRAM port width:** All three shared BRAMs use a 36-bit wide ×512-deep configuration (BRAM18 primitive). Width and depth are adjusted per BRAM usage by parameterization in `bram_access_mux`.
- **No combinational paths through Module 11:** All outputs are registered to prevent combinational glitches from propagating between modules.
- **Resource estimate:**
  - DSP48E1: 0
  - BRAM18: 0 (only arbitrates access; BRAMs instantiated in MOD_12)
  - LUTs: ~200
  - FFs: ~150

---

## Testing & Verification

| Test Point                            | Verification Method                                          |
|---------------------------------------|--------------------------------------------------------------|
| CE strobe fan-out timing              | Verify all destinations see strobe within same cycle         |
| Port-A EKF grant (highest priority)  | Simultaneous EKF + Kalman requests; verify EKF wins          |
| Port-B conflict: WP vs mission        | Simultaneous requests; verify WP read wins; mission stalled  |
| Port-C: gain @ 1 kHz vs AXI          | Simultaneous; verify gain read wins; AXI stalled ≤ 10 cycles |
| Stall latency ≤ 10 cycles             | Measure cycles between request and grant on low-priority port|
| EKF→PID delay exactly 1 cycle        | Assert EKF output; verify PID input 1 cycle later            |
| CE strobe phase alignment             | All four CE strobes from MOD_1 arrive in phase at MOD_11     |

---

## Optimization Scope

| Area            | Opportunity                                                               | Impact  |
|-----------------|---------------------------------------------------------------------------|---------|
| **Resource**    | Collapse 3-port arbiter into single time-sliced BRAM access controller    | Medium  |
| **Performance** | Pre-arbitrate BRAM requests 1 cycle ahead using request look-ahead        | Medium  |
| **Area**        | Reduce CE fan-out logic by using downstream module enable pins instead     | Low     |
| **Timing**      | Pipeline arbiter grant decode to improve critical path in tight budgets    | Low     |
| **Power**       | Gate CE strobe distribution when respective subsystem is powered down      | Low     |

---

*Module 11 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
