# UAV Module 1: System Clock & Control Strobes

## Overview

UAV Module 1 is the foundational clock and timing infrastructure for the entire UAV FPGA system. It accepts a 50 MHz system clock, detects PLL lock, and generates all clock-enable (CE) strobes distributed to every downstream subsystem. Metastability synchronizers protect all asynchronous inputs from glitching.

- **Target Platform:** Xilinx Artix-7 XC7A35T
- **Criticality:** 🔴 CRITICAL — All other modules depend on this module for timing

---

## Key Functionality

- PLL lock detection on 50 MHz input clock
- CE strobe generation at **1 kHz** → PID inner rate loop (MOD_2)
- CE strobe generation at **100 Hz** → PID outer loop, EKF predict, FSM, health checks (MOD_2, 3, 4, 6, 7, 8, 9, 11)
- CE strobe generation at **50 Hz** → Barometer sampling (MOD_5)
- CE strobe generation at **10 Hz** → GPS/magnetometer/MAVLink RX & telemetry TX (MOD_5, 6, 7, 10, 11)
- Metastability synchronizers for all asynchronous external inputs (2-FF synchronizer chain)
- Synchronous reset distribution, gated until PLL lock asserted

---

## Input Signals

| Signal Name    | Width | Type   | Source               | Description                              |
|----------------|-------|--------|----------------------|------------------------------------------|
| `clk_50mhz`    | 1-bit | logic  | External PLL / MMCM  | Main 50 MHz system clock                 |
| `pll_locked`   | 1-bit | logic  | MMCM lock indicator  | Asserted when PLL has achieved lock      |
| `async_reset_n`| 1-bit | logic  | Hardware reset pin   | Active-low asynchronous external reset   |

---

## Output Signals

| Signal Name  | Width | Type  | Destination                        | Description                                   |
|--------------|-------|-------|------------------------------------|-----------------------------------------------|
| `sys_clk`    | 1-bit | logic | All modules                        | Buffered 50 MHz system clock                  |
| `rst_n`      | 1-bit | logic | All modules                        | Synchronous active-low reset (gated by PLL)   |
| `ce_1khz`    | 1-bit | logic | MOD_2 (PID inner loop)             | 1 kHz clock-enable strobe (every 50 000 clks) |
| `ce_100hz`   | 1-bit | logic | MOD_2, 3, 4, 6, 7, 8, 9, 11       | 100 Hz clock-enable strobe (every 500 000 clks)|
| `ce_50hz`    | 1-bit | logic | MOD_5 (barometer)                  | 50 Hz clock-enable strobe (every 1 M clks)    |
| `ce_10hz`    | 1-bit | logic | MOD_5, 6, 7, 10, 11               | 10 Hz clock-enable strobe (every 5 M clks)    |

---

## Architecture

### High-Level Components

```
                  ┌──────────────────────────────────────┐
  clk_50mhz ───► │           clk_divider.sv              │──► sys_clk
  pll_locked ──► │  (Counter-based CE strobe generator)  │──► ce_1khz
                  └──────────────────────────────────────┘──► ce_100hz
                                                          ──► ce_50hz
  async_reset_n ► ┌──────────────────┐                   ──► ce_10hz
                  │  synchronizer.sv │──► rst_n (sync)
                  └──────────────────┘
                  ┌──────────────────┐
  async_inputs ─► │  synchronizer.sv │──► sync_inputs
                  └──────────────────┘
```

### Reused Modules from RTL_20

| Module            | Role                                                   |
|-------------------|--------------------------------------------------------|
| `clk_divider.sv`  | Counter-based divider to generate CE strobes           |
| `synchronizer.sv` | 2-FF metastability synchronizer for async inputs       |
| `tick_gen.sv`     | Tick generator for deterministic strobe timing         |

### New Modules Created

*None — Module 1 is fully implemented by composing existing RTL_20 primitives.*

---

## Data Formats

| Parameter     | Format     | Notes                               |
|---------------|------------|-------------------------------------|
| CE strobes    | 1-bit pulse | Single clock-cycle active-high pulse |
| Clock         | —          | 50 MHz continuous                   |
| Reset         | 1-bit      | Active-low, synchronous after PLL   |

---

## Register Interface

Module 1 does not expose AXI4-Lite registers. All configuration (clock frequency, strobe rates) is determined at synthesis time via parameters.

---

## File Structure

```
Uav 1 to 12 Modules/module 1/
└── module1_uav_clk_ctrl.sv    # Top-level clock & CE strobe controller
```

### Dependencies (from RTL_20 shared library)

```
clk_divider.sv     # Counter-based CE strobe generator
synchronizer.sv    # 2-FF metastability synchronizer
tick_gen.sv        # Deterministic tick generation
```

---

## Module Interconnections

```
MOD_1 ──► ce_1khz  ──────────────────────────────► MOD_2 (PID inner rate loop)
MOD_1 ──► ce_100hz ──► MOD_2, MOD_3, MOD_4,
                        MOD_6, MOD_7, MOD_8,
                        MOD_9, MOD_11
MOD_1 ──► ce_50hz  ──────────────────────────────► MOD_5 (barometer)
MOD_1 ──► ce_10hz  ──► MOD_5, MOD_6, MOD_7, MOD_10, MOD_11
MOD_1 ──► sys_clk  ──────────────────────────────► ALL modules
MOD_1 ──► rst_n    ──────────────────────────────► ALL modules
```

---

## Design Considerations

- **Base module — foundational layer:** Every other module clocks itself from the CE strobes produced here. Any failure in strobe generation has system-wide impact.
- **PLL gating:** `rst_n` is held low until `pll_locked` asserts, preventing any downstream logic from operating with an unstable clock.
- **Metastability:** All asynchronous inputs (hardware buttons, external flags) pass through 2-FF synchronizers before distribution.
- **Resource estimate:**
  - LUTs: ~50 (counters + combinational decode)
  - FFs: ~80 (counter registers + synchronizer stages)
  - DSP48E1: 0
  - BRAM18: 0

---

## Testing & Verification

| Test Point                    | Verification Method                                      |
|-------------------------------|----------------------------------------------------------|
| CE_1kHz period = 1 ms         | Oscilloscope / testbench cycle counter                   |
| CE_100Hz period = 10 ms       | Testbench assertion                                      |
| CE_50Hz period = 20 ms        | Testbench assertion                                      |
| CE_10Hz period = 100 ms       | Testbench assertion                                      |
| Reset synchronization         | Ensure rst_n deasserts only after pll_locked             |
| Metastability synchronizer    | Inject async transitions; verify 0 glitches after 2 FFs |
| Strobe phase alignment        | All strobes share common timebase, checked by simulation |

---

## Optimization Scope

| Area        | Opportunity                                                          | Impact  |
|-------------|----------------------------------------------------------------------|---------|
| **Timing**  | All CE paths are combinational decode; no critical paths expected    | Low     |
| **Power**   | Gating unused clock domains when respective modules are idle         | Medium  |
| **Area**    | Shared counter for multiple strobe rates (single 26-bit counter)     | Low     |
| **Resource**| No DSPs or BRAMs used; already minimal                               | None    |
| **Timing**  | BUFG insertion for sys_clk to meet hold requirements across device   | Low     |

---

*Module 1 of 12 | UAV RTL System | Target: Xilinx Artix-7 XC7A35T | Date: 2026-04-03*
