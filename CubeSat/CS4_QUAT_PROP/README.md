# CS4 — Quaternion Propagator

## 1. Module Title & ID

**Module:** CS4 — Quaternion Propagator
**Subsystem ID:** CS4
**Requirement:** CS-ADCS-004

---

## 2. Overview

CS4 propagates the spacecraft attitude quaternion forward one time step (dt = 0.01 s at 100 Hz) using the angular velocity measurement from the IMU (CS1). It applies the small-angle quaternion kinematic equation in Q15 fixed-point arithmetic: `q(k+1) = normalize(q(k) ⊗ dq)`, where `dq = [1, ω·dt/2]`. The pipeline is 3-stage (multiply) + 3-stage (normalize), delivering the result within ≈ 7 clock cycles. A post-normalization health checker verifies that the output quaternion remains a unit quaternion to within tolerance.

**Target Platform:** Xilinx Artix-7 XC7A35T

---

## 3. Criticality

**CRITICAL** — CS-ADCS-004. The quaternion propagator provides the between-update attitude prediction required by the EKF (CS5) and is the primary attitude state when CS5 is initializing. Norm drift > 0.001 over 60 s indicates a failure condition.

---

## 4. Key Functionality

- Accepts current quaternion `q_in[0:3]` (Q15) and angular velocity `omega[0:2]` (Q15 rad/s) every `ce_100hz`.
- Computes delta quaternion `dq = [32767, ω_x·DT_HALF, ω_y·DT_HALF, ω_z·DT_HALF]` in Q15 (DT_HALF = 164 ≈ 0.005 × 32768).
- 3-cycle pipelined Hamilton product `q(k) ⊗ dq` via `quat_multiply`.
- 3-cycle Newton-Raphson normalization via `quat_normalize`; sets `norm_error` if pre-norm magnitude deviates > 5 %.
- Post-normalization `norm_checker` verifies output norm within 2 additional cycles; outputs `norm_ok` and `norm_val`.
- Total pipeline latency: ~7 cycles (multiply + normalize); ~9 cycles to `norm_ok_valid`.
- Identity quaternion `[32767, 0, 0, 0]` on reset.
- `quat_ready` is an alias of `q_valid_out` for downstream handshaking.

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `clk` | input | 1 | `sys_clk` | 100 MHz system clock |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `ce_100hz` | input | 1 | `sys_clk` | 100 Hz clock-enable from CS12 |
| `q_in[0:3]` | input | 4 × 16 | `sys_clk` | Current attitude quaternion [w,x,y,z] in Q15 |
| `q_valid_in` | input | 1 | `sys_clk` | `q_in` is valid |
| `omega[0:2]` | input | 3 × 16 | `sys_clk` | Angular velocity [x,y,z] in Q15 rad/s (from CS1) |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `q_out[0:3]` | output | 4 × 16 | `sys_clk` | Propagated + normalised quaternion [w,x,y,z] in Q15 |
| `q_valid_out` | output | 1 | `sys_clk` | One-cycle strobe — `q_out` updated |
| `quat_ready` | output | 1 | `sys_clk` | Alias of `q_valid_out` (propagation complete) |
| `norm_error` | output | 1 | `sys_clk` | Pre-norm magnitude deviates > 5 % from 1.0 |
| `q_norm` | output | 16 | `sys_clk` | Squared-norm in Q15 (≈ 32767 for unit quaternion) |
| `q_norm_valid` | output | 1 | `sys_clk` | Strobe aligned with `q_valid_out` |
| `norm_ok` | output | 1 | `sys_clk` | Post-norm output is within tolerance |
| `norm_val` | output | 16 | `sys_clk` | `|‖q‖²−1|` deviation in Q15 |
| `norm_ok_valid` | output | 1 | `sys_clk` | Strobe: `norm_ok` / `norm_val` are valid (+2 cycles) |

---

## 7. Architecture

```
ce_100hz && q_valid_in
      │
      ▼ (register inputs)
  q_in_r[0:3], dq_r[0:3]
      │
      ▼
┌────────────────────┐  q_mul_out[0:3]   ┌────────────────────┐  q_out[0:3]
│   quat_multiply    │──────────────────▶│   quat_normalize   │──────────────▶
│  (3-cycle pipeline,│  mul_valid_out     │  (3-cycle pipeline,│  q_valid_out
│   Hamilton product)│                   │   Newton-Raphson)  │  norm_error
└────────────────────┘                   └────────────────────┘  q_norm
                                                    │
                                                    ▼ (+2 cycles)
                                         ┌──────────────────────┐
                                         │    norm_checker       │
                                         │  (post-norm health   │
                                         │   check on q_out)    │
                                         └──────────────────────┘
                                              norm_ok, norm_val
                                              norm_ok_valid
```

**Reused Helper IPs (from `CubeSat/`):**
- `cordic.sv` — CORDIC engine (used for quaternion magnitude in normalization)
- `sqrt.sv` — Square root (Newton-Raphson normalization support)
- `fp_divider.sv` — Fixed-point divider (normalization step)

**Sub-modules (CS4-specific):**
- `quat_multiply.sv` — 3-cycle pipelined Hamilton product
- `quat_normalize.sv` — 3-cycle Newton-Raphson normalization
- `norm_checker.sv` — 2-cycle post-normalization health check
- `quaternion_math.sv` — Utility: quaternion conjugate and algebra helpers

---

## 8. Data Formats

| Signal | Format | Notes |
|---|---|---|
| `q_in[0:3]`, `q_out[0:3]` | Q15 signed (16-bit) | 1.0 = 32767 (0x7FFF); unit quaternion |
| `omega[0:2]` | Q15 signed (16-bit) | rad/s; 1.0 = 32767 |
| `dq` (internal) | Q15 signed (16-bit) | `dq[0] = 32767`; `dq[1:3] = omega × DT_HALF >> 15` |
| `q_norm` | Q15 unsigned (16-bit) | Squared norm; ≈ 32767 for unit quaternion |
| `norm_val` | Q15 unsigned (16-bit) | `|‖q‖²−1|`; 0 = perfect unit quaternion |
| DT_HALF constant | Q15 integer = 164 | `round(0.005 × 32768)` for 100 Hz rate |

---

## 9. Register Interface

CS4 has **no AXI4-Lite register interface**. The `DT_HALF` constant is a compile-time `localparam`. If the update rate changes (e.g., 500 Hz), update `DT_HALF = round(0.001 × 32768) = 33` accordingly.

**Parameters (synthesised):**

| Parameter | Default | Description |
|---|---|---|
| None (wrapper) | — | Parameters are embedded as `localparam` inside `quat_propagator_wrapper` |

---

## 10. File Structure

```
CubeSat/CS4_QUAT_PROP/
├── quat_propagator_wrapper.sv   ← Top-level wrapper; CS12 integration point
├── quat_multiply.sv             ← 3-cycle pipelined Hamilton product
├── quat_normalize.sv            ← 3-cycle Newton-Raphson normalization
├── norm_checker.sv              ← 2-cycle post-norm health check
├── quaternion_math.sv           ← Quaternion conjugate and utility helpers
├── quat_propagator.sv           ← Standalone propagator core (internal)
├── tb_quat_propagator_wrapper.sv ← Integration testbench
└── README.md                    ← This file

CubeSat/ (shared helper IPs used by CS4):
├── cordic.sv                    ← CORDIC trigonometric engine
├── sqrt.sv                      ← Square root calculator
└── fp_divider.sv                ← Fixed-point divider
```

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `ce_100hz` | CS4 ← CS12 | `clk_manager` (CS12) | 100 Hz update trigger |
| `q_in[0:3]` | CS4 ← CS5 | `ekf_wrapper` (CS5) | EKF-corrected quaternion fed back |
| `omega[0:2]` | CS4 ← CS1 | `spi_imu_wrapper` (CS1) | Angular velocity from IMU |
| `q_out[0:3]` | CS4 → CS5 | `ekf_wrapper` (CS5) | Predicted quaternion for EKF prediction step |
| `q_out[0:3]` | CS4 → CS6 | `pd_control_wrapper` (CS6) | Attitude estimate for error computation |
| `norm_error` | CS4 → CS8 | `adcs_fsm_wrapper` (CS8) | Quaternion health flag |
| `q_norm_valid` | CS4 → CS8 | `adcs_fsm_wrapper` (CS8) | Norm validity strobe |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- Pipeline latency: 7 clock cycles (70 ns @ 100 MHz) << 10 ms cycle budget. No stalls.
- 100 Hz update is sufficient for small-satellite angular rates (< 1 °/s typical).

**Resource:**
- `quat_multiply`: 4 multiply-accumulate pairs → 4 DSP48E1 (sharing possible with TDM).
- `quat_normalize`: Newton-Raphson needs 1 multiply + 1 subtract per iteration; ≈ 2 DSP48E1.
- Total CS4 DSP48 estimate: 3 (per plan); BRAM: 96 B (for coefficient tables).

**Optimization Opportunities:**
1. Time-multiplex DSP48E1 blocks across the 4 Hamilton product terms to reduce from 4 to 1 DSP.
2. If angular rate exceeds small-angle assumption (> 0.1 rad/step), switch to full matrix exponential for higher fidelity.
3. Increase update rate to 500 Hz or 1 kHz by adjusting `DT_HALF` and connecting to `ce_1khz`.

**Timing:**
- Critical path: DSP48E1 multiply-accumulate chain in `quat_multiply`. Ensure registered pipeline stages.
- `norm_error` is combinational within `quat_normalize`; pipeline if timing is tight.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS4_QUAT_PROP/tb_quat_propagator_wrapper.sv`

**Test Scenarios:**
- Start from identity quaternion; apply zero omega → verify `q_out` remains identity.
- Apply known constant omega = [0.1, 0, 0] rad/s for 10 cycles; verify rotation about X-axis.
- Verify `norm_error` does not assert for normal inputs; verify it asserts on degenerate input.
- Apply omega large enough to cause small-angle error; verify `norm_ok` still asserts post-normalize.
- Verify pipeline latency: `q_valid_out` arrives exactly 7 cycles after `ce_100hz && q_valid_in`.
- Verify `norm_ok_valid` arrives 2 cycles after `q_valid_out`.

**Simulation Notes:**
- Compile with `iverilog -g2012` including `cordic.sv`, `sqrt.sv`, `fp_divider.sv`.
- Timescale: 1 ns / 1 ps.
- Use a golden-reference model (MATLAB/Python quaternion toolbox) for numerical verification.

**Requirements Coverage:**
- CS-ADCS-004: q(k+1) = q(k) + 0.5×q(k)⊗ω×Δt, norm 1 ± 0.001, drift < 0.01° over 60 s.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
