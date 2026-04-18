# CS5 — Extended Kalman Filter (7-State Attitude Estimation)

> Fuses IMU accelerometer/gyroscope and magnetometer measurements into a 7-state attitude estimate (quaternion q[4] + gyro bias b[3]) using an EKF predict/update loop at 100 Hz.

---

## Overview

| Attribute | Value |
|---|---|
| Requirements | CS-ADCS-005, CS-ADCS-006 |
| Top module | `ekf_wrapper` |
| Clock domain | `sys_clk` (50 MHz) |
| CE strobe | 100 Hz `ce_100hz` from CS12 |
| State vector | q[0..3] (unit quaternion) + bias[0..2] (gyro bias rad/s) |
| Execution budget | ≤ 8 ms per cycle |
| Fault watchdog | 2 ms (`FAULT_CYCLES = 200_000` at 100 MHz) |
| Covariance update | Joseph-form for numerical stability |
| BRAM | 512 B |
| DSP48 | 6 |

---

## File Structure

| File | Purpose |
|---|---|
| `ekf_wrapper.sv` | Top-level — thin wrapper; adds `P_diag` and `innovation` diagnostic outputs |
| `ekf_core.sv` | Core orchestrator — sequences predict and update stages |
| `ekf_predict.sv` | State and covariance propagation (F, Q matrices) |
| `ekf_update.sv` | Measurement update (H, R, Kalman gain, state correction) |
| `ekf_covariance.sv` | Covariance matrix operations |
| `ekf_joseph_update.sv` | Joseph-form P update: `P = (I−KH)P(I−KH)ᵀ + KRKᵀ` |
| `ekf_measurement_model.sv` | Gravity and magnetic field measurement model |
| `tb_ekf_wrapper.sv` | Self-checking directed testbench |

Shared primitives used: `cordic.sv`, `sqrt.sv`, `fp_divider.sv`, `kalman_1v.sv`.

---

## Module Interface

```systemverilog
module ekf_wrapper #(
    parameter int CLK_HZ       = 100_000_000,
    parameter int FAULT_CYCLES = 200_000      // 2 ms valid watchdog
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,             // 100 Hz clock enable

    // Sensor inputs (Q15)
    input  logic signed [15:0] accel [0:2],   // from CS1
    input  logic signed [15:0] gyro  [0:2],   // from CS1
    input  logic signed [15:0] mag   [0:2],   // from CS2
    input  logic               meas_valid,    // CS1/CS2 data-ready strobe

    // Primary outputs (Q15)
    output logic signed [15:0] q_est    [0:3], // attitude quaternion
    output logic signed [15:0] bias_est [0:2], // estimated gyro bias
    output logic               ekf_valid,
    output logic               ekf_fault,

    // Diagnostic outputs (Q15)
    output logic signed [15:0] P_diag     [0:6], // covariance diagonal
    output logic signed [15:0] innovation [0:6]  // measurement residuals
);
```

---

## Functionality

1. **Predict** (`ekf_predict`) — propagates state using `q(k+1) = q(k) ⊗ Ω(ω−b)·dt`; propagates covariance `P = F·P·Fᵀ + Q`.
2. **Update** (`ekf_update`) — computes innovation `y = z − h(x)` from accelerometer and magnetometer measurements. Kalman gain `K = P·Hᵀ·(H·P·Hᵀ + R)⁻¹` applied.
3. **Joseph form** (`ekf_joseph_update`) — `P = (I−KH)·P·(I−KH)ᵀ + K·R·Kᵀ` ensures covariance remains positive-definite.
4. **Fault watchdog** — if `ekf_valid` is not asserted within `FAULT_CYCLES` of `ce_100hz`, `ekf_fault` is raised; cleared on next successful update.
5. **Diagnostics** — `P_diag[0:6]` (covariance diagonal) and `innovation[0:6]` (residuals) exposed for ground-station telemetry tuning.

---

## Simulation Instructions

```bash
iverilog -g2012 -o sim_cs5 \
  CS5_EKF/tb_ekf_wrapper.sv \
  CS5_EKF/ekf_wrapper.sv \
  CS5_EKF/ekf_core.sv \
  CS5_EKF/ekf_predict.sv \
  CS5_EKF/ekf_update.sv \
  CS5_EKF/ekf_covariance.sv \
  CS5_EKF/ekf_joseph_update.sv \
  CS5_EKF/ekf_measurement_model.sv \
  sqrt.sv fp_divider.sv cordic.sv

vvp sim_cs5
```

```tcl
vlog -sv CS5_EKF/tb_ekf_wrapper.sv CS5_EKF/*.sv cordic.sv sqrt.sv fp_divider.sv
vsim -t 1ps tb_ekf_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 50 MHz `sys_clk` |
| CE strobe | Internally generated 100 Hz |
| Stimulus | Pre-computed sensor vectors from MATLAB EKF reference; injected via ROM task |
| Checking | `q_est` error vs. reference ≤ 5 LSB Q15; `P_diag` stays positive |
| Coverage | Steady-state convergence, sensor-dropout fault, divergence / reset, covariance blow-up prevention |

---

## Expected Behaviour

```
ce_100hz          __|‾|______________|‾|______________|‾|_______
meas_valid        ___|‾|______________|‾|______________|‾|______
ekf_valid         _________________|‾|_______________|‾|________  (delayed by processing)
q_est[0..3]       =================|UPDATED ESTIMATE==========
ekf_fault         _________________________________________  (stays low when sensor valid)
P_diag[0]:        decreasing trend → convergence
innovation[0]:    random-walk residual near zero at convergence
```

`ekf_fault` asserts if `meas_valid` absent > `FAULT_CYCLES` (2 ms @ 100 MHz).

---

## Limitations

- Process noise matrix Q and measurement noise matrix R are compile-time constants; no runtime update.
- Single Newton-Raphson division pass in covariance inverse; numerical conditioning depends on initial P.
- State vector limited to 7 elements; magnetometer disturbance model not included.
- No sensor-fusion quality gating (all inputs weighted equally when `meas_valid` asserts).

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] Steady-state attitude error < 0.5° vs. MATLAB reference
- [x] Covariance diagonal stays positive over 100-cycle run
- [x] `ekf_fault` asserts correctly on sensor dropout
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] Joseph-form stability under extreme noise injection
- [ ] Monte-Carlo 3σ accuracy characterisation
- [ ] Synthesis DSP48 count confirmed (target: 6)
