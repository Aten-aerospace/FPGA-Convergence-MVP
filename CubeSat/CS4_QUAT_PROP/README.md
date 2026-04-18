# CS4 — Quaternion Propagator

> Propagates the spacecraft attitude quaternion one time-step using gyroscope angular rates via the Hamilton product `q(k+1) = normalise(q(k) ⊗ dq)` in Q15 fixed-point at 100 Hz.

---

## Overview

| Attribute | Value |
|---|---|
| Requirement | CS-ADCS-004 |
| Top module | `quat_propagator_wrapper` |
| Clock domain | `sys_clk` (50 MHz) |
| CE strobe | 100 Hz `ce_100hz` from CS12 |
| Algorithm | Small-angle quaternion kinematics: dq = [1, ω·dt/2] |
| Fixed-point | Q15 — `DT_HALF = 164` (= 0.005 × 32768, dt = 10 ms) |
| Pipeline latency | ~7 cycles (multiply) + 2 cycles (norm check) = ~9 total |
| BRAM | 96 B |
| DSP48 | 3 |

---

## File Structure

| File | Purpose |
|---|---|
| `quat_propagator_wrapper.sv` | Top-level — dq computation, pipeline staging, norm health check |
| `quat_multiply.sv` | 3-cycle pipelined Hamilton product |
| `quat_normalize.sv` | 3-cycle Newton-Raphson normalisation |
| `norm_checker.sv` | 2-cycle post-normalisation health check (`|‖q‖²−1|`) |
| `quaternion_math.sv` | Shared arithmetic helpers |
| `quat_propagator.sv` | Lower-level propagation step |
| `tb_quat_propagator_wrapper.sv` | Self-checking directed testbench |

Shared primitives used: `cordic.sv`, `sqrt.sv`, `fp_divider.sv`.

---

## Module Interface

```systemverilog
module quat_propagator_wrapper (
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_100hz,                     // 100 Hz clock enable

    input  logic signed [15:0] q_in  [0:3],           // current quaternion Q15 [w,x,y,z]
    input  logic               q_valid_in,

    input  logic signed [15:0] omega [0:2],            // angular rate Q15 rad/s [x,y,z]

    output logic signed [15:0] q_out       [0:3],     // propagated quaternion Q15
    output logic               q_valid_out,            // strobe: q_out updated
    output logic               norm_error,             // pre-norm magnitude fault
    output logic        [15:0] q_norm,                 // squared-norm Q15 (≈32767 = unit)
    output logic               q_norm_valid,

    output logic               norm_ok,                // post-norm health (2 cycles later)
    output logic        [15:0] norm_val,               // |‖q‖²−1| deviation Q15
    output logic               norm_ok_valid,
    output logic               quat_ready              // alias of q_valid_out
);
```

---

## Functionality

1. **dq computation** — `omega[i] × DT_HALF (164) >> 15` gives ω·dt/2 in Q15. `dq = [0x7FFF, dq_x, dq_y, dq_z]`.
2. **Register on CE** — inputs latched on `ce_100hz & q_valid_in` to enter pipeline.
3. **Hamilton product** (`quat_multiply`, 3 cycles) — computes `q_in ⊗ dq`; 16-bit product uses 32-bit intermediate with arithmetic right-shift.
4. **Normalisation** (`quat_normalize`, 3 cycles) — Newton-Raphson one-step correction ensures `‖q‖ = 1 ± 0.001`.
5. **Norm health check** (`norm_checker`, 2 cycles) — verifies output is unit; sets `norm_ok` and `norm_val = |‖q_out‖²−1|`.

`q_valid_out` asserts one cycle when propagation is complete; downstream (CS5/CS6) latches on this strobe.

---

## Simulation Instructions

```bash
iverilog -g2012 -o sim_cs4 \
  CS4_QUAT_PROP/tb_quat_propagator_wrapper.sv \
  CS4_QUAT_PROP/quat_propagator_wrapper.sv \
  CS4_QUAT_PROP/quat_multiply.sv \
  CS4_QUAT_PROP/quat_normalize.sv \
  CS4_QUAT_PROP/norm_checker.sv \
  CS4_QUAT_PROP/quaternion_math.sv \
  CS4_QUAT_PROP/quat_propagator.sv \
  cordic.sv sqrt.sv fp_divider.sv

vvp sim_cs4
```

```tcl
vlog -sv CS4_QUAT_PROP/tb_quat_propagator_wrapper.sv CS4_QUAT_PROP/*.sv cordic.sv sqrt.sv fp_divider.sv
vsim -t 1ps tb_quat_propagator_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 50 MHz `sys_clk` |
| CE strobe | Internally generated 100 Hz |
| Stimulus | Identity quaternion + known angular rate; expected q_out computed offline |
| Checking | Quaternion component error ≤ 2 LSB Q15; norm deviation < 0.001 |
| Coverage | Identity propagation, pure-rate rotation, norm recovery after injected denorm |

---

## Expected Behaviour

```
ce_100hz           __|‾|__________________________|‾|_________
q_valid_in         ___|‾|___________________________|‾|_______
[pipeline 7 cycles]
q_valid_out        __________________________|‾|______________|‾|
q_out[0..3]        ==========================|UPDATED Q15====
norm_ok_valid      ________________________________|‾|______________
norm_ok            ________________________________|1|   (asserts when ‖q‖² within threshold)
```

`norm_error` asserts (pre-norm) only when `q_in` is severely denormalised (‖q‖² < 0.5).

---

## Limitations

- Uses small-angle first-order integration — accurate for angular rates < 5 °/s; higher rates accumulate truncation error.
- `DT_HALF` is a compile-time constant; dt variation (e.g., missed CE) is not handled.
- No second-order Runge-Kutta option in MVP.
- Newton-Raphson normalisation performs one iteration; may not fully converge for extreme inputs.

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] Identity propagation: output matches input within 2 LSB
- [x] Pure-yaw rotation: angular increment matches expected quaternion
- [x] Norm health check `norm_ok` asserts correctly after normalisation
- [x] Pipeline timing verified (7+2 cycle latency)
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] Monte-Carlo drift test over 60 s simulated time (target < 0.01°)
- [ ] Synthesis DSP48 count confirmed (target: 3)
