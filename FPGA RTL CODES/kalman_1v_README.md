# Kalman Filter (1-Variable)

## Overview
A simplified 1-dimensional Kalman filter for position/velocity estimation with acceleration input, using fixed-point arithmetic for FPGA implementation.

## Features
- **3-axis Position/Velocity**: Tracks (x,y,z) position and velocity
- **Accelerometer Input**: Uses acceleration measurements
- **Fixed-Point Math**: Q15.16 and Q8.24 formats
- **Dual-Rate Operation**: 100Hz predict, 10Hz update
- **State-Space Model**: Linear constant-velocity model
- **Preset Gains**: Optimized Kalman gains

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `POS_W` | 32 | Position width (Q15.16) |
| `VEL_W` | 32 | Velocity width (Q8.24) |
| `MEAS_W` | 32 | Measurement width (Q15.16) |
| `VID` | 0 | Variable ID (unused) |

## Port Description

### Inputs
- `clk`, `rst_n`: Clock and reset
- `predict_en`: Prediction step trigger (100Hz)
- `update_en`: Measurement update trigger (10Hz)
- `meas_px, meas_py, meas_pz[31:0]`: Position measurements (Q15.16)
- `accel_x, accel_y, accel_z[31:0]`: Acceleration inputs (Q8.24)

### Outputs
- `est_px, est_py, est_pz[31:0]`: Estimated positions (Q15.16)
- `est_vx, est_vy, est_vz[31:0]`: Estimated velocities (Q8.24)
- `data_valid`: Output valid flag

## Operation

### State Vector
For each axis:
```
x = [position, velocity]ᵀ
```

### Prediction Step (100Hz)
State prediction using constant-velocity model with acceleration:
```
p_new = p + v × Ts
v_new = v + a × Ts
```

Where:
- Ts = 0.01s (100Hz update rate)
- a = accelerometer measurement

### Update Step (10Hz)
Measurement update using Kalman gains:
```
innovation = measurement - predicted_position
p_updated = p_pred + Kp × innovation
v_updated = v_pred + Kv × innovation
```

### Kalman Gains (Fixed)
- **Kp** ≈ 0.8 (position gain)
- **Kv** ≈ 0.05 (velocity gain)

These are precomputed for a specific process/measurement noise model.

## Fixed-Point Formats

### Q15.16 (Position, Measurement)
- **Integer**: 15 bits + sign
- **Fractional**: 16 bits
- **Range**: ±32768
- **Resolution**: 0.0000152587890625

### Q8.24 (Velocity, Acceleration)
- **Integer**: 8 bits + sign
- **Fractional**: 24 bits
- **Range**: ±256
- **Resolution**: 0.000000059604644775390625

## State Machine
```
IDLE → PREDICT → UPDATE (if update_en) → OUTPUT → IDLE
                   ↓
              OUTPUT (if !update_en)
```

## Timing
- **Predict Rate**: 100 Hz (every 10ms)
- **Update Rate**: 10 Hz (every 100ms)
- **Latency**: 3-4 clock cycles per step
- **Output**: Valid for 1 cycle after each operation

## Usage Example

### Basic Integration
```systemverilog
kalman_1v #(
    .POS_W(32),
    .VEL_W(32)
) kalman_inst (
    .clk(clk),
    .rst_n(rst_n),

    .predict_en(tick_100hz),    // 100Hz tick
    .update_en(tick_10hz),      // 10Hz tick

    // Measurements (from GPS/position sensor)
    .meas_px(gps_x),
    .meas_py(gps_y),
    .meas_pz(gps_z),

    // Accelerometer inputs
    .accel_x(imu_accel_x),
    .accel_y(imu_accel_y),
    .accel_z(imu_accel_z),

    // Outputs
    .est_px(filtered_x),
    .est_py(filtered_y),
    .est_pz(filtered_z),
    .est_vx(velocity_x),
    .est_vy(velocity_y),
    .est_vz(velocity_z),

    .data_valid(estimate_ready)
);
```

### Tick Generation
```systemverilog
// 100Hz prediction tick
tick_gen #(.CLK_HZ(50_000_000), .TICK_HZ(100))
    tick_predict (.clk(clk), .rst_n(rst_n), .tick(predict_tick));

// 10Hz update tick
tick_gen #(.CLK_HZ(50_000_000), .TICK_HZ(10))
    tick_update (.clk(clk), .rst_n(rst_n), .tick(update_tick));
```

## Applications
- **CubeSat Navigation**: Position/velocity estimation
- **Sensor Fusion**: Combine GPS + IMU
- **Drone Autopilot**: State estimation
- **Robotics**: Localization
- **Tracking Systems**: Object position prediction
- **Vibration Filtering**: Remove high-frequency noise

## Filter Characteristics

### Prediction
- **Process Model**: Constant velocity + acceleration
- **Prediction Error**: Grows between updates
- **Update Rate**: Fast (100Hz) for responsive tracking

### Measurement Update
- **Measurement Model**: Direct position observation
- **Innovation**: Difference between measurement and prediction
- **Correction**: Applied to both position and velocity
- **Update Rate**: Slower (10Hz) typical for GPS

## Design Notes
1. **Simplified Kalman**: Fixed gains, not adaptive
2. **No Covariance**: P matrix not computed (reduces complexity)
3. **3-Axis Independent**: Each axis filtered separately
4. **Tuning**: Adjust Kp/Kv for different noise characteristics
5. **Scaling**: Fixed-point requires careful overflow management

## Resource Usage
- **Multipliers**: ~9 (3 axes × 3 operations)
- **Registers**: ~200 (state + intermediate values)
- **Logic**: Moderate (fixed-point arithmetic)

## Tuning Guidelines

### Increase Kp (position gain)
- Trusts measurements more
- Faster convergence
- More noise in output

### Increase Kv (velocity gain)
- Faster velocity adaptation
- May cause instability if too high

### Typical Values
- **High measurement noise**: Kp=0.5, Kv=0.01
- **Low measurement noise**: Kp=0.9, Kv=0.1
- **Current implementation**: Kp=0.8, Kv=0.05

## Limitations
1. **Linear Model**: Assumes constant velocity
2. **Fixed Gains**: Not optimal for all scenarios
3. **No Outlier Rejection**: Bad measurements affect estimate
4. **Single Variable**: No cross-axis correlation
5. **No Covariance**: Can't assess estimate uncertainty
