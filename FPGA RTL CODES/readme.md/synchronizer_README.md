# Clock Domain Crossing Synchronizer

## Overview
A metastability-hardened synchronizer for safely crossing asynchronous signals between clock domains using a flip-flop chain.

## Features
- **Configurable Stages**: 2-FF or 3-FF synchronization chain
- **Vector Support**: Synchronize multi-bit signals
- **FPGA-Optimized**: Uses ASYNC_REG attribute for Xilinx tools
- **No Reset**: Recommended CDC practice for maximum reliability
- **Metastability Mitigation**: Multi-stage sampling reduces MTBF

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `STAGES` | 2 | Number of synchronization stages (2 or 3) |
| `WIDTH` | 1 | Bit width of signal to synchronize |

## Port Description

### Inputs
- `dst_clk`: Destination clock domain
- `async_in[WIDTH-1:0]`: Asynchronous input signal

### Outputs
- `sync_out[WIDTH-1:0]`: Synchronized output in dst_clk domain

## Operation

### Metastability Protection
When a signal transition occurs near a clock edge, a flip-flop may enter a metastable state. The synchronizer chain allows metastability to resolve before the signal is used.

### Synchronization Stages
- **2-Stage (Standard)**: Sufficient for most applications
  - MTBF typically > 10^6 years at moderate frequencies

- **3-Stage (High Reliability)**: For critical applications
  - MTBF > 10^12 years
  - Adds one cycle of latency

### ASYNC_REG Attribute
The `ASYNC_REG = "TRUE"` attribute:
- Prevents optimization across synchronizer FFs
- Places FFs close together in layout
- Ensures timing analysis treats path correctly
- Xilinx Vivado/ISE specific

## Timing Characteristics
- **Latency**: STAGES clock cycles in dst_clk domain
- **Uncertainty**: ±1 clock cycle edge alignment
- **MTBF**: Depends on stages, clock frequency, and technology

## Usage Examples

### Example 1: Single-Bit Synchronizer
```systemverilog
synchronizer #(
    .STAGES(2),
    .WIDTH(1)
) sync_enable (
    .dst_clk(fast_clk),
    .async_in(async_enable_signal),
    .sync_out(sync_enable_signal)
);
```

### Example 2: Multi-Bit Synchronizer
```systemverilog
synchronizer #(
    .STAGES(2),
    .WIDTH(8)
) sync_status (
    .dst_clk(system_clk),
    .async_in(async_status_bus),
    .sync_out(sync_status_bus)
);
```

### Example 3: High-Reliability Synchronizer
```systemverilog
synchronizer #(
    .STAGES(3),          // 3-FF for critical signals
    .WIDTH(1)
) sync_critical (
    .dst_clk(control_clk),
    .async_in(critical_fault_signal),
    .sync_out(sync_fault_signal)
);
```

## Important Considerations

### ⚠️ Multi-Bit Signals
When synchronizing multi-bit buses:
- **Gray Code**: Use for counters/pointers
- **One-Hot**: Safe if only one bit changes
- **Binary**: May have coherency issues during transitions
- **Consider**: Using handshake protocols for data buses

### Best Practices
1. Only synchronize control signals (enable, flags, resets)
2. Use Gray code for multi-bit counters crossing domains
3. Use FIFOs or handshake protocols for data buses
4. Never synchronize high-speed toggling signals
5. Verify CDC constraints in timing analysis

## CDC Violations to Avoid
- ❌ Directly connecting async signals to data paths
- ❌ Using synchronized signal before it stabilizes
- ❌ Synchronizing rapidly changing data buses
- ❌ Placing combinatorial logic between stages

## Resource Usage
- **Flip-Flops**: STAGES × WIDTH
- **Logic**: None (pure register chain)

## Applications
- Button/switch inputs to FPGA
- Asynchronous FIFO flag synchronization
- External interrupt signals
- Cross-clock domain handshake
- Reset synchronization
- Status flag crossing
- Sensor inputs to clock domain

## Verification
Always verify CDC paths with:
- Static timing analysis (STA)
- CDC-specific tools (Spyglass CDC, Questa CDC)
- Timing constraints for false paths
- Simulation with random delays
