# Asynchronous FIFO

## Overview
A dual-clock asynchronous FIFO using Gray code pointers for safe clock domain crossing with configurable depth and data width.

## Features
- **Dual Clock Domains**: Independent read and write clocks
- **Gray Code Pointers**: Safe CDC with minimal synchronization stages
- **Configurable**: Parameterized width and depth
- **Full/Empty Flags**: Reliable status indicators
- **Registered Output**: Pipelined read data
- **Power-of-2 Depth**: Efficient address wrapping

## Module Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `DATA_W` | 32 | Data bus width in bits |
| `DEPTH` | 16 | FIFO depth (must be power of 2) |

## Port Description

### Write Domain
- `wr_clk`: Write clock
- `wr_rst_n`: Write domain reset (active-low)
- `wr_data[DATA_W-1:0]`: Data to write
- `wr_en`: Write enable
- `wr_full`: Full flag (write domain)

### Read Domain
- `rd_clk`: Read clock
- `rd_rst_n`: Read domain reset (active-low)
- `rd_data[DATA_W-1:0]`: Read data output (registered)
- `rd_en`: Read enable
- `rd_empty`: Empty flag (read domain)

## Operation

### Gray Code Synchronization
Binary pointers are converted to Gray code before crossing clock domains:
- Only one bit changes per increment
- Eliminates multi-bit coherency issues
- 2-FF synchronizer stages for metastability protection

### Write Operation
1. Check `wr_full` flag
2. If not full, assert `wr_en` with valid `wr_data`
3. Data written to memory on `wr_clk` rising edge
4. Write pointer increments

### Read Operation
1. Check `rd_empty` flag
2. If not empty, assert `rd_en`
3. Data available on `rd_data` next cycle
4. Read pointer increments

### Full Detection
FIFO is full when:
- Write pointer (Gray) matches read pointer (Gray) with MSB inverted
- Indicates all locations occupied

### Empty Detection
FIFO is empty when:
- Read pointer (Gray) equals write pointer (Gray)
- Indicates no data available

## Timing Characteristics
- **Write Latency**: 1 wr_clk cycle
- **Read Latency**: 1 rd_clk cycle (registered output)
- **Full Flag Delay**: 2-3 rd_clk cycles (synchronization)
- **Empty Flag Delay**: 2-3 wr_clk cycles (synchronization)

## Usage Example

### Basic Async FIFO
```systemverilog
async_fifo #(
    .DATA_W(32),
    .DEPTH(16)
) fifo_inst (
    // Write side
    .wr_clk(fast_clk),
    .wr_rst_n(wr_rst_n),
    .wr_data(write_data),
    .wr_en(write_enable && !wr_full),
    .wr_full(wr_full),

    // Read side
    .rd_clk(slow_clk),
    .rd_rst_n(rd_rst_n),
    .rd_data(read_data),
    .rd_en(read_enable && !rd_empty),
    .rd_empty(rd_empty)
);
```

### Write Controller Example
```systemverilog
always_ff @(posedge wr_clk) begin
    if (data_available && !wr_full) begin
        wr_data <= input_data;
        wr_en <= 1'b1;
    end else begin
        wr_en <= 1'b0;
    end
end
```

### Read Controller Example
```systemverilog
always_ff @(posedge rd_clk) begin
    if (!rd_empty && consumer_ready) begin
        rd_en <= 1'b1;
    end else begin
        rd_en <= 1'b0;
    end

    if (rd_en && !rd_empty) begin
        process_data(rd_data);
    end
end
```

## Important Considerations

### ⚠️ Almost Full/Empty
This design does not include almost-full/almost-empty flags. To add them:
- Compare synchronized pointer difference to threshold
- Add safety margin for synchronization delay

### ⚠️ Reset Synchronization
Both domains must be reset properly:
- Asynchronous reset in each domain
- Hold reset for several clock cycles
- Release write reset before read reset

### ⚠️ Clock Frequency Ratio
- No restrictions on clock frequency ratio
- Works with wr_clk faster, slower, or equal to rd_clk
- Throughput limited by slower clock

## Resource Usage
- **RAM**: DEPTH × DATA_W bits (inferred dual-port RAM)
- **Flip-Flops**: ~20-30 per domain
- **Logic**: Minimal (Gray conversion + comparators)

## Applications
- Clock domain crossing for data streams
- Rate matching between modules
- Buffering between fast producer/slow consumer
- UART/SPI to system clock bridging
- Video pixel buffering
- Network packet buffering
- ADC to processing pipeline
- Multi-rate DSP systems

## Design Notes
1. **Depth must be power of 2** for efficient addressing
2. **Registered read output** adds 1 cycle latency
3. **Gray code** ensures reliable synchronization
4. **No backpressure** - check flags before read/write
5. **Overflow/underflow** - user responsible to check flags

## Common Pitfalls
- ❌ Writing when full (data lost)
- ❌ Reading when empty (invalid data)
- ❌ Ignoring flag synchronization delay
- ❌ Using non-power-of-2 depth
- ❌ Improper reset sequencing

## Verification
Key test scenarios:
- Write faster than read (fills up)
- Read faster than write (empties)
- Simultaneous read/write at same rate
- Full FIFO writes (check full flag)
- Empty FIFO reads (check empty flag)
- Random read/write enables
