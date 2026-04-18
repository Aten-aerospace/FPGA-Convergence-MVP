# CS11 — Telemetry Encoder & Command Interface

> Packages ADCS, orbit, laser, and housekeeping data into CCSDS-framed telemetry packets and transmits them over 8N1 UART at 115200 baud; receives and dispatches uplink commands over the same physical interface.

---

## Overview

| Attribute | Value |
|---|---|
| Requirements | CS-TLM-001 through CS-TLM-008 |
| Top module | `telemetry_wrapper` |
| Clock domain | `sys_clk` (50 MHz) |
| CE strobes | 1 Hz `ce_1hz` (superframe trigger), 1 kHz `ce_1ms` (arbiter tick) |
| UART baud | 115,200 bps 8N1 (configurable via `BAUD_HZ`) |
| Frame format | SYNC(4) \| APID(2) \| SEQ(2) \| LEN(2) \| TS(4) \| PAYLOAD(N) \| CRC16(2) |
| Superframe | HK@0ms, ADCS@100ms, ORBIT@200ms, LASER@300ms |
| BRAM | 768 B |
| DSP48 | 0 |

---

## File Structure

| File | Purpose |
|---|---|
| `telemetry_wrapper.sv` | Top-level — integrates downlink and uplink paths |
| `tlm_arbiter.sv` | Superframe scheduler; selects payload channel per slot |
| `ccsds_encoder.sv` | CCSDS-aligned framing: SYNC + header + payload + CRC16 |
| `command_decoder.sv` | UART RX; sync detection; CRC16 validation; field extraction |
| `command_dispatcher.sv` | Routes decoded commands to AXI4-Lite write channel |
| `tle_parser.sv` | Decodes 69-char ASCII TLE bytes for CS9 uplink |
| `tb_telemetry_wrapper.sv` | Self-checking directed testbench |

Shared primitives used: `uart_controller.sv`, `crc_calc.sv`, `async_fifo.sv`, `mavlink_parser.sv`.

---

## Module Interface

```systemverilog
module telemetry_wrapper #(
    parameter int CLK_HZ  = 100_000_000,
    parameter int BAUD_HZ = 115_200
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        ce_1hz,              // 1 Hz superframe trigger
    input  logic        ce_1ms,              // 1 kHz arbiter tick

    // Telemetry payload inputs (pre-packed byte arrays)
    input  logic [7:0]  adcs_tlm  [0:43],   // 44 bytes (CS-TLM-003)
    input  logic        adcs_valid,
    input  logic [7:0]  orbit_tlm [0:46],   // 47 bytes (CS-TLM-004)
    input  logic        orbit_valid,
    input  logic [7:0]  laser_tlm [0:19],   // 20 bytes (CS-TLM-005)
    input  logic        laser_valid,
    input  logic [7:0]  hk_tlm    [0:17],   // 18 bytes (CS-TLM-007)
    input  logic        hk_valid,

    input  logic [31:0] uptime_sec,          // MET timestamp for frame header

    // UART downlink
    output logic        uart_tx,

    // UART uplink
    input  logic        uart_rx,

    // Decoded command outputs
    output logic        cmd_valid,
    output logic [15:0] cmd_apid,
    output logic [7:0]  cmd_code,
    output logic [7:0]  cmd_data [0:15],
    output logic [4:0]  cmd_data_len,
    output logic        cmd_crc_error,
    output logic        cmd_sync_error,

    // AXI4-Lite master (command dispatch)
    output logic [31:0] axi_awaddr,
    output logic        axi_awvalid,
    input  logic        axi_awready,
    output logic [31:0] axi_wdata,
    output logic        axi_wvalid,
    input  logic        axi_wready
);
```

---

## Functionality

### Downlink path
1. **Superframe** (`tlm_arbiter`) — `ce_1hz` starts a 1-second superframe. Slots at 0/100/200/300 ms select HK/ADCS/ORBIT/LASER payloads respectively via `ce_1ms` tick counter.
2. **Frame build** (`ccsds_encoder`) — prepends 4-byte sync word, APID, sequence counter, length, and 4-byte MET timestamp. Appends CRC16/CCITT over entire frame.
3. **UART TX** — serialises frame bytes at `CLK_HZ / BAUD_HZ` (868 cycles/bit @ 115200 bps); 8N1 framing.

### Uplink path
1. **UART RX** (`command_decoder`) — detects sync word, extracts APID, command code, data payload, and CRC16. Sets `cmd_crc_error` or `cmd_sync_error` on failures.
2. **Dispatch** (`command_dispatcher`) — maps `cmd_apid` + `cmd_code` to AXI4-Lite register writes for runtime subsystem configuration.

### Frame sizes (total bytes including header + CRC)
| Packet | APID | Payload | Frame |
|---|---|---|---|
| HK | 0x0100 | 18 B | 34 B |
| ADCS | 0x0101 | 44 B | 60 B |
| Orbit | 0x0102 | 47 B | 63 B |
| Laser | 0x0103 | 20 B | 36 B |

---

## Simulation Instructions

```bash
iverilog -g2012 -o sim_cs11 \
  CS11_TELEMETRY/tb_telemetry_wrapper.sv \
  CS11_TELEMETRY/telemetry_wrapper.sv \
  CS11_TELEMETRY/tlm_arbiter.sv \
  CS11_TELEMETRY/ccsds_encoder.sv \
  CS11_TELEMETRY/command_decoder.sv \
  CS11_TELEMETRY/command_dispatcher.sv \
  CS11_TELEMETRY/tle_parser.sv \
  uart_controller.sv crc_calc.sv async_fifo.sv mavlink_parser.sv

vvp sim_cs11
```

```tcl
vlog -sv CS11_TELEMETRY/tb_telemetry_wrapper.sv CS11_TELEMETRY/*.sv uart_controller.sv crc_calc.sv async_fifo.sv mavlink_parser.sv
vsim -t 1ps tb_telemetry_wrapper -do "run -all; quit"
```

---

## Testbench Description

| Aspect | Detail |
|---|---|
| Type | Directed self-checking |
| Clock | 50 MHz |
| CE | 1 Hz and 1 kHz internally generated |
| Stimulus | Pre-filled payload byte arrays; loopback uart_tx → uart_rx for command round-trip |
| Checking | CRC16 of transmitted frames; decoded command fields match injected values; `cmd_crc_error` on corrupted byte |
| Coverage | All 4 superframe slots, CRC error injection, sync-word miss, AXI4-Lite dispatch |

---

## Expected Behaviour

```
ce_1hz              __|‾|______________________________
ce_1ms              |‾|_|‾|_|‾|_|‾|  ...  (1 kHz)
Slot 0 (HK @ 0ms):  uart_tx: SYNC + APID 0x0100 + 18B payload + CRC
Slot 1 (ADCS @ 100ms): uart_tx: SYNC + APID 0x0101 + 44B payload + CRC
tlm_valid:          |‾| briefly after each frame transmission completes
uart_rx (uplink):   SYNC + APID + CMD_CODE + DATA + CRC → cmd_valid pulse
cmd_crc_error:      _______ (only if RX CRC mismatch)
```

---

## Limitations

- Superframe schedule is fixed-slot; no adaptive priority for burst fault telemetry.
- Command authentication (HMAC, sequence number replay check) not implemented in MVP.
- No ARQ / retransmission for packet loss on the downlink.
- UART baud is a compile-time parameter; runtime baud change not supported.

---

## Verification Status

- [x] Compiles without warnings (`iverilog -g2012`)
- [x] All 4 superframe slots transmit in correct time order
- [x] CRC16 of each frame verified by testbench
- [x] `cmd_valid` asserts on correctly framed uplink command
- [x] `cmd_crc_error` asserts on corrupted command byte
- [x] AXI4-Lite dispatch generates correct address/data
- [x] Integrated in `top_cubesat_mvp` (CS12)
- [ ] Adaptive schedule / QoS for burst faults
- [ ] Command security (replay protection, HMAC)
