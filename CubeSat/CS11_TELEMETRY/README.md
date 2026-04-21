# CS11 — Telemetry Encoder & Multiplexer

## 1. Module Title & ID

**Module:** CS11 — Telemetry Encoder & Multiplexer (CCSDS)
**Subsystem ID:** CS11
**Requirements:** CS-TLM-001 through CS-TLM-008

---

## 2. Overview

CS11 assembles and transmits CCSDS-compatible telemetry frames over UART at up to 115,200 bps. It schedules four packet types (HK, ADCS, Orbit, Laser) in a 1-second superframe, builds framed packets with SYNC + APID + sequence counter + timestamp + payload + CRC-16/CCITT-FALSE, and serialises them to the UART TX output. An uplink path (UART RX) decodes incoming command frames, validates CRC, and dispatches routing via AXI4-Lite master writes to target subsystem registers.

**Target Platform:** Xilinx Artix-7 XC7A35T

---

## 3. Criticality

**HIGH** — CS-TLM-001 to CS-TLM-008. Telemetry is the primary downlink channel for health monitoring and mission data. Loss of telemetry does not affect ADCS, but prevents ground station visibility into system state. Command uplink (CS-TLM-008) enables ground-commanded mode transitions.

---

## 4. Key Functionality

**Downlink Path:**
- `tlm_arbiter` schedules 4 packet slots in a 1-second superframe: HK @ 0 ms, ADCS @ 100 ms, Orbit @ 200 ms, Laser @ 300 ms.
- `tlm_frame_builder` assembles: `SYNC(4) | APID(2) | SEQ(2) | LEN(2) | TS(4) | PAYLOAD(N) | CRC16(2)`.
- CRC-16/CCITT-FALSE (`crc_calc` reused) computed over APID + SEQ + LEN + TS + PAYLOAD.
- 8N1 UART TX at configurable baud rate (default 115,200 bps) via `uart_controller`.
- Total frame sizes: HK = 34 B, ADCS = 60 B, Orbit = 63 B, Laser = 36 B.

**Uplink Path:**
- `command_decoder` parses UART RX stream; validates sync word and CRC-16; outputs `cmd_apid`, `cmd_code`, `cmd_data[0:15]`, `cmd_data_len`.
- `command_dispatcher` routes by APID to target subsystem via AXI4-Lite master write.
- `cmd_crc_error` and `cmd_sync_error` flags for telemetry error reporting.

**Sequence Counter:**
- 16-bit `seq_cnt` increments on each transmitted frame; exposed for gap detection.

---

## 5. Inputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `clk` | input | 1 | `sys_clk` | 100 MHz system clock |
| `rst_n` | input | 1 | async | Active-low synchronous reset |
| `ce_1hz` | input | 1 | `sys_clk` | 1 Hz superframe trigger from CS12 |
| `ce_1ms` | input | 1 | `sys_clk` | 1 kHz arbiter tick from CS12 |
| `adcs_tlm[0:43]` | input | 44 × 8 | `sys_clk` | 44-byte ADCS packet (CS-TLM-003) |
| `adcs_valid` | input | 1 | `sys_clk` | ADCS packet data valid |
| `orbit_tlm[0:46]` | input | 47 × 8 | `sys_clk` | 47-byte Orbit packet (CS-TLM-004) |
| `orbit_valid` | input | 1 | `sys_clk` | Orbit packet data valid |
| `laser_tlm[0:19]` | input | 20 × 8 | `sys_clk` | 20-byte Laser packet (CS-TLM-005) |
| `laser_valid` | input | 1 | `sys_clk` | Laser packet data valid |
| `hk_tlm[0:17]` | input | 18 × 8 | `sys_clk` | 18-byte HK packet (CS-TLM-007) |
| `hk_valid` | input | 1 | `sys_clk` | HK packet data valid |
| `uptime_sec[31:0]` | input | 32 | `sys_clk` | Mission elapsed time in seconds (frame timestamp) |
| `uart_rx` | input | 1 | `sys_clk` | UART uplink RX (command reception) |
| `axi_awready` | input | 1 | `sys_clk` | AXI4-Lite write address ready (from target) |
| `axi_wready` | input | 1 | `sys_clk` | AXI4-Lite write data ready |
| `axi_bresp[1:0]` | input | 2 | `sys_clk` | AXI4-Lite write response |
| `axi_bvalid` | input | 1 | `sys_clk` | AXI4-Lite write response valid |

---

## 6. Outputs

| Port | Direction | Width | Clock Domain | Description |
|---|---|---|---|---|
| `uart_tx` | output | 1 | `sys_clk` | UART downlink TX (8N1) |
| `tlm_valid` | output | 1 | `sys_clk` | Pulse on each `frame_done` |
| `tlm_busy` | output | 1 | `sys_clk` | Asserted while frame is being transmitted |
| `cmd_valid` | output | 1 | `sys_clk` | Decoded uplink command valid |
| `cmd_apid[15:0]` | output | 16 | `sys_clk` | Command APID |
| `cmd_code[7:0]` | output | 8 | `sys_clk` | Command code byte |
| `cmd_data[0:15]` | output | 16 × 8 | `sys_clk` | Command payload (up to 16 bytes) |
| `cmd_data_len[4:0]` | output | 5 | `sys_clk` | Valid bytes in `cmd_data` |
| `cmd_crc_error` | output | 1 | `sys_clk` | Uplink CRC mismatch |
| `cmd_sync_error` | output | 1 | `sys_clk` | Uplink sync word not found |
| `axi_awaddr[31:0]` | output | 32 | `sys_clk` | AXI4-Lite write address (to target subsystem) |
| `axi_awvalid` | output | 1 | `sys_clk` | AXI4-Lite write address valid |
| `axi_wdata[31:0]` | output | 32 | `sys_clk` | AXI4-Lite write data |
| `axi_wvalid` | output | 1 | `sys_clk` | AXI4-Lite write data valid |
| `axi_bready` | output | 1 | `sys_clk` | AXI4-Lite write response ready |

---

## 7. Architecture

```
ce_1hz (1 Hz superframe trigger)
      │
      ▼
┌────────────────────────────────────────────────────────────────────┐
│  telemetry_wrapper                                                  │
│                                                                    │
│  ┌───────────────┐  slot 0–3    ┌──────────────────────────────┐  │
│  │  tlm_arbiter  │─────────────▶│  tlm_frame_builder           │  │
│  │  (1-sec super-│  payload mux │  SYNC(4)|APID(2)|SEQ(2)|     │  │
│  │   frame sched)│              │  LEN(2)|TS(4)|PAYLOAD|CRC(2) │  │
│  └───────────────┘              └────────────────┬─────────────┘  │
│  Slots:                                           │ frame bytes    │
│   0ms  → HK    (APID 0x0100, 18 B)               ▼                │
│   100ms → ADCS (APID 0x0101, 44 B)   ┌─────────────────────────┐  │
│   200ms → Orbit(APID 0x0102, 47 B)   │  uart_controller (TX)   │  │
│   300ms → Laser(APID 0x0103, 20 B)   │  8N1, 115200 bps        │  │──▶ uart_tx
│                                       └─────────────────────────┘  │
│                                                                    │
│  uart_rx ──▶ ┌──────────────────┐  cmd fields  ┌──────────────┐  │
│              │  command_decoder  │─────────────▶│  command_    │  │
│              │  (sync, CRC check)│              │  dispatcher  │──│──▶ AXI4-Lite
│              └──────────────────┘              └──────────────┘  │
└────────────────────────────────────────────────────────────────────┘
```

**CCSDS Frame Format:**

```
Byte  0–3   : SYNC word  0x1A 0xCF 0xFC 0x1D
Byte  4–5   : APID (MSB first)
Byte  6–7   : SEQ counter (MSB first)
Byte  8–9   : LEN = payload length in bytes
Byte  10–13 : Timestamp = uptime_sec (32-bit, MSB first)
Byte  14–13+N: PAYLOAD (N bytes)
Last 2 bytes: CRC-16/CCITT-FALSE
```

**Reused Helper IPs (from `CubeSat/`):**
- `uart_controller.sv` — UART TX/RX engine @ configurable baud
- `crc_calc.sv` — CRC-16/CCITT-FALSE (polynomial 0x1021, init 0xFFFF)
- `async_fifo.sv` — UART RX → sys_clk domain FIFO (CDC)

---

## 8. Data Formats

| Signal | Format | Notes |
|---|---|---|
| `hk_tlm[0:17]` | 18 × uint8 | HK packet bytes; 18 B payload |
| `adcs_tlm[0:43]` | 44 × uint8 | ADCS packet bytes; 44 B payload |
| `orbit_tlm[0:46]` | 47 × uint8 | Orbit packet bytes; 47 B payload |
| `laser_tlm[0:19]` | 20 × uint8 | Laser packet bytes; 20 B payload |
| APID | 16-bit | HK=0x0100, ADCS=0x0101, Orbit=0x0102, Laser=0x0103 |
| Baud period | 868 clk cycles | = CLK_HZ / BAUD_HZ = 100e6 / 115200 |
| CRC polynomial | 0x1021 | CCITT-FALSE; init 0xFFFF |

---

## 9. Register Interface

CS11 itself has **no AXI4-Lite slave registers** in the current implementation. CS11 acts as an **AXI4-Lite master** (`command_dispatcher`) to write to target subsystem registers after decoding uplink commands.

**Parameters (synthesised):**

| Parameter | Default | Description |
|---|---|---|
| `CLK_HZ` | 100_000_000 | System clock frequency in Hz |
| `BAUD_HZ` | 115_200 | UART baud rate |

---

## 10. File Structure

```
CubeSat/CS11_TELEMETRY/
├── telemetry_wrapper.sv      ← Top-level wrapper; CS12 integration point
├── tlm_arbiter.sv            ← 1-second superframe scheduler; slot arbitration
├── tlm_frame_builder.sv      ← CCSDS frame assembly: SYNC+APID+SEQ+LEN+TS+CRC
├── command_decoder.sv        ← UART RX parsing; sync word + CRC validation
├── command_dispatcher.sv     ← Route commands to targets via AXI4-Lite master
├── tb_telemetry_wrapper.sv   ← Integration testbench
└── README.md                 ← This file

CubeSat/ (shared helper IPs used by CS11):
├── uart_controller.sv        ← UART TX/RX (8N1, configurable baud)
├── crc_calc.sv               ← CRC-16/CCITT-FALSE calculator
└── async_fifo.sv             ← UART RX → sys_clk CDC FIFO
```

---

## 11. Interconnections

| Signal | Direction | Connected Module | Purpose |
|---|---|---|---|
| `ce_1hz` | CS11 ← CS12 | `clk_manager` (CS12) | 1 Hz superframe trigger |
| `ce_1ms` | CS11 ← CS12 | `clk_manager` (CS12) | 1 kHz arbiter tick |
| `hk_tlm[0:17]` | CS11 ← CS12 | `top_cubesat_mvp` (CS12) | Housekeeping telemetry |
| `adcs_tlm[0:43]` | CS11 ← CS5/CS8 | EKF + FSM (CS5, CS8) | ADCS state telemetry |
| `orbit_tlm[0:46]` | CS11 ← CS9 | `orbit_propagator_wrapper` (CS9) | Orbit state telemetry |
| `laser_tlm[0:19]` | CS11 ← CS10 | `laser_fsm_wrapper` (CS10) | Laser pointing telemetry |
| `uptime_sec` | CS11 ← CS9 | `orbit_propagator_wrapper` (CS9) | MET timestamp for frame header |
| `uart_tx` | CS11 → Physical | FPGA I/O | Radio transceiver or debug UART |
| `uart_rx` | CS11 ← Physical | FPGA I/O | Command uplink from ground |
| AXI4-Lite master | CS11 → CS6 | `pd_control_wrapper` (CS6) | Gain update via command dispatcher |

---

## 12. Design Considerations / Optimization Scope

**Performance:**
- Frame transmission time @ 115,200 bps: HK=34 B → 3.0 ms; ADCS=60 B → 5.2 ms; Orbit=63 B → 5.5 ms; Laser=36 B → 3.1 ms. All fit within 100 ms inter-slot gap.
- CRC computation: pipeline in `crc_calc`; 1 byte per clock cycle.

**Resource:**
- BRAM: 768 B (per plan) for frame buffer and FIFO storage.
- DSP48E1: 0 (no arithmetic intensive operations).

**Optimization Opportunities:**
1. Add configurable baud rate register (AXI4-Lite) for ground-commanded rate change.
2. Extend `tlm_arbiter` to support variable-slot scheduling based on `*_valid` flags.
3. Add RS-232 error detection (parity) to `uart_controller` for noisy link environments.
4. Buffer multiple frames for burst download during contact windows.

**Timing:**
- UART bit period = CLK_HZ / BAUD_HZ clock cycles; must be stable across temperature.
- `async_fifo` CDC: ensure correct FIFO grey-code pointer synchronisation.

---

## 13. Testing & Verification

**Testbench:** `CubeSat/CS11_TELEMETRY/tb_telemetry_wrapper.sv`

**Test Scenarios:**
- Assert `hk_valid` and drive `hk_tlm[0:17]`; verify HK frame transmitted on `ce_1hz` with correct APID 0x0100.
- Verify SYNC word (0x1A 0xCF 0xFC 0x1D) appears at start of each frame on `uart_tx`.
- Drive `adcs_tlm`, `orbit_tlm`, `laser_tlm` with known byte sequences; verify CRC-16 matches golden reference.
- Increment `uptime_sec`; verify timestamp field in frame header updates.
- Inject valid uplink command frame on `uart_rx`; verify `cmd_valid` asserts and `cmd_apid/code/data` are correct.
- Inject malformed uplink frame; verify `cmd_crc_error` or `cmd_sync_error` asserts.
- Verify `seq_cnt` increments by 1 per transmitted frame.
- Verify `tlm_busy` is asserted during frame transmission and `tlm_valid` pulses on `frame_done`.

**Simulation Notes:**
- Compile with `iverilog -g2012` including `uart_controller.sv`, `crc_calc.sv`, `async_fifo.sv`.
- Timescale: 1 ns / 1 ps.
- Include UART RX model to inject command frames.

**Requirements Coverage:**
- CS-TLM-001: CCSDS frame format SYNC+APID+SEQ+LEN+TS+PAYLOAD+CRC16.
- CS-TLM-003/004/005/007: ADCS(44 B), Orbit(47 B), Laser(20 B), HK(18 B) packet formats.
- CS-TLM-006: 1 Hz superframe with 100 ms inter-slot spacing.
- CS-TLM-008: UART RX command reception with CRC validation and AXI4-Lite routing.
- Architecture: `Architecture/SUBSYSTEM_MODULE_MAPPING.md`
