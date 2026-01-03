# AXI-4 DMA Controller

## Overview
The AXI4 DMA Subsystem is a high-performance, single-channel Direct Memory Access (DMA)
controller. It bridges an AXI4-Lite control plane with a high-bandwidth AXI4 data plane to move data
between memory regions without CPU intervention.

## Key Features
- **High Performance**: AXI4 Master with 128-bit data path, single-cycle throughput (100%).
- **Robust Architecture**: Store-and-forward mechanism for data integrity and deadlock avoidance.
- **Strict Compliance**: Enforces 4KB boundary checks and 16-byte alignment.
- **Safety**: Independent source and destination watchdog timers.
- **Control**: Simple AXI4-Lite slave interface with status and error reporting.
- **Interrupts**: Configurable interrupt support for completion and error events.
- **Elastic Buffering**: Integrated 4KB FWFT FIFO with skid buffer for maximum bandwidth.


### Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                   AXI4 DMA Subsystem                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐      ┌─────────────────┐                  │
│  │              │      │                 │                  │
│  │ dma_reg_block│◄────►│  axi_dma_master │                  │
│  │  (AXI4-Lite) │      │   (DMA Engine)  │                  │
│  │              │      │                 │                  │
│  └──────────────┘      └────────┬────────┘                  │
│         │                       │                           │
│         │                       ▼                           │
│         │              ┌─────────────────┐                  │
│         │              │  fifo_bram_fwft │                  │
│         │              │  (Skid Buffer)  │                  │
│         │              └─────────────────┘                  │
│         │                                                   │
│         ▼                       ▼                           │
│   Control/Status          AXI4 Master                       │
│   (Registers)            (Memory Access)                    │
└─────────────────────────────────────────────────────────────┘
```

### Module Breakdown

**1. `axi_dma_subsystem.sv` (Top-Level Wrapper)**

- Instantiates and connects all sub-modules
- Exposes AXI4-Lite slave and AXI4 master interfaces
- Manages interrupt output

**2. `dma_reg_block.sv` (Register Interface)**

- Implements 5 memory-mapped registers (CTRL, STATUS, SRC, DST, LEN)
- Handles AXI4-Lite protocol with full backpressure support
- Provides W1C (Write-1-to-Clear) semantics for status bits
- Generates interrupt signals based on IntEn configuration

**3. `axi_4_dma.sv` (DMA Protocol Engine)**

- **Store-and-Forward Architecture**: Reads entire burst into FIFO before writing
- **Parameter Validation**: Checks alignment (16-byte), length (≤4096), 4KB boundaries
- **Watchdog Timers**: Detects stuck transactions on source/destination
- **Error Handling**: Maps AXI BRESP/RRESP errors to status codes
- **State Machine**: IDLE → VALIDATE → READ → WRITE → DONE/ERROR

**4. `fifo.sv` (FWFT FIFO with Skid Buffer)**

- **Critical Constraint**: Must NOT use combinational BRAM-to-output path
- **Skid Buffer**: Output pipeline register for timing closure
- **FWFT Behavior**: Data available on `dout` when `empty=0` (0-cycle latency)
- **Depth**: 256 entries × 128-bit data width

### Test Coverage (18 Comprehensive Tests)

1. **Basic Functionality**: Standard 64-byte transfer
2. **Control Logic**: Block start if interrupt pending
3. **Interrupt Handling**: W1C persistence and masking
4. **Register Validation**: Invalid address SLVERR responses
5. **Length Boundaries**: Zero, max (4096), oversize detection
6. **Alignment**: 16-byte alignment enforcement
7. **4KB Boundary**: Source and destination crossing detection
8. **Data Integrity**: Back-to-back transfers
9. **Stress Testing**: Random AXI delays and backpressure
10. **Overlap Handling**: Forward and reverse overlapping regions
11. **Reset Recovery**: Mid-flight asynchronous reset
12. **AXI Errors**: SLVERR/DECERR response handling
13. **Interrupt Masking**: IntEn=0 behavior
14. **Watchdog Timers**: Source/destination timeout detection
15. **Throughput**: 1 cycle/beat performance validation
16. **Protocol Invariants**: AXI signal stability and WLAST consistency
17. **FIFO Reset (Source)**: Verify FIFO soft reset prevents stale data corruption after source timeout
18. **FIFO Reset (Destination)**: Verify FIFO soft reset prevents stale data corruption after destination timeout

