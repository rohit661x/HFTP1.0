# HFT-OS: A Custom Bare-Metal Operating System for High-Frequency Trading

**Proof of Concept | Experimental Infrastructure Project**

---

## Overview

This project explores the design and implementation of an ultra-low-latency, kernel-less custom operating system built from scratch specifically for high-frequency trading (HFT). The goal is to create a system that sits in the performance gray zone between fully hardcoded FPGA solutions and customized Linux-based software stacks.

This minimal bare-metal runtime:
- Has no kernel, no syscalls, no context switches
- Offers deterministic CPU access without schedulers
- Interfaces directly with a network interface card (NIC)
- Is flexible and reprogrammable, unlike fixed-function FPGA logic

---

## Motivation

Modern HFT infrastructure typically leans toward two extremes:

| Approach        | Pros                                    | Cons                                                             |
|-----------------|-----------------------------------------|------------------------------------------------------------------|
| FPGA            | Hard real-time, sub-microsecond latency | Inflexible, costly, difficult to modify                          |
| Customized Linux| Flexible, large ecosystem.              | Suffers from syscall overhead, jitter, and general-purpose bloat |


This project explores a third approach — a custom operating system that:
- Runs directly on bare metal
- Interfaces with the NIC using memory-mapped I/O (MMIO) and polling
- Executes application logic deterministically
- Minimizes stack complexity while retaining programmability

---

## Architecture Overview

                +--------------------------+
                |      Stock Exchange      |
                +--------------------------+
                           ⇅
                    [Market Data Feed]
                           ⇅
                +--------------------------+
                |       NIC (RX/TX)        |
                |  - MMIO / Polling Driver |  <-- Ethernet frame order & duplications 
                |  - Time Stamping (HW)    |      are checked by CRC and FCS
                +--------------------------+
                           ⇅
                +--------------------------+
                |   Custom Bare-Metal OS   |
                |                          |
                | +----------------------+ |
                | |  Market Data Ingest  | |  <-- RX Ring (polling)
                | +----------------------+ |
                | +----------------------+ |
                | |   Strategy Engine    | |  <-- Pinned CPU core
                | +----------------------+ |
                | +----------------------+ |
                | | Order Execution Engine| |  <-- TX Ring (write-only)
                | +----------------------+ |
                +--------------------------+
                           ⇅
                [Ultra-Low Latency Switch]
                           ⇅
                +--------------------------+
                |      Stock Exchange      |
                +--------------------------+

    


- **NIC (Network Interface Card)**:
  - Uses memory-mapped I/O (MMIO) to expose RX/TX descriptor rings directly to the OS.
  - Timestamps packets using hardware-based PTP or internal NIC clock.
  - Bypasses kernel — driver communicates directly via PCIe.

- **Custom OS**:
  - No kernel, no scheduler, no system calls.
  - Market Data Ingest: Processes packets directly off the RX ring.
  - Strategy Engine: Hard-coded or config-driven trading logic pinned to a specific CPU core.
  - Order Execution: Prepares and injects packets into the TX ring (NIC DMA handles transmission).

- **Ultra-Low Latency Switch**:
  - Layer 2/3 network appliance that forwards packets with sub-microsecond jitter.
  - Ensures deterministic and minimal-hop routing to exchange.

- **Exchange**:
  - Acts as both market data publisher and order receiver.

Each instance of the OS runs on a pinned CPU core. The logic flow includes:
- Receiving raw market data from the NIC via polling
- Executing a fixed-purpose trading strategy
- Transmitting orders via the NIC with minimal delay

There are no system calls, no kernel-user transitions, and no scheduling overhead — just tight control of every clock cycle.

---

## System Design

- Architecture: x86_64 (UEFI or BIOS boot)
- Timing: Hardware timestamping via NIC or HPET
- I/O Model: MMIO-based polling loops (RX/TX queues)
- Network: Ethernet (10G/25G), raw packet-level access
- Execution: Single address space, monolithic binary runtime
- Logging: Optional ring-buffer logging or serial out


[1] Bootloader            ← Hardware-tied
[2] NIC Driver            ← Mix: low-level hardware + C++ (Maybe Assembly in the future)
[3] ITCH Parser           ← Pure software logic 
[4] Order Logic Engine    ← Software (modular logic, decision trees)
[5] TX Engine             ← Mix: software + register control
[6] Benchmarks            ← Software testing with hardware constraints

---

## Core Components

### Bootloader
- Written in x86 assembly (NASM)
- Initializes CPU to long mode, sets up paging
- Loads runtime binary into memory and jumps to entry point

### Runtime Engine
- Freestanding C/C++ code (no libc)
- NIC driver initialization and polling
- Strategy execution loop (user-defined logic)
- Direct TX ring writes for order dispatch

### NIC Driver
- Targets Intel 82599, Mellanox, or other PCIe-based NICs
- Initializes descriptor rings 
- Performs zero-copy packet handling
- Uses polling for minimal latency

### Time Synchronization (Optional)
- PTP or NIC-level timestamps for precise nanosecond accuracy
- Used to align local system time with exchange clocks

---

## Build & Run

### Requirements
- x86_64 host (bare metal or QEMU)
- NASM, GCC (freestanding), LD
- PCIe NIC (Intel/Mellanox recommended)
- USB stick or PXE bootloader for deployment

### Build Instructions

```bash
make bootloader
make runtime
make image



