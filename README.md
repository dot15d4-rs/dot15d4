# dot15d4

[![codecov](https://codecov.io/gh/thvdveld/dot15d4/graph/badge.svg?token=XETJ1SV5B0)](https://codecov.io/gh/thvdveld/dot15d4)
![CI](https://github.com/thvdveld/dot15d4/actions/workflows/rust.yml/badge.svg)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](LICENSE-MIT)

A _partial_ IEEE 802.15.4 MAC layer implementation in Rust, designed for embedded systems with strict memory and timing requirements.

## Overview

`dot15d4` aims to provide a full-featured IEEE 802.15.4 MAC layer stack targeting resource-constrained embedded devices. The implementation is `no_std` compatible, uses zero-copy frame parsing, and leverages Rust's type system to enforce correct usage at compile time.

### Key Features

- **`no_std` by default** : Suitable for bare-metal embedded systems with no heap allocations required
- **Zero-copy frame parsing** : Efficient MPDU parsing and building without intermediate buffers
- **Type-state driven design** : Compile-time guarantees for radio state machine transitions
- **Multiple MAC protocols** : CSMA-CA and TSCH (Time Slotted Channel Hopping) schedulers
- **IEEE 802.15.4 compliant** : Support for Information Elements, security headers, and enhanced frames
- **Hardware abstraction** : Clean separation between protocol logic and hardware drivers
- **Embassy integration** : First-class support for the Embassy async embedded framework

## Main Components

This workspace consists of five crates, each with a focused responsibility:

### [`dot15d4`](dot15d4/) : MAC Layer Implementation

The main crate providing the complete MAC layer stack including:

- **MAC Service** : Request/indication primitives following the IEEE 802.15.4 SAP model
- **CSMA-CA Scheduler** : Contention-based medium access with configurable backoff parameters
- **TSCH Scheduler** : Time-synchronized channel hopping for deterministic, low-power operation
- **MLME Primitives** : `RESET`, `SET`, `SCAN`, `TSCH-MODE`, `SET-SLOTFRAME`, `SET-LINK`
- **MCPS Primitives** : `DATA` request/indication for frame transmission and reception
- **PAN Information Base (PIB)** : Standard-compliant MAC attribute storage

```toml
[dependencies]
dot15d4 = { version = "0.1.2", default-features = false }
```

### [`dot15d4-frame`](dot15d4-frame/) : Frame Parsing & Building

Zero-copy IEEE 802.15.4 MPDU frame handling using a type-state pattern for safe, incremental parsing:

- **Frame Control** : All frame types (Beacon, Data, ACK, MAC Command, Multipurpose)
- **Addressing** : Short, Extended, and Absent addressing modes with PAN ID compression
- **Security Headers** : Auxiliary security header parsing (optional feature)
- **Information Elements** : Header and Payload IEs including TSCH-specific nested IEs
- **Type-state parsing** : Progressive parsing with compile-time field access guarantees

```toml
[dependencies]
dot15d4-frame = { version = "0.0.1", default-features = false }
```

### [`dot15d4-driver`](dot15d4-driver/) : Hardware Abstraction Layer

Platform-agnostic radio driver abstraction with reference implementations:

- **Type-state Radio FSM** : UML behavior state machine modeling for radio task scheduling
- **High-precision Timer API** : Nanosecond-resolution timing for deterministic operations
- **Hardware Shortcuts** : Support for operation pipelining without CPU intervention
- **nRF52840 Driver** : Complete driver for Nordic nRF52840 IEEE 802.15.4 radio
- **Async Interrupt-based Executor** : Lightweight single-threaded executor for driver tasks
- **Tracing Support** : GPIO and SystemView (RTOS-Trace) instrumentation

```toml
[dependencies]
dot15d4-driver = { version = "0.0.1", default-features = false, features = ["nrf52840"] }
```

### [`dot15d4-util`](dot15d4-util/) : Shared Utilities

Common infrastructure used across the workspace:

- **Buffer Allocator** : Static, no-heap buffer pool management
- **Async Channels** : Multi-producer multi-consumer channels with backpressure
- **Synchronization** : Async mutex, select combinators, and cancellation tokens
- **Logging Facade** : Unified interface for `log` and `defmt` backends
- **Tracing Infrastructure** : SystemView integration for real-time analysis

```toml
[dependencies]
dot15d4-util = { version = "0.0.1", default-features = false }
```

### [`dot15d4-embassy`](dot15d4-embassy/) : Embassy Integration

Seamless integration with the Embassy async embedded framework:

- **Network Driver** : `embassy-net-driver` trait implementation for 6LoWPAN stacks
- **Stack Integration** : Ready-to-use stack initialization for Embassy applications
- **nRF52840 Binding** : Embassy HAL integration for Nordic platforms

```toml
[dependencies]
dot15d4-embassy = { version = "0.0.1", default-features = false, features = ["nrf52840"] }
```

## Getting Started

> [!NOTE]
> This library is under active development. APIs may change between versions.

### Basic Usage

Add the main crate to your `Cargo.toml`:

```toml
[dependencies]
dot15d4 = { version = "0.1.2", default-features = false }
```

### Feature Flags

The `dot15d4` crate supports the following features:

| Feature | Description |
|---------|-------------|
| `default` | Enables `security`, `ies`, and strict warnings |
| `std` | Standard library support (enables `log`) |
| `log` | Structured logging via the `log` crate |
| `defmt` | Efficient logging via `defmt` for embedded targets |
| `security` | IEEE 802.15.4 security header support |
| `ies` | Information Elements parsing and building |
| `tsch` | TSCH scheduler support (implies `ies`) |
| `tsch-coordinator` | TSCH coordinator features including beacon transmission |
| `nrf52840` | Nordic nRF52840 hardware support |

### Compile-time Configuration

MAC layer parameters can be configured via environment variables at build time:

| Variable | Type | Default | Description |
|----------|------|---------|-------------|
| `DOT15D4_MAC_MIN_BE` | `u8` | `0` | Minimum backoff exponent for CSMA-CA |
| `DOT15D4_MAC_MAX_BE` | `u8` | `8` | Maximum backoff exponent for CSMA-CA |
| `DOT15D4_MAC_MAX_CSMA_BACKOFFS` | `u8` | `16` | Maximum CSMA backoff attempts |
| `DOT15D4_MAC_MAX_FRAME_RETRIES` | `u8` | `3` | Maximum frame transmission retries |
| `DOT15D4_MAC_PAN_ID` | `u16` | `0xbeef` | Default PAN identifier |
| `DOT15D4_MAC_TSCH_MIN_BE` | `u8` | `1` | Minimum backoff exponent for TSCH |
| `DOT15D4_MAC_TSCH_MAX_BE` | `u8` | `7` | Maximum backoff exponent for TSCH |
| `DOT15D4_MAC_TSCH_MAX_LINKS` | `usize` | `5` | Maximum TSCH links per slotframe |
| `DOT15D4_MAC_TSCH_MAX_SLOTFRAMES` | `usize` | `1` | Maximum number of slotframes |

Example:
```sh
DOT15D4_MAC_MAX_FRAME_RETRIES=5 cargo build --release
```

## Examples

The [`examples/nrf52840`](examples/nrf52840/) directory contains ready-to-run examples for the Nordic nRF52840 DK:

- **Radio Tests** : Low-level radio TX/RX verification
- **TSCH Coordinator** : Time-synchronized network coordinator
- **Embassy UDP** : Full network stack with smoltcp integration

Build and flash an example:
```sh
cd examples/nrf52840
COORDINATOR=1 cargo build --release --features=tsch-coordinator,defmt --bin tsch-node
probe-rs run --chip nRF52840_xxAA target/thumbv7em-none-eabihf/release/tsch-node
```

## Supported Hardware

For now, we only support nrf52840 with radio/timer driver.

## Funding

This project is funded through [NGI Zero Core](https://nlnet.nl/core), a fund established by [NLnet](https://nlnet.nl) with financial support from the European Commission's [Next Generation Internet](https://ngi.eu) program. Learn more at the [NLnet project page](https://nlnet.nl/project/TSCH-rs).

[<img src="https://nlnet.nl/logo/banner.png" alt="NLnet foundation logo" width="20%" />](https://nlnet.nl)
[<img src="https://nlnet.nl/image/logos/NGI0_tag.svg" alt="NGI Zero Logo" width="20%" />](https://nlnet.nl/core)

## Coverage

![Coverage](https://codecov.io/gh/thvdveld/dot15d4/graphs/sunburst.svg?token=XETJ1SV5B0)

## License

Licensed under either of

- Apache License, Version 2.0, ([LICENSE-APACHE](LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you,
as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
