# dot15d4 [![codecov](https://codecov.io/gh/thvdveld/dot15d4/graph/badge.svg?token=XETJ1SV5B0)](https://codecov.io/gh/thvdveld/dot15d4) ![example workflow](https://github.com/thvdveld/dot15d4/actions/workflows/rust.yml/badge.svg)

`dot15d4` is an IEEE 802.15.4 implementation written in Rust.
The library is designed to be used in embedded systems, and is `no_std` by default.

The `dot15d4-frame` library is used to parse and emit IEEE 802.15.4 frames.
It uses the same style of parsing and emitting as the [`smoltcp`](https://github.com/smoltcp-rs/smoltcp) library.

Another library that is similar to `dot15d4-frame` is the [`ieee802154`](https://github.com/rust-iot/rust-ieee802.15.4) library.

The `dot15d4-macros` library contains utility macros that are used in the `dot15d4-frame` library.

## Usage

> [!NOTE]
> This library is still in development and is not yet published to crates.io.

Add this to your `Cargo.toml`:

```toml
[dependencies]
dot15d4 = "0.1.0"
```

### Configurable features

- `std`: Enables `std` only features
- `log`: Use the `log` crate for structured logging
- `defmt`: Use the `defmt` crate for structured logging

### Configurable environment variables

- `DOT15D4_MAC_MIN_BE` (default: 0): Minimum backoff exponent used in `CSMA`
- `DOT15D4_MAC_MAX_BE` (default: 8): Maximum backoff exponent used in `CSMA`
- `DOT15D4_MAC_MAX_FRAME_RETRIES` (default: 3): Maximum CCA/ACK rounds

For more information, see the [API documentation](https://docs.rs/dot15d4).

## dot15d4 as a binary

This repository also contains a binary that can be used to parse IEEE 802.15.4 frames.

### Usage

```sh
dot15d4 40ebcdabffff0100010001000100003f1188061a0e0000000000011c0001c800011b00
```

Output:

```txt
Frame Control
  frame type: Enhanced Beacon
  security: 0
  frame pending: 0
  ack request: 0
  pan id compression: 1
  sequence number suppression: 1
  information elements present: 1
  dst addressing mode: Short
  src addressing mode: Extended
  frame version: 2 (Ieee802154_2020)
Addressing
  dst pan id: abcd
  dst addr: ff:ff (broadcast)
  src addr: 00:01:00:01:00:01:00:01
Information Elements
  Header Information Elements
    HeaderTermination1
  Payload Information Elements
    MLME
      TschSynchronization
        ASN: 14, join metric: 0
      TschTimeslot
        slot ID: 0
      ChannelHopping
        sequence ID: 0
      TschSlotframeAndLink
        #slot frames: 0
```

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
