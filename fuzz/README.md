# Fuzzing dot15d4

This crate fuzzes the parts of the stack that read attacker-controlled bytes,
and the parts that hold and reuse frame buffers. It supports the memory-safety
claim with measurement instead of assertion. The stack performs no unchecked
memory access, so a defect appears as a panic, an arithmetic overflow, or a
memory fault under the sanitizer, never as silent corruption.

The targets run with debug assertions and overflow checks on.

## Targets

- `frame` (T1): parse arbitrary bytes as an incoming MPDU. Runs the full read
  surface: frame control, sequence number, addressing, Information Elements, and
  payload.
- `repr` (T2): build a frame from a generated description, then parse it back and
  check that the parser reads what the builder wrote.
- `ack` (T3): parse and validate arbitrary bytes as an acknowledgment.
- `tsch` (T4): drive the TSCH state machine through an arbitrary sequence of
  radio events while it holds and reuses frame buffers. The oracle checks that
  the task never panics and that no buffer is dropped instead of returned.

## Requirements

Fuzzing needs a nightly toolchain and `cargo-fuzz`:

```
rustup toolchain install nightly
cargo install cargo-fuzz
```

## Build

Build every target without running it :

```
cargo +nightly fuzz build
```

## Run

Run one target. Without a time or run limit it runs until stopped:

```
cargo +nightly fuzz run frame
cargo +nightly fuzz run repr
cargo +nightly fuzz run ack
cargo +nightly fuzz run tsch
```

