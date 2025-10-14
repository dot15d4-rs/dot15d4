#!/bin/sh -e

echo "CHECK(examples): ARM, default features"
cargo clippy -p dot15d4-examples-nrf52840 --bins --target thumbv7em-none-eabihf -- -D warnings

echo "CHECK(examples): ARM, optional features"
cargo clippy -p dot15d4-examples-nrf52840 --bins --target thumbv7em-none-eabihf --features rtos-trace,gpio-trace -- -D warnings

echo "TEST(dot15d4-frame): no features"
cargo test -p dot15d4-frame --no-default-features

echo "TEST(dot15d4-frame): security"
cargo test -p dot15d4-frame --no-default-features --features=security

echo "TEST(dot15d4-frame): ies"
cargo test -p dot15d4-frame --no-default-features --features=ies

echo "TEST(dot15d4-frame): security,ies"
cargo test -p dot15d4-frame --no-default-features --features=security,ies

echo "TEST(workspace): std"
cargo test --features=std

echo "DOC(workspace)"
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --document-private-items
