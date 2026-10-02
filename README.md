# MyActuator RMD Driver

Standalone Rust crates for the MyActuator RMD-X CAN protocol and SLCAN transport.
Maintained by nop at MechanicalGirl LLC.

## Crates

- `myactuator-rmd`: `no_std` protocol encoding and decoding.
- `myactuator-rmd-can`: SLCAN transport, transactions, and command-line interface.

## Verification

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
cargo check -p myactuator-rmd --no-default-features
```

Licensed under Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
