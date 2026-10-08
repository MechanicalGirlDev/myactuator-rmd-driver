# MyActuator RMD Driver

Standalone Rust crates for the MyActuator RMD-X CAN protocol and SLCAN transport.
Maintained by nop at MechanicalGirl LLC.

## Crates

- `myactuator-rmd`: `no_std` protocol encoding and decoding.
- `myactuator-rmd-can`: SLCAN transport, transactions, and command-line interface.

## Reiny 0.8 integration

`myactuator-rmd` remains a dependency-free `no_std` protocol crate;
`myactuator-rmd-can` remains a SLCAN transport. The CLI is a standalone
bring-up tool, not a managed Reiny leaf.

Put Reiny's `main.yaml` and SDK in the consuming adapter. Declare and open its
named command/feedback ports, initialize the CAN connection and then report
`Cloudy::ready()`. The adapter owns actuator-ID mapping, physical units,
freshness and safe stop/torque behavior. Complete that behavior on
`Cloudy::shutdown()` before dropping the transport; a stop acknowledgement is
not a completed hardware shutdown. Preserve the full module namespace as
message provenance rather than using a CAN identifier as a publisher identity.

Use published `reiny = "0.8.0"` and `reiny-build = "0.8.0"` in the adapter.
Its runtime definition and schema catalog use `version: 2`. Declare the
executable, build and endpoint policies in the adapter's own `main.yaml`:
input `replay`/`buffer` and output `qos`/`retention`. Callers reuse it through
`source` and wire inputs with `from`, without repeating child outputs.

## Verification

```sh
cargo fmt --all --check
cargo clippy --workspace --all-targets --all-features -- -D warnings
cargo test --workspace --all-features
cargo check -p myactuator-rmd --no-default-features
```

Licensed under Apache-2.0. See [LICENSE](LICENSE) and [NOTICE](NOTICE).
