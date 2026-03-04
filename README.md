# DShot Protocol Implementation

DShot and Bidirectional Shot protocol parser for embedded (`no_std`) environments.

## Features

- `default = []` (platform-agnostic core only)
- `defmt` for logging-friendly formatting
- `std` for host mocks and SITL-style test utilities
- `embassy` for async transport trait contracts
- `embassy-stm32` for experimental STM32 runtime integration surface
- `stm32f405` and `stm32f411` chip selectors (require `embassy-stm32`)

## Host Examples

- `cargo run --example local_simple`
- `cargo run --example local_decode`

## Inspirations

1. <https://github.com/symonb/Bidirectional-DSHOT-and-RPM-Filter>
2. <https://symonb.github.io/docs/drone/ESC/ESC_prot_impl_2_2>
3. <https://brushlesswhoop.com/dshot-and-bidirectional-dshot/>
