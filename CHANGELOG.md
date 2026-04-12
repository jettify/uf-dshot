# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0](https://github.com/jettify/uf-dshot/releases/tag/v0.1.0) - 2026-04-12

### Added

- Release infra. ([#17](https://github.com/jettify/uf-dshot/issues/17))
- Enable release profile with more aggresive optimizations.
- Optimize disable in IRQ. ([#14](https://github.com/jettify/uf-dshot/issues/14))
- Align API between dshot modes.
- Multi pin support for bidirectional dshot.
- Clean public API for command encoding and telemetry decoding.
- Rework API mistakes.
- Big banging approach. ([#9](https://github.com/jettify/uf-dshot/issues/9))
- Bidir telemetry visible for am32.
- Bidir mode spins fro am32.
- Revert to oversampling apporach.
- Add bidirectional simple example.
- Upgrade to new embassy stm32
- Make pwm based dshot controller work.
- Debug am32.
- Add function to decode samples with tunred preable hint.
- Add CI using github actions.
- Rework protocol parsing.
- Add experimental duty cycle helper.

### Fixed

- Extended telemetry decoding. ([#16](https://github.com/jettify/uf-dshot/issues/16))
- Repeat same command 10 times instead of 6 for ESC. ([#11](https://github.com/jettify/uf-dshot/issues/11))
- More robust telemetry handling
- Fix rest period for dshot stm32 implemenmtation. ([#8](https://github.com/jettify/uf-dshot/issues/8))

### Other

- Better readme and cargo.toml descriptions.
- Change license to apache. ([#15](https://github.com/jettify/uf-dshot/issues/15))
- Split stm32 module ([#12](https://github.com/jettify/uf-dshot/issues/12))
- Add simple telemetry examples without MCU.
- Remove streaming telemetry parsing implementation.
- Add example with dedicated motor task. ([#10](https://github.com/jettify/uf-dshot/issues/10))
- More consitent naming across helper functions.
- Add license.
- Compile stm32 on CI machine.
- Basic readme and configuration for clippy in tests. ([#7](https://github.com/jettify/uf-dshot/issues/7))
- Adjust features not to fail on stm32
- Add stm32 support
- Address lints for more robust code.
- Code review cleanup to make code more redable.
- Cleanup after rebase.
- [**breaking**] New framing API.
- wip
- Add faulty bitbagging
- Initial commit.
