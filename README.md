# uf-dshot

[![CI](https://github.com/jettify/uf-dshot/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-dshot/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-dshot/graph/badge.svg?token=NFUQBCTUXF)](https://codecov.io/gh/jettify/uf-dshot)
[![crates.io](https://img.shields.io/crates/v/uf-dshot)](https://crates.io/crates/uf-dshot)
[![docs.rs](https://img.shields.io/docsrs/uf-dshot)](https://docs.rs/uf-dshot/latest/uf_dshot/)

`uf-dshot` rust `no_std` library for controlling ESC using `dshot` and `bidirectional dshot` protocols.


## Installation

Add `uf-dshot` to your `Cargo.toml`:

```toml
[dependencies]
uf-dshot = "*" # replace * by the latest version of the crate.
```

Or use the command line:

```bash
cargo add uf-dshot
```


## Inspirations

1. <https://github.com/symonb/Bidirectional-DSHOT-and-RPM-Filter>
2. <https://symonb.github.io/docs/drone/ESC/ESC_prot_impl_2_2>
3. <https://brushlesswhoop.com/dshot-and-bidirectional-dshot/>

## Note

Library is under active development and testing, API might change at any time.

## License

This project is licensed under the `Apache 2.0`. See the [LICENSE](https://github.com/jettify/uf-dshot/blob/master/LICENSE) file for details.
