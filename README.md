# uf-dshot

[![CI](https://github.com/jettify/uf-dshot/actions/workflows/CI.yml/badge.svg)](https://github.com/jettify/uf-dshot/actions/workflows/CI.yml)
[![codecov](https://codecov.io/gh/jettify/uf-dshot/graph/badge.svg?token=NFUQBCTUXF)](https://codecov.io/gh/jettify/uf-dshot)
[![crates.io](https://img.shields.io/crates/v/uf-dshot)](https://crates.io/crates/uf-dshot)
[![docs.rs](https://img.shields.io/docsrs/uf-dshot)](https://docs.rs/uf-dshot/latest/uf_dshot/)

`uf-dshot` is a `no_std` Rust crate for packing and unpacking DShot frames. It supports standard DShot command and throttle frames, bidirectional DShot telemetry, and an STM32 example backend for Embassy based projects.

## Features

- Encode DShot throttle and command frames
- Request telemetry on the separate wire for standard DShot
- Decode bidirectional DShot telemetry from the same wire
- Provide protocol level types and timing hints for platform code
- Offer an STM32 specific integration layer behind the `embassy-stm32` feature

## Features

- `defmt`
  - Enables `defmt::Format` for public types
- `embassy-stm32`
  - Enables Embassy based STM32 helpers
  - Pulls in `embassy-sync`, `embassy-time`, and the STM32 HAL dependency

## Installation

Add the crate to `Cargo.toml`:

```toml
[dependencies]
uf-dshot = "0.1"
```

Enable features when needed:

```toml
[dependencies]
uf-dshot = { version = "0.1", features = ["defmt", "embassy-stm32"] }
```

Or use Cargo:

```bash
cargo add uf-dshot
```

## Simple Example


```rust
#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_time::{Duration, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use embassy_executor::Spawner;
use uf_dshot::embassy_stm32::{DshotPortPin, InterruptHandler, Stm32BidirDshotPort};
use uf_dshot::DshotSpeed;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const DEMO_THROTTLE: u16 = 150;
const LOG_INTERVAL: u32 = 100;
const LOG_INTERVAL_ERR: u32 = 1;

embassy_stm32::bind_interrupts!(struct DmaIrqs {
    DMA2_STREAM3 => InterruptHandler<embassy_stm32::peripherals::DMA2_CH3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let motor_pin = DshotPortPin::new(p.PA8);

    let mut esc = unwrap!(Stm32BidirDshotPort::new_ch1(
        p.TIM1, p.DMA2_CH3, DmaIrqs, motor_pin, ESC_SPEED,
    ));

    let frame_period =
        Duration::from_micros(u64::from(ESC_SPEED.timing_hints().min_frame_period_us) * 3);
    let mut ticker = Ticker::every(frame_period);

    info!("start arm");
    unwrap!(esc.arm_for(Duration::from_secs(5)).await);

    info!("spinning at throttle {}", DEMO_THROTTLE);
    let mut frame_count = 0u32;
    loop {
        frame_count = frame_count.wrapping_add(1);
        match esc.send_throttle_and_receive(DEMO_THROTTLE).await {
            Ok(telemetry) => {
                if frame_count % LOG_INTERVAL == 0 {
                    info!("frame={} telemetry={:?}", frame_count, telemetry);
                }
            }
            Err(err) => {
                if frame_count % LOG_INTERVAL_ERR == 0 {
                    error!("frame={} err={:?}", frame_count, err);
                }
            }
        }
        ticker.next().await;
    }
}
```


## Inspirations

1. https://github.com/symonb/Bidirectional-DSHOT-and-RPM-Filter
2. https://symonb.github.io/docs/drone/ESC/ESC_prot_impl_2_2
3. https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
4. https://github.com/Ragarnoy/embassy-dshot

## License

This project is licensed under the `Apache 2.0`. See the [LICENSE](https://github.com/jettify/uf-dshot/blob/master/LICENSE) file for details.
