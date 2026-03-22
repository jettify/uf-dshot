#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_time::{Duration, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::embassy_stm32::{DshotTxPin, Stm32DshotController};
use uf_dshot::DshotSpeed;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const DEMO_THROTTLE: u16 = 200;

embassy_stm32::bind_interrupts!(struct Irqs {
    DMA2_STREAM5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH5>;
});

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = embassy_stm32::init(Default::default());

    let tx_pin = DshotTxPin::new_ch1(p.PA8);
    let mut esc = Stm32DshotController::tx_only(p.TIM1, tx_pin, p.DMA2_CH5, Irqs, ESC_SPEED);
    let tx_period_us = u64::from(ESC_SPEED.timing_hints().min_frame_period_us);
    let mut ticker = Ticker::every(Duration::from_micros(tx_period_us));
    info!("start arm");
    unwrap!(esc.arm().await);

    info!("spinning at throttle {}", DEMO_THROTTLE);
    loop {
        unwrap!(esc.send_throttle(DEMO_THROTTLE).await);
        ticker.next().await;
    }
}
