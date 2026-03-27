#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_executor::Spawner;
use embassy_time::{Duration, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::embassy_stm32::{DshotPortPin, InterruptHandler, Stm32TxPortController};
use uf_dshot::DshotSpeed;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const DEMO_THROTTLE: u16 = 150;

embassy_stm32::bind_interrupts!(struct DmaIrqs {
    DMA2_STREAM3 => InterruptHandler<embassy_stm32::peripherals::DMA2_CH3>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let tx_pin = DshotPortPin::new(p.PA8);
    let mut esc = unwrap!(Stm32TxPortController::new_ch1(
        p.TIM1,
        p.DMA2_CH3,
        DmaIrqs,
        [tx_pin],
        ESC_SPEED,
    ));
    let frame_period =
        Duration::from_micros(u64::from(ESC_SPEED.timing_hints().min_frame_period_us));
    let mut ticker = Ticker::every(frame_period);
    info!("start arm");
    unwrap!(esc.arm_for(Duration::from_secs(5)).await);

    info!("spinning at throttle {}", DEMO_THROTTLE);
    loop {
        unwrap!(esc.send_throttles([DEMO_THROTTLE]).await);
        ticker.next().await;
    }
}
