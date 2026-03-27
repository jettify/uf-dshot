#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_time::{Duration, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use embassy_executor::Spawner;
use uf_dshot::embassy_stm32::{
    BidirCaptureConfig, DshotPortPin, InterruptHandler, Stm32BidirPinController,
};
use uf_dshot::{DshotSpeed, OversamplingConfig};
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
    let rx_cfg = BidirCaptureConfig::new(OversamplingConfig::default()).with_sample_count(140);

    let mut esc = unwrap!(Stm32BidirPinController::new_ch1(
        p.TIM1, p.DMA2_CH3, DmaIrqs, motor_pin, rx_cfg, ESC_SPEED,
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
            Ok(Ok(telemetry)) => {
                if frame_count % LOG_INTERVAL == 0 {
                    info!("frame={} telemetry={:?}", frame_count, telemetry);
                }
            }
            Ok(Err(err)) => {
                if frame_count % LOG_INTERVAL_ERR == 0 {
                    error!("frame={} decode_err={:?}", frame_count, err);
                }
            }
            Err(err) => {
                if frame_count % LOG_INTERVAL_ERR == 0 {
                    error!("frame={} io_err={:?}", frame_count, err);
                }
            }
        }
        ticker.next().await;
    }
}
