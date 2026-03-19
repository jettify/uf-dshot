#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Receiver, Sender, Watch};
use embassy_time::{Duration, Ticker, Timer as EmbassyTimer};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::embassy_stm32::{
    DshotTxPin, RuntimeTimeouts, Stm32BidirCapture, Stm32BidirController,
};
use uf_dshot::{DshotSpeed, OversamplingConfig};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

static MOTOR_THROTTLE: Watch<ThreadModeRawMutex, u16, 1> = Watch::new_with(0);

const ARMING_DELAY_MS: u64 = 7_000;
const TX_PERIOD_US: u64 = 70;
const TELEMETRY_LOG_EVERY_FRAMES: u32 = 20_000;
const DEMO_THROTTLE_MAX_PERCENT: u16 = 25;
const DSHOT_MAX_THROTTLE: u16 = 1_999;

fn throttle_percent_to_raw(percent: u16) -> u16 {
    let clamped = percent.clamp(0, 100);
    ((clamped as u32 * DSHOT_MAX_THROTTLE as u32) / 100) as u16
}

async fn run_demo(sender: &Sender<'static, ThreadModeRawMutex, u16, 1>) -> ! {
    EmbassyTimer::after(Duration::from_millis(ARMING_DELAY_MS)).await;
    info!("start loop");

    loop {
        for value in 0..=DEMO_THROTTLE_MAX_PERCENT {
            sender.send(value);
            info!("bdshot up {}", value);
            EmbassyTimer::after(Duration::from_secs(1)).await;
        }
        for value in (0..=DEMO_THROTTLE_MAX_PERCENT).rev() {
            sender.send(value);
            info!("bdshot down {}", value);
            EmbassyTimer::after(Duration::from_secs(1)).await;
        }
    }
}

#[embassy_executor::task]
async fn motor_task(
    mut controller: Stm32BidirController<
        'static,
        embassy_stm32::peripherals::TIM1,
        embassy_stm32::peripherals::DMA2_CH5,
        embassy_stm32::peripherals::TIM2,
        embassy_stm32::peripherals::DMA1_CH1,
    >,
    mut receiver: Receiver<'static, ThreadModeRawMutex, u16, 1>,
) -> ! {
    unwrap!(controller.arm().await);

    let mut ticker = Ticker::every(Duration::from_micros(TX_PERIOD_US));
    let mut frames_sent = 0u32;
    let mut throttle_percent = 0u16;

    info!("tx/rx loop started dshot300");

    loop {
        frames_sent = frames_sent.wrapping_add(1);
        throttle_percent = receiver.try_get().unwrap_or(throttle_percent);

        let raw_throttle = throttle_percent_to_raw(throttle_percent);
        match controller.send_throttle_and_receive(raw_throttle).await {
            Ok(frame) if frames_sent % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!(
                    "telemetry throttle={} raw={} value={:?}",
                    throttle_percent,
                    raw_throttle,
                    frame
                );
            }
            Err(err) if frames_sent % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!(
                    "telemetry err throttle={} raw={} err={:?}",
                    throttle_percent,
                    raw_throttle,
                    err
                );
            }
            _ => {}
        }

        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // Adjust timer/DMA/pin mapping here to match your board.
    let tx_pin = DshotTxPin::new_ch1(p.PA8);
    let rx_cfg = Stm32BidirCapture::new(p.TIM2, p.DMA1_CH1, OversamplingConfig::default())
        .with_timeouts(RuntimeTimeouts {
            tx: Duration::from_millis(2),
            rx: Duration::from_micros(100),
        });
    let controller = unwrap!(Stm32BidirController::bidirectional(
        p.TIM1,
        tx_pin,
        p.DMA2_CH5,
        rx_cfg,
        DshotSpeed::Dshot300,
    ));

    let sender = MOTOR_THROTTLE.sender();
    sender.send(0);
    let receiver = unwrap!(MOTOR_THROTTLE.receiver());

    spawner.must_spawn(motor_task(controller, receiver));
    run_demo(&sender).await
}
