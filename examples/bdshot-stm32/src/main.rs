#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_executor::Spawner;
use embassy_stm32::gpio::Pull;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Receiver, Sender, Watch};
use embassy_time::{Duration, Ticker, Timer as EmbassyTimer};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::embassy_stm32::bidir_capture::{DshotPortPin, Stm32BidirPortController};
use uf_dshot::embassy_stm32::Stm32BidirCapture;
use uf_dshot::{DshotSpeed, OversamplingConfig};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

static MOTOR_THROTTLE: Watch<ThreadModeRawMutex, u16, 1> = Watch::new_with(0);

const ARMING_DELAY_MS: u64 = 7_000;
// Bidir DShot300 needs enough time for the 16-bit command, ESC turnaround, and
// the telemetry capture window. A 70 us loop overruns once capture is enabled,
// which makes motor updates arrive with irregular cadence.
const TX_PERIOD_US: u64 = 180;
const TELEMETRY_LOG_EVERY_FRAMES: u32 = 20_000;
const DEMO_THROTTLE_MAX_PERCENT: u16 = 25;
const DSHOT_MAX_THROTTLE: u16 = 1_999;
const PACER_COMPARE_PERCENT: u8 = 50;
const RX_OVERSAMPLING: OversamplingConfig = OversamplingConfig {
    sample_bit_index: 0,
    oversampling: 6,
    frame_bits: 21,
    min_detected_bits: 18,
    bit_tolerance: 2,
};
const RX_SAMPLE_COUNT: usize = 240;

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
    mut controller:
        Stm32BidirPortController<'static, embassy_stm32::peripherals::TIM1, embassy_stm32::peripherals::DMA2_CH1, 1>,
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
        match controller.send_throttles_and_receive([raw_throttle]).await {
            Ok([Ok(frame)]) if frames_sent % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!(
                    "telemetry throttle={} raw={} value={:?}",
                    throttle_percent,
                    raw_throttle,
                    frame
                );
            }
            Ok([Err(err)]) if frames_sent % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!(
                    "telemetry decode err throttle={} raw={} err={:?}",
                    throttle_percent,
                    raw_throttle,
                    err
                );
            }
            Err(err) if frames_sent % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!(
                    "telemetry io err throttle={} raw={} err={:?}",
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

embassy_stm32::bind_interrupts!(struct DmaIrqs {
    DMA2_STREAM1 => uf_dshot::embassy_stm32::bidir_capture::InterruptHandler<embassy_stm32::peripherals::DMA2_CH1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    // Adjust timer/DMA/pin mapping here to match your board.
    let motor_pin = DshotPortPin::new(p.PA8);
    let rx_cfg = Stm32BidirCapture::new(RX_OVERSAMPLING)
        .with_sample_count(RX_SAMPLE_COUNT)
        .with_pull(Pull::None)
        .with_pacer_compare_percent(PACER_COMPARE_PERCENT);
    let controller = unwrap!(Stm32BidirPortController::new_ch1(
        p.TIM1,
        p.DMA2_CH1,
        DmaIrqs,
        [motor_pin],
        rx_cfg,
        DshotSpeed::Dshot300,
    ));

    let sender = MOTOR_THROTTLE.sender();
    sender.send(0);
    let receiver = unwrap!(MOTOR_THROTTLE.receiver());

    spawner.spawn(unwrap!(motor_task(controller, receiver)));
    run_demo(&sender).await
}
