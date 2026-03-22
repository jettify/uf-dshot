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

use uf_dshot::embassy_stm32::{DshotTxPin, Stm32DshotController};
use uf_dshot::DshotSpeed;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

static MOTOR_THROTTLE: Watch<ThreadModeRawMutex, u16, 1> = Watch::new_with(0);

const ARMING_DELAY_MS: u64 = 3_000;
const TX_PERIOD_US: u64 = 70;
const TX_LOG_EVERY_FRAMES: u32 = 20_000;
const DEMO_THROTTLE_MAX_PERCENT: u16 = 25;
const DSHOT_MAX_THROTTLE: u16 = 1_999;

embassy_stm32::bind_interrupts!(struct Irqs {
    DMA2_STREAM5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH5>;
});

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
            info!("dshot dma up {}", value);
            EmbassyTimer::after(Duration::from_millis(1000)).await;
        }
        for value in (0..=DEMO_THROTTLE_MAX_PERCENT).rev() {
            sender.send(value);
            info!("dshot dma down {}", value);
            EmbassyTimer::after(Duration::from_millis(1000)).await;
        }
    }
}

#[embassy_executor::task]
async fn motor_task(
    mut controller: Stm32DshotController<
        'static,
        embassy_stm32::peripherals::TIM1,
        embassy_stm32::peripherals::DMA2_CH5,
    >,
    mut receiver: Receiver<'static, ThreadModeRawMutex, u16, 1>,
) -> ! {
    unwrap!(controller.arm().await);

    let mut ticker = Ticker::every(Duration::from_micros(TX_PERIOD_US));
    let mut frames_sent = 0u32;
    let mut throttle_percent = 0u16;

    info!("tx loop started dshot300");

    loop {
        frames_sent = frames_sent.wrapping_add(1);
        throttle_percent = receiver.try_get().unwrap_or(throttle_percent);

        let raw_throttle = throttle_percent_to_raw(throttle_percent);
        let frame = unwrap!(controller.send_throttle(raw_throttle).await);

        if frames_sent % TX_LOG_EVERY_FRAMES == 0 {
            info!(
                "tx throttle={} raw={} payload=0x{:04X} crc=0x{:X}",
                throttle_percent,
                raw_throttle,
                frame.payload,
                frame.crc_4()
            );
        }

        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let tx_pin = DshotTxPin::new_ch1(p.PA8);
    let controller = Stm32DshotController::tx_only(
        p.TIM1,
        tx_pin,
        p.DMA2_CH5,
        Irqs,
        DshotSpeed::Dshot300,
    );

    let sender = MOTOR_THROTTLE.sender();
    sender.send(0);
    let receiver = unwrap!(MOTOR_THROTTLE.receiver());

    spawner.spawn(unwrap!(motor_task(controller, receiver)));
    run_demo(&sender).await
}
