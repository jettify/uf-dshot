#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Receiver, Sender, Watch};
use embassy_time::{Duration, Ticker, Timer};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::embassy_stm32::{DshotPortPin, InterruptHandler, Stm32BidirDshotPort};
use uf_dshot::DshotSpeed;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

static THROTTLE_WATCH: Watch<ThreadModeRawMutex, u16, 1> = Watch::new();

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const DSHOT_MAX_THROTTLE: u16 = 1_999;
const ARMING_DELAY_MS: u64 = 7_000;
const ARM_FOR_SECS: u64 = 5;
const FRAME_PERIOD_US: u64 = 180;
const CONTROL_STEP_PERIOD_MS: u64 = 100;
const THROTTLE_MAX_PERCENT: u16 = 25;
const TELEMETRY_LOG_EVERY_FRAMES: u32 = 100;

fn throttle_percent_to_raw(percent: u16) -> u16 {
    let clamped = percent.clamp(0, 100);
    ((clamped as u32 * DSHOT_MAX_THROTTLE as u32) / 100) as u16
}

async fn run_demo(sender: &Sender<'static, ThreadModeRawMutex, u16, 1>) -> ! {
    Timer::after(Duration::from_millis(ARMING_DELAY_MS)).await;
    info!(
        "start throttle sweep 0..{}..0 percent",
        THROTTLE_MAX_PERCENT
    );

    loop {
        for throttle in 0..=THROTTLE_MAX_PERCENT {
            sender.send(throttle);
            info!("throttle setpoint={}%", throttle);
            Timer::after(Duration::from_millis(CONTROL_STEP_PERIOD_MS)).await;
        }
        for throttle in (0..=THROTTLE_MAX_PERCENT).rev() {
            sender.send(throttle);
            info!("throttle setpoint={}%", throttle);
            Timer::after(Duration::from_millis(CONTROL_STEP_PERIOD_MS)).await;
        }
    }
}

#[embassy_executor::task]
async fn motor_task(
    mut esc: Stm32BidirDshotPort<
        'static,
        embassy_stm32::peripherals::TIM1,
        embassy_stm32::peripherals::DMA2_CH3,
    >,
    mut receiver: Receiver<'static, ThreadModeRawMutex, u16, 1>,
) -> ! {
    info!("start arm");
    unwrap!(esc.arm_for(Duration::from_secs(ARM_FOR_SECS)).await);
    info!("motor io task started");

    let mut ticker = Ticker::every(Duration::from_micros(FRAME_PERIOD_US));
    let mut frame_count = 0u32;
    let mut throttle_percent = 0u16;

    loop {
        frame_count = frame_count.wrapping_add(1);
        while let Some(next) = receiver.try_changed() {
            throttle_percent = next.clamp(0, THROTTLE_MAX_PERCENT);
        }

        let raw_throttle = throttle_percent_to_raw(throttle_percent);
        match esc.send_throttle_and_receive(raw_throttle).await {
            Ok(telemetry) if frame_count % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!(
                    "frame={} throttle={} raw={} telemetry={:?}",
                    frame_count, throttle_percent, raw_throttle, telemetry
                );
            }
            Err(err) => {
                info!(
                    "frame={} throttle={} raw={} telemetry_err={:?}",
                    frame_count, throttle_percent, raw_throttle, err
                );
            }
            _ => {}
        }

        ticker.next().await;
    }
}

embassy_stm32::bind_interrupts!(struct DmaIrqs {
    DMA2_STREAM3 => InterruptHandler<embassy_stm32::peripherals::DMA2_CH3>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let motor_pin = DshotPortPin::new(p.PA8);
    let esc = unwrap!(Stm32BidirDshotPort::new_ch1(
        p.TIM1, p.DMA2_CH3, DmaIrqs, motor_pin, ESC_SPEED,
    ));

    let sender = THROTTLE_WATCH.sender();
    let receiver = THROTTLE_WATCH.receiver().unwrap();
    spawner.spawn(unwrap!(motor_task(esc, receiver)));
    run_demo(&sender).await
}
