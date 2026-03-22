#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use cortex_m::asm;
use cortex_m_rt::entry;
use embassy_executor::InterruptExecutor;
use embassy_stm32::gpio::Pull;
use embassy_stm32::interrupt::{self, InterruptExt, Priority};
use embassy_time::{Duration, Instant, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::embassy_stm32::{
    DshotTxPin,
    RuntimeTimeouts,
    Stm32BidirCapture,
    Stm32BidirController,
    Stm32RuntimeError,
};
use uf_dshot::{Command, DshotSpeed, OversamplingConfig};
#[cfg(feature = "defmt")]
use {defmt::warn, defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const ARM_DURATION_MS: u64 = 3_000;
const ARM_PERIOD_US: u64 = 200;
const THROTTLE_DELAY_SECS: u64 = 30;
const DEMO_THROTTLE: u16 = 200;
const STATUS_LOG_EVERY_FRAMES: u32 = 1_000;
const RX_TIMEOUT_US: u64 = 300;
const RX_SAMPLE_COUNT: usize = 120;

static EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[interrupt]
unsafe fn PVD() {
    EXECUTOR.on_interrupt();
}

#[embassy_executor::task]
async fn run(p: embassy_stm32::Peripherals) {

    let mut tx_pin = DshotTxPin::new_ch1(p.PA8);
    tx_pin.prepare_bidirectional_idle();

    let rx_cfg = Stm32BidirCapture::new(OversamplingConfig::default())
        .with_pull(Pull::None)
        .with_sample_count(RX_SAMPLE_COUNT)
        .with_timeouts(RuntimeTimeouts {
            tx: Duration::from_millis(2),
            rx: Duration::from_micros(RX_TIMEOUT_US),
        });
    let mut esc =
        unwrap!(Stm32BidirController::bidirectional(p.TIM1, tx_pin, p.DMA2_CH5, rx_cfg, ESC_SPEED));
    let tx_period_us = u64::from(ESC_SPEED.timing_hints().min_frame_period_us);
    info!(
        "bdshot rx cfg pull=None sample_count={} rx_timeout_us={}",
        RX_SAMPLE_COUNT, RX_TIMEOUT_US
    );

    info!("arming at {} us packet period", ARM_PERIOD_US);
    let mut arm_ticker = Ticker::every(Duration::from_micros(ARM_PERIOD_US));
    let arm_frames = (ARM_DURATION_MS * 1_000) / ARM_PERIOD_US;
    for _ in 0..arm_frames {
        unwrap!(esc.send_command(Command::MotorStop).await);
        arm_ticker.next().await;
    }

    let mut ticker = Ticker::every(Duration::from_micros(tx_period_us));
    let mut send_throttle = false;
    let switch_at = Instant::now() + Duration::from_secs(THROTTLE_DELAY_SECS);
    let mut frames = 0u32;
    let mut rx_ok = 0u32;
    let mut rx_timeout = 0u32;
    let mut rx_decode_err = 0u32;
    info!(
        "sending bidirectional MotorStop for {} s, then throttle {}",
        THROTTLE_DELAY_SECS, DEMO_THROTTLE
    );
    loop {
        frames = frames.wrapping_add(1);
        if !send_throttle && Instant::now() >= switch_at {InterruptExecutor
            send_throttle = true;
            info!("switching to bidirectional throttle {}", DEMO_THROTTLE);
        }

        let result = if send_throttle {
            esc.send_throttle_and_receive(DEMO_THROTTLE).await
        } else {
            esc.send_command_and_receive(Command::MotorStop).await
        };

        match result {
            Ok(frame) => {
                rx_ok = rx_ok.wrapping_add(1);
                info!("telemetry {:?}", frame);
            }
            Err(Stm32RuntimeError::Telemetry(err)) => {
                rx_decode_err = rx_decode_err.wrapping_add(1);
                if frames % STATUS_LOG_EVERY_FRAMES == 0 {
                    info!("rx decode err {:?}", err);
                }
            }
            Err(Stm32RuntimeError::RxTimeout) => {
                rx_timeout = rx_timeout.wrapping_add(1);
            }
            Err(Stm32RuntimeError::TxTimeout) => warn!("bdshot tx timeout"),
        }
        if frames % STATUS_LOG_EVERY_FRAMES == 0 {
            info!(
                "status frames={} mode={} rx_ok={} rx_timeout={} rx_decode_err={}",
                frames,
                if send_throttle { "throttle" } else { "stop" },
                rx_ok,
                rx_timeout,
                rx_decode_err
            );
        }
        ticker.next().await;
    }
}

#[entry]
fn main() -> ! {
    let p = embassy_stm32::init(Default::default());

    interrupt::PVD.set_priority(Priority::P6);
    let spawner = EXECUTOR.start(interrupt::PVD);
    unwrap!(spawner.spawn(run(p)));

    loop {
        asm::wfi();
    }
}
