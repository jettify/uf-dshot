#![no_std]
#![no_main]

#[macro_use]
mod fmt;
#[path = "embassy_stm32.rs"]
mod embassy_stm32_local;

use cortex_m::asm;
use embassy_stm32::gpio::Pull;
use embassy_time::{Duration, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use crate::embassy_stm32_local::bidir_capture::{DshotPortPin, Stm32BidirPortController};
use crate::embassy_stm32_local::Stm32BidirCapture;
use uf_dshot::{DshotSpeed, OversamplingConfig};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const DEMO_THROTTLE: u16 = 200;
const TELEMETRY_LOG_EVERY_FRAMES: u32 = 128;
const PACER_COMPARE_PERCENT: u8 = 90;
const RX_OVERSAMPLING: OversamplingConfig = OversamplingConfig {
    sample_bit_index: 0,
    oversampling: 6,
    frame_bits: 21,
    min_detected_bits: 18,
    bit_tolerance: 2,
};
const RX_SAMPLE_COUNT: usize = 240;

static EXECUTOR: embassy_executor::InterruptExecutor = embassy_executor::InterruptExecutor::new();

// Use an otherwise-unused IRQ as the executor's software-pended wakeup source.
struct ExecutorInterruptHandler;

impl embassy_stm32::interrupt::typelevel::Handler<embassy_stm32::interrupt::typelevel::PVD>
    for ExecutorInterruptHandler
{
    unsafe fn on_interrupt() {
        EXECUTOR.on_interrupt();
    }
}

embassy_stm32::bind_interrupts!(struct ExecutorIrqs {
    PVD => ExecutorInterruptHandler;
});

embassy_stm32::bind_interrupts!(struct DmaIrqs {
    DMA2_STREAM1 => crate::embassy_stm32_local::bidir_capture::InterruptHandler<embassy_stm32::peripherals::DMA2_CH1>;
});

#[embassy_executor::task]
async fn app() {
    let p = embassy_stm32::init(Default::default());

    let motor_pin = DshotPortPin::new(p.PA8);
    let rx_cfg = Stm32BidirCapture::new(RX_OVERSAMPLING)
        .with_sample_count(RX_SAMPLE_COUNT)
        .with_pull(Pull::None)
        .with_pacer_compare_percent(PACER_COMPARE_PERCENT);
    let mut esc = unwrap!(Stm32BidirPortController::new_ch1(
        p.TIM1,
        p.DMA2_CH1,
        DmaIrqs,
        [motor_pin],
        rx_cfg,
        ESC_SPEED,
    ));

    let frame_period =
        Duration::from_micros(u64::from(3 * ESC_SPEED.timing_hints().min_frame_period_us));
    let mut ticker = Ticker::every(frame_period);
    let mut frames_sent = 0u32;

    info!("start arm");
    unwrap!(esc.arm_for(Duration::from_secs(5)).await);

    info!("spinning at throttle {}", DEMO_THROTTLE);
    loop {
        frames_sent = frames_sent.wrapping_add(1);

        match esc.send_throttles_and_receive([DEMO_THROTTLE]).await {
            Ok([Ok(frame)]) if frames_sent % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!("telemetry {:?}", frame);
            }
            Ok([Err(err)]) if frames_sent % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!("telemetry decode err {:?}", err);
            }
            Err(err) if frames_sent % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!("telemetry io err {:?}", err);
            }
            _ => {}
        }

        ticker.next().await;
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    use embassy_stm32::interrupt::{self, InterruptExt, Priority};

    interrupt::PVD.set_priority(Priority::P1);
    let spawner = EXECUTOR.start(interrupt::PVD);
    spawner.spawn(unwrap!(app()));

    loop {
        asm::wfi();
    }
}
