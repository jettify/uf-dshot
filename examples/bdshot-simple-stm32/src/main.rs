#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use cortex_m::asm;
use embassy_time::{Duration, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::embassy_stm32::{DshotTxPin, Stm32BidirCapture, Stm32BidirController};
use uf_dshot::{DshotSpeed, OversamplingConfig};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const DEMO_THROTTLE: u16 = 200;
const TELEMETRY_LOG_EVERY_FRAMES: u32 = 128;

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
    DMA2_STREAM5 => embassy_stm32::dma::InterruptHandler<embassy_stm32::peripherals::DMA2_CH5>;
});

#[embassy_executor::task]
async fn app() {
    let p = embassy_stm32::init(Default::default());

    let tx_pin = DshotTxPin::new_ch1(p.PA8);
    let rx_cfg = Stm32BidirCapture::new(OversamplingConfig::default());
    let mut esc = unwrap!(Stm32BidirController::bidirectional(
        p.TIM1, tx_pin, p.DMA2_CH5, DmaIrqs, rx_cfg, ESC_SPEED,
    ));

    let frame_period =
        Duration::from_micros(u64::from(3 * ESC_SPEED.timing_hints().min_frame_period_us));
    let mut ticker = Ticker::every(frame_period);
    let mut frames_sent = 0u32;

    info!("start arm");
    unwrap!(esc.arm_for(Duration::from_secs(20)).await);

    info!("spinning at throttle {}", DEMO_THROTTLE);
    loop {
        frames_sent = frames_sent.wrapping_add(1);

        match esc.send_throttle_and_receive(DEMO_THROTTLE).await {
            Ok(frame) if frames_sent % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!("telemetry {:?}", frame);
            }
            Err(err) if frames_sent % TELEMETRY_LOG_EVERY_FRAMES == 0 => {
                info!("telemetry err {:?}", err);
            }
            _ => {}
        }

        ticker.next().await;
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    use embassy_stm32::interrupt::{self, InterruptExt, Priority};

    interrupt::PVD.set_priority(Priority::P6);
    let spawner = EXECUTOR.start(interrupt::PVD);
    spawner.spawn(unwrap!(app()));

    loop {
        asm::wfi();
    }
}
