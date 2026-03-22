#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use cortex_m::asm;
use embassy_time::{Duration, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::embassy_stm32::{DshotTxPin, Stm32DshotController};
use uf_dshot::DshotSpeed;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const DEMO_THROTTLE: u16 = 200;
static EXECUTOR: embassy_stm32::executor::InterruptExecutor =
    embassy_stm32::executor::InterruptExecutor::new();

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
    let mut esc = Stm32DshotController::tx_only(p.TIM1, tx_pin, p.DMA2_CH5, DmaIrqs, ESC_SPEED);
    let frame_period = Duration::from_micros(u64::from(ESC_SPEED.timing_hints().min_frame_period_us));
    let mut ticker = Ticker::every(frame_period);
    info!("start arm");
    unwrap!(esc.arm().await);

    info!("spinning at throttle {}", DEMO_THROTTLE);
    loop {
        unwrap!(esc.send_throttle(DEMO_THROTTLE).await);
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
