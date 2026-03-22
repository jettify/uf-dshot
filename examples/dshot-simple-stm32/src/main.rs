#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use cortex_m::asm;
use cortex_m_rt::entry;
use embassy_executor::InterruptExecutor;
use embassy_stm32::interrupt::{self, InterruptExt, Priority};
use embassy_time::{Duration, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::embassy_stm32::{DshotTxPin, Stm32DshotController};
use uf_dshot::DshotSpeed;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const DEMO_THROTTLE: u16 = 150;

static EXECUTOR: InterruptExecutor = InterruptExecutor::new();

#[embassy_stm32::interrupt]
unsafe fn PVD() {
    EXECUTOR.on_interrupt();
}

#[embassy_executor::task]
async fn run(p: embassy_stm32::Peripherals) {
    let frame_period =
        Duration::from_micros(u64::from(ESC_SPEED.timing_hints().min_frame_period_us));

    let tx_pin = DshotTxPin::new_ch1(p.PA8);
    let mut esc = Stm32DshotController::tx_only(p.TIM1, tx_pin, p.DMA2_CH5, ESC_SPEED);

    info!("arming");
    unwrap!(esc.arm().await);

    let mut ticker = Ticker::every(frame_period);
    info!("spinning at throttle {}", DEMO_THROTTLE);
    loop {
        unwrap!(esc.send_throttle(DEMO_THROTTLE).await);
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
