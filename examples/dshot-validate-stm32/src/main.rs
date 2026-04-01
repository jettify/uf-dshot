#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use cortex_m::asm;
use embassy_time::{Duration, Instant, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::embassy_stm32::{DshotPortPin, InterruptHandler, Stm32TxPortController};
use uf_dshot::DshotSpeed;

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const FRAME_MULTIPLIER: u64 = 2;
const ARM_TIME: Duration = Duration::from_secs(5);
const STEP_HOLD: Duration = Duration::from_secs(3);
const STEP_THROTTLES: [u16; 6] = [0, 80, 120, 160, 220, 300];
const PULSE_THROTTLE: u16 = 260;
const PULSE_HIGH: Duration = Duration::from_millis(300);
const PULSE_LOW: Duration = Duration::from_millis(700);
const PULSE_COUNT: u32 = 10;

static EXECUTOR: embassy_executor::InterruptExecutor = embassy_executor::InterruptExecutor::new();

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
    DMA2_STREAM3 => InterruptHandler<embassy_stm32::peripherals::DMA2_CH3>;
});

async fn hold_throttle(
    esc: &mut Stm32TxPortController<
        'static,
        embassy_stm32::peripherals::TIM1,
        embassy_stm32::peripherals::DMA2_CH3,
        1,
    >,
    throttle: u16,
    duration: Duration,
    frame_period: Duration,
) {
    let started = Instant::now();
    let mut ticker = Ticker::every(frame_period);
    while Instant::now().saturating_duration_since(started) < duration {
        unwrap!(esc.send_throttles([throttle]).await);
        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn app() {
    let p = embassy_stm32::init(Default::default());

    let tx_pin = DshotPortPin::new(p.PA8);
    let mut esc = unwrap!(Stm32TxPortController::new_ch1(
        p.TIM1,
        p.DMA2_CH3,
        DmaIrqs,
        [tx_pin],
        ESC_SPEED,
    ));
    let frame_period = Duration::from_micros(
        u64::from(ESC_SPEED.timing_hints().min_frame_period_us) * FRAME_MULTIPLIER,
    );

    info!("unidirectional validation start speed={:?}", ESC_SPEED);
    info!("arm for {} ms", ARM_TIME.as_millis() as u32);
    unwrap!(esc.arm_for(ARM_TIME).await);

    loop {
        info!("phase=idle hold_ms={}", STEP_HOLD.as_millis() as u32);
        hold_throttle(&mut esc, 0, STEP_HOLD, frame_period).await;

        for throttle in STEP_THROTTLES {
            info!(
                "phase=step throttle={} hold_ms={}",
                throttle,
                STEP_HOLD.as_millis() as u32
            );
            hold_throttle(&mut esc, throttle, STEP_HOLD, frame_period).await;
        }

        info!(
            "phase=pulse throttle={} high_ms={} low_ms={} count={}",
            PULSE_THROTTLE,
            PULSE_HIGH.as_millis() as u32,
            PULSE_LOW.as_millis() as u32,
            PULSE_COUNT
        );
        for pulse in 0..PULSE_COUNT {
            info!("pulse_index={}", pulse);
            hold_throttle(&mut esc, PULSE_THROTTLE, PULSE_HIGH, frame_period).await;
            hold_throttle(&mut esc, 0, PULSE_LOW, frame_period).await;
        }
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
