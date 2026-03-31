#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use cortex_m::asm;
use embassy_stm32::gpio::Pull;
use embassy_time::{Duration, Instant, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::proto::bitbang_bf::stm32::{
    BidirCaptureConfig, BidirPortRuntimeConfig, DshotPortPin, InterruptHandler,
    Stm32BidirPinController,
};
use uf_dshot::{DecodeHint, DshotSpeed, OversamplingConfig, TelemetryFrame};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const FRAME_MULTIPLIER: u64 = 3;
const ARM_TIME: Duration = Duration::from_secs(5);
const STEP_HOLD: Duration = Duration::from_secs(3);
const TELEMETRY_LOG_EVERY_FRAMES: u32 = 64;
const PACER_COMPARE_PERCENT: u8 = 90;
const RX_COMPARE_PERCENT: u8 = 50;
const RX_COMPARE_SWEEP: [u8; 3] = [25, 50, 75];
const RX_SAMPLE_PERCENT: u8 = 100;
const RX_HINT_SKIP: usize = 0;
const RX_PULL: Pull = Pull::None;
const RX_OVERSAMPLING: OversamplingConfig = OversamplingConfig {
    sample_bit_index: 0,
    oversampling: 3,
    frame_bits: 21,
    min_detected_bits: 18,
    bit_tolerance: 2,
};
const RX_SAMPLE_COUNT: usize = 140;
const STEP_THROTTLES: [u16; 5] = [80, 120, 160, 220, 300];

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

#[derive(Default)]
struct TelemetryCounters {
    frames: u32,
    ok: u32,
    decode_err: u32,
    io_err: u32,
    erpm: u32,
    other: u32,
}

fn note_frame(frame: &TelemetryFrame, counters: &mut TelemetryCounters) {
    counters.ok = counters.ok.wrapping_add(1);
    match frame {
        TelemetryFrame::Erpm(_) => counters.erpm = counters.erpm.wrapping_add(1),
        _ => counters.other = counters.other.wrapping_add(1),
    }
}

async fn hold_throttle(
    esc: &mut Stm32BidirPinController<
        'static,
        embassy_stm32::peripherals::TIM1,
        embassy_stm32::peripherals::DMA2_CH3,
    >,
    throttle: u16,
    duration: Duration,
    frame_period: Duration,
    counters: &mut TelemetryCounters,
    frame_index: &mut u32,
) {
    let started = Instant::now();
    let mut ticker = Ticker::every(3 * frame_period);

    while Instant::now().saturating_duration_since(started) < duration {
        *frame_index = frame_index.wrapping_add(1);
        counters.frames = counters.frames.wrapping_add(1);

        match esc.send_throttle_and_receive(throttle).await {
            Ok(Ok(frame)) => {
                note_frame(&frame, counters);
                if *frame_index % TELEMETRY_LOG_EVERY_FRAMES == 0 {
                    info!(
                        "frame={} throttle={} telemetry={:?}",
                        *frame_index, throttle, frame
                    );
                }
            }
            Ok(Err(err)) => {
                counters.decode_err = counters.decode_err.wrapping_add(1);
                if *frame_index % TELEMETRY_LOG_EVERY_FRAMES == 0 {
                    info!(
                        "frame={} throttle={} telemetry_decode_err={:?}",
                        *frame_index, throttle, err
                    );
                }
            }
            Err(err) => {
                counters.io_err = counters.io_err.wrapping_add(1);
                if *frame_index % TELEMETRY_LOG_EVERY_FRAMES == 0 {
                    info!(
                        "frame={} throttle={} telemetry_io_err={:?}",
                        *frame_index, throttle, err
                    );
                }
            }
        }

        ticker.next().await;
    }
}

#[embassy_executor::task]
async fn app() {
    let p = embassy_stm32::init(Default::default());

    let motor_pin = DshotPortPin::new(p.PA8);
    let rx_cfg = BidirCaptureConfig::new(RX_OVERSAMPLING)
        .with_sample_count(RX_SAMPLE_COUNT)
        .with_decode_hint(DecodeHint {
            preamble_skip: RX_HINT_SKIP,
        })
        .with_pull(RX_PULL);
    let mut esc = unwrap!(Stm32BidirPinController::new_ch1(
        p.TIM1, p.DMA2_CH3, DmaIrqs, motor_pin, rx_cfg, ESC_SPEED,
    )
    .map(|esc| {
        esc.with_runtime_config(BidirPortRuntimeConfig {
            tx_timeout: Duration::from_millis(2),
            rx_timeout: Duration::from_millis(2),
            pacer_compare_percent: PACER_COMPARE_PERCENT,
            rx_compare_percent: RX_COMPARE_PERCENT,
            rx_sample_percent: RX_SAMPLE_PERCENT,
        })
    }));

    let frame_period = Duration::from_micros(
        u64::from(ESC_SPEED.timing_hints().min_frame_period_us) * FRAME_MULTIPLIER,
    );
    let mut frame_index = 0u32;

    info!(
        "bidirectional validation start speed={:?} rx_sample_percent={} rx_oversampling={} rx_sample_count={} rx_hint_skip={} rx_pull={:?}",
        ESC_SPEED,
        RX_SAMPLE_PERCENT,
        RX_OVERSAMPLING.oversampling,
        RX_SAMPLE_COUNT,
        RX_HINT_SKIP,
        RX_PULL
    );
    info!("arm for {} ms", ARM_TIME.as_millis() as u32);
    unwrap!(esc.arm_for(ARM_TIME).await);

    info!("start loop");
    loop {
        for rx_compare_percent in RX_COMPARE_SWEEP {
            let mut counters = TelemetryCounters::default();
            esc.set_runtime_config(BidirPortRuntimeConfig {
                tx_timeout: Duration::from_millis(2),
                rx_timeout: Duration::from_millis(2),
                pacer_compare_percent: PACER_COMPARE_PERCENT,
                rx_compare_percent,
                rx_sample_percent: RX_SAMPLE_PERCENT,
            });
            info!(
                "phase=idle rx_sample_percent={} rx_compare_percent={} hold_ms={} rx_oversampling={} rx_sample_count={}",
                RX_SAMPLE_PERCENT,
                rx_compare_percent,
                STEP_HOLD.as_millis() as u32,
                RX_OVERSAMPLING.oversampling,
                RX_SAMPLE_COUNT
            );
            hold_throttle(
                &mut esc,
                0,
                STEP_HOLD,
                frame_period,
                &mut counters,
                &mut frame_index,
            )
            .await;

            for throttle in STEP_THROTTLES {
                info!(
                    "phase=step rx_sample_percent={} rx_compare_percent={} throttle={} hold_ms={} rx_oversampling={} rx_sample_count={}",
                    RX_SAMPLE_PERCENT,
                    rx_compare_percent,
                    throttle,
                    STEP_HOLD.as_millis() as u32,
                    RX_OVERSAMPLING.oversampling,
                    RX_SAMPLE_COUNT
                );
                hold_throttle(
                    &mut esc,
                    throttle,
                    STEP_HOLD,
                    frame_period,
                    &mut counters,
                    &mut frame_index,
                )
                .await;
            }

            info!(
                "run summary rx_sample_percent={} rx_compare_percent={} total_frames={} ok={} decode_err={} io_err={} erpm={} other={}",
                RX_SAMPLE_PERCENT,
                rx_compare_percent,
                frame_index,
                counters.ok,
                counters.decode_err,
                counters.io_err,
                counters.erpm,
                counters.other
            );
        }
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
