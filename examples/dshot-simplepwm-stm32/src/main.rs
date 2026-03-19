#![no_std]
#![no_main]

mod fmt;

use embassy_executor::Spawner;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::DMA2_CH5;
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Receiver, Sender, Watch};
use embassy_time::{Duration, Ticker, Timer as EmbassyTimer};
use fmt::info;
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
use uf_dshot::{Command, DshotSpeed, UniTx, WaveformTiming};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

#[derive(Clone, Copy, PartialEq, Eq)]
enum MotorRequest {
    Stop,
    ThrottlePercent(u16),
}

static MOTOR_REQUEST: Watch<ThreadModeRawMutex, MotorRequest, 1> =
    Watch::new_with(MotorRequest::Stop);

const DEMO_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const TX_PERIOD_US: u64 = 70;
const ARMING_DELAY_MS: u64 = 2_000;
const STARTUP_MOTOR_STOP_MS: u64 = 1_000;
const DEMO_THROTTLE_MAX_PERCENT: u16 = 10;
const DSHOT_MAX_THROTTLE: u16 = 1_999;
const TX_LOG_EVERY_FRAMES: u32 = 20_000;

fn throttle_percent_to_raw(percent: u16) -> u16 {
    let clamped = percent.clamp(0, 100);
    ((clamped as u32 * DSHOT_MAX_THROTTLE as u32) / 100) as u16
}

fn dshot_waveform_timing(max_duty: u16) -> WaveformTiming {
    WaveformTiming {
        period_ticks: max_duty,
        bit0_high_ticks: (max_duty * 3) / 8,
        bit1_high_ticks: (max_duty * 3) / 4,
    }
}

fn make_motor_stop_cycles(timing: WaveformTiming) -> [u16; 17] {
    let frame = UniTx::command(Command::MotorStop).encode();
    make_frame_cycles(frame, timing)
}

fn make_frame_cycles(frame: uf_dshot::EncodedFrame, timing: WaveformTiming) -> [u16; 17] {
    let waveform = frame.to_waveform_ticks(timing, false);
    let mut cycles = [0u16; 17];
    cycles[..16].copy_from_slice(&waveform.bit_high_ticks);
    cycles
}

fn write_frame_cycles(
    cycles: &mut [u16; 17],
    frame: uf_dshot::EncodedFrame,
    timing: WaveformTiming,
) {
    let waveform = frame.to_waveform_ticks(timing, false);
    cycles[..16].copy_from_slice(&waveform.bit_high_ticks);
    cycles[16] = 0;
}

async fn run_demo(sender: &Sender<'static, ThreadModeRawMutex, MotorRequest, 1>) -> ! {
    EmbassyTimer::after(Duration::from_millis(ARMING_DELAY_MS)).await;
    info!("start loop");

    loop {
        sender.send(MotorRequest::ThrottlePercent(1));
        info!("dshot 1pct");
        EmbassyTimer::after(Duration::from_secs(1)).await;

        for value in 1..=DEMO_THROTTLE_MAX_PERCENT {
            sender.send(MotorRequest::ThrottlePercent(value));
            info!("dshot up {}", value);
            EmbassyTimer::after(Duration::from_secs(1)).await;
        }

        for value in (1..=DEMO_THROTTLE_MAX_PERCENT).rev() {
            sender.send(MotorRequest::ThrottlePercent(value));
            info!("dshot down {}", value);
            EmbassyTimer::after(Duration::from_secs(1)).await;
        }

        sender.send(MotorRequest::Stop);
        sender.send(MotorRequest::ThrottlePercent(1));
        info!("dshot 1pct");
        EmbassyTimer::after(Duration::from_secs(1)).await;
    }
}

#[embassy_executor::task]
async fn motor_task(
    mut pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM1>,
    mut dma_ch: Peri<'static, DMA2_CH5>,
    mut receiver: Receiver<'static, ThreadModeRawMutex, MotorRequest, 1>,
) -> ! {
    let max_duty = pwm.max_duty_cycle();
    let timing = dshot_waveform_timing(max_duty);
    let mut ch1 = pwm.ch1();
    ch1.enable();
    ch1.set_duty_cycle_fully_off();

    let mut ticker = Ticker::every(Duration::from_micros(TX_PERIOD_US));
    let motor_stop_cycles = make_motor_stop_cycles(timing);
    let startup_deadline =
        embassy_time::Instant::now() + Duration::from_millis(STARTUP_MOTOR_STOP_MS);

    info!("streaming MotorStop for {} ms", STARTUP_MOTOR_STOP_MS);
    while embassy_time::Instant::now() < startup_deadline {
        pwm.waveform_up(dma_ch.reborrow(), Channel::Ch1, &motor_stop_cycles)
            .await;
        ticker.next().await;
    }

    info!("MotorStop startup complete");

    let mut request = MotorRequest::Stop;
    let mut frames_sent = 0u32;
    let mut throttle_cycles = [0u16; 17];

    loop {
        frames_sent = frames_sent.wrapping_add(1);
        request = receiver.try_get().unwrap_or(request);
        let (cycles, throttle_percent, raw_throttle, payload) = match request {
            MotorRequest::Stop => {
                let frame = UniTx::command(Command::MotorStop).encode();
                (&motor_stop_cycles, 0, 0, frame.payload)
            }
            MotorRequest::ThrottlePercent(throttle_percent) => {
                let raw_throttle = throttle_percent_to_raw(throttle_percent);
                let frame = UniTx::throttle_clamped(raw_throttle).encode();
                write_frame_cycles(&mut throttle_cycles, frame, timing);
                (
                    &throttle_cycles,
                    throttle_percent,
                    raw_throttle,
                    frame.payload,
                )
            }
        };
        pwm.waveform_up(dma_ch.reborrow(), Channel::Ch1, cycles)
            .await;

        if frames_sent % TX_LOG_EVERY_FRAMES == 0 {
            info!(
                "tx throttle={} raw={} payload=0x{:04X}",
                throttle_percent, raw_throttle, payload
            );
        }

        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    let ch1_pin = PwmPin::new(p.PA8, OutputType::PushPull);

    let pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1_pin),
        None,
        None,
        None,
        khz(DEMO_SPEED.timing_hints().nominal_bitrate_hz / 1_000),
        Default::default(),
    );
    let dma_ch = p.DMA2_CH5;

    let sender = MOTOR_REQUEST.sender();
    sender.send(MotorRequest::Stop);
    let receiver = MOTOR_REQUEST.receiver().unwrap();

    spawner.must_spawn(motor_task(pwm, dma_ch, receiver));
    led.set_high();
    run_demo(&sender).await
}
