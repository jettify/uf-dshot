#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_stm32::gpio::OutputType;
use embassy_stm32::peripherals::{DMA2_CH5, TIM1};
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::Peri;
use embassy_time::{Duration, Instant, Ticker};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::{Command, DshotSpeed, EncodedFrame, UniTx, WaveformTiming};

#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const DEMO_THROTTLE: u16 = 200;
const ARM_DURATION: Duration = Duration::from_millis(3_000);

struct PwmDshotController<'d> {
    pwm: SimplePwm<'d, TIM1>,
    dma_ch: Peri<'d, DMA2_CH5>,
    tx_period: Duration,
    timing: WaveformTiming,
    motor_stop_cycles: [u16; 17],
    throttle_cycles: [u16; 17],
}

impl<'d> PwmDshotController<'d> {
    fn new(mut pwm: SimplePwm<'d, TIM1>, dma_ch: Peri<'d, DMA2_CH5>, speed: DshotSpeed) -> Self {
        let mut ch1 = pwm.ch1();
        ch1.enable();
        ch1.set_duty_cycle_fully_off();

        let timing = dshot_waveform_timing(pwm.max_duty_cycle());

        Self {
            pwm,
            dma_ch,
            tx_period: Duration::from_micros(u64::from(speed.timing_hints().min_frame_period_us)),
            timing,
            motor_stop_cycles: frame_cycles(UniTx::command(Command::MotorStop).encode(), timing),
            throttle_cycles: [0; 17],
        }
    }

    async fn arm(&mut self) {
        let deadline = Instant::now() + ARM_DURATION;
        let mut ticker = Ticker::every(self.tx_period);

        while Instant::now() < deadline {
            self.pwm
                .waveform_up(
                    self.dma_ch.reborrow(),
                    Channel::Ch1,
                    &self.motor_stop_cycles,
                )
                .await;
            ticker.next().await;
        }
    }

    async fn send_throttle(&mut self, throttle: u16) {
        write_frame_cycles(
            &mut self.throttle_cycles,
            UniTx::throttle_clamped(throttle).encode(),
            self.timing,
        );

        self.pwm
            .waveform_up(self.dma_ch.reborrow(), Channel::Ch1, &self.throttle_cycles)
            .await;
    }
}

fn dshot_waveform_timing(max_duty: u16) -> WaveformTiming {
    WaveformTiming {
        period_ticks: max_duty,
        bit0_high_ticks: (max_duty * 3) / 8,
        bit1_high_ticks: (max_duty * 3) / 4,
    }
}

fn frame_cycles(frame: EncodedFrame, timing: WaveformTiming) -> [u16; 17] {
    let waveform = frame.to_waveform_ticks(timing, true);
    let mut cycles = [0u16; 17];
    cycles[..16].copy_from_slice(&waveform.bit_high_ticks);
    cycles[16] = waveform.reset_low_ticks.unwrap_or(0);
    cycles
}

fn write_frame_cycles(cycles: &mut [u16; 17], frame: EncodedFrame, timing: WaveformTiming) {
    *cycles = frame_cycles(frame, timing);
}

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = embassy_stm32::init(Default::default());

    let ch1_pin = PwmPin::new(p.PA8, OutputType::PushPull);
    let pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1_pin),
        None,
        None,
        None,
        khz(ESC_SPEED.timing_hints().nominal_bitrate_hz / 1_000),
        Default::default(),
    );

    let mut esc = PwmDshotController::new(pwm, p.DMA2_CH5, ESC_SPEED);
    let mut ticker = Ticker::every(Duration::from_micros(u64::from(
        2 * ESC_SPEED.timing_hints().min_frame_period_us,
    )));

    info!("start arm");
    esc.arm().await;

    info!("spinning at throttle {}", DEMO_THROTTLE);
    loop {
        esc.send_throttle(DEMO_THROTTLE).await;
        ticker.next().await;
    }
}
