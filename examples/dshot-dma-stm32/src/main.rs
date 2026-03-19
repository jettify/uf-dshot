#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_executor::Spawner;
use embassy_stm32::dma::{Transfer, TransferOptions};
use embassy_stm32::gpio::{AfType, Flex, OutputType, Speed};
use embassy_stm32::timer::low_level::{OutputCompareMode, OutputPolarity, Timer};
use embassy_stm32::timer::{
    AdvancedInstance4Channel, BasicInstance, BasicNoCr2Instance, Channel as TimerChannel,
    GeneralInstance1Channel,
};
use embassy_stm32::{time::Hertz, Peri};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Receiver, Sender, Watch};
use embassy_time::{Duration, Ticker, Timer as EmbassyTimer};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::{UniTx, WaveformTiming};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

static MOTOR_THROTTLE: Watch<ThreadModeRawMutex, u16, 1> = Watch::new_with(0);

const DSHOT_BITRATE_HZ: u32 = 300_000;
const ARMING_TIME_MS: u32 = 3_000;
const TX_PERIOD_US: u32 = 54;
const TX_LOG_EVERY_FRAMES: u32 = 20_000;
const WAVEFORM_LEN: usize = 17;
const TIM1_CH1_AF: u8 = 1;

struct BoardConfig {
    line: Flex<'static>,
    send_timer: Timer<'static, embassy_stm32::peripherals::TIM1>,
    send_dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA2_CH5>,
    send_channel: TimerChannel,
}

fn setup_dshot_timer<T: AdvancedInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: TimerChannel,
    bitrate_hz: u32,
) -> u16 {
    timer.stop();
    timer.set_output_compare_mode(channel, OutputCompareMode::PwmMode1);
    timer.set_output_compare_preload(channel, true);
    timer.set_output_polarity(channel, OutputPolarity::ActiveHigh);
    timer.enable_channel(channel, true);

    timer.set_frequency(Hertz(bitrate_hz));
    timer.set_compare_value(channel, 0.into());
    timer.enable_outputs();
    timer.set_moe(true);
    timer.start();

    (timer.get_max_compare_value().into() as u16).saturating_add(1)
}

async fn send_waveform<T: GeneralInstance1Channel + BasicNoCr2Instance + BasicInstance>(
    timer: &Timer<'_, T>,
    channel: TimerChannel,
    dma: Peri<'_, impl embassy_stm32::timer::UpDma<T>>,
    duty: &[u16; WAVEFORM_LEN],
) {
    let original_update_dma_state = timer.get_update_dma_state();
    if !original_update_dma_state {
        timer.enable_update_dma(true);
    }

    let req = dma.request();
    let ccr = timer.regs_1ch().ccr(channel.index()).as_ptr() as *mut u16;
    let mut options = TransferOptions::default();
    options.fifo_threshold = Some(embassy_stm32::dma::FifoThreshold::Full);
    options.mburst = embassy_stm32::dma::Burst::Incr8;

    // SAFETY: DMA writes the duty buffer to the timer CCR while both remain alive.
    unsafe {
        Transfer::new_write(dma, req, duty, ccr, options).await;
    }

    if !original_update_dma_state {
        timer.enable_update_dma(false);
    }
}

fn make_timing(max_duty: u16) -> WaveformTiming {
    WaveformTiming {
        period_ticks: max_duty,
        bit0_high_ticks: (max_duty * 3) / 8,
        bit1_high_ticks: (max_duty * 3) / 4,
    }
}

fn make_cycles(throttle_percent: u16, timing: WaveformTiming) -> ([u16; WAVEFORM_LEN], u16, u8) {
    let throttle_percent = throttle_percent.clamp(0, 100);
    let raw_throttle = (throttle_percent as u32 * 1999 / 100) as u16;
    let encoded = UniTx::throttle_clamped(raw_throttle).encode();

    let waveform = encoded.to_waveform_ticks(timing, true);

    let mut cycles = [0u16; WAVEFORM_LEN];
    cycles[..16].copy_from_slice(&waveform.bit_high_ticks);
    cycles[16] = 0;

    (cycles, encoded.payload, encoded.crc_4())
}

fn init_output_pin(line: &mut Flex<'static>) {
    line.set_as_af_unchecked(
        TIM1_CH1_AF,
        AfType::output(OutputType::PushPull, Speed::VeryHigh),
    );
}

async fn run_demo(sender: &Sender<'static, ThreadModeRawMutex, u16, 1>) -> ! {
    EmbassyTimer::after(Duration::from_millis(7000)).await;
    info!("start loop");

    loop {
        for value in 0..=25 {
            sender.send(value);
            info!("dshot dma up {}", value);
            EmbassyTimer::after(Duration::from_millis(1000)).await;
        }
        for value in (0..=25).rev() {
            sender.send(value);
            info!("dshot dma down {}", value);
            EmbassyTimer::after(Duration::from_millis(1000)).await;
        }
    }
}

#[embassy_executor::task]
async fn motor_task(
    mut cfg: BoardConfig,
    mut receiver: Receiver<'static, ThreadModeRawMutex, u16, 1>,
) -> ! {
    let max_duty = setup_dshot_timer(&cfg.send_timer, cfg.send_channel, DSHOT_BITRATE_HZ);
    let timing = make_timing(max_duty);
    let arming_frames = (ARMING_TIME_MS * 1_000) / TX_PERIOD_US;
    let mut frames_sent = 0u32;
    let mut throttle = 0u16;
    let mut ticker = Ticker::every(Duration::from_micros(TX_PERIOD_US as u64));
    let mut arming_remaining = arming_frames;

    info!("tx loop started dshot300 bitrate={}Hz", DSHOT_BITRATE_HZ);

    loop {
        frames_sent = frames_sent.wrapping_add(1);
        throttle = receiver.try_get().unwrap_or(throttle);

        let arming = arming_remaining > 0;
        let commanded = if arming { 0 } else { throttle };
        if arming_remaining > 0 {
            arming_remaining -= 1;
        } else if frames_sent == arming_frames.wrapping_add(1) {
            info!("arming complete, accepting throttle");
        }

        let (cycles, payload, crc) = make_cycles(commanded, timing);
        if frames_sent % TX_LOG_EVERY_FRAMES == 0 {
            info!(
                "tx throttle={} payload=0x{:04X} crc=0x{:X} hi=[{},{},{},{}] arming={}",
                commanded, payload, crc, cycles[0], cycles[1], cycles[2], cycles[3], arming
            );
        }

        send_waveform(
            &cfg.send_timer,
            cfg.send_channel,
            cfg.send_dma.reborrow(),
            &cycles,
        )
        .await;

        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let mut cfg = BoardConfig {
        line: Flex::new(p.PA8),
        send_timer: Timer::new(p.TIM1),
        send_dma: p.DMA2_CH5,
        send_channel: TimerChannel::Ch1,
    };
    init_output_pin(&mut cfg.line);

    let sender = MOTOR_THROTTLE.sender();
    sender.send(0);
    let receiver = MOTOR_THROTTLE.receiver().unwrap();

    spawner.must_spawn(motor_task(cfg, receiver));
    run_demo(&sender).await
}
