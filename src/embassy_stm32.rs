//! Experimental Embassy STM32 runtime API for single-line DShot.

use core::marker::PhantomData;

#[cfg(feature = "defmt")]
use defmt::warn;
use embassy_stm32_hal::dma::{Transfer, TransferOptions};
use embassy_stm32_hal::gpio::{AfType, Flex, OutputType, Pull, Speed};
use embassy_stm32_hal::time::Hertz;
use embassy_stm32_hal::timer::low_level::{OutputCompareMode, OutputPolarity, Timer};
use embassy_stm32_hal::timer::{
    BasicInstance, BasicNoCr2Instance, Ch1, Ch2, Ch3, Ch4, Channel, GeneralInstance4Channel,
    TimerChannel, TimerPin, UpDma,
};
use embassy_stm32_hal::Peri;
use embassy_time::{with_timeout, Duration, Instant, Timer as EmbassyTimer};

use crate::command::{BidirTx, Command, DshotSpeed, EncodedFrame, UniTx, WaveformTiming};
use crate::telemetry::{
    BidirDecoder, DecodeHint, OversamplingConfig, PreambleTuningConfig, TelemetryFrame,
    TelemetryPipelineError,
};

const TX_BUFFER_SLOTS: usize = 18;
const MAX_CAPTURE_SAMPLES: usize = 512;
// docs/stm32.rs samples a 100-slot logic-analyzer window for bidirectional telemetry.
const PREAMBLE_MARGIN_SAMPLES: usize = 37;
const DEFAULT_ARM_DURATION: Duration = Duration::from_millis(3_000);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeTimeouts {
    pub tx: Duration,
    pub rx: Duration,
}

#[cfg(feature = "defmt")]
impl defmt::Format for RuntimeTimeouts {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "RuntimeTimeouts {{ tx_us: {}, rx_us: {} }}",
            self.tx.as_micros() as u64,
            self.rx.as_micros() as u64
        );
    }
}

impl Default for RuntimeTimeouts {
    fn default() -> Self {
        Self {
            tx: Duration::from_millis(2),
            rx: Duration::from_millis(2),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Stm32ConfigError {
    SampleBufferTooSmall { requested: usize, capacity: usize },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Stm32RuntimeError {
    TxTimeout,
    RxTimeout,
    Telemetry(TelemetryPipelineError),
}

#[derive(Debug, Clone, Copy)]
enum TxIdleMode {
    DriveLow,
    DriveHigh,
    Input(Pull),
}

#[cfg(feature = "defmt")]
impl defmt::Format for TxIdleMode {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Self::DriveLow => defmt::write!(fmt, "DriveLow"),
            Self::DriveHigh => defmt::write!(fmt, "DriveHigh"),
            Self::Input(Pull::None) => defmt::write!(fmt, "Input(None)"),
            Self::Input(Pull::Up) => defmt::write!(fmt, "Input(Up)"),
            Self::Input(Pull::Down) => defmt::write!(fmt, "Input(Down)"),
        }
    }
}

pub struct DshotTxPin<'d, T: GeneralInstance4Channel> {
    line: Flex<'d>,
    af_num: u8,
    pin_index: u8,
    channel: Channel,
    _timer: PhantomData<T>,
}

impl<'d, T: GeneralInstance4Channel> DshotTxPin<'d, T> {
    pub fn new_ch1(pin: Peri<'d, impl TimerPin<T, Ch1>>) -> Self {
        Self::new(pin, Channel::Ch1)
    }

    pub fn new_ch2(pin: Peri<'d, impl TimerPin<T, Ch2>>) -> Self {
        Self::new(pin, Channel::Ch2)
    }

    pub fn new_ch3(pin: Peri<'d, impl TimerPin<T, Ch3>>) -> Self {
        Self::new(pin, Channel::Ch3)
    }

    pub fn new_ch4(pin: Peri<'d, impl TimerPin<T, Ch4>>) -> Self {
        Self::new(pin, Channel::Ch4)
    }

    fn new<C: TimerChannel>(pin: Peri<'d, impl TimerPin<T, C>>, channel: Channel) -> Self {
        let af_num = pin.af_num();
        let pin_index = pin.pin();

        Self {
            line: Flex::new(pin),
            af_num,
            pin_index,
            channel,
            _timer: PhantomData,
        }
    }

    pub fn channel(&self) -> Channel {
        self.channel
    }

    pub fn pin_index(&self) -> u8 {
        self.pin_index
    }

    /// Release the line and bias it high so an ESC can observe bidirectional DShot idle
    /// before the controller starts sending frames.
    pub fn prepare_bidirectional_idle(&mut self) {
        self.enter_rx_pullup(Pull::Up);
    }

    /// Drive the line low so an ESC sees regular DShot idle before the controller starts.
    pub fn prepare_unidirectional_idle(&mut self) {
        self.enter_idle_low();
    }

    fn enter_tx(&mut self) {
        self.line.set_as_af_unchecked(
            self.af_num,
            AfType::output(OutputType::PushPull, Speed::VeryHigh),
        );
    }

    fn enter_rx_pullup(&mut self, pull: Pull) {
        self.line.set_as_input(pull);
    }

    fn enter_idle_low(&mut self) {
        self.line.set_low();
        self.line.set_as_output(Speed::VeryHigh);
    }

    fn enter_idle_high(&mut self) {
        self.line.set_high();
        self.line.set_as_output(Speed::VeryHigh);
    }

    fn sample_level(&self) -> u32 {
        u32::from(self.line.is_high())
    }
}

pub struct Stm32BidirCapture {
    oversampling: OversamplingConfig,
    sample_count: usize,
    decode_hint: DecodeHint,
    pull: Pull,
    preamble_tuning: PreambleTuningConfig,
    timeouts: RuntimeTimeouts,
}

impl Stm32BidirCapture {
    pub fn new(oversampling: OversamplingConfig) -> Self {
        let sample_count = oversampling.frame_bits as usize * oversampling.oversampling as usize
            + PREAMBLE_MARGIN_SAMPLES;

        Self {
            oversampling,
            sample_count,
            decode_hint: DecodeHint::default(),
            pull: Pull::Up,
            preamble_tuning: PreambleTuningConfig::default(),
            timeouts: RuntimeTimeouts::default(),
        }
    }

    pub fn with_sample_count(mut self, sample_count: usize) -> Self {
        self.sample_count = sample_count;
        self
    }

    pub fn with_decode_hint(mut self, hint: DecodeHint) -> Self {
        self.decode_hint = hint;
        self
    }

    pub fn with_pull(mut self, pull: Pull) -> Self {
        self.pull = pull;
        self
    }

    pub fn with_preamble_tuning(mut self, tuning: PreambleTuningConfig) -> Self {
        self.preamble_tuning = tuning;
        self
    }

    pub fn with_timeouts(mut self, timeouts: RuntimeTimeouts) -> Self {
        self.timeouts = timeouts;
        self
    }
}

struct TxBackend<'d, T, D>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    timer: Timer<'d, T>,
    tx_pin: DshotTxPin<'d, T>,
    tx_dma: Peri<'d, D>,
    waveform_timing: WaveformTiming,
    output_polarity: OutputPolarity,
    timeouts: RuntimeTimeouts,
    idle_mode: TxIdleMode,
    duty_buffer: [u16; TX_BUFFER_SLOTS],
}

impl<'d, T, D> TxBackend<'d, T, D>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    fn new(
        timer: Peri<'d, T>,
        tx_pin: DshotTxPin<'d, T>,
        tx_dma: Peri<'d, D>,
        speed: DshotSpeed,
        output_polarity: OutputPolarity,
        idle_mode: TxIdleMode,
    ) -> Self {
        let timer = Timer::new(timer);
        let period_ticks = prepare_tx_timer(&timer, tx_pin.channel(), speed, output_polarity);
        let waveform_timing = WaveformTiming {
            period_ticks,
            bit0_high_ticks: (period_ticks * 3) / 8,
            bit1_high_ticks: (period_ticks * 3) / 4,
        };

        let mut backend = Self {
            timer,
            tx_pin,
            tx_dma,
            waveform_timing,
            output_polarity,
            timeouts: RuntimeTimeouts::default(),
            idle_mode,
            duty_buffer: [0; TX_BUFFER_SLOTS],
        };
        stop_tx_timer(&backend.timer, backend.tx_pin.channel());
        backend.enter_idle_state();
        backend
    }

    fn with_timeouts(mut self, timeouts: RuntimeTimeouts) -> Self {
        self.timeouts = timeouts;
        self
    }

    async fn send_encoded(
        &mut self,
        frame: EncodedFrame,
        speed: DshotSpeed,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        let waveform = frame.to_waveform_ticks(self.waveform_timing, true);
        self.duty_buffer[..16].copy_from_slice(&waveform.bit_high_ticks);
        self.duty_buffer[16] = waveform.reset_low_ticks.unwrap_or(0);
        self.duty_buffer[17] = 0;

        self.tx_pin.enter_tx();

        // Bidirectional mode needs a deterministic line release after one full reset-low slot.
        // Drive the timer synchronously so the last slot duration does not depend on async wakeup
        // timing or CCR preload transfer ordering.
        if matches!(self.idle_mode, TxIdleMode::Input(_)) {
            send_waveform_sync(&self.timer, self.tx_pin.channel(), &self.duty_buffer[..17]);
            self.enter_idle_state();
            Ok(frame)
        } else {
            match with_timeout(
                self.timeouts.tx,
                send_waveform(
                    &self.timer,
                    self.tx_pin.channel(),
                    self.tx_dma.reborrow(),
                    &self.duty_buffer[..17],
                ),
            )
            .await
            {
                Ok(()) => {
                    self.enter_idle_state();
                    Ok(frame)
                }
                Err(_) => {
                    #[cfg(feature = "defmt")]
                    warn!("dshot tx timeout payload=0x{:04x}", frame.payload);
                    Err(Stm32RuntimeError::TxTimeout)
                }
            }
        }
    }

    fn enter_idle_state(&mut self) {
        match self.idle_mode {
            TxIdleMode::DriveLow => self.tx_pin.enter_idle_low(),
            TxIdleMode::DriveHigh => self.tx_pin.enter_idle_high(),
            TxIdleMode::Input(pull) => self.tx_pin.enter_rx_pullup(pull),
        }
    }
}

struct RxBackend {
    decoder: BidirDecoder,
    sample_count: usize,
    pull: Pull,
    timeouts: RuntimeTimeouts,
    #[cfg(feature = "defmt")]
    capture_attempts: u32,
    #[cfg(feature = "defmt")]
    timeout_count: u32,
    #[cfg(feature = "defmt")]
    decode_error_count: u32,
    raw_samples: [u32; MAX_CAPTURE_SAMPLES],
    sample_words: [u16; MAX_CAPTURE_SAMPLES],
}

impl RxBackend {
    fn new(cfg: Stm32BidirCapture) -> Result<Self, Stm32ConfigError> {
        if cfg.sample_count == 0 || cfg.sample_count > MAX_CAPTURE_SAMPLES {
            return Err(Stm32ConfigError::SampleBufferTooSmall {
                requested: cfg.sample_count,
                capacity: MAX_CAPTURE_SAMPLES,
            });
        }

        let mut decoder = BidirDecoder::with_preamble_tuning(cfg.oversampling, cfg.preamble_tuning);
        decoder.set_stream_hint(cfg.decode_hint);

        Ok(Self {
            decoder,
            sample_count: cfg.sample_count,
            pull: cfg.pull,
            timeouts: cfg.timeouts,
            #[cfg(feature = "defmt")]
            capture_attempts: 0,
            #[cfg(feature = "defmt")]
            timeout_count: 0,
            #[cfg(feature = "defmt")]
            decode_error_count: 0,
            raw_samples: [0; MAX_CAPTURE_SAMPLES],
            sample_words: [0; MAX_CAPTURE_SAMPLES],
        })
    }

    async fn capture<T, TX>(
        &mut self,
        timer: &Timer<'_, T>,
        tx_pin: &mut DshotTxPin<'_, TX>,
        speed: DshotSpeed,
    ) -> Result<TelemetryFrame, Stm32RuntimeError>
    where
        T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
        TX: GeneralInstance4Channel,
    {
        #[cfg(feature = "defmt")]
        {
            self.capture_attempts = self.capture_attempts.wrapping_add(1);
        }
        tx_pin.enter_rx_pullup(self.pull);
        let sample_hz =
            speed.timing_hints().nominal_bitrate_hz * self.decoder.cfg.oversampling as u32;
        let slice = &mut self.raw_samples[..self.sample_count];

        match with_timeout(
            self.timeouts.rx,
            capture_port_samples(timer, tx_pin, sample_hz, slice),
        )
        .await
        {
            Ok(Ok(())) => {}
            Ok(Err(err)) => return Err(err),
            Err(_) => {
                #[cfg(feature = "defmt")]
                {
                    self.timeout_count = self.timeout_count.wrapping_add(1);
                    if self.timeout_count <= 4 || self.timeout_count % 1000 == 0 {
                        let summary = summarize_samples(slice);
                        warn!(
                            "bdshot rx timeout attempts={} timeouts={} sample_hz={} count={} highs={} lows={} edges={} first_edge={:?} last_edge={:?} head=0x{:08x},0x{:08x},0x{:08x},0x{:08x}",
                            self.capture_attempts,
                            self.timeout_count,
                            sample_hz,
                            self.sample_count,
                            summary.high_count,
                            summary.low_count,
                            summary.edge_count,
                            summary.first_edge,
                            summary.last_edge,
                            slice.first().copied().unwrap_or(0),
                            slice.get(1).copied().unwrap_or(0),
                            slice.get(2).copied().unwrap_or(0),
                            slice.get(3).copied().unwrap_or(0),
                        );
                    }
                }
                return Err(Stm32RuntimeError::RxTimeout);
            }
        }

        for (dst, src) in self.sample_words[..self.sample_count]
            .iter_mut()
            .zip(slice.iter())
        {
            *dst = *src as u16;
        }

        match self
            .decoder
            .decode_frame_tuned(&self.sample_words[..self.sample_count])
        {
            Ok(frame) => Ok(frame),
            Err(err) => {
                #[cfg(feature = "defmt")]
                {
                    self.decode_error_count = self.decode_error_count.wrapping_add(1);
                    let summary = summarize_samples_u16(&self.sample_words[..self.sample_count]);
                    let stats = self.decoder.stats();
                    warn!(
                        "bdshot decode err count={} err={} highs={} lows={} edges={} first_edge={:?} last_edge={:?} hint_skip={} stats ok={} no_edge={} short={} invalid_frame={} invalid_gcr={} invalid_crc={} start_margin={} head={},{},{},{}",
                        self.decode_error_count,
                        err,
                        summary.high_count,
                        summary.low_count,
                        summary.edge_count,
                        summary.first_edge,
                        summary.last_edge,
                        self.decoder.stream_hint().preamble_skip,
                        stats.successful_frames,
                        stats.no_edge,
                        stats.frame_too_short,
                        stats.invalid_frame,
                        stats.invalid_gcr_symbol,
                        stats.invalid_crc,
                        stats.last_start_margin,
                        self.sample_words.first().copied().unwrap_or(0),
                        self.sample_words.get(1).copied().unwrap_or(0),
                        self.sample_words.get(2).copied().unwrap_or(0),
                        self.sample_words.get(3).copied().unwrap_or(0),
                    );
                }
                Err(Stm32RuntimeError::Telemetry(err))
            }
        }
    }
}

#[cfg(feature = "defmt")]
struct SampleSummary {
    high_count: usize,
    low_count: usize,
    edge_count: usize,
    first_edge: Option<usize>,
    last_edge: Option<usize>,
}

#[cfg(feature = "defmt")]
fn summarize_levels<I>(mut iter: I) -> SampleSummary
where
    I: Iterator<Item = bool>,
{
    let mut high_count = 0usize;
    let mut low_count = 0usize;
    let mut edge_count = 0usize;
    let mut first_edge = None;
    let mut last_edge = None;
    let mut prev = None;

    for (idx, level) in iter.by_ref().enumerate() {
        if level {
            high_count += 1;
        } else {
            low_count += 1;
        }

        if let Some(prev_level) = prev {
            if prev_level != level {
                edge_count += 1;
                if first_edge.is_none() {
                    first_edge = Some(idx);
                }
                last_edge = Some(idx);
            }
        }
        prev = Some(level);
    }

    SampleSummary {
        high_count,
        low_count,
        edge_count,
        first_edge,
        last_edge,
    }
}

#[cfg(feature = "defmt")]
fn summarize_samples(samples: &[u32]) -> SampleSummary {
    summarize_levels(samples.iter().copied().map(|v| v != 0))
}

#[cfg(feature = "defmt")]
fn summarize_samples_u16(samples: &[u16]) -> SampleSummary {
    summarize_levels(samples.iter().copied().map(|v| v != 0))
}

struct BidirBackend<'d, T, D>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    tx: TxBackend<'d, T, D>,
    rx: RxBackend,
    speed: DshotSpeed,
    output_polarity: OutputPolarity,
}

impl<'d, T, D> BidirBackend<'d, T, D>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    fn new(
        timer: Peri<'d, T>,
        tx_pin: DshotTxPin<'d, T>,
        tx_dma: Peri<'d, D>,
        mut rx_cfg: Stm32BidirCapture,
        speed: DshotSpeed,
    ) -> Result<Self, Stm32ConfigError> {
        // We sample a single pin via Flex::is_high(), so the result is always in bit 0.
        rx_cfg.oversampling.sample_bit_index = 0;
        let output_polarity = OutputPolarity::ActiveLow;
        let tx = TxBackend::new(
            timer,
            tx_pin,
            tx_dma,
            speed,
            output_polarity,
            TxIdleMode::Input(Pull::Up),
        )
        .with_timeouts(rx_cfg.timeouts);

        Ok(Self {
            tx,
            rx: RxBackend::new(rx_cfg)?,
            speed,
            output_polarity,
        })
    }

    async fn send_encoded(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        prepare_tx_timer(
            &self.tx.timer,
            self.tx.tx_pin.channel(),
            self.speed,
            self.output_polarity,
        );

        let res = self.tx.send_encoded(frame, self.speed).await;
        stop_tx_timer(&self.tx.timer, self.tx.tx_pin.channel());
        res
    }

    async fn capture(&mut self) -> Result<TelemetryFrame, Stm32RuntimeError> {
        self.rx
            .capture(&self.tx.timer, &mut self.tx.tx_pin, self.speed)
            .await
    }
}

pub struct Stm32DshotController<'d, T, D>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    tx: TxBackend<'d, T, D>,
    speed: DshotSpeed,
}

pub struct Stm32BidirTxController<'d, T, D>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    tx: TxBackend<'d, T, D>,
    speed: DshotSpeed,
}

impl<'d, T, D> Stm32DshotController<'d, T, D>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    pub fn tx_only(
        timer: Peri<'d, T>,
        tx_pin: DshotTxPin<'d, T>,
        tx_dma: Peri<'d, D>,
        speed: DshotSpeed,
    ) -> Self {
        Self {
            tx: TxBackend::new(
                timer,
                tx_pin,
                tx_dma,
                speed,
                OutputPolarity::ActiveHigh,
                TxIdleMode::DriveLow,
            ),
            speed,
        }
    }

    pub fn with_timeouts(mut self, timeouts: RuntimeTimeouts) -> Self {
        self.tx = self.tx.with_timeouts(timeouts);
        self
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), Stm32RuntimeError> {
        let output_polarity = self.tx.output_polarity;
        arm_tx_backend_for(
            &mut self.tx,
            self.speed,
            output_polarity,
            duration,
            UniTx::command(Command::MotorStop).encode(),
        )
        .await
    }

    pub async fn arm(&mut self) -> Result<(), Stm32RuntimeError> {
        self.arm_for(DEFAULT_ARM_DURATION).await
    }

    pub async fn send_throttle(
        &mut self,
        throttle: u16,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        self.send_frame(UniTx::throttle_clamped(throttle).encode())
            .await
    }

    pub async fn send_command(
        &mut self,
        command: Command,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        let output_polarity = self.tx.output_polarity;
        send_command_with_policy(
            &mut self.tx,
            self.speed,
            output_polarity,
            UniTx::command(command).encode(),
            command,
        )
        .await
    }

    pub async fn send_frame(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        prepare_tx_timer(
            &self.tx.timer,
            self.tx.tx_pin.channel(),
            self.speed,
            self.tx.output_polarity,
        );
        let res = self.tx.send_encoded(frame, self.speed).await;
        stop_tx_timer(&self.tx.timer, self.tx.tx_pin.channel());
        res
    }
}

impl<'d, T, D> Stm32BidirTxController<'d, T, D>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    pub fn tx_only(
        timer: Peri<'d, T>,
        tx_pin: DshotTxPin<'d, T>,
        tx_dma: Peri<'d, D>,
        speed: DshotSpeed,
    ) -> Self {
        Self {
            tx: TxBackend::new(
                timer,
                tx_pin,
                tx_dma,
                speed,
                OutputPolarity::ActiveLow,
                TxIdleMode::Input(Pull::Up),
            ),
            speed,
        }
    }

    pub fn with_timeouts(mut self, timeouts: RuntimeTimeouts) -> Self {
        self.tx = self.tx.with_timeouts(timeouts);
        self
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), Stm32RuntimeError> {
        let output_polarity = self.tx.output_polarity;
        arm_tx_backend_for(
            &mut self.tx,
            self.speed,
            output_polarity,
            duration,
            BidirTx::command(Command::MotorStop).encode(),
        )
        .await
    }

    pub async fn arm(&mut self) -> Result<(), Stm32RuntimeError> {
        self.arm_for(DEFAULT_ARM_DURATION).await
    }

    pub async fn send_throttle(
        &mut self,
        throttle: u16,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        self.send_frame(BidirTx::throttle_clamped(throttle).encode())
            .await
    }

    pub async fn send_command(
        &mut self,
        command: Command,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        let output_polarity = self.tx.output_polarity;
        send_command_with_policy(
            &mut self.tx,
            self.speed,
            output_polarity,
            BidirTx::command(command).encode(),
            command,
        )
        .await
    }

    pub async fn send_frame(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        prepare_tx_timer(
            &self.tx.timer,
            self.tx.tx_pin.channel(),
            self.speed,
            self.tx.output_polarity,
        );
        let res = self.tx.send_encoded(frame, self.speed).await;
        stop_tx_timer(&self.tx.timer, self.tx.tx_pin.channel());
        res
    }
}

pub struct Stm32BidirController<'d, T, D>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    backend: BidirBackend<'d, T, D>,
}

impl<'d, T, D> Stm32BidirController<'d, T, D>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    pub fn bidirectional(
        timer: Peri<'d, T>,
        tx_pin: DshotTxPin<'d, T>,
        tx_dma: Peri<'d, D>,
        rx_cfg: Stm32BidirCapture,
        speed: DshotSpeed,
    ) -> Result<Self, Stm32ConfigError> {
        Ok(Self {
            backend: BidirBackend::new(timer, tx_pin, tx_dma, rx_cfg, speed)?,
        })
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), Stm32RuntimeError> {
        let output_polarity = self.backend.output_polarity;
        arm_tx_backend_for(
            &mut self.backend.tx,
            self.backend.speed,
            output_polarity,
            duration,
            BidirTx::command(Command::MotorStop).encode(),
        )
        .await
    }

    pub async fn arm(&mut self) -> Result<(), Stm32RuntimeError> {
        self.arm_for(DEFAULT_ARM_DURATION).await
    }

    pub async fn send_throttle(
        &mut self,
        throttle: u16,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        self.send_frame(BidirTx::throttle_clamped(throttle).encode())
            .await
    }

    pub async fn send_command(
        &mut self,
        command: Command,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        let output_polarity = self.backend.output_polarity;
        send_command_with_policy(
            &mut self.backend.tx,
            self.backend.speed,
            output_polarity,
            BidirTx::command(command).encode(),
            command,
        )
        .await
    }

    pub async fn send_command_and_receive(
        &mut self,
        command: Command,
    ) -> Result<TelemetryFrame, Stm32RuntimeError> {
        let output_polarity = self.backend.output_polarity;
        send_command_with_policy(
            &mut self.backend.tx,
            self.backend.speed,
            output_polarity,
            BidirTx::command(command).encode(),
            command,
        )
        .await?;
        self.backend.capture().await
    }

    pub async fn send_frame(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        self.backend.send_encoded(frame).await
    }

    pub async fn send_throttle_and_receive(
        &mut self,
        throttle: u16,
    ) -> Result<TelemetryFrame, Stm32RuntimeError> {
        self.send_frame(BidirTx::throttle_clamped(throttle).encode())
            .await?;
        self.backend.capture().await
    }
}

fn prepare_tx_timer<T>(
    timer: &Timer<'_, T>,
    channel: Channel,
    speed: DshotSpeed,
    output_polarity: OutputPolarity,
) -> u16
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
{
    timer.stop();
    timer.enable_update_dma(false);
    timer.clear_update_interrupt();
    timer.set_output_compare_mode(channel, OutputCompareMode::PwmMode1);
    timer.set_output_compare_preload(channel, true);
    timer.set_output_polarity(channel, output_polarity);
    timer.enable_channel(channel, true);
    timer.set_frequency(Hertz(speed.timing_hints().nominal_bitrate_hz));
    timer.set_compare_value(channel, 0.into());
    timer.enable_outputs();
    // Latch PSC/ARR/CCR preload state and restart from a known phase for each frame.
    timer.generate_update_event();
    timer.reset();
    timer.clear_update_interrupt();
    timer.start();

    (timer.get_max_compare_value().into() as u16).saturating_add(1)
}

fn stop_tx_timer<T>(timer: &Timer<'_, T>, channel: Channel)
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
{
    timer.enable_update_dma(false);
    timer.set_compare_value(channel, 0.into());
    timer.enable_channel(channel, false);
    timer.stop();
}

fn send_waveform_sync<T>(timer: &Timer<'_, T>, channel: Channel, duty: &[u16])
where
    T: GeneralInstance4Channel + BasicNoCr2Instance + BasicInstance,
{
    if duty.is_empty() {
        return;
    }

    // Manual slot stepping writes CCR after each update event, so preload must be disabled.
    // Prime the first slot before starting the counter, then program each next slot on the
    // update boundary and wait for the final slot to complete before releasing the line.
    timer.stop();
    timer.set_output_compare_preload(channel, false);
    timer.set_compare_value(channel, duty[0].into());
    timer.reset();
    timer.clear_update_interrupt();
    timer.start();

    for &val in &duty[1..] {
        while !timer.clear_update_interrupt() {
            core::hint::spin_loop();
        }
        timer.set_compare_value(channel, val.into());
    }

    // Wait for the final programmed slot to complete before the caller releases the line.
    while !timer.clear_update_interrupt() {
        core::hint::spin_loop();
    }
}

async fn send_waveform<T, D>(timer: &Timer<'_, T>, channel: Channel, dma: Peri<'_, D>, duty: &[u16])
where
    T: GeneralInstance4Channel + BasicNoCr2Instance + BasicInstance,
    D: UpDma<T>,
{
    let original_update_dma_state = timer.get_update_dma_state();
    if !original_update_dma_state {
        timer.enable_update_dma(true);
    }

    let req = dma.request();
    let ccr = timer.regs_gp16().ccr(channel.index()).as_ptr() as *mut u16;
    let options = TransferOptions::default();

    // SAFETY: DMA writes the provided waveform buffer into the timer CCR register while
    // both the timer peripheral and the buffer remain alive for the transfer duration.
    unsafe {
        Transfer::new_write(dma, req, duty, ccr, options).await;
    }

    if !original_update_dma_state {
        timer.enable_update_dma(false);
    }
}

fn bit_period_duration(speed: DshotSpeed) -> Duration {
    let bitrate_hz = u64::from(speed.timing_hints().nominal_bitrate_hz);
    Duration::from_micros(1_000_000_u64.div_ceil(bitrate_hz))
}

async fn capture_port_samples<T, TX>(
    timer: &Timer<'_, T>,
    tx_pin: &DshotTxPin<'_, TX>,
    sample_hz: u32,
    buffer: &mut [u32],
) -> Result<(), Stm32RuntimeError>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    TX: GeneralInstance4Channel,
{
    timer.stop();
    timer.set_frequency(Hertz(sample_hz));
    timer.clear_update_interrupt();
    timer.start();

    let sample_period_us = 1_000_000_u64.div_ceil(u64::from(sample_hz));
    let capture_budget_us = sample_period_us
        .saturating_mul(buffer.len() as u64)
        .saturating_add(sample_period_us.saturating_mul(4));
    let deadline = Instant::now() + Duration::from_micros(capture_budget_us);

    for sample in buffer.iter_mut() {
        while !timer.clear_update_interrupt() {
            if Instant::now() >= deadline {
                timer.stop();
                return Err(Stm32RuntimeError::RxTimeout);
            }
            core::hint::spin_loop();
        }

        *sample = tx_pin.sample_level();
    }

    timer.stop();
    Ok(())
}

async fn send_command_with_policy<T, D>(
    tx: &mut TxBackend<'_, T, D>,
    speed: DshotSpeed,
    output_polarity: OutputPolarity,
    frame: EncodedFrame,
    command: Command,
) -> Result<EncodedFrame, Stm32RuntimeError>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    let policy = command.exec_policy();
    let mut last_frame = frame;

    prepare_tx_timer(&tx.timer, tx.tx_pin.channel(), speed, output_polarity);
    for _ in 0..policy.repeat_count().get() {
        last_frame = tx.send_encoded(frame, speed).await?;
    }
    stop_tx_timer(&tx.timer, tx.tx_pin.channel());

    let min_gap = policy.min_gap();
    if !min_gap.is_zero() {
        EmbassyTimer::after(Duration::from_micros(min_gap.as_micros() as u64)).await;
    }

    Ok(last_frame)
}

async fn arm_tx_backend_for<T, D>(
    tx: &mut TxBackend<'_, T, D>,
    speed: DshotSpeed,
    output_polarity: OutputPolarity,
    duration: Duration,
    frame: EncodedFrame,
) -> Result<(), Stm32RuntimeError>
where
    T: GeneralInstance4Channel + BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    let frame_period = Duration::from_micros(speed.timing_hints().min_frame_period_us as u64);
    let deadline = Instant::now() + duration;
    let mut next_frame_at = Instant::now();

    prepare_tx_timer(&tx.timer, tx.tx_pin.channel(), speed, output_polarity);
    while Instant::now() < deadline {
        tx.send_encoded(frame, speed).await?;
        next_frame_at += frame_period;
        let now = Instant::now();
        if now < next_frame_at {
            EmbassyTimer::at(next_frame_at).await;
        } else {
            next_frame_at = now;
        }
    }
    stop_tx_timer(&tx.timer, tx.tx_pin.channel());

    Ok(())
}
