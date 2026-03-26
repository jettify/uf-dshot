//! Experimental Embassy STM32 runtime API for single-line DShot.

use core::marker::PhantomData;

use embassy_stm32_hal::dma::{
    Channel as DmaChannel, InterruptHandler as DmaInterruptHandler, TransferOptions,
};
use embassy_stm32_hal::gpio::{AfType, AnyPin, Flex, OutputType, Pull, Speed};
use embassy_stm32_hal::interrupt::typelevel::Binding;
use embassy_stm32_hal::time::Hertz;
use embassy_stm32_hal::timer::low_level::{OutputCompareMode, OutputPolarity, RoundTo, Timer};
use embassy_stm32_hal::timer::{
    Ch1, Ch2, Ch3, Ch4, Channel, GeneralInstance4Channel, TimerChannel, TimerPin, UpDma,
};
use embassy_stm32_hal::Peri;
use embassy_time::{with_timeout, Duration, Instant, Timer as EmbassyTimer};

use crate::command::{BidirTx, Command, DshotSpeed, EncodedFrame, UniTx, WaveformTiming};
use crate::telemetry::{
    BidirDecoder, DecodeHint, OversamplingConfig, PreambleTuningConfig, TelemetryFrame,
    TelemetryPipelineError,
};

const TX_BUFFER_SLOTS: usize = 17;
const MAX_CAPTURE_SAMPLES: usize = 512;
const PINS_PER_GPIO_PORT: u8 = 16;
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
    Input(Pull),
}

#[cfg(feature = "defmt")]
impl defmt::Format for TxIdleMode {
    fn format(&self, fmt: defmt::Formatter) {
        match self {
            Self::DriveLow => defmt::write!(fmt, "DriveLow"),
            Self::Input(Pull::None) => defmt::write!(fmt, "Input(None)"),
            Self::Input(Pull::Up) => defmt::write!(fmt, "Input(Up)"),
            Self::Input(Pull::Down) => defmt::write!(fmt, "Input(Down)"),
        }
    }
}

pub struct DshotTxPin<'d, T: GeneralInstance4Channel> {
    line: Flex<'d>,
    af_num: u8,
    pin_mask: u32,
    idr_ptr: *const u32,
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
        let pin_mask = 1u32 << pin.pin();
        let pin_port = (pin.port() as usize) * (PINS_PER_GPIO_PORT as usize) + (pin.pin() as usize);
        let idr_ptr = unsafe { AnyPin::steal(pin_port as u8) }
            .block()
            .idr()
            .as_ptr() as *const u32;

        Self {
            line: Flex::new(pin),
            af_num,
            pin_mask,
            idr_ptr,
            channel,
            _timer: PhantomData,
        }
    }

    pub fn channel(&self) -> Channel {
        self.channel
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

    fn idr_ptr(&self) -> *const u32 {
        self.idr_ptr
    }

    fn normalize_sample(&self, sample: u32) -> u16 {
        if (sample & self.pin_mask) != 0 {
            1
        } else {
            0
        }
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
        let sample_count = oversampling.recommended_capture_samples(PREAMBLE_MARGIN_SAMPLES);

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
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    timer: Timer<'d, T>,
    tx_pin: DshotTxPin<'d, T>,
    tx_dma: DmaChannel<'d>,
    tx_dma_request: embassy_stm32_hal::dma::Request,
    _tx_dma: PhantomData<D>,
    speed: DshotSpeed,
    waveform_timing: WaveformTiming,
    output_polarity: OutputPolarity,
    timeouts: RuntimeTimeouts,
    idle_mode: TxIdleMode,
    line_in_tx_mode: bool,
    duty_buffer: [T::Word; TX_BUFFER_SLOTS],
}

impl<'d, T, D> TxBackend<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    fn new(
        timer: Peri<'d, T>,
        tx_pin: DshotTxPin<'d, T>,
        tx_dma: Peri<'d, D>,
        tx_dma_irq: impl Binding<D::Interrupt, DmaInterruptHandler<D>> + 'd,
        speed: DshotSpeed,
        output_polarity: OutputPolarity,
        idle_mode: TxIdleMode,
    ) -> Self {
        let timer = Timer::new(timer);
        let tx_dma_request = tx_dma.request();
        let tx_dma = DmaChannel::new(tx_dma, tx_dma_irq);
        let period_ticks = configure_tx_timer(&timer, tx_pin.channel(), speed, output_polarity);
        let waveform_timing = WaveformTiming {
            period_ticks,
            bit0_high_ticks: (period_ticks * 3) / 8,
            bit1_high_ticks: (period_ticks * 3) / 4,
        };

        let mut backend = Self {
            timer,
            tx_pin,
            tx_dma,
            tx_dma_request,
            _tx_dma: PhantomData,
            speed,
            waveform_timing,
            output_polarity,
            timeouts: RuntimeTimeouts::default(),
            idle_mode,
            line_in_tx_mode: true,
            duty_buffer: [T::Word::from(0u16); TX_BUFFER_SLOTS],
        };
        stop_tx_timer(&backend.timer, backend.tx_pin.channel());
        backend.enter_idle_state();
        backend
    }

    fn with_timeouts(mut self, timeouts: RuntimeTimeouts) -> Self {
        self.timeouts = timeouts;
        self
    }

    fn prepare_for_tx(&mut self) {
        configure_tx_timer(
            &self.timer,
            self.tx_pin.channel(),
            self.speed,
            self.output_polarity,
        );

        if !self.line_in_tx_mode {
            self.tx_pin.enter_tx();
            self.line_in_tx_mode = true;
        }
    }

    async fn send_encoded(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        let waveform = frame.to_waveform_ticks(self.waveform_timing, true);
        for (slot, ticks) in self.duty_buffer[..16]
            .iter_mut()
            .zip(waveform.bit_high_ticks.iter().copied())
        {
            *slot = ticks.into();
        }
        self.duty_buffer[16] = waveform.reset_low_ticks.unwrap_or(0).into();

        self.prepare_for_tx();
        let _cleanup = TxCleanupGuard::new(
            &self.timer,
            &mut self.tx_pin,
            self.idle_mode,
            &mut self.line_in_tx_mode,
        );

        let ccr = self
            .timer
            .regs_gp16()
            .ccr(_cleanup.channel.index())
            .as_ptr() as *mut T::Word;
        let options = TransferOptions::default();
        let duty_buffer = &self.duty_buffer;
        let tx_dma = &mut self.tx_dma;
        let tx_dma_request = self.tx_dma_request;

        // SAFETY: DMA writes the provided waveform buffer into the timer CCR register while
        // both the timer peripheral and the buffer remain alive for the transfer duration.
        let tx_result = unsafe {
            let transfer = tx_dma.write(tx_dma_request, duty_buffer, ccr, options);
            start_timer(&self.timer);
            with_timeout(self.timeouts.tx, transfer).await
        };

        match tx_result {
            Ok(()) => {
                // DMA completion means the trailing CCR=0 reset slot has been loaded into the
                // timer, not that the final low interval has finished on the wire yet. Wait one
                // bit period before cleanup so the line does not glitch between frames.
                //EmbassyTimer::after(bit_period_duration(self.speed)).await;

                Ok(frame)
            }
            Err(_) => Err(Stm32RuntimeError::TxTimeout),
        }
    }

    fn enter_idle_state(&mut self) {
        match self.idle_mode {
            TxIdleMode::DriveLow => self.tx_pin.enter_idle_low(),
            TxIdleMode::Input(pull) => self.tx_pin.enter_rx_pullup(pull),
        }
        self.line_in_tx_mode = false;
    }
}

struct TxCleanupGuard<'a, 'd, T>
where
    T: GeneralInstance4Channel,
{
    timer: &'a Timer<'d, T>,
    channel: Channel,
    tx_pin: &'a mut DshotTxPin<'d, T>,
    idle_mode: TxIdleMode,
    line_in_tx_mode: &'a mut bool,
}

impl<'a, 'd, T> TxCleanupGuard<'a, 'd, T>
where
    T: GeneralInstance4Channel,
{
    fn new(
        timer: &'a Timer<'d, T>,
        tx_pin: &'a mut DshotTxPin<'d, T>,
        idle_mode: TxIdleMode,
        line_in_tx_mode: &'a mut bool,
    ) -> Self {
        Self {
            timer,
            channel: tx_pin.channel(),
            tx_pin,
            idle_mode,
            line_in_tx_mode,
        }
    }
}

impl<T> Drop for TxCleanupGuard<'_, '_, T>
where
    T: GeneralInstance4Channel,
{
    fn drop(&mut self) {
        match self.idle_mode {
            TxIdleMode::DriveLow => {
                // Disconnect the pin from the timer before stopping the channel so the timer
                // shutdown sequence cannot leak a final pulse onto the wire.
                self.tx_pin.enter_idle_low();
                stop_tx_timer(self.timer, self.channel);
            }
            TxIdleMode::Input(pull) => {
                // In bidirectional mode the ESC needs to take over the line immediately after
                // the trailing reset slot. Release the pin to pulled-up input before stopping
                // the timer so TIM shutdown activity cannot mask the first telemetry edge.
                self.tx_pin.enter_rx_pullup(pull);
                stop_tx_timer(self.timer, self.channel);
            }
        }
        *self.line_in_tx_mode = false;
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
    fn new(mut cfg: Stm32BidirCapture) -> Result<Self, Stm32ConfigError> {
        if cfg.sample_count == 0 || cfg.sample_count > MAX_CAPTURE_SAMPLES {
            return Err(Stm32ConfigError::SampleBufferTooSmall {
                requested: cfg.sample_count,
                capacity: MAX_CAPTURE_SAMPLES,
            });
        }

        // Raw GPIO IDR samples are normalized into bit 0 before decode.
        cfg.oversampling.sample_bit_index = 0;

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

    async fn capture<T>(
        &mut self,
        timer: &Timer<'_, T>,
        tx_pin: &mut DshotTxPin<'_, T>,
        dma: &mut DmaChannel<'_>,
        dma_request: embassy_stm32_hal::dma::Request,
        speed: DshotSpeed,
    ) -> Result<TelemetryFrame, Stm32RuntimeError>
    where
        T: GeneralInstance4Channel,
    {
        #[cfg(feature = "defmt")]
        {
            self.capture_attempts = self.capture_attempts.wrapping_add(1);
        }

        tx_pin.enter_rx_pullup(self.pull);
        let sample_hz =
            speed.timing_hints().nominal_bitrate_hz * self.decoder.cfg.oversampling as u32;
        let slice = &mut self.raw_samples[..self.sample_count];

        match capture_port_samples(
            timer,
            tx_pin,
            dma,
            dma_request,
            sample_hz,
            slice,
            self.timeouts.rx,
        )
        .await
        {
            Ok(()) => {}
            Err(Stm32RuntimeError::RxTimeout) => {
                #[cfg(feature = "defmt")]
                {
                    self.timeout_count = self.timeout_count.wrapping_add(1);
                    if self.timeout_count <= 4 || self.timeout_count % 1000 == 0 {
                        let summary = summarize_samples(slice);
                        defmt::warn!(
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
            Err(err) => return Err(err),
        }

        for (dst, src) in self.sample_words[..self.sample_count]
            .iter_mut()
            .zip(slice.iter().copied())
        {
            *dst = tx_pin.normalize_sample(src);
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
                    if self.decode_error_count <= 4 || self.decode_error_count % 1000 == 0 {
                        let summary =
                            summarize_samples_u16(&self.sample_words[..self.sample_count]);
                        let stats = self.decoder.stats();
                        defmt::warn!(
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

pub struct Stm32DshotController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    tx: TxBackend<'d, T, D>,
    speed: DshotSpeed,
}

pub struct Stm32BidirTxController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    tx: TxBackend<'d, T, D>,
    speed: DshotSpeed,
}

pub struct Stm32BidirController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    tx: TxBackend<'d, T, D>,
    rx: RxBackend,
    speed: DshotSpeed,
}

impl<'d, T, D> Stm32DshotController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    pub fn tx_only(
        timer: Peri<'d, T>,
        tx_pin: DshotTxPin<'d, T>,
        tx_dma: Peri<'d, D>,
        tx_dma_irq: impl Binding<D::Interrupt, DmaInterruptHandler<D>> + 'd,
        speed: DshotSpeed,
    ) -> Self {
        Self {
            tx: TxBackend::new(
                timer,
                tx_pin,
                tx_dma,
                tx_dma_irq,
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
        arm_tx_backend_for(
            &mut self.tx,
            self.speed,
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
        send_command_with_policy(&mut self.tx, UniTx::command(command).encode(), command).await
    }

    pub async fn send_frame(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        self.tx.send_encoded(frame).await
    }
}

impl<'d, T, D> Stm32BidirTxController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    pub fn tx_only(
        timer: Peri<'d, T>,
        tx_pin: DshotTxPin<'d, T>,
        tx_dma: Peri<'d, D>,
        tx_dma_irq: impl Binding<D::Interrupt, DmaInterruptHandler<D>> + 'd,
        speed: DshotSpeed,
    ) -> Self {
        Self {
            tx: TxBackend::new(
                timer,
                tx_pin,
                tx_dma,
                tx_dma_irq,
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
        arm_tx_backend_for(
            &mut self.tx,
            self.speed,
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
        send_command_with_policy(&mut self.tx, BidirTx::command(command).encode(), command).await
    }

    pub async fn send_frame(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        self.tx.send_encoded(frame).await
    }
}

impl<'d, T, D> Stm32BidirController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    pub fn bidirectional(
        timer: Peri<'d, T>,
        tx_pin: DshotTxPin<'d, T>,
        tx_dma: Peri<'d, D>,
        tx_dma_irq: impl Binding<D::Interrupt, DmaInterruptHandler<D>> + 'd,
        rx_cfg: Stm32BidirCapture,
        speed: DshotSpeed,
    ) -> Result<Self, Stm32ConfigError> {
        Ok(Self {
            tx: TxBackend::new(
                timer,
                tx_pin,
                tx_dma,
                tx_dma_irq,
                speed,
                OutputPolarity::ActiveLow,
                TxIdleMode::Input(rx_cfg.pull),
            )
            .with_timeouts(rx_cfg.timeouts),
            rx: RxBackend::new(rx_cfg)?,
            speed,
        })
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), Stm32RuntimeError> {
        arm_tx_backend_for(
            &mut self.tx,
            self.speed,
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
        send_command_with_policy(&mut self.tx, BidirTx::command(command).encode(), command).await
    }

    pub async fn send_command_and_receive(
        &mut self,
        command: Command,
    ) -> Result<TelemetryFrame, Stm32RuntimeError> {
        self.send_command(command).await?;
        self.capture().await
    }

    pub async fn send_frame(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        self.tx.send_encoded(frame).await
    }

    pub async fn send_throttle_and_receive(
        &mut self,
        throttle: u16,
    ) -> Result<TelemetryFrame, Stm32RuntimeError> {
        self.send_frame(BidirTx::throttle_clamped(throttle).encode())
            .await?;
        self.capture().await
    }

    async fn capture(&mut self) -> Result<TelemetryFrame, Stm32RuntimeError> {
        self.rx
            .capture(
                &self.tx.timer,
                &mut self.tx.tx_pin,
                &mut self.tx.tx_dma,
                self.tx.tx_dma_request,
                self.speed,
            )
            .await
    }
}

fn configure_tx_timer<T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    speed: DshotSpeed,
    output_polarity: OutputPolarity,
) -> u16 {
    timer.stop();
    timer.enable_update_dma(false);
    timer.reset();
    timer.set_output_compare_mode(channel, OutputCompareMode::PwmMode1);
    timer.set_output_compare_preload(channel, true);
    timer.set_output_polarity(channel, output_polarity);
    timer.enable_channel(channel, true);
    timer.set_frequency(
        Hertz(speed.timing_hints().nominal_bitrate_hz),
        RoundTo::Faster,
    );
    timer.set_compare_value(channel, 0u16.into());
    timer.enable_outputs();
    timer.generate_update_event();
    timer.enable_update_dma(true);

    let max_compare_value: u32 = timer.get_max_compare_value().into();
    match u16::try_from(max_compare_value) {
        Ok(value) => value.saturating_add(1),
        Err(_) => u16::MAX,
    }
}

fn start_timer<T: GeneralInstance4Channel>(timer: &Timer<'_, T>) {
    timer.start();
}

fn stop_tx_timer<T: GeneralInstance4Channel>(timer: &Timer<'_, T>, channel: Channel) {
    timer.enable_update_dma(false);
    timer.set_compare_value(channel, 0u16.into());
    timer.stop();
    timer.enable_channel(channel, false);
}

fn bit_period_duration(speed: DshotSpeed) -> Duration {
    let bitrate_hz = u64::from(speed.timing_hints().nominal_bitrate_hz);
    Duration::from_micros(1_000_000_u64.div_ceil(bitrate_hz))
}

struct RxSampleTimerGuard<'a, T>
where
    T: GeneralInstance4Channel,
{
    timer: &'a Timer<'a, T>,
}

impl<'a, T> RxSampleTimerGuard<'a, T>
where
    T: GeneralInstance4Channel,
{
    fn new(timer: &'a Timer<'a, T>, sample_hz: u32) -> Self {
        timer.stop();
        timer.enable_update_dma(false);
        timer.reset();
        timer.set_frequency(Hertz(sample_hz), RoundTo::Faster);
        timer.generate_update_event();
        timer.enable_update_dma(true);

        Self { timer }
    }

    fn start(&self) {
        self.timer.start();
    }
}

impl<T> Drop for RxSampleTimerGuard<'_, T>
where
    T: GeneralInstance4Channel,
{
    fn drop(&mut self) {
        self.timer.stop();
        self.timer.enable_update_dma(false);
    }
}

async fn capture_port_samples<T>(
    timer: &Timer<'_, T>,
    tx_pin: &DshotTxPin<'_, T>,
    dma: &mut DmaChannel<'_>,
    dma_request: embassy_stm32_hal::dma::Request,
    sample_hz: u32,
    buffer: &mut [u32],
    timeout: Duration,
) -> Result<(), Stm32RuntimeError>
where
    T: GeneralInstance4Channel,
{
    let timer_guard = RxSampleTimerGuard::new(timer, sample_hz);

    let options = TransferOptions::default();
    let rx_result = unsafe {
        // SAFETY: DMA reads from the GPIO IDR register for the lifetime of `tx_pin`, writes into
        // `buffer` for the duration of the transfer, and `dma` is exclusively borrowed here.
        // Embassy's DMA API takes a `*mut` peripheral address for `read_raw`, even though GPIO IDR
        // is a read-only register and this transfer only reads from it.
        let transfer = dma.read_raw(
            dma_request,
            tx_pin.idr_ptr() as *mut u32,
            buffer as *mut [u32],
            options,
        );
        timer_guard.start();
        with_timeout(timeout, transfer).await
    };

    match rx_result {
        Ok(()) => Ok(()),
        Err(_) => Err(Stm32RuntimeError::RxTimeout),
    }
}

async fn send_command_with_policy<T, D>(
    tx: &mut TxBackend<'_, T, D>,
    frame: EncodedFrame,
    command: Command,
) -> Result<EncodedFrame, Stm32RuntimeError>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    let policy = command.exec_policy();
    let mut last_frame = frame;

    for _ in 0..policy.repeat_count().get() {
        last_frame = tx.send_encoded(frame).await?;
    }

    let min_gap = policy.min_gap();
    if !min_gap.is_zero() {
        EmbassyTimer::after(Duration::from_micros(min_gap.as_micros() as u64)).await;
    }

    Ok(last_frame)
}

async fn arm_tx_backend_for<T, D>(
    tx: &mut TxBackend<'_, T, D>,
    speed: DshotSpeed,
    duration: Duration,
    frame: EncodedFrame,
) -> Result<(), Stm32RuntimeError>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    let frame_period = Duration::from_micros(speed.timing_hints().min_frame_period_us as u64);
    let deadline = Instant::now() + duration;
    let mut next_frame_at = Instant::now();

    while Instant::now() < deadline {
        tx.send_encoded(frame).await?;
        next_frame_at += frame_period;
        let now = Instant::now();
        if now < next_frame_at {
            EmbassyTimer::at(next_frame_at).await;
        } else {
            next_frame_at = now;
        }
    }

    Ok(())
}
