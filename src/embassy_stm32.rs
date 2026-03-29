//! Experimental Embassy STM32 runtime API for single-line DShot.

#[path = "embassy_stm32_bf_compat.rs"]
pub mod bf_port;
#[path = "embassy_stm32_bidir_capture.rs"]
pub mod bidir_capture;

use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU8, Ordering};

use embassy_stm32_hal::dma::{
    Channel as DmaChannel, ChannelInstance as DmaChannelInstance,
    InterruptHandler as EmbassyDmaInterruptHandler, Request, TransferOptions,
};
use embassy_stm32_hal::gpio::{AfType, AnyPin, Flex, OutputType, Pull, Speed};
use embassy_stm32_hal::interrupt::typelevel::{Binding, Handler};
use embassy_stm32_hal::pac;
use embassy_stm32_hal::time::Hertz;
use embassy_stm32_hal::timer::low_level::{OutputCompareMode, OutputPolarity, RoundTo, Timer};
use embassy_stm32_hal::timer::{
    Ch1, Ch2, Ch3, Ch4, Channel, GeneralInstance4Channel, TimerChannel, TimerPin, UpDma,
};
use embassy_stm32_hal::Peri;
use embassy_time::{with_timeout, Duration, Instant, Timer as EmbassyTimer};

#[cfg(not(feature = "defmt"))]
use crate::bidir_capture::decode_frame_bf_port_samples_u16;
#[cfg(feature = "defmt")]
use crate::bidir_capture::{decode_bf_raw_21, decode_frame_bf_port_samples_with_debug_u16};
use crate::command::{BidirTx, Command, DshotSpeed, EncodedFrame, UniTx, WaveformTiming};
use crate::telemetry::{
    BidirDecoder, DecodeHint, GcrFrame, OversamplingConfig, PreambleTuningConfig, TelemetryFrame,
    TelemetryPipelineError,
};

const TX_BUFFER_SLOTS: usize = 17;
const DIRECT_TX_HOLD_SLOTS: usize = 1;
const DIRECT_TX_STATE_SLOTS: usize = 16 * 3 + DIRECT_TX_HOLD_SLOTS;
const MAX_CAPTURE_SAMPLES: usize = 512;
const PINS_PER_GPIO_PORT: u8 = 16;
const DMA_IRQ_SLOTS: usize = 16;
// The ESC turnaround before bidirectional telemetry can exceed 40 us on AM32 setups.
// Keep enough leading idle-high samples so the default capture window still contains
// the full 21-bit telemetry frame at DShot300 with 3x oversampling.
const PREAMBLE_MARGIN_SAMPLES: usize = 64;
#[cfg(feature = "defmt")]
const RX_PROGRESS_PROBE_DELAY_US: u64 = 10;
#[cfg(feature = "defmt")]
const RX_DEBUG_LOG_EVERY_ATTEMPTS: u32 = 2048;
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
    bsrr_ptr: *mut u32,
    idr_ptr: *mut u16,
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
        let regs = unsafe { AnyPin::steal(pin_port as u8) }.block();
        let bsrr_ptr = regs.bsrr().as_ptr() as *mut u32;
        let idr_ptr = regs.idr().as_ptr() as *mut u16;

        Self {
            line: Flex::new(pin),
            af_num,
            pin_mask,
            bsrr_ptr,
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

    fn enter_output_high(&mut self) {
        self.line.set_high();
        self.line.set_as_output(Speed::VeryHigh);
    }

    fn bsrr_ptr(&self) -> *mut u32 {
        self.bsrr_ptr
    }

    fn idr_ptr(&self) -> *const u32 {
        self.idr_ptr as *const u32
    }

    fn idr_ptr_u16(&self) -> *mut u16 {
        self.idr_ptr
    }

    fn pin_mask(&self) -> u32 {
        self.pin_mask
    }

    fn normalize_sample(&self, sample: u32) -> u16 {
        if (sample & self.pin_mask) != 0 {
            1
        } else {
            0
        }
    }
}

#[derive(Clone, Copy)]
pub struct Stm32BidirCapture {
    oversampling: OversamplingConfig,
    sample_count: usize,
    decode_hint: DecodeHint,
    pull: Pull,
    pacer_compare_percent: u8,
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
            pacer_compare_percent: 50,
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

    pub fn with_pacer_compare_percent(mut self, percent: u8) -> Self {
        self.pacer_compare_percent = percent.clamp(1, 99);
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
        tx_dma_irq: impl Binding<D::Interrupt, EmbassyDmaInterruptHandler<D>> + 'd,
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
            Ok(()) => Ok(frame),
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
    raw_samples: [u16; MAX_CAPTURE_SAMPLES],
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
        #[cfg(feature = "defmt")]
        let log_debug =
            self.capture_attempts <= 4 || self.capture_attempts % RX_DEBUG_LOG_EVERY_ATTEMPTS == 0;

        tx_pin.enter_rx_pullup(self.pull);
        let sample_hz =
            speed.timing_hints().nominal_bitrate_hz * 5 * self.decoder.cfg.oversampling as u32 / 4;
        let slice = &mut self.raw_samples[..self.sample_count];

        match capture_port_samples(
            timer,
            tx_pin,
            dma,
            dma_request,
            sample_hz,
            slice,
            self.timeouts.rx,
            #[cfg(feature = "defmt")]
            log_debug,
        )
        .await
        {
            Ok(()) => {}
            Err(Stm32RuntimeError::RxTimeout) => {
                #[cfg(feature = "defmt")]
                {
                    self.timeout_count = self.timeout_count.wrapping_add(1);
                    if self.timeout_count <= 4 || self.timeout_count % 1000 == 0 {
                        let summary = summarize_samples(slice, tx_pin.pin_mask() as u16);
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
                            slice.first().copied().unwrap_or(0) as u32,
                            slice.get(1).copied().unwrap_or(0) as u32,
                            slice.get(2).copied().unwrap_or(0) as u32,
                            slice.get(3).copied().unwrap_or(0) as u32,
                        );
                    }
                }
                return Err(Stm32RuntimeError::RxTimeout);
            }
            Err(err) => return Err(err),
        }

        #[cfg(feature = "defmt")]
        {
            let outcome = decode_frame_bf_port_samples_with_debug_u16(
                &mut self.decoder,
                &self.raw_samples[..self.sample_count],
                tx_pin.pin_mask() as u16,
            );
            if let Err(err) = outcome.frame {
                self.decode_error_count = self.decode_error_count.wrapping_add(1);
                if self.decode_error_count <= 4 || self.decode_error_count % 1000 == 0 {
                    let summary = summarize_samples(
                        &self.raw_samples[..self.sample_count],
                        tx_pin.pin_mask() as u16,
                    );
                    let stats = self.decoder.stats();
                    let (bf_raw_valid, bf_payload) = if outcome.debug.raw_21 != 0 {
                        match decode_bf_raw_21(&self.decoder, outcome.debug.raw_21) {
                            Ok(payload) => (true, payload.raw_16 as u32),
                            Err(_) => (false, 0),
                        }
                    } else {
                        (false, 0)
                    };
                    defmt::warn!(
                        "bdshot decode err count={} err={} highs={} lows={} edges={} first_edge={:?} last_edge={:?} hint_skip={} stats ok={} no_edge={} short={} invalid_frame={} invalid_gcr={} invalid_crc={} start_margin={} bf_start={} bf_end={} bf_bits={} bf_raw=0x{:06x} bf_raw_valid={} bf_payload=0x{:04x} head={},{},{},{}",
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
                        outcome.debug.start_margin,
                        outcome.debug.frame_end,
                        outcome.debug.bits_found,
                        outcome.debug.raw_21,
                        bf_raw_valid,
                        bf_payload,
                        self.raw_samples.first().copied().unwrap_or(0),
                        self.raw_samples.get(1).copied().unwrap_or(0),
                        self.raw_samples.get(2).copied().unwrap_or(0),
                        self.raw_samples.get(3).copied().unwrap_or(0),
                    );
                }
                Err(Stm32RuntimeError::Telemetry(err))
            } else {
                outcome.frame.map_err(Stm32RuntimeError::Telemetry)
            }
        }

        #[cfg(not(feature = "defmt"))]
        {
            decode_frame_bf_port_samples_u16(
                &mut self.decoder,
                &self.raw_samples[..self.sample_count],
                tx_pin.pin_mask() as u16,
            )
            .map_err(Stm32RuntimeError::Telemetry)
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
fn summarize_samples(samples: &[u16], mask: u16) -> SampleSummary {
    summarize_levels(samples.iter().copied().map(|v| (v & mask) != 0))
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
    #[cfg(feature = "defmt")]
    last_tx_completed_at: Instant,
    #[cfg(feature = "defmt")]
    last_tx_to_capture_start_us: u32,
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
        tx_dma_irq: impl Binding<D::Interrupt, EmbassyDmaInterruptHandler<D>> + 'd,
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
        tx_dma_irq: impl Binding<D::Interrupt, EmbassyDmaInterruptHandler<D>> + 'd,
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
        tx_dma_irq: impl Binding<D::Interrupt, EmbassyDmaInterruptHandler<D>> + 'd,
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
            #[cfg(feature = "defmt")]
            last_tx_completed_at: Instant::from_ticks(0),
            #[cfg(feature = "defmt")]
            last_tx_to_capture_start_us: 0,
        })
    }

    pub fn with_timeouts(mut self, timeouts: RuntimeTimeouts) -> Self {
        self.tx = self.tx.with_timeouts(timeouts);
        self.rx.timeouts = timeouts;
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

    pub async fn send_command_and_receive(
        &mut self,
        command: Command,
    ) -> Result<TelemetryFrame, Stm32RuntimeError> {
        self.send_command(command).await?;
        #[cfg(feature = "defmt")]
        {
            self.last_tx_completed_at = Instant::now();
        }
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
        #[cfg(feature = "defmt")]
        {
            self.last_tx_completed_at = Instant::now();
        }
        self.capture().await
    }

    async fn capture(&mut self) -> Result<TelemetryFrame, Stm32RuntimeError> {
        #[cfg(feature = "defmt")]
        let capture_started_at = Instant::now();
        #[cfg(feature = "defmt")]
        {
            self.last_tx_to_capture_start_us = capture_started_at
                .saturating_duration_since(self.last_tx_completed_at)
                .as_micros() as u32;
            if self.rx.capture_attempts < 8 || self.rx.capture_attempts % 256 == 0 {
                defmt::debug!(
                    "bdshot direct capture start tx_to_capture_start_us={}",
                    self.last_tx_to_capture_start_us,
                );
            }
        }
        self.rx
            .capture(
                &self.tx.timer,
                &mut self.tx.tx_pin,
                &mut self.tx.tx_dma,
                self.tx.tx_dma_request,
                self.speed,
            )
            .await
            .map_err(|err| {
                #[cfg(feature = "defmt")]
                {
                    let capture_call_us = Instant::now()
                        .saturating_duration_since(capture_started_at)
                        .as_micros() as u32;
                    if self.rx.capture_attempts <= 8 || self.rx.capture_attempts % 256 == 0 {
                        defmt::debug!(
                            "bdshot direct capture err tx_to_capture_start_us={} capture_call_us={} err={}",
                            self.last_tx_to_capture_start_us,
                            capture_call_us,
                            err,
                        );
                    }
                }
                err
            })
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
        let _ = timer.clear_update_interrupt();
        timer.set_frequency(Hertz(sample_hz), RoundTo::Faster);
        // `set_frequency` already emits an update event to latch PSC/ARR. Clear the
        // resulting pending update state before arming DMA so RX capture starts from
        // timer-paced requests instead of immediately consuming buffered software UGs.
        let _ = timer.clear_update_interrupt();

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
    buffer: &mut [u16],
    timeout: Duration,
    #[cfg(feature = "defmt")] log_debug: bool,
) -> Result<(), Stm32RuntimeError>
where
    T: GeneralInstance4Channel,
{
    let timer_guard = RxSampleTimerGuard::new(timer, sample_hz);

    let options = TransferOptions::default();
    #[cfg(feature = "defmt")]
    let cnt_before_start = timer.regs_gp16().cnt().read().cnt();
    #[cfg(feature = "defmt")]
    let psc = u32::from(timer.regs_gp16().psc().read());
    #[cfg(feature = "defmt")]
    let arr = u32::from(timer.regs_gp16().arr().read().arr());
    #[cfg(feature = "defmt")]
    let timer_hz = timer.get_clock_frequency().0;

    #[cfg(feature = "defmt")]
    if log_debug {
        let dier = timer.regs_gp16().dier().read().0;
        let smcr = timer.regs_gp16().smcr().read().0;
        let cr1 = timer.regs_gp16().cr1().read().0;
        let cr2 = timer.regs_gp16().cr2().read().0;
        let sr = timer.regs_gp16().sr().read().0;

        defmt::debug!(
            "bdshot rx pre req={} sample_hz={} count={} cnt={} psc={} arr={} timer_hz={} dier=0x{:04x} smcr=0x{:04x} cr1=0x{:04x} cr2=0x{:04x} sr=0x{:04x}",
            dma_request,
            sample_hz,
            buffer.len(),
            cnt_before_start,
            psc,
            arr,
            timer_hz,
            dier,
            smcr,
            cr1,
            cr2,
            sr,
        );
    }

    let mut transfer = unsafe {
        // SAFETY: DMA reads from the GPIO IDR register for the lifetime of `tx_pin`, writes into
        // `buffer` for the duration of the transfer, and `dma` is exclusively borrowed here.
        // Embassy's DMA API takes a `*mut` peripheral address for `read_raw`, even though GPIO IDR
        // is a read-only register and this transfer only reads from it.
        dma.read_raw(
            dma_request,
            tx_pin.idr_ptr_u16(),
            buffer as *mut [u16],
            options,
        )
    };

    #[cfg(feature = "defmt")]
    if log_debug {
        let running = transfer.is_running();
        let remaining = transfer.get_remaining_transfers();
        let cnt = timer.regs_gp16().cnt().read().cnt();
        let dier = timer.regs_gp16().dier().read().0;
        let cr1 = timer.regs_gp16().cr1().read().0;
        let sr = timer.regs_gp16().sr().read().0;
        defmt::debug!(
            "bdshot rx armed req={} running={} remaining={} cnt={} dier=0x{:04x} cr1=0x{:04x} sr=0x{:04x}",
            dma_request,
            running,
            remaining,
            cnt,
            dier,
            cr1,
            sr,
        );
    }

    let _ = timer.clear_update_interrupt();
    timer_guard.start();
    timer.enable_update_dma(true);
    let _ = timer.clear_update_interrupt();

    #[cfg(feature = "defmt")]
    if log_debug {
        let ude_after_enable = timer.get_update_dma_state();
        let dier = timer.regs_gp16().dier().read().0;
        let smcr = timer.regs_gp16().smcr().read().0;
        let cr1 = timer.regs_gp16().cr1().read().0;
        let cr2 = timer.regs_gp16().cr2().read().0;
        let sr = timer.regs_gp16().sr().read().0;

        defmt::debug!(
            "bdshot rx arm req={} sample_hz={} count={} timeout_us={} ude={} dier=0x{:04x} cnt={} psc={} arr={} timer_hz={} smcr=0x{:04x} cr1=0x{:04x} cr2=0x{:04x} sr=0x{:04x}",
            dma_request,
            sample_hz,
            buffer.len(),
            timeout.as_micros() as u64,
            ude_after_enable,
            dier,
            cnt_before_start,
            psc,
            arr,
            timer_hz,
            smcr,
            cr1,
            cr2,
            sr,
        );
    }

    #[cfg(feature = "defmt")]
    if log_debug {
        let cnt_after_start = timer.regs_gp16().cnt().read().cnt();
        let remaining = transfer.get_remaining_transfers();
        let head0 = buffer.get(0).copied().unwrap_or(0);
        let head1 = buffer.get(1).copied().unwrap_or(0);
        let head2 = buffer.get(2).copied().unwrap_or(0);
        let head3 = buffer.get(3).copied().unwrap_or(0);
        defmt::debug!(
            "bdshot rx start req={} ude={} cnt={} remaining={} head=0x{:08x},0x{:08x},0x{:08x},0x{:08x}",
            dma_request,
            timer.get_update_dma_state(),
            cnt_after_start,
            remaining,
            head0 as u32,
            head1 as u32,
            head2 as u32,
            head3 as u32,
        );

        EmbassyTimer::after(Duration::from_micros(RX_PROGRESS_PROBE_DELAY_US)).await;

        let running = transfer.is_running();
        let remaining = transfer.get_remaining_transfers();
        let transferred = buffer.len().saturating_sub(remaining as usize);
        let cnt_probe = timer.regs_gp16().cnt().read().cnt();
        defmt::debug!(
            "bdshot rx probe delay_us={} req={} running={} remaining={} transferred={} ude={} cnt={}",
            RX_PROGRESS_PROBE_DELAY_US,
            dma_request,
            running,
            remaining,
            transferred,
            timer.get_update_dma_state(),
            cnt_probe,
        );
    }

    let rx_result = with_timeout(timeout, &mut transfer).await;

    match rx_result {
        Ok(()) => {
            #[cfg(feature = "defmt")]
            if log_debug {
                let remaining = transfer.get_remaining_transfers();
                let changed_samples = buffer
                    .iter()
                    .skip(1)
                    .filter(|&&sample| sample != buffer[0])
                    .count();
                let summary = summarize_samples(buffer, tx_pin.pin_mask() as u16);
                defmt::debug!(
                    "bdshot rx done req={} remaining={} changed={} bit_highs={} bit_lows={} bit_edges={} first=0x{:08x} last=0x{:08x}",
                    dma_request,
                    remaining,
                    changed_samples,
                    summary.high_count,
                    summary.low_count,
                    summary.edge_count,
                    buffer.first().copied().unwrap_or(0) as u32,
                    buffer.last().copied().unwrap_or(0) as u32,
                );
            }
            Ok(())
        }
        Err(_) => {
            #[cfg(feature = "defmt")]
            {
                let running = transfer.is_running();
                let remaining = transfer.get_remaining_transfers();
                let changed_samples = buffer
                    .iter()
                    .skip(1)
                    .filter(|&&sample| sample != buffer[0])
                    .count();
                defmt::warn!(
                    "bdshot rx timeout req={} running={} remaining={} changed={} first=0x{:08x} last=0x{:08x} psc={} arr={} timer_hz={}",
                    dma_request,
                    running,
                    remaining,
                    changed_samples,
                    buffer.first().copied().unwrap_or(0) as u32,
                    buffer.last().copied().unwrap_or(0) as u32,
                    psc,
                    arr,
                    timer_hz,
                );
            }
            Err(Stm32RuntimeError::RxTimeout)
        }
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
