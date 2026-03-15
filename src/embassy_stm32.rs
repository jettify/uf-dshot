//! Experimental Embassy STM32 runtime API for single-line DShot.

use core::marker::PhantomData;

use embassy_stm32_hal::dma::{Transfer, TransferOptions};
use embassy_stm32_hal::gpio::{AfType, AnyPin, Flex, OutputType, Pull, Speed};
use embassy_stm32_hal::time::Hertz;
use embassy_stm32_hal::timer::low_level::{OutputCompareMode, OutputPolarity, Timer};
use embassy_stm32_hal::timer::{
    BasicInstance, BasicNoCr2Instance, Ch1, Ch2, Ch3, Ch4, Channel, GeneralInstance1Channel,
    GeneralInstance4Channel, TimerChannel, TimerPin, UpDma,
};
use embassy_stm32_hal::Peri;
use embassy_time::{with_timeout, Duration, Timer as EmbassyTimer};

use crate::command::{BidirTx, Command, DshotSpeed, EncodedFrame, UniTx, WaveformTiming};
use crate::telemetry::{
    BidirDecoder, DecodeHint, OversamplingConfig, PreambleTuningConfig, TelemetryFrame,
    TelemetryPipelineError,
};

const TX_BUFFER_SLOTS: usize = 17;
const MAX_CAPTURE_SAMPLES: usize = 96;
const PREAMBLE_MARGIN_SAMPLES: usize = 6;
const DEFAULT_ARM_DURATION: Duration = Duration::from_secs(1);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RuntimeTimeouts {
    pub tx: Duration,
    pub rx: Duration,
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

pub struct DshotTxPin<'d, T: GeneralInstance4Channel> {
    line: Flex<'d>,
    af_num: u8,
    pin_port: u8,
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
        let pin_port = pin.port() * 16 + pin.pin();

        Self {
            line: Flex::new(pin),
            af_num,
            pin_port,
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

    fn enter_tx(&mut self) {
        self.line.set_as_af_unchecked(
            self.af_num,
            AfType::output(OutputType::PushPull, Speed::VeryHigh),
        );
    }

    fn enter_rx_pullup(&mut self, pull: Pull) {
        self.line.set_as_input(pull);
    }

    fn idr_addr(&self) -> *mut u32 {
        // SAFETY: the controller owns this GPIO line for its full lifetime.
        unsafe { AnyPin::steal(self.pin_port).block().idr().as_ptr() as *mut u32 }
    }
}

pub struct Stm32BidirCapture<'d, T, D>
where
    T: BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    sample_timer: Peri<'d, T>,
    sample_dma: Peri<'d, D>,
    oversampling: OversamplingConfig,
    sample_count: usize,
    decode_hint: DecodeHint,
    pull: Pull,
    preamble_tuning: PreambleTuningConfig,
    timeouts: RuntimeTimeouts,
}

impl<'d, T, D> Stm32BidirCapture<'d, T, D>
where
    T: BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    pub fn new(
        sample_timer: Peri<'d, T>,
        sample_dma: Peri<'d, D>,
        oversampling: OversamplingConfig,
    ) -> Self {
        let sample_count = oversampling.frame_bits as usize * oversampling.oversampling as usize
            + PREAMBLE_MARGIN_SAMPLES;

        Self {
            sample_timer,
            sample_dma,
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
    tx_dma: Peri<'d, D>,
    waveform_timing: WaveformTiming,
    timeouts: RuntimeTimeouts,
    duty_buffer: [u16; TX_BUFFER_SLOTS],
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
        speed: DshotSpeed,
    ) -> Self {
        let timer = Timer::new(timer);
        let period_ticks = setup_dshot_timer(&timer, tx_pin.channel(), speed);
        let waveform_timing = WaveformTiming {
            period_ticks,
            bit0_high_ticks: (period_ticks * 3) / 8,
            bit1_high_ticks: (period_ticks * 3) / 4,
        };

        Self {
            timer,
            tx_pin,
            tx_dma,
            waveform_timing,
            timeouts: RuntimeTimeouts::default(),
            duty_buffer: [0; TX_BUFFER_SLOTS],
        }
    }

    fn with_timeouts(mut self, timeouts: RuntimeTimeouts) -> Self {
        self.timeouts = timeouts;
        self
    }

    async fn send_encoded(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        let waveform = frame.to_waveform_ticks(self.waveform_timing, true);
        self.duty_buffer[..16].copy_from_slice(&waveform.bit_high_ticks);
        self.duty_buffer[16] = waveform.reset_low_ticks.unwrap_or(0);

        self.tx_pin.enter_tx();
        match with_timeout(
            self.timeouts.tx,
            send_waveform(
                &self.timer,
                self.tx_pin.channel(),
                self.tx_dma.reborrow(),
                &self.duty_buffer,
            ),
        )
        .await
        {
            Ok(()) => Ok(frame),
            Err(_) => Err(Stm32RuntimeError::TxTimeout),
        }
    }
}

struct RxBackend<'d, T, D>
where
    T: BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    sample_timer: Timer<'d, T>,
    sample_dma: Peri<'d, D>,
    decoder: BidirDecoder,
    sample_count: usize,
    decode_hint: DecodeHint,
    pull: Pull,
    timeouts: RuntimeTimeouts,
    raw_samples: [u32; MAX_CAPTURE_SAMPLES],
    sample_words: [u16; MAX_CAPTURE_SAMPLES],
}

impl<'d, T, D> RxBackend<'d, T, D>
where
    T: BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    fn new(cfg: Stm32BidirCapture<'d, T, D>) -> Result<Self, Stm32ConfigError> {
        if cfg.sample_count == 0 || cfg.sample_count > MAX_CAPTURE_SAMPLES {
            return Err(Stm32ConfigError::SampleBufferTooSmall {
                requested: cfg.sample_count,
                capacity: MAX_CAPTURE_SAMPLES,
            });
        }

        Ok(Self {
            sample_timer: Timer::new(cfg.sample_timer),
            sample_dma: cfg.sample_dma,
            decoder: BidirDecoder::with_preamble_tuning(cfg.oversampling, cfg.preamble_tuning),
            sample_count: cfg.sample_count,
            decode_hint: cfg.decode_hint,
            pull: cfg.pull,
            timeouts: cfg.timeouts,
            raw_samples: [0; MAX_CAPTURE_SAMPLES],
            sample_words: [0; MAX_CAPTURE_SAMPLES],
        })
    }

    async fn capture<TX>(
        &mut self,
        tx_pin: &mut DshotTxPin<'_, TX>,
        speed: DshotSpeed,
    ) -> Result<TelemetryFrame, Stm32RuntimeError>
    where
        TX: GeneralInstance4Channel,
    {
        tx_pin.enter_rx_pullup(self.pull);
        let sample_hz =
            speed.timing_hints().nominal_bitrate_hz * self.decoder.cfg.oversampling as u32;
        let slice = &mut self.raw_samples[..self.sample_count];

        match with_timeout(
            self.timeouts.rx,
            capture_port_samples(
                &self.sample_timer,
                self.sample_dma.reborrow(),
                tx_pin.idr_addr(),
                sample_hz,
                slice,
            ),
        )
        .await
        {
            Ok(()) => {}
            Err(_) => return Err(Stm32RuntimeError::RxTimeout),
        }

        for (dst, src) in self.sample_words[..self.sample_count]
            .iter_mut()
            .zip(slice.iter())
        {
            *dst = *src as u16;
        }

        self.decoder
            .decode_frame(&self.sample_words[..self.sample_count], self.decode_hint)
            .map_err(Stm32RuntimeError::Telemetry)
    }
}

pub struct Stm32DshotController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    tx: TxBackend<'d, T, D>,
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
        speed: DshotSpeed,
    ) -> Self {
        Self {
            tx: TxBackend::new(timer, tx_pin, tx_dma, speed),
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
            UniTx::throttle_clamped(0).encode(),
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

pub struct Stm32BidirController<'d, TXT, TXD, RXT, RXD>
where
    TXT: GeneralInstance4Channel,
    TXD: UpDma<TXT>,
    RXT: BasicInstance + BasicNoCr2Instance,
    RXD: UpDma<RXT>,
{
    tx: TxBackend<'d, TXT, TXD>,
    rx: RxBackend<'d, RXT, RXD>,
    speed: DshotSpeed,
}

impl<'d, TXT, TXD, RXT, RXD> Stm32BidirController<'d, TXT, TXD, RXT, RXD>
where
    TXT: GeneralInstance4Channel,
    TXD: UpDma<TXT>,
    RXT: BasicInstance + BasicNoCr2Instance,
    RXD: UpDma<RXT>,
{
    pub fn bidirectional(
        timer: Peri<'d, TXT>,
        tx_pin: DshotTxPin<'d, TXT>,
        tx_dma: Peri<'d, TXD>,
        mut rx_cfg: Stm32BidirCapture<'d, RXT, RXD>,
        speed: DshotSpeed,
    ) -> Result<Self, Stm32ConfigError> {
        rx_cfg.oversampling.sample_bit_index = tx_pin.pin_index();

        Ok(Self {
            tx: TxBackend::new(timer, tx_pin, tx_dma, speed).with_timeouts(rx_cfg.timeouts),
            rx: RxBackend::new(rx_cfg)?,
            speed,
        })
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), Stm32RuntimeError> {
        arm_tx_backend_for(
            &mut self.tx,
            self.speed,
            duration,
            BidirTx::throttle_clamped(0).encode(),
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

    pub async fn send_throttle_and_receive(
        &mut self,
        throttle: u16,
    ) -> Result<TelemetryFrame, Stm32RuntimeError> {
        self.send_frame(BidirTx::throttle_clamped(throttle).encode())
            .await?;
        self.rx.capture(&mut self.tx.tx_pin, self.speed).await
    }
}

fn setup_dshot_timer<T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    speed: DshotSpeed,
) -> u16 {
    timer.stop();
    timer.set_output_compare_mode(channel, OutputCompareMode::PwmMode1);
    timer.set_output_compare_preload(channel, true);
    timer.set_output_polarity(channel, OutputPolarity::ActiveHigh);
    timer.enable_channel(channel, true);
    timer.set_frequency(Hertz(speed.timing_hints().nominal_bitrate_hz));
    timer.set_compare_value(channel, 0);
    timer.enable_outputs();
    timer.start();

    (timer.get_max_compare_value() as u16).saturating_add(1)
}

async fn send_waveform<T, D>(timer: &Timer<'_, T>, channel: Channel, dma: Peri<'_, D>, duty: &[u16])
where
    T: GeneralInstance1Channel + BasicNoCr2Instance + BasicInstance,
    D: UpDma<T>,
{
    let original_update_dma_state = timer.get_update_dma_state();
    if !original_update_dma_state {
        timer.enable_update_dma(true);
    }

    let req = dma.request();
    let ccr = timer.regs_1ch().ccr(channel.index()).as_ptr() as *mut u16;
    let mut options = TransferOptions::default();
    options.fifo_threshold = Some(embassy_stm32_hal::dma::FifoThreshold::Full);
    options.mburst = embassy_stm32_hal::dma::Burst::Incr8;

    // SAFETY: DMA writes the provided waveform buffer into the timer CCR register while
    // both the timer peripheral and the buffer remain alive for the transfer duration.
    unsafe {
        Transfer::new_write(dma, req, duty, ccr, options).await;
    }

    if !original_update_dma_state {
        timer.enable_update_dma(false);
    }
}

async fn capture_port_samples<T, D>(
    timer: &Timer<'_, T>,
    dma: Peri<'_, D>,
    idr_addr: *mut u32,
    sample_hz: u32,
    buffer: &mut [u32],
) where
    T: BasicInstance + BasicNoCr2Instance,
    D: UpDma<T>,
{
    timer.stop();
    timer.set_frequency(Hertz(sample_hz));
    timer.enable_update_dma(true);
    timer.start();

    let req = dma.request();
    let options = TransferOptions::default();

    // SAFETY: DMA reads a fixed GPIO IDR register into an exclusive caller-owned sample buffer.
    unsafe {
        Transfer::new_read(dma, req, idr_addr, buffer, options).await;
    }

    timer.stop();
    timer.enable_update_dma(false);
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
    let total_us = duration.as_micros() as u64;
    let frame_period_us = frame_period.as_micros() as u64;
    let frames = if total_us == 0 {
        0
    } else {
        total_us.div_ceil(frame_period_us)
    };

    for _ in 0..frames {
        tx.send_encoded(frame).await?;
        EmbassyTimer::after(frame_period).await;
    }

    Ok(())
}
