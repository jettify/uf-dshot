use core::marker::PhantomData;

use embassy_stm32_hal::dma::{
    Channel as DmaChannel, InterruptHandler as DmaInterruptHandler, TransferOptions,
};
use embassy_stm32_hal::gpio::{AfType, Flex, OutputType, Pull, Speed};
use embassy_stm32_hal::interrupt::typelevel::Binding;
use embassy_stm32_hal::time::Hertz;
use embassy_stm32_hal::timer::input_capture::InputCapture;
use embassy_stm32_hal::timer::low_level::{
    CountingMode, OutputCompareMode, OutputPolarity, RoundTo, Timer,
};
use embassy_stm32_hal::timer::{
    CaptureCompareInterruptHandler, Ch1, Ch2, Ch3, Ch4, Channel, GeneralInstance4Channel,
    TimerChannel, TimerPin, UpDma,
};
use embassy_stm32_hal::Peri;
use embassy_time::{with_timeout, Duration, Instant, Timer as EmbassyTimer};

use crate::command::{BidirTx, Command, DshotSpeed, EncodedFrame, UniTx, WaveformTiming};
use crate::telemetry::{
    BidirDecoder, OversamplingConfig, PreambleTuningConfig, TelemetryFrame, TelemetryPipelineError,
};

const TX_BUFFER_SLOTS: usize = 17;
const MAX_CAPTURE_EDGES: usize = 32;
const DEFAULT_ARM_DURATION: Duration = Duration::from_millis(3_000);

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

        Self {
            line: Flex::new(pin),
            af_num,
            channel,
            _timer: PhantomData,
        }
    }

    pub fn channel(&self) -> Channel {
        self.channel
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

    fn enter_capture(&mut self, pull: Pull) {
        self.line
            .set_as_af_unchecked(self.af_num, AfType::input(pull));
    }
}

pub struct Stm32BidirCapture {
    oversampling: OversamplingConfig,
    sample_count: usize,
    pull: Pull,
    preamble_tuning: PreambleTuningConfig,
    timeouts: RuntimeTimeouts,
}

impl Stm32BidirCapture {
    pub fn new(oversampling: OversamplingConfig) -> Self {
        let sample_count = usize::from(oversampling.frame_bits)
            .saturating_add(1)
            .min(MAX_CAPTURE_EDGES);

        Self {
            oversampling,
            sample_count,
            pull: Pull::Up,
            preamble_tuning: PreambleTuningConfig::default(),
            timeouts: RuntimeTimeouts::default(),
        }
    }

    pub fn with_sample_count(mut self, sample_count: usize) -> Self {
        self.sample_count = sample_count;
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
    release_pull: Option<Pull>,
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
        mut tx_pin: DshotTxPin<'d, T>,
        tx_dma: Peri<'d, D>,
        tx_dma_irq: impl Binding<D::Interrupt, DmaInterruptHandler<D>> + 'd,
        speed: DshotSpeed,
        output_polarity: OutputPolarity,
    ) -> Self {
        let timer = Timer::new(timer);
        let tx_dma_request = tx_dma.request();
        let tx_dma = DmaChannel::new(tx_dma, tx_dma_irq);
        tx_pin.enter_tx();
        let period_ticks = setup_dshot_timer(&timer, tx_pin.channel(), speed, output_polarity);
        let waveform_timing = WaveformTiming {
            period_ticks,
            bit0_high_ticks: (period_ticks * 3) / 8,
            bit1_high_ticks: (period_ticks * 3) / 4,
        };

        Self {
            timer,
            tx_pin,
            tx_dma,
            tx_dma_request,
            _tx_dma: PhantomData,
            speed,
            waveform_timing,
            output_polarity,
            timeouts: RuntimeTimeouts::default(),
            release_pull: None,
            line_in_tx_mode: true,
            duty_buffer: [T::Word::from(0u16); TX_BUFFER_SLOTS],
        }
    }

    fn with_timeouts(mut self, timeouts: RuntimeTimeouts) -> Self {
        self.timeouts = timeouts;
        self
    }

    fn with_release_pull(mut self, pull: Pull) -> Self {
        self.release_pull = Some(pull);
        self
    }

    fn prepare_for_tx(&mut self) {
        if !self.line_in_tx_mode {
            self.tx_pin.enter_tx();
            self.line_in_tx_mode = true;
        }

        let channel = self.tx_pin.channel();
        setup_dshot_timer(&self.timer, channel, self.speed, self.output_polarity);
        self.timer.set_compare_value(channel, 0u16.into());
        self.timer.enable_outputs();
        self.timer.enable_channel(channel, true);
    }

    fn recover_from_tx_timeout(&mut self) {
        let channel = self.tx_pin.channel();

        self.timer.enable_update_dma(false);
        self.timer.stop();
        self.timer.set_compare_value(channel, 0u16.into());
        self.timer.enable_channel(channel, false);

        if let Some(pull) = self.release_pull {
            self.tx_pin.enter_rx_pullup(pull);
            self.line_in_tx_mode = false;
        }
    }

    fn release_line_to_idle(&mut self) {
        if let Some(pull) = self.release_pull {
            self.tx_pin.enter_rx_pullup(pull);
            self.line_in_tx_mode = false;
        }
    }

    async fn send_encoded(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        self.send_encoded_with_release(frame, true).await
    }

    async fn send_encoded_with_release(
        &mut self,
        frame: EncodedFrame,
        release_after_send: bool,
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
        let original_update_dma_state = self.timer.get_update_dma_state();
        if !original_update_dma_state {
            self.timer.enable_update_dma(true);
        }

        let ccr = self
            .timer
            .regs_gp16()
            .ccr(self.tx_pin.channel().index())
            .as_ptr() as *mut T::Word;
        let options = TransferOptions::default();
        let duty_buffer = &self.duty_buffer;
        let tx_dma = &mut self.tx_dma;
        let tx_dma_request = self.tx_dma_request;

        // SAFETY: DMA writes the provided waveform buffer into the timer CCR register while
        // both the timer peripheral and the buffer remain alive for the transfer duration.
        let tx_result = unsafe {
            with_timeout(
                self.timeouts.tx,
                tx_dma.write(tx_dma_request, duty_buffer, ccr, options),
            )
            .await
        };

        if !original_update_dma_state {
            self.timer.enable_update_dma(false);
        }

        match tx_result {
            Ok(()) => {
                if release_after_send {
                    if self.release_pull.is_some() {
                        // DMA completion only means the trailing CCR=0 reset slot was loaded into
                        // the timer. Wait one bit period so the timer can finish driving that low
                        // interval on the wire before the line is released for telemetry.
                        EmbassyTimer::after(Duration::from_hz(
                            self.speed.timing_hints().nominal_bitrate_hz as u64,
                        ))
                        .await;
                    }

                    self.release_line_to_idle();
                }

                Ok(frame)
            }
            Err(_) => {
                self.recover_from_tx_timeout();
                Err(Stm32RuntimeError::TxTimeout)
            }
        }
    }
}

struct RxBackend<'d, T>
where
    T: GeneralInstance4Channel,
{
    input_capture: InputCapture<'d, T>,
    decoder: BidirDecoder,
    sample_count: usize,
    pull: Pull,
    timeouts: RuntimeTimeouts,
    capture_buffer: [u32; MAX_CAPTURE_EDGES],
}

impl<'d, T> RxBackend<'d, T>
where
    T: GeneralInstance4Channel,
{
    fn new(
        timer: Peri<'d, T>,
        cfg: Stm32BidirCapture,
        irq: impl Binding<T::CaptureCompareInterrupt, CaptureCompareInterruptHandler<T>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, Stm32ConfigError> {
        if cfg.sample_count == 0 || cfg.sample_count > MAX_CAPTURE_EDGES {
            return Err(Stm32ConfigError::SampleBufferTooSmall {
                requested: cfg.sample_count,
                capacity: MAX_CAPTURE_EDGES,
            });
        }

        let decoder = BidirDecoder::with_preamble_tuning(cfg.oversampling, cfg.preamble_tuning);
        let sample_hz =
            speed.timing_hints().nominal_bitrate_hz * cfg.oversampling.oversampling as u32;

        Ok(Self {
            input_capture: InputCapture::new(
                timer,
                None,
                None,
                None,
                None,
                irq,
                Hertz(sample_hz),
                CountingMode::EdgeAlignedUp,
            ),
            decoder,
            sample_count: cfg.sample_count,
            pull: cfg.pull,
            timeouts: cfg.timeouts,
            capture_buffer: [0; MAX_CAPTURE_EDGES],
        })
    }

    async fn capture(
        &mut self,
        tx_pin: &mut DshotTxPin<'_, T>,
        speed: DshotSpeed,
    ) -> Result<TelemetryFrame, Stm32RuntimeError> {
        tx_pin.enter_capture(self.pull);
        let channel = tx_pin.channel();
        let captures = &mut self.capture_buffer[..self.sample_count];
        let mut captured = 0usize;

        let first_edge = with_timeout(
            self.timeouts.rx,
            self.input_capture.wait_for_any_edge(channel),
        )
        .await
        .map_err(|_| Stm32RuntimeError::RxTimeout)?;
        captures[captured] = first_edge.into();
        captured += 1;

        let inter_edge_timeout =
            Duration::from_micros(speed.timing_hints().min_frame_period_us as u64);
        while captured < captures.len() {
            match with_timeout(
                inter_edge_timeout,
                self.input_capture.wait_for_any_edge(channel),
            )
            .await
            {
                Ok(ts) => {
                    captures[captured] = ts.into();
                    captured += 1;
                }
                Err(_) => break,
            }
        }

        tx_pin.enter_rx_pullup(self.pull);
        self.decoder
            .decode_capture_frame_with_ticks(&captures[..captured], 1)
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
        let t = UniTx::throttle_clamped(throttle).encode();
        self.send_frame(t).await
    }

    pub async fn send_command(
        &mut self,
        command: Command,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        let frame = UniTx::command(command).encode();
        send_command_with_policy(&mut self.tx, frame, command).await
    }

    pub async fn send_frame(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, Stm32RuntimeError> {
        self.tx.send_encoded(frame).await
    }
}

pub struct Stm32BidirController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: UpDma<T>,
{
    tx: TxBackend<'d, T, D>,
    rx: RxBackend<'d, T>,
    speed: DshotSpeed,
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
        irq: impl Binding<T::CaptureCompareInterrupt, CaptureCompareInterruptHandler<T>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, Stm32ConfigError> {
        // SAFETY: TX and RX use the same hardware timer sequentially under one controller.
        let rx_timer = unsafe { timer.clone_unchecked() };

        Ok(Self {
            tx: TxBackend::new(
                timer,
                tx_pin,
                tx_dma,
                tx_dma_irq,
                speed,
                OutputPolarity::ActiveLow,
            )
            .with_timeouts(rx_cfg.timeouts)
            .with_release_pull(rx_cfg.pull),
            rx: RxBackend::new(rx_timer, rx_cfg, irq, speed)?,
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
    output_polarity: OutputPolarity,
) -> u16 {
    timer.stop();
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
    timer.start();

    let max_compare_value: u32 = timer.get_max_compare_value().into();
    match u16::try_from(max_compare_value) {
        Ok(value) => value.saturating_add(1),
        Err(_) => u16::MAX,
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
    let start = Instant::now();
    let deadline = start + duration;
    let mut next_frame_at = start;

    while Instant::now() < deadline {
        tx.send_encoded_with_release(frame, false).await?;
        next_frame_at += frame_period;

        let now = Instant::now();
        if next_frame_at > now {
            EmbassyTimer::at(next_frame_at).await;
        } else {
            next_frame_at = now;
        }
    }

    if tx.release_pull.is_some() {
        EmbassyTimer::after(Duration::from_hz(
            speed.timing_hints().nominal_bitrate_hz as u64,
        ))
        .await;
        tx.release_line_to_idle();
    }

    Ok(())
}
