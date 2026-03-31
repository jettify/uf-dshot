use core::convert::TryInto;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{AtomicPtr, AtomicU8, Ordering};

use embassy_stm32_hal::dma::{
    self, Channel as DmaChannel, ChannelInstance as DmaChannelInstance,
    ReadableRingBuffer as DmaReadableRingBuffer,
};
use embassy_stm32_hal::gpio::{AfType, Flex, OutputType, Pull, Speed};
use embassy_stm32_hal::interrupt::typelevel::{Binding, Handler};
use embassy_stm32_hal::pac;
use embassy_stm32_hal::time::Hertz;
use embassy_stm32_hal::timer::low_level::{
    FilterValue, InputCaptureMode, InputTISelection, OutputCompareMode, OutputPolarity, RoundTo,
    Timer,
};
use embassy_stm32_hal::timer::{
    Ch1, Ch2, Ch3, Ch4, Channel, Dma, GeneralInstance4Channel, TimerChannel, TimerPin, UpDma,
};
use embassy_stm32_hal::Peri;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{with_timeout, Duration, Instant, Timer as EmbassyTimer};

use crate::command::{BidirTx, Command, EncodedFrame, WaveformTiming};
use crate::proto::pwm_capture_bf::{
    decode_frame_from_timestamps, decode_gcr_from_timestamps, DecodeFrameFromTimestampsError,
    PwmCaptureDecodeConfig, PwmCaptureDecodeDebug, PwmCaptureDecodeError, BF_CAPTURE_FILTER,
};
use crate::{DshotSpeed, GcrDecodeResult, TelemetryFrame};

const TX_BUFFER_SLOTS: usize = 17;
const DEFAULT_ARM_DURATION: Duration = Duration::from_millis(3_000);
const TX_DMA_IRQ_SLOTS: usize = 16;

pub const MAX_CAPTURE_EDGES: usize = 32;

type TxDmaIrqFn = unsafe fn(*mut ());

static TX_DMA_IRQ_CTX: [AtomicPtr<()>; TX_DMA_IRQ_SLOTS] =
    [const { AtomicPtr::new(ptr::null_mut()) }; TX_DMA_IRQ_SLOTS];
static TX_DMA_IRQ_FN: [AtomicPtr<()>; TX_DMA_IRQ_SLOTS] =
    [const { AtomicPtr::new(ptr::null_mut()) }; TX_DMA_IRQ_SLOTS];

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum TxRxPhase {
    Idle = 0,
    TxActive = 1,
    RxActive = 2,
    Error = 3,
}

struct TxRxWakerState {
    phase: AtomicU8,
    waker: AtomicWaker,
}

impl TxRxWakerState {
    const fn new() -> Self {
        Self {
            phase: AtomicU8::new(TxRxPhase::Idle as u8),
            waker: AtomicWaker::new(),
        }
    }
}

pub trait RawDmaChannel: DmaChannelInstance {
    const IRQ_SLOT: usize;
    fn regs() -> pac::dma::Dma;
    fn stream_num() -> usize;
}

pub struct TxDmaInterruptHandler<D: RawDmaChannel> {
    _phantom: PhantomData<D>,
}

impl<D: RawDmaChannel> Handler<D::Interrupt> for TxDmaInterruptHandler<D> {
    unsafe fn on_interrupt() {
        embassy_stm32_hal::dma::InterruptHandler::<D>::on_interrupt();

        let ctx = TX_DMA_IRQ_CTX[D::IRQ_SLOT].load(Ordering::Acquire);
        let func = TX_DMA_IRQ_FN[D::IRQ_SLOT].load(Ordering::Acquire);
        if ctx.is_null() || func.is_null() {
            return;
        }

        let func: TxDmaIrqFn = core::mem::transmute(func);
        func(ctx);
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH1 {
    const IRQ_SLOT: usize = 9;
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        1
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH2 {
    const IRQ_SLOT: usize = 10;
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        2
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH3 {
    const IRQ_SLOT: usize = 11;
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        3
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH4 {
    const IRQ_SLOT: usize = 12;
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        4
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH5 {
    const IRQ_SLOT: usize = 13;
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        5
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH6 {
    const IRQ_SLOT: usize = 14;
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        6
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmCaptureRuntimeConfig {
    pub rx_timeout: Duration,
    pub capture_filter: u8,
    pub capture_edge_count: usize,
    pub capture_pull: Pull,
}

impl Default for PwmCaptureRuntimeConfig {
    fn default() -> Self {
        Self {
            rx_timeout: Duration::from_millis(2),
            capture_filter: BF_CAPTURE_FILTER,
            capture_edge_count: MAX_CAPTURE_EDGES,
            capture_pull: Pull::Up,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmCaptureBidirRuntimeConfig {
    pub tx_timeout: Duration,
    pub capture: PwmCaptureRuntimeConfig,
}

impl Default for PwmCaptureBidirRuntimeConfig {
    fn default() -> Self {
        Self {
            tx_timeout: Duration::from_millis(2),
            capture: PwmCaptureRuntimeConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PwmCaptureConfigError {
    InvalidCaptureEdgeCount {
        requested: usize,
        max_supported: usize,
    },
    InvalidTickRate,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PwmCaptureRuntimeError {
    Timeout,
    Capture(PwmCaptureDecodeError),
}

impl From<PwmCaptureDecodeError> for PwmCaptureRuntimeError {
    fn from(value: PwmCaptureDecodeError) -> Self {
        Self::Capture(value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PwmCaptureBidirRuntimeError {
    TxTimeout,
    RxTimeout,
    Capture(PwmCaptureDecodeError),
    Frame(DecodeFrameFromTimestampsError),
}

impl From<PwmCaptureRuntimeError> for PwmCaptureBidirRuntimeError {
    fn from(value: PwmCaptureRuntimeError) -> Self {
        match value {
            PwmCaptureRuntimeError::Timeout => Self::RxTimeout,
            PwmCaptureRuntimeError::Capture(err) => Self::Capture(err),
        }
    }
}

impl From<DecodeFrameFromTimestampsError> for PwmCaptureBidirRuntimeError {
    fn from(value: DecodeFrameFromTimestampsError) -> Self {
        Self::Frame(value)
    }
}

pub struct DshotPwmCapturePin<'d, T: GeneralInstance4Channel> {
    line: Flex<'d>,
    af_num: u8,
    channel: Channel,
    _timer: PhantomData<T>,
}

impl<'d, T: GeneralInstance4Channel> DshotPwmCapturePin<'d, T> {
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
        let mut line = Flex::new(pin);
        line.set_as_input(Pull::Up);

        Self {
            line,
            af_num,
            channel,
            _timer: PhantomData,
        }
    }

    pub fn channel(&self) -> Channel {
        self.channel
    }

    fn enter_tx(&mut self) {
        self.line.set_low();
        self.line.set_as_af_unchecked(
            self.af_num,
            AfType::output(OutputType::PushPull, Speed::VeryHigh),
        );
    }

    fn enter_capture(&mut self, pull: Pull) {
        self.line
            .set_as_af_unchecked(self.af_num, AfType::input(pull));
    }
}

pub struct Stm32PwmCaptureReceiver<'d, T, D, C>
where
    T: GeneralInstance4Channel,
    D: DmaChannelInstance,
    C: TimerChannel,
{
    timer: Timer<'d, T>,
    pin: DshotPwmCapturePin<'d, T>,
    dma: DmaChannel<'d>,
    dma_request: embassy_stm32_hal::dma::Request,
    speed: DshotSpeed,
    runtime_cfg: PwmCaptureRuntimeConfig,
    decode_cfg: PwmCaptureDecodeConfig,
    raw_capture: [u16; MAX_CAPTURE_EDGES],
    timestamps: [u32; MAX_CAPTURE_EDGES],
    captured_edges: usize,
    _dma: PhantomData<D>,
    _channel: PhantomData<C>,
}

impl<'d, T, D, C> Stm32PwmCaptureReceiver<'d, T, D, C>
where
    T: GeneralInstance4Channel,
    D: DmaChannelInstance + Dma<T, C>,
    C: TimerChannel,
{
    fn new_inner(
        timer: Peri<'d, T>,
        pin: DshotPwmCapturePin<'d, T>,
        dma: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, embassy_stm32_hal::dma::InterruptHandler<D>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, PwmCaptureConfigError> {
        let dma_request = dma.request();
        let dma = dma::Channel::new(dma, dma_irq);
        let timer = Timer::new(timer);
        timer.enable_outputs();

        let mut this = Self {
            timer,
            pin,
            dma,
            dma_request,
            speed,
            runtime_cfg: PwmCaptureRuntimeConfig::default(),
            decode_cfg: PwmCaptureDecodeConfig::default(),
            raw_capture: [0; MAX_CAPTURE_EDGES],
            timestamps: [0; MAX_CAPTURE_EDGES],
            captured_edges: 0,
            _dma: PhantomData,
            _channel: PhantomData,
        };
        this.configure_capture_timer()?;
        Ok(this)
    }

    pub fn with_runtime_config(
        mut self,
        runtime_cfg: PwmCaptureRuntimeConfig,
    ) -> Result<Self, PwmCaptureConfigError> {
        self.set_runtime_config(runtime_cfg)?;
        Ok(self)
    }

    pub fn set_runtime_config(
        &mut self,
        runtime_cfg: PwmCaptureRuntimeConfig,
    ) -> Result<(), PwmCaptureConfigError> {
        validate_capture_edge_count(runtime_cfg.capture_edge_count)?;
        self.runtime_cfg = runtime_cfg;
        self.configure_capture_timer()
    }

    pub fn with_decode_config(mut self, decode_cfg: PwmCaptureDecodeConfig) -> Self {
        self.decode_cfg = decode_cfg;
        self
    }

    pub fn decode_config(&self) -> PwmCaptureDecodeConfig {
        self.decode_cfg
    }

    pub fn runtime_config(&self) -> PwmCaptureRuntimeConfig {
        self.runtime_cfg
    }

    pub fn captured_edge_count(&self) -> usize {
        self.captured_edges
    }

    pub fn captured_timestamps(&self) -> &[u32] {
        &self.timestamps[..self.captured_edges]
    }

    pub fn raw_capture_values(&self) -> &[u16] {
        &self.raw_capture[..self.captured_edges]
    }

    unsafe fn register_tx_irq_callback(&mut self) {
        TX_DMA_IRQ_CTX[TxD::IRQ_SLOT].store(self as *mut _ as *mut (), Ordering::Release);
        TX_DMA_IRQ_FN[TxD::IRQ_SLOT].store(Self::tx_dma_irq as *mut (), Ordering::Release);
    }

    fn clear_tx_irq_callback(&mut self) {
        TX_DMA_IRQ_CTX[TxD::IRQ_SLOT].store(ptr::null_mut(), Ordering::Release);
        TX_DMA_IRQ_FN[TxD::IRQ_SLOT].store(ptr::null_mut(), Ordering::Release);
    }

    pub async fn capture_edges(&mut self) -> Result<usize, PwmCaptureRuntimeError> {
        self.pin.enter_capture(self.runtime_cfg.capture_pull);
        capture_edges_inner(
            &mut self.timer,
            self.pin.channel(),
            &mut self.dma,
            self.dma_request,
            &mut self.raw_capture,
            &mut self.timestamps,
            &mut self.captured_edges,
            self.runtime_cfg,
        )
        .await
    }

    pub async fn capture_and_decode_gcr(
        &mut self,
    ) -> Result<(GcrDecodeResult, PwmCaptureDecodeDebug), PwmCaptureRuntimeError> {
        self.capture_edges().await?;
        self.decode_gcr()
    }

    pub async fn capture_and_decode_frame(
        &mut self,
    ) -> Result<Result<TelemetryFrame, DecodeFrameFromTimestampsError>, PwmCaptureRuntimeError>
    {
        self.capture_edges().await?;
        Ok(self.decode_frame())
    }

    pub fn decode_gcr(
        &self,
    ) -> Result<(GcrDecodeResult, PwmCaptureDecodeDebug), PwmCaptureRuntimeError> {
        decode_gcr_from_timestamps(self.captured_timestamps(), self.decode_cfg).map_err(Into::into)
    }

    pub fn decode_frame(&self) -> Result<TelemetryFrame, DecodeFrameFromTimestampsError> {
        decode_frame_from_timestamps(self.captured_timestamps(), self.decode_cfg)
            .map(|(frame, _)| frame)
    }

    fn configure_capture_timer(&mut self) -> Result<(), PwmCaptureConfigError> {
        configure_capture_timer(&mut self.timer, self.speed)
    }
}

pub struct Stm32PwmCaptureBidirController<'d, T, TxD, RxD, C>
where
    T: GeneralInstance4Channel,
    TxD: UpDma<T> + RawDmaChannel,
    RxD: DmaChannelInstance,
    C: TimerChannel,
{
    timer: Timer<'d, T>,
    pin: DshotPwmCapturePin<'d, T>,
    tx_dma: DmaChannel<'d>,
    tx_dma_request: embassy_stm32_hal::dma::Request,
    rx_dma: DmaChannel<'d>,
    rx_dma_request: embassy_stm32_hal::dma::Request,
    speed: DshotSpeed,
    runtime_cfg: PwmCaptureBidirRuntimeConfig,
    decode_cfg: PwmCaptureDecodeConfig,
    waveform_timing: WaveformTiming,
    duty_buffer: [T::Word; TX_BUFFER_SLOTS],
    raw_capture: [u16; MAX_CAPTURE_EDGES],
    timestamps: [u32; MAX_CAPTURE_EDGES],
    captured_edges: usize,
    txrx_state: TxRxWakerState,
    _tx_dma: PhantomData<TxD>,
    _rx_dma: PhantomData<RxD>,
    _channel: PhantomData<C>,
}

impl<'d, T, TxD, RxD, C> Stm32PwmCaptureBidirController<'d, T, TxD, RxD, C>
where
    T: GeneralInstance4Channel,
    TxD: UpDma<T> + RawDmaChannel,
    RxD: DmaChannelInstance + Dma<T, C>,
    C: TimerChannel,
{
    fn new_inner(
        timer: Peri<'d, T>,
        pin: DshotPwmCapturePin<'d, T>,
        tx_dma: Peri<'d, TxD>,
        tx_dma_irq: impl Binding<TxD::Interrupt, TxDmaInterruptHandler<TxD>> + 'd,
        rx_dma: Peri<'d, RxD>,
        rx_dma_irq: impl Binding<RxD::Interrupt, embassy_stm32_hal::dma::InterruptHandler<RxD>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, PwmCaptureConfigError> {
        let tx_dma_request = tx_dma.request();
        let rx_dma_request = rx_dma.request();
        let tx_dma = dma::Channel::new(tx_dma, tx_dma_irq);
        let rx_dma = dma::Channel::new(rx_dma, rx_dma_irq);

        let mut timer = Timer::new(timer);
        timer.enable_outputs();
        let period_ticks = configure_tx_timer(&mut timer, pin.channel(), speed);
        let waveform_timing = WaveformTiming {
            period_ticks,
            bit0_high_ticks: (period_ticks * 3) / 8,
            bit1_high_ticks: (period_ticks * 3) / 4,
        };

        let runtime_cfg = PwmCaptureBidirRuntimeConfig::default();
        let mut this = Self {
            timer,
            pin,
            tx_dma,
            tx_dma_request,
            rx_dma,
            rx_dma_request,
            speed,
            runtime_cfg,
            decode_cfg: PwmCaptureDecodeConfig::default(),
            waveform_timing,
            duty_buffer: [T::Word::from(0u16); TX_BUFFER_SLOTS],
            raw_capture: [0; MAX_CAPTURE_EDGES],
            timestamps: [0; MAX_CAPTURE_EDGES],
            captured_edges: 0,
            txrx_state: TxRxWakerState::new(),
            _tx_dma: PhantomData,
            _rx_dma: PhantomData,
            _channel: PhantomData,
        };
        this.pin
            .enter_capture(this.runtime_cfg.capture.capture_pull);
        stop_tx_timer(&mut this.timer, this.pin.channel());
        this.configure_capture_timer()?;
        Ok(this)
    }

    pub fn with_runtime_config(
        mut self,
        runtime_cfg: PwmCaptureBidirRuntimeConfig,
    ) -> Result<Self, PwmCaptureConfigError> {
        self.set_runtime_config(runtime_cfg)?;
        Ok(self)
    }

    pub fn set_runtime_config(
        &mut self,
        runtime_cfg: PwmCaptureBidirRuntimeConfig,
    ) -> Result<(), PwmCaptureConfigError> {
        validate_capture_edge_count(runtime_cfg.capture.capture_edge_count)?;
        self.runtime_cfg = runtime_cfg;
        self.configure_capture_timer()
    }

    pub fn with_decode_config(mut self, decode_cfg: PwmCaptureDecodeConfig) -> Self {
        self.decode_cfg = decode_cfg;
        self
    }

    pub fn captured_timestamps(&self) -> &[u32] {
        &self.timestamps[..self.captured_edges]
    }

    pub fn captured_edge_count(&self) -> usize {
        self.captured_edges
    }

    pub fn raw_capture_values(&self) -> &[u16] {
        &self.raw_capture[..self.captured_edges]
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), PwmCaptureBidirRuntimeError> {
        let frame_period =
            Duration::from_micros(self.speed.timing_hints().min_frame_period_us as u64);
        let deadline = Instant::now() + duration;
        let stop_frame = BidirTx::command(Command::MotorStop).encode();

        while Instant::now() < deadline {
            self.send_frame(stop_frame).await?;
            EmbassyTimer::after(frame_period).await;
        }

        Ok(())
    }

    pub async fn arm(&mut self) -> Result<(), PwmCaptureBidirRuntimeError> {
        self.arm_for(DEFAULT_ARM_DURATION).await
    }

    pub async fn send_throttle(
        &mut self,
        throttle: u16,
    ) -> Result<EncodedFrame, PwmCaptureBidirRuntimeError> {
        self.send_frame(BidirTx::throttle_clamped(throttle).encode())
            .await
    }

    pub async fn send_command(
        &mut self,
        command: Command,
    ) -> Result<EncodedFrame, PwmCaptureBidirRuntimeError> {
        self.send_frame(BidirTx::command(command).encode()).await
    }

    pub async fn send_frame(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<EncodedFrame, PwmCaptureBidirRuntimeError> {
        self.run_tx(frame).await?;
        Ok(frame)
    }

    pub async fn send_throttle_and_receive(
        &mut self,
        throttle: u16,
    ) -> Result<TelemetryFrame, PwmCaptureBidirRuntimeError> {
        self.send_frame_and_receive(BidirTx::throttle_clamped(throttle).encode())
            .await
    }

    pub async fn send_command_and_receive(
        &mut self,
        command: Command,
    ) -> Result<TelemetryFrame, PwmCaptureBidirRuntimeError> {
        self.send_frame_and_receive(BidirTx::command(command).encode())
            .await
    }

    pub async fn send_frame_and_receive(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<TelemetryFrame, PwmCaptureBidirRuntimeError> {
        self.run_tx_then_capture(frame).await?;
        self.decode_frame().map_err(Into::into)
    }

    pub async fn send_frame_and_decode_gcr(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<(GcrDecodeResult, PwmCaptureDecodeDebug), PwmCaptureBidirRuntimeError> {
        self.run_tx_then_capture(frame).await?;
        self.decode_gcr().map_err(Into::into)
    }

    pub async fn capture_edges(&mut self) -> Result<usize, PwmCaptureBidirRuntimeError> {
        self.pin
            .enter_capture(self.runtime_cfg.capture.capture_pull);
        capture_edges_inner(
            &mut self.timer,
            self.pin.channel(),
            &mut self.rx_dma,
            self.rx_dma_request,
            &mut self.raw_capture,
            &mut self.timestamps,
            &mut self.captured_edges,
            self.runtime_cfg.capture,
        )
        .await
        .map_err(Into::into)
    }

    pub fn decode_gcr(
        &self,
    ) -> Result<(GcrDecodeResult, PwmCaptureDecodeDebug), PwmCaptureRuntimeError> {
        decode_gcr_from_timestamps(self.captured_timestamps(), self.decode_cfg).map_err(Into::into)
    }

    pub fn decode_frame(&self) -> Result<TelemetryFrame, DecodeFrameFromTimestampsError> {
        decode_frame_from_timestamps(self.captured_timestamps(), self.decode_cfg)
            .map(|(frame, _)| frame)
    }

    fn configure_capture_timer(&mut self) -> Result<(), PwmCaptureConfigError> {
        configure_capture_timer(&mut self.timer, self.speed)
    }

    async fn run_tx(&mut self, frame: EncodedFrame) -> Result<(), PwmCaptureBidirRuntimeError> {
        let waveform = frame.to_waveform_ticks(self.waveform_timing, true);
        for (slot, ticks) in self.duty_buffer[..16]
            .iter_mut()
            .zip(waveform.bit_high_ticks.iter().copied())
        {
            *slot = ticks.into();
        }
        self.duty_buffer[16] = waveform.reset_low_ticks.unwrap_or(0).into();

        configure_tx_timer(&mut self.timer, self.pin.channel(), self.speed);
        self.pin.enter_tx();

        let ccr = self
            .timer
            .regs_gp16()
            .ccr(self.pin.channel().index())
            .as_ptr() as *mut T::Word;

        let transfer = unsafe {
            self.tx_dma.write(
                self.tx_dma_request,
                &self.duty_buffer,
                ccr,
                dma::TransferOptions::default(),
            )
        };

        start_tx_timer(&mut self.timer);
        let result = with_timeout(self.runtime_cfg.tx_timeout, transfer).await;

        self.pin
            .enter_capture(self.runtime_cfg.capture.capture_pull);
        stop_tx_timer(&mut self.timer, self.pin.channel());

        match result {
            Ok(()) => Ok(()),
            Err(_) => Err(PwmCaptureBidirRuntimeError::TxTimeout),
        }
    }

    async fn run_tx_then_capture(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<(), PwmCaptureBidirRuntimeError> {
        let waveform = frame.to_waveform_ticks(self.waveform_timing, true);
        for (slot, ticks) in self.duty_buffer[..16]
            .iter_mut()
            .zip(waveform.bit_high_ticks.iter().copied())
        {
            *slot = ticks.into();
        }
        self.duty_buffer[16] = waveform.reset_low_ticks.unwrap_or(0).into();

        let capture_len = self.runtime_cfg.capture.capture_edge_count;
        self.captured_edges = 0;
        self.raw_capture[..capture_len].fill(0);

        self.timer.stop();
        self.timer.reset();
        self.timer.clear_input_interrupt(self.pin.channel());

        let ccr_ptr = self
            .timer
            .regs_gp16()
            .ccr(self.pin.channel().index())
            .as_ptr() as *mut u16;
        let mut ring = unsafe {
            DmaReadableRingBuffer::new(
                self.rx_dma.reborrow(),
                self.rx_dma_request,
                ccr_ptr,
                &mut self.raw_capture[..capture_len],
                dma::TransferOptions::default(),
            )
        };
        ring.clear();
        ring.start();

        configure_tx_timer(&mut self.timer, self.pin.channel(), self.speed);
        self.pin.enter_tx();

        let tx_ccr = self
            .timer
            .regs_gp16()
            .ccr(self.pin.channel().index())
            .as_ptr() as *mut T::Word;
        let transfer = unsafe {
            self.tx_dma.write(
                self.tx_dma_request,
                &self.duty_buffer,
                tx_ccr,
                dma::TransferOptions::default(),
            )
        };

        self.txrx_state
            .phase
            .store(TxRxPhase::TxActive as u8, Ordering::Release);
        unsafe {
            self.register_tx_irq_callback();
        }
        start_tx_timer(&mut self.timer);

        let tx_result = with_timeout(
            self.runtime_cfg.tx_timeout,
            poll_fn(|cx| {
                self.txrx_state.waker.register(cx.waker());
                match self.txrx_state.phase.load(Ordering::Acquire) {
                    x if x == TxRxPhase::TxActive as u8 => core::task::Poll::Pending,
                    x if x == TxRxPhase::RxActive as u8 => core::task::Poll::Ready(Ok(())),
                    _ => core::task::Poll::Ready(Err(PwmCaptureBidirRuntimeError::TxTimeout)),
                }
            }),
        )
        .await;

        self.clear_tx_irq_callback();
        drop(transfer);

        match tx_result {
            Ok(Ok(())) => {}
            Ok(Err(err)) => {
                self.finish_capture_early(&mut ring);
                return Err(err);
            }
            Err(_) => {
                self.finish_capture_early(&mut ring);
                return Err(PwmCaptureBidirRuntimeError::TxTimeout);
            }
        }

        EmbassyTimer::after(self.runtime_cfg.capture.rx_timeout).await;
        let read = ring
            .len()
            .map_err(|_| PwmCaptureBidirRuntimeError::RxTimeout)?;
        self.finish_capture_early(&mut ring);

        if read == 0 {
            return Err(PwmCaptureBidirRuntimeError::RxTimeout);
        }

        normalize_timestamps(
            &self.raw_capture,
            &mut self.timestamps,
            read,
            &mut self.captured_edges,
        );
        Ok(())
    }

    fn finish_capture_early(&mut self, ring: &mut DmaReadableRingBuffer<'_, u16>) {
        ring.request_reset();
        while ring.is_running() {}
        self.timer
            .set_cc_dma_enable_state(self.pin.channel(), false);
        self.timer.enable_channel(self.pin.channel(), false);
        self.timer.stop();
        self.pin
            .enter_capture(self.runtime_cfg.capture.capture_pull);
        self.txrx_state
            .phase
            .store(TxRxPhase::Idle as u8, Ordering::Release);
    }

    unsafe fn tx_dma_irq(ctx: *mut ()) {
        let this = &mut *(ctx as *mut Self);
        let regs = TxD::regs();
        let isr = regs.isr(TxD::stream_num() / 4).read();
        let bit = TxD::stream_num() % 4;

        if isr.teif(bit) {
            regs.ifcr(TxD::stream_num() / 4)
                .write(|w| w.set_teif(bit, true));
            stop_tx_timer(&mut this.timer, this.pin.channel());
            this.pin
                .enter_capture(this.runtime_cfg.capture.capture_pull);
            this.txrx_state
                .phase
                .store(TxRxPhase::Error as u8, Ordering::Release);
            this.txrx_state.waker.wake();
            return;
        }

        if !isr.tcif(bit) {
            return;
        }

        stop_tx_timer(&mut this.timer, this.pin.channel());
        configure_capture_handoff_timer(
            &mut this.timer,
            this.pin.channel(),
            this.runtime_cfg.capture,
        );
        this.pin
            .enter_capture(this.runtime_cfg.capture.capture_pull);
        this.timer.set_cc_dma_enable_state(this.pin.channel(), true);
        this.timer.enable_channel(this.pin.channel(), true);
        this.timer.start();
        this.txrx_state
            .phase
            .store(TxRxPhase::RxActive as u8, Ordering::Release);
        this.txrx_state.waker.wake();
    }
}

impl<'d, T, D> Stm32PwmCaptureReceiver<'d, T, D, Ch1>
where
    T: GeneralInstance4Channel,
    D: DmaChannelInstance + Dma<T, Ch1>,
{
    pub fn new_ch1(
        timer: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, Ch1>>,
        dma: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, embassy_stm32_hal::dma::InterruptHandler<D>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, PwmCaptureConfigError> {
        Self::new_inner(timer, DshotPwmCapturePin::new_ch1(pin), dma, dma_irq, speed)
    }
}

impl<'d, T, D> Stm32PwmCaptureReceiver<'d, T, D, Ch2>
where
    T: GeneralInstance4Channel,
    D: DmaChannelInstance + Dma<T, Ch2>,
{
    pub fn new_ch2(
        timer: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, Ch2>>,
        dma: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, embassy_stm32_hal::dma::InterruptHandler<D>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, PwmCaptureConfigError> {
        Self::new_inner(timer, DshotPwmCapturePin::new_ch2(pin), dma, dma_irq, speed)
    }
}

impl<'d, T, D> Stm32PwmCaptureReceiver<'d, T, D, Ch3>
where
    T: GeneralInstance4Channel,
    D: DmaChannelInstance + Dma<T, Ch3>,
{
    pub fn new_ch3(
        timer: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, Ch3>>,
        dma: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, embassy_stm32_hal::dma::InterruptHandler<D>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, PwmCaptureConfigError> {
        Self::new_inner(timer, DshotPwmCapturePin::new_ch3(pin), dma, dma_irq, speed)
    }
}

impl<'d, T, D> Stm32PwmCaptureReceiver<'d, T, D, Ch4>
where
    T: GeneralInstance4Channel,
    D: DmaChannelInstance + Dma<T, Ch4>,
{
    pub fn new_ch4(
        timer: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, Ch4>>,
        dma: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, embassy_stm32_hal::dma::InterruptHandler<D>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, PwmCaptureConfigError> {
        Self::new_inner(timer, DshotPwmCapturePin::new_ch4(pin), dma, dma_irq, speed)
    }
}

impl<'d, T, TxD, RxD> Stm32PwmCaptureBidirController<'d, T, TxD, RxD, Ch1>
where
    T: GeneralInstance4Channel,
    TxD: UpDma<T> + RawDmaChannel,
    RxD: DmaChannelInstance + Dma<T, Ch1>,
{
    pub fn new_ch1(
        timer: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, Ch1>>,
        tx_dma: Peri<'d, TxD>,
        tx_dma_irq: impl Binding<TxD::Interrupt, TxDmaInterruptHandler<TxD>> + 'd,
        rx_dma: Peri<'d, RxD>,
        rx_dma_irq: impl Binding<RxD::Interrupt, embassy_stm32_hal::dma::InterruptHandler<RxD>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, PwmCaptureConfigError> {
        Self::new_inner(
            timer,
            DshotPwmCapturePin::new_ch1(pin),
            tx_dma,
            tx_dma_irq,
            rx_dma,
            rx_dma_irq,
            speed,
        )
    }
}

impl<'d, T, TxD, RxD> Stm32PwmCaptureBidirController<'d, T, TxD, RxD, Ch2>
where
    T: GeneralInstance4Channel,
    TxD: UpDma<T> + RawDmaChannel,
    RxD: DmaChannelInstance + Dma<T, Ch2>,
{
    pub fn new_ch2(
        timer: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, Ch2>>,
        tx_dma: Peri<'d, TxD>,
        tx_dma_irq: impl Binding<TxD::Interrupt, TxDmaInterruptHandler<TxD>> + 'd,
        rx_dma: Peri<'d, RxD>,
        rx_dma_irq: impl Binding<RxD::Interrupt, embassy_stm32_hal::dma::InterruptHandler<RxD>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, PwmCaptureConfigError> {
        Self::new_inner(
            timer,
            DshotPwmCapturePin::new_ch2(pin),
            tx_dma,
            tx_dma_irq,
            rx_dma,
            rx_dma_irq,
            speed,
        )
    }
}

impl<'d, T, TxD, RxD> Stm32PwmCaptureBidirController<'d, T, TxD, RxD, Ch3>
where
    T: GeneralInstance4Channel,
    TxD: UpDma<T> + RawDmaChannel,
    RxD: DmaChannelInstance + Dma<T, Ch3>,
{
    pub fn new_ch3(
        timer: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, Ch3>>,
        tx_dma: Peri<'d, TxD>,
        tx_dma_irq: impl Binding<TxD::Interrupt, TxDmaInterruptHandler<TxD>> + 'd,
        rx_dma: Peri<'d, RxD>,
        rx_dma_irq: impl Binding<RxD::Interrupt, embassy_stm32_hal::dma::InterruptHandler<RxD>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, PwmCaptureConfigError> {
        Self::new_inner(
            timer,
            DshotPwmCapturePin::new_ch3(pin),
            tx_dma,
            tx_dma_irq,
            rx_dma,
            rx_dma_irq,
            speed,
        )
    }
}

impl<'d, T, TxD, RxD> Stm32PwmCaptureBidirController<'d, T, TxD, RxD, Ch4>
where
    T: GeneralInstance4Channel,
    TxD: UpDma<T> + RawDmaChannel,
    RxD: DmaChannelInstance + Dma<T, Ch4>,
{
    pub fn new_ch4(
        timer: Peri<'d, T>,
        pin: Peri<'d, impl TimerPin<T, Ch4>>,
        tx_dma: Peri<'d, TxD>,
        tx_dma_irq: impl Binding<TxD::Interrupt, TxDmaInterruptHandler<TxD>> + 'd,
        rx_dma: Peri<'d, RxD>,
        rx_dma_irq: impl Binding<RxD::Interrupt, embassy_stm32_hal::dma::InterruptHandler<RxD>> + 'd,
        speed: DshotSpeed,
    ) -> Result<Self, PwmCaptureConfigError> {
        Self::new_inner(
            timer,
            DshotPwmCapturePin::new_ch4(pin),
            tx_dma,
            tx_dma_irq,
            rx_dma,
            rx_dma_irq,
            speed,
        )
    }
}

fn validate_capture_edge_count(capture_edge_count: usize) -> Result<(), PwmCaptureConfigError> {
    if capture_edge_count == 0 || capture_edge_count > MAX_CAPTURE_EDGES {
        return Err(PwmCaptureConfigError::InvalidCaptureEdgeCount {
            requested: capture_edge_count,
            max_supported: MAX_CAPTURE_EDGES,
        });
    }
    Ok(())
}

fn configure_capture_timer<T: GeneralInstance4Channel>(
    timer: &mut Timer<'_, T>,
    speed: DshotSpeed,
) -> Result<(), PwmCaptureConfigError> {
    let tick_hz = speed
        .timing_hints()
        .nominal_bitrate_hz
        .checked_mul(20)
        .ok_or(PwmCaptureConfigError::InvalidTickRate)?;

    timer.stop();
    timer.reset();
    timer.set_tick_freq(Hertz(tick_hz));
    timer.set_max_compare_value(
        (u16::MAX as u32)
            .try_into()
            .ok()
            .ok_or(PwmCaptureConfigError::InvalidTickRate)?,
    );
    timer.generate_update_event();
    Ok(())
}

async fn capture_edges_inner<T: GeneralInstance4Channel>(
    timer: &mut Timer<'_, T>,
    channel: Channel,
    dma: &mut DmaChannel<'_>,
    dma_request: embassy_stm32_hal::dma::Request,
    raw_capture: &mut [u16; MAX_CAPTURE_EDGES],
    timestamps: &mut [u32; MAX_CAPTURE_EDGES],
    captured_edges: &mut usize,
    runtime_cfg: PwmCaptureRuntimeConfig,
) -> Result<usize, PwmCaptureRuntimeError> {
    let capture_len = runtime_cfg.capture_edge_count;
    *captured_edges = 0;
    raw_capture[..capture_len].fill(0);

    timer.stop();
    timer.reset();
    timer.clear_input_interrupt(channel);
    timer.set_input_ti_selection(channel, InputTISelection::Normal);
    timer.set_input_capture_mode(channel, InputCaptureMode::BothEdges);
    timer.set_input_capture_filter(channel, FilterValue::from_bits(runtime_cfg.capture_filter));
    timer.set_input_capture_prescaler(channel, 0);

    let ccr_ptr = timer.regs_gp16().ccr(channel.index()).as_ptr() as *mut u16;
    let read = {
        let mut ring = unsafe {
            DmaReadableRingBuffer::new(
                dma.reborrow(),
                dma_request,
                ccr_ptr,
                &mut raw_capture[..capture_len],
                dma::TransferOptions::default(),
            )
        };
        ring.clear();
        timer.set_cc_dma_enable_state(channel, true);
        timer.enable_channel(channel, true);
        ring.start();
        timer.start();

        EmbassyTimer::after(runtime_cfg.rx_timeout).await;

        let received = ring.len().map_err(|_| PwmCaptureRuntimeError::Timeout)?;
        timer.set_cc_dma_enable_state(channel, false);
        timer.enable_channel(channel, false);
        timer.stop();
        received
    };

    if read == 0 {
        Err(PwmCaptureRuntimeError::Timeout)
    } else {
        normalize_timestamps(raw_capture, timestamps, read, captured_edges);
        Ok(*captured_edges)
    }
}

fn normalize_timestamps(
    raw_capture: &[u16; MAX_CAPTURE_EDGES],
    timestamps: &mut [u32; MAX_CAPTURE_EDGES],
    capture_len: usize,
    captured_edges: &mut usize,
) {
    if capture_len == 0 {
        *captured_edges = 0;
        return;
    }

    timestamps[0] = raw_capture[0] as u32;
    let mut acc = timestamps[0];
    for idx in 1..capture_len {
        let delta = raw_capture[idx].wrapping_sub(raw_capture[idx - 1]) as u32;
        acc = acc.wrapping_add(delta);
        timestamps[idx] = acc;
    }
    *captured_edges = capture_len;
}

fn configure_tx_timer<T: GeneralInstance4Channel>(
    timer: &mut Timer<'_, T>,
    channel: Channel,
    speed: DshotSpeed,
) -> u16 {
    timer.stop();
    timer.enable_update_dma(false);
    timer.reset();
    timer.set_output_compare_mode(channel, OutputCompareMode::PwmMode1);
    timer.set_output_compare_preload(channel, true);
    timer.set_output_polarity(channel, OutputPolarity::ActiveLow);
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
    (max_compare_value + 1).try_into().ok().unwrap_or(u16::MAX)
}

fn start_tx_timer<T: GeneralInstance4Channel>(timer: &mut Timer<'_, T>) {
    timer.start();
}

fn stop_tx_timer<T: GeneralInstance4Channel>(timer: &mut Timer<'_, T>, channel: Channel) {
    timer.stop();
    timer.enable_update_dma(false);
    timer.enable_channel(channel, false);
    timer.set_compare_value(channel, 0u16.into());
}
