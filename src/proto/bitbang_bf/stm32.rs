use core::array;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{AtomicPtr, AtomicU8, Ordering};

use embassy_stm32_hal::dma::{ChannelInstance as DmaChannelInstance, Request};
use embassy_stm32_hal::gpio::{AnyPin, Flex, Pin, Pull, Speed};
use embassy_stm32_hal::interrupt::typelevel::{Binding, Handler};
use embassy_stm32_hal::pac;
use embassy_stm32_hal::timer::low_level::{OutputCompareMode, OutputPolarity, Timer};
use embassy_stm32_hal::timer::{
    Ch1, Ch2, Ch3, Ch4, Channel, Dma, GeneralInstance4Channel, TimerChannel,
};
use embassy_stm32_hal::Peri;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{with_timeout, Duration, Instant, Ticker, Timer as EmbassyTimer};

#[cfg(not(feature = "defmt"))]
use crate::bidir_capture::decode_frame_bf_strict_port_samples_u16;
#[cfg(feature = "defmt")]
use crate::bidir_capture::{decode_bf_raw_21, decode_frame_bf_strict_port_samples_with_debug_u16};
use crate::proto::bitbang_bf::{build_port_words, PortFrameError, SignalPolarity, TX_STATE_SLOTS};
use crate::telemetry::{
    BidirDecoder, DecodeHint, OversamplingConfig, PreambleTuningConfig, TelemetryFrame,
    TelemetryPipelineError,
};
use crate::{BidirTx, Command, DshotSpeed, EncodedFrame, UniTx};

const MAX_PORT_MOTORS: usize = 4;
const MAX_CAPTURE_SAMPLES: usize = 512;
const PINS_PER_GPIO_PORT: u8 = 16;
const DEFAULT_ARM_DURATION: Duration = Duration::from_millis(3_000);
const PREAMBLE_MARGIN_SAMPLES: usize = 64;
const DMA_IRQ_SLOTS: usize = 16;

type DmaIrqFn = unsafe fn(*mut ());

static DMA_IRQ_CTX: [AtomicPtr<()>; DMA_IRQ_SLOTS] =
    [const { AtomicPtr::new(ptr::null_mut()) }; DMA_IRQ_SLOTS];
static DMA_IRQ_FN: [AtomicPtr<()>; DMA_IRQ_SLOTS] =
    [const { AtomicPtr::new(ptr::null_mut()) }; DMA_IRQ_SLOTS];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PortRuntimeConfig {
    pub tx_timeout: Duration,
    pub pacer_compare_percent: u8,
}

impl Default for PortRuntimeConfig {
    fn default() -> Self {
        Self {
            tx_timeout: Duration::from_millis(2),
            pacer_compare_percent: 50,
        }
    }
}

impl PortRuntimeConfig {
    fn normalized(self) -> Self {
        Self {
            tx_timeout: self.tx_timeout,
            pacer_compare_percent: self.pacer_compare_percent.clamp(1, 99),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PortConfigError {
    InvalidMotorCount {
        requested: usize,
        max_supported: usize,
    },
    MixedPorts,
    DuplicatePins,
    SampleBufferTooSmall {
        requested: usize,
        capacity: usize,
    },
    Frame(PortFrameError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PortRuntimeError {
    TxTimeout,
    TxDmaError,
    RxTimeout,
    Frame(PortFrameError),
}

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum IrqPhase {
    Idle = 0,
    TxActive = 1,
    RxActive = 2,
    Done = 3,
    Error = 4,
}

struct IrqWakerState {
    phase: AtomicU8,
    waker: AtomicWaker,
}

impl IrqWakerState {
    const fn new() -> Self {
        Self {
            phase: AtomicU8::new(IrqPhase::Idle as u8),
            waker: AtomicWaker::new(),
        }
    }

    fn load_phase(&self) -> IrqPhase {
        match self.phase.load(Ordering::Acquire) {
            x if x == IrqPhase::Idle as u8 => IrqPhase::Idle,
            x if x == IrqPhase::TxActive as u8 => IrqPhase::TxActive,
            x if x == IrqPhase::RxActive as u8 => IrqPhase::RxActive,
            x if x == IrqPhase::Done as u8 => IrqPhase::Done,
            x if x == IrqPhase::Error as u8 => IrqPhase::Error,
            _ => IrqPhase::Error,
        }
    }

    fn set_phase(&self, phase: IrqPhase) {
        self.phase.store(phase as u8, Ordering::Release);
    }

    fn transition(&self, phase: IrqPhase) {
        self.set_phase(phase);
        self.waker.wake();
    }

    fn register(&self, waker: &core::task::Waker) {
        self.waker.register(waker);
    }
}

#[derive(Clone, Copy)]
struct PacerTimerConfig {
    psc: u16,
    arr: u16,
    compare: u16,
}

#[derive(Clone, Copy)]
struct PreparedRxDmaConfig {
    request: Request,
    peri_addr: *mut u16,
    len: usize,
}

#[derive(Clone, Copy)]
struct PortPinSet<const N: usize> {
    pin_masks: [u32; N],
    group_mask: u32,
}

pub trait RawDmaChannel: DmaChannelInstance {
    const IRQ_SLOT: usize;
    fn regs() -> pac::dma::Dma;
    fn stream_num() -> usize;
}

pub struct InterruptHandler<D: RawDmaChannel> {
    _phantom: PhantomData<D>,
}

impl<D: RawDmaChannel> Handler<D::Interrupt> for InterruptHandler<D> {
    unsafe fn on_interrupt() {
        let ctx = DMA_IRQ_CTX[D::IRQ_SLOT].load(Ordering::Acquire);
        let func = DMA_IRQ_FN[D::IRQ_SLOT].load(Ordering::Acquire);
        if ctx.is_null() || func.is_null() {
            return;
        }

        let func: DmaIrqFn = core::mem::transmute(func);
        func(ctx);
    }
}

macro_rules! impl_raw_dma_channel {
    ($periph:ty, $irq_slot:expr, $stream_num:expr) => {
        impl RawDmaChannel for $periph {
            const IRQ_SLOT: usize = $irq_slot;

            fn regs() -> pac::dma::Dma {
                unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
            }

            fn stream_num() -> usize {
                $stream_num
            }
        }
    };
}

impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH1, 9, 1);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH2, 10, 2);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH3, 11, 3);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH4, 12, 4);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH5, 13, 5);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH6, 14, 6);

pub struct DshotPortPin<'d> {
    line: Flex<'d>,
    port: u8,
    pin_mask: u32,
    bsrr_ptr: *mut u32,
    idr_ptr: *mut u16,
}

impl<'d> DshotPortPin<'d> {
    pub fn new(pin: Peri<'d, impl Pin>) -> Self {
        let pin_mask = 1u32 << pin.pin();
        let port = pin.port();
        let pin_port = (port as usize) * (PINS_PER_GPIO_PORT as usize) + (pin.pin() as usize);
        let regs = unsafe { AnyPin::steal(pin_port as u8) }.block();

        Self {
            line: Flex::new(pin),
            port,
            pin_mask,
            bsrr_ptr: regs.bsrr().as_ptr() as *mut u32,
            idr_ptr: regs.idr().as_ptr() as *mut u16,
        }
    }

    fn enter_output_low(&mut self) {
        self.line.set_low();
        self.line.set_as_output(Speed::VeryHigh);
    }

    fn enter_output_high(&mut self) {
        self.line.set_high();
        self.line.set_as_output(Speed::VeryHigh);
    }

    fn enter_input(&mut self, pull: Pull) {
        self.line.set_as_input(pull);
    }

    fn port(&self) -> u8 {
        self.port
    }

    fn pin_mask(&self) -> u32 {
        self.pin_mask
    }

    fn bsrr_ptr(&self) -> *mut u32 {
        self.bsrr_ptr
    }

    fn idr_ptr(&self) -> *mut u16 {
        self.idr_ptr
    }
}

fn validate_port_pins<const N: usize>(
    pins: &[DshotPortPin<'_>; N],
) -> Result<PortPinSet<N>, PortConfigError> {
    if N == 0 || N > MAX_PORT_MOTORS {
        return Err(PortConfigError::InvalidMotorCount {
            requested: N,
            max_supported: MAX_PORT_MOTORS,
        });
    }

    let first_port = pins[0].port();
    let mut group_mask = 0u32;
    for pin in pins {
        if pin.port() != first_port {
            return Err(PortConfigError::MixedPorts);
        }
        if (group_mask & pin.pin_mask()) != 0 {
            return Err(PortConfigError::DuplicatePins);
        }
        group_mask |= pin.pin_mask();
    }

    Ok(PortPinSet {
        pin_masks: array::from_fn(|idx| pins[idx].pin_mask()),
        group_mask,
    })
}

macro_rules! impl_tx_port_channel_ctors {
    ($(($name:ident, $channel:ty)),+ $(,)?) => {
        $(
            pub fn $name(
                timer: Peri<'d, T>,
                dma: Peri<'d, D>,
                pins: [DshotPortPin<'d>; N],
                speed: DshotSpeed,
            ) -> Result<Self, PortConfigError>
            where
                D: Dma<T, $channel>,
            {
                Self::new_inner::<$channel>(timer, dma, pins, speed)
            }
        )+
    };
}

macro_rules! impl_bidir_pin_channel_ctors {
    ($(($name:ident, $channel:ty)),+ $(,)?) => {
        $(
            pub fn $name(
                timer: Peri<'d, T>,
                dma: Peri<'d, D>,
                dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
                pin: DshotPortPin<'d>,
                capture_cfg: BidirCaptureConfig,
                speed: DshotSpeed,
            ) -> Result<Self, PortConfigError>
            where
                D: Dma<T, $channel>,
            {
                Self::new_inner::<$channel>(timer, dma, dma_irq, pin, capture_cfg, speed)
            }
        )+
    };
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BidirPortRuntimeConfig {
    pub tx_timeout: Duration,
    pub rx_timeout: Duration,
    pub pacer_compare_percent: u8,
    pub rx_compare_percent: u8,
    pub rx_sample_percent: u8,
}

impl Default for BidirPortRuntimeConfig {
    fn default() -> Self {
        Self {
            tx_timeout: Duration::from_millis(2),
            rx_timeout: Duration::from_millis(2),
            pacer_compare_percent: 50,
            rx_compare_percent: 50,
            rx_sample_percent: 100,
        }
    }
}

impl BidirPortRuntimeConfig {
    fn normalized(self) -> Self {
        Self {
            tx_timeout: self.tx_timeout,
            rx_timeout: self.rx_timeout,
            pacer_compare_percent: self.pacer_compare_percent.clamp(1, 99),
            rx_compare_percent: self.rx_compare_percent.clamp(1, 99),
            rx_sample_percent: self.rx_sample_percent.clamp(1, 200),
        }
    }
}

#[derive(Clone, Copy)]
pub struct BidirCaptureConfig {
    oversampling: OversamplingConfig,
    sample_count: usize,
    decode_hint: DecodeHint,
    pull: Pull,
    preamble_tuning: PreambleTuningConfig,
}

impl BidirCaptureConfig {
    pub fn new(oversampling: OversamplingConfig) -> Self {
        Self {
            sample_count: oversampling.recommended_capture_samples(PREAMBLE_MARGIN_SAMPLES),
            oversampling,
            decode_hint: DecodeHint::default(),
            pull: Pull::Up,
            preamble_tuning: PreambleTuningConfig::default(),
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
}

pub struct Stm32TxPortController<'d, T, D, const N: usize>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    timer: Timer<'d, T>,
    dma_request: Request,
    pins: [DshotPortPin<'d>; N],
    pin_masks: [u32; N],
    group_mask: u32,
    channel: Channel,
    speed: DshotSpeed,
    cfg: PortRuntimeConfig,
    tx_timer_cfg: PacerTimerConfig,
    tx_words: [u32; TX_STATE_SLOTS],
    _dma: PhantomData<D>,
}

impl<'d, T, D, const N: usize> Stm32TxPortController<'d, T, D, N>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    impl_tx_port_channel_ctors! {
        (new_ch1, Ch1),
        (new_ch2, Ch2),
        (new_ch3, Ch3),
        (new_ch4, Ch4),
    }

    pub fn with_runtime_config(mut self, cfg: PortRuntimeConfig) -> Self {
        self.cfg = cfg.normalized();
        self.tx_timer_cfg = compute_pacer_timer_config(
            &self.timer,
            self.speed.timing_hints().nominal_bitrate_hz * 3,
            self.cfg.pacer_compare_percent,
        );
        configure_pacer_timer(&self.timer, self.channel, self.tx_timer_cfg);
        self
    }

    fn new_inner<C>(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        mut pins: [DshotPortPin<'d>; N],
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        C: TimerChannel,
        D: Dma<T, C>,
    {
        let pin_set = validate_port_pins(&pins)?;
        let dma_request = dma.request();
        dma.remap();
        drop(dma);

        let timer = Timer::new(timer);
        let cfg = PortRuntimeConfig::default().normalized();
        let tx_timer_cfg = compute_pacer_timer_config(
            &timer,
            speed.timing_hints().nominal_bitrate_hz * 3,
            cfg.pacer_compare_percent,
        );
        configure_pacer_timer(&timer, C::CHANNEL, tx_timer_cfg);

        for pin in pins.iter_mut() {
            pin.enter_output_low();
        }

        Ok(Self {
            timer,
            dma_request,
            pins,
            pin_masks: pin_set.pin_masks,
            group_mask: pin_set.group_mask,
            channel: C::CHANNEL,
            speed,
            cfg,
            tx_timer_cfg,
            tx_words: [0; TX_STATE_SLOTS],
            _dma: PhantomData,
        })
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), PortRuntimeError> {
        let frame_period =
            Duration::from_micros(self.speed.timing_hints().min_frame_period_us as u64);
        let stop_frames = [UniTx::command(Command::MotorStop).encode(); N];
        let mut ticker = Ticker::every(frame_period);

        match with_timeout(duration, async {
            loop {
                self.send_frames(stop_frames).await?;
                ticker.next().await;
            }
            #[allow(unreachable_code)]
            Ok::<(), PortRuntimeError>(())
        })
        .await
        {
            Ok(result) => result,
            Err(_) => Ok(()),
        }
    }

    pub async fn arm(&mut self) -> Result<(), PortRuntimeError> {
        self.arm_for(DEFAULT_ARM_DURATION).await
    }

    pub async fn send_throttles(&mut self, throttles: [u16; N]) -> Result<(), PortRuntimeError> {
        self.send_frames(throttles.map(|throttle| UniTx::throttle_clamped(throttle).encode()))
            .await
    }

    pub async fn send_frames(&mut self, frames: [EncodedFrame; N]) -> Result<(), PortRuntimeError> {
        let port_words = build_port_words(
            self.group_mask,
            self.pin_masks,
            frames,
            SignalPolarity::Normal,
        )
        .map_err(PortRuntimeError::Frame)?;
        self.tx_words = port_words.words;
        self.run_tx_dma().await
    }

    async fn run_tx_dma(&mut self) -> Result<(), PortRuntimeError> {
        let tx_timeout = self.cfg.tx_timeout;
        let _session = TxPortSession::<T, D, N>::start(self)?;

        let result = with_timeout(tx_timeout, async {
            loop {
                if DmaStream::<D>::transfer_error() {
                    return Err(PortRuntimeError::TxDmaError);
                }
                if DmaStream::<D>::transfer_complete() {
                    return Ok(());
                }

                EmbassyTimer::after_micros(1).await;
            }
        })
        .await;

        match result {
            Ok(result) => result,
            Err(_) => Err(PortRuntimeError::TxTimeout),
        }
    }
}

pub struct Stm32BidirPinController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    timer: Timer<'d, T>,
    dma_request: Request,
    pin: DshotPortPin<'d>,
    channel: Channel,
    speed: DshotSpeed,
    runtime_cfg: BidirPortRuntimeConfig,
    capture_cfg: BidirCaptureConfig,
    tx_timer_cfg: PacerTimerConfig,
    rx_timer_cfg: PacerTimerConfig,
    rx_dma_cfg: PreparedRxDmaConfig,
    decoder: BidirDecoder,
    tx_words: [u32; TX_STATE_SLOTS],
    raw_samples: [u16; MAX_CAPTURE_SAMPLES],
    irq_state: IrqWakerState,
    #[cfg(feature = "defmt")]
    capture_attempts: u32,
    #[cfg(feature = "defmt")]
    decode_error_count: u32,
    #[cfg(feature = "defmt")]
    tx_irq_started_at: Instant,
    #[cfg(feature = "defmt")]
    last_tx_done_irq_us: u32,
    #[cfg(feature = "defmt")]
    last_rx_armed_irq_us: u32,
    #[cfg(feature = "defmt")]
    last_rx_handoff_us: u32,
    #[cfg(feature = "defmt")]
    last_rx_capture_us: u32,
    _dma: PhantomData<D>,
}

impl<'d, T, D> Stm32BidirPinController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    impl_bidir_pin_channel_ctors! {
        (new_ch1, Ch1),
        (new_ch2, Ch2),
        (new_ch3, Ch3),
        (new_ch4, Ch4),
    }

    pub fn with_runtime_config(mut self, runtime_cfg: BidirPortRuntimeConfig) -> Self {
        self.set_runtime_config(runtime_cfg);
        self
    }

    pub fn set_runtime_config(&mut self, runtime_cfg: BidirPortRuntimeConfig) {
        self.runtime_cfg = runtime_cfg.normalized();
        self.tx_timer_cfg = compute_pacer_timer_config(
            &self.timer,
            self.speed.timing_hints().nominal_bitrate_hz * 3,
            self.runtime_cfg.pacer_compare_percent,
        );
        self.rx_timer_cfg =
            compute_rx_timer_config(&self.timer, self.speed, self.runtime_cfg, self.capture_cfg);
        configure_pacer_timer(&self.timer, self.channel, self.tx_timer_cfg);
    }

    fn new_inner<C>(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        _dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
        mut pin: DshotPortPin<'d>,
        capture_cfg: BidirCaptureConfig,
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        C: TimerChannel,
        D: Dma<T, C>,
    {
        if capture_cfg.sample_count == 0 || capture_cfg.sample_count > MAX_CAPTURE_SAMPLES {
            return Err(PortConfigError::SampleBufferTooSmall {
                requested: capture_cfg.sample_count,
                capacity: MAX_CAPTURE_SAMPLES,
            });
        }

        let dma_request = dma.request();
        dma.remap();
        drop(dma);

        let timer = Timer::new(timer);
        let runtime_cfg = BidirPortRuntimeConfig::default().normalized();
        let tx_timer_cfg = compute_pacer_timer_config(
            &timer,
            speed.timing_hints().nominal_bitrate_hz * 3,
            runtime_cfg.pacer_compare_percent,
        );
        let rx_timer_cfg = compute_rx_timer_config(&timer, speed, runtime_cfg, capture_cfg);
        configure_pacer_timer(&timer, C::CHANNEL, tx_timer_cfg);
        pin.enter_input(capture_cfg.pull);
        let pin_idr_ptr = pin.idr_ptr();

        let mut decoder = BidirDecoder::with_preamble_tuning(
            capture_cfg.oversampling,
            capture_cfg.preamble_tuning,
        );
        decoder.set_stream_hint(capture_cfg.decode_hint);

        Ok(Self {
            timer,
            dma_request,
            pin,
            channel: C::CHANNEL,
            speed,
            runtime_cfg,
            capture_cfg,
            tx_timer_cfg,
            rx_timer_cfg,
            rx_dma_cfg: PreparedRxDmaConfig {
                request: dma_request,
                peri_addr: pin_idr_ptr,
                len: capture_cfg.sample_count,
            },
            decoder,
            tx_words: [0; TX_STATE_SLOTS],
            raw_samples: [0; MAX_CAPTURE_SAMPLES],
            irq_state: IrqWakerState::new(),
            #[cfg(feature = "defmt")]
            capture_attempts: 0,
            #[cfg(feature = "defmt")]
            decode_error_count: 0,
            #[cfg(feature = "defmt")]
            tx_irq_started_at: Instant::from_ticks(0),
            #[cfg(feature = "defmt")]
            last_tx_done_irq_us: 0,
            #[cfg(feature = "defmt")]
            last_rx_armed_irq_us: 0,
            #[cfg(feature = "defmt")]
            last_rx_handoff_us: 0,
            #[cfg(feature = "defmt")]
            last_rx_capture_us: 0,
            _dma: PhantomData,
        })
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), PortRuntimeError> {
        let frame_period =
            Duration::from_micros(self.speed.timing_hints().min_frame_period_us as u64);
        let stop_frame = BidirTx::command(Command::MotorStop).encode();
        let mut ticker = Ticker::every(frame_period);

        match with_timeout(duration, async {
            loop {
                self.send_frame(stop_frame).await?;
                ticker.next().await;
            }
            #[allow(unreachable_code)]
            Ok::<(), PortRuntimeError>(())
        })
        .await
        {
            Ok(result) => result,
            Err(_) => Ok(()),
        }
    }

    pub async fn arm(&mut self) -> Result<(), PortRuntimeError> {
        self.arm_for(DEFAULT_ARM_DURATION).await
    }

    pub async fn send_throttle_and_receive(
        &mut self,
        throttle: u16,
    ) -> Result<Result<TelemetryFrame, TelemetryPipelineError>, PortRuntimeError> {
        self.send_frame_and_receive(BidirTx::throttle_clamped(throttle).encode())
            .await
    }

    pub async fn send_frame(&mut self, frame: EncodedFrame) -> Result<(), PortRuntimeError> {
        self.prepare_bidir_frame(frame)?;
        self.run_tx_dma().await
    }

    pub async fn send_frame_and_receive(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<Result<TelemetryFrame, TelemetryPipelineError>, PortRuntimeError> {
        self.prepare_bidir_frame(frame)?;
        self.run_tx_then_capture().await?;
        self.decode_captured_frame()
    }

    async fn run_tx_dma(&mut self) -> Result<(), PortRuntimeError> {
        let tx_timeout = self.runtime_cfg.tx_timeout;
        let _session = TxPinSession::<T, D>::start(self)?;

        let result = with_timeout(tx_timeout, async {
            loop {
                if DmaStream::<D>::transfer_error() {
                    return Err(PortRuntimeError::TxDmaError);
                }
                if DmaStream::<D>::transfer_complete() {
                    return Ok(());
                }

                EmbassyTimer::after_micros(1).await;
            }
        })
        .await;

        match result {
            Ok(result) => result,
            Err(_) => Err(PortRuntimeError::TxTimeout),
        }
    }

    async fn run_tx_then_capture(&mut self) -> Result<(), PortRuntimeError> {
        let tx_timeout = self.runtime_cfg.tx_timeout;
        let rx_timeout = self.runtime_cfg.rx_timeout;
        let session = BidirCaptureSession::<T, D>::start(self)?;
        #[cfg(feature = "defmt")]
        {
            session.controller.tx_irq_started_at = Instant::now();
            session.controller.last_tx_done_irq_us = 0;
            session.controller.last_rx_armed_irq_us = 0;
            session.controller.last_rx_handoff_us = 0;
            session.controller.last_rx_capture_us = 0;
        }

        let tx_deadline = Instant::now() + tx_timeout;
        let rx_deadline = tx_deadline + rx_timeout;
        match session
            .wait_for_tx_handoff(tx_deadline.saturating_duration_since(Instant::now()))
            .await
        {
            Ok(()) => {}
            Err(err) => return Err(err),
        }

        session
            .wait_for_rx_done(rx_deadline.saturating_duration_since(Instant::now()))
            .await?;

        #[cfg(feature = "defmt")]
        {
            session.controller.last_rx_handoff_us = session
                .controller
                .last_rx_armed_irq_us
                .saturating_sub(session.controller.last_tx_done_irq_us);
        }

        Ok(())
    }

    unsafe fn dma_irq(ctx: *mut ()) {
        let this = &mut *(ctx as *mut Self);
        let regs = D::regs();
        let isr = regs.isr(D::stream_num() / 4).read();
        let bit = D::stream_num() % 4;

        if isr.teif(bit) {
            regs.ifcr(D::stream_num() / 4)
                .write(|w| w.set_teif(bit, true));
            this.timer.stop();
            this.timer.set_cc_dma_enable_state(this.channel, false);
            this.irq_state.transition(IrqPhase::Error);
            return;
        }

        if !isr.tcif(bit) {
            return;
        }

        regs.ifcr(D::stream_num() / 4)
            .write(|w| w.set_tcif(bit, true));
        this.timer.set_cc_dma_enable_state(this.channel, false);
        DmaStream::<D>::disable();

        match this.irq_state.load_phase() {
            IrqPhase::TxActive => {
                #[cfg(feature = "defmt")]
                {
                    this.last_tx_done_irq_us = Instant::now()
                        .saturating_duration_since(this.tx_irq_started_at)
                        .as_micros() as u32;
                }

                // Release the line immediately after TX DMA completion so the ESC can seize it
                // while we reconfigure the timer and DMA for RX sampling.
                this.pin.enter_input(this.capture_cfg.pull);
                switch_pacer_timer_config_fast(&this.timer, this.channel, this.rx_timer_cfg);
                DmaStream::<D>::start_prepared_read_no_reset(
                    this.rx_dma_cfg,
                    this.raw_samples.as_mut_ptr(),
                );
                this.irq_state.set_phase(IrqPhase::RxActive);
                this.timer.set_cc_dma_enable_state(this.channel, true);
                #[cfg(feature = "defmt")]
                {
                    this.last_rx_armed_irq_us = Instant::now()
                        .saturating_duration_since(this.tx_irq_started_at)
                        .as_micros() as u32;
                }
                this.irq_state.waker.wake();
            }
            IrqPhase::RxActive => {
                this.timer.stop();
                this.timer.set_cc_dma_enable_state(this.channel, false);
                #[cfg(feature = "defmt")]
                {
                    let now_us = Instant::now()
                        .saturating_duration_since(this.tx_irq_started_at)
                        .as_micros() as u32;
                    this.last_rx_capture_us = now_us.saturating_sub(this.last_rx_armed_irq_us);
                }
                this.irq_state.transition(IrqPhase::Done);
            }
            _ => {}
        }
    }
}

impl<'d, T, D> Stm32BidirPinController<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn prepare_bidir_frame(&mut self, frame: EncodedFrame) -> Result<(), PortRuntimeError> {
        self.tx_words = build_port_words(
            self.pin.pin_mask(),
            [self.pin.pin_mask()],
            [frame],
            SignalPolarity::Inverted,
        )
        .map_err(PortRuntimeError::Frame)?
        .words;
        Ok(())
    }

    #[cfg(feature = "defmt")]
    fn decode_captured_frame(
        &mut self,
    ) -> Result<Result<TelemetryFrame, TelemetryPipelineError>, PortRuntimeError> {
        self.capture_attempts = self.capture_attempts.wrapping_add(1);
        let sample_count = self.capture_cfg.sample_count;
        let mut samples_buf = [0u16; MAX_CAPTURE_SAMPLES];
        samples_buf[..sample_count].copy_from_slice(&self.raw_samples[..sample_count]);
        let samples = &samples_buf[..sample_count];
        let outcome = decode_frame_bf_strict_port_samples_with_debug_u16(
            &mut self.decoder,
            samples,
            self.pin.pin_mask() as u16,
        );

        if outcome.salvaged && (self.capture_attempts <= 8 || self.capture_attempts % 256 == 0) {
            defmt::info!(
                "proto bdshot salvage attempts={} bf_start={} bf_end={} bf_bits={} bf_raw=0x{:06x}",
                self.capture_attempts,
                outcome.debug.start_margin,
                outcome.debug.frame_end,
                outcome.debug.bits_found,
                outcome.debug.raw_21,
            );
        }

        if let Err(err) = outcome.frame {
            self.log_decode_error(samples, &outcome, err);
            return Ok(Err(err));
        }

        Ok(outcome.frame)
    }

    #[cfg(not(feature = "defmt"))]
    fn decode_captured_frame(
        &mut self,
    ) -> Result<Result<TelemetryFrame, TelemetryPipelineError>, PortRuntimeError> {
        Ok(decode_frame_bf_strict_port_samples_u16(
            &mut self.decoder,
            &self.raw_samples[..self.capture_cfg.sample_count],
            self.pin.pin_mask() as u16,
        ))
    }

    #[cfg(feature = "defmt")]
    fn log_decode_error(
        &mut self,
        samples: &[u16],
        outcome: &crate::bidir_capture::BfDecodeOutcome,
        err: TelemetryPipelineError,
    ) {
        self.decode_error_count = self.decode_error_count.wrapping_add(1);
        let count = self.decode_error_count;
        if count > 8 && count % 256 != 0 {
            return;
        }

        let summary = summarize_port_samples(samples, self.pin.pin_mask() as u16);
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
            "proto bdshot decode err attempts={} count={} err={} tx_done_irq_us={} rx_armed_irq_us={} handoff_us={} capture_us={} highs={} lows={} first_low={:?} edges={} first_edge={:?} last_edge={:?} hint_skip={} stats ok={} no_edge={} short={} invalid_frame={} invalid_gcr={} invalid_crc={} start_margin={} bf_start={} bf_end={} bf_bits={} bf_raw=0x{:06x} bf_raw_valid={} bf_payload=0x{:04x} bf_runs={},{},{},{},{},{},{},{},{},{},{},{} raw_head=0x{:04x},0x{:04x},0x{:04x},0x{:04x}",
            self.capture_attempts,
            count,
            err,
            self.last_tx_done_irq_us,
            self.last_rx_armed_irq_us,
            self.last_rx_handoff_us,
            self.last_rx_capture_us,
            summary.high_count,
            summary.low_count,
            summary.first_low,
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
            outcome.debug.runs[0],
            outcome.debug.runs[1],
            outcome.debug.runs[2],
            outcome.debug.runs[3],
            outcome.debug.runs[4],
            outcome.debug.runs[5],
            outcome.debug.runs[6],
            outcome.debug.runs[7],
            outcome.debug.runs[8],
            outcome.debug.runs[9],
            outcome.debug.runs[10],
            outcome.debug.runs[11],
            samples.first().copied().unwrap_or(0) as u32,
            samples.get(1).copied().unwrap_or(0) as u32,
            samples.get(2).copied().unwrap_or(0) as u32,
            samples.get(3).copied().unwrap_or(0) as u32,
        );
    }
}

struct DmaStream<D>(PhantomData<D>);

impl<D: RawDmaChannel> DmaStream<D> {
    fn transfer_complete() -> bool {
        let regs = D::regs();
        let stream = D::stream_num();
        regs.isr(stream / 4).read().tcif(stream % 4)
    }

    fn transfer_error() -> bool {
        let regs = D::regs();
        let stream = D::stream_num();
        regs.isr(stream / 4).read().teif(stream % 4)
    }

    fn clear_flags() {
        let regs = D::regs();
        let stream = D::stream_num();
        let idx = stream / 4;
        let bit = stream % 4;
        regs.ifcr(idx).write(|w| {
            w.set_htif(bit, true);
            w.set_tcif(bit, true);
            w.set_teif(bit, true);
        });
    }

    fn disable() {
        let regs = D::regs();
        let st = regs.st(D::stream_num());
        st.cr().modify(|w| w.set_en(false));
        while st.cr().read().en() {}
    }

    unsafe fn start_write(request: Request, mem_addr: *const u32, peri_addr: *mut u32, len: usize) {
        let regs = D::regs();
        let st = regs.st(D::stream_num());
        Self::disable();
        Self::clear_flags();

        st.par().write_value(peri_addr as u32);
        st.m0ar().write_value(mem_addr as u32);
        st.ndtr().write_value(pac::dma::regs::Ndtr(len as _));
        st.fcr()
            .write(|w| w.set_dmdis(pac::dma::vals::Dmdis::ENABLED));
        st.cr().write(|w| {
            w.set_dir(pac::dma::vals::Dir::MEMORY_TO_PERIPHERAL);
            w.set_msize(pac::dma::vals::Size::BITS32);
            w.set_psize(pac::dma::vals::Size::BITS32);
            w.set_pl(pac::dma::vals::Pl::VERY_HIGH);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_teie(false);
            w.set_tcie(false);
            w.set_htie(false);
            w.set_circ(false);
            w.set_chsel(request);
            w.set_pburst(pac::dma::vals::Burst::SINGLE);
            w.set_mburst(pac::dma::vals::Burst::SINGLE);
            w.set_pfctrl(pac::dma::vals::Pfctrl::DMA);
            w.set_en(true);
        });
    }

    unsafe fn start_write_irq(
        request: Request,
        mem_addr: *const u32,
        peri_addr: *mut u32,
        len: usize,
    ) {
        let regs = D::regs();
        let st = regs.st(D::stream_num());
        Self::disable();
        Self::clear_flags();

        st.par().write_value(peri_addr as u32);
        st.m0ar().write_value(mem_addr as u32);
        st.ndtr().write_value(pac::dma::regs::Ndtr(len as _));
        st.fcr()
            .write(|w| w.set_dmdis(pac::dma::vals::Dmdis::ENABLED));
        st.cr().write(|w| {
            w.set_dir(pac::dma::vals::Dir::MEMORY_TO_PERIPHERAL);
            w.set_msize(pac::dma::vals::Size::BITS32);
            w.set_psize(pac::dma::vals::Size::BITS32);
            w.set_pl(pac::dma::vals::Pl::VERY_HIGH);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_teie(true);
            w.set_tcie(true);
            w.set_htie(false);
            w.set_circ(false);
            w.set_chsel(request);
            w.set_pburst(pac::dma::vals::Burst::SINGLE);
            w.set_mburst(pac::dma::vals::Burst::SINGLE);
            w.set_pfctrl(pac::dma::vals::Pfctrl::DMA);
            w.set_en(true);
        });
    }

    unsafe fn start_prepared_read_no_reset(cfg: PreparedRxDmaConfig, mem_addr: *mut u16) {
        let regs = D::regs();
        let st = regs.st(D::stream_num());

        st.par().write_value(cfg.peri_addr as u32);
        st.m0ar().write_value(mem_addr as u32);
        st.ndtr().write_value(pac::dma::regs::Ndtr(cfg.len as _));
        st.fcr()
            .write(|w| w.set_dmdis(pac::dma::vals::Dmdis::ENABLED));
        st.cr().write(|w| {
            w.set_dir(pac::dma::vals::Dir::PERIPHERAL_TO_MEMORY);
            w.set_msize(pac::dma::vals::Size::BITS16);
            w.set_psize(pac::dma::vals::Size::BITS16);
            w.set_pl(pac::dma::vals::Pl::VERY_HIGH);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_teie(true);
            w.set_tcie(true);
            w.set_htie(false);
            w.set_circ(false);
            w.set_chsel(cfg.request);
            w.set_pburst(pac::dma::vals::Burst::SINGLE);
            w.set_mburst(pac::dma::vals::Burst::SINGLE);
            w.set_pfctrl(pac::dma::vals::Pfctrl::DMA);
            w.set_en(true);
        });
    }
}

struct TxPortSession<'a, 'd, T, D, const N: usize>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    controller: &'a mut Stm32TxPortController<'d, T, D, N>,
}

impl<'a, 'd, T, D, const N: usize> TxPortSession<'a, 'd, T, D, N>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn start(
        controller: &'a mut Stm32TxPortController<'d, T, D, N>,
    ) -> Result<Self, PortRuntimeError> {
        configure_pacer_timer(
            &controller.timer,
            controller.channel,
            controller.tx_timer_cfg,
        );
        controller
            .timer
            .set_cc_dma_enable_state(controller.channel, false);
        controller.timer.reset();

        for pin in &mut controller.pins {
            pin.enter_output_low();
        }

        unsafe {
            DmaStream::<D>::start_write(
                controller.dma_request,
                controller.tx_words.as_ptr(),
                controller.pins[0].bsrr_ptr(),
                controller.tx_words.len(),
            );
        }

        controller
            .timer
            .set_cc_dma_enable_state(controller.channel, true);
        controller.timer.start();

        Ok(Self { controller })
    }
}

impl<T, D, const N: usize> Drop for TxPortSession<'_, '_, T, D, N>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn drop(&mut self) {
        DmaStream::<D>::disable();
        DmaStream::<D>::clear_flags();
        self.controller.timer.stop();
        self.controller
            .timer
            .set_cc_dma_enable_state(self.controller.channel, false);
        for pin in &mut self.controller.pins {
            pin.enter_output_low();
        }
    }
}

struct TxPinSession<'a, 'd, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    controller: &'a mut Stm32BidirPinController<'d, T, D>,
}

impl<'a, 'd, T, D> TxPinSession<'a, 'd, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn start(
        controller: &'a mut Stm32BidirPinController<'d, T, D>,
    ) -> Result<Self, PortRuntimeError> {
        configure_pacer_timer(
            &controller.timer,
            controller.channel,
            controller.tx_timer_cfg,
        );
        controller
            .timer
            .set_cc_dma_enable_state(controller.channel, false);
        controller.timer.reset();
        controller.pin.enter_output_high();

        unsafe {
            DmaStream::<D>::start_write(
                controller.dma_request,
                controller.tx_words.as_ptr(),
                controller.pin.bsrr_ptr(),
                controller.tx_words.len(),
            );
        }

        controller
            .timer
            .set_cc_dma_enable_state(controller.channel, true);
        controller.timer.start();

        Ok(Self { controller })
    }
}

impl<T, D> Drop for TxPinSession<'_, '_, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn drop(&mut self) {
        DmaStream::<D>::disable();
        DmaStream::<D>::clear_flags();
        self.controller.timer.stop();
        self.controller
            .timer
            .set_cc_dma_enable_state(self.controller.channel, false);
        self.controller
            .pin
            .enter_input(self.controller.capture_cfg.pull);
        self.controller.irq_state.set_phase(IrqPhase::Idle);
    }
}

struct BidirCaptureSession<'a, 'd, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    controller: &'a mut Stm32BidirPinController<'d, T, D>,
}

impl<'a, 'd, T, D> BidirCaptureSession<'a, 'd, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn start(
        controller: &'a mut Stm32BidirPinController<'d, T, D>,
    ) -> Result<Self, PortRuntimeError> {
        configure_pacer_timer(
            &controller.timer,
            controller.channel,
            controller.tx_timer_cfg,
        );
        controller
            .timer
            .set_cc_dma_enable_state(controller.channel, false);
        controller.timer.reset();
        controller.irq_state.set_phase(IrqPhase::TxActive);
        controller.pin.enter_output_high();

        unsafe {
            DMA_IRQ_CTX[D::IRQ_SLOT].store(controller as *mut _ as *mut (), Ordering::Release);
            DMA_IRQ_FN[D::IRQ_SLOT].store(
                Stm32BidirPinController::<T, D>::dma_irq as *mut (),
                Ordering::Release,
            );
            DmaStream::<D>::start_write_irq(
                controller.dma_request,
                controller.tx_words.as_ptr(),
                controller.pin.bsrr_ptr(),
                controller.tx_words.len(),
            );
            controller
                .timer
                .set_cc_dma_enable_state(controller.channel, true);
            controller.timer.start();
        }

        Ok(Self { controller })
    }

    async fn wait_for_tx_handoff(&self, timeout: Duration) -> Result<(), PortRuntimeError> {
        with_timeout(
            timeout,
            poll_fn(|cx| {
                self.controller.irq_state.register(cx.waker());
                match self.controller.irq_state.load_phase() {
                    IrqPhase::TxActive => core::task::Poll::Pending,
                    IrqPhase::RxActive | IrqPhase::Done => core::task::Poll::Ready(Ok(())),
                    IrqPhase::Error => core::task::Poll::Ready(Err(PortRuntimeError::TxDmaError)),
                    IrqPhase::Idle => core::task::Poll::Ready(Err(PortRuntimeError::TxTimeout)),
                }
            }),
        )
        .await
        .map_err(|_| PortRuntimeError::TxTimeout)?
    }

    async fn wait_for_rx_done(&self, timeout: Duration) -> Result<(), PortRuntimeError> {
        with_timeout(
            timeout,
            poll_fn(|cx| {
                self.controller.irq_state.register(cx.waker());
                match self.controller.irq_state.load_phase() {
                    IrqPhase::RxActive => core::task::Poll::Pending,
                    IrqPhase::Done => core::task::Poll::Ready(Ok(())),
                    IrqPhase::Error => core::task::Poll::Ready(Err(PortRuntimeError::TxDmaError)),
                    IrqPhase::Idle | IrqPhase::TxActive => {
                        core::task::Poll::Ready(Err(PortRuntimeError::RxTimeout))
                    }
                }
            }),
        )
        .await
        .map_err(|_| PortRuntimeError::RxTimeout)?
    }
}

impl<T, D> Drop for BidirCaptureSession<'_, '_, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn drop(&mut self) {
        DMA_IRQ_CTX[D::IRQ_SLOT].store(ptr::null_mut(), Ordering::Release);
        DMA_IRQ_FN[D::IRQ_SLOT].store(ptr::null_mut(), Ordering::Release);
        DmaStream::<D>::disable();
        DmaStream::<D>::clear_flags();
        self.controller.timer.stop();
        self.controller
            .timer
            .set_cc_dma_enable_state(self.controller.channel, false);
        self.controller
            .pin
            .enter_input(self.controller.capture_cfg.pull);
        self.controller.irq_state.set_phase(IrqPhase::Idle);
    }
}

fn configure_pacer_timer<T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    cfg: PacerTimerConfig,
) {
    timer.stop();
    timer.set_cc_dma_enable_state(channel, false);
    timer.reset();
    timer.set_output_compare_mode(channel, OutputCompareMode::PwmMode1);
    timer.set_output_compare_preload(channel, true);
    timer.set_output_polarity(channel, OutputPolarity::ActiveHigh);
    timer.enable_channel(channel, true);
    apply_pacer_timer_config_fast(timer, channel, cfg);
    timer.reset();
    let _ = timer.clear_update_interrupt();
}

fn apply_pacer_timer_config_fast<T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    cfg: PacerTimerConfig,
) {
    let regs = timer.regs_gp16();
    regs.psc().write_value(cfg.psc);
    regs.arr().write(|r| r.set_arr(cfg.arr.into()));
    timer.set_compare_value(channel, cfg.compare.into());
    timer.generate_update_event();
    let _ = timer.clear_update_interrupt();
}

fn switch_pacer_timer_config_fast<T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    cfg: PacerTimerConfig,
) {
    let regs = timer.regs_gp16();

    if regs.psc().read() != cfg.psc {
        regs.psc().write_value(cfg.psc);
    }
    regs.arr().write(|r| r.set_arr(cfg.arr.into()));
    timer.set_compare_value(channel, cfg.compare.into());
    timer.generate_update_event();
}

fn compute_pacer_timer_config<T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    hz: u32,
    compare_percent: u8,
) -> PacerTimerConfig {
    let timer_hz = timer.get_clock_frequency().0 as u64;
    let target_hz = hz as u64;
    let total_ticks = (timer_hz + (target_hz / 2))
        .saturating_div(target_hz)
        .max(1);

    let mut psc = ((total_ticks.saturating_sub(1)) / (u16::MAX as u64 + 1)).min(u16::MAX as u64);
    let mut arr = (total_ticks / (psc + 1)).saturating_sub(1);
    if arr > u16::MAX as u64 {
        psc = psc.saturating_add(1);
        arr = (total_ticks / (psc + 1)).saturating_sub(1);
    }

    let arr = arr.clamp(0, u16::MAX as u64) as u16;
    let period_ticks = u32::from(arr).saturating_add(1);
    let compare = ((period_ticks.saturating_mul(compare_percent.clamp(1, 99) as u32)) / 100)
        .clamp(1, period_ticks.max(1)) as u16;

    PacerTimerConfig {
        psc: psc as u16,
        arr,
        compare,
    }
}

fn compute_rx_timer_config<T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    speed: DshotSpeed,
    runtime_cfg: BidirPortRuntimeConfig,
    capture_cfg: BidirCaptureConfig,
) -> PacerTimerConfig {
    // Match Betaflight's telemetry input pacing:
    // inputFreq = outputFreq * 5 * 2 * oversample / 24
    // For the BF default oversample=3, this becomes outputFreq * 5 / 4.
    let symbol_rate_hz = speed.timing_hints().nominal_bitrate_hz;
    let mut rx_sample_hz = symbol_rate_hz * 5 * capture_cfg.oversampling.oversampling as u32 / 4;
    rx_sample_hz =
        rx_sample_hz.saturating_mul(runtime_cfg.rx_sample_percent.clamp(1, 200) as u32) / 100;
    compute_pacer_timer_config(timer, rx_sample_hz, runtime_cfg.rx_compare_percent)
}

#[cfg(feature = "defmt")]
struct SampleSummary {
    high_count: usize,
    low_count: usize,
    edge_count: usize,
    first_low: Option<usize>,
    first_edge: Option<usize>,
    last_edge: Option<usize>,
}

#[cfg(feature = "defmt")]
fn summarize_levels<I>(iter: I) -> SampleSummary
where
    I: Iterator<Item = bool>,
{
    let mut high_count = 0usize;
    let mut low_count = 0usize;
    let mut edge_count = 0usize;
    let mut first_low = None;
    let mut first_edge = None;
    let mut last_edge = None;
    let mut prev = None;

    for (idx, level) in iter.enumerate() {
        if level {
            high_count += 1;
        } else {
            low_count += 1;
            if first_low.is_none() {
                first_low = Some(idx);
            }
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
        first_low,
        first_edge,
        last_edge,
    }
}

#[cfg(feature = "defmt")]
fn summarize_port_samples(samples: &[u16], bit_mask: u16) -> SampleSummary {
    summarize_levels(
        samples
            .iter()
            .copied()
            .map(|sample| (sample & bit_mask) != 0),
    )
}
