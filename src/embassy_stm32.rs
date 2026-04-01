use core::array;
use core::future::poll_fn;
use core::marker::PhantomData;
use core::ptr;
use core::slice;
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
use embassy_time::{with_timeout, Duration, Instant, Timer as EmbassyTimer};

use crate::bidir_capture::decode_frame_bf_strict_port_samples_u16;
use crate::telemetry::{
    BidirDecoder, OversamplingConfig, PreambleTuningConfig, TelemetryFrame, TelemetryPipelineError,
};
use crate::{BidirTx, Command, DshotSpeed, EncodedFrame, UniTx};

const MAX_PORT_MOTORS: usize = 4;
const MAX_CAPTURE_SAMPLES: usize = 512;
const PINS_PER_GPIO_PORT: u8 = 16;
const DEFAULT_ARM_DURATION: Duration = Duration::from_millis(3_000);
const PREAMBLE_MARGIN_SAMPLES: usize = 64;
const DMA_IRQ_SLOTS: usize = 16;

type DmaIrqFn = unsafe fn(*mut ());

struct IrqSlot {
    func: AtomicPtr<()>,
    ctx: AtomicPtr<()>,
}

impl IrqSlot {
    const fn new() -> Self {
        Self {
            func: AtomicPtr::new(ptr::null_mut()),
            ctx: AtomicPtr::new(ptr::null_mut()),
        }
    }

    fn install(&self, ctx: *mut (), irq_fn: DmaIrqFn) {
        // Store ctx first so it is visible before func (the presence flag).
        self.ctx.store(ctx, Ordering::Release);
        self.func.store(irq_fn as *mut (), Ordering::Release);
    }

    fn clear(&self) {
        self.func.store(ptr::null_mut(), Ordering::Release);
        self.ctx.store(ptr::null_mut(), Ordering::Release);
    }

    /// Dispatch the installed IRQ handler, if any.
    ///
    /// # Safety
    /// The installed `ctx` pointer must still be valid.
    #[inline(always)]
    unsafe fn dispatch(&self) {
        let func = self.func.load(Ordering::Acquire);
        if !func.is_null() {
            let ctx = self.ctx.load(Ordering::Acquire);
            let f: DmaIrqFn = unsafe { core::mem::transmute(func) };
            f(ctx);
        }
    }
}

static DMA_IRQ_SLOTS_TABLE: [IrqSlot; DMA_IRQ_SLOTS] = [const { IrqSlot::new() }; DMA_IRQ_SLOTS];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DshotError {
    TxTimeout,
    TxDmaError,
    RxDmaError,
    RxTimeout,
    Frame(PortFrameError),
    Telemetry(TelemetryPipelineError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DshotConfig {
    pub speed: DshotSpeed,
    pub tx_timeout: Duration,
    pub rx_timeout: Duration,
    pub pacer_compare_percent: u8,
    // Bidir specific
    pub oversampling: OversamplingConfig,
    pub preamble_tuning: PreambleTuningConfig,
    pub pull: Pull,
    pub rx_compare_percent: u8,
    pub rx_sample_percent: u8,
}

impl DshotConfig {
    pub fn new(speed: DshotSpeed) -> Self {
        Self {
            speed,
            tx_timeout: Duration::from_millis(2),
            rx_timeout: Duration::from_millis(2),
            pacer_compare_percent: 50,
            oversampling: OversamplingConfig::default(),
            preamble_tuning: PreambleTuningConfig::default(),
            pull: Pull::Up,
            rx_compare_percent: 50,
            rx_sample_percent: 100,
        }
    }

    pub fn with_tx_timeout(mut self, timeout: Duration) -> Self {
        self.tx_timeout = timeout;
        self
    }

    pub fn with_rx_timeout(mut self, timeout: Duration) -> Self {
        self.rx_timeout = timeout;
        self
    }

    pub fn with_pacer_compare_percent(mut self, percent: u8) -> Self {
        self.pacer_compare_percent = percent.clamp(1, 99);
        self
    }

    pub fn with_bidir_pull(mut self, pull: Pull) -> Self {
        self.pull = pull;
        self
    }

    pub fn with_oversampling(mut self, oversampling: OversamplingConfig) -> Self {
        self.oversampling = oversampling;
        self
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

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
enum IrqPhase {
    Idle = 0,
    TxActive = 1,
    RxActive = 2,
    Done = 3,
    TxError = 4,
    RxError = 5,
}

impl IrqPhase {
    fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Idle,
            1 => Self::TxActive,
            2 => Self::RxActive,
            3 => Self::Done,
            4 => Self::TxError,
            5 => Self::RxError,
            _ => Self::TxError,
        }
    }
}

enum PhaseDisposition {
    Pending,
    Done,
    Error(DshotError),
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
        IrqPhase::from_u8(self.phase.load(Ordering::Acquire))
    }

    fn set_phase(&self, phase: IrqPhase) {
        self.phase.store(phase as u8, Ordering::Release);
    }

    fn transition(&self, phase: IrqPhase) {
        self.set_phase(phase);
        self.waker.wake();
    }

    async fn wait_for_phase(
        &self,
        timeout: Duration,
        timeout_err: DshotError,
        classify: impl Fn(IrqPhase) -> PhaseDisposition,
    ) -> Result<(), DshotError> {
        with_timeout(
            timeout,
            poll_fn(|cx| {
                self.waker.register(cx.waker());
                match classify(self.load_phase()) {
                    PhaseDisposition::Pending => core::task::Poll::Pending,
                    PhaseDisposition::Done => core::task::Poll::Ready(Ok(())),
                    PhaseDisposition::Error(e) => core::task::Poll::Ready(Err(e)),
                }
            }),
        )
        .await
        .map_err(|_| timeout_err)?
    }

    async fn wait_done(&self, timeout: Duration) -> Result<(), DshotError> {
        self.wait_for_phase(timeout, DshotError::TxTimeout, |phase| match phase {
            IrqPhase::TxActive | IrqPhase::RxActive => PhaseDisposition::Pending,
            IrqPhase::Done => PhaseDisposition::Done,
            IrqPhase::TxError => PhaseDisposition::Error(DshotError::TxDmaError),
            IrqPhase::RxError => PhaseDisposition::Error(DshotError::RxDmaError),
            IrqPhase::Idle => PhaseDisposition::Error(DshotError::TxTimeout),
        })
        .await
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
        DMA_IRQ_SLOTS_TABLE[D::IRQ_SLOT].dispatch();
    }
}

macro_rules! impl_raw_dma_channel {
    ($periph:ty, $regs:expr, $irq_slot:expr, $stream_num:expr) => {
        impl RawDmaChannel for $periph {
            const IRQ_SLOT: usize = $irq_slot;

            fn regs() -> pac::dma::Dma {
                unsafe { pac::dma::Dma::from_ptr($regs.as_ptr()) }
            }

            fn stream_num() -> usize {
                $stream_num
            }
        }
    };
}

impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA1_CH1, pac::DMA1, 1, 1);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA1_CH2, pac::DMA1, 2, 2);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA1_CH3, pac::DMA1, 3, 3);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA1_CH4, pac::DMA1, 4, 4);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA1_CH5, pac::DMA1, 5, 5);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA1_CH6, pac::DMA1, 6, 6);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA1_CH7, pac::DMA1, 7, 7);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH1, pac::DMA2, 9, 1);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH2, pac::DMA2, 10, 2);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH3, pac::DMA2, 11, 3);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH4, pac::DMA2, 12, 4);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH5, pac::DMA2, 13, 5);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH6, pac::DMA2, 14, 6);

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
                dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
                pins: [DshotPortPin<'d>; N],
                speed: DshotSpeed,
            ) -> Result<Self, PortConfigError>
            where
                D: Dma<T, $channel>,
            {
                Self::new_inner::<$channel>(timer, dma, dma_irq, pins, speed)
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
                speed: DshotSpeed,
            ) -> Result<Self, PortConfigError>
            where
                D: Dma<T, $channel>,
            {
                Self::new_inner::<$channel>(timer, dma, dma_irq, pin, speed)
            }
        )+
    };
}

pub struct Stm32DshotPort<'d, T, D, const N: usize>
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
    config: DshotConfig,
    tx_timer_cfg: PacerTimerConfig,
    tx_words: [u32; TX_STATE_SLOTS],
    irq_state: IrqWakerState,
    _dma: PhantomData<D>,
}

impl<'d, T, D, const N: usize> Stm32DshotPort<'d, T, D, N>
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

    pub fn set_config(&mut self, config: DshotConfig) {
        self.config = config;
        self.tx_timer_cfg = compute_pacer_timer_config(
            &self.timer,
            self.config.speed.timing_hints().nominal_bitrate_hz * 3,
            self.config.pacer_compare_percent,
        );
        configure_pacer_timer(&self.timer, self.channel, self.tx_timer_cfg);
    }

    fn new_inner<C>(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        _dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
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
        let config = DshotConfig::new(speed);
        let tx_timer_cfg = compute_pacer_timer_config(
            &timer,
            speed.timing_hints().nominal_bitrate_hz * 3,
            config.pacer_compare_percent,
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
            config,
            tx_timer_cfg,
            tx_words: [0; TX_STATE_SLOTS],
            irq_state: IrqWakerState::new(),
            _dma: PhantomData,
        })
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), DshotError> {
        let frame_period =
            Duration::from_micros(self.config.speed.timing_hints().min_frame_period_us as u64);
        let stop_frames = [UniTx::command(Command::MotorStop).encode(); N];
        let deadline = Instant::now() + duration;

        while Instant::now() < deadline {
            self.send_frames(stop_frames).await?;
            EmbassyTimer::after(frame_period).await;
        }

        Ok(())
    }

    pub async fn arm(&mut self) -> Result<(), DshotError> {
        self.arm_for(DEFAULT_ARM_DURATION).await
    }

    pub async fn send_throttles(&mut self, throttles: [u16; N]) -> Result<(), DshotError> {
        self.send_frames(throttles.map(|throttle| UniTx::throttle_clamped(throttle).encode()))
            .await
    }

    pub async fn send_frames(&mut self, frames: [EncodedFrame; N]) -> Result<(), DshotError> {
        let port_words = build_port_words(
            self.group_mask,
            self.pin_masks,
            frames,
            SignalPolarity::Normal,
        )
        .map_err(DshotError::Frame)?;
        self.tx_words = port_words.words;
        self.run_tx_dma().await
    }

    async fn run_tx_dma(&mut self) -> Result<(), DshotError> {
        let tx_timeout = self.config.tx_timeout;
        let session = TxPortSession::<T, D, N>::start(self)?;
        session.wait_done(tx_timeout).await
    }

    unsafe fn tx_dma_irq(ctx: *mut ()) {
        let this = &mut *(ctx as *mut Self);
        handle_tx_complete_irq::<D, T>(&this.timer, this.channel, &this.irq_state);
    }
}

pub struct Stm32BidirDshotPort<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    timer: Timer<'d, T>,
    dma_request: Request,
    pin: DshotPortPin<'d>,
    channel: Channel,
    config: DshotConfig,
    tx_timer_cfg: PacerTimerConfig,
    rx_timer_cfg: PacerTimerConfig,
    rx_dma_cfg: PreparedRxDmaConfig,
    decoder: BidirDecoder,
    tx_words: [u32; TX_STATE_SLOTS],
    raw_samples: [u16; MAX_CAPTURE_SAMPLES],
    irq_state: IrqWakerState,
    _dma: PhantomData<D>,
}

impl<'d, T, D> Stm32BidirDshotPort<'d, T, D>
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

    pub fn set_config(&mut self, config: DshotConfig) {
        self.config = config;
        self.tx_timer_cfg = compute_pacer_timer_config(
            &self.timer,
            self.config.speed.timing_hints().nominal_bitrate_hz * 3,
            self.config.pacer_compare_percent,
        );
        self.rx_timer_cfg = compute_rx_timer_config(&self.timer, &self.config);
        configure_pacer_timer(&self.timer, self.channel, self.tx_timer_cfg);

        // Update decoder configuration
        self.decoder = BidirDecoder::with_preamble_tuning(
            self.config.oversampling,
            self.config.preamble_tuning,
        );

        let sample_count = self
            .config
            .oversampling
            .recommended_capture_samples(PREAMBLE_MARGIN_SAMPLES);
        self.rx_dma_cfg = PreparedRxDmaConfig {
            request: self.dma_request,
            peri_addr: self.pin.idr_ptr(),
            len: sample_count,
        };
    }

    fn new_inner<C>(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        _dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
        mut pin: DshotPortPin<'d>,
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        C: TimerChannel,
        D: Dma<T, C>,
    {
        let dma_request = dma.request();
        dma.remap();
        drop(dma);

        let timer = Timer::new(timer);
        let config = DshotConfig::new(speed);
        let tx_timer_cfg = compute_pacer_timer_config(
            &timer,
            speed.timing_hints().nominal_bitrate_hz * 3,
            config.pacer_compare_percent,
        );
        let rx_timer_cfg = compute_rx_timer_config(&timer, &config);
        configure_pacer_timer(&timer, C::CHANNEL, tx_timer_cfg);
        pin.enter_input(config.pull);
        let pin_idr_ptr = pin.idr_ptr();

        let decoder =
            BidirDecoder::with_preamble_tuning(config.oversampling, config.preamble_tuning);

        let sample_count = config
            .oversampling
            .recommended_capture_samples(PREAMBLE_MARGIN_SAMPLES);
        if sample_count > MAX_CAPTURE_SAMPLES {
            return Err(PortConfigError::SampleBufferTooSmall {
                requested: sample_count,
                capacity: MAX_CAPTURE_SAMPLES,
            });
        }

        Ok(Self {
            timer,
            dma_request,
            pin,
            channel: C::CHANNEL,
            config,
            tx_timer_cfg,
            rx_timer_cfg,
            rx_dma_cfg: PreparedRxDmaConfig {
                request: dma_request,
                peri_addr: pin_idr_ptr,
                len: sample_count,
            },
            decoder,
            tx_words: [0; TX_STATE_SLOTS],
            raw_samples: [0; MAX_CAPTURE_SAMPLES],
            irq_state: IrqWakerState::new(),
            _dma: PhantomData,
        })
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), DshotError> {
        let frame_period =
            Duration::from_micros(self.config.speed.timing_hints().min_frame_period_us as u64);
        let stop_frame = BidirTx::command(Command::MotorStop).encode();
        let deadline = Instant::now() + duration;

        while Instant::now() < deadline {
            self.send_frame(stop_frame).await?;
            EmbassyTimer::after(frame_period).await;
        }

        Ok(())
    }

    pub async fn arm(&mut self) -> Result<(), DshotError> {
        self.arm_for(DEFAULT_ARM_DURATION).await
    }

    pub async fn send_throttle_and_receive(
        &mut self,
        throttle: u16,
    ) -> Result<TelemetryFrame, DshotError> {
        self.send_frame_and_receive(BidirTx::throttle_clamped(throttle).encode())
            .await
    }

    pub async fn send_frame(&mut self, frame: EncodedFrame) -> Result<(), DshotError> {
        self.prepare_bidir_frame(frame)?;
        self.run_tx_dma().await
    }

    pub async fn send_frame_and_receive(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<TelemetryFrame, DshotError> {
        self.prepare_bidir_frame(frame)?;
        self.run_tx_then_capture().await?;
        self.decode_captured_frame()
    }

    async fn run_tx_dma(&mut self) -> Result<(), DshotError> {
        let tx_timeout = self.config.tx_timeout;
        let session = TxPinSession::<T, D>::start(self)?;
        session.wait_done(tx_timeout).await
    }

    unsafe fn tx_only_dma_irq(ctx: *mut ()) {
        let this = &mut *(ctx as *mut Self);
        handle_tx_complete_irq::<D, T>(&this.timer, this.channel, &this.irq_state);
    }

    async fn run_tx_then_capture(&mut self) -> Result<(), DshotError> {
        let tx_timeout = self.config.tx_timeout;
        let rx_timeout = self.config.rx_timeout;
        let total_timeout = tx_timeout + rx_timeout;
        let session = BidirCaptureSession::<T, D>::start(self)?;

        session.wait_done(total_timeout).await?;

        Ok(())
    }

    unsafe fn dma_irq(ctx: *mut ()) {
        let this = &mut *(ctx as *mut Self);
        match check_and_clear_dma_irq_flags::<D>() {
            Some(false) => {
                this.timer.stop();
                this.timer.set_cc_dma_enable_state(this.channel, false);
                DmaStream::<D>::disable();
                let error_phase = match this.irq_state.load_phase() {
                    IrqPhase::RxActive => IrqPhase::RxError,
                    _ => IrqPhase::TxError,
                };
                this.irq_state.transition(error_phase);
            }
            Some(true) => {
                this.timer.set_cc_dma_enable_state(this.channel, false);
                DmaStream::<D>::disable();
                match this.irq_state.load_phase() {
                    IrqPhase::TxActive => {
                        this.pin.enter_input(this.config.pull);
                        switch_pacer_timer_config_fast(
                            &this.timer,
                            this.channel,
                            this.rx_timer_cfg,
                        );
                        DmaStream::<D>::start_prepared_read_no_reset(
                            this.rx_dma_cfg,
                            this.raw_samples.as_mut_ptr(),
                        );
                        this.irq_state.set_phase(IrqPhase::RxActive);
                        this.timer.set_cc_dma_enable_state(this.channel, true);
                    }
                    IrqPhase::RxActive => {
                        this.timer.stop();
                        this.timer.set_cc_dma_enable_state(this.channel, false);
                        this.irq_state.transition(IrqPhase::Done);
                    }
                    _ => {}
                }
            }
            None => {}
        }
    }
}

impl<'d, T, D> Stm32BidirDshotPort<'d, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn prepare_bidir_frame(&mut self, frame: EncodedFrame) -> Result<(), DshotError> {
        self.tx_words = build_port_words(
            self.pin.pin_mask(),
            [self.pin.pin_mask()],
            [frame],
            SignalPolarity::Inverted,
        )
        .map_err(DshotError::Frame)?
        .words;
        Ok(())
    }

    fn decode_captured_frame(&mut self) -> Result<TelemetryFrame, DshotError> {
        decode_frame_bf_strict_port_samples_u16(
            &mut self.decoder,
            &self.raw_samples[..self.rx_dma_cfg.len],
            self.pin.pin_mask() as u16,
        )
        .map_err(DshotError::Telemetry)
    }
}

struct DmaStream<D>(PhantomData<D>);

#[derive(Clone, Copy)]
enum DmaInterruptMode {
    Irq,
}

impl<D: RawDmaChannel> DmaStream<D> {
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

    unsafe fn configure_and_start(
        request: Request,
        mem_addr: u32,
        peri_addr: u32,
        len: usize,
        dir: pac::dma::vals::Dir,
        size: pac::dma::vals::Size,
        interrupts: DmaInterruptMode,
        reset: bool,
    ) {
        let regs = D::regs();
        let st = regs.st(D::stream_num());

        if reset {
            Self::disable();
            Self::clear_flags();
        }

        st.par().write_value(peri_addr);
        st.m0ar().write_value(mem_addr);
        st.ndtr().write_value(pac::dma::regs::Ndtr(len as _));
        st.fcr()
            .write(|w| w.set_dmdis(pac::dma::vals::Dmdis::ENABLED));
        st.cr().write(|w| {
            w.set_dir(dir);
            w.set_msize(size);
            w.set_psize(size);
            w.set_pl(pac::dma::vals::Pl::VERY_HIGH);
            w.set_minc(true);
            w.set_pinc(false);
            w.set_teie(matches!(interrupts, DmaInterruptMode::Irq));
            w.set_tcie(matches!(interrupts, DmaInterruptMode::Irq));
            w.set_htie(false);
            w.set_circ(false);
            w.set_chsel(request);
            w.set_pburst(pac::dma::vals::Burst::SINGLE);
            w.set_mburst(pac::dma::vals::Burst::SINGLE);
            w.set_pfctrl(pac::dma::vals::Pfctrl::DMA);
            w.set_en(true);
        });
    }

    unsafe fn start_write(
        request: Request,
        mem_addr: *const u32,
        peri_addr: *mut u32,
        len: usize,
        interrupts: DmaInterruptMode,
    ) {
        Self::configure_and_start(
            request,
            mem_addr as u32,
            peri_addr as u32,
            len,
            pac::dma::vals::Dir::MEMORY_TO_PERIPHERAL,
            pac::dma::vals::Size::BITS32,
            interrupts,
            true,
        );
    }

    unsafe fn start_prepared_read_no_reset(cfg: PreparedRxDmaConfig, mem_addr: *mut u16) {
        Self::configure_and_start(
            cfg.request,
            mem_addr as u32,
            cfg.peri_addr as u32,
            cfg.len,
            pac::dma::vals::Dir::PERIPHERAL_TO_MEMORY,
            pac::dma::vals::Size::BITS16,
            DmaInterruptMode::Irq,
            false,
        );
    }
}

#[inline(always)]
fn check_and_clear_dma_irq_flags<D: RawDmaChannel>() -> Option<bool> {
    let regs = D::regs();
    let isr = regs.isr(D::stream_num() / 4).read();
    let bit = D::stream_num() % 4;

    if isr.teif(bit) {
        regs.ifcr(D::stream_num() / 4)
            .write(|w| w.set_teif(bit, true));
        return Some(false);
    }
    if isr.tcif(bit) {
        regs.ifcr(D::stream_num() / 4)
            .write(|w| w.set_tcif(bit, true));
        return Some(true);
    }
    None
}

#[inline(always)]
fn handle_tx_complete_irq<D: RawDmaChannel, T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    irq_state: &IrqWakerState,
) {
    match check_and_clear_dma_irq_flags::<D>() {
        Some(false) => {
            timer.stop();
            timer.set_cc_dma_enable_state(channel, false);
            DmaStream::<D>::disable();
            irq_state.transition(IrqPhase::TxError);
        }
        Some(true) => {
            timer.set_cc_dma_enable_state(channel, false);
            DmaStream::<D>::disable();
            timer.stop();
            irq_state.transition(IrqPhase::Done);
        }
        None => {}
    }
}

fn arm_timer_for_dma_transfer<T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    cfg: PacerTimerConfig,
) {
    configure_pacer_timer(timer, channel, cfg);
    timer.set_cc_dma_enable_state(channel, false);
    timer.reset();
}

fn start_timer_dma_transfer<T: GeneralInstance4Channel>(timer: &Timer<'_, T>, channel: Channel) {
    timer.set_cc_dma_enable_state(channel, true);
    timer.start();
}

fn stop_timer_dma_transfer<T: GeneralInstance4Channel>(timer: &Timer<'_, T>, channel: Channel) {
    timer.stop();
    timer.set_cc_dma_enable_state(channel, false);
}

fn install_irq_ctx<D: RawDmaChannel>(ctx: *mut (), irq_fn: DmaIrqFn) {
    DMA_IRQ_SLOTS_TABLE[D::IRQ_SLOT].install(ctx, irq_fn);
}

fn clear_irq_ctx<D: RawDmaChannel>() {
    DMA_IRQ_SLOTS_TABLE[D::IRQ_SLOT].clear();
}

fn teardown_dma_session<D: RawDmaChannel, T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    irq_state: &IrqWakerState,
) {
    clear_irq_ctx::<D>();
    DmaStream::<D>::disable();
    DmaStream::<D>::clear_flags();
    stop_timer_dma_transfer(timer, channel);
    irq_state.set_phase(IrqPhase::Idle);
}

unsafe fn begin_tx_dma_session<D: RawDmaChannel, T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    timer_cfg: PacerTimerConfig,
    irq_state: &IrqWakerState,
    irq_fn: DmaIrqFn,
    ctx: *mut (),
    dma_request: Request,
    tx_words: &[u32],
    bsrr_ptr: *mut u32,
) {
    arm_timer_for_dma_transfer(timer, channel, timer_cfg);
    irq_state.set_phase(IrqPhase::TxActive);
    install_irq_ctx::<D>(ctx, irq_fn);
    DmaStream::<D>::start_write(
        dma_request,
        tx_words.as_ptr(),
        bsrr_ptr,
        tx_words.len(),
        DmaInterruptMode::Irq,
    );
    start_timer_dma_transfer(timer, channel);
}

fn teardown_session_with_pins<'d, D: RawDmaChannel, T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    irq_state: &IrqWakerState,
    pins: &mut [DshotPortPin<'d>],
    reset_to_input: Option<Pull>,
) {
    teardown_dma_session::<D, T>(timer, channel, irq_state);
    for pin in pins {
        if let Some(pull) = reset_to_input {
            pin.enter_input(pull);
        } else {
            pin.enter_output_low();
        }
    }
}

struct TxPortSession<'a, 'd, T, D, const N: usize>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    controller: &'a mut Stm32DshotPort<'d, T, D, N>,
}

impl<'a, 'd, T, D, const N: usize> TxPortSession<'a, 'd, T, D, N>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn start(controller: &'a mut Stm32DshotPort<'d, T, D, N>) -> Result<Self, DshotError> {
        for pin in &mut controller.pins {
            pin.enter_output_low();
        }
        let ctx = controller as *mut _ as *mut ();
        let bsrr = controller.pins[0].bsrr_ptr();
        unsafe {
            begin_tx_dma_session::<D, T>(
                &controller.timer,
                controller.channel,
                controller.tx_timer_cfg,
                &controller.irq_state,
                Stm32DshotPort::<T, D, N>::tx_dma_irq,
                ctx,
                controller.dma_request,
                &controller.tx_words,
                bsrr,
            );
        }
        Ok(Self { controller })
    }

    async fn wait_done(&self, timeout: Duration) -> Result<(), DshotError> {
        self.controller.irq_state.wait_done(timeout).await
    }
}

impl<T, D, const N: usize> Drop for TxPortSession<'_, '_, T, D, N>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn drop(&mut self) {
        teardown_session_with_pins::<D, T>(
            &self.controller.timer,
            self.controller.channel,
            &self.controller.irq_state,
            &mut self.controller.pins,
            None,
        );
    }
}

struct TxPinSession<'a, 'd, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    controller: &'a mut Stm32BidirDshotPort<'d, T, D>,
}

impl<'a, 'd, T, D> TxPinSession<'a, 'd, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn start(controller: &'a mut Stm32BidirDshotPort<'d, T, D>) -> Result<Self, DshotError> {
        controller.pin.enter_output_high();
        let ctx = controller as *mut _ as *mut ();
        let bsrr = controller.pin.bsrr_ptr();
        unsafe {
            begin_tx_dma_session::<D, T>(
                &controller.timer,
                controller.channel,
                controller.tx_timer_cfg,
                &controller.irq_state,
                Stm32BidirDshotPort::<T, D>::tx_only_dma_irq,
                ctx,
                controller.dma_request,
                &controller.tx_words,
                bsrr,
            );
        }
        Ok(Self { controller })
    }

    async fn wait_done(&self, timeout: Duration) -> Result<(), DshotError> {
        self.controller.irq_state.wait_done(timeout).await
    }
}

impl<T, D> Drop for TxPinSession<'_, '_, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn drop(&mut self) {
        teardown_session_with_pins::<D, T>(
            &self.controller.timer,
            self.controller.channel,
            &self.controller.irq_state,
            slice::from_mut(&mut self.controller.pin),
            Some(self.controller.config.pull),
        );
    }
}

struct BidirCaptureSession<'a, 'd, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    controller: &'a mut Stm32BidirDshotPort<'d, T, D>,
}

impl<'a, 'd, T, D> BidirCaptureSession<'a, 'd, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn start(controller: &'a mut Stm32BidirDshotPort<'d, T, D>) -> Result<Self, DshotError> {
        controller.pin.enter_output_high();
        let ctx = controller as *mut _ as *mut ();
        let bsrr = controller.pin.bsrr_ptr();
        unsafe {
            begin_tx_dma_session::<D, T>(
                &controller.timer,
                controller.channel,
                controller.tx_timer_cfg,
                &controller.irq_state,
                Stm32BidirDshotPort::<T, D>::dma_irq,
                ctx,
                controller.dma_request,
                &controller.tx_words,
                bsrr,
            );
        }
        Ok(Self { controller })
    }

    async fn wait_done(&self, timeout: Duration) -> Result<(), DshotError> {
        self.controller.irq_state.wait_done(timeout).await
    }
}

impl<T, D> Drop for BidirCaptureSession<'_, '_, T, D>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn drop(&mut self) {
        teardown_session_with_pins::<D, T>(
            &self.controller.timer,
            self.controller.channel,
            &self.controller.irq_state,
            slice::from_mut(&mut self.controller.pin),
            Some(self.controller.config.pull),
        );
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
    config: &DshotConfig,
) -> PacerTimerConfig {
    // Match Betaflight's telemetry input pacing:
    // inputFreq = outputFreq * 5 * 2 * oversample / 24
    // For the BF default oversample=3, this becomes outputFreq * 5 / 4.
    let symbol_rate_hz = config.speed.timing_hints().nominal_bitrate_hz;
    let mut rx_sample_hz = symbol_rate_hz * 5 * config.oversampling.oversampling as u32 / 4;
    rx_sample_hz = rx_sample_hz.saturating_mul(config.rx_sample_percent.clamp(1, 200) as u32) / 100;
    compute_pacer_timer_config(timer, rx_sample_hz, config.rx_compare_percent)
}

pub const FRAME_BITS: usize = 16;
pub const STATES_PER_BIT: usize = 3;
pub const TX_HOLD_SLOTS: usize = 1;
pub const TX_STATE_SLOTS: usize = FRAME_BITS * STATES_PER_BIT + TX_HOLD_SLOTS;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SignalPolarity {
    Normal,
    Inverted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PortFrameError {
    EmptyGroupMask,
    PinMaskOutsideGroup { pin_mask: u32, group_mask: u32 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PortWords {
    pub words: [u32; TX_STATE_SLOTS],
}

impl PortWords {
    pub const fn new() -> Self {
        Self {
            words: [0; TX_STATE_SLOTS],
        }
    }
}

impl Default for PortWords {
    fn default() -> Self {
        Self::new()
    }
}

pub fn build_port_words<const N: usize>(
    group_mask: u32,
    pin_masks: [u32; N],
    frames: [EncodedFrame; N],
    polarity: SignalPolarity,
) -> Result<PortWords, PortFrameError> {
    if group_mask == 0 {
        return Err(PortFrameError::EmptyGroupMask);
    }

    let mut out = PortWords::new();
    init_base_words(&mut out.words, group_mask, polarity);

    for (pin_mask, frame) in pin_masks.into_iter().zip(frames.into_iter()) {
        if (pin_mask & group_mask) == 0 {
            return Err(PortFrameError::PinMaskOutsideGroup {
                pin_mask,
                group_mask,
            });
        }
        apply_frame(&mut out.words, pin_mask, frame, polarity);
    }

    Ok(out)
}

fn init_base_words(words: &mut [u32; TX_STATE_SLOTS], group_mask: u32, polarity: SignalPolarity) {
    let (set_mask, reset_mask) = match polarity {
        SignalPolarity::Normal => (group_mask, group_mask << 16),
        SignalPolarity::Inverted => (group_mask << 16, group_mask),
    };

    for bit in 0..FRAME_BITS {
        let base = bit * STATES_PER_BIT;
        words[base] = set_mask;
        words[base + 1] = 0;
        words[base + 2] = reset_mask;
    }

    words[FRAME_BITS * STATES_PER_BIT] = match polarity {
        SignalPolarity::Normal => group_mask << 16,
        SignalPolarity::Inverted => group_mask,
    };
}

fn apply_frame(
    words: &mut [u32; TX_STATE_SLOTS],
    pin_mask: u32,
    frame: EncodedFrame,
    polarity: SignalPolarity,
) {
    let mid_clear_mask = match polarity {
        SignalPolarity::Normal => pin_mask << 16,
        SignalPolarity::Inverted => pin_mask,
    };

    for (bit_idx, is_one) in frame.bits_msb_first().into_iter().enumerate() {
        if !is_one {
            words[bit_idx * STATES_PER_BIT + 1] |= mid_clear_mask;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BidirTx, UniTx};

    #[test]
    fn normal_words_match_three_state_layout() {
        let frame = UniTx::throttle(0).unwrap().encode();
        let out = build_port_words(1 << 3, [1 << 3], [frame], SignalPolarity::Normal).unwrap();

        assert_eq!(out.words[0], 1 << 3);
        assert_eq!(out.words[2], 1 << 19);
        assert_eq!(out.words[48], 1 << 19);
    }

    #[test]
    fn zero_bits_clear_in_middle_slot() {
        let frame = UniTx::command(crate::Command::MotorStop).encode();
        let out = build_port_words(1 << 8, [1 << 8], [frame], SignalPolarity::Normal).unwrap();

        assert_eq!(out.words[1], 1 << 24);
    }

    #[test]
    fn inverted_mode_flips_idle_and_clear_masks() {
        let frame = BidirTx::throttle(100).unwrap().encode();
        let out = build_port_words(1 << 2, [1 << 2], [frame], SignalPolarity::Inverted).unwrap();

        assert_eq!(out.words[0], 1 << 18);
        assert_eq!(out.words[2], 1 << 2);
        assert_eq!(out.words[48], 1 << 2);
    }
}
