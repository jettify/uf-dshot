use core::array;
use core::marker::PhantomData;

use embassy_stm32_hal::dma::{ChannelInstance as DmaChannelInstance, Request};
use embassy_stm32_hal::gpio::{AnyPin, Flex, Pin, Pull, Speed};
use embassy_stm32_hal::interrupt::typelevel::{Binding, Handler};
use embassy_stm32_hal::pac;
use embassy_stm32_hal::timer::low_level::Timer;
use embassy_stm32_hal::timer::{
    Ch1, Ch2, Ch3, Ch4, Channel, Dma, GeneralInstance4Channel, TimerChannel,
};
use embassy_stm32_hal::Peri;
use embassy_time::{Duration, Instant, Timer as EmbassyTimer};

use crate::bidir_capture::decode_frame_strict_port_samples_many_u16;
use crate::telemetry::{
    BidirDecoder, OversamplingConfig, PreambleTuningConfig, TelemetryError, TelemetryFrame,
};
use crate::{Command, DshotSpeed, DshotTx, EncodedFrame};

use super::dma_stream::{
    check_and_clear_dma_irq_flags, DmaInterruptMode, DmaStream, PreparedRxDmaConfig,
};
use super::irq_state::{
    clear_irq_slot, dispatch_irq_slot, install_irq_slot, DmaIrqFn, IrqPhase, IrqWakerState,
};
use super::port_words::{build_port_words, PortFrameError, SignalPolarity, TX_STATE_SLOTS};
use super::timer_cfg::{
    compute_pacer_timer_config, compute_rx_timer_config, configure_pacer_timer,
    switch_pacer_timer_config_fast, PacerTimerConfig,
};

const MAX_PORT_MOTORS: usize = 4;
const MAX_CAPTURE_SAMPLES: usize = 512;
const PINS_PER_GPIO_PORT: u8 = 16;
const DEFAULT_ARM_DURATION: Duration = Duration::from_millis(3_000);
const PREAMBLE_MARGIN_SAMPLES: usize = 64;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DshotError {
    TxTimeout,
    TxDmaError,
    RxDmaError,
    RxTimeout,
    Frame(PortFrameError),
    Telemetry(TelemetryError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DshotConfig {
    pub speed: DshotSpeed,
    pub tx_timeout: Duration,
    pub pacer_compare_percent: u8,
}

impl DshotConfig {
    pub fn new(speed: DshotSpeed) -> Self {
        Self {
            speed,
            tx_timeout: Duration::from_millis(2),
            pacer_compare_percent: 50,
        }
    }

    pub fn with_tx_timeout(mut self, timeout: Duration) -> Self {
        self.tx_timeout = timeout;
        self
    }

    pub fn with_pacer_compare_percent(mut self, percent: u8) -> Self {
        self.pacer_compare_percent = percent.clamp(1, 99);
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BidirDshotConfig {
    pub tx: DshotConfig,
    pub rx_timeout: Duration,
    pub oversampling: OversamplingConfig,
    pub preamble_tuning: PreambleTuningConfig,
    pub pull: Pull,
    pub rx_compare_percent: u8,
    pub rx_sample_percent: u8,
}

impl BidirDshotConfig {
    pub fn new(speed: DshotSpeed) -> Self {
        Self {
            tx: DshotConfig::new(speed),
            rx_timeout: Duration::from_millis(2),
            oversampling: OversamplingConfig::default(),
            preamble_tuning: PreambleTuningConfig::default(),
            pull: Pull::Up,
            rx_compare_percent: 50,
            rx_sample_percent: 100,
        }
    }

    pub fn with_tx_timeout(mut self, timeout: Duration) -> Self {
        self.tx = self.tx.with_tx_timeout(timeout);
        self
    }

    pub fn with_rx_timeout(mut self, timeout: Duration) -> Self {
        self.rx_timeout = timeout;
        self
    }

    pub fn with_pacer_compare_percent(mut self, percent: u8) -> Self {
        self.tx = self.tx.with_pacer_compare_percent(percent);
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

    pub fn with_preamble_tuning(mut self, preamble_tuning: PreambleTuningConfig) -> Self {
        self.preamble_tuning = preamble_tuning;
        self
    }

    pub fn with_rx_compare_percent(mut self, percent: u8) -> Self {
        self.rx_compare_percent = percent.clamp(1, 99);
        self
    }

    pub fn with_rx_sample_percent(mut self, percent: u8) -> Self {
        self.rx_sample_percent = percent.clamp(1, 200);
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
    /// # Safety
    /// Embassy invokes this in the interrupt context for `D`; the registered slot
    /// must hold a valid callback/context pair while DMA is active.
    #[inline(always)]
    unsafe fn on_interrupt() {
        dispatch_irq_slot(D::IRQ_SLOT);
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

fn recommended_capture_samples(oversampling: OversamplingConfig) -> usize {
    oversampling.recommended_capture_samples(PREAMBLE_MARGIN_SAMPLES)
}

#[doc(hidden)]
pub trait IntoBidirPins<'d, const N: usize> {
    fn into_bidir_pins(self) -> [DshotPortPin<'d>; N];
}

impl<'d> IntoBidirPins<'d, 1> for DshotPortPin<'d> {
    fn into_bidir_pins(self) -> [DshotPortPin<'d>; 1] {
        [self]
    }
}

impl<'d, const N: usize> IntoBidirPins<'d, N> for [DshotPortPin<'d>; N] {
    fn into_bidir_pins(self) -> [DshotPortPin<'d>; N] {
        self
    }
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
            pub fn $name<Pins>(
                timer: Peri<'d, T>,
                dma: Peri<'d, D>,
                dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
                pins: Pins,
                speed: DshotSpeed,
            ) -> Result<Self, PortConfigError>
            where
                D: Dma<T, $channel>,
                Pins: IntoBidirPins<'d, N>,
            {
                Self::new_inner::<$channel>(timer, dma, dma_irq, pins.into_bidir_pins(), speed)
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
        let stop_frames = [DshotTx::standard().command(Command::MotorStop); N];
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
        self.send_frames(throttles.map(|throttle| DshotTx::standard().throttle_clamped(throttle)))
            .await
    }

    /// Encodes and transmits one DShot frame for each configured pin in a shared DMA burst.
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

    /// DMA IRQ callback for TX-only multi-pin transfers.
    ///
    /// # Safety
    /// `ctx` must be a valid pointer to `Self` for the lifetime of the IRQ session.
    unsafe fn tx_dma_irq(ctx: *mut ()) {
        let this = &mut *(ctx as *mut Self);
        handle_tx_complete_irq::<D, T>(&this.timer, this.channel, &this.irq_state);
    }
}

impl<'d, T, D> Stm32DshotPort<'d, T, D, 1>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    pub async fn send_throttle(&mut self, throttle: u16) -> Result<(), DshotError> {
        self.send_throttles([throttle]).await
    }

    pub async fn send_frame(&mut self, frame: EncodedFrame) -> Result<(), DshotError> {
        self.send_frames([frame]).await
    }
}

pub struct Stm32BidirDshotPort<'d, T, D, const N: usize = 1>
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
    config: BidirDshotConfig,
    tx_timer_cfg: PacerTimerConfig,
    rx_timer_cfg: PacerTimerConfig,
    rx_dma_cfg: PreparedRxDmaConfig,
    decoders: [BidirDecoder; N],
    tx_words: [u32; TX_STATE_SLOTS],
    raw_samples: [u16; MAX_CAPTURE_SAMPLES],
    irq_state: IrqWakerState,
    _dma: PhantomData<D>,
}

impl<'d, T, D, const N: usize> Stm32BidirDshotPort<'d, T, D, N>
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

    pub fn set_config(&mut self, config: BidirDshotConfig) {
        self.config = config;
        self.tx_timer_cfg = compute_pacer_timer_config(
            &self.timer,
            self.config.tx.speed.timing_hints().nominal_bitrate_hz * 3,
            self.config.tx.pacer_compare_percent,
        );
        self.rx_timer_cfg = compute_rx_timer_config(&self.timer, &self.config);
        configure_pacer_timer(&self.timer, self.channel, self.tx_timer_cfg);

        self.decoders = array::from_fn(|_| {
            BidirDecoder::with_preamble_tuning(
                self.config.oversampling,
                self.config.preamble_tuning,
            )
        });

        let sample_count = recommended_capture_samples(self.config.oversampling);
        let sample_count = sample_count.min(MAX_CAPTURE_SAMPLES);
        self.rx_dma_cfg = PreparedRxDmaConfig {
            request: self.dma_request,
            peri_addr: self.pins[0].idr_ptr(),
            len: sample_count,
        };
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
        let config = BidirDshotConfig::new(speed);
        let tx_timer_cfg = compute_pacer_timer_config(
            &timer,
            speed.timing_hints().nominal_bitrate_hz * 3,
            config.tx.pacer_compare_percent,
        );
        let rx_timer_cfg = compute_rx_timer_config(&timer, &config);
        configure_pacer_timer(&timer, C::CHANNEL, tx_timer_cfg);
        for pin in pins.iter_mut() {
            pin.enter_input(config.pull);
        }
        let pin_idr_ptr = pins[0].idr_ptr();

        let sample_count = recommended_capture_samples(config.oversampling);
        if sample_count > MAX_CAPTURE_SAMPLES {
            return Err(PortConfigError::SampleBufferTooSmall {
                requested: sample_count,
                capacity: MAX_CAPTURE_SAMPLES,
            });
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
            rx_timer_cfg,
            rx_dma_cfg: PreparedRxDmaConfig {
                request: dma_request,
                peri_addr: pin_idr_ptr,
                len: sample_count,
            },
            decoders: array::from_fn(|_| {
                BidirDecoder::with_preamble_tuning(config.oversampling, config.preamble_tuning)
            }),
            tx_words: [0; TX_STATE_SLOTS],
            raw_samples: [0; MAX_CAPTURE_SAMPLES],
            irq_state: IrqWakerState::new(),
            _dma: PhantomData,
        })
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), DshotError> {
        let frame_period =
            Duration::from_micros(self.config.tx.speed.timing_hints().min_frame_period_us as u64);
        let stop_frames = [DshotTx::bidirectional().command(Command::MotorStop); N];
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
        self.send_frames(
            throttles.map(|throttle| DshotTx::bidirectional().throttle_clamped(throttle)),
        )
        .await
    }

    /// Encodes and transmits one bidirectional DShot frame for each configured pin.
    pub async fn send_frames(&mut self, frames: [EncodedFrame; N]) -> Result<(), DshotError> {
        self.prepare_bidir_frames(frames)?;
        self.run_tx_dma().await
    }

    /// Encodes and transmits one bidirectional frame for each configured pin, then captures
    /// and decodes the returned telemetry per pin.
    pub async fn send_frames_and_receive(
        &mut self,
        frames: [EncodedFrame; N],
    ) -> Result<[Result<TelemetryFrame, TelemetryError>; N], DshotError> {
        self.prepare_bidir_frames(frames)?;
        self.run_tx_then_capture().await?;
        Ok(self.decode_captured_frames())
    }

    pub async fn send_throttles_and_receive(
        &mut self,
        throttles: [u16; N],
    ) -> Result<[Result<TelemetryFrame, TelemetryError>; N], DshotError> {
        let frames = throttles.map(|throttle| DshotTx::bidirectional().throttle_clamped(throttle));
        self.send_frames_and_receive(frames).await
    }

    async fn run_tx_dma(&mut self) -> Result<(), DshotError> {
        let tx_timeout = self.config.tx.tx_timeout;
        let session = TxBidirSession::<T, D, N>::start(self)?;
        session.wait_done(tx_timeout).await
    }

    /// DMA IRQ callback for TX-only bidirectional-port transfers.
    ///
    /// # Safety
    /// `ctx` must be a valid pointer to `Self` for the lifetime of the IRQ session.
    unsafe fn tx_only_dma_irq(ctx: *mut ()) {
        let this = &mut *(ctx as *mut Self);
        handle_tx_complete_irq::<D, T>(&this.timer, this.channel, &this.irq_state);
    }

    async fn run_tx_then_capture(&mut self) -> Result<(), DshotError> {
        let tx_timeout = self.config.tx.tx_timeout;
        let rx_timeout = self.config.rx_timeout;
        let total_timeout = tx_timeout + rx_timeout;
        let session = BidirCaptureSession::<T, D, N>::start(self)?;

        session.wait_done(total_timeout).await?;

        Ok(())
    }

    /// DMA IRQ callback for TX->RX bidirectional capture sessions.
    ///
    /// # Safety
    /// `ctx` must be a valid pointer to `Self` for the lifetime of the IRQ session.
    unsafe fn dma_irq(ctx: *mut ()) {
        let this = &mut *(ctx as *mut Self);
        match check_and_clear_dma_irq_flags::<D>() {
            Some(false) => this.on_dma_error(),
            Some(true) => {
                this.timer.set_cc_dma_enable_state(this.channel, false);
                match this.irq_state.load_phase() {
                    IrqPhase::TxActive => {
                        // We must wait here because we immediately reconfigure the same stream
                        // for RX capture in this IRQ path.
                        DmaStream::<D>::disable();
                        this.on_tx_complete_start_rx();
                    }
                    IrqPhase::RxActive => {
                        DmaStream::<D>::disable_no_wait();
                        this.on_rx_complete();
                    }
                    _ => {
                        DmaStream::<D>::disable_no_wait();
                    }
                }
            }
            None => {}
        }
    }

    #[cold]
    fn on_dma_error(&mut self) {
        self.timer.stop();
        self.timer.set_cc_dma_enable_state(self.channel, false);
        DmaStream::<D>::disable_no_wait();
        let error_phase = match self.irq_state.load_phase() {
            IrqPhase::RxActive => IrqPhase::RxError,
            _ => IrqPhase::TxError,
        };
        self.irq_state.transition(error_phase);
    }

    fn on_tx_complete_start_rx(&mut self) {
        for pin in &mut self.pins {
            pin.enter_input(self.config.pull);
        }
        switch_pacer_timer_config_fast(&self.timer, self.channel, self.rx_timer_cfg);
        // Safety: sample buffer and GPIO IDR pointer belong to this controller and are valid
        // throughout the active IRQ-driven capture session.
        unsafe {
            DmaStream::<D>::start_prepared_read_no_reset(
                self.rx_dma_cfg,
                self.raw_samples.as_mut_ptr(),
            );
        }
        self.irq_state.set_phase(IrqPhase::RxActive);
        self.timer.set_cc_dma_enable_state(self.channel, true);
    }

    fn on_rx_complete(&mut self) {
        self.timer.stop();
        self.timer.set_cc_dma_enable_state(self.channel, false);
        self.irq_state.transition(IrqPhase::Done);
    }

    fn prepare_bidir_frames(&mut self, frames: [EncodedFrame; N]) -> Result<(), DshotError> {
        self.tx_words = build_port_words(
            self.group_mask,
            self.pin_masks,
            frames,
            SignalPolarity::Inverted,
        )
        .map_err(DshotError::Frame)?
        .words;
        Ok(())
    }

    fn decode_captured_frames(&mut self) -> [Result<TelemetryFrame, TelemetryError>; N] {
        let mut decoded = [Err(TelemetryError::NoEdge); N];
        let bit_masks = self.pin_masks.map(|mask| mask as u16);
        let results = decode_frame_strict_port_samples_many_u16(
            &mut self.decoders,
            &self.raw_samples[..self.rx_dma_cfg.len],
            bit_masks,
        );

        for idx in 0..N {
            decoded[idx] = results[idx].map_err(Into::into);
        }

        decoded
    }
}

impl<'d, T, D> Stm32BidirDshotPort<'d, T, D, 1>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    pub async fn send_throttle_and_receive(
        &mut self,
        throttle: u16,
    ) -> Result<TelemetryFrame, DshotError> {
        let [result] = self.send_throttles_and_receive([throttle]).await?;
        result.map_err(DshotError::Telemetry)
    }

    pub async fn send_frame(&mut self, frame: EncodedFrame) -> Result<(), DshotError> {
        self.send_frames([frame]).await
    }

    /// Sends one inverted (bidirectional) frame, then captures and decodes return telemetry.
    pub async fn send_frame_and_receive(
        &mut self,
        frame: EncodedFrame,
    ) -> Result<TelemetryFrame, DshotError> {
        let [result] = self.send_frames_and_receive([frame]).await?;
        result.map_err(DshotError::Telemetry)
    }
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
            DmaStream::<D>::disable_no_wait();
            irq_state.transition(IrqPhase::TxError);
        }
        Some(true) => {
            timer.set_cc_dma_enable_state(channel, false);
            DmaStream::<D>::disable_no_wait();
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
    install_irq_slot(D::IRQ_SLOT, ctx, irq_fn);
}

fn clear_irq_ctx<D: RawDmaChannel>() {
    clear_irq_slot(D::IRQ_SLOT);
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

/// Starts a TX DMA session by arming timer, installing IRQ context, and enabling DMA transfer.
///
/// # Safety
/// `ctx` must point to the concrete controller expected by `irq_fn`; `tx_words` and `bsrr_ptr`
/// must remain valid until session teardown disables DMA.
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

struct TxBidirSession<'a, 'd, T, D, const N: usize>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    controller: &'a mut Stm32BidirDshotPort<'d, T, D, N>,
}

impl<'a, 'd, T, D, const N: usize> TxBidirSession<'a, 'd, T, D, N>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn start(controller: &'a mut Stm32BidirDshotPort<'d, T, D, N>) -> Result<Self, DshotError> {
        for pin in &mut controller.pins {
            pin.enter_output_high();
        }
        let ctx = controller as *mut _ as *mut ();
        let bsrr = controller.pins[0].bsrr_ptr();
        unsafe {
            begin_tx_dma_session::<D, T>(
                &controller.timer,
                controller.channel,
                controller.tx_timer_cfg,
                &controller.irq_state,
                Stm32BidirDshotPort::<T, D, N>::tx_only_dma_irq,
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

impl<T, D, const N: usize> Drop for TxBidirSession<'_, '_, T, D, N>
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
            Some(self.controller.config.pull),
        );
    }
}

struct BidirCaptureSession<'a, 'd, T, D, const N: usize>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    controller: &'a mut Stm32BidirDshotPort<'d, T, D, N>,
}

impl<'a, 'd, T, D, const N: usize> BidirCaptureSession<'a, 'd, T, D, N>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn start(controller: &'a mut Stm32BidirDshotPort<'d, T, D, N>) -> Result<Self, DshotError> {
        for pin in &mut controller.pins {
            pin.enter_output_high();
        }
        let ctx = controller as *mut _ as *mut ();
        let bsrr = controller.pins[0].bsrr_ptr();
        unsafe {
            begin_tx_dma_session::<D, T>(
                &controller.timer,
                controller.channel,
                controller.tx_timer_cfg,
                &controller.irq_state,
                Stm32BidirDshotPort::<T, D, N>::dma_irq,
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

impl<T, D, const N: usize> Drop for BidirCaptureSession<'_, '_, T, D, N>
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
            Some(self.controller.config.pull),
        );
    }
}
