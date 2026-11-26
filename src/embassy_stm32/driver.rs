use core::array;
use core::marker::PhantomData;

use embassy_stm32_hal::dma::{ChannelInstance as DmaChannelInstance, Request};
use embassy_stm32_hal::gpio::{AnyPin, Flex, Pin, Pull, Speed};
use embassy_stm32_hal::interrupt::typelevel::{Binding, Handler, Interrupt as _};
use embassy_stm32_hal::pac;
use embassy_stm32_hal::timer::low_level::Timer;
use embassy_stm32_hal::timer::{
    Ch1, Ch2, Ch3, Ch4, Channel, Dma, GeneralInstance4Channel, TimerChannel,
};
use embassy_stm32_hal::Peri;
use embassy_time::{Duration, Instant, Timer as EmbassyTimer};

use crate::stm32_timing::{
    telemetry_sample_hz, telemetry_window_us, ReplyWindow, MAX_CAPTURE_SAMPLES,
};
use crate::telemetry::{
    validate_oversampling_config, BidirDecoder, OversamplingConfig, PreambleTuningConfig,
    TelemetryError, TelemetryFrame,
};
use crate::{Command, DshotSpeed, DshotTx, EncodedFrame};

use super::dma_stream::{check_and_clear_dma_irq_flags, DmaStream, PreparedRxDmaConfig};
use super::irq_state::{dma_slot, BidirPinSet, TransferState};
use super::port_words::{build_port_words, PortFrameError, SignalPolarity, TX_STATE_SLOTS};
use super::timer_cfg::{
    compute_pacer_timer_config, configure_pacer_timer, set_cc_dma_regs, stop_timer_cc_dma_regs,
    stop_timer_regs, switch_pacer_timer_regs_fast, PacerTimerConfig,
};

pub(crate) const MAX_PORT_MOTORS: usize = 4;
const PINS_PER_GPIO_PORT: u8 = 16;
const DEFAULT_ARM_DURATION: Duration = Duration::from_millis(3_000);
const BIDIR_ARM_FRAME_PERIOD: Duration = Duration::from_millis(1);

/// Static DMA buffer storage for [`Stm32DshotPort`].
///
/// Must have `'static` lifetime so active DMA transfers remain sound even if a transfer future
/// is forgotten via [`core::mem::forget`]. Place it in DMA-accessible SRAM.
/// On STM32F7 with D-cache enabled, the buffer must be in a non-cacheable region.
pub struct DshotTxBuffers {
    tx_words: [u32; TX_STATE_SLOTS],
}

impl DshotTxBuffers {
    pub const fn new() -> Self {
        Self {
            tx_words: [0; TX_STATE_SLOTS],
        }
    }
}

impl Default for DshotTxBuffers {
    fn default() -> Self {
        Self::new()
    }
}

/// Static DMA buffer storage for [`Stm32BidirDshotPort`].
///
/// Must have `'static` lifetime so active DMA transfers remain sound even if a transfer future
/// is forgotten via [`core::mem::forget`]. Place it in DMA-accessible SRAM.
/// On STM32F7 with D-cache enabled, the entire buffer must be in a non-cacheable region.
pub struct BidirDshotBuffers {
    tx_words: [u32; TX_STATE_SLOTS],
    raw_samples: [u16; MAX_CAPTURE_SAMPLES],
}

impl BidirDshotBuffers {
    pub const fn new() -> Self {
        Self {
            tx_words: [0; TX_STATE_SLOTS],
            raw_samples: [0; MAX_CAPTURE_SAMPLES],
        }
    }
}

impl Default for BidirDshotBuffers {
    fn default() -> Self {
        Self::new()
    }
}

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
    InvalidOversampling,
    UnsupportedTimerRate {
        timer_hz: u32,
        target_hz: u32,
    },
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

mod sealed {
    pub trait Sealed {}
}

pub trait RawDmaChannel: sealed::Sealed + DmaChannelInstance {
    const IRQ_SLOT: usize;
    fn regs() -> pac::dma::Dma;
    fn stream_num() -> usize;
}

pub struct InterruptHandler<D: RawDmaChannel> {
    _phantom: PhantomData<D>,
}

impl<D: RawDmaChannel> Handler<D::Interrupt> for InterruptHandler<D> {
    /// # Safety
    /// Embassy invokes this in the interrupt context for `D`.
    unsafe fn on_interrupt() {
        on_dma_irq::<D>();
    }
}

macro_rules! impl_raw_dma_channel {
    ($periph:ty, $regs:expr, $irq_slot:expr, $stream_num:expr) => {
        impl sealed::Sealed for $periph {}

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

// On STM32F2/F4/F7 (DMA v2, ST AN4031), only DMA2 has a bus-matrix path to AHB1 GPIO
// registers (`BSRR`/`IDR`). Restricting `RawDmaChannel` to DMA2 prevents selecting an
// invalid DMA1 + timer route that would always fail with a DMA transfer error.
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH0, pac::DMA2, 0, 0);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH1, pac::DMA2, 1, 1);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH2, pac::DMA2, 2, 2);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH3, pac::DMA2, 3, 3);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH4, pac::DMA2, 4, 4);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH5, pac::DMA2, 5, 5);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH6, pac::DMA2, 6, 6);
impl_raw_dma_channel!(embassy_stm32_hal::peripherals::DMA2_CH7, pac::DMA2, 7, 7);

pub struct DshotPortPin {
    line: Flex<'static>,
    port: u8,
    pin_num: u8,
    pin_mask: u32,
    gpio: pac::gpio::Gpio,
    bsrr_ptr: *mut u32,
    idr_addr: u32,
}

impl DshotPortPin {
    pub fn new(pin: Peri<'static, impl Pin>) -> Self {
        let pin_num = pin.pin();
        let pin_mask = 1u32 << pin_num;
        let port = pin.port();
        let pin_port = (port as usize) * (PINS_PER_GPIO_PORT as usize) + (pin_num as usize);
        let regs = unsafe { AnyPin::steal(pin_port as u8) }.block();

        Self {
            line: Flex::new(pin),
            port,
            pin_num,
            pin_mask,
            gpio: regs,
            bsrr_ptr: regs.bsrr().as_ptr() as *mut u32,
            idr_addr: regs.idr().as_ptr() as u32,
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

    fn pin_num(&self) -> u8 {
        self.pin_num
    }

    fn pin_mask(&self) -> u32 {
        self.pin_mask
    }

    fn gpio(&self) -> pac::gpio::Gpio {
        self.gpio
    }

    fn bsrr_ptr(&self) -> *mut u32 {
        self.bsrr_ptr
    }

    fn idr_addr(&self) -> u32 {
        self.idr_addr
    }
}

fn validate_port_pins<const N: usize>(
    pins: &[DshotPortPin; N],
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

struct BidirTiming {
    tx: PacerTimerConfig,
    rx: PacerTimerConfig,
    sample_count: usize,
    reply_window: Duration,
}

impl BidirTiming {
    fn new(timer_hz: u32, config: &BidirDshotConfig) -> Result<Self, PortConfigError> {
        let oversampling = config.oversampling;
        validate_oversampling_config(oversampling)
            .map_err(|_| PortConfigError::InvalidOversampling)?;

        let bitrate = config.tx.speed.timing_hints().nominal_bitrate_hz;
        let tx =
            compute_pacer_timer_config(timer_hz, bitrate * 3, config.tx.pacer_compare_percent)?;
        let sample_hz =
            telemetry_sample_hz(bitrate, oversampling.oversampling, config.rx_sample_percent);
        let rx = compute_pacer_timer_config(timer_hz, sample_hz, config.rx_compare_percent)?;
        let window_us =
            telemetry_window_us(bitrate, oversampling.frame_bits, oversampling.bit_tolerance);
        let sample_count = rx.capture_samples(timer_hz, window_us);
        if sample_count > MAX_CAPTURE_SAMPLES {
            return Err(PortConfigError::SampleBufferTooSmall {
                requested: sample_count,
                capacity: MAX_CAPTURE_SAMPLES,
            });
        }
        Ok(Self {
            tx,
            rx,
            sample_count,
            reply_window: Duration::from_micros(window_us),
        })
    }
}

fn make_bidir_pin_set<const N: usize>(pins: &[DshotPortPin; N], pull: Pull) -> BidirPinSet {
    let mut pin_nums = [0u8; MAX_PORT_MOTORS];
    for (dst, pin) in pin_nums.iter_mut().zip(pins.iter()) {
        *dst = pin.pin_num();
    }
    BidirPinSet {
        gpio: pins[0].gpio(),
        pin_nums,
        count: N as u8,
        pull,
    }
}

#[doc(hidden)]
pub trait IntoBidirPins<const N: usize> {
    fn into_bidir_pins(self) -> [DshotPortPin; N];
}

impl IntoBidirPins<1> for DshotPortPin {
    fn into_bidir_pins(self) -> [DshotPortPin; 1] {
        [self]
    }
}

impl<const N: usize> IntoBidirPins<N> for [DshotPortPin; N] {
    fn into_bidir_pins(self) -> [DshotPortPin; N] {
        self
    }
}

macro_rules! impl_tx_port_channel_ctors {
    ($(($name:ident, $channel:ty)),+ $(,)?) => {
        $(
            pub fn $name(
                timer: Peri<'static, T>,
                dma: Peri<'static, D>,
                dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'static,
                pins: [DshotPortPin; N],
                speed: DshotSpeed,
                buffers: &'static mut DshotTxBuffers,
            ) -> Result<Self, PortConfigError>
            where
                D: Dma<T, $channel>,
            {
                Self::new_inner::<$channel>(timer, dma, dma_irq, pins, speed, buffers)
            }
        )+
    };
}

macro_rules! impl_bidir_pin_channel_ctors {
    ($(($name:ident, $channel:ty)),+ $(,)?) => {
        $(
            pub fn $name<Pins>(
                timer: Peri<'static, T>,
                dma: Peri<'static, D>,
                dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'static,
                pins: Pins,
                speed: DshotSpeed,
                buffers: &'static mut BidirDshotBuffers,
            ) -> Result<Self, PortConfigError>
            where
                D: Dma<T, $channel>,
                Pins: IntoBidirPins<N>,
            {
                Self::new_inner::<$channel>(
                    timer,
                    dma,
                    dma_irq,
                    pins.into_bidir_pins(),
                    speed,
                    buffers,
                )
            }
        )+
    };
}

pub struct Stm32DshotPort<T, D, const N: usize>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    timer: Timer<'static, T>,
    _dma: Peri<'static, D>,
    dma_request: Request,
    pins: [DshotPortPin; N],
    pin_masks: [u32; N],
    group_mask: u32,
    channel: Channel,
    config: DshotConfig,
    tx_timer_cfg: PacerTimerConfig,
    buffers: &'static mut DshotTxBuffers,
}

impl<T, D, const N: usize> Stm32DshotPort<T, D, N>
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

    fn quiesce(&mut self) {
        quiesce_stream::<D, T>(&self.timer, self.channel);
    }

    /// Applies a new configuration after validating the timer rate.
    pub fn set_config(&mut self, config: DshotConfig) -> Result<(), PortConfigError> {
        let tx_timer_cfg = compute_pacer_timer_config(
            self.timer.get_clock_frequency().0,
            config.speed.timing_hints().nominal_bitrate_hz * 3,
            config.pacer_compare_percent,
        )?;
        self.quiesce();
        for pin in &mut self.pins {
            pin.enter_output_low();
        }
        self.config = config;
        self.tx_timer_cfg = tx_timer_cfg;
        configure_pacer_timer(&self.timer, self.channel, self.tx_timer_cfg);
        Ok(())
    }

    fn new_inner<C>(
        timer: Peri<'static, T>,
        dma: Peri<'static, D>,
        _dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'static,
        mut pins: [DshotPortPin; N],
        speed: DshotSpeed,
        buffers: &'static mut DshotTxBuffers,
    ) -> Result<Self, PortConfigError>
    where
        C: TimerChannel,
        D: Dma<T, C>,
    {
        let pin_set = validate_port_pins(&pins)?;
        let timer = Timer::new(timer);
        let config = DshotConfig::new(speed);
        let tx_timer_cfg = compute_pacer_timer_config(
            timer.get_clock_frequency().0,
            speed.timing_hints().nominal_bitrate_hz * 3,
            config.pacer_compare_percent,
        )?;
        let dma_request = dma.request();
        dma.remap();
        reset_dma_stream::<D, T>(&timer, C::CHANNEL);
        configure_pacer_timer(&timer, C::CHANNEL, tx_timer_cfg);

        for pin in pins.iter_mut() {
            pin.enter_output_low();
        }

        Ok(Self {
            timer,
            _dma: dma,
            dma_request,
            pins,
            pin_masks: pin_set.pin_masks,
            group_mask: pin_set.group_mask,
            channel: C::CHANNEL,
            config,
            tx_timer_cfg,
            buffers,
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
        self.quiesce();
        let port_words = build_port_words(
            self.group_mask,
            self.pin_masks,
            frames,
            SignalPolarity::Normal,
        )
        .map_err(DshotError::Frame)?;
        self.buffers.tx_words = port_words.words;
        self.run_tx_dma().await
    }

    async fn run_tx_dma(&mut self) -> Result<(), DshotError> {
        for pin in &mut self.pins {
            pin.enter_output_low();
        }
        let tx_timeout = self.config.tx_timeout;
        let bsrr = self.pins[0].bsrr_ptr();
        let timer_regs = self.timer.regs_gp16();
        let channel = self.channel;
        let _guard = DmaSessionGuard::<D, T>::new(&self.timer, channel, &mut self.pins, None);

        // Safety: `self.buffers.tx_words` is `'static` and idle after `quiesce()`.
        let tx_deadline = unsafe {
            begin_tx_dma_session::<D, T>(
                &self.timer,
                channel,
                self.tx_timer_cfg,
                tx_timeout,
                |deadline| TransferState::TxStandard {
                    timer: timer_regs,
                    channel,
                    deadline,
                },
                self.dma_request,
                &self.buffers.tx_words,
                bsrr,
            )
        };
        dma_slot(D::IRQ_SLOT).wait_tx_done(tx_deadline).await
    }
}

impl<T, D, const N: usize> Drop for Stm32DshotPort<T, D, N>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn drop(&mut self) {
        self.quiesce();
        for pin in &mut self.pins {
            pin.enter_output_low();
        }
    }
}

impl<T, D> Stm32DshotPort<T, D, 1>
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

pub struct Stm32BidirDshotPort<T, D, const N: usize = 1>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    timer: Timer<'static, T>,
    _dma: Peri<'static, D>,
    dma_request: Request,
    pins: [DshotPortPin; N],
    pin_masks: [u32; N],
    group_mask: u32,
    channel: Channel,
    config: BidirDshotConfig,
    tx_timer_cfg: PacerTimerConfig,
    rx_timer_cfg: PacerTimerConfig,
    rx_dma_cfg: PreparedRxDmaConfig,
    reply_duration: Duration,
    reply_window: ReplyWindow,
    decoders: [BidirDecoder; N],
    buffers: &'static mut BidirDshotBuffers,
}

impl<T, D, const N: usize> Stm32BidirDshotPort<T, D, N>
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

    fn quiesce(&mut self) {
        quiesce_stream::<D, T>(&self.timer, self.channel);
        if matches!(self.reply_window, ReplyWindow::Pending(_)) {
            // A forgotten future skips the DMA session guard.
            finish_bidir_transfer::<D>(&mut self.pins, self.config.pull, &mut self.reply_window);
        }
    }

    /// Applies a new configuration after validating timer rates and capture buffer capacity.
    pub fn set_config(&mut self, config: BidirDshotConfig) -> Result<(), PortConfigError> {
        let timing = BidirTiming::new(self.timer.get_clock_frequency().0, &config)?;
        self.quiesce();

        self.config = config;
        for pin in &mut self.pins {
            pin.enter_input(self.config.pull);
        }
        self.tx_timer_cfg = timing.tx;
        self.rx_timer_cfg = timing.rx;
        self.reply_duration = timing.reply_window;
        configure_pacer_timer(&self.timer, self.channel, self.tx_timer_cfg);

        self.decoders = array::from_fn(|_| {
            BidirDecoder::with_preamble_tuning(
                self.config.oversampling,
                self.config.preamble_tuning,
            )
        });

        self.rx_dma_cfg = PreparedRxDmaConfig {
            request: self.dma_request,
            peri_addr: self.pins[0].idr_addr(),
            len: timing.sample_count,
        };
        Ok(())
    }

    fn new_inner<C>(
        timer: Peri<'static, T>,
        dma: Peri<'static, D>,
        _dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'static,
        mut pins: [DshotPortPin; N],
        speed: DshotSpeed,
        buffers: &'static mut BidirDshotBuffers,
    ) -> Result<Self, PortConfigError>
    where
        C: TimerChannel,
        D: Dma<T, C>,
    {
        let pin_set = validate_port_pins(&pins)?;
        let config = BidirDshotConfig::new(speed);
        let timer = Timer::new(timer);
        let timing = BidirTiming::new(timer.get_clock_frequency().0, &config)?;
        let dma_request = dma.request();
        dma.remap();
        reset_dma_stream::<D, T>(&timer, C::CHANNEL);

        configure_pacer_timer(&timer, C::CHANNEL, timing.tx);
        for pin in pins.iter_mut() {
            pin.enter_input(config.pull);
        }
        let pin_idr_addr = pins[0].idr_addr();

        Ok(Self {
            timer,
            _dma: dma,
            dma_request,
            pins,
            pin_masks: pin_set.pin_masks,
            group_mask: pin_set.group_mask,
            channel: C::CHANNEL,
            config,
            tx_timer_cfg: timing.tx,
            rx_timer_cfg: timing.rx,
            rx_dma_cfg: PreparedRxDmaConfig {
                request: dma_request,
                peri_addr: pin_idr_addr,
                len: timing.sample_count,
            },
            reply_duration: timing.reply_window,
            reply_window: ReplyWindow::Idle,
            decoders: array::from_fn(|_| {
                BidirDecoder::with_preamble_tuning(config.oversampling, config.preamble_tuning)
            }),
            buffers,
        })
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), DshotError> {
        let stop_frames = [DshotTx::bidirectional().command(Command::MotorStop); N];
        let deadline = Instant::now() + duration;

        while Instant::now() < deadline {
            self.send_frames(stop_frames).await?;
            EmbassyTimer::after(BIDIR_ARM_FRAME_PERIOD).await;
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
    ///
    /// Returns after TX releases the pins. A subsequent transmission waits out the ESC reply.
    pub async fn send_frames(&mut self, frames: [EncodedFrame; N]) -> Result<(), DshotError> {
        self.wait_for_reply_window().await;
        self.prepare_bidir_frames(frames)?;
        self.run_tx_dma().await
    }

    /// Encodes and transmits one bidirectional frame for each configured pin, then captures
    /// and decodes the returned telemetry per pin.
    pub async fn send_frames_and_receive(
        &mut self,
        frames: [EncodedFrame; N],
    ) -> Result<[Result<TelemetryFrame, TelemetryError>; N], DshotError> {
        self.wait_for_reply_window().await;
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

    async fn wait_for_reply_window(&mut self) {
        self.quiesce();
        if let Some(ticks) = self.reply_window.deadline() {
            let deadline = Instant::from_ticks(ticks);
            if Instant::now() < deadline {
                EmbassyTimer::at(deadline).await;
            }
        }
    }

    fn begin_output(&mut self) {
        self.reply_window = ReplyWindow::Pending(self.reply_duration.as_ticks());
        for pin in &mut self.pins {
            pin.enter_output_high();
        }
    }

    async fn run_tx_dma(&mut self) -> Result<(), DshotError> {
        self.begin_output();
        let tx_timeout = self.config.tx.tx_timeout;
        let bsrr = self.pins[0].bsrr_ptr();
        let timer_regs = self.timer.regs_gp16();
        let channel = self.channel;
        let bidir_pins = make_bidir_pin_set(&self.pins, self.config.pull);
        let _guard = DmaSessionGuard::<D, T>::new(
            &self.timer,
            channel,
            &mut self.pins,
            Some((self.config.pull, &mut self.reply_window)),
        );

        // Safety: `self.buffers.tx_words` is `'static` and idle after `quiesce()`.
        let tx_deadline = unsafe {
            begin_tx_dma_session::<D, T>(
                &self.timer,
                channel,
                self.tx_timer_cfg,
                tx_timeout,
                |deadline| TransferState::TxBidirRelease {
                    timer: timer_regs,
                    channel,
                    pins: bidir_pins,
                    deadline,
                },
                self.dma_request,
                &self.buffers.tx_words,
                bsrr,
            )
        };
        dma_slot(D::IRQ_SLOT).wait_tx_done(tx_deadline).await
    }

    async fn run_tx_then_capture(&mut self) -> Result<(), DshotError> {
        self.begin_output();
        let tx_timeout = self.config.tx.tx_timeout;
        let rx_timeout = self.config.rx_timeout;
        let bsrr = self.pins[0].bsrr_ptr();
        let timer_regs = self.timer.regs_gp16();
        let channel = self.channel;
        let bidir_pins = make_bidir_pin_set(&self.pins, self.config.pull);
        let rx_timer_cfg = self.rx_timer_cfg;
        let rx_dma_cfg = self.rx_dma_cfg;
        let rx_buf_addr = self.buffers.raw_samples.as_mut_ptr() as u32;
        let _guard = DmaSessionGuard::<D, T>::new(
            &self.timer,
            channel,
            &mut self.pins,
            Some((self.config.pull, &mut self.reply_window)),
        );

        // Safety: `self.buffers` is `'static` and idle after `quiesce()`.
        let tx_deadline = unsafe {
            begin_tx_dma_session::<D, T>(
                &self.timer,
                channel,
                self.tx_timer_cfg,
                tx_timeout,
                |deadline| TransferState::TxBidirCapture {
                    timer: timer_regs,
                    channel,
                    pins: bidir_pins,
                    tx_deadline: deadline,
                    rx_timeout,
                    rx_timer_cfg,
                    rx_dma_cfg,
                    rx_buf_addr,
                },
                self.dma_request,
                &self.buffers.tx_words,
                bsrr,
            )
        };

        let slot = dma_slot(D::IRQ_SLOT);
        if let Some(rx_deadline) = slot.wait_tx_handoff(tx_deadline).await? {
            slot.wait_rx_done(rx_deadline).await?;
        }
        Ok(())
    }

    fn prepare_bidir_frames(&mut self, frames: [EncodedFrame; N]) -> Result<(), DshotError> {
        self.buffers.tx_words = build_port_words(
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
        let samples = &self.buffers.raw_samples[..self.rx_dma_cfg.len];
        array::from_fn(|idx| {
            self.decoders[idx]
                .decode_frame_tuned_port_samples_u16(samples, self.pin_masks[idx] as u16)
        })
    }
}

impl<T, D, const N: usize> Drop for Stm32BidirDshotPort<T, D, N>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    fn drop(&mut self) {
        self.quiesce();
        for pin in &mut self.pins {
            pin.enter_input(self.config.pull);
        }
    }
}

impl<T, D> Stm32BidirDshotPort<T, D, 1>
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

fn on_dma_irq<D: RawDmaChannel>() {
    critical_section::with(|cs| {
        let slot = dma_slot(D::IRQ_SLOT);
        match check_and_clear_dma_irq_flags::<D>() {
            Some(false) => {
                DmaStream::<D>::disable_no_wait();
                match slot.state(cs) {
                    TransferState::TxStandard { timer, channel, .. } => {
                        stop_timer_cc_dma_regs(timer, channel);
                        slot.transition(cs, TransferState::TxError);
                    }
                    TransferState::TxBidirRelease {
                        timer,
                        channel,
                        pins,
                        ..
                    }
                    | TransferState::TxBidirCapture {
                        timer,
                        channel,
                        pins,
                        ..
                    } => {
                        stop_timer_cc_dma_regs(timer, channel);
                        switch_port_pins_to_input(pins);
                        slot.record_release(cs);
                        slot.transition(cs, TransferState::TxError);
                    }
                    TransferState::RxCapture { timer, channel, .. } => {
                        stop_timer_cc_dma_regs(timer, channel);
                        slot.transition(cs, TransferState::RxError);
                    }
                    _ => {
                        DmaStream::<D>::clear_flags();
                    }
                }
            }
            Some(true) => {
                match slot.state(cs) {
                    TransferState::TxStandard {
                        timer,
                        channel,
                        deadline,
                    } => {
                        let now = Instant::now();
                        stop_timer_cc_dma_regs(timer, channel);
                        let next = if now >= deadline {
                            TransferState::TxTimeout
                        } else {
                            TransferState::Done
                        };
                        slot.transition(cs, next);
                    }
                    TransferState::TxBidirRelease {
                        timer,
                        channel,
                        pins,
                        deadline,
                    } => {
                        stop_timer_cc_dma_regs(timer, channel);
                        switch_port_pins_to_input(pins);
                        let released_at = slot.record_release(cs);
                        let next = if released_at >= deadline {
                            TransferState::TxTimeout
                        } else {
                            TransferState::Done
                        };
                        slot.transition(cs, next);
                    }
                    TransferState::TxBidirCapture {
                        timer,
                        channel,
                        pins,
                        tx_deadline,
                        rx_timeout,
                        rx_timer_cfg,
                        rx_dma_cfg,
                        rx_buf_addr,
                    } => {
                        set_cc_dma_regs(timer, channel, false);
                        switch_port_pins_to_input(pins);
                        let released_at = slot.record_release(cs);
                        if released_at >= tx_deadline {
                            stop_timer_regs(timer);
                            slot.transition(cs, TransferState::TxTimeout);
                        } else {
                            debug_assert!(!DmaStream::<D>::is_enabled());
                            switch_pacer_timer_regs_fast(timer, channel, rx_timer_cfg);
                            // Safety: `rx_buf_addr` points into `'static` `BidirDshotBuffers`
                            // and `rx_dma_cfg.peri_addr` points to the GPIO port's IDR register.
                            unsafe {
                                DmaStream::<D>::start_prepared_read_no_reset(
                                    rx_dma_cfg,
                                    rx_buf_addr,
                                );
                            }
                            let rx_deadline = released_at + rx_timeout;
                            slot.transition(
                                cs,
                                TransferState::RxCapture {
                                    timer,
                                    channel,
                                    deadline: rx_deadline,
                                },
                            );
                            set_cc_dma_regs(timer, channel, true);
                        }
                    }
                    TransferState::RxCapture {
                        timer,
                        channel,
                        deadline,
                    } => {
                        let now = Instant::now();
                        stop_timer_cc_dma_regs(timer, channel);
                        let next = if now >= deadline {
                            TransferState::RxTimeout
                        } else {
                            TransferState::Done
                        };
                        slot.transition(cs, next);
                    }
                    _ => {
                        DmaStream::<D>::disable_no_wait();
                        DmaStream::<D>::clear_flags();
                    }
                }
            }
            None => {}
        }
    });
}

#[inline(always)]
fn switch_port_pins_to_input(pins: BidirPinSet) {
    let pupdr = match pins.pull {
        Pull::None => pac::gpio::vals::Pupdr::FLOATING,
        Pull::Up => pac::gpio::vals::Pupdr::PULL_UP,
        Pull::Down => pac::gpio::vals::Pupdr::PULL_DOWN,
    };
    let pin_slice = pins.pin_slice();
    pins.gpio.pupdr().modify(|w| {
        for &pin in pin_slice {
            w.set_pupdr(pin as usize, pupdr);
        }
    });
    pins.gpio.moder().modify(|w| {
        for &pin in pin_slice {
            w.set_moder(pin as usize, pac::gpio::vals::Moder::INPUT);
        }
    });
}

fn arm_timer_for_dma_transfer<T: GeneralInstance4Channel>(
    timer: &Timer<'static, T>,
    channel: Channel,
    cfg: PacerTimerConfig,
) {
    configure_pacer_timer(timer, channel, cfg);
    timer.set_cc_dma_enable_state(channel, false);
    timer.reset();
}

fn start_timer_dma_transfer<T: GeneralInstance4Channel>(
    timer: &Timer<'static, T>,
    channel: Channel,
) {
    timer.set_cc_dma_enable_state(channel, true);
    timer.start();
}

fn stop_timer_dma_transfer<T: GeneralInstance4Channel>(
    timer: &Timer<'static, T>,
    channel: Channel,
) {
    timer.stop();
    timer.set_cc_dma_enable_state(channel, false);
}

/// Resets the stream if the transfer slot or DMA hardware is still active.
fn quiesce_stream<D: RawDmaChannel, T: GeneralInstance4Channel>(
    timer: &Timer<'static, T>,
    channel: Channel,
) {
    if !dma_slot(D::IRQ_SLOT).is_idle() || DmaStream::<D>::is_enabled() {
        reset_dma_stream::<D, T>(timer, channel);
    }
}

/// Stops the stream, clears pending flags, resets the slot to [`TransferState::Idle`],
/// and enables the stream IRQ even on cold start.
fn reset_dma_stream<D: RawDmaChannel, T: GeneralInstance4Channel>(
    timer: &Timer<'static, T>,
    channel: Channel,
) {
    // Synchronize the NVIC mask before restoring PRIMASK so the IRQ cannot
    // preempt teardown while we wait for the stream to drain.
    critical_section::with(|_| {
        D::Interrupt::disable();
        cortex_m::asm::dsb();
        cortex_m::asm::isb();
    });

    stop_timer_dma_transfer(timer, channel);
    DmaStream::<D>::disable();
    DmaStream::<D>::clear_flags();
    D::Interrupt::unpend();

    critical_section::with(|cs| {
        dma_slot(D::IRQ_SLOT).set_state(cs, TransferState::Idle);
        // Safety: stream is disabled, flags and pending NVIC bit are cleared, and state is Idle.
        unsafe {
            D::Interrupt::enable();
        }
    });
}

/// Starts a TX DMA session by arming timer, publishing the initial [`TransferState`],
/// and enabling DMA + timer requests inside a critical section.
///
/// Returns the absolute TX deadline anchored to hardware start.
///
/// # Safety
/// `tx_words` must point into `'static` storage that remains exclusively owned by this stream
/// until quiesced, and `bsrr_ptr` must point to a valid GPIO BSRR register.
#[allow(clippy::too_many_arguments)]
unsafe fn begin_tx_dma_session<D: RawDmaChannel, T: GeneralInstance4Channel>(
    timer: &Timer<'static, T>,
    channel: Channel,
    timer_cfg: PacerTimerConfig,
    tx_timeout: Duration,
    make_state: impl FnOnce(Instant) -> TransferState,
    dma_request: Request,
    tx_words: &[u32],
    bsrr_ptr: *mut u32,
) -> Instant {
    arm_timer_for_dma_transfer(timer, channel, timer_cfg);
    critical_section::with(|cs| {
        let tx_deadline = Instant::now() + tx_timeout;
        dma_slot(D::IRQ_SLOT).begin(cs, make_state(tx_deadline));
        DmaStream::<D>::start_write(dma_request, tx_words.as_ptr(), bsrr_ptr, tx_words.len());
        start_timer_dma_transfer(timer, channel);
        tx_deadline
    })
}

struct DmaSessionGuard<'a, D, T>
where
    D: RawDmaChannel,
    T: GeneralInstance4Channel,
{
    timer: &'a Timer<'static, T>,
    channel: Channel,
    pins: &'a mut [DshotPortPin],
    bidir: Option<(Pull, &'a mut ReplyWindow)>,
    _dma: PhantomData<D>,
}

impl<'a, D, T> DmaSessionGuard<'a, D, T>
where
    D: RawDmaChannel,
    T: GeneralInstance4Channel,
{
    fn new(
        timer: &'a Timer<'static, T>,
        channel: Channel,
        pins: &'a mut [DshotPortPin],
        bidir: Option<(Pull, &'a mut ReplyWindow)>,
    ) -> Self {
        Self {
            timer,
            channel,
            pins,
            bidir,
            _dma: PhantomData,
        }
    }
}

impl<D, T> Drop for DmaSessionGuard<'_, D, T>
where
    D: RawDmaChannel,
    T: GeneralInstance4Channel,
{
    fn drop(&mut self) {
        quiesce_stream::<D, T>(self.timer, self.channel);
        if let Some((pull, reply_window)) = &mut self.bidir {
            finish_bidir_transfer::<D>(self.pins, *pull, reply_window);
        } else {
            for pin in self.pins.iter_mut() {
                pin.enter_output_low();
            }
        }
    }
}

fn finish_bidir_transfer<D: RawDmaChannel>(
    pins: &mut [DshotPortPin],
    pull: Pull,
    reply_window: &mut ReplyWindow,
) {
    let released_at = dma_slot(D::IRQ_SLOT).take_released_at();
    if released_at.is_none() {
        for pin in pins {
            pin.enter_input(pull);
        }
    }
    reply_window.finish(
        released_at.map(|instant| instant.as_ticks()),
        Instant::now().as_ticks(),
    );
}
