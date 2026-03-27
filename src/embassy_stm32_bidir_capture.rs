use core::array;
use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU8, Ordering};

use embassy_stm32_hal::dma::{ChannelInstance as DmaChannelInstance, Request};
use embassy_stm32_hal::gpio::{AnyPin, Flex, Pin, Pull, Speed};
use embassy_stm32_hal::interrupt::typelevel::{Binding, Handler};
use embassy_stm32_hal::pac;
use embassy_stm32_hal::timer::low_level::{OutputCompareMode, OutputPolarity, Timer};
use embassy_stm32_hal::timer::{
    Ch1, Ch2, Ch3, Ch4, Channel, Dma, GeneralInstance4Channel, TimerChannel,
};
use embassy_stm32_hal::Peri;
use embassy_time::{Duration, Instant, Timer as EmbassyTimer};

#[cfg(not(feature = "defmt"))]
use crate::bidir_capture::decode_frame_bf_port_samples_u16;
#[cfg(feature = "defmt")]
use crate::bidir_capture::{decode_bf_raw_21, decode_frame_bf_port_samples_with_debug_u16};
use crate::command::{BidirTx, Command, DshotSpeed, EncodedFrame};
use crate::telemetry::{BidirDecoder, GcrFrame, TelemetryFrame, TelemetryPipelineError};

use super::{RuntimeTimeouts, Stm32BidirCapture};

const MAX_PORT_MOTORS: usize = 4;
// Keep only a minimal idle-high tail before switching to RX. Longer tails delay the
// TX-complete IRQ and directly eat into telemetry turnaround margin.
const TX_HOLD_SLOTS: usize = 1;
const TX_STATE_SLOTS: usize = 16 * 3 + TX_HOLD_SLOTS;
const MAX_CAPTURE_SAMPLES: usize = 512;
const PINS_PER_GPIO_PORT: u8 = 16;
const DEFAULT_ARM_DURATION: Duration = Duration::from_millis(3_000);
const DMA_IRQ_SLOTS: usize = 16;

type DmaIrqFn = unsafe fn(*mut ());

static DMA_IRQ_CTX: [AtomicPtr<()>; DMA_IRQ_SLOTS] =
    [const { AtomicPtr::new(ptr::null_mut()) }; DMA_IRQ_SLOTS];
static DMA_IRQ_FN: [AtomicPtr<()>; DMA_IRQ_SLOTS] =
    [const { AtomicPtr::new(ptr::null_mut()) }; DMA_IRQ_SLOTS];

#[derive(Clone, Copy)]
struct PacerTimerConfig {
    psc: u16,
    arr: u16,
    compare: u16,
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
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PortRuntimeError {
    TxTimeout,
    RxTimeout,
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

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH6 {
    const IRQ_SLOT: usize = 14;
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }
    fn stream_num() -> usize {
        6
    }
}

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
        let bsrr_ptr = regs.bsrr().as_ptr() as *mut u32;
        let idr_ptr = regs.idr().as_ptr() as *mut u16;

        Self {
            line: Flex::new(pin),
            port,
            pin_mask,
            bsrr_ptr,
            idr_ptr,
        }
    }

    fn enter_output_high(&mut self) {
        self.line.set_high();
        self.line.set_as_output(Speed::VeryHigh);
    }

    fn enter_input_pullup(&mut self, pull: Pull) {
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

struct RxMotorState {
    decoder: BidirDecoder,
    #[cfg(feature = "defmt")]
    decode_error_count: u32,
}

impl RxMotorState {
    fn new(mut cfg: Stm32BidirCapture) -> Self {
        cfg.oversampling.sample_bit_index = 0;
        let mut decoder = BidirDecoder::with_preamble_tuning(cfg.oversampling, cfg.preamble_tuning);
        decoder.set_stream_hint(cfg.decode_hint);
        Self {
            decoder,
            #[cfg(feature = "defmt")]
            decode_error_count: 0,
        }
    }
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

pub struct Stm32BidirPortController<'d, T, D, const N: usize>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    timer: Timer<'d, T>,
    dma_request: Request,
    pins: [DshotPortPin<'d>; N],
    group_mask: u32,
    channel: Channel,
    speed: DshotSpeed,
    timeouts: RuntimeTimeouts,
    capture_pull: Pull,
    sample_count: usize,
    tx_timer_cfg: PacerTimerConfig,
    rx_timer_cfg: PacerTimerConfig,
    rx: [RxMotorState; N],
    tx_words: [u32; TX_STATE_SLOTS],
    raw_samples: [u16; MAX_CAPTURE_SAMPLES],
    #[cfg(feature = "defmt")]
    capture_attempts: u32,
    #[cfg(feature = "defmt")]
    last_rx_handoff_us: u32,
    #[cfg(feature = "defmt")]
    last_rx_capture_us: u32,
    #[cfg(feature = "defmt")]
    last_tx_resume_us: u32,
    #[cfg(feature = "defmt")]
    last_release_gap_us: u32,
    #[cfg(feature = "defmt")]
    tx_irq_started_at: Instant,
    #[cfg(feature = "defmt")]
    last_tx_done_irq_us: u32,
    #[cfg(feature = "defmt")]
    last_rx_armed_irq_us: u32,
    irq_phase: AtomicU8,
    capture_requested: AtomicBool,
    _dma: PhantomData<D>,
}

impl<'d, T, D, const N: usize> Stm32BidirPortController<'d, T, D, N>
where
    T: GeneralInstance4Channel,
    D: RawDmaChannel,
{
    pub fn new_ch1(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
        pins: [DshotPortPin<'d>; N],
        rx_cfg: Stm32BidirCapture,
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        D: Dma<T, Ch1>,
    {
        Self::new_inner::<Ch1>(timer, dma, dma_irq, pins, rx_cfg, speed)
    }

    pub fn new_ch2(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
        pins: [DshotPortPin<'d>; N],
        rx_cfg: Stm32BidirCapture,
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        D: Dma<T, Ch2>,
    {
        Self::new_inner::<Ch2>(timer, dma, dma_irq, pins, rx_cfg, speed)
    }

    pub fn new_ch3(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
        pins: [DshotPortPin<'d>; N],
        rx_cfg: Stm32BidirCapture,
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        D: Dma<T, Ch3>,
    {
        Self::new_inner::<Ch3>(timer, dma, dma_irq, pins, rx_cfg, speed)
    }

    pub fn new_ch4(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
        pins: [DshotPortPin<'d>; N],
        rx_cfg: Stm32BidirCapture,
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        D: Dma<T, Ch4>,
    {
        Self::new_inner::<Ch4>(timer, dma, dma_irq, pins, rx_cfg, speed)
    }

    fn new_inner<C>(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        _dma_irq: impl Binding<D::Interrupt, InterruptHandler<D>> + 'd,
        mut pins: [DshotPortPin<'d>; N],
        rx_cfg: Stm32BidirCapture,
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        C: TimerChannel,
        D: Dma<T, C>,
    {
        if N == 0 || N > MAX_PORT_MOTORS {
            return Err(PortConfigError::InvalidMotorCount {
                requested: N,
                max_supported: MAX_PORT_MOTORS,
            });
        }
        if rx_cfg.sample_count == 0 || rx_cfg.sample_count > MAX_CAPTURE_SAMPLES {
            return Err(PortConfigError::SampleBufferTooSmall {
                requested: rx_cfg.sample_count,
                capacity: MAX_CAPTURE_SAMPLES,
            });
        }

        let first_port = pins[0].port();
        let mut group_mask = 0u32;
        for pin in pins.iter() {
            if pin.port() != first_port {
                return Err(PortConfigError::MixedPorts);
            }
            if (group_mask & pin.pin_mask()) != 0 {
                return Err(PortConfigError::DuplicatePins);
            }
            group_mask |= pin.pin_mask();
        }

        let dma_request = dma.request();
        dma.remap();
        drop(dma);
        let timer = Timer::new(timer);
        let tx_state_hz = speed.timing_hints().nominal_bitrate_hz * 3;
        let rx_sample_hz = tx_state_hz * 5 * 2 * rx_cfg.oversampling.oversampling as u32 / 24;
        let tx_timer_cfg =
            compute_pacer_timer_config(&timer, tx_state_hz, rx_cfg.pacer_compare_percent);
        let rx_timer_cfg =
            compute_pacer_timer_config(&timer, rx_sample_hz, rx_cfg.pacer_compare_percent);

        configure_pacer_timer(&timer, C::CHANNEL, tx_timer_cfg);
        for pin in pins.iter_mut() {
            pin.enter_input_pullup(rx_cfg.pull);
        }

        Ok(Self {
            timer,
            dma_request,
            pins,
            group_mask,
            channel: C::CHANNEL,
            speed,
            timeouts: rx_cfg.timeouts,
            capture_pull: rx_cfg.pull,
            sample_count: rx_cfg.sample_count,
            tx_timer_cfg,
            rx_timer_cfg,
            rx: array::from_fn(|_| RxMotorState::new(rx_cfg)),
            tx_words: [0; TX_STATE_SLOTS],
            raw_samples: [0; MAX_CAPTURE_SAMPLES],
            #[cfg(feature = "defmt")]
            capture_attempts: 0,
            #[cfg(feature = "defmt")]
            last_rx_handoff_us: 0,
            #[cfg(feature = "defmt")]
            last_rx_capture_us: 0,
            #[cfg(feature = "defmt")]
            last_tx_resume_us: 0,
            #[cfg(feature = "defmt")]
            last_release_gap_us: 0,
            #[cfg(feature = "defmt")]
            tx_irq_started_at: Instant::from_ticks(0),
            #[cfg(feature = "defmt")]
            last_tx_done_irq_us: 0,
            #[cfg(feature = "defmt")]
            last_rx_armed_irq_us: 0,
            irq_phase: AtomicU8::new(IrqPhase::Idle as u8),
            capture_requested: AtomicBool::new(false),
            _dma: PhantomData,
        })
    }

    pub async fn arm_for(&mut self, duration: Duration) -> Result<(), PortRuntimeError> {
        let frame_period =
            Duration::from_micros(self.speed.timing_hints().min_frame_period_us as u64);
        let deadline = Instant::now() + duration;
        let stop_frames = [BidirTx::command(Command::MotorStop).encode(); N];

        while Instant::now() < deadline {
            self.send_frames(stop_frames).await?;
            EmbassyTimer::after(frame_period).await;
        }

        Ok(())
    }

    pub async fn arm(&mut self) -> Result<(), PortRuntimeError> {
        self.arm_for(DEFAULT_ARM_DURATION).await
    }

    pub async fn send_frames(&mut self, frames: [EncodedFrame; N]) -> Result<(), PortRuntimeError> {
        self.build_tx_words(&frames);
        self.run_tx_dma().await
    }

    pub async fn send_throttles_and_receive(
        &mut self,
        throttles: [u16; N],
    ) -> Result<[Result<TelemetryFrame, TelemetryPipelineError>; N], PortRuntimeError> {
        self.send_frames_and_receive(
            throttles.map(|throttle| BidirTx::throttle_clamped(throttle).encode()),
        )
        .await
    }

    pub async fn send_frames_and_receive(
        &mut self,
        frames: [EncodedFrame; N],
    ) -> Result<[Result<TelemetryFrame, TelemetryPipelineError>; N], PortRuntimeError> {
        #[cfg(feature = "defmt")]
        {
            self.capture_attempts = self.capture_attempts.wrapping_add(1);
        }

        self.build_tx_words(&frames);
        self.run_tx_then_capture().await?;

        let samples = &self.raw_samples[..self.sample_count];
        let mut results = array::from_fn(|_| {
            Err(TelemetryPipelineError::Samples(
                crate::telemetry::SampleDecodeError::NoEdge,
            ))
        });
        for i in 0..N {
            #[cfg(feature = "defmt")]
            {
                let outcome = decode_frame_bf_port_samples_with_debug_u16(
                    &mut self.rx[i].decoder,
                    samples,
                    self.pins[i].pin_mask() as u16,
                );
                let bf_debug = outcome.debug;
                results[i] = outcome.frame;
                if results[i].is_err() && bf_debug.raw_21 != 0 {
                    if let Ok(payload) = self.rx[i].decoder.decode_payload(GcrFrame {
                        raw_21: bf_debug.raw_21,
                    }) {
                        if let Ok(frame) = self.rx[i].decoder.parse_payload(payload) {
                            if self.capture_attempts <= 8 || self.capture_attempts % 256 == 0 {
                                defmt::info!(
                                    "bdshot port salvage motor={} attempts={} bf_start={} bf_end={} bf_bits={} bf_raw=0x{:06x}",
                                    i,
                                    self.capture_attempts,
                                    bf_debug.start_margin,
                                    bf_debug.frame_end,
                                    bf_debug.bits_found,
                                    bf_debug.raw_21,
                                );
                            }
                            results[i] = Ok(frame);
                        }
                    }
                }
                if outcome.salvaged
                    && (self.capture_attempts <= 8 || self.capture_attempts % 256 == 0)
                {
                    defmt::info!(
                        "bdshot port salvage motor={} attempts={} bf_start={} bf_end={} bf_bits={} bf_raw=0x{:06x}",
                        i,
                        self.capture_attempts,
                        bf_debug.start_margin,
                        bf_debug.frame_end,
                        bf_debug.bits_found,
                        bf_debug.raw_21,
                    );
                }
                if let Err(err) = results[i] {
                    self.rx[i].decode_error_count = self.rx[i].decode_error_count.wrapping_add(1);
                    let count = self.rx[i].decode_error_count;
                    if count <= 8 || count % 256 == 0 {
                        let summary =
                            summarize_port_samples(samples, self.pins[i].pin_mask() as u16);
                        let stats = self.rx[i].decoder.stats();
                        let (bf_raw_valid, bf_payload) = if bf_debug.raw_21 != 0 {
                            match decode_bf_raw_21(&self.rx[i].decoder, bf_debug.raw_21) {
                                Ok(payload) => (true, payload.raw_16 as u32),
                                Err(_) => (false, 0),
                            }
                        } else {
                            (false, 0)
                        };
                        defmt::warn!(
                            "bdshot port decode err motor={} attempts={} count={} err={} tx_resume_us={} release_gap_us={} handoff_us={} capture_us={} highs={} lows={} first_low={:?} edges={} first_edge={:?} last_edge={:?} hint_skip={} stats ok={} no_edge={} short={} invalid_frame={} invalid_gcr={} invalid_crc={} start_margin={} bf_start={} bf_end={} bf_bits={} bf_raw=0x{:06x} bf_raw_valid={} bf_payload=0x{:04x} bf_runs={},{},{},{},{},{},{},{},{},{},{},{} raw_head=0x{:04x},0x{:04x},0x{:04x},0x{:04x}",
                            i,
                            self.capture_attempts,
                            count,
                            err,
                            self.last_tx_resume_us,
                            self.last_release_gap_us,
                            self.last_rx_handoff_us,
                            self.last_rx_capture_us,
                            summary.high_count,
                            summary.low_count,
                            summary.first_low,
                            summary.edge_count,
                            summary.first_edge,
                            summary.last_edge,
                            self.rx[i].decoder.stream_hint().preamble_skip,
                            stats.successful_frames,
                            stats.no_edge,
                            stats.frame_too_short,
                            stats.invalid_frame,
                            stats.invalid_gcr_symbol,
                            stats.invalid_crc,
                            stats.last_start_margin,
                            bf_debug.start_margin,
                            bf_debug.frame_end,
                            bf_debug.bits_found,
                            bf_debug.raw_21,
                            bf_raw_valid,
                            bf_payload,
                            bf_debug.runs[0],
                            bf_debug.runs[1],
                            bf_debug.runs[2],
                            bf_debug.runs[3],
                            bf_debug.runs[4],
                            bf_debug.runs[5],
                            bf_debug.runs[6],
                            bf_debug.runs[7],
                            bf_debug.runs[8],
                            bf_debug.runs[9],
                            bf_debug.runs[10],
                            bf_debug.runs[11],
                            samples.first().copied().unwrap_or(0) as u32,
                            samples.get(1).copied().unwrap_or(0) as u32,
                            samples.get(2).copied().unwrap_or(0) as u32,
                            samples.get(3).copied().unwrap_or(0) as u32,
                        );
                    }
                }
            }
            #[cfg(not(feature = "defmt"))]
            {
                results[i] = decode_frame_bf_port_samples_u16(
                    &mut self.rx[i].decoder,
                    samples,
                    self.pins[i].pin_mask() as u16,
                );
            }
        }
        Ok(results)
    }

    fn build_tx_words(&mut self, frames: &[EncodedFrame; N]) {
        let mut slot = 0usize;
        let group_low = self.group_mask << 16;

        for bit_idx in 0..16 {
            self.tx_words[slot] = group_low;
            slot += 1;

            let mut mid_high = 0u32;
            for (frame, pin) in frames.iter().zip(self.pins.iter()) {
                let shift = 15 - bit_idx;
                let bit_is_one = ((frame.payload >> shift) & 1) != 0;
                if !bit_is_one {
                    mid_high |= pin.pin_mask();
                }
            }
            self.tx_words[slot] = mid_high;
            slot += 1;

            self.tx_words[slot] = self.group_mask;
            slot += 1;
        }

        for hold_slot in &mut self.tx_words[slot..slot + TX_HOLD_SLOTS] {
            *hold_slot = self.group_mask;
        }
    }

    unsafe fn register_irq_callback(&mut self) {
        DMA_IRQ_CTX[D::IRQ_SLOT].store(self as *mut _ as *mut (), Ordering::Release);
        DMA_IRQ_FN[D::IRQ_SLOT].store(Self::dma_irq as *mut (), Ordering::Release);
    }

    fn clear_irq_callback(&mut self) {
        DMA_IRQ_CTX[D::IRQ_SLOT].store(ptr::null_mut(), Ordering::Release);
        DMA_IRQ_FN[D::IRQ_SLOT].store(ptr::null_mut(), Ordering::Release);
    }

    async fn wait_for_irq_phase(&mut self) -> Result<(), PortRuntimeError> {
        let mut tx_deadline = Instant::now() + self.timeouts.tx;
        let mut rx_deadline = Instant::now() + self.timeouts.rx;

        loop {
            match self.irq_phase.load(Ordering::Acquire) {
                x if x == IrqPhase::Done as u8 => return Ok(()),
                x if x == IrqPhase::Error as u8 => {
                    self.stop_dma_and_timer();
                    return Err(if self.capture_requested.load(Ordering::Acquire) {
                        PortRuntimeError::RxTimeout
                    } else {
                        PortRuntimeError::TxTimeout
                    });
                }
                x if x == IrqPhase::TxActive as u8 => {
                    if Instant::now() >= tx_deadline {
                        self.stop_dma_and_timer();
                        return Err(PortRuntimeError::TxTimeout);
                    }
                }
                x if x == IrqPhase::RxActive as u8 => {
                    if Instant::now() >= rx_deadline {
                        self.stop_dma_and_timer();
                        return Err(PortRuntimeError::RxTimeout);
                    }
                }
                _ => {}
            }

            EmbassyTimer::after(Duration::from_micros(1)).await;
            tx_deadline = tx_deadline.max(Instant::now());
            rx_deadline = rx_deadline.max(Instant::now());
        }
    }

    fn stop_dma_and_timer(&mut self) {
        dma_disable::<D>();
        dma_clear_flags::<D>();
        self.timer.stop();
        self.timer.set_cc_dma_enable_state(self.channel, false);
        self.clear_irq_callback();
        self.irq_phase
            .store(IrqPhase::Idle as u8, Ordering::Release);
    }

    async fn run_tx_dma(&mut self) -> Result<(), PortRuntimeError> {
        configure_pacer_timer(&self.timer, self.channel, self.tx_timer_cfg);
        self.timer.set_cc_dma_enable_state(self.channel, false);
        self.timer.reset();
        self.capture_requested.store(false, Ordering::Release);
        self.irq_phase
            .store(IrqPhase::TxActive as u8, Ordering::Release);
        #[cfg(feature = "defmt")]
        {
            self.tx_irq_started_at = Instant::now();
            self.last_tx_done_irq_us = 0;
            self.last_rx_armed_irq_us = 0;
            self.last_rx_handoff_us = 0;
            self.last_rx_capture_us = 0;
        }

        for pin in self.pins.iter_mut() {
            pin.enter_output_high();
        }

        unsafe {
            self.register_irq_callback();
            dma_start_write::<D>(
                self.dma_request,
                &self.tx_words as *const [u32] as *const u32,
                self.pins[0].bsrr_ptr(),
                self.tx_words.len(),
            );
            self.timer.set_cc_dma_enable_state(self.channel, true);
            self.timer.start();
        }

        let tx_result = self.wait_for_irq_phase().await;
        self.clear_irq_callback();

        if tx_result.is_err() {
            for pin in self.pins.iter_mut() {
                pin.enter_input_pullup(self.capture_pull);
            }
            self.timer.stop();
            self.timer.set_cc_dma_enable_state(self.channel, false);
            return Err(PortRuntimeError::TxTimeout);
        }

        #[cfg(feature = "defmt")]
        {
            self.last_tx_resume_us = self.last_tx_done_irq_us;
        }
        #[cfg(feature = "defmt")]
        let release_started_at = Instant::now();

        // The TX buffer already includes three trailing idle-high hold slots, matching the
        // Betaflight bitbang backend. As soon as DMA completes, release the line immediately so
        // the ESC can seize it for telemetry without extra software-added latency.
        for pin in self.pins.iter_mut() {
            pin.enter_input_pullup(self.capture_pull);
        }
        #[cfg(feature = "defmt")]
        {
            self.last_release_gap_us = Instant::now()
                .saturating_duration_since(release_started_at)
                .as_micros() as u32;
            if self.capture_attempts <= 8 || self.capture_attempts % 256 == 0 {
                defmt::debug!(
                    "bdshot port tx done attempts={} tx_resume_us={} release_gap_us={}",
                    self.capture_attempts,
                    self.last_tx_resume_us,
                    self.last_release_gap_us,
                );
            }
        }
        self.timer.stop();
        self.timer.set_cc_dma_enable_state(self.channel, false);

        Ok(())
    }

    async fn run_tx_then_capture(&mut self) -> Result<(), PortRuntimeError> {
        configure_pacer_timer(&self.timer, self.channel, self.tx_timer_cfg);
        self.timer.set_cc_dma_enable_state(self.channel, false);
        self.timer.reset();
        self.capture_requested.store(true, Ordering::Release);
        self.irq_phase
            .store(IrqPhase::TxActive as u8, Ordering::Release);
        #[cfg(feature = "defmt")]
        {
            self.tx_irq_started_at = Instant::now();
            self.last_tx_done_irq_us = 0;
            self.last_rx_armed_irq_us = 0;
            self.last_rx_handoff_us = 0;
            self.last_rx_capture_us = 0;
        }

        for pin in self.pins.iter_mut() {
            pin.enter_output_high();
        }

        unsafe {
            self.register_irq_callback();
            dma_start_write::<D>(
                self.dma_request,
                &self.tx_words as *const [u32] as *const u32,
                self.pins[0].bsrr_ptr(),
                self.tx_words.len(),
            );
            self.timer.set_cc_dma_enable_state(self.channel, true);
            self.timer.start();
        }

        let result = self.wait_for_irq_phase().await;
        self.clear_irq_callback();

        #[cfg(feature = "defmt")]
        {
            // In IRQ-handoff mode this is "TX armed -> TX DMA done IRQ".
            self.last_tx_resume_us = self.last_tx_done_irq_us;
            self.last_rx_handoff_us = self
                .last_rx_armed_irq_us
                .saturating_sub(self.last_tx_done_irq_us);
        }

        result
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
            this.irq_phase
                .store(IrqPhase::Error as u8, Ordering::Release);
            return;
        }

        if !isr.tcif(bit) {
            return;
        }

        regs.ifcr(D::stream_num() / 4)
            .write(|w| w.set_tcif(bit, true));
        this.timer.set_cc_dma_enable_state(this.channel, false);
        dma_disable::<D>();

        match this.irq_phase.load(Ordering::Acquire) {
            x if x == IrqPhase::TxActive as u8 => {
                #[cfg(feature = "defmt")]
                {
                    this.last_tx_done_irq_us = Instant::now()
                        .saturating_duration_since(this.tx_irq_started_at)
                        .as_micros() as u32;
                }

                if this.capture_requested.load(Ordering::Acquire) {
                    apply_pacer_timer_config_fast(&this.timer, this.channel, this.rx_timer_cfg);
                    this.timer.reset();

                    dma_start_read::<D>(
                        this.dma_request,
                        this.pins[0].idr_ptr(),
                        this.raw_samples.as_mut_ptr(),
                        this.sample_count,
                    );
                    for pin in this.pins.iter_mut() {
                        pin.enter_input_pullup(this.capture_pull);
                    }
                    this.irq_phase
                        .store(IrqPhase::RxActive as u8, Ordering::Release);
                    this.timer.set_cc_dma_enable_state(this.channel, true);
                    this.timer.start();
                    #[cfg(feature = "defmt")]
                    {
                        this.last_rx_armed_irq_us = Instant::now()
                            .saturating_duration_since(this.tx_irq_started_at)
                            .as_micros() as u32;
                    }
                } else {
                    for pin in this.pins.iter_mut() {
                        pin.enter_input_pullup(this.capture_pull);
                    }
                    this.timer.stop();
                    this.irq_phase
                        .store(IrqPhase::Done as u8, Ordering::Release);
                }
            }
            x if x == IrqPhase::RxActive as u8 => {
                this.timer.stop();
                #[cfg(feature = "defmt")]
                {
                    let now_us = Instant::now()
                        .saturating_duration_since(this.tx_irq_started_at)
                        .as_micros() as u32;
                    this.last_rx_capture_us = now_us.saturating_sub(this.last_rx_armed_irq_us);
                }
                this.irq_phase
                    .store(IrqPhase::Done as u8, Ordering::Release);
            }
            _ => {}
        }
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
    let compare = ((period_ticks.saturating_mul(compare_percent as u32)) / 100)
        .clamp(1, period_ticks.max(1)) as u16;

    PacerTimerConfig {
        psc: psc as u16,
        arr,
        compare,
    }
}

fn dma_clear_flags<D: RawDmaChannel>() {
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

fn dma_disable<D: RawDmaChannel>() {
    let regs = D::regs();
    let st = regs.st(D::stream_num());
    st.cr().modify(|w| w.set_en(false));
    while st.cr().read().en() {}
}

unsafe fn dma_start_write<D: RawDmaChannel>(
    request: Request,
    mem_addr: *const u32,
    peri_addr: *mut u32,
    len: usize,
) {
    let regs = D::regs();
    let st = regs.st(D::stream_num());
    dma_disable::<D>();
    dma_clear_flags::<D>();

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

unsafe fn dma_start_read<D: RawDmaChannel>(
    request: Request,
    peri_addr: *mut u16,
    mem_addr: *mut u16,
    len: usize,
) {
    let regs = D::regs();
    let st = regs.st(D::stream_num());
    dma_disable::<D>();
    dma_clear_flags::<D>();

    st.par().write_value(peri_addr as u32);
    st.m0ar().write_value(mem_addr as u32);
    st.ndtr().write_value(pac::dma::regs::Ndtr(len as _));
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
        w.set_chsel(request);
        w.set_pburst(pac::dma::vals::Burst::SINGLE);
        w.set_mburst(pac::dma::vals::Burst::SINGLE);
        w.set_pfctrl(pac::dma::vals::Pfctrl::DMA);
        w.set_en(true);
    });
}
