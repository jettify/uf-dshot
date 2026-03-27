use core::array;
use core::marker::PhantomData;

use embassy_stm32_hal::dma::{ChannelInstance as DmaChannelInstance, Request};
use embassy_stm32_hal::gpio::{AnyPin, Flex, Pin, Speed};
use embassy_stm32_hal::pac;
use embassy_stm32_hal::timer::low_level::{OutputCompareMode, OutputPolarity, Timer};
use embassy_stm32_hal::timer::{
    Ch1, Ch2, Ch3, Ch4, Channel, Dma, GeneralInstance4Channel, TimerChannel,
};
use embassy_stm32_hal::Peri;
use embassy_time::{Duration, Instant, Timer as EmbassyTimer};

use crate::proto::bitbang_bf::{build_port_words, PortFrameError, SignalPolarity, TX_STATE_SLOTS};
use crate::{Command, DshotSpeed, EncodedFrame, UniTx};

const MAX_PORT_MOTORS: usize = 4;
const PINS_PER_GPIO_PORT: u8 = 16;
const DEFAULT_ARM_DURATION: Duration = Duration::from_millis(3_000);

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PortConfigError {
    InvalidMotorCount {
        requested: usize,
        max_supported: usize,
    },
    MixedPorts,
    DuplicatePins,
    Frame(PortFrameError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PortRuntimeError {
    TxTimeout,
    TxDmaError,
    Frame(PortFrameError),
}

#[derive(Clone, Copy)]
struct PacerTimerConfig {
    psc: u16,
    arr: u16,
    compare: u16,
}

pub trait RawDmaChannel: DmaChannelInstance {
    fn regs() -> pac::dma::Dma;
    fn stream_num() -> usize;
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH1 {
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        1
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH2 {
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        2
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH3 {
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        3
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH4 {
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        4
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH5 {
    fn regs() -> pac::dma::Dma {
        unsafe { pac::dma::Dma::from_ptr(pac::DMA2.as_ptr()) }
    }

    fn stream_num() -> usize {
        5
    }
}

impl RawDmaChannel for embassy_stm32_hal::peripherals::DMA2_CH6 {
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
        }
    }

    fn enter_output_low(&mut self) {
        self.line.set_low();
        self.line.set_as_output(Speed::VeryHigh);
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
    pub fn new_ch1(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        pins: [DshotPortPin<'d>; N],
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        D: Dma<T, Ch1>,
    {
        Self::new_inner::<Ch1>(timer, dma, pins, speed)
    }

    pub fn new_ch2(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        pins: [DshotPortPin<'d>; N],
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        D: Dma<T, Ch2>,
    {
        Self::new_inner::<Ch2>(timer, dma, pins, speed)
    }

    pub fn new_ch3(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        pins: [DshotPortPin<'d>; N],
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        D: Dma<T, Ch3>,
    {
        Self::new_inner::<Ch3>(timer, dma, pins, speed)
    }

    pub fn new_ch4(
        timer: Peri<'d, T>,
        dma: Peri<'d, D>,
        pins: [DshotPortPin<'d>; N],
        speed: DshotSpeed,
    ) -> Result<Self, PortConfigError>
    where
        D: Dma<T, Ch4>,
    {
        Self::new_inner::<Ch4>(timer, dma, pins, speed)
    }

    pub fn with_runtime_config(mut self, cfg: PortRuntimeConfig) -> Self {
        self.cfg = cfg;
        self.tx_timer_cfg = compute_pacer_timer_config(
            &self.timer,
            self.speed.timing_hints().nominal_bitrate_hz * 3,
            cfg.pacer_compare_percent,
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
        if N == 0 || N > MAX_PORT_MOTORS {
            return Err(PortConfigError::InvalidMotorCount {
                requested: N,
                max_supported: MAX_PORT_MOTORS,
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

        let pin_masks = array::from_fn(|idx| pins[idx].pin_mask());
        let dma_request = dma.request();
        dma.remap();
        drop(dma);

        let timer = Timer::new(timer);
        let cfg = PortRuntimeConfig::default();
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
            pin_masks,
            group_mask,
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
        let deadline = Instant::now() + duration;
        let stop_frames = [UniTx::command(Command::MotorStop).encode(); N];

        while Instant::now() < deadline {
            self.send_frames(stop_frames).await?;
            EmbassyTimer::after(frame_period).await;
        }

        Ok(())
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
        configure_pacer_timer(&self.timer, self.channel, self.tx_timer_cfg);
        self.timer.set_cc_dma_enable_state(self.channel, false);
        self.timer.reset();

        for pin in self.pins.iter_mut() {
            pin.enter_output_low();
        }

        unsafe {
            dma_start_write::<D>(
                self.dma_request,
                self.tx_words.as_ptr(),
                self.pins[0].bsrr_ptr(),
                self.tx_words.len(),
            );
        }

        self.timer.set_cc_dma_enable_state(self.channel, true);
        self.timer.start();

        let deadline = Instant::now() + self.cfg.tx_timeout;
        loop {
            if dma_transfer_error::<D>() {
                self.stop_tx_dma();
                return Err(PortRuntimeError::TxDmaError);
            }
            if dma_transfer_complete::<D>() {
                self.stop_tx_dma();
                return Ok(());
            }
            if Instant::now() >= deadline {
                self.stop_tx_dma();
                return Err(PortRuntimeError::TxTimeout);
            }

            EmbassyTimer::after(Duration::from_micros(1)).await;
        }
    }

    fn stop_tx_dma(&mut self) {
        dma_disable::<D>();
        dma_clear_flags::<D>();
        self.timer.stop();
        self.timer.set_cc_dma_enable_state(self.channel, false);
        for pin in self.pins.iter_mut() {
            pin.enter_output_low();
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
    let compare = ((period_ticks.saturating_mul(compare_percent.clamp(1, 99) as u32)) / 100)
        .clamp(1, period_ticks.max(1)) as u16;

    PacerTimerConfig {
        psc: psc as u16,
        arr,
        compare,
    }
}

fn dma_transfer_complete<D: RawDmaChannel>() -> bool {
    let regs = D::regs();
    let stream = D::stream_num();
    regs.isr(stream / 4).read().tcif(stream % 4)
}

fn dma_transfer_error<D: RawDmaChannel>() -> bool {
    let regs = D::regs();
    let stream = D::stream_num();
    regs.isr(stream / 4).read().teif(stream % 4)
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
