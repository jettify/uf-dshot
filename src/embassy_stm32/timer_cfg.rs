use embassy_stm32_hal::pac;
use embassy_stm32_hal::timer::low_level::{OutputCompareMode, OutputPolarity, Timer};
use embassy_stm32_hal::timer::{Channel, GeneralInstance4Channel};

use super::driver::PortConfigError;

pub(super) use crate::stm32_timing::PacerTimerConfig;

pub(crate) fn configure_pacer_timer<T: GeneralInstance4Channel>(
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

pub(crate) fn apply_pacer_timer_config_fast<T: GeneralInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    cfg: PacerTimerConfig,
) {
    let regs = timer.regs_gp16();
    regs.psc().write_value(cfg.psc);
    regs.arr().write(|r| r.set_arr(cfg.arr));
    timer.set_compare_value(channel, cfg.compare.into());
    timer.generate_update_event();
    let _ = timer.clear_update_interrupt();
}

#[inline(always)]
pub(crate) fn set_cc_dma_regs(regs: pac::timer::TimGp16, channel: Channel, enable: bool) {
    regs.dier().modify(|r| r.set_ccde(channel.index(), enable));
}

#[inline(always)]
pub(crate) fn stop_timer_regs(regs: pac::timer::TimGp16) {
    regs.cr1().modify(|r| r.set_cen(false));
}

#[inline(always)]
pub(crate) fn stop_timer_cc_dma_regs(regs: pac::timer::TimGp16, channel: Channel) {
    stop_timer_regs(regs);
    set_cc_dma_regs(regs, channel, false);
}

#[inline(always)]
pub(crate) fn switch_pacer_timer_regs_fast(
    regs: pac::timer::TimGp16,
    channel: Channel,
    cfg: PacerTimerConfig,
) {
    if regs.psc().read() != cfg.psc {
        regs.psc().write_value(cfg.psc);
    }
    regs.arr().write(|r| r.set_arr(cfg.arr));
    regs.ccr(channel.index()).write(|r| r.set_ccr(cfg.compare));
    regs.egr().write(|r| r.set_ug(true));
}

pub(crate) fn compute_pacer_timer_config(
    timer_hz: u32,
    target_hz: u32,
    compare_percent: u8,
) -> Result<PacerTimerConfig, PortConfigError> {
    PacerTimerConfig::new(timer_hz, target_hz, compare_percent).ok_or(
        PortConfigError::UnsupportedTimerRate {
            timer_hz,
            target_hz,
        },
    )
}
