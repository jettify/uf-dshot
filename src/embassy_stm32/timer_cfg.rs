use embassy_stm32_hal::timer::low_level::{OutputCompareMode, OutputPolarity, Timer};
use embassy_stm32_hal::timer::{Channel, GeneralInstance4Channel};

use super::driver::DshotConfig;

#[derive(Clone, Copy)]
pub(crate) struct PacerTimerConfig {
    pub(crate) psc: u16,
    pub(crate) arr: u16,
    pub(crate) compare: u16,
}

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
    regs.arr().write(|r| r.set_arr(cfg.arr.into()));
    timer.set_compare_value(channel, cfg.compare.into());
    timer.generate_update_event();
    let _ = timer.clear_update_interrupt();
}

pub(crate) fn switch_pacer_timer_config_fast<T: GeneralInstance4Channel>(
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

pub(crate) fn compute_pacer_timer_config<T: GeneralInstance4Channel>(
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

pub(crate) fn compute_rx_timer_config<T: GeneralInstance4Channel>(
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
