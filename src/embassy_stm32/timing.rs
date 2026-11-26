const MICROS_PER_SECOND: u64 = 1_000_000;
const ESC_TURNAROUND_US: u64 = 30;
const TELEMETRY_SLACK_US: u64 = 5;
const TELEMETRY_FRAME_BITS: u8 = 21;
const TIMER_REGISTER_PERIOD: u64 = u16::MAX as u64 + 1;
pub(crate) const MAX_CAPTURE_SAMPLES: usize = 512;

#[derive(Clone, Copy)]
pub(crate) struct PacerTimerConfig {
    pub(crate) psc: u16,
    pub(crate) arr: u16,
    pub(crate) compare: u16,
}

impl PacerTimerConfig {
    pub(crate) fn new(timer_hz: u32, target_hz: u32, compare_percent: u8) -> Option<Self> {
        if target_hz == 0 || target_hz > timer_hz / 2 {
            return None;
        }
        let target_hz = u64::from(target_hz);
        let total_ticks = (u64::from(timer_hz) + target_hz / 2) / target_hz;
        let prescaler = total_ticks.div_ceil(TIMER_REGISTER_PERIOD).max(1);
        let period = total_ticks / prescaler;
        // Keep ARR nonzero and the compare event within the active counting period.
        let compare = (period * u64::from(compare_percent.clamp(1, 99)) / 100).max(1);
        Some(Self {
            psc: (prescaler - 1) as u16,
            arr: (period - 1) as u16,
            compare: compare as u16,
        })
    }

    pub(crate) fn capture_samples(self, timer_hz: u32, window_us: u64) -> usize {
        let divider = (u64::from(self.psc) + 1) * (u64::from(self.arr) + 1);
        // Include one more sample to account for the phase of the first compare event.
        (window_us * u64::from(timer_hz)).div_ceil(divider * MICROS_PER_SECOND) as usize + 1
    }
}

pub(crate) fn telemetry_sample_hz(bitrate: u32, oversampling: u8, sample_percent: u8) -> u32 {
    // ESC telemetry runs at 5/4 of the selected DShot bitrate.
    (u64::from(bitrate) * 5 * u64::from(oversampling) * u64::from(sample_percent.clamp(1, 200))
        / 400) as u32
}

pub(crate) fn telemetry_window_us(bitrate: u32, frame_bits: u8, bit_tolerance: u8) -> u64 {
    let symbols = u64::from(frame_bits.max(TELEMETRY_FRAME_BITS)) + u64::from(bit_tolerance);
    let reply_us = (symbols * 4 * MICROS_PER_SECOND).div_ceil(u64::from(bitrate) * 5);
    ESC_TURNAROUND_US + reply_us + TELEMETRY_SLACK_US
}

pub(crate) enum ReplyWindow {
    Idle,
    Pending(u64),
    Until(u64),
}

impl ReplyWindow {
    pub(crate) fn finish(&mut self, released_at: Option<u64>, teardown_finished: u64) {
        if let Self::Pending(reply_ticks) = *self {
            // The reply duration is rounded up to ticks. Add one tick to cover
            // the fraction of a tick already elapsed at the release timestamp.
            let deadline = released_at
                .unwrap_or(teardown_finished)
                .saturating_add(reply_ticks)
                .saturating_add(1);
            *self = Self::Until(deadline);
        }
    }

    pub(crate) fn deadline(&self) -> Option<u64> {
        match *self {
            Self::Until(ticks) => Some(ticks),
            Self::Idle | Self::Pending(_) => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn capture_covers_dshot600_reply_at_eight_times_oversampling() {
        let timer_hz = 96_000_000;
        let bitrate = 600_000;
        let rate = telemetry_sample_hz(bitrate, 8, 100);
        let pacer = PacerTimerConfig::new(timer_hz, rate, 50).unwrap();
        let window = telemetry_window_us(bitrate, 21, 2);

        assert_eq!(rate, 6_000_000);
        assert_eq!(window, 66);
        assert_eq!(pacer.capture_samples(timer_hz, window), 397);
        assert_eq!(telemetry_window_us(bitrate, 20, 0), 63);
    }

    #[test]
    fn a_timer_rate_can_fit_while_its_capture_exceeds_buffer_capacity() {
        let timer_hz = 96_000_000;
        let bitrate = 600_000;
        let window = telemetry_window_us(bitrate, 21, 2);
        let eight_times =
            PacerTimerConfig::new(timer_hz, telemetry_sample_hz(bitrate, 8, 100), 50).unwrap();
        let twelve_times =
            PacerTimerConfig::new(timer_hz, telemetry_sample_hz(bitrate, 12, 100), 50).unwrap();

        assert!(eight_times.capture_samples(timer_hz, window) <= MAX_CAPTURE_SAMPLES);
        assert!(twelve_times.capture_samples(timer_hz, window) > MAX_CAPTURE_SAMPLES);
    }

    #[test]
    fn capture_uses_realized_timer_rate_and_prescaler() {
        let pacer = PacerTimerConfig::new(16_000_000, 6_000_000, 50).unwrap();
        assert_eq!(pacer.arr, 2);
        assert_eq!(pacer.capture_samples(16_000_000, 66), 353);

        let pacer = PacerTimerConfig::new(96_000_000, 500, 99).unwrap();
        assert_eq!(pacer.psc, 2);
        assert_eq!(pacer.capture_samples(96_000_000, 20_000), 11);
    }

    #[test]
    fn pacing_rejects_a_stopped_counter_and_keeps_compare_in_range() {
        let rate = telemetry_sample_hz(600_000, 16, 100);
        assert!(PacerTimerConfig::new(16_000_000, rate, 50).is_none());
        assert!(PacerTimerConfig::new(16_000_000, 0, 50).is_none());
        assert!(PacerTimerConfig::new(16_000_000, 8_000_001, 50).is_none());
        for percent in [0, 100, u8::MAX] {
            let pacer = PacerTimerConfig::new(16_000_000, 8_000_000, percent).unwrap();
            assert_eq!(pacer.arr, 1);
            assert_eq!(pacer.compare, 1);
        }
    }

    #[test]
    fn reply_deadline_uses_irq_release_or_falls_back_to_teardown() {
        let mut idle = ReplyWindow::Idle;
        idle.finish(None, 112);
        assert_eq!(idle.deadline(), None);

        let mut completed = ReplyWindow::Pending(66);
        completed.finish(Some(100), 112);
        assert_eq!(completed.deadline(), Some(167));

        let mut aborted = ReplyWindow::Pending(66);
        aborted.finish(None, 112);
        aborted.finish(None, 130);
        assert_eq!(aborted.deadline(), Some(179));
    }
}
