use core::num::NonZeroU8;
use core::time::Duration;

const FRAME_BITS: usize = 16;
const CRC_MASK_4BIT: u16 = 0x0F;
// DShot reserves 0..=47 for commands; throttle starts at 48.
const THROTTLE_VALUE_OFFSET: u16 = 48;
const THROTTLE_MAX_RAW: u16 = 1999;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    MotorStop = 0,
    /// Wait at least 260ms before next command.
    Beep1,
    /// Wait at least 260ms before next command.
    Beep2,
    /// Wait at least 260ms before next command.
    Beep3,
    /// Wait at least 260ms before next command.
    Beep4,
    /// Wait at least 260ms before next command.
    Beep5,
    /// Wait at least 12ms before next command.
    EscInfo,
    /// Needs 6 transmissions.
    SpinDirection1,
    /// Needs 6 transmissions.
    SpinDirection2,
    /// Needs 6 transmissions.
    ThreeDModeOn,
    /// Needs 6 transmissions.
    ThreeDModeOff,
    SettingsRequest,
    /// Needs 6 transmissions. Wait at least 35ms before next command.
    SettingsSave,
    /// Needs 6 transmissions.
    ExtendedTelemetryEnable,
    /// Needs 6 transmissions.
    ExtendedTelemetryDisable,

    // 15-19 are unassigned.
    /// Needs 6 transmissions.
    SpinDirectionNormal = 20,
    /// Needs 6 transmissions.
    SpinDirectionReversed,
    Led0On,
    Led1On,
    Led2On,
    Led3On,
    Led0Off,
    Led1Off,
    Led2Off,
    Led3Off,
    AudioStreamModeToggle,
    SilentModeToggle,
    /// Needs 6 transmissions. Enables individual signal line commands.
    SignalLineTelemetryEnable,
    /// Needs 6 transmissions. Disables individual signal line commands.
    SignalLineTelemetryDisable,
    /// Needs 6 transmissions. Enables individual signal line commands.
    SignalLineContinuousERPMTelemetry,
    /// Needs 6 transmissions. Enables individual signal line commands.
    SignalLineContinuousERPMPeriodTelemetry,

    // 36-41 are unassigned.
    /// 1ºC per LSB.
    SignalLineTemperatureTelemetry = 42,
    /// 10mV per LSB, 40.95V max.
    SignalLineVoltageTelemetry,
    /// 100mA per LSB, 409.5A max.
    SignalLineCurrentTelemetry,
    /// 10mAh per LSB, 40.95Ah max.
    SignalLineConsumptionTelemetry,
    /// 100erpm per LSB, 409500erpm max.
    SignalLineERPMTelemetry,
    /// 16us per LSB, 65520us max.
    SignalLineERPMPeriodTelemetry,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CommandTiming {
    min_gap: Duration,
    repeat_count: NonZeroU8,
}

impl CommandTiming {
    pub const fn min_gap(self) -> Duration {
        self.min_gap
    }

    pub const fn repeat_count(self) -> NonZeroU8 {
        self.repeat_count
    }
}

impl Command {
    pub const fn min_repetitions(self) -> u8 {
        match self {
            Command::SpinDirection1
            | Command::SpinDirection2
            | Command::ThreeDModeOn
            | Command::ThreeDModeOff
            | Command::SettingsSave
            | Command::ExtendedTelemetryEnable
            | Command::ExtendedTelemetryDisable
            | Command::SpinDirectionNormal
            | Command::SpinDirectionReversed
            | Command::SignalLineTelemetryEnable
            | Command::SignalLineTelemetryDisable
            | Command::SignalLineContinuousERPMTelemetry
            | Command::SignalLineContinuousERPMPeriodTelemetry => 10,
            _ => 1,
        }
    }

    pub const fn min_delay_us(self) -> u32 {
        match self {
            Command::Beep1 | Command::Beep2 | Command::Beep3 | Command::Beep4 | Command::Beep5 => {
                260_000
            }
            Command::EscInfo => 12_000,
            Command::SettingsSave => 35_000,
            _ => 0,
        }
    }

    pub const fn exec_policy(self) -> CommandTiming {
        let repeat_count = match NonZeroU8::new(self.min_repetitions()) {
            Some(v) => v,
            None => unreachable!(),
        };

        CommandTiming {
            min_gap: Duration::from_micros(self.min_delay_us() as u64),
            repeat_count,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ThrottleError {
    OutOfRange { raw: u16 },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DshotMode {
    Standard,
    Bidirectional,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DshotTx {
    mode: DshotMode,
    request_separate_wire_telemetry: bool,
}

impl DshotTx {
    pub const fn standard() -> Self {
        Self {
            mode: DshotMode::Standard,
            request_separate_wire_telemetry: false,
        }
    }

    pub const fn bidirectional() -> Self {
        Self {
            mode: DshotMode::Bidirectional,
            request_separate_wire_telemetry: false,
        }
    }

    pub const fn mode(mode: DshotMode) -> Self {
        match mode {
            DshotMode::Standard => Self::standard(),
            DshotMode::Bidirectional => Self::bidirectional(),
        }
    }

    pub const fn dshot_mode(self) -> DshotMode {
        self.mode
    }

    pub const fn with_telemetry_request(mut self, enabled: bool) -> Self {
        if matches!(self.mode, DshotMode::Standard) {
            self.request_separate_wire_telemetry = enabled;
        }
        self
    }

    pub const fn telemetry_request(self) -> bool {
        match self.mode {
            DshotMode::Standard => self.request_separate_wire_telemetry,
            DshotMode::Bidirectional => true,
        }
    }

    pub fn throttle(self, raw: u16) -> Result<EncodedFrame, ThrottleError> {
        if raw > THROTTLE_MAX_RAW {
            return Err(ThrottleError::OutOfRange { raw });
        }
        Ok(self.encode_raw_value(raw + THROTTLE_VALUE_OFFSET, None))
    }

    pub fn throttle_clamped(self, raw: u16) -> EncodedFrame {
        self.encode_raw_value(raw.min(THROTTLE_MAX_RAW) + THROTTLE_VALUE_OFFSET, None)
    }

    pub fn throttle_with_telemetry_request(
        self,
        raw: u16,
        enabled: bool,
    ) -> Result<EncodedFrame, ThrottleError> {
        if raw > THROTTLE_MAX_RAW {
            return Err(ThrottleError::OutOfRange { raw });
        }
        Ok(self.encode_raw_value(raw + THROTTLE_VALUE_OFFSET, Some(enabled)))
    }

    pub fn command(self, command: Command) -> EncodedFrame {
        self.encode_raw_value(command as u16, None)
    }

    fn encode_raw_value(self, raw_value: u16, telemetry_override: Option<bool>) -> EncodedFrame {
        let telemetry_bit = match self.mode {
            DshotMode::Standard => {
                telemetry_override.unwrap_or(self.request_separate_wire_telemetry)
            }
            DshotMode::Bidirectional => true,
        };

        let data_12 = encode_data_12(raw_value, telemetry_bit);
        let base = base_crc(data_12);
        let crc_4 = match self.mode {
            DshotMode::Standard => base,
            DshotMode::Bidirectional => (!base) & 0x0F,
        };

        EncodedFrame {
            payload: (data_12 << 4) | u16::from(crc_4),
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EncodedFrame {
    pub payload: u16,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WaveformTiming {
    pub period_ticks: u16,
    pub bit0_high_ticks: u16,
    pub bit1_high_ticks: u16,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WaveformTicks {
    pub bit_high_ticks: [u16; 16],
    /// Optional trailing slot value for backends that append one explicit low reset period.
    pub reset_low_ticks: Option<u16>,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DshotSpeed {
    Dshot150,
    Dshot300,
    Dshot600,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrameTimingHints {
    pub nominal_bitrate_hz: u32,
    pub min_frame_period_us: u16,
}

impl DshotSpeed {
    pub const fn timing_hints(self) -> FrameTimingHints {
        match self {
            DshotSpeed::Dshot150 => FrameTimingHints {
                nominal_bitrate_hz: 150_000,
                min_frame_period_us: 107,
            },
            DshotSpeed::Dshot300 => FrameTimingHints {
                nominal_bitrate_hz: 300_000,
                min_frame_period_us: 54,
            },
            DshotSpeed::Dshot600 => FrameTimingHints {
                nominal_bitrate_hz: 600_000,
                min_frame_period_us: 27,
            },
        }
    }
}

impl EncodedFrame {
    pub const fn data_12(&self) -> u16 {
        self.payload >> 4
    }

    pub const fn crc_4(&self) -> u8 {
        (self.payload & CRC_MASK_4BIT) as u8
    }

    pub fn bits_msb_first(&self) -> [bool; 16] {
        let mut bits = [false; FRAME_BITS];
        for (i, bit) in bits.iter_mut().enumerate() {
            *bit = ((self.payload >> (FRAME_BITS - 1 - i)) & 1) != 0;
        }
        bits
    }

    pub fn to_waveform_ticks(
        &self,
        timing: WaveformTiming,
        with_reset_slot: bool,
    ) -> WaveformTicks {
        let bits = self.bits_msb_first();
        let mut bit_high_ticks = [0u16; FRAME_BITS];
        for (tick, bit) in bit_high_ticks.iter_mut().zip(bits.iter()) {
            *tick = if *bit {
                timing.bit1_high_ticks
            } else {
                timing.bit0_high_ticks
            };
        }

        WaveformTicks {
            bit_high_ticks,
            reset_low_ticks: if with_reset_slot { Some(0) } else { None },
        }
    }
}

fn encode_data_12(raw_value: u16, telemetry_request: bool) -> u16 {
    (raw_value << 1) | u16::from(telemetry_request)
}

fn base_crc(data_12: u16) -> u8 {
    ((data_12 ^ (data_12 >> 4) ^ (data_12 >> 8)) & CRC_MASK_4BIT) as u8
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn throttle_checked_and_clamped() {
        assert_eq!(
            DshotTx::bidirectional().throttle(2000),
            Err(ThrottleError::OutOfRange { raw: 2000 })
        );
        assert_eq!(
            DshotTx::bidirectional().throttle_clamped(5000).data_12(),
            0xFFF
        );
    }

    #[test]
    fn command_metadata() {
        assert_eq!(Command::Beep1.min_delay_us(), 260_000);
        assert_eq!(Command::Beep1.min_repetitions(), 1);
        assert_eq!(Command::SettingsSave.min_delay_us(), 35_000);
        assert_eq!(Command::SettingsSave.min_repetitions(), 10);
    }

    #[test]
    fn dshot_crc_vector() {
        let frame = DshotTx::standard().throttle(1000).unwrap();
        assert_eq!(frame.payload, 0x830B);
        assert_eq!(frame.data_12(), 0x830);
        assert_eq!(frame.crc_4(), 0xB);
    }

    #[test]
    fn dshot_crc_with_telemetry_request_vector() {
        let frame = DshotTx::standard()
            .with_telemetry_request(true)
            .throttle(1000)
            .unwrap();
        assert_eq!(frame.payload, 0x831A);
        assert_eq!(frame.data_12(), 0x831);
        assert_eq!(frame.crc_4(), 0xA);
    }

    #[test]
    fn bidir_inverted_crc_vector() {
        let frame = DshotTx::bidirectional().throttle(1000).unwrap();
        assert_eq!(frame.payload, 0x8315);
        assert_eq!(frame.data_12(), 0x831);
        assert_eq!(frame.crc_4(), 0x5);
    }

    #[test]
    fn throttle_zero_does_not_overlap_max_command() {
        let throttle_zero = DshotTx::standard().throttle(0).unwrap().data_12();
        let max_command = DshotTx::standard()
            .command(Command::SignalLineERPMPeriodTelemetry)
            .data_12();
        assert_ne!(throttle_zero, max_command);
    }

    #[test]
    fn bits_msb_first_order_matches_payload() {
        let frame = DshotTx::standard()
            .with_telemetry_request(true)
            .command(Command::Beep1);
        assert_eq!(frame.payload, 0x0033);
        assert_eq!(
            frame.bits_msb_first(),
            [
                false, false, false, false, false, false, false, false, false, false, true, true,
                false, false, true, true,
            ]
        );
    }

    #[test]
    fn waveform_ticks_mapping_and_reset_slot() {
        let frame = DshotTx::standard()
            .with_telemetry_request(true)
            .command(Command::Beep1);
        let timing = WaveformTiming {
            period_ticks: 100,
            bit0_high_ticks: 37,
            bit1_high_ticks: 75,
        };

        let with_reset = frame.to_waveform_ticks(timing, true);
        let no_reset = frame.to_waveform_ticks(timing, false);

        assert_eq!(with_reset.reset_low_ticks, Some(0));
        assert_eq!(no_reset.reset_low_ticks, None);
        assert_eq!(with_reset.bit_high_ticks[10], 75);
        assert_eq!(with_reset.bit_high_ticks[0], 37);
    }

    #[test]
    fn speed_hints_are_consistent() {
        let s150 = DshotSpeed::Dshot150.timing_hints();
        let s300 = DshotSpeed::Dshot300.timing_hints();
        let s600 = DshotSpeed::Dshot600.timing_hints();

        assert!(s150.min_frame_period_us > s300.min_frame_period_us);
        assert!(s300.min_frame_period_us > s600.min_frame_period_us);
        assert!(s150.nominal_bitrate_hz < s300.nominal_bitrate_hz);
        assert!(s300.nominal_bitrate_hz < s600.nominal_bitrate_hz);
    }

    #[test]
    fn exec_policy_remains_derived_from_metadata() {
        let policy = Command::EscInfo.exec_policy();
        assert_eq!(policy.min_gap(), Duration::from_micros(12_000));
        assert_eq!(policy.repeat_count().get(), 1);
    }

    #[test]
    fn per_frame_standard_telemetry_override_works() {
        let tx = DshotTx::standard().with_telemetry_request(false);
        let no_telem = tx.throttle(1000).unwrap();
        let with_telem = tx.throttle_with_telemetry_request(1000, true).unwrap();
        assert_eq!(no_telem.data_12(), 0x830);
        assert_eq!(with_telem.data_12(), 0x831);
    }

    #[test]
    fn bidir_forces_telemetry_even_when_override_is_false() {
        let tx = DshotTx::bidirectional();
        let frame = tx.throttle_with_telemetry_request(1000, false).unwrap();
        assert_eq!(frame.data_12(), 0x831);
    }
}
