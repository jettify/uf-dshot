/// Fixed commands that occupy the lower 48 speed values.
///
/// Some commands need to be sent multiple times to be acted upon to prevent accidental bit-flips
/// wreaking havoc.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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
    ESCInfo,
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

/// Represents the value to be sent in a DShot frame, which can be
/// either a throttle value or a command.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DShotValue {
    Throttle(u16),
    Command(Command),
}

impl DShotValue {
    pub fn new_clipped_throttle(throttle: u16) -> Self {
        Self::Throttle(u16::min(throttle, 1999))
    }
    pub fn new_command(command: Command) -> Self {
        Self::Command(command)
    }
}

pub(crate) fn to_payload(
    value: DShotValue,
    telemetry_request: bool,
    crc_calculator: fn(u16) -> u8,
) -> u16 {
    let dshot_value = match value {
        // Commands 0-47 map to DShot values 0-47.
        DShotValue::Command(c) => c as u16,
        // Throttle 1-1999 maps to DShot values 48-2046.
        DShotValue::Throttle(t) => t + 47,
    };

    let mut payload = dshot_value << 1;
    if telemetry_request {
        payload |= 1;
    }

    let crc = crc_calculator(payload);
    (payload << 4) | (crc as u16)
}
