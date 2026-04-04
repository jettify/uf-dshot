#![no_std]

mod bidir_capture;
mod command;
#[cfg(all(feature = "embassy-stm32", target_arch = "arm", target_os = "none"))]
pub mod embassy_stm32;
mod telemetry;

pub use command::{
    Command,
    CommandTiming,
    DshotMode,
    DshotSpeed,
    DshotTx,
    EncodedFrame,
    FrameTimingHints,
    ThrottleError,
    WaveformTicks,
    WaveformTiming,
};
pub use telemetry::{
    BidirDecoder,
    ErpmReading,
    OversamplingConfig,
    PreambleTuningConfig,
    TelemetryError,
    TelemetryDecoderStats,
    TelemetryFrame,
    parse_telemetry_payload,
};
