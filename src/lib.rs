#![no_std]

pub mod command;
#[cfg(all(feature = "embassy-stm32", target_arch = "arm", target_os = "none"))]
pub mod embassy_stm32;
pub mod telemetry;

pub use command::{
    // comment just to make one item per line
    BidirTx,
    Command,
    DshotSpeed,
    EncodedFrame,
    FrameTimingHints,
    FrameValue,
    Throttle,
    ThrottleError,
    UniTx,
    WaveformTicks,
    WaveformTiming,
};
pub use telemetry::{
    // comment just to make one item per line
    BidirDecoder,
    DecodeHint,
    DecodedTelemetry,
    ErpmReading,
    GcrDecodeError,
    GcrDecodeResult,
    GcrFrame,
    OversamplingConfig,
    PayloadParseError,
    PreambleTuningConfig,
    SampleDecodeError,
    TelemetryDecoderStats,
    TelemetryFrame,
    TelemetryPayload,
    TelemetryPipelineError,
};
