#![no_std]
pub mod command;
pub mod telemetry;

pub use command::{
    //
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
    //
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
