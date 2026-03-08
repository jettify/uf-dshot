#[cfg(feature = "embassy")]
pub mod embassy;
#[cfg(feature = "std")]
pub mod mock;
pub mod telemetry;

pub use command::{
    BidirTx, Command, DshotSpeed, EncodedFrame, FrameTimingHints, FrameValue, Throttle,
    ThrottleError, UniTx, WaveformTicks, WaveformTiming,
};
pub use telemetry::{
    BidirDecoder, DecodeHint, DecodedTelemetry, ErpmReading, GcrDecodeError, GcrDecodeResult,
    GcrFrame, OversamplingConfig, PayloadParseError, SampleDecodeError, TelemetryFrame,
    TelemetryPayload, TelemetryPipelineError,
};
