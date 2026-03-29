mod core;
mod decode;

#[cfg(all(feature = "embassy-stm32", target_arch = "arm", target_os = "none"))]
pub mod stm32;

pub use core::{
    PwmCaptureDecodeConfig, PwmCaptureDecodeDebug, PwmCaptureDecodeError, BF_CAPTURE_FILTER,
    BF_FRAME_BITS, BF_MIN_EDGE_COUNT, BF_QUANTIZER_DIVISOR, BF_QUANTIZER_ROUNDING,
};
pub use decode::{
    decode_frame_from_timestamps, decode_gcr_from_timestamps, decode_payload_from_timestamps,
    DecodeFrameFromTimestampsError, DecodePayloadFromTimestampsError,
};
