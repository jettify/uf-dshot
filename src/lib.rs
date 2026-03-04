#![cfg_attr(not(feature = "std"), no_std)]

#[cfg(all(feature = "stm32f405", feature = "stm32f411"))]
compile_error!("Features `stm32f405` and `stm32f411` are mutually exclusive.");

#[cfg(any(
    all(feature = "stm32f405", not(feature = "embassy-stm32")),
    all(feature = "stm32f411", not(feature = "embassy-stm32"))
))]
compile_error!("Chip features require enabling `embassy-stm32`.");

#[cfg(all(
    feature = "embassy-stm32",
    not(any(feature = "stm32f405", feature = "stm32f411"))
))]
compile_error!("Feature `embassy-stm32` requires selecting one chip feature (`stm32f405` or `stm32f411`).");

pub mod bit_banging;
pub mod bidir;
pub mod common;
pub mod dshot;
pub mod telemetry;
#[cfg(feature = "embassy")]
pub mod embassy;
#[cfg(feature = "std")]
pub mod mock;

pub use common::{
    throttle_percent_to_value, throttle_value_to_percent, Command, DShotValue, MAX_THROTTLE,
};
pub use dshot::{BiDirDShotFrame, DShotFrame, DshotSpeed, Frame};
pub use telemetry::{
    decode_gcr, encode_gcr, erpm_period_to_erpm_x100, erpm_period_to_hz, erpm_period_to_rpm,
    erpm_x100_to_hz, erpm_x100_to_rpm, parse_telemetry_payload, Telemetry, TelemetryError,
};
pub use bidir::{
    decode_telemetry_from_samples, decode_telemetry_with_state, BidirDecodeConfig,
    BidirDecodeError, BidirDecodeState, BidirTelemetryFrame,
};
#[cfg(feature = "embassy-stm32")]
pub use embassy::{
    BidirLine, BidirRuntimeError, RxError, Stm32BidirController, Stm32BidirParts, TxError,
};
