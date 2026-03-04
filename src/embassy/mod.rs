pub mod traits;
#[cfg(feature = "embassy-stm32")]
pub mod stm32;

pub use traits::{
    BidirController, BidirControllerError, TelemetryRxTransport, TxTransport,
};
#[cfg(feature = "embassy-stm32")]
pub use stm32::{
    BidirLine, BidirRuntimeError, RxError, Stm32BidirController, Stm32BidirParts, TxError,
};
