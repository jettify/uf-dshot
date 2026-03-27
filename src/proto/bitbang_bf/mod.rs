mod core;

#[cfg(all(feature = "embassy-stm32", target_arch = "arm", target_os = "none"))]
pub mod stm32;

pub use core::{
    build_port_words, PortFrameError, PortWords, SignalPolarity, TX_HOLD_SLOTS, TX_STATE_SLOTS,
};
