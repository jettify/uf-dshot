//! Safer, high-level Embassy-based replacement for the original Betaflight-style
//! bitbang STM32 runtime.
//!
//! This module intentionally avoids the raw DMA/IRQ/PAC plumbing from
//! [`crate::proto::bitbang_bf`] and instead exposes the simpler runtime
//! controllers built on the crate's higher-level Embassy STM32 support.

#[cfg(all(feature = "embassy-stm32", target_arch = "arm", target_os = "none"))]
pub mod stm32;
