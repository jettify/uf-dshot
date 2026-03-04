//! Experimental STM32 Embassy adapter layer.
//! This module is compile-tested but not hardware-validated yet.

pub mod bidir_line;
pub mod controller;
pub mod rx_dma;
pub mod tx_dma;

pub use bidir_line::BidirLine;
pub use controller::{BidirRuntimeError, Stm32BidirController, Stm32BidirParts};
pub use rx_dma::RxError;
pub use tx_dma::TxError;
