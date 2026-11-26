//! Embassy STM32 DShot and bidirectional DShot drivers.
//!
//! # Supported STM32 DMA Hardware
//!
//! This backend directly configures STM32 DMA v2 stream registers (`DMA2`, streams 0..=7,
//! `SxCR.CHSEL`, `SxFCR`, `LISR`/`HISR`, `LIFCR`/`HIFCR`), targeting STM32F2, STM32F4 (e.g.
//! STM32F405), and STM32F7 families. Only `DMA2` streams are exposed because `DMA1` cannot
//! access AHB1 GPIO registers (`BSRR`/`IDR`) on these families (ST AN4031). STM32H7
//! (`DMAMUX1`) support is planned separately.

mod dma_stream;
mod driver;
mod irq_state;
mod port_words;
mod timer_cfg;

pub use driver::*;
pub use port_words::{
    //
    build_port_words,
    PortFrameError,
    PortWords,
    SignalPolarity,
    FRAME_BITS,
    STATES_PER_BIT,
    TX_HOLD_SLOTS,
    TX_STATE_SLOTS,
};
