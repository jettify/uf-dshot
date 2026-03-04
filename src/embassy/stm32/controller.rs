use embassy_stm32::dma::Channel as DmaChannel;
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::timer::{
    AdvancedInstance4Channel, BasicNoCr2Instance, Channel, CoreInstance,
};
use embassy_stm32::Peri;

use crate::bidir::{decode_telemetry_with_state, BidirDecodeConfig, BidirDecodeError, BidirDecodeState, BidirTelemetryFrame};

use super::bidir_line::BidirLine;
use super::rx_dma::{capture_port_samples, RxError};
use super::tx_dma::{send_waveform, setup_dshot_timer, TxError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BidirRuntimeError {
    Tx(TxError),
    Rx(RxError),
    Decode(BidirDecodeError),
}

/// Experimental STM32 bidirectional DShot controller for a single line.
///
/// This API is not hardware-validated yet.
pub struct Stm32BidirController<
    'd,
    TxT: CoreInstance,
    RxT: CoreInstance,
    TxC: DmaChannel,
    RxC: DmaChannel,
> {
    line: BidirLine<'d>,
    tx_timer: Timer<'d, TxT>,
    tx_channel: Channel,
    tx_dma: Peri<'d, TxC>,
    rx_timer: Timer<'d, RxT>,
    rx_dma: Peri<'d, RxC>,
    rx_idr_addr: *mut u16,
    rx_pin_bit: u32,
    rx_sample_hz: u32,
    decode_cfg: BidirDecodeConfig,
    decode_state: BidirDecodeState,
    max_duty: u16,
}

/// Static wiring/configuration used to construct [`Stm32BidirController`].
pub struct Stm32BidirParts<
    'd,
    TxT: CoreInstance,
    RxT: CoreInstance,
    TxC: DmaChannel,
    RxC: DmaChannel,
> {
    pub line: BidirLine<'d>,
    pub tx_timer: Timer<'d, TxT>,
    pub tx_channel: Channel,
    pub tx_dma: Peri<'d, TxC>,
    pub tx_bitrate_hz: u32,
    pub rx_timer: Timer<'d, RxT>,
    pub rx_dma: Peri<'d, RxC>,
    pub rx_idr_addr: *mut u16,
    pub rx_pin_bit: u32,
    pub rx_sample_hz: u32,
    pub decode_cfg: BidirDecodeConfig,
}

impl<'d, TxT, RxT, TxC: DmaChannel, RxC: DmaChannel> Stm32BidirController<'d, TxT, RxT, TxC, RxC>
where
    TxT: AdvancedInstance4Channel + BasicNoCr2Instance,
    RxT: CoreInstance + BasicNoCr2Instance,
{
    pub fn new(parts: Stm32BidirParts<'d, TxT, RxT, TxC, RxC>) -> Self {
        let max_duty = setup_dshot_timer(&parts.tx_timer, parts.tx_channel, parts.tx_bitrate_hz);

        Self {
            line: parts.line,
            tx_timer: parts.tx_timer,
            tx_channel: parts.tx_channel,
            tx_dma: parts.tx_dma,
            rx_timer: parts.rx_timer,
            rx_dma: parts.rx_dma,
            rx_idr_addr: parts.rx_idr_addr,
            rx_pin_bit: parts.rx_pin_bit,
            rx_sample_hz: parts.rx_sample_hz,
            decode_cfg: parts.decode_cfg,
            decode_state: BidirDecodeState::new(),
            max_duty,
        }
    }

    pub fn max_duty(&self) -> u16 {
        self.max_duty
    }

    pub async fn send_and_receive(
        &mut self,
        duty: &[u16],
        rx_buffer: &mut [u16],
    ) -> Result<BidirTelemetryFrame, BidirRuntimeError> {
        self.line.enter_tx();
        send_waveform(
            &self.tx_timer,
            self.tx_channel,
            self.tx_dma.reborrow(),
            duty,
        )
        .await
        .map_err(BidirRuntimeError::Tx)?;

        self.line.enter_rx_pullup();
        capture_port_samples(
            &self.rx_timer,
            self.rx_dma.reborrow(),
            self.rx_idr_addr,
            self.rx_sample_hz,
            rx_buffer,
        )
        .await
        .map_err(BidirRuntimeError::Rx)?;

        decode_telemetry_with_state(
            &mut self.decode_state,
            self.decode_cfg,
            rx_buffer,
            self.rx_pin_bit,
        )
        .map_err(BidirRuntimeError::Decode)
    }
}
