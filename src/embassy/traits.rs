use core::future::Future;

use crate::bidir::BidirTelemetryFrame;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BidirControllerError<TxE, RxE> {
    Tx(TxE),
    Rx(RxE),
}

pub trait TxTransport {
    type Error;
    type SendFuture<'a>: Future<Output = Result<(), Self::Error>>
    where
        Self: 'a;

    fn send_payload(&mut self, payload: u16) -> Self::SendFuture<'_>;
}

pub trait TelemetryRxTransport {
    type Error;
    type ReceiveFuture<'a>: Future<Output = Result<BidirTelemetryFrame, Self::Error>>
    where
        Self: 'a;

    fn receive_telemetry(&mut self) -> Self::ReceiveFuture<'_>;
}

pub trait BidirController {
    type TxError;
    type RxError;
    type ExchangeFuture<'a>: Future<
            Output = Result<BidirTelemetryFrame, BidirControllerError<Self::TxError, Self::RxError>>,
        >
    where
        Self: 'a;

    fn send_and_receive(&mut self, payload: u16) -> Self::ExchangeFuture<'_>;
}
