use std::collections::VecDeque;

use crate::bidir::BidirTelemetryFrame;
#[cfg(feature = "embassy")]
use crate::embassy::traits::{BidirController, BidirControllerError, TelemetryRxTransport, TxTransport};
#[cfg(feature = "embassy")]
use std::future::Ready;

use super::link::{MockLink, MockLinkError};

pub struct ScriptedTelemetry {
    queue: VecDeque<Result<BidirTelemetryFrame, MockLinkError>>,
}

impl ScriptedTelemetry {
    pub fn new() -> Self {
        Self {
            queue: VecDeque::new(),
        }
    }

    pub fn push_ok(&mut self, frame: BidirTelemetryFrame) {
        self.queue.push_back(Ok(frame));
    }

    pub fn push_err(&mut self, err: MockLinkError) {
        self.queue.push_back(Err(err));
    }

    fn pop_next(&mut self) -> Result<BidirTelemetryFrame, MockLinkError> {
        self.queue.pop_front().ok_or(MockLinkError::EmptyQueue)?
    }
}

impl Default for ScriptedTelemetry {
    fn default() -> Self {
        Self::new()
    }
}

pub struct MockController {
    pub link: MockLink,
    scripted: ScriptedTelemetry,
}

impl MockController {
    pub fn new() -> Self {
        Self {
            link: MockLink::new(),
            scripted: ScriptedTelemetry::new(),
        }
    }

    pub fn scripted_mut(&mut self) -> &mut ScriptedTelemetry {
        &mut self.scripted
    }

    pub fn send_payload(&mut self, payload: u16) {
        self.link.record_payload(payload);
    }

    pub fn send_and_receive(&mut self, payload: u16) -> Result<BidirTelemetryFrame, MockLinkError> {
        self.link.record_payload(payload);
        self.scripted.pop_next()
    }

    pub fn sent_payloads(&self) -> &[u16] {
        self.link.sent_payloads()
    }
}

impl Default for MockController {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "embassy")]
impl TxTransport for MockController {
    type Error = MockLinkError;
    type SendFuture<'a> = Ready<Result<(), Self::Error>>;

    fn send_payload(&mut self, payload: u16) -> Self::SendFuture<'_> {
        self.link.record_payload(payload);
        std::future::ready(Ok(()))
    }
}

#[cfg(feature = "embassy")]
impl TelemetryRxTransport for MockController {
    type Error = MockLinkError;
    type ReceiveFuture<'a> = Ready<Result<BidirTelemetryFrame, Self::Error>>;

    fn receive_telemetry(&mut self) -> Self::ReceiveFuture<'_> {
        std::future::ready(self.scripted.pop_next())
    }
}

#[cfg(feature = "embassy")]
impl BidirController for MockController {
    type TxError = MockLinkError;
    type RxError = MockLinkError;
    type ExchangeFuture<'a> = Ready<Result<BidirTelemetryFrame, BidirControllerError<Self::TxError, Self::RxError>>>;

    fn send_and_receive(&mut self, payload: u16) -> Self::ExchangeFuture<'_> {
        self.link.record_payload(payload);
        std::future::ready(self.scripted.pop_next().map_err(BidirControllerError::Rx))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bidir::BidirTelemetryFrame;
    use crate::telemetry::Telemetry;

    fn sample_frame() -> BidirTelemetryFrame {
        BidirTelemetryFrame {
            gcr_value: 1,
            payload: 2,
            telemetry: Telemetry::ERPM(128),
            start_margin: 3,
        }
    }

    #[test]
    fn scripted_controller_records_payload_and_replays_telemetry() {
        let mut ctrl = MockController::new();
        let frame = sample_frame();
        ctrl.scripted_mut().push_ok(frame);

        let out = ctrl.send_and_receive(0x1234).unwrap();
        assert_eq!(out, frame);
        assert_eq!(ctrl.sent_payloads(), &[0x1234]);
    }

    #[test]
    fn scripted_controller_surfaces_scripted_errors() {
        let mut ctrl = MockController::new();
        ctrl.scripted_mut().push_err(MockLinkError::EmptyQueue);

        let err = ctrl.send_and_receive(0x42).unwrap_err();
        assert_eq!(err, MockLinkError::EmptyQueue);
    }
}
