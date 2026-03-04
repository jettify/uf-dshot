use std::collections::VecDeque;

use crate::bidir::{BidirDecodeError, BidirTelemetryFrame};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MockLinkError {
    EmptyQueue,
    Decode(BidirDecodeError),
}

pub struct MockLink {
    sent_payloads: Vec<u16>,
    queued_telemetry: VecDeque<Result<BidirTelemetryFrame, MockLinkError>>,
}

impl MockLink {
    pub fn new() -> Self {
        Self {
            sent_payloads: Vec::new(),
            queued_telemetry: VecDeque::new(),
        }
    }

    pub fn record_payload(&mut self, payload: u16) {
        self.sent_payloads.push(payload);
    }

    pub fn sent_payloads(&self) -> &[u16] {
        &self.sent_payloads
    }

    pub fn push_telemetry(&mut self, frame: BidirTelemetryFrame) {
        self.queued_telemetry.push_back(Ok(frame));
    }

    pub fn push_decode_error(&mut self, err: BidirDecodeError) {
        self.queued_telemetry
            .push_back(Err(MockLinkError::Decode(err)));
    }

    pub fn receive_telemetry(&mut self) -> Result<BidirTelemetryFrame, MockLinkError> {
        self.queued_telemetry
            .pop_front()
            .ok_or(MockLinkError::EmptyQueue)?
    }
}

impl Default for MockLink {
    fn default() -> Self {
        Self::new()
    }
}
