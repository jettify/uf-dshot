use crate::bit_banging::{decode_gcr_from_samples, DecodeError};
use crate::telemetry::{decode_gcr, parse_telemetry_payload, Telemetry, TelemetryError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum BidirDecodeError {
    BitBang(DecodeError),
    InvalidGcr,
    InvalidTelemetry(TelemetryError),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BidirTelemetryFrame {
    pub gcr_value: u32,
    pub payload: u16,
    pub telemetry: Telemetry,
    pub start_margin: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BidirDecodeState {
    preamble_skip: usize,
    min_margin: usize,
    frame_count: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct BidirDecodeConfig {
    pub target_start_margin: usize,
    pub check_interval_frames: u32,
}

impl Default for BidirDecodeConfig {
    fn default() -> Self {
        Self {
            target_start_margin: 5,
            check_interval_frames: 500,
        }
    }
}

impl BidirDecodeState {
    pub fn new() -> Self {
        Self {
            preamble_skip: 0,
            min_margin: usize::MAX,
            frame_count: 0,
        }
    }

    pub fn preamble_skip(&self) -> usize {
        self.preamble_skip
    }

    pub fn on_no_edge(&mut self) {
        if self.preamble_skip > 0 {
            self.preamble_skip -= 1;
        }
    }

    pub fn update_from_margin(&mut self, cfg: BidirDecodeConfig, start_margin: usize) {
        if start_margin < self.min_margin {
            self.min_margin = start_margin;
        }

        self.frame_count = self.frame_count.saturating_add(1);
        if self.frame_count >= cfg.check_interval_frames {
            if self.min_margin > cfg.target_start_margin {
                self.preamble_skip = self.min_margin - cfg.target_start_margin;
            } else {
                self.preamble_skip = 0;
            }
            self.min_margin = usize::MAX;
            self.frame_count = 0;
        }
    }
}

impl Default for BidirDecodeState {
    fn default() -> Self {
        Self::new()
    }
}

pub fn decode_telemetry_from_samples(
    buffer: &[u16],
    bit: u32,
    preamble_skip: usize,
) -> Result<BidirTelemetryFrame, BidirDecodeError> {
    let result = decode_gcr_from_samples(buffer, bit, preamble_skip)
        .map_err(BidirDecodeError::BitBang)?;

    let payload = decode_gcr(result.gcr_value).ok_or(BidirDecodeError::InvalidGcr)?;
    let telemetry = parse_telemetry_payload(payload).map_err(BidirDecodeError::InvalidTelemetry)?;

    Ok(BidirTelemetryFrame {
        gcr_value: result.gcr_value,
        payload,
        telemetry,
        start_margin: result.start_margin,
    })
}

pub fn decode_telemetry_with_state(
    state: &mut BidirDecodeState,
    cfg: BidirDecodeConfig,
    buffer: &[u16],
    bit: u32,
) -> Result<BidirTelemetryFrame, BidirDecodeError> {
    match decode_telemetry_from_samples(buffer, bit, state.preamble_skip()) {
        Ok(frame) => {
            state.update_from_margin(cfg, frame.start_margin);
            Ok(frame)
        }
        Err(BidirDecodeError::BitBang(DecodeError::NoEdge)) => {
            state.on_no_edge();
            Err(BidirDecodeError::BitBang(DecodeError::NoEdge))
        }
        Err(err) => Err(err),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::{encode_gcr, Telemetry};
    extern crate std;
    use std::vec::Vec;

    fn build_buffer(bit: u32, preamble_high: usize, pulses: &[usize]) -> Vec<u16> {
        let mask = 1u16 << bit;
        let mut buffer = Vec::new();
        buffer.resize(preamble_high, mask);

        let mut is_high = false;
        for &width in pulses {
            let value = if is_high { mask } else { 0 };
            for _ in 0..width {
                buffer.push(value);
            }
            is_high = !is_high;
        }
        // Decoder requires at least ~57 samples after first edge.
        while buffer.len() < preamble_high + 64 {
            buffer.push(mask);
        }
        buffer
    }

    fn gcr_to_pulse_widths(gcr_value: u32) -> Vec<usize> {
        let mut ones = Vec::new();
        for idx in 0..21 {
            let bit = (gcr_value >> (20 - idx)) & 1;
            if bit == 1 {
                ones.push(idx);
            }
        }

        let mut lengths = Vec::new();
        for pair in ones.windows(2) {
            lengths.push(pair[1] - pair[0]);
        }
        if let Some(&last) = ones.last() {
            lengths.push(21 - last);
        }
        lengths.into_iter().map(|len| len * 3 - 1).collect()
    }

    #[test]
    fn test_pipeline_decode_success() {
        let payload = 0x8109; // ERPM period 256 with correct CRC
        let gcr = encode_gcr(payload);
        let pulses = gcr_to_pulse_widths(gcr);
        let buffer = build_buffer(0, 8, &pulses);

        let frame = decode_telemetry_from_samples(&buffer, 0, 0).unwrap();
        assert_eq!(frame.gcr_value, gcr);
        assert_eq!(frame.payload, payload);
        assert_eq!(frame.telemetry, Telemetry::ERPM(256));
        assert_eq!(frame.start_margin, 8);
    }

    #[test]
    fn test_pipeline_decode_no_edge() {
        let buffer = [0u16; 80];
        let err = decode_telemetry_from_samples(&buffer, 0, 0).unwrap_err();
        assert_eq!(err, BidirDecodeError::BitBang(DecodeError::NoEdge));
    }

    #[test]
    fn test_decode_state_adaptation() {
        let mut state = BidirDecodeState::new();
        let cfg = BidirDecodeConfig {
            target_start_margin: 5,
            check_interval_frames: 2,
        };

        state.update_from_margin(cfg, 10);
        assert_eq!(state.preamble_skip(), 0);
        state.update_from_margin(cfg, 9);
        assert_eq!(state.preamble_skip(), 4);

        state.on_no_edge();
        assert_eq!(state.preamble_skip(), 3);
    }
}
