use crate::proto::pwm_capture_bf::core::{
    PwmCaptureDecodeConfig, PwmCaptureDecodeDebug, PwmCaptureDecodeError,
};
use crate::telemetry::{
    decode_gcr, parse_telemetry_payload, GcrDecodeError, GcrDecodeResult, GcrFrame,
    PayloadParseError, TelemetryFrame,
};

pub fn decode_gcr_from_timestamps(
    timestamps: &[u32],
    cfg: PwmCaptureDecodeConfig,
) -> Result<(GcrDecodeResult, PwmCaptureDecodeDebug), PwmCaptureDecodeError> {
    validate_config(cfg)?;

    if timestamps.len() < cfg.min_edge_count {
        return Err(PwmCaptureDecodeError::NotEnoughEdges {
            count: timestamps.len(),
            min_required: cfg.min_edge_count,
        });
    }

    let frame_bits = u32::from(cfg.frame_bits);
    let mut raw_21 = 0u32;
    let mut bits_decoded = 0u32;
    let mut deltas_used = 0usize;
    let mut zero_deltas_skipped = 0usize;
    let mut previous = timestamps[0];

    for &timestamp in timestamps.iter().skip(1) {
        let diff = timestamp.wrapping_sub(previous);
        previous = timestamp;

        if diff == 0 {
            zero_deltas_skipped += 1;
            continue;
        }

        let len =
            ((diff + u32::from(cfg.quantizer_rounding)) / u32::from(cfg.ticks_per_symbol)).max(1);
        bits_decoded = bits_decoded.saturating_add(len);
        if bits_decoded > frame_bits {
            return Err(PwmCaptureDecodeError::InvalidFrame);
        }

        raw_21 <<= len;
        raw_21 |= 1 << (len - 1);
        deltas_used += 1;

        if bits_decoded >= frame_bits {
            break;
        }
    }

    if deltas_used == 0 {
        return Err(PwmCaptureDecodeError::NoUsefulEdges);
    }

    let bits_before_tail = bits_decoded;
    let tail_len = frame_bits.saturating_sub(bits_decoded);
    if tail_len == 0 {
        return Err(PwmCaptureDecodeError::InvalidFrame);
    }

    raw_21 <<= tail_len;
    raw_21 |= 1 << (tail_len - 1);

    let debug = PwmCaptureDecodeDebug {
        edge_count: timestamps.len(),
        deltas_used,
        bits_decoded_before_tail: bits_before_tail as u8,
        bits_decoded_total: frame_bits as u8,
        zero_deltas_skipped,
        raw_21,
    };

    Ok((
        GcrDecodeResult {
            frame: GcrFrame { raw_21 },
            start_margin: 0,
        },
        debug,
    ))
}

pub fn decode_payload_from_timestamps(
    timestamps: &[u32],
    cfg: PwmCaptureDecodeConfig,
) -> Result<(u16, PwmCaptureDecodeDebug), DecodePayloadFromTimestampsError> {
    let (gcr, debug) = decode_gcr_from_timestamps(timestamps, cfg)?;
    let payload = decode_gcr(gcr.frame.raw_21).ok_or(GcrDecodeError::InvalidGcrSymbol)?;
    Ok((payload, debug))
}

pub fn decode_frame_from_timestamps(
    timestamps: &[u32],
    cfg: PwmCaptureDecodeConfig,
) -> Result<(TelemetryFrame, PwmCaptureDecodeDebug), DecodeFrameFromTimestampsError> {
    let (payload, debug) = decode_payload_from_timestamps(timestamps, cfg)?;
    let frame = parse_telemetry_payload(payload)?;
    Ok((frame, debug))
}

fn validate_config(cfg: PwmCaptureDecodeConfig) -> Result<(), PwmCaptureDecodeError> {
    if cfg.frame_bits == 0 || cfg.frame_bits > 31 || cfg.ticks_per_symbol == 0 {
        return Err(PwmCaptureDecodeError::InvalidConfig);
    }

    if cfg.min_edge_count < 2 {
        return Err(PwmCaptureDecodeError::InvalidConfig);
    }

    Ok(())
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DecodePayloadFromTimestampsError {
    Capture(PwmCaptureDecodeError),
    Gcr(GcrDecodeError),
}

impl From<PwmCaptureDecodeError> for DecodePayloadFromTimestampsError {
    fn from(value: PwmCaptureDecodeError) -> Self {
        Self::Capture(value)
    }
}

impl From<GcrDecodeError> for DecodePayloadFromTimestampsError {
    fn from(value: GcrDecodeError) -> Self {
        Self::Gcr(value)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DecodeFrameFromTimestampsError {
    Capture(PwmCaptureDecodeError),
    Gcr(GcrDecodeError),
    Payload(PayloadParseError),
}

impl From<DecodePayloadFromTimestampsError> for DecodeFrameFromTimestampsError {
    fn from(value: DecodePayloadFromTimestampsError) -> Self {
        match value {
            DecodePayloadFromTimestampsError::Capture(err) => Self::Capture(err),
            DecodePayloadFromTimestampsError::Gcr(err) => Self::Gcr(err),
        }
    }
}

impl From<PwmCaptureDecodeError> for DecodeFrameFromTimestampsError {
    fn from(value: PwmCaptureDecodeError) -> Self {
        Self::Capture(value)
    }
}

impl From<GcrDecodeError> for DecodeFrameFromTimestampsError {
    fn from(value: GcrDecodeError) -> Self {
        Self::Gcr(value)
    }
}

impl From<PayloadParseError> for DecodeFrameFromTimestampsError {
    fn from(value: PayloadParseError) -> Self {
        Self::Payload(value)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::{encode_gcr, TelemetryFrame};

    fn telemetry_crc(data: u16) -> u16 {
        ((!(((data ^ (data >> 4) ^ (data >> 8)) & 0x0F) as u8)) & 0x0F) as u16
    }

    fn build_timestamps_from_raw_21(
        raw_21: u32,
        ticks_per_symbol: u32,
        start: u32,
    ) -> ([u32; 22], usize) {
        let mut ones = [0u8; 21];
        let mut ones_len = 0usize;
        let mut bit = 21usize;
        while bit > 0 {
            let idx = bit - 1;
            if ((raw_21 >> idx) & 1) != 0 {
                ones[ones_len] = idx as u8;
                ones_len += 1;
            }
            bit -= 1;
        }

        let mut timestamps = [0u32; 22];
        let mut ts_len = 1usize;
        let mut current = start;
        timestamps[0] = current;

        let mut i = 0usize;
        while i + 1 < ones_len {
            let run_len = u32::from(ones[i].saturating_sub(ones[i + 1]));
            current = current.wrapping_add(run_len * ticks_per_symbol);
            timestamps[ts_len] = current;
            ts_len += 1;
            i += 1;
        }

        (timestamps, ts_len)
    }

    #[test]
    fn decodes_gcr_from_betaflight_style_timestamps() {
        let payload = 0x1234;
        let raw_21 = encode_gcr(payload);
        let (timestamps, len) = build_timestamps_from_raw_21(raw_21, 16, 1000);

        let (gcr, debug) =
            decode_gcr_from_timestamps(&timestamps[..len], PwmCaptureDecodeConfig::default())
                .unwrap();

        assert_eq!(gcr.frame.raw_21, raw_21);
        assert_eq!(debug.bits_decoded_total, 21);
    }

    #[test]
    fn decodes_frame_from_betaflight_style_timestamps() {
        let data = 0x5A5u16;
        let payload = (data << 4) | telemetry_crc(data);
        let raw_21 = encode_gcr(payload);
        let (timestamps, len) = build_timestamps_from_raw_21(raw_21, 16, 500);

        let (frame, _) =
            decode_frame_from_timestamps(&timestamps[..len], PwmCaptureDecodeConfig::default())
                .unwrap();

        assert_eq!(frame, TelemetryFrame::Erpm(crate::ErpmReading::new(1684)));
    }

    #[test]
    fn rejects_too_few_edges() {
        let timestamps = [10u32, 26, 42, 58];
        let err =
            decode_gcr_from_timestamps(&timestamps, PwmCaptureDecodeConfig::default()).unwrap_err();

        assert_eq!(
            err,
            PwmCaptureDecodeError::NotEnoughEdges {
                count: 4,
                min_required: 8,
            }
        );
    }

    #[test]
    fn skips_duplicate_timestamps() {
        let payload = 0x1234;
        let raw_21 = encode_gcr(payload);
        let (mut timestamps, len) = build_timestamps_from_raw_21(raw_21, 16, 1000);
        timestamps[len] = timestamps[len - 1];

        let (gcr, debug) =
            decode_gcr_from_timestamps(&timestamps[..=len], PwmCaptureDecodeConfig::default())
                .unwrap();

        assert_eq!(gcr.frame.raw_21, raw_21);
        assert_eq!(debug.zero_deltas_skipped, 1);
    }
}
