use crate::telemetry::{
    BidirDecoder, DecodeHint, GcrDecodeError, GcrDecodeResult, GcrFrame, OversamplingConfig,
    PayloadParseError, SampleDecodeError, TelemetryFrame, TelemetryPayload, TelemetryPipelineError,
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct DecodeDebug {
    pub(crate) start_margin: usize,
    pub(crate) frame_end: usize,
    pub(crate) bits_found: u32,
    pub(crate) raw_21: u32,
    pub(crate) run_count: usize,
    pub(crate) runs: [u8; 12],
}

impl DecodeDebug {
    pub(crate) const fn new() -> Self {
        Self {
            start_margin: 0,
            frame_end: 0,
            bits_found: 0,
            raw_21: 0,
            run_count: 0,
            runs: [0; 12],
        }
    }

    fn push_run(&mut self, run_bits: u32) {
        if self.run_count < self.runs.len() {
            self.runs[self.run_count] = run_bits as u8;
        }
        self.run_count += 1;
    }

    fn phase_adjusted(self, phase: usize) -> Self {
        let mut out = self;
        out.start_margin = out.start_margin.saturating_add(phase);
        out.frame_end = out.frame_end.saturating_add(phase);
        out
    }
}

#[derive(Debug)]
#[allow(dead_code)]
pub(crate) struct DecodeOutcome {
    pub(crate) frame: Result<TelemetryFrame, TelemetryPipelineError>,
    pub(crate) debug: DecodeDebug,
    pub(crate) salvaged: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum CandidateRank {
    Sample = 0,
    Gcr = 1,
    Payload = 2,
    Frame = 3,
}

#[derive(Clone, Debug)]
struct PhaseCandidate {
    rank: CandidateRank,
    quality: u8,
    frame: Result<TelemetryFrame, TelemetryPipelineError>,
    debug: DecodeDebug,
    gcr: Option<GcrDecodeResult>,
}

fn candidate_better(candidate: &PhaseCandidate, best: &PhaseCandidate) -> bool {
    candidate.rank > best.rank
        || (candidate.rank == best.rank
            && (candidate.quality > best.quality
                || (candidate.quality == best.quality
                    && candidate.debug.bits_found >= best.debug.bits_found)))
}

fn payload_quality(err: PayloadParseError) -> u8 {
    match err {
        PayloadParseError::InvalidCrc {
            calculated_crc,
            packet_crc,
        } => 16u8.saturating_sub((calculated_crc ^ packet_crc).count_ones() as u8),
        PayloadParseError::InvalidErpmPeriod => 0,
    }
}

fn try_salvage_single_bit(decoder: &BidirDecoder, raw_21: u32) -> Option<u32> {
    for bit in 0..decoder.cfg.frame_bits.min(21) {
        let candidate = raw_21 ^ (1u32 << bit);
        let payload = match decoder.decode_payload(GcrFrame { raw_21: candidate }) {
            Ok(payload) => payload,
            Err(_) => continue,
        };
        if decoder.parse_payload(payload).is_ok() {
            return Some(candidate);
        }
    }
    None
}

fn try_salvage_two_bits(decoder: &BidirDecoder, raw_21: u32) -> Option<u32> {
    let frame_bits = decoder.cfg.frame_bits.min(21);
    for first in 0..frame_bits {
        for second in (first + 1)..frame_bits {
            let candidate = raw_21 ^ (1u32 << first) ^ (1u32 << second);
            let payload = match decoder.decode_payload(GcrFrame { raw_21: candidate }) {
                Ok(payload) => payload,
                Err(_) => continue,
            };
            if decoder.parse_payload(payload).is_ok() {
                return Some(candidate);
            }
        }
    }
    None
}

#[allow(dead_code)]
pub(crate) fn decode_frame_port_samples_u16(
    decoder: &mut BidirDecoder,
    samples: &[u16],
    bit_mask: u16,
) -> Result<TelemetryFrame, TelemetryPipelineError> {
    decode_frame_port_samples_with_debug_u16(decoder, samples, bit_mask).frame
}

#[allow(dead_code)]
pub(crate) fn decode_frame_strict_port_samples_u16(
    decoder: &mut BidirDecoder,
    samples: &[u16],
    bit_mask: u16,
) -> Result<TelemetryFrame, TelemetryPipelineError> {
    decode_frame_strict_port_samples_with_debug_u16(decoder, samples, bit_mask).frame
}

pub(crate) fn decode_frame_port_samples_with_debug_u16(
    decoder: &mut BidirDecoder,
    samples: &[u16],
    bit_mask: u16,
) -> DecodeOutcome {
    let mut hint = decoder.stream_hint();
    if hint.preamble_skip >= samples.len() {
        hint.preamble_skip = samples.len().saturating_sub(1);
    }

    let oversampling = decoder.cfg.oversampling as usize;
    let mut best = PhaseCandidate {
        rank: CandidateRank::Sample,
        quality: 0,
        frame: Err(TelemetryPipelineError::Samples(SampleDecodeError::NoEdge)),
        debug: DecodeDebug::new(),
        gcr: None,
    };

    for phase in 0..oversampling.max(1) {
        if phase >= samples.len() {
            break;
        }

        let phase_hint = DecodeHint {
            preamble_skip: hint.preamble_skip.saturating_sub(phase),
        };
        let candidate = {
            let mut debug = DecodeDebug::new();
            let gcr = decode_gcr_port_samples_u16(
                &samples[phase..],
                bit_mask,
                phase_hint,
                decoder.cfg,
                &mut debug,
            )
            .map(|mut gcr| {
                gcr.start_margin = gcr.start_margin.saturating_add(phase);
                gcr
            });
            let adjusted = debug.phase_adjusted(phase);
            match gcr {
                Ok(gcr_ok) => match decoder.decode_payload(gcr_ok.frame) {
                    Ok(payload) => match decoder.parse_payload(payload) {
                        Ok(frame) => PhaseCandidate {
                            rank: CandidateRank::Frame,
                            quality: u8::MAX,
                            frame: Ok(frame),
                            debug: adjusted,
                            gcr: Some(gcr_ok),
                        },
                        Err(err) => PhaseCandidate {
                            rank: CandidateRank::Payload,
                            quality: payload_quality(err),
                            frame: Err(TelemetryPipelineError::PayloadParse(err)),
                            debug: adjusted,
                            gcr: Some(gcr_ok),
                        },
                    },
                    Err(err) => PhaseCandidate {
                        rank: CandidateRank::Gcr,
                        quality: 0,
                        frame: Err(TelemetryPipelineError::GcrDecode(err)),
                        debug: adjusted,
                        gcr: Some(gcr_ok),
                    },
                },
                Err(err) => PhaseCandidate {
                    rank: CandidateRank::Sample,
                    quality: 0,
                    frame: Err(TelemetryPipelineError::Samples(err)),
                    debug: adjusted,
                    gcr: None,
                },
            }
        };

        if candidate_better(&candidate, &best) {
            best = candidate;
        }
    }

    if let Some(gcr) = best.gcr {
        if matches!(
            best.frame,
            Err(TelemetryPipelineError::PayloadParse(
                PayloadParseError::InvalidCrc { .. }
            ))
        ) {
            if let Some(raw_21) = try_salvage_single_bit(decoder, gcr.frame.raw_21) {
                let salvaged_gcr = GcrDecodeResult {
                    frame: GcrFrame { raw_21 },
                    start_margin: gcr.start_margin,
                };
                if let Ok(frame) = decoder.decode_frame_from_gcr_tuned(Ok(salvaged_gcr)) {
                    return DecodeOutcome {
                        frame: Ok(frame),
                        debug: best.debug,
                        salvaged: true,
                    };
                }
            }
            if best.debug.bits_found >= 19 {
                if let Some(raw_21) = try_salvage_two_bits(decoder, gcr.frame.raw_21) {
                    let salvaged_gcr = GcrDecodeResult {
                        frame: GcrFrame { raw_21 },
                        start_margin: gcr.start_margin,
                    };
                    if let Ok(frame) = decoder.decode_frame_from_gcr_tuned(Ok(salvaged_gcr)) {
                        return DecodeOutcome {
                            frame: Ok(frame),
                            debug: best.debug,
                            salvaged: true,
                        };
                    }
                }
            }
        }
        if let Ok(frame) = decoder.decode_frame_from_gcr_tuned(Ok(gcr)) {
            return DecodeOutcome {
                frame: Ok(frame),
                debug: best.debug,
                salvaged: best.rank != CandidateRank::Frame,
            };
        }
    }

    DecodeOutcome {
        frame: best.frame,
        debug: best.debug,
        salvaged: false,
    }
}

pub(crate) fn decode_frame_strict_port_samples_with_debug_u16(
    decoder: &mut BidirDecoder,
    samples: &[u16],
    bit_mask: u16,
) -> DecodeOutcome {
    let mut hint = decoder.stream_hint();
    if hint.preamble_skip >= samples.len() {
        hint.preamble_skip = samples.len().saturating_sub(1);
    }

    let mut debug = DecodeDebug::new();
    let gcr = decode_gcr_strict_port_samples_u16(samples, bit_mask, hint, decoder.cfg, &mut debug);
    if let Some(gcr_ok) = gcr.as_ref().ok().copied() {
        if let Ok(frame) = decoder.decode_frame_from_gcr_tuned(Ok(gcr_ok)) {
            return DecodeOutcome {
                frame: Ok(frame),
                debug,
                salvaged: false,
            };
        }
    }

    if let Ok(gcr_ok) = gcr {
        let payload_err = match decoder.decode_payload(gcr_ok.frame) {
            Ok(payload) => match decoder.parse_payload(payload) {
                Ok(frame) => {
                    return DecodeOutcome {
                        frame: Ok(frame),
                        debug,
                        salvaged: false,
                    };
                }
                Err(err) => TelemetryPipelineError::PayloadParse(err),
            },
            Err(err) => TelemetryPipelineError::GcrDecode(err),
        };

        if matches!(
            payload_err,
            TelemetryPipelineError::PayloadParse(PayloadParseError::InvalidCrc { .. })
        ) {
            if let Some(raw_21) = try_salvage_single_bit(decoder, gcr_ok.frame.raw_21) {
                let salvaged_gcr = GcrDecodeResult {
                    frame: GcrFrame { raw_21 },
                    start_margin: gcr_ok.start_margin,
                };
                if let Ok(frame) = decoder.decode_frame_from_gcr_tuned(Ok(salvaged_gcr)) {
                    return DecodeOutcome {
                        frame: Ok(frame),
                        debug,
                        salvaged: true,
                    };
                }
            }
            if debug.bits_found >= 19 {
                if let Some(raw_21) = try_salvage_two_bits(decoder, gcr_ok.frame.raw_21) {
                    let salvaged_gcr = GcrDecodeResult {
                        frame: GcrFrame { raw_21 },
                        start_margin: gcr_ok.start_margin,
                    };
                    if let Ok(frame) = decoder.decode_frame_from_gcr_tuned(Ok(salvaged_gcr)) {
                        return DecodeOutcome {
                            frame: Ok(frame),
                            debug,
                            salvaged: true,
                        };
                    }
                }
            }
        }

        return DecodeOutcome {
            frame: Err(payload_err),
            debug,
            salvaged: false,
        };
    }

    DecodeOutcome {
        frame: decoder.decode_frame_from_gcr_tuned(gcr),
        debug,
        salvaged: false,
    }
}

#[allow(dead_code)]
pub(crate) fn decode_gcr_port_samples_u16(
    samples: &[u16],
    bit_mask: u16,
    hint: DecodeHint,
    cfg: OversamplingConfig,
    debug: &mut DecodeDebug,
) -> Result<GcrDecodeResult, SampleDecodeError> {
    if cfg.oversampling == 0 || cfg.frame_bits == 0 || cfg.min_detected_bits > cfg.frame_bits {
        return Err(SampleDecodeError::InvalidConfig);
    }

    if bit_mask == 0 {
        return Err(SampleDecodeError::InvalidSampleBitIndex {
            bit: cfg.sample_bit_index,
        });
    }

    if hint.preamble_skip >= samples.len() {
        return Err(SampleDecodeError::NoEdge);
    }

    let oversampling = cfg.oversampling as usize;
    let frame_bits = cfg.frame_bits as u32;
    let min_detected_bits = cfg.min_detected_bits as u32;
    let min_frame_samples =
        cfg.frame_bits.saturating_sub(cfg.bit_tolerance) as usize * oversampling;
    let max_frame_samples = (cfg.frame_bits as usize + cfg.bit_tolerance as usize) * oversampling;

    let start_margin = samples
        .iter()
        .enumerate()
        .skip(hint.preamble_skip)
        .find(|&(_, &sample)| (sample & bit_mask) == 0)
        .map(|(idx, _)| idx)
        .ok_or(SampleDecodeError::NoEdge)?;

    if samples.len() < start_margin + min_frame_samples {
        return Err(SampleDecodeError::FrameTooShort);
    }

    let frame_end = (start_margin + max_frame_samples).min(samples.len());
    let mut last_transition = None;
    for idx in (start_margin + 1)..frame_end {
        let prev_is_high = (samples[idx - 1] & bit_mask) != 0;
        let curr_is_high = (samples[idx] & bit_mask) != 0;
        if prev_is_high != curr_is_high {
            last_transition = Some(idx);
        }
    }
    let observed_end = last_transition.ok_or(SampleDecodeError::InvalidFrame)?;

    debug.start_margin = start_margin;
    debug.frame_end = observed_end;

    let mut gcr_value = 0u32;
    let mut bits_found = 0u32;
    let mut run_start = start_margin;
    while run_start < observed_end {
        let run_level_is_high = (samples[run_start] & bit_mask) != 0;
        let mut run_end = run_start + 1;
        while run_end < observed_end && ((samples[run_end] & bit_mask) != 0) == run_level_is_high {
            run_end += 1;
        }

        let run_samples = run_end - run_start;
        let run_bits =
            ((run_samples.saturating_add(oversampling / 2)) / oversampling).max(1) as u32;
        bits_found += run_bits;
        if bits_found > frame_bits {
            return Err(SampleDecodeError::InvalidFrame);
        }
        gcr_value <<= run_bits;
        gcr_value |= 1 << (run_bits - 1);
        debug.push_run(run_bits);
        run_start = run_end;
    }

    debug.bits_found = bits_found;
    if bits_found < min_detected_bits {
        return Err(SampleDecodeError::InvalidFrame);
    }

    let remaining_bits = frame_bits.saturating_sub(bits_found);
    if remaining_bits > 0 {
        gcr_value <<= remaining_bits;
        gcr_value |= 1 << (remaining_bits - 1);
        debug.push_run(remaining_bits);
    }
    debug.raw_21 = gcr_value;

    Ok(GcrDecodeResult {
        frame: GcrFrame { raw_21: gcr_value },
        start_margin,
    })
}

pub(crate) fn decode_gcr_strict_port_samples_u16(
    samples: &[u16],
    bit_mask: u16,
    hint: DecodeHint,
    cfg: OversamplingConfig,
    debug: &mut DecodeDebug,
) -> Result<GcrDecodeResult, SampleDecodeError> {
    if cfg.oversampling == 0 || cfg.frame_bits == 0 || cfg.min_detected_bits > cfg.frame_bits {
        return Err(SampleDecodeError::InvalidConfig);
    }

    if bit_mask == 0 {
        return Err(SampleDecodeError::InvalidSampleBitIndex {
            bit: cfg.sample_bit_index,
        });
    }

    if hint.preamble_skip >= samples.len() {
        return Err(SampleDecodeError::NoEdge);
    }

    let oversampling = cfg.oversampling as usize;
    let frame_bits = cfg.frame_bits as u32;
    let min_detected_bits = cfg.min_detected_bits as u32;
    let min_frame_samples =
        cfg.frame_bits.saturating_sub(cfg.bit_tolerance) as usize * oversampling;
    let max_frame_samples = (cfg.frame_bits as usize + cfg.bit_tolerance as usize) * oversampling;

    let start_margin = samples
        .iter()
        .enumerate()
        .skip(hint.preamble_skip)
        .find(|&(_, &sample)| (sample & bit_mask) == 0)
        .map(|(idx, _)| idx)
        .ok_or(SampleDecodeError::NoEdge)?;

    if samples.len() < start_margin + min_frame_samples {
        return Err(SampleDecodeError::FrameTooShort);
    }

    let frame_end = (start_margin + max_frame_samples).min(samples.len());
    let mut p = start_margin;
    let mut old_p = p;
    let mut gcr_value = 0u32;
    let mut bits_found = 0u32;
    let mut expect_high = true;

    debug.start_margin = start_margin;

    while p < frame_end {
        while p < frame_end && (((samples[p] & bit_mask) != 0) != expect_high) {
            p += 1;
        }

        if p >= frame_end {
            break;
        }

        debug.frame_end = p;
        let run_samples = p.saturating_sub(old_p).saturating_add(1);
        let run_bits = ((run_samples + 1) / oversampling).max(1) as u32;
        bits_found = bits_found.saturating_add(run_bits);
        if bits_found > frame_bits {
            return Err(SampleDecodeError::InvalidFrame);
        }
        gcr_value <<= run_bits;
        gcr_value |= 1 << (run_bits - 1);
        debug.push_run(run_bits);

        old_p = p;
        expect_high = !expect_high;
    }

    debug.bits_found = bits_found;
    if bits_found < min_detected_bits {
        return Err(SampleDecodeError::NoEdge);
    }

    let remaining_bits = frame_bits.saturating_sub(bits_found);
    if remaining_bits > 0 {
        gcr_value <<= remaining_bits;
        gcr_value |= 1 << (remaining_bits - 1);
        debug.push_run(remaining_bits);
    }
    debug.raw_21 = gcr_value;
    if debug.frame_end == 0 {
        debug.frame_end = frame_end.min(samples.len());
    }

    Ok(GcrDecodeResult {
        frame: GcrFrame { raw_21: gcr_value },
        start_margin,
    })
}

#[allow(dead_code)]
pub(crate) fn decode_raw_21(
    decoder: &BidirDecoder,
    raw_21: u32,
) -> Result<TelemetryPayload, GcrDecodeError> {
    decoder.decode_payload(GcrFrame { raw_21 })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::{encode_gcr, ErpmReading};

    fn pulse_lengths_from_gcr(mut gcr: u32) -> ([usize; 21], usize) {
        let mut out = [0usize; 21];
        let mut out_len = 0usize;

        gcr &= (1 << 21) - 1;
        let mut pos = 20i32;
        while pos >= 0 {
            let bit = (gcr >> pos) & 1;
            if bit == 0 {
                break;
            }

            let mut next = pos - 1;
            while next >= 0 {
                let next_bit = (gcr >> next) & 1;
                if next_bit == 1 {
                    break;
                }
                next -= 1;
            }

            out[out_len] = (pos - next) as usize;
            out_len += 1;
            pos = next;
        }

        (out, out_len)
    }

    fn build_port_samples_u16(
        bit_mask: u16,
        preamble_high: usize,
        pulse_bits: &[usize],
    ) -> [u16; 128] {
        let mut out = [bit_mask; 128];
        let mut write = 0usize;

        while write < preamble_high && write < out.len() {
            out[write] = bit_mask;
            write += 1;
        }

        let mut is_high = false;
        for &bit_len in pulse_bits {
            let width = (bit_len * 3).saturating_sub(1).max(1);
            let level = if is_high { bit_mask } else { 0 };
            let mut k = 0usize;
            while k < width && write < out.len() {
                out[write] = level;
                write += 1;
                k += 1;
            }
            is_high = !is_high;
        }

        out
    }

    #[test]
    fn port_decoder_reconstructs_known_frame() {
        let data = 0x5A5u16;
        let payload = (data << 4) | 0x5;
        let gcr = encode_gcr(payload);
        let (pulse_lengths, pulse_len_count) = pulse_lengths_from_gcr(gcr);
        let samples = build_port_samples_u16(1 << 9, 7, &pulse_lengths[..pulse_len_count]);

        let mut decoder = BidirDecoder::new(OversamplingConfig::default());
        let outcome = decode_frame_port_samples_with_debug_u16(&mut decoder, &samples, 1 << 9);

        assert_eq!(
            outcome.frame,
            Ok(TelemetryFrame::Erpm(ErpmReading::new(1684)))
        );
        assert!(outcome.debug.bits_found >= 18);
    }

    #[test]
    fn strict_port_decoder_reconstructs_known_frame() {
        let data = 0x5A5u16;
        let payload = (data << 4) | 0x5;
        let gcr = encode_gcr(payload);
        let (pulse_lengths, pulse_len_count) = pulse_lengths_from_gcr(gcr);
        let samples = build_port_samples_u16(1 << 9, 7, &pulse_lengths[..pulse_len_count]);

        let mut decoder = BidirDecoder::new(OversamplingConfig::default());
        let outcome =
            decode_frame_strict_port_samples_with_debug_u16(&mut decoder, &samples, 1 << 9);

        assert_eq!(
            outcome.frame,
            Ok(TelemetryFrame::Erpm(ErpmReading::new(1684)))
        );
        assert!(outcome.debug.bits_found >= 18);
    }

    #[test]
    fn port_decoder_reports_no_edge_on_idle_high() {
        let samples = [1 << 4; 96];
        let mut decoder = BidirDecoder::new(OversamplingConfig::default());
        let outcome = decode_frame_port_samples_with_debug_u16(&mut decoder, &samples, 1 << 4);

        assert_eq!(
            outcome.frame,
            Err(TelemetryPipelineError::Samples(SampleDecodeError::NoEdge))
        );
    }

    #[test]
    fn port_decoder_replays_truncated_capture_as_invalid_frame() {
        let samples = [
            0xE400, 0xE400, 0xE500, 0xE500, 0xE500, 0xE400, 0xE400, 0xE400, 0xE400, 0xE400, 0xE400,
            0xE500, 0xE500, 0xE500, 0xE400, 0xE400, 0xE400, 0xE400, 0xE400, 0xE400, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE400, 0xE400, 0xE400, 0xE400, 0xE400, 0xE400, 0xE400, 0xE400,
            0xE400, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
            0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500, 0xE500,
        ];

        let mut decoder = BidirDecoder::new(OversamplingConfig::default());
        let outcome = decode_frame_port_samples_with_debug_u16(&mut decoder, &samples, 0x0100);

        assert_eq!(
            outcome.frame,
            Err(TelemetryPipelineError::Samples(
                SampleDecodeError::InvalidFrame
            ))
        );
        assert_eq!(outcome.debug.start_margin, 1);
        assert_eq!(outcome.debug.frame_end, 34);
        assert_eq!(outcome.debug.bits_found, 12);
        assert_eq!(outcome.debug.raw_21, 0);
        assert_eq!(&outcome.debug.runs[..7], &[1, 1, 2, 1, 2, 2, 3]);
    }

    #[test]
    fn raw_candidate_decodes_for_crate_decoder() {
        let decoder = BidirDecoder::new(OversamplingConfig::default());
        let expected_payload = 0xad84u16;
        let raw_21 = encode_gcr(expected_payload);
        let payload = decode_raw_21(&decoder, raw_21).unwrap();
        assert_eq!(payload.raw_16, expected_payload);
    }

    #[test]
    fn raw_candidate_0_is_invalid_for_crate_decoder() {
        let decoder = BidirDecoder::new(OversamplingConfig::default());
        assert!(decode_raw_21(&decoder, 0x000000).is_err());
    }

    #[test]
    fn single_bit_salvage_recovers_observed_crc_miss() {
        let mut decoder = BidirDecoder::new(OversamplingConfig::default());
        let data = 0x7BEu16;
        let packet_crc = (!((data ^ (data >> 4) ^ (data >> 8)) & 0x0F)) & 0x0F;
        let expected_payload = (data << 4) | packet_crc as u16;
        let correct_raw_21 = encode_gcr(expected_payload);
        let buggy_raw_21 = correct_raw_21 ^ (1 << 5); // Flip one bit

        let raw_21 = try_salvage_single_bit(&decoder, buggy_raw_21).unwrap();
        assert_eq!(raw_21, correct_raw_21);

        let frame = decoder
            .decode_frame_from_gcr_tuned(Ok(GcrDecodeResult {
                frame: GcrFrame { raw_21 },
                start_margin: 0,
            }))
            .unwrap();
        assert_eq!(frame, TelemetryFrame::Erpm(ErpmReading::new(3568)));
    }

    #[test]
    fn integer_run_quantizer_matches_expected_bins() {
        let oversampling = 6usize;
        let decode =
            |samples: usize| ((samples.saturating_add(oversampling / 2)) / oversampling).max(1);

        assert_eq!(decode(1), 1);
        assert_eq!(decode(5), 1);
        assert_eq!(decode(6), 1);
        assert_eq!(decode(9), 2);
        assert_eq!(decode(12), 2);
        assert_eq!(decode(15), 3);
    }
}
