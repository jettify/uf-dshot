#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct OversamplingConfig {
    pub sample_bit_index: u8,
    pub oversampling: u8,
    pub frame_bits: u8,
    pub min_detected_bits: u8,
    pub bit_tolerance: u8,
}

impl Default for OversamplingConfig {
    fn default() -> Self {
        Self {
            sample_bit_index: 0,
            oversampling: 3,
            frame_bits: 21,
            min_detected_bits: 18,
            bit_tolerance: 2,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DecodeHint {
    pub preamble_skip: usize,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GcrFrame {
    pub raw_21: u32,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GcrDecodeResult {
    pub frame: GcrFrame,
    pub start_margin: usize,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TelemetryPayload {
    pub raw_16: u16,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ErpmReading {
    period: u16,
}

impl ErpmReading {
    pub const fn new(period: u16) -> Self {
        Self { period }
    }

    pub const fn period(self) -> u16 {
        self.period
    }

    pub const fn electrical_hz(self) -> u32 {
        if self.period == 0 {
            0
        } else {
            1_000_000 / self.period as u32
        }
    }

    pub const fn mechanical_rpm(self, pole_pairs: u8) -> u32 {
        if pole_pairs == 0 {
            0
        } else {
            (self.electrical_hz() * 60) / pole_pairs as u32
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TelemetryFrame {
    Erpm(ErpmReading),
    Temperature(u8),
    Voltage(u8),
    Current(u8),
    Debug1(u8),
    Debug2(u8),
    Debug3(u8),
    StateEvent(u8),
    UnknownExtended { type_id: u8, value: u8 },
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DecodedTelemetry {
    pub gcr: GcrDecodeResult,
    pub payload: TelemetryPayload,
    pub frame: TelemetryFrame,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SampleDecodeError {
    InvalidSampleBitIndex { bit: u8 },
    InvalidConfig,
    NoEdge,
    FrameTooShort,
    InvalidFrame,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum GcrDecodeError {
    InvalidGcrSymbol,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PayloadParseError {
    InvalidCrc { calculated_crc: u8, packet_crc: u8 },
}

pub type TelemetryError = PayloadParseError;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TelemetryPipelineError {
    Samples(SampleDecodeError),
    GcrDecode(GcrDecodeError),
    PayloadParse(PayloadParseError),
}

const STREAM_BUFFER_CAPACITY: usize = 256;

pub struct BidirDecoder {
    pub cfg: OversamplingConfig,
    stream_buf: [u16; STREAM_BUFFER_CAPACITY],
    stream_len: usize,
}

impl BidirDecoder {
    pub const fn new(cfg: OversamplingConfig) -> Self {
        Self {
            cfg,
            stream_buf: [0; STREAM_BUFFER_CAPACITY],
            stream_len: 0,
        }
    }

    pub fn decode(
        &self,
        samples: &[u16],
        hint: DecodeHint,
    ) -> Result<DecodedTelemetry, TelemetryPipelineError> {
        let gcr = self
            .decode_gcr(samples, hint)
            .map_err(TelemetryPipelineError::Samples)?;
        let payload = self
            .decode_payload(gcr.frame)
            .map_err(TelemetryPipelineError::GcrDecode)?;
        let frame = self
            .parse_payload(payload)
            .map_err(TelemetryPipelineError::PayloadParse)?;

        Ok(DecodedTelemetry {
            gcr,
            payload,
            frame,
        })
    }

    pub fn decode_gcr(
        &self,
        samples: &[u16],
        hint: DecodeHint,
    ) -> Result<GcrDecodeResult, SampleDecodeError> {
        decode_gcr_from_samples_cfg(samples, hint, self.cfg)
    }

    pub fn decode_payload(&self, gcr: GcrFrame) -> Result<TelemetryPayload, GcrDecodeError> {
        match decode_gcr(gcr.raw_21) {
            Some(raw_16) => Ok(TelemetryPayload { raw_16 }),
            None => Err(GcrDecodeError::InvalidGcrSymbol),
        }
    }

    pub fn parse_payload(
        &self,
        payload: TelemetryPayload,
    ) -> Result<TelemetryFrame, PayloadParseError> {
        parse_telemetry_payload(payload.raw_16)
    }

    pub fn push_sample(
        &mut self,
        sample: u16,
    ) -> Option<Result<DecodedTelemetry, TelemetryPipelineError>> {
        if self.stream_len < STREAM_BUFFER_CAPACITY {
            self.stream_buf[self.stream_len] = sample;
            self.stream_len += 1;
        } else {
            self.stream_buf.copy_within(1..STREAM_BUFFER_CAPACITY, 0);
            self.stream_buf[STREAM_BUFFER_CAPACITY - 1] = sample;
        }

        match self.decode(&self.stream_buf[..self.stream_len], DecodeHint::default()) {
            Ok(decoded) => {
                self.stream_len = 0;
                Some(Ok(decoded))
            }
            Err(TelemetryPipelineError::Samples(SampleDecodeError::NoEdge))
            | Err(TelemetryPipelineError::Samples(SampleDecodeError::FrameTooShort)) => None,
            Err(err) => Some(Err(err)),
        }
    }
}

fn calculate_crc(value: u16) -> u8 {
    ((value ^ (value >> 4) ^ (value >> 8)) & 0x0F) as u8
}

pub fn parse_telemetry_payload(payload: u16) -> Result<TelemetryFrame, PayloadParseError> {
    let data = payload >> 4;
    let packet_crc = (payload & 0x0F) as u8;
    let calculated_crc = calculate_crc(data);

    if packet_crc != calculated_crc {
        return Err(PayloadParseError::InvalidCrc {
            calculated_crc,
            packet_crc,
        });
    }

    let exponent = (data >> 9) & 0b111;
    let mantissa = data & 0x1FF;

    if (exponent & 0b001) != 0 && (mantissa & 0b100000000) == 0 {
        let telemetry_type = (data >> 8) as u8;
        let value = (data & 0xFF) as u8;
        let t = match telemetry_type {
            0x02 => TelemetryFrame::Temperature(value),
            0x04 => TelemetryFrame::Voltage(value),
            0x06 => TelemetryFrame::Current(value),
            0x08 => TelemetryFrame::Debug1(value),
            0x0A => TelemetryFrame::Debug2(value),
            0x0C => TelemetryFrame::Debug3(value),
            0x0E => TelemetryFrame::StateEvent(value),
            _ => TelemetryFrame::UnknownExtended {
                type_id: telemetry_type,
                value,
            },
        };
        Ok(t)
    } else {
        let period = mantissa << exponent;
        Ok(TelemetryFrame::Erpm(ErpmReading::new(period)))
    }
}

/// Convert a telemetry eRPM period into eRPM * 100.
/// Returns 0 if the period is zero.
pub fn erpm_period_to_erpm_x100(period: u16) -> u32 {
    if period == 0 {
        return 0;
    }
    let period = period as u32;
    (1_000_000 * 60 / 100 + period / 2) / period
}

/// Convert eRPM * 100 into mechanical RPM for a given number of pole pairs.
pub fn erpm_x100_to_rpm(erpm_x100: u32, pole_pairs: u8) -> u32 {
    if pole_pairs == 0 {
        return 0;
    }
    (erpm_x100 / 100) / (pole_pairs as u32)
}

/// Convert eRPM * 100 into mechanical Hz for a given number of pole pairs.
pub fn erpm_x100_to_hz(erpm_x100: u32, pole_pairs: u8) -> u32 {
    if pole_pairs == 0 {
        return 0;
    }
    (erpm_x100 / 100) / (pole_pairs as u32) / 60
}

/// Convert a telemetry eRPM period into mechanical RPM for a given number of pole pairs.
pub fn erpm_period_to_rpm(period: u16, pole_pairs: u8) -> u32 {
    let erpm_x100 = erpm_period_to_erpm_x100(period);
    erpm_x100_to_rpm(erpm_x100, pole_pairs)
}

/// Convert a telemetry eRPM period into mechanical Hz for a given number of pole pairs.
pub fn erpm_period_to_hz(period: u16, pole_pairs: u8) -> u32 {
    let erpm_x100 = erpm_period_to_erpm_x100(period);
    erpm_x100_to_hz(erpm_x100, pole_pairs)
}

const GCR_DECODE_TABLE: [u8; 32] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07, 0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF,
];

pub fn decode_gcr(gcr_value: u32) -> Option<u16> {
    let gcr_encoded = (gcr_value ^ (gcr_value >> 1)) & 0xFFFFF;

    let mut result: u16 = 0;
    let mut i = 0;
    while i < 4 {
        let chunk = (gcr_encoded >> (15 - i * 5)) & 0x1F;
        let nibble = GCR_DECODE_TABLE[chunk as usize];
        if nibble == 0xFF {
            return None;
        }
        result = (result << 4) | nibble as u16;
        i += 1;
    }
    Some(result)
}

const GCR_ENCODE_TABLE: [u8; 16] = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111, 0b11010, 0b01001,
    0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
];

pub fn encode_gcr(payload: u16) -> u32 {
    let mut gcr_encoded: u32 = 0;
    let mut i = 0;
    while i < 4 {
        let nibble = (payload >> (12 - i * 4)) & 0x0F;
        let chunk = GCR_ENCODE_TABLE[nibble as usize] as u32;
        gcr_encoded = (gcr_encoded << 5) | chunk;
        i += 1;
    }

    let mut gcr_value: u32 = 1 << 20;
    let mut last_bit = 1;
    i = 0;
    while i < 20 {
        let gcr_bit = (gcr_encoded >> (19 - i)) & 1;
        let current_bit = last_bit ^ gcr_bit;
        gcr_value |= current_bit << (19 - i);
        last_bit = current_bit;
        i += 1;
    }
    gcr_value
}

fn decode_gcr_from_samples_cfg(
    samples: &[u16],
    hint: DecodeHint,
    cfg: OversamplingConfig,
) -> Result<GcrDecodeResult, SampleDecodeError> {
    validate_oversampling_config(cfg)?;

    if cfg.sample_bit_index > 15 {
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

    let mask = 1u16 << cfg.sample_bit_index;
    let start_margin = samples
        .iter()
        .enumerate()
        .skip(hint.preamble_skip)
        .find(|&(_, &sample)| (sample & mask) == 0)
        .map(|(idx, _)| idx)
        .ok_or(SampleDecodeError::NoEdge)?;

    if samples.len() < start_margin + min_frame_samples {
        return Err(SampleDecodeError::FrameTooShort);
    }

    let search_window_len = (samples.len() - start_margin).min(max_frame_samples);
    let frame_samples = &samples[start_margin..start_margin + search_window_len];

    let mut gcr_value: u32 = 0;
    let mut bits_found: u32 = 0;
    let mut last_edge_index = 0usize;
    let mut last_state_is_high = false;

    for (i, &sample) in frame_samples.iter().enumerate().skip(1) {
        let current_state_is_high = (sample & mask) != 0;
        if current_state_is_high != last_state_is_high {
            let pulse_width_samples = i - last_edge_index;
            let len = ((pulse_width_samples + oversampling / 2) / oversampling).max(1) as u32;

            bits_found += len;
            gcr_value <<= len;
            gcr_value |= 1 << (len - 1);

            if bits_found > frame_bits {
                return Err(SampleDecodeError::InvalidFrame);
            }

            last_edge_index = i;
            last_state_is_high = current_state_is_high;
        }
    }

    let remaining_bits = frame_bits.saturating_sub(bits_found);
    if remaining_bits > 0 {
        gcr_value <<= remaining_bits;
        gcr_value |= 1 << (remaining_bits - 1);
    }

    if bits_found + remaining_bits < min_detected_bits {
        return Err(SampleDecodeError::InvalidFrame);
    }

    Ok(GcrDecodeResult {
        frame: GcrFrame { raw_21: gcr_value },
        start_margin,
    })
}

fn validate_oversampling_config(cfg: OversamplingConfig) -> Result<(), SampleDecodeError> {
    if cfg.oversampling == 0 {
        return Err(SampleDecodeError::InvalidConfig);
    }

    if cfg.frame_bits == 0 || cfg.frame_bits > 31 {
        return Err(SampleDecodeError::InvalidConfig);
    }

    if cfg.bit_tolerance > cfg.frame_bits {
        return Err(SampleDecodeError::InvalidConfig);
    }

    if cfg.min_detected_bits > cfg.frame_bits {
        return Err(SampleDecodeError::InvalidConfig);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_signal_samples(
        sample_bit_index: u8,
        preamble_high: usize,
        pulse_bits: &[usize],
        oversampling: usize,
    ) -> [u16; 96] {
        let mut out = [0u16; 96];
        let mask = 1u16 << sample_bit_index;
        let mut write = 0usize;

        let mut i = 0usize;
        while i < preamble_high && write < out.len() {
            out[write] = mask;
            write += 1;
            i += 1;
        }

        let mut is_high = false;
        for &bit_len in pulse_bits {
            let width = (bit_len * oversampling).saturating_sub(1).max(1);
            let level = if is_high { mask } else { 0 };
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

    fn pulse_lengths_from_gcr(mut gcr: u32) -> ([usize; 21], usize) {
        let mut out = [0usize; 21];
        let mut out_len = 0usize;

        // Keep only frame bits and decode run lengths of `1 0*` symbols.
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

    #[test]
    fn gcr_encode_decode_roundtrip() {
        let values = [23130, 0xAAAA, 0x5555, 0x1234, 0xFEDC, u16::MAX];
        for value in values {
            let gcr = encode_gcr(value);
            assert_eq!(decode_gcr(gcr), Some(value));
        }
    }

    #[test]
    fn payload_parse_crc_error() {
        let payload = 0x8108;
        assert_eq!(
            parse_telemetry_payload(payload),
            Err(PayloadParseError::InvalidCrc {
                calculated_crc: 9,
                packet_crc: 8,
            })
        );
    }

    #[test]
    fn payload_parse_erpm() {
        let payload = 0x8109;
        assert_eq!(
            parse_telemetry_payload(payload),
            Ok(TelemetryFrame::Erpm(ErpmReading::new(256)))
        );
    }

    #[test]
    fn payload_parse_extended_temperature() {
        let payload = 0x219A;
        assert_eq!(
            parse_telemetry_payload(payload),
            Ok(TelemetryFrame::Temperature(25))
        );
    }

    #[test]
    fn decode_gcr_stage_errors_invalid_bit() {
        let decoder = BidirDecoder::new(OversamplingConfig {
            sample_bit_index: 16,
            ..OversamplingConfig::default()
        });
        assert_eq!(
            decoder.decode_gcr(&[0u16; 8], DecodeHint::default()),
            Err(SampleDecodeError::InvalidSampleBitIndex { bit: 16 })
        );
    }

    #[test]
    fn decode_gcr_stage_errors_invalid_cfg_zero_frame_bits() {
        let decoder = BidirDecoder::new(OversamplingConfig {
            frame_bits: 0,
            ..OversamplingConfig::default()
        });
        assert_eq!(
            decoder.decode_gcr(&[0u16; 8], DecodeHint::default()),
            Err(SampleDecodeError::InvalidConfig)
        );
    }

    #[test]
    fn decode_gcr_stage_errors_invalid_cfg_frame_bits_too_large() {
        let decoder = BidirDecoder::new(OversamplingConfig {
            frame_bits: 32,
            ..OversamplingConfig::default()
        });
        assert_eq!(
            decoder.decode_gcr(&[0u16; 8], DecodeHint::default()),
            Err(SampleDecodeError::InvalidConfig)
        );
    }

    #[test]
    fn decode_gcr_stage_errors_invalid_cfg_zero_oversampling() {
        let decoder = BidirDecoder::new(OversamplingConfig {
            oversampling: 0,
            ..OversamplingConfig::default()
        });
        assert_eq!(
            decoder.decode_gcr(&[0u16; 8], DecodeHint::default()),
            Err(SampleDecodeError::InvalidConfig)
        );
    }

    #[test]
    fn decode_gcr_stage_errors_invalid_cfg_tolerance_exceeds_frame() {
        let decoder = BidirDecoder::new(OversamplingConfig {
            frame_bits: 20,
            bit_tolerance: 21,
            ..OversamplingConfig::default()
        });
        assert_eq!(
            decoder.decode_gcr(&[0u16; 8], DecodeHint::default()),
            Err(SampleDecodeError::InvalidConfig)
        );
    }

    #[test]
    fn decode_gcr_stage_errors_invalid_cfg_min_detected_exceeds_frame() {
        let decoder = BidirDecoder::new(OversamplingConfig {
            frame_bits: 20,
            min_detected_bits: 21,
            ..OversamplingConfig::default()
        });
        assert_eq!(
            decoder.decode_gcr(&[0u16; 8], DecodeHint::default()),
            Err(SampleDecodeError::InvalidConfig)
        );
    }

    #[test]
    fn decode_gcr_stage_errors_no_edge() {
        let decoder = BidirDecoder::new(OversamplingConfig::default());
        assert_eq!(
            decoder.decode_gcr(&[1u16; 8], DecodeHint::default()),
            Err(SampleDecodeError::NoEdge)
        );
    }

    #[test]
    fn decode_gcr_stage_errors_frame_too_short() {
        let decoder = BidirDecoder::new(OversamplingConfig::default());
        let samples = [0u16; 8];
        assert_eq!(
            decoder.decode_gcr(&samples, DecodeHint::default()),
            Err(SampleDecodeError::FrameTooShort)
        );
    }

    #[test]
    fn end_to_end_decode_known_value() {
        let payload = 23130u16;
        let gcr = encode_gcr(payload);
        let (bit_lengths, lengths_len) = pulse_lengths_from_gcr(gcr);

        let samples = build_signal_samples(
            0,
            8,
            &bit_lengths[..lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );

        let decoder = BidirDecoder::new(OversamplingConfig::default());
        let decoded = decoder.decode(&samples, DecodeHint::default()).unwrap();

        assert_eq!(decoded.payload.raw_16, payload);
        assert_eq!(decoded.frame, TelemetryFrame::Erpm(ErpmReading::new(1684)));
    }

    #[test]
    fn pipeline_error_maps_stage() {
        let decoder = BidirDecoder::new(OversamplingConfig::default());
        let err = decoder
            .decode(&[1u16; 8], DecodeHint::default())
            .unwrap_err();
        assert_eq!(
            err,
            TelemetryPipelineError::Samples(SampleDecodeError::NoEdge)
        );
    }

    #[test]
    fn push_sample_streaming_yields_result() {
        let payload = 23130u16;
        let gcr = encode_gcr(payload);
        let (bit_lengths, lengths_len) = pulse_lengths_from_gcr(gcr);

        let samples = build_signal_samples(
            0,
            8,
            &bit_lengths[..lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );

        let mut decoder = BidirDecoder::new(OversamplingConfig::default());
        let mut got = None;
        for sample in samples {
            if let Some(result) = decoder.push_sample(sample) {
                got = Some(result.unwrap());
                break;
            }
        }

        let got = got.expect("expected streaming decode result");
        assert_eq!(got.payload.raw_16, payload);
    }

    #[test]
    fn erpm_helpers() {
        let erpm = ErpmReading::new(250);
        assert_eq!(erpm.period(), 250);
        assert_eq!(erpm.electrical_hz(), 4000);
        assert_eq!(erpm.mechanical_rpm(7), (4000 * 60) / 7);
        assert_eq!(erpm.mechanical_rpm(0), 0);
    }

    #[test]
    fn test_decode_gcr_invalid_chunks() {
        // All zeros map to invalid GCR chunks.
        assert_eq!(decode_gcr(0), None);
        // Short random invalid matrix cases.
        assert_eq!(decode_gcr(0b000000000000000000001), None);
        assert_eq!(decode_gcr(0b111111111111111111111), None);
    }

    #[test]
    fn test_parse_extended_telemetry_types() {
        // Type 0x06 (current) with value 0x55 and valid crc.
        let data = 0x655;
        let payload = (data << 4) | (calculate_crc(data) as u16);
        assert_eq!(parse_telemetry_payload(payload), Ok(Telemetry::Current(0x55)));

        // Another reachable extended type (0x0A).
        let data = 0xA55;
        let payload = (data << 4) | (calculate_crc(data) as u16);
        assert_eq!(parse_telemetry_payload(payload), Ok(Telemetry::Debug2(0x55)));
    }

    #[test]
    fn test_erpm_conversions() {
        let period = 1;
        let erpm_x100 = erpm_period_to_erpm_x100(period);
        assert!(erpm_x100 > 0);
        let rpm = erpm_period_to_rpm(period, 7);
        let hz = erpm_period_to_hz(period, 7);
        assert!(rpm > 0);
        assert!(hz > 0);
    }

    #[test]
    fn test_erpm_conversion_zero_guards() {
        assert_eq!(erpm_period_to_erpm_x100(0), 0);
        assert_eq!(erpm_x100_to_rpm(1000, 0), 0);
        assert_eq!(erpm_x100_to_hz(1000, 0), 0);
    }
}
