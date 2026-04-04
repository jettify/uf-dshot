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

impl OversamplingConfig {
    pub const fn max_frame_samples(self) -> usize {
        (self.frame_bits as usize + self.bit_tolerance as usize) * self.oversampling as usize
    }

    pub const fn recommended_capture_samples(self, preamble_margin_samples: usize) -> usize {
        self.max_frame_samples() + preamble_margin_samples
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct DecodeHint {
    pub preamble_skip: usize,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PreambleTuningConfig {
    pub enabled: bool,
    pub target_start_margin: usize,
    pub update_interval_frames: u16,
}

impl Default for PreambleTuningConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            target_start_margin: 5,
            update_interval_frames: 32,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct TelemetryDecoderStats {
    pub successful_frames: u32,
    pub no_edge: u32,
    pub frame_too_short: u32,
    pub invalid_frame: u32,
    pub invalid_gcr_symbol: u32,
    pub invalid_crc: u32,
    pub last_start_margin: usize,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct GcrFrame {
    pub raw_21: u32,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct GcrDecodeResult {
    pub frame: GcrFrame,
    pub start_margin: usize,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) struct TelemetryPayload {
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
pub(crate) struct DecodedTelemetry {
    pub(crate) gcr: GcrDecodeResult,
    pub(crate) payload: TelemetryPayload,
    pub frame: TelemetryFrame,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum SampleDecodeError {
    InvalidSampleBitIndex { bit: u8 },
    InvalidConfig,
    NoEdge,
    FrameTooShort,
    InvalidFrame,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum GcrDecodeError {
    InvalidGcrSymbol,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum PayloadParseError {
    InvalidCrc { calculated_crc: u8, packet_crc: u8 },
    InvalidErpmPeriod,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TelemetryError {
    InvalidSampleBitIndex { bit: u8 },
    InvalidConfig,
    NoEdge,
    FrameTooShort,
    InvalidFrame,
    InvalidGcrSymbol,
    InvalidCrc { calculated_crc: u8, packet_crc: u8 },
    InvalidErpmPeriod,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum TelemetryPipelineError {
    Samples(SampleDecodeError),
    GcrDecode(GcrDecodeError),
    PayloadParse(PayloadParseError),
}

impl From<SampleDecodeError> for TelemetryError {
    fn from(err: SampleDecodeError) -> Self {
        match err {
            SampleDecodeError::InvalidSampleBitIndex { bit } => {
                TelemetryError::InvalidSampleBitIndex { bit }
            }
            SampleDecodeError::InvalidConfig => TelemetryError::InvalidConfig,
            SampleDecodeError::NoEdge => TelemetryError::NoEdge,
            SampleDecodeError::FrameTooShort => TelemetryError::FrameTooShort,
            SampleDecodeError::InvalidFrame => TelemetryError::InvalidFrame,
        }
    }
}

impl From<GcrDecodeError> for TelemetryError {
    fn from(err: GcrDecodeError) -> Self {
        match err {
            GcrDecodeError::InvalidGcrSymbol => TelemetryError::InvalidGcrSymbol,
        }
    }
}

impl From<PayloadParseError> for TelemetryError {
    fn from(err: PayloadParseError) -> Self {
        match err {
            PayloadParseError::InvalidCrc {
                calculated_crc,
                packet_crc,
            } => TelemetryError::InvalidCrc {
                calculated_crc,
                packet_crc,
            },
            PayloadParseError::InvalidErpmPeriod => TelemetryError::InvalidErpmPeriod,
        }
    }
}

impl From<TelemetryPipelineError> for TelemetryError {
    fn from(err: TelemetryPipelineError) -> Self {
        match err {
            TelemetryPipelineError::Samples(e) => e.into(),
            TelemetryPipelineError::GcrDecode(e) => e.into(),
            TelemetryPipelineError::PayloadParse(e) => e.into(),
        }
    }
}

const STREAM_BUFFER_CAPACITY: usize = 256;
const PAYLOAD_DATA_SHIFT: u16 = 4;
const PAYLOAD_CRC_MASK: u16 = 0x0F;
const PERIOD_EXPONENT_SHIFT: u16 = 9;
const PERIOD_EXPONENT_MASK: u16 = 0b111;
const PERIOD_MANTISSA_MASK: u16 = 0x1FF;
const EXTENDED_EXPONENT_FLAG_MASK: u16 = 0b001;
const EXTENDED_MANTISSA_FLAG_MASK: u16 = 0x100;
const EXTENDED_TYPE_SHIFT: u16 = 8;
const EXTENDED_VALUE_MASK: u16 = 0xFF;
const EXTENDED_TYPE_TEMPERATURE: u8 = 0x02;
const EXTENDED_TYPE_VOLTAGE: u8 = 0x04;
const EXTENDED_TYPE_CURRENT: u8 = 0x06;
const EXTENDED_TYPE_DEBUG1: u8 = 0x08;
const EXTENDED_TYPE_DEBUG2: u8 = 0x0A;
const EXTENDED_TYPE_DEBUG3: u8 = 0x0C;
const EXTENDED_TYPE_STATE_EVENT: u8 = 0x0E;
const ERPM_SENTINEL_OUT_OF_RANGE: u16 = 0x0FFF;

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct PreambleTuningState {
    hint: DecodeHint,
    min_start_margin: usize,
    frames_since_update: u16,
}

impl Default for PreambleTuningState {
    fn default() -> Self {
        Self {
            hint: DecodeHint { preamble_skip: 0 },
            min_start_margin: usize::MAX,
            frames_since_update: 0,
        }
    }
}

pub struct BidirDecoder {
    pub cfg: OversamplingConfig,
    pub preamble_tuning: PreambleTuningConfig,
    stream_buf: [u16; STREAM_BUFFER_CAPACITY],
    stream_len: usize,
    stream_tuning_state: PreambleTuningState,
    stats: TelemetryDecoderStats,
}

impl BidirDecoder {
    pub const fn new(cfg: OversamplingConfig) -> Self {
        Self::with_preamble_tuning(
            cfg,
            PreambleTuningConfig {
                enabled: true,
                target_start_margin: 5,
                update_interval_frames: 32,
            },
        )
    }

    pub const fn with_preamble_tuning(
        cfg: OversamplingConfig,
        preamble_tuning: PreambleTuningConfig,
    ) -> Self {
        Self {
            cfg,
            preamble_tuning,
            stream_buf: [0; STREAM_BUFFER_CAPACITY],
            stream_len: 0,
            stream_tuning_state: PreambleTuningState {
                hint: DecodeHint { preamble_skip: 0 },
                min_start_margin: usize::MAX,
                frames_since_update: 0,
            },
            stats: TelemetryDecoderStats {
                successful_frames: 0,
                no_edge: 0,
                frame_too_short: 0,
                invalid_frame: 0,
                invalid_gcr_symbol: 0,
                invalid_crc: 0,
                last_start_margin: 0,
            },
        }
    }

    pub(crate) fn decode(
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

    pub fn decode_frame(&self, samples: &[u16]) -> Result<TelemetryFrame, TelemetryError> {
        self.decode(samples, DecodeHint::default())
            .map(|decoded| decoded.frame)
            .map_err(Into::into)
    }

    pub fn decode_frame_with_hint(
        &self,
        samples: &[u16],
        hint: DecodeHint,
    ) -> Result<TelemetryFrame, TelemetryError> {
        self.decode(samples, hint)
            .map(|decoded| decoded.frame)
            .map_err(Into::into)
    }

    pub fn decode_frame_tuned(
        &mut self,
        samples: &[u16],
    ) -> Result<TelemetryFrame, TelemetryError> {
        let hint = self.bounded_stream_hint(samples.len());
        let result = self.decode(samples, hint);
        self.finish_tuned_decode(result).map_err(Into::into)
    }

    pub fn decode_frame_tuned_port_samples(
        &mut self,
        samples: &[u32],
        bit_mask: u32,
    ) -> Result<TelemetryFrame, TelemetryError> {
        let len = self.fill_stream_buf_from_port_samples(samples, bit_mask);
        let hint = self.bounded_stream_hint(len);
        let result = self.decode(&self.stream_buf[..len], hint);
        self.finish_tuned_decode(result).map_err(Into::into)
    }

    pub fn decode_frame_tuned_port_samples_u16(
        &mut self,
        samples: &[u16],
        bit_mask: u16,
    ) -> Result<TelemetryFrame, TelemetryError> {
        let len = self.fill_stream_buf_from_port_samples(samples, u32::from(bit_mask));
        let hint = self.bounded_stream_hint(len);
        let result = self.decode(&self.stream_buf[..len], hint);
        self.finish_tuned_decode(result).map_err(Into::into)
    }

    pub(crate) fn decode_frame_from_gcr_tuned(
        &mut self,
        gcr: Result<GcrDecodeResult, SampleDecodeError>,
    ) -> Result<TelemetryFrame, TelemetryPipelineError> {
        match gcr {
            Ok(gcr) => {
                let payload = self
                    .decode_payload(gcr.frame)
                    .map_err(TelemetryPipelineError::GcrDecode)?;
                let frame = self
                    .parse_payload(payload)
                    .map_err(TelemetryPipelineError::PayloadParse)?;
                self.observe_start_margin(gcr.start_margin);
                Ok(frame)
            }
            Err(SampleDecodeError::NoEdge) => {
                self.observe_no_edge();
                Err(TelemetryPipelineError::Samples(SampleDecodeError::NoEdge))
            }
            Err(err) => Err(TelemetryPipelineError::Samples(err)),
        }
    }

    pub(crate) fn decode_gcr(
        &self,
        samples: &[u16],
        hint: DecodeHint,
    ) -> Result<GcrDecodeResult, SampleDecodeError> {
        decode_gcr_from_samples_cfg(samples, hint, self.cfg)
    }

    pub(crate) fn decode_payload(&self, gcr: GcrFrame) -> Result<TelemetryPayload, GcrDecodeError> {
        match decode_gcr(gcr.raw_21) {
            Some(raw_16) => Ok(TelemetryPayload { raw_16 }),
            None => Err(GcrDecodeError::InvalidGcrSymbol),
        }
    }

    pub(crate) fn parse_payload(
        &self,
        payload: TelemetryPayload,
    ) -> Result<TelemetryFrame, PayloadParseError> {
        parse_telemetry_payload_inner(payload.raw_16)
    }

    pub(crate) fn push_sample(
        &mut self,
        sample: u16,
    ) -> Option<Result<DecodedTelemetry, TelemetryPipelineError>> {
        if self.stream_len < STREAM_BUFFER_CAPACITY {
            self.stream_buf[self.stream_len] = sample;
            self.stream_len += 1;
        } else {
            self.stream_buf.copy_within(1..STREAM_BUFFER_CAPACITY, 0);
            self.stream_buf[STREAM_BUFFER_CAPACITY - 1] = sample;
            self.stream_tuning_state.hint.preamble_skip = self
                .stream_tuning_state
                .hint
                .preamble_skip
                .saturating_sub(1);
        }

        let mut hint = self.stream_tuning_state.hint;
        if hint.preamble_skip >= self.stream_len {
            hint.preamble_skip = self.stream_len.saturating_sub(1);
        }

        match self.decode(&self.stream_buf[..self.stream_len], hint) {
            Ok(decoded) => {
                self.stats.successful_frames = self.stats.successful_frames.saturating_add(1);
                self.stats.last_start_margin = decoded.gcr.start_margin;
                self.observe_start_margin(decoded.gcr.start_margin);
                self.reset_stream();
                Some(Ok(decoded))
            }
            Err(TelemetryPipelineError::Samples(SampleDecodeError::NoEdge)) => {
                self.stats.no_edge = self.stats.no_edge.saturating_add(1);
                self.observe_no_edge();
                None
            }
            Err(TelemetryPipelineError::Samples(SampleDecodeError::FrameTooShort)) => {
                self.stats.frame_too_short = self.stats.frame_too_short.saturating_add(1);
                None
            }
            Err(TelemetryPipelineError::Samples(SampleDecodeError::InvalidFrame)) => {
                self.stats.invalid_frame = self.stats.invalid_frame.saturating_add(1);
                self.reset_stream();
                Self::emit_err(TelemetryPipelineError::Samples(
                    SampleDecodeError::InvalidFrame,
                ))
            }
            Err(TelemetryPipelineError::Samples(err)) => {
                Self::emit_err(TelemetryPipelineError::Samples(err))
            }
            Err(TelemetryPipelineError::GcrDecode(GcrDecodeError::InvalidGcrSymbol)) => {
                self.stats.invalid_gcr_symbol = self.stats.invalid_gcr_symbol.saturating_add(1);
                self.reset_stream();
                Self::emit_err(TelemetryPipelineError::GcrDecode(
                    GcrDecodeError::InvalidGcrSymbol,
                ))
            }
            Err(TelemetryPipelineError::PayloadParse(PayloadParseError::InvalidCrc {
                calculated_crc,
                packet_crc,
            })) => {
                self.stats.invalid_crc = self.stats.invalid_crc.saturating_add(1);
                self.reset_stream();
                Self::emit_err(TelemetryPipelineError::PayloadParse(
                    PayloadParseError::InvalidCrc {
                        calculated_crc,
                        packet_crc,
                    },
                ))
            }
            Err(TelemetryPipelineError::PayloadParse(err)) => {
                self.reset_stream();
                Self::emit_err(TelemetryPipelineError::PayloadParse(err))
            }
        }
    }

    pub fn push_sample_frame(
        &mut self,
        sample: u16,
    ) -> Option<Result<TelemetryFrame, TelemetryError>> {
        self.push_sample(sample)
            .map(|result| result.map(|decoded| decoded.frame).map_err(Into::into))
    }

    pub const fn stream_hint(&self) -> DecodeHint {
        self.stream_tuning_state.hint
    }

    pub fn set_stream_hint(&mut self, hint: DecodeHint) {
        self.stream_tuning_state.hint = hint;
    }

    pub const fn stats(&self) -> TelemetryDecoderStats {
        self.stats
    }

    pub fn reset_stats(&mut self) {
        self.stats = TelemetryDecoderStats::default();
    }

    fn reset_stream(&mut self) {
        self.stream_len = 0;
    }

    fn finish_tuned_decode(
        &mut self,
        result: Result<DecodedTelemetry, TelemetryPipelineError>,
    ) -> Result<TelemetryFrame, TelemetryPipelineError> {
        match result {
            Ok(decoded) => {
                self.observe_start_margin(decoded.gcr.start_margin);
                Ok(decoded.frame)
            }
            Err(TelemetryPipelineError::Samples(SampleDecodeError::NoEdge)) => {
                self.observe_no_edge();
                Err(TelemetryPipelineError::Samples(SampleDecodeError::NoEdge))
            }
            Err(err) => Err(err),
        }
    }

    fn bounded_stream_hint(&self, len: usize) -> DecodeHint {
        let mut hint = self.stream_tuning_state.hint;
        if hint.preamble_skip >= len {
            hint.preamble_skip = len.saturating_sub(1);
        }
        hint
    }

    fn sample_level_mask(&self) -> u16 {
        if self.cfg.sample_bit_index < 16 {
            1u16 << self.cfg.sample_bit_index
        } else {
            1
        }
    }

    fn fill_stream_buf_from_port_samples<T>(&mut self, samples: &[T], bit_mask: u32) -> usize
    where
        T: Copy + Into<u32>,
    {
        let len = samples.len().min(self.stream_buf.len());
        let sample_level_mask = self.sample_level_mask();
        for (dst, sample) in self.stream_buf[..len]
            .iter_mut()
            .zip(samples.iter().copied())
        {
            *dst = if (sample.into() & bit_mask) != 0 {
                sample_level_mask
            } else {
                0
            };
        }
        len
    }

    fn observe_start_margin(&mut self, start_margin: usize) {
        if !self.preamble_tuning.enabled || self.preamble_tuning.update_interval_frames == 0 {
            return;
        }

        if start_margin < self.stream_tuning_state.min_start_margin {
            self.stream_tuning_state.min_start_margin = start_margin;
        }
        self.stream_tuning_state.frames_since_update = self
            .stream_tuning_state
            .frames_since_update
            .saturating_add(1);

        if self.stream_tuning_state.frames_since_update
            >= self.preamble_tuning.update_interval_frames
        {
            let min_margin = self.stream_tuning_state.min_start_margin;
            let target = self.preamble_tuning.target_start_margin;
            self.stream_tuning_state.hint.preamble_skip = min_margin.saturating_sub(target);
            self.stream_tuning_state.frames_since_update = 0;
            self.stream_tuning_state.min_start_margin = usize::MAX;
        }
    }

    fn observe_no_edge(&mut self) {
        if self.preamble_tuning.enabled && self.stream_tuning_state.hint.preamble_skip > 0 {
            self.stream_tuning_state.hint.preamble_skip -= 1;
        }
    }

    fn emit_err(
        err: TelemetryPipelineError,
    ) -> Option<Result<DecodedTelemetry, TelemetryPipelineError>> {
        Some(Err(err))
    }
}

fn calculate_crc(value: u16) -> u8 {
    ((value ^ (value >> 4) ^ (value >> 8)) & 0x0F) as u8
}

fn calculate_telemetry_crc(value: u16) -> u8 {
    (!calculate_crc(value)) & 0x0F
}

pub fn parse_telemetry_payload(payload: u16) -> Result<TelemetryFrame, TelemetryError> {
    parse_telemetry_payload_inner(payload).map_err(Into::into)
}

fn parse_telemetry_payload_inner(payload: u16) -> Result<TelemetryFrame, PayloadParseError> {
    let data = payload >> PAYLOAD_DATA_SHIFT;
    let packet_crc = (payload & PAYLOAD_CRC_MASK) as u8;
    let calculated_crc = calculate_telemetry_crc(data);

    if packet_crc != calculated_crc {
        return Err(PayloadParseError::InvalidCrc {
            calculated_crc,
            packet_crc,
        });
    }

    let exponent = (data >> PERIOD_EXPONENT_SHIFT) & PERIOD_EXPONENT_MASK;
    let mantissa = data & PERIOD_MANTISSA_MASK;

    if is_extended_telemetry(exponent, mantissa) {
        let telemetry_type = (data >> EXTENDED_TYPE_SHIFT) as u8;
        let value = (data & EXTENDED_VALUE_MASK) as u8;
        let t = match telemetry_type {
            EXTENDED_TYPE_TEMPERATURE => TelemetryFrame::Temperature(value),
            EXTENDED_TYPE_VOLTAGE => TelemetryFrame::Voltage(value),
            EXTENDED_TYPE_CURRENT => TelemetryFrame::Current(value),
            EXTENDED_TYPE_DEBUG1 => TelemetryFrame::Debug1(value),
            EXTENDED_TYPE_DEBUG2 => TelemetryFrame::Debug2(value),
            EXTENDED_TYPE_DEBUG3 => TelemetryFrame::Debug3(value),
            EXTENDED_TYPE_STATE_EVENT => TelemetryFrame::StateEvent(value),
            _ => TelemetryFrame::UnknownExtended {
                type_id: telemetry_type,
                value,
            },
        };
        Ok(t)
    } else {
        // Betaflight compatibility: 0x0FFF means out-of-range eRPM and maps to zero.
        if data == ERPM_SENTINEL_OUT_OF_RANGE {
            return Ok(TelemetryFrame::Erpm(ErpmReading::new(0)));
        }

        let period = mantissa << exponent;
        if period == 0 {
            return Err(PayloadParseError::InvalidErpmPeriod);
        }
        Ok(TelemetryFrame::Erpm(ErpmReading::new(period)))
    }
}

const fn is_extended_telemetry(exponent: u16, mantissa: u16) -> bool {
    (exponent & EXTENDED_EXPONENT_FLAG_MASK) != 0 && (mantissa & EXTENDED_MANTISSA_FLAG_MASK) == 0
}

#[cfg(test)]
fn erpm_period_to_erpm_x100(period: u16) -> u32 {
    if period == 0 {
        return 0;
    }
    let period = u32::from(period);
    (1_000_000 * 60 / 100 + period / 2) / period
}

#[cfg(test)]
fn erpm_x100_to_rpm(erpm_x100: u32, pole_pairs: u8) -> u32 {
    if pole_pairs == 0 {
        return 0;
    }
    (erpm_x100 / 100) / u32::from(pole_pairs)
}

#[cfg(test)]
fn erpm_x100_to_hz(erpm_x100: u32, pole_pairs: u8) -> u32 {
    if pole_pairs == 0 {
        return 0;
    }
    (erpm_x100 / 100) / u32::from(pole_pairs) / 60
}

#[cfg(test)]
fn erpm_period_to_rpm(period: u16, pole_pairs: u8) -> u32 {
    let erpm_x100 = erpm_period_to_erpm_x100(period);
    erpm_x100_to_rpm(erpm_x100, pole_pairs)
}

#[cfg(test)]
fn erpm_period_to_hz(period: u16, pole_pairs: u8) -> u32 {
    let erpm_x100 = erpm_period_to_erpm_x100(period);
    erpm_x100_to_hz(erpm_x100, pole_pairs)
}

const GCR_DECODE_TABLE: [u8; 32] = [
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F,
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07, 0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF,
];

pub fn decode_gcr(gcr_value: u32) -> Option<u16> {
    let gcr_encoded = gcr_value & 0xFFFFF;

    let mut result: u16 = 0;
    let mut i = 0;
    while i < 4 {
        let chunk = (gcr_encoded >> (15 - i * 5)) & 0x1F;
        let nibble = GCR_DECODE_TABLE[chunk as usize];
        if nibble == 0xFF {
            return None;
        }
        result = (result << 4) | u16::from(nibble);
        i += 1;
    }
    Some(result)
}

#[cfg(test)]
const GCR_ENCODE_TABLE: [u8; 16] = [
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111, 0b11010, 0b01001,
    0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
];

#[cfg(test)]
pub(crate) fn encode_gcr(payload: u16) -> u32 {
    let mut gcr_encoded: u32 = 0;
    let mut i = 0;
    while i < 4 {
        let nibble = (payload >> (12 - i * 4)) & 0x0F;
        let chunk = u32::from(GCR_ENCODE_TABLE[nibble as usize]);
        gcr_encoded = (gcr_encoded << 5) | chunk;
        i += 1;
    }

    gcr_encoded | (1 << 20)
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
    let frame_bits = u32::from(cfg.frame_bits);
    let min_detected_bits = u32::from(cfg.min_detected_bits);
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
    let mut last_edge_index: usize = 0;
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

    if bits_found < min_detected_bits {
        return Err(SampleDecodeError::InvalidFrame);
    }

    let remaining_bits = frame_bits.saturating_sub(bits_found);
    if remaining_bits > 0 {
        gcr_value <<= remaining_bits;
        gcr_value |= 1 << (remaining_bits - 1);
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

    fn build_signal_samples_with_tail_level(
        sample_bit_index: u8,
        preamble_high: usize,
        pulse_bits: &[usize],
        oversampling: usize,
        tail_high: bool,
    ) -> [u16; 96] {
        let mask = 1u16 << sample_bit_index;
        let mut out = [if tail_high { mask } else { 0 }; 96];
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

    fn build_signal_samples_from_widths_with_tail_level(
        sample_bit_index: u8,
        preamble_high: usize,
        pulse_width_samples: &[usize],
        tail_high: bool,
    ) -> [u16; 96] {
        let mask = 1u16 << sample_bit_index;
        let mut out = [if tail_high { mask } else { 0 }; 96];
        let mut write = 0usize;

        let mut i = 0usize;
        while i < preamble_high && write < out.len() {
            out[write] = mask;
            write += 1;
            i += 1;
        }

        let mut is_high = false;
        for &width in pulse_width_samples {
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

    fn gcr_raw_from_encoded_20(gcr_encoded: u32) -> u32 {
        gcr_encoded | (1 << 20)
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
    fn raw_candidate_15ea6f_decodes_to_valid_erpm() {
        let decoder = BidirDecoder::new(OversamplingConfig::default());
        // 0x15ea6f is the raw_21 value from the reported error log.
        // It should decode to 0xB83F which is a valid telemetry frame.
        let payload = decoder
            .decode_payload(GcrFrame { raw_21: 0x15ea6f })
            .unwrap();
        assert_eq!(payload.raw_16, 0xB83F);
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
    fn oversampling_default_matches_reference_window() {
        let cfg = OversamplingConfig::default();
        assert_eq!(cfg.oversampling, 3);
        assert_eq!(cfg.frame_bits, 21);
        assert_eq!(cfg.min_detected_bits, 18);
        assert_eq!(cfg.bit_tolerance, 2);

        let min_samples = (cfg.frame_bits - cfg.bit_tolerance) as usize * cfg.oversampling as usize;
        let max_samples = cfg.max_frame_samples();
        assert_eq!(min_samples, 57);
        assert_eq!(max_samples, 69);
    }

    #[test]
    fn recommended_capture_samples_covers_max_frame_and_preamble() {
        let cfg = OversamplingConfig::default();
        assert_eq!(cfg.max_frame_samples(), 69);
        assert_eq!(cfg.recommended_capture_samples(6), 75);
    }

    #[test]
    fn preamble_tuning_default_matches_reference() {
        let tuning = PreambleTuningConfig::default();
        assert!(tuning.enabled);
        assert_eq!(tuning.target_start_margin, 5);
        assert_eq!(tuning.update_interval_frames, 32);
    }

    #[test]
    fn payload_parse_crc_error() {
        let payload = 0x8108;
        assert_eq!(
            parse_telemetry_payload(payload),
            Err(TelemetryError::InvalidCrc {
                calculated_crc: 6,
                packet_crc: 8,
            })
        );
    }

    #[test]
    fn payload_parse_erpm() {
        let payload = 0x8106;
        assert_eq!(
            parse_telemetry_payload(payload),
            Ok(TelemetryFrame::Erpm(ErpmReading::new(256)))
        );
    }

    #[test]
    fn payload_parse_erpm_range_sentinel_maps_to_zero() {
        let data = 0x0FFFu16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        assert_eq!(
            parse_telemetry_payload(payload),
            Ok(TelemetryFrame::Erpm(ErpmReading::new(0)))
        );
    }

    #[test]
    fn payload_parse_erpm_zero_period_is_invalid() {
        let data = 0x000u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        assert_eq!(
            parse_telemetry_payload(payload),
            Err(TelemetryError::InvalidErpmPeriod)
        );
    }

    #[test]
    fn payload_parse_extended_temperature() {
        let data = 0x219u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
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
    fn decode_gcr_stage_errors_below_min_detected_bits() {
        let decoder = BidirDecoder::new(OversamplingConfig::default());
        let samples = [0u16; 96];
        assert_eq!(
            decoder.decode_gcr(&samples, DecodeHint::default()),
            Err(SampleDecodeError::InvalidFrame)
        );
    }

    #[test]
    fn decode_gcr_min_detected_bits_thresholds() {
        let decoder = BidirDecoder::new(OversamplingConfig::default());
        let oversampling = OversamplingConfig::default().oversampling as usize;

        let samples_17 = build_signal_samples_with_tail_level(0, 8, &[17, 1], oversampling, true);
        assert_eq!(
            decoder.decode_gcr(&samples_17, DecodeHint::default()),
            Err(SampleDecodeError::InvalidFrame)
        );

        let samples_18 = build_signal_samples_with_tail_level(0, 8, &[18, 1], oversampling, true);
        assert!(decoder
            .decode_gcr(&samples_18, DecodeHint::default())
            .is_ok());

        let samples_19 = build_signal_samples_with_tail_level(0, 8, &[19, 1], oversampling, true);
        assert!(decoder
            .decode_gcr(&samples_19, DecodeHint::default())
            .is_ok());
    }

    #[test]
    fn decode_gcr_rounding_threshold_bins_are_stable() {
        let decoder = BidirDecoder::new(OversamplingConfig {
            min_detected_bits: 1,
            ..OversamplingConfig::default()
        });
        let cases = [(4usize, 1usize), (5, 2), (7, 2), (8, 3), (10, 3), (11, 4)];

        for (width, expected_len) in cases {
            let samples = build_signal_samples_from_widths_with_tail_level(0, 8, &[width, 1], true);
            let decoded = decoder
                .decode_gcr(&samples, DecodeHint::default())
                .expect("expected valid frame");
            let (pulse_lengths, pulse_len_count) = pulse_lengths_from_gcr(decoded.frame.raw_21);
            assert!(pulse_len_count > 0);
            assert_eq!(pulse_lengths[0], expected_len);
        }
    }

    #[test]
    fn end_to_end_decode_known_value() {
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
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
    fn end_to_end_decode_frame_known_value() {
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        let gcr = encode_gcr(payload);
        let (bit_lengths, lengths_len) = pulse_lengths_from_gcr(gcr);

        let samples = build_signal_samples(
            0,
            8,
            &bit_lengths[..lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );

        let decoder = BidirDecoder::new(OversamplingConfig::default());
        let decoded = decoder
            .decode_frame_with_hint(&samples, DecodeHint::default())
            .unwrap();

        assert_eq!(decoded, TelemetryFrame::Erpm(ErpmReading::new(1684)));
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
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
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
    fn push_sample_frame_streaming_yields_result() {
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
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
            if let Some(result) = decoder.push_sample_frame(sample) {
                got = Some(result.unwrap());
                break;
            }
        }

        let got = got.expect("expected streaming frame decode result");
        assert_eq!(got, TelemetryFrame::Erpm(ErpmReading::new(1684)));
    }

    #[test]
    fn push_sample_updates_stats_on_success() {
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        let gcr = encode_gcr(payload);
        let (bit_lengths, lengths_len) = pulse_lengths_from_gcr(gcr);
        let samples = build_signal_samples(
            0,
            8,
            &bit_lengths[..lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );

        let mut decoder = BidirDecoder::new(OversamplingConfig::default());
        for sample in samples {
            let _ = decoder.push_sample(sample);
        }

        let stats = decoder.stats();
        assert_eq!(stats.successful_frames, 1);
        assert_eq!(stats.last_start_margin, 8);
    }

    #[test]
    fn push_sample_tracks_no_edge_stats() {
        let mut decoder = BidirDecoder::new(OversamplingConfig::default());
        for _ in 0..12 {
            let _ = decoder.push_sample(1);
        }
        assert!(decoder.stats().no_edge > 0);
    }

    #[test]
    fn push_sample_invalid_crc_is_consumed_before_next_frame() {
        let data = 0x5A5u16;
        let valid_payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        let invalid_crc_payload = (valid_payload & !0x000F) | ((valid_payload + 1) & 0x000F);

        let invalid_gcr = encode_gcr(invalid_crc_payload);
        let (invalid_bit_lengths, invalid_lengths_len) = pulse_lengths_from_gcr(invalid_gcr);
        let invalid_samples = build_signal_samples(
            0,
            8,
            &invalid_bit_lengths[..invalid_lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );

        let valid_gcr = encode_gcr(valid_payload);
        let (valid_bit_lengths, valid_lengths_len) = pulse_lengths_from_gcr(valid_gcr);
        let valid_samples = build_signal_samples(
            0,
            8,
            &valid_bit_lengths[..valid_lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );

        let mut decoder = BidirDecoder::new(OversamplingConfig::default());
        let mut saw_invalid_crc = false;
        for sample in invalid_samples {
            if let Some(result) = decoder.push_sample(sample) {
                assert!(
                    matches!(
                        result,
                        Err(TelemetryPipelineError::PayloadParse(
                            PayloadParseError::InvalidCrc { .. },
                        ))
                    ),
                    "expected invalid CRC error, got {result:?}"
                );
                saw_invalid_crc = true;
                break;
            }
        }

        assert!(saw_invalid_crc, "expected invalid CRC to be surfaced");

        let mut got = None;
        for sample in valid_samples {
            if let Some(result) = decoder.push_sample(sample) {
                got = Some(result.unwrap());
                break;
            }
        }

        let got = got.expect("expected valid frame after invalid CRC");
        assert_eq!(got.payload.raw_16, valid_payload);
    }

    #[test]
    fn push_sample_frame_too_short_keeps_partial_frame() {
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        let gcr = encode_gcr(payload);
        let (bit_lengths, lengths_len) = pulse_lengths_from_gcr(gcr);
        let samples = build_signal_samples(
            0,
            8,
            &bit_lengths[..lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );

        let split_at = samples.len() / 2;
        let (prefix, suffix) = samples.split_at(split_at);

        let mut decoder = BidirDecoder::new(OversamplingConfig::default());
        for sample in prefix {
            assert!(decoder.push_sample(*sample).is_none());
        }

        let mut got = None;
        for sample in suffix {
            if let Some(result) = decoder.push_sample(*sample) {
                got = Some(result.unwrap());
                break;
            }
        }

        let got = got.expect("expected frame completion after partial prefix");
        assert_eq!(got.payload.raw_16, payload);
    }

    #[test]
    fn push_sample_overflow_keeps_preamble_skip_aligned_with_shifted_window() {
        let tuning = PreambleTuningConfig {
            enabled: false,
            target_start_margin: 5,
            update_interval_frames: 32,
        };
        let mut decoder = BidirDecoder::with_preamble_tuning(OversamplingConfig::default(), tuning);
        decoder.set_stream_hint(DecodeHint { preamble_skip: 8 });

        for _ in 0..=STREAM_BUFFER_CAPACITY {
            let _ = decoder.push_sample(1);
        }
        assert_eq!(decoder.stream_hint().preamble_skip, 7);

        for _ in 0..4 {
            let _ = decoder.push_sample(1);
        }
        assert_eq!(decoder.stream_hint().preamble_skip, 3);
    }

    #[test]
    fn preamble_tuning_updates_stream_hint_after_window() {
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        let gcr = encode_gcr(payload);
        let (bit_lengths, lengths_len) = pulse_lengths_from_gcr(gcr);
        let samples = build_signal_samples(
            0,
            12,
            &bit_lengths[..lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );

        let tuning = PreambleTuningConfig {
            enabled: true,
            target_start_margin: 5,
            update_interval_frames: 1,
        };
        let mut decoder = BidirDecoder::with_preamble_tuning(OversamplingConfig::default(), tuning);

        for sample in samples {
            if decoder.push_sample(sample).is_some() {
                break;
            }
        }

        assert_eq!(decoder.stream_hint().preamble_skip, 7);
    }

    #[test]
    fn preamble_tuning_no_edge_backs_off_skip() {
        let tuning = PreambleTuningConfig {
            enabled: true,
            target_start_margin: 5,
            update_interval_frames: 32,
        };
        let mut decoder = BidirDecoder::with_preamble_tuning(OversamplingConfig::default(), tuning);
        decoder.set_stream_hint(DecodeHint { preamble_skip: 3 });

        let _ = decoder.push_sample(1);
        assert_eq!(decoder.stream_hint().preamble_skip, 2);
    }

    #[test]
    fn preamble_tuning_frame_too_short_does_not_backoff_skip() {
        let tuning = PreambleTuningConfig {
            enabled: true,
            target_start_margin: 5,
            update_interval_frames: 32,
        };
        let mut decoder = BidirDecoder::with_preamble_tuning(OversamplingConfig::default(), tuning);
        decoder.set_stream_hint(DecodeHint { preamble_skip: 4 });

        let _ = decoder.push_sample(0);
        assert_eq!(decoder.stream_hint().preamble_skip, 4);
    }

    #[test]
    fn decode_frame_tuned_updates_hint_after_success() {
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        let gcr = encode_gcr(payload);
        let (bit_lengths, lengths_len) = pulse_lengths_from_gcr(gcr);
        let samples = build_signal_samples(
            0,
            12,
            &bit_lengths[..lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );

        let tuning = PreambleTuningConfig {
            enabled: true,
            target_start_margin: 5,
            update_interval_frames: 1,
        };
        let mut decoder = BidirDecoder::with_preamble_tuning(OversamplingConfig::default(), tuning);

        let frame = decoder.decode_frame_tuned(&samples);
        assert!(frame.is_ok());
        assert_eq!(decoder.stream_hint().preamble_skip, 7);
    }

    #[test]
    fn decode_frame_tuned_port_samples_reuses_tuned_decoder() {
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        let gcr = encode_gcr(payload);
        let (bit_lengths, lengths_len) = pulse_lengths_from_gcr(gcr);
        let samples = build_signal_samples(
            0,
            12,
            &bit_lengths[..lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );
        let mut port_samples = [0u32; 128];
        for (dst, sample) in port_samples.iter_mut().zip(samples.iter().copied()) {
            *dst = if sample != 0 { 1 << 7 } else { 0 };
        }

        let tuning = PreambleTuningConfig {
            enabled: true,
            target_start_margin: 5,
            update_interval_frames: 1,
        };
        let mut decoder = BidirDecoder::with_preamble_tuning(OversamplingConfig::default(), tuning);

        let frame = decoder.decode_frame_tuned_port_samples(&port_samples[..samples.len()], 1 << 7);
        assert_eq!(frame, Ok(TelemetryFrame::Erpm(ErpmReading::new(1684))));
        assert_eq!(decoder.stream_hint().preamble_skip, 7);
    }

    #[test]
    fn decode_frame_tuned_port_samples_respects_nonzero_sample_bit_index() {
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        let gcr = encode_gcr(payload);
        let (bit_lengths, lengths_len) = pulse_lengths_from_gcr(gcr);
        let samples = build_signal_samples(
            0,
            12,
            &bit_lengths[..lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );
        let mut port_samples = [0u32; 128];
        for (dst, sample) in port_samples.iter_mut().zip(samples.iter().copied()) {
            *dst = if sample != 0 { 1 << 7 } else { 0 };
        }

        let tuning = PreambleTuningConfig {
            enabled: true,
            target_start_margin: 5,
            update_interval_frames: 1,
        };
        let cfg = OversamplingConfig {
            sample_bit_index: 7,
            ..OversamplingConfig::default()
        };
        let mut decoder = BidirDecoder::with_preamble_tuning(cfg, tuning);

        let frame = decoder.decode_frame_tuned_port_samples(&port_samples[..samples.len()], 1 << 7);
        assert_eq!(frame, Ok(TelemetryFrame::Erpm(ErpmReading::new(1684))));
    }

    #[test]
    fn decode_frame_tuned_port_samples_u16_respects_nonzero_sample_bit_index() {
        let data = 0x5A5u16;
        let payload = (data << 4) | calculate_telemetry_crc(data) as u16;
        let gcr = encode_gcr(payload);
        let (bit_lengths, lengths_len) = pulse_lengths_from_gcr(gcr);
        let samples = build_signal_samples(
            0,
            12,
            &bit_lengths[..lengths_len],
            OversamplingConfig::default().oversampling as usize,
        );
        let mut port_samples = [0u16; 128];
        for (dst, sample) in port_samples.iter_mut().zip(samples.iter().copied()) {
            *dst = if sample != 0 { 1 << 7 } else { 0 };
        }

        let tuning = PreambleTuningConfig {
            enabled: true,
            target_start_margin: 5,
            update_interval_frames: 1,
        };
        let cfg = OversamplingConfig {
            sample_bit_index: 7,
            ..OversamplingConfig::default()
        };
        let mut decoder = BidirDecoder::with_preamble_tuning(cfg, tuning);

        let frame =
            decoder.decode_frame_tuned_port_samples_u16(&port_samples[..samples.len()], 1 << 7);
        assert_eq!(frame, Ok(TelemetryFrame::Erpm(ErpmReading::new(1684))));
    }

    #[test]
    fn decode_frame_tuned_port_samples_no_edge_backs_off_hint() {
        let tuning = PreambleTuningConfig {
            enabled: true,
            target_start_margin: 5,
            update_interval_frames: 32,
        };
        let mut decoder = BidirDecoder::with_preamble_tuning(OversamplingConfig::default(), tuning);
        decoder.set_stream_hint(DecodeHint { preamble_skip: 3 });

        let samples = [1u32 << 7; 96];
        let frame = decoder.decode_frame_tuned_port_samples(&samples, 1 << 7);
        assert_eq!(frame, Err(TelemetryError::NoEdge));
        assert_eq!(decoder.stream_hint().preamble_skip, 2);
    }

    #[test]
    fn decode_frame_tuned_no_edge_backs_off_hint() {
        let tuning = PreambleTuningConfig {
            enabled: true,
            target_start_margin: 5,
            update_interval_frames: 32,
        };
        let mut decoder = BidirDecoder::with_preamble_tuning(OversamplingConfig::default(), tuning);
        decoder.set_stream_hint(DecodeHint { preamble_skip: 3 });

        let samples = [1u16; 96];
        let frame = decoder.decode_frame_tuned(&samples);
        assert_eq!(frame, Err(TelemetryError::NoEdge));
        assert_eq!(decoder.stream_hint().preamble_skip, 2);
    }

    #[test]
    fn decode_payload_table_reference_vectors() {
        let decoder = BidirDecoder::new(OversamplingConfig::default());

        // 0x0123 in 4b/5b table chunks (0,1,2,3) => (25,27,18,19).
        let valid_encoded = (25u32 << 15) | (27u32 << 10) | (18u32 << 5) | 19u32;
        let valid_raw = gcr_raw_from_encoded_20(valid_encoded);
        let valid = decoder.decode_payload(GcrFrame { raw_21: valid_raw });
        assert_eq!(valid, Ok(TelemetryPayload { raw_16: 0x0123 }));

        // Chunk 0 is invalid in decode table.
        let invalid_encoded = (0u32 << 15) | (27u32 << 10) | (18u32 << 5) | 19u32;
        let invalid_raw = gcr_raw_from_encoded_20(invalid_encoded);
        let invalid = decoder.decode_payload(GcrFrame {
            raw_21: invalid_raw,
        });
        assert_eq!(invalid, Err(GcrDecodeError::InvalidGcrSymbol));
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
        let payload = (data << 4) | (calculate_telemetry_crc(data) as u16);
        assert_eq!(
            parse_telemetry_payload(payload),
            Ok(TelemetryFrame::Current(0x55))
        );

        // Another reachable extended type (0x0A).
        let data = 0xA55;
        let payload = (data << 4) | (calculate_telemetry_crc(data) as u16);
        assert_eq!(
            parse_telemetry_payload(payload),
            Ok(TelemetryFrame::Debug2(0x55))
        );
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
