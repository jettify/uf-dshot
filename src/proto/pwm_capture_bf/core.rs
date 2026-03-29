pub const BF_FRAME_BITS: u8 = 21;
pub const BF_MIN_EDGE_COUNT: usize = 8;
pub const BF_QUANTIZER_DIVISOR: u16 = 16;
pub const BF_QUANTIZER_ROUNDING: u16 = 8;
pub const BF_CAPTURE_FILTER: u8 = 2;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmCaptureDecodeConfig {
    pub frame_bits: u8,
    pub min_edge_count: usize,
    pub ticks_per_symbol: u16,
    pub quantizer_rounding: u16,
}

impl Default for PwmCaptureDecodeConfig {
    fn default() -> Self {
        Self {
            frame_bits: BF_FRAME_BITS,
            min_edge_count: BF_MIN_EDGE_COUNT,
            ticks_per_symbol: BF_QUANTIZER_DIVISOR,
            quantizer_rounding: BF_QUANTIZER_ROUNDING,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PwmCaptureDecodeDebug {
    pub edge_count: usize,
    pub deltas_used: usize,
    pub bits_decoded_before_tail: u8,
    pub bits_decoded_total: u8,
    pub zero_deltas_skipped: usize,
    pub raw_21: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PwmCaptureDecodeError {
    InvalidConfig,
    NotEnoughEdges { count: usize, min_required: usize },
    NoUsefulEdges,
    InvalidFrame,
}
