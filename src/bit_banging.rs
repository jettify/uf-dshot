/// Represents the possible errors during the decoding process.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DecodeError {
    /// The provided bit index is out of the valid range (0-15).
    InvalidBit,
    /// No valid signal edge was found in the buffer. This typically means
    /// no telemetry data was transmitted.
    NoEdge,
    /// A signal was detected, but its timing or structure was invalid,
    /// suggesting a corrupted data frame.
    InvalidFrame,
}

/// Contains the successful result of a decoding operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DecodeResult {
    /// The raw 21-bit GCR-encoded value, ready for the next stage of
    /// decoding (GCR-to-binary lookup).
    pub gcr_value: u32,
    /// The number of samples from the start of the buffer to the first
    /// valid signal edge. This can be used by the caller to optimize the
    /// `preamble_skip` for subsequent calls.
    pub start_margin: usize,
}

/// Decodes a DShot telemetry signal from a raw sample buffer into a GCR value.
///
/// This function is platform-agnostic and suitable for `#[no_std]` environments.
/// It analyzes the timing of rising and falling edges in the signal to reconstruct
/// the GCR-encoded data frame.
///
/// # Arguments
///
/// * `buffer`: A slice of `u16` values from a DMA or ADC buffer. Each `u16` can
///   contain samples for multiple signals.
/// * `bit`: The bit index (0-15) within each `u16` sample that corresponds to the
///   signal line to be decoded.
/// * `preamble_skip`: An optimization hint. The number of initial samples to skip
///   before searching for the first signal edge. This should be adjusted by the
///   caller based on the `start_margin` of previous successful decodes.
///
/// # Returns
///
/// A `Result` containing either the successfully decoded `DecodeResult` or a
/// `DecodeError` explaining the failure.
pub fn decode_gcr_from_samples(
    buffer: &[u16],
    bit: u32,
    preamble_skip: usize,
) -> Result<DecodeResult, DecodeError> {
    if bit >= 16 {
        return Err(DecodeError::InvalidBit);
    }

    // --- Constants based on DShot telemetry protocol ---
    const TOTAL_EXPECTED_BITS: u32 = 21;
    // Minimum number of samples for a valid frame: (21 bits - 2 tolerance) * 3x oversampling
    const MIN_FRAME_SAMPLES: usize = (TOTAL_EXPECTED_BITS as usize - 2) * 3;
    // Maximum number of samples for a valid frame: (21 bits + 2 tolerance) * 3x oversampling
    const MAX_FRAME_SAMPLES: usize = (TOTAL_EXPECTED_BITS as usize + 2) * 3;

    // Create a bitmask to isolate the relevant signal bit from each sample.
    let mask = 1u16 << bit;

    // 1. FIND PACKET START: Locate the first falling edge, which marks the end of the
    // high-level preamble and the beginning of the data frame.
    let (first_edge_index, _) = buffer
        .iter()
        .enumerate()
        .skip(preamble_skip)
        .find(|&(_, &sample)| (sample & mask) == 0)
        .ok_or(DecodeError::NoEdge)?; // Return error if no edge is found.

    let start_margin = first_edge_index;

    // Ensure there are enough remaining samples to constitute a valid frame.
    if buffer.len() < start_margin + MIN_FRAME_SAMPLES {
        return Err(DecodeError::InvalidFrame);
    }

    // Define the window of samples to analyze for this frame.
    let search_window_len = (buffer.len() - start_margin).min(MAX_FRAME_SAMPLES);
    let frame_samples = &buffer[start_margin..start_margin + search_window_len];

    // --- State for edge detection loop ---
    let mut gcr_value: u32 = 0;
    let mut bits_found: u32 = 0;
    let mut edges_found: u32 = 0;
    let mut last_edge_index = 0; // Index relative to the start of `frame_samples`
    let mut last_state_is_high = false; // We start at the first falling edge, so the state is low.

    // 2. EDGE DETECTION LOOP: Iterate through the frame to find all subsequent edges.
    for (i, &sample) in frame_samples.iter().enumerate().skip(1) {
        let current_state_is_high = (sample & mask) != 0;

        if current_state_is_high != last_state_is_high {
            // An edge was detected at index `i`.
            let pulse_width_samples = i - last_edge_index;

            // Convert the pulse duration (in samples) into a bit length for GCR.
            // The logic `(width + 1) / 3` replicates the original C implementation.
            let len = ((pulse_width_samples + 1) / 3).max(1) as u32;

            bits_found += len;
            edges_found += 1;

            // A pulse of `len` is encoded as a '''1''' followed by `len - 1` zeros.
            gcr_value <<= len;
            gcr_value |= 1 << (len - 1);

            // The frame cannot exceed the total expected bit count.
            if bits_found > TOTAL_EXPECTED_BITS {
                return Err(DecodeError::InvalidFrame);
            }

            // Update state for the next iteration.
            last_edge_index = i;
            last_state_is_high = current_state_is_high;
        }
    }

    // 3. INFER FINAL PULSE & VALIDATE: The signal always ends high, so the last falling
    // edge is absent. We infer its length based on the bits we've already found.
    if edges_found == 0 {
        return Err(DecodeError::NoEdge);
    }

    let remaining_bits = TOTAL_EXPECTED_BITS - bits_found;
    if remaining_bits > 0 {
        // The shift amount must be less than the type's bit width.
        gcr_value <<= remaining_bits;
        gcr_value |= 1 << (remaining_bits - 1);
    }

    Ok(DecodeResult {
        gcr_value,
        start_margin,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate std;
    use std::vec;
    use std::vec::Vec;

    /// Helper to generate a test buffer.
    /// `bit`: The bit position to set in the u16 samples.
    /// `preamble_high`: Number of high samples before the first (falling) edge.
    /// `pulses`: Sequence of pulse widths in samples. First pulse is low, second high, etc.
    fn build_buffer(bit: u32, preamble_high: usize, pulses: &[usize]) -> Vec<u16> {
        let mask = 1u16 << bit;
        let mut buffer = Vec::new();

        // Preamble (high)
        buffer.resize(preamble_high, mask);

        // Pulses
        let mut is_high = false;
        for &width in pulses {
            let value = if is_high { mask } else { 0 };
            for _ in 0..width {
                buffer.push(value);
            }
            is_high = !is_high;
        }
        buffer
    }

    #[test]
    fn test_perfect_frame() {
        // Sequence of pulse lengths (in bits) that sum to 21
        let bit_lens = [1, 2, 3, 4, 5, 6];
        // Corresponding sample widths for 3x oversampling
        let pulse_widths: Vec<usize> = bit_lens.iter().map(|&len| len * 3 - 1).collect();
        let preamble_len = 10;
        let bit = 0;

        let buffer = build_buffer(bit, preamble_len, &pulse_widths);

        // Calculate expected GCR value from the bit lengths
        let mut expected_gcr = 0u32;
        for &len in &bit_lens {
            expected_gcr <<= len;
            expected_gcr |= 1 << (len - 1);
        }

        let result = decode_gcr_from_samples(&buffer, bit, 0).unwrap();
        assert_eq!(result.gcr_value, expected_gcr);
        assert_eq!(result.start_margin, preamble_len);
    }

    #[test]
    fn test_inferred_final_pulse() {
        let bit_lens = [1, 2, 3, 4, 5, 6]; // Sum to 21
        let pulse_widths: Vec<usize> = bit_lens.iter().map(|&len| len * 3 - 1).collect();
        let preamble_len = 10;
        let bit = 7;

        let mut full_buffer = build_buffer(bit, preamble_len, &pulse_widths);
        full_buffer.extend([1u16 << bit; 16]);
        // Cut the buffer short in the middle of the final pulse
        let partial_buffer = &full_buffer[..full_buffer.len() - 8];

        let mut expected_gcr = 0u32;
        for &len in &bit_lens {
            expected_gcr <<= len;
            expected_gcr |= 1 << (len - 1);
        }

        // The result should be the same as the full frame
        let result = decode_gcr_from_samples(partial_buffer, bit, 0).unwrap();
        assert_eq!(result.gcr_value, expected_gcr);
        assert_eq!(result.start_margin, preamble_len);
    }

    #[test]
    fn test_preamble_skip() {
        let pulse_widths = [2, 5, 8, 11, 14, 17];
        let preamble_len = 20;
        let bit = 15;
        let buffer = build_buffer(bit, preamble_len, &pulse_widths);

        // Same expected GCR as test_perfect_frame
        let expected_gcr = 0b110100100010000100000;

        // Skip part of the preamble
        let result = decode_gcr_from_samples(&buffer, bit, 10).unwrap();
        assert_eq!(result.gcr_value, expected_gcr);
        assert_eq!(result.start_margin, preamble_len);
    }

    #[test]
    fn test_error_invalid_bit() {
        let err = decode_gcr_from_samples(&[], 16, 0).unwrap_err();
        assert_eq!(err, DecodeError::InvalidBit);
    }

    #[test]
    fn test_error_no_edge() {
        // All high
        let buffer_high = vec![1u16; 100];
        let err_high = decode_gcr_from_samples(&buffer_high, 0, 0).unwrap_err();
        assert_eq!(err_high, DecodeError::NoEdge);

        // All low
        let buffer_low = vec![0u16; 100];
        let err_low = decode_gcr_from_samples(&buffer_low, 0, 0).unwrap_err();
        assert_eq!(err_low, DecodeError::NoEdge);
    }

    #[test]
    fn test_error_frame_too_short() {
        // Frame is too short to contain MIN_FRAME_SAMPLES
        let buffer = build_buffer(0, 10, &[3, 3, 3]); // Not enough samples after margin
        let err = decode_gcr_from_samples(&buffer, 0, 0).unwrap_err();
        assert_eq!(err, DecodeError::InvalidFrame);
    }

    #[test]
    fn test_error_too_few_bits() {
        // Total bits found is 17, which is less than MIN_DETECTED_BITS (18)
        let bit_lens = [3, 3, 3, 3, 3, 2]; // Sum to 17
        let pulse_widths: Vec<usize> = bit_lens.iter().map(|&len| len * 3 - 1).collect();
        let buffer = build_buffer(0, 10, &pulse_widths);

        let err = decode_gcr_from_samples(&buffer, 0, 0).unwrap_err();
        assert_eq!(err, DecodeError::InvalidFrame);
    }

    #[test]
    fn test_error_too_many_bits() {
        // First low pulse is very long and decodes to >21 bits immediately.
        let pulse_widths = [68, 3];
        let buffer = build_buffer(0, 10, &pulse_widths);

        let err = decode_gcr_from_samples(&buffer, 0, 0).unwrap_err();
        assert_eq!(err, DecodeError::InvalidFrame);
    }

    #[test]
    fn test_error_invalid_frame_on_short_search_window() {
        let buffer = build_buffer(0, 40, &[2, 2, 2, 2, 2, 2]);
        let err = decode_gcr_from_samples(&buffer, 0, 0).unwrap_err();
        assert_eq!(err, DecodeError::InvalidFrame);
    }
}
