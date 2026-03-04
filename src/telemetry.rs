#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Telemetry {
    ERPM(u16),
    // Extended telemetry
    Temperature(u8),
    Voltage(u8),
    Current(u8),
    Debug1(u8),
    Debug2(u8),
    Debug3(u8),
    StateEvent(u8),
    Unknown(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TelemetryError {
    InvalidCrc { calculated_crc: u8, packet_crc: u8 },
}

fn calculate_crc(value: u16) -> u8 {
    ((value ^ (value >> 4) ^ (value >> 8)) & 0x0F) as u8
}

/// Parses a 16-bit telemetry payload.
/// The payload is the 16-bit value *after* GCR decoding.
pub fn parse_telemetry_payload(payload: u16) -> Result<Telemetry, TelemetryError> {
    let data = payload >> 4;
    let packet_crc = (payload & 0x0F) as u8;
    let calculated_crc = calculate_crc(data);

    if packet_crc != calculated_crc {
        return Err(TelemetryError::InvalidCrc {
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
            0x02 => Telemetry::Temperature(value),
            0x04 => Telemetry::Voltage(value),
            0x06 => Telemetry::Current(value),
            0x08 => Telemetry::Debug1(value),
            0x0A => Telemetry::Debug2(value),
            0x0C => Telemetry::Debug3(value),
            0x0E => Telemetry::StateEvent(value),
            _ => Telemetry::Unknown(value),
        };
        Ok(t)
    } else {
        let period = mantissa << exponent;
        Ok(Telemetry::ERPM(period))
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
    // 0x00 - 0x07
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // 0x08 - 0x0F
    0xFF, 0x09, 0x0A, 0x0B, 0xFF, 0x0D, 0x0E, 0x0F, // 0x10 - 0x17
    0xFF, 0xFF, 0x02, 0x03, 0xFF, 0x05, 0x06, 0x07, // 0x18 - 0x1F
    0xFF, 0x00, 0x08, 0x01, 0xFF, 0x04, 0x0C, 0xFF,
];

/// Decodes a 21-bit GCR value into a 16-bit telemetry payload.
/// The input `gcr_value` should only contain the 21 bits.
/// Returns None if the decoding is invalid.
pub fn decode_gcr(gcr_value: u32) -> Option<u16> {
    let gcr_encoded = (gcr_value ^ (gcr_value >> 1)) & 0xFFFFF;

    let mut result: u16 = 0;
    for i in 0..4 {
        let chunk = (gcr_encoded >> (15 - i * 5)) & 0x1F;
        let nibble = GCR_DECODE_TABLE[chunk as usize];
        if nibble == 0xFF {
            return None;
        }
        result = (result << 4) | (nibble as u16);
    }
    Some(result)
}

const GCR_ENCODE_TABLE: [u8; 16] = [
    // 0x0,    0x1,    0x2,    0x3,    0x4,    0x5,    0x6,    0x7
    0b11001, 0b11011, 0b10010, 0b10011, 0b11101, 0b10101, 0b10110, 0b10111,
    // 0x8,    0x9,    0xA,    0xB,    0xC,    0xD,    0xE,    0xF
    0b11010, 0b01001, 0b01010, 0b01011, 0b11110, 0b01101, 0b01110, 0b01111,
];

/// Encodes a 16-bit telemetry payload into a 21-bit GCR value.
/// This is the reverse of `decode_gcr` and is useful for testing.
pub fn encode_gcr(payload: u16) -> u32 {
    let mut gcr_encoded: u32 = 0;
    for i in 0..4 {
        let nibble = (payload >> (12 - i * 4)) & 0x0F;
        let chunk = GCR_ENCODE_TABLE[nibble as usize] as u32;
        gcr_encoded = (gcr_encoded << 5) | chunk;
    }

    // NRZI encode the 20-bit GCR value into a 21-bit frame.
    // The first bit (MSB) of the frame is a start bit, which is always 1.
    // Subsequent bits are inverted whenever there is a 1 in the GCR value.
    // This is equivalent to: n[i+1] = n[i] ^ g[i]
    // where n is the NRZI stream and g is the GCR stream.
    // We start with n[20] = 1.
    let mut gcr_value: u32 = 1 << 20;
    let mut last_bit = 1;
    for i in 0..20 {
        let gcr_bit = (gcr_encoded >> (19 - i)) & 1;
        let current_bit = last_bit ^ gcr_bit;
        gcr_value |= current_bit << (19 - i);
        last_bit = current_bit;
    }
    gcr_value
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gcr_encode_decode() {
        // let test_payloads = [0, 1, 0xAAAA, 0x5555, 0x1234, 0xFEDC, u16::MAX];
        let test_payloads = [23130, 0xAAAA, 0x5555, 0x1234, 0xFEDC, u16::MAX];
        for &payload in &test_payloads {
            let encoded = encode_gcr(payload);

            let decoded = decode_gcr(encoded);
            assert_eq!(
                decoded,
                Some(payload),
                "Failed for payload {:#06x}",
                payload
            );
        }
    }

    #[test]
    fn test_calculate_crc() {
        assert_eq!(calculate_crc(0x0), 0x0);
        assert_eq!(calculate_crc(0x123), 0x0);
        assert_eq!(calculate_crc(0xFFF), 0xF);
        assert_eq!(calculate_crc(0x000), 0x0);
        assert_eq!(calculate_crc(0x800), 0x8);
        assert_eq!(calculate_crc(0x810), 0x9);
        assert_eq!(calculate_crc(0x219), 0xA);
        assert_eq!(calculate_crc(0x82C), 0x6);
        assert_eq!(calculate_crc(0b100000101100), 0b000000000110);
    }

    #[test]
    fn test_parse_telemetry_payload_erpm() {
        let payload = 0x8109;
        assert_eq!(parse_telemetry_payload(payload), Ok(Telemetry::ERPM(256)));
    }

    #[test]
    fn test_parse_telemetry_payload_extended_temperature() {
        let payload = 0x219A;
        assert_eq!(
            parse_telemetry_payload(payload),
            Ok(Telemetry::Temperature(25))
        );
    }

    #[test]
    fn test_parse_telemetry_payload_crc_error() {
        let payload = 0x8108;
        let err = Err(TelemetryError::InvalidCrc {
            calculated_crc: 9,
            packet_crc: 8,
        });
        assert_eq!(parse_telemetry_payload(payload), err);
    }

    #[test]
    fn test_decode_gcr_from_known_values() {
        let expected = Telemetry::ERPM(1684);
        let input = 0b011001100110011001100;
        let gcr = decode_gcr(input).unwrap();
        let actual = parse_telemetry_payload(gcr);
        assert_eq!(actual, Ok(expected));
    }
    #[test]
    fn test_encode_gcr_from_known_values() {
        assert_eq!(encode_gcr(23130), 0b100110011001100110011);
        assert_eq!(decode_gcr(0b011001100110011001100).unwrap(), 23130);
        assert_eq!(decode_gcr(0b100110011001100110011).unwrap(), 23130);
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
