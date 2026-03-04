use crate::{Command, DShotValue};

pub trait Frame {
    fn to_payload(&self) -> u16;

    fn duty_cycles(&self, max_duty_cycle: u16) -> [u16; 17];
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum DshotSpeed {
    Dshot150,
    Dshot300,
    Dshot600,
}

impl DshotSpeed {
    pub const fn bitrate(self) -> u32 {
        match self {
            Self::Dshot150 => 150_000,
            Self::Dshot300 => 300_000,
            Self::Dshot600 => 600_000,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct DShotFrame {
    pub(crate) value: DShotValue,
    pub(crate) telemetry_request: bool,
}

impl DShotFrame {
    /// Creates a throttle frame and clamps values above `1999`.
    pub fn new_throttle(throttle: u16, telemetry_request: bool) -> Self {
        Self {
            value: DShotValue::new_clipped_throttle(throttle),
            telemetry_request,
        }
    }

    /// Creates a throttle frame and returns `None` when `throttle > 1999`.
    pub fn new_throttle_checked(throttle: u16, telemetry_request: bool) -> Option<Self> {
        DShotValue::new_checked_throttle(throttle).map(|value| Self {
            value,
            telemetry_request,
        })
    }

    /// Creates a new DShot frame with a command.
    pub fn new_command(command: Command, telemetry_request: bool) -> Self {
        Self {
            value: DShotValue::new_command(command),
            telemetry_request,
        }
    }
}

fn calculate_crc(value: u16) -> u8 {
    ((value ^ (value >> 4) ^ (value >> 8)) & 0x0F) as u8
}

fn calculate_inverted_crc(value: u16) -> u8 {
    let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    !((crc as u8) & 0x0F) & 0x0F
}

impl Frame for DShotFrame {
    fn to_payload(&self) -> u16 {
        crate::common::to_payload(self.value, self.telemetry_request, calculate_crc)
    }

    fn duty_cycles(&self, max_duty_cycle: u16) -> [u16; 17] {
        let one = max_duty_cycle * 3 / 4;
        let zero = max_duty_cycle * 3 / 8;

        let mut value = self.to_payload();
        let mut rv = [one; 17];
        for item in rv.iter_mut() {
            let bit = value & 0x8000;
            if bit == 0 {
                *item = zero;
            }
            value <<= 1;
        }
        rv[16] = 0;
        rv
    }
}

/// Represents a Bidirectional DShot frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BiDirDShotFrame {
    pub(crate) value: DShotValue,
}

impl BiDirDShotFrame {
    /// Creates a bidirectional throttle frame and clamps values above `1999`.
    pub fn new_throttle(throttle: u16) -> Self {
        Self {
            value: DShotValue::new_clipped_throttle(throttle),
        }
    }

    /// Creates a bidirectional throttle frame and returns `None` when `throttle > 1999`.
    pub fn new_throttle_checked(throttle: u16) -> Option<Self> {
        DShotValue::new_checked_throttle(throttle).map(|value| Self { value })
    }

    /// Creates a new Bidirectional DShot frame with a command.
    pub fn new_command(command: Command) -> Self {
        Self {
            value: DShotValue::Command(command),
        }
    }
}

impl Frame for BiDirDShotFrame {
    fn to_payload(&self) -> u16 {
        crate::common::to_payload(self.value, true, calculate_inverted_crc)
    }

    fn duty_cycles(&self, max_duty_cycle: u16) -> [u16; 17] {
        let one = max_duty_cycle * 3 / 4;
        let zero = max_duty_cycle * 3 / 8;

        let mut value = self.to_payload();
        let mut rv = [one; 17];
        for item in rv.iter_mut() {
            let bit = value & 0x8000;
            if bit == 0 {
                *item = zero;
            }
            value <<= 1;
        }
        rv[16] = 0;
        rv
    }
}

#[cfg(test)]
mod tests {
    use super::{BiDirDShotFrame, Command, DShotFrame, DShotValue, DshotSpeed, Frame};
    use crate::common::MAX_THROTTLE;

    #[test]
    fn test_dshot_frame_new_throttle() {
        // Valid throttle, no telemetry
        let frame = DShotFrame::new_throttle(500, false);
        assert_eq!(frame.value, DShotValue::Throttle(500));
        assert!(!frame.telemetry_request);

        // Valid throttle, with telemetry
        let frame = DShotFrame::new_throttle(1000, true);
        assert_eq!(frame.value, DShotValue::Throttle(1000));
        assert!(frame.telemetry_request);

        // Min valid throttle
        let frame = DShotFrame::new_throttle(0, false);
        assert_eq!(frame.value, DShotValue::Throttle(0));

        // Max valid throttle
        let frame = DShotFrame::new_throttle(1999, false);
        assert_eq!(frame.value, DShotValue::Throttle(1999));

        // Invalid throttle (too high)
        let frame = DShotFrame::new_throttle(2058, false);
        assert_eq!(frame.value, DShotValue::Throttle(1999));
    }

    #[test]
    fn test_dshot_frame_new_command() {
        // Command, no telemetry
        let frame = DShotFrame::new_command(Command::Beep1, false);
        assert_eq!(frame.value, DShotValue::Command(Command::Beep1));
        assert!(!frame.telemetry_request);

        // Command, with telemetry
        let frame = DShotFrame::new_command(Command::Beep4, true);
        assert_eq!(frame.value, DShotValue::Command(Command::Beep4));
        assert!(frame.telemetry_request);
    }

    #[test]
    fn test_bidir_dshot_frame_new_throttle() {
        // Valid throttle, no telemetry
        let frame = BiDirDShotFrame::new_throttle(500);
        assert_eq!(frame.value, DShotValue::Throttle(500));

        // Min valid throttle
        let frame = BiDirDShotFrame::new_throttle(0);
        assert_eq!(frame.value, DShotValue::Throttle(0));

        // Max valid throttle
        let frame = BiDirDShotFrame::new_throttle(1999);
        assert_eq!(frame.value, DShotValue::Throttle(1999));

        // Invalid throttle (too high)
        let frame = BiDirDShotFrame::new_throttle(2058);
        assert_eq!(frame.value, DShotValue::Throttle(1999));
    }

    #[test]
    fn test_checked_throttle_apis() {
        let frame = DShotFrame::new_throttle_checked(MAX_THROTTLE, true).unwrap();
        assert_eq!(frame.value, DShotValue::Throttle(MAX_THROTTLE));
        assert!(DShotFrame::new_throttle_checked(MAX_THROTTLE + 1, false).is_none());

        let bidir = BiDirDShotFrame::new_throttle_checked(MAX_THROTTLE).unwrap();
        assert_eq!(bidir.value, DShotValue::Throttle(MAX_THROTTLE));
        assert!(BiDirDShotFrame::new_throttle_checked(MAX_THROTTLE + 1).is_none());
    }

    #[test]
    fn test_bidir_dshot_frame_new_command() {
        // Command, no telemetry
        let frame = BiDirDShotFrame::new_command(Command::Beep1);
        assert_eq!(frame.value, DShotValue::Command(Command::Beep1));

        // Command, with telemetry
        let frame = BiDirDShotFrame::new_command(Command::Beep4);
        assert_eq!(frame.value, DShotValue::Command(Command::Beep4));
    }

    #[test]
    fn test_dshot_frame_to_payload() {
        let frame = DShotFrame::new_throttle(0, false);
        assert_eq!(frame.to_payload(), 0x05EB);

        let frame = DShotFrame::new_throttle(1000, false);
        assert_eq!(frame.to_payload(), 0x82E4);

        let frame = DShotFrame::new_throttle(1000, true);
        assert_eq!(frame.to_payload(), 0x82F5);

        let frame = DShotFrame::new_command(Command::MotorStop, false);
        assert_eq!(frame.to_payload(), 0);

        let frame = DShotFrame::new_command(Command::Beep1, true);
        assert_eq!(frame.to_payload(), 0x33);
    }

    #[test]
    fn test_bidir_dshot_frame_to_payload() {
        let frame = BiDirDShotFrame::new_throttle(0);
        assert_eq!(frame.to_payload(), 0x05F5);

        let frame = BiDirDShotFrame::new_throttle(1000);
        assert_eq!(frame.to_payload(), 0x82FA);

        let frame = BiDirDShotFrame::new_command(Command::MotorStop);
        assert_eq!(frame.to_payload(), 0x001E);

        let frame = BiDirDShotFrame::new_command(Command::Beep1);
        assert_eq!(frame.to_payload(), 0x003C);
    }

    #[test]
    fn test_dshot_speed_bitrates() {
        assert_eq!(DshotSpeed::Dshot150.bitrate(), 150_000);
        assert_eq!(DshotSpeed::Dshot300.bitrate(), 300_000);
        assert_eq!(DshotSpeed::Dshot600.bitrate(), 600_000);
    }
}
