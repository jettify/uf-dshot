use crate::{Command, DShotValue};

pub trait Frame {
    fn to_payload(&self) -> u16;
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct DShotFrame {
    pub(crate) value: DShotValue,
    pub(crate) telemetry_request: bool,
}

impl DShotFrame {
    /// Creates a new DShot frame with a throttle value.
    /// Returns None if the throttle is outside the valid range (0-1999).
    pub fn new_throttle(throttle: u16, telemetry_request: bool) -> Self {
        Self {
            value: DShotValue::new_clipped_throttle(throttle),
            telemetry_request,
        }
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
}

/// Represents a Bidirectional DShot frame.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct BiDirDShotFrame {
    pub(crate) value: DShotValue,
}

impl BiDirDShotFrame {
    /// Creates a new Bidirectional DShot frame with a throttle value.
    /// Returns None if the throttle is outside the valid range (0-1999).
    pub fn new_throttle(throttle: u16) -> Self {
        Self {
            value: DShotValue::new_clipped_throttle(throttle),
        }
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
}

#[cfg(test)]
mod tests {
    use super::{BiDirDShotFrame, Command, DShotFrame, DShotValue, Frame};

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
}
