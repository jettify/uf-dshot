use crate::EncodedFrame;

pub const FRAME_BITS: usize = 16;
pub const STATES_PER_BIT: usize = 3;
pub const TX_HOLD_SLOTS: usize = 1;
pub const TX_STATE_SLOTS: usize = FRAME_BITS * STATES_PER_BIT + TX_HOLD_SLOTS;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SignalPolarity {
    Normal,
    Inverted,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PortFrameError {
    EmptyGroupMask,
    PinMaskOutsideGroup { pin_mask: u32, group_mask: u32 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PortWords {
    pub words: [u32; TX_STATE_SLOTS],
}

impl PortWords {
    pub const fn new() -> Self {
        Self {
            words: [0; TX_STATE_SLOTS],
        }
    }
}

pub fn build_port_words<const N: usize>(
    group_mask: u32,
    pin_masks: [u32; N],
    frames: [EncodedFrame; N],
    polarity: SignalPolarity,
) -> Result<PortWords, PortFrameError> {
    if group_mask == 0 {
        return Err(PortFrameError::EmptyGroupMask);
    }

    let mut out = PortWords::new();
    init_base_words(&mut out.words, group_mask, polarity);

    for (pin_mask, frame) in pin_masks.into_iter().zip(frames.into_iter()) {
        if (pin_mask & group_mask) == 0 {
            return Err(PortFrameError::PinMaskOutsideGroup {
                pin_mask,
                group_mask,
            });
        }
        apply_frame(&mut out.words, pin_mask, frame, polarity);
    }

    Ok(out)
}

fn init_base_words(words: &mut [u32; TX_STATE_SLOTS], group_mask: u32, polarity: SignalPolarity) {
    let (set_mask, reset_mask) = match polarity {
        SignalPolarity::Normal => (group_mask, group_mask << 16),
        SignalPolarity::Inverted => (group_mask << 16, group_mask),
    };

    for bit in 0..FRAME_BITS {
        let base = bit * STATES_PER_BIT;
        words[base] = set_mask;
        words[base + 1] = 0;
        words[base + 2] = reset_mask;
    }

    words[FRAME_BITS * STATES_PER_BIT] = match polarity {
        SignalPolarity::Normal => group_mask << 16,
        SignalPolarity::Inverted => group_mask,
    };
}

fn apply_frame(
    words: &mut [u32; TX_STATE_SLOTS],
    pin_mask: u32,
    frame: EncodedFrame,
    polarity: SignalPolarity,
) {
    let mid_clear_mask = match polarity {
        SignalPolarity::Normal => pin_mask << 16,
        SignalPolarity::Inverted => pin_mask,
    };

    for (bit_idx, is_one) in frame.bits_msb_first().into_iter().enumerate() {
        if !is_one {
            words[bit_idx * STATES_PER_BIT + 1] |= mid_clear_mask;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BidirTx, UniTx};

    #[test]
    fn normal_words_match_three_state_layout() {
        let frame = UniTx::throttle(0).unwrap().encode();
        let out = build_port_words(1 << 3, [1 << 3], [frame], SignalPolarity::Normal).unwrap();

        assert_eq!(out.words[0], 1 << 3);
        assert_eq!(out.words[2], 1 << 19);
        assert_eq!(out.words[48], 1 << 19);
    }

    #[test]
    fn zero_bits_clear_in_middle_slot() {
        let frame = UniTx::command(crate::Command::MotorStop).encode();
        let out = build_port_words(1 << 8, [1 << 8], [frame], SignalPolarity::Normal).unwrap();

        assert_eq!(out.words[1], 1 << 24);
    }

    #[test]
    fn inverted_mode_flips_idle_and_clear_masks() {
        let frame = BidirTx::throttle(100).unwrap().encode();
        let out = build_port_words(1 << 2, [1 << 2], [frame], SignalPolarity::Inverted).unwrap();

        assert_eq!(out.words[0], 1 << 18);
        assert_eq!(out.words[2], 1 << 2);
        assert_eq!(out.words[48], 1 << 2);
    }
}
