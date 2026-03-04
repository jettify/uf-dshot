use embassy_stm32::gpio::{AfType, Flex, OutputType, Pull, Speed};

/// Bidirectional DShot line ownership and turnaround helpers.
pub struct BidirLine<'d> {
    pin: Flex<'d>,
    af: u8,
}

impl<'d> BidirLine<'d> {
    pub fn new(pin: Flex<'d>, af: u8) -> Self {
        Self { pin, af }
    }

    pub fn enter_tx(&mut self) {
        self.pin.set_as_af_unchecked(
            self.af,
            AfType::output(OutputType::PushPull, Speed::VeryHigh),
        );
    }

    pub fn enter_rx_pullup(&mut self) {
        self.pin.set_as_input(Pull::Up);
    }
}
