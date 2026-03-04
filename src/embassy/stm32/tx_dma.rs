use embassy_stm32::dma::{Request, Transfer, TransferOptions};
use embassy_stm32::timer::low_level::{OutputCompareMode, OutputPolarity, Timer};
use embassy_stm32::timer::{
    AdvancedInstance4Channel, BasicNoCr2Instance, Channel, GeneralInstance1Channel,
};
use embassy_stm32::{time::Hertz, Peri};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TxError {
    EmptyDuty,
}

pub fn setup_dshot_timer<T: AdvancedInstance4Channel>(
    timer: &Timer<'_, T>,
    channel: Channel,
    bitrate_hz: u32,
) -> u16 {
    timer.stop();
    timer.set_output_compare_mode(channel, OutputCompareMode::PwmMode1);
    timer.set_output_compare_preload(channel, true);
    timer.set_output_polarity(channel, OutputPolarity::ActiveLow);
    timer.enable_channel(channel, true);

    timer.set_frequency(Hertz(bitrate_hz));
    timer.set_compare_value(channel, 0);
    timer.enable_outputs();
    timer.set_moe(true);
    timer.start();

    (timer.get_max_compare_value() as u16).saturating_add(1)
}

pub async fn send_waveform<T: GeneralInstance1Channel + BasicNoCr2Instance>(
    timer: &Timer<'_, T>,
    channel: Channel,
    dma: Peri<'_, impl embassy_stm32::dma::Channel>,
    duty: &[u16],
) -> Result<(), TxError> {
    if duty.is_empty() {
        return Err(TxError::EmptyDuty);
    }

    let original_update_dma_state = timer.get_update_dma_state();
    if !original_update_dma_state {
        timer.enable_update_dma(true);
    }

    let ccr = timer.regs_1ch().ccr(channel.index()).as_ptr() as *mut u16;
    let options = TransferOptions::default();

    // SAFETY:
    // - `ccr` points to the timer capture/compare register for `channel`.
    // - `duty` is valid for the duration of the transfer.
    // - DMA channel ownership is unique through `Peri`.
    unsafe {
        Transfer::new_write(dma, Request::default(), duty, ccr, options).await;
    }

    if !original_update_dma_state {
        timer.enable_update_dma(false);
    }

    Ok(())
}
