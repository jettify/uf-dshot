use embassy_stm32::dma::{Request, Transfer, TransferOptions};
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::timer::{BasicNoCr2Instance, CoreInstance};
use embassy_stm32::{time::Hertz, Peri};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RxError {
    EmptyBuffer,
}

pub async fn capture_port_samples<T: CoreInstance + BasicNoCr2Instance>(
    timer: &Timer<'_, T>,
    dma: Peri<'_, impl embassy_stm32::dma::Channel>,
    idr_addr: *mut u16,
    sample_hz: u32,
    buffer: &mut [u16],
) -> Result<(), RxError> {
    if buffer.is_empty() {
        return Err(RxError::EmptyBuffer);
    }

    timer.stop();
    timer.set_frequency(Hertz(sample_hz));
    timer.enable_update_dma(true);

    let options = TransferOptions::default();

    // SAFETY:
    // - `idr_addr` must point to a valid GPIO IDR register for the entire transfer.
    // - `buffer` is uniquely borrowed for DMA write.
    // - DMA channel ownership is unique through `Peri`.
    unsafe {
        Transfer::new_read(dma, Request::default(), idr_addr, buffer, options).await;
    }

    timer.stop();
    timer.enable_update_dma(false);
    Ok(())
}
