use embassy_stm32::dma::TransferOptions;
use embassy_stm32::dma::{Channel, Priority, Request, Transfer};
use embassy_stm32::gpio::{Input, Pin, Pull};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::timer::{BasicNoCr2Instance, CoreInstance};

const BDSHOT_RESPONSE_BITRATE: u32 = 300_000;
const BDSHOT_RESPONSE_OVERSAMPLING: u32 = 3;
const RX_GAP_SAMPLES: usize = 2;
const RX_DATA_SAMPLES: usize = 2;
const RX_TOTAL_SAMPLES: usize = (RX_GAP_SAMPLES + RX_DATA_SAMPLES) as usize;
static mut RX_BUFFER: [u32; RX_TOTAL_SAMPLES] = [0; RX_TOTAL_SAMPLES];

pub async fn receive_telemetry<'d, P, T, C>(
    pin_peripheral: P,
    timer: &Timer<'d, T>,
    dma_channel: C,
    dma_request: Request,
) where
    P: Pin,
    T: CoreInstance + BasicNoCr2Instance,
    C: Channel,
{
    // --- 1. Reconfigure GPIO (Unchanged) ---
    let input_pin = Input::new(pin_peripheral, Pull::Up);
    let port = input_pin.port();

    // --- 2. Configure Timer (Unchanged) ---
    timer.stop();
    let sample_freq = Hertz((BDSHOT_RESPONSE_BITRATE * BDSHOT_RESPONSE_OVERSAMPLING) as u32);
    timer.set_frequency(sample_freq);
    timer.enable_update_dma(true);

    // --- 3. Configure DMA (Updated) ---

    // Get the address of the GPIO Port's Input Data Register (IDR)
    // We cast it to *mut u32 as the `new_read` API expects a mutable peripheral pointer.
    let idr_addr = port.idr().as_ptr() as *mut u32;

    // **Use TransferOptions instead of Config**
    let mut options = TransferOptions::default();
    options.priority = Priority::High;
    // Word sizes are now inferred from the pointer/slice types (u32).
    // Peripheral-to-Memory direction is set by `new_read`.
    // Memory increment is the default for `new_read`.
    // Peripheral increment is off by default.

    // SAFETY: We have 'static mut access to RX_BUFFER
    // and exclusive access to the dma_channel.
    // **Use Transfer::new_read**
    let dma_transfer = unsafe {
        Transfer::new_read(
            dma_channel,
            dma_request,
            idr_addr,       // Source: GPIOA->IDR (as *mut u32)
            &mut RX_BUFFER, // Destination: our buffer (&mut [u32])
            options,        // The new options struct
        )
    };

    // --- 4. Start Timer and DMA Transfer (Unchanged) ---
    timer.start();
    dma_transfer.await;

    timer.stop();
    timer.enable_update_dma(false);
}
