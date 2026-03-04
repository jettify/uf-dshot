#![no_std]
#![no_main]

mod fmt;

use embassy_executor::Spawner;
use embassy_stm32::gpio::{AnyPin, Flex, Pin as _};
use embassy_stm32::timer::low_level::Timer;
use embassy_stm32::timer::Channel as TimerChannel;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Receiver, Watch};
use embassy_time::{Duration, Timer as EmbassyTimer};
use fmt::info;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::{
    BidirDecodeConfig, BiDirDShotFrame, BidirLine, Frame, Stm32BidirController, Stm32BidirParts,
};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

static MOTOR_THROTTLE: Watch<ThreadModeRawMutex, u16, 1> = Watch::new_with(0);

const DSHOT_BITRATE_HZ: u32 = 300_000;
const TELEMETRY_OVERSAMPLE: u32 = 3;
const TELEMETRY_SAMPLE_HZ: u32 = DSHOT_BITRATE_HZ * TELEMETRY_OVERSAMPLE;
const TELEMETRY_SAMPLES: usize = 69;

struct BoardConfig {
    line: BidirLine<'static>,
    pin_bit: u32,
    idr_addr: *mut u16,
    send_timer: Timer<'static, embassy_stm32::peripherals::TIM1>,
    send_dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA2_CH5>,
    send_channel: TimerChannel,
    sample_timer: Timer<'static, embassy_stm32::peripherals::TIM2>,
    sample_dma: embassy_stm32::Peri<'static, embassy_stm32::peripherals::DMA1_CH1>,
}

fn make_cycles(throttle: u16, max_duty: u16) -> [u16; 17] {
    let throttle = throttle.clamp(0, 100);
    let value = (2000 * throttle) / 100;
    let frame = BiDirDShotFrame::new_throttle(value);
    frame.duty_cycles(max_duty)
}

#[embassy_executor::task]
async fn motor_task(
    mut controller: Stm32BidirController<
        'static,
        embassy_stm32::peripherals::TIM1,
        embassy_stm32::peripherals::TIM2,
        embassy_stm32::peripherals::DMA2_CH5,
        embassy_stm32::peripherals::DMA1_CH1,
    >,
    mut receiver: Receiver<'static, ThreadModeRawMutex, u16, 1>,
) -> ! {
    let mut throttle: u16 = 0;
    let mut counter = 0u32;
    let mut buffer = [0u16; TELEMETRY_SAMPLES];

    loop {
        counter = counter.wrapping_add(1);
        throttle = receiver.try_get().unwrap_or(throttle);

        let cycles = make_cycles(throttle, controller.max_duty());
        match controller.send_and_receive(&cycles, &mut buffer).await {
            Ok(frame) if counter % 1000 == 0 => info!("telemetry: {:?}", frame.telemetry),
            Err(err) if counter % 1000 == 0 => info!("telemetry err: {:?}", err),
            _ => {}
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    let pin_port = p.PA8.port() * 16 + p.PA8.pin();
    let pin_bit = p.PA8.pin() as u32;
    let idr_addr = unsafe { AnyPin::steal(pin_port).block().idr().as_ptr() as *mut u16 };

    let cfg = BoardConfig {
        // Update these resources to match your board's timer/DMA/pin mapping.
        line: BidirLine::new(Flex::new(p.PA8), 1),
        pin_bit,
        idr_addr,
        send_timer: Timer::new(p.TIM1),
        send_dma: p.DMA2_CH5,
        send_channel: TimerChannel::Ch1,
        sample_timer: Timer::new(p.TIM2),
        sample_dma: p.DMA1_CH1,
    };

    let controller = Stm32BidirController::new(Stm32BidirParts {
        line: cfg.line,
        tx_timer: cfg.send_timer,
        tx_channel: cfg.send_channel,
        tx_dma: cfg.send_dma,
        tx_bitrate_hz: DSHOT_BITRATE_HZ,
        rx_timer: cfg.sample_timer,
        rx_dma: cfg.sample_dma,
        rx_idr_addr: cfg.idr_addr,
        rx_pin_bit: cfg.pin_bit,
        rx_sample_hz: TELEMETRY_SAMPLE_HZ,
        decode_cfg: BidirDecodeConfig::default(),
    });

    let sender = MOTOR_THROTTLE.sender();
    sender.send(0);
    let receiver = MOTOR_THROTTLE.receiver().unwrap();

    spawner.must_spawn(motor_task(controller, receiver));

    EmbassyTimer::after(Duration::from_millis(7000)).await;
    info!("start loop");
    loop {
        EmbassyTimer::after(Duration::from_secs(3600)).await;
    }
}
