#![no_std]
#![no_main]

mod fmt;

use embassy_executor::Spawner;
use embassy_stm32::gpio::OutputType;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::DMA2_CH5;
use embassy_stm32::time::khz;
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::timer::Channel;
use embassy_stm32::Peri;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::watch::{Receiver, Watch};
use embassy_time::{Duration, Ticker, Timer};
use fmt::info;

#[cfg(not(feature = "defmt"))]
use panic_halt as _;

use uf_dshot::{BiDirDShotFrame, Frame};
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

static MOTOR_THROTTLE: Watch<ThreadModeRawMutex, u16, 1> = Watch::new_with(0);

fn make_cycles(throgle: u16, max_duty: u16) -> [u16; 17] {
    let throtle = throgle.clamp(0, 100);
    let v = (2000 * throtle) / 100;
    let frame = BiDirDShotFrame::new_throttle(v);
    frame.duty_cycles(max_duty)
}

#[embassy_executor::task]
async fn motor_task(
    mut pwm: SimplePwm<'static, embassy_stm32::peripherals::TIM1>,
    mut dma_ch: Peri<'static, DMA2_CH5>,
    mut receiver: Receiver<'static, ThreadModeRawMutex, u16, 1>,
) -> ! {
    let max_duty = pwm.max_duty_cycle();
    let mut ch1 = pwm.ch1();
    ch1.enable();
    ch1.set_polarity(embassy_stm32::timer::low_level::OutputPolarity::ActiveLow);
    //ch1.set_polarity(embassy_stm32::timer::low_level::OutputPolarity::ActiveLow);
    //ch1.set_duty_cycle_fully_on();
    //ch1.set_duty_cycle_fully_on();

    let cycles = make_cycles(0, max_duty);
    pwm.waveform_up(dma_ch.reborrow(), Channel::Ch1, &cycles)
        .await;

    Timer::after(Duration::from_millis(300)).await;
    //Timer::after(Duration::from_micros(300)).await;

    info!("Armed");

    //let mut ticker = Ticker::every(Duration::from_micros(50));
    //let mut ticker = Ticker::every(Duration::from_micros(50));
    let mut ticker = Ticker::every(Duration::from_micros(50));
    let mut throttle: u16 = 0;
    let mut counter = 0;
    loop {
        counter = (counter + 1) % 1000;
        throttle = receiver.try_get().unwrap_or(throttle);
        if counter == 0 {
            info!("throgle: {:?}", throttle);
        }
        let cycles = make_cycles(throttle, max_duty);
        pwm.waveform_up(dma_ch.reborrow(), Channel::Ch1, &cycles)
            .await;
        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    let mut led = Output::new(p.PC13, Level::High, Speed::Low);

    let ch1_pin = PwmPin::new(p.PA8, OutputType::PushPull);

    let pwm = SimplePwm::new(
        p.TIM1,
        Some(ch1_pin),
        None,
        None,
        None,
        // DShot300 ?
        khz(300),
        Default::default(),
    );
    let dma_ch = p.DMA2_CH5;

    let sender = MOTOR_THROTTLE.sender();
    sender.send(0);

    let receiver = MOTOR_THROTTLE.receiver().unwrap();

    spawner.must_spawn(motor_task(pwm, dma_ch, receiver));
    sender.send(0);

    Timer::after(Duration::from_millis(0)).await;
    Timer::after(Duration::from_millis(7000)).await;

    led.set_high();

    info!("starat loop");
    loop {
        led.set_high();
        for i in 0..=25 {
            sender.send(i);
            info!("DShot sending {:?}", i);
            Timer::after(Duration::from_millis(1000)).await;
        }
        led.set_low();
        info!("DShot going down");
        for i in (0..=25).rev() {
            sender.send(i);
            info!("DShot sending {:?}", i);
            Timer::after(Duration::from_millis(1000)).await;
        }
    }
}
