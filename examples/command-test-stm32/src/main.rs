#![no_std]
#![no_main]

#[macro_use]
mod fmt;

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use uf_dshot::embassy_stm32::{
    BidirDshotConfig, DshotPortPin, InterruptHandler, Stm32BidirDshotPort,
};
use uf_dshot::{Command, DshotSpeed, DshotTx, TelemetryFrame};

#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

const ESC_SPEED: DshotSpeed = DshotSpeed::Dshot300;
const TEST_THROTTLE: u16 = 150;
const ESC_INFO_RX_TIMEOUT: Duration = Duration::from_millis(20);
const COMMAND_POST_GAP: Duration = Duration::from_millis(1);
const ARM_IDLE_SECS: u64 = 1;
const SPIN_STEP: u16 = 10;
const SPIN_SAMPLES_PER_STEP: usize = 2;
const STOP_FRAMES_AFTER_PHASE: usize = 200;
const STOP_FRAMES_FINAL: usize = 100;
const SPIN_RAMP_HOLD_FRAMES: usize = 10;
const THREED_NEUTRAL: u16 = 1000;
const THREED_GENTLE_DELTA: u16 = 150;
const THREED_TEST_MS: u32 = 800;

embassy_stm32::bind_interrupts!(struct DmaIrqs {
    DMA2_STREAM3 => InterruptHandler<embassy_stm32::peripherals::DMA2_CH3>;
});

#[derive(Default)]
struct TelemetryStats {
    ok: u32,
    err: u32,
}

#[derive(Clone, Copy)]
struct CommandStep {
    cmd: Command,
    label: &'static str,
}

fn frame_period() -> Duration {
    Duration::from_micros(30)
        + Duration::from_micros((ESC_SPEED.timing_hints().min_frame_period_us as u64) * 3)
}

async fn arm_for<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>, secs: u64, message: &str)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!("{}", message);
    unwrap!(esc.arm_for(Duration::from_secs(secs)).await);
}

/// Execute a command according to its DShot execution policy (repetitions and timing).
async fn execute_command<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>, cmd: Command)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    let policy = cmd.exec_policy();
    let encoded = DshotTx::bidirectional().command(cmd);

    for _ in 0..policy.repeat_count().get() {
        unwrap!(esc.send_frame(encoded).await);
        Timer::after(COMMAND_POST_GAP).await;
    }

    // Apply the mandatory post-command delay (e.g., 35ms for Save, 260ms for Beeps).
    if policy.min_gap().as_micros() > 0 {
        Timer::after(Duration::from_micros(policy.min_gap().as_micros() as u64)).await;
    }
}

async fn send_stop_frames<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>, frames: usize)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    let stop_frame = DshotTx::bidirectional().command(Command::MotorStop);
    let period = frame_period();

    for _ in 0..frames {
        unwrap!(esc.send_frame(stop_frame).await);
        Timer::after(period).await;
    }
}

async fn run_command_sequence<T, D>(
    esc: &mut Stm32BidirDshotPort<'_, T, D>,
    commands: &[CommandStep],
) where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    for step in commands {
        info!("{}", step.label);
        execute_command(esc, step.cmd).await;
    }
}

async fn send_telemetry_sample<T, D>(
    esc: &mut Stm32BidirDshotPort<'_, T, D>,
    throttle: u16,
    stats: &mut TelemetryStats,
) where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    match esc.send_throttle_and_receive(throttle).await {
        Ok(_) => stats.ok += 1,
        Err(e) => {
            stats.err += 1;
            error!("Telemetry error: {:?}", e);
        }
    }
}

/// Brief motor spin at low throttle for direction/mode verification.
async fn brief_spin<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>, throttle: u16, duration_ms: u32)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!(
        "Spinning motor at throttle {} for {}ms...",
        throttle, duration_ms
    );
    let period = frame_period();
    let mut stats = TelemetryStats::default();

    for throttle in (0..=throttle).step_by(SPIN_STEP as usize) {
        for _ in 0..SPIN_SAMPLES_PER_STEP {
            send_telemetry_sample(esc, throttle, &mut stats).await;
            Timer::after(period).await;
        }
    }

    let hold_frames = ((duration_ms as u64 * 1_000) / period.as_micros()).max(1) as u32;
    for _ in 0..hold_frames {
        send_telemetry_sample(esc, throttle, &mut stats).await;
        Timer::after(period).await;
    }

    for throttle in (0..=throttle).rev().step_by(SPIN_STEP as usize) {
        for _ in 0..SPIN_SAMPLES_PER_STEP {
            send_telemetry_sample(esc, throttle, &mut stats).await;
            Timer::after(period).await;
        }
    }

    info!("Waiting for motor to stop...");
    send_stop_frames(esc, SPIN_RAMP_HOLD_FRAMES).await;

    info!(
        "Spin summary: {} successful telemetry frames, {} errors",
        stats.ok, stats.err
    );
}

/// Hold a fixed throttle for a short duration (no ramp).
async fn hold_throttle<T, D>(
    esc: &mut Stm32BidirDshotPort<'_, T, D>,
    throttle: u16,
    duration_ms: u32,
) where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!(
        "Holding throttle {} for {}ms (no ramp)...",
        throttle, duration_ms
    );
    let period = frame_period();
    let mut stats = TelemetryStats::default();
    let hold_frames = ((duration_ms as u64 * 1_000) / period.as_micros()).max(1) as u32;

    for _ in 0..hold_frames {
        send_telemetry_sample(esc, throttle, &mut stats).await;
        Timer::after(period).await;
    }

    info!("Waiting for motor to stop...");
    send_stop_frames(esc, SPIN_RAMP_HOLD_FRAMES).await;

    info!(
        "Hold summary: {} successful telemetry frames, {} errors",
        stats.ok, stats.err
    );
}

async fn phase_1_beep_test<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!("Phase 1: Beep Test (Commands 1-5)");
    let beeps = [
        CommandStep {
            cmd: Command::Beep1,
            label: "Sending Beep1...",
        },
        CommandStep {
            cmd: Command::Beep2,
            label: "Sending Beep2...",
        },
        CommandStep {
            cmd: Command::Beep3,
            label: "Sending Beep3...",
        },
        CommandStep {
            cmd: Command::Beep4,
            label: "Sending Beep4...",
        },
        CommandStep {
            cmd: Command::Beep5,
            label: "Sending Beep5...",
        },
    ];

    for step in beeps {
        info!("{}", step.label);
        execute_command(esc, step.cmd).await;
        //send_stop_frames(esc, STOP_FRAMES_AFTER_PHASE).await;
    }

    info!("Phase 1 complete — verify: 5 distinct beep tones heard");
}

async fn phase_2_spin_direction<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!("Phase 2: Spin Direction");
    arm_for(
        esc,
        ARM_IDLE_SECS,
        "Preparing for SpinDirectionNormal (1s idle)...",
    )
    .await;
    execute_command(esc, Command::SpinDirection1).await;
    info!("Saving settings...");
    execute_command(esc, Command::SettingsSave).await;
    info!("SSpinDirectionNormal ->>");
    brief_spin(esc, TEST_THROTTLE, 2000).await;

    arm_for(
        esc,
        ARM_IDLE_SECS,
        "Preparing for SpinDirectionReversed (1s idle)...",
    )
    .await;
    execute_command(esc, Command::SpinDirection2).await;
    info!("Saving settings...");
    execute_command(esc, Command::SettingsSave).await;
    info!("SpinDirectionReversed <<-");
    brief_spin(esc, TEST_THROTTLE, 2000).await;

    arm_for(
        esc,
        ARM_IDLE_SECS,
        "Restoring SpinDirectionNormal (1s idle)...",
    )
    .await;
    execute_command(esc, Command::SpinDirectionNormal).await;
    info!("Saving settings...");
    execute_command(esc, Command::SettingsSave).await;
    info!("Phase 2 complete — verify: motor spun both directions");
}

async fn phase_3_three_d_mode<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!("Phase 3: 3D Mode");
    arm_for(esc, ARM_IDLE_SECS, "Preparing for 3D Mode (1s idle)...").await;
    execute_command(esc, Command::ThreeDModeOn).await;
    info!("Saving settings...");
    execute_command(esc, Command::SettingsSave).await;
    info!("Gentle forward (positive) test in 3D mode...");
    hold_throttle(esc, 1100, THREED_TEST_MS).await;
    arm_for(
        esc,
        ARM_IDLE_SECS,
        "Preparing for gentle reverse 3D test (1s idle)...",
    )
    .await;
    info!("Gentle reverse (negative) test in 3D mode...");
    hold_throttle(esc, 996, THREED_TEST_MS).await;

    execute_command(esc, Command::MotorStop).await;
    arm_for(
        esc,
        ARM_IDLE_SECS,
        "Preparing for ThreeDModeOff (1s idle)...",
    )
    .await;
    execute_command(esc, Command::ThreeDModeOff).await;
    info!("Saving settings...");
    execute_command(esc, Command::SettingsSave).await;
    info!("Phase 3 complete — verify: 3D mode toggled (if ESC supports it)");
}

async fn phase_4_led_control<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!("Phase 4: LED Control");
    arm_for(esc, ARM_IDLE_SECS, "Preparing for LED Control (1s idle)...").await;
    let leds_on = [
        CommandStep {
            cmd: Command::Led0On,
            label: "Sending LED0 On...",
        },
        CommandStep {
            cmd: Command::Led1On,
            label: "Sending LED1 On...",
        },
        CommandStep {
            cmd: Command::Led2On,
            label: "Sending LED2 On...",
        },
        CommandStep {
            cmd: Command::Led3On,
            label: "Sending LED3 On...",
        },
    ];
    let leds_off = [
        CommandStep {
            cmd: Command::Led0Off,
            label: "Sending LED0 Off...",
        },
        CommandStep {
            cmd: Command::Led1Off,
            label: "Sending LED1 Off...",
        },
        CommandStep {
            cmd: Command::Led2Off,
            label: "Sending LED2 Off...",
        },
        CommandStep {
            cmd: Command::Led3Off,
            label: "Sending LED3 Off...",
        },
    ];
    run_command_sequence(esc, &leds_on).await;
    run_command_sequence(esc, &leds_off).await;
    info!("Phase 4 complete — verify: LEDs toggled (if ESC has them)");
}

async fn phase_5_extended_telemetry<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!("Phase 5: Extended Telemetry (EDT)");
    info!("Enabling ExtendedTelemetryEnable...");
    execute_command(esc, Command::ExtendedTelemetryEnable).await;
    info!(
        "Reading telemetry frames for 2 seconds at throttle {} (skipping eRPM)...",
        TEST_THROTTLE
    );
    let period = frame_period();
    let mut ticker = embassy_time::Ticker::every(period);

    let deadline = embassy_time::Instant::now() + Duration::from_secs(15);
    let mut frames = 0u32;
    while embassy_time::Instant::now() < deadline {
        match esc.send_throttle_and_receive(TEST_THROTTLE).await {
            Ok(telem) => {
                frames += 1;
                if !matches!(telem, TelemetryFrame::Erpm(_)) {
                    info!("Frame {}: {:?}", frames, telem);
                }
            }
            Err(e) => {
                error!("Telemetry error: {:?}", e);
            }
        }
        ticker.next().await;
    }

    info!("Disabling ExtendedTelemetryDisable...");
    execute_command(esc, Command::ExtendedTelemetryDisable).await;
    info!("Phase 5 complete");
}

async fn phase_6_audio_silent_mode<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!("Phase 6: Audio/Silent Mode");
    info!("Sending AudioStreamModeToggle...");
    execute_command(esc, Command::AudioStreamModeToggle).await;
    info!("Sending SilentModeToggle...");
    execute_command(esc, Command::SilentModeToggle).await;
    info!("Phase 6 complete");
}

async fn phase_7_signal_line_telemetry<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!("Phase 7: Signal Line Telemetry");
    info!("Enabling SignalLineTelemetryEnable...");
    execute_command(esc, Command::SignalLineTelemetryEnable).await;
    info!("Setting SignalLineContinuousERPMTelemetry...");
    execute_command(esc, Command::SignalLineContinuousERPMTelemetry).await;
    info!("Spinning motor and reading continuous telemetry...");
    brief_spin(esc, TEST_THROTTLE, 1000).await;
    info!("Disabling SignalLineTelemetryDisable...");
    execute_command(esc, Command::SignalLineTelemetryDisable).await;
    info!("Phase 7 complete");
}

async fn phase_8_esc_info<T, D>(esc: &mut Stm32BidirDshotPort<'_, T, D>)
where
    T: embassy_stm32::timer::GeneralInstance4Channel,
    D: uf_dshot::embassy_stm32::RawDmaChannel,
{
    info!("Phase 8: ESC Info");
    info!("Temporarily extending the receive window for ESC info capture...");
    esc.set_config(
        BidirDshotConfig::new(ESC_SPEED).with_rx_timeout(ESC_INFO_RX_TIMEOUT),
    );
    info!("Sending ESCInfo command...");
    match esc
        .send_frame_and_receive(DshotTx::bidirectional().command(Command::EscInfo))
        .await
    {
        Ok(telem) => info!("ESC info telemetry: {:?}", telem),
        Err(e) => error!("ESC info telemetry error: {:?}", e),
    }

    // Restore the default receive window for subsequent operations.
    esc.set_config(BidirDshotConfig::new(ESC_SPEED));
    info!("Phase 8 complete");
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());

    info!("ESC Command Validation Test (STM32)");

    let motor_pin = DshotPortPin::new(p.PA8);
    let mut esc = unwrap!(Stm32BidirDshotPort::new_ch1(
        p.TIM1, p.DMA2_CH3, DmaIrqs, motor_pin, ESC_SPEED,
    ));

    info!("Arming ESC (5s)...");
    unwrap!(esc.arm_for(Duration::from_secs(5)).await);
    info!("ESC armed");

    phase_1_beep_test(&mut esc).await;
    phase_2_spin_direction(&mut esc).await;
    //phase_3_three_d_mode(&mut esc).await;
    phase_4_led_control(&mut esc).await;
    phase_5_extended_telemetry(&mut esc).await;
    phase_6_audio_silent_mode(&mut esc).await;
    phase_7_signal_line_telemetry(&mut esc).await;
    phase_8_esc_info(&mut esc).await;

    info!("All phases complete");
    send_stop_frames(&mut esc, STOP_FRAMES_FINAL).await;
    info!("ESC Command Validation Test finished.");
    loop {
        Timer::after_secs(60).await;
    }
}
