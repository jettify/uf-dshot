use uf_dshot::{Command, DshotTx};

fn print_frame(label: &str, frame: uf_dshot::EncodedFrame) {
    println!(
        "{label:<38} payload=0x{:04X} data_12=0x{:03X} crc=0x{:X}",
        frame.payload,
        frame.data_12(),
        frame.crc_4()
    );
}

fn main() {
    // DShot Frame Encoder
    let uni_tx = DshotTx::standard();

    let uni_throttle = uni_tx.throttle(1000).expect("throttle in 0..=1999");
    print_frame("unidirectional throttle", uni_throttle);

    let uni_telem = uni_tx
        .with_telemetry_request(true)
        .throttle(1000)
        .expect("throttle in 0..=1999");
    print_frame("unidirectional throttle + telem", uni_telem);

    let stop = uni_tx.command(Command::MotorStop);
    print_frame("unidirectional command MotorStop", stop);

    let stop = uni_tx.command(Command::Beep1);
    print_frame("unidirectional command Beep1", stop);

    // Bidirectional DShot Frame Encoder
    let bidir_tx = DshotTx::bidirectional();
    let bidir_throttle = bidir_tx.throttle(1000).expect("throttle in 0..=1999");
    print_frame("bidirdirectional throttle", bidir_throttle);

    let bidir_telem = bidir_tx
        .with_telemetry_request(true)
        .throttle(1000)
        .expect("throttle in 0..=1999");
    print_frame("bidirdirectional throttle + telem", bidir_telem);

    let stop = bidir_tx.command(Command::MotorStop);
    print_frame("bidirdirectional command MotorStop", stop);

    let stop = bidir_tx.command(Command::Beep1);
    print_frame("bidirdirectional command Beep1", stop);
}
