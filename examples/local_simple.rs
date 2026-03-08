use uf_dshot::UniTx;

fn main() {
    let frame = UniTx::throttle(1000)
        .expect("throttle within 0..=1999")
        .with_telemetry_request(true)
        .encode();

    println!(
        "payload=0x{:04X}, data_12=0x{:03X}, crc_4=0x{:X}",
        frame.payload,
        frame.data_12(),
        frame.crc_4()
    );
}
