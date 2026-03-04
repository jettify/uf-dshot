use uf_dshot::{decode_telemetry_from_samples, encode_gcr, parse_telemetry_payload, DShotFrame, Frame};

fn main() {
    let frame = DShotFrame::new_throttle(1000, true);
    println!("tx payload: {:#06x}", frame.to_payload());

    let payload = 0x8109u16;
    let gcr = encode_gcr(payload);
    println!("sample telemetry gcr: {:#023b}", gcr);

    // This buffer is intentionally empty and demonstrates error handling path.
    let empty = [0u16; 80];
    println!(
        "decode result: {:?}",
        decode_telemetry_from_samples(&empty, 0, 0)
    );

    println!("direct telemetry parse: {:?}", parse_telemetry_payload(payload));
}
