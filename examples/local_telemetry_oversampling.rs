use uf_dshot::{BidirDecoder, OversamplingConfig, TelemetryFrame};

const RUNS_5A55: [usize; 12] = [1, 2, 2, 2, 2, 2, 2, 2, 1, 2, 2, 1];

fn build_oversampled_samples(
    sample_bit_index: u8,
    preamble_high_samples: usize,
    run_bits: &[usize],
    oversampling: usize,
) -> [u16; 128] {
    let mut out = [0u16; 128];
    let mask = 1u16 << sample_bit_index;
    let mut write = 0usize;

    while write < preamble_high_samples && write < out.len() {
        out[write] = mask;
        write += 1;
    }

    // ESC line transitions between low/high runs; each run length is represented
    // by an oversampled pulse width.
    let mut is_high = false;
    for &bit_len in run_bits {
        let width = (bit_len * oversampling).saturating_sub(1).max(1);
        let level = if is_high { mask } else { 0 };
        let mut i = 0usize;
        while i < width && write < out.len() {
            out[write] = level;
            write += 1;
            i += 1;
        }
        is_high = !is_high;
    }

    out
}

fn main() {
    let cfg = OversamplingConfig {
        sample_bit_index: 0,
        oversampling: 4,
        frame_bits: 21,
        min_detected_bits: 18,
        bit_tolerance: 2,
    };
    let samples = build_oversampled_samples(0, 8, &RUNS_5A55, cfg.oversampling as usize);

    let decoder = BidirDecoder::new(cfg);
    match decoder.decode_frame(&samples) {
        Ok(TelemetryFrame::Erpm(erpm)) => {
            println!(
                "decoded eRPM period={}us mech_rpm(pole_pairs=14)={}",
                erpm.period(),
                erpm.mechanical_rpm(14)
            );
        }
        Ok(other) => println!("decoded frame={other:?}"),
        Err(err) => println!("decode error={err:?}"),
    }
}
