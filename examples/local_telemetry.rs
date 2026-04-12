use uf_dshot::{parse_telemetry_payload, TelemetryFrame};

fn print_frame(label: &str, payload: u16) {
    match parse_telemetry_payload(payload) {
        Ok(TelemetryFrame::Erpm(erpm)) => {
            println!(
                "{label:<20} eRPM period={}us mech_rpm(pole_pairs=14)={}",
                erpm.period(),
                erpm.mechanical_rpm(14)
            );
        }
        Ok(TelemetryFrame::Temperature(c)) => {
            println!("{label:<20} temperature={}C", c);
        }
        Ok(TelemetryFrame::Voltage(v)) => {
            println!("{label:<20} voltage_raw={} (10mV/LSB)", v);
        }
        Ok(TelemetryFrame::Current(i)) => {
            println!("{label:<20} current_raw={} (100mA/LSB)", i);
        }
        Ok(other) => {
            println!("{label:<20} frame={other:?}");
        }
        Err(err) => {
            println!("{label:<20} error={err:?}");
        }
    }
}

fn main() {
    // Raw, already-encoded payload values (data+crc).
    let erpm_payload = 0x5A55;
    let temperature_payload = 0x2195;
    let voltage_payload = 0x42A3;
    let current_payload = 0x6559;

    print_frame("eRPM", erpm_payload);
    print_frame("temperature", temperature_payload);
    print_frame("voltage", voltage_payload);
    print_frame("current", current_payload);
}
