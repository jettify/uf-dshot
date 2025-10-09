use uf_dshot::DShotFrame;

fn main() {
    let frame = DShotFrame::new_throttle(1000, false);
    println!("{:?}", frame);
}
