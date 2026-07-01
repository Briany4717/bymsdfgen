fn load(path: &str) -> (u32, u32, usize, Vec<u8>) {
    let dec = png::Decoder::new(std::fs::File::open(path).unwrap());
    let mut r = dec.read_info().unwrap();
    let mut buf = vec![0; r.output_buffer_size()];
    let info = r.next_frame(&mut buf).unwrap();
    buf.truncate(info.buffer_size());
    let ch = match info.color_type {
        png::ColorType::Rgb => 3,
        png::ColorType::Rgba => 4,
        png::ColorType::Grayscale => 1,
        _ => 3,
    };
    (info.width, info.height, ch, buf)
}
fn med(a: u8, b: u8, c: u8) -> u8 {
    a.max(b).min(a.min(b).max(c))
}
fn main() {
    let a = std::env::args().nth(1).unwrap();
    let b = std::env::args().nth(2).unwrap();
    let (aw, ah, ac, ad) = load(&a);
    let (_, _, bc, bd) = load(&b);
    let mut maxd = 0u8;
    let mut sum = 0u64; // raw channel diff
    for i in 0..ad.len().min(bd.len()) {
        let d = ad[i].abs_diff(bd[i]);
        if d > maxd {
            maxd = d;
        }
        sum += d as u64;
    }
    // median diff (functional SDF)
    let n = (aw * ah) as usize;
    let mut mmax = 0u8;
    let mut msum = 0u64;
    for p in 0..n {
        let am = med(
            ad[p * ac],
            ad[p * ac + 1.min(ac - 1)],
            ad[p * ac + 2.min(ac - 1)],
        );
        let bm = med(
            bd[p * bc],
            bd[p * bc + 1.min(bc - 1)],
            bd[p * bc + 2.min(bc - 1)],
        );
        let d = am.abs_diff(bm);
        if d > mmax {
            mmax = d;
        }
        msum += d as u64;
    }
    println!(
        "{}: raw max={maxd} mean={:.2} | MEDIAN max={mmax} mean={:.3}",
        a.rsplit('/').next().unwrap(),
        sum as f64 / ad.len() as f64,
        msum as f64 / n as f64
    );
}
