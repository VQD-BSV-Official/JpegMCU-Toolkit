// mcudecode.rs - port của MCUDecode.py (GPL v2+), chỉ dùng std
// Build: rustc -C opt-level=3 -C target-cpu=native mcudecode.rs

use std::collections::HashMap;
use std::fmt::Write as _;
use std::path::Path;
use std::time::Instant;

const USAGE: &str = "Usage: mcudecode image.jpg [--adv]

Writes Log_MCUs.txt (main image) and Log_MCUs_Thumb.txt (EXIF thumbnail, if any) next to the image.
Offsets are <byte>.<bit> from the start of the file (bit 0 = MSB).
--adv: keep decoding MCUs until FFD9, ignoring the header size.";

const INF: usize = 1 << 60;
type R<T> = Result<T, String>;
type Tables = HashMap<(u8, u8), Vec<u16>>;

struct Frame {
    w: usize,
    h: usize,
    comps: HashMap<u8, (u8, u8)>, // cid -> (h, v)
}

struct Blk<'a> {
    ld: &'a [u16],
    la: &'a [u16],
    lab: char,
    td: u8,
    ta: u8,
    bi: usize,
}

#[derive(PartialEq)]
enum St {
    End,
    Eod,
    Done,
}

/// LUT 16 bit. DC: codelen+size. AC: (codelen+size)<<8 | inc (0 = EOB). 0 = mã sai.
fn build_lut(counts: &[u8], symbols: &[u8], is_dc: bool) -> Vec<u16> {
    let mut lut = vec![0u16; 65536];
    let (mut code, mut k) = (0usize, 0usize);
    for l in 1..=16usize {
        for _ in 0..counts[l - 1] {
            if k >= symbols.len() || (code >> l) != 0 {
                return lut;
            }
            let sym = symbols[k];
            k += 1;
            let (lo, hi) = (code << (16 - l), (code + 1) << (16 - l));
            if is_dc {
                if sym <= 15 {
                    lut[lo..hi].fill((l + sym as usize) as u16);
                }
            } else {
                let (run, size) = (sym >> 4, (sym & 15) as usize);
                let inc = if size == 0 { if run == 15 { 16 } else { 0 } } else { run as u16 + 1 };
                lut[lo..hi].fill((((l + size) << 8) as u16) | inc);
            }
            code += 1;
        }
        code <<= 1;
    }
    lut
}

fn tail(u: &[u8], pos: usize) -> u32 {
    u.get(pos..).unwrap_or(&[]).iter().take(4).fold(0, |a, &b| a << 8 | b as u32)
}

fn decode_interval(
    data: &[u8], p: usize, q: usize, n0: usize, cnt: usize, mcux: usize,
    blocks: &[Blk], out: &mut String, stop_at_end: bool,
) -> (St, usize, i64) {
    let seg: &[u8] = if p < q { &data[p..q] } else { &[] };
    // bỏ byte stuffing; s = vị trí (trong u) của các FF đã stuffing
    let mut u = Vec::with_capacity(seg.len() + 512);
    let mut s: Vec<usize> = Vec::new();
    let mut i = 0;
    while i < seg.len() {
        u.push(seg[i]);
        if seg[i] == 0xFF && seg.get(i + 1) == Some(&0) {
            s.push(u.len() - 1);
            i += 1;
        }
        i += 1;
    }
    let limit = (u.len() << 3) as i64;
    u.resize(u.len() + 512, 0); // đệm tránh kiểm tra biên ở vòng nóng

    let (mut acc, mut nbits, mut pos) = (0u64, 0usize, 0usize);
    macro_rules! refill {
        () => {
            if nbits < 32 {
                let w = if pos + 4 <= u.len() {
                    u32::from_be_bytes([u[pos], u[pos + 1], u[pos + 2], u[pos + 3]])
                } else {
                    tail(&u, pos)
                };
                acc = ((acc & ((1u64 << nbits) - 1)) << 32) | w as u64;
                pos += 4;
                nbits += 32;
            }
        };
    }
    macro_rules! peek {
        () => {
            ((acc >> (nbits - 16)) & 0xFFFF) as usize
        };
    }
    let off = |bp: usize, s: &[usize]| {
        let ub = bp >> 3;
        format!("{}.{}", p + ub + s.partition_point(|&x| x < ub), bp & 7)
    };

    for n in n0..n0 + cnt {
        let bp = pos * 8 - nbits;
        if stop_at_end && limit - (bp as i64) < 8 {
            return (St::End, n, limit - bp as i64);
        }
        let _ = writeln!(out, "MCU {} ({},{}) ({})", n, n % mcux, n / mcux, off(bp, &s));

        for b in blocks {
            let (bs_pos, bs_n) = (pos, nbits);
            let mut err = 0u8;

            refill!();
            let e = b.ld[peek!()] as usize;
            if e != 0 {
                nbits -= e;
                let mut k = 1;
                while k < 64 {
                    refill!();
                    let e = b.la[peek!()] as usize;
                    if e == 0 {
                        nbits -= 16;
                        err = 2;
                        break;
                    }
                    nbits -= e >> 8;
                    let inc = e & 255;
                    if inc == 0 {
                        break;
                    }
                    k += inc;
                    if k > 64 {
                        err = 3;
                        break;
                    }
                }
            } else {
                nbits -= 16;
                err = 1;
            }

            let over = ((pos * 8 - nbits) as i64) > limit;
            if over || err != 0 {
                let o = off(bs_pos * 8 - bs_n, &s);
                if over {
                    let _ = writeln!(out, "*** [{}@{}] End of data -> stop", b.lab, o);
                    return (St::Eod, n + 1, 0);
                }
                let _ = match err {
                    1 => writeln!(out, "*** [{}@{}] Huffman error -> DC:{} #{}", b.lab, o, b.td, b.bi),
                    2 => writeln!(out, "*** [{}@{}] Huffman error -> AC:{} #{}", b.lab, o, b.ta, b.bi),
                    _ => writeln!(out, "*** [{}@{}] Coefficient overflow -> AC:{} #{}", b.lab, o, b.ta, b.bi),
                };
            }
        }
    }
    (St::Done, n0 + cnt, limit - (pos * 8 - nbits) as i64)
}

/// Tìm "FF xx" (xx thoả pred) trong data[from..to]
fn find_marker(data: &[u8], from: usize, to: usize, pred: impl Fn(u8) -> bool) -> Option<usize> {
    let to = to.min(data.len());
    let mut i = from;
    while i + 1 < to {
        if data[i] == 0xFF && pred(data[i + 1]) {
            return Some(i);
        }
        i += 1;
    }
    None
}

fn decode_scan(
    data: &[u8], a: usize, end: usize, frame: &Frame, scan: &[(u8, u8, u8, u8, u8)],
    tables: &Tables, dri: usize, log_path: &Path, adv: bool,
) -> R<()> {
    if scan.is_empty() || a > end {
        return Err("Invalid SOS".into());
    }
    let hmax = frame.comps.values().map(|c| c.0).max().unwrap_or(1) as usize;
    let vmax = frame.comps.values().map(|c| c.1).max().unwrap_or(1) as usize;

    let mut blocks: Vec<Blk> = Vec::new();
    for (ci, &(cid, h, v, td, ta)) in scan.iter().enumerate() {
        let (Some(ld), Some(la)) = (tables.get(&(0, td)), tables.get(&(1, ta))) else {
            return Err(format!("Missing Huffman table for component {cid}"));
        };
        let nb = if scan.len() == 1 { 1 } else { h as usize * v as usize };
        for _ in 0..nb {
            let bi = blocks.len();
            blocks.push(Blk { ld, la, lab: if ci == 0 { 'Y' } else { 'C' }, td, ta, bi });
        }
    }

    let (w, h) = (frame.w, frame.h);
    let (mcu_w, mcu_h, mcux, mcuy);
    if scan.len() == 1 {
        let (_, sh, sv, _, _) = scan[0];
        let cw = (w * sh as usize).div_ceil(hmax);
        let ch = (h * sv as usize).div_ceil(vmax);
        mcu_w = 8;
        mcu_h = 8;
        mcux = cw.div_ceil(8);
        mcuy = ch.div_ceil(8);
    } else {
        mcu_w = 8 * hmax;
        mcu_h = 8 * vmax;
        mcux = w.div_ceil(mcu_w);
        mcuy = h.div_ceil(mcu_h);
    }
    let total = mcux * mcuy;

    let mut out = String::with_capacity(total * 24 + 256);
    let _ = writeln!(out, "// {}x{} {} components:", w, h, scan.len());
    for &(cid, h, v, td, _) in scan {
        let _ = writeln!(out, "// ID:{cid} [{h}{v}] Dest:{td}");
    }
    let lay: String = blocks.iter().map(|b| b.lab).collect();
    let _ = writeln!(out, "// MCU: {lay} ({mcu_w}x{mcu_h} pixel)");
    let _ = writeln!(out, "//[{mcux}x{mcuy}={total} MCU]\n");

    // vùng entropy: từ sau SOS đến EOI cuối cùng
    let scan_end = data[a..end].windows(2).rposition(|w| w == [0xFF, 0xD9]).map_or(end, |x| x + a);

    let total_eff = if adv { INF } else { total };
    let ivl = if dri != 0 { dri } else { INF };
    let (mut n, mut p, mut k) = (0usize, a, 0usize);
    while n < total_eff {
        let mut cnt = ivl.min(total_eff - n);
        let mut q = scan_end;
        let mut found = false;
        if dri != 0 {
            let rk = 0xD0 + (k & 7) as u8;
            let mut f = find_marker(data, p, scan_end, |b| b == rk);
            if f.is_none() {
                f = find_marker(data, p, scan_end, |b| (0xD0..=0xD7).contains(&b));
                let _ = writeln!(out, "*** RST marker FF{rk:02X} not found after {p}");
            }
            if let Some(x) = f {
                q = x;
                found = true;
            }
        }
        let last = !found;
        if adv && last {
            cnt = INF;
        }
        let (status, nn, rem) = decode_interval(data, p, q, n, cnt, mcux, &blocks, &mut out, adv && last);
        if last {
            let mk = if data.get(q..q + 2) == Some(&[0xFF, 0xD9][..]) { "FFD9" } else { "end of data" };
            if adv {
                let _ = writeln!(out, "// END: {mk} at {q} - {nn} MCU (header: {total} MCU)");
            } else if status == St::Done && rem >= 8 {
                let _ = writeln!(out, "// NOTE: còn {} byte trước {mk} (offset {q}) sau {total} MCU - dùng --adv", rem >> 3);
            }
            break;
        }
        n += cnt;
        p = q + 2;
        k += 1;
    }
    std::fs::write(log_path, out).map_err(|e| e.to_string())
}

fn parse_jpeg(
    data: &[u8], start: usize, end: usize, log: &Path, find_thumb: bool, adv: bool,
) -> R<Option<(usize, usize)>> {
    if data.get(start..start + 2) != Some(&[0xFF, 0xD8][..]) {
        return Err("Missing SOI (FFD8)".into());
    }
    let mut i = start + 2;
    let mut tables = Tables::new();
    let (mut frame, mut dri, mut thumb): (Option<Frame>, usize, Option<(usize, usize)>) = (None, 0, None);

    while i + 4 <= end {
        if data[i] != 0xFF {
            i += 1;
            continue;
        }
        let m = data[i + 1];
        if m == 0xFF {
            i += 1;
            continue;
        }
        if m <= 1 || (0xD0..=0xD8).contains(&m) {
            i += 2;
            continue;
        }
        if m == 0xD9 {
            break;
        }
        let ln = (data[i + 2] as usize) << 8 | data[i + 3] as usize;
        let se = (i + 2 + ln).min(data.len());
        let seg = &data[(i + 4).min(se)..se];
        let bad = || format!("Truncated segment FF{m:02X}");

        if m == 0xE1 && find_thumb && thumb.is_none() && seg.starts_with(b"Exif\0\0") {
            if let Some(idx) = seg[6..].windows(3).position(|w| w == [0xFF, 0xD8, 0xFF]) {
                thumb = Some((i + 4 + 6 + idx, se));
            }
        } else if m == 0xC4 {
            let mut p = 0;
            while p + 17 <= seg.len() {
                let (tc, th) = (seg[p] >> 4, seg[p] & 15);
                let counts = &seg[p + 1..p + 17];
                let n: usize = counts.iter().map(|&c| c as usize).sum();
                let syms = &seg[(p + 17).min(seg.len())..(p + 17 + n).min(seg.len())];
                let is_dc = tc == 0;
                tables.insert((!is_dc as u8, th), build_lut(counts, syms, is_dc));
                p += 17 + n;
            }
        } else if m == 0xC0 || m == 0xC1 {
            if seg.len() < 6 || seg.len() < 6 + 3 * seg[5] as usize {
                return Err(bad());
            }
            let (h, w) = ((seg[1] as usize) << 8 | seg[2] as usize, (seg[3] as usize) << 8 | seg[4] as usize);
            let mut comps = HashMap::new();
            for k in 0..seg[5] as usize {
                let (cid, hv) = (seg[6 + 3 * k], seg[7 + 3 * k]);
                if hv >> 4 == 0 || hv & 15 == 0 {
                    return Err("Invalid sampling factor".into());
                }
                comps.insert(cid, (hv >> 4, hv & 15));
            }
            if w == 0 || h == 0 || comps.is_empty() {
                return Err("Invalid SOF".into());
            }
            frame = Some(Frame { w, h, comps });
        } else if m == 0xC2 || m == 0xC3 || ((0xC5..=0xCF).contains(&m) && m != 0xC8 && m != 0xCC) {
            return Err(format!("SOF 0xFF{m:02X} not supported (baseline/extended sequential Huffman only)"));
        } else if m == 0xDD {
            if seg.len() < 2 {
                return Err(bad());
            }
            dri = (seg[0] as usize) << 8 | seg[1] as usize;
        } else if m == 0xDA {
            let Some(fr) = &frame else { return Err("SOS before SOF".into()) };
            if seg.is_empty() || seg.len() < 1 + 2 * seg[0] as usize {
                return Err(bad());
            }
            let mut scan = Vec::new();
            for k in 0..seg[0] as usize {
                let (cs, t) = (seg[1 + 2 * k], seg[2 + 2 * k]);
                let &(h, v) = fr.comps.get(&cs).ok_or(format!("Unknown component {cs} in SOS"))?;
                scan.push((cs, h, v, t >> 4, t & 15));
            }
            decode_scan(data, i + 2 + ln, end, fr, &scan, &tables, dri, log, adv)?;
            return Ok(thumb);
        }
        i += 2 + ln;
    }
    Err("SOS not found".into())
}

fn main() {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let adv = argv.iter().any(|a| a == "--adv");
    let Some(path) = argv.iter().find(|a| *a != "--adv") else {
        eprintln!("{USAGE}");
        std::process::exit(1);
    };
    let run = || -> R<String> {
        let data = std::fs::read(path).map_err(|e| e.to_string())?;
        let d = std::fs::canonicalize(path).map_err(|e| e.to_string())?;
        let d = d.parent().unwrap_or(Path::new("."));
        let (log, tlog) = (d.join("Log_MCUs.txt"), d.join("Log_MCUs_Thumb.txt"));
        let t0 = Instant::now();
        let thumb = parse_jpeg(&data, 0, data.len(), &log, true, adv)?;
        if let Some((s, e)) = thumb {
            parse_jpeg(&data, s, e, &tlog, false, adv)?;
        }
        let names = if thumb.is_some() { format!("{}, {}", log.display(), tlog.display()) } else { log.display().to_string() };
        Ok(format!("Wrote {names} ({:.2}s)", t0.elapsed().as_secs_f64()))
    };
    match run() {
        Ok(s) => println!("{s}"),
        Err(e) => {
            eprintln!("Error: {e}");
            std::process::exit(1);
        }
    }
}
