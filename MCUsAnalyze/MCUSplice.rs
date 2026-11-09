// mcusplice.rs - port của MCUSplice.py (GPL v2+), chỉ dùng std
// Build: rustc -C opt-level=3 -C strip=symbols -C panic=abort -C lto=fat -C codegen-units=1 mcusplice.rs
// ins/del không kèm màu cần Log_MCUs.txt (tạo bằng mcudecode) cạnh file ảnh; thiếu/không khớp thì báo lỗi.

use std::collections::HashMap;
use std::path::Path;

type R<T> = Result<T, String>;
const EOB_SEG: u32 = u32::MAX; // Blk.seg đánh dấu AC = EOB

const USAGE: &str = "usage: mcusplice in.jpg [out.jpg] [{ins|del}N][@x,y][@x2,y2] [yV] [cbV] [crV] [yccY,Cb,Cr]";

// ----------------------------------------------------------------- Huffman
struct Huff {
    enc: [(u16, u8); 256], // sym -> (code, len); len 0 = không có
    lk: Vec<u16>,          // 16 bit -> (len<<8)|sym; 0 = mã sai
}

fn build_huff(counts: &[u8], symbols: &[u8]) -> R<Huff> {
    let mut h = Huff { enc: [(0, 0); 256], lk: vec![0; 65536] };
    let (mut code, mut k) = (0usize, 0usize);
    for ln in 1..=16usize {
        for _ in 0..counts[ln - 1] {
            let sym = *symbols.get(k).ok_or("Invalid DHT")?;
            k += 1;
            let (start, span) = (code << (16 - ln), 1usize << (16 - ln));
            if start + span > 65536 {
                return Err("Invalid DHT".into());
            }
            h.enc[sym as usize] = (code as u16, ln as u8);
            h.lk[start..start + span].fill(((ln as u16) << 8) | sym as u16);
            code += 1;
        }
        code <<= 1;
    }
    Ok(h)
}

// ------------------------------------------------------------------ parse
struct Parsed {
    w: usize,
    h: usize,
    comps: Vec<(u8, u8, u8, u8)>, // cid, h, v, tq
    huff: HashMap<(u8, u8), Huff>,
    dri: usize,
    sos: Vec<(u8, u8, u8)>, // cid, td, ta
    scan_start: usize,
    q0: HashMap<u8, i32>,
}

fn parse_jpeg(data: &[u8]) -> R<Parsed> {
    if data.get(..2) != Some(&[0xFF, 0xD8][..]) {
        return Err("Missing SOI (FFD8)".into());
    }
    let (mut pos, mut dri, mut sof) = (2usize, 0usize, None);
    let (mut huff, mut q0) = (HashMap::new(), HashMap::new());
    loop {
        if pos + 4 > data.len() || data[pos] != 0xFF {
            return Err(format!("Bad marker at offset {pos}"));
        }
        let m = data[pos + 1];
        if m == 0xFF {
            pos += 1;
            continue;
        }
        if m == 0xD8 || m == 1 || (0xD0..=0xD7).contains(&m) {
            pos += 2;
            continue;
        }
        if m == 0xD9 {
            return Err("SOS (FFDA) not found".into());
        }
        let ln = (data[pos + 2] as usize) << 8 | data[pos + 3] as usize;
        let se = (pos + 2 + ln).min(data.len());
        let seg = &data[(pos + 4).min(se)..se];
        let bad = || format!("Truncated segment FF{m:02X}");

        if matches!(m, 0xC2 | 0xC3 | 0xC5 | 0xC6 | 0xC7 | 0xC9 | 0xCA | 0xCB | 0xCD | 0xCE | 0xCF) {
            return Err("Only baseline/extended Huffman JPEG is supported".into());
        }
        match m {
            0xC0 | 0xC1 => {
                if seg.len() < 6 || seg.len() < 6 + 3 * seg[5] as usize {
                    return Err(bad());
                }
                let h = (seg[1] as usize) << 8 | seg[2] as usize;
                let w = (seg[3] as usize) << 8 | seg[4] as usize;
                let comps: Vec<_> = (0..seg[5] as usize)
                    .map(|i| (seg[6 + 3 * i], seg[7 + 3 * i] >> 4, seg[7 + 3 * i] & 15, seg[8 + 3 * i]))
                    .collect();
                if comps.is_empty() || comps.iter().any(|c| c.1 == 0 || c.2 == 0) {
                    return Err("Invalid SOF".into());
                }
                sof = Some((w, h, comps));
            }
            0xDB => {
                let mut i = 0;
                while i < seg.len() {
                    let (pq, tq) = ((seg[i] >> 4) as usize, seg[i] & 15);
                    if i + 1 + pq >= seg.len() {
                        return Err(bad());
                    }
                    let v = if pq == 0 { seg[i + 1] as i32 } else { (seg[i + 1] as i32) << 8 | seg[i + 2] as i32 };
                    q0.insert(tq, v);
                    i += 1 + 64 * (pq + 1);
                }
            }
            0xC4 => {
                let mut i = 0;
                while i < seg.len() {
                    if i + 17 > seg.len() {
                        return Err("Invalid DHT".into());
                    }
                    let (cls, tid) = (seg[i] >> 4, seg[i] & 15);
                    let counts = &seg[i + 1..i + 17];
                    let n: usize = counts.iter().map(|&c| c as usize).sum();
                    if i + 17 + n > seg.len() {
                        return Err("Invalid DHT".into());
                    }
                    huff.insert((cls, tid), build_huff(counts, &seg[i + 17..i + 17 + n])?);
                    i += 17 + n;
                }
            }
            0xDD => {
                if seg.len() < 2 {
                    return Err(bad());
                }
                dri = (seg[0] as usize) << 8 | seg[1] as usize;
            }
            0xDA => {
                let Some((w, h, comps)) = sof else { return Err("SOS before SOF".into()) };
                if seg.is_empty() || seg.len() < 1 + 2 * seg[0] as usize {
                    return Err(bad());
                }
                let sos = (0..seg[0] as usize).map(|j| (seg[1 + 2 * j], seg[2 + 2 * j] >> 4, seg[2 + 2 * j] & 15)).collect();
                return Ok(Parsed { w, h, comps, huff, dri, sos, scan_start: pos + 2 + ln, q0 });
            }
            _ => {}
        }
        pos += 2 + ln;
    }
}

// ----------------------------------------------------------------- bits
fn load32(d: &[u8], i: usize) -> u32 {
    if i + 4 <= d.len() {
        u32::from_be_bytes([d[i], d[i + 1], d[i + 2], d[i + 3]])
    } else {
        (0..4).fold(0, |a, k| a << 8 | *d.get(i + k).unwrap_or(&0xFF) as u32)
    }
}
#[inline(always)]
fn peek16(d: &[u8], pos: usize) -> usize {
    ((load32(d, pos >> 3) << (pos & 7)) >> 16) as usize
}
/// 1 <= n <= 25
#[inline(always)]
fn getbits(d: &[u8], pos: usize, n: u32) -> u32 {
    (load32(d, pos >> 3) << (pos & 7)) >> (32 - n)
}

fn destuff(buf: &[u8]) -> Vec<u8> {
    let mut d = Vec::with_capacity(buf.len() + 4);
    let mut i = 0;
    while i < buf.len() {
        d.push(buf[i]);
        if buf[i] == 0xFF && buf.get(i + 1) == Some(&0) {
            i += 1;
        }
        i += 1;
    }
    d.extend_from_slice(&[0xFF; 4]); // đệm
    d
}

struct BitWriter {
    buf: Vec<u8>,
    acc: u64,
    n: u32,
}
impl BitWriter {
    fn new() -> Self {
        BitWriter { buf: Vec::new(), acc: 0, n: 0 }
    }
    /// n <= 32, v < 2^n
    fn put(&mut self, v: u32, n: u32) {
        if n == 0 {
            return;
        }
        self.acc = (self.acc << n) | v as u64;
        self.n += n;
        while self.n >= 8 {
            self.n -= 8;
            self.buf.push((self.acc >> self.n) as u8);
        }
        self.acc &= (1u64 << self.n) - 1;
    }
    fn put_range(&mut self, src: &[u8], a: usize, b: usize) {
        let mut p = a;
        while p < b {
            let t = (b - p).min(24);
            self.put(getbits(src, p, t as u32), t as u32);
            p += t;
        }
    }
    /// pad bit 1, thêm byte stuffing, ghi vào out
    fn finish(mut self, out: &mut Vec<u8>) {
        if self.n != 0 {
            let pad = 8 - self.n;
            self.put((1 << pad) - 1, pad);
        }
        for &b in &self.buf {
            out.push(b);
            if b == 0xFF {
                out.push(0);
            }
        }
    }
}

// ----------------------------------------------------------------- decode
#[derive(Clone, Copy)]
struct Blk {
    dc: i32, // DC tuyệt đối
    seg: u32,
    a: u32, // bit đầu AC trong ds[seg]
    len: u16,
}

#[derive(Clone, Copy)]
struct Chunk {
    seg: u32,
    a: usize,
    len: usize,
}

/// Giải mã nmcu MCU, giữ phần đã giải mã được khi gặp lỗi.
fn decode_segment(
    d: &[u8], nmcu: usize, layout: &[usize], dc: &[&Huff], ac: &[&Huff],
    out: &mut Vec<Vec<Blk>>, seg: u32, base: usize,
) -> R<()> {
    let limit = (d.len() - 4) * 8;
    let mut pos = 0usize;
    let mut preds = vec![0i32; dc.len()];
    for mi in 0..nmcu {
        let mut mcu = Vec::with_capacity(layout.len());
        for &c in layout {
            let e = dc[c].lk[peek16(d, pos)];
            let s = (e & 255) as u32;
            if e == 0 || s > 16 {
                return Err(format!("bad DC code at MCU #{}", base + mi));
            }
            pos += (e >> 8) as usize;
            if s != 0 {
                let mut b = getbits(d, pos, s) as i32;
                pos += s as usize;
                if b < 1 << (s - 1) {
                    b -= (1 << s) - 1;
                }
                preds[c] = preds[c].wrapping_add(b);
            }
            let a = pos;
            let mut k = 1;
            while k < 64 {
                let e = ac[c].lk[peek16(d, pos)];
                if e == 0 {
                    return Err(format!("bad AC code at MCU #{}", base + mi));
                }
                pos += (e >> 8) as usize;
                let rs = (e & 255) as usize;
                let s = rs & 15;
                if s == 0 {
                    if rs == 0xF0 {
                        k += 16;
                        continue;
                    }
                    break;
                }
                pos += s;
                k += (rs >> 4) + 1;
            }
            mcu.push(Blk { dc: preds[c], seg, a: a as u32, len: (pos - a).min(65535) as u16 });
        }
        if pos > limit {
            return Err(format!("scan truncated at MCU #{}", base + mi));
        }
        out.push(mcu);
    }
    Ok(())
}

// ----------------------------------------------------------------- encode
fn put_dc(w: &mut BitWriter, enc: &[(u16, u8); 256], diff: i32) -> R<()> {
    let s = 32 - diff.unsigned_abs().leading_zeros();
    let (code, ln) = enc.get(s as usize).copied().filter(|e| e.1 != 0 && s <= 16).ok_or_else(|| {
        format!("DC Huffman table has no code for category {s}; cannot re-encode DC")
    })?;
    w.put(code as u32, ln as u32);
    if s != 0 {
        let v = if diff > 0 { diff } else { diff + (1 << s) - 1 };
        w.put(v as u32, s);
    }
    Ok(())
}

fn rebuild_scan(
    mcus: &[Option<Vec<Blk>>], ds: &[Vec<u8>], ri: usize, layout: &[usize], nc: usize,
    dc_h: &[&Huff], eob: &[(u32, u32)],
) -> R<Vec<u8>> {
    let total = mcus.len();
    let seg_size = if ri != 0 { ri } else { total };
    let nseg = total.div_ceil(seg_size);
    let mut out = Vec::new();
    for si in 0..nseg {
        let mut w = BitWriter::new();
        let mut preds = vec![0i32; nc];
        for mcu in &mcus[si * seg_size..((si + 1) * seg_size).min(total)] {
            for (bi, &c) in layout.iter().enumerate() {
                match mcu {
                    None => {
                        put_dc(&mut w, &dc_h[c].enc, 0)?;
                        w.put(eob[c].0, eob[c].1);
                    }
                    Some(blks) => {
                        let b = blks[bi];
                        put_dc(&mut w, &dc_h[c].enc, b.dc.wrapping_sub(preds[c]))?;
                        preds[c] = b.dc;
                        if b.seg == EOB_SEG {
                            w.put(eob[c].0, eob[c].1);
                        } else {
                            w.put_range(&ds[b.seg as usize], b.a as usize, b.a as usize + b.len as usize);
                        }
                    }
                }
            }
        }
        w.finish(&mut out);
        if si != nseg - 1 {
            out.extend_from_slice(&[0xFF, 0xD0 + (si & 7) as u8]);
        }
    }
    Ok(out)
}

// -------------------------------------------------------- log-based splice
fn parse_log_line(l: &str) -> Option<(usize, usize)> {
    // "MCU n (x,y) (byte.bit)" -> (n, bit offset)
    let r = l.strip_prefix("MCU ")?;
    let (n, r) = r.split_once(' ')?;
    let (_, r) = r.strip_prefix('(')?.split_once(") (")?;
    let (byte, r) = r.split_once('.')?;
    let bit = r.chars().next()?.to_digit(10)? as usize;
    Some((n.parse().ok()?, byte.parse::<usize>().ok()? * 8 + bit))
}

fn read_log(path: &Path, hdr: &str, first: usize) -> Option<HashMap<usize, usize>> {
    let txt = std::fs::read_to_string(path).ok()?;
    let offs: HashMap<usize, usize> = txt.lines().filter_map(parse_log_line).collect();
    (txt.contains(hdr) && offs.get(&0) == Some(&first)).then_some(offs)
}

fn get_offs(src: &str, mx: usize, my: usize, total: usize, scan_start: usize) -> R<HashMap<usize, usize>> {
    let dir = std::fs::canonicalize(src).ok().and_then(|p| p.parent().map(Path::to_path_buf)).unwrap_or(".".into());
    let hdr = format!("//[{mx}x{my}={total} MCU]");
    read_log(&dir.join("Log_MCUs.txt"), &hdr, scan_start * 8)
        .ok_or_else(|| "Log_MCUs.txt missing or does not match the file (run MCUDecode first)".into())
}

/// Bit ngay sau MCU bắt đầu tại pos
fn mcu_end(d: &[u8], mut pos: usize, layout: &[usize], dc: &[&Huff], ac: &[&Huff]) -> R<usize> {
    for &c in layout {
        let e = dc[c].lk[peek16(d, pos)];
        if e == 0 {
            return Err("DC".into());
        }
        pos += (e >> 8) as usize + (e & 255) as usize;
        let mut k = 1;
        while k < 64 {
            let e = ac[c].lk[peek16(d, pos)];
            if e == 0 {
                return Err("AC".into());
            }
            pos += (e >> 8) as usize;
            let rs = (e & 255) as usize;
            if rs & 15 != 0 {
                pos += rs & 15;
                k += (rs >> 4) + 1;
            } else if rs == 0xF0 {
                k += 16;
            } else {
                break;
            }
        }
    }
    Ok(pos)
}

/// Mỗi MCU -> Chunk cắt từ scan đã bỏ stuffing; không có trong log / không cắt được -> None.
fn to_chunks(
    parts: &[&[u8]], ds: &[Vec<u8>], ps: &[usize], seg: usize, total: usize,
    offs: &HashMap<usize, usize>, layout: &[usize], dc: &[&Huff], ac: &[&Huff],
) -> Vec<Option<Chunk>> {
    let mut mcus = Vec::with_capacity(total);
    for (si, buf) in parts.iter().enumerate() {
        let d = &ds[si];
        let limit = (d.len() - 4) * 8;
        // vị trí FF00 -> đổi offset file sang offset đã bỏ stuffing
        let mut f = Vec::new();
        let mut i = 0;
        while i + 1 < buf.len() {
            if buf[i] == 0xFF && buf[i + 1] == 0 {
                f.push(i);
                i += 2;
            } else {
                i += 1;
            }
        }
        let (lo, hi) = (si * seg, ((si + 1) * seg).min(total));
        let mut st: Vec<Option<usize>> = (lo..hi)
            .map(|m| {
                let o = *offs.get(&m)?;
                let rel = (o >> 3) as i64 - ps[si] as i64;
                (0..=buf.len() as i64).contains(&rel).then(|| {
                    let r = rel as usize;
                    ((r - f.partition_point(|&j| j < r)) << 3) | (o & 7)
                })
            })
            .collect();
        st.push(None);
        for i in 0..hi - lo {
            let a = st[i];
            let mut b = st[i + 1];
            if let (Some(a0), None) = (a, b) {
                b = mcu_end(d, a0, layout, dc, ac).ok();
            }
            mcus.push(match (a, b) {
                (Some(a), Some(b)) if a < b && b <= limit => Some(Chunk { seg: si as u32, a, len: b - a }),
                _ => None,
            });
        }
    }
    mcus
}

fn rebuild_chunks(chunks: &[Option<Chunk>], ds: &[Vec<u8>], ri: usize, neutral: &[(u32, u32)]) -> Vec<u8> {
    let total = chunks.len();
    let seg = if ri != 0 { ri } else { total };
    let mut out = Vec::new();
    for si in 0..total.div_ceil(seg) {
        let mut w = BitWriter::new();
        for ch in &chunks[si * seg..((si + 1) * seg).min(total)] {
            match ch {
                Some(c) => w.put_range(&ds[c.seg as usize], c.a, c.a + c.len),
                None => neutral.iter().for_each(|&(v, l)| w.put(v, l)),
            }
        }
        w.finish(&mut out);
        if (si + 1) * seg < total {
            out.extend_from_slice(&[0xFF, 0xD0 + (si & 7) as u8]);
        }
    }
    out
}

// ------------------------------------------------------------------- main
/// ins: chèn N MCU trung tính (None) tại idx, cắt đuôi | del: xoá N MCU, bù đuôi.
/// Trả về (mcus mới, hi, n hiệu dụng)
fn apply<T>(mut v: Vec<Option<T>>, act: Option<bool>, idx: usize, n: usize, total: usize) -> R<(Vec<Option<T>>, usize, usize)> {
    match act {
        Some(true) => {
            let ne = n.min(total - idx);
            v.splice(idx..idx, std::iter::repeat_with(|| None).take(ne));
            v.truncate(total);
            Ok((v, idx.saturating_add(n).min(total), n))
        }
        Some(false) => {
            if idx >= total {
                return Err("No MCU left to delete".into());
            }
            let n = n.min(total - idx);
            v.drain(idx..idx + n);
            v.extend((0..n).map(|_| None));
            Ok((v, (idx + n).min(total - n), n))
        }
        None => Ok((v, total, n)),
    }
}

type Adj = [Option<(bool, i32)>; 3]; // (tuyệt đối?, giá trị)

fn process(data: &[u8], x: usize, y: usize, act: Option<bool>, n: usize, adj: Adj, src: &str, range_end: Option<(usize, usize)>) -> R<Vec<u8>> {
    let pj = parse_jpeg(data)?;
    let comps = &pj.comps;
    let nf = comps.len();
    if pj.sos.len() != nf {
        return Err("Non-interleaved (multi-scan) JPEG is not supported".into());
    }
    let mut idx_of = HashMap::new();
    for (i, c) in comps.iter().enumerate() {
        idx_of.insert(c.0, i);
    }
    let order: Vec<usize> = pj.sos.iter().map(|s| idx_of.get(&s.0).copied()).collect::<Option<_>>()
        .ok_or("SOS references a missing component")?;

    let miss = || "Missing Huffman table for scan";
    let (mut dc_o, mut ac_o): (Vec<Option<&Huff>>, Vec<Option<&Huff>>) = (vec![None; nf], vec![None; nf]);
    for &(cid, td, ta) in &pj.sos {
        let c = idx_of[&cid];
        dc_o[c] = pj.huff.get(&(0, td));
        ac_o[c] = pj.huff.get(&(1, ta));
    }
    let dc_h: Vec<&Huff> = dc_o.into_iter().collect::<Option<_>>().ok_or_else(miss)?;
    let ac_h: Vec<&Huff> = ac_o.into_iter().collect::<Option<_>>().ok_or_else(miss)?;
    let eob: Vec<(u32, u32)> = ac_h.iter().map(|h| (h.enc[0].1 != 0).then(|| (h.enc[0].0 as u32, h.enc[0].1 as u32)))
        .collect::<Option<_>>().ok_or("AC Huffman table has no EOB code")?;
    let has_adj = adj.iter().any(|a| a.is_some());
    for c in 0..3 {
        if adj[c].is_some() && (c >= nf || !pj.q0.contains_key(&comps[c].3)) {
            return Err("Missing component or DQT table for color adjust".into());
        }
    }

    let hmax = comps.iter().map(|c| c.1 as usize).max().unwrap();
    let vmax = comps.iter().map(|c| c.2 as usize).max().unwrap();
    let (layout, mx, my): (Vec<usize>, usize, usize) = if nf == 1 {
        (vec![0], pj.w.div_ceil(8), pj.h.div_ceil(8))
    } else {
        let l = order.iter().flat_map(|&ci| std::iter::repeat(ci).take(comps[ci].1 as usize * comps[ci].2 as usize)).collect();
        (l, pj.w.div_ceil(8 * hmax), pj.h.div_ceil(8 * vmax))
    };
    let total = mx * my;

    if x >= mx || y >= my {
        return Err(format!("MCU ({x},{y}) out of range (x<{mx}, y<{my})"));
    }
    let idx = y * mx + x;
    let mut n = n;
    if let Some((ex, ey)) = range_end {
        if ex >= mx || ey >= my {
            return Err(format!("MCU ({ex}, {ey}) out of range (x<{mx}, y<{my})"));
        }
        let e = ey * mx + ex;
        if e < idx {
            return Err("End MCU is before start MCU".into());
        }
        n = e - idx + 1;
    }

    // tách scan
    let sc = pj.scan_start;
    let end = (sc..data.len().saturating_sub(1))
        .find(|&i| data[i] == 0xFF && !matches!(data[i + 1], 0x00 | 0xD0..=0xD7 | 0xFF));
    let end_pos = match end {
        Some(e) => {
            if data[e + 1] != 0xD9 {
                return Err("Multi-scan JPEG is not supported".into());
            }
            e
        }
        None => data.len(),
    };
    let (raw, tail) = (&data[sc.min(end_pos)..end_pos], &data[end_pos..]);
    let mut parts: Vec<&[u8]> = Vec::new();
    let (mut last, mut i) = (0, 0);
    while i + 1 < raw.len() {
        if raw[i] == 0xFF && (0xD0..=0xD7).contains(&raw[i + 1]) {
            parts.push(&raw[last..i]);
            i += 2;
            last = i;
        } else {
            i += 1;
        }
    }
    parts.push(&raw[last.min(raw.len())..]);
    let seg_size = if pj.dri != 0 { pj.dri } else { total };
    let nseg = total.div_ceil(seg_size);
    if parts.len() != nseg {
        return Err(format!("RST segments ({}) != expected ({nseg}); corrupt file?", parts.len()));
    }
    let ds: Vec<Vec<u8>> = parts.iter().map(|p| destuff(p)).collect();

    let new_scan;
    let info;
    if act.is_some() && !has_adj {
        // cắt bit theo log, không giải mã
        let offs = get_offs(src, mx, my, total, sc)?;
        let mut ps = Vec::with_capacity(nseg);
        let mut p = sc;
        for b in &parts {
            ps.push(p);
            p += b.len() + 2;
        }
        let mcus = to_chunks(&parts, &ds, &ps, seg_size, total, &offs, &layout, &dc_h, &ac_h);
        let lost = mcus.iter().filter(|m| m.is_none()).count();
        if lost != 0 {
            println!("Warning: {lost} MCU missing in log/unreadable -> neutral");
        }
        let (new, _, n) = apply(mcus, act, idx, n, total)?;
        info = format!("{} {n} @{x},{y}", if act == Some(true) { "ins" } else { "del" });
        if layout.iter().any(|&c| dc_h[c].enc[0].1 == 0) {
            return Err("DC Huffman table has no category 0; cannot build neutral MCU".into());
        }
        // MCU trung tính: mỗi block = DC category 0 + EOB
        let mut neutral = Vec::new();
        for &c in &layout {
            neutral.push((dc_h[c].enc[0].0 as u32, dc_h[c].enc[0].1 as u32));
            neutral.push(eob[c]);
        }
        new_scan = rebuild_chunks(&new, &ds, pj.dri, &neutral);
    } else {
        let mut mcus: Vec<Option<Vec<Blk>>> = Vec::with_capacity(total);
        for (si, d) in ds.iter().enumerate() {
            let cnt = seg_size.min(total - si * seg_size);
            let mut seg = Vec::new();
            if let Err(e) = decode_segment(d, cnt, &layout, &dc_h, &ac_h, &mut seg, si as u32, mcus.len()) {
                // scan hỏng: giữ phần giải mã được, phần mất -> MCU trung tính
                println!("Warning: {e}; {} MCU lost", cnt - seg.len());
            }
            let got = seg.len();
            mcus.extend(seg.into_iter().map(Some));
            mcus.extend((0..cnt - got).map(|_| None));
        }
        let (mut new, hi, n) = apply(mcus, act, idx, n, total)?;
        let mut inf = match act {
            Some(true) => format!("ins {n} @{x},{y}"),
            Some(false) => format!("del {n} @{x},{y}"),
            None => format!("@{x},{y}"),
        };
        if has_adj {
            inf += &format!(" color x{}", hi.saturating_sub(idx));
            // V (mức pixel) -> DC lượng tử = 8·V/Q0; Y tuyệt đối tính so với mức 128
            let mut aq: Adj = [None; 3];
            for c in 0..3 {
                if let Some((ab, v)) = adj[c] {
                    let q = pj.q0[&comps[c].3] as f64;
                    if q == 0.0 {
                        return Err("Invalid quantization table".into());
                    }
                    let off = if ab && c == 0 { 128 } else { 0 };
                    aq[c] = Some((ab, (8.0 * (v - off) as f64 / q).round_ties_even() as i32));
                }
            }
            let mut base = vec![0i32; nf];
            if idx > 0 {
                if let Some(blks) = &new[idx - 1] {
                    for (&c, b) in layout.iter().zip(blks) {
                        base[c] = b.dc;
                    }
                }
            }
            for i in idx..hi {
                let blks = new[i].take().unwrap_or_else(|| {
                    layout.iter().map(|&c| Blk { dc: base[c], seg: EOB_SEG, a: 0, len: 0 }).collect()
                });
                new[i] = Some(
                    blks.into_iter().zip(&layout).map(|(mut b, &c)| {
                        if let Some((ab, v)) = aq[c] {
                            b.dc = (if ab { v } else { b.dc.wrapping_add(v) }).clamp(-2047, 2047);
                        }
                        b
                    }).collect(),
                );
            }
        }
        info = inf;
        new_scan = rebuild_scan(&new, &ds, pj.dri, &layout, nf, &dc_h, &eob)?;
    }
    println!("{}x{} MCU {mx}x{my}={total} DRI={} | {info}", pj.w, pj.h, pj.dri);
    let mut out = Vec::with_capacity(sc + new_scan.len() + tail.len());
    out.extend_from_slice(&data[..sc]);
    out.extend_from_slice(&new_scan);
    out.extend_from_slice(tail);
    Ok(out)
}

// ---- tham số dòng lệnh
fn digits(s: &str) -> (&str, &str) {
    s.split_at(s.find(|c: char| !c.is_ascii_digit()).unwrap_or(s.len()))
}

fn parse_at(s: &str) -> Option<((usize, usize), &str)> {
    let (a, s) = digits(s.strip_prefix('@')?);
    let (b, s) = digits(s.strip_prefix(',')?);
    Some(((a.parse().ok()?, b.parse().ok()?), s))
}

struct Tok {
    act: Option<bool>, // true = ins
    n: Option<usize>,
    pos: Option<(usize, usize)>,
    end: Option<(usize, usize)>,
}

/// ^(?:(ins|del)(\d*))?(?:@(\d+),(\d+))?(?:@(\d+),(\d+))?$ (không phân biệt hoa thường)
fn parse_tok(t: &str) -> Option<Tok> {
    let l = t.to_ascii_lowercase();
    let mut s = l.as_str();
    let (mut act, mut n) = (None, None);
    if let Some((a, r)) = s.strip_prefix("ins").map(|r| (true, r)).or_else(|| s.strip_prefix("del").map(|r| (false, r))) {
        let (d, r) = digits(r);
        act = Some(a);
        n = d.parse().ok();
        s = r;
    }
    let (mut pos, mut end) = (None, None);
    if let Some((p, r)) = parse_at(s) {
        pos = Some(p);
        s = r;
        if let Some((p, r)) = parse_at(s) {
            end = Some(p);
            s = r;
        }
    }
    s.is_empty().then_some(Tok { act, n, pos, end })
}

fn parse_ch(t: &str) -> Option<(usize, i32)> {
    let l = t.to_ascii_lowercase();
    [("y", 0), ("cb", 1), ("cr", 2)].iter().find_map(|&(p, c)| Some((c, l.strip_prefix(p)?.parse().ok()?)))
}

fn parse_ycc(t: &str) -> Option<[Option<i32>; 3]> {
    let l = t.to_ascii_lowercase();
    let v: Vec<&str> = l.strip_prefix("ycc")?.split(',').collect();
    if v.len() != 3 {
        return None;
    }
    let mut r = [None; 3];
    for (i, s) in v.iter().enumerate() {
        r[i] = if *s == "_" { None } else { Some(s.parse().ok()?) };
    }
    Some(r)
}

fn fail(msg: &str) -> ! {
    eprintln!("{USAGE}\nmcusplice: error: {msg}");
    std::process::exit(2)
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.is_empty() || args.iter().any(|a| a == "-h" || a == "--help") {
        println!("{USAGE}");
        std::process::exit(if args.is_empty() { 2 } else { 0 });
    }
    let (mut files, mut act, mut n, mut x, mut y, mut adj, mut end): (Vec<&String>, _, usize, usize, usize, Adj, _) =
        (vec![], None, 1, 0, 0, [None; 3], None);
    for t in &args {
        if let Some(tk) = Some(t).filter(|t| !t.is_empty()).and_then(|t| parse_tok(t)) {
            if let Some(a) = tk.act {
                act = Some(a);
                n = tk.n.unwrap_or(1);
            }
            if let Some(p) = tk.pos {
                (x, y) = p;
            }
            if let Some(e) = tk.end {
                if !(tk.act == Some(false) && tk.n.is_none() && tk.pos.is_some()) {
                    fail("Range syntax: del@x1,y1@x2,y2");
                }
                end = Some(e);
            }
        } else if let Some((c, v)) = parse_ch(t) {
            adj[c] = Some((false, v));
        } else if let Some(v) = parse_ycc(t) {
            for c in 0..3 {
                if let Some(v) = v[c] {
                    adj[c] = Some((true, v));
                }
            }
        } else {
            files.push(t);
        }
    }
    if !(act.is_some() || adj.iter().any(|a| a.is_some())) || !(1..=2).contains(&files.len()) {
        fail("Demo: input.jpg [out.jpg] ins10@10,50 y30 cb-20 cr15");
    }

    let data = std::fs::read(files[0]).unwrap_or_else(|e| {
        eprintln!("Error: {e}");
        std::process::exit(1)
    });
    let out = process(&data, x, y, act, n, adj, files[0], end).unwrap_or_else(|e| {
        eprintln!("Error: {e}");
        std::process::exit(1)
    });
    let path = match files.get(1) {
        Some(p) => p.to_string(),
        None => {
            let f = files[0];
            let dot = f.rfind('.').filter(|&i| f[..i].rfind(['/', '\\']).map_or(i > 0, |s| i > s + 1));
            match dot {
                Some(i) => format!("{}_out{}", &f[..i], &f[i..]),
                None => format!("{f}_out"),
            }
        }
    };
    if let Err(e) = std::fs::write(&path, out) {
        eprintln!("Error: {e}");
        std::process::exit(1);
    }
    println!("Saved: {path}");
}
