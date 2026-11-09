// jpgfix2.rs - sửa file JPEG baseline bị hỏng vùng đầu scan (port từ jpgfix2.py)
// Copyright (C) 2026 Vũ Quang Đại
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; version 2 of the License.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
// Cargo.toml:
//   [dependencies]
//   image = { version = "0.25", default-features = false, features = ["jpeg"] }
//   [profile.release]
//   opt-level = 3
//   lto = true
//   codegen-units = 1
//
// CLI: jpgfix2 in.jpg out.jpg [ref=anh.jpg] [off=dY,dCb,dCr] [win=256] [skip=8]
// Lib: fix(data, ref, off, win, skip) -> Result<(bytes, info), String>
use image::{imageops::{resize, FilterType}, Rgb, RgbImage};
use std::{collections::HashMap, env, fmt::Display, fs, process};

type Res<T> = Result<T, String>;

struct Huff { dec: Vec<u16>, enc: Vec<(u32, u8)>, max_sym: usize }

fn huff(cnt: &[u8], sym: &[u8]) -> Huff {
    let mut h = Huff { dec: vec![0; 65536], enc: vec![(0, 0); 256], max_sym: 0 };
    let (mut code, mut k) = (0usize, 0usize);
    for l in 1..=16usize {
        for _ in 0..cnt[l - 1] {
            let s = sym[k] as usize; k += 1;
            h.enc[s] = (code as u32, l as u8);
            h.max_sym = h.max_sym.max(s);
            let lo = code << (16 - l);
            for x in lo..(lo + (1 << (16 - l))).min(65536) { h.dec[x] = ((l << 8) | s) as u16; }
            code += 1;
        }
        code <<= 1;
    }
    h
}

struct Hdr {
    se: usize, tab: HashMap<(u8, u8), Huff>, w: usize, h: usize,
    cs: Vec<(u8, usize, usize, u8)>, sel: Vec<u8>, dri: usize, sofp: usize, q: HashMap<u8, u32>,
}

fn be16(b: &[u8]) -> usize { (b[0] as usize) << 8 | b[1] as usize }

fn parse(b: &[u8]) -> Res<Hdr> {
    let bad = || "header JPEG lỗi".to_string();
    let (mut i, mut tab, mut q, mut dri) = (2usize, HashMap::new(), HashMap::new(), 0usize);
    let (mut dim, mut cs, mut sofp) = (None, vec![], 0usize);
    loop {
        if i + 4 > b.len() { return Err(bad()); }
        let (m, l) = (b[i + 1], be16(&b[i + 2..]));
        let end = i + 2 + l;
        if l < 2 || end > b.len() { return Err(bad()); }
        let mut s = &b[i + 4..end];
        match m {
            0xC4 => while !s.is_empty() {
                if s.len() < 17 { return Err(bad()); }
                let n: usize = s[1..17].iter().map(|&x| x as usize).sum();
                if s.len() < 17 + n { return Err(bad()); }
                tab.insert((s[0] >> 4, s[0] & 15), huff(&s[1..17], &s[17..17 + n]));
                s = &s[17 + n..];
            },
            0xDB => while !s.is_empty() {
                let pr = (s[0] >> 4) as usize;
                if s.len() < 2 + pr { return Err(bad()); }
                q.insert(s[0] & 15, if pr > 0 { (s[1] as u32) << 8 | s[2] as u32 } else { s[1] as u32 });
                s = &s[(65 + 64 * pr).min(s.len())..];
            },
            0xC0 => {
                if s.len() < 6 || s.len() < 6 + 3 * s[5] as usize { return Err(bad()); }
                dim = Some((be16(&s[3..]), be16(&s[1..])));
                sofp = i;
                cs = (0..s[5] as usize)
                    .map(|k| (s[6 + 3 * k], (s[7 + 3 * k] >> 4) as usize, (s[7 + 3 * k] & 15) as usize, s[8 + 3 * k]))
                    .collect();
            }
            0xC2 => return Err("progressive không hỗ trợ".into()),
            0xDD => { if s.len() < 2 { return Err(bad()); } dri = be16(s); }
            0xDA => {
                let (w, h) = dim.ok_or_else(bad)?;
                if s.is_empty() || s.len() < 2 * s[0] as usize + 1 { return Err(bad()); }
                let sel = (0..s[0] as usize).map(|k| s[2 + 2 * k]).collect();
                return Ok(Hdr { se: end, tab, w, h, cs, sel, dri, sofp, q });
            }
            _ => {}
        }
        i = end;
    }
}

// AC: (bit tiến << 8) | số hệ số tiến; 255 = EOB, 0 = sai
fn conv(e: u16) -> u16 {
    let (l, rs) = (e >> 8, e & 255);
    if l == 0 { 0 }
    else if rs & 15 != 0 { ((l + (rs & 15)) << 8) | ((rs >> 4) + 1) }
    else { match rs { 0 => (l << 8) | 255, 0xF0 => (l << 8) | 16, _ => 0 } }
}

struct BitW { out: Vec<u8>, acc: u64, nb: u32 }

impl BitW {
    fn put(&mut self, v: u64, n: u32) {
        self.acc = (self.acc << n) | v; self.nb += n;
        while self.nb >= 8 { self.out.push((self.acc >> (self.nb - 8)) as u8); self.nb -= 8; }
        self.acc &= (1u64 << self.nb) - 1;
    }
    // chép bit [x, y) từ d
    fn range(&mut self, d: &[u8], mut x: usize, y: usize) {
        while x < y && x & 7 != 0 { self.put(((d[x >> 3] >> (7 - (x & 7))) & 1) as u64, 1); x += 1; }
        if x < y {
            let s = &d[x >> 3..(x >> 3) + ((y - x) >> 3)];
            if self.nb == 0 { self.out.extend_from_slice(s); } else { for &b in s { self.put(b as u64, 8); } }
            x += s.len() * 8;
        }
        while x < y { self.put(((d[x >> 3] >> (7 - (x & 7))) & 1) as u64, 1); x += 1; }
    }
    // pad bit 1 + byte stuffing
    fn finish(mut self) -> Vec<u8> {
        if self.nb > 0 { let p = 8 - self.nb; self.put((1u64 << p) - 1, p); }
        let mut o = Vec::with_capacity(self.out.len() + self.out.len() / 64 + 2);
        for b in self.out { o.push(b); if b == 0xFF { o.push(0); } }
        o
    }
}

struct Ctx<'a> { d: &'a [u8], e: usize, dc: Vec<&'a [u16]>, ac: &'a [Vec<u16>], seq: &'a [usize], tot: usize }

impl Ctx<'_> {
    // 16 bit bắt đầu tại bit p
    #[inline(always)]
    fn pk(&self, p: usize) -> usize {
        let i = p >> 3;
        let v = (self.d[i] as u32) << 16 | (self.d[i + 1] as u32) << 8 | self.d[i + 2] as u32;
        ((v >> (8 - (p & 7))) & 0xFFFF) as usize
    }

    // -> (cuối block, DC diff, cuối phần DC)
    fn blk(&self, mut p: usize, k: usize) -> Option<(usize, i64, usize)> {
        let e = self.dc[k][self.pk(p)] as usize;
        if e == 0 { return None; }
        let s = e & 255; p += e >> 8;
        let mut v = 0i64;
        if s > 0 {
            if s > 16 { return None; }
            v = (self.pk(p) >> (16 - s)) as i64;
            if v < 1 << (s - 1) { v -= (1 << s) - 1; }
            p += s;
        }
        let (q, mut i, a) = (p, 1usize, &self.ac[k]);
        while i < 64 {
            let e = a[self.pk(p)] as usize;
            if e == 0 { return None; }
            p += e >> 8;
            let c = e & 255;
            if c == 255 { break; }
            i += c;
        }
        (i <= 64 && p <= self.e).then_some((p, v, q))
    }

    // giải mã tới hết; đúng nếu kết thúc ở EOI + pad 1 -> (st, số MCU, tổng DC)
    fn core(&self, mut p: usize, ph: usize, dead: &[u64], vis: &mut Vec<usize>) -> Option<(usize, usize, Vec<i64>)> {
        for &k in &self.seq[ph..] { if ph == 0 { break; } p = self.blk(p, k)?.0; }
        let nc = self.ac.len();
        let (st, mut cnt, mut pred, mut sum) = (p, 0usize, vec![0i64; nc], vec![0i64; nc]);
        loop {
            let r = self.e as i64 - p as i64;
            if r < 8 {
                return (r >= 0 && self.pk(p) >> (16 - r as usize) == (1usize << r) - 1).then_some((st, cnt, sum));
            }
            if cnt == self.tot { return None; }
            if cnt < 64 {
                if dead[p >> 6] >> (p & 63) & 1 == 1 { return None; }
                vis.push(p);
            }
            for &k in self.seq {
                let e = self.dc[k][self.pk(p)] as usize;
                if e == 0 { return None; }
                let s = e & 255; p += e >> 8;
                if s > 0 {
                    if s > 16 { return None; }
                    let mut v = (self.pk(p) >> (16 - s)) as i64;
                    if v < 1 << (s - 1) { v -= (1 << s) - 1; }
                    p += s; pred[k] += v;
                }
                sum[k] += pred[k];
                let (mut i, a) = (1usize, &self.ac[k]);
                while i < 64 {
                    let e = a[self.pk(p)] as usize;
                    if e == 0 { return None; }
                    p += e >> 8;
                    let c = e & 255;
                    if c == 255 { break; }
                    i += c;
                }
                if i > 64 || p > self.e { return None; }
            }
            cnt += 1;
        }
    }

    // ranh giới MCU đã biết dẫn tới fail được nhớ trong dead: ứng viên hội tụ vào đây bị loại ngay
    fn run(&self, p: usize, ph: usize, dead: &mut [u64]) -> Option<(usize, usize, Vec<i64>)> {
        let mut vis = Vec::new();
        let r = self.core(p, ph, dead, &mut vis);
        if r.is_none() { for v in vis { dead[v >> 6] |= 1 << (v & 63); } }
        r
    }
}

fn ycc(p: &[u8]) -> [i32; 3] {
    let (r, g, b) = (p[0] as i32, p[1] as i32, p[2] as i32);
    [((19595 * r + 38470 * g + 7471 * b + 0x8000) >> 16).clamp(0, 255),
     ((-11059 * r - 21709 * g + 32768 * b + 0x808000) >> 16).clamp(0, 255),
     ((32768 * r - 27439 * g - 5329 * b + 0x808000) >> 16).clamp(0, 255)]
}

fn mean3(px: impl Iterator<Item = [i32; 3]>) -> [f64; 3] {
    let (mut s, mut n) = ([0u64; 3], 0u64);
    for c in px { for k in 0..3 { s[k] += c[k] as u64; } n += 1; }
    s.map(|x| x as f64 / n.max(1) as f64)
}

// làm tròn half-even như Python round()
fn rte(x: f64) -> i64 {
    if (x - x.trunc()).abs() == 0.5 { (2.0 * (x / 2.0).round()) as i64 } else { x.round() as i64 }
}

pub fn fix(b: &[u8], ref_img: Option<&RgbImage>, off: Option<Vec<i64>>, win: usize, skip: usize) -> Res<(Vec<u8>, String)> {
    let Hdr { se, tab, w, h, cs, sel, dri, sofp, q } = parse(b)?;
    let nc = cs.len();
    if nc == 0 || sel.len() < nc { return Err("header JPEG lỗi".into()); }
    let hdr = &b[..se];
    let seq: Vec<usize> = cs.iter().enumerate().flat_map(|(k, c)| std::iter::repeat(k).take(c.1 * c.2)).collect();
    let vm = cs.iter().map(|c| c.2).max().unwrap();
    let hm = cs.iter().map(|c| c.1).max().unwrap();
    if vm == 0 || hm == 0 || seq.is_empty() { return Err("sampling factor sai".into()); }
    let mx = w.div_ceil(8 * hm);
    let tot = mx * h.div_ceil(8 * vm);
    let miss = || "thiếu bảng DHT".to_string();
    let dcs: Vec<&Huff> = (0..nc).map(|k| tab.get(&(0, sel[k] >> 4)).ok_or_else(miss)).collect::<Res<_>>()?;
    let acs: Vec<&Huff> = (0..nc).map(|k| tab.get(&(1, sel[k] & 15)).ok_or_else(miss)).collect::<Res<_>>()?;

    let sc = &b[se..];
    let mut t = match sc.windows(2).position(|x| x == [0xFF, 0xD9]) {
        Some(e) => sc[..e + 2].to_vec(), // EOI thật đầu tiên, bỏ trailer phía sau
        None => { let mut v = sc.to_vec(); v.extend([0xFF, 0xD9]); v }
    };

    // height mới = bỏ các hàng MCU đã mất -> không còn vùng xám
    let new_h = |rows: usize| -> Res<usize> {
        h.checked_sub(rows * 8 * vm).filter(|&x| x < 65536).ok_or_else(|| "height sau khi cắt không hợp lệ".to_string())
    };
    let mk = |body: &[u8], rows: usize| -> Res<Vec<u8>> {
        let mut o = Vec::with_capacity(se + body.len());
        o.extend_from_slice(&hdr[..sofp + 5]);
        o.extend_from_slice(&(new_h(rows)? as u16).to_be_bytes());
        o.extend_from_slice(&hdr[sofp + 7..]);
        o.extend_from_slice(body);
        Ok(o)
    };

    if dri > 0 { // có RST: DC tự reset, đồng bộ theo marker
        let is_rst = |t: &[u8], i: usize| t[i] == 0xFF && i + 1 < t.len() && (0xD0..=0xD7).contains(&t[i + 1]);
        let (mut ms, mut i) = (vec![], 0);
        while i < t.len() { if is_rst(&t, i) { ms.push(i); i += 2; } else { i += 1; } }
        let j0 = tot.div_ceil(dri) as i64 - 1 - ms.len() as i64; // chỉ số interval của marker đầu tiên còn nguyên
        let sk = (0..ms.len()).find(|&x| ((j0 + 1 + x as i64) * dri as i64).rem_euclid(mx as i64) == 0)
            .ok_or("không tìm được RST khớp đầu hàng MCU")?;
        let rows = ((j0 + 1 + sk as i64) * dri as i64).div_euclid(mx as i64);
        if rows < 0 { return Err("số hàng MCU âm".into()); }
        let rows = rows as usize;
        let (mut body, mut n, mut i) = (Vec::with_capacity(t.len()), 0u8, ms[sk] + 2);
        while i < t.len() { // đánh số lại RST từ 0
            if is_rst(&t, i) { body.extend([0xFF, 0xD0 + n % 8]); n = n.wrapping_add(1); i += 2; }
            else { body.push(t[i]); i += 1; }
        }
        return Ok((mk(&body, rows)?, format!("RST: cắt {rows} hàng MCU, height {h} -> {}", h - rows * 8 * vm)));
    }

    t.truncate(t.len() - 2);
    let mut d = Vec::with_capacity(t.len() + 2048);
    let mut i = 0;
    while i < t.len() { d.push(t[i]); i += if t[i] == 0xFF && t.get(i + 1) == Some(&0) { 2 } else { 1 }; }
    let e = d.len() * 8;
    d.resize(d.len() + 2048, 0); // đệm để pk() không vượt biên
    let ac: Vec<Vec<u16>> = acs.iter().map(|a| a.dec.iter().map(|&x| conv(x)).collect()).collect();
    let cx = Ctx { d: &d, e, dc: dcs.iter().map(|x| &x.dec[..]).collect(), ac: &ac, seq: &seq, tot };

    let mut dead = vec![0u64; e / 64 + 2];
    let (st, cnt, sum) = (0..8 * win)
        .flat_map(|p0| (0..seq.len()).map(move |ph| (p0, ph)))
        .find_map(|(p0, ph)| cx.run(p0, ph, &mut dead))
        .ok_or("không tìm được điểm đồng bộ (DHT/kích thước trong header sai?)")?;

    // bỏ >=skip MCU đầu (rác do Huffman tự đồng bộ) rồi căn tới đầu hàng MCU
    let sk = skip + (-((tot - cnt + skip) as i64)).rem_euclid(mx as i64) as usize;
    if cnt <= sk { return Err("còn lại quá ít MCU".into()); }
    let rows = (tot - cnt + sk) / mx;
    new_h(rows)?;

    let (mut pred, mut first, mut p) = (vec![0i64; nc], vec![], st);
    for m in 0..=sk { // chỉ giải mã lại tới MCU đầu giữ lại
        for &k in &seq {
            let (q2, v, x) = cx.blk(p, k).ok_or("giải mã lại MCU đầu thất bại")?;
            pred[k] += v;
            if m == sk { first.push((k, pred[k], x, q2, p)); }
            p = q2;
        }
    }

    // chỉ MCU đầu phải mã hoá lại (DC); phần đuôi chép nguyên bit
    let build = |off: &[i64]| -> Res<Vec<u8>> {
        let (mut bw, mut seen) = (BitW { out: Vec::new(), acc: 0, nb: 0 }, vec![false; nc]);
        for &(k, v, x, q2, p) in &first {
            if seen[k] { bw.range(&d, p, q2); continue; }
            seen[k] = true;
            let mxs = dcs[k].max_sym.min(40);
            let v = (v + off[k]).clamp(1 - (1i64 << mxs), (1i64 << mxs) - 1);
            let s = (64 - v.unsigned_abs().leading_zeros()) as usize;
            let (c, l) = dcs[k].enc[s];
            if l == 0 { return Err("thiếu mã Huffman DC".into()); }
            bw.put(c as u64, l as u32);
            bw.put((if v > 0 { v } else { v + (1 << s) - 1 }) as u64, s as u32);
            bw.range(&d, x, q2);
        }
        bw.range(&d, first.last().unwrap().3, e);
        let mut body = bw.finish();
        body.extend([0xFF, 0xD9]);
        mk(&body, rows)
    };

    // DC tuyệt đối đầu bị mất -> cả ảnh lệch một hằng số mỗi kênh
    let mut off = match off {
        Some(o) if o.len() == nc => o,
        Some(_) => return Err("off phải có đúng một giá trị mỗi kênh".into()),
        None => (0..nc).map(|k| -rte(sum[k] as f64 / (cnt * seq.iter().filter(|&&x| x == k).count()) as f64)).collect(),
    };
    if let Some(img) = ref_img { // khớp trung bình YCbCr với ảnh tham chiếu ở đúng vùng còn lại
        let (rw, rh) = (w as u32, h as u32);
        let yimg = RgbImage::from_fn(img.width(), img.height(), |x, y| {
            let c = ycc(&img.get_pixel(x, y).0);
            Rgb([c[0] as u8, c[1] as u8, c[2] as u8])
        });
        let rs = resize(&yimg, rw, rh, FilterType::CatmullRom);
        let r = mean3(rs.as_raw()[rows * 8 * vm * w * 3..].chunks_exact(3).map(|c| [c[0] as i32, c[1] as i32, c[2] as i32]));
        for _ in 0..3 { // lặp để bù clip
            let jpg = build(&off)?;
            let im = image::load_from_memory(&jpg).map_err(|e| e.to_string())?.to_rgb8();
            let m = mean3(im.as_raw().chunks_exact(3).map(ycc));
            for k in 0..nc {
                let qd = *q.get(&cs[k].3).ok_or("thiếu bảng DQT")? as f64;
                off[k] += rte((r[k] - m[k]) * 8.0 / qd);
            }
        }
    }
    let i = first[0].4 >> 3; // đổi về offset thật (cộng lại byte stuffing)
    let fo = se + i + d[..i].iter().filter(|&&x| x == 0xFF).count();
    let info = format!(
        "MCU tốt đầu: #{}/{tot} (hàng {rows}) tại offset {fo} bit {}; sync gốc offset {}; cắt height {h} -> {}; off DC {off:?}",
        tot - cnt + sk, first[0].4 & 7, se + st / 8, h - rows * 8 * vm
    );
    Ok((build(&off)?, info))
}

fn die(m: impl Display) -> ! { eprintln!("{m}"); process::exit(1) }

fn main() {
    let a: Vec<String> = env::args().collect();
    if a.len() < 3 { die("jpgfix2 in.jpg out.jpg [ref=anh.jpg] [off=dY,dCb,dCr] [win=256] [skip=8]"); }
    let kv: HashMap<&str, &str> = a[3..].iter().filter_map(|x| x.split_once('=')).collect();
    let data = fs::read(&a[1]).unwrap_or_else(|e| die(e));
    let rf = kv.get("ref").map(|p| {
        let b = fs::read(p).unwrap_or_else(|e| die(e));
        image::load_from_memory(&b).unwrap_or_else(|e| die(e)).to_rgb8()
    });
    let off = kv.get("off").map(|s| s.split(',').map(|x| x.trim().parse::<i64>().unwrap_or_else(|e| die(e))).collect());
    let num = |k: &str, d: usize| kv.get(k).map_or(d, |v| v.parse().unwrap_or_else(|e| die(e)));
    match fix(&data, rf.as_ref(), off, num("win", 256), num("skip", 8)) {
        Ok((r, info)) => { fs::write(&a[2], r).unwrap_or_else(|e| die(e)); println!("{info}"); }
        Err(e) => die(e),
    }
}
