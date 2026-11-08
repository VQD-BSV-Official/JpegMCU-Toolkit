#![allow(dead_code)]
mod chroma; mod dequantizer; mod dimension; mod global; mod huffman; mod kmeans;
mod offset; mod pre_check; mod program; mod quantization; mod util;

use pre_check::PreCheck;
use program::Program;
use util::{install_panic_hook, mode, try_catch, ByteStream};

type Sizes = Vec<(i64, i64)>;

fn main() {
    install_panic_hook();
    let Some(path) = std::env::args().nth(1) else {
        eprintln!("Usage: jpeg_size <input.raw>");
        std::process::exit(2);
    };
    let mut input = ByteStream::open(&path).unwrap_or_else(|e| {
        eprintln!("{path}: {e}");
        std::process::exit(1)
    });
    let pc = PreCheck::new();
    let mut ws = Sizes::new();
    let mut rest = ByteStream::from_vec(par_head(&pc, &mut input, &mut ws));
    non_head(&pc, &mut rest, &mut ws);
    // width phổ biến nhất; height = fragment cao nhất có width đó
    let w = mode(&ws.iter().map(|x| x.0).collect::<Vec<_>>());
    match ws.iter().filter(|x| x.0 == w).map(|x| x.1).max() {
        Some(h) => println!("{w}x{h}"),
        None => std::process::exit(1),
    }
}

/// Trả về last_point; kích thước hợp lệ (nếu có) được đẩy vào `ws`.
fn run<'a>(ws: &mut Sizes, f: impl FnOnce() -> Program<'a>) -> Option<i64> {
    let (lp, size) = try_catch(|| {
        let mut p = f();
        p.start_decoding();
        (p.finalize_decoding(), p.size)
    })
    .ok()?;
    ws.extend(size);
    Some(lp)
}

/// Giải mã các đoạn có partial header; trả về phần còn lại (cand_non_head).
fn par_head(pc: &PreCheck, s: &mut ByteStream, ws: &mut Sizes) -> Vec<u8> {
    let mut pts: Vec<(i64, i64)> = Vec::new();
    let mut sos = pc.get_sos_cnt_and_point(s, false).1;
    while sos > 0 && s.position() < s.length() - 1024 {
        let end = run(ws, || Program::from_stream(s, sos, huffman::switch_huffman(1)))
            .unwrap_or(s.position());
        pts.push((sos, end));
        sos = pc.get_sos_cnt_and_point(s, false).1;
    }
    let (mut out, mut from) = (Vec::new(), 0);
    for &(a, b) in &pts {
        out.extend_from_slice(s.slice(from, a));
        from = b;
    }
    out.extend_from_slice(s.slice(from, s.length()));
    out
}

fn non_head(pc: &PreCheck, s: &mut ByteStream, ws: &mut Sizes) {
    let end = s.length();
    let mut pts: Vec<(i64, i64)> = Vec::new();
    while s.position() <= end - 4096 {
        let f = s.position();
        let (mut l, mut saved) = (0, false);
        let mut ok = pc.is_jpeg(s);
        while ok && s.position() < end - 1024 {
            saved = true;
            l = s.position();
            ok = pc.is_jpeg(s);
        }
        s.set_position(if saved { l + 4096 } else { f + 4096 });
        if saved { pts.push((f, l)); }
    }
    for (mut f, l) in pts {
        let mut size = l - f;
        run(ws, || Program::from_range(s, f, l, huffman::switch_huffman(1)));
        // nửa sau của segment vẫn còn: thử lại sau 128 byte
        while s.position() < f + size / 2 {
            f = s.position() + 128;
            size = l - f;
            run(ws, || Program::from_range(s, f, l, huffman::switch_huffman(1)));
        }
    }
}
