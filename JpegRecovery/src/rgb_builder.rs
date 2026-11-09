//! Writes the RGB block lists into a JPEG file (replacement for System.Drawing.Bitmap.Save).

use crate::util::*;
use image::codecs::jpeg::JpegEncoder;
use image::ColorType;
use std::fs::File;
use std::io::BufWriter;

struct Bmp {
    w: i64,
    h: i64,
    data: Vec<u8>, // RGB24, zero (black) initialised like a fresh Bitmap
}

impl Bmp {
    fn new(w: i64, h: i64) -> Bmp {
        if w <= 0 || h <= 0 || w > 65535 || h > 65535 {
            panic!("Parameter is not valid.");
        }
        Bmp { w, h, data: vec![0u8; (w * h * 3) as usize] }
    }
    /// false when out of range (SetPixel threw in C#)
    fn set_pixel(&mut self, x: i64, y: i64, r: u8, g: u8, b: u8) -> bool {
        if x < 0 || y < 0 || x >= self.w || y >= self.h {
            return false;
        }
        let o = ((y * self.w + x) * 3) as usize;
        self.data[o] = r;
        self.data[o + 1] = g;
        self.data[o + 2] = b;
        true
    }
}

fn sl<'a, 'b>(l: &'b [&'a DBlock], skip: i64, take: i64) -> &'b [&'a DBlock] {
    let s = (skip.max(0) as usize).min(l.len());
    let e = if take < 0 { s } else { s.saturating_add(take as usize).min(l.len()) };
    &l[s..e]
}

/// Returns the length of the written file.
pub fn write_pixels(rgb: &RgbList, width: i32, chr: usize, offset: i32, file_name: &str) -> u64 {
    let mut width = width as i64;
    let offset = offset as i64;
    let mut block_count = rgb[0].len() as i64 + offset;
    // note: `blockCount % 2 * width` == `(blockCount % 2) * width` in C#
    let tail = if offset == 0 {
        0
    } else if chr == 0 {
        width - (block_count % width)
    } else if chr == 3 {
        4 * width - ((block_count % 2) * width)
    } else {
        2 * width - ((block_count % 2) * width)
    };
    println!("Tail:{}", tail);
    block_count += tail;
    width *= (chr as i64) / 2 + 1;
    let height = if chr % 2 == 1 { 16 * (block_count / (2 * width)) } else { 8 * (block_count / width) };

    let mut bmp = Bmp::new(width * 8, height);

    let all: [Vec<&DBlock>; 3] = [rgb[0].iter().collect(), rgb[1].iter().collect(), rgb[2].iter().collect()];

    if chr % 2 == 0 {
        let row = |skip: i64, take: i64| -> [&[&DBlock]; 3] {
            [sl(&all[0], skip, take), sl(&all[1], skip, take), sl(&all[2], skip, take)]
        };
        let r = row(0, width - offset);
        write_row_block(&r, &mut bmp, 0, offset);
        let mut skip = width - offset;

        let mut i = 1;
        while i < height / 8 - 1 {
            let r = row(skip, width);
            write_row_block(&r, &mut bmp, i, 0);
            skip += width;
            i += 1;
        }

        let r = row(skip, i64::MAX);
        write_row_block(&r, &mut bmp, height / 8 - 1, 0);
    } else {
        // "even" = rows 0 of the MCU, "odd" = rows 1 (Dictionary<bool,...> from GroupBy)
        fn split<'a>(l: &[&'a DBlock], chr: usize) -> (Vec<&'a DBlock>, Vec<&'a DBlock>) {
            let mut ev = Vec::new();
            let mut od = Vec::new();
            for (i, b) in l.iter().enumerate() {
                let is_even = if chr == 1 { i % 2 == 0 } else { i % 4 == 0 || i % 4 == 1 };
                if is_even { ev.push(*b) } else { od.push(*b) }
            }
            if ev.is_empty() || od.is_empty() {
                panic!("The given key was not present in the dictionary.");
            }
            (ev, od)
        }
        let (r_even, r_odd) = split(&all[0], chr);
        let (g_even, g_odd) = split(&all[1], chr);
        let (b_even, b_odd) = split(&all[2], chr);

        let row = |e: &[Vec<&'_ DBlock>; 3], skip: i64, take: i64, bmp: &mut Bmp, last_row: i64, off: i64| {
            let r = [sl(&e[0], skip, take), sl(&e[1], skip, take), sl(&e[2], skip, take)];
            write_row_block(&r, bmp, last_row, off);
        };
        let even = [r_even, g_even, b_even];
        let odd = [r_odd, g_odd, b_odd];

        row(&even, 0, width - offset, &mut bmp, 0, offset);
        row(&odd, 0, width - offset, &mut bmp, 1, offset);

        let mut skip = width - offset;
        let mut i = 2;
        while i < height / 8 - 2 {
            row(&even, skip, width, &mut bmp, i, 0);
            row(&odd, skip, width, &mut bmp, i + 1, 0);
            skip += width;
            i += 2;
        }

        row(&even, skip, i64::MAX, &mut bmp, height / 8 - 2, 0);
        row(&odd, skip, i64::MAX, &mut bmp, height / 8 - 1, 0);
    }

    let file = File::create(file_name).expect("could not create output file");
    {
        let mut w = BufWriter::new(&file);
        let mut enc = JpegEncoder::new_with_quality(&mut w, 75);
        enc.encode(&bmp.data, bmp.w as u32, bmp.h as u32, ColorType::Rgb8)
            .expect("JPEG encoding failed");
    }
    file.metadata().map(|m| m.len()).unwrap_or(0)
}

fn write_row_block(rgb: &[&[&DBlock]; 3], bmp: &mut Bmp, last_row: i64, offset: i64) {
    let iter = rgb[0].len();
    for i in 0..iter {
        for j in 0..8usize {
            for k in 0..8usize {
                let x = (offset + i as i64) * 8 + j as i64;
                let y = last_row * 8 + k as i64;
                let rp = (255.0 * rgb[0][i][k][j]) as u8;
                let gp = (255.0 * rgb[1][i][k][j]) as u8;
                let bp = (255.0 * rgb[2][i][k][j]) as u8;
                if !bmp.set_pixel(x, y, rp, gp, bp) {
                    // Chroma subsampling error: C# returned from the method here
                    return;
                }
            }
        }
    }
}
