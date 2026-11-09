#![allow(dead_code)]
//! JpegRecovery - port of the C# project to Rust.
//! (MainClass.cs -> main.rs; the other C# files each map to one module.)

mod chroma;
mod create_orphan_fragment;
mod dequantizer;
mod dimension;
mod global;
mod huffman;
mod kmeans;
mod offset;
mod pre_check;
mod program;
mod quantization;
mod rgb_builder;
mod util;

use pre_check::PreCheck;
use program::Program;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::time::Instant;
use util::{install_panic_hook, try_catch, ByteStream};

struct MainClass {
    pre_check: PreCheck,
    nrof_par_head: [i32; 25],
    len_par_head: [i64; 25],

    nrof_cand_non_head: [i32; 25],
    len_cand_non_head: [i64; 25],
    nrof_rec_non_head: [i32; 25],
    len_rec_non_head: [i64; 25],

    current_sd: usize,
}

fn main() {
    install_panic_hook();

    let mut args = std::env::args_os();
    let exe = args.next().unwrap_or_default();
    let Some(input_arg) = args.next() else {
        eprintln!("Usage: {} <input.raw>", std::path::Path::new(&exe).display());
        std::process::exit(2);
    };
    if args.next().is_some() {
        eprintln!("Usage: {} <input.raw>", std::path::Path::new(&exe).display());
        std::process::exit(2);
    }

    let input_path = std::path::PathBuf::from(input_arg);
    if !input_path.is_file() {
        eprintln!("Input file does not exist or is not a file: {}", input_path.display());
        std::process::exit(2);
    }

    let input_stem = input_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("recovery");
    let output_root = input_path
        .parent()
        .filter(|p| !p.as_os_str().is_empty())
        .unwrap_or_else(|| std::path::Path::new("."))
        .join(format!("{}_recovery", input_stem));

    let cand_non_head_dir = output_root.join("cand_non_head").join("1");
    let rec_par_head_dir = output_root.join("rec_par_head").join("1");
    let rec_non_head_dir = output_root.join("rec_non_head").join("1");
    let cand_non_head_path = cand_non_head_dir.join("1.raw");

    for dir in [&cand_non_head_dir, &rec_par_head_dir, &rec_non_head_dir] {
        if let Err(e) = fs::create_dir_all(dir) {
            eprintln!("Cannot create output directory {}: {}", dir.display(), e);
            std::process::exit(1);
        }
    }

    println!("Input : {}", input_path.display());
    println!("Output: {}", output_root.display());

    let mut m = MainClass::new();
    let dt = Instant::now();

    println!("[1/2] Processing partial headers...");
    m.process_sd_par_head(&input_path, &cand_non_head_path, &rec_par_head_dir);

    println!("[2/2] Processing non-header fragments...");
    m.process_sd_non_head(&cand_non_head_path, &rec_non_head_dir);

    m.print_stats();
    println!("Total time: {:.3}s", dt.elapsed().as_secs_f64());
}

impl MainClass {
    fn new() -> Self {
        MainClass {
            pre_check: PreCheck::new(),
            nrof_par_head: [0; 25],
            len_par_head: [0; 25],
            nrof_cand_non_head: [0; 25],
            len_cand_non_head: [0; 25],
            nrof_rec_non_head: [0; 25],
            len_rec_non_head: [0; 25],
            current_sd: 0,
        }
    }

    fn process_sd_par_head(
        &mut self,
        cand_rec_path: &std::path::Path,
        cand_non_head_path: &std::path::Path,
        rec_par_head_dir: &std::path::Path,
    ) {
        self.current_sd = 0;
        let par_head_pre = rec_par_head_dir.join("par_head_").to_string_lossy().into_owned();
        self.recover_par_head(
            &cand_rec_path.to_string_lossy(),
            &cand_non_head_path.to_string_lossy(),
            &par_head_pre,
        );
        println!("Partial headers processed.");
    }

    /// Recovers jpegs with partial header; residual segments are concatenated and saved under
    /// the cand_non_head directory as a single file (e.g., 1.raw).
    fn recover_par_head(&mut self, cand_rec_path: &str, cand_non_head_path: &str, par_head_pre: &str) {
        let mut par_points: Vec<[i64; 2]> = Vec::new();

        let mut cand_rec_stream = ByteStream::open(cand_rec_path).expect("cannot open cand_rec file");
        let (_sos_index, mut sos_point) = self.pre_check.get_sos_cnt_and_point(&mut cand_rec_stream, false);

        let mut par_cnt = 0;
        while sos_point > 0 && cand_rec_stream.position() < cand_rec_stream.length() - 1024 {
            let par_head_i = format!("{}{}", par_head_pre, par_cnt);
            par_cnt += 1;
            let par_last_position = self.recover_par_segment(&par_head_i, &mut cand_rec_stream, sos_point);

            par_points.push([sos_point, par_last_position]);
            println!(
                "{} -- {}",
                util::round_even(sos_point as f64 / 1024.0),
                util::round_even(par_last_position as f64 / 1024.0)
            );

            let (_i, p) = self.pre_check.get_sos_cnt_and_point(&mut cand_rec_stream, false);
            sos_point = p;
        }

        self.nrof_par_head[self.current_sd] = par_points.len() as i32;
        self.split_par_and_non_head(par_head_pre, cand_non_head_path, &mut cand_rec_stream, &par_points);
    }

    fn recover_par_segment(&mut self, par_head_i: &str, cand_rec_stream: &mut ByteStream, sos_point: i64) -> i64 {
        let mut last_position: i64;
        let mut max_last_position: i64 = 0;
        let mut max_huffman_index = 1;

        for i in 1..2usize {
            let r = try_catch(|| {
                let mut p = Program::from_stream(&format!("{}_{}", par_head_i, i), cand_rec_stream, sos_point, huffman::switch_huffman(i));
                p.start_decoding();
                p.finalize_decoding()
            });
            match r {
                Ok(lp) => {
                    last_position = lp;
                    if max_last_position < last_position {
                        max_last_position = last_position;
                        max_huffman_index = i;
                    }
                }
                Err(e) => println!("{}", e),
            }
        }

        if max_huffman_index < 2 {
            let r = try_catch(|| {
                let mut p = Program::from_stream(
                    &format!("{}_{}", par_head_i, max_huffman_index),
                    cand_rec_stream,
                    sos_point,
                    huffman::switch_huffman(max_huffman_index),
                );
                p.start_decoding();
                p.finalize_decoding()
            });
            match r {
                Ok(v) => return v,
                Err(e) => println!("{}", e),
            }
        }
        cand_rec_stream.position()
    }

    fn save_par_segment_binary(&self, par_head_i: &str, cand_rec_stream: &mut ByteStream, fpoint: i64, lpoint: i64) {
        let mut w = BufWriter::new(File::create(par_head_i).expect("create par segment"));
        for _ in fpoint..lpoint {
            w.write_all(&[cand_rec_stream.read_byte() as u8]).unwrap();
        }
        w.flush().unwrap();
    }

    fn dump_cand_non_head<W: Write>(&self, cand_non_head_stream: &mut W, cand_rec_stream: &mut ByteStream, fpoint: i64, lpoint: i64) {
        for _ in fpoint..lpoint {
            cand_non_head_stream.write_all(&[cand_rec_stream.read_byte() as u8]).unwrap();
        }
    }

    fn split_par_and_non_head(
        &mut self,
        par_head_pre: &str,
        cand_non_head_path: &str,
        cand_rec_stream: &mut ByteStream,
        par_points: &[[i64; 2]],
    ) {
        let mut par_head_len: i64 = 0;
        let mut non_head_len: i64 = 0;
        cand_rec_stream.set_position(0);
        let mut non_head_fpoint: i64 = 0;

        let mut cand_non_head_stream = BufWriter::new(File::create(cand_non_head_path).expect("create cand_non_head"));

        for i in 0..par_points.len() {
            let par_fpoint = par_points[i][0];
            let par_lpoint = par_points[i][1];
            par_head_len += par_lpoint - par_fpoint;

            let non_head_lpoint = par_fpoint;
            non_head_len += non_head_lpoint - non_head_fpoint;

            self.dump_cand_non_head(&mut cand_non_head_stream, cand_rec_stream, non_head_fpoint, non_head_lpoint);
            self.save_par_segment_binary(&format!("{}{}", par_head_pre, i), cand_rec_stream, par_fpoint, par_lpoint);
            non_head_fpoint = par_lpoint;
        }
        let non_head_lpoint = cand_rec_stream.length();
        non_head_len += non_head_lpoint - non_head_fpoint;
        self.dump_cand_non_head(&mut cand_non_head_stream, cand_rec_stream, non_head_fpoint, non_head_lpoint);

        cand_non_head_stream.flush().unwrap();
        self.len_par_head[self.current_sd] = par_head_len;
        self.len_cand_non_head[self.current_sd] = non_head_len;
    }

    fn print_stats(&self) {
        for i in 0..self.nrof_rec_non_head.len() {
            println!(
                "{:>12.1}{:>12.2}{:>12.1}{:>12.2}",
                self.nrof_cand_non_head[i] as f64,
                self.len_cand_non_head[i] as f64 / 1048576.0,
                self.nrof_rec_non_head[i] as f64,
                self.len_rec_non_head[i] as f64 / 1048576.0
            );
        }
    }

    fn process_sd_non_head(
        &mut self,
        cand_non_head_path: &std::path::Path,
        rec_non_head_dir: &std::path::Path,
    ) {
        self.current_sd = 0;
        let non_head_pre = rec_non_head_dir.join("non_head_").to_string_lossy().into_owned();

        let dt = Instant::now();
        let cand_path = cand_non_head_path.to_string_lossy().into_owned();
        let jpg_data_points = self.find_cand_non_head_fragments(&cand_path);
        println!("Fragment scan time: {:.3}s", dt.elapsed().as_secs_f64());

        self.recover_non_head_fragments(&cand_path, &non_head_pre, &jpg_data_points);
        println!("Non-header fragments processed.");
    }

    fn find_cand_non_head_fragments(&mut self, cand_non_head_path: &str) -> Vec<[i64; 2]> {
        let mut jpg_data_points: Vec<[i64; 2]> = Vec::new();

        let mut fpoint: i64;
        let mut lpoint: i64 = 0;
        let mut pair_save = false;

        let mut stream = ByteStream::open(cand_non_head_path).expect("cannot open cand_non_head file");
        let end_of_stream = stream.length();

        while stream.position() <= end_of_stream - 4096 {
            fpoint = stream.position();
            let mut is_jpeg = self.pre_check.is_jpeg(&mut stream);
            while is_jpeg && stream.position() < end_of_stream - 1024 {
                pair_save = true;
                lpoint = stream.position();
                is_jpeg = self.pre_check.is_jpeg(&mut stream);
            }
            if pair_save {
                pair_save = false;
                self.len_cand_non_head[self.current_sd] += lpoint - fpoint;
                jpg_data_points.push([fpoint, lpoint]);
                stream.set_position(lpoint + 4096);
            } else {
                stream.set_position(fpoint + 4096);
            }
        }

        self.nrof_cand_non_head[self.current_sd] = jpg_data_points.len() as i32;
        jpg_data_points
    }

    fn recover_non_head_fragments(&mut self, cand_non_head_path: &str, non_head_pre: &str, jpg_data_points: &[[i64; 2]]) {
        let mut stream = ByteStream::open(cand_non_head_path).expect("cannot open cand_non_head file");

        let mut rec_jpg_segments: Vec<[i64; 2]> = Vec::new();
        let mut cnt = 0;

        for pt in jpg_data_points {
            let mut fpoint = pt[0];
            let lpoint = pt[1];
            let mut segment_size = lpoint - fpoint;

            let rec_point = self.recover_non_head_segment(&format!("{}{}", non_head_pre, cnt), &mut stream, fpoint, lpoint);
            if rec_point > 0 {
                // a jpeg fragment is saved, so save the fragment binary too
                rec_jpg_segments.push([fpoint, rec_point]);
                cnt += 1;
            }

            while stream.position() < fpoint + segment_size / 2 {
                // the second half of the segment is still available: start again after 128 bytes
                fpoint = stream.position() + 128;
                segment_size = lpoint - fpoint;
                let rec_point = self.recover_non_head_segment(&format!("{}{}", non_head_pre, cnt), &mut stream, fpoint, lpoint);
                if rec_point > 0 {
                    rec_jpg_segments.push([fpoint, rec_point]);
                    cnt += 1;
                }
            }
        }

        self.nrof_rec_non_head[self.current_sd] = rec_jpg_segments.len() as i32;

        for i in 0..rec_jpg_segments.len() {
            let fpoint = rec_jpg_segments[i][0];
            let lpoint = rec_jpg_segments[i][1];

            self.save_non_head_segment_binary(&format!("{}{}", non_head_pre, i), &mut stream, fpoint, lpoint);
            self.len_rec_non_head[self.current_sd] += lpoint - fpoint;
        }
    }

    fn recover_non_head_segment(&mut self, non_head_i: &str, stream: &mut ByteStream, fpoint: i64, lpoint: i64) -> i64 {
        let mut last_position: i64 = 0;
        let mut max_last_position: i64 = 0;
        let mut max_huffman_index = 1;

        for i in 1..2usize {
            let r = try_catch(|| {
                let mut p = Program::from_range(&format!("{}_{}", non_head_i, i), stream, fpoint, lpoint, huffman::switch_huffman(i));
                p.start_decoding();
                p.finalize_decoding()
            });
            match r {
                Ok(lp) => {
                    last_position = lp;
                    if max_last_position < last_position {
                        max_last_position = last_position;
                        max_huffman_index = i;
                    }
                }
                Err(e) => println!("{}", e),
            }
        }
        if max_huffman_index < 2 {
            let r = try_catch(|| {
                let mut p = Program::from_range(
                    &format!("{}_{}", non_head_i, max_huffman_index),
                    stream,
                    fpoint,
                    lpoint,
                    huffman::switch_huffman(max_huffman_index),
                );
                p.start_decoding();
                p.finalize_decoding()
            });
            match r {
                Ok(v) => return v,
                Err(e) => println!("{}", e),
            }
        }
        last_position
    }

    fn save_non_head_segment_binary(&self, non_head_i: &str, stream: &mut ByteStream, fpoint: i64, lpoint: i64) {
        stream.set_position(fpoint);
        let mut w = BufWriter::new(File::create(non_head_i).expect("create segment file"));
        for _ in fpoint..lpoint {
            w.write_all(&[stream.read_byte() as u8]).unwrap();
        }
        w.flush().unwrap();
    }
}
