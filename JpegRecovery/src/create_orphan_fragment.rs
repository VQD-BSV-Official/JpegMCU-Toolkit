use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Read, Seek, SeekFrom, Write};

pub fn to_file_fragments() {
    let sr = BufReader::new(File::open(r"I:\JpegRecovery\HuffmanImg\list.txt").expect("list.txt"));
    for line in sr.lines() {
        let line = line.expect("read line");
        create_fragment_file(&line);
    }
    println!("yazdik aq");
}

pub fn create_fragment_file(file: &str) {
    let r_file = format!("{}.jpg", file);
    let mut fsr = File::open(&r_file).expect("open source");
    let len = fsr.metadata().expect("metadata").len() as i64;
    // FileMode.OpenOrCreate: no truncation
    let mut fsw = OpenOptions::new().write(true).create(true).open(file).expect("open target");

    fsr.seek(SeekFrom::Start(1024 * 10)).expect("seek");
    let l = len - 4;
    let mut pos: i64 = 1024 * 10;
    let mut out = Vec::new();
    let mut i = 0;
    while i < 64 * 1024 && pos < l {
        let mut b = [0u8; 1];
        if fsr.read(&mut b).expect("read") == 0 {
            out.push(255u8); // (byte)ReadByte() == (byte)-1
        } else {
            out.push(b[0]);
        }
        pos += 1;
        i += 1;
    }
    fsw.write_all(&out).expect("write");
}
