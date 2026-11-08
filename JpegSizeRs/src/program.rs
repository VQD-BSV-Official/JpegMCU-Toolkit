//! JPEG entropy-coded fragment decoder (C# class `Program`).

use crate::chroma::Chroma;
use crate::dimension::Dimension;
use crate::global::ZZMASK;
use crate::huffman::HuffTable;
use crate::offset::Offset;
use crate::util::*;

pub struct Program<'a> {
    src: &'a mut ByteStream,
    huff: &'static HuffTable,

    cnt: i32,                // next bit counter in byte buffer
    byte_boundary: u32,
    is_not_rst_marker: i32,  // 0 right after a restart marker (DC is not differential)

    init_position: i64,
    pub end_of_stream: i64,
    temp_last: i64,
    last_point: [i64; 3],

    byte_: u8,  // byte buffer
    byte1: u8,  // previous raw byte
    byte2: u8,

    is_terminated: bool,
    is_end_of_stream: bool,
    is_invalid_cw: bool,
    is_coeff_overflow: bool,

    overload: i32,
    invalidcw: i32,
    invalid_codeword: [i32; 3], // flags for YCbCr, YYCbCr, YYYYCbCr decodings
    overflow_codeword: [i32; 3],

    // results of startDecoding (these were static fields of `Global` in C#)
    mcu: usize,
    width: i32,
    pub size: Option<(i64, i64)>, // (width, height) pixel

    chr: usize,
    rgb_list: RgbList,
}

impl<'a> Program<'a> {
    fn empty(src: &'a mut ByteStream, huff: &'static HuffTable) -> Self {
        Program {
            src,
            huff,
            cnt: 0,
            byte_boundary: 0,
            is_not_rst_marker: 0,
            init_position: 0,
            end_of_stream: 0,
            temp_last: 0,
            last_point: [0; 3],
            byte_: 0,
            byte1: 0,
            byte2: 0,
            is_terminated: false,
            is_end_of_stream: false,
            is_invalid_cw: false,
            is_coeff_overflow: false,
            overload: 0,
            invalidcw: 0,
            invalid_codeword: [0; 3],
            overflow_codeword: [0; 3],
            mcu: 0,
            width: 0,
            size: None,
            chr: 0,
            rgb_list: [Vec::new(), Vec::new(), Vec::new()],
        }
    }

    /// Program(string file, FileStream, long initPosition)
    pub fn from_stream(stream: &'a mut ByteStream, init_position: i64, huff: &'static HuffTable) -> Self {
        let mut p = Program::empty(stream, huff);
        p.init_position = init_position;
        p.end_of_stream = p.src.length();
        p.src.set_position(init_position);
        p.find_byte_boundary();
        p
    }

    /// Program(string file, FileStream, long initPosition, long lastPosition)
    pub fn from_range(
        stream: &'a mut ByteStream,
        init_position: i64,
        last_position: i64,
        huff: &'static HuffTable,
    ) -> Self {
        let mut p = Program::empty(stream, huff);
        p.init_position = init_position;
        p.end_of_stream = last_position + 1024;
        p.src.set_position(init_position);
        p.find_byte_boundary();
        p
    }

    pub fn finalize_decoding(&mut self) -> i64 {
        self.size = try_catch(|| {
            let off = Offset.image_offset(&self.rgb_list, self.width, self.chr);
            image_size(self.rgb_list[0].len(), self.width, self.chr, off)
        })
        .ok();
        self.last_point[self.mcu]
    }

    pub fn start_decoding(&mut self) {
        let b1 = self.decode_as(1);
        self.last_point[0] = self.temp_last;
        let b2 = self.decode_as(2);
        self.last_point[1] = self.temp_last;
        let b4 = self.decode_as(4);
        self.last_point[2] = self.temp_last;

        let ch = Chroma::new(b1, b2, b4, self.invalid_codeword, self.overflow_codeword);
        let res = ch.chroma_subsampling();
        self.mcu = res.mcu;
        self.chr = res.chr;
        self.rgb_list = res.rgb_list;
        let ycc_list = res.ycc_list;
        let lp = self.last_point[self.mcu];
        self.src.set_position(lp);

        let mut d = Dimension::new();
        self.width = d.image_width(&self.rgb_list, &ycc_list, self.chr);
    }

    fn find_byte_boundary(&mut self) {
        let mut cand = [1i32; 8];
        let init = self.init_position;
        let s = &mut *self.src;
        let a = s.read_byte();
        let b = s.read_byte();
        let c = s.read_byte();
        let mut bound_value: u32 = (((a << 8).wrapping_add(b)) << 8).wrapping_add(c) as u32;
        for _ in 0..(8 * 1024) {
            for j in 0..8usize {
                let test_value = (bound_value << (j + 8)) >> 16;
                if (65280 < test_value && test_value < 65488) || (65497 < test_value && test_value <= 65535) {
                    cand[j] = 0;
                    if cand.iter().sum::<i32>() == 1 {
                        self.byte_boundary = cand.iter().position(|&x| x == 1).unwrap() as u32;
                        self.src.set_position(init);
                        return;
                    }
                }
            }
            let rb = self.src.read_byte();
            bound_value = ((bound_value << 16) >> 8).wrapping_add(rb as u32);
        }
        self.src.set_position(init);
    }

    fn initialize_settings(&mut self) {
        self.cnt = 0;
        self.is_not_rst_marker = 1;

        let init = self.init_position;
        self.src.set_position(init);

        self.byte_ = 0;
        self.byte1 = self.src.read_byte() as u8;
        self.byte2 = 0;

        self.overload = 0;
        self.invalidcw = 0;

        self.is_terminated = false;
        self.is_end_of_stream = false;
        self.is_invalid_cw = false;
        self.is_coeff_overflow = false;
    }

    fn next_bit(&mut self) -> u8 {
        if self.cnt == 0 {
            self.byte_ = self.next_byte();
            self.cnt = 8;

            if self.byte_ == 255 {
                let b2 = self.next_byte();
                if (208..=215).contains(&b2) {
                    // restart marker: DC prediction is reset
                    self.is_not_rst_marker = 0;
                    self.byte_ = self.next_byte();
                }
                // else: b2 is the stuffed byte (or garbage); it is consumed
            }
        }
        let bit = self.byte_ >> 7;
        self.cnt -= 1;
        self.byte_ <<= 1;
        bit
    }

    fn next_byte(&mut self) -> u8 {
        let mut my_byte = 0u8;
        if self.src.position() < self.end_of_stream {
            self.byte2 = self.src.read_byte() as u8;
            let bb = self.byte_boundary;
            my_byte = (((self.byte1 as u32) << bb) + ((self.byte2 as u32) >> (8 - bb))) as u8;
            self.byte1 = self.byte2;
            self.temp_last = self.src.position();
        } else {
            self.is_end_of_stream = true;
            self.is_terminated = true;
            self.temp_last = self.end_of_stream;
        }
        my_byte
    }

    fn decode(&mut self, comp: usize) -> u8 {
        let mut value: u8 = 255;
        let h = self.huff;
        let mcl = h.maxcodelength[comp];

        let mut code: u16 = self.next_bit() as u16;
        let mut i = 0i32;
        while i < mcl && !self.is_end_of_stream {
            if (code as i32) > h.maxcode[comp][i as usize] {
                code = (((code as u32) << 1) + self.next_bit() as u32) as u16;
                if code as i32 == (65535i32 >> (16 - mcl)) {
                    // invalid codeword
                    self.invalidcw += 1;
                    break;
                }
            } else {
                let j = h.valptr[comp][i as usize] + code as i32 - h.mincode[comp][i as usize];
                // an out-of-range index was an exception in C# (caught below -> invalid)
                if j >= 0 {
                    if let Some(&v) = h.huffval[comp].get(j as usize) {
                        value = v;
                    }
                }
                break;
            }
            i += 1;
        }
        self.is_invalid_cw = value == 255;
        self.is_terminated = self.is_invalid_cw || self.is_end_of_stream;
        value
    }

    fn receive(&mut self, s: i32) -> i32 {
        let mut i = 0;
        let mut v: i32 = 0;
        while i != s && !self.is_terminated {
            v = (v << 1).wrapping_add(self.next_bit() as i32);
            i += 1;
        }
        v
    }

    fn extend(v: i32, t: i32) -> i32 {
        if (v as f64) < 2f64.powi(t - 1) {
            v.wrapping_add((-1i32).wrapping_shl(t as u32).wrapping_add(1))
        } else {
            v
        }
    }

    fn decode_dc(&mut self, dc_index: usize, pre_dc: i32) -> i32 {
        let t = self.decode(dc_index) as i32;
        if self.is_terminated {
            return 0; // invalid codeword or end of stream
        }
        let r = self.receive(t);
        let diff = Self::extend(r, t).wrapping_add(self.is_not_rst_marker.wrapping_mul(pre_dc));
        self.is_not_rst_marker = 1;
        diff
    }

    fn decode_ac(&mut self, ac_index: usize, block: &mut IBlock) {
        let mut k: i32 = 0;
        while !self.is_terminated {
            let rs = self.decode(ac_index);
            if self.is_terminated {
                break; // invalid codeword or end of stream
            }
            let s = (rs % 16) as i32;
            let r = (rs >> 4) as i32;

            if s == 0 {
                if r == 0 {
                    break; // 00 EOB
                }
                k += 16; // F0 ZRL
            } else {
                k += r;
                self.is_coeff_overflow = k > 62;
                self.overload += if self.is_coeff_overflow { 1 } else { 0 };
                if self.overload > 10 {
                    self.is_terminated = true;
                    self.is_end_of_stream = true;
                    break;
                }
                if self.is_coeff_overflow {
                    break;
                }
                let z = ZZMASK[(k + 1) as usize];
                let rcv = self.receive(s);
                block[z % 8][z / 8] = Self::extend(rcv, s);
                if k == 62 {
                    break;
                }
                k += 1;
            }
        }
    }

    fn decode_block(&mut self, dc_index: usize, ac_index: usize, pre_dc: i32) -> IBlock {
        let mut block: IBlock = [[0; 8]; 8];
        block[0][0] = self.decode_dc(dc_index, pre_dc);
        if !self.is_terminated {
            self.decode_ac(ac_index, &mut block);
        }
        block
    }

    fn decode_as(&mut self, y_count: usize) -> YccList {
        self.initialize_settings();

        let mut y_pre_dc = 0;
        let mut cb_pre_dc = 0;
        let mut cr_pre_dc = 0;
        let mut block_list: YccList = [Vec::new(), Vec::new(), Vec::new()];
        let mut y_block: Vec<IBlock> = vec![[[0; 8]; 8]; y_count];

        while !self.is_end_of_stream {
            let mut i = 0;
            while i < y_count && !self.is_end_of_stream {
                y_block[i] = self.decode_block(0, 1, y_pre_dc);
                y_pre_dc = y_block[i][0][0];
                i += 1;
            }
            if self.is_end_of_stream {
                break;
            }
            let cb_block = self.decode_block(2, 3, cb_pre_dc);
            cb_pre_dc = cb_block[0][0];

            if self.is_end_of_stream {
                break;
            }
            let cr_block = self.decode_block(2, 3, cr_pre_dc);
            cr_pre_dc = cr_block[0][0];

            if self.is_end_of_stream {
                break;
            }
            for i in 0..y_count {
                block_list[0].push(y_block[i]);
            }
            block_list[1].push(cb_block);
            block_list[2].push(cr_block);
        }

        // y_count=1 -> 0, y_count=2 -> 1, y_count=4 -> 2
        self.invalid_codeword[y_count / 2] = if self.invalidcw > 0 { 1 } else if self.invalidcw < 0 { -1 } else { 0 };
        self.overflow_codeword[y_count / 2] = if self.overload >= 10 { 1 } else { 0 };

        block_list
    }
}

/// (width, height) pixel; panic nếu kích thước vô lý (fragment rác).
fn image_size(blocks: usize, width: i32, chr: usize, offset: i32) -> (i64, i64) {
    let (mut w, offset) = (width as i64, offset as i64);
    let mut bc = blocks as i64 + offset;
    bc += match (offset, chr) {
        (0, _) => 0,
        (_, 0) => w - bc % w,
        (_, 3) => 4 * w - (bc % 2) * w,
        _ => 2 * w - (bc % 2) * w,
    };
    w *= chr as i64 / 2 + 1;
    let h = if chr % 2 == 1 { 16 * (bc / (2 * w)) } else { 8 * (bc / w) };
    if w <= 0 || h <= 0 || w * 8 > 65535 || h > 65535 {
        panic!("invalid size");
    }
    (w * 8, h)
}
