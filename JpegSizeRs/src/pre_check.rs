use crate::util::ByteStream;

pub struct PreCheck {
    chunk_size: usize,
}

/// (uint)((acc << 8) + ReadByte())  -- ReadByte() may be -1 at EOF
fn push8(acc: u32, b: i32) -> u32 {
    (acc << 8).wrapping_add(b as u32)
}

/// (uint)(((bound << 16) >> 8) + ReadByte())
fn push16_8(bound: u32, b: i32) -> u32 {
    ((bound << 16) >> 8).wrapping_add(b as u32)
}

impl PreCheck {
    pub fn new() -> Self {
        PreCheck { chunk_size: 4 * 1024 }
    }

    pub fn is_jpeg(&self, fs: &mut ByteStream) -> bool {
        // 1) no remaining byte boundary  -> non-JPEG
        // 2) exactly one                 -> JPEG
        // 3) more than one               -> Oscar method
        let cpoint = fs.position();
        let cnt_rem_bnd = self.get_cnt_rem_bnd(fs);

        if cnt_rem_bnd == 0 {
            false
        } else if cnt_rem_bnd == 1 {
            true
        } else {
            fs.set_position(cpoint);
            let r = self.search_ff00_ffda_rst(fs);
            let flag_ff00 = (r[0] as f64) > 9.7 && r[0] < 47;
            // flag_rst = (not a) or b
            let flag_rst = r[2] == 0 || r[3] == 1;
            flag_ff00 && flag_rst
        }
    }

    fn get_cnt_rem_bnd(&self, fs: &mut ByteStream) -> i32 {
        let mut cand = [1i32; 8];
        let mut bound_value = self.read_next_n_bytes(fs, 3);
        for _ in 0..self.chunk_size {
            for j in 0..8usize {
                let test_value = (bound_value << (j + 8)) >> 16;
                if (65280 < test_value && test_value < 65488) || (65497 < test_value && test_value <= 65535) {
                    cand[j] = 0;
                    if cand.iter().sum::<i32>() == 0 {
                        return 0;
                    }
                }
            }
            bound_value = push16_8(bound_value, fs.read_byte());
        }
        cand.iter().sum()
    }

    /// returns [cnt_ff00, cnt_ffda, has_rst_marker, is_rst_in_mod8]
    fn search_ff00_ffda_rst(&self, fs: &mut ByteStream) -> [i32; 4] {
        let mut cnt_ff00 = 0;
        let mut cnt_ffda = 0;
        let mut has_rst_marker = 0;
        let mut is_rst_in_mod8 = 0;

        let mut check_rst = true;
        let mut flag_loop_all_rst_markers = true;
        let mut next_rst_marker: u32 = 0;
        let mut rst_marker_offset: usize = 0;

        let mut bound_value = self.read_next_n_bytes(fs, 3);
        for _ in 0..self.chunk_size {
            for j in 0..8usize {
                let test_value = (bound_value << (j + 8)) >> 16;
                if 65280 == test_value {
                    cnt_ff00 += 1;
                } else if 65498 == test_value {
                    cnt_ffda += 1;
                }

                if check_rst {
                    if flag_loop_all_rst_markers {
                        for k in 0..8u32 {
                            if 65488 + k == test_value {
                                flag_loop_all_rst_markers = false;
                                next_rst_marker = if k < 7 { 65488 + k + 1 } else { 65488 };
                                rst_marker_offset = j;
                                has_rst_marker = 1;
                                is_rst_in_mod8 = 1;
                            }
                        }
                    } else if next_rst_marker == test_value {
                        if rst_marker_offset == j {
                            next_rst_marker = if next_rst_marker == 65495 { 65488 } else { next_rst_marker + 1 };
                        } else {
                            // RST modulus is not 8 -> not JPEG encoded data
                            check_rst = false;
                            is_rst_in_mod8 = 0;
                        }
                    }
                }
            }
            bound_value = push16_8(bound_value, fs.read_byte());
        }
        [cnt_ff00, cnt_ffda, has_rst_marker, is_rst_in_mod8]
    }

    /// Returns (sos_i, sos_point). sos_i is -1 .. 5 (see the original); sos_point == 0 when nothing was found.
    pub fn get_sos_cnt_and_point(&self, fs: &mut ByteStream, is_chunk: bool) -> (i32, i64) {
        // 0xFFDA00 08010100 003F00
        // 0xFFDA00 0C030100 02110311 003F00
        // 0xFFDA00 0C030000 01110211 003F00
        // 0xFFDA00 0C030000 01000200 003F00
        // 0xFFDA00 0C035200 47004200 003F00
        let last_point = if is_chunk { fs.position() + 4 * 1024 } else { fs.length() };

        let sos2nd: [u32; 4] = [0x08010100, 0x0C030100, 0x0C030000, 0x0C035200];
        let sos3rd: [u32; 4] = [0x02110311, 0x01110211, 0x01000200, 0x47004200];
        let mut sos_point: i64 = 0;
        let mut bit_buffer = self.read_next_n_bytes(fs, 4);

        let mut sos_i: i32 = -1;

        while fs.position() < last_point {
            let offset = search_hex(bit_buffer, 0xFFDA00);
            let mut remainder = get_remainder(bit_buffer, offset);
            if offset < 8 {
                // hit
                bit_buffer = self.read_next_n_bytes(fs, 4);
                let mut sos_buffer = join2_remainder(remainder, bit_buffer, offset);
                remainder = get_remainder(bit_buffer, offset);

                // search 2nd sos block
                sos_i = search_sos_block(sos_buffer, &sos2nd);

                if sos_i < 4 {
                    if sos_i > 0 {
                        // check for 3rd blocks in sos-2/3/4/5
                        bit_buffer = self.read_next_n_bytes(fs, 4);
                        sos_buffer = join2_remainder(remainder, bit_buffer, offset);
                        remainder = get_remainder(bit_buffer, offset);
                        sos_i = search_sos_block(sos_buffer, &sos3rd) + 1;
                    }

                    if sos_i < 5 {
                        // check sos_fin
                        bit_buffer = self.read_next_n_bytes(fs, 4);
                        sos_buffer = join2_remainder(remainder, bit_buffer, offset);
                        let _ = get_remainder(bit_buffer, offset);
                        let sos_fin = search_hex(sos_buffer, 0x003F00);
                        if sos_fin < 8 {
                            // match!
                            sos_point = fs.position() + if sos_i == 0 { 11 } else { 15 };
                            break;
                        }
                    }
                }
            }
            bit_buffer = push8(bit_buffer, fs.read_byte());
        }
        (sos_i, sos_point)
    }

    fn read_next_n_bytes(&self, fs: &mut ByteStream, n: usize) -> u32 {
        let mut v: u32 = 0;
        for _ in 0..n {
            v = push8(v, fs.read_byte());
        }
        v
    }
}

fn search_hex(bit_buffer: u32, hex: u32) -> usize {
    let mut j = 0;
    while j < 8 {
        let test_value = (bit_buffer << j) >> 8;
        if hex == test_value {
            return j;
        }
        j += 1;
    }
    j
}

fn get_remainder(bit_buffer: u32, j: usize) -> u32 {
    // C# masks the shift count to 5 bits (j can be 8 -> shift 32 -> 0)
    bit_buffer.wrapping_shl((j + 24) as u32)
}

fn join2_remainder(remainder: u32, bit_buffer: u32, j: usize) -> u32 {
    remainder.wrapping_add((bit_buffer << j) >> (8 - j))
}

fn search_sos_block(sos_buffer: u32, blocks: &[u32; 4]) -> i32 {
    let mut k = 0;
    while k < blocks.len() {
        if sos_buffer == blocks[k] {
            break;
        }
        k += 1;
    }
    k as i32
}

