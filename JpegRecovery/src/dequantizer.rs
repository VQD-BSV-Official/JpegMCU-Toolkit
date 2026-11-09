use crate::global::{DCTMTX, TDCTMTX};
use crate::quantization::{DQT_C, DQT_Y};
use crate::util::*;

/// Converts DCT coefficient lists to RGB (0..1) block lists.
/// chr_type: 0 -> 4:4:4, 1 -> 4:4:0, 2 -> 4:2:2, 3 -> 4:2:0
pub fn ycc2rgb_block_list(block_list: &YccList, chr_type: usize) -> RgbList {
    let mut ycc: RgbList = [Vec::new(), Vec::new(), Vec::new()];
    for i in 0..3 {
        let dqt = if i == 0 { &DQT_Y[chr_type] } else { &DQT_C[chr_type] };
        // integer division: panics (DivideByZero) when a list is empty, like the C# code
        let upsampling = block_list[0].len() / block_list[i].len();
        for j in 0..block_list[i].len() {
            let idct_block = idct(&block_list[i][j], dqt);
            for _ in 0..upsampling {
                ycc[i].push(idct_block);
            }
        }
    }
    ycbcr2rgb(&ycc)
}

fn idct(block: &IBlock, qt: &[[i32; 8]; 8]) -> DBlock {
    let deq = dot_product(block, qt);
    let mut idct = matrix_product(&matrix_product(&TDCTMTX, &deq), &DCTMTX);
    for i in 0..8 {
        for j in 0..8 {
            idct[i][j] = round_even(idct[i][j] + 128.0).max(0.0).min(255.0);
        }
    }
    idct
}

fn dot_product(a: &IBlock, b: &[[i32; 8]; 8]) -> DBlock {
    let mut c = [[0.0; 8]; 8];
    for i in 0..8 {
        for j in 0..8 {
            c[i][j] = (a[i][j] as f64) * (b[i][j] as f64);
        }
    }
    c
}

fn matrix_product(a: &DBlock, b: &DBlock) -> DBlock {
    let mut c = [[0.0; 8]; 8];
    for i in 0..8 {
        for j in 0..8 {
            for k in 0..8 {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

fn ycbcr2rgb(ycc: &RgbList) -> RgbList {
    let mut r_list = Vec::new();
    let mut g_list = Vec::new();
    let mut b_list = Vec::new();
    for i in 0..ycc[0].len() {
        // C# swallowed the IndexOutOfRange here and returned what it had so far
        if i >= ycc[1].len() || i >= ycc[2].len() {
            break;
        }
        let mut rb = [[0.0; 8]; 8];
        let mut gb = [[0.0; 8]; 8];
        let mut bb = [[0.0; 8]; 8];
        for j in 0..8 {
            for k in 0..8 {
                let y = ycc[0][i][j][k];
                let cb = ycc[1][i][j][k];
                let cr = ycc[2][i][j][k];
                let r = round_even(1.164383561643836 * y + 0.000000301124397 * cb + 1.596026887335704 * cr - 222.9216171091943);
                let g = round_even(1.164383561643836 * y - 0.391762539941450 * cb - 0.812968292162205 * cr + 135.5754095229665);
                let b = round_even(1.164383561643836 * y + 2.017232639556459 * cb + 0.000003054261745 * cr - 276.8363057950315);
                rb[j][k] = r.max(0.0).min(255.0) / 255.0;
                gb[j][k] = g.max(0.0).min(255.0) / 255.0;
                bb[j][k] = b.max(0.0).min(255.0) / 255.0;
            }
        }
        r_list.push(rb);
        g_list.push(gb);
        b_list.push(bb);
    }
    [r_list, g_list, b_list]
}
