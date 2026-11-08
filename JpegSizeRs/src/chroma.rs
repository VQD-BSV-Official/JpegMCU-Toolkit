//! Detect chroma subsampling.
//! Output (chr): 0 -> 4:4:4, 1 -> 4:4:0, 2 -> 4:2:2, 3 -> 4:2:0
//! Original author: Erkam Uzun (03/26/2015)

use crate::dequantizer;
use crate::global::MAX_WIDTH;
use crate::util::*;

pub struct ChromaResult {
    pub mcu: usize,
    pub ycc_list: YccList,
    pub chr: usize,
    pub rgb_list: RgbList,
}

pub struct Chroma {
    block_list_with_1y: YccList,
    block_list_with_2y: YccList,
    block_list_with_4y: YccList,
    feature_flag: [i32; 3],
}

impl Chroma {
    pub fn new(
        b1: YccList,
        b2: YccList,
        b4: YccList,
        invalid_codeword: [i32; 3],
        overflow_codeword: [i32; 3],
    ) -> Self {
        Chroma {
            feature_flag: [
                invalid_codeword[0] | overflow_codeword[0],
                invalid_codeword[1] | overflow_codeword[1],
                invalid_codeword[2] | overflow_codeword[2],
            ],
            block_list_with_1y: b1,
            block_list_with_2y: b2,
            block_list_with_4y: b4,
        }
    }

    pub fn chroma_subsampling(self) -> ChromaResult {
        let mcu = self.mcu_structure();
        match mcu {
            0 => {
                let ycc = self.block_list_with_1y;
                let rgb = dequantizer::ycc2rgb_block_list(&ycc, 0);
                ChromaResult { mcu, ycc_list: ycc, chr: 0, rgb_list: rgb }
            }
            1 => {
                let ycc = self.block_list_with_2y;
                let (chr, rgb) = Self::layout(&ycc);
                ChromaResult { mcu, ycc_list: ycc, chr, rgb_list: rgb }
            }
            _ => {
                let ycc = self.block_list_with_4y;
                let rgb = dequantizer::ycc2rgb_block_list(&ycc, 3);
                ChromaResult { mcu, ycc_list: ycc, chr: 3, rgb_list: rgb }
            }
        }
    }

    /// 0 -> YCbCr, 1 -> YYCbCr, 2 -> YYYYCbCr
    fn mcu_structure(&self) -> usize {
        if self.feature_flag.iter().sum::<i32>() == 2 {
            self.feature_flag.iter().position(|&x| x == 0).unwrap_or(usize::MAX)
        } else {
            self.feature_decision()
        }
    }

    // ---------------------------------------------------------------- DECISION ARRAY
    fn feature_decision(&self) -> usize {
        // features order: DCMEAN, ACMEAN, COEFFMEAN, VARMEAN, VARMAX
        let max_int = (i32::MAX / 3 - 1) as f64;
        let max_arr = [max_int; 5];
        let mut f1 = [max_arr, max_arr, max_arr];
        let mut f2 = [max_arr, max_arr, max_arr];
        let mut f4 = [max_arr, max_arr, max_arr];

        let take = |l: &Vec<IBlock>, n: usize| -> Vec<IBlock> { l.iter().take(n).cloned().collect() };

        if self.feature_flag[0] == 1 {
            let b = &self.block_list_with_1y;
            let s = MAX_WIDTH.min(b[0].len());
            f1 = [comp_features(&take(&b[0], s)), comp_features(&take(&b[1], s)), comp_features(&take(&b[2], s))];
        }
        if self.feature_flag[1] == 1 {
            let b = &self.block_list_with_2y;
            let s = MAX_WIDTH.min(b[0].len());
            f2 = [comp_features(&take(&b[0], s)), comp_features(&take(&b[1], s / 2)), comp_features(&take(&b[2], s / 2))];
        }
        if self.feature_flag[2] == 1 {
            let b = &self.block_list_with_4y;
            let s = MAX_WIDTH.min(b[0].len());
            f4 = [comp_features(&take(&b[0], s)), comp_features(&take(&b[1], s / 4)), comp_features(&take(&b[2], s / 4))];
        }

        // DCMEAN: Y + Cb + Cr
        let dc_mean = [
            f1[0][0] + f1[1][0] + f1[2][0],
            f2[0][0] + f2[1][0] + f2[2][0],
            f4[0][0] + f4[1][0] + f4[2][0],
        ];
        // ACMEAN, VARMEAN, VARMAX: chroma only (Y term was commented out in the original)
        let ac_mean = [f1[1][1] + f1[2][1], f2[1][1] + f2[2][1], f4[1][1] + f4[2][1]];
        let coeff_mean = [
            f1[0][2] + f1[1][2] + f1[2][2],
            f2[0][2] + f2[1][2] + f2[2][2],
            f4[0][2] + f4[1][2] + f4[2][2],
        ];
        let var_mean = [f1[1][3] + f1[2][3], f2[1][3] + f2[2][3], f4[1][3] + f4[2][3]];
        let var_max = [f1[1][4] + f1[2][4], f2[1][4] + f2[2][4], f4[1][4] + f4[2][4]];

        let mut decision = [0i32; 3];
        decision[index_of_min(&dc_mean)] += 1;
        decision[index_of_min(&ac_mean)] += 1;
        decision[index_of_min(&coeff_mean)] += 1;
        decision[index_of_min(&var_mean)] += 1;
        decision[index_of_min(&var_max)] += 1;

        let max = *decision.iter().max().unwrap();
        decision.iter().position(|&x| x == max).unwrap()
    }

    // ---------------------------------------------------------------- YYCbCr LAYOUT DETECTION
    /// Returns (chr, rgb_list): 1 -> 4:4:0 (vertical), 2 -> 4:2:2 (horizontal)
    fn layout(ycc_list: &YccList) -> (usize, RgbList) {
        let ver_list = dequantizer::ycc2rgb_block_list(ycc_list, 1);
        let hor_list = dequantizer::ycc2rgb_block_list(ycc_list, 2);

        let mut ver_diff: Vec<[f64; 2]> = Vec::new();
        let mut hor_diff: Vec<[f64; 2]> = Vec::new();

        let mut i = 0;
        while i < ver_list[0].len() {
            ver_diff.push(border_diff(
                [&ver_list[0][i], &ver_list[1][i], &ver_list[2][i]],
                [&ver_list[0][i + 1], &ver_list[1][i + 1], &ver_list[2][i + 1]],
            ));
            i += 2;
        }
        let mut i = 0;
        while i < hor_list[0].len() {
            hor_diff.push(border_diff(
                [&hor_list[0][i], &hor_list[1][i], &hor_list[2][i]],
                [&hor_list[0][i + 1], &hor_list[1][i + 1], &hor_list[2][i + 1]],
            ));
            i += 2;
        }

        let col = |v: &Vec<[f64; 2]>, c: usize| -> f64 { average(&v.iter().map(|x| x[c]).collect::<Vec<_>>()) };

        if col(&ver_diff, 0) < col(&hor_diff, 1) && col(&ver_diff, 1) > col(&hor_diff, 0) {
            return (1, ver_list);
        }
        (2, hor_list)
    }
}

// -------------------------------------------------------------------- FEATURES
/// Returns [dcMean, acMean, coeffMean, mean of block variances, max of block variances]
fn comp_features(block_list: &[IBlock]) -> [f64; 5] {
    let mut counter_dc = 0i32;
    let mut counter_non_zero_dc = 0i32;
    let mut counter_ac = 0i32;
    let mut sum_dc = 0.0f64;
    let mut sum_ac = 0.0f64;

    let mut ac_list: Vec<i32> = Vec::new();
    let mut ac_variance = vec![0.0f64; block_list.len()];

    for i in 0..block_list.len() {
        let mut block = block_list[i];

        sum_dc += (block[0][0] as f64).abs();
        counter_dc += 1;
        counter_non_zero_dc += if block[0][0] == 0 { 0 } else { 1 };

        block[0][0] = 0;
        for j in 0..8 {
            for k in 0..8 {
                sum_ac += (block[j][k] as f64).abs();
                counter_ac += if block[j][k] == 0 { 0 } else { 1 };
                if j + k > 0 {
                    ac_list.push(block[j][k]);
                }
            }
        }

        let neg_mean = ac_list.iter().map(|&x| x as f64).sum::<f64>() / ac_list.len() as f64;
        ac_variance[i] = ac_list.iter().map(|&n| (n as f64 - neg_mean).powf(2.0)).sum::<f64>() / 63.0;
        ac_list.clear();
    }
    let sum_coeff = sum_dc + sum_ac;
    let imax = i32::MAX as f64;
    let dc_mean = if sum_dc > 0.0 { sum_dc / counter_dc as f64 } else { imax };
    let ac_mean = if sum_ac > 0.0 { sum_ac / counter_ac as f64 } else { imax };
    let coeff_mean = if sum_coeff > 0.0 { sum_coeff / (counter_non_zero_dc + counter_ac) as f64 } else { imax };
    let var_mean = if !ac_variance.is_empty() { average(&ac_variance) } else { imax };
    let var_max = if !ac_variance.is_empty() { ac_variance.iter().cloned().fold(f64::MIN, f64::max) } else { imax };

    [dc_mean, ac_mean, coeff_mean, var_mean, var_max]
}

/// [0] diff between bottom of F and top of S, [1] diff between right of F and left of S
fn border_diff(f: [&DBlock; 3], s: [&DBlock; 3]) -> [f64; 2] {
    let mut sum_tb = 0.0;
    let mut sum_lr = 0.0;
    for i in 0..8 {
        sum_tb += (sqr(f[0][7][i] - s[0][0][i]) + sqr(f[1][7][i] - s[1][0][i]) + sqr(f[2][7][i] - s[2][0][i])).sqrt();
        sum_lr += (sqr(f[0][i][7] - s[0][i][0]) + sqr(f[1][i][7] - s[1][i][0]) + sqr(f[2][i][7] - s[2][i][0])).sqrt();
    }
    [sum_tb / 8.0, sum_lr / 8.0]
}
