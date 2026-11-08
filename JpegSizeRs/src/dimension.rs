//! Image width detection (fusion of 5 methods).

use crate::global::MAX_WIDTH;
use crate::kmeans;
use crate::util::*;
use rustfft::{num_complex::Complex, FftPlanner};

type Ch<'a> = [Vec<&'a DBlock>; 3];
type RgbSlice<'a> = [&'a [DBlock]; 3];

pub struct Dimension {
    pub wlist: [i32; 5],
}

fn pick3<'a>(rgb: &RgbSlice<'a>, skip: i64, modulus: i64, keep: &[i64], take: i64) -> Ch<'a> {
    [
        pick(rgb[0], skip, modulus, keep, take),
        pick(rgb[1], skip, modulus, keep, take),
        pick(rgb[2], skip, modulus, keep, take),
    ]
}

fn blk<'a>(rgb: &RgbSlice<'a>, i: usize) -> [&'a DBlock; 3] {
    [&rgb[0][i], &rgb[1][i], &rgb[2][i]]
}

impl Dimension {
    pub fn new() -> Self {
        Dimension { wlist: [0; 5] }
    }

    pub fn image_width(&mut self, rgb_list: &RgbList, ycc_list: &YccList, chr: usize) -> i32 {
        let sub_img_size = (MAX_WIDTH * 3).min(rgb_list[0].len());
        let chr_size = sub_img_size / (1usize << ((chr + 1) / 2));
        let rgb_sub: RgbSlice = [
            &rgb_list[0][..sub_img_size.min(rgb_list[0].len())],
            &rgb_list[1][..sub_img_size.min(rgb_list[1].len())],
            &rgb_list[2][..sub_img_size.min(rgb_list[2].len())],
        ];
        let ycc_sub: [&[IBlock]; 3] = [
            &ycc_list[0][..sub_img_size.min(ycc_list[0].len())],
            &ycc_list[1][..chr_size.min(ycc_list[1].len())],
            &ycc_list[2][..chr_size.min(ycc_list[2].len())],
        ];

        let d_array = self.mcu_difference_array(&rgb_sub, chr);
        let w1 = self.method1(&d_array);
        let w2 = self.method2(&d_array);
        let w3 = self.method3(&rgb_sub, chr);
        let w4 = self.method4(&ycc_sub, chr);
        let w5 = self.method5(&rgb_sub, chr);
        let mut wfinal = self.final_width(rgb_list, chr, w1, w2, w3, w4, w5);

        self.wlist[0] = if w1 == 0 { 20 } else { w1 };
        self.wlist[1] = if w2 == 0 { 21 } else { w2 };
        self.wlist[2] = if w3 == 0 { 22 } else { w3 };
        self.wlist[3] = if w4 == 0 { 23 } else { w4 };
        self.wlist[4] = if w5 == 0 { 24 } else { w5 };
        wfinal = if wfinal < 20 { 25 } else { wfinal };
        wfinal
    }

    // ------------------------------------------------------------ MCU DIFFERENCE ARRAY
    fn mcu_difference_array(&self, rgb: &RgbSlice, chr: usize) -> Vec<f64> {
        let n = rgb[0].len() as i64;
        let mut out = Vec::new();
        match chr {
            0 => {
                let list_size = n - 1;
                for i in 0..list_size.max(0) as usize {
                    out.push(lr_diff_sum(blk(rgb, i), blk(rgb, i + 1)));
                }
            }
            1 => {
                let list_size = n / 2 - 1;
                let mut i = 0usize;
                while (i as i64) < 2 * list_size {
                    out.push(
                        (lr_diff_sum(blk(rgb, i), blk(rgb, i + 2)) + lr_diff_sum(blk(rgb, i + 1), blk(rgb, i + 3))) / 2.0,
                    );
                    i += 2;
                }
            }
            2 => {
                let list_size = n / 2 - 1;
                let mut i = 1usize;
                while (i as i64) < 2 * list_size {
                    out.push(lr_diff_sum(blk(rgb, i), blk(rgb, i + 1)));
                    i += 2;
                }
            }
            3 => {
                let list_size = n / 4 - 1;
                let mut i = 1usize;
                while (i as i64) < 4 * list_size {
                    out.push(
                        (lr_diff_sum(blk(rgb, i), blk(rgb, i + 3)) + lr_diff_sum(blk(rgb, i + 2), blk(rgb, i + 5))) / 2.0,
                    );
                    i += 4;
                }
            }
            _ => {}
        }
        out
    }

    // ------------------------------------------------------------ METHOD 1
    fn method1(&self, d_array: &[f64]) -> i32 {
        // Autocorrelation of mcu difference array
        let acorr = autocorr(d_array);
        let peaks = find_peaks_adaptive(&acorr);
        let diff = get_diff(&peaks, 4);
        mode(&diff)
    }

    // ------------------------------------------------------------ METHOD 2
    fn method2(&self, d_array: &[f64]) -> i32 {
        // 2-Means of mcu difference array
        let mut idx = kmeans::cluster(d_array, 2);
        let mut bound = get_bound(&idx);
        if bound.len() > 1000 {
            let sub_mean: Vec<f64> = bound.iter().map(|&b| d_array[b as usize]).collect();
            idx = kmeans::cluster(&sub_mean, 2);
            bound = get_bound(&idx);
        }
        let diff = get_diff(&bound, 1);
        mode(&diff)
    }

    // ------------------------------------------------------------ METHOD 3
    fn method3(&self, rgb: &RgbSlice, chr: usize) -> i32 {
        // Horizontal mask fit
        let mut w_cand: Vec<i32> = Vec::new();
        let mut w_temp: Vec<f64> = Vec::new();

        let mask_size: i64 = 40;
        let mut mask_offset: i64 = 4;
        let wind_size = (MAX_WIDTH as i64).min(rgb[0].len() as i64);

        match chr {
            0 => {
                for _ in 0..7 {
                    let m = pick3(rgb, mask_offset, 1, &[], mask_size);
                    let mut i = mask_offset + mask_size;
                    while i < wind_size - mask_size {
                        let s = pick3(rgb, i, 1, &[], mask_size);
                        w_temp.push(ub_diff_sum(&m, &s));
                        i += 1;
                    }
                    w_cand.push((mask_size + index_of_min(&w_temp) as i64) as i32);
                    w_temp.clear();
                    mask_offset += 1;
                }
            }
            1 => {
                for _ in 0..7 {
                    let m = pick3(rgb, mask_offset, 2, &[1], mask_size);
                    let mut i = mask_offset + mask_size;
                    while i < wind_size - mask_size {
                        let s = pick3(rgb, i, 2, &[0], mask_size);
                        w_temp.push(ub_diff_sum(&m, &s));
                        i += 2;
                    }
                    w_cand.push((mask_size / 2 + index_of_min(&w_temp) as i64) as i32);
                    w_temp.clear();
                    mask_offset += 2;
                }
            }
            2 => {
                for _ in 0..7 {
                    let m = pick3(rgb, mask_offset, 1, &[], mask_size);
                    let mut i = mask_offset + mask_size;
                    while i < wind_size - mask_size {
                        let s = pick3(rgb, i, 1, &[], mask_size);
                        w_temp.push(ub_diff_sum(&m, &s));
                        i += 2;
                    }
                    w_cand.push((mask_size / 2 + index_of_min(&w_temp) as i64) as i32);
                    w_temp.clear();
                    mask_offset += 2;
                }
            }
            3 => {
                for _ in 0..7 {
                    let m = pick3(rgb, mask_offset, 4, &[2, 3], mask_size);
                    let mut i = mask_offset + 2 * mask_size;
                    while i < wind_size - 2 * mask_size {
                        let s = pick3(rgb, i, 4, &[0, 1], mask_size);
                        w_temp.push(ub_diff_sum(&m, &s));
                        i += 4;
                    }
                    w_cand.push((mask_size / 2 + index_of_min(&w_temp) as i64) as i32);
                    w_temp.clear();
                    mask_offset += 4;
                }
            }
            _ => {}
        }
        mode(&w_cand)
    }

    // ------------------------------------------------------------ METHOD 4
    fn method4(&self, ycc: &[&[IBlock]; 3], chr: usize) -> i32 {
        // Autocorrelation of DC values
        let dc = |l: &[IBlock]| -> Vec<f64> { l.iter().map(|b| b[0][0] as f64).collect() };
        let y_array: Vec<f64> = if chr == 0 {
            dc(ycc[0])
        } else {
            // C#: Where((item, index) => index % 2 * ((chr + 1) / 2) == 0)  ==  ((index % 2) * ((chr+1)/2)) == 0
            dc(ycc[0])
                .into_iter()
                .enumerate()
                .filter(|(i, _)| (*i % 2) * ((chr + 1) / 2) == 0)
                .map(|(_, v)| v)
                .collect()
        };
        let cb_array = dc(ycc[1]);
        let cr_array = dc(ycc[2]);

        let mut diff: Vec<i32> = if chr == 3 {
            comp_diff(&y_array).into_iter().map(|x| x / 2).collect()
        } else {
            comp_diff(&y_array)
        };
        diff.extend(comp_diff(&cb_array));
        diff.extend(comp_diff(&cr_array));
        mode(&diff)
    }

    // ------------------------------------------------------------ METHOD 5
    fn method5(&self, rgb: &RgbSlice, chr: usize) -> i32 {
        // Autocorrelation of RGB horizontal border values
        let mut sub_diff: Vec<f64> = Vec::new();
        let list_size = rgb[0].len() as i64;
        let window_size = (MAX_WIDTH as i64).min(list_size);
        let min_width: i64 = 40;

        match chr {
            0 => {
                let mut i = min_width;
                while i < window_size {
                    let m = pick3(rgb, 0, 1, &[], list_size - i);
                    let s = pick3(rgb, i, 1, &[], ALL);
                    sub_diff.push(ub_diff_sum(&m, &s));
                    i += 1;
                }
                (index_of_min(&sub_diff) as i64 + min_width) as i32
            }
            1 => {
                let mut i = min_width;
                while i < window_size {
                    let m = pick3(rgb, 0, 2, &[1], list_size / 2 - i);
                    let s = pick3(rgb, 2 * i, 2, &[0], ALL);
                    sub_diff.push(ub_diff_sum(&m, &s));
                    i += 1;
                }
                (index_of_min(&sub_diff) as i64 + min_width) as i32
            }
            2 => {
                let mut i = min_width;
                while i < window_size {
                    let m = pick3(rgb, 0, 1, &[], list_size - i);
                    let s = pick3(rgb, i, 1, &[], ALL);
                    sub_diff.push(ub_diff_sum(&m, &s));
                    i += 2;
                }
                (index_of_min(&sub_diff) as i64 + min_width / 2) as i32
            }
            3 => {
                let mut i = min_width;
                while i < window_size {
                    let m = pick3(rgb, 0, 4, &[2, 3], list_size / 2 - i);
                    let s = pick3(rgb, 2 * i, 4, &[0, 1], ALL);
                    sub_diff.push(ub_diff_sum(&m, &s));
                    i += 2;
                }
                (index_of_min(&sub_diff) as i64 + min_width / 2) as i32
            }
            _ => 0,
        }
    }

    // ------------------------------------------------------------ FUSION OF 5 METHODS
    fn final_width(&self, rgb_list: &RgbList, chr: usize, w1: i32, w2: i32, w3: i32, w4: i32, w5: i32) -> i32 {
        let mut wlist = vec![w1, w2, w3, w4, w5];
        let md = mode(&wlist);
        // NOTE (kept from the original): `Select(item => item == mode).ToList().Count` counts ALL
        // elements, so `count` is always 5 and the grid-diff branch below is never reached.
        let count = wlist.iter().map(|&item| item == md).collect::<Vec<bool>>().len();
        if count > 2 {
            md
        } else {
            let mut modes: Vec<i32> = Vec::new();
            let mut grid_diffs: Vec<f64> = Vec::new();
            while !wlist.is_empty() {
                grid_diffs.push(grid_diff(rgb_list, chr, md));
                modes.push(md);
                wlist.retain(|&item| item != md);
            }
            modes[index_of_min(&grid_diffs)]
        }
    }
}

// ================================================================== helpers

fn lr_diff_sum(f: [&DBlock; 3], s: [&DBlock; 3]) -> f64 {
    let mut sum = 0.0;
    for i in 0..8 {
        sum += (sqr(f[0][i][7] - s[0][i][0]) + sqr(f[1][i][7] - s[1][i][0]) + sqr(f[2][i][7] - s[2][i][0])).sqrt();
    }
    sum / 8.0
}

/// Upper/bottom border difference between M (bottom row) and S (top row).
fn ub_diff_sum(m: &Ch, s: &Ch) -> f64 {
    let m_l = m[0].len();
    let mut cum = 0.0;
    for i in 0..m_l {
        for j in 0..8 {
            cum += (sqr(m[0][i][7][j] - s[0][i][0][j]) + sqr(m[1][i][7][j] - s[1][i][0][j]) + sqr(m[2][i][7][j] - s[2][i][0][j])).sqrt();
        }
    }
    cum / (m_l as f64 * 8.0)
}

fn vertical_grid_diff(m: &Ch, s: &Ch) -> f64 {
    let m_l = m[0].len().min(s[0].len());
    let mut cum = 0.0;
    for i in 0..m_l {
        for j in 0..8 {
            cum += (sqr(m[0][i][j][7] - s[0][i][j][0]) + sqr(m[1][i][j][7] - s[1][i][j][0]) + sqr(m[2][i][j][7] - s[2][i][j][0])).sqrt();
        }
    }
    cum / (m_l as f64 * 8.0)
}

fn get_bound(idx: &[usize]) -> Vec<i32> {
    let ones: usize = idx.iter().sum();
    let zeros = idx.len() - ones;
    let bindex = if ones < zeros { 1 } else { 0 };
    idx.iter().enumerate().filter(|(_, &v)| v == bindex).map(|(i, _)| i as i32).collect()
}

fn get_diff(array: &[i32], thr: i32) -> Vec<i32> {
    let mut diff = Vec::new();
    for i in 0..array.len() {
        for j in (i + 1)..array.len() {
            let d = array[j] - array[i];
            if d > thr {
                diff.push(d);
            }
        }
    }
    diff
}

fn comp_diff(d_array: &[f64]) -> Vec<i32> {
    let acorr = autocorr(d_array);
    let peaks = find_peaks_adaptive(&acorr);
    get_diff(&peaks, 4)
}

fn find_peaks_adaptive(acorr: &[f64]) -> Vec<i32> {
    let mut peaks = find_peaks(acorr, 4.0);
    if peaks.len() < 4 {
        peaks = find_peaks(acorr, 8.0);
    }
    if peaks.len() < 4 {
        peaks = find_peaks(acorr, 16.0);
    }
    peaks
}

/// Autocorrelation through FFT (MathNet "Matlab" scaling: forward unscaled, inverse 1/N).
fn autocorr(x: &[f64]) -> Vec<f64> {
    let n = x.len();
    if n == 0 {
        panic!("Array dimensions exceeded supported range.");
    }
    let len = 2 * n - 1;
    let mut cx: Vec<Complex<f64>> = x.iter().map(|&v| Complex::new(v, 0.0)).collect();
    cx.resize(len, Complex::new(0.0, 0.0));

    let mut planner = FftPlanner::<f64>::new();
    planner.plan_fft_forward(len).process(&mut cx);
    for c in cx.iter_mut() {
        *c = Complex::new(sqr(c.norm()), 0.0);
    }
    planner.plan_fft_inverse(len).process(&mut cx);
    let acorr: Vec<f64> = cx.iter().map(|c| (*c / len as f64).norm()).collect();

    // acorr.Skip(n).Concat(acorr.Take(n))
    let mut out = Vec::with_capacity(len);
    out.extend_from_slice(&acorr[n..]);
    out.extend_from_slice(&acorr[..n]);
    out
}

fn derivative(x: &[f64]) -> Vec<f64> {
    x.windows(2).map(|w| w[1] - w[0]).collect()
}

/// Peak finder (MATLAB peakfinder-like). `thresh_per`: sel = (max - min) / thresh_per
fn find_peaks(x: &[f64], thresh_per: f64) -> Vec<i32> {
    let mut peak_loc: Vec<i32> = Vec::new();

    let mut d_x = derivative(x);
    // so we find the first of repeated values
    for v in d_x.iter_mut() {
        if *v == 0.0 {
            *v = -2.2204e-16;
        }
    }

    // where the derivative changes sign
    let p_x_prod: Vec<f64> = d_x.windows(2).map(|w| w[0] * w[1]).collect();
    let mut ind: Vec<usize> = Vec::new();
    for i in 0..p_x_prod.len() {
        if p_x_prod[i] < 0.0 {
            ind.push(i + 1);
        }
    }

    // include endpoints
    ind.insert(0, 0);
    ind.push(x.len() - 1);
    let mut p_x: Vec<f64> = ind.iter().map(|&i| x[i]).collect();

    let xmax = x.iter().cloned().fold(f64::MIN, f64::max);
    let xmin = x.iter().cloned().fold(f64::MAX, f64::min);
    let sel = (xmax - xmin) / thresh_per;

    // p_x only has the peaks, valleys and start/end points
    let mut len = p_x.len() as i64;
    let min_mag = p_x.iter().cloned().fold(f64::MAX, f64::min);

    if len > 2 {
        let mut temp_mag = min_mag;
        let mut found_peak = false;
        let mut left_min = min_mag;

        // first point is handled differently because it was tacked on
        let d = derivative(&p_x[0..3]);
        if sign(d[0]) <= 0 {
            // first point is larger or equal to the second
            if sign(d[0]) == sign(d[1]) {
                p_x.remove(1);
                ind.remove(1);
                len -= 1;
            }
        } else if sign(d[0]) == sign(d[1]) {
            p_x.remove(0);
            ind.remove(0);
            len -= 1;
        }

        // skip the first point if it is smaller so we always start on a maxima
        let mut ii: i64 = if p_x[0] > p_x[1] { -1 } else { 0 };
        let mut temp_loc: usize = 0;

        while ii < len {
            ii += 1; // this is a peak
            if found_peak {
                temp_mag = min_mag;
                found_peak = false;
            }
            if ii == len {
                break;
            }
            // new peak larger than temp_mag and `sel` above the minimum to its left
            if p_x[ii as usize] > temp_mag && p_x[ii as usize] > left_min + sel {
                temp_loc = ii as usize;
                temp_mag = p_x[ii as usize];
            }

            ii += 1; // move onto the valley
            if ii == len {
                break;
            }
            // come down at least `sel` from peak
            if !found_peak && temp_mag > sel + p_x[ii as usize] {
                found_peak = true;
                left_min = p_x[ii as usize];
                peak_loc.push(ind[temp_loc] as i32);
            } else if p_x[ii as usize] < left_min {
                left_min = p_x[ii as usize];
            }
        }

        // check end point
        // (as in the C# original, ind[len] is out of range here and throws -> caught by callers)
        let last = *p_x.last().unwrap();
        if last > temp_mag && last > left_min + sel {
            peak_loc.push(ind[len as usize] as i32);
        } else if !found_peak && temp_mag > min_mag {
            peak_loc.push(ind[temp_loc] as i32);
        }
    }
    peak_loc
}

// ================================================================== grid diff (only reachable from the dead branch)
fn grid_diff(rgb_list: &RgbList, chr: usize, w: i32) -> f64 {
    let w = w as i64;
    let mcu_size: i64 = 1i64 << ((chr + 1) / 2);
    let list_size = w * mcu_size * (rgb_list[0].len() as i64 / (w * mcu_size));
    let rgb: RgbSlice = [&rgb_list[0], &rgb_list[1], &rgb_list[2]];

    let mut m: Ch = [Vec::new(), Vec::new(), Vec::new()];
    let mut s: Ch = [Vec::new(), Vec::new(), Vec::new()];

    let hor_diff;
    let ver_diff;

    // appends pick(...) for the 3 channels
    fn add<'a>(dst: &mut Ch<'a>, rgb: &RgbSlice<'a>, skip: i64, modulus: i64, keep: &[i64], take: i64) {
        let p = pick3(rgb, skip, modulus, keep, take);
        for c in 0..3 {
            dst[c].extend(p[c].iter().cloned());
        }
    }

    if chr % 2 == 0 {
        let step = w * mcu_size;
        m = pick3(&rgb, 0, 1, &[], list_size - step);
        s = pick3(&rgb, step, 1, &[], list_size);
        hor_diff = ub_diff_sum(&m, &s);

        m = pick3(&rgb, 0, 1, &[], list_size);
        s = pick3(&rgb, 0, 1, &[], list_size);
        for c in 0..3 {
            m[c] = m[c].iter().enumerate().filter(|(i, _)| (*i as i64) % step != step - 1).map(|(_, b)| *b).collect();
            s[c] = s[c].iter().enumerate().filter(|(i, _)| (*i as i64) % step != 0).map(|(_, b)| *b).collect();
        }
        ver_diff = vertical_grid_diff(&m, &s);
    } else if chr == 1 {
        let step = w * mcu_size;
        for i in 0..(list_size / step - 1) {
            add(&mut m, &rgb, i * step, 2, &[1], w);
            add(&mut m, &rgb, i * step, 2, &[0], w);
            add(&mut s, &rgb, (i + 1) * step, 2, &[1], w);
            add(&mut s, &rgb, (i + 1) * step, 2, &[0], w);
        }
        hor_diff = ub_diff_sum(&m, &s);
        m = [Vec::new(), Vec::new(), Vec::new()];
        s = [Vec::new(), Vec::new(), Vec::new()];
        for i in 0..(list_size / step) {
            add(&mut m, &rgb, i * step, 2, &[1], w - 1);
            add(&mut m, &rgb, i * step, 2, &[0], w - 1);
            add(&mut s, &rgb, i * step + mcu_size, 2, &[1], w - 1);
            add(&mut s, &rgb, i * step + mcu_size, 2, &[0], w - 1);
        }
        ver_diff = vertical_grid_diff(&m, &s);
    } else {
        let step = w * mcu_size;
        for i in 0..(list_size / step - 1) {
            add(&mut m, &rgb, i * step, 4, &[2, 3], 2 * w);
            add(&mut m, &rgb, i * step, 4, &[0, 1], 2 * w);
            add(&mut s, &rgb, (i + 1) * step, 4, &[2, 3], 2 * w);
            add(&mut s, &rgb, (i + 1) * step, 4, &[0, 1], 2 * w);
        }
        hor_diff = ub_diff_sum(&m, &s);
        m = [Vec::new(), Vec::new(), Vec::new()];
        s = [Vec::new(), Vec::new(), Vec::new()];
        for i in 0..(list_size / step) {
            add(&mut m, &rgb, i * step, 4, &[2, 3], 2 * w - 1);
            add(&mut m, &rgb, i * step, 4, &[0, 1], 2 * w - 1);
            add(&mut s, &rgb, i * step + 2, 4, &[2, 3], 2 * w);
            add(&mut s, &rgb, i * step + 2, 4, &[0, 1], 2 * w);
        }
        ver_diff = vertical_grid_diff(&m, &s);
    }

    (sqr(hor_diff) + sqr(ver_diff)).sqrt()
}
