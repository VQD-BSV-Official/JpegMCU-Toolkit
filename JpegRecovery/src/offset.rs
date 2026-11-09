use crate::util::*;

pub struct Offset;

type Ch<'a> = [Vec<&'a DBlock>; 3];

fn pick3<'a>(rgb: &'a RgbList, skip: i64, modulus: i64, keep: &[i64], take: i64) -> Ch<'a> {
    [
        pick(&rgb[0], skip, modulus, keep, take),
        pick(&rgb[1], skip, modulus, keep, take),
        pick(&rgb[2], skip, modulus, keep, take),
    ]
}

fn skip1<'a>(c: Ch<'a>) -> Ch<'a> {
    fn f<'a>(v: Vec<&'a DBlock>) -> Vec<&'a DBlock> {
        v.into_iter().skip(1).collect()
    }
    let [a, b, d] = c;
    [f(a), f(b), f(d)]
}

impl Offset {
    pub fn image_offset(&self, rgb: &RgbList, width: i32, chr: usize) -> i32 {
        let mut width = width as i64;
        width *= (chr as i64) / 2 + 1;
        let height = 50i64.min(rgb[0].len() as i64 / width);
        let mut ver_diff: Vec<f64> = Vec::new();

        if chr % 2 == 0 {
            // 4:4:4 || 4:2:2
            let iter = width - 1;
            let l = pick3(rgb, 0, width, &[width - 1], height - 1);
            let r = pick3(rgb, width, width, &[0], height - 1);
            ver_diff.push(lr_diff_mean(&l, &r));
            for i in 0..iter {
                let l = pick3(rgb, 0, width, &[i], height);
                let r = pick3(rgb, 0, width, &[i + 1], height);
                ver_diff.push(lr_diff_mean(&l, &r));
            }
            (width - index_of_max(&ver_diff) as i64) as i32
        } else if chr == 1 {
            // 4:4:0
            let iter = width - 2;
            let m = 2 * width;
            let l = pick3(rgb, 0, m, &[m - 2, m - 1], height - 1);
            let r = skip1(pick3(rgb, 0, m, &[0, 1], height));
            ver_diff.push(lr_diff_mean(&l, &r));
            for i in 0..iter {
                let l = pick3(rgb, 0, m, &[2 * i, 2 * i + 1], height);
                let r = pick3(rgb, 0, m, &[2 * (i + 1), 2 * (i + 1) + 1], height);
                ver_diff.push(lr_diff_mean(&l, &r));
            }
            (width - index_of_max(&ver_diff) as i64) as i32
        } else {
            // 4:2:0
            let iter = width - 2;
            let m = 2 * width;
            let l = pick3(rgb, 0, m, &[m - 3, m - 1], height - 1);
            let r = skip1(pick3(rgb, 0, m, &[0, 2], height));
            ver_diff.push(lr_diff_mean(&l, &r));
            for i in 0..iter {
                let li1 = 2 * i - i % 2;
                let li2 = 2 * (i + 1) - i % 2;
                let l = pick3(rgb, 0, m, &[li1, li2], height);
                let ri1 = 2 * (i + 1) - (i + 1) % 2;
                let ri2 = 2 * (i + 2) - (i + 1) % 2;
                let r = pick3(rgb, 0, m, &[ri1, ri2], height);
                ver_diff.push(lr_diff_mean(&l, &r));
            }
            (width - index_of_max(&ver_diff) as i64) as i32
        }
    }
}

fn lr_diff_mean(l: &Ch, r: &Ch) -> f64 {
    let m_l = l[0].len().min(r[0].len());
    let mut cum = 0.0;
    for i in 0..m_l {
        for j in 0..8 {
            cum += (sqr(l[0][i][j][7] - r[0][i][j][0]) + sqr(l[1][i][j][7] - r[1][i][j][0]) + sqr(l[2][i][j][7] - r[2][i][j][0])).sqrt();
        }
    }
    cum / (m_l as f64 * 8.0)
}
