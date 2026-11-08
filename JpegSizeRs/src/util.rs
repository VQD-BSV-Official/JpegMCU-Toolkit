//! Helpers that replace .NET behaviours the original code relied on.

use std::sync::atomic::{AtomicUsize, Ordering};

// ---------------------------------------------------------------------------
// Block types (C#: int[][] / double[][] with 8x8 shape)
// ---------------------------------------------------------------------------
pub type IBlock = [[i32; 8]; 8];
pub type DBlock = [[f64; 8]; 8];
pub type YccList = [Vec<IBlock>; 3];
pub type RgbList = [Vec<DBlock>; 3];

// ---------------------------------------------------------------------------
// FileStream-like byte reader (ReadByte returns -1 at EOF, Position settable)
// ---------------------------------------------------------------------------
pub struct ByteStream { data: Vec<u8>, pos: i64 }

impl ByteStream {
    pub fn open(path: &str) -> std::io::Result<Self> { Ok(Self::from_vec(std::fs::read(path)?)) }
    pub fn from_vec(data: Vec<u8>) -> Self { ByteStream { data, pos: 0 } }
    pub fn length(&self) -> i64 { self.data.len() as i64 }
    pub fn position(&self) -> i64 { self.pos }
    pub fn set_position(&mut self, p: i64) {
        if p < 0 { panic!("Non-negative number required. (Parameter 'value')"); }
        self.pos = p;
    }
    pub fn read_byte(&mut self) -> i32 {
        match self.data.get(self.pos as usize) {
            Some(&b) => { self.pos += 1; b as i32 }
            None => -1,
        }
    }
    pub fn slice(&self, a: i64, b: i64) -> &[u8] {
        let (a, b) = (a.max(0) as usize, (b.max(0) as usize).min(self.data.len()));
        if a < b { &self.data[a..b] } else { &[] }
    }
}

// ---------------------------------------------------------------------------
// Math / LINQ-like helpers
// ---------------------------------------------------------------------------
pub fn sqr(x: f64) -> f64 {
    x * x
}

/// Math.Round(double) in .NET: banker's rounding (ties to even).
pub fn round_even(x: f64) -> f64 {
    if (x - x.trunc()).abs() == 0.5 {
        2.0 * (x / 2.0).round()
    } else {
        x.round()
    }
}

pub fn sign(x: f64) -> i32 {
    if x.is_nan() {
        panic!("Function does not accept floating point Not-a-Number values.");
    }
    if x > 0.0 { 1 } else if x < 0.0 { -1 } else { 0 }
}

pub fn average(v: &[f64]) -> f64 {
    if v.is_empty() {
        panic!("Sequence contains no elements");
    }
    v.iter().sum::<f64>() / v.len() as f64
}

/// Array.IndexOf(v, v.Min()) with .NET NaN semantics (NaN propagates, NaN.Equals(NaN) is true).
pub fn index_of_min(v: &[f64]) -> usize {
    if v.is_empty() {
        panic!("Sequence contains no elements");
    }
    if let Some(i) = v.iter().position(|x| x.is_nan()) {
        return i;
    }
    let mut best = 0;
    for i in 1..v.len() {
        if v[i] < v[best] {
            best = i;
        }
    }
    best
}

/// Array.IndexOf(v, v.Max()) (NaN is ordered below everything, as in Enumerable.Max).
pub fn index_of_max(v: &[f64]) -> usize {
    if v.is_empty() {
        panic!("Sequence contains no elements");
    }
    let mut best: Option<usize> = None;
    for i in 0..v.len() {
        if v[i].is_nan() {
            continue;
        }
        match best {
            None => best = Some(i),
            Some(b) => {
                if v[i] > v[b] {
                    best = Some(i)
                }
            }
        }
    }
    best.unwrap_or(0)
}

/// The "Mode<T>" helper: most frequent element, first-inserted wins on ties, default for empty.
pub fn mode<T: Copy + PartialEq + Default>(list: &[T]) -> T {
    let mut counts: Vec<(T, usize)> = Vec::new();
    for &e in list {
        if let Some(c) = counts.iter_mut().find(|(k, _)| *k == e) {
            c.1 += 1;
        } else {
            counts.push((e, 1));
        }
    }
    let mut m = T::default();
    let mut max = 0;
    for (k, c) in counts {
        if c > max {
            m = k;
            max = c;
        }
    }
    m
}

// ---------------------------------------------------------------------------
// System.Random (legacy Knuth subtractive generator, seeded) - bit-exact with .NET
// ---------------------------------------------------------------------------
pub struct DotNetRandom {
    seed_array: [i32; 56],
    inext: usize,
    inextp: usize,
}

impl DotNetRandom {
    const MBIG: i32 = i32::MAX;
    const MSEED: i32 = 161803398;

    pub fn new(seed: i32) -> Self {
        let mut sa = [0i32; 56];
        let subtraction = if seed == i32::MIN { i32::MAX } else { seed.abs() };
        let mut mj = Self::MSEED - subtraction;
        sa[55] = mj;
        let mut mk = 1i32;
        for i in 1..55 {
            let ii = (21 * i) % 55;
            sa[ii] = mk;
            mk = mj - mk;
            if mk < 0 {
                mk += Self::MBIG;
            }
            mj = sa[ii];
        }
        for _k in 1..5 {
            for i in 1..56 {
                sa[i] = sa[i].wrapping_sub(sa[1 + (i + 30) % 55]);
                if sa[i] < 0 {
                    sa[i] += Self::MBIG;
                }
            }
        }
        DotNetRandom { seed_array: sa, inext: 0, inextp: 21 }
    }

    fn internal_sample(&mut self) -> i32 {
        let mut a = self.inext + 1;
        let mut b = self.inextp + 1;
        if a >= 56 {
            a = 1;
        }
        if b >= 56 {
            b = 1;
        }
        let mut r = self.seed_array[a].wrapping_sub(self.seed_array[b]);
        if r == Self::MBIG {
            r -= 1;
        }
        if r < 0 {
            r += Self::MBIG;
        }
        self.seed_array[a] = r;
        self.inext = a;
        self.inextp = b;
        r
    }

    /// Random.Next(minValue, maxValue)
    pub fn next_range(&mut self, min: i32, max: i32) -> i32 {
        let range = max as i64 - min as i64;
        let sample = self.internal_sample() as f64 * (1.0 / Self::MBIG as f64);
        (sample * range as f64) as i32 + min
    }
}

// ---------------------------------------------------------------------------
// try { ... } catch (Exception e) { ... } emulation
// ---------------------------------------------------------------------------
static TRY_DEPTH: AtomicUsize = AtomicUsize::new(0);

/// Install a panic hook that stays quiet while inside `try_catch`
/// (the C# code caught and printed exceptions; panics play that role here).
pub fn install_panic_hook() {
    let default = std::panic::take_hook();
    std::panic::set_hook(Box::new(move |info| {
        if TRY_DEPTH.load(Ordering::SeqCst) == 0 {
            default(info);
        }
    }));
}

pub fn try_catch<T>(f: impl FnOnce() -> T) -> Result<T, String> {
    TRY_DEPTH.fetch_add(1, Ordering::SeqCst);
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(f));
    TRY_DEPTH.fetch_sub(1, Ordering::SeqCst);
    r.map_err(|e| {
        if let Some(s) = e.downcast_ref::<&str>() {
            s.to_string()
        } else if let Some(s) = e.downcast_ref::<String>() {
            s.clone()
        } else {
            "Unknown error".to_string()
        }
    })
}

// ---------------------------------------------------------------------------
// Skip/Where(index % m in keep)/Take  (LINQ chains used all over the original)
// ---------------------------------------------------------------------------
/// `list.Skip(skip).Where((_, index) => keep.Contains(index % modulus)).Take(take)`.
/// An empty `keep` means "no Where". Negative skip/take behave like LINQ (no-op / empty).
pub fn pick<'a>(list: &'a [DBlock], skip: i64, modulus: i64, keep: &[i64], take: i64) -> Vec<&'a DBlock> {
    let skip = skip.max(0) as usize;
    let take = take.max(0) as usize;
    list.iter()
        .skip(skip)
        .enumerate()
        .filter(|(i, _)| keep.is_empty() || keep.contains(&((*i as i64) % modulus)))
        .map(|(_, b)| b)
        .take(take)
        .collect()
}

pub const ALL: i64 = i64::MAX;
