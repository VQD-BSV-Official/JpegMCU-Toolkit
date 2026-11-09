use crate::util::*;

/// 1-D k-means. Returns the cluster index of every element.
pub fn cluster(raw_data: &[f64], num_clusters: usize) -> Vec<usize> {
    let data = normalized(raw_data);
    let mut changed = true;
    let mut success = true;

    let mut clustering = init_clustering(data.len(), num_clusters, 0);
    let mut means = vec![0.0; num_clusters];

    let max_count = data.len() * 10;
    let mut ct = 0;
    while changed && success && ct < max_count {
        ct += 1;
        success = update_means(&data, &clustering, &mut means);
        changed = update_clustering(&data, &mut clustering, &means);
    }
    clustering
}

fn normalized(raw: &[f64]) -> Vec<f64> {
    let mean = average(raw);
    // note: the original divides by the variance (not the standard deviation)
    let sd = raw.iter().map(|&i| (i - mean) * (i - mean)).sum::<f64>() / raw.len() as f64;
    raw.iter().map(|&x| (x - mean) / sd).collect()
}

fn init_clustering(num_tuples: usize, num_clusters: usize, seed: i32) -> Vec<usize> {
    let mut random = DotNetRandom::new(seed);
    let mut clustering = vec![0usize; num_tuples];
    for i in 0..num_clusters {
        clustering[i] = i; // panics if fewer tuples than clusters (as in C#)
    }
    for i in num_clusters..num_tuples {
        clustering[i] = random.next_range(0, num_clusters as i32) as usize;
    }
    clustering
}

fn update_means(data: &[f64], clustering: &[usize], means: &mut [f64]) -> bool {
    let num_clusters = means.len();
    let mut counts = vec![0usize; num_clusters];
    for i in 0..data.len() {
        counts[clustering[i]] += 1;
    }
    for k in 0..num_clusters {
        if counts[k] == 0 {
            return false;
        }
    }
    for m in means.iter_mut() {
        *m = 0.0;
    }
    for i in 0..data.len() {
        means[clustering[i]] += data[i];
    }
    for k in 0..num_clusters {
        means[k] /= counts[k] as f64;
    }
    true
}

fn update_clustering(data: &[f64], clustering: &mut [usize], means: &[f64]) -> bool {
    let num_clusters = means.len();
    let mut changed = false;
    let mut new_clustering = clustering.to_vec();
    let mut distances = vec![0.0; num_clusters];

    for i in 0..data.len() {
        for k in 0..num_clusters {
            distances[k] = (data[i] - means[k]).abs();
        }
        let new_id = index_of_min(&distances);
        if new_id != new_clustering[i] {
            changed = true;
            new_clustering[i] = new_id;
        }
    }
    if !changed {
        return false;
    }
    let mut counts = vec![0usize; num_clusters];
    for i in 0..data.len() {
        counts[new_clustering[i]] += 1;
    }
    for k in 0..num_clusters {
        if counts[k] == 0 {
            return false;
        }
    }
    clustering.copy_from_slice(&new_clustering);
    true
}
