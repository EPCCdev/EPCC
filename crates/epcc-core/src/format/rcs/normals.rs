//! Normals: the 16-bit code points store them in, and their estimation from the points.
//!
//! A code names a cell on the surface of a cube: `face * 2704 + i * 52 + j`, faces +x, -x, +y,
//! -y, +z, -z, and `j`, `i` the cell along the two remaining axes in ascending order. The cells
//! are equal angles rather than equal lengths. Code 0 means "no normal".

use std::f64::consts::FRAC_PI_4;
use std::sync::OnceLock;

use rayon::prelude::*;

const CELLS: usize = 52;
const FACE_CODES: usize = CELLS * CELLS;
const CODES: usize = 6 * FACE_CODES;
/// The encoder's cells are slightly wider than the decoder's, scaled by this about the face
/// centre. Matches the codes consuming applications expect for given directions.
const ENCODE_SCALE: f64 = 52.0 / 51.8;

/// Leaves storing fewer points get no estimated normals.
pub const MIN_POINTS: usize = 9;
/// The neighbourhood a normal is estimated from, the point itself included.
pub const NEIGHBOURS: usize = 16;

/// The two axes besides `axis`, in ascending order.
fn other_axes(axis: usize) -> (usize, usize) {
    match axis {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    }
}

/// The code of a direction. Zero-length and non-finite vectors give 0, "no normal".
pub fn encode(normal: [f64; 3]) -> u16 {
    if !normal.iter().all(|value| value.is_finite()) {
        return 0;
    }
    // The first of equally long components wins.
    let mut axis = 0;
    for candidate in 1..3 {
        if normal[candidate].abs() > normal[axis].abs() {
            axis = candidate;
        }
    }
    let main = normal[axis];
    if main == 0.0 {
        return 0;
    }
    let face = 2 * axis + usize::from(main < 0.0);
    let (first, second) = other_axes(axis);
    let cell = |value: f64| {
        let half = (CELLS / 2) as f64;
        let angle = (value / main.abs()).atan();
        (half + angle / FRAC_PI_4 * half / ENCODE_SCALE).floor().clamp(0.0, (CELLS - 1) as f64) as usize
    };
    (face * FACE_CODES + cell(normal[second]) * CELLS + cell(normal[first])) as u16
}

/// The unit vector of a code; 0 and codes beyond the last cell give the zero vector.
pub fn decode(code: u16) -> [f32; 3] {
    static TABLE: OnceLock<Vec<[f32; 3]>> = OnceLock::new();
    let table = TABLE.get_or_init(|| (0..CODES).map(decode_cell).collect());
    match usize::from(code) {
        0 => [0.0; 3],
        code if code < CODES => table[code],
        _ => [0.0; 3],
    }
}

fn decode_cell(code: usize) -> [f32; 3] {
    let (face, rest) = (code / FACE_CODES, code % FACE_CODES);
    let (i, j) = (rest / CELLS, rest % CELLS);
    let axis = face / 2;
    let (first, second) = other_axes(axis);
    let warp = |k: usize| ((2.0 * (k as f64 + 0.5) / CELLS as f64 - 1.0) * std::f64::consts::PI / 4.0).tan();
    let mut vector = [0.0; 3];
    vector[axis] = if face % 2 == 0 { 1.0 } else { -1.0 };
    vector[first] = warp(j);
    vector[second] = warp(i);
    let length = (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt();
    vector.map(|value| (value / length) as f32)
}

/// Normal codes for the stored points of one leaf, in their order. Leaves with fewer than
/// [`MIN_POINTS`] points get code 0 throughout.
pub fn estimate_codes(points: &[[f64; 3]]) -> Vec<u16> {
    let count = points.len();
    if count < MIN_POINTS {
        return vec![0; count];
    }
    // Centring keeps the arithmetic well conditioned and changes no covariance.
    let mut mean = [0.0; 3];
    for point in points {
        for axis in 0..3 {
            mean[axis] += point[axis];
        }
    }
    let mean = mean.map(|sum| sum / count as f64);
    let centred: Vec<[f64; 3]> =
        points.iter().map(|point| [point[0] - mean[0], point[1] - mean[1], point[2] - mean[2]]).collect();

    if count <= NEIGHBOURS {
        let all: Vec<u32> = (0..count as u32).collect();
        let code = encode(smallest_eigenvector(covariance(&centred, &all)));
        return vec![code; count];
    }
    let tree = KdTree::build(&centred);
    centred
        .par_iter()
        .map_init(Vec::new, |neighbours, point| {
            tree.nearest(*point, NEIGHBOURS, neighbours);
            encode(smallest_eigenvector(covariance(&centred, neighbours)))
        })
        .collect()
}

/// The covariance of the points picked by `indices`, centred on their mean and divided by
/// their number.
fn covariance(points: &[[f64; 3]], indices: &[u32]) -> [[f64; 3]; 3] {
    let count = indices.len() as f64;
    let mut mean = [0.0; 3];
    for &index in indices {
        for axis in 0..3 {
            mean[axis] += points[index as usize][axis];
        }
    }
    let mean = mean.map(|sum| sum / count);
    let mut sums = [[0.0; 3]; 3];
    for &index in indices {
        let point = points[index as usize];
        let d = [point[0] - mean[0], point[1] - mean[1], point[2] - mean[2]];
        for row in 0..3 {
            for column in 0..3 {
                sums[row][column] += d[row] * d[column];
            }
        }
    }
    sums.map(|row| row.map(|sum| sum / count))
}

/// The smallest eigenvalue of a symmetric 3×3 matrix by the closed-form cubic. Gives 0 when
/// the determinant vanishes or the smallest root is not positive.
fn smallest_root(m: &[[f64; 3]; 3]) -> f64 {
    let (a, b, c) = (m[0][0], m[1][1], m[2][2]);
    let (d, e, f) = (m[0][1], m[0][2], m[1][2]);
    let c0 = a * b * c + 2.0 * d * e * f - a * f * f - b * e * e - c * d * d;
    let c1 = a * b - d * d + a * c - e * e + b * c - f * f;
    let c2 = a + b + c;
    let c2_over_3 = c2 / 3.0;
    let a_over_3 = ((c1 - c2 * c2_over_3) / 3.0).min(0.0);
    let half_b = 0.5 * (c0 + c2_over_3 * (2.0 * c2_over_3 * c2_over_3 - c1));
    let q = (half_b * half_b + a_over_3.powf(3.0)).min(0.0);
    let rho = (-a_over_3).sqrt();
    let theta = (-q).sqrt().atan2(half_b) / 3.0;
    let (cos, sin) = (theta.cos(), theta.sin());
    let sqrt3 = 3.0f64.sqrt();
    let roots =
        [c2_over_3 + 2.0 * rho * cos, c2_over_3 - rho * (cos + sqrt3 * sin), c2_over_3 - rho * (cos - sqrt3 * sin)];
    let smallest = roots[0].min(roots[1]).min(roots[2]);
    if c0.abs() < f64::EPSILON || smallest <= 0.0 { 0.0 } else { smallest }
}

/// The unit eigenvector of the smallest eigenvalue, by cross products of the rows of the
/// shifted matrix; its sign is whatever that gives. NaN where it is undefined.
fn smallest_eigenvector(covariance: [[f64; 3]; 3]) -> [f64; 3] {
    let mut scale = covariance.iter().flatten().fold(0.0f64, |max, value| max.max(value.abs()));
    if scale == 0.0 {
        scale = 1.0;
    }
    let mut s = covariance.map(|row| row.map(|value| value / scale));
    let root = smallest_root(&s);
    for (axis, row) in s.iter_mut().enumerate() {
        row[axis] -= root;
    }
    let cross =
        |u: [f64; 3], v: [f64; 3]| [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
    let candidates = [cross(s[0], s[1]), cross(s[0], s[2]), cross(s[1], s[2])];
    let lengths = candidates.map(|v| v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    // The first of the longest.
    let best = if lengths[0] >= lengths[1] && lengths[0] >= lengths[2] {
        0
    } else if lengths[1] >= lengths[2] {
        1
    } else {
        2
    };
    let norm = lengths[best].sqrt();
    if norm == 0.0 {
        return [f64::NAN; 3];
    }
    candidates[best].map(|value| value / norm)
}

/// Points per leaf of the k-d tree.
const KD_LEAF_POINTS: usize = 8;

/// A k-d tree for nearest-neighbour queries.
struct KdTree<'a> {
    points: &'a [[f64; 3]],
    /// Point indices, arranged so that every node covers a contiguous range.
    order: Vec<u32>,
    nodes: Vec<KdNode>,
}

enum KdNode {
    Leaf {
        start: usize,
        end: usize,
    },
    /// Points up to `split` along `axis` are on the low side.
    Inner {
        axis: usize,
        split: f64,
        low: usize,
        high: usize,
    },
}

impl<'a> KdTree<'a> {
    fn build(points: &'a [[f64; 3]]) -> Self {
        let mut tree = KdTree { points, order: (0..points.len() as u32).collect(), nodes: Vec::new() };
        tree.build_node(0, points.len());
        tree
    }

    /// Builds the node covering `order[start..end]` and returns its index.
    fn build_node(&mut self, start: usize, end: usize) -> usize {
        let index = self.nodes.len();
        if end - start <= KD_LEAF_POINTS {
            self.nodes.push(KdNode::Leaf { start, end });
            return index;
        }
        let points = self.points;
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for &point in &self.order[start..end] {
            for axis in 0..3 {
                low[axis] = low[axis].min(points[point as usize][axis]);
                high[axis] = high[axis].max(points[point as usize][axis]);
            }
        }
        let axis = (0..3).max_by(|&a, &b| (high[a] - low[a]).total_cmp(&(high[b] - low[b]))).unwrap_or(0);
        let middle = start + (end - start) / 2;
        self.order[start..end].select_nth_unstable_by(middle - start, |&a, &b| {
            points[a as usize][axis].total_cmp(&points[b as usize][axis]).then(a.cmp(&b))
        });
        let split = points[self.order[middle] as usize][axis];
        self.nodes.push(KdNode::Leaf { start, end }); // replaced below
        let low_node = self.build_node(start, middle);
        let high_node = self.build_node(middle, end);
        self.nodes[index] = KdNode::Inner { axis, split, low: low_node, high: high_node };
        index
    }

    /// Fills `result` with the indices of the `k` points nearest to `query`, nearest first.
    /// Equally distant points are taken in index order.
    fn nearest(&self, query: [f64; 3], k: usize, result: &mut Vec<u32>) {
        let mut best: Vec<(f64, u32)> = Vec::with_capacity(k + 1);
        self.search(0, query, k, &mut best);
        result.clear();
        result.extend(best.iter().map(|&(_, index)| index));
    }

    fn search(&self, node: usize, query: [f64; 3], k: usize, best: &mut Vec<(f64, u32)>) {
        match self.nodes[node] {
            KdNode::Leaf { start, end } => {
                for &index in &self.order[start..end] {
                    let point = self.points[index as usize];
                    let d = [point[0] - query[0], point[1] - query[1], point[2] - query[2]];
                    let distance = d[0] * d[0] + d[1] * d[1] + d[2] * d[2];
                    let candidate = (distance, index);
                    let closer =
                        |other: &(f64, u32)| candidate.0 < other.0 || (candidate.0 == other.0 && index < other.1);
                    if best.len() == k && !best.last().is_some_and(closer) {
                        continue;
                    }
                    let at = best.iter().position(closer).unwrap_or(best.len());
                    best.insert(at, candidate);
                    best.truncate(k);
                }
            }
            KdNode::Inner { axis, split, low, high } => {
                let offset = query[axis] - split;
                let (near, far) = if offset <= 0.0 { (low, high) } else { (high, low) };
                self.search(near, query, k, best);
                if best.len() < k || offset * offset <= best[best.len() - 1].0 {
                    self.search(far, query, k, best);
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn codes_round_trip_through_their_cells() {
        for code in 1..CODES as u16 {
            let normal = decode(code).map(f64::from);
            assert_eq!(encode(normal), code, "{normal:?}");
        }
        assert_eq!(decode(0), [0.0; 3]);
        assert_eq!(encode([0.0; 3]), 0);
        assert_eq!(encode([f64::NAN, 0.0, 1.0]), 0);
    }

    #[test]
    fn faces_follow_the_main_axis() {
        assert_eq!(encode([0.0, 0.0, 1.0]) as usize / FACE_CODES, 4);
        assert_eq!(encode([0.0, -1.0, 0.0]) as usize / FACE_CODES, 3);
        let up = decode(encode([0.0, 0.0, 1.0]));
        assert!(up[2] > 0.999, "{up:?}");
    }

    #[test]
    fn planes_get_their_normal() {
        // A tilted plane z = 0.5 x with some spread in y.
        let points: Vec<[f64; 3]> = (0..400)
            .map(|i| {
                let (x, y) = ((i % 20) as f64 * 0.01, (i / 20) as f64 * 0.013);
                [x, y, 0.5 * x]
            })
            .collect();
        let want = [-0.5, 0.0, 1.0].map(|v: f64| v / 1.25f64.sqrt());
        for code in estimate_codes(&points) {
            let got = decode(code);
            let dot: f64 = (0..3).map(|axis| f64::from(got[axis]) * want[axis]).sum();
            assert!(dot.abs() > 0.995, "{got:?}");
        }
    }

    #[test]
    fn small_leaves_get_no_normals() {
        let points: Vec<[f64; 3]> = (0..8).map(|i| [i as f64, (i * i) as f64, 0.0]).collect();
        assert_eq!(estimate_codes(&points), vec![0; 8]);
    }

    #[test]
    fn nearest_neighbours_match_brute_force() {
        let mut state = 12345u64;
        let mut random = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            (state % 10_000) as f64 / 1000.0
        };
        let points: Vec<[f64; 3]> = (0..2000).map(|_| [random(), random(), random()]).collect();
        let tree = KdTree::build(&points);
        let mut found = Vec::new();
        for query in points.iter().step_by(37) {
            tree.nearest(*query, NEIGHBOURS, &mut found);
            let mut all: Vec<(f64, u32)> = points
                .iter()
                .enumerate()
                .map(|(index, p)| {
                    let d = [p[0] - query[0], p[1] - query[1], p[2] - query[2]];
                    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2], index as u32)
                })
                .collect();
            all.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
            let want: Vec<u32> = all[..NEIGHBOURS].iter().map(|&(_, index)| index).collect();
            assert_eq!(found, want);
        }
    }
}
