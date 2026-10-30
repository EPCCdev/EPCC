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

/// Leaves storing fewer points get no estimated normals. A normal is estimated from the smallest
/// cube around its point that holds at least this many points, the point itself included.
pub const MIN_POINTS: usize = 9;
/// The smallest such cube has a half edge of the leaf's edge halved this many times.
const MAX_HALVINGS: i32 = 16;

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
    encode_on_face(normal, axis)
}

/// The code of an estimated normal. Its components are compared in single precision, and of
/// equally long ones y wins over x and x over z, as in the consuming applications' own files.
fn encode_estimated(normal: [f64; 3]) -> u16 {
    if !normal.iter().all(|value| value.is_finite()) {
        return 0;
    }
    let length = normal.map(|value| (value as f32).abs());
    let axis = if length[1] >= length[0] && length[1] >= length[2] {
        1
    } else if length[0] >= length[2] {
        0
    } else {
        2
    };
    encode_on_face(normal, axis)
}

/// The code of a direction on the faces of `axis`.
fn encode_on_face(normal: [f64; 3], axis: usize) -> u16 {
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
///
/// Each point's normal comes from the points in the smallest cube centred on it, with a half
/// edge of `edge` (the leaf's) halved up to [`MAX_HALVINGS`] times, that holds [`MIN_POINTS`]
/// of them; points on its boundary count. The positions must be the source's own: points on a
/// boundary would fall in or out under any shift.
pub fn estimate_codes(points: &[[f64; 3]], edge: f64) -> Vec<u16> {
    if points.len() < MIN_POINTS {
        return vec![0; points.len()];
    }
    let tree = KdTree::build(points);
    points
        .par_iter()
        .map_init(Vec::new, |inside, &point| {
            let half_edge = cube_half_edge(tree.kth_distance(point, MIN_POINTS), edge);
            tree.within(point, half_edge, inside);
            inside.sort_unstable();
            encode_estimated(smallest_eigenvector(covariance(points, inside)))
        })
        .collect()
}

/// The half edge `edge / 2^j` of the smallest cube, `j` up to [`MAX_HALVINGS`], that reaches
/// `distance` (the largest coordinate difference to a point).
fn cube_half_edge(distance: f64, edge: f64) -> f64 {
    let mut halvings = if distance > 0.0 {
        (edge / distance).log2().floor().clamp(0.0, f64::from(MAX_HALVINGS)) as i32
    } else {
        MAX_HALVINGS
    };
    // The logarithm may round across a power of two; settle on the exact comparison.
    for _ in 0..2 {
        if halvings < MAX_HALVINGS && edge / 2f64.powi(halvings + 1) >= distance {
            halvings += 1;
        }
        if halvings > 0 && edge / 2f64.powi(halvings) < distance {
            halvings -= 1;
        }
    }
    edge / 2f64.powi(halvings)
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

/// A k-d tree for neighbour queries by the largest coordinate difference.
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

    /// The distance from `query` to its `k`-th nearest point, the query itself counted when it is
    /// one of the points. Distances are the largest coordinate difference.
    fn kth_distance(&self, query: [f64; 3], k: usize) -> f64 {
        let mut best: Vec<f64> = Vec::with_capacity(k + 1);
        self.search_nearest(0, query, k, &mut best);
        best.last().copied().unwrap_or(f64::INFINITY)
    }

    fn search_nearest(&self, node: usize, query: [f64; 3], k: usize, best: &mut Vec<f64>) {
        match self.nodes[node] {
            KdNode::Leaf { start, end } => {
                for &index in &self.order[start..end] {
                    let distance = distance(self.points[index as usize], query);
                    if best.len() == k && distance >= best[k - 1] {
                        continue;
                    }
                    let at = best.partition_point(|&other| other <= distance);
                    best.insert(at, distance);
                    best.truncate(k);
                }
            }
            KdNode::Inner { axis, split, low, high } => {
                let offset = query[axis] - split;
                let (near, far) = if offset <= 0.0 { (low, high) } else { (high, low) };
                self.search_nearest(near, query, k, best);
                if best.len() < k || offset.abs() <= best[k - 1] {
                    self.search_nearest(far, query, k, best);
                }
            }
        }
    }

    /// Fills `result` with the indices of the points at most `reach` from `query`, in no
    /// particular order.
    fn within(&self, query: [f64; 3], reach: f64, result: &mut Vec<u32>) {
        result.clear();
        self.search_within(0, query, reach, result);
    }

    fn search_within(&self, node: usize, query: [f64; 3], reach: f64, result: &mut Vec<u32>) {
        match self.nodes[node] {
            KdNode::Leaf { start, end } => {
                let inside = |&&index: &&u32| distance(self.points[index as usize], query) <= reach;
                result.extend(self.order[start..end].iter().filter(inside));
            }
            KdNode::Inner { axis, split, low, high } => {
                let offset = query[axis] - split;
                if offset <= reach {
                    self.search_within(low, query, reach, result);
                }
                if -offset <= reach {
                    self.search_within(high, query, reach, result);
                }
            }
        }
    }
}

/// The largest coordinate difference between two points.
fn distance(a: [f64; 3], b: [f64; 3]) -> f64 {
    (a[0] - b[0]).abs().max((a[1] - b[1]).abs()).max((a[2] - b[2]).abs())
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
    fn estimated_normals_prefer_y_then_x_on_ties() {
        let face = |code: u16| code as usize / FACE_CODES;
        let ties = [[1.0, 1.0, 0.0], [1.0, 0.0, 1.0], [0.0, 1.0, 1.0], [1.0, 1.0, 1.0]];
        assert_eq!(ties.map(|v| face(encode(v))), [0, 0, 2, 0]);
        assert_eq!(ties.map(|v| face(encode_estimated(v))), [2, 0, 2, 2]);
        // Equal in single precision is a tie too.
        assert_eq!(face(encode_estimated([1.0 + 1e-12, 1.0, 0.0])), 2);
        assert_eq!(encode_estimated([1.0, 0.5, 0.2]), encode([1.0, 0.5, 0.2]));
    }

    #[test]
    fn the_cube_is_the_smallest_holding_enough_points() {
        let edge = 0.16;
        // Nine points within 16 mm of the first one: the 20 mm cube holds them.
        let mut points: Vec<[f64; 3]> = vec![[0.08, 0.08, 0.05]];
        points.extend((1..9).map(|step| [0.08 + step as f64 * 0.002, 0.08, 0.05]));
        points.push([0.15, 0.15, 0.05]);
        let tree = KdTree::build(&points);
        assert_eq!(cube_half_edge(tree.kth_distance(points[0], MIN_POINTS), edge), edge / 8.0);
        points[8][0] = 0.08 + 0.0205;
        let tree = KdTree::build(&points);
        assert_eq!(cube_half_edge(tree.kth_distance(points[0], MIN_POINTS), edge), edge / 4.0);
        // Exactly on a power of two, and the limits.
        assert_eq!(cube_half_edge(0.02, edge), 0.02);
        assert_eq!(cube_half_edge(0.0, edge), edge / 65536.0);
        assert_eq!(cube_half_edge(1.0, edge), edge);
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
        for code in estimate_codes(&points, 0.5) {
            let got = decode(code);
            let dot: f64 = (0..3).map(|axis| f64::from(got[axis]) * want[axis]).sum();
            assert!(dot.abs() > 0.995, "{got:?}");
        }
    }

    #[test]
    fn small_leaves_get_no_normals() {
        let points: Vec<[f64; 3]> = (0..8).map(|i| [i as f64, (i * i) as f64, 0.0]).collect();
        assert_eq!(estimate_codes(&points, 64.0), vec![0; 8]);
    }

    #[test]
    fn neighbour_queries_match_brute_force() {
        let mut state = 12345u64;
        let mut random = || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            // A coarse grid, so that many distances tie.
            (state % 200) as f64 / 100.0
        };
        let points: Vec<[f64; 3]> = (0..2000).map(|_| [random(), random(), random()]).collect();
        let tree = KdTree::build(&points);
        let mut found = Vec::new();
        for query in points.iter().step_by(37) {
            let mut all: Vec<f64> = points.iter().map(|&p| distance(p, *query)).collect();
            all.sort_by(f64::total_cmp);
            assert_eq!(tree.kth_distance(*query, MIN_POINTS), all[MIN_POINTS - 1]);
            for reach in [0.0, 0.05, 0.13] {
                tree.within(*query, reach, &mut found);
                found.sort_unstable();
                let want: Vec<u32> =
                    (0..points.len() as u32).filter(|&i| distance(points[i as usize], *query) <= reach).collect();
                assert_eq!(found, want);
            }
        }
    }
}
