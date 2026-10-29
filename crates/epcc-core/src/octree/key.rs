//! Addressing octree nodes: the root cube and the keys of the nodes inside it.

/// The cube the whole octree lives in. Every node is an equal subdivision of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cube {
    pub min: [f64; 3],
    pub size: f64,
}

impl Cube {
    /// The smallest cube anchored at `min` that holds everything up to `max`.
    pub fn enclosing(min: [f64; 3], max: [f64; 3]) -> Cube {
        let extent = (0..3).map(|axis| max[axis] - min[axis]).fold(0.0, f64::max);
        Cube { min, size: if extent > 0.0 { extent } else { 1.0 } }
    }

    pub fn max(&self) -> [f64; 3] {
        self.min.map(|value| value + self.size)
    }

    pub fn center(&self) -> [f64; 3] {
        self.min.map(|value| value + self.size / 2.0)
    }

    /// The edge length of the nodes at `level`.
    pub fn node_size(&self, level: u8) -> f64 {
        self.size / f64::from(1u32 << level)
    }

    /// The cube of `key`'s node.
    pub fn of(&self, key: NodeKey) -> Cube {
        let size = self.node_size(key.level);
        let cell = [key.x, key.y, key.z];
        Cube { min: std::array::from_fn(|axis| self.min[axis] + f64::from(cell[axis]) * size), size }
    }

    /// The node at `level` that holds `position`.
    ///
    /// This is the only place positions are turned into nodes, so a point on a border between
    /// nodes always lands in the same one. Positions outside the cube go to the nearest node.
    pub fn key_at(&self, position: [f64; 3], level: u8) -> NodeKey {
        let cells = 1u32 << level;
        let scale = f64::from(cells) / self.size;
        let [x, y, z] = std::array::from_fn(|axis| {
            let cell = ((position[axis] - self.min[axis]) * scale).floor();
            cell.clamp(0.0, f64::from(cells - 1)) as u32
        });
        NodeKey { level, x, y, z }
    }
}

/// One node of the octree: its depth and its integer position among the nodes of that depth.
///
/// Keys sort level by level, shallow levels first, and by position within a level. Two runs over
/// the same input produce the same keys, so sorting by key gives a deterministic node order.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct NodeKey {
    pub level: u8,
    pub x: u32,
    pub y: u32,
    pub z: u32,
}

impl NodeKey {
    pub const ROOT: NodeKey = NodeKey { level: 0, x: 0, y: 0, z: 0 };

    /// The child in `octant`, numbered as bits `x y z` (x is the highest bit).
    pub fn child(self, octant: u8) -> NodeKey {
        let bit = |shift: u8| u32::from((octant >> shift) & 1);
        NodeKey { level: self.level + 1, x: self.x * 2 + bit(2), y: self.y * 2 + bit(1), z: self.z * 2 + bit(0) }
    }

    /// The parent node, or `None` for the root.
    pub fn parent(self) -> Option<NodeKey> {
        (self.level > 0).then(|| NodeKey { level: self.level - 1, x: self.x / 2, y: self.y / 2, z: self.z / 2 })
    }

    /// Which child of its parent this node is, numbered like [`NodeKey::child`].
    pub fn octant(self) -> u8 {
        ((self.x & 1) << 2 | (self.y & 1) << 1 | (self.z & 1)) as u8
    }

    /// The ancestor of this node at `level`, which must not be deeper than this node.
    pub fn ancestor_at(self, level: u8) -> NodeKey {
        let shift = self.level - level;
        NodeKey { level, x: self.x >> shift, y: self.y >> shift, z: self.z >> shift }
    }

    pub fn contains(self, other: NodeKey) -> bool {
        other.level >= self.level && other.ancestor_at(self.level) == self
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn children_and_parents_agree() {
        let node = NodeKey { level: 3, x: 5, y: 2, z: 7 };
        for octant in 0..8 {
            let child = node.child(octant);
            assert_eq!(child.parent(), Some(node));
            assert_eq!(child.octant(), octant);
            assert!(node.contains(child));
        }
        assert_eq!(NodeKey::ROOT.parent(), None);
    }

    #[test]
    fn positions_land_in_their_node() {
        let cube = Cube::enclosing([0.0; 3], [8.0, 4.0, 2.0]);
        assert_eq!(cube.size, 8.0);
        assert_eq!(cube.key_at([0.0; 3], 3), NodeKey { level: 3, x: 0, y: 0, z: 0 });
        assert_eq!(cube.key_at([8.0, 8.0, 8.0], 3), NodeKey { level: 3, x: 7, y: 7, z: 7 });
        let key = cube.key_at([3.5, 1.0, 6.2], 3);
        let node = cube.of(key);
        assert_eq!(node.min, [3.0, 1.0, 6.0]);
        assert_eq!(node.size, 1.0);
        assert_eq!(cube.key_at([3.5, 1.0, 6.2], 1), key.ancestor_at(1));
    }
}
