//! RCS (experimental): a single-scan point cloud stored as an octree.
//!
//! The file starts with a fixed header that holds the scan centre, which every other coordinate
//! is relative to, the root cube and a table of sections:
//! - **leaves** (type 2): one record per leaf of the octree: its cube, the bounds of its
//!   points and the shape of the small octree inside it;
//! - **nodes** (type 3): every node of the octree from the root down, breadth first, with an
//!   average colour and intensity;
//! - **points** (type 1): per leaf, the inner octree as 32-bit words followed by the points as
//!   16-byte records: a position on a 1 mm grid inside the leaf, an 8-bit intensity, 8-bit
//!   colour and a normal packed into 16 bits;
//! - **properties** (type 11): a few tagged values.
//!
//! Reading only needs the leaves and the points. Writing reproduces the layout that consuming
//! applications are known to accept: the points are thinned to one per cell of 1.5 to 3 mm,
//! and normals are estimated from the points unless the source has its own.

mod leaf;
mod normals;
mod reader;
#[cfg(test)]
mod tests;
mod writer;

pub(super) use reader::open;
pub(super) use writer::create;

const MAGIC: &[u8; 8] = b"ADOCT\0\0\0";
/// Where the number of sections is stored, followed by one entry per section.
const SECTION_TABLE: usize = 0x129;
const SECTION_ENTRY_SIZE: usize = 24;
/// Where the first section starts; the header takes everything before it.
const HEADER_SIZE: usize = 0x01_812D;

const SECTION_POINTS: u64 = 1;
const SECTION_LEAVES: u64 = 2;
const SECTION_NODES: u64 = 3;
const SECTION_PROPERTIES: u64 = 11;

const LEAF_RECORD_SIZE: usize = 368;
const NODE_RECORD_SIZE: usize = 72;
const POINT_RECORD_SIZE: usize = 16;

/// Grid step of the stored positions, in metres.
const QUANTUM: f64 = 0.001;
/// Bits per coordinate in a point record.
const FIELD_BITS: u32 = 18;
const FIELD_MASK: u64 = (1 << FIELD_BITS) - 1;

/// Offsets of header fields.
mod header {
    pub const CENTRE: usize = 0x10;
    pub const SCALE: usize = 0x40;
    pub const CENTRE_COPY: usize = 0x58;
    pub const ROOT_CUBE: usize = 0x70;
    pub const BOUNDS: usize = 0xA0;
    pub const IMPORT_CODES: usize = 0xD0;
    pub const POINT_COUNT: usize = 0xD8;
    pub const HAS_COLOR: usize = 0xE0;
    pub const HAS_NORMALS: usize = 0xE1;
    pub const HAS_INTENSITY: usize = 0xE2;
    pub const GUID: usize = 0xE3;
}

/// Offsets of the fields of a leaf record.
mod leaf_record {
    pub const CUBE_MIN: usize = 0;
    pub const CUBE_MAX: usize = 24;
    pub const POINTS_MIN: usize = 48;
    pub const POINTS_MAX: usize = 72;
    pub const LEVELS: usize = 96;
    pub const POINT_COUNT: usize = 100;
    pub const BLOCK_SIZE: usize = 104;
    pub const INNER_PER_LEVEL: usize = 108;
    pub const LEAVES_PER_LEVEL: usize = 236;
    /// Entries of the table of inner words per level; the table of point words has one more.
    pub const INNER_LEVELS: usize = 32;
}

/// Bits of the inner octree words in a leaf's block.
mod word {
    /// Set for a word that stands for one point.
    pub const POINT: u32 = 1 << 27;
    pub const MASK_SHIFT: u32 = 19;
}

fn read_u32(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().expect("four bytes"))
}

fn read_u64(bytes: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(bytes[at..at + 8].try_into().expect("eight bytes"))
}

fn read_f64s<const N: usize>(bytes: &[u8], at: usize) -> [f64; N] {
    std::array::from_fn(|index| f64::from_bits(read_u64(bytes, at + 8 * index)))
}

fn put_f64s(bytes: &mut [u8], at: usize, values: &[f64]) {
    for (index, value) in values.iter().enumerate() {
        bytes[at + 8 * index..at + 8 * index + 8].copy_from_slice(&value.to_le_bytes());
    }
}

fn put_u32s(bytes: &mut [u8], at: usize, values: &[u32]) {
    for (index, value) in values.iter().enumerate() {
        bytes[at + 4 * index..at + 4 * index + 4].copy_from_slice(&value.to_le_bytes());
    }
}

/// Where a section is in the file.
#[derive(Clone, Copy, Debug)]
struct Section {
    offset: u64,
    size: u64,
}
