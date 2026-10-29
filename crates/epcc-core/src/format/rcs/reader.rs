//! Reading RCS: the leaf table says where each leaf's points are; the inner octree words before
//! them are skipped.

use std::fs::File;
use std::io::{BufReader, Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

use super::{
    FIELD_BITS, FIELD_MASK, LEAF_RECORD_SIZE, MAGIC, POINT_RECORD_SIZE, QUANTUM, SECTION_ENTRY_SIZE, SECTION_LEAVES,
    SECTION_POINTS, SECTION_TABLE, Section, header, leaf_record, normals, read_f64s, read_u32, read_u64,
};
use crate::error::IoContext;
use crate::format::PointReader;
use crate::model::{Attributes, CHUNK_POINTS, CloudInfo, PointChunk, ScanInfo, color_from_u8};
use crate::{Error, Progress, Result};

/// The most sections a file may list; real files have four.
const MAX_SECTIONS: u32 = 64;

pub fn open(path: &Path) -> Result<Box<dyn PointReader>> {
    let mut file = BufReader::new(File::open(path).at(path)?);
    let file_size = file.get_ref().metadata().at(path)?.len();
    let invalid = |message: &str| Error::invalid(format!("{}: {message}", path.display()));

    let mut head = vec![0u8; SECTION_TABLE + 4];
    file.read_exact(&mut head).map_err(|_| invalid("not an RCS file (too short)"))?;
    if head[..5] != MAGIC[..5] {
        return Err(invalid("not an RCS file"));
    }
    let section_count = read_u32(&head, SECTION_TABLE);
    if section_count > MAX_SECTIONS {
        return Err(invalid("damaged section table"));
    }
    let mut table = vec![0u8; section_count as usize * SECTION_ENTRY_SIZE];
    file.read_exact(&mut table).map_err(|_| invalid("damaged section table"))?;
    let mut leaves_section = None;
    let mut points_section = None;
    for entry in table.as_chunks::<SECTION_ENTRY_SIZE>().0 {
        let section = Section { offset: read_u64(entry, 8), size: read_u64(entry, 16) };
        if section.offset.checked_add(section.size).is_none_or(|end| end > file_size) {
            return Err(invalid("a section ends beyond the end of the file (truncated?)"));
        }
        match read_u64(entry, 0) {
            SECTION_LEAVES => leaves_section = Some(section),
            SECTION_POINTS => points_section = Some(section),
            _ => {}
        }
    }
    let leaves_section = leaves_section.ok_or_else(|| invalid("the leaf section is missing"))?;
    let points_section = points_section.ok_or_else(|| invalid("the point section is missing"))?;

    let leaf_table = read_section(&mut file, leaves_section, path)?;
    let leaf_count = if leaf_table.len() >= 8 { read_u32(&leaf_table, 0) as usize } else { usize::MAX };
    if leaf_count.checked_mul(LEAF_RECORD_SIZE).is_none_or(|bytes| bytes + 8 > leaf_table.len()) {
        return Err(invalid("the leaf section is too small for its leaves"));
    }
    let mut leaves = Vec::with_capacity(leaf_count);
    let mut block_offset = points_section.offset;
    for record in leaf_table[8..8 + leaf_count * LEAF_RECORD_SIZE].as_chunks::<LEAF_RECORD_SIZE>().0 {
        let points = u64::from(read_u32(record, leaf_record::POINT_COUNT));
        let block_size = u64::from(read_u32(record, leaf_record::BLOCK_SIZE));
        let records = points * POINT_RECORD_SIZE as u64;
        // Every point has a word in the inner octree, so the words take at least 4 bytes each.
        let words = block_size.checked_sub(records).filter(|words| words % 4 == 0 && *words >= 4 * points);
        let Some(words) = words else {
            return Err(invalid("a leaf's block does not fit its point count"));
        };
        leaves.push(Leaf { min: read_f64s(record, leaf_record::CUBE_MIN), points, records_at: block_offset + words });
        block_offset += block_size;
    }
    if block_offset != points_section.offset + points_section.size {
        return Err(invalid("the leaf blocks do not fill the point section"));
    }

    let info = CloudInfo {
        attributes: Attributes {
            intensity: head[header::HAS_INTENSITY] != 0,
            color: head[header::HAS_COLOR] != 0,
            normal: head[header::HAS_NORMALS] != 0,
            ..Attributes::NONE
        },
        scans: vec![ScanInfo::named(super::super::Format::stem_of(path))],
        point_count: Some(leaves.iter().map(|leaf| leaf.points).sum()),
        crs_wkt: None,
        extra_fields: Vec::new(),
    };
    Ok(Box::new(RcsReader { path: path.to_path_buf(), centre: read_f64s(&head, header::CENTRE), leaves, info }))
}

fn read_section(file: &mut BufReader<File>, section: Section, path: &Path) -> Result<Vec<u8>> {
    let size = usize::try_from(section.size).map_err(|_| Error::invalid("RCS section too large"))?;
    let mut bytes = vec![0u8; size];
    file.seek(SeekFrom::Start(section.offset)).at(path)?;
    file.read_exact(&mut bytes).at(path)?;
    Ok(bytes)
}

struct Leaf {
    /// Lower corner of the leaf's cube, relative to the centre.
    min: [f64; 3],
    points: u64,
    /// Where the leaf's point records start in the file.
    records_at: u64,
}

struct RcsReader {
    path: PathBuf,
    centre: [f64; 3],
    leaves: Vec<Leaf>,
    info: CloudInfo,
}

impl PointReader for RcsReader {
    fn info(&self) -> &CloudInfo {
        &self.info
    }

    fn read(self: Box<Self>, emit: &mut dyn FnMut(PointChunk) -> Result<()>, progress: &Progress) -> Result<()> {
        let mut file = BufReader::with_capacity(1 << 20, File::open(&self.path).at(&self.path)?);
        let total = self.info.point_count.unwrap_or(0);
        let attributes = self.info.attributes;
        let mut chunk = PointChunk::with_capacity(0, attributes, CHUNK_POINTS);
        let mut records = Vec::new();
        let mut done = 0;
        for leaf in &self.leaves {
            if leaf.points == 0 {
                continue;
            }
            records.resize(leaf.points as usize * POINT_RECORD_SIZE, 0);
            file.seek(SeekFrom::Start(leaf.records_at)).at(&self.path)?;
            file.read_exact(&mut records).at(&self.path)?;
            let origin: [f64; 3] = std::array::from_fn(|axis| self.centre[axis] + leaf.min[axis]);
            for record in records.as_chunks::<POINT_RECORD_SIZE>().0 {
                push_point(&mut chunk, record, origin);
            }
            if chunk.len() >= CHUNK_POINTS {
                done += chunk.len() as u64;
                emit(std::mem::replace(&mut chunk, PointChunk::with_capacity(0, attributes, CHUNK_POINTS)))?;
                progress.set_done_of(done, total);
            }
        }
        if !chunk.is_empty() {
            emit(chunk)?;
        }
        Ok(())
    }
}

fn push_point(chunk: &mut PointChunk, record: &[u8], origin: [f64; 3]) {
    let packed = read_u64(record, 0);
    chunk.positions.push(std::array::from_fn(|axis| {
        let field = (packed >> (FIELD_BITS * axis as u32)) & FIELD_MASK;
        origin[axis] + field as f64 * QUANTUM
    }));
    if let Some(values) = &mut chunk.intensity {
        values.push(f32::from(record[8]) / 255.0);
    }
    if let Some(values) = &mut chunk.color {
        values.push([record[11], record[10], record[9]].map(color_from_u8));
    }
    if let Some(values) = &mut chunk.normal {
        values.push(normals::decode(u16::from_le_bytes([record[12], record[13]])));
    }
}
