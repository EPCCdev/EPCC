//! COPC: a LAZ 1.4 file whose points are stored as an octree, one compressed chunk per node, so
//! viewers can load just the part and level of detail they show.
//!
//! Reading needs nothing special: a COPC file is a valid LAZ file, read by [`super::las`].
//! Writing builds the octree with [`crate::octree`] and lays the file out as the COPC 1.0
//! specification asks: the COPC info record as the first VLR, the nodes as LAZ chunks with
//! variable point counts, and the node hierarchy as an extended VLR at the end.

use std::fs::File;
use std::io::{BufWriter, Cursor, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU64, Ordering};

use laz::laszip::{ChunkTable, ChunkTableEntry, LazItemRecordBuilder, LazVlr, LazVlrBuilder};
use laz::record::{LayeredPointRecordCompressor, RecordCompressor};

use super::las::{self as las_format, RecordLayout};
use super::{PointWriter, WriteRequest};
use crate::error::IoContext;
use crate::model::{Attributes, CloudInfo, ExtraField, PointChunk, Returns, ValueRange};
use crate::octree::{self, Layout, NodeEntry, NodeKey, NodeSink, OctreeParams};
use crate::{Error, Progress, Result};

/// The octree shape COPC viewers are tuned for. The full depth keeps a dense cloud splittable
/// even when a stray point far away makes the octree cube huge.
const PARAMS: OctreeParams = OctreeParams {
    grid_cells: 128,
    max_leaf_points: 100_000,
    max_subtree_points: 5_000_000,
    max_depth: octree::MAX_DEPTH,
};

const HEADER_SIZE: u16 = 375;
const VLR_HEADER_SIZE: usize = 54;
const EVLR_HEADER_SIZE: usize = 60;
const INFO_SIZE: usize = 160;
const HIERARCHY_ENTRY_SIZE: usize = 32;
/// The LAZ convention for marking a compressed point format.
const COMPRESSED_BIT: u8 = 0x80;
/// LAS 1.4 requires this global encoding bit for point formats 6 and above.
const WKT_ENCODING_BIT: u16 = 1 << 4;
/// Return numbers LAS 1.4 counts in its header.
const RETURN_NUMBERS: usize = 15;

pub(super) fn create(request: &WriteRequest) -> Result<Box<dyn PointWriter>> {
    let work_folder = request.path.parent().map_or_else(|| PathBuf::from("."), Path::to_path_buf);
    Ok(Box::new(CopcWriter {
        path: request.path.to_path_buf(),
        work_folder,
        cloud: request.cloud.clone(),
        precision: request.settings.las.precision,
    }))
}

struct CopcWriter {
    path: PathBuf,
    work_folder: PathBuf,
    cloud: CloudInfo,
    precision: f64,
}

impl PointWriter for CopcWriter {
    fn write(self: Box<Self>, chunks: &mut dyn Iterator<Item = PointChunk>, progress: &Progress) -> Result<()> {
        octree::build(chunks, self.cloud.attributes, &PARAMS, &self.work_folder, progress, |layout| {
            CopcFile::create(&self.path, layout, self.precision, &self.cloud)
        })
    }
}

/// How points are stored: LAS point format 6, or 7 when there is colour, followed by the extra
/// bytes.
#[derive(Clone, Copy, Debug)]
struct PointLayout {
    attributes: Attributes,
    records: RecordLayout,
    scale: f64,
    offset: [f64; 3],
}

impl PointLayout {
    fn format_id(&self) -> u8 {
        self.records.format_number()
    }

    fn record_size(&self) -> usize {
        self.records.size()
    }

    fn laz_vlr(&self) -> LazVlr {
        let items = LazItemRecordBuilder::default_for_point_format_id(self.format_id(), self.records.extra_bytes)
            .expect("point formats 6 and 7 are supported by LAZ");
        LazVlrBuilder::new(items).with_variable_chunk_size().build()
    }

    fn quantize(&self, position: [f64; 3]) -> Result<[i32; 3]> {
        let mut quantized = [0; 3];
        for axis in 0..3 {
            let steps = ((position[axis] - self.offset[axis]) / self.scale).round();
            if steps.abs() > f64::from(i32::MAX) {
                return Err(Error::InvalidSetting(format!(
                    "The cloud is too large for a precision of {} m. Use a coarser precision.",
                    self.scale
                )));
            }
            quantized[axis] = steps as i32;
        }
        Ok(quantized)
    }

    fn dequantize(&self, quantized: [i32; 3]) -> [f64; 3] {
        std::array::from_fn(|axis| self.offset[axis] + f64::from(quantized[axis]) * self.scale)
    }

    /// The node's points as uncompressed LAS records, counting them by return number.
    fn records(&self, points: &PointChunk, by_return: &[AtomicU64; RETURN_NUMBERS]) -> Result<Vec<u8>> {
        let mut records = Vec::with_capacity(points.len() * self.record_size());
        let mut counts = [0u64; RETURN_NUMBERS + 1];
        for index in 0..points.len() {
            let mut point = points.record(index);
            if !self.attributes.returns {
                point.returns = Returns { number: 1, count: 1 };
            }
            counts[usize::from(point.returns.number.min(15))] += 1;
            let extra = points.extra.as_ref().map_or(&[][..], |extra| extra.get(index));
            self.records.encode(&point, self.quantize(point.position)?, extra, &mut records);
        }
        for (total, count) in by_return.iter().zip(&counts[1..]) {
            total.fetch_add(*count, Ordering::Relaxed);
        }
        Ok(records)
    }
}

/// The COPC file being written, which is the octree's node sink.
///
/// The header and the info record can only be filled in at the end, so they are written as
/// placeholders first and overwritten in [`NodeSink::finish`].
struct CopcFile {
    path: PathBuf,
    file: BufWriter<File>,
    points: PointLayout,
    laz_vlr: LazVlr,
    /// Where the point data starts: the LAZ offset to the chunk table, then the chunks.
    point_data_offset: u64,
    vlr_count: u32,
    /// Points per return number 1 to 15, counted as the nodes are encoded.
    by_return: [AtomicU64; RETURN_NUMBERS],
}

impl CopcFile {
    fn create(path: &Path, layout: &Layout, precision: f64, cloud: &CloudInfo) -> Result<CopcFile> {
        let extent = &layout.extent;
        let points = PointLayout {
            attributes: layout.attributes,
            records: RecordLayout::new(layout.attributes, true),
            scale: precision,
            offset: extent.min.map(f64::floor),
        };
        points.quantize(extent.max)?; // fails before the octree is built if the cloud is too large
        let laz_vlr = points.laz_vlr();
        let mut vlrs = vec![Vlr::new("copc", 1, "COPC info", vec![0; INFO_SIZE])];
        let mut laz_data = Vec::new();
        laz_vlr.write_to(&mut laz_data).at(path)?;
        vlrs.push(Vlr::new(LazVlr::USER_ID, LazVlr::RECORD_ID, LazVlr::DESCRIPTION, laz_data));
        if layout.attributes.has_extra() {
            let data = las_format::extra_bytes_record(&cloud.extra_fields);
            debug_assert_eq!(ExtraField::total_size(&cloud.extra_fields), layout.attributes.extra_bytes);
            vlrs.push(Vlr::new(
                las_format::EXTRA_BYTES_USER_ID,
                las_format::EXTRA_BYTES_RECORD_ID,
                las_format::EXTRA_BYTES_DESCRIPTION,
                data,
            ));
        }
        if let Some(wkt) = &cloud.crs_wkt {
            let mut data = wkt.as_bytes().to_vec();
            data.push(0);
            vlrs.push(Vlr::new(las_format::WKT_USER_ID, las_format::WKT_RECORD_ID, "OGC WKT coordinate system", data));
        }

        let mut file = BufWriter::with_capacity(1 << 20, File::create(path).at(path)?);
        file.write_all(&[0; HEADER_SIZE as usize]).at(path)?;
        for vlr in &vlrs {
            vlr.write_to(&mut file).at(path)?;
        }
        let point_data_offset = u64::from(HEADER_SIZE) + vlrs.iter().map(Vlr::size).sum::<u64>();
        file.write_all(&(-1i64).to_le_bytes()).at(path)?;
        Ok(CopcFile {
            path: path.to_path_buf(),
            file,
            points,
            laz_vlr,
            point_data_offset,
            vlr_count: vlrs.len() as u32,
            by_return: Default::default(),
        })
    }

    /// Where the first node's chunk starts.
    fn first_chunk_offset(&self) -> u64 {
        self.point_data_offset + 8
    }

    fn compress(&self, records: &[u8]) -> Result<Vec<u8>> {
        let mut compressor = LayeredPointRecordCompressor::new(Cursor::new(Vec::new()));
        compressor.set_fields_from(self.laz_vlr.items()).map_err(laz_error)?;
        compressor.compress_many(records).at(&self.path)?;
        compressor.done().at(&self.path)?;
        Ok(compressor.into_inner().into_inner())
    }

    fn write_hierarchy(&mut self, nodes: &[NodeEntry]) -> Result<(u64, u64)> {
        let mut page = Vec::with_capacity(nodes.len() * HIERARCHY_ENTRY_SIZE);
        for node in nodes {
            let (offset, size) = if node.point_count == 0 {
                (0, 0)
            } else {
                (self.first_chunk_offset() + node.byte_offset, node.byte_size)
            };
            for value in [node.key.level as i32, node.key.x as i32, node.key.y as i32, node.key.z as i32] {
                page.extend_from_slice(&value.to_le_bytes());
            }
            page.extend_from_slice(&offset.to_le_bytes());
            page.extend_from_slice(&(size as i32).to_le_bytes());
            page.extend_from_slice(&(node.point_count as i32).to_le_bytes());
        }
        let evlr_start = self.position()?;
        Vlr::new("copc", 1000, "EPT hierarchy", page).write_extended_to(&mut self.file).at(&self.path)?;
        let page_offset = evlr_start + EVLR_HEADER_SIZE as u64;
        Ok((evlr_start, page_offset))
    }

    fn position(&mut self) -> Result<u64> {
        self.file.stream_position().at(&self.path)
    }

    fn rewrite(&mut self, offset: u64, bytes: &[u8]) -> Result<()> {
        self.file.seek(SeekFrom::Start(offset)).at(&self.path)?;
        self.file.write_all(bytes).at(&self.path)
    }
}

impl NodeSink for CopcFile {
    fn encode(&self, _key: NodeKey, points: &PointChunk) -> Result<Vec<u8>> {
        self.compress(&self.points.records(points, &self.by_return)?)
    }

    fn append(&mut self, _key: NodeKey, bytes: &[u8]) -> Result<()> {
        self.file.write_all(bytes).at(&self.path)
    }

    fn finish(mut self, layout: &Layout, nodes: &[NodeEntry]) -> Result<()> {
        let mut chunk_table = ChunkTable::default();
        for node in nodes.iter().filter(|node| node.point_count > 0) {
            chunk_table.push(ChunkTableEntry { point_count: node.point_count, byte_count: node.byte_size });
        }
        let chunk_table_offset = self.position()?;
        chunk_table.write_to(&mut self.file, &self.laz_vlr).at(&self.path)?;
        let (evlr_start, page_offset) = self.write_hierarchy(nodes)?;
        let page_size = (nodes.len() * HIERARCHY_ENTRY_SIZE) as u64;

        let header = self.header(layout, evlr_start);
        let info = self.info(layout, page_offset, page_size);
        self.rewrite(0, &header)?;
        self.rewrite(u64::from(HEADER_SIZE) + VLR_HEADER_SIZE as u64, &info)?;
        self.rewrite(self.point_data_offset, &chunk_table_offset.to_le_bytes())?;
        self.file.flush().at(&self.path)
    }
}

impl CopcFile {
    fn header(&self, layout: &Layout, evlr_start: u64) -> Vec<u8> {
        let points = &self.points;
        let quantized = |position| points.quantize(position).expect("checked when the file was created");
        let min = points.dequantize(quantized(layout.extent.min));
        let max = points.dequantize(quantized(layout.extent.max));
        let mut header = Bytes(Vec::with_capacity(HEADER_SIZE as usize));
        header.raw(b"LASF");
        header.u16(0); // file source
        header.u16(WKT_ENCODING_BIT);
        header.raw(&[0; 16]); // project id
        header.raw(&[1, 4]);
        header.text("EPCC", 32);
        header.text(&format!("EPCC {}", crate::VERSION), 32);
        header.u16(0); // creation day, left out so equal input gives an equal file
        header.u16(0); // creation year
        header.u16(HEADER_SIZE);
        header.u32(self.point_data_offset as u32);
        header.u32(self.vlr_count);
        header.raw(&[points.format_id() | COMPRESSED_BIT]);
        header.u16(points.record_size() as u16);
        header.raw(&[0; 4 + 5 * 4]); // legacy point counts, zero for point formats 6 and above
        (0..3).for_each(|_| header.f64(points.scale));
        points.offset.iter().for_each(|&offset| header.f64(offset));
        for axis in 0..3 {
            header.f64(max[axis]);
            header.f64(min[axis]);
        }
        header.u64(0); // waveform data
        header.u64(evlr_start);
        header.u32(1); // extended VLRs
        header.u64(layout.point_count);
        for count in &self.by_return {
            header.u64(count.load(Ordering::Relaxed));
        }
        debug_assert_eq!(header.0.len(), HEADER_SIZE as usize);
        header.0
    }

    fn info(&self, layout: &Layout, page_offset: u64, page_size: u64) -> Vec<u8> {
        let time = layout.extent.time.unwrap_or(ValueRange { min: 0.0, max: 0.0 });
        let mut info = Bytes(Vec::with_capacity(INFO_SIZE));
        layout.cube.center().iter().for_each(|&value| info.f64(value));
        info.f64(layout.cube.size / 2.0);
        info.f64(layout.root_spacing);
        info.u64(page_offset);
        info.u64(page_size);
        info.f64(time.min);
        info.f64(time.max);
        info.raw(&[0; 11 * 8]);
        debug_assert_eq!(info.0.len(), INFO_SIZE);
        info.0
    }
}

/// A variable length record, written with the short header of a VLR or the long one of an
/// extended VLR.
struct Vlr {
    user_id: &'static str,
    record_id: u16,
    description: &'static str,
    data: Vec<u8>,
}

impl Vlr {
    fn new(user_id: &'static str, record_id: u16, description: &'static str, data: Vec<u8>) -> Vlr {
        Vlr { user_id, record_id, description, data }
    }

    fn size(&self) -> u64 {
        (VLR_HEADER_SIZE + self.data.len()) as u64
    }

    fn write_to(&self, out: &mut impl Write) -> std::io::Result<()> {
        let mut header = self.header_start();
        header.u16(self.data.len() as u16);
        header.text(self.description, 32);
        out.write_all(&header.0)?;
        out.write_all(&self.data)
    }

    fn write_extended_to(&self, out: &mut impl Write) -> std::io::Result<()> {
        let mut header = self.header_start();
        header.u64(self.data.len() as u64);
        header.text(self.description, 32);
        out.write_all(&header.0)?;
        out.write_all(&self.data)
    }

    fn header_start(&self) -> Bytes {
        let mut header = Bytes(Vec::with_capacity(EVLR_HEADER_SIZE));
        header.u16(0); // reserved
        header.text(self.user_id, 16);
        header.u16(self.record_id);
        header
    }
}

/// Little-endian byte building for headers.
struct Bytes(Vec<u8>);

impl Bytes {
    fn raw(&mut self, bytes: &[u8]) {
        self.0.extend_from_slice(bytes);
    }

    fn u16(&mut self, value: u16) {
        self.raw(&value.to_le_bytes());
    }

    fn u32(&mut self, value: u32) {
        self.raw(&value.to_le_bytes());
    }

    fn u64(&mut self, value: u64) {
        self.raw(&value.to_le_bytes());
    }

    fn f64(&mut self, value: f64) {
        self.raw(&value.to_le_bytes());
    }

    /// `text` padded with zeros to `width` bytes.
    fn text(&mut self, text: &str, width: usize) {
        let bytes = &text.as_bytes()[..text.len().min(width)];
        self.raw(bytes);
        self.raw(&vec![0; width - bytes.len()]);
    }
}

fn laz_error(error: laz::LasZipError) -> Error {
    Error::InvalidData(format!("LAZ compression: {error}"))
}
