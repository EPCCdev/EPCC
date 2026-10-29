//! Temporary point files: fixed-size binary records holding exactly a cloud's attributes.
//!
//! Used for the staged input and for the subtrees the partition step splits it into. A point
//! file is either a file of its own or a list of segments in a log, one file shared by many
//! subtrees.
//!
//! The points are nothing but their records, one after another. Each record is little-endian:
//! the position as three `f64`, then only the attributes the cloud carries, in this order:
//! intensity `f32`, colour 3 × `u16`, normal 3 × `f32`, classification `u8`, time `f64`, return
//! number and count 2 × `u8`, lidar details (scan angle `f32`, point source `u16`, user data and
//! flags 2 × `u8`), then the extra values as they are.

use std::fs::{self, File, OpenOptions};
use std::io::{BufWriter, ErrorKind, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

use crossbeam_channel::bounded;
use rayon::prelude::*;

use crate::error::IoContext;
use crate::model::{Attributes, ExtraBytes, LidarDetails, PointChunk, Returns, TASK_POINTS};
use crate::{Error, Result};

/// Bytes collected before a writer goes to disk; larger writes go to disk directly.
const WRITE_BUFFER_BYTES: usize = 8 << 20;
/// Bytes read at a time when the caller does not choose a block size.
const READ_BLOCK_BYTES: usize = 32 << 20;
/// Blocks read from disk ahead of the one being processed.
const READ_AHEAD_BLOCKS: usize = 1;

/// A folder for temporary files that is removed with everything in it when dropped.
pub(crate) struct TempFolder {
    path: PathBuf,
}

impl TempFolder {
    /// Creates a new, uniquely named folder inside `parent`.
    pub fn create(parent: &Path) -> Result<TempFolder> {
        static COUNTER: AtomicU32 = AtomicU32::new(0);
        loop {
            let number = COUNTER.fetch_add(1, Ordering::Relaxed);
            let path = parent.join(format!(".epcc-temp-{}-{number}", std::process::id()));
            match fs::create_dir(&path) {
                Ok(()) => return Ok(TempFolder { path }),
                // Left behind by an earlier process that had the same id.
                Err(error) if error.kind() == ErrorKind::AlreadyExists => continue,
                Err(source) => return Err(Error::Io { path, source }),
            }
        }
    }

    pub fn path(&self) -> &Path {
        &self.path
    }
}

impl Drop for TempFolder {
    fn drop(&mut self) {
        let _ = fs::remove_dir_all(&self.path);
    }
}

/// Appends points to a new point file of its own.
pub(crate) struct PointFileWriter {
    path: PathBuf,
    file: BufWriter<File>,
    attributes: Attributes,
    record_size: usize,
    /// The records of the chunk being written, reused from chunk to chunk.
    encoded: Vec<u8>,
    point_count: u64,
}

impl PointFileWriter {
    pub fn create(path: &Path, attributes: Attributes) -> Result<PointFileWriter> {
        let file = File::create(path).at(path)?;
        Ok(PointFileWriter {
            path: path.to_path_buf(),
            file: BufWriter::with_capacity(WRITE_BUFFER_BYTES, file),
            attributes,
            record_size: record_size(attributes),
            encoded: Vec::new(),
            point_count: 0,
        })
    }

    /// Appends all points of `chunk`, which must carry exactly the writer's attributes.
    pub fn write(&mut self, chunk: &PointChunk) -> Result<()> {
        if chunk.attributes() != self.attributes {
            return Err(Error::invalid(format!(
                "points with {:?} cannot be staged as {:?}",
                chunk.attributes().names(),
                self.attributes.names()
            )));
        }
        let size = self.record_size;
        self.encoded.resize(chunk.len() * size, 0);
        self.encoded
            .par_chunks_mut(size * TASK_POINTS)
            .enumerate()
            .for_each(|(task, records)| encode(chunk, task * TASK_POINTS, records, size));
        self.file.write_all(&self.encoded).at(&self.path)?;
        self.point_count += chunk.len() as u64;
        Ok(())
    }

    pub fn finish(mut self) -> Result<PointFile> {
        self.file.flush().at(&self.path)?;
        Ok(PointFile {
            attributes: self.attributes,
            point_count: self.point_count,
            storage: Storage::Whole { path: self.path },
        })
    }
}

/// A complete set of temporary points.
#[derive(Debug)]
pub(crate) struct PointFile {
    pub attributes: Attributes,
    pub point_count: u64,
    storage: Storage,
}

/// Where the records of a [`PointFile`] are.
#[derive(Debug)]
enum Storage {
    /// All of one file.
    Whole { path: PathBuf },
    /// These pieces of a log, in order.
    Segments { log: Arc<LogFile>, segments: Vec<Segment> },
}

impl PointFile {
    /// Bytes per point.
    pub fn record_size(&self) -> usize {
        record_size(self.attributes)
    }

    /// Hands the undecoded records to `each` in their order, in blocks of whole records.
    /// [`position_of`] reads a record's position.
    pub fn read_records(&self, each: &mut dyn FnMut(&[u8]) -> Result<()>) -> Result<()> {
        match &self.storage {
            Storage::Whole { path } => self.read_whole(path, self.default_block_points(), each),
            Storage::Segments { log, segments } => log.read_segments(segments, each),
        }
    }

    /// Reads all points into one chunk.
    pub fn read_all(&self) -> Result<PointChunk> {
        let len = usize::try_from(self.point_count).map_err(|_| {
            Error::invalid(format!("{} temporary points are too many to hold in memory", self.point_count))
        })?;
        let mut chunk = PointChunk::with_capacity(0, self.attributes, len);
        self.read_records(&mut |records| {
            decode_into(records, self.attributes, &mut chunk);
            Ok(())
        })?;
        Ok(chunk)
    }

    /// Deletes the points. A log is deleted once none of its point files is left.
    pub fn remove(self) -> Result<()> {
        match self.storage {
            Storage::Whole { path } => fs::remove_file(&path).at(&path),
            Storage::Segments { .. } => Ok(()),
        }
    }

    fn default_block_points(&self) -> usize {
        (READ_BLOCK_BYTES / self.record_size()).max(1)
    }

    /// Hands the file at `path` to `each` in blocks of at most `block_points` records. The next
    /// block is read on another thread while `each` works on the current one.
    fn read_whole(&self, path: &Path, block_points: usize, each: &mut dyn FnMut(&[u8]) -> Result<()>) -> Result<()> {
        let mut file = File::open(path).at(path)?;
        let total_bytes = self.point_count * self.record_size() as u64;
        let file_bytes = file.metadata().at(path)?.len();
        if file_bytes != total_bytes {
            return Err(Error::invalid(format!(
                "{}: temporary point file holds {file_bytes} bytes instead of {total_bytes}",
                path.display()
            )));
        }
        let block_bytes = (block_points.max(1) * self.record_size()) as u64;
        let (full_tx, full_rx) = bounded::<Result<Vec<u8>>>(READ_AHEAD_BLOCKS);
        let (empty_tx, empty_rx) = bounded::<Vec<u8>>(READ_AHEAD_BLOCKS + 1);

        thread::scope(|scope| {
            scope.spawn(move || {
                let mut remaining = total_bytes;
                while remaining > 0 {
                    let mut block = empty_rx.try_recv().unwrap_or_default();
                    block.resize(remaining.min(block_bytes) as usize, 0);
                    remaining -= block.len() as u64;
                    let read = file.read_exact(&mut block).at(path).map(|()| block);
                    let failed = read.is_err();
                    // A closed queue means the consumer stopped; it reports why.
                    if full_tx.send(read).is_err() || failed {
                        break;
                    }
                }
            });
            for block in full_rx {
                let block = block?;
                each(&block)?;
                let _ = empty_tx.try_send(block);
            }
            Ok(())
        })
    }
}

/// A contiguous piece of a log.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Segment {
    offset: u64,
    len: u64,
}

/// Appends pages of records to a new log, a file whose pieces belong to many point files.
pub(crate) struct LogWriter {
    path: PathBuf,
    file: BufWriter<File>,
    written: u64,
}

impl LogWriter {
    pub fn create(path: &Path) -> Result<LogWriter> {
        let file = OpenOptions::new().read(true).write(true).create(true).truncate(true).open(path).at(path)?;
        Ok(LogWriter { path: path.to_path_buf(), file: BufWriter::with_capacity(WRITE_BUFFER_BYTES, file), written: 0 })
    }

    /// Appends `records` to the log and their place to `segments`, which lists one point
    /// file's pieces. A piece right after the previous one extends it.
    pub fn append(&mut self, records: &[u8], segments: &mut Vec<Segment>) -> Result<()> {
        self.file.write_all(records).at(&self.path)?;
        let len = records.len() as u64;
        match segments.last_mut() {
            Some(last) if last.offset + last.len == self.written => last.len += len,
            _ => segments.push(Segment { offset: self.written, len }),
        }
        self.written += len;
        Ok(())
    }

    /// Completes the log, so that point files can be made of its segments.
    pub fn finish(self) -> Result<Log> {
        let handle = self.file.into_inner().map_err(|error| error.into_error()).at(&self.path)?;
        Ok(Log { file: Arc::new(LogFile { handle: Mutex::new(handle), path: RemovedOnDrop(self.path) }) })
    }
}

/// A finished log, for making point files of its segments. The file is deleted once this and all
/// point files made from it are gone.
pub(crate) struct Log {
    file: Arc<LogFile>,
}

impl Log {
    /// The point file made of `segments`, whose records carry `attributes`.
    pub fn point_file(&self, attributes: Attributes, segments: Vec<Segment>) -> PointFile {
        let bytes: u64 = segments.iter().map(|segment| segment.len).sum();
        PointFile {
            attributes,
            point_count: bytes / record_size(attributes) as u64,
            storage: Storage::Segments { log: Arc::clone(&self.file), segments },
        }
    }
}

/// The file of a finished log, shared by the point files made of it.
#[derive(Debug)]
struct LogFile {
    /// One handle for all readers; each read is a seek followed by a read.
    handle: Mutex<File>,
    /// Dropped after the handle is closed.
    path: RemovedOnDrop,
}

impl LogFile {
    /// Hands the records in `segments` to `each`, joining neighbouring segments into blocks of
    /// up to [`READ_BLOCK_BYTES`].
    fn read_segments(&self, segments: &[Segment], each: &mut dyn FnMut(&[u8]) -> Result<()>) -> Result<()> {
        let mut block = Vec::new();
        for segment in segments {
            if !block.is_empty() && block.len() as u64 + segment.len > READ_BLOCK_BYTES as u64 {
                each(&block)?;
                block.clear();
            }
            self.read_segment(*segment, &mut block)?;
        }
        if block.is_empty() { Ok(()) } else { each(&block) }
    }

    /// Appends the bytes of `segment` to `block`.
    fn read_segment(&self, segment: Segment, block: &mut Vec<u8>) -> Result<()> {
        let start = block.len();
        block.resize(start + segment.len as usize, 0);
        let mut file = self.handle.lock().unwrap_or_else(|poisoned| poisoned.into_inner());
        let path = &self.path.0;
        file.seek(SeekFrom::Start(segment.offset)).at(path)?;
        file.read_exact(&mut block[start..]).at(path)
    }
}

/// A path whose file is deleted when this is dropped.
#[derive(Debug)]
struct RemovedOnDrop(PathBuf);

impl Drop for RemovedOnDrop {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.0);
    }
}

/// Bytes per record of a cloud with `attributes`.
pub(crate) fn record_size(attributes: Attributes) -> usize {
    let optional = [
        (attributes.intensity, f32::SIZE),
        (attributes.color, <[u16; 3]>::SIZE),
        (attributes.normal, <[f32; 3]>::SIZE),
        (attributes.classification, u8::SIZE),
        (attributes.time, f64::SIZE),
        (attributes.returns, Returns::SIZE),
        (attributes.lidar, LidarDetails::SIZE),
        (attributes.has_extra(), usize::from(attributes.extra_bytes)),
    ];
    <[f64; 3]>::SIZE + optional.iter().filter(|(present, _)| *present).map(|(_, size)| size).sum::<usize>()
}

/// The position stored in `record`.
pub(crate) fn position_of(record: &[u8]) -> [f64; 3] {
    <[f64; 3]>::get(record)
}

/// Encodes the points of `chunk` from `start` on into `records`, as many as it has room for.
fn encode(chunk: &PointChunk, start: usize, records: &mut [u8], size: usize) {
    let range = start..start + records.len() / size;
    let mut writer = RecordWriter { records, size, offset: 0 };
    writer.put(&chunk.positions[range.clone()]);
    if let Some(values) = &chunk.intensity {
        writer.put(&values[range.clone()]);
    }
    if let Some(values) = &chunk.color {
        writer.put(&values[range.clone()]);
    }
    if let Some(values) = &chunk.normal {
        writer.put(&values[range.clone()]);
    }
    if let Some(values) = &chunk.classification {
        writer.put(&values[range.clone()]);
    }
    if let Some(values) = &chunk.time {
        writer.put(&values[range.clone()]);
    }
    if let Some(values) = &chunk.returns {
        writer.put(&values[range.clone()]);
    }
    if let Some(values) = &chunk.lidar {
        writer.put(&values[range.clone()]);
    }
    if let Some(extra) = &chunk.extra {
        writer.put_bytes(&extra.bytes[range.start * extra.width..range.end * extra.width], extra.width);
    }
}

/// Decodes `records`, whose points carry `attributes`, on all cores into a chunk of their own.
pub(crate) fn decode_records(records: &[u8], attributes: Attributes) -> PointChunk {
    let mut chunk = PointChunk::new(0, attributes);
    decode_into(records, attributes, &mut chunk);
    chunk
}

/// Decodes `records`, whose points carry `attributes`, on all cores and appends them to `chunk`.
fn decode_into(records: &[u8], attributes: Attributes, chunk: &mut PointChunk) {
    let size = record_size(attributes);
    let tasks: Vec<PointChunk> =
        records.par_chunks(size * TASK_POINTS).map(|task| decode(task, attributes, size)).collect();
    tasks.into_iter().for_each(|task| chunk.append(task));
}

/// Decodes `records` into a chunk of their own.
fn decode(records: &[u8], attributes: Attributes, size: usize) -> PointChunk {
    let mut reader = RecordReader { records, size, offset: 0 };
    // The fields are read in the order they have in a record.
    PointChunk {
        scan: 0,
        positions: reader.column(),
        intensity: reader.column_if(attributes.intensity),
        color: reader.column_if(attributes.color),
        normal: reader.column_if(attributes.normal),
        classification: reader.column_if(attributes.classification),
        time: reader.column_if(attributes.time),
        returns: reader.column_if(attributes.returns),
        lidar: reader.column_if(attributes.lidar),
        extra: attributes.has_extra().then(|| reader.bytes(usize::from(attributes.extra_bytes))),
        grid: None,
    }
}

/// A value with a fixed-size little-endian encoding.
trait Field: Copy {
    const SIZE: usize;
    /// Writes the value to the start of `bytes`.
    fn put(self, bytes: &mut [u8]);
    /// Reads the value from the start of `bytes`.
    fn get(bytes: &[u8]) -> Self;
}

macro_rules! number_fields {
    ($($number:ty),*) => {$(
        impl Field for $number {
            const SIZE: usize = size_of::<$number>();

            fn put(self, bytes: &mut [u8]) {
                bytes[..Self::SIZE].copy_from_slice(&self.to_le_bytes());
            }

            fn get(bytes: &[u8]) -> Self {
                Self::from_le_bytes(bytes[..Self::SIZE].try_into().expect("slice has the field's size"))
            }
        }
    )*};
}

number_fields!(u8, u16, f32, f64);

impl Field for Returns {
    const SIZE: usize = 2;

    fn put(self, bytes: &mut [u8]) {
        bytes[0] = self.number;
        bytes[1] = self.count;
    }

    fn get(bytes: &[u8]) -> Self {
        Returns { number: bytes[0], count: bytes[1] }
    }
}

impl Field for LidarDetails {
    const SIZE: usize = 8;

    fn put(self, bytes: &mut [u8]) {
        self.scan_angle.put(bytes);
        self.point_source.put(&mut bytes[4..]);
        bytes[6] = self.user_data;
        bytes[7] = self.flags;
    }

    fn get(bytes: &[u8]) -> Self {
        LidarDetails {
            scan_angle: f32::get(bytes),
            point_source: u16::get(&bytes[4..]),
            user_data: bytes[6],
            flags: bytes[7],
        }
    }
}

impl<T: Field> Field for [T; 3] {
    const SIZE: usize = 3 * T::SIZE;

    fn put(self, bytes: &mut [u8]) {
        for (index, value) in self.into_iter().enumerate() {
            value.put(&mut bytes[index * T::SIZE..]);
        }
    }

    fn get(bytes: &[u8]) -> Self {
        std::array::from_fn(|index| T::get(&bytes[index * T::SIZE..]))
    }
}

/// Fills consecutive records column by column.
struct RecordWriter<'a> {
    records: &'a mut [u8],
    size: usize,
    /// Where the next column starts within a record.
    offset: usize,
}

impl RecordWriter<'_> {
    fn put<T: Field>(&mut self, values: &[T]) {
        for (record, &value) in self.records.chunks_exact_mut(self.size).zip(values) {
            value.put(&mut record[self.offset..]);
        }
        self.offset += T::SIZE;
    }

    /// Puts `width` raw bytes per record.
    fn put_bytes(&mut self, values: &[u8], width: usize) {
        for (record, values) in self.records.chunks_exact_mut(self.size).zip(values.chunks_exact(width)) {
            record[self.offset..self.offset + width].copy_from_slice(values);
        }
        self.offset += width;
    }
}

/// Reads consecutive records column by column.
struct RecordReader<'a> {
    records: &'a [u8],
    size: usize,
    /// Where the next column starts within a record.
    offset: usize,
}

impl RecordReader<'_> {
    fn column<T: Field>(&mut self) -> Vec<T> {
        let values = self.records.chunks_exact(self.size).map(|record| T::get(&record[self.offset..])).collect();
        self.offset += T::SIZE;
        values
    }

    /// The next `width` raw bytes of every record.
    fn bytes(&mut self, width: usize) -> ExtraBytes {
        let offset = self.offset;
        let bytes = self.records.chunks_exact(self.size).flat_map(|record| &record[offset..offset + width]).copied();
        self.offset += width;
        ExtraBytes { width, bytes: bytes.collect() }
    }

    /// The next column if the records carry it.
    fn column_if<T: Field>(&mut self, present: bool) -> Option<Vec<T>> {
        present.then(|| self.column())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::PointRecord;

    /// Deterministic pseudo-random points carrying `attributes`, with awkward position values
    /// among them.
    fn random_chunk(len: usize, attributes: Attributes, seed: u64) -> PointChunk {
        let mut state = seed | 1;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        let special = [-0.0, f64::MIN_POSITIVE, 1e300, -1.0 / 3.0, 0.1 + 0.2];
        let mut chunk = PointChunk::new(3, attributes);
        for index in 0..len {
            let mut value = || {
                let bits = next();
                if bits.is_multiple_of(7) { special[(bits >> 8) as usize % special.len()] } else { bits as f64 / 1e9 }
            };
            let position = [value(), value(), value()];
            let bits = next();
            chunk.push(&PointRecord {
                position,
                intensity: (bits & 0xffff) as f32 / 65535.0,
                color: [bits as u16, (bits >> 16) as u16, (bits >> 32) as u16],
                normal: [(bits >> 40) as f32, -0.5, 0.25],
                classification: (bits >> 56) as u8,
                time: index as f64 + 0.5,
                returns: Returns { number: (bits >> 3) as u8 & 15, count: (bits >> 9) as u8 & 15 },
                lidar: LidarDetails {
                    scan_angle: (bits >> 20) as i16 as f32 * 0.006,
                    point_source: (bits >> 24) as u16,
                    user_data: (bits >> 44) as u8,
                    flags: (bits >> 50) as u8,
                },
                ..PointRecord::default()
            });
            if let Some(extra) = &mut chunk.extra {
                let values = extra.get_mut(index);
                for (byte, value) in values.iter_mut().zip(bits.to_le_bytes().into_iter().cycle()) {
                    *byte = value;
                }
            }
        }
        chunk
    }

    fn assert_bit_exact(got: &PointChunk, want: &PointChunk) {
        assert_eq!(got.scan, 0);
        assert_eq!(got.len(), want.len());
        let bits = |chunk: &PointChunk| chunk.positions.iter().map(|p| p.map(f64::to_bits)).collect::<Vec<_>>();
        assert_eq!(bits(got), bits(want));
        assert_eq!(PointChunk { scan: want.scan, ..got.clone() }, *want);
    }

    fn write_file(folder: &TempFolder, name: &str, chunks: &[PointChunk], attributes: Attributes) -> PointFile {
        let mut writer = PointFileWriter::create(&folder.path().join(name), attributes).unwrap();
        for chunk in chunks {
            writer.write(chunk).unwrap();
        }
        writer.finish().unwrap()
    }

    fn path_of(file: &PointFile) -> PathBuf {
        match &file.storage {
            Storage::Whole { path } => path.clone(),
            Storage::Segments { log, .. } => log.path.0.clone(),
        }
    }

    fn whole_blocks(file: &PointFile, block_points: usize, each: &mut dyn FnMut(&[u8]) -> Result<()>) -> Result<()> {
        file.read_whole(&path_of(file), block_points, each)
    }

    fn encoded(chunk: &PointChunk) -> Vec<u8> {
        let size = record_size(chunk.attributes());
        let mut records = vec![0; chunk.len() * size];
        encode(chunk, 0, &mut records, size);
        records
    }

    #[test]
    fn records_round_trip_for_any_attributes() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let combinations = [
            Attributes::NONE,
            Attributes::ALL,
            Attributes { intensity: true, ..Attributes::NONE },
            Attributes { color: true, time: true, ..Attributes::NONE },
            Attributes { normal: true, classification: true, ..Attributes::NONE },
            Attributes { returns: true, extra_bytes: 13, ..Attributes::NONE },
            Attributes { lidar: true, extra_bytes: 2, ..Attributes::NONE },
        ];
        for (index, attributes) in combinations.into_iter().enumerate() {
            let chunks = [random_chunk(50_000, attributes, 11), random_chunk(7, attributes, 12)];
            let file = write_file(&folder, &format!("{index}.bin"), &chunks, attributes);
            assert_eq!(file.point_count, 50_007);
            assert_eq!(fs::metadata(path_of(&file)).unwrap().len(), 50_007 * record_size(attributes) as u64);

            let mut whole = chunks[0].clone();
            whole.append(chunks[1].clone());
            assert_bit_exact(&file.read_all().unwrap(), &whole);
        }
    }

    #[test]
    fn record_sizes_follow_the_attributes() {
        assert_eq!(record_size(Attributes::NONE), 24);
        assert_eq!(record_size(Attributes::ALL), 24 + 4 + 6 + 12 + 1 + 8 + 2 + 8 + 1);
        assert_eq!(record_size(Attributes { extra_bytes: 10, ..Attributes::NONE }), 34);
        assert_eq!(record_size(Attributes { color: true, ..Attributes::NONE }), 30);
    }

    #[test]
    fn blocks_come_back_in_order_and_size() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let attributes = Attributes { time: true, ..Attributes::NONE };
        let chunk = random_chunk(100_000, attributes, 5);
        let file = write_file(&folder, "points.bin", std::slice::from_ref(&chunk), attributes);

        let mut sizes = Vec::new();
        let mut decoded = PointChunk::new(0, attributes);
        whole_blocks(&file, 30_000, &mut |records| {
            sizes.push(records.len() / file.record_size());
            decode_into(records, attributes, &mut decoded);
            Ok(())
        })
        .unwrap();
        assert_eq!(sizes, [30_000, 30_000, 30_000, 10_000]);
        assert_bit_exact(&decoded, &chunk);
    }

    #[test]
    fn errors_stop_reading() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let chunk = random_chunk(10_000, Attributes::NONE, 9);
        let file = write_file(&folder, "points.bin", &[chunk], Attributes::NONE);
        let mut blocks = 0;
        let result = whole_blocks(&file, 100, &mut |_| {
            blocks += 1;
            if blocks == 3 { Err(Error::Cancelled) } else { Ok(()) }
        });
        assert!(matches!(result, Err(Error::Cancelled)));
        assert_eq!(blocks, 3);
    }

    #[test]
    fn mismatched_attributes_are_refused() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let mut writer = PointFileWriter::create(&folder.path().join("points.bin"), Attributes::NONE).unwrap();
        let chunk = random_chunk(10, Attributes { color: true, ..Attributes::NONE }, 1);
        assert!(writer.write(&chunk).is_err());
    }

    #[test]
    fn empty_and_removed_files() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let file = write_file(&folder, "empty.bin", &[], Attributes::ALL);
        assert_eq!(file.point_count, 0);
        assert_eq!(file.read_all().unwrap(), PointChunk::new(0, Attributes::ALL));
        let path = path_of(&file);
        file.remove().unwrap();
        assert!(!path.exists());
    }

    /// Interleaves the pages of three point files in one log, as the partition step does.
    #[test]
    fn log_files_read_back_their_own_points_in_order() {
        let folder = TempFolder::create(&std::env::temp_dir()).unwrap();
        let attributes = Attributes::ALL;
        let size = record_size(attributes);
        let chunks: Vec<PointChunk> = (0..3).map(|seed| random_chunk(40_000, attributes, seed + 20)).collect();
        let records: Vec<Vec<u8>> = chunks.iter().map(encoded).collect();

        let mut writer = LogWriter::create(&folder.path().join("points.log")).unwrap();
        let mut segments = vec![Vec::new(); 3];
        let mut pages: Vec<_> = records.iter().map(|records| records.chunks(size * 1_000)).collect();
        for _ in 0..20 {
            for file in [0, 0, 1, 2, 1, 2] {
                writer.append(pages[file].next().unwrap(), &mut segments[file]).unwrap();
            }
        }
        let log = writer.finish().unwrap();
        let path = folder.path().join("points.log");
        assert_eq!(fs::metadata(&path).unwrap().len(), (3 * 40_000 * size) as u64);
        assert_eq!(segments.iter().map(Vec::len).collect::<Vec<_>>(), [20, 40, 40], "neighbouring pages are joined");

        let files: Vec<PointFile> = segments.into_iter().map(|segments| log.point_file(attributes, segments)).collect();
        drop(log);
        for (file, chunk) in files.iter().zip(&chunks) {
            assert_eq!(file.point_count, 40_000);
            assert_bit_exact(&file.read_all().unwrap(), chunk);
        }
        for file in files {
            assert!(path.exists(), "the log stays while a file uses it");
            file.remove().unwrap();
        }
        assert!(!path.exists(), "the last file deletes the log");
    }

    #[test]
    fn temp_folders_are_unique_and_removed_on_drop() {
        let first = TempFolder::create(&std::env::temp_dir()).unwrap();
        let second = TempFolder::create(&std::env::temp_dir()).unwrap();
        assert_ne!(first.path(), second.path());
        fs::create_dir(first.path().join("inner")).unwrap();
        fs::write(first.path().join("inner").join("file.bin"), b"data").unwrap();
        let path = first.path().to_path_buf();
        drop(first);
        assert!(!path.exists());
        assert!(second.path().is_dir());
    }
}
