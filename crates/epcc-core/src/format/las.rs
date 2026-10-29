//! LAS and LAZ, through the `las` crate. LAZ is LAS with compressed point records; the crate
//! compresses and decompresses on all cores.
//!
//! The point records themselves are decoded and encoded here rather than through the crate's
//! `Point`, so that every standard field and the extra bytes come along. COPC writes its records
//! with the same code.

use std::fs::File;
use std::io::{BufReader, BufWriter};
use std::path::{Path, PathBuf};

use las::point::Format as PointFormat;
use las::{Builder, PointData, PointDataBuilder, Transform, Vector, Version, Vlr};
use rayon::prelude::*;

use super::{PointReader, PointWriter, WriteRequest};
use crate::error::IoContext;
use crate::model::{
    Attributes, CHUNK_POINTS, CloudInfo, ExtraBytes, ExtraField, LidarDetails, PointChunk, PointRecord, Returns,
    ScalarType, ScanInfo, color_from_u8,
};
use crate::settings::{LasSettings, LasVersion};
use crate::{Error, Progress, Result};

pub(super) const WKT_USER_ID: &str = "LASF_Projection";
pub(super) const WKT_RECORD_ID: u16 = 2112;
pub(super) const EXTRA_BYTES_USER_ID: &str = "LASF_Spec";
pub(super) const EXTRA_BYTES_RECORD_ID: u16 = 4;
pub(super) const EXTRA_BYTES_DESCRIPTION: &str = "Extra Bytes";
/// Bytes of one entry of the extra bytes record.
const DESCRIPTOR_SIZE: usize = 192;
/// Where the parts of an extra bytes descriptor lie.
const DESCRIPTOR_NAME: std::ops::Range<usize> = 4..36;
const DESCRIPTOR_SCALE: usize = 112;
const DESCRIPTOR_OFFSET: usize = 136;
const DESCRIPTOR_DESCRIPTION: std::ops::Range<usize> = 160..192;
/// Option bits of an extra bytes descriptor.
const OPTION_SCALE: u8 = 1 << 3;
const OPTION_OFFSET: u8 = 1 << 4;

/// Up to LAS 1.3, class 12 marks overlap points; LAS 1.4 point formats have a flag instead.
const OVERLAP_CLASS: u8 = 12;
const UNCLASSIFIED: u8 = 1;
/// The unit of scan angles in LAS 1.4 point formats, in degrees.
const SCAN_ANGLE_STEP: f32 = 0.006;
/// Returns per pulse that LAS 1.2 stores.
const LEGACY_MAX_RETURNS: u8 = 5;

/// Points converted to LAS records per parallel task.
const RECORDS_PER_TASK: usize = 1 << 16;

pub(super) fn open(path: &Path) -> Result<Box<dyn PointReader>> {
    let reader = open_reader(path)?;
    let header = reader.header();
    let format = header.point_format();
    let info = CloudInfo {
        attributes: Attributes {
            intensity: true,
            color: format.has_color,
            normal: false,
            classification: true,
            time: format.has_gps_time,
            returns: true,
            lidar: true,
            extra_bytes: format.extra_bytes,
        },
        scans: vec![ScanInfo::named(super::Format::stem_of(path))],
        point_count: Some(header.number_of_points()),
        crs_wkt: wkt_of(header),
        extra_fields: extra_fields_of(header, format.extra_bytes),
    };
    Ok(Box::new(LasReader { path: path.to_path_buf(), info }))
}

fn open_reader(path: &Path) -> Result<las::Reader> {
    let file = File::open(path).at(path)?;
    Ok(las::Reader::new(BufReader::with_capacity(1 << 20, file))?)
}

/// `las::Reader` cannot move between threads, so the file is opened again in [`PointReader::read`],
/// which runs on the reading thread.
struct LasReader {
    path: PathBuf,
    info: CloudInfo,
}

impl PointReader for LasReader {
    fn info(&self) -> &CloudInfo {
        &self.info
    }

    fn read(self: Box<Self>, emit: &mut dyn FnMut(PointChunk) -> Result<()>, progress: &Progress) -> Result<()> {
        let mut reader = open_reader(&self.path)?;
        let total = self.info.point_count.unwrap_or(0);
        let mut data = PointDataBuilder::new().for_header(reader.header()).build();
        let mut colors = ColorDepth::Unknown;
        let mut done = 0;
        loop {
            let read = reader.fill_points(CHUNK_POINTS as u64, &mut data)?;
            if read == 0 {
                return Ok(());
            }
            emit(to_chunk(&data, self.info.attributes, &mut colors))?;
            done += read;
            progress.set_done_of(done, total);
        }
    }
}

/// Many LAS writers store 8-bit colours in the 16-bit fields. If the first chunk has no value
/// above 255, the file is taken to be one of those and its colours are scaled up.
#[derive(Clone, Copy, PartialEq)]
enum ColorDepth {
    Unknown,
    EightBit,
    SixteenBit,
}

fn to_chunk(data: &PointData, attributes: Attributes, depth: &mut ColorDepth) -> PointChunk {
    let positions = data.x().zip(data.y()).zip(data.z()).map(|((x, y), z)| [x, y, z]).collect();
    let record_len = data.record_len();
    let extended = data.format().is_extended;
    let records = || data.raw_bytes().chunks_exact(record_len);
    let mut chunk = PointChunk {
        scan: 0,
        positions,
        intensity: Some(data.intensity().map(|value| f32::from(value) / 65535.0).collect()),
        classification: Some(data.classification().collect()),
        time: data.gps_time().map(|times| times.collect()),
        returns: Some(records().map(|record| returns_of(record, extended)).collect()),
        lidar: Some(records().map(|record| lidar_of(record, extended)).collect()),
        extra: attributes.has_extra().then(|| {
            let width = usize::from(attributes.extra_bytes);
            let bytes = records().flat_map(|record| &record[record_len - width..]).copied().collect();
            ExtraBytes { width, bytes }
        }),
        ..PointChunk::default()
    };
    if attributes.color
        && let Some(rgb) = data.rgb()
    {
        let rgb: Vec<[u16; 3]> = rgb.map(|(r, g, b)| [r, g, b]).collect();
        if *depth == ColorDepth::Unknown {
            let eight_bit = rgb.iter().flatten().all(|&channel| channel <= 255);
            *depth = if eight_bit { ColorDepth::EightBit } else { ColorDepth::SixteenBit };
        }
        chunk.color = Some(match depth {
            ColorDepth::EightBit => rgb.iter().map(|c| c.map(|v| color_from_u8(v.min(255) as u8))).collect(),
            _ => rgb,
        });
    }
    chunk
}

fn returns_of(record: &[u8], extended: bool) -> Returns {
    let bits = record[14];
    if extended {
        Returns { number: bits & 15, count: bits >> 4 }
    } else {
        Returns { number: bits & 7, count: (bits >> 3) & 7 }
    }
}

fn lidar_of(record: &[u8], extended: bool) -> LidarDetails {
    if extended {
        LidarDetails {
            scan_angle: f32::from(i16::from_le_bytes([record[18], record[19]])) * SCAN_ANGLE_STEP,
            point_source: u16::from_le_bytes([record[20], record[21]]),
            user_data: record[17],
            flags: record[15],
        }
    } else {
        // Scan direction and edge of flight line have the same bits as in LAS 1.4.
        let direction_and_edge = record[14] & (LidarDetails::SCAN_DIRECTION | LidarDetails::EDGE_OF_FLIGHT_LINE);
        LidarDetails {
            scan_angle: f32::from(record[16] as i8),
            point_source: u16::from_le_bytes([record[18], record[19]]),
            user_data: record[17],
            flags: (record[15] >> 5) | direction_and_edge,
        }
    }
}

fn wkt_of(header: &las::Header) -> Option<String> {
    header
        .all_vlrs()
        .find(|vlr| vlr.user_id == WKT_USER_ID && vlr.record_id == WKT_RECORD_ID)
        .map(|vlr| String::from_utf8_lossy(&vlr.data).trim_end_matches('\0').to_string())
}

/// The fields of the `width` extra bytes per point, from the extra bytes record. Bytes it does
/// not describe, or all of them if it does not fit the records, become one unnamed field.
fn extra_fields_of(header: &las::Header, width: u16) -> Vec<ExtraField> {
    if width == 0 {
        return Vec::new();
    }
    let described: Option<Vec<ExtraField>> = header
        .all_vlrs()
        .find(|vlr| vlr.user_id == EXTRA_BYTES_USER_ID && vlr.record_id == EXTRA_BYTES_RECORD_ID)
        .and_then(|vlr| vlr.data.as_chunks::<DESCRIPTOR_SIZE>().0.iter().map(parse_descriptor).collect());
    let mut fields = described.filter(|fields| ExtraField::total_size(fields) <= width).unwrap_or_default();
    let undescribed = width - ExtraField::total_size(&fields);
    if undescribed > 0 {
        fields.push(ExtraField {
            name: "extra bytes".into(),
            description: "Bytes without a description".into(),
            size: undescribed,
            scalar: None,
            scale: 1.0,
            offset: 0.0,
            las_descriptor: None,
        });
    }
    fields
}

/// Reads one extra bytes descriptor. `None` for a reserved data type, whose size is unknown.
fn parse_descriptor(descriptor: &[u8; DESCRIPTOR_SIZE]) -> Option<ExtraField> {
    let (data_type, options) = (descriptor[2], descriptor[3]);
    let (size, scalar) = match data_type {
        // Undocumented bytes; the options give their number.
        0 => (usize::from(options), None),
        1..=10 => {
            let scalar = ScalarType::ALL[usize::from(data_type - 1)];
            (scalar.size(), Some(scalar))
        }
        // Arrays of two or three numbers, deprecated since LAS 1.4 R14. Carried as plain bytes.
        11..=30 => {
            let base = ScalarType::ALL[usize::from((data_type - 11) % 10)];
            (base.size() * usize::from((data_type - 11) / 10 + 2), None)
        }
        _ => return None,
    };
    let number_at = |offset: usize| f64::from_le_bytes(descriptor[offset..offset + 8].try_into().expect("8 bytes"));
    let scale = number_at(DESCRIPTOR_SCALE);
    let scaled = scalar.is_some() && options & OPTION_SCALE != 0 && scale != 0.0;
    let shifted = scalar.is_some() && options & OPTION_OFFSET != 0;
    Some(ExtraField {
        name: text_of(&descriptor[DESCRIPTOR_NAME]),
        description: text_of(&descriptor[DESCRIPTOR_DESCRIPTION]),
        size: size as u16,
        scalar,
        scale: if scaled { scale } else { 1.0 },
        offset: if shifted { number_at(DESCRIPTOR_OFFSET) } else { 0.0 },
        las_descriptor: Some(descriptor.to_vec()),
    })
}

/// A text field that ends at its first zero byte.
fn text_of(bytes: &[u8]) -> String {
    let end = bytes.iter().position(|&byte| byte == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..end]).trim().to_string()
}

/// The contents of the extra bytes record for `fields`. Fields read from LAS keep their
/// descriptors as they were.
pub(super) fn extra_bytes_record(fields: &[ExtraField]) -> Vec<u8> {
    let mut data = Vec::with_capacity(fields.len() * DESCRIPTOR_SIZE);
    for field in fields {
        match &field.las_descriptor {
            Some(descriptor) => data.extend_from_slice(descriptor),
            None => describe(field, &mut data),
        }
    }
    data
}

fn describe(field: &ExtraField, data: &mut Vec<u8>) {
    let mut descriptor = |data_type: u8, options: u8, size: u16| {
        let mut entry = [0u8; DESCRIPTOR_SIZE];
        entry[2] = data_type;
        entry[3] = options;
        put_text(&mut entry[DESCRIPTOR_NAME], &field.name);
        put_text(&mut entry[DESCRIPTOR_DESCRIPTION], &field.description);
        if data_type != 0 && field.is_scaled() {
            entry[3] |= OPTION_SCALE | OPTION_OFFSET;
            entry[DESCRIPTOR_SCALE..DESCRIPTOR_SCALE + 8].copy_from_slice(&field.scale.to_le_bytes());
            entry[DESCRIPTOR_OFFSET..DESCRIPTOR_OFFSET + 8].copy_from_slice(&field.offset.to_le_bytes());
        }
        debug_assert!(data_type != 0 || size <= 255);
        data.extend_from_slice(&entry);
    };
    match field.scalar {
        Some(scalar) => {
            let code = ScalarType::ALL.iter().position(|&known| known == scalar).expect("every type has a code");
            descriptor(code as u8 + 1, 0, field.size);
        }
        // Undocumented bytes are described 255 at a time, the most one entry can hold.
        None => {
            let mut left = field.size;
            while left > 0 {
                let part = left.min(255);
                descriptor(0, part as u8, part);
                left -= part;
            }
        }
    }
}

/// Copies `text` into a zero-padded field, cut to its width.
fn put_text(field: &mut [u8], text: &str) {
    let mut end = text.len().min(field.len());
    while !text.is_char_boundary(end) {
        end -= 1;
    }
    field[..end].copy_from_slice(&text.as_bytes()[..end]);
}

/// The point records EPCC writes: LAS point format 0 to 3 for LAS 1.2, or 6 or 7 for LAS 1.4,
/// each followed by the extra bytes.
#[derive(Clone, Copy, Debug)]
pub(super) struct RecordLayout {
    pub extended: bool,
    pub time: bool,
    pub color: bool,
    pub extra_bytes: u16,
}

impl RecordLayout {
    pub(super) fn new(attributes: Attributes, extended: bool) -> RecordLayout {
        RecordLayout {
            extended,
            time: extended || attributes.time,
            color: attributes.color,
            extra_bytes: attributes.extra_bytes,
        }
    }

    pub(super) fn format_number(&self) -> u8 {
        if self.extended { 6 + u8::from(self.color) } else { u8::from(self.time) + 2 * u8::from(self.color) }
    }

    /// Bytes per record, extra bytes included.
    pub(super) fn size(&self) -> usize {
        let base = if self.extended { 30 } else { 20 + 8 * usize::from(self.time) };
        base + 6 * usize::from(self.color) + usize::from(self.extra_bytes)
    }

    fn point_format(&self, compressed: bool) -> Result<PointFormat> {
        let mut format = PointFormat::new(self.format_number())?;
        format.extra_bytes = self.extra_bytes;
        format.is_compressed = compressed;
        Ok(format)
    }

    /// Appends the record of `point`, whose position is already quantised to `xyz`.
    pub(super) fn encode(&self, point: &PointRecord, xyz: [i32; 3], extra: &[u8], out: &mut Vec<u8>) {
        for value in xyz {
            out.extend_from_slice(&value.to_le_bytes());
        }
        out.extend_from_slice(&((point.intensity * 65535.0).round() as u16).to_le_bytes());
        let Returns { number, count } = point.returns;
        let lidar = point.lidar;
        if self.extended {
            let (class, overlap) = match point.classification {
                OVERLAP_CLASS => (UNCLASSIFIED, LidarDetails::OVERLAP),
                class => (class, 0),
            };
            out.push(number.min(15) | count.min(15) << 4);
            out.push(lidar.flags | overlap);
            out.push(class);
            out.push(lidar.user_data);
            let angle = (lidar.scan_angle / SCAN_ANGLE_STEP).round().clamp(-30_000.0, 30_000.0) as i16;
            out.extend_from_slice(&angle.to_le_bytes());
            out.extend_from_slice(&lidar.point_source.to_le_bytes());
        } else {
            let class = if lidar.flags & LidarDetails::OVERLAP != 0 {
                OVERLAP_CLASS
            } else if point.classification > 31 {
                UNCLASSIFIED
            } else {
                point.classification
            };
            let direction_and_edge = lidar.flags & (LidarDetails::SCAN_DIRECTION | LidarDetails::EDGE_OF_FLIGHT_LINE);
            out.push(number.min(LEGACY_MAX_RETURNS) | count.min(LEGACY_MAX_RETURNS) << 3 | direction_and_edge);
            out.push(class | (lidar.flags & (LidarDetails::CLASS_FLAGS & !LidarDetails::OVERLAP)) << 5);
            out.push(lidar.scan_angle.round().clamp(-90.0, 90.0) as i8 as u8);
            out.push(lidar.user_data);
            out.extend_from_slice(&lidar.point_source.to_le_bytes());
        }
        if self.time {
            out.extend_from_slice(&point.time.to_le_bytes());
        }
        if self.color {
            point.color.iter().for_each(|channel| out.extend_from_slice(&channel.to_le_bytes()));
        }
        out.extend_from_slice(extra);
    }
}

pub(super) fn create(request: &WriteRequest, compressed: bool) -> Result<Box<dyn PointWriter>> {
    let settings = request.settings.las.clone();
    let version = choose_version(&settings, request.cloud);
    Ok(Box::new(LasWriter {
        path: request.path.to_path_buf(),
        cloud: request.cloud.clone(),
        settings,
        version,
        compressed,
    }))
}

struct LasWriter {
    path: PathBuf,
    cloud: CloudInfo,
    settings: LasSettings,
    version: Version,
    compressed: bool,
}

impl PointWriter for LasWriter {
    fn write(self: Box<Self>, chunks: &mut dyn Iterator<Item = PointChunk>, progress: &Progress) -> Result<()> {
        let mut chunks = chunks.peekable();
        // The offset only has to keep coordinates within the integer range, so any point of the
        // cloud will do. The first one is known before anything is written.
        let origin = chunks.peek().and_then(|chunk| chunk.positions.first().copied()).unwrap_or_default();
        let layout = RecordLayout::new(self.cloud.attributes, self.version.minor >= 4);
        let header = self.header(origin, layout, progress)?;
        let format = *header.point_format();
        let transforms = *header.transforms();

        let file = File::create(&self.path).at(&self.path)?;
        let mut writer = las::Writer::new(BufWriter::with_capacity(1 << 20, file), header)?;
        let mut losses = LegacyLosses::default();
        for chunk in chunks {
            if !layout.extended {
                losses.check(&chunk, progress);
            }
            for records in to_records(&chunk, layout, format, transforms)? {
                writer.write_points(&records)?;
            }
        }
        writer.close()?;
        Ok(())
    }
}

impl LasWriter {
    fn header(&self, origin: [f64; 3], layout: RecordLayout, progress: &Progress) -> Result<las::Header> {
        let axis = |origin: f64| Transform { scale: self.settings.precision, offset: origin.floor() };
        let mut builder = Builder::new(Default::default())?;
        builder.version = self.version;
        builder.point_format = layout.point_format(self.compressed)?;
        builder.transforms = Vector { x: axis(origin[0]), y: axis(origin[1]), z: axis(origin[2]) };
        builder.generating_software = format!("EPCC {}", crate::VERSION);

        if let Some(wkt) = &self.cloud.crs_wkt {
            if layout.extended {
                builder.has_wkt_crs = true;
                let mut data = wkt.clone().into_bytes();
                data.push(0);
                builder.vlrs.push(Vlr {
                    user_id: WKT_USER_ID.into(),
                    record_id: WKT_RECORD_ID,
                    description: "OGC WKT coordinate system".into(),
                    data,
                });
            } else {
                progress
                    .warn("LAS 1.2 cannot store a WKT coordinate system; it was left out. Choose LAS 1.4 to keep it.");
            }
        }
        if layout.extra_bytes > 0 {
            builder.vlrs.push(Vlr {
                user_id: EXTRA_BYTES_USER_ID.into(),
                record_id: EXTRA_BYTES_RECORD_ID,
                description: EXTRA_BYTES_DESCRIPTION.into(),
                data: extra_bytes_record(&self.cloud.extra_fields),
            });
        }
        Ok(builder.into_header()?)
    }
}

fn choose_version(settings: &LasSettings, cloud: &CloudInfo) -> Version {
    let needs_1_4 = cloud.crs_wkt.is_some() || cloud.point_count.is_some_and(|count| count > u64::from(u32::MAX));
    match settings.version {
        LasVersion::V1_2 => Version::new(1, 2),
        LasVersion::V1_4 => Version::new(1, 4),
        LasVersion::Auto if needs_1_4 => Version::new(1, 4),
        LasVersion::Auto => Version::new(1, 2),
    }
}

/// What LAS 1.2 cannot store, each reported once.
#[derive(Default)]
struct LegacyLosses {
    classes: bool,
    returns: bool,
    channels: bool,
}

impl LegacyLosses {
    fn check(&mut self, chunk: &PointChunk, progress: &Progress) {
        if !self.classes && chunk.classification.as_ref().is_some_and(|classes| classes.iter().any(|&c| c > 31)) {
            progress.warn("LAS 1.2 stores classes 0 to 31 only; higher classes were written as 1 (unclassified).");
            self.classes = true;
        }
        let many = |returns: &Returns| returns.number.max(returns.count) > LEGACY_MAX_RETURNS;
        if !self.returns && chunk.returns.as_ref().is_some_and(|returns| returns.iter().any(many)) {
            progress.warn(
                "LAS 1.2 stores up to 5 returns per pulse; higher return numbers were written as 5. \
                 Choose LAS 1.4 to keep them.",
            );
            self.returns = true;
        }
        if !self.channels && chunk.lidar.as_ref().is_some_and(|lidar| lidar.iter().any(|d| d.scanner_channel() > 0)) {
            progress.warn("LAS 1.2 cannot store scanner channels; they were left out. Choose LAS 1.4 to keep them.");
            self.channels = true;
        }
    }
}

/// Converts a chunk to LAS records, in parallel slices that are written in order.
fn to_records(
    chunk: &PointChunk,
    layout: RecordLayout,
    format: PointFormat,
    transforms: Vector<Transform>,
) -> Result<Vec<PointData>> {
    let builder = PointDataBuilder::new().with_format(format).with_transforms(transforms);
    (0..chunk.len())
        .step_by(RECORDS_PER_TASK)
        .collect::<Vec<_>>()
        .into_par_iter()
        .map(|start| {
            let end = (start + RECORDS_PER_TASK).min(chunk.len());
            let mut bytes = Vec::with_capacity((end - start) * layout.size());
            for index in start..end {
                let point = chunk.record(index);
                let [x, y, z] = point.position;
                let xyz = [transforms.x.inverse(x)?, transforms.y.inverse(y)?, transforms.z.inverse(z)?];
                let extra = chunk.extra.as_ref().map_or(&[][..], |extra| extra.get(index));
                layout.encode(&point, xyz, extra, &mut bytes);
            }
            builder.clone().build_from_bytes(bytes).map_err(Error::from)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_layouts_match_the_las_point_formats() {
        for extended in [false, true] {
            for (time, color) in [(false, false), (true, false), (false, true), (true, true)] {
                let attributes = Attributes { time, color, extra_bytes: 5, ..Attributes::NONE };
                let layout = RecordLayout::new(attributes, extended);
                let format = layout.point_format(false).unwrap();
                assert_eq!(layout.size(), usize::from(format.len()), "format {}", layout.format_number());
                let mut record = Vec::new();
                layout.encode(&PointRecord::default(), [0; 3], &[7; 5], &mut record);
                assert_eq!(record.len(), layout.size());
            }
        }
    }

    #[test]
    fn point_fields_decode_as_they_were_encoded() {
        let point = PointRecord {
            classification: 6,
            returns: Returns { number: 2, count: 3 },
            lidar: LidarDetails {
                scan_angle: -12.0,
                point_source: 4321,
                user_data: 99,
                flags: LidarDetails::SYNTHETIC | LidarDetails::WITHHELD | LidarDetails::EDGE_OF_FLIGHT_LINE,
            },
            ..PointRecord::default()
        };
        for extended in [false, true] {
            let mut record = Vec::new();
            RecordLayout::new(Attributes::NONE, extended).encode(&point, [1, 2, 3], &[], &mut record);
            assert_eq!(returns_of(&record, extended), point.returns);
            let lidar = lidar_of(&record, extended);
            assert_eq!((lidar.point_source, lidar.user_data, lidar.flags), (4321, 99, point.lidar.flags));
            assert!((lidar.scan_angle - point.lidar.scan_angle).abs() < 1e-4, "{}", lidar.scan_angle);
        }
    }

    #[test]
    fn extra_bytes_descriptors_round_trip() {
        let mut scaled = ExtraField::number("height above ground", ScalarType::I32);
        scaled.description = "Metres".into();
        scaled.scale = 0.01;
        scaled.offset = -5.0;
        let opaque = ExtraField {
            name: "blob".into(),
            description: String::new(),
            size: 300,
            scalar: None,
            scale: 1.0,
            offset: 0.0,
            las_descriptor: None,
        };
        let fields = [ExtraField::number("amplitude", ScalarType::F64), scaled, opaque];
        let record = extra_bytes_record(&fields);
        assert_eq!(record.len(), 4 * DESCRIPTOR_SIZE, "300 undocumented bytes need two entries");
        let parsed: Vec<ExtraField> =
            record.as_chunks::<DESCRIPTOR_SIZE>().0.iter().map(|entry| parse_descriptor(entry).unwrap()).collect();
        let summary = |field: &ExtraField| (field.name.clone(), field.size, field.scalar, field.scale, field.offset);
        assert_eq!(summary(&parsed[0]), summary(&fields[0]));
        assert_eq!(summary(&parsed[1]), summary(&fields[1]));
        assert_eq!(parsed[1].description, "Metres");
        assert_eq!((parsed[2].size, parsed[3].size), (255, 45));
        assert_eq!(ExtraField::total_size(&parsed), ExtraField::total_size(&fields));
        // Read fields keep their descriptors unchanged.
        assert_eq!(extra_bytes_record(&parsed), record);
    }

    #[test]
    fn deprecated_arrays_are_carried_as_bytes() {
        let mut entry = [0u8; DESCRIPTOR_SIZE];
        entry[2] = 29; // three floats
        let field = parse_descriptor(&entry).unwrap();
        assert_eq!((field.size, field.scalar), (12, None));
        entry[2] = 31;
        assert!(parse_descriptor(&entry).is_none(), "reserved types have no known size");
    }
}
