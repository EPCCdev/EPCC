//! PLY, binary (either byte order) and ASCII. Only the `vertex` element is read; faces and
//! other elements are skipped.
//!
//! Vertex properties EPCC does not know become extra values when reading binary files, and
//! extra values are written as vertex properties of their own.

use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use rayon::prelude::*;

use super::text::{self, Column, LineFormat, TextReader, encode_in_parallel, write_fixed, write_int};
use super::{PointReader, PointWriter, WriteRequest};
use crate::convert::field_names;
use crate::error::IoContext;
use crate::model::{
    Attributes, CHUNK_POINTS, CloudInfo, ExtraField, LidarDetails, PointChunk, PointRecord, ScalarType, ScanInfo,
    ValueRange, color_to_u8,
};
use crate::settings::{CoordinateType, PlyEncoding};
use crate::{Error, Progress, Result};

/// Vertices decoded per parallel task.
const VERTICES_PER_TASK: usize = 1 << 16;
/// Vertices looked at to find the range of float intensities.
const INTENSITY_SAMPLE: usize = 10_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Encoding {
    Ascii,
    LittleEndian,
    BigEndian,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scalar {
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    F32,
    F64,
}

impl Scalar {
    fn from_name(name: &str) -> Option<Scalar> {
        let scalar = match name {
            "char" | "int8" => Scalar::I8,
            "uchar" | "uint8" => Scalar::U8,
            "short" | "int16" => Scalar::I16,
            "ushort" | "uint16" => Scalar::U16,
            "int" | "int32" => Scalar::I32,
            "uint" | "uint32" => Scalar::U32,
            "float" | "float32" => Scalar::F32,
            "double" | "float64" => Scalar::F64,
            _ => return None,
        };
        Some(scalar)
    }

    fn name(self) -> &'static str {
        match self {
            Scalar::I8 => "char",
            Scalar::U8 => "uchar",
            Scalar::I16 => "short",
            Scalar::U16 => "ushort",
            Scalar::I32 => "int",
            Scalar::U32 => "uint",
            Scalar::F32 => "float",
            Scalar::F64 => "double",
        }
    }

    fn to_model(self) -> ScalarType {
        match self {
            Scalar::I8 => ScalarType::I8,
            Scalar::U8 => ScalarType::U8,
            Scalar::I16 => ScalarType::I16,
            Scalar::U16 => ScalarType::U16,
            Scalar::I32 => ScalarType::I32,
            Scalar::U32 => ScalarType::U32,
            Scalar::F32 => ScalarType::F32,
            Scalar::F64 => ScalarType::F64,
        }
    }

    /// The PLY type that holds `scalar` exactly; PLY has no 64-bit integers.
    fn from_model(scalar: ScalarType) -> Option<Scalar> {
        let scalar = match scalar {
            ScalarType::I8 => Scalar::I8,
            ScalarType::U8 => Scalar::U8,
            ScalarType::I16 => Scalar::I16,
            ScalarType::U16 => Scalar::U16,
            ScalarType::I32 => Scalar::I32,
            ScalarType::U32 => Scalar::U32,
            ScalarType::F32 => Scalar::F32,
            ScalarType::F64 => Scalar::F64,
            ScalarType::U64 | ScalarType::I64 => return None,
        };
        Some(scalar)
    }

    fn size(self) -> usize {
        match self {
            Scalar::I8 | Scalar::U8 => 1,
            Scalar::I16 | Scalar::U16 => 2,
            Scalar::I32 | Scalar::U32 | Scalar::F32 => 4,
            Scalar::F64 => 8,
        }
    }

    /// The value range of integer types; floats have none.
    fn integer_range(self) -> Option<ValueRange> {
        let (min, max) = match self {
            Scalar::I8 => (i8::MIN as f64, i8::MAX as f64),
            Scalar::U8 => (0.0, u8::MAX as f64),
            Scalar::I16 => (i16::MIN as f64, i16::MAX as f64),
            Scalar::U16 => (0.0, u16::MAX as f64),
            Scalar::I32 => (i32::MIN as f64, i32::MAX as f64),
            Scalar::U32 => (0.0, u32::MAX as f64),
            Scalar::F32 | Scalar::F64 => return None,
        };
        Some(ValueRange { min, max })
    }

    fn decode(self, bytes: &[u8], big_endian: bool) -> f64 {
        macro_rules! number {
            ($type:ty) => {{
                let raw = bytes[..std::mem::size_of::<$type>()].try_into().expect("slice has the type's size");
                (if big_endian { <$type>::from_be_bytes(raw) } else { <$type>::from_le_bytes(raw) }) as f64
            }};
        }
        match self {
            Scalar::I8 => number!(i8),
            Scalar::U8 => number!(u8),
            Scalar::I16 => number!(i16),
            Scalar::U16 => number!(u16),
            Scalar::I32 => number!(i32),
            Scalar::U32 => number!(u32),
            Scalar::F32 => number!(f32),
            Scalar::F64 => number!(f64),
        }
    }
}

#[derive(Clone, Debug)]
struct Property {
    name: String,
    /// `None` for list properties, which only faces use.
    scalar: Option<Scalar>,
}

#[derive(Clone, Debug)]
struct Element {
    name: String,
    count: u64,
    properties: Vec<Property>,
}

impl Element {
    /// Bytes per record in binary files, if every property has a fixed size.
    fn stride(&self) -> Option<usize> {
        self.properties.iter().map(|p| p.scalar.map(Scalar::size)).sum()
    }
}

struct Header {
    encoding: Encoding,
    elements: Vec<Element>,
    /// Byte offset where the data starts.
    data_start: u64,
}

fn read_header(path: &Path) -> Result<Header> {
    let mut reader = BufReader::new(File::open(path).at(path)?);
    let mut line = String::new();
    let mut offset = 0u64;
    let mut next_line = |line: &mut String| -> Result<bool> {
        line.clear();
        let read = reader.read_line(line).at(path)?;
        offset += read as u64;
        Ok(read > 0)
    };

    next_line(&mut line)?;
    if line.trim_end() != "ply" {
        return Err(Error::invalid(format!("{}: not a PLY file", path.display())));
    }
    let mut encoding = None;
    let mut elements: Vec<Element> = Vec::new();
    loop {
        if !next_line(&mut line)? {
            return Err(Error::invalid(format!("{}: PLY header has no end_header", path.display())));
        }
        let words: Vec<&str> = line.split_whitespace().collect();
        match words.as_slice() {
            ["end_header"] => break,
            ["format", "ascii", _] => encoding = Some(Encoding::Ascii),
            ["format", "binary_little_endian", _] => encoding = Some(Encoding::LittleEndian),
            ["format", "binary_big_endian", _] => encoding = Some(Encoding::BigEndian),
            ["element", name, count] => elements.push(Element {
                name: name.to_string(),
                count: count.parse().map_err(|_| Error::invalid(format!("PLY element count \"{count}\"")))?,
                properties: Vec::new(),
            }),
            ["property", "list", _, _, name] => push_property(&mut elements, name, None)?,
            ["property", kind, name] => {
                let scalar =
                    Scalar::from_name(kind).ok_or_else(|| Error::invalid(format!("PLY property type \"{kind}\"")))?;
                push_property(&mut elements, name, Some(scalar))?;
            }
            _ => {} // comments, obj_info
        }
    }
    let encoding =
        encoding.ok_or_else(|| Error::invalid(format!("{}: PLY header has no format line", path.display())))?;
    Ok(Header { encoding, elements, data_start: offset })
}

fn push_property(elements: &mut [Element], name: &str, scalar: Option<Scalar>) -> Result<()> {
    let element = elements.last_mut().ok_or_else(|| Error::invalid("PLY property before any element"))?;
    element.properties.push(Property { name: name.to_string(), scalar });
    Ok(())
}

/// What a vertex property means to us, if it is one we know.
fn column_of(property: &str) -> Option<Column> {
    match property.to_ascii_lowercase().as_str() {
        "diffuse_red" => Some(Column::Red),
        "diffuse_green" => Some(Column::Green),
        "diffuse_blue" => Some(Column::Blue),
        name => Column::from_name(name),
    }
}

pub(super) fn open(path: &Path) -> Result<Box<dyn PointReader>> {
    let header = read_header(path)?;
    let vertex_index = header
        .elements
        .iter()
        .position(|element| element.name == "vertex")
        .ok_or_else(|| Error::invalid(format!("{}: PLY file has no vertices", path.display())))?;
    let vertex = header.elements[vertex_index].clone();
    let columns: Vec<Option<Column>> = vertex.properties.iter().map(|p| column_of(&p.name)).collect();
    if ![Column::X, Column::Y, Column::Z].iter().all(|&axis| columns.contains(&Some(axis))) {
        return Err(Error::invalid(format!("{}: PLY vertices have no x, y and z", path.display())));
    }
    match header.encoding {
        Encoding::Ascii => open_ascii(path, &header, vertex_index, columns),
        Encoding::LittleEndian | Encoding::BigEndian => open_binary(path, &header, vertex_index, columns),
    }
}

/// ASCII vertices are parsed like a text file, which has no room for extra values; unknown
/// properties are skipped.
fn open_ascii(
    path: &Path,
    header: &Header,
    vertex_index: usize,
    columns: Vec<Option<Column>>,
) -> Result<Box<dyn PointReader>> {
    let columns: Vec<Column> = columns.into_iter().map(|column| column.unwrap_or(Column::Skip)).collect();
    let vertex = &header.elements[vertex_index];
    let lines_before: u64 = header.elements[..vertex_index].iter().map(|e| e.count).sum();
    let start = line_offset(path, header.data_start, lines_before)?;
    let end =
        if vertex_index + 1 < header.elements.len() { Some(line_offset(path, start, vertex.count)?) } else { None };
    let type_of = |column| columns.iter().position(|&c| c == column).and_then(|index| vertex.properties[index].scalar);
    let format = LineFormat {
        intensity: type_of(Column::Intensity).and_then(Scalar::integer_range).unwrap_or(ValueRange::UNIT),
        color: type_of(Column::Red).and_then(Scalar::integer_range).unwrap_or(ValueRange::UNIT),
        columns,
        ..LineFormat::default()
    };
    let reader = TextReader::new(path, format, 0).limited_to(start, end).with_point_count(vertex.count);
    Ok(Box::new(reader))
}

/// The byte offset `lines` lines after `from`.
fn line_offset(path: &Path, from: u64, lines: u64) -> Result<u64> {
    let mut file = File::open(path).at(path)?;
    file.seek(SeekFrom::Start(from)).at(path)?;
    let mut reader = BufReader::with_capacity(1 << 20, file);
    let mut offset = from;
    let mut remaining = lines;
    while remaining > 0 {
        let buffer = reader.fill_buf().at(path)?;
        if buffer.is_empty() {
            break;
        }
        let mut used = buffer.len();
        for (index, &byte) in buffer.iter().enumerate() {
            if byte == b'\n' {
                remaining -= 1;
                if remaining == 0 {
                    used = index + 1;
                    break;
                }
            }
        }
        offset += used as u64;
        reader.consume(used);
    }
    Ok(offset)
}

/// Where each attribute sits inside a binary vertex record.
#[derive(Clone, Debug, Default)]
struct VertexLayout {
    stride: usize,
    fields: Vec<(Column, usize, Scalar)>,
    /// Properties kept as extra values: where they are in a record and their type.
    extra: Vec<(usize, Scalar)>,
    extra_bytes: usize,
    big_endian: bool,
    intensity: Option<ValueRange>,
    color: Option<ValueRange>,
}

impl VertexLayout {
    fn attributes(&self) -> Attributes {
        let columns: Vec<Column> = self.fields.iter().map(|&(column, _, _)| column).collect();
        Attributes { extra_bytes: self.extra_bytes as u16, ..text::attributes_of(&columns) }
    }

    /// Copies the extra values of `record` into `out`, as little-endian numbers.
    fn decode_extra(&self, record: &[u8], out: &mut [u8]) {
        let mut at = 0;
        for &(offset, scalar) in &self.extra {
            let size = scalar.size();
            let value = &mut out[at..at + size];
            value.copy_from_slice(&record[offset..offset + size]);
            if self.big_endian {
                value.reverse();
            }
            at += size;
        }
    }

    fn decode(&self, record: &[u8]) -> PointRecord {
        let mut point = PointRecord::default();
        for &(column, offset, scalar) in &self.fields {
            let value = scalar.decode(&record[offset..], self.big_endian);
            match column {
                Column::X => point.position[0] = value,
                Column::Y => point.position[1] = value,
                Column::Z => point.position[2] = value,
                Column::Intensity => point.intensity = self.intensity.unwrap_or(ValueRange::UNIT).normalize(value),
                Column::Red => point.color[0] = self.color16(value),
                Column::Green => point.color[1] = self.color16(value),
                Column::Blue => point.color[2] = self.color16(value),
                other => other.store_plain(value, &mut point),
            }
        }
        point
    }

    fn color16(&self, value: f64) -> u16 {
        (f64::from(self.color.unwrap_or(ValueRange::UNIT).normalize(value)) * 65535.0).round() as u16
    }

    fn decode_all(&self, records: &[u8]) -> PointChunk {
        let attributes = self.attributes();
        let pieces: Vec<PointChunk> = records
            .par_chunks(self.stride * VERTICES_PER_TASK)
            .map(|piece| {
                let mut chunk = PointChunk::with_capacity(0, attributes, piece.len() / self.stride);
                for record in piece.chunks_exact(self.stride) {
                    chunk.push(&self.decode(record));
                    if let Some(extra) = &mut chunk.extra {
                        let index = extra.bytes.len() / extra.width - 1;
                        self.decode_extra(record, extra.get_mut(index));
                    }
                }
                chunk
            })
            .collect();
        let mut all = PointChunk::with_capacity(0, attributes, records.len() / self.stride);
        for piece in pieces {
            all.append(piece);
        }
        all
    }
}

fn open_binary(
    path: &Path,
    header: &Header,
    vertex_index: usize,
    columns: Vec<Option<Column>>,
) -> Result<Box<dyn PointReader>> {
    let mut start = header.data_start;
    for element in &header.elements[..vertex_index] {
        let stride = element.stride().ok_or_else(|| {
            Error::Unsupported(format!("{}: PLY lists before the vertices are not supported", path.display()))
        })?;
        start += stride as u64 * element.count;
    }
    let vertex = &header.elements[vertex_index];
    let stride = vertex.stride().ok_or_else(|| {
        Error::Unsupported(format!("{}: PLY vertices with list properties are not supported", path.display()))
    })?;

    let mut layout =
        VertexLayout { stride, big_endian: header.encoding == Encoding::BigEndian, ..VertexLayout::default() };
    let mut offset = 0;
    let mut extra_fields = Vec::new();
    for (property, column) in vertex.properties.iter().zip(columns) {
        let scalar = property.scalar.expect("stride exists, so every property is a scalar");
        match column {
            Some(Column::Skip) => {}
            Some(column) => layout.fields.push((column, offset, scalar)),
            None => {
                layout.extra.push((offset, scalar));
                layout.extra_bytes += scalar.size();
                extra_fields.push(ExtraField::number(&property.name, scalar.to_model()));
            }
        }
        let column = column.unwrap_or(Column::Skip);
        if column == Column::Red {
            layout.color = Some(scalar.integer_range().unwrap_or(ValueRange::UNIT));
        }
        if column == Column::Intensity {
            layout.intensity = scalar.integer_range();
        }
        offset += scalar.size();
    }
    if layout.attributes().intensity && layout.intensity.is_none() {
        layout.intensity = Some(sample_float_intensity(path, start, vertex.count, &layout)?);
    }

    let info = CloudInfo {
        attributes: layout.attributes(),
        scans: vec![ScanInfo::named(path.file_stem().map(|s| s.to_string_lossy().into_owned()).unwrap_or_default())],
        point_count: Some(vertex.count),
        crs_wkt: None,
        extra_fields,
    };
    Ok(Box::new(BinaryPlyReader { path: path.to_path_buf(), start, count: vertex.count, layout, info }))
}

/// Float intensities have no declared range, so it is guessed from the first vertices.
fn sample_float_intensity(path: &Path, start: u64, count: u64, layout: &VertexLayout) -> Result<ValueRange> {
    let (_, offset, scalar) =
        *layout.fields.iter().find(|(column, _, _)| *column == Column::Intensity).expect("intensity field exists");
    let sample = count.min(INTENSITY_SAMPLE as u64) as usize;
    let mut records = vec![0; sample * layout.stride];
    let mut file = File::open(path).at(path)?;
    file.seek(SeekFrom::Start(start)).at(path)?;
    file.read_exact(&mut records).at(path)?;
    let values = records.chunks_exact(layout.stride).map(|record| scalar.decode(&record[offset..], layout.big_endian));
    Ok(ValueRange::guess_intensity(values))
}

struct BinaryPlyReader {
    path: PathBuf,
    start: u64,
    count: u64,
    layout: VertexLayout,
    info: CloudInfo,
}

impl PointReader for BinaryPlyReader {
    fn info(&self) -> &CloudInfo {
        &self.info
    }

    fn read(self: Box<Self>, emit: &mut dyn FnMut(PointChunk) -> Result<()>, progress: &Progress) -> Result<()> {
        let mut file = File::open(&self.path).at(&self.path)?;
        file.seek(SeekFrom::Start(self.start)).at(&self.path)?;
        let mut records = Vec::new();
        let mut done = 0u64;
        while done < self.count {
            let batch = (self.count - done).min(CHUNK_POINTS as u64) as usize;
            records.resize(batch * self.layout.stride, 0);
            file.read_exact(&mut records).at(&self.path)?;
            emit(self.layout.decode_all(&records))?;
            done += batch as u64;
            progress.set_done_of(done, self.count);
        }
        Ok(())
    }
}

pub(super) fn create(request: &WriteRequest) -> Result<Box<dyn PointWriter>> {
    let attributes = request.cloud.attributes;
    let (extra, left_out) = extra_properties(&request.cloud.extra_fields, &standard_properties(attributes));
    Ok(Box::new(PlyWriter {
        path: request.path.to_path_buf(),
        attributes,
        extra,
        left_out,
        encoding: request.settings.ply.encoding,
        coordinates: request.settings.ply.coordinates,
    }))
}

struct PlyWriter {
    path: PathBuf,
    attributes: Attributes,
    extra: Vec<ExtraProperty>,
    /// Extra values that are not numbers, which PLY cannot hold.
    left_out: Vec<ExtraField>,
    encoding: PlyEncoding,
    coordinates: CoordinateType,
}

/// An extra value written as a vertex property.
struct ExtraProperty {
    name: String,
    /// Where the value lies among a point's extra bytes.
    at: usize,
    field: ExtraField,
    /// The type it is written as.
    scalar: Scalar,
    /// Scaled values and 64-bit integers are written as their value, in a double; everything
    /// else as it is stored.
    as_value: bool,
}

impl ExtraProperty {
    fn value(&self, extra: &[u8]) -> f64 {
        self.field.value(&extra[self.at..]).unwrap_or_default()
    }
}

/// The lidar details as vertex properties, with their PLY types.
const LIDAR_PROPERTIES: [(&str, &str); 7] = [
    ("float", "scan_angle"),
    ("uchar", "user_data"),
    ("ushort", "point_source_id"),
    ("uchar", "class_flags"),
    ("uchar", "scanner_channel"),
    ("uchar", "scan_direction"),
    ("uchar", "edge_of_flight_line"),
];

/// The names of the properties written for `attributes`.
fn standard_properties(a: Attributes) -> Vec<&'static str> {
    let mut names = vec!["x", "y", "z"];
    let groups: [(bool, &[&'static str]); 6] = [
        (a.intensity, &["intensity"]),
        (a.color, &["red", "green", "blue"]),
        (a.normal, &["nx", "ny", "nz"]),
        (a.classification, &["classification"]),
        (a.time, &["gps_time"]),
        (a.returns, &["return_number", "number_of_returns"]),
    ];
    names.extend(groups.into_iter().filter(|(present, _)| *present).flat_map(|(_, group)| group.iter().copied()));
    if a.lidar {
        names.extend(LIDAR_PROPERTIES.iter().map(|&(_, name)| name));
    }
    names
}

/// Which extra values become properties, with names that are valid and unique, and which are
/// left out.
fn extra_properties(fields: &[ExtraField], taken: &[&str]) -> (Vec<ExtraProperty>, Vec<ExtraField>) {
    let mut used: Vec<String> = taken.iter().map(|name| name.to_string()).collect();
    let mut properties = Vec::new();
    let mut left_out = Vec::new();
    let mut at = 0;
    for field in fields {
        if let Some(stored) = field.scalar {
            let mut name: String = field.name.chars().map(|c| if c.is_ascii_graphic() { c } else { '_' }).collect();
            if name.is_empty() {
                name = "extra".into();
            }
            let base = name.clone();
            let mut number = 2;
            while used.iter().any(|used| used.eq_ignore_ascii_case(&name)) {
                name = format!("{base}_{number}");
                number += 1;
            }
            used.push(name.clone());
            let exact = Scalar::from_model(stored).filter(|_| !field.is_scaled());
            properties.push(ExtraProperty {
                name,
                at,
                field: field.clone(),
                scalar: exact.unwrap_or(Scalar::F64),
                as_value: exact.is_none(),
            });
        } else {
            left_out.push(field.clone());
        }
        at += usize::from(field.size);
    }
    (properties, left_out)
}

/// Room reserved for the vertex count, which is only known at the end.
const COUNT_WIDTH: usize = 20;

impl PointWriter for PlyWriter {
    fn write(self: Box<Self>, chunks: &mut dyn Iterator<Item = PointChunk>, progress: &Progress) -> Result<()> {
        if !self.left_out.is_empty() {
            progress.warn(format!(
                "PLY can only store extra values that are numbers; {} left out.",
                field_names(&self.left_out)
            ));
        }
        let mut chunks = chunks.peekable();
        let double = match self.coordinates {
            CoordinateType::Float32 => false,
            CoordinateType::Float64 => true,
            CoordinateType::Auto => chunks.peek().is_some_and(needs_double),
        };
        let file = File::create(&self.path).at(&self.path)?;
        let mut out = BufWriter::with_capacity(1 << 20, file);
        let (header, count_at) = self.header(double);
        out.write_all(header.as_bytes()).at(&self.path)?;

        let mut count = 0u64;
        for chunk in chunks {
            count += chunk.len() as u64;
            let pieces = match self.encoding {
                PlyEncoding::Binary => {
                    encode_in_parallel(&chunk, 40, |point, extra, out| self.encode_binary(point, extra, double, out))
                }
                PlyEncoding::Ascii => {
                    encode_in_parallel(&chunk, 60, |point, extra, out| self.encode_ascii(point, extra, out))
                }
            };
            for piece in pieces {
                out.write_all(&piece).at(&self.path)?;
            }
        }
        out.seek(SeekFrom::Start(count_at as u64)).at(&self.path)?;
        write!(out, "{count:>width$}", width = COUNT_WIDTH).at(&self.path)?;
        out.flush().at(&self.path)
    }
}

/// The class flags, scanner channel, scan direction and edge of flight line, as written.
fn lidar_flag_values(lidar: &LidarDetails) -> [u8; 4] {
    [
        lidar.flags & LidarDetails::CLASS_FLAGS,
        lidar.scanner_channel(),
        u8::from(lidar.flags & LidarDetails::SCAN_DIRECTION != 0),
        u8::from(lidar.flags & LidarDetails::EDGE_OF_FLIGHT_LINE != 0),
    ]
}

/// 32-bit floats resolve about 0.1 mm up to 1 km from the origin; beyond that, use 64 bits.
fn needs_double(chunk: &PointChunk) -> bool {
    chunk.positions.iter().flatten().any(|value| value.abs() > 1000.0)
}

impl PlyWriter {
    /// The header text, and the byte offset of the vertex count in it.
    fn header(&self, double: bool) -> (String, usize) {
        let format = match self.encoding {
            PlyEncoding::Binary => "binary_little_endian",
            PlyEncoding::Ascii => "ascii",
        };
        let coordinate = if double { "double" } else { "float" };
        let mut header = format!("ply\nformat {format} 1.0\nelement vertex ");
        let count_at = header.len();
        header += &format!("{:width$}\n", "", width = COUNT_WIDTH);
        for axis in ["x", "y", "z"] {
            header += &format!("property {coordinate} {axis}\n");
        }
        let a = self.attributes;
        if a.intensity {
            header += "property float intensity\n";
        }
        if a.color {
            header += "property uchar red\nproperty uchar green\nproperty uchar blue\n";
        }
        if a.normal {
            header += "property float nx\nproperty float ny\nproperty float nz\n";
        }
        if a.classification {
            header += "property uchar classification\n";
        }
        if a.time {
            header += "property double gps_time\n";
        }
        if a.returns {
            header += "property uchar return_number\nproperty uchar number_of_returns\n";
        }
        if a.lidar {
            for (kind, name) in LIDAR_PROPERTIES {
                header += &format!("property {kind} {name}\n");
            }
        }
        for property in &self.extra {
            header += &format!("property {} {}\n", property.scalar.name(), property.name);
        }
        header += "end_header\n";
        (header, count_at)
    }

    fn encode_binary(&self, point: &PointRecord, extra: &[u8], double: bool, out: &mut Vec<u8>) {
        for value in point.position {
            if double {
                out.extend_from_slice(&value.to_le_bytes());
            } else {
                out.extend_from_slice(&(value as f32).to_le_bytes());
            }
        }
        let a = self.attributes;
        if a.intensity {
            out.extend_from_slice(&point.intensity.to_le_bytes());
        }
        if a.color {
            out.extend(point.color.map(color_to_u8));
        }
        if a.normal {
            for component in point.normal {
                out.extend_from_slice(&component.to_le_bytes());
            }
        }
        if a.classification {
            out.push(point.classification);
        }
        if a.time {
            out.extend_from_slice(&point.time.to_le_bytes());
        }
        if a.returns {
            out.extend([point.returns.number, point.returns.count]);
        }
        if a.lidar {
            let lidar = &point.lidar;
            out.extend_from_slice(&lidar.scan_angle.to_le_bytes());
            out.push(lidar.user_data);
            out.extend_from_slice(&lidar.point_source.to_le_bytes());
            out.extend(lidar_flag_values(lidar));
        }
        for property in &self.extra {
            if property.as_value {
                out.extend_from_slice(&property.value(extra).to_le_bytes());
            } else {
                out.extend_from_slice(&extra[property.at..property.at + property.scalar.size()]);
            }
        }
    }

    fn encode_ascii(&self, point: &PointRecord, extra: &[u8], out: &mut Vec<u8>) {
        let mut values = Vec::with_capacity(12);
        for value in point.position {
            values.push((value, 6));
        }
        let a = self.attributes;
        if a.intensity {
            values.push((f64::from(point.intensity), 6));
        }
        if a.color {
            values.extend(point.color.map(|channel| (f64::from(color_to_u8(channel)), 0)));
        }
        if a.normal {
            values.extend(point.normal.map(|component| (f64::from(component), 6)));
        }
        if a.classification {
            values.push((f64::from(point.classification), 0));
        }
        if a.time {
            values.push((point.time, 6));
        }
        if a.returns {
            values.extend([point.returns.number, point.returns.count].map(|value| (f64::from(value), 0)));
        }
        if a.lidar {
            let lidar = &point.lidar;
            values.push((f64::from(lidar.scan_angle), 3));
            values.push((f64::from(lidar.user_data), 0));
            values.push((f64::from(lidar.point_source), 0));
            values.extend(lidar_flag_values(lidar).map(|value| (f64::from(value), 0)));
        }
        for property in &self.extra {
            let integer = !property.as_value && !matches!(property.scalar, Scalar::F32 | Scalar::F64);
            values.push((property.value(extra), if integer { 0 } else { 6 }));
        }
        for (index, (value, decimals)) in values.into_iter().enumerate() {
            if index > 0 {
                out.push(b' ');
            }
            if decimals == 0 {
                write_int(out, value as i64);
            } else {
                write_fixed(out, value, decimals);
            }
        }
        out.push(b'\n');
    }
}
