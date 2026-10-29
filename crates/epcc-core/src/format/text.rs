//! Shared machinery for the text formats: column detection, parallel line parsing and fast
//! fixed-point number formatting.

use std::fs::File;
use std::io::{BufWriter, Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use rayon::prelude::*;

use super::{PointReader, PointWriter};
use crate::error::IoContext;
use crate::model::{
    Attributes, CloudInfo, LidarDetails, PointChunk, PointRecord, TASK_POINTS, ValueRange, color_to_u8,
};
use crate::{Error, Progress, Result};

/// Bytes read from disk at a time. Each block is parsed on all cores.
const BLOCK_BYTES: usize = 32 << 20;
/// Bytes one parallel task parses.
const PIECE_BYTES: usize = 1 << 20;
/// Bytes looked at to detect the layout of a file.
const SNIFF_BYTES: usize = 1 << 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Column {
    X,
    Y,
    Z,
    Intensity,
    Red,
    Green,
    Blue,
    NormalX,
    NormalY,
    NormalZ,
    Classification,
    Time,
    ReturnNumber,
    NumberOfReturns,
    ScanAngle,
    UserData,
    PointSource,
    /// The LAS classification flags: synthetic, key point, withheld and overlap as bits 0 to 3.
    ClassFlags,
    ScannerChannel,
    ScanDirection,
    EdgeOfFlightLine,
    Skip,
}

impl Column {
    /// Understands the usual header spellings, e.g. `X`, `//X`, `red`, `scalar_Intensity`.
    pub(super) fn from_name(name: &str) -> Option<Column> {
        let name = name.trim().trim_start_matches("//").to_ascii_lowercase();
        let name = name.strip_prefix("scalar_").unwrap_or(&name);
        let column = match name {
            "x" => Column::X,
            "y" => Column::Y,
            "z" => Column::Z,
            "i" | "intensity" => Column::Intensity,
            "r" | "red" => Column::Red,
            "g" | "green" => Column::Green,
            "b" | "blue" => Column::Blue,
            "nx" | "normal_x" | "normalx" => Column::NormalX,
            "ny" | "normal_y" | "normaly" => Column::NormalY,
            "nz" | "normal_z" | "normalz" => Column::NormalZ,
            "c" | "class" | "classification" => Column::Classification,
            "t" | "time" | "gps_time" | "gpstime" | "timestamp" => Column::Time,
            "return_number" | "returnnumber" => Column::ReturnNumber,
            "number_of_returns" | "numberofreturns" => Column::NumberOfReturns,
            "scan_angle" | "scanangle" | "scan_angle_rank" | "scananglerank" => Column::ScanAngle,
            "user_data" | "userdata" => Column::UserData,
            "point_source_id" | "pointsourceid" | "point_source" => Column::PointSource,
            "class_flags" | "classflags" => Column::ClassFlags,
            "scanner_channel" | "scannerchannel" | "scan_channel" | "scanchannel" => Column::ScannerChannel,
            "scan_direction" | "scan_direction_flag" | "scandirectionflag" => Column::ScanDirection,
            "edge_of_flight_line" | "edgeofflightline" => Column::EdgeOfFlightLine,
            "_" | "-" | "skip" => Column::Skip,
            _ => return None,
        };
        Some(column)
    }

    /// Stores `value` in `point` for the columns whose values need no conversion settings.
    /// Positions, intensity and colour are left to the caller.
    pub(super) fn store_plain(self, value: f64, point: &mut PointRecord) {
        let byte = |max: f64| value.clamp(0.0, max) as u8;
        let set_flag = |flags: &mut u8, flag: u8| {
            if value != 0.0 {
                *flags |= flag;
            } else {
                *flags &= !flag;
            }
        };
        let lidar = &mut point.lidar;
        match self {
            Column::NormalX => point.normal[0] = value as f32,
            Column::NormalY => point.normal[1] = value as f32,
            Column::NormalZ => point.normal[2] = value as f32,
            Column::Classification => point.classification = byte(255.0),
            Column::Time => point.time = value,
            Column::ReturnNumber => point.returns.number = byte(15.0),
            Column::NumberOfReturns => point.returns.count = byte(15.0),
            Column::ScanAngle => lidar.scan_angle = value as f32,
            Column::UserData => lidar.user_data = byte(255.0),
            Column::PointSource => lidar.point_source = value.clamp(0.0, 65535.0) as u16,
            Column::ClassFlags => {
                lidar.flags = (lidar.flags & !LidarDetails::CLASS_FLAGS) | byte(15.0);
            }
            Column::ScannerChannel => {
                lidar.flags = (lidar.flags & !LidarDetails::CHANNEL) | (byte(3.0) << LidarDetails::CHANNEL_SHIFT);
            }
            Column::ScanDirection => set_flag(&mut lidar.flags, LidarDetails::SCAN_DIRECTION),
            Column::EdgeOfFlightLine => set_flag(&mut lidar.flags, LidarDetails::EDGE_OF_FLIGHT_LINE),
            Column::X
            | Column::Y
            | Column::Z
            | Column::Intensity
            | Column::Red
            | Column::Green
            | Column::Blue
            | Column::Skip => {}
        }
    }
}

/// Parses a column list such as `x,y,z,i,r,g,b`. Commas or spaces separate the names.
pub(super) fn parse_column_list(spec: &str) -> Result<Vec<Column>> {
    let columns = spec
        .split([',', ' ', ';'])
        .filter(|name| !name.is_empty())
        .map(|name| {
            Column::from_name(name).ok_or_else(|| {
                Error::InvalidSetting(format!(
                    "Unknown column \"{name}\". Use x, y, z, i, r, g, b, nx, ny, nz, class, time or _ (skip)."
                ))
            })
        })
        .collect::<Result<Vec<_>>>()?;
    if ![Column::X, Column::Y, Column::Z].iter().all(|axis| columns.contains(axis)) {
        return Err(Error::InvalidSetting("The column list needs x, y and z.".into()));
    }
    Ok(columns)
}

pub(super) fn attributes_of(columns: &[Column]) -> Attributes {
    let has = |column| columns.contains(&column);
    Attributes {
        intensity: has(Column::Intensity),
        color: has(Column::Red) || has(Column::Green) || has(Column::Blue),
        normal: has(Column::NormalX) || has(Column::NormalY) || has(Column::NormalZ),
        classification: has(Column::Classification),
        time: has(Column::Time),
        returns: has(Column::ReturnNumber) || has(Column::NumberOfReturns),
        lidar: [
            Column::ScanAngle,
            Column::UserData,
            Column::PointSource,
            Column::ClassFlags,
            Column::ScannerChannel,
            Column::ScanDirection,
            Column::EdgeOfFlightLine,
        ]
        .into_iter()
        .any(has),
        extra_bytes: 0,
    }
}

/// What the first part of a text file looks like.
pub(super) struct Sniffed {
    /// Leading lines that hold column names instead of numbers.
    pub header_lines: usize,
    /// Column names from such a header line, if they could all be understood.
    pub header_columns: Option<Vec<Column>>,
    /// The first data lines, parsed.
    pub samples: Vec<Vec<f64>>,
    /// Numbers are written as `1,5` and fields separated by `;`.
    pub decimal_comma: bool,
}

impl Sniffed {
    /// The number of fields most sample lines have.
    pub(super) fn field_count(&self) -> usize {
        let mut counts = std::collections::HashMap::<usize, usize>::new();
        for sample in &self.samples {
            *counts.entry(sample.len()).or_default() += 1;
        }
        counts.into_iter().max_by_key(|&(len, count)| (count, len)).map_or(0, |(len, _)| len)
    }

    /// Sample values of field `index`.
    pub(super) fn field(&self, index: usize) -> impl Iterator<Item = f64> + '_ {
        self.samples.iter().filter_map(move |sample| sample.get(index).copied())
    }

    /// Whether field `index` holds whole numbers from 0 to 255 or 65535, like colour channels.
    pub(super) fn looks_like_color(&self, index: usize) -> bool {
        self.field(index).all(|v| v.fract() == 0.0 && (0.0..=65535.0).contains(&v))
    }

    pub(super) fn looks_like_normal(&self, index: usize) -> bool {
        self.field(index).all(|v| (-1.0..=1.0).contains(&v))
    }
}

pub(super) fn sniff(path: &Path) -> Result<Sniffed> {
    let mut start = Vec::with_capacity(SNIFF_BYTES);
    File::open(path).at(path)?.take(SNIFF_BYTES as u64).read_to_end(&mut start).at(path)?;
    let text = String::from_utf8_lossy(&start);
    // The last line may be cut off, so it is not used.
    let complete_lines = text.lines().take(text.lines().count().saturating_sub(1).max(1));

    let decimal_comma = text.contains(';') && text.lines().skip(1).take(10).any(|line| line.contains(','));
    let format = LineFormat { decimal_comma, ..LineFormat::default() };
    let mut sniffed = Sniffed { header_lines: 0, header_columns: None, samples: Vec::new(), decimal_comma };
    for (index, line) in complete_lines.enumerate() {
        let fields: Vec<&[u8]> = format.fields(line.trim().as_bytes()).collect();
        if fields.is_empty() {
            continue;
        }
        let numbers: Option<Vec<f64>> = fields.iter().map(|field| format.number(field)).collect();
        match numbers {
            Some(numbers) if numbers.len() >= 3 => sniffed.samples.push(numbers),
            Some(_) => {} // a point count or similar, which the parser ignores anyway
            None if sniffed.samples.is_empty() => {
                // Text before the first point: a title, a comment or column names.
                sniffed.header_lines = index + 1;
                let names: Option<Vec<Column>> =
                    line.split([',', ';', ' ', '\t']).filter(|name| !name.is_empty()).map(Column::from_name).collect();
                if names.as_ref().is_some_and(|names| names.contains(&Column::X)) {
                    sniffed.header_columns = names;
                }
            }
            None => {}
        }
        if sniffed.samples.len() >= 2000 {
            break;
        }
    }
    if sniffed.samples.is_empty() {
        return Err(Error::invalid(format!("{}: no lines with coordinates found", path.display())));
    }
    Ok(sniffed)
}

/// How the fields of a line turn into a point.
#[derive(Clone, Debug)]
pub(super) struct LineFormat {
    pub columns: Vec<Column>,
    pub intensity: ValueRange,
    pub color: ValueRange,
    /// Multiplies coordinates to get metres.
    pub unit_scale: f64,
    /// Lines holding a single whole number start a new scan (PTS) instead of being ignored.
    pub scan_breaks: bool,
    pub decimal_comma: bool,
}

impl Default for LineFormat {
    fn default() -> Self {
        LineFormat {
            columns: vec![Column::X, Column::Y, Column::Z],
            intensity: ValueRange::UNIT,
            color: ValueRange::BYTE,
            unit_scale: 1.0,
            scan_breaks: false,
            decimal_comma: false,
        }
    }
}

enum Line {
    Point,
    Ignored,
    ScanBreak,
    Invalid,
}

impl LineFormat {
    pub(super) fn attributes(&self) -> Attributes {
        attributes_of(&self.columns)
    }

    /// Picks the colour range from sample values: 0..1 floats, 8 bit or 16 bit.
    pub(super) fn color_range_from(samples: impl IntoIterator<Item = f64>) -> ValueRange {
        let mut max = 0.0f64;
        let mut whole = true;
        for value in samples {
            max = max.max(value);
            whole &= value.fract() == 0.0;
        }
        if max <= 1.0 && !whole {
            ValueRange::UNIT
        } else if max <= 255.0 {
            ValueRange::BYTE
        } else {
            ValueRange::WORD
        }
    }

    fn fields<'a>(&self, line: &'a [u8]) -> impl Iterator<Item = &'a [u8]> {
        let decimal_comma = self.decimal_comma;
        line.split(move |&byte| byte == b' ' || byte == b'\t' || byte == b';' || (byte == b',' && !decimal_comma))
            .filter(|field| !field.is_empty())
    }

    fn number(&self, field: &[u8]) -> Option<f64> {
        if self.decimal_comma && field.contains(&b',') {
            let fixed: Vec<u8> = field.iter().map(|&b| if b == b',' { b'.' } else { b }).collect();
            fast_float2::parse(fixed).ok()
        } else {
            fast_float2::parse(field).ok()
        }
    }

    fn parse_line(&self, line: &[u8], point: &mut PointRecord) -> Line {
        let line = line.strip_suffix(b"\r").unwrap_or(line);
        if is_comment(line) {
            return Line::Ignored;
        }
        *point = PointRecord::default();
        let mut found = 0;
        for (field, column) in self.fields(line).zip(&self.columns) {
            if *column == Column::Skip {
                found += 1;
                continue;
            }
            let Some(value) = self.number(field) else {
                return Line::Invalid;
            };
            found += 1;
            match column {
                Column::X => point.position[0] = value * self.unit_scale,
                Column::Y => point.position[1] = value * self.unit_scale,
                Column::Z => point.position[2] = value * self.unit_scale,
                Column::Intensity => point.intensity = self.intensity.normalize(value),
                Column::Red => point.color[0] = self.color16(value),
                Column::Green => point.color[1] = self.color16(value),
                Column::Blue => point.color[2] = self.color16(value),
                other => other.store_plain(value, point),
            }
        }
        match found {
            0 => Line::Ignored,
            1 if self.scan_breaks => Line::ScanBreak,
            1 | 2 => Line::Ignored, // point counts and similar
            _ if found >= self.position_fields() => Line::Point,
            _ => Line::Invalid,
        }
    }

    /// How many leading fields a line needs to contain x, y and z.
    fn position_fields(&self) -> usize {
        self.columns
            .iter()
            .rposition(|column| matches!(column, Column::X | Column::Y | Column::Z))
            .map_or(3, |index| index + 1)
    }

    fn color16(&self, value: f64) -> u16 {
        (f64::from(self.color.normalize(value)) * 65535.0).round() as u16
    }
}

fn is_comment(line: &[u8]) -> bool {
    line.starts_with(b"#") || line.starts_with(b"//")
}

/// Points parsed from a piece of text, split where a new scan starts.
struct Parsed {
    segments: Vec<Segment>,
    invalid_lines: u64,
}

struct Segment {
    starts_scan: bool,
    points: PointChunk,
}

/// Parses complete lines on all cores, keeping their order.
fn parse_block(text: &[u8], format: &LineFormat) -> Parsed {
    let pieces = split_at_lines(text, PIECE_BYTES);
    let parsed: Vec<Parsed> = pieces.into_par_iter().map(|piece| parse_piece(piece, format)).collect();
    let mut all = Parsed { segments: Vec::new(), invalid_lines: 0 };
    for piece in parsed {
        all.invalid_lines += piece.invalid_lines;
        for segment in piece.segments {
            match all.segments.last_mut() {
                Some(last) if !segment.starts_scan => last.points.append(segment.points),
                _ => all.segments.push(segment),
            }
        }
    }
    all
}

fn parse_piece(text: &[u8], format: &LineFormat) -> Parsed {
    let attributes = format.attributes();
    let new_segment =
        |starts_scan| Segment { starts_scan, points: PointChunk::with_capacity(0, attributes, text.len() / 40) };
    let mut parsed = Parsed { segments: vec![new_segment(false)], invalid_lines: 0 };
    let mut point = PointRecord::default();
    for line in text.split(|&byte| byte == b'\n') {
        match format.parse_line(line, &mut point) {
            Line::Point => parsed.segments.last_mut().expect("never empty").points.push(&point),
            Line::Ignored => {}
            Line::ScanBreak => parsed.segments.push(new_segment(true)),
            Line::Invalid => parsed.invalid_lines += 1,
        }
    }
    parsed
}

/// Splits `text` into pieces of about `size` bytes that end at line breaks.
fn split_at_lines(text: &[u8], size: usize) -> Vec<&[u8]> {
    let mut pieces = Vec::new();
    let mut rest = text;
    while rest.len() > size {
        let cut = rest[size..].iter().position(|&byte| byte == b'\n').map_or(rest.len(), |at| size + at + 1);
        let (piece, tail) = rest.split_at(cut);
        pieces.push(piece);
        rest = tail;
    }
    if !rest.is_empty() {
        pieces.push(rest);
    }
    pieces
}

/// Reads any line-based point format.
pub(super) struct TextReader {
    path: PathBuf,
    format: LineFormat,
    header_lines: usize,
    /// The part of the file holding points; formats with a binary header or trailing data
    /// (ASCII PLY) narrow it down.
    start: u64,
    end: Option<u64>,
    info: CloudInfo,
}

impl TextReader {
    pub(super) fn new(path: &Path, format: LineFormat, header_lines: usize) -> Self {
        let info = CloudInfo {
            attributes: format.attributes(),
            // Scans are only found while reading; they get numbered names.
            scans: Vec::new(),
            point_count: None,
            crs_wkt: None,
            extra_fields: Vec::new(),
        };
        TextReader { path: path.to_path_buf(), format, header_lines, start: 0, end: None, info }
    }

    /// Reads only bytes `start..end` of the file.
    pub(super) fn limited_to(mut self, start: u64, end: Option<u64>) -> Self {
        self.start = start;
        self.end = end;
        self
    }

    pub(super) fn with_point_count(mut self, count: u64) -> Self {
        self.info.point_count = Some(count);
        self
    }
}

impl PointReader for TextReader {
    fn info(&self) -> &CloudInfo {
        &self.info
    }

    fn read(self: Box<Self>, emit: &mut dyn FnMut(PointChunk) -> Result<()>, progress: &Progress) -> Result<()> {
        let mut file = File::open(&self.path).at(&self.path)?;
        let file_len = file.metadata().at(&self.path)?.len();
        let end = self.end.unwrap_or(file_len).min(file_len);
        file.seek(SeekFrom::Start(self.start)).at(&self.path)?;
        let total = end.saturating_sub(self.start);
        let mut file = file.take(total);
        let mut done = 0u64;
        let mut carry = Vec::new();
        let mut first_block = true;
        let mut scan = 0;
        let mut scan_has_points = false;
        let mut invalid_lines = 0;

        loop {
            let mut block = std::mem::take(&mut carry);
            let before = block.len();
            (&mut file).take(BLOCK_BYTES as u64).read_to_end(&mut block).at(&self.path)?;
            let at_end = block.len() == before;
            done += (block.len() - before) as u64;

            let cut = if at_end {
                block.len()
            } else {
                match block.iter().rposition(|&byte| byte == b'\n') {
                    Some(newline) => newline + 1,
                    None => {
                        carry = block; // a single line longer than a block: read on
                        continue;
                    }
                }
            };
            carry = block.split_off(cut);
            let mut text = block.as_slice();
            if first_block {
                text = skip_lines(text, self.header_lines);
                first_block = false;
            }

            let parsed = parse_block(text, &self.format);
            invalid_lines += parsed.invalid_lines;
            for mut segment in parsed.segments {
                if segment.starts_scan && scan_has_points {
                    scan += 1;
                    scan_has_points = false;
                }
                if !segment.points.is_empty() {
                    segment.points.scan = scan;
                    scan_has_points = true;
                    emit(segment.points)?;
                }
            }
            progress.set_done_of(done, total);
            if at_end {
                break;
            }
        }
        if invalid_lines > 0 {
            progress.warn(format!("{invalid_lines} lines could not be read and were skipped."));
        }
        Ok(())
    }
}

fn skip_lines(text: &[u8], count: usize) -> &[u8] {
    let mut rest = text;
    for _ in 0..count {
        rest = rest.iter().position(|&byte| byte == b'\n').map_or(&[][..], |at| &rest[at + 1..]);
    }
    rest
}

/// How each written line looks.
#[derive(Clone, Debug)]
pub(super) struct LineStyle {
    pub separator: u8,
    pub decimals: u8,
    pub intensity: IntensityStyle,
    pub color: bool,
    pub normal: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) enum IntensityStyle {
    None,
    /// Whole numbers from -2048 to 2047.
    Pts,
    /// A constant 0, for formats that need the column even without intensity.
    PtsZero,
    /// 0 to 1.
    Unit,
}

impl LineStyle {
    fn write_line(&self, point: &PointRecord, out: &mut Vec<u8>) {
        let separator = self.separator;
        for (axis, &value) in point.position.iter().enumerate() {
            if axis > 0 {
                out.push(separator);
            }
            write_fixed(out, value, self.decimals);
        }
        match self.intensity {
            IntensityStyle::None => {}
            IntensityStyle::Pts => {
                out.push(separator);
                write_int(out, ValueRange::PTS.denormalize(point.intensity).round() as i64);
            }
            IntensityStyle::PtsZero => {
                out.push(separator);
                out.push(b'0');
            }
            IntensityStyle::Unit => {
                out.push(separator);
                write_fixed(out, f64::from(point.intensity), 4);
            }
        }
        if self.color {
            for channel in point.color {
                out.push(separator);
                write_int(out, i64::from(color_to_u8(channel)));
            }
        }
        if self.normal {
            for component in point.normal {
                out.push(separator);
                write_fixed(out, f64::from(component), 6);
            }
        }
        out.push(b'\n');
    }
}

/// Writes line-based point formats, optionally with a point count as first line.
pub(super) struct TextWriter {
    pub path: PathBuf,
    pub style: LineStyle,
    pub count_line: bool,
}

/// Room reserved for the point count. The count is only known at the end, and is then written
/// right-aligned into this space; readers skip leading spaces.
const COUNT_WIDTH: usize = 20;

impl PointWriter for TextWriter {
    fn write(self: Box<Self>, chunks: &mut dyn Iterator<Item = PointChunk>, _progress: &Progress) -> Result<()> {
        let file = File::create(&self.path).at(&self.path)?;
        let mut out = BufWriter::with_capacity(1 << 20, file);
        if self.count_line {
            writeln!(out, "{:width$}", "", width = COUNT_WIDTH).at(&self.path)?;
        }
        let mut count = 0u64;
        for chunk in chunks {
            count += chunk.len() as u64;
            for text in encode_in_parallel(&chunk, 48, |point, _, out| self.style.write_line(point, out)) {
                out.write_all(&text).at(&self.path)?;
            }
        }
        if self.count_line {
            out.seek(SeekFrom::Start(0)).at(&self.path)?;
            write!(out, "{count:>width$}", width = COUNT_WIDTH).at(&self.path)?;
        }
        out.flush().at(&self.path)
    }
}

/// Encodes every point of a chunk on all cores, given with its extra values. The returned pieces
/// are in point order.
pub(super) fn encode_in_parallel(
    chunk: &PointChunk,
    bytes_per_point: usize,
    encode: impl Fn(&PointRecord, &[u8], &mut Vec<u8>) + Sync,
) -> Vec<Vec<u8>> {
    (0..chunk.len())
        .step_by(TASK_POINTS)
        .collect::<Vec<_>>()
        .into_par_iter()
        .map(|start| {
            let end = (start + TASK_POINTS).min(chunk.len());
            let mut bytes = Vec::with_capacity((end - start) * bytes_per_point);
            for index in start..end {
                let extra = chunk.extra.as_ref().map_or(&[][..], |extra| extra.get(index));
                encode(&chunk.record(index), extra, &mut bytes);
            }
            bytes
        })
        .collect()
}

const POWERS_OF_TEN: [f64; 10] = [1.0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9];

/// Writes `value` with exactly `decimals` decimal places. Much faster than `format!`, which
/// matters when writing billions of numbers.
pub(super) fn write_fixed(out: &mut Vec<u8>, value: f64, decimals: u8) {
    let decimals = usize::from(decimals.min(9));
    let scaled = (value * POWERS_OF_TEN[decimals]).round();
    if !scaled.is_finite() || scaled.abs() >= 9.0e18 {
        out.extend_from_slice(format!("{value:.decimals$}").as_bytes());
        return;
    }
    let scaled = scaled as i64;
    if scaled < 0 {
        out.push(b'-');
    }
    let digits = scaled.unsigned_abs();
    let unit = 10u64.pow(decimals as u32);
    write_digits(out, digits / unit, 1);
    if decimals > 0 {
        out.push(b'.');
        write_digits(out, digits % unit, decimals);
    }
}

pub(super) fn write_int(out: &mut Vec<u8>, value: i64) {
    if value < 0 {
        out.push(b'-');
    }
    write_digits(out, value.unsigned_abs(), 1);
}

/// Writes `value` in decimal, left-padded with zeros to at least `min_width` digits.
fn write_digits(out: &mut Vec<u8>, mut value: u64, min_width: usize) {
    let mut digits = [b'0'; 20];
    let mut at = digits.len();
    while value > 0 || digits.len() - at < min_width {
        at -= 1;
        digits[at] = b'0' + (value % 10) as u8;
        value /= 10;
    }
    out.extend_from_slice(&digits[at..]);
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixed(value: f64, decimals: u8) -> String {
        let mut out = Vec::new();
        write_fixed(&mut out, value, decimals);
        String::from_utf8(out).unwrap()
    }

    #[test]
    fn fixed_point_formatting() {
        assert_eq!(fixed(1.5, 3), "1.500");
        assert_eq!(fixed(-0.0004, 3), "0.000");
        assert_eq!(fixed(-12.3456, 2), "-12.35");
        assert_eq!(fixed(0.005, 3), "0.005");
        assert_eq!(fixed(5_432_100.25, 0), "5432100");
        assert_eq!(fixed(123.0, 9), "123.000000000");
    }

    #[test]
    fn lines_parse_with_any_separator() {
        let format = LineFormat {
            columns: vec![Column::X, Column::Y, Column::Z, Column::Red, Column::Green, Column::Blue],
            ..LineFormat::default()
        };
        let parsed = parse_block(b"1 2 3 255 0 0\n4,5,6,0,255,0\r\n# comment\n7\t8\t9\t0\t0\t255\nbad line\n", &format);
        let points = &parsed.segments[0].points;
        assert_eq!(points.positions, [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]]);
        assert_eq!(points.color.as_ref().unwrap()[1], [0, 65535, 0]);
        assert_eq!(parsed.invalid_lines, 1);
    }

    #[test]
    fn decimal_commas_with_semicolons() {
        let format = LineFormat { decimal_comma: true, ..LineFormat::default() };
        let parsed = parse_block(b"1,5;2,25;-3\n", &format);
        assert_eq!(parsed.segments[0].points.positions, [[1.5, 2.25, -3.0]]);
    }

    #[test]
    fn single_numbers_start_new_scans() {
        let format = LineFormat { scan_breaks: true, ..LineFormat::default() };
        let parsed = parse_block(b"2\n1 1 1\n2 2 2\n1\n3 3 3\n", &format);
        let sizes: Vec<(bool, usize)> = parsed.segments.iter().map(|s| (s.starts_scan, s.points.len())).collect();
        assert_eq!(sizes, [(false, 0), (true, 2), (true, 1)]);
    }

    #[test]
    fn column_lists() {
        assert_eq!(
            parse_column_list("x,y,z,i,_,r,g,b").unwrap(),
            [
                Column::X,
                Column::Y,
                Column::Z,
                Column::Intensity,
                Column::Skip,
                Column::Red,
                Column::Green,
                Column::Blue
            ]
        );
        assert!(parse_column_list("x,y,i").is_err());
        assert!(parse_column_list("x,y,z,foo").is_err());
    }
}
