//! The format-independent point representation. Readers produce it, writers consume it.
//!
//! Conventions, so no format needs to know about any other:
//! - positions are metres, in the cloud's global frame (scan poses already applied),
//! - intensity is 0..1,
//! - colour is 16 bit per channel (0..65535).

use std::iter::Peekable;

use serde::{Deserialize, Serialize};

/// Number of points a reader should put into one chunk. Large enough to amortise per-chunk
/// overhead, small enough to keep memory flat.
pub const CHUNK_POINTS: usize = 1 << 20;

/// Number of points one parallel task works on. Large enough to amortise scheduling, small
/// enough to spread a chunk over all cores.
pub const TASK_POINTS: usize = 1 << 15;

/// The optional per-point attributes a cloud carries besides its position.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Attributes {
    pub intensity: bool,
    pub color: bool,
    pub normal: bool,
    pub classification: bool,
    pub time: bool,
    /// Return number and number of returns of the laser pulse, see [`Returns`].
    pub returns: bool,
    /// Scan angle, point source, user data and the LAS point flags, see [`LidarDetails`].
    pub lidar: bool,
    /// Bytes of extra values per point, described by [`CloudInfo::extra_fields`]; 0 for none.
    ///
    /// Where attributes only say what is wanted or possible (the attributes to leave out, what a
    /// format can store), any value above 0 stands for "extra values".
    #[serde(rename = "extra_values", with = "present_flag")]
    pub extra_bytes: u16,
}

impl Attributes {
    pub const ALL: Attributes = Attributes {
        intensity: true,
        color: true,
        normal: true,
        classification: true,
        time: true,
        returns: true,
        lidar: true,
        extra_bytes: 1,
    };

    pub const NONE: Attributes = Attributes {
        intensity: false,
        color: false,
        normal: false,
        classification: false,
        time: false,
        returns: false,
        lidar: false,
        extra_bytes: 0,
    };

    pub fn intersection(self, other: Attributes) -> Attributes {
        self.combine(other, |a, b| a && b)
    }

    pub fn union(self, other: Attributes) -> Attributes {
        self.combine(other, |a, b| a || b)
    }

    pub fn without(self, removed: Attributes) -> Attributes {
        self.combine(removed, |a, b| a && !b)
    }

    pub fn is_empty(self) -> bool {
        self == Attributes::NONE
    }

    pub fn has_extra(self) -> bool {
        self.extra_bytes > 0
    }

    /// Human readable names of the attributes that are set, e.g. `["intensity", "colour"]`.
    pub fn names(self) -> Vec<&'static str> {
        [
            (self.intensity, "intensity"),
            (self.color, "colour"),
            (self.normal, "normals"),
            (self.classification, "classification"),
            (self.time, "time"),
            (self.returns, "return numbers"),
            (self.lidar, "lidar details"),
            (self.has_extra(), "extra values"),
        ]
        .into_iter()
        .filter_map(|(set, name)| set.then_some(name))
        .collect()
    }

    fn combine(self, other: Attributes, op: impl Fn(bool, bool) -> bool) -> Attributes {
        // A width only comes from a cloud; the other side is at most a flag, so the larger wins.
        let extra = op(self.has_extra(), other.has_extra());
        Attributes {
            intensity: op(self.intensity, other.intensity),
            color: op(self.color, other.color),
            normal: op(self.normal, other.normal),
            classification: op(self.classification, other.classification),
            time: op(self.time, other.time),
            returns: op(self.returns, other.returns),
            lidar: op(self.lidar, other.lidar),
            extra_bytes: if extra { self.extra_bytes.max(other.extra_bytes) } else { 0 },
        }
    }
}

/// Stores a width of extra values as "present or not", which is all a settings file needs.
mod present_flag {
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<S: Serializer>(width: &u16, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_bool(*width > 0)
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<u16, D::Error> {
        bool::deserialize(deserializer).map(u16::from)
    }
}

/// Which return of its laser pulse a point is, and how many returns the pulse had. LAS stores
/// 0 to 15 for each; 0 means unknown.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Returns {
    pub number: u8,
    pub count: u8,
}

/// The remaining per-point fields of airborne and mobile laser scans, as LAS stores them.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LidarDetails {
    /// Degrees from nadir, negative to the left of the flight direction.
    pub scan_angle: f32,
    /// The flight line or file the point came from.
    pub point_source: u16,
    /// Free for the data producer to use.
    pub user_data: u8,
    /// The bits of a LAS 1.4 point: synthetic, key point, withheld, overlap, two bits of
    /// scanner channel, scan direction and edge of flight line. See the constants.
    pub flags: u8,
}

impl LidarDetails {
    pub const SYNTHETIC: u8 = 1;
    pub const KEY_POINT: u8 = 1 << 1;
    pub const WITHHELD: u8 = 1 << 2;
    pub const OVERLAP: u8 = 1 << 3;
    /// The four classification flags above.
    pub const CLASS_FLAGS: u8 = 0x0f;
    pub const CHANNEL_SHIFT: u32 = 4;
    pub const CHANNEL: u8 = 0b11 << Self::CHANNEL_SHIFT;
    /// Set when the scanner mirror moved from left to right.
    pub const SCAN_DIRECTION: u8 = 1 << 6;
    pub const EDGE_OF_FLIGHT_LINE: u8 = 1 << 7;

    pub fn scanner_channel(&self) -> u8 {
        (self.flags & Self::CHANNEL) >> Self::CHANNEL_SHIFT
    }
}

/// The number type of an extra value. These are the types of LAS extra bytes, in the order of
/// their LAS type codes 1 to 10.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScalarType {
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    U64,
    I64,
    F32,
    F64,
}

impl ScalarType {
    pub const ALL: [ScalarType; 10] = [
        ScalarType::U8,
        ScalarType::I8,
        ScalarType::U16,
        ScalarType::I16,
        ScalarType::U32,
        ScalarType::I32,
        ScalarType::U64,
        ScalarType::I64,
        ScalarType::F32,
        ScalarType::F64,
    ];

    pub fn size(self) -> usize {
        match self {
            ScalarType::U8 | ScalarType::I8 => 1,
            ScalarType::U16 | ScalarType::I16 => 2,
            ScalarType::U32 | ScalarType::I32 | ScalarType::F32 => 4,
            ScalarType::U64 | ScalarType::I64 | ScalarType::F64 => 8,
        }
    }

    /// Reads a little-endian value from the start of `bytes`.
    pub fn decode(self, bytes: &[u8]) -> f64 {
        macro_rules! number {
            ($type:ty) => {
                <$type>::from_le_bytes(bytes[..size_of::<$type>()].try_into().expect("slice has the type's size"))
                    as f64
            };
        }
        match self {
            ScalarType::U8 => number!(u8),
            ScalarType::I8 => number!(i8),
            ScalarType::U16 => number!(u16),
            ScalarType::I16 => number!(i16),
            ScalarType::U32 => number!(u32),
            ScalarType::I32 => number!(i32),
            ScalarType::U64 => number!(u64),
            ScalarType::I64 => number!(i64),
            ScalarType::F32 => number!(f32),
            ScalarType::F64 => number!(f64),
        }
    }
}

/// A named value each point carries besides the standard attributes, such as a LAS extra bytes
/// field or a custom PLY vertex property. The values of all fields of a point lie one after
/// another in [`PointChunk::extra`], in the order of [`CloudInfo::extra_fields`].
#[derive(Clone, Debug, PartialEq)]
pub struct ExtraField {
    pub name: String,
    pub description: String,
    /// Bytes per point.
    pub size: u16,
    /// The little-endian number the bytes hold, or `None` for bytes that are not one number.
    pub scalar: Option<ScalarType>,
    /// A stored number `n` stands for the value `n * scale + offset`.
    pub scale: f64,
    pub offset: f64,
    /// The LAS extra bytes descriptor the field was read from, so LAS output repeats it as is.
    pub las_descriptor: Option<Vec<u8>>,
}

impl ExtraField {
    /// A field holding one unscaled number per point.
    pub fn number(name: impl Into<String>, scalar: ScalarType) -> ExtraField {
        ExtraField {
            name: name.into(),
            description: String::new(),
            size: scalar.size() as u16,
            scalar: Some(scalar),
            scale: 1.0,
            offset: 0.0,
            las_descriptor: None,
        }
    }

    pub fn is_scaled(&self) -> bool {
        self.scale != 1.0 || self.offset != 0.0
    }

    /// The value the field's bytes of one point stand for, if they hold a number.
    pub fn value(&self, bytes: &[u8]) -> Option<f64> {
        self.scalar.map(|scalar| scalar.decode(bytes) * self.scale + self.offset)
    }

    /// Bytes per point of all `fields` together.
    pub fn total_size(fields: &[ExtraField]) -> u16 {
        fields.iter().map(|field| field.size).sum()
    }
}

/// The extra values of a chunk's points: `width` bytes per point.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ExtraBytes {
    pub width: usize,
    pub bytes: Vec<u8>,
}

impl ExtraBytes {
    pub fn with_capacity(width: usize, points: usize) -> ExtraBytes {
        ExtraBytes { width, bytes: Vec::with_capacity(width * points) }
    }

    /// The bytes of point `index`.
    pub fn get(&self, index: usize) -> &[u8] {
        &self.bytes[index * self.width..(index + 1) * self.width]
    }

    /// The bytes of point `index`, to fill in.
    pub fn get_mut(&mut self, index: usize) -> &mut [u8] {
        &mut self.bytes[index * self.width..(index + 1) * self.width]
    }

    fn retain(&mut self, keep: &[bool]) {
        let width = self.width;
        let mut kept = 0;
        for (index, _) in keep.iter().enumerate().filter(|(_, keep)| **keep) {
            self.bytes.copy_within(index * width..(index + 1) * width, kept * width);
            kept += 1;
        }
        self.bytes.truncate(kept * width);
    }

    fn take(&mut self, selected: &[bool], count: usize) -> ExtraBytes {
        let mut taken = ExtraBytes::with_capacity(self.width, count);
        for (index, _) in selected.iter().enumerate().filter(|(_, take)| **take) {
            taken.bytes.extend_from_slice(self.get(index));
        }
        let keep: Vec<bool> = selected.iter().map(|take| !take).collect();
        self.retain(&keep);
        taken
    }
}

/// One point with every attribute, used where code handles points one at a time.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct PointRecord {
    pub position: [f64; 3],
    pub intensity: f32,
    pub color: [u16; 3],
    pub normal: [f32; 3],
    pub classification: u8,
    pub time: f64,
    pub returns: Returns,
    pub lidar: LidarDetails,
    /// Row and column in the scan's grid; see [`PointChunk::grid`].
    pub grid: [i32; 2],
}

/// A batch of points from one scan, stored column by column.
///
/// An attribute column is `Some` exactly when the cloud carries that attribute, and then has one
/// value per position.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PointChunk {
    pub scan: usize,
    pub positions: Vec<[f64; 3]>,
    pub intensity: Option<Vec<f32>>,
    pub color: Option<Vec<[u16; 3]>>,
    pub normal: Option<Vec<[f32; 3]>>,
    pub classification: Option<Vec<u8>>,
    pub time: Option<Vec<f64>>,
    pub returns: Option<Vec<Returns>>,
    pub lidar: Option<Vec<LidarDetails>>,
    pub extra: Option<ExtraBytes>,
    /// Row and column of each point, for scans stored as a grid (structured E57 scans).
    ///
    /// Not one of the [`Attributes`]: it belongs to the scan (see [`ScanInfo::grid`]), is
    /// `Some` exactly for chunks of such scans, and formats without grids ignore it.
    pub grid: Option<Vec<[i32; 2]>>,
}

impl PointChunk {
    pub fn new(scan: usize, attributes: Attributes) -> Self {
        Self::with_capacity(scan, attributes, 0)
    }

    pub fn with_capacity(scan: usize, attributes: Attributes, capacity: usize) -> Self {
        PointChunk {
            scan,
            positions: Vec::with_capacity(capacity),
            intensity: new_column(attributes.intensity, capacity),
            color: new_column(attributes.color, capacity),
            normal: new_column(attributes.normal, capacity),
            classification: new_column(attributes.classification, capacity),
            time: new_column(attributes.time, capacity),
            returns: new_column(attributes.returns, capacity),
            lidar: new_column(attributes.lidar, capacity),
            extra: attributes
                .has_extra()
                .then(|| ExtraBytes::with_capacity(usize::from(attributes.extra_bytes), capacity)),
            grid: None,
        }
    }

    /// Adds the row and column column, for points of a gridded scan.
    pub fn with_grid(mut self) -> Self {
        self.grid = Some(Vec::with_capacity(self.positions.capacity()));
        self
    }

    pub fn len(&self) -> usize {
        self.positions.len()
    }

    pub fn is_empty(&self) -> bool {
        self.positions.is_empty()
    }

    pub fn attributes(&self) -> Attributes {
        Attributes {
            intensity: self.intensity.is_some(),
            color: self.color.is_some(),
            normal: self.normal.is_some(),
            classification: self.classification.is_some(),
            time: self.time.is_some(),
            returns: self.returns.is_some(),
            lidar: self.lidar.is_some(),
            extra_bytes: self.extra.as_ref().map_or(0, |extra| extra.width as u16),
        }
    }

    /// Appends a point. Its extra values, if the chunk has them, start out as zeros; see
    /// [`ExtraBytes::get_mut`].
    pub fn push(&mut self, point: &PointRecord) {
        self.positions.push(point.position);
        push_if_present(&mut self.intensity, point.intensity);
        push_if_present(&mut self.color, point.color);
        push_if_present(&mut self.normal, point.normal);
        push_if_present(&mut self.classification, point.classification);
        push_if_present(&mut self.time, point.time);
        push_if_present(&mut self.returns, point.returns);
        push_if_present(&mut self.lidar, point.lidar);
        if let Some(extra) = &mut self.extra {
            extra.bytes.resize(extra.bytes.len() + extra.width, 0);
        }
        push_if_present(&mut self.grid, point.grid);
    }

    /// Reads point `index` back as a record. Missing attributes come back as zero. Extra values
    /// are not part of a record; they are in [`PointChunk::extra`].
    pub fn record(&self, index: usize) -> PointRecord {
        PointRecord {
            position: self.positions[index],
            intensity: value_at(&self.intensity, index),
            color: value_at(&self.color, index),
            normal: value_at(&self.normal, index),
            classification: value_at(&self.classification, index),
            time: value_at(&self.time, index),
            returns: value_at(&self.returns, index),
            lidar: value_at(&self.lidar, index),
            grid: value_at(&self.grid, index),
        }
    }

    /// Appends all points of `other`, which must carry the same attributes.
    pub fn append(&mut self, mut other: PointChunk) {
        debug_assert_eq!(self.attributes(), other.attributes());
        self.positions.append(&mut other.positions);
        append_column(&mut self.intensity, other.intensity);
        append_column(&mut self.color, other.color);
        append_column(&mut self.normal, other.normal);
        append_column(&mut self.classification, other.classification);
        append_column(&mut self.time, other.time);
        append_column(&mut self.returns, other.returns);
        append_column(&mut self.lidar, other.lidar);
        if let (Some(extra), Some(mut more)) = (&mut self.extra, other.extra) {
            extra.bytes.append(&mut more.bytes);
        }
        append_column(&mut self.grid, other.grid);
    }

    /// Keeps the points whose entry in `keep` is true.
    pub fn retain(&mut self, keep: &[bool]) {
        debug_assert_eq!(keep.len(), self.len());
        retain_column(&mut self.positions, keep);
        if let Some(values) = &mut self.intensity {
            retain_column(values, keep);
        }
        if let Some(values) = &mut self.color {
            retain_column(values, keep);
        }
        if let Some(values) = &mut self.normal {
            retain_column(values, keep);
        }
        if let Some(values) = &mut self.classification {
            retain_column(values, keep);
        }
        if let Some(values) = &mut self.time {
            retain_column(values, keep);
        }
        if let Some(values) = &mut self.returns {
            retain_column(values, keep);
        }
        if let Some(values) = &mut self.lidar {
            retain_column(values, keep);
        }
        if let Some(extra) = &mut self.extra {
            extra.retain(keep);
        }
        if let Some(values) = &mut self.grid {
            retain_column(values, keep);
        }
    }

    /// Makes the chunk carry exactly `attributes`: extra columns are removed, missing ones are
    /// filled with zeros. The grid column is left as it is.
    pub fn conform_to(&mut self, attributes: Attributes) {
        let len = self.len();
        conform_column(&mut self.intensity, attributes.intensity, len);
        conform_column(&mut self.color, attributes.color, len);
        conform_column(&mut self.normal, attributes.normal, len);
        conform_column(&mut self.classification, attributes.classification, len);
        conform_column(&mut self.time, attributes.time, len);
        conform_column(&mut self.returns, attributes.returns, len);
        conform_column(&mut self.lidar, attributes.lidar, len);
        let width = usize::from(attributes.extra_bytes);
        match &self.extra {
            Some(extra) if extra.width == width => {}
            _ if width == 0 => self.extra = None,
            _ => self.extra = Some(ExtraBytes { width, bytes: vec![0; width * len] }),
        }
    }
}

fn new_column<T>(present: bool, capacity: usize) -> Option<Vec<T>> {
    present.then(|| Vec::with_capacity(capacity))
}

fn value_at<T: Copy + Default>(column: &Option<Vec<T>>, index: usize) -> T {
    column.as_ref().map_or_else(T::default, |values| values[index])
}

fn push_if_present<T>(column: &mut Option<Vec<T>>, value: T) {
    if let Some(values) = column {
        values.push(value);
    }
}

fn append_column<T>(column: &mut Option<Vec<T>>, other: Option<Vec<T>>) {
    if let (Some(values), Some(mut more)) = (column, other) {
        values.append(&mut more);
    }
}

fn retain_column<T>(values: &mut Vec<T>, keep: &[bool]) {
    retain_flagged(values, keep, |_, kept| kept);
}

/// Keeps the values for which `keep` returns true. `keep` is given each value and its entry in
/// `flags`, in order.
fn retain_flagged<T>(values: &mut Vec<T>, flags: &[bool], mut keep: impl FnMut(&T, bool) -> bool) {
    let mut index = 0;
    values.retain(|value| {
        let flag = flags[index];
        index += 1;
        keep(value, flag)
    });
}

fn conform_column<T: Clone + Default>(column: &mut Option<Vec<T>>, wanted: bool, len: usize) {
    match (wanted, column.is_some()) {
        (true, false) => *column = Some(vec![T::default(); len]),
        (false, true) => *column = None,
        _ => {}
    }
}

/// What is known about a cloud before its points are read.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CloudInfo {
    pub attributes: Attributes,
    /// The scans in the order their points arrive. Formats without scans report one.
    /// Some text formats only discover further scans while reading.
    pub scans: Vec<ScanInfo>,
    /// Number of points the file says it holds, if it says so.
    pub point_count: Option<u64>,
    /// Coordinate reference system as OGC WKT, passed through unchanged.
    pub crs_wkt: Option<String>,
    /// What the extra values of each point mean. Their sizes add up to
    /// [`Attributes::extra_bytes`].
    pub extra_fields: Vec<ExtraField>,
}

impl CloudInfo {
    /// Scan `index`, or a numbered placeholder for scans found while reading.
    pub fn scan(&self, index: usize) -> ScanInfo {
        self.scans.get(index).cloned().unwrap_or_else(|| ScanInfo::named(format!("Scan {}", index + 1)))
    }
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct ScanInfo {
    pub name: String,
    /// Identifier from formats that have one (E57), kept so references stay valid.
    pub guid: Option<String>,
    /// Scanner position and orientation, if known.
    pub pose: Option<Pose>,
    /// The grid of a structured scan, whose points then carry a row and column.
    pub grid: Option<ScanGrid>,
}

/// The row and column ranges of a structured scan, each as inclusive `[first, last]`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ScanGrid {
    pub rows: [i32; 2],
    pub columns: [i32; 2],
}

impl ScanGrid {
    pub fn contains(&self, [row, column]: [i32; 2]) -> bool {
        (self.rows[0]..=self.rows[1]).contains(&row) && (self.columns[0]..=self.columns[1]).contains(&column)
    }
}

impl ScanInfo {
    pub fn named(name: impl Into<String>) -> Self {
        ScanInfo { name: name.into(), ..ScanInfo::default() }
    }
}

/// A rigid transform from scanner coordinates to global coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose {
    /// Unit quaternion `[w, x, y, z]`.
    pub rotation: [f64; 4],
    pub translation: [f64; 3],
}

impl Pose {
    /// Converts a global position back into scanner coordinates.
    pub fn to_local(&self, global: [f64; 3]) -> [f64; 3] {
        let [w, x, y, z] = self.rotation;
        let shifted =
            [global[0] - self.translation[0], global[1] - self.translation[1], global[2] - self.translation[2]];
        rotate([w, -x, -y, -z], shifted)
    }

    /// Rotates a direction (a normal) from scanner coordinates into global coordinates.
    pub fn direction_to_global(&self, local: [f64; 3]) -> [f64; 3] {
        rotate(self.rotation, local)
    }

    /// Rotates a direction (a normal) from global coordinates into scanner coordinates.
    pub fn direction_to_local(&self, global: [f64; 3]) -> [f64; 3] {
        let [w, x, y, z] = self.rotation;
        rotate([w, -x, -y, -z], global)
    }
}

/// Rotates `v` by the unit quaternion `q = [w, x, y, z]`.
fn rotate(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let [w, x, y, z] = q;
    // t = 2 * cross(q.xyz, v); v' = v + w * t + cross(q.xyz, t)
    let t = [2.0 * (y * v[2] - z * v[1]), 2.0 * (z * v[0] - x * v[2]), 2.0 * (x * v[1] - y * v[0])];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

/// Splits a chunk stream into runs of consecutive chunks from the same scan.
///
/// Readers deliver scans one after another, so this is enough to write one scan at a time.
pub struct ScanGroups<I: Iterator<Item = PointChunk>> {
    chunks: Peekable<I>,
}

impl<I: Iterator<Item = PointChunk>> ScanGroups<I> {
    pub fn new(chunks: I) -> Self {
        ScanGroups { chunks: chunks.peekable() }
    }

    /// The next scan index and an iterator over its chunks.
    pub fn next_group(&mut self) -> Option<(usize, ScanGroup<'_, I>)> {
        let scan = self.chunks.peek()?.scan;
        Some((scan, ScanGroup { scan, groups: self }))
    }
}

pub struct ScanGroup<'a, I: Iterator<Item = PointChunk>> {
    scan: usize,
    groups: &'a mut ScanGroups<I>,
}

impl<I: Iterator<Item = PointChunk>> Iterator for ScanGroup<'_, I> {
    type Item = PointChunk;

    fn next(&mut self) -> Option<PointChunk> {
        let scan = self.scan;
        self.groups.chunks.next_if(|chunk| chunk.scan == scan)
    }
}

/// The range a file's raw values span, used to map them onto 0..1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ValueRange {
    pub min: f64,
    pub max: f64,
}

impl ValueRange {
    pub const UNIT: ValueRange = ValueRange { min: 0.0, max: 1.0 };
    pub const BYTE: ValueRange = ValueRange { min: 0.0, max: 255.0 };
    pub const WORD: ValueRange = ValueRange { min: 0.0, max: 65535.0 };
    /// The intensity range of Leica PTS files.
    pub const PTS: ValueRange = ValueRange { min: -2048.0, max: 2047.0 };

    /// Guesses the range of intensity values from a sample of them. Formats without a declared
    /// range (text, float PLY) need this; the common conventions are easy to tell apart.
    pub fn guess_intensity(samples: impl IntoIterator<Item = f64>) -> ValueRange {
        let (min, max) =
            samples.into_iter().fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), v| (lo.min(v), hi.max(v)));
        if min > max {
            ValueRange::UNIT
        } else if min < 0.0 {
            ValueRange::PTS
        } else if max <= 1.0 {
            ValueRange::UNIT
        } else if max <= 255.0 {
            ValueRange::BYTE
        } else if max <= 4095.0 {
            ValueRange { min: 0.0, max: 4095.0 }
        } else {
            ValueRange::WORD
        }
    }

    pub fn normalize(&self, value: f64) -> f32 {
        ((value - self.min) / (self.max - self.min)).clamp(0.0, 1.0) as f32
    }

    pub fn denormalize(&self, value: f32) -> f64 {
        self.min + f64::from(value.clamp(0.0, 1.0)) * (self.max - self.min)
    }
}

pub fn color_from_u8(value: u8) -> u16 {
    u16::from(value) * 257
}

pub fn color_to_u8(value: u16) -> u8 {
    ((u32::from(value) * 255 + 32767) / 65535) as u8
}

pub fn color_from_unit(value: f32) -> u16 {
    (value.clamp(0.0, 1.0) * 65535.0).round() as u16
}

pub fn color_to_unit(value: u16) -> f32 {
    f32::from(value) / 65535.0
}

/// Moving points between chunks, for building octree nodes.
impl PointChunk {
    /// Moves the points whose entry in `selected` is true into a new chunk. The moved points
    /// and the ones left behind both keep their order.
    pub fn take(&mut self, selected: &[bool]) -> PointChunk {
        debug_assert_eq!(selected.len(), self.len());
        let count = selected.iter().filter(|&&taken| taken).count();
        PointChunk {
            scan: self.scan,
            positions: take_column(&mut self.positions, selected, count),
            intensity: self.intensity.as_mut().map(|values| take_column(values, selected, count)),
            color: self.color.as_mut().map(|values| take_column(values, selected, count)),
            normal: self.normal.as_mut().map(|values| take_column(values, selected, count)),
            classification: self.classification.as_mut().map(|values| take_column(values, selected, count)),
            time: self.time.as_mut().map(|values| take_column(values, selected, count)),
            returns: self.returns.as_mut().map(|values| take_column(values, selected, count)),
            lidar: self.lidar.as_mut().map(|values| take_column(values, selected, count)),
            extra: self.extra.as_mut().map(|extra| extra.take(selected, count)),
            grid: self.grid.as_mut().map(|values| take_column(values, selected, count)),
        }
    }

    /// Distributes the points into `group_count` chunks by their entry in `group_of`. Within
    /// each group the points keep their order.
    pub fn split_by(self, group_of: &[u8], group_count: usize) -> Vec<PointChunk> {
        debug_assert_eq!(group_of.len(), self.len());
        let mut sizes = vec![0; group_count];
        for &group in group_of {
            sizes[usize::from(group)] += 1;
        }
        let (scan, attributes, gridded) = (self.scan, self.attributes(), self.grid.is_some());
        let mut groups: Vec<PointChunk> = sizes
            .into_iter()
            .map(|size| {
                let group = PointChunk::with_capacity(scan, attributes, size);
                if gridded { group.with_grid() } else { group }
            })
            .collect();
        split_column(Some(self.positions), group_of, &mut groups, |group| Some(&mut group.positions));
        split_column(self.intensity, group_of, &mut groups, |group| group.intensity.as_mut());
        split_column(self.color, group_of, &mut groups, |group| group.color.as_mut());
        split_column(self.normal, group_of, &mut groups, |group| group.normal.as_mut());
        split_column(self.classification, group_of, &mut groups, |group| group.classification.as_mut());
        split_column(self.time, group_of, &mut groups, |group| group.time.as_mut());
        split_column(self.returns, group_of, &mut groups, |group| group.returns.as_mut());
        split_column(self.lidar, group_of, &mut groups, |group| group.lidar.as_mut());
        if let Some(extra) = self.extra {
            for (values, &group) in extra.bytes.chunks_exact(extra.width).zip(group_of) {
                if let Some(target) = &mut groups[usize::from(group)].extra {
                    target.bytes.extend_from_slice(values);
                }
            }
        }
        split_column(self.grid, group_of, &mut groups, |group| group.grid.as_mut());
        groups
    }
}

fn take_column<T: Copy>(values: &mut Vec<T>, selected: &[bool], count: usize) -> Vec<T> {
    let mut taken = Vec::with_capacity(count);
    retain_flagged(values, selected, |&value, take| {
        if take {
            taken.push(value);
        }
        !take
    });
    taken
}

/// Moves each value of a column into the same column of its group, which `column` picks out.
/// Does nothing for a missing column.
fn split_column<T>(
    values: Option<Vec<T>>,
    group_of: &[u8],
    groups: &mut [PointChunk],
    column: impl Fn(&mut PointChunk) -> Option<&mut Vec<T>>,
) {
    for (value, &group) in values.into_iter().flatten().zip(group_of) {
        if let Some(values) = column(&mut groups[usize::from(group)]) {
            values.push(value);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_round_trip() {
        let half = std::f64::consts::FRAC_PI_4; // 90 degrees about z
        let pose = Pose { rotation: [half.cos(), 0.0, 0.0, half.sin()], translation: [10.0, 20.0, 30.0] };
        // Local (1, 0, 0) rotated 90 degrees about z is (0, 1, 0), then translated.
        let local = pose.to_local([10.0, 21.0, 30.0]);
        for (got, want) in local.iter().zip([1.0, 0.0, 0.0]) {
            assert!((got - want).abs() < 1e-12, "{local:?}");
        }
    }

    #[test]
    fn colour_conversions_are_inverse() {
        for value in 0..=255u8 {
            assert_eq!(color_to_u8(color_from_u8(value)), value);
        }
    }

    #[test]
    fn scan_groups_split_on_scan_change() {
        let chunk = |scan| PointChunk::new(scan, Attributes::NONE);
        let mut groups = ScanGroups::new(vec![chunk(0), chunk(0), chunk(1), chunk(2), chunk(2)].into_iter());
        let mut sizes = Vec::new();
        while let Some((scan, group)) = groups.next_group() {
            sizes.push((scan, group.count()));
        }
        assert_eq!(sizes, [(0, 2), (1, 1), (2, 2)]);
    }

    #[test]
    fn extra_widths_survive_attribute_arithmetic() {
        let cloud = Attributes { returns: true, extra_bytes: 12, ..Attributes::NONE };
        let flag = Attributes { extra_bytes: 1, ..Attributes::NONE };
        assert_eq!(cloud.intersection(Attributes::ALL).extra_bytes, 12);
        assert_eq!(Attributes::ALL.intersection(cloud).extra_bytes, 12);
        assert_eq!(cloud.without(Attributes::NONE).extra_bytes, 12);
        assert_eq!(cloud.without(flag), Attributes { returns: true, ..Attributes::NONE });
        assert_eq!(cloud.names(), ["return numbers", "extra values"]);
    }

    /// A chunk whose points carry their index in the return number, the lidar user data and
    /// both extra bytes.
    fn numbered(count: u8) -> PointChunk {
        let attributes = Attributes { returns: true, lidar: true, extra_bytes: 2, ..Attributes::NONE };
        let mut chunk = PointChunk::new(1, attributes);
        for index in 0..count {
            chunk.push(&PointRecord {
                returns: Returns { number: index, count },
                lidar: LidarDetails { user_data: index, ..LidarDetails::default() },
                ..PointRecord::default()
            });
            chunk.extra.as_mut().unwrap().get_mut(usize::from(index)).copy_from_slice(&[index, !index]);
        }
        chunk
    }

    fn assert_numbered(chunk: &PointChunk, want: &[u8]) {
        assert_eq!(chunk.len(), want.len());
        for (index, &number) in want.iter().enumerate() {
            let point = chunk.record(index);
            assert_eq!((point.returns.number, point.lidar.user_data), (number, number));
            assert_eq!(chunk.extra.as_ref().unwrap().get(index), [number, !number]);
        }
    }

    #[test]
    fn point_fields_and_extra_values_move_with_their_points() {
        let mut chunk = numbered(6);
        assert_eq!(chunk.attributes().extra_bytes, 2);
        chunk.retain(&[true, false, true, true, false, true]);
        assert_numbered(&chunk, &[0, 2, 3, 5]);
        let taken = chunk.take(&[false, true, false, true]);
        assert_numbered(&taken, &[2, 5]);
        assert_numbered(&chunk, &[0, 3]);
        chunk.append(taken);
        let groups = chunk.split_by(&[1, 0, 1, 0], 2);
        assert_numbered(&groups[0], &[3, 5]);
        assert_numbered(&groups[1], &[0, 2]);

        let mut conformed = groups[0].clone();
        conformed.conform_to(Attributes { extra_bytes: 3, ..Attributes::NONE });
        assert_eq!(conformed.extra, Some(ExtraBytes { width: 3, bytes: vec![0; 6] }));
        assert_eq!((conformed.returns.is_none(), conformed.lidar.is_none()), (true, true));
    }

    #[test]
    fn extra_values_are_read_as_numbers() {
        let mut field = ExtraField::number("range", ScalarType::I16);
        assert_eq!(field.value(&(-300i16).to_le_bytes()), Some(-300.0));
        field.scale = 0.01;
        field.offset = 5.0;
        assert!(field.is_scaled());
        assert_eq!(field.value(&(-300i16).to_le_bytes()), Some(2.0));
        assert_eq!(ScalarType::F64.decode(&1.5f64.to_le_bytes()), 1.5);
        assert_eq!(ScalarType::ALL.iter().map(|scalar| scalar.size()).sum::<usize>(), 2 * (1 + 2 + 4 + 8) + 4 + 8);
    }

    #[test]
    fn intensity_ranges_are_recognised() {
        assert_eq!(ValueRange::guess_intensity([-100.0, 2000.0]), ValueRange::PTS);
        assert_eq!(ValueRange::guess_intensity([0.1, 0.9]), ValueRange::UNIT);
        assert_eq!(ValueRange::guess_intensity([3.0, 200.0]), ValueRange::BYTE);
        assert_eq!(ValueRange::guess_intensity([3.0, 60000.0]), ValueRange::WORD);
    }

    #[test]
    fn grid_positions_follow_their_points() {
        let mut chunk = PointChunk::with_capacity(0, Attributes::NONE, 6).with_grid();
        for i in 0..6 {
            chunk.push(&PointRecord {
                position: [f64::from(i), 0.0, 0.0],
                grid: [i, 10 + i],
                ..PointRecord::default()
            });
        }
        let grid_matches = |chunk: &PointChunk| {
            (0..chunk.len()).all(|i| {
                let point = chunk.record(i);
                point.grid == [point.position[0] as i32, 10 + point.position[0] as i32]
            })
        };

        chunk.retain(&[true, true, false, true, true, true]);
        let mut taken = chunk.take(&[false, true, false, true, false]);
        assert!(grid_matches(&chunk) && grid_matches(&taken));
        let groups = chunk.split_by(&[1, 0, 1], 2);
        assert!(groups.iter().all(|group| group.grid.is_some() && grid_matches(group)));
        taken.append(groups.into_iter().next().unwrap());
        assert_eq!(taken.grid.as_ref().map(Vec::len), Some(taken.len()));
        assert!(grid_matches(&taken));
    }
}
