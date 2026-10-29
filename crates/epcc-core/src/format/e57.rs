//! E57, through the `e57` crate.
//!
//! Each E57 point cloud becomes one scan. Points are read in global coordinates (the pose is
//! applied) and written back relative to their scan's pose, so scanner positions survive.
//!
//! Normals use the published surface normals extension (`nor:normalX` etc.), which the `e57`
//! crate handles as extension point fields. Structured scans keep their `rowIndex` and
//! `columnIndex` and the grid size. Points without a valid position are not read, so a written
//! grid simply has no record for those cells.

use std::io::Read;
use std::path::{Path, PathBuf};

use e57::{
    CartesianCoordinate, ColorLimits, E57Reader, E57Writer, Extension, Image, IndexBounds, IntensityLimits, Projection,
    Quaternion, Record, RecordDataType, RecordName, RecordValue, Transform, Translation,
};

use super::{PointReader, PointWriter, WriteRequest};
use crate::model::{
    Attributes, CHUNK_POINTS, CloudInfo, PointChunk, PointRecord, Pose, ScanGrid, ScanGroups, ScanInfo,
    color_from_unit, color_to_u8,
};
use crate::{Error, Format, Progress, Result};

type FileReader = E57Reader<std::io::BufReader<std::fs::File>>;

/// The surface normals extension: its usual namespace prefix and its URI.
const NORMALS_PREFIX: &str = "nor";
const NORMALS_URI: &str = "http://www.libe57.org/E57_EXT_surface_normals.txt";
const NORMAL_FIELDS: [&str; 3] = ["normalX", "normalY", "normalZ"];

pub(super) fn open(path: &Path) -> Result<Box<dyn PointReader>> {
    let reader = E57Reader::from_file(path)?;
    let clouds = reader.pointclouds();
    let prefix = normals_prefix(&reader.extensions());
    let normal_fields: Vec<_> = clouds.iter().map(|cloud| normal_fields(cloud, &prefix)).collect();
    let attributes = clouds.iter().zip(&normal_fields).fold(Attributes::NONE, |found, (cloud, normals)| {
        found.union(Attributes {
            intensity: cloud.has_intensity(),
            color: cloud.has_color(),
            normal: normals.is_some(),
            ..Attributes::NONE
        })
    });
    let scans = clouds
        .iter()
        .enumerate()
        .map(|(index, cloud)| ScanInfo {
            name: cloud.name.clone().unwrap_or_else(|| format!("Scan {}", index + 1)),
            guid: cloud.guid.clone(),
            pose: cloud.transform.as_ref().map(pose_from),
            grid: grid_of(cloud),
        })
        .collect();
    let info = CloudInfo {
        attributes,
        scans,
        point_count: Some(clouds.iter().map(|cloud| cloud.records).sum()),
        crs_wkt: reader.coordinate_metadata().map(str::to_string),
        extra_fields: Vec::new(),
    };
    Ok(Box::new(E57PointReader { path: path.to_path_buf(), reader, clouds, normal_fields, info }))
}

/// The namespace prefix the file uses for the surface normals extension.
fn normals_prefix(extensions: &[Extension]) -> String {
    extensions
        .iter()
        .find(|extension| extension.url == NORMALS_URI)
        .map_or_else(|| NORMALS_PREFIX.to_string(), |extension| extension.namespace.clone())
}

/// Where a cloud's normal components are in its records, if it has normals.
fn normal_fields(cloud: &e57::PointCloud, prefix: &str) -> Option<[usize; 3]> {
    let find = |field: &str| {
        cloud.prototype.iter().position(|record| {
            matches!(&record.name, RecordName::Unknown { namespace, name }
                if name == field && (namespace == prefix || namespace.is_empty()))
        })
    };
    Some([find(NORMAL_FIELDS[0])?, find(NORMAL_FIELDS[1])?, find(NORMAL_FIELDS[2])?])
}

/// The grid of a structured scan: its index bounds, or else the range its records allow.
fn grid_of(cloud: &e57::PointCloud) -> Option<ScanGrid> {
    let bounds = cloud.index_bounds.clone().unwrap_or_default();
    let range = |name: RecordName, min: Option<i64>, max: Option<i64>| -> Option<[i32; 2]> {
        let record = cloud.prototype.iter().find(|record| record.name == name)?;
        let (min, max) = match (min, max, &record.data_type) {
            (Some(min), Some(max), _) => (min, max),
            (_, _, RecordDataType::Integer { min, max }) => (*min, *max),
            _ => return None,
        };
        if min > max {
            return None;
        }
        Some([i32::try_from(min).ok()?, i32::try_from(max).ok()?])
    };
    Some(ScanGrid {
        rows: range(RecordName::RowIndex, bounds.row_min, bounds.row_max)?,
        columns: range(RecordName::ColumnIndex, bounds.column_min, bounds.column_max)?,
    })
}

struct E57PointReader {
    path: PathBuf,
    reader: FileReader,
    clouds: Vec<e57::PointCloud>,
    normal_fields: Vec<Option<[usize; 3]>>,
    info: CloudInfo,
}

impl PointReader for E57PointReader {
    fn info(&self) -> &CloudInfo {
        &self.info
    }

    fn read(mut self: Box<Self>, emit: &mut dyn FnMut(PointChunk) -> Result<()>, progress: &Progress) -> Result<()> {
        let total = self.info.point_count.unwrap_or(0);
        let attributes = self.info.attributes;
        // The crate's simple reader does not return extension fields, so normals come from a
        // second, raw reader that walks the same records in step.
        let mut normals_file: Option<FileReader> = None;
        let mut done = 0;
        for (scan, cloud) in self.clouds.iter().enumerate() {
            let mut points = self.reader.pointcloud_simple(cloud)?;
            points.apply_pose(true);
            points.spherical_to_cartesian(true);
            points.intensity_to_color(false);

            let fields = self.normal_fields[scan];
            if fields.is_some() && normals_file.is_none() {
                normals_file = Some(E57Reader::from_file(&self.path)?);
            }
            let mut normals = match (fields, normals_file.as_mut()) {
                (Some(fields), Some(file)) => Some((file.pointcloud_raw(cloud)?, fields)),
                _ => None,
            };
            let pose = cloud.transform.as_ref().map(pose_from);
            let gridded = self.info.scans[scan].grid.is_some();
            let new_chunk = || {
                let chunk = PointChunk::with_capacity(scan, attributes, CHUNK_POINTS);
                if gridded { chunk.with_grid() } else { chunk }
            };

            let mut chunk = new_chunk();
            for point in points {
                let point = point?;
                done += 1;
                let normal = match &mut normals {
                    Some((raw, fields)) => {
                        let values =
                            raw.next().ok_or_else(|| Error::invalid("E57 normals end before the points"))??;
                        let normal = read_normal(&values, *fields, &cloud.prototype)?;
                        pose.map_or(normal, |pose| pose.direction_to_global(normal)).map(|v| v as f32)
                    }
                    None => [0.0; 3],
                };
                // Points without a valid position (no laser return) carry no geometry.
                let CartesianCoordinate::Valid { x, y, z } = point.cartesian else {
                    continue;
                };
                let index = |value: i64| value.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32;
                chunk.push(&PointRecord {
                    position: [x, y, z],
                    intensity: point.intensity.unwrap_or(0.0),
                    color: point.color.map(|c| [c.red, c.green, c.blue].map(color_from_unit)).unwrap_or_default(),
                    normal,
                    grid: [index(point.row), index(point.column)],
                    ..PointRecord::default()
                });
                if chunk.len() == CHUNK_POINTS {
                    emit(std::mem::replace(&mut chunk, new_chunk()))?;
                    progress.set_done_of(done, total);
                }
            }
            if !chunk.is_empty() {
                emit(chunk)?;
            }
            progress.set_done_of(done, total);
        }
        Ok(())
    }
}

fn read_normal(values: &[RecordValue], fields: [usize; 3], prototype: &[Record]) -> Result<[f64; 3]> {
    let mut normal = [0.0; 3];
    for (component, field) in normal.iter_mut().zip(fields) {
        *component = values[field].to_f64(&prototype[field].data_type)?;
    }
    Ok(normal)
}

fn pose_from(transform: &Transform) -> Pose {
    let Quaternion { w, x, y, z } = transform.rotation;
    let Translation { x: tx, y: ty, z: tz } = transform.translation;
    Pose { rotation: [w, x, y, z], translation: [tx, ty, tz] }
}

fn transform_from(pose: &Pose) -> Transform {
    let [w, x, y, z] = pose.rotation;
    let [tx, ty, tz] = pose.translation;
    Transform { rotation: Quaternion { w, x, y, z }, translation: Translation { x: tx, y: ty, z: tz } }
}

pub(super) fn create(request: &WriteRequest) -> Result<Box<dyn PointWriter>> {
    let copy_images = request.settings.e57.keep_images && request.source_format == Format::E57;
    Ok(Box::new(E57PointWriter {
        path: request.path.to_path_buf(),
        cloud: request.cloud.clone(),
        image_source: copy_images.then(|| request.source.to_path_buf()),
    }))
}

struct E57PointWriter {
    path: PathBuf,
    cloud: CloudInfo,
    image_source: Option<PathBuf>,
}

impl PointWriter for E57PointWriter {
    fn write(self: Box<Self>, chunks: &mut dyn Iterator<Item = PointChunk>, _progress: &Progress) -> Result<()> {
        let attributes = self.cloud.attributes;
        let mut file = E57Writer::from_file(&self.path, &new_guid())?;
        file.set_coordinate_metadata(self.cloud.crs_wkt.clone());
        if attributes.normal {
            file.register_extension(Extension::new(NORMALS_PREFIX, NORMALS_URI))?;
        }

        let mut scans = ScanGroups::new(chunks);
        while let Some((index, chunks)) = scans.next_group() {
            let scan = self.cloud.scan(index);
            let guid = scan.guid.clone().unwrap_or_else(new_guid);
            let mut cloud = file.add_pointcloud(&guid, prototype(attributes, scan.grid))?;
            cloud.set_name(Some(scan.name.clone()));
            cloud.set_transform(scan.pose.as_ref().map(transform_from));
            if attributes.intensity {
                cloud.set_intensity_limits(Some(IntensityLimits {
                    intensity_min: Some(RecordValue::Single(0.0)),
                    intensity_max: Some(RecordValue::Single(1.0)),
                }));
            }
            if attributes.color {
                let (min, max) = (Some(RecordValue::Integer(0)), Some(RecordValue::Integer(255)));
                cloud.set_color_limits(Some(ColorLimits {
                    red_min: min.clone(),
                    red_max: max.clone(),
                    green_min: min.clone(),
                    green_max: max.clone(),
                    blue_min: min,
                    blue_max: max,
                }));
            }
            for chunk in chunks {
                if scan.grid.is_some() && chunk.grid.is_none() {
                    return Err(Error::invalid(format!("scan \"{}\" lost its grid positions", scan.name)));
                }
                for index in 0..chunk.len() {
                    let mut point = chunk.record(index);
                    if let Some(pose) = scan.pose {
                        point.position = pose.to_local(point.position);
                        point.normal = pose.direction_to_local(point.normal.map(f64::from)).map(|v| v as f32);
                    }
                    if scan.grid.is_some_and(|grid| !grid.contains(point.grid)) {
                        return Err(Error::invalid(format!(
                            "a point of scan \"{}\" lies outside the scan's grid (row {}, column {})",
                            scan.name, point.grid[0], point.grid[1]
                        )));
                    }
                    cloud.add_point(record_values(&point, attributes, scan.grid.is_some()))?;
                }
            }
            // The grid's extent, which can be larger than the rows and columns that have points.
            if let Some(grid) = scan.grid {
                cloud.set_index_bounds(Some(IndexBounds {
                    row_min: Some(i64::from(grid.rows[0])),
                    row_max: Some(i64::from(grid.rows[1])),
                    column_min: Some(i64::from(grid.columns[0])),
                    column_max: Some(i64::from(grid.columns[1])),
                    ..IndexBounds::default()
                }));
            }
            cloud.finalize()?;
        }

        if let Some(source) = &self.image_source {
            copy_images(source, &mut file)?;
        }
        file.finalize()?;
        Ok(())
    }
}

/// The record layout written for a cloud with these attributes, and a scan with this grid.
fn prototype(attributes: Attributes, grid: Option<ScanGrid>) -> Vec<Record> {
    let double = RecordDataType::Double { min: None, max: None };
    let color = RecordDataType::Integer { min: 0, max: 255 };
    let mut records = vec![
        Record { name: RecordName::CartesianX, data_type: double.clone() },
        Record { name: RecordName::CartesianY, data_type: double.clone() },
        Record { name: RecordName::CartesianZ, data_type: double.clone() },
    ];
    if attributes.intensity {
        records.push(Record {
            name: RecordName::Intensity,
            data_type: RecordDataType::Single { min: Some(0.0), max: Some(1.0) },
        });
    }
    if attributes.color {
        for name in [RecordName::ColorRed, RecordName::ColorGreen, RecordName::ColorBlue] {
            records.push(Record { name, data_type: color.clone() });
        }
    }
    if attributes.time {
        records.push(Record { name: RecordName::TimeStamp, data_type: double });
    }
    if attributes.normal {
        for field in NORMAL_FIELDS {
            records.push(Record {
                name: RecordName::Unknown { namespace: NORMALS_PREFIX.to_string(), name: field.to_string() },
                data_type: RecordDataType::Single { min: Some(-1.0), max: Some(1.0) },
            });
        }
    }
    if let Some(grid) = grid {
        let range = |[min, max]: [i32; 2]| RecordDataType::Integer { min: i64::from(min), max: i64::from(max) };
        records.push(Record { name: RecordName::RowIndex, data_type: range(grid.rows) });
        records.push(Record { name: RecordName::ColumnIndex, data_type: range(grid.columns) });
    }
    records
}

/// One point's values, in the order of [`prototype`]. Position and normal are already in
/// scanner coordinates.
fn record_values(point: &PointRecord, attributes: Attributes, gridded: bool) -> Vec<RecordValue> {
    let mut values: Vec<RecordValue> = point.position.into_iter().map(RecordValue::Double).collect();
    if attributes.intensity {
        values.push(RecordValue::Single(point.intensity));
    }
    if attributes.color {
        values.extend(point.color.map(|channel| RecordValue::Integer(i64::from(color_to_u8(channel)))));
    }
    if attributes.time {
        values.push(RecordValue::Double(point.time));
    }
    if attributes.normal {
        values.extend(point.normal.map(|component| RecordValue::Single(component.clamp(-1.0, 1.0))));
    }
    if gridded {
        values.extend(point.grid.map(|index| RecordValue::Integer(i64::from(index))));
    }
    values
}

/// Copies every image of `source` into `target`. Point clouds keep their GUIDs when written, so
/// the images' references to them stay valid.
fn copy_images(source: &Path, target: &mut E57Writer<std::fs::File>) -> Result<()> {
    let mut reader = E57Reader::from_file(source)?;
    for image in reader.images() {
        copy_image(&mut reader, &image, target)?;
    }
    Ok(())
}

fn copy_image(reader: &mut FileReader, image: &Image, target: &mut E57Writer<std::fs::File>) -> Result<()> {
    let guid = image.guid.clone().unwrap_or_else(new_guid);
    let mut writer = target.add_image(&guid)?;
    if let Some(name) = &image.name {
        writer.set_name(name);
    }
    if let Some(description) = &image.description {
        writer.set_description(description);
    }
    if let Some(cloud) = &image.pointcloud_guid {
        writer.set_pointcloud_guid(cloud);
    }
    if let Some(transform) = &image.transform {
        writer.set_transform(transform.clone());
    }
    if let Some(time) = &image.acquisition {
        writer.set_acquisition(time.clone());
    }
    if let Some(reference) = &image.visual_reference {
        let (data, mask) = read_image_data(reader, &reference.blob.data, &reference.mask)?;
        let mut mask = mask.as_deref();
        let format = reference.blob.format.clone();
        let properties = reference.properties.clone();
        writer.add_visual_reference(
            format,
            &mut data.as_slice(),
            properties,
            mask.as_mut().map(|m| m as &mut dyn Read),
        )?;
    }
    match &image.projection {
        Some(Projection::Pinhole(image)) => {
            let (data, mask) = read_image_data(reader, &image.blob.data, &image.mask)?;
            let mut mask = mask.as_deref();
            let format = image.blob.format.clone();
            writer.add_pinhole(
                format,
                &mut data.as_slice(),
                image.properties.clone(),
                mask.as_mut().map(|m| m as &mut dyn Read),
            )?;
        }
        Some(Projection::Spherical(image)) => {
            let (data, mask) = read_image_data(reader, &image.blob.data, &image.mask)?;
            let mut mask = mask.as_deref();
            let format = image.blob.format.clone();
            writer.add_spherical(
                format,
                &mut data.as_slice(),
                image.properties.clone(),
                mask.as_mut().map(|m| m as &mut dyn Read),
            )?;
        }
        Some(Projection::Cylindrical(image)) => {
            let (data, mask) = read_image_data(reader, &image.blob.data, &image.mask)?;
            let mut mask = mask.as_deref();
            let format = image.blob.format.clone();
            writer.add_cylindrical(
                format,
                &mut data.as_slice(),
                image.properties.clone(),
                mask.as_mut().map(|m| m as &mut dyn Read),
            )?;
        }
        None => {}
    }
    writer.finalize()?;
    Ok(())
}

/// Reads an image and its optional mask into memory.
fn read_image_data(
    reader: &mut FileReader,
    image: &e57::Blob,
    mask: &Option<e57::Blob>,
) -> Result<(Vec<u8>, Option<Vec<u8>>)> {
    let mut read = |blob: &e57::Blob| -> Result<Vec<u8>> {
        let mut data = Vec::with_capacity(blob.length as usize);
        reader.blob(blob, &mut data)?;
        Ok(data)
    };
    let image = read(image)?;
    let mask = mask.as_ref().map(read).transpose()?;
    Ok((image, mask))
}

/// A random version 4 UUID, which is what E57 uses to identify files, clouds and images.
fn new_guid() -> String {
    use std::hash::{BuildHasher, Hasher};
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(0);

    let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_nanos();
    let random_half = |salt: u64| {
        let mut hasher = std::collections::hash_map::RandomState::new().build_hasher();
        hasher.write_u64(salt);
        hasher.write_u128(nanos);
        hasher.finish().to_be_bytes()
    };
    let count = COUNTER.fetch_add(1, Ordering::Relaxed);
    let mut bytes = [0u8; 16];
    bytes[..8].copy_from_slice(&random_half(count));
    bytes[8..].copy_from_slice(&random_half(!count));
    bytes[6] = (bytes[6] & 0x0f) | 0x40; // version 4
    bytes[8] = (bytes[8] & 0x3f) | 0x80; // RFC 4122 variant

    let hex: String = bytes.iter().map(|byte| format!("{byte:02x}")).collect();
    format!("{{{}-{}-{}-{}-{}}}", &hex[0..8], &hex[8..12], &hex[12..16], &hex[16..20], &hex[20..32])
}
