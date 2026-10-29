//! End-to-end tests for Potree: the folder layout as the Potree viewer reads it, round trips
//! through the point model, output naming for folder outputs, and reading Potree 1.x folders.

use std::fs;
use std::path::{Path, PathBuf};

use epcc_core::model::{ExtraField, LidarDetails, Returns, ScalarType};
use epcc_core::settings::{ExistingFiles, LasVersion, ScanMode};
use epcc_core::{
    Attributes, ClaimedPaths, CloudInfo, ConversionJob, Format, Outcome, PointChunk, Progress, Settings, convert,
};
use las::point::{Classification, ScanDirection};
use serde_json::Value;

/// More points than a leaf holds, so the octree has several levels.
const POINTS: usize = 150_000;
/// The extra values of the generated files: three described fields and four undescribed bytes.
const EXTRA_BYTES: u16 = 2 + 4 + 4 + 4;

fn test_folder(name: &str) -> PathBuf {
    let folder = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("potree_{name}"));
    let _ = fs::remove_dir_all(&folder);
    fs::create_dir_all(&folder).unwrap();
    folder
}

fn run(input: &Path, format: Format, settings: &Settings) -> epcc_core::Result<Outcome> {
    let job =
        ConversionJob { input: input.to_path_buf(), format, output_folder: input.parent().unwrap().to_path_buf() };
    convert(&job, settings, &Progress::new(), &ClaimedPaths::default())
}

fn convert_to(input: &Path, format: Format, settings: &Settings) -> PathBuf {
    match run(input, format, settings).unwrap() {
        Outcome::Converted(mut report) => report.outputs.remove(0),
        Outcome::Skipped(path) => panic!("unexpectedly skipped {}", path.display()),
    }
}

fn read_all(path: &Path) -> (CloudInfo, PointChunk) {
    let format = Format::from_path(path).unwrap();
    let reader = format.open(path, &Default::default()).unwrap();
    let info = reader.info().clone();
    let mut all = PointChunk::new(0, info.attributes);
    reader
        .read(
            &mut |chunk| {
                all.append(chunk);
                Ok(())
            },
            &Progress::new(),
        )
        .unwrap();
    (info, all)
}

fn names_in(folder: &Path) -> Vec<String> {
    let mut names: Vec<String> =
        fs::read_dir(folder).unwrap().map(|entry| entry.unwrap().file_name().to_string_lossy().into_owned()).collect();
    names.sort();
    names
}

/// An entry of the LAS extra bytes record.
fn descriptor(data_type: u8, options: u8, name: &str, scale: f64, offset: f64, description: &str) -> Vec<u8> {
    let mut entry = vec![0u8; 192];
    entry[2] = data_type;
    entry[3] = options;
    entry[4..4 + name.len()].copy_from_slice(name.as_bytes());
    entry[112..120].copy_from_slice(&scale.to_le_bytes());
    entry[136..144].copy_from_slice(&offset.to_le_bytes());
    entry[160..160 + description.len()].copy_from_slice(description.as_bytes());
    entry
}

fn extra_bytes_of(index: usize) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&((index * 7) as u16).to_le_bytes()); // amplitude
    bytes.extend_from_slice(&(index as f32 * 0.25 - 100.0).to_le_bytes()); // deviation
    bytes.extend_from_slice(&(index as i32 * 3 - 5000).to_le_bytes()); // range, scaled
    bytes.extend_from_slice(&(index as u32).to_be_bytes()); // undescribed
    bytes
}

/// A LAS 1.4 file of `count` points with every point field set to varying values, extra bytes
/// and a coordinate system, spread through a block of 50 × 30 × 10 m.
fn write_lidar_las(path: &Path, count: usize) {
    let mut builder = las::Builder::from((1, 4));
    builder.point_format = las::point::Format::new(7).unwrap();
    builder.point_format.extra_bytes = EXTRA_BYTES;
    let transform = las::Transform { scale: 0.001, offset: 0.0 };
    builder.transforms = las::Vector { x: transform, y: transform, z: transform };
    let mut data = descriptor(3, 0, "amplitude", 0.0, 0.0, "Echo amplitude");
    data.extend(descriptor(9, 0, "deviation", 0.0, 0.0, "Pulse shape deviation"));
    data.extend(descriptor(6, 0b11000, "range", 0.001, 10.0, "Range in metres"));
    builder.vlrs.push(las::Vlr { user_id: "LASF_Spec".into(), record_id: 4, description: "Extra Bytes".into(), data });
    let mut wkt = b"LOCAL_CS[\"test grid\",UNIT[\"metre\",1]]".to_vec();
    wkt.push(0);
    builder.vlrs.push(las::Vlr {
        user_id: "LASF_Projection".into(),
        record_id: 2112,
        description: "WKT".into(),
        data: wkt,
    });
    let header = builder.into_header().unwrap();
    let mut writer = las::Writer::from_path(path, header).unwrap();
    let classes = [1, 2, 3, 6, 9, 17];
    for index in 0..count {
        let t = index as f64;
        let returns = (index % 7) as u8 + 1;
        let point = las::Point {
            x: 1000.0 + ((t * 137.0) % 50_000.0).round() / 1000.0,
            y: 2000.0 + ((t * 71.0) % 30_000.0).round() / 1000.0,
            z: 50.0 + ((t * 13.0) % 10_000.0).round() / 1000.0,
            intensity: (index * 37 % 65536) as u16,
            return_number: (index % returns as usize) as u8 + 1,
            number_of_returns: returns,
            scan_direction: if index % 2 == 0 { ScanDirection::LeftToRight } else { ScanDirection::RightToLeft },
            is_edge_of_flight_line: index % 11 == 0,
            classification: Classification::new(classes[index % classes.len()]).unwrap(),
            is_synthetic: index % 3 == 0,
            is_key_point: index % 5 == 0,
            is_withheld: index % 17 == 0,
            is_overlap: index % 19 == 0,
            scanner_channel: (index % 4) as u8,
            scan_angle: (index % 61) as f32 - 30.0,
            user_data: (index % 256) as u8,
            point_source_id: (index / 1000) as u16 + 100,
            gps_time: Some(1000.0 + index as f64 * 0.5),
            color: Some(las::Color { red: (index * 3) as u16, green: (index * 5) as u16, blue: 40_000 }),
            extra_bytes: extra_bytes_of(index),
            ..las::Point::default()
        };
        writer.write_point(point).unwrap();
    }
    writer.close().unwrap();
}

fn las_1_4() -> Settings {
    let mut settings = Settings::default();
    settings.export.las.version = LasVersion::V1_4;
    settings
}

/// Every point field a LAS round trip keeps, as exact values.
#[derive(Clone, Debug, PartialEq)]
struct Row {
    millimetres: [i64; 3],
    time: f64,
    intensity: u16,
    color: [u16; 3],
    classification: u8,
    returns: Returns,
    lidar: LidarDetails,
    extra: Vec<u8>,
}

/// The points in time order, which is unique per point.
fn rows(chunk: &PointChunk) -> Vec<Row> {
    let mut rows: Vec<Row> = (0..chunk.len())
        .map(|index| {
            let point = chunk.record(index);
            Row {
                millimetres: point.position.map(|value| (value * 1000.0).round() as i64),
                time: point.time,
                intensity: (point.intensity * 65535.0).round() as u16,
                color: point.color,
                classification: point.classification,
                returns: point.returns,
                lidar: point.lidar,
                extra: chunk.extra.as_ref().map(|extra| extra.get(index).to_vec()).unwrap_or_default(),
            }
        })
        .collect();
    rows.sort_by(|a, b| a.time.total_cmp(&b.time));
    rows
}

/// Name, description, type and scaling of each extra field.
fn field_summary(fields: &[ExtraField]) -> Vec<(String, String, u16, Option<ScalarType>, f64, f64)> {
    fields.iter().map(|f| (f.name.clone(), f.description.clone(), f.size, f.scalar, f.scale, f.offset)).collect()
}

/// A Potree 2.0 folder parsed the way the Potree viewer reads it, independently of EPCC's reader.
struct PotreeFolder {
    metadata: Value,
    octree: Vec<u8>,
    nodes: Vec<Node>,
    /// Bytes per point and each attribute's name, start, element type and element count.
    record_size: usize,
    attributes: Vec<(String, usize, String, usize)>,
}

#[derive(Clone, Debug)]
struct Node {
    name: String,
    min: [f64; 3],
    size: [f64; 3],
    points: u64,
    byte_offset: u64,
    byte_size: u64,
    children: u8,
}

impl PotreeFolder {
    fn read(folder: &Path) -> PotreeFolder {
        let metadata: Value = serde_json::from_str(&fs::read_to_string(folder.join("metadata.json")).unwrap()).unwrap();
        let hierarchy = fs::read(folder.join("hierarchy.bin")).unwrap();
        let octree = fs::read(folder.join("octree.bin")).unwrap();
        assert_eq!(metadata["version"], "2.0");
        assert_eq!(metadata["encoding"], "DEFAULT");

        let mut attributes = Vec::new();
        let mut record_size = 0;
        for attribute in metadata["attributes"].as_array().unwrap() {
            let size = attribute["size"].as_u64().unwrap() as usize;
            let elements = attribute["numElements"].as_u64().unwrap() as usize;
            let element_size = attribute["elementSize"].as_u64().unwrap() as usize;
            let kind = attribute["type"].as_str().unwrap().to_string();
            assert_eq!(size, elements * element_size, "{attribute}");
            assert_eq!(element_size, type_size(&kind), "{attribute}");
            assert_eq!(attribute["min"].as_array().unwrap().len(), elements, "{attribute}");
            assert_eq!(attribute["max"].as_array().unwrap().len(), elements, "{attribute}");
            attributes.push((attribute["name"].as_str().unwrap().to_string(), record_size, kind, elements));
            record_size += size;
        }
        assert_eq!(attributes[0].0, "position");

        let min = vector(&metadata["boundingBox"]["min"]);
        let max = vector(&metadata["boundingBox"]["max"]);
        let size = std::array::from_fn(|axis| max[axis] - min[axis]);
        assert!(size.iter().all(|&edge| (edge - size[0]).abs() < 1e-6), "the bounding box is a cube: {size:?}");
        let root = Node { name: "r".into(), min, size, points: 0, byte_offset: 0, byte_size: 0, children: 0 };
        let first_chunk = metadata["hierarchy"]["firstChunkSize"].as_u64().unwrap();
        let mut nodes = Vec::new();
        let mut chunk_bytes = 0;
        let mut chunks = vec![(root, 0, first_chunk)];
        while let Some((root, offset, size)) = chunks.pop() {
            chunk_bytes += size;
            let chunk = &hierarchy[offset as usize..(offset + size) as usize];
            let mut pending = vec![root];
            for (index, entry) in chunk.as_chunks::<22>().0.iter().enumerate() {
                let mut node = pending[index].clone();
                let kind = entry[0];
                let mask = entry[1];
                node.points = u64::from(u32::from_le_bytes(entry[2..6].try_into().unwrap()));
                let byte_offset = u64::from_le_bytes(entry[6..14].try_into().unwrap());
                let byte_size = u64::from_le_bytes(entry[14..22].try_into().unwrap());
                if kind == 2 {
                    assert!(index > 0, "a chunk starts with its real node");
                    chunks.push((node, byte_offset, byte_size));
                    continue;
                }
                assert_eq!(kind, if mask == 0 { 1 } else { 0 }, "node {} has the wrong type", node.name);
                (node.byte_offset, node.byte_size, node.children) = (byte_offset, byte_size, mask);
                for child in (0..8).filter(|child| mask & (1 << child) != 0) {
                    let half = node.size.map(|edge| edge / 2.0);
                    let bits = [4, 2, 1].map(|bit| f64::from(u8::from(child & bit != 0)));
                    pending.push(Node {
                        name: format!("{}{child}", node.name),
                        min: std::array::from_fn(|axis| node.min[axis] + bits[axis] * half[axis]),
                        size: half,
                        ..node.clone()
                    });
                }
                nodes.push(node);
            }
            assert_eq!(pending.len(), chunk.len() / 22, "the child masks account for every entry of the chunk");
        }
        assert_eq!(chunk_bytes, hierarchy.len() as u64, "every hierarchy chunk is used once");
        PotreeFolder { metadata, octree, nodes, record_size, attributes }
    }

    fn attribute(&self, name: &str) -> &(String, usize, String, usize) {
        self.attributes.iter().find(|attribute| attribute.0 == name).unwrap_or_else(|| panic!("no attribute {name}"))
    }

    /// The stored values of every element of attribute `name`, node by node.
    fn values(&self, node: &Node, name: &str) -> Vec<Vec<f64>> {
        let (_, at, kind, elements) = self.attribute(name).clone();
        let start = node.byte_offset as usize;
        let records = &self.octree[start..start + node.byte_size as usize];
        records
            .chunks_exact(self.record_size)
            .map(|record| {
                (0..elements).map(|element| decode(&kind, &record[at + element * type_size(&kind)..])).collect()
            })
            .collect()
    }

    /// The positions of a node's points, in metres.
    fn positions(&self, node: &Node) -> Vec<[f64; 3]> {
        let scale = vector(&self.metadata["scale"]);
        let offset = vector(&self.metadata["offset"]);
        let values = self.values(node, "position");
        values.iter().map(|value| std::array::from_fn(|axis| value[axis] * scale[axis] + offset[axis])).collect()
    }

    /// Checks everything the viewer relies on, for a cloud of `points` points.
    fn check(&self, points: u64) {
        assert_eq!(self.metadata["points"].as_u64(), Some(points));
        let total: u64 = self.nodes.iter().map(|node| node.points).sum();
        assert_eq!(total, points, "the nodes hold every point");
        let depth = self.nodes.iter().map(|node| node.name.len() - 1).max().unwrap();
        assert_eq!(self.metadata["hierarchy"]["depth"].as_u64(), Some(depth as u64));
        assert_eq!(self.metadata["hierarchy"]["stepSize"].as_u64(), Some(4));
        let cube = vector(&self.metadata["boundingBox"]["max"])[0] - vector(&self.metadata["boundingBox"]["min"])[0];
        assert!((self.metadata["spacing"].as_f64().unwrap() - cube / 128.0).abs() < 1e-9);

        // The nodes' points lie one after another, filling octree.bin exactly.
        let mut stored: Vec<&Node> = self.nodes.iter().filter(|node| node.byte_size > 0).collect();
        stored.sort_by_key(|node| node.byte_offset);
        let mut end = 0;
        for node in &stored {
            assert_eq!(node.byte_offset, end, "node {} follows the one before it", node.name);
            assert_eq!(node.byte_size, node.points * self.record_size as u64, "node {}", node.name);
            end += node.byte_size;
        }
        assert_eq!(end, self.octree.len() as u64);
        assert!(self.nodes.iter().all(|node| node.points > 0 || node.byte_size == 0));

        // Every point lies in its node's cube, give or take the rounding to the scale.
        let tolerance = vector(&self.metadata["scale"])[0];
        for node in &self.nodes {
            for position in self.positions(node) {
                for axis in 0..3 {
                    let (low, high) = (node.min[axis], node.min[axis] + node.size[axis]);
                    assert!(
                        position[axis] >= low - tolerance && position[axis] <= high + tolerance,
                        "a point of node {} lies outside it: {position:?}",
                        node.name
                    );
                }
            }
        }

        // The ranges in the metadata are those of the stored values.
        for attribute in self.metadata["attributes"].as_array().unwrap() {
            let name = attribute["name"].as_str().unwrap();
            let elements = attribute["numElements"].as_u64().unwrap() as usize;
            let mut low = vec![f64::INFINITY; elements];
            let mut high = vec![f64::NEG_INFINITY; elements];
            for node in &self.nodes {
                let values = if name == "position" {
                    self.positions(node).iter().map(|position| position.to_vec()).collect()
                } else {
                    self.values(node, name)
                };
                for value in values {
                    for element in 0..elements {
                        low[element] = low[element].min(value[element]);
                        high[element] = high[element].max(value[element]);
                    }
                }
            }
            if points == 0 {
                continue;
            }
            for element in 0..elements {
                let (min, max) =
                    (attribute["min"][element].as_f64().unwrap(), attribute["max"][element].as_f64().unwrap());
                assert!((min - low[element]).abs() < 1e-9 && (max - high[element]).abs() < 1e-9, "range of {name}");
            }
        }
    }
}

fn vector(value: &Value) -> [f64; 3] {
    std::array::from_fn(|axis| value[axis].as_f64().unwrap())
}

fn type_size(kind: &str) -> usize {
    match kind {
        "int8" | "uint8" => 1,
        "int16" | "uint16" => 2,
        "int32" | "uint32" | "float" => 4,
        "int64" | "uint64" | "double" => 8,
        other => panic!("unknown type {other}"),
    }
}

fn decode(kind: &str, bytes: &[u8]) -> f64 {
    macro_rules! number {
        ($type:ty) => {
            <$type>::from_le_bytes(bytes[..size_of::<$type>()].try_into().unwrap()) as f64
        };
    }
    match kind {
        "int8" => number!(i8),
        "uint8" => number!(u8),
        "int16" => number!(i16),
        "uint16" => number!(u16),
        "int32" => number!(i32),
        "uint32" => number!(u32),
        "int64" => number!(i64),
        "uint64" => number!(u64),
        "float" => number!(f32),
        "double" => number!(f64),
        other => panic!("unknown type {other}"),
    }
}

#[test]
fn potree_folders_are_laid_out_as_viewers_read_them() {
    let folder = test_folder("layout");
    let source = folder.join("site.las");
    write_lidar_las(&source, POINTS);
    let written = convert_to(&source, Format::Potree, &Settings::default());
    assert_eq!(written, folder.join("site.potree"));
    assert_eq!(names_in(&written), ["hierarchy.bin", "metadata.json", "octree.bin"]);
    assert_eq!(names_in(&folder), ["site.las", "site.potree"], "no temporary files are left");

    let potree = PotreeFolder::read(&written);
    potree.check(POINTS as u64);
    assert!(potree.nodes.iter().any(|node| node.name.len() > 2), "the octree has several levels");
    let (leaves, inner): (Vec<&Node>, Vec<&Node>) = potree.nodes.iter().partition(|node| node.children == 0);
    assert!(leaves.iter().all(|node| node.points <= 20_000), "leaves are split at 20 000 points");
    assert!(inner.iter().all(|node| node.points > 0), "inner nodes hold a sample of the points below them");
    let names: Vec<&str> = potree.attributes.iter().map(|attribute| attribute.0.as_str()).collect();
    assert_eq!(
        names,
        [
            "position",
            "intensity",
            "return number",
            "number of returns",
            "classification flags",
            "scanner channel",
            "scan direction flag",
            "edge of flight line",
            "classification",
            "user data",
            "scan angle",
            "point source id",
            "gps-time",
            "rgb",
            "amplitude",
            "deviation",
            "range",
            "extra bytes",
        ]
    );
    assert_eq!(potree.metadata["name"], "site");
    assert!(potree.metadata["projection"].as_str().unwrap().starts_with("LOCAL_CS"));
    let range = &potree.metadata["attributes"][16];
    assert_eq!((range["scale"][0].as_f64(), range["offset"][0].as_f64()), (Some(0.001), Some(10.0)));

    // Each source point is in exactly one node: the stored points are the source points.
    let mut stored: Vec<(i64, [i64; 3])> = Vec::new();
    for node in &potree.nodes {
        let times = potree.values(node, "gps-time");
        for (position, time) in potree.positions(node).iter().zip(times) {
            stored.push(((time[0] * 2.0) as i64, position.map(|value| (value * 1000.0).round() as i64)));
        }
    }
    stored.sort_unstable();
    let (_, source_points) = read_all(&source);
    let mut want: Vec<(i64, [i64; 3])> =
        rows(&source_points).iter().map(|row| ((row.time * 2.0) as i64, row.millimetres)).collect();
    want.sort_unstable();
    assert!(stored == want, "the nodes hold a different set of points");
}

#[test]
fn las_to_potree_to_las_keeps_every_attribute() {
    let folder = test_folder("las_round_trip");
    let source = folder.join("lidar.las");
    write_lidar_las(&source, 40_000);
    let (source_info, source_chunk) = read_all(&source);
    let want = rows(&source_chunk);

    let potree = convert_to(&source, Format::Potree, &las_1_4());
    let (info, chunk) = read_all(&potree);
    assert_eq!(info.attributes, source_info.attributes);
    assert_eq!(info.crs_wkt, source_info.crs_wkt);
    assert_eq!(info.point_count, Some(40_000));
    assert_eq!(field_summary(&info.extra_fields), field_summary(&source_info.extra_fields));
    assert!(rows(&chunk) == want, "the Potree folder holds different point fields");

    let back = convert_to(&potree, Format::Las, &las_1_4());
    assert_eq!(back, folder.join("lidar (1).las"), "named after the Potree folder, next to the source");
    let (back_info, back_chunk) = read_all(&back);
    assert_eq!(back_info.attributes, source_info.attributes);
    assert_eq!(field_summary(&back_info.extra_fields), field_summary(&source_info.extra_fields));
    assert!(rows(&back_chunk) == want, "the LAS file read back holds different point fields");
}

#[test]
fn normals_survive_potree() {
    let folder = test_folder("normals");
    let source = folder.join("normals.xyz");
    let mut text = String::from("x y z nx ny nz\n");
    for index in 0..30_000 {
        let angle = index as f32 * 0.001;
        let (x, y, z) = ((index % 300) as f32 * 0.01, (index / 300) as f32 * 0.01, angle.sin());
        text.push_str(&format!("{x:.3} {y:.3} {z:.3} {} {} 0.25\n", angle.cos(), -angle.sin()));
    }
    fs::write(&source, text).unwrap();
    let (_, want) = read_all(&source);

    let potree = convert_to(&source, Format::Potree, &Settings::default());
    let names: Vec<String> =
        PotreeFolder::read(&potree).attributes.iter().map(|attribute| attribute.0.clone()).collect();
    assert_eq!(names, ["position", "NormalX", "NormalY", "NormalZ"]);
    let (info, got) = read_all(&potree);
    assert_eq!(info.attributes, Attributes { normal: true, ..Attributes::NONE });
    let key = |chunk: &PointChunk| {
        let mut points: Vec<([i64; 3], [u32; 3])> = (0..chunk.len())
            .map(|index| {
                let point = chunk.record(index);
                (point.position.map(|value| (value * 1000.0).round() as i64), point.normal.map(f32::to_bits))
            })
            .collect();
        points.sort_unstable();
        points
    };
    assert!(key(&got) == key(&want), "normals are kept exactly");
}

#[test]
fn several_scans_are_merged_or_split() {
    let folder = test_folder("scans");
    let source = folder.join("site.pts");
    fs::write(&source, "2\n0 0 0 100 10 20 30\n1 0 0 -100 10 20 30\n1\n5 5 5 0 40 50 60\n").unwrap();

    let merged = convert_to(&source, Format::Potree, &Settings::default());
    assert_eq!(merged, folder.join("site.potree"));
    PotreeFolder::read(&merged).check(3);
    assert_eq!(read_all(&merged).1.len(), 3);

    let mut split = Settings::default();
    split.processing.scans = ScanMode::Split;
    let Outcome::Converted(report) = run(&source, Format::Potree, &split).unwrap() else { panic!("skipped") };
    assert_eq!(report.outputs, [folder.join("site_Scan_1.potree"), folder.join("site_Scan_2.potree")]);
    assert_eq!(read_all(&report.outputs[0]).1.len(), 2);
    assert_eq!(read_all(&report.outputs[1]).1.len(), 1);
    assert_eq!(names_in(&folder), ["site.potree", "site.pts", "site_Scan_1.potree", "site_Scan_2.potree"]);
}

#[test]
fn empty_and_single_point_clouds() {
    let folder = test_folder("tiny");
    for count in [0, 1] {
        let source = folder.join(format!("points{count}.las"));
        write_lidar_las(&source, count);
        let written = convert_to(&source, Format::Potree, &Settings::default());
        let potree = PotreeFolder::read(&written);
        potree.check(count as u64);
        assert_eq!(potree.nodes.len(), 1, "only the root");
        let (_, chunk) = read_all(&written);
        assert_eq!(chunk.len(), count);
        let back = convert_to(&written, Format::Laz, &Settings::default());
        assert_eq!(read_all(&back).1.len(), count);
    }
}

#[test]
fn potree_output_is_reproducible() {
    let folder = test_folder("reproducible");
    let source = folder.join("cloud.las");
    write_lidar_las(&source, 120_000);
    let mut overwrite = Settings::default();
    overwrite.general.existing_files = ExistingFiles::Overwrite;
    let files = ["metadata.json", "hierarchy.bin", "octree.bin"];
    let first = convert_to(&source, Format::Potree, &overwrite);
    let before: Vec<Vec<u8>> = files.iter().map(|name| fs::read(first.join(name)).unwrap()).collect();
    let second = convert_to(&source, Format::Potree, &overwrite);
    assert_eq!(first, second);
    for (name, bytes) in files.iter().zip(before) {
        assert!(fs::read(second.join(name)).unwrap() == bytes, "{name} differs between runs");
    }
}

#[test]
fn folder_outputs_follow_the_existing_files_setting() {
    let folder = test_folder("existing");
    let source = folder.join("cloud.las");
    write_lidar_las(&source, 500);
    let with = |existing| {
        let mut settings = Settings::default();
        settings.general.existing_files = existing;
        settings
    };

    let first = convert_to(&source, Format::Potree, &Settings::default());
    assert_eq!(first, folder.join("cloud.potree"));
    assert_eq!(convert_to(&source, Format::Potree, &with(ExistingFiles::Rename)), folder.join("cloud (1).potree"));
    let skipped = run(&source, Format::Potree, &with(ExistingFiles::Skip)).unwrap();
    assert_eq!(skipped, Outcome::Skipped(folder.join("cloud.potree")));

    // Overwriting replaces the whole cloud folder.
    fs::write(first.join("stale.txt"), "old").unwrap();
    assert_eq!(convert_to(&source, Format::Potree, &with(ExistingFiles::Overwrite)), first);
    assert_eq!(names_in(&first), ["hierarchy.bin", "metadata.json", "octree.bin"]);

    // A folder of the same name that is not a point cloud is never replaced.
    let other = folder.join("other");
    fs::create_dir(&other).unwrap();
    fs::copy(&source, other.join("cloud.las")).unwrap();
    fs::create_dir(other.join("cloud.potree")).unwrap();
    fs::write(other.join("cloud.potree").join("notes.txt"), "keep").unwrap();
    let written = convert_to(&other.join("cloud.las"), Format::Potree, &with(ExistingFiles::Overwrite));
    assert_eq!(written, other.join("cloud (1).potree"));
    assert_eq!(names_in(&other.join("cloud.potree")), ["notes.txt"]);

    // Nor is the input, when a Potree folder is converted to Potree.
    let again = convert_to(&first, Format::Potree, &with(ExistingFiles::Overwrite));
    assert_eq!(again, folder.join("cloud (2).potree"));
    assert_eq!(read_all(&again).1.len(), 500);
    assert_eq!(read_all(&first).1.len(), 500);

    // The cloud's metadata.json works as the input as well, named after the folder.
    let metadata = first.join("metadata.json");
    assert_eq!(Format::from_path(&metadata), Some(Format::Potree));
    assert_eq!(Format::stem_of(&metadata), "cloud");
    assert_eq!(convert_to(&metadata, Format::Laz, &Settings::default()), first.join("cloud.laz"));
}

#[test]
fn failed_and_cancelled_runs_leave_nothing_behind() {
    let folder = test_folder("failed");
    let source = folder.join("cloud.las");
    write_lidar_las(&source, 5000);

    // 50 m at 0.1 nm steps do not fit into the 32-bit positions: the writer stops after reading.
    let mut too_fine = Settings::default();
    too_fine.export.las.precision = 1e-10;
    let error = run(&source, Format::Potree, &too_fine).unwrap_err();
    assert!(error.to_string().contains("precision"), "{error}");
    assert_eq!(names_in(&folder), ["cloud.las"]);

    let progress = Progress::new();
    progress.cancel();
    let job = ConversionJob { input: source.clone(), format: Format::Potree, output_folder: folder.clone() };
    let result = convert(&job, &Settings::default(), &progress, &ClaimedPaths::default());
    assert!(matches!(result, Err(epcc_core::Error::Cancelled)), "{result:?}");
    assert_eq!(names_in(&folder), ["cloud.las"]);
}

#[test]
fn compressed_potree_is_reported_as_unsupported() {
    let folder = test_folder("brotli");
    let source = folder.join("cloud.las");
    write_lidar_las(&source, 100);
    let written = convert_to(&source, Format::Potree, &Settings::default());
    let path = written.join("metadata.json");
    let text = fs::read_to_string(&path).unwrap().replace("\"DEFAULT\"", "\"BROTLI\"");
    fs::write(&path, text).unwrap();
    let error = Format::Potree.open(&written, &Default::default()).err().expect("BROTLI is refused");
    assert!(error.to_string().contains("BROTLI"), "{error}");
}

#[test]
fn potree_1_binary_folders_are_read() {
    let folder = test_folder("v1_binary").join("old.potree");
    fs::create_dir_all(folder.join("data/r")).unwrap();
    let cloud_js = r#"{
        "version": "1.7",
        "octreeDir": "data",
        "projection": "",
        "points": 3,
        "boundingBox": {"lx": 10, "ly": 20, "lz": 30, "ux": 18, "uy": 28, "uz": 38},
        "tightBoundingBox": {"lx": 10, "ly": 20, "lz": 30, "ux": 15, "uy": 25, "uz": 33},
        "pointAttributes": ["POSITION_CARTESIAN", "COLOR_PACKED", "INTENSITY", "CLASSIFICATION", "NORMAL_OCT16"],
        "spacing": 0.1,
        "scale": 0.001,
        "hierarchyStepSize": 5
    }"#;
    fs::write(folder.join("cloud.js"), cloud_js).unwrap();
    let record = |position: [u32; 3], color: [u8; 3], intensity: u16, class: u8| {
        let mut bytes = Vec::new();
        position.iter().for_each(|value| bytes.extend_from_slice(&value.to_le_bytes()));
        bytes.extend_from_slice(&[color[0], color[1], color[2], 255]);
        bytes.extend_from_slice(&intensity.to_le_bytes());
        bytes.extend_from_slice(&[class, 128, 128]);
        bytes
    };
    let mut root = record([1000, 2000, 3000], [10, 20, 30], 65535, 2);
    root.extend(record([0, 0, 0], [0, 0, 0], 0, 6));
    fs::write(folder.join("data/r/r.bin"), root).unwrap();
    // Node 6 is the upper half in x and y: its cube starts at (14, 24, 30).
    fs::write(folder.join("data/r/r6.bin"), record([500, 0, 250], [255, 255, 255], 32768, 9)).unwrap();
    fs::write(folder.join("data/r/r.hrc"), [0b0100_0000u8, 1, 0, 0, 0]).unwrap();

    assert_eq!(Format::from_path(&folder), Some(Format::Potree));
    assert_eq!(Format::from_path(&folder.join("cloud.js")), Some(Format::Potree));
    let (info, chunk) = read_all(&folder);
    assert_eq!(info.point_count, Some(3));
    assert_eq!(info.scans[0].name, "old");
    assert_eq!(
        info.attributes,
        Attributes { intensity: true, color: true, normal: true, classification: true, ..Attributes::NONE }
    );
    let points: Vec<_> = (0..chunk.len()).map(|index| chunk.record(index)).collect();
    assert_eq!(
        points.iter().map(|point| point.position).collect::<Vec<_>>(),
        [[11.0, 22.0, 33.0], [10.0, 20.0, 30.0], [14.5, 24.0, 30.25]]
    );
    assert_eq!(points[0].color, [10, 20, 30].map(|value| value * 257));
    assert_eq!((points[0].intensity, points[2].classification), (1.0, 9));
    assert!(points.iter().all(|point| (point.normal[2] - 1.0).abs() < 1e-3));

    let las = convert_to(&folder, Format::Las, &Settings::default());
    assert_eq!(las, folder.parent().unwrap().join("old.las"));
    assert_eq!(read_all(&las).1.len(), 3);
}

#[test]
fn potree_1_las_folders_are_read() {
    let folder = test_folder("v1_las").join("tiles");
    fs::create_dir_all(folder.join("data/r")).unwrap();
    let cloud_js = r#"{"version": "1.8", "octreeDir": "data", "points": 4,
        "boundingBox": {"lx": 0, "ly": 0, "lz": 0, "ux": 4, "uy": 4, "uz": 4},
        "pointAttributes": "LAS", "spacing": 0.5, "scale": 0.001, "hierarchyStepSize": 5}"#;
    fs::write(folder.join("cloud.js"), cloud_js).unwrap();
    for (name, positions) in
        [("r.las", [[0.5, 0.5, 0.5], [3.5, 3.5, 3.5]]), ("r7.las", [[3.0, 3.0, 3.0], [2.5, 2.5, 2.5]])]
    {
        let mut builder = las::Builder::from((1, 2));
        builder.point_format = las::point::Format::new(2).unwrap();
        let mut writer =
            las::Writer::from_path(folder.join("data/r").join(name), builder.into_header().unwrap()).unwrap();
        for [x, y, z] in positions {
            let color = Some(las::Color { red: 1000, green: 2000, blue: 3000 });
            writer.write_point(las::Point { x, y, z, intensity: 7, color, ..las::Point::default() }).unwrap();
        }
        writer.close().unwrap();
    }
    let (info, chunk) = read_all(&folder);
    assert_eq!(info.point_count, Some(4));
    assert!(info.attributes.color && info.attributes.intensity && info.attributes.returns);
    let positions: Vec<[f64; 3]> = (0..chunk.len()).map(|index| chunk.record(index).position).collect();
    assert_eq!(positions, [[0.5, 0.5, 0.5], [3.5, 3.5, 3.5], [3.0, 3.0, 3.0], [2.5, 2.5, 2.5]]);
    assert!((0..chunk.len()).all(|index| chunk.record(index).color == [1000, 2000, 3000]));
}

#[test]
fn unrelated_metadata_files_are_not_potree() {
    let folder = test_folder("not_potree");
    fs::write(folder.join("metadata.json"), "{}").unwrap();
    assert_eq!(Format::from_path(&folder.join("metadata.json")), None);
    assert_eq!(Format::from_path(&folder), None);
    fs::write(folder.join("octree.bin"), []).unwrap();
    assert_eq!(Format::from_path(&folder.join("metadata.json")), Some(Format::Potree));
    assert_eq!(Format::from_path(&folder), Some(Format::Potree));
}
