//! End-to-end tests: real files are written and read back through the public API.

use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use epcc_core::model::{PointRecord, color_from_u8};
use epcc_core::settings::{ExistingFiles, ScanMode};
use epcc_core::{
    Attributes, ClaimedPaths, CloudInfo, ConversionJob, Format, Outcome, PointChunk, Progress, Settings, convert,
};

/// An empty folder for one test.
fn test_folder(name: &str) -> PathBuf {
    let folder = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = fs::remove_dir_all(&folder);
    fs::create_dir_all(&folder).unwrap();
    folder
}

/// Points with every attribute XYZ files can carry, on a deterministic pattern.
fn source_points(count: usize, offset: [f64; 3]) -> Vec<PointRecord> {
    (0..count)
        .map(|i| {
            let t = i as f64;
            let normal = [0.6f32, -0.8, 0.0];
            PointRecord {
                position: [
                    offset[0] + (t * 0.137) % 50.0,
                    offset[1] + (t * 0.071) % 30.0,
                    offset[2] + (t * 0.013) % 10.0,
                ],
                intensity: (i % 1000) as f32 / 999.0,
                color: [(i % 256) as u8, (i * 7 % 256) as u8, (i * 13 % 256) as u8].map(color_from_u8),
                normal,
                ..PointRecord::default()
            }
        })
        .collect()
}

fn write_xyz(path: &Path, points: &[PointRecord]) {
    let mut text = String::from("x y z i r g b nx ny nz\n");
    for p in points {
        let [x, y, z] = p.position;
        let [r, g, b] = p.color.map(|c| c / 257);
        let [nx, ny, nz] = p.normal;
        let _ = writeln!(text, "{x:.4} {y:.4} {z:.4} {:.4} {r} {g} {b} {nx} {ny} {nz}", p.intensity);
    }
    fs::write(path, text).unwrap();
}

fn run(input: &Path, format: Format, settings: &Settings) -> Outcome {
    let job =
        ConversionJob { input: input.to_path_buf(), format, output_folder: input.parent().unwrap().to_path_buf() };
    convert(&job, settings, &Progress::new(), &ClaimedPaths::default()).unwrap()
}

fn outputs(outcome: Outcome) -> Vec<PathBuf> {
    match outcome {
        Outcome::Converted(report) => report.outputs,
        Outcome::Skipped(path) => panic!("unexpectedly skipped {}", path.display()),
    }
}

fn read_all(path: &Path) -> (CloudInfo, Vec<PointChunk>) {
    let format = Format::from_path(path).unwrap();
    let reader = format.open(path, &Default::default()).unwrap();
    let info = reader.info().clone();
    let mut chunks = Vec::new();
    reader
        .read(
            &mut |chunk| {
                chunks.push(chunk);
                Ok(())
            },
            &Progress::new(),
        )
        .unwrap();
    (info, chunks)
}

fn read_points(path: &Path) -> Vec<PointRecord> {
    let (_, chunks) = read_all(path);
    chunks.iter().flat_map(|chunk| (0..chunk.len()).map(|i| chunk.record(i))).collect()
}

fn assert_close(got: &[PointRecord], want: &[PointRecord], stored: Attributes, tolerance: f64, label: &str) {
    assert_eq!(got.len(), want.len(), "{label}: point count");
    for (index, (g, w)) in got.iter().zip(want).enumerate() {
        for axis in 0..3 {
            let error = (g.position[axis] - w.position[axis]).abs();
            assert!(error <= tolerance, "{label}: point {index} axis {axis}: {:?} vs {:?}", g.position, w.position);
        }
        if stored.intensity {
            assert!((g.intensity - w.intensity).abs() < 0.002, "{label}: intensity {} vs {}", g.intensity, w.intensity);
        }
        if stored.color {
            assert_eq!(g.color.map(|c| c / 257), w.color.map(|c| c / 257), "{label}: colour of point {index}");
        }
        if stored.normal {
            for axis in 0..3 {
                assert!((g.normal[axis] - w.normal[axis]).abs() < 1e-5, "{label}: normal");
            }
        }
    }
}

#[test]
fn every_format_round_trips() {
    let folder = test_folder("every_format");
    let source = folder.join("cloud.xyz");
    let points = source_points(50_000, [0.0; 3]);
    write_xyz(&source, &points);

    // COPC, Potree and RCS store their points in octree order, and RCS thins them; they have
    // their own tests (Potree in `potree.rs`).
    let octree_formats = [Format::Xyz, Format::Copc, Format::Potree, Format::Rcs];
    for format in Format::ALL.into_iter().filter(|f| !octree_formats.contains(f)) {
        let written = outputs(run(&source, format, &Settings::default()));
        assert_eq!(written, [folder.join(format!("cloud.{}", format.extension()))]);
        let stored = format.stored_attributes();
        assert_close(&read_points(&written[0]), &points, stored, 0.0011, format.name());
        assert!(!folder.join(format!("cloud.{}.epcc-part", format.extension())).exists());
    }
}

#[test]
fn chained_conversions_keep_the_cloud() {
    let folder = test_folder("chain");
    let points = source_points(20_000, [0.0; 3]);
    let mut current = folder.join("start.xyz");
    write_xyz(&current, &points);

    for format in [Format::E57, Format::Laz, Format::Ply, Format::Pts, Format::Las, Format::Xyz] {
        current = outputs(run(&current, format, &Settings::default())).remove(0);
    }
    let colour_and_intensity = Attributes { intensity: true, color: true, ..Attributes::NONE };
    assert_close(&read_points(&current), &points, colour_and_intensity, 0.0011, "chain");
}

#[test]
fn georeferenced_coordinates_keep_their_precision() {
    let folder = test_folder("georeferenced");
    let source = folder.join("utm.xyz");
    let points = source_points(5_000, [500_000.0, 5_400_000.0, 250.0]);
    write_xyz(&source, &points);

    for format in [Format::Laz, Format::Ply, Format::E57] {
        let written = outputs(run(&source, format, &Settings::default())).remove(0);
        assert_close(&read_points(&written), &points, Attributes::NONE, 0.0011, format.name());
    }
}

#[test]
fn min_spacing_thins_the_cloud() {
    let folder = test_folder("min_spacing");
    let source = folder.join("dense.xyz");
    write_xyz(&source, &source_points(30_000, [0.0; 3]));

    let mut settings = Settings::default();
    settings.processing.min_spacing = Some(0.5);
    let written = outputs(run(&source, Format::Las, &settings)).remove(0);
    let kept = read_points(&written);
    assert!(kept.len() > 100 && kept.len() < 30_000, "kept {}", kept.len());

    let tolerance = 0.002; // LAS rounds coordinates to the 1 mm precision
    for (i, a) in kept.iter().enumerate() {
        for b in &kept[i + 1..] {
            let distance: f64 = a.position.iter().zip(b.position).map(|(x, y)| (x - y).powi(2)).sum::<f64>().sqrt();
            assert!(distance >= 0.5 - tolerance, "points {distance} m apart");
        }
    }
}

#[test]
fn existing_outputs_follow_the_setting() {
    let folder = test_folder("existing");
    let source = folder.join("scan.xyz");
    write_xyz(&source, &source_points(100, [0.0; 3]));

    let first = outputs(run(&source, Format::Ply, &Settings::default()));
    let second = outputs(run(&source, Format::Ply, &Settings::default()));
    assert_eq!(first, [folder.join("scan.ply")]);
    assert_eq!(second, [folder.join("scan (1).ply")]);

    let mut skip = Settings::default();
    skip.general.existing_files = ExistingFiles::Skip;
    assert!(matches!(run(&source, Format::Ply, &skip), Outcome::Skipped(_)));

    // Converting a file to its own format never overwrites it.
    let mut overwrite = Settings::default();
    overwrite.general.existing_files = ExistingFiles::Overwrite;
    let again = outputs(run(&source, Format::Xyz, &overwrite));
    assert_eq!(again, [folder.join("scan (1).xyz")]);
    assert_eq!(read_points(&source).len(), 100);
}

#[test]
fn pts_scans_can_be_split_or_kept() {
    let folder = test_folder("scans");
    let source = folder.join("site.pts");
    fs::write(&source, "2\n0 0 0 100 10 20 30\n1 0 0 -100 10 20 30\n1\n5 5 5 0 40 50 60\n").unwrap();

    let mut split = Settings::default();
    split.processing.scans = ScanMode::Split;
    let files = outputs(run(&source, Format::Laz, &split));
    assert_eq!(files, [folder.join("site_Scan_1.laz"), folder.join("site_Scan_2.laz")]);
    assert_eq!(read_points(&files[0]).len(), 2);
    assert_eq!(read_points(&files[1]).len(), 1);

    let e57 = outputs(run(&source, Format::E57, &Settings::default())).remove(0);
    let (info, chunks) = read_all(&e57);
    assert_eq!(info.scans.len(), 2);
    assert_eq!(chunks.iter().map(|c| (c.scan, c.len())).collect::<Vec<_>>(), [(0, 2), (1, 1)]);
}

#[test]
fn dropped_attributes_are_left_out() {
    let folder = test_folder("drop");
    let source = folder.join("cloud.xyz");
    write_xyz(&source, &source_points(100, [0.0; 3]));

    let mut settings = Settings::default();
    settings.processing.drop.color = true;
    settings.processing.drop.normal = true;
    let written = outputs(run(&source, Format::Ply, &settings)).remove(0);
    let (info, _) = read_all(&written);
    assert_eq!(info.attributes, Attributes { intensity: true, ..Attributes::NONE });
}

#[test]
fn cancelled_conversion_leaves_no_files() {
    let folder = test_folder("cancel");
    let source = folder.join("cloud.xyz");
    write_xyz(&source, &source_points(1000, [0.0; 3]));

    let progress = Progress::new();
    progress.cancel();
    let job = ConversionJob { input: source.clone(), format: Format::Laz, output_folder: folder.clone() };
    let result = convert(&job, &Settings::default(), &progress, &ClaimedPaths::default());
    assert!(matches!(result, Err(epcc_core::Error::Cancelled)), "{result:?}");
    let files: Vec<_> = fs::read_dir(&folder).unwrap().map(|e| e.unwrap().file_name()).collect();
    assert_eq!(files, ["cloud.xyz"]);
}

#[test]
fn e57_poses_and_images_survive() {
    let folder = test_folder("e57_pose");
    let source = folder.join("posed.e57");
    write_posed_e57(&source);

    // Global coordinates come out with the pose applied: local (1, 0, 0) rotated 90 degrees
    // about z is (0, 1, 0), then shifted by (10, 20, 30).
    let las = outputs(run(&source, Format::Las, &Settings::default())).remove(0);
    let point = read_points(&las)[0].position;
    assert!(
        (point[0] - 10.0).abs() < 1e-3 && (point[1] - 21.0).abs() < 1e-3 && (point[2] - 30.0).abs() < 1e-3,
        "{point:?}"
    );

    let mut keep_images = Settings::default();
    keep_images.export.e57.keep_images = true;
    let copy = outputs(run(&source, Format::E57, &keep_images)).remove(0);
    let reader = e57::E57Reader::from_file(&copy).unwrap();
    let cloud = &reader.pointclouds()[0];
    assert_eq!(cloud.name.as_deref(), Some("Station 1"));
    let pose = cloud.transform.as_ref().expect("pose kept");
    assert!((pose.translation.x - 10.0).abs() < 1e-9);
    assert_eq!(reader.images().len(), 1);
    assert_eq!(reader.images()[0].pointcloud_guid, cloud.guid);

    let without_images = outputs(run(&source, Format::E57, &Settings::default())).remove(0);
    assert_eq!(e57::E57Reader::from_file(&without_images).unwrap().images().len(), 0);
}

fn write_posed_e57(path: &Path) {
    use e57::{Record, RecordDataType, RecordName, RecordValue};
    let mut writer = e57::E57Writer::from_file(path, "{file-guid}").unwrap();
    let double = RecordDataType::Double { min: None, max: None };
    let prototype = [RecordName::CartesianX, RecordName::CartesianY, RecordName::CartesianZ]
        .map(|name| Record { name, data_type: double.clone() })
        .to_vec();
    let mut cloud = writer.add_pointcloud("{cloud-guid}", prototype).unwrap();
    cloud.set_name(Some("Station 1".into()));
    let half = std::f64::consts::FRAC_PI_4;
    cloud.set_transform(Some(e57::Transform {
        rotation: e57::Quaternion { w: half.cos(), x: 0.0, y: 0.0, z: half.sin() },
        translation: e57::Translation { x: 10.0, y: 20.0, z: 30.0 },
    }));
    cloud.add_point(vec![RecordValue::Double(1.0), RecordValue::Double(0.0), RecordValue::Double(0.0)]).unwrap();
    cloud.finalize().unwrap();

    let mut image = writer.add_image("{image-guid}").unwrap();
    image.set_pointcloud_guid("{cloud-guid}");
    let png = [0x89, b'P', b'N', b'G', 0, 0, 0, 0];
    let properties = e57::VisualReferenceImageProperties { width: 1, height: 1 };
    image.add_visual_reference(e57::ImageFormat::Png, &mut png.as_slice(), properties, None).unwrap();
    image.finalize().unwrap();
    writer.finalize().unwrap();
}

/// Points on a millimetre grid, so a COPC round trip can compare them exactly.
fn millimetre_points(count: usize) -> Vec<PointRecord> {
    let mut points = source_points(count, [0.0; 3]);
    for point in &mut points {
        point.position = point.position.map(|value| (value * 1000.0).round() / 1000.0);
    }
    points
}

/// What a COPC round trip must keep: position in millimetres, 8-bit colour and intensity.
fn copc_key(point: &PointRecord) -> ([i64; 3], [u16; 3], i32) {
    let millimetres = point.position.map(|value| (value * 1000.0).round() as i64);
    (millimetres, point.color.map(|c| c / 257), (point.intensity * 999.0).round() as i32)
}

#[test]
fn copc_keeps_every_point() {
    let folder = test_folder("copc_round_trip");
    let source = folder.join("cloud.xyz");
    let points = millimetre_points(300_000);
    write_xyz(&source, &points);

    let written = outputs(run(&source, Format::Copc, &Settings::default()));
    assert_eq!(written, [folder.join("cloud.copc.laz")]);
    assert_eq!(Format::from_path(&written[0]), Some(Format::Copc));
    let mut got: Vec<_> = read_points(&written[0]).iter().map(copc_key).collect();
    let mut want: Vec<_> = points.iter().map(copc_key).collect();
    got.sort_unstable();
    want.sort_unstable();
    assert!(got == want, "the COPC file holds a different set of points");

    // Converting the COPC file again names the output after "cloud", not "cloud.copc".
    let las = outputs(run(&written[0], Format::Las, &Settings::default()));
    assert_eq!(las, [folder.join("cloud.las")]);
    let files: Vec<_> = fs::read_dir(&folder).unwrap().map(|entry| entry.unwrap().file_name()).collect();
    assert_eq!(files.len(), 3, "only the input and the two outputs remain: {files:?}");
}

#[test]
fn copc_files_are_reproducible() {
    let folder = test_folder("copc_reproducible");
    let source = folder.join("cloud.xyz");
    write_xyz(&source, &millimetre_points(120_000));
    let mut overwrite = Settings::default();
    overwrite.general.existing_files = ExistingFiles::Overwrite;
    let first = fs::read(outputs(run(&source, Format::Copc, &overwrite)).remove(0)).unwrap();
    let second = fs::read(outputs(run(&source, Format::Copc, &overwrite)).remove(0)).unwrap();
    assert!(first == second);
}

#[test]
fn copc_layout_follows_the_specification() {
    let folder = test_folder("copc_layout");
    let source = folder.join("cloud.xyz");
    write_xyz(&source, &millimetre_points(300_000));
    let file = fs::read(outputs(run(&source, Format::Copc, &Settings::default())).remove(0)).unwrap();
    CopcFile::parse(&file).check_nodes(&file, 300_000);
}

/// Finds, for every point of `got`, a different point of `want` within `tolerance`.
fn match_points(got: &[PointRecord], want: &[PointRecord], tolerance: f64) -> Vec<usize> {
    let cell = |position: [f64; 3]| position.map(|value| (value / (2.0 * tolerance)).floor() as i64);
    let mut grid: std::collections::HashMap<[i64; 3], Vec<usize>> = std::collections::HashMap::new();
    for (index, point) in want.iter().enumerate() {
        grid.entry(cell(point.position)).or_default().push(index);
    }
    let mut used = vec![false; want.len()];
    got.iter()
        .map(|point| {
            let [x, y, z] = cell(point.position);
            let near = (-1..=1)
                .flat_map(|dx| (-1..=1).flat_map(move |dy| (-1..=1).map(move |dz| [x + dx, y + dy, z + dz])))
                .flat_map(|key| grid.get(&key).into_iter().flatten().copied())
                .find(|&index| {
                    !used[index]
                        && (0..3).all(|axis| (want[index].position[axis] - point.position[axis]).abs() <= tolerance)
                });
            let index = near.unwrap_or_else(|| panic!("no source point near {:?}", point.position));
            used[index] = true;
            index
        })
        .collect()
}

#[test]
fn rcs_round_trips() {
    let folder = test_folder("rcs_round_trip");
    let source = folder.join("cloud.xyz");
    let points = source_points(50_000, [500_000.0, 5_400_000.0, 250.0]);
    write_xyz(&source, &points);

    let written = outputs(run(&source, Format::Rcs, &Settings::default()));
    assert_eq!(written, [folder.join("cloud.rcs")]);
    let (info, _) = read_all(&written[0]);
    assert_eq!(info.attributes, Attributes { intensity: true, color: true, normal: true, ..Attributes::NONE });
    let got = read_points(&written[0]);
    assert_eq!(got.len(), points.len(), "no two points share a thinning cell");

    // Positions are rounded to 1 mm; the text file already rounded them to 0.1 mm.
    let matched = match_points(&got, &points, 0.0006);
    for (point, &index) in got.iter().zip(&matched) {
        let want = &points[index];
        assert_eq!(point.color.map(|c| c / 257), want.color.map(|c| c / 257));
        // Intensity is stored as 8 bits of its range in the file, which here is 0..1.
        let stored = (f64::from(want.intensity) * 255.0).floor() / 255.0;
        assert!((f64::from(point.intensity) - stored).abs() < 0.0045, "{} vs {}", point.intensity, want.intensity);
        let dot: f32 = (0..3).map(|axis| point.normal[axis] * want.normal[axis]).sum();
        assert!(dot > 0.999, "normal {:?} vs {:?}", point.normal, want.normal);
    }

    // RCS input converts like any other.
    let las = outputs(run(&written[0], Format::Las, &Settings::default())).remove(0);
    assert_eq!(read_points(&las).len(), points.len());
}

/// A tilted plane on a 2 cm grid, as an XYZ file without normals.
fn write_plane(path: &Path) {
    let mut text = String::from(
        "x y z
",
    );
    for i in 0..100 {
        for j in 0..100 {
            let (x, y) = (i as f64 * 0.02, j as f64 * 0.02);
            let _ = writeln!(text, "{x:.4} {y:.4} {:.4}", 0.5 * x);
        }
    }
    fs::write(path, text).unwrap();
}

#[test]
fn rcs_normals_are_estimated_unless_switched_off() {
    let folder = test_folder("rcs_normals");
    let source = folder.join("plane.xyz");
    write_plane(&source);
    let mut settings = Settings::default();
    settings.general.existing_files = ExistingFiles::Overwrite;

    let estimated = read_points(&outputs(run(&source, Format::Rcs, &settings)).remove(0));
    let plane_normal = [-0.5f32, 0.0, 1.0].map(|value| value / 1.25f32.sqrt());
    let along_plane_normal = estimated
        .iter()
        .filter(|point| (0..3).map(|axis| point.normal[axis] * plane_normal[axis]).sum::<f32>().abs() > 0.99)
        .count();
    assert!(along_plane_normal > estimated.len() * 9 / 10, "{along_plane_normal} of {}", estimated.len());

    settings.export.rcs.estimate_normals = false;
    let plain = read_points(&outputs(run(&source, Format::Rcs, &settings)).remove(0));
    assert_eq!(plain.len(), estimated.len());
    assert!(plain.iter().all(|point| point.normal == [0.0; 3]));
}

#[test]
fn rcs_files_are_reproducible() {
    let folder = test_folder("rcs_reproducible");
    let source = folder.join("cloud.xyz");
    write_xyz(&source, &source_points(20_000, [0.0; 3]));
    let mut overwrite = Settings::default();
    overwrite.general.existing_files = ExistingFiles::Overwrite;
    let first = fs::read(outputs(run(&source, Format::Rcs, &overwrite)).remove(0)).unwrap();
    let second = fs::read(outputs(run(&source, Format::Rcs, &overwrite)).remove(0)).unwrap();
    assert!(first == second);
}

/// A COPC file read the way a streaming viewer reads it: header, info VLR and hierarchy first,
/// then each node's LAZ chunk on its own.
struct CopcFile {
    point_count: u64,
    scale: [f64; 3],
    offset: [f64; 3],
    center: [f64; 3],
    halfsize: f64,
    laz_items: Vec<laz::LazItem>,
    nodes: Vec<HierarchyEntry>,
}

struct HierarchyEntry {
    key: [i32; 4],
    offset: usize,
    size: usize,
    point_count: usize,
}

fn u16_at(bytes: &[u8], at: usize) -> u16 {
    u16::from_le_bytes(bytes[at..at + 2].try_into().unwrap())
}

fn u32_at(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap())
}

fn i32_at(bytes: &[u8], at: usize) -> i32 {
    i32::from_le_bytes(bytes[at..at + 4].try_into().unwrap())
}

fn u64_at(bytes: &[u8], at: usize) -> u64 {
    u64::from_le_bytes(bytes[at..at + 8].try_into().unwrap())
}

fn f64_at(bytes: &[u8], at: usize) -> f64 {
    f64::from_le_bytes(bytes[at..at + 8].try_into().unwrap())
}

fn text_at(bytes: &[u8], at: usize, width: usize) -> String {
    String::from_utf8_lossy(&bytes[at..at + width]).trim_end_matches('\0').to_string()
}

impl CopcFile {
    fn parse(file: &[u8]) -> CopcFile {
        assert_eq!(&file[0..4], b"LASF");
        assert_eq!(&file[24..26], &[1, 4], "LAS 1.4");
        assert_eq!(u16_at(file, 94), 375, "header size");
        let format = file[104];
        assert_eq!(format & 0x80, 0x80, "compressed point format");
        assert!([6, 7, 8].contains(&(format & 0x3f)), "point format {}", format & 0x3f);
        let point_data_offset = u32_at(file, 96) as usize;

        // The COPC info VLR comes first, right after the header.
        assert_eq!(text_at(file, 377, 16), "copc");
        assert_eq!(u16_at(file, 393), 1, "info record id");
        assert_eq!(u16_at(file, 395), 160, "info size");
        let info = 375 + 54;
        let hierarchy_offset = u64_at(file, info + 40) as usize;
        let hierarchy_size = u64_at(file, info + 48) as usize;

        let mut laz_items = None;
        let mut at = 375;
        for _ in 0..u32_at(file, 100) {
            let length = u16_at(file, at + 20) as usize;
            if text_at(file, at + 2, 16) == "laszip encoded" {
                let vlr = laz::LazVlr::from_buffer(&file[at + 54..at + 54 + length]).unwrap();
                assert!(vlr.uses_variable_size_chunks());
                laz_items = Some(vlr.items().clone());
            }
            at += 54 + length;
        }
        assert_eq!(at, point_data_offset, "the VLRs end where the points start");

        let evlr = u64_at(file, 235) as usize;
        assert_eq!(u32_at(file, 243), 1, "one extended VLR");
        assert_eq!(text_at(file, evlr + 2, 16), "copc");
        assert_eq!(u16_at(file, evlr + 18), 1000, "hierarchy record id");
        assert_eq!(evlr + 60, hierarchy_offset);
        assert_eq!(u64_at(file, evlr + 20) as usize, hierarchy_size);
        let chunk_table = u64_at(file, point_data_offset) as usize;
        assert!(point_data_offset < chunk_table && chunk_table < evlr, "the chunk table lies after the points");

        let (entries, _) = file[hierarchy_offset..hierarchy_offset + hierarchy_size].as_chunks::<32>();
        let nodes = entries
            .iter()
            .map(|entry| HierarchyEntry {
                key: std::array::from_fn(|i| i32_at(entry, i * 4)),
                offset: u64_at(entry, 16) as usize,
                size: u32_at(entry, 24) as usize,
                point_count: u32_at(entry, 28) as usize,
            })
            .collect();
        CopcFile {
            point_count: u64_at(file, 247),
            scale: [f64_at(file, 131), f64_at(file, 139), f64_at(file, 147)],
            offset: [f64_at(file, 155), f64_at(file, 163), f64_at(file, 171)],
            center: [f64_at(file, info), f64_at(file, info + 8), f64_at(file, info + 16)],
            halfsize: f64_at(file, info + 24),
            laz_items: laz_items.expect("a LAZ VLR"),
            nodes,
        }
    }

    fn check_nodes(&self, file: &[u8], expected_points: u64) {
        assert_eq!(self.point_count, expected_points);
        let keys: std::collections::HashSet<[i32; 4]> = self.nodes.iter().map(|node| node.key).collect();
        assert_eq!(keys.len(), self.nodes.len(), "each node once");
        assert!(keys.contains(&[0, 0, 0, 0]), "a root node");
        for node in &self.nodes {
            let [level, x, y, z] = node.key;
            if level > 0 {
                assert!(keys.contains(&[level - 1, x / 2, y / 2, z / 2]), "parent of {:?}", node.key);
            }
            if node.point_count > 0 {
                self.check_chunk(&file[node.offset..node.offset + node.size], node);
            }
        }
        let total: usize = self.nodes.iter().map(|node| node.point_count).sum();
        assert_eq!(total as u64, expected_points, "the nodes hold every point");
    }

    /// Decompresses the node's chunk on its own and checks that its points lie in the node.
    fn check_chunk(&self, chunk: &[u8], node: &HierarchyEntry) {
        use laz::record::{LayeredPointRecordDecompressor, RecordDecompressor};

        let [level, x, y, z] = node.key;
        let mut decompressor = LayeredPointRecordDecompressor::new(std::io::Cursor::new(chunk));
        decompressor.set_fields_from(&self.laz_items).unwrap();
        let mut record = vec![0u8; decompressor.record_size()];
        let node_size = 2.0 * self.halfsize / f64::from(1u32 << level);
        let cell = [x, y, z];
        for _ in 0..node.point_count {
            decompressor.decompress_next(&mut record).unwrap();
            for (axis, &cell) in cell.iter().enumerate() {
                let value = self.offset[axis] + f64::from(i32_at(&record, axis * 4)) * self.scale[axis];
                let min = self.center[axis] - self.halfsize + f64::from(cell) * node_size;
                let slack = self.scale[axis];
                assert!(value >= min - slack && value <= min + node_size + slack, "point outside {:?}", node.key);
            }
        }
    }
}
