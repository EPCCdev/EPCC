//! End-to-end tests for E57 normals and structured (gridded) scans, with generated files.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use e57::{E57Reader, E57Writer, Extension, Quaternion, Record, RecordDataType, RecordName, RecordValue, Transform};
use epcc_core::model::{PointRecord, ScanGrid};
use epcc_core::{
    ClaimedPaths, CloudInfo, ConversionJob, ConversionReport, Format, Outcome, Progress, Settings, convert,
};

const NORMALS_URI: &str = "http://www.libe57.org/E57_EXT_surface_normals.txt";

fn test_folder(name: &str) -> PathBuf {
    let folder = Path::new(env!("CARGO_TARGET_TMPDIR")).join("e57_extras").join(name);
    let _ = fs::remove_dir_all(&folder);
    fs::create_dir_all(&folder).unwrap();
    folder
}

fn convert_to(input: &Path, format: Format, settings: &Settings) -> ConversionReport {
    let job =
        ConversionJob { input: input.to_path_buf(), format, output_folder: input.parent().unwrap().to_path_buf() };
    match convert(&job, settings, &Progress::new(), &ClaimedPaths::default()).unwrap() {
        Outcome::Converted(report) => report,
        Outcome::Skipped(path) => panic!("unexpectedly skipped {}", path.display()),
    }
}

fn output(input: &Path, format: Format) -> PathBuf {
    convert_to(input, format, &Settings::default()).outputs.remove(0)
}

/// Reads a file through EPCC: its description and its points, scan by scan.
fn read_epcc(path: &Path) -> (CloudInfo, Vec<(usize, PointRecord)>) {
    let reader = Format::from_path(path).unwrap().open(path, &Default::default()).unwrap();
    let info = reader.info().clone();
    let mut points = Vec::new();
    let mut emit = |chunk: epcc_core::PointChunk| {
        points.extend((0..chunk.len()).map(|i| (chunk.scan, chunk.record(i))));
        Ok(())
    };
    reader.read(&mut emit, &Progress::new()).unwrap();
    (info, points)
}

fn unit(v: [f64; 3]) -> [f64; 3] {
    let length = v.iter().map(|c| c * c).sum::<f64>().sqrt();
    v.map(|c| c / length)
}

fn rotate(q: &Quaternion, v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q.w, q.x, q.y, q.z);
    let t = [2.0 * (y * v[2] - z * v[1]), 2.0 * (z * v[0] - x * v[2]), 2.0 * (x * v[1] - y * v[0])];
    [
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    ]
}

fn assert_near(got: [f64; 3], want: [f64; 3], tolerance: f64, label: &str) {
    for axis in 0..3 {
        assert!((got[axis] - want[axis]).abs() <= tolerance, "{label}: {got:?} vs {want:?}");
    }
}

/// One generated scan, in scanner coordinates.
struct TestScan {
    name: &'static str,
    /// Rows and columns, or `None` for an unstructured scan.
    grid: Option<(i64, i64)>,
    pose: Option<Transform>,
    /// Number of points of an unstructured scan.
    points: i64,
}

impl TestScan {
    /// The cells of the scan: row, column (both -1 without grid) and whether the cell has a return.
    fn cells(&self) -> Vec<(i64, i64, bool)> {
        match self.grid {
            Some((rows, columns)) => (0..rows)
                .flat_map(|row| (0..columns).map(move |column| (row, column)))
                // The last row has no returns at all, so the grid is larger than its points.
                .map(|(row, column)| (row, column, row != rows - 1 && (row + column) % 7 != 3))
                .collect(),
            None => (0..self.points).map(|index| (-1, index, true)).collect(),
        }
    }

    fn position(&self, row: i64, column: i64) -> [f64; 3] {
        [1.0 + row as f64 * 0.1, column as f64 * 0.1, (row + column) as f64 * 0.01]
    }

    fn normal(&self, row: i64, column: i64) -> [f64; 3] {
        let angle = (row * 31 + column) as f64 * 0.1;
        unit([angle.cos(), angle.sin(), 0.75])
    }

    /// Expected global position and normal of each valid cell, keyed by row and column.
    fn expected_global(&self) -> HashMap<(i64, i64), ([f64; 3], [f64; 3])> {
        let identity = Transform::default();
        let pose = self.pose.as_ref().unwrap_or(&identity);
        let t = &pose.translation;
        self.cells()
            .into_iter()
            .filter(|cell| cell.2)
            .map(|(row, column, _)| {
                let p = rotate(&pose.rotation, self.position(row, column));
                let n = rotate(&pose.rotation, self.normal(row, column));
                ((row, column), ([p[0] + t.x, p[1] + t.y, p[2] + t.z], n))
            })
            .collect()
    }
}

fn pose(angle: f64, axis: [f64; 3], translation: [f64; 3]) -> Option<Transform> {
    let [x, y, z] = unit(axis).map(|c| c * (angle / 2.0).sin());
    Some(Transform {
        rotation: Quaternion { w: (angle / 2.0).cos(), x, y, z },
        translation: e57::Translation { x: translation[0], y: translation[1], z: translation[2] },
    })
}

/// Writes the scans as a structured E57 file the way scanners do: every grid cell has a record,
/// cells without a return are marked with `cartesianInvalidState`.
fn write_scans(path: &Path, scans: &[TestScan]) {
    let mut writer = E57Writer::from_file(path, "{00000000-0000-4000-8000-000000000001}").unwrap();
    writer.register_extension(Extension::new("nor", NORMALS_URI)).unwrap();
    for (index, scan) in scans.iter().enumerate() {
        let double = RecordDataType::Double { min: None, max: None };
        let mut prototype = [RecordName::CartesianX, RecordName::CartesianY, RecordName::CartesianZ]
            .map(|name| Record { name, data_type: double.clone() })
            .to_vec();
        prototype.push(Record::CARTESIAN_INVALID_STATE);
        for name in ["normalX", "normalY", "normalZ"] {
            prototype.push(Record {
                name: RecordName::Unknown { namespace: "nor".into(), name: name.into() },
                data_type: RecordDataType::Single { min: Some(-1.0), max: Some(1.0) },
            });
        }
        if let Some((rows, columns)) = scan.grid {
            prototype.push(Record {
                name: RecordName::RowIndex,
                data_type: RecordDataType::Integer { min: 0, max: rows - 1 },
            });
            let data_type = RecordDataType::Integer { min: 0, max: columns - 1 };
            prototype.push(Record { name: RecordName::ColumnIndex, data_type });
        }
        let guid = format!("{{00000000-0000-4000-8000-0000000001{index:02}}}");
        let mut cloud = writer.add_pointcloud(&guid, prototype).unwrap();
        cloud.set_name(Some(scan.name.into()));
        cloud.set_transform(scan.pose.clone());
        for (row, column, valid) in scan.cells() {
            let position = if valid { scan.position(row, column) } else { [0.0; 3] };
            let mut values: Vec<RecordValue> = position.into_iter().map(RecordValue::Double).collect();
            values.push(RecordValue::Integer(if valid { 0 } else { 2 }));
            values.extend(scan.normal(row, column).map(|c| RecordValue::Single(c as f32)));
            if scan.grid.is_some() {
                values.extend([RecordValue::Integer(row), RecordValue::Integer(column)]);
            }
            cloud.add_point(values).unwrap();
        }
        cloud.finalize().unwrap();
    }
    writer.finalize().unwrap();
}

/// Checks a written E57 file directly with the `e57` crate: grids, index bounds, and each point's
/// local position and normal against the generated scan.
fn check_e57_file(path: &Path, scans: &[TestScan]) {
    let mut reader = E57Reader::from_file(path).unwrap();
    assert!(reader.extensions().iter().any(|e| e.namespace == "nor" && e.url == NORMALS_URI), "extension declared");
    let clouds = reader.pointclouds();
    assert_eq!(clouds.len(), scans.len());
    for (cloud, scan) in clouds.iter().zip(scans) {
        assert_eq!(cloud.name.as_deref(), Some(scan.name));
        let field = |name: &RecordName| cloud.prototype.iter().position(|r| &r.name == name);
        let normal = |name: &str| field(&RecordName::Unknown { namespace: "nor".into(), name: name.into() });
        let normals = [normal("normalX"), normal("normalY"), normal("normalZ")].map(|i| i.expect("normal field"));
        let xyz = [RecordName::CartesianX, RecordName::CartesianY, RecordName::CartesianZ].map(|n| field(&n).unwrap());
        let (row_field, column_field) = (field(&RecordName::RowIndex), field(&RecordName::ColumnIndex));
        match scan.grid {
            Some((rows, columns)) => {
                assert!(cloud.has_row_column(), "{}: grid fields", scan.name);
                let bounds = cloud.index_bounds.as_ref().expect("index bounds");
                assert_eq!(
                    (bounds.row_min, bounds.row_max, bounds.column_min, bounds.column_max),
                    (Some(0), Some(rows - 1), Some(0), Some(columns - 1)),
                    "{}: index bounds",
                    scan.name
                );
                let range = |i: usize| match cloud.prototype[i].data_type {
                    RecordDataType::Integer { min, max } => (min, max),
                    ref other => panic!("{other:?}"),
                };
                assert_eq!(range(row_field.unwrap()), (0, rows - 1));
                assert_eq!(range(column_field.unwrap()), (0, columns - 1));
            }
            None => assert!(row_field.is_none() && column_field.is_none(), "{}: no grid fields", scan.name),
        }

        let expected: HashMap<(i64, i64), ([f64; 3], [f64; 3])> = scan
            .cells()
            .into_iter()
            .filter(|cell| cell.2)
            .map(|(row, column, _)| ((row, column), (scan.position(row, column), scan.normal(row, column))))
            .collect();
        assert_eq!(cloud.records as usize, expected.len(), "{}: one record per return", scan.name);
        let prototype = cloud.prototype.clone();
        let value = |values: &[RecordValue], i: usize| values[i].to_f64(&prototype[i].data_type).unwrap();
        for (index, values) in reader.pointcloud_raw(cloud).unwrap().enumerate() {
            let values = values.unwrap();
            let key = match (row_field, column_field) {
                (Some(r), Some(c)) => (value(&values, r) as i64, value(&values, c) as i64),
                _ => (-1, index as i64),
            };
            let (position, normal) = expected[&key];
            assert_near(xyz.map(|i| value(&values, i)), position, 1e-9, "local position");
            assert_near(normals.map(|i| value(&values, i)), normal, 1e-6, "local normal");
        }
    }
}

/// Checks what EPCC reads from a written file: per-scan grids, global positions and normals.
fn check_epcc_read(path: &Path, scans: &[TestScan]) {
    let (info, points) = read_epcc(path);
    assert!(info.attributes.normal);
    for (index, scan) in scans.iter().enumerate() {
        let grid =
            scan.grid.map(|(rows, columns)| ScanGrid { rows: [0, rows as i32 - 1], columns: [0, columns as i32 - 1] });
        assert_eq!(info.scans[index].grid, grid, "{}: grid", scan.name);
        let expected = scan.expected_global();
        let mine: Vec<_> = points.iter().filter(|(s, _)| *s == index).map(|(_, p)| p).collect();
        assert_eq!(mine.len(), expected.len(), "{}: points", scan.name);
        for (order, point) in mine.into_iter().enumerate() {
            let key = if scan.grid.is_some() {
                (i64::from(point.grid[0]), i64::from(point.grid[1]))
            } else {
                (-1, order as i64)
            };
            let (position, normal) = expected[&key];
            assert_near(point.position, position, 1e-9, "global position");
            assert_near(point.normal.map(f64::from), normal, 1e-6, "global normal");
        }
    }
}

fn one_gridded_scan() -> Vec<TestScan> {
    vec![TestScan {
        name: "Station 1",
        grid: Some((6, 8)),
        pose: pose(0.7, [0.2, 0.3, 1.0], [10.0, 20.0, 3.0]),
        points: 0,
    }]
}

fn three_scans() -> Vec<TestScan> {
    vec![
        TestScan { name: "North", grid: Some((6, 8)), pose: pose(0.7, [0.2, 0.3, 1.0], [10.0, 20.0, 3.0]), points: 0 },
        TestScan { name: "South", grid: Some((4, 11)), pose: pose(-2.1, [0.0, 0.0, 1.0], [-5.0, 2.0, 0.5]), points: 0 },
        TestScan { name: "Loose", grid: None, pose: None, points: 25 },
    ]
}

#[test]
fn gridded_scan_round_trips() {
    let folder = test_folder("gridded");
    let source = folder.join("station.e57");
    let scans = one_gridded_scan();
    write_scans(&source, &scans);
    check_epcc_read(&source, &scans);

    let copy = output(&source, Format::E57);
    check_e57_file(&copy, &scans);
    check_epcc_read(&copy, &scans);

    // A second pass changes nothing.
    let again = output(&copy, Format::E57);
    check_e57_file(&again, &scans);
}

#[test]
fn multi_scan_files_keep_each_grid() {
    let folder = test_folder("multi");
    let source = folder.join("site.e57");
    let scans = three_scans();
    write_scans(&source, &scans);

    let copy = output(&source, Format::E57);
    check_e57_file(&copy, &scans);
    check_epcc_read(&copy, &scans);
}

#[test]
fn thinned_grids_keep_row_and_column() {
    let folder = test_folder("thinned");
    let source = folder.join("station.e57");
    let scans = one_gridded_scan();
    write_scans(&source, &scans);

    let mut settings = Settings::default();
    settings.processing.every_nth = Some(3);
    let thinned = convert_to(&source, Format::E57, &settings).outputs.remove(0);
    let reader = E57Reader::from_file(&thinned).unwrap();
    let bounds = reader.pointclouds()[0].index_bounds.clone().unwrap();
    assert_eq!((bounds.row_max, bounds.column_max), (Some(5), Some(7)), "the grid keeps its size");

    let (info, points) = read_epcc(&thinned);
    let expected = scans[0].expected_global();
    assert_eq!(points.len(), expected.len().div_ceil(3));
    assert!(info.scans[0].grid.is_some());
    for (_, point) in points {
        let (position, _) = expected[&(i64::from(point.grid[0]), i64::from(point.grid[1]))];
        assert_near(point.position, position, 1e-9, "thinned point at its cell");
    }
}

#[test]
fn other_formats_ignore_the_grid() {
    let folder = test_folder("ignore");
    let source = folder.join("station.e57");
    let scans = one_gridded_scan();
    write_scans(&source, &scans);

    let report = convert_to(&source, Format::Ply, &Settings::default());
    assert!(report.warnings.is_empty(), "{:?}", report.warnings);
    let (info, points) = read_epcc(&report.outputs[0]);
    assert_eq!(info.scans[0].grid, None);
    assert_eq!(points.len(), scans[0].expected_global().len());

    // Back to E57: the normals return, the grid is gone.
    let back = output(&report.outputs[0], Format::E57);
    let reader = E57Reader::from_file(&back).unwrap();
    assert!(!reader.pointclouds()[0].has_row_column());
    assert!(read_epcc(&back).0.attributes.normal);
}

/// Points with unit normals that point in many directions.
fn points_with_normals(count: usize) -> Vec<PointRecord> {
    (0..count)
        .map(|i| {
            let t = i as f64;
            let normal = unit([(t * 0.37).cos(), (t * 0.37).sin(), (t * 0.11).cos()]).map(|c| c as f32);
            PointRecord {
                position: [(t * 0.137) % 50.0, (t * 0.071) % 30.0, (t * 0.013) % 10.0],
                intensity: (i % 1000) as f32 / 999.0,
                normal,
                ..PointRecord::default()
            }
        })
        .collect()
}

fn write_xyz(path: &Path, points: &[PointRecord]) {
    let mut text = String::from("x y z i nx ny nz\n");
    for p in points {
        let [x, y, z] = p.position;
        let [nx, ny, nz] = p.normal;
        let _ = writeln!(text, "{x:.4} {y:.4} {z:.4} {:.4} {nx} {ny} {nz}", p.intensity);
    }
    fs::write(path, text).unwrap();
}

fn assert_same_normals(path: &Path, want: &[PointRecord], label: &str) {
    let (info, points) = read_epcc(path);
    assert!(info.attributes.normal, "{label}: normals kept");
    assert_eq!(points.len(), want.len(), "{label}: point count");
    for ((_, got), want) in points.iter().zip(want) {
        assert_near(got.position, want.position, 0.0011, label);
        assert_near(got.normal.map(f64::from), want.normal.map(f64::from), 1e-6, label);
    }
}

#[test]
fn normals_round_trip_through_e57() {
    let folder = test_folder("normals");
    let source = folder.join("cloud.xyz");
    let points = points_with_normals(5_000);
    write_xyz(&source, &points);

    let e57 = output(&source, Format::E57);
    assert_same_normals(&e57, &points, "XYZ to E57");
    let reader = E57Reader::from_file(&e57).unwrap();
    assert!(reader.extensions().iter().any(|e| e.url == NORMALS_URI));

    let copy = output(&e57, Format::E57);
    assert_same_normals(&copy, &points, "E57 to E57");

    let ply = output(&source, Format::Ply);
    let ply_e57 = output(&ply, Format::E57);
    assert_same_normals(&ply_e57, &points, "PLY to E57");
    let back = output(&ply_e57, Format::Ply);
    assert_same_normals(&back, &points, "PLY to E57 to PLY");
}

/// LAS to E57 to LAS keeps normals, once LAS stores them (as extra bytes). Until then there is
/// nothing to carry and the test has nothing to check.
#[test]
fn las_normals_round_trip_through_e57() {
    if !Format::Las.stored_attributes().normal {
        return;
    }
    let folder = test_folder("las_normals");
    let source = folder.join("cloud.xyz");
    let points = points_with_normals(5_000);
    write_xyz(&source, &points);

    let las = output(&source, Format::Las);
    assert_same_normals(&las, &points, "XYZ to LAS");
    let e57 = output(&las, Format::E57);
    assert_same_normals(&e57, &points, "LAS to E57");
    let back = output(&e57, Format::Las);
    assert_same_normals(&back, &points, "LAS to E57 to LAS");
}
