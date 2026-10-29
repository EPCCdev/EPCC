//! End-to-end tests for the LAS point fields beyond position, colour, intensity, class and time:
//! return numbers, the lidar details and extra bytes.

use std::fs;
use std::path::{Path, PathBuf};

use epcc_core::model::{ExtraField, LidarDetails, Returns, ScalarType};
use epcc_core::settings::LasVersion;
use epcc_core::{
    ClaimedPaths, CloudInfo, ConversionJob, ConversionReport, Format, Outcome, PointChunk, Progress, Settings, convert,
};
use las::point::{Classification, ScanDirection};

const POINTS: usize = 30_000;
/// The extra values of the generated files: three described fields and four undescribed bytes.
const EXTRA_BYTES: u16 = 2 + 4 + 4 + 4;

fn test_folder(name: &str) -> PathBuf {
    let folder = Path::new(env!("CARGO_TARGET_TMPDIR")).join(format!("lidar_{name}"));
    let _ = fs::remove_dir_all(&folder);
    fs::create_dir_all(&folder).unwrap();
    folder
}

fn run(input: &Path, format: Format, settings: &Settings) -> ConversionReport {
    let job =
        ConversionJob { input: input.to_path_buf(), format, output_folder: input.parent().unwrap().to_path_buf() };
    match convert(&job, settings, &Progress::new(), &ClaimedPaths::default()).unwrap() {
        Outcome::Converted(report) => report,
        Outcome::Skipped(path) => panic!("unexpectedly skipped {}", path.display()),
    }
}

fn convert_to(input: &Path, format: Format, settings: &Settings) -> PathBuf {
    run(input, format, settings).outputs.remove(0)
}

fn las_1_4() -> Settings {
    let mut settings = Settings::default();
    settings.export.las.version = LasVersion::V1_4;
    settings
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

/// A LAS 1.4 file with every point field set to varying values, and extra bytes.
fn write_lidar_las(path: &Path) {
    let mut builder = las::Builder::from((1, 4));
    builder.point_format = las::point::Format::new(7).unwrap();
    builder.point_format.extra_bytes = EXTRA_BYTES;
    let transform = las::Transform { scale: 0.001, offset: 0.0 };
    builder.transforms = las::Vector { x: transform, y: transform, z: transform };
    let mut data = descriptor(3, 0, "amplitude", 0.0, 0.0, "Echo amplitude");
    data.extend(descriptor(9, 0, "deviation", 0.0, 0.0, "Pulse shape deviation"));
    data.extend(descriptor(6, 0b11000, "range", 0.001, 10.0, "Range in metres"));
    builder.vlrs.push(las::Vlr { user_id: "LASF_Spec".into(), record_id: 4, description: "Extra Bytes".into(), data });
    let header = builder.into_header().unwrap();
    let mut writer = las::Writer::from_path(path, header).unwrap();
    let classes = [1, 2, 3, 6, 9, 17];
    for index in 0..POINTS {
        let count = (index % 7) as u8 + 1;
        let point = las::Point {
            x: (index % 211) as f64 * 0.01,
            y: (index % 97) as f64 * 0.013,
            z: (index % 13) as f64 * 0.1,
            intensity: (index * 37 % 65536) as u16,
            return_number: (index % count as usize) as u8 + 1,
            number_of_returns: count,
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

/// The point fields that a LAS round trip keeps exactly, in time order.
#[derive(Clone, Debug, PartialEq)]
struct Row {
    millimetres: [i64; 3],
    time: f64,
    classification: u8,
    returns: Returns,
    lidar: LidarDetails,
    extra: Vec<u8>,
}

fn rows(chunk: &PointChunk) -> Vec<Row> {
    let mut rows: Vec<Row> = (0..chunk.len())
        .map(|index| {
            let point = chunk.record(index);
            Row {
                millimetres: point.position.map(|value| (value * 1000.0).round() as i64),
                time: point.time,
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

#[test]
fn las_point_fields_are_read() {
    let folder = test_folder("read");
    let source = folder.join("lidar.las");
    write_lidar_las(&source);
    let (info, chunk) = read_all(&source);
    assert!(info.attributes.returns && info.attributes.lidar);
    assert_eq!(info.attributes.extra_bytes, EXTRA_BYTES);
    assert_eq!(
        field_summary(&info.extra_fields),
        [
            ("amplitude".into(), "Echo amplitude".into(), 2, Some(ScalarType::U16), 1.0, 0.0),
            ("deviation".into(), "Pulse shape deviation".into(), 4, Some(ScalarType::F32), 1.0, 0.0),
            ("range".into(), "Range in metres".into(), 4, Some(ScalarType::I32), 0.001, 10.0),
            ("extra bytes".into(), "Bytes without a description".into(), 4, None, 1.0, 0.0),
        ]
    );

    let index = 3 * 17; // synthetic and withheld, scanned right to left, no other flag
    let point = chunk.record(index);
    let count = (index % 7) as u8 + 1;
    assert_eq!(point.returns, Returns { number: (index % count as usize) as u8 + 1, count });
    assert_eq!(point.lidar.user_data, (index % 256) as u8);
    assert_eq!(point.lidar.point_source, (index / 1000) as u16 + 100);
    assert_eq!(point.lidar.scanner_channel(), (index % 4) as u8);
    assert!((point.lidar.scan_angle - ((index % 61) as f32 - 30.0)).abs() <= 0.006);
    let flags = point.lidar.flags;
    assert_eq!(flags & LidarDetails::SCAN_DIRECTION, 0, "odd points scan right to left");
    assert_eq!(flags & LidarDetails::CLASS_FLAGS, LidarDetails::SYNTHETIC | LidarDetails::WITHHELD);
    assert_eq!(chunk.extra.as_ref().unwrap().get(index), extra_bytes_of(index));
    let range = &info.extra_fields[2];
    let at = 6;
    let value = range.value(&chunk.extra.as_ref().unwrap().get(index)[at..]).unwrap();
    assert!((value - ((index as i32 * 3 - 5000) as f64 * 0.001 + 10.0)).abs() < 1e-9);
}

#[test]
fn las_laz_and_copc_keep_every_point_field() {
    let folder = test_folder("las_laz_copc");
    let source = folder.join("lidar.las");
    write_lidar_las(&source);
    let (source_info, source_chunk) = read_all(&source);
    let want = rows(&source_chunk);
    let settings = las_1_4();

    // LAS to LAS, then LAS to LAZ to LAS.
    let las = convert_to(&source, Format::Las, &settings);
    let laz = convert_to(&las, Format::Laz, &settings);
    let back = convert_to(&laz, Format::Las, &settings);
    for path in [&las, &laz, &back] {
        let (info, chunk) = read_all(path);
        assert_eq!(info.extra_fields[..3], source_info.extra_fields[..3], "descriptors are copied as they were");
        assert_eq!(info.attributes, source_info.attributes, "{}", path.display());
        assert_eq!(field_summary(&info.extra_fields), field_summary(&source_info.extra_fields), "{}", path.display());
        assert!(rows(&chunk) == want, "{} holds different point fields", path.display());
    }

    // COPC sorts the points; compared in time order they are the same.
    let copc = convert_to(&source, Format::Copc, &Settings::default());
    let (info, chunk) = read_all(&copc);
    assert_eq!(field_summary(&info.extra_fields), field_summary(&source_info.extra_fields));
    assert!(rows(&chunk) == want, "the COPC file holds different point fields");
    let file = fs::read(&copc).unwrap();
    let by_return: Vec<u64> =
        (0..15).map(|i| u64::from_le_bytes(file[255 + 8 * i..263 + 8 * i].try_into().unwrap())).collect();
    let mut expected = vec![0u64; 15];
    for row in &want {
        expected[usize::from(row.returns.number) - 1] += 1;
    }
    assert_eq!(by_return, expected, "points by return in the COPC header");
}

#[test]
fn las_1_2_keeps_what_it_can_and_says_what_it_cannot() {
    let folder = test_folder("las_1_2");
    let source = folder.join("lidar.las");
    write_lidar_las(&source);
    let (_, source_chunk) = read_all(&source);
    let want = rows(&source_chunk);
    let mut settings = Settings::default();
    settings.export.las.version = LasVersion::V1_2;
    let report = run(&source, Format::Laz, &settings);
    assert!(report.warnings.iter().any(|w| w.contains("up to 5 returns")), "{:?}", report.warnings);
    assert!(report.warnings.iter().any(|w| w.contains("scanner channels")), "{:?}", report.warnings);

    let (info, chunk) = read_all(&report.outputs[0]);
    assert_eq!(info.attributes.extra_bytes, EXTRA_BYTES);
    for (got, want) in rows(&chunk).iter().zip(&want) {
        assert_eq!(got.returns.number, want.returns.number.min(5));
        assert_eq!(got.returns.count, want.returns.count.min(5));
        assert_eq!(got.lidar.user_data, want.lidar.user_data);
        assert_eq!(got.lidar.point_source, want.lidar.point_source);
        assert_eq!(got.lidar.scan_angle, want.lidar.scan_angle.round());
        let kept = LidarDetails::SYNTHETIC
            | LidarDetails::KEY_POINT
            | LidarDetails::WITHHELD
            | LidarDetails::SCAN_DIRECTION
            | LidarDetails::EDGE_OF_FLIGHT_LINE;
        assert_eq!(got.lidar.flags, want.lidar.flags & kept);
        // LAS 1.2 marks overlap points with class 12.
        let overlap = want.lidar.flags & LidarDetails::OVERLAP != 0;
        assert_eq!(got.classification, if overlap { 12 } else { want.classification });
        assert_eq!(got.extra, want.extra);
    }

    // Back in LAS 1.4, class 12 becomes the overlap flag again, on unclassified points.
    let again = convert_to(&report.outputs[0], Format::Las, &las_1_4());
    let (_, chunk) = read_all(&again);
    for (got, want) in rows(&chunk).iter().zip(&want) {
        let overlap = want.lidar.flags & LidarDetails::OVERLAP != 0;
        assert_eq!(got.classification, if overlap { 1 } else { want.classification });
        assert_eq!(got.lidar.flags & LidarDetails::OVERLAP, want.lidar.flags & LidarDetails::OVERLAP);
    }
}

#[test]
fn ply_holds_returns_lidar_details_and_numeric_extra_values() {
    let folder = test_folder("ply");
    let source = folder.join("lidar.las");
    write_lidar_las(&source);
    let (source_info, source_chunk) = read_all(&source);
    let want = rows(&source_chunk);

    let report = run(&source, Format::Ply, &Settings::default());
    assert!(
        report.warnings.iter().any(|w| w.contains("numbers; extra bytes left out")),
        "the undescribed bytes are reported: {:?}",
        report.warnings
    );
    let ply = &report.outputs[0];
    let (info, chunk) = read_all(ply);
    assert!(info.attributes.returns && info.attributes.lidar);
    assert_eq!(
        info.extra_fields.iter().map(|f| (f.name.as_str(), f.scalar)).collect::<Vec<_>>(),
        [("amplitude", Some(ScalarType::U16)), ("deviation", Some(ScalarType::F32)), ("range", Some(ScalarType::F64))]
    );
    let got = rows(&chunk);
    assert_eq!(got.len(), want.len());
    for (got, want) in got.iter().zip(&want) {
        assert_eq!(got.returns, want.returns);
        assert_eq!(got.lidar, want.lidar);
        assert_eq!(got.classification, want.classification);
        assert_eq!(got.extra[..6], want.extra[..6], "amplitude and deviation as stored");
        let range = f64::from_le_bytes(got.extra[6..14].try_into().unwrap());
        assert_eq!(range, source_info.extra_fields[2].value(&want.extra[6..]).unwrap(), "range as its value");
    }

    // PLY to LAS keeps the PLY properties as extra bytes.
    let las = convert_to(ply, Format::Las, &las_1_4());
    let (las_info, las_chunk) = read_all(&las);
    assert_eq!(field_summary(&las_info.extra_fields), field_summary(&info.extra_fields));
    let las_rows = rows(&las_chunk);
    for (got, want) in las_rows.iter().zip(&got) {
        assert_eq!(got.extra, want.extra);
        assert_eq!(got.returns, want.returns);
    }

    // ASCII PLY has no room for extra values, but keeps the rest.
    let mut ascii = Settings::default();
    ascii.export.ply.encoding = epcc_core::settings::PlyEncoding::Ascii;
    fs::remove_file(ply).unwrap();
    let ply = convert_to(&source, Format::Ply, &ascii);
    let (info, chunk) = read_all(&ply);
    assert_eq!(info.attributes.extra_bytes, 0);
    for (got, want) in rows(&chunk).iter().zip(&want) {
        assert_eq!(got.returns, want.returns);
        assert_eq!((got.lidar.flags, got.lidar.point_source), (want.lidar.flags, want.lidar.point_source));
        assert!((got.lidar.scan_angle - want.lidar.scan_angle).abs() < 0.001);
    }
}

#[test]
fn formats_without_room_report_what_they_leave_out() {
    let folder = test_folder("report");
    let source = folder.join("lidar.las");
    write_lidar_las(&source);
    let report = run(&source, Format::Pts, &Settings::default());
    let warning = report.warnings.iter().find(|w| w.starts_with("PTS cannot store")).expect("a warning");
    assert!(warning.contains("return numbers, lidar details"), "{warning}");
    assert!(warning.contains("extra values (amplitude, deviation, range, extra bytes)"), "{warning}");
}

#[test]
fn dropped_point_fields_are_left_out() {
    let folder = test_folder("drop");
    let source = folder.join("lidar.las");
    write_lidar_las(&source);
    let mut settings = las_1_4();
    settings.processing.drop.returns = true;
    settings.processing.drop.extra_bytes = 1;
    let report = run(&source, Format::Las, &settings);
    assert!(report.warnings.is_empty(), "{:?}", report.warnings);
    let (info, chunk) = read_all(&report.outputs[0]);
    assert_eq!(info.attributes.extra_bytes, 0);
    assert!(info.extra_fields.is_empty());
    let header_len = u16::from_le_bytes(fs::read(&report.outputs[0]).unwrap()[105..107].try_into().unwrap());
    assert_eq!(header_len, 36, "point format 7 without extra bytes");
    // Without return numbers, LAS stores 0 (unknown); the lidar details stay.
    assert!(rows(&chunk).iter().all(|row| row.returns == Returns::default()));
    let (_, source_chunk) = read_all(&source);
    assert!(rows(&chunk).iter().zip(rows(&source_chunk)).all(|(got, want)| got.lidar == want.lidar));
}
