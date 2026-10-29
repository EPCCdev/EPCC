//! Leica PTS: a point count line, then `x y z intensity [r g b]` per line, intensity from -2048
//! to 2047. Files with several scans repeat the count line before each scan.

use std::path::Path;

use super::text::{self, Column, IntensityStyle, LineFormat, LineStyle, Sniffed, TextReader, TextWriter};
use super::{PointReader, PointWriter, WriteRequest};
use crate::model::ValueRange;
use crate::settings::ImportSettings;
use crate::{Error, Result};

pub(super) fn open(path: &Path, settings: &ImportSettings) -> Result<Box<dyn PointReader>> {
    let sniffed = text::sniff(path)?;
    let columns = columns_for(&sniffed)?;
    let intensity_samples = field_of(&sniffed, &columns, Column::Intensity);
    let color_samples = field_of(&sniffed, &columns, Column::Red);
    let format = LineFormat {
        intensity: intensity_range(&intensity_samples),
        color: LineFormat::color_range_from(color_samples),
        unit_scale: settings.text_units.in_metres(),
        scan_breaks: true,
        decimal_comma: sniffed.decimal_comma,
        columns,
    };
    Ok(Box::new(TextReader::new(path, format, sniffed.header_lines)))
}

fn columns_for(sniffed: &Sniffed) -> Result<Vec<Column>> {
    use Column::*;
    let columns = match sniffed.field_count() {
        3 => vec![X, Y, Z],
        4 => vec![X, Y, Z, Intensity],
        6 => vec![X, Y, Z, Red, Green, Blue],
        n if n >= 7 => vec![X, Y, Z, Intensity, Red, Green, Blue],
        n => return Err(Error::invalid(format!("PTS lines need 3, 4, 6 or 7 values, found {n}"))),
    };
    Ok(columns)
}

fn field_of(sniffed: &Sniffed, columns: &[Column], column: Column) -> Vec<f64> {
    columns.iter().position(|&c| c == column).map(|index| sniffed.field(index).collect()).unwrap_or_default()
}

/// PTS intensity is defined as -2048..2047, but some programs write 0..255 or 0..1.
fn intensity_range(samples: &[f64]) -> ValueRange {
    let min = samples.iter().copied().fold(f64::INFINITY, f64::min);
    let max = samples.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let whole = samples.iter().all(|v| v.fract() == 0.0);
    if min >= 0.0 && max <= 1.0 && !whole {
        ValueRange::UNIT
    } else if min >= 0.0 && max <= 255.0 {
        ValueRange::BYTE
    } else {
        ValueRange::PTS
    }
}

pub(super) fn create(request: &WriteRequest) -> Result<Box<dyn PointWriter>> {
    let attributes = request.cloud.attributes;
    // PTS readers expect the intensity column whenever colours follow it.
    let intensity = match (attributes.intensity, attributes.color) {
        (true, _) => IntensityStyle::Pts,
        (false, true) => IntensityStyle::PtsZero,
        (false, false) => IntensityStyle::None,
    };
    Ok(Box::new(TextWriter {
        path: request.path.to_path_buf(),
        style: LineStyle {
            separator: b' ',
            decimals: request.settings.pts.decimals,
            intensity,
            color: attributes.color,
            normal: false,
        },
        count_line: true,
    }))
}
