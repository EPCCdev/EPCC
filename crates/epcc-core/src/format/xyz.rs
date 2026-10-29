//! Plain text columns (`.xyz`, `.txt`, `.asc`, `.csv`). There is no standard, so the column
//! layout comes from the settings, a header line, or a guess from the values.

use std::path::Path;

use super::text::{self, Column, IntensityStyle, LineFormat, LineStyle, Sniffed, TextReader, TextWriter};
use super::{PointReader, PointWriter, WriteRequest};
use crate::Result;
use crate::model::ValueRange;
use crate::settings::ImportSettings;

pub(super) fn open(path: &Path, settings: &ImportSettings) -> Result<Box<dyn PointReader>> {
    let sniffed = text::sniff(path)?;
    let columns = if settings.text_columns.trim().is_empty() {
        sniffed.header_columns.clone().unwrap_or_else(|| guess_columns(&sniffed))
    } else {
        text::parse_column_list(&settings.text_columns)?
    };
    let samples_of = |column| {
        columns
            .iter()
            .position(|&c| c == column)
            .map(|index| sniffed.field(index).collect::<Vec<_>>())
            .unwrap_or_default()
    };
    let format = LineFormat {
        intensity: ValueRange::guess_intensity(samples_of(Column::Intensity)),
        color: LineFormat::color_range_from(samples_of(Column::Red)),
        unit_scale: settings.text_units.in_metres(),
        scan_breaks: false,
        decimal_comma: sniffed.decimal_comma,
        columns,
    };
    Ok(Box::new(TextReader::new(path, format, sniffed.header_lines)))
}

/// Guesses the meaning of each column from the number of columns and their values.
fn guess_columns(sniffed: &Sniffed) -> Vec<Column> {
    use Column::*;
    let colors_at = |first: usize| (first..first + 3).all(|index| sniffed.looks_like_color(index));
    let normals_at = |first: usize| (first..first + 3).all(|index| sniffed.looks_like_normal(index));
    let rgb = [Red, Green, Blue];
    let normal = [NormalX, NormalY, NormalZ];

    let mut columns = vec![X, Y, Z];
    match sniffed.field_count() {
        0..=3 => {}
        4 => columns.push(Intensity),
        5 => columns.extend([Intensity, Skip]),
        6 if normals_at(3) && !colors_at(3) => columns.extend(normal),
        6 => columns.extend(rgb),
        7 if colors_at(3) && !sniffed.looks_like_color(6) => columns.extend([Red, Green, Blue, Intensity]),
        7 => columns.extend([Intensity, Red, Green, Blue]),
        9 if colors_at(3) => columns.extend(rgb.into_iter().chain(normal)),
        9 => columns.extend(normal.into_iter().chain(rgb)),
        n => {
            if colors_at(4) {
                columns.extend([Intensity, Red, Green, Blue]);
            } else if colors_at(3) {
                columns.extend(rgb);
            }
            if columns.len() + 3 <= n && normals_at(columns.len()) {
                columns.extend(normal);
            }
        }
    }
    columns
}

pub(super) fn create(request: &WriteRequest) -> Result<Box<dyn PointWriter>> {
    let attributes = request.cloud.attributes;
    let settings = &request.settings.xyz;
    Ok(Box::new(TextWriter {
        path: request.path.to_path_buf(),
        style: LineStyle {
            separator: settings.separator.byte(),
            decimals: settings.decimals,
            intensity: if attributes.intensity { IntensityStyle::Unit } else { IntensityStyle::None },
            color: attributes.color,
            normal: attributes.normal,
        },
        count_line: false,
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sniffed(lines: &[&[f64]]) -> Sniffed {
        Sniffed {
            header_lines: 0,
            header_columns: None,
            samples: lines.iter().map(|line| line.to_vec()).collect(),
            decimal_comma: false,
        }
    }

    #[test]
    fn colours_and_normals_are_told_apart() {
        use Column::*;
        let colours = sniffed(&[&[1.5, 2.5, 3.5, 255.0, 0.0, 12.0]]);
        assert_eq!(guess_columns(&colours), [X, Y, Z, Red, Green, Blue]);
        let normals = sniffed(&[&[1.5, 2.5, 3.5, 0.6, -0.8, 0.0]]);
        assert_eq!(guess_columns(&normals), [X, Y, Z, NormalX, NormalY, NormalZ]);
        let pts_like = sniffed(&[&[1.5, 2.5, 3.5, -1200.0, 10.0, 20.0, 30.0]]);
        assert_eq!(guess_columns(&pts_like), [X, Y, Z, Intensity, Red, Green, Blue]);
        let intensity_last = sniffed(&[&[1.5, 2.5, 3.5, 10.0, 20.0, 30.0, 0.25]]);
        assert_eq!(guess_columns(&intensity_last), [X, Y, Z, Red, Green, Blue, Intensity]);
    }
}
