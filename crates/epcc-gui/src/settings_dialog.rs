//! The settings dialog. It edits a copy of the settings; OK saves the copy, Cancel drops it.

use std::path::PathBuf;

use eframe::egui::{self, Align, Color32, Layout, RichText};
use epcc_core::Settings;
use epcc_core::settings::{
    CoordinateType, ExistingFiles, ExportSettings, LasVersion, LengthUnit, PlyEncoding, ProcessingSettings, ScanMode,
    Separator,
};

const LIST_SIZE: egui::Vec2 = egui::vec2(130.0, 330.0);
const PAGE_SIZE: egui::Vec2 = egui::vec2(520.0, 330.0);
const FOLDER_WIDTH: f32 = 300.0;

pub enum DialogResult {
    Open,
    Saved(Settings),
    Cancelled,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Page {
    General,
    Import,
    Processing,
    Las,
    E57,
    Pts,
    Xyz,
    Ply,
    Rcs,
}

impl Page {
    const ALL: [Page; 9] = [
        Page::General,
        Page::Import,
        Page::Processing,
        Page::Las,
        Page::E57,
        Page::Pts,
        Page::Xyz,
        Page::Ply,
        Page::Rcs,
    ];

    fn title(self) -> &'static str {
        match self {
            Page::General => "General",
            Page::Import => "Text import",
            Page::Processing => "Reduction",
            Page::Las => "LAS / LAZ / COPC",
            Page::E57 => "E57",
            Page::Pts => "PTS",
            Page::Xyz => "XYZ",
            Page::Ply => "PLY",
            Page::Rcs => "RCS",
        }
    }

    /// Puts this page's settings back to the built-in defaults.
    fn reset(self, settings: &mut Settings) {
        let defaults = Settings::default();
        match self {
            Page::General => settings.general = defaults.general,
            Page::Import => settings.import = defaults.import,
            Page::Processing => settings.processing = defaults.processing,
            Page::Las => settings.export.las = defaults.export.las,
            Page::E57 => settings.export.e57 = defaults.export.e57,
            Page::Pts => settings.export.pts = defaults.export.pts,
            Page::Xyz => settings.export.xyz = defaults.export.xyz,
            Page::Ply => settings.export.ply = defaults.export.ply,
            Page::Rcs => settings.export.rcs = defaults.export.rcs,
        }
    }
}

pub struct SettingsDialog {
    draft: Settings,
    page: Page,
    error: Option<String>,
    /// Values shown while a reduction is switched off, so switching it on again restores them.
    spacing: f64,
    every_nth: u64,
}

impl SettingsDialog {
    pub fn new(settings: &Settings) -> Self {
        SettingsDialog {
            draft: settings.clone(),
            page: Page::General,
            error: None,
            spacing: settings.processing.min_spacing.unwrap_or(0.005),
            every_nth: settings.processing.every_nth.unwrap_or(2),
        }
    }

    pub fn show(&mut self, ctx: &egui::Context) -> DialogResult {
        let mut result = DialogResult::Open;
        let modal = egui::Modal::new(egui::Id::unique("settings")).show(ctx, |ui| {
            ui.set_width(LIST_SIZE.x + PAGE_SIZE.x + 20.0);
            ui.heading("Settings");
            ui.add_space(4.0);
            ui.horizontal_top(|ui| {
                list_box().show(ui, |ui| {
                    ui.allocate_ui_with_layout(LIST_SIZE, Layout::top_down_justified(Align::Min), |ui| {
                        ui.set_min_size(LIST_SIZE);
                        for page in Page::ALL {
                            if ui.add(egui::Button::selectable(self.page == page, page.title())).clicked() {
                                self.page = page;
                            }
                        }
                    });
                });
                ui.add_space(8.0);
                ui.allocate_ui_with_layout(PAGE_SIZE, Layout::top_down(Align::Min), |ui| {
                    ui.set_min_size(PAGE_SIZE);
                    ui.set_max_width(PAGE_SIZE.x);
                    ui.label(RichText::new(self.page.title()).strong());
                    ui.add_space(6.0);
                    self.page_contents(ui);
                });
            });
            ui.separator();
            if let Some(error) = &self.error {
                ui.label(RichText::new(error).color(Color32::from_rgb(180, 0, 0)));
            }
            ui.horizontal(|ui| {
                if ui.button("Restore defaults for this page").clicked() {
                    self.page.reset(&mut self.draft);
                    self.spacing = 0.005;
                    self.every_nth = 2;
                }
                ui.with_layout(Layout::right_to_left(Align::Center), |ui| {
                    if ui.button("Cancel").clicked() {
                        result = DialogResult::Cancelled;
                    }
                    if ui.add(egui::Button::new("OK").min_size([80.0, 0.0].into())).clicked() {
                        result = self.save();
                    }
                });
            });
        });
        if modal.should_close() {
            result = DialogResult::Cancelled;
        }
        result
    }

    fn save(&mut self) -> DialogResult {
        if let Err(error) = self.draft.validate() {
            self.error = Some(error.to_string());
            return DialogResult::Open;
        }
        match self.draft.save() {
            Ok(_) => DialogResult::Saved(self.draft.clone()),
            Err(error) => {
                self.error = Some(format!("Could not save: {error}"));
                DialogResult::Open
            }
        }
    }

    fn page_contents(&mut self, ui: &mut egui::Ui) {
        match self.page {
            Page::General => general_page(ui, &mut self.draft),
            Page::Import => import_page(ui, &mut self.draft),
            Page::Processing => processing_page(ui, &mut self.draft.processing, &mut self.spacing, &mut self.every_nth),
            Page::Las => las_page(ui, &mut self.draft.export),
            Page::E57 => e57_page(ui, &mut self.draft.export),
            Page::Pts => pts_page(ui, &mut self.draft.export),
            Page::Xyz => xyz_page(ui, &mut self.draft.export),
            Page::Ply => ply_page(ui, &mut self.draft.export),
            Page::Rcs => rcs_page(ui, &mut self.draft.export),
        }
    }
}

fn general_page(ui: &mut egui::Ui, settings: &mut Settings) {
    let general = &mut settings.general;
    ui.label("Open files from:");
    folder_field(ui, &mut general.input_folder, "the folder used last");
    ui.add_space(4.0);
    ui.label("Default output folder:");
    folder_field(ui, &mut general.output_folder, "the folder of each input file");
    ui.add_space(8.0);

    egui::Grid::new("general").num_columns(2).spacing([12.0, 10.0]).show(ui, |ui| {
        ui.label("If an output file exists:");
        combo(ui, "existing", &mut general.existing_files, &ExistingFiles::ALL, |mode| mode.label());
        ui.end_row();

        ui.label("Files at the same time:");
        ui.horizontal(|ui| {
            ui.add(egui::DragValue::new(&mut general.parallel_files).range(0..=32));
            ui.label(hint("0 = automatic"));
        });
        ui.end_row();
    });
    ui.add_space(6.0);
    ui.checkbox(&mut general.write_log, "Write a log file next to each output");
}

fn import_page(ui: &mut egui::Ui, settings: &mut Settings) {
    let import = &mut settings.import;
    ui.label("PTS and XYZ files do not record their units or column meanings.");
    ui.add_space(6.0);
    egui::Grid::new("import").num_columns(2).spacing([12.0, 10.0]).show(ui, |ui| {
        ui.label("Coordinates are in:");
        combo(ui, "units", &mut import.text_units, &LengthUnit::ALL, |unit| unit.label());
        ui.end_row();

        ui.label("XYZ columns:");
        ui.vertical(|ui| {
            ui.add(
                egui::TextEdit::singleline(&mut import.text_columns)
                    .hint_text("detect automatically")
                    .desired_width(220.0),
            );
            ui.label(hint("e.g. x,y,z,i,r,g,b — also nx,ny,nz, class, time, _ to skip"));
        });
        ui.end_row();
    });
}

fn processing_page(ui: &mut egui::Ui, processing: &mut ProcessingSettings, spacing: &mut f64, every_nth: &mut u64) {
    ui.label("Applies to every export.");
    ui.add_space(6.0);

    let mut thin = processing.min_spacing.is_some();
    ui.horizontal(|ui| {
        ui.checkbox(&mut thin, "Keep no two points closer than");
        ui.add_enabled(
            thin,
            egui::DragValue::new(spacing).range(0.0001..=10.0).speed(0.0005).max_decimals(4).suffix(" m"),
        );
    });
    processing.min_spacing = thin.then_some(*spacing);

    let mut every = processing.every_nth.is_some();
    ui.horizontal(|ui| {
        ui.checkbox(&mut every, "Keep only one point in every");
        ui.add_enabled(every, egui::DragValue::new(every_nth).range(2..=1000));
    });
    processing.every_nth = every.then_some(*every_nth);

    ui.add_space(10.0);
    ui.label("Leave out of the output:");
    ui.horizontal_wrapped(|ui| {
        let drop = &mut processing.drop;
        ui.checkbox(&mut drop.intensity, "Intensity");
        ui.checkbox(&mut drop.color, "Colour");
        ui.checkbox(&mut drop.normal, "Normals");
        ui.checkbox(&mut drop.classification, "Classification");
        ui.checkbox(&mut drop.time, "Time");
        ui.checkbox(&mut drop.returns, "Return numbers")
            .on_hover_text("Which return of its laser pulse each point is (LAS)");
        ui.checkbox(&mut drop.lidar, "Lidar details")
            .on_hover_text("Scan angle, point source, user data and point flags (LAS)");
        let mut extra = drop.has_extra();
        ui.checkbox(&mut extra, "Extra values")
            .on_hover_text("Further named values per point, such as LAS extra bytes");
        drop.extra_bytes = u16::from(extra);
    });

    ui.add_space(10.0);
    ui.label("Files with several scans, saved as LAS, LAZ, PTS, XYZ, PLY or Potree:");
    ui.radio_value(&mut processing.scans, ScanMode::Merge, "Merge all scans into one file");
    ui.radio_value(&mut processing.scans, ScanMode::Split, "One file per scan");
}

fn las_page(ui: &mut egui::Ui, export: &mut ExportSettings) {
    let las = &mut export.las;
    egui::Grid::new("las").num_columns(2).spacing([12.0, 10.0]).show(ui, |ui| {
        ui.label("Version:");
        combo(ui, "las-version", &mut las.version, &LasVersion::ALL, |version| version.label());
        ui.end_row();

        ui.label("Coordinate precision:");
        const PRECISIONS: [f64; 4] = [0.01, 0.001, 0.0001, 0.00001];
        combo(ui, "precision", &mut las.precision, &PRECISIONS, |&precision| precision_label(precision));
        ui.end_row();
    });
    ui.add_space(6.0);
    ui.label(hint("Automatic writes LAS 1.2 for the widest compatibility, and 1.4 when the cloud has a coordinate system or more than 4 billion points. COPC is always LAS 1.4. The precision also applies to COPC and Potree."));
}

fn e57_page(ui: &mut egui::Ui, export: &mut ExportSettings) {
    ui.checkbox(&mut export.e57.keep_images, "Keep panorama and camera images (E57 to E57 only)");
    ui.label(hint("Images are often most of an E57 file's size. Other formats cannot store them."));
}

fn pts_page(ui: &mut egui::Ui, export: &mut ExportSettings) {
    ui.horizontal(|ui| {
        ui.label("Decimal places:");
        ui.add(egui::DragValue::new(&mut export.pts.decimals).range(0..=9));
    });
}

fn xyz_page(ui: &mut egui::Ui, export: &mut ExportSettings) {
    let xyz = &mut export.xyz;
    egui::Grid::new("xyz").num_columns(2).spacing([12.0, 10.0]).show(ui, |ui| {
        ui.label("Decimal places:");
        ui.add(egui::DragValue::new(&mut xyz.decimals).range(0..=9));
        ui.end_row();

        ui.label("Separator:");
        combo(ui, "separator", &mut xyz.separator, &Separator::ALL, |separator| separator.label());
        ui.end_row();
    });
}

fn ply_page(ui: &mut egui::Ui, export: &mut ExportSettings) {
    let ply = &mut export.ply;
    ui.radio_value(&mut ply.encoding, PlyEncoding::Binary, "Binary (small and fast)");
    ui.radio_value(&mut ply.encoding, PlyEncoding::Ascii, "Text");
    ui.add_space(6.0);
    ui.horizontal(|ui| {
        ui.label("Coordinates:");
        combo(ui, "ply-coordinates", &mut ply.coordinates, &CoordinateType::ALL, |kind| kind.label());
    });
}

fn rcs_page(ui: &mut egui::Ui, export: &mut ExportSettings) {
    ui.checkbox(&mut export.rcs.estimate_normals, "Estimate normals when the source has none");
    ui.label(hint(
        "Estimated normals are approximate. Normals from the source are always kept. RCS support is experimental.",
    ));
}

fn combo<T: Copy + PartialEq, L: Into<String>>(
    ui: &mut egui::Ui,
    id: &str,
    value: &mut T,
    options: &[T],
    label: impl Fn(&T) -> L,
) {
    egui::ComboBox::from_id_salt(id).width(220.0).selected_text(label(value).into()).show_ui(ui, |ui| {
        for option in options {
            ui.selectable_value(value, *option, label(option).into());
        }
    });
}

/// A read-only folder path with Browse and Clear buttons. `None` shows `empty`.
fn folder_field(ui: &mut egui::Ui, folder: &mut Option<PathBuf>, empty: &str) {
    ui.horizontal(|ui| {
        let shown = match folder {
            Some(path) => RichText::new(path.display().to_string()),
            None => RichText::new(format!("({empty})")).color(Color32::GRAY),
        };
        list_box().inner_margin(egui::Margin::symmetric(6, 3)).show(ui, |ui| {
            ui.set_width(FOLDER_WIDTH);
            ui.add(egui::Label::new(shown).truncate());
        });
        if ui.button("Browse...").clicked()
            && let Some(picked) = rfd::FileDialog::new().pick_folder()
        {
            *folder = Some(picked);
        }
        if ui.add_enabled(folder.is_some(), egui::Button::new("Clear")).clicked() {
            *folder = None;
        }
    });
}

fn precision_label(metres: f64) -> String {
    if metres >= 0.01 {
        format!("{} cm", metres * 100.0)
    } else if metres >= 0.001 {
        format!("{} mm", metres * 1000.0)
    } else {
        format!("{} mm", (metres * 1000.0 * 100_000.0).round() / 100_000.0)
    }
}

/// A white, bordered box like a classic list control.
fn list_box() -> egui::Frame {
    egui::Frame::new().fill(Color32::WHITE).stroke(egui::Stroke::new(1.0, Color32::from_gray(160))).inner_margin(2.0)
}

fn hint(text: &str) -> RichText {
    RichText::new(text).small().color(Color32::from_gray(90))
}
