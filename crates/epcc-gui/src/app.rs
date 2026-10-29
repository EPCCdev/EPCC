//! The main window: the list of files to convert and the export options.

use std::path::{Path, PathBuf};
use std::time::Duration;

use eframe::egui::{self, Align, Color32, Layout, RichText};
use epcc_core::{ConversionJob, Format, Settings};

use crate::batch::{Batch, JobEnd};
use crate::settings_dialog::{DialogResult, SettingsDialog};

const ERROR_RED: Color32 = Color32::from_rgb(180, 0, 0);
const DONE_GREEN: Color32 = Color32::from_rgb(0, 110, 0);

pub struct EpccApp {
    ctx: egui::Context,
    settings: Settings,
    files: Vec<InputFile>,
    /// The folder picked for this export when the default folder is not used.
    chosen_folder: Option<PathBuf>,
    batch: Option<Batch>,
    settings_dialog: Option<SettingsDialog>,
    about_open: bool,
    /// A message shown above the file list until the next action.
    notice: Option<String>,
}

struct InputFile {
    path: PathBuf,
    format: Format,
    size: u64,
    state: FileState,
}

enum FileState {
    Ready,
    Queued,
    Ended(JobEnd),
}

impl EpccApp {
    pub fn new(ctx: egui::Context, files: Vec<PathBuf>) -> Self {
        let (settings, notice) = match Settings::load() {
            Ok(settings) => (settings, None),
            Err(error) => {
                (Settings::default(), Some(format!("Saved settings could not be read, defaults are used: {error}")))
            }
        };
        let mut app = EpccApp {
            ctx,
            settings,
            files: Vec::new(),
            chosen_folder: None,
            batch: None,
            settings_dialog: None,
            about_open: false,
            notice,
        };
        app.add_files(files);
        app
    }

    fn is_converting(&self) -> bool {
        self.batch.is_some()
    }

    fn add_files(&mut self, paths: impl IntoIterator<Item = PathBuf>) {
        let mut unsupported = Vec::new();
        for mut path in paths {
            let format = Format::from_path(&path);
            if path.is_dir() && format.is_none() {
                let inside = std::fs::read_dir(&path).into_iter().flatten().flatten().map(|entry| entry.path());
                let mut inside: Vec<PathBuf> = inside.filter(|p| Format::from_path(p).is_some()).collect();
                inside.sort();
                self.add_files(inside);
                continue;
            }
            let Some(format) = format else {
                unsupported.push(file_name(&path));
                continue;
            };
            // A Potree cloud is listed as its folder, also when its description file was picked.
            if format.writes_folders()
                && path.is_file()
                && let Some(folder) = path.parent()
            {
                path = folder.to_path_buf();
            }
            if self.files.iter().any(|file| file.path == path) {
                continue;
            }
            let size = size_of(&path);
            self.files.push(InputFile { path, format, size, state: FileState::Ready });
        }
        if !unsupported.is_empty() {
            self.notice = Some(format!("Not a supported point cloud format: {}", unsupported.join(", ")));
        }
    }

    fn pick_files(&mut self) {
        let named = Format::ALL.iter().flat_map(|format| format.input_file_names());
        let named_extensions = named.filter_map(|name| name.rsplit_once('.').map(|(_, extension)| extension));
        let mut extensions: Vec<&str> =
            Format::ALL.iter().flat_map(|format| format.input_extensions()).copied().chain(named_extensions).collect();
        extensions.dedup();
        let mut dialog = rfd::FileDialog::new().add_filter("Point clouds", &extensions);
        if let Some(folder) = &self.settings.general.input_folder {
            dialog = dialog.set_directory(folder);
        }
        if let Some(paths) = dialog.pick_files() {
            self.add_files(paths);
        }
    }

    /// Where outputs of `input` go, or `None` if a folder still has to be chosen.
    fn output_folder_for(&self, input: &Path) -> Option<PathBuf> {
        let general = &self.settings.general;
        if general.use_output_folder {
            general.output_folder.clone().or_else(|| input.parent().map(Path::to_path_buf))
        } else {
            self.chosen_folder.clone()
        }
    }

    fn start_conversion(&mut self) {
        self.notice = None;
        if !self.settings.general.use_output_folder && self.chosen_folder.is_none() {
            self.chosen_folder = rfd::FileDialog::new().pick_folder();
            if self.chosen_folder.is_none() {
                return;
            }
        }
        let format = self.settings.general.output_format;
        let jobs: Vec<(usize, ConversionJob)> = self
            .files
            .iter()
            .enumerate()
            .filter_map(|(row, file)| {
                let output_folder = self.output_folder_for(&file.path)?;
                Some((row, ConversionJob { input: file.path.clone(), format, output_folder }))
            })
            .collect();
        if jobs.is_empty() {
            return;
        }
        for file in &mut self.files {
            file.state = FileState::Queued;
        }
        self.batch = Some(Batch::start(jobs, self.settings.clone(), self.ctx.clone()));
    }

    fn collect_results(&mut self) {
        let Some(batch) = &mut self.batch else { return };
        for (row, end) in batch.take_ended() {
            if let Some(file) = self.files.get_mut(row) {
                file.state = FileState::Ended(end);
            }
        }
        if batch.is_finished() {
            self.batch = None;
        } else {
            // Keep the progress bars moving.
            self.ctx.request_repaint_after(Duration::from_millis(150));
        }
    }

    /// Remembers choices made in the main window, like the output format, for next time.
    fn save_settings_quietly(&mut self) {
        if let Err(error) = self.settings.save() {
            self.notice = Some(format!("Settings could not be saved: {error}"));
        }
    }
}

impl eframe::App for EpccApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let dropped: Vec<PathBuf> =
            ui.ctx().input(|input| input.raw.dropped_files.iter().map(|file| file.path().to_path_buf()).collect());
        if !dropped.is_empty() && !self.is_converting() {
            self.add_files(dropped);
        }
        self.collect_results();

        egui::Panel::top("menu").show(ui, |ui| self.menu_bar(ui));
        egui::Panel::bottom("export").show(ui, |ui| self.export_panel(ui));
        egui::CentralPanel::default().show(ui, |ui| self.file_list(ui));

        self.dialogs(ui.ctx());
    }
}

impl EpccApp {
    fn menu_bar(&mut self, ui: &mut egui::Ui) {
        egui::MenuBar::new().ui(ui, |ui| {
            let idle = !self.is_converting();
            ui.menu_button("File", |ui| {
                if ui.add_enabled(idle, egui::Button::new("Add files...")).clicked() {
                    self.pick_files();
                }
                if ui.add_enabled(idle, egui::Button::new("Add folder...")).clicked()
                    && let Some(folder) = rfd::FileDialog::new().pick_folder()
                {
                    self.add_files([folder]);
                }
                if ui.add_enabled(idle && !self.files.is_empty(), egui::Button::new("Clear list")).clicked() {
                    self.files.clear();
                }
                ui.separator();
                if ui.button("Exit").clicked() {
                    ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
                }
            });
            if ui.add_enabled(idle, egui::Button::new("Settings...")).clicked() {
                self.settings_dialog = Some(SettingsDialog::new(&self.settings));
            }
            ui.menu_button("Help", |ui| {
                if ui.button("About EPCC").clicked() {
                    self.about_open = true;
                }
            });
        });
    }

    fn file_list(&mut self, ui: &mut egui::Ui) {
        if let Some(notice) = &self.notice {
            ui.label(RichText::new(notice).color(ERROR_RED));
            ui.add_space(4.0);
        }
        let hovering = ui.ctx().input(|input| !input.raw.hovered_files.is_empty());

        ui.horizontal(|ui| {
            let idle = !self.is_converting();
            if ui.add_enabled(idle, egui::Button::new("Add files...")).clicked() {
                self.pick_files();
            }
            if ui.add_enabled(idle && !self.files.is_empty(), egui::Button::new("Clear list")).clicked() {
                self.files.clear();
            }
        });
        ui.add_space(4.0);

        let frame = egui::Frame::new()
            .fill(Color32::WHITE)
            .stroke(egui::Stroke::new(
                1.0,
                if hovering { Color32::from_rgb(0, 120, 215) } else { Color32::from_gray(160) },
            ))
            .inner_margin(6.0);
        frame.show(ui, |ui| {
            ui.set_min_size(ui.available_size());
            if self.files.is_empty() {
                ui.centered_and_justified(|ui| {
                    ui.label(RichText::new("Drop point cloud files or folders here").size(16.0).color(Color32::GRAY));
                });
                return;
            }
            egui::ScrollArea::vertical().auto_shrink(false).show(ui, |ui| self.file_rows(ui));
        });
    }

    fn file_rows(&mut self, ui: &mut egui::Ui) {
        let mut remove = None;
        egui::Grid::new("files").striped(true).num_columns(5).spacing([16.0, 6.0]).show(ui, |ui| {
            for heading in ["Name", "Type", "Size", "Status", ""] {
                ui.label(RichText::new(heading).strong());
            }
            ui.end_row();

            for (row, file) in self.files.iter().enumerate() {
                ui.label(file_name(&file.path)).on_hover_text(file.path.display().to_string());
                ui.label(file.format.name());
                ui.label(human_size(file.size));
                self.status_cell(ui, row, file);
                if ui.add_enabled(self.batch.is_none(), egui::Button::new("Remove").small()).clicked() {
                    remove = Some(row);
                }
                ui.end_row();
            }
        });
        if let Some(row) = remove {
            self.files.remove(row);
        }
    }

    fn status_cell(&self, ui: &mut egui::Ui, row: usize, file: &InputFile) {
        match &file.state {
            FileState::Ready => {
                ui.label("Ready");
            }
            FileState::Queued => match self.batch.as_ref().and_then(|batch| batch.progress_of(row)) {
                Some(fraction) if fraction > 0.0 => {
                    ui.add(egui::ProgressBar::new(fraction).desired_width(140.0).show_percentage().corner_radius(0));
                }
                _ => {
                    ui.label("Waiting");
                }
            },
            FileState::Ended(JobEnd::Converted { outputs, points_written, warnings }) => {
                let mut text = format!("Done, {} points", group_digits(*points_written));
                if !warnings.is_empty() {
                    text += &format!(" ({} warning{})", warnings.len(), if warnings.len() == 1 { "" } else { "s" });
                }
                let mut details: Vec<String> = outputs.iter().map(|path| path.display().to_string()).collect();
                details.extend(warnings.iter().map(|warning| format!("Warning: {warning}")));
                ui.label(RichText::new(text).color(DONE_GREEN)).on_hover_text(details.join("\n"));
            }
            FileState::Ended(JobEnd::Skipped(existing)) => {
                ui.label("Skipped, output exists").on_hover_text(existing.display().to_string());
            }
            FileState::Ended(JobEnd::Failed(message)) => {
                // The row already names the file, so a leading path is left out.
                let prefix = format!("{}: ", file.path.display());
                let reason = message.strip_prefix(&prefix).unwrap_or(message);
                ui.scope(|ui| {
                    ui.set_max_width(300.0);
                    let text = RichText::new(format!("Failed: {reason}")).color(ERROR_RED);
                    ui.add(egui::Label::new(text).truncate()).on_hover_text(message);
                });
            }
            FileState::Ended(JobEnd::Cancelled) => {
                ui.label("Cancelled");
            }
        }
    }

    fn export_panel(&mut self, ui: &mut egui::Ui) {
        ui.add_space(6.0);
        let idle = !self.is_converting();
        ui.add_enabled_ui(idle, |ui| {
            egui::Grid::new("export").num_columns(2).spacing([12.0, 8.0]).show(ui, |ui| {
                ui.label("Output format:");
                let mut format = self.settings.general.output_format;
                egui::ComboBox::from_id_salt("format").width(220.0).selected_text(format.description()).show_ui(
                    ui,
                    |ui| {
                        for option in Format::ALL {
                            ui.selectable_value(&mut format, option, option.description());
                        }
                    },
                );
                if format != self.settings.general.output_format {
                    self.settings.general.output_format = format;
                    self.save_settings_quietly();
                }
                ui.end_row();

                ui.label("Save to:");
                ui.vertical(|ui| self.output_folder_choice(ui));
                ui.end_row();
            });
        });
        ui.add_space(6.0);
        ui.separator();
        self.convert_row(ui);
        ui.add_space(6.0);
    }

    fn output_folder_choice(&mut self, ui: &mut egui::Ui) {
        let general = &self.settings.general;
        let default_folder = match &general.output_folder {
            Some(folder) => folder.display().to_string(),
            None => "the folder of each input file".into(),
        };
        let mut use_default = general.use_output_folder;
        ui.checkbox(&mut use_default, format!("Default folder: {default_folder}"));
        if use_default != self.settings.general.use_output_folder {
            self.settings.general.use_output_folder = use_default;
            self.save_settings_quietly();
        }
        if !use_default {
            ui.horizontal(|ui| {
                let shown = self
                    .chosen_folder
                    .as_ref()
                    .map_or_else(|| "(choose a folder)".into(), |folder| folder.display().to_string());
                ui.label(shown);
                if ui.button("Choose...").clicked() {
                    let mut dialog = rfd::FileDialog::new();
                    if let Some(folder) = &self.chosen_folder {
                        dialog = dialog.set_directory(folder);
                    }
                    if let Some(folder) = dialog.pick_folder() {
                        self.chosen_folder = Some(folder);
                    }
                }
            });
        }
    }

    fn convert_row(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            match &self.batch {
                Some(batch) => {
                    let done = batch.jobs.len()
                        - batch
                            .jobs
                            .iter()
                            .filter(|(row, _)| matches!(self.files[*row].state, FileState::Queued))
                            .count();
                    let text = format!("{done} of {} files", batch.jobs.len());
                    ui.add(egui::ProgressBar::new(batch.fraction()).desired_width(320.0).text(text).corner_radius(0));
                }
                None => {
                    let summary = self.summary();
                    ui.label(summary);
                }
            }
            ui.with_layout(Layout::right_to_left(Align::Center), |ui| match &self.batch {
                Some(batch) => {
                    let label = if batch.is_cancelling() { "Cancelling..." } else { "Cancel" };
                    if ui
                        .add_enabled(!batch.is_cancelling(), egui::Button::new(label).min_size([90.0, 28.0].into()))
                        .clicked()
                    {
                        batch.cancel();
                    }
                }
                None => {
                    let convert = egui::Button::new(RichText::new("Convert").strong()).min_size([110.0, 28.0].into());
                    if ui.add_enabled(!self.files.is_empty(), convert).clicked() {
                        self.start_conversion();
                    }
                }
            });
        });
    }

    /// A one-line summary of the last run, or of the list before any run.
    fn summary(&self) -> String {
        let count = |wanted: fn(&FileState) -> bool| self.files.iter().filter(|file| wanted(&file.state)).count();
        let converted = count(|state| matches!(state, FileState::Ended(JobEnd::Converted { .. })));
        let failed = count(|state| matches!(state, FileState::Ended(JobEnd::Failed(_))));
        let skipped = count(|state| matches!(state, FileState::Ended(JobEnd::Skipped(_))));
        if converted + failed + skipped == 0 {
            let total: u64 = self.files.iter().map(|file| file.size).sum();
            return match self.files.len() {
                0 => String::new(),
                1 => format!("1 file, {}", human_size(total)),
                n => format!("{n} files, {}", human_size(total)),
            };
        }
        let mut parts = vec![format!("{converted} converted")];
        if skipped > 0 {
            parts.push(format!("{skipped} skipped"));
        }
        if failed > 0 {
            parts.push(format!("{failed} failed"));
        }
        parts.join(", ")
    }

    fn dialogs(&mut self, ctx: &egui::Context) {
        if let Some(dialog) = &mut self.settings_dialog {
            match dialog.show(ctx) {
                DialogResult::Open => {}
                DialogResult::Cancelled => self.settings_dialog = None,
                DialogResult::Saved(settings) => {
                    self.settings = settings;
                    self.settings_dialog = None;
                }
            }
        }
        if self.about_open {
            let modal = egui::Modal::new(egui::Id::unique("about")).show(ctx, |ui| {
                ui.set_width(360.0);
                ui.heading("EPCC - Easy Point Cloud Converter");
                ui.label(format!("Version {}", epcc_core::VERSION));
                ui.add_space(6.0);
                ui.label("Converts between LAS, LAZ, COPC, E57, PTS, XYZ, PLY and RCS.");
                ui.label("Free software under the MIT or Apache 2.0 licence.");
                ui.add_space(8.0);
                ui.with_layout(Layout::right_to_left(Align::Min), |ui| ui.button("Close").clicked()).inner
            });
            if modal.inner || modal.should_close() {
                self.about_open = false;
            }
        }
    }
}

fn file_name(path: &Path) -> String {
    path.file_name().map_or_else(|| path.display().to_string(), |name| name.to_string_lossy().into_owned())
}

/// The size of a file, or of everything in a folder.
fn size_of(path: &Path) -> u64 {
    let Ok(meta) = std::fs::metadata(path) else { return 0 };
    if !meta.is_dir() {
        return meta.len();
    }
    let entries = std::fs::read_dir(path).into_iter().flatten().flatten();
    entries.map(|entry| size_of(&entry.path())).sum()
}

fn human_size(bytes: u64) -> String {
    const UNITS: [&str; 5] = ["B", "KB", "MB", "GB", "TB"];
    let mut value = bytes as f64;
    let mut unit = 0;
    while value >= 1024.0 && unit < UNITS.len() - 1 {
        value /= 1024.0;
        unit += 1;
    }
    if unit == 0 { format!("{bytes} B") } else { format!("{value:.1} {}", UNITS[unit]) }
}

/// 1234567 → "1,234,567".
fn group_digits(value: u64) -> String {
    let digits = value.to_string();
    let mut grouped = String::new();
    for (index, digit) in digits.chars().enumerate() {
        if index > 0 && (digits.len() - index).is_multiple_of(3) {
            grouped.push(',');
        }
        grouped.push(digit);
    }
    grouped
}
