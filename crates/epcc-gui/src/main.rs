//! `epcc-gui`: the desktop window. Files given on the command line (or dropped onto the program
//! icon) are added to the list.

// No console window behind the GUI in release builds on Windows.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

mod app;
mod batch;
mod settings_dialog;
mod style;

use eframe::egui;

const TITLE: &str = "EPCC - Easy Point Cloud Converter";

/// Window icon: 128 x 128 RGBA pixels rendered from `assets/icon.svg` by `scripts/make_icons.py`.
const ICON_RGBA: &[u8] = include_bytes!("../../../assets/icon-128.rgba");

fn main() -> eframe::Result {
    let files: Vec<std::path::PathBuf> = std::env::args_os().skip(1).map(Into::into).collect();
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title(TITLE)
            .with_inner_size([780.0, 540.0])
            .with_min_inner_size([600.0, 420.0])
            .with_drag_and_drop(true)
            .with_icon(egui::IconData { rgba: ICON_RGBA.to_vec(), width: 128, height: 128 }),
        ..Default::default()
    };
    eframe::run_native(
        TITLE,
        options,
        Box::new(move |creation| {
            style::apply(&creation.egui_ctx);
            Ok(Box::new(app::EpccApp::new(creation.egui_ctx.clone(), files)))
        }),
    )
}
