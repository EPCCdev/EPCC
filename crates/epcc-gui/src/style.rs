//! A plain, classic look: light grey surfaces, square corners, no shadows, the system font.

use std::sync::Arc;

use eframe::egui::{self, Color32, CornerRadius, FontData, FontDefinitions, FontFamily, Shadow, Stroke, Theme, vec2};

const WINDOW_GREY: Color32 = Color32::from_gray(240);
const BUTTON_GREY: Color32 = Color32::from_gray(225);
const BORDER_GREY: Color32 = Color32::from_gray(160);
const SELECTION_BLUE: Color32 = Color32::from_rgb(0, 120, 215);

pub fn apply(ctx: &egui::Context) {
    ctx.set_theme(egui::ThemePreference::Light);
    use_system_font(ctx);
    ctx.style_mut_of(Theme::Light, |style| {
        let visuals = &mut style.visuals;
        visuals.panel_fill = WINDOW_GREY;
        visuals.window_fill = WINDOW_GREY;
        visuals.faint_bg_color = Color32::from_gray(232);
        visuals.extreme_bg_color = Color32::WHITE;
        visuals.window_corner_radius = CornerRadius::ZERO;
        visuals.menu_corner_radius = CornerRadius::ZERO;
        visuals.window_shadow = Shadow::NONE;
        visuals.popup_shadow = Shadow::NONE;
        visuals.window_stroke = Stroke::new(1.0, BORDER_GREY);
        visuals.selection.bg_fill = SELECTION_BLUE;
        visuals.selection.stroke = Stroke::new(1.0, Color32::WHITE);

        let widgets = &mut visuals.widgets;
        for state in [
            &mut widgets.noninteractive,
            &mut widgets.inactive,
            &mut widgets.hovered,
            &mut widgets.active,
            &mut widgets.open,
        ] {
            state.corner_radius = CornerRadius::ZERO;
            state.expansion = 0.0;
        }
        widgets.noninteractive.fg_stroke = Stroke::new(1.0, Color32::BLACK);
        widgets.inactive.fg_stroke = Stroke::new(1.0, Color32::BLACK);
        widgets.inactive.bg_fill = BUTTON_GREY;
        widgets.inactive.weak_bg_fill = BUTTON_GREY;
        widgets.inactive.bg_stroke = Stroke::new(1.0, BORDER_GREY);
        widgets.hovered.bg_fill = Color32::from_rgb(229, 241, 251);
        widgets.hovered.weak_bg_fill = Color32::from_rgb(229, 241, 251);
        widgets.hovered.bg_stroke = Stroke::new(1.0, SELECTION_BLUE);
        widgets.active.bg_fill = Color32::from_rgb(204, 228, 247);
        widgets.active.weak_bg_fill = Color32::from_rgb(204, 228, 247);
        widgets.active.bg_stroke = Stroke::new(1.0, SELECTION_BLUE);

        style.spacing.button_padding = vec2(10.0, 4.0);
        style.spacing.item_spacing = vec2(8.0, 6.0);
        style.spacing.interact_size.y = 24.0;
    });
}

/// Uses the operating system's interface font when it can be found; egui's own font otherwise.
fn use_system_font(ctx: &egui::Context) {
    const CANDIDATES: [&str; 4] = [
        "C:\\Windows\\Fonts\\segoeui.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/TTF/DejaVuSans.ttf",
    ];
    let Some(bytes) = CANDIDATES.iter().find_map(|path| std::fs::read(path).ok()) else {
        return;
    };
    let mut fonts = FontDefinitions::default();
    fonts.font_data.insert("system".into(), Arc::new(FontData::from_owned(bytes)));
    if let Some(family) = fonts.families.get_mut(&FontFamily::Proportional) {
        family.insert(0, "system".into());
    }
    ctx.set_fonts(fonts);
}
