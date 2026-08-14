use eframe::egui;
use vibro_analysis::math::complex::VibroVector;

use crate::format::format_compact_value;

/// Сколько оборотов до/после курсора показывать в trail.
pub(crate) const POLAR_TRAIL_HALF: usize = 24;

/// Convert a rotor phase (radians) to a screen-space direction vector,
/// applying view rotation and optional mirror.
///
/// Base convention: 0° at top, CW → x = sin(phase), y = -cos(phase).
/// rotation_rad: additional CW rotation of the *view* (labels + vectors rotate together).
/// mirror: flip the view left↔right (negate x component).
fn phase_to_screen_dir(phase_rad: f64, rotation_rad: f32, mirror: bool) -> egui::Vec2 {
    let p = phase_rad as f32 + rotation_rad;
    let x = if mirror { -p.sin() } else { p.sin() };
    let y = -p.cos();
    egui::vec2(x, y)
}

fn vector_endpoint(center: egui::Pos2, radius: f32, vv: VibroVector, rotation_rad: f32, mirror: bool) -> egui::Pos2 {
    center + phase_to_screen_dir(vv.phase, rotation_rad, mirror) * radius
}

pub(crate) fn draw_polar_view(
    ui: &mut egui::Ui,
    vectors: [VibroVector; 2],
    trail: &[[VibroVector; 2]],
    channel_mode: usize,
    rotation_deg: f32,
    mirror: bool,
    max_amp_override: Option<f64>,
    // Текущий угол ротора (0..2π) между двумя соседними KP в позиции курсора.
    // None если курсор не находится между двумя KP.
    cursor_rotor_phase: Option<f64>,
) {
    let colors = [
        egui::Color32::from_rgb(100, 200, 100),
        egui::Color32::from_rgb(100, 150, 255),
    ];

    ui.vertical(|ui| {
        // Квадрат: занимаем всю доступную ширину, а числовые подписи выводим ниже.
        let side = ui.available_width().min(240.0);
        let desired_size = egui::vec2(side, side);
        let (rect, _) = ui.allocate_exact_size(desired_size, egui::Sense::hover());
        let painter = ui.painter_at(rect);

        let center = rect.center();
        let radius = rect.width().min(rect.height()) * 0.34;
        let ring_color = egui::Color32::from_gray(90);
        let axis_color = egui::Color32::from_gray(70);
        let text_color = egui::Color32::from_gray(180);
        let rotation_rad = rotation_deg.to_radians();

        painter.circle_stroke(center, radius, egui::Stroke::new(1.5, ring_color));
        painter.circle_stroke(center, radius * 0.66, egui::Stroke::new(1.0, axis_color));

        // Draw four cardinal axes and their labels, all rotated by rotation_rad.
        // label_angle_deg: logical rotor angle the label represents.
        let axis_labels = [
            (0.0_f32, "0° KP", egui::Color32::from_rgb(255, 210, 80)),
            (90.0, "90°", text_color),
            (180.0, "180°", text_color),
            (270.0, "270°", text_color),
        ];
        for (label_deg, label, color) in axis_labels {
            // Effective label angle: logical angle after mirror + rotation.
            // Mirror flips CW→CCW, so mirror negates the logical angle before adding rotation.
            let effective_rad = if mirror {
                (-label_deg.to_radians()) + rotation_rad
            } else {
                label_deg.to_radians() + rotation_rad
            };
            let dir = egui::vec2(effective_rad.sin(), -effective_rad.cos());
            painter.line_segment([center, center + dir * radius], egui::Stroke::new(1.0, axis_color));
            let label_pos = center + dir * (radius + 8.0);
            let align = dir_to_align(dir);
            painter.text(label_pos, align, label, egui::FontId::proportional(12.0), color);
        }

        let max_amp = max_amp_override.unwrap_or_else(|| {
            trail
                .iter()
                .flat_map(|pair| pair.iter().map(|vv| vv.amplitude))
                .chain(vectors.iter().map(|vv| vv.amplitude))
                .fold(1.0_f64, f64::max)
        });

        for (trail_idx, pair) in trail.iter().enumerate() {
            let alpha = ((trail_idx + 1) as f32 / trail.len().max(1) as f32 * 120.0) as u8;
            for channel in 0..2 {
                if !(channel_mode == channel || channel_mode == 2) || pair[channel].amplitude <= 0.0 {
                    continue;
                }
                let trail_radius = radius * (pair[channel].amplitude / max_amp) as f32;
                let end = vector_endpoint(center, trail_radius, pair[channel], rotation_rad, mirror);
                painter.line_segment(
                    [center, end],
                    egui::Stroke::new(
                        1.5,
                        egui::Color32::from_rgba_unmultiplied(
                            colors[channel].r(),
                            colors[channel].g(),
                            colors[channel].b(),
                            alpha,
                        ),
                    ),
                );
            }
        }

        for channel in 0..2 {
            if !(channel_mode == channel || channel_mode == 2) || vectors[channel].amplitude <= 0.0 {
                continue;
            }
            let vec_radius = radius * (vectors[channel].amplitude / max_amp) as f32;
            let end = vector_endpoint(center, vec_radius, vectors[channel], rotation_rad, mirror);
            painter.line_segment([center, end], egui::Stroke::new(3.0, colors[channel]));
            painter.circle_filled(end, 4.5, colors[channel]);
        }

        // Маркер позиции курсора внутри оборота: тонкая линия от центра до края кольца.
        if let Some(phase_rad) = cursor_rotor_phase {
            let dir = phase_to_screen_dir(phase_rad, rotation_rad, mirror);
            let tip = center + dir * radius;
            // Небольшой треугольный маркер: линия до края + маленький круг на нём.
            let cursor_color = egui::Color32::from_rgba_unmultiplied(255, 220, 50, 220);
            painter.line_segment([center, tip], egui::Stroke::new(1.5, cursor_color));
            painter.circle_filled(tip, 4.0, cursor_color);
            // Текстовая подпись фазы рядом с маркером.
            let label_pos = center + dir * (radius + 14.0);
            let phase_deg = phase_rad.to_degrees().rem_euclid(360.0);
            painter.text(
                label_pos,
                dir_to_align(dir),
                format!("{:.0}°", phase_deg),
                egui::FontId::proportional(11.0),
                cursor_color,
            );
        }

        ui.add_space(6.0);
        for channel in 0..2 {
            if !(channel_mode == channel || channel_mode == 2) || vectors[channel].amplitude <= 0.0 {
                continue;
            }

            let phase_deg = vectors[channel].phase.to_degrees().rem_euclid(360.0);
            let line = format!(
                "CH{}  A {}   φ {:.1}°",
                channel + 1,
                format_compact_value(vectors[channel].amplitude),
                phase_deg,
            );
            ui.colored_label(colors[channel], line);
        }
    });
}

pub(crate) fn draw_balance_hint(
    ui: &mut egui::Ui,
    vectors: [VibroVector; 2],
    channel_mode: usize,
    mass_g: f64,
    radius_mm: f64,
) {
    let colors = [
        egui::Color32::from_rgb(100, 200, 100),
        egui::Color32::from_rgb(100, 150, 255),
    ];

    ui.add_space(6.0);
    ui.label("Балансировочная подсказка:");
    ui.small("Фазовая оценка по текущему 1x-вектору, без influence coefficient.");

    let correction_moment = mass_g * radius_mm;
    for channel in 0..2 {
        if !(channel_mode == channel || channel_mode == 2) || vectors[channel].amplitude <= 0.0 {
            continue;
        }

        let suggested_deg = (vectors[channel].phase.to_degrees() + 180.0).rem_euclid(360.0);
        let line = format!(
            "CH{}  груз {} г @ R {:.0} мм -> ставить около {:.1}° KP  ({})",
            channel + 1,
            format_compact_value(mass_g),
            radius_mm,
            suggested_deg,
            format!("{} g*mm", format_compact_value(correction_moment)),
        );
        ui.colored_label(colors[channel], line);
    }
}

/// Choose egui text alignment based on which direction a label extends from center.
fn dir_to_align(dir: egui::Vec2) -> egui::Align2 {
    let horiz = if dir.x > 0.3 {
        egui::Align::LEFT
    } else if dir.x < -0.3 {
        egui::Align::RIGHT
    } else {
        egui::Align::Center
    };
    let vert = if dir.y < -0.3 {
        egui::Align::BOTTOM
    } else if dir.y > 0.3 {
        egui::Align::TOP
    } else {
        egui::Align::Center
    };
    egui::Align2([horiz, vert])
}
