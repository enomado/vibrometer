// Кастомный strip chart: синхронизированные по времени лейны,
// скроллинг, зум, выделение диапазона. Без egui_plot.
//
// Координатные пространства:
//   World  — секунды (X), единицы данных (Y, per-lane)
//   Screen — пиксели egui (Pos2, Rect)
//
// Transform переводит между ними. Один общий по X (время),
// per-lane по Y (каждый lane имеет свой YTransform).

use eframe::egui;
use egui::{
    Color32,
    CornerRadius,
    PointerButton,
    Pos2,
    Rect,
    Sense,
    Stroke,
    Vec2,
};

use crate::format::format_compact_value;

// ---------------------------------------------------------------------------
// Transform — перевод world ↔ screen
// ---------------------------------------------------------------------------

/// Аффинный 1D transform: world_range ↔ screen_range.
/// world = offset + value * scale, screen = pixel.
#[derive(Clone, Copy, Debug)]
struct Axis {
    /// Начало видимого диапазона в мировых координатах.
    world_min: f64,
    /// Конец видимого диапазона.
    world_max: f64,
    /// Начало в пикселях.
    screen_min: f32,
    /// Конец в пикселях.
    screen_max: f32,
}

impl Axis {
    fn new(world_min: f64, world_max: f64, screen_min: f32, screen_max: f32) -> Self {
        Self { world_min, world_max, screen_min, screen_max }
    }

    /// Мировая длительность видимого диапазона.
    fn world_range(&self) -> f64 {
        (self.world_max - self.world_min).max(1e-12)
    }

    /// Пиксельная длина.
    fn screen_range(&self) -> f32 {
        (self.screen_max - self.screen_min).abs().max(1e-6)
    }

    /// Масштаб: пиксели / мировые единицы.
    fn scale(&self) -> f64 {
        self.screen_range() as f64 / self.world_range()
    }

    /// World → Screen.
    fn to_screen(&self, w: f64) -> f32 {
        let frac = (w - self.world_min) / self.world_range();
        self.screen_min + frac as f32 * (self.screen_max - self.screen_min)
    }

    /// Screen → World.
    fn to_world(&self, s: f32) -> f64 {
        let frac = (s - self.screen_min) as f64 / (self.screen_max - self.screen_min) as f64;
        self.world_min + frac * self.world_range()
    }

    /// Zoom вокруг мировой точки `center`. factor > 1 = zoom in.
    fn zoom(&mut self, factor: f64, center: f64) {
        let d = self.world_range();
        let new_d = d / factor;
        let ratio = (center - self.world_min) / d;
        self.world_min = center - ratio * new_d;
        self.world_max = self.world_min + new_d;
    }

    /// Pan на `delta` мировых единиц.
    fn pan(&mut self, delta: f64) {
        self.world_min += delta;
        self.world_max += delta;
    }

    /// Clamp мировых границ в пределы [lo, hi].
    fn clamp(&mut self, lo: f64, hi: f64) {
        let d = self.world_range().min(hi - lo);
        if self.world_min < lo {
            self.world_min = lo;
            self.world_max = lo + d;
        }
        if self.world_max > hi {
            self.world_max = hi;
            self.world_min = (hi - d).max(lo);
        }
    }
}

/// Полный 2D transform для одного lane: X (время) + Y (значение).
/// X axis общий для всех lanes, Y — per-lane.
struct Transform {
    x: Axis,
    y: Axis,
}

impl Transform {
    /// World (t, v) → Screen Pos2.
    fn to_screen(&self, t: f64, v: f64) -> Pos2 {
        Pos2::new(self.x.to_screen(t), self.y.to_screen(v))
    }

    /// Screen Pos2 → World (t, v).
    fn to_world(&self, pos: Pos2) -> (f64, f64) {
        (self.x.to_world(pos.x), self.y.to_world(pos.y))
    }
}

// ---------------------------------------------------------------------------
// Selection
// ---------------------------------------------------------------------------

/// Выделенный пользователем диапазон (секунды).
#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub(crate) struct Selection {
    pub(crate) t_from: f64,
    pub(crate) t_to: f64,
}

// ---------------------------------------------------------------------------
// StripChart — основной виджет
// ---------------------------------------------------------------------------

/// Per-lane Y viewport state.
struct YView {
    /// Центр видимого Y-диапазона (мировые координаты).
    center: f64,
    /// Полуширина видимого Y-диапазона.
    half_range: f64,
    /// Пользователь вручную менял Y (drag/zoom) — не перезаписывать center из данных.
    user_touched: bool,
}

impl YView {
    /// Pan по Y на delta мировых единиц.
    fn pan(&mut self, delta: f64) {
        self.center += delta;
        self.user_touched = true;
    }

    /// Применить Y zoom вокруг мировой точки `y_center`.
    /// factor > 1 = zoom in (видим меньший диапазон).
    fn zoom(&mut self, factor: f64, y_center: f64) {
        let old_min = self.min();
        let old_max = self.max();
        let old_range = old_max - old_min;
        let new_range = (old_range / factor).max(1e-12);
        // Сохраняем пропорцию: y_center остаётся на том же месте экрана.
        let ratio = if old_range > 1e-15 { (y_center - old_min) / old_range } else { 0.5 };
        let new_min = y_center - ratio * new_range;
        self.center = new_min + new_range * 0.5;
        self.half_range = new_range * 0.5;
        self.user_touched = true;
    }

    fn min(&self) -> f64 { self.center - self.half_range }
    fn max(&self) -> f64 { self.center + self.half_range }

    /// Обновить из данных (auto-fit). Сбрасывает user_touched.
    fn fit(&mut self, data_min: f64, data_max: f64) {
        self.center = (data_min + data_max) * 0.5;
        self.half_range = ((data_max - data_min) * 0.5).max(1e-12);
        self.user_touched = false;
    }
}

/// Состояние strip chart (персистентное между фреймами).
pub(crate) struct StripChart {
    /// Видимый диапазон по времени (X).
    x_min: f64,
    x_max: f64,
    /// Полная длительность данных.
    x_total: f64,
    /// Автоследование за правым краем во время live-записи.
    auto_follow_x: bool,
    /// Per-lane Y viewport.
    y_views: Vec<YView>,
    /// Выделенный диапазон.
    pub(crate) selection: Option<Selection>,
    /// Идёт ли drag для выделения (Shift+drag).
    selecting: bool,
    /// Начало drag-выделения (секунды).
    sel_anchor: f64,
    /// Последнее время под hover-курсором (секунды от начала данных).
    /// Не сбрасывается при уходе мыши с графика, чтобы зависимые view сохраняли состояние.
    pub(crate) cursor_t: Option<f64>,
    /// Закреплённое время курсора по клику.
    pub(crate) locked_cursor_t: Option<f64>,
}

impl StripChart {
    pub(crate) fn new(t_total: f64) -> Self {
        Self {
            x_min: 0.0,
            x_max: t_total,
            x_total: t_total,
            auto_follow_x: true,
            y_views: Vec::new(),
            selection: None,
            selecting: false,
            sel_anchor: 0.0,
            cursor_t: None,
            locked_cursor_t: None,
        }
    }

    /// Обновить t_total (данные растут во время записи).
    pub(crate) fn set_total(&mut self, t_total: f64) {
        self.x_total = t_total;
    }

    /// Показать весь диапазон по X и сбросить Y viewport'ы (re-fit из данных).
    pub(crate) fn fit_all(&mut self) {
        self.fit_x();
        // Сброс Y: вернуть в default-состояние → auto-fit на следующем фрейме.
        for yv in &mut self.y_views {
            yv.center = 0.0;
            yv.half_range = 1.0;
            yv.user_touched = false;
        }
    }

    /// Показать весь диапазон по X, Y viewport'ы не трогать.
    pub(crate) fn fit_x(&mut self) {
        self.x_min = 0.0;
        self.x_max = self.x_total;
        self.auto_follow_x = true;
    }

    /// Автоскролл к правому краю (для live recording).
    pub(crate) fn follow_end(&mut self) {
        let d = self.x_duration();
        self.x_max = self.x_total;
        self.x_min = (self.x_total - d).max(0.0);
        self.auto_follow_x = true;
    }

    pub(crate) fn should_auto_follow_x(&self) -> bool {
        self.auto_follow_x
    }

    fn x_duration(&self) -> f64 {
        (self.x_max - self.x_min).max(1e-9)
    }

    fn zoom_x(&mut self, factor: f64, t_center: f64) {
        let d = self.x_duration();
        // Максимальный zoom-out = 2× от полного диапазона (чтобы не уехать в бесконечность).
        let new_d = (d / factor).clamp(1e-6, self.x_total * 2.0);
        let ratio = (t_center - self.x_min) / d;
        self.x_min = t_center - ratio * new_d;
        self.x_max = self.x_min + new_d;
        self.auto_follow_x = false;
        // Не кламим — viewport может выходить за пределы данных.
    }

    fn pan_x(&mut self, dt: f64) {
        self.x_min += dt;
        self.x_max += dt;
        if dt.abs() > f64::EPSILON {
            self.auto_follow_x = false;
        }
        // Не кламим — пусть viewport свободно скроллится за пределы данных.
    }

    fn clamp_x(&mut self) {
        let d = self.x_duration().min(self.x_total);
        if self.x_min < 0.0 {
            self.x_min = 0.0;
            self.x_max = d;
        }
        if self.x_max > self.x_total {
            self.x_max = self.x_total;
            self.x_min = (self.x_total - d).max(0.0);
        }
    }

    /// Гарантировать что y_views.len() >= n, дополняя из data ranges.
    fn ensure_y_views(&mut self, n: usize) {
        while self.y_views.len() < n {
            self.y_views.push(YView { center: 0.0, half_range: 1.0, user_touched: false });
        }
    }

    /// Построить X Axis для текущего состояния.
    fn x_axis(&self, rect: Rect) -> Axis {
        Axis::new(self.x_min, self.x_max, rect.left(), rect.right())
    }

    pub(crate) fn active_cursor_t(&self) -> Option<f64> {
        self.locked_cursor_t.or(self.cursor_t)
    }
}

// ---------------------------------------------------------------------------
// Lane
// ---------------------------------------------------------------------------

/// Данные одного lane для отрисовки.
pub(crate) struct LaneData<'a> {
    pub(crate) label: &'a str,
    pub(crate) color: Color32,
    /// Y-диапазон данных (min, max). Используется для auto-fit.
    pub(crate) y_range: Option<(f64, f64)>,
    /// Отрисовка содержимого lane.
    pub(crate) draw: Box<dyn FnOnce(&LanePainter) + 'a>,
    /// Индекс в y_views[] (стабильный, не зависит от фильтрации видимых lane'ов).
    pub(crate) y_view_idx: usize,
}

/// Контекст для рисования внутри одного lane.
pub(crate) struct LanePainter<'a> {
    pub(crate) painter: &'a egui::Painter,
    pub(crate) rect: Rect,
    pub(crate) tr: Transform,
}

impl<'a> LanePainter<'a> {
    /// World time → screen X.
    pub(crate) fn t_to_x(&self, t: f64) -> f32 {
        self.tr.x.to_screen(t)
    }

    /// World value → screen Y.
    pub(crate) fn y_to_px(&self, y: f64) -> f32 {
        self.tr.y.to_screen(y)
    }

    /// Нарисовать линейный график: (t, y).
    /// Decimation при zoomout для сохранения пиков.
    pub(crate) fn line(&self, points: &[(f64, f64)], color: Color32, width: f32) {
        if points.len() < 2 {
            return;
        }
        let t_lo = self.tr.x.world_min;
        let t_hi = self.tr.x.world_max;

        let start = points
            .partition_point(|p| p.0 < t_lo)
            .saturating_sub(1);
        let end = points
            .partition_point(|p| p.0 <= t_hi)
            .min(points.len());

        if start >= end {
            return;
        }

        let n_visible = end - start;
        let px_width = self.rect.width() as usize;
        if px_width == 0 {
            return;
        }
        let step = (n_visible / (px_width * 2)).max(1);

        let stroke = Stroke::new(width, color);
        let slice = &points[start..end];

        let mut prev_pos: Option<Pos2> = None;
        for chunk in slice.chunks(step) {
            let (t_first, mut y_min, mut y_max) = (chunk[0].0, chunk[0].1, chunk[0].1);
            let mut t_last = t_first;
            let mut y_last = chunk[0].1;
            for &(t, y) in chunk {
                if y < y_min { y_min = y; }
                if y > y_max { y_max = y; }
                t_last = t;
                y_last = y;
            }

            if step > 1 {
                let x = self.t_to_x((t_first + t_last) * 0.5);
                let py_min = self.y_to_px(y_min);
                let py_max = self.y_to_px(y_max);
                if let Some(p) = prev_pos {
                    self.painter.line_segment([p, Pos2::new(x, py_max)], stroke);
                }
                if (py_min - py_max).abs() > 0.5 {
                    self.painter
                        .line_segment([Pos2::new(x, py_max), Pos2::new(x, py_min)], stroke);
                }
                prev_pos = Some(Pos2::new(x, py_min));
            } else {
                let pos = Pos2::new(self.t_to_x(t_last), self.y_to_px(y_last));
                if let Some(p) = prev_pos {
                    self.painter.line_segment([p, pos], stroke);
                }
                prev_pos = Some(pos);
            }
        }
    }

    /// Ступенчатый график: (t_start, t_end, y).
    pub(crate) fn step_line(&self, steps: &[(f64, f64, f64)], color: Color32, width: f32) {
        let stroke = Stroke::new(width, color);
        let t_lo = self.tr.x.world_min;
        let t_hi = self.tr.x.world_max;

        let mut prev_end: Option<Pos2> = None;
        for &(t0, t1, y) in steps {
            if t1 < t_lo || t0 > t_hi {
                prev_end = Some(Pos2::new(self.t_to_x(t1), self.y_to_px(y)));
                continue;
            }

            let x0 = self.t_to_x(t0);
            let x1 = self.t_to_x(t1);
            let py = self.y_to_px(y);

            if let Some(p) = prev_end {
                self.painter.line_segment([p, Pos2::new(x0, py)], stroke);
            }
            self.painter
                .line_segment([Pos2::new(x0, py), Pos2::new(x1, py)], stroke);

            prev_end = Some(Pos2::new(x1, py));
        }
    }

    /// Вертикальные штрихи в мировых координатах: от y_lo до y_hi.
    pub(crate) fn vlines(&self, times: &[f64], y_lo: f64, y_hi: f64, color: Color32, width: f32) {
        let stroke = Stroke::new(width, color);
        let t_lo = self.tr.x.world_min;
        let t_hi = self.tr.x.world_max;
        let py_lo = self.y_to_px(y_lo);
        let py_hi = self.y_to_px(y_hi);

        for &t in times {
            if t < t_lo || t > t_hi {
                continue;
            }
            let x = self.t_to_x(t);
            self.painter
                .line_segment([Pos2::new(x, py_lo), Pos2::new(x, py_hi)], stroke);
        }
    }

    /// Вертикальные штрихи со смещением в пикселях (для визуального разделения).
    /// `py_offset` < 0 = выше, > 0 = ниже.
    pub(crate) fn vlines_offset(
        &self,
        times: &[f64],
        y_lo: f64,
        y_hi: f64,
        color: Color32,
        width: f32,
        py_offset: f32,
    ) {
        let stroke = Stroke::new(width, color);
        let t_lo = self.tr.x.world_min;
        let t_hi = self.tr.x.world_max;
        let py_lo = self.y_to_px(y_lo) + py_offset;
        let py_hi = self.y_to_px(y_hi) + py_offset;

        for &t in times {
            if t < t_lo || t > t_hi {
                continue;
            }
            let x = self.t_to_x(t);
            self.painter
                .line_segment([Pos2::new(x, py_lo), Pos2::new(x, py_hi)], stroke);
        }
    }

    /// Конвертация экранной доли (0.0 = bottom, 1.0 = top) в мировую Y координату.
    /// Используется для привязки элементов к пиксельной позиции на экране,
    /// а не к мировым данным (например, KP штрихи всегда внизу lane).
    pub(crate) fn screen_frac_to_y(&self, frac: f32) -> f64 {
        // frac=0 → bottom → y_min, frac=1 → top → y_max
        self.tr.y.world_min + frac as f64 * self.tr.y.world_range()
    }

    /// Горизонтальная линия на заданном Y-значении (на всю ширину lane).
    pub(crate) fn hline(&self, y: f64, color: Color32, width: f32) {
        let py = self.y_to_px(y);
        if py >= self.rect.top() && py <= self.rect.bottom() {
            self.painter.line_segment(
                [Pos2::new(self.rect.left(), py), Pos2::new(self.rect.right(), py)],
                Stroke::new(width, color),
            );
        }
    }

    /// Легенда в правом верхнем углу лейна: цветные подписи.
    pub(crate) fn legend(&self, entries: &[(&str, Color32)]) {
        let font = egui::FontId::proportional(11.0);
        let mut y = self.rect.top() + 4.0;
        let x = self.rect.right() - 6.0;
        for &(label, color) in entries {
            self.painter.text(
                Pos2::new(x, y),
                egui::Align2::RIGHT_TOP,
                label,
                font.clone(),
                color,
            );
            y += 14.0;
        }
    }
}

// ---------------------------------------------------------------------------
// Rendering
// ---------------------------------------------------------------------------

const LANE_MIN_HEIGHT: f32 = 60.0;
const TIME_AXIS_HEIGHT: f32 = 24.0;
const LABEL_WIDTH: f32 = 48.0;
const GRID_COLOR: Color32 = Color32::from_rgb(40, 40, 45);
const SELECTION_COLOR: Color32 = Color32::from_rgba_premultiplied(20, 40, 80, 9);
const SELECTION_BORDER: Color32 = Color32::from_rgba_premultiplied(35, 65, 128, 50);
const AXIS_COLOR: Color32 = Color32::from_rgb(120, 120, 130);

pub(crate) fn show_strip_chart(
    ui: &mut egui::Ui,
    chart: &mut StripChart,
    lanes: Vec<LaneData<'_>>,
) -> Rect {
    let n_lanes = lanes.len().max(1);
    let available = ui.available_size();
    let total_height = available.y.max(LANE_MIN_HEIGHT * n_lanes as f32 + TIME_AXIS_HEIGHT);
    let lane_height = (total_height - TIME_AXIS_HEIGHT) / n_lanes as f32;

    let (response, painter) =
        ui.allocate_painter(Vec2::new(available.x, total_height), Sense::click_and_drag());
    let full_rect = response.rect;

    // Область графиков (без временной оси снизу и label слева).
    let chart_rect = Rect::from_min_max(
        Pos2::new(full_rect.left() + LABEL_WIDTH, full_rect.top()),
        Pos2::new(full_rect.right(), full_rect.bottom() - TIME_AXIS_HEIGHT),
    );

    // y_views индексируются по y_view_idx (стабильный индекс), а не по позиции в lanes.
    let max_yv_idx = lanes.iter().map(|l| l.y_view_idx).max().unwrap_or(0);
    chart.ensure_y_views(max_yv_idx + 1);

    // Lane rects.
    let lane_rects: Vec<Rect> = (0..n_lanes)
        .map(|i| {
            let top = chart_rect.top() + lane_height * i as f32;
            Rect::from_min_max(
                Pos2::new(chart_rect.left(), top),
                Pos2::new(chart_rect.right(), top + lane_height),
            )
        })
        .collect();

    // Маппинг визуальной позиции lane → индекс в y_views[].
    let yv_indices: Vec<usize> = lanes.iter().map(|l| l.y_view_idx).collect();

    // --- Input ---
    handle_input(ui, chart, &response, chart_rect, &lane_rects, &yv_indices);

    // --- Background ---
    painter.rect_filled(full_rect, CornerRadius::ZERO, Color32::from_rgb(18, 18, 22));

    // --- Lanes ---
    let x_axis = chart.x_axis(chart_rect);

    for (i, lane) in lanes.into_iter().enumerate() {
        let lane_rect = lane_rects[i];

        // Разделитель.
        if i > 0 {
            painter.line_segment(
                [Pos2::new(full_rect.left(), lane_rect.top()), Pos2::new(full_rect.right(), lane_rect.top())],
                Stroke::new(1.0, GRID_COLOR),
            );
        }

        // Label.
        let label_rect = Rect::from_min_max(
            Pos2::new(full_rect.left(), lane_rect.top()),
            Pos2::new(chart_rect.left(), lane_rect.bottom()),
        );
        painter.text(
            label_rect.center(),
            egui::Align2::CENTER_CENTER,
            lane.label,
            egui::FontId::proportional(11.0),
            lane.color,
        );

        // Y auto-fit: однократно при первом появлении данных (is_default).
        // После первого fit или ручного зума — Y фиксируется.
        // Сброс через "Fit all".
        let yvi = lane.y_view_idx;
        if let Some((data_min, data_max)) = lane.y_range {
            let yv = &mut chart.y_views[yvi];
            let is_default = yv.center == 0.0 && yv.half_range == 1.0;
            if is_default {
                yv.fit(data_min, data_max);
            }
        }

        let yv = &chart.y_views[yvi];
        // Y axis: screen_min = bottom (min value), screen_max = top (max value).
        // Инвертируем: больше Y = выше на экране = меньший screen Y.
        let y_axis = Axis::new(yv.min(), yv.max(), lane_rect.bottom(), lane_rect.top());

        // Y ticks.
        draw_y_ticks(&painter, label_rect, yv.min(), yv.max());

        let tr = Transform { x: x_axis, y: y_axis };

        // Clip painter to lane rect — линии не вылезают за границу lane.
        let clipped = painter.with_clip_rect(lane_rect);

        let lp = LanePainter {
            painter: &clipped,
            rect: lane_rect,
            tr,
        };

        // Selection overlay.
        if let Some(ref sel) = chart.selection {
            let x0 = lp.t_to_x(sel.t_from);
            let x1 = lp.t_to_x(sel.t_to);
            let sel_rect = Rect::from_x_y_ranges(x0.min(x1)..=x0.max(x1), lane_rect.y_range());
            clipped.rect_filled(sel_rect, CornerRadius::ZERO, SELECTION_COLOR);
            clipped.line_segment(
                [Pos2::new(x0.min(x1), lane_rect.top()), Pos2::new(x0.min(x1), lane_rect.bottom())],
                Stroke::new(1.0, SELECTION_BORDER),
            );
            clipped.line_segment(
                [Pos2::new(x0.max(x1), lane_rect.top()), Pos2::new(x0.max(x1), lane_rect.bottom())],
                Stroke::new(1.0, SELECTION_BORDER),
            );
        }

        (lane.draw)(&lp);

        // Cursor — hover-курсор или закреплённая вертикальная линия по клику.
        if let Some(ct) = chart.active_cursor_t() {
            let cx = lp.t_to_x(ct);
            clipped.line_segment(
                [Pos2::new(cx, lane_rect.top()), Pos2::new(cx, lane_rect.bottom())],
                Stroke::new(1.0, Color32::from_rgba_unmultiplied(255, 255, 255, 80)),
            );
        }
    }

    // --- Time axis ---
    draw_time_axis(&painter, chart_rect, &x_axis);

    full_rect
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------

fn handle_input(
    ui: &egui::Ui,
    chart: &mut StripChart,
    response: &egui::Response,
    chart_rect: Rect,
    lane_rects: &[Rect],
    yv_indices: &[usize],
) {
    // Читаем raw scroll events — они не перехватываются Ctrl+zoom системным.
    let (raw_scroll_y, pointer_pos, modifiers) = ui.input(|i| {
        let mut scroll = 0.0_f32;
        for event in &i.events {
            if let egui::Event::MouseWheel { delta, modifiers, .. } = event {
                // Ловим только если модификатор нужный или без модификатора.
                // Главное — raw event не перехватывается egui zoom.
                let _ = modifiers;
                scroll += delta.y;
            }
        }
        (scroll, i.pointer.hover_pos(), i.modifiers)
    });

    // Обновляем hover cursor_t только когда указатель реально над графиком.
    // Последнее значение сохраняем, чтобы связанные панели (например, Polar)
    // не пропадали сразу после выхода мыши за пределы chart.
    if response.hovered() {
        if let Some(cursor_t) = pointer_pos
            .filter(|p| chart_rect.contains(*p))
            .map(|p| chart.x_axis(chart_rect).to_world(p.x))
        {
            chart.cursor_t = Some(cursor_t);
        }
    }

    // Одинарный правый клик по графику переключает фиксацию вертикального курсора:
    // если курсор не зафиксирован — фиксируем в точке клика,
    // если уже зафиксирован — снимаем фиксацию повторным кликом.
    if response.clicked_by(PointerButton::Secondary)
        && !response.double_clicked_by(PointerButton::Secondary)
    {
        if let Some(pos) = response.interact_pointer_pos().filter(|p| chart_rect.contains(*p)) {
            chart.locked_cursor_t = if chart.locked_cursor_t.is_some() {
                None
            } else {
                Some(chart.x_axis(chart_rect).to_world(pos.x))
            };
        }
    }

    if response.hovered() && raw_scroll_y.abs() > 0.01 {
        let pointer = pointer_pos.unwrap_or(chart_rect.center());

        if modifiers.ctrl || modifiers.command {
            // Ctrl+scroll → Y zoom на lane под курсором, вокруг точки мыши.
            if let Some(lane_idx) = lane_rects.iter().position(|r| r.contains(pointer)) {
                let yvi = yv_indices[lane_idx];
                let factor = (1.0 + raw_scroll_y as f64 * 0.1).max(0.1);
                let lr = lane_rects[lane_idx];
                let yv = &chart.y_views[yvi];
                // Screen Y → world Y (инвертировано: top = max, bottom = min).
                let y_axis = Axis::new(yv.min(), yv.max(), lr.bottom(), lr.top());
                let y_world = y_axis.to_world(pointer.y);
                chart.y_views[yvi].zoom(factor, y_world);
            }
        } else {
            // Обычный scroll → zoom по X.
            let x_axis = chart.x_axis(chart_rect);
            let t_center = x_axis.to_world(pointer.x);
            let factor = 1.0 + raw_scroll_y as f64 * 0.1;
            chart.zoom_x(factor, t_center);
        }
    }

    // Drag.
    if response.dragged() {
        let delta_x = response.drag_delta().x;

        if modifiers.shift {
            // Shift+drag → selection.
            if !chart.selecting {
                chart.selecting = true;
                let x_axis = chart.x_axis(chart_rect);
                let start_x = ui
                    .input(|i| i.pointer.press_origin())
                    .map(|p| p.x)
                    .unwrap_or(chart_rect.center().x);
                chart.sel_anchor = x_axis.to_world(start_x);
            }
            let x_axis = chart.x_axis(chart_rect);
            let current_x = pointer_pos
                .map(|p| p.x)
                .unwrap_or(chart_rect.center().x);
            let t_now = x_axis.to_world(current_x);
            let (a, b) = if chart.sel_anchor < t_now {
                (chart.sel_anchor, t_now)
            } else {
                (t_now, chart.sel_anchor)
            };
            chart.selection = Some(Selection { t_from: a, t_to: b });
        } else {
            // Pan по X.
            let dt = -(delta_x as f64) / chart_rect.width() as f64 * chart.x_duration();
            chart.pan_x(dt);

            // Pan по Y на lane под курсором.
            let delta_y = response.drag_delta().y;
            if delta_y.abs() > 0.01 {
                let pointer = pointer_pos.unwrap_or(chart_rect.center());
                if let Some(lane_idx) = lane_rects.iter().position(|r| r.contains(pointer)) {
                    let yvi = yv_indices[lane_idx];
                    let yv = &chart.y_views[yvi];
                    let lr = lane_rects[lane_idx];
                    // delta_y в пикселях → мировые единицы.
                    // Y инвертирован (top = max), поэтому + delta_y = pan вверх по Y.
                    let dy = delta_y as f64 / lr.height() as f64 * (yv.half_range * 2.0);
                    chart.y_views[yvi].pan(dy);
                }
            }
        }
    }

    if response.drag_stopped() {
        chart.selecting = false;
    }

    // Double-click → fit all.
    if response.double_clicked() {
        chart.fit_all();
        chart.locked_cursor_t = None;
        // Reset Y zoom.
        for yv in &mut chart.y_views {
            // Mark as default so auto-fit kicks in.
            yv.center = 0.0;
            yv.half_range = 1.0;
            yv.user_touched = false;
        }
    }

    // Shift+click (без drag) или Escape → снять выделение.
    if chart.selection.is_some() {
        let shift_click = response.clicked() && modifiers.shift;
        let escape = ui.input(|i| i.key_pressed(egui::Key::Escape));
        if shift_click || escape {
            chart.selection = None;
        }
    }
}

// ---------------------------------------------------------------------------
// Time axis
// ---------------------------------------------------------------------------

fn draw_time_axis(painter: &egui::Painter, chart_rect: Rect, x_axis: &Axis) {
    let axis_top = chart_rect.bottom();

    painter.line_segment(
        [Pos2::new(chart_rect.left(), axis_top), Pos2::new(chart_rect.right(), axis_top)],
        Stroke::new(1.0, AXIS_COLOR),
    );

    let duration = x_axis.world_range();
    let target_ticks = (chart_rect.width() / 100.0).max(2.0) as f64;
    let raw_step = duration / target_ticks;
    let step = nice_step(raw_step);

    let first_tick = (x_axis.world_min / step).ceil() * step;
    let mut t = first_tick;
    while t <= x_axis.world_max {
        let x = x_axis.to_screen(t);
        painter.line_segment(
            [Pos2::new(x, axis_top), Pos2::new(x, axis_top + 4.0)],
            Stroke::new(1.0, AXIS_COLOR),
        );
        painter.line_segment(
            [Pos2::new(x, chart_rect.top()), Pos2::new(x, chart_rect.bottom())],
            Stroke::new(0.5, GRID_COLOR),
        );
        let label = format_time(t, step);
        painter.text(
            Pos2::new(x, axis_top + 6.0),
            egui::Align2::CENTER_TOP,
            label,
            egui::FontId::monospace(10.0),
            AXIS_COLOR,
        );
        t += step;
    }
}

/// Y-axis тики.
fn draw_y_ticks(painter: &egui::Painter, label_rect: Rect, y_min: f64, y_max: f64) {
    let font = egui::FontId::monospace(9.0);
    let color = Color32::from_rgb(90, 90, 100);

    painter.text(
        Pos2::new(label_rect.right() - 2.0, label_rect.top() + 2.0),
        egui::Align2::RIGHT_TOP,
        format_compact_value(y_max),
        font.clone(),
        color,
    );
    painter.text(
        Pos2::new(label_rect.right() - 2.0, label_rect.bottom() - 2.0),
        egui::Align2::RIGHT_BOTTOM,
        format_compact_value(y_min),
        font,
        color,
    );
}

// ---------------------------------------------------------------------------
// Formatting
// ---------------------------------------------------------------------------

fn format_time(t: f64, step: f64) -> String {
    if step < 0.01 {
        format!("{:.1}ms", t * 1000.0)
    } else if step < 0.1 {
        format!("{:.2}s", t)
    } else if step < 1.0 {
        format!("{:.1}s", t)
    } else if step < 60.0 {
        format!("{:.0}s", t)
    } else {
        let m = (t / 60.0).floor() as u32;
        let s = t - m as f64 * 60.0;
        format!("{m}:{s:04.1}")
    }
}

fn nice_step(raw: f64) -> f64 {
    let exp = raw.log10().floor();
    let base = 10.0_f64.powf(exp);
    let frac = raw / base;

    let nice = if frac <= 1.5 {
        1.0
    } else if frac <= 3.5 {
        2.0
    } else if frac <= 7.5 {
        5.0
    } else {
        10.0
    };

    nice * base
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nice_step_picks_round_values() {
        assert_eq!(nice_step(0.07), 0.05);
        assert_eq!(nice_step(0.12), 0.1);
        assert_eq!(nice_step(0.3), 0.2);
        assert_eq!(nice_step(0.6), 0.5);
        assert_eq!(nice_step(3.0), 2.0);
        assert_eq!(nice_step(8.0), 10.0);
        assert_eq!(nice_step(18.0), 20.0);
    }

    #[test]
    fn axis_roundtrip() {
        let ax = Axis::new(1.0, 5.0, 100.0, 500.0);
        let screen = ax.to_screen(3.0);
        let world = ax.to_world(screen);
        assert!((world - 3.0).abs() < 1e-9);
    }

    #[test]
    fn axis_zoom_preserves_center() {
        let mut ax = Axis::new(0.0, 10.0, 0.0, 100.0);
        ax.zoom(2.0, 5.0);
        // Диапазон должен уменьшиться вдвое, центр ~5.
        assert!((ax.world_range() - 5.0).abs() < 1e-9);
        let mid = (ax.world_min + ax.world_max) * 0.5;
        assert!((mid - 5.0).abs() < 1e-9);
    }

    #[test]
    fn chart_pan_x_free() {
        let mut chart = StripChart::new(10.0);
        chart.x_min = 0.0;
        chart.x_max = 5.0;
        chart.pan_x(-100.0);
        assert_eq!(chart.x_min, -100.0);
        assert_eq!(chart.x_max, -95.0);
        chart.pan_x(200.0);
        assert_eq!(chart.x_min, 100.0);
        assert_eq!(chart.x_max, 105.0);
    }

    #[test]
    fn y_view_zoom() {
        let mut yv = YView { center: 100.0, half_range: 50.0, user_touched: false };
        // Zoom in ×2 вокруг центра — half_range уменьшается вдвое.
        yv.zoom(2.0, 100.0);
        assert!((yv.half_range - 25.0).abs() < 1e-9);
        assert!((yv.center - 100.0).abs() < 1e-9);
    }
}
