mod analysis;
mod format;
mod recording;
mod sound;
mod strip;
mod tcp;
mod ui_polar;


use std::sync::{
    Arc,
    Mutex,
};

use eframe::egui;
use vibro_analysis::signal::pll::{
    PllParams,
    PllPoint,
    aligned_kp_times,
    run_pll,
};
use vibro_analysis::signal::spline::{
    smoothing_spline,
    spline_to_pll,
};
use vibro_protocol::{
    Command,
    Sample,
};
use vibro_types::SampleRateHz;

use crate::recording::{
    ConnStatus,
    RecordMode,
    Shared,
    finish_recording,
    load_all_recordings,
};
use crate::analysis::{
    order_track,
    smooth_order_points,
};
use crate::strip::{
    LaneData,
    StripChart,
    show_strip_chart,
};
use crate::tcp::tcp_listener_thread;

// ---------------------------------------------------------------------------
// ADC options
// ---------------------------------------------------------------------------

const PGA_OPTIONS: &[u8] = &[1, 2, 4, 8, 16, 32, 64];

const DATA_RATE_OPTIONS: &[SampleRateHz] = &[
    SampleRateHz(30000),
    SampleRateHz(15000),
    SampleRateHz(7500),
    SampleRateHz(3750),
    SampleRateHz(2000),
    SampleRateHz(1000),
    SampleRateHz(500),
    SampleRateHz(100),
    SampleRateHz(60),
    SampleRateHz(50),
    SampleRateHz(30),
    SampleRateHz(25),
    SampleRateHz(15),
    SampleRateHz(10),
    SampleRateHz(5),
];

use crate::recording::SYSTIMER_HZ;

// ---------------------------------------------------------------------------
// Tacho data: RPM ступеньки из keyphasor-импульсов
// ---------------------------------------------------------------------------

/// Один оборот: начало, конец (секунды), мгновенная RPM.
struct TachoStep {
    t_start: f64,
    t_end: f64,
    rpm: f64,
}

/// Конвертировать абсолютные kp_ticks в позиции на timeline (секунды от начала данных).
/// Фильтрует тики, попадающие в окно [start_tick, end_tick).
fn kp_ticks_to_times(kp_ticks: &[u64], start_tick: u64, end_tick: u64) -> Vec<f64> {
    kp_ticks
        .iter()
        .filter(|&&t| t >= start_tick && t < end_tick)
        .map(|&t| (t - start_tick) as f64 / SYSTIMER_HZ)
        .collect()
}

/// Вычислить угол ротора в точке cursor_t (секунды от start_tick).
/// Находит два соседних KP-тика, возвращает фазу 0..2π между ними.
/// Возвращает None если cursor_t не попадает между двумя KP.
fn cursor_rotor_phase(cursor_t: f64, kp_ticks: &[u64], start_tick: u64) -> Option<f64> {
    if kp_ticks.len() < 2 {
        return None;
    }
    // Перевод cursor_t в абсолютные тики.
    let cursor_tick = (cursor_t * SYSTIMER_HZ) as u64 + start_tick;
    // Бинарный поиск: найти индекс первого KP > cursor_tick.
    let idx_after = kp_ticks.partition_point(|&t| t <= cursor_tick);
    if idx_after == 0 || idx_after >= kp_ticks.len() {
        return None;
    }
    let t_before = kp_ticks[idx_after - 1];
    let t_after  = kp_ticks[idx_after];
    let period = (t_after - t_before) as f64;
    if period < 1.0 {
        return None;
    }
    let phase = (cursor_tick - t_before) as f64 / period * std::f64::consts::TAU;
    Some(phase)
}

/// Группировка KEYPHASOR_LEVEL_FLAG в отрезки (t_start, t_end) секунды.
/// Используем hardware tick каждого сэмпла, чтобы спаны не плыли относительно KpEvent.
fn kp_flag_spans(samples: &[Sample], start_tick: u64) -> Vec<(f64, f64)> {
    let mut spans = Vec::new();
    let mut span_start: Option<usize> = None;

    for (i, s) in samples.iter().enumerate() {
        let level = s.keyphasor_level();
        match (level, span_start) {
            (true, None) => span_start = Some(i),
            (false, Some(start)) => {
                let t0 = (samples[start].tick - start_tick) as f64 / SYSTIMER_HZ;
                let t1 = (s.tick - start_tick) as f64 / SYSTIMER_HZ;
                spans.push((t0, t1));
                span_start = None;
            }
            _ => {}
        }
    }
    // Незакрытый span в конце данных.
    if let Some(start) = span_start {
        let t0 = (samples[start].tick - start_tick) as f64 / SYSTIMER_HZ;
        let t1 = samples
            .last()
            .map(|s| (s.tick - start_tick) as f64 / SYSTIMER_HZ)
            .unwrap_or(t0);
        spans.push((t0, t1));
    }
    spans
}

/// Извлечь ступеньки RPM из keyphasor-тиков.
/// Каждая ступенька — один оборот между двумя фронтами.
/// RPM = 60 / dt. Время — относительно start_tick.
fn tacho_steps(kp_ticks: &[u64], start_tick: u64, end_tick: u64) -> Vec<TachoStep> {
    // Фильтруем тики, попадающие в окно данных.
    let visible: Vec<u64> = kp_ticks
        .iter()
        .copied()
        .filter(|&t| t >= start_tick && t < end_tick)
        .collect();

    if visible.len() < 2 {
        return Vec::new();
    }

    let mut steps = Vec::with_capacity(visible.len() - 1);
    for pair in visible.windows(2) {
        let dt_ticks = pair[1] - pair[0];
        let dt = dt_ticks as f64 / SYSTIMER_HZ;
        let rpm = 60.0 / dt;
        let t_start = (pair[0] - start_tick) as f64 / SYSTIMER_HZ;
        let t_end = (pair[1] - start_tick) as f64 / SYSTIMER_HZ;
        steps.push(TachoStep { t_start, t_end, rpm });
    }
    steps
}

// ---------------------------------------------------------------------------
// App state
// ---------------------------------------------------------------------------

struct ViewData {
    samples:     Vec<Sample>,
    sample_rate: SampleRateHz,
    /// PGA при записи/live (для нормализации: raw / pga).
    pga:         u8,
    kp_ticks:    Vec<u64>,
}

impl ViewData {
    /// SystemTimer tick первого сэмпла (0 если пусто).
    fn start_tick(&self) -> u64 {
        self.samples.first().map_or(0, |s| s.tick)
    }

    /// SystemTimer tick последнего сэмпла (0 если пусто).
    fn end_tick(&self) -> u64 {
        self.samples.last().map_or(0, |s| s.tick)
    }
}

/// Состояние collapse/expand для секций side panel.
/// Сохраняется между сессиями через eframe::Storage.
#[derive(serde::Serialize, serde::Deserialize)]
struct SectionState {
    record:  bool,
    adc:     bool,
    chart:   bool,
    phase:   bool,
    polar:   bool,
}

impl Default for SectionState {
    fn default() -> Self {
        Self { record: true, adc: true, chart: true, phase: true, polar: true }
    }
}

struct VibroApp {
    shared:        Arc<Mutex<Shared>>,
    pga_idx:       usize,
    data_rate_idx: usize,
    /// Strip chart для записи. None если нет данных.
    chart:         StripChart,
    /// Какую запись смотрим (None = live/текущая).
    view_recording: Option<usize>,
    /// PLL smoothing (обороты). Больше → глаже RPM.
    pll_smoothing: f64,
    /// Whittaker lambda для spline-сглаживания.
    spline_lambda: f64,
    /// true = Whittaker spline, false = PLL.
    use_spline: bool,
    /// Усреднение 1x-вектора: количество оборотов скользящего среднего.
    smooth_1x: usize,
    /// Зеркалирование polar plot (отражение по оси X).
    polar_mirror: bool,
    /// Инверсия CH0 (отладка полярности датчика).
    invert_ch0: bool,
    /// Velocity transducer: зеркало фазы + 90° (velocity опережает displacement на π/2).
    velocity_sensor: bool,
    balance_mass_g: f64,
    balance_radius_mm: f64,
    /// Видимость lane'ов: [Tacho, CH0, 1xA, 1xΦ, 2xA, 2xΦ].
    lane_visible: [bool; 6],
    /// Collapse-стейт секций side panel (сохраняется между сессиями).
    sections: SectionState,
    /// Комментарии к записям: filename → comment text.
    /// Хранятся в recordings/comments.json, загружаются при старте.
    rec_comments: std::collections::HashMap<String, String>,
    /// Буфер редактирования комментария для текущей выбранной записи.
    /// Синхронизируется с rec_comments при смене выбора.
    editing_comment: String,
    /// Кеш списка записей: (имя без .parquet, filename).
    /// Пересчитывается только когда длина recordings изменилась.
    rec_list_cache: Vec<(String, String)>,
    /// Длина recordings на момент последнего пересчёта кеша.
    rec_list_cached_len: usize,
    /// Кеш средних 1x-векторов по selection-области: recording idx → VibroVector.
    /// Заполняется при просмотре записи (если есть selection и order_points).
    rec_mean_1x: std::collections::HashMap<usize, vibro_analysis::math::complex::VibroVector>,
    /// Длина recordings на предыдущем кадре — для детекции завершения записи.
    prev_rec_count: usize,
}

impl VibroApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let shared = Arc::new(Mutex::new(Shared::new()));

        // Загрузить ранее сохранённые записи с диска.
        let loaded = load_all_recordings();
        let last_idx = if loaded.is_empty() { None } else { Some(loaded.len() - 1) };
        shared.lock().unwrap().recordings = loaded;

        // Начальные желаемые настройки ADC — TCP-поток отправит их firmware при коннекте.
        {
            let mut sh = shared.lock().unwrap();
            sh.desired_pga = PGA_OPTIONS[0];
            sh.desired_rate = DATA_RATE_OPTIONS[4];
        }

        let shared_clone = shared.clone();
        let ctx = cc.egui_ctx.clone();
        std::thread::spawn(move || tcp_listener_thread(shared_clone, ctx));

        // Загрузить SectionState из eframe persistent storage.
        let sections = cc.storage
            .and_then(|s| s.get_string("section_state"))
            .and_then(|s| serde_json::from_str(&s).ok())
            .unwrap_or_default();

        // Загрузить комментарии к записям.
        let rec_comments = load_comments();

        // Инициализировать editing_comment из комментария к последней открытой записи.
        let n_loaded = shared.lock().unwrap().recordings.len();
        let editing_comment = last_idx
            .and_then(|i| {
                let sh = shared.lock().unwrap();
                sh.recordings.get(i).map(|r| {
                    rec_comments.get(&r.filename).cloned().unwrap_or_default()
                })
            })
            .unwrap_or_default();

        // Восстановить selection для последней записи из parquet metadata.
        let mut chart = StripChart::new(0.0);
        if let Some(idx) = last_idx {
            let sh = shared.lock().unwrap();
            if let Some(r) = sh.recordings.get(idx) {
                chart.selection = r.selection.clone();
            }
        }

        Self {
            shared,
            pga_idx: 0,
            data_rate_idx: 4,
            chart,
            view_recording: last_idx,
            pll_smoothing: 8.0,
            spline_lambda: 100.0,
            use_spline: false,
            smooth_1x: 1,
            polar_mirror: false,
            invert_ch0: false,
            velocity_sensor: false,
            balance_mass_g: 1.0,
            balance_radius_mm: 50.0,
            lane_visible: [true; 6],
            sections,
            rec_comments,
            editing_comment,
            rec_list_cache: Vec::new(),
            rec_list_cached_len: n_loaded + 1, // +1 чтобы первый кадр пересчитал кеш
            rec_mean_1x: std::collections::HashMap::new(),
            prev_rec_count: n_loaded,
        }
    }

    /// Сохранить текущее выделение в Recording (+ перезаписать parquet),
    /// затем переключиться на запись `new_idx` и восстановить её выделение.
    fn switch_recording(&mut self, new_idx: Option<usize>) {
        self.switch_recording_impl(new_idx, false);
    }

    /// Переключиться на запись, сохранив текущие Y-масштабы (для авто-перехода).
    fn switch_recording_keep_y(&mut self, new_idx: Option<usize>) {
        self.switch_recording_impl(new_idx, true);
    }

    fn switch_recording_impl(&mut self, new_idx: Option<usize>, keep_y: bool) {
        // Сохранить выделение текущей записи в Recording и на диск.
        self.flush_selection();

        self.view_recording = new_idx;
        // Восстановить выделение новой записи.
        if let Some(idx) = new_idx {
            let sh = self.shared.lock().unwrap();
            self.chart.selection = sh.recordings.get(idx).and_then(|r| r.selection.clone());
        } else {
            self.chart.selection = None;
        }

        if keep_y {
            // Только X fit — Y остаётся от предыдущей записи.
            self.chart.fit_x();
        } else {
            // Полный reset: X + Y.
            self.chart.fit_all();
        }
    }

    /// Записать текущее chart.selection в Recording и перезаписать parquet.
    fn flush_selection(&self) {
        if let Some(idx) = self.view_recording {
            let mut sh = self.shared.lock().unwrap();
            if let Some(rec) = sh.recordings.get_mut(idx) {
                let changed = rec.selection != self.chart.selection;
                if changed {
                    rec.selection = self.chart.selection.clone();
                    recording::resave_recording(rec);
                }
            }
        }
    }

    /// Данные для отображения: сэмплы, sample_rate, kp_ticks.
    /// start_tick/end_tick вычисляются из samples[0].tick / samples.last().tick.
    fn view_data(&self) -> Option<ViewData> {
        let sh = self.shared.lock().unwrap();
        match self.view_recording {
            None => {
                let sr = sh.sample_rate;
                if sr.as_f64() < 1.0 {
                    return None;
                }
                let samples = if sh.recording && !sh.rec_buf.is_empty() {
                    sh.rec_buf.clone()
                } else {
                    sh.live_buf.iter().copied().collect()
                };
                Some(ViewData {
                    samples,
                    sample_rate: sr,
                    pga: sh.pga,
                    kp_ticks: sh.kp_ticks.clone(),
                })
            }
            Some(idx) => {
                let rec = sh.recordings.get(idx)?;
                Some(ViewData {
                    samples: rec.samples.clone(),
                    sample_rate: rec.sample_rate,
                    pga: rec.pga,
                    kp_ticks: rec.kp_ticks.clone(),
                })
            }
        }
    }

    /// Вычислить сглаженные keyphasor-точки: PLL или Whittaker spline.
    /// Возвращает Vec<PllPoint> (единый формат для order_track и strip chart).
    fn compute_phase_points(&self, kp_ticks: &[u64]) -> Vec<PllPoint> {
        if kp_ticks.len() < 2 {
            return Vec::new();
        }
        if self.use_spline && kp_ticks.len() >= 3 {
            let spline = smoothing_spline(kp_ticks, self.spline_lambda);
            spline_to_pll(&spline)
        } else {
            let pll_params = PllParams {
                smoothing: self.pll_smoothing,
                damping: 0.707,
                systimer_hz: SYSTIMER_HZ,
            };
            run_pll(kp_ticks, &pll_params)
        }
    }
}

impl eframe::App for VibroApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        let json = serde_json::to_string(&self.sections).unwrap();
        storage.set_string("section_state", json);
        self.flush_selection();
    }

    // egui 0.35: App отдаёт корневой `Ui` вместо `Context`; панели показываются
    // внутрь этого ui (`Panel::show(ui, …)`), а не в контекст.
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        // --- Детекция новой записи: авто-переход с сохранением Y-масштабов ---
        {
            let cur_count = self.shared.lock().unwrap().recordings.len();
            if cur_count > self.prev_rec_count {
                // Новая запись добавлена → переключиться на неё, сохранив Y.
                let new_idx = cur_count - 1;
                self.switch_recording_keep_y(Some(new_idx));
                self.editing_comment.clear();
            }
            self.prev_rec_count = cur_count;
        }

        // --- Top panel: connection status ---
        egui::Panel::top("top_panel").show(ui, |ui| {
            ui.horizontal(|ui| {
                let sh = self.shared.lock().unwrap();
                match &sh.status {
                    ConnStatus::Listening => {
                        ui.colored_label(egui::Color32::YELLOW, "⏳ Listening on :7100");
                    }
                    ConnStatus::Connected(addr) => {
                        ui.colored_label(egui::Color32::GREEN, format!("● Connected: {addr}"));
                    }
                    ConnStatus::Disconnected(reason) => {
                        ui.colored_label(egui::Color32::RED, format!("✕ {reason}"));
                    }
                }
                ui.separator();
                let stream_status = match sh.last_packet_at {
                    Some(t) => {
                        let age_ms = t.elapsed().as_millis();
                        if age_ms < 500 {
                            format!("stream: live ({age_ms} ms)")
                        } else {
                            format!("stream: stale ({age_ms} ms)")
                        }
                    }
                    None => "stream: no packets yet".to_string(),
                };
                ui.label(format!(
                    "rate: {}Hz | samples: {} | kp: {} | seq: {} | {}",
                    sh.sample_rate, sh.total_samples, sh.keyphasor_count, sh.last_seq, stream_status
                ));
            });
        });

        // --- Pre-compute: один раз на кадр ---
        // ViewData, PLL/spline, order_track — используются и в side panel, и в central.
        let view_data = self.view_data();
        let pll_points: Vec<PllPoint> = view_data
            .as_ref()
            .filter(|vd| vd.kp_ticks.len() >= 2)
            .map(|vd| self.compute_phase_points(&vd.kp_ticks))
            .unwrap_or_default();
        let order_points: Vec<crate::analysis::OrderPoint> = view_data
            .as_ref()
            .filter(|vd| !vd.samples.is_empty() && pll_points.len() >= 2)
            .map(|vd| {
                let raw = order_track(
                    &vd.samples, &pll_points, vd.start_tick(), SYSTIMER_HZ,
                    vd.pga, self.invert_ch0, self.velocity_sensor,
                );
                smooth_order_points(raw, self.smooth_1x)
            })
            .unwrap_or_default();
        // kp_info для cursor_rotor_phase в polar view.
        let kp_info: Option<(&[u64], u64)> = view_data
            .as_ref()
            .filter(|vd| vd.kp_ticks.len() >= 2)
            .map(|vd| (vd.kp_ticks.as_slice(), vd.start_tick()));

        // Обновить кеш средних 1x для текущей записи (по selection области).
        if let Some(idx) = self.view_recording {
            if let Some(ref sel) = self.chart.selection {
                if !order_points.is_empty() {
                    if let Some(vv) = crate::analysis::mean_1x_in_range(&order_points, sel.t_from, sel.t_to) {
                        self.rec_mean_1x.insert(idx, vv);
                    }
                }
            } else {
                // Нет selection → убрать из кеша.
                self.rec_mean_1x.remove(&idx);
            }
        }

        // --- Left panel: Record + ADC ---
        egui::Panel::left("controls")
            .default_size(160.0)
            .show(ui, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {

                // Helper: рисует collapsible заголовок с нашим стейтом.
                // Возвращает true если секция открыта.
                // Используем egui::CollapsingHeader с open(…) для ручного управления стейтом.

                // ── Record ───────────────────────────────────────────────
                let hdr = egui::CollapsingHeader::new("Record")
                    .open(Some(self.sections.record))
                    .show(ui, |ui| {
                        // Record / Stop / Auto
                        {
                            let mut sh = self.shared.lock().unwrap();
                            if sh.recording {
                                let n = sh.rec_buf.len();
                                let sr = sh.sample_rate.as_f64().max(1.0);
                                let dur = n as f64 / sr;
                                let plus_label = if sh.auto_rec_plus { " [auto+]" }
                                    else if sh.auto_rec_last_kp_at.is_some() { " [auto]" }
                                    else { "" };
                                ui.label(format!("Recording: {n} samples ({dur:.1}s){plus_label}"));
                                if ui.button("⏹ Stop").clicked() {
                                    sh.auto_rec_plus = false;
                                    finish_recording(&mut sh);
                                }
                            } else if sh.auto_rec_armed {
                                let plus_label = if sh.auto_rec_plus { " (auto+)" } else { "" };
                                ui.label(format!("Waiting for KP…{plus_label}"));
                                if ui.button("✕ Cancel").clicked() {
                                    sh.auto_rec_armed = false;
                                    sh.auto_rec_plus = false;
                                }
                            } else {
                                ui.horizontal(|ui| {
                                    if ui.button("⏺ Record").clicked() {
                                        sh.recording = true;
                                        sh.record_mode = RecordMode::Continuous;
                                        sh.rec_buf.clear();
                                        sh.rec_captured_revs = 0;
                                        sh.rec_waiting_kp = false;
                                    }
                                    // Запуск по первому KP-фронту; авто-стоп через 2с без KP.
                                    if ui.button("⏺ Auto").on_hover_text("Start on first KP, stop after 2s idle").clicked() {
                                        sh.auto_rec_armed = true;
                                        sh.auto_rec_plus = false;
                                    }
                                    // Auto+: цикл записей — re-arm после каждого finish.
                                    if ui.button("⏺ Auto+").on_hover_text("Loop: record on KP, stop on idle, repeat").clicked() {
                                        sh.auto_rec_armed = true;
                                        sh.auto_rec_plus = true;
                                    }
                                    // Тестовая кнопка для проверки звука.
                                    if ui.button("🔊 Test").clicked() {
                                        sound::beep_rec_start();
                                    }
                                });
                            }
                        }

                        ui.add_space(4.0);

                        // Обновить кеш списка только при изменении числа записей.
                        {
                            let sh = self.shared.lock().unwrap();
                            if sh.recordings.len() != self.rec_list_cached_len {
                                self.rec_list_cache = sh.recordings.iter().map(|r| {
                                    let display = if r.filename.is_empty() {
                                        "new".to_string()
                                    } else {
                                        r.filename.strip_suffix(".parquet").unwrap_or(&r.filename).to_string()
                                    };
                                    (display, r.filename.clone())
                                }).collect();
                                self.rec_list_cached_len = sh.recordings.len();
                            }
                        }

                        // Recordings list (обратный порядок — свежие сверху).
                        ui.label("Recordings:");
                        let n_recs = self.rec_list_cache.len();
                        egui::ScrollArea::vertical()
                            .max_height(120.0)
                            .scroll_bar_visibility(egui::scroll_area::ScrollBarVisibility::AlwaysVisible)
                            .id_salt("rec_list")
                            .show(ui, |ui| {
                                ui.set_width(ui.available_width());
                                if ui
                                    .selectable_label(self.view_recording.is_none(), "Live")
                                    .clicked()
                                {
                                    self.switch_recording(None);
                                    self.editing_comment.clear();
                                }
                                // Обратный порядок: свежие записи сверху.
                                for i in (0..n_recs).rev() {
                                    let is_sel = self.view_recording == Some(i);
                                    let label = &self.rec_list_cache[i].0;

                                    // Средний 1x из selection: имя + амплитуда (красный) + фаза (зелёный).
                                    let clicked = if let Some(vv) = self.rec_mean_1x.get(&i) {
                                        let amp_str = format_amp_k(vv.amplitude);
                                        let phase_str = format!("{:.0}deg", vv.phase_deg());
                                        let font = egui::FontId::proportional(13.0);
                                        let amp_color = egui::Color32::from_rgb(255, 100, 100);
                                        let phase_color = egui::Color32::from_rgb(100, 220, 100);
                                        let dim = egui::Color32::from_gray(160);

                                        let mut job = egui::text::LayoutJob::default();
                                        job.append(label, 0.0, egui::TextFormat { font_id: font.clone(), color: dim, ..Default::default() });
                                        job.append(&format!(" {}", amp_str), 0.0, egui::TextFormat { font_id: font.clone(), color: amp_color, ..Default::default() });
                                        job.append(&format!(" {}", phase_str), 0.0, egui::TextFormat { font_id: font.clone(), color: phase_color, ..Default::default() });
                                        ui.selectable_label(is_sel, job).clicked()
                                    } else {
                                        ui.selectable_label(is_sel, label).clicked()
                                    };

                                    if clicked {
                                        self.switch_recording(Some(i));
                                        self.editing_comment = self.rec_comments
                                            .get(&self.rec_list_cache[i].1)
                                            .cloned()
                                            .unwrap_or_default();
                                    }
                                }
                            });

                        // Комментарий к выбранной записи.
                        if let Some(idx) = self.view_recording {
                            if let Some((_, fname)) = self.rec_list_cache.get(idx) {
                                let fname = fname.clone();
                                ui.add_space(4.0);
                                ui.label("Comment:");
                                let resp = ui.add(
                                    egui::TextEdit::multiline(&mut self.editing_comment)
                                        .desired_rows(2)
                                        .desired_width(f32::INFINITY)
                                        .hint_text("добавить комментарий…"),
                                );
                                if resp.lost_focus() {
                                    if self.editing_comment.trim().is_empty() {
                                        self.rec_comments.remove(&fname);
                                    } else {
                                        self.rec_comments.insert(fname, self.editing_comment.clone());
                                    }
                                    save_comments(&self.rec_comments);
                                }
                            }
                        }
                    });
                if hdr.header_response.clicked() {
                    self.sections.record = !self.sections.record;
                }

                ui.separator();

                // ── ADC Control ──────────────────────────────────────────
                let hdr = egui::CollapsingHeader::new("ADC Control")
                    .open(Some(self.sections.adc))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            let pga_pending = self.shared.lock().unwrap().pending_pga.is_some();
                            ui.label("PGA:");
                            egui::ComboBox::from_id_salt("pga_combo")
                                .selected_text(format!("×{}", PGA_OPTIONS[self.pga_idx]))
                                .show_ui(ui, |ui| {
                                    for (i, &val) in PGA_OPTIONS.iter().enumerate() {
                                        ui.selectable_value(&mut self.pga_idx, i, format!("×{val}"));
                                    }
                                });
                            if ui.add_enabled(!pga_pending, egui::Button::new("Set")).clicked() {
                                let pga = PGA_OPTIONS[self.pga_idx];
                                let mut sh = self.shared.lock().unwrap();
                                sh.desired_pga = pga;
                                sh.enqueue_command(Command::SetPga(pga));
                            }
                            if pga_pending {
                                ui.small("Applying...");
                            }
                        });
                        ui.horizontal(|ui| {
                            let rate_pending = self.shared.lock().unwrap().pending_rate.is_some();
                            ui.label("Rate:");
                            egui::ComboBox::from_id_salt("rate_combo")
                                .selected_text(format!("{} SPS", DATA_RATE_OPTIONS[self.data_rate_idx]))
                                .show_ui(ui, |ui| {
                                    for (i, &val) in DATA_RATE_OPTIONS.iter().enumerate() {
                                        ui.selectable_value(&mut self.data_rate_idx, i, format!("{val} SPS"));
                                    }
                                });
                            if ui.add_enabled(!rate_pending, egui::Button::new("Set")).clicked() {
                                let rate = DATA_RATE_OPTIONS[self.data_rate_idx];
                                let mut sh = self.shared.lock().unwrap();
                                sh.desired_rate = rate;
                                sh.enqueue_command(Command::SetDataRate(rate));
                            }
                            if rate_pending {
                                ui.small("Applying...");
                            }
                        });
                    });
                if hdr.header_response.clicked() {
                    self.sections.adc = !self.sections.adc;
                }

                ui.separator();

                // ── Chart ────────────────────────────────────────────────
                let hdr = egui::CollapsingHeader::new("Chart")
                    .open(Some(self.sections.chart))
                    .show(ui, |ui| {
                        // Lane visibility toggles.
                        {
                            let names = ["Tacho", "CH0", "1x A", "1x Φ", "2x A", "2x Φ"];
                            for (i, name) in names.iter().enumerate() {
                                ui.checkbox(&mut self.lane_visible[i], *name);
                            }
                        }
                        if ui.button("Fit all").clicked() {
                            self.chart.fit_all();
                        }
                        if ui.button("Clear selection").clicked() {
                            self.chart.selection = None;
                        }
                        if ui.button("Clear cursor lock").clicked() {
                            self.chart.locked_cursor_t = None;
                        }
                        if let Some(ref sel) = self.chart.selection {
                            ui.label(format!(
                                "Sel: {:.3}s – {:.3}s ({:.3}s)",
                                sel.t_from,
                                sel.t_to,
                                sel.t_to - sel.t_from
                            ));
                        }
                        if let Some(ct) = self.chart.active_cursor_t() {
                            let suffix = if self.chart.locked_cursor_t.is_some() { " [locked]" } else { "" };
                            ui.label(format!("Cursor: {ct:.3}s{suffix}"));
                        }
                    });
                if hdr.header_response.clicked() {
                    self.sections.chart = !self.sections.chart;
                }

                ui.separator();

                // ── Phase ────────────────────────────────────────────────
                let hdr = egui::CollapsingHeader::new("Phase")
                    .open(Some(self.sections.phase))
                    .show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.selectable_value(&mut self.use_spline, false, "PLL");
                            ui.selectable_value(&mut self.use_spline, true, "Spline");
                        });
                        if self.use_spline {
                            ui.horizontal(|ui| {
                                ui.label("lambda:");
                                ui.add(
                                    egui::DragValue::new(&mut self.spline_lambda)
                                        .range(1.0..=10000.0)
                                        .speed(1.0),
                                );
                            });
                        } else {
                            ui.horizontal(|ui| {
                                ui.label("Smooth:");
                                ui.add(
                                    egui::DragValue::new(&mut self.pll_smoothing)
                                        .range(1.0..=600.0)
                                        .speed(0.2)
                                        .suffix(" rev"),
                                );
                            });
                        }
                        ui.checkbox(&mut self.velocity_sensor, "Velocity sensor")
                            .on_hover_text("Коррекция фазы для датчика скорости: зеркало + 90° (velocity опережает displacement на π/2)");
                        ui.horizontal(|ui| {
                            ui.label("1x avg:");
                            ui.add(
                                egui::DragValue::new(&mut self.smooth_1x)
                                    .range(1..=200)
                                    .speed(1.0)
                                    .suffix(" rev"),
                            );
                        });
                    });
                if hdr.header_response.clicked() {
                    self.sections.phase = !self.sections.phase;
                }

                // ── 1x Polar ─────────────────────────────────────────────
                if !order_points.is_empty() {
                    ui.separator();

                    let hdr = egui::CollapsingHeader::new("1x Polar")
                        .open(Some(self.sections.polar))
                        .show(ui, |ui| {
                            // Найти ближайший OrderPoint к активному курсору (hover или locked).
                            let cursor_vectors = self.chart.active_cursor_t().and_then(|ct| {
                                let idx = order_points
                                    .binary_search_by(|p| p.t_mid.partial_cmp(&ct).unwrap_or(std::cmp::Ordering::Less))
                                    .unwrap_or_else(|i| i.min(order_points.len() - 1));
                                Some([order_points[idx].vv_ch0_1x, order_points[idx].vv_ch1_1x])
                            });

                            // Угол ротора в точке курсора (0..2π между двумя соседними KP).
                            let cursor_rotor = self.chart.active_cursor_t().and_then(|ct| {
                                let (kp_ticks, start_tick) = kp_info?;
                                cursor_rotor_phase(ct, kp_ticks, start_tick)
                            });

                            // Trail: POLAR_TRAIL_HALF оборотов до и после курсора из order_points.
                            let polar_trail: Vec<[vibro_analysis::math::complex::VibroVector; 2]> =
                                self.chart.active_cursor_t().map(|ct| {
                                    let idx = order_points
                                        .binary_search_by(|p| p.t_mid.partial_cmp(&ct).unwrap_or(std::cmp::Ordering::Less))
                                        .unwrap_or_else(|i| i.min(order_points.len().saturating_sub(1)));
                                    let from = idx.saturating_sub(ui_polar::POLAR_TRAIL_HALF);
                                    let to = (idx + ui_polar::POLAR_TRAIL_HALF + 1).min(order_points.len());
                                    order_points[from..to]
                                        .iter()
                                        .map(|p| [p.vv_ch0_1x, p.vv_ch1_1x])
                                        .collect()
                                }).unwrap_or_default();

                            if let Some(vectors) = cursor_vectors {
                                ui.horizontal(|ui| {
                                    let mirror_label = if self.polar_mirror { "Mirror: ON" } else { "Mirror: OFF" };
                                    if ui.button(mirror_label).clicked() {
                                        self.polar_mirror = !self.polar_mirror;
                                    }
                                    let inv_label = if self.invert_ch0 { "CH0 inv: ON" } else { "CH0 inv: OFF" };
                                    if ui.button(inv_label).clicked() {
                                        self.invert_ch0 = !self.invert_ch0;
                                    }
                                });
                                ui_polar::draw_polar_view(
                                    ui,
                                    vectors,
                                    &polar_trail,
                                    2,                  // channel_mode: both
                                    0.0,                // rotation_deg
                                    self.polar_mirror,  // mirror
                                    None,               // max_amp_override
                                    cursor_rotor,
                                );

                                ui.add_space(6.0);
                                ui.horizontal(|ui| {
                                    ui.label("Mass:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.balance_mass_g)
                                            .range(0.1..=10_000.0)
                                            .speed(0.1)
                                            .suffix(" g"),
                                    );
                                    ui.label("R:");
                                    ui.add(
                                        egui::DragValue::new(&mut self.balance_radius_mm)
                                            .range(1.0..=10_000.0)
                                            .speed(1.0)
                                            .suffix(" mm"),
                                    );
                                });
                                ui_polar::draw_balance_hint(
                                    ui,
                                    vectors,
                                    2,
                                    self.balance_mass_g,
                                    self.balance_radius_mm,
                                );
                            } else {
                                ui.small("Наведите курсор на график или кликните, чтобы зафиксировать вертикальную линию.");
                            }
                        });
                    if hdr.header_response.clicked() {
                        self.sections.polar = !self.sections.polar;
                    }
                }

                }); // ScrollArea
            });

        // --- Central panel: strip chart ---
        egui::CentralPanel::default().show(ui, |ui| {
            let vd = match view_data {
                Some(vd) if !vd.samples.is_empty() => vd,
                _ => {
                    ui.centered_and_justified(|ui| {
                        ui.label("No data. Connect firmware and press Record.");
                    });
                    return;
                }
            };

            let samples = &vd.samples;
            let sample_rate = vd.sample_rate;

            // Обновляем viewport total.
            let sr = sample_rate.as_f64().max(1.0);
            let t_total = samples.len() as f64 / sr;
            self.chart.set_total(t_total);

            // Во время записи — автоскролл к правому краю.
            let is_recording = self.shared.lock().unwrap().recording;
            if is_recording && self.view_recording.is_none() && self.chart.should_auto_follow_x() {
                self.chart.follow_end();
            }

            // --- Подготовка данных для lanes ---

            // Tacho: ступеньки RPM из точных kp_ticks.
            let steps = tacho_steps(&vd.kp_ticks, vd.start_tick(), vd.end_tick());

            // PLL/spline уже вычислены в pre-compute блоке.
            // PLL RPM как (t, rpm) для line().
            let pll_rpm_line: Vec<(f64, f64)> = pll_points
                .iter()
                .map(|p| {
                    let t = (p.tick - vd.start_tick()) as f64 / SYSTIMER_HZ;
                    (t, p.rpm(SYSTIMER_HZ))
                })
                .collect();
            // Выровненные KP метки (секунды от start_tick).
            let pll_aligned_kp = if !pll_points.is_empty() {
                aligned_kp_times(&pll_points, vd.start_tick(), SYSTIMER_HZ)
            } else {
                Vec::new()
            };


            let (rpm_min, rpm_max) = {
                // Y-range из всех источников RPM через квантили —
                // выбрасываем пики (артефакты при старте/стопе вала).
                let mut rpms = Vec::with_capacity(steps.len() + pll_rpm_line.len());
                for s in &steps {
                    rpms.push(s.rpm);
                }
                for &(_, rpm) in &pll_rpm_line {
                    rpms.push(rpm);
                }
                if rpms.is_empty() {
                    (0.0, 1.0)
                } else {
                    rpms.sort_unstable_by(|a, b| a.partial_cmp(b).unwrap());
                    // 2-й и 98-й перцентили: обрезаем хвосты с артефактами.
                    let lo_idx = (rpms.len() as f64 * 0.02) as usize;
                    let hi_idx = ((rpms.len() as f64 * 0.98) as usize).max(lo_idx + 1).min(rpms.len() - 1);
                    let lo = rpms[lo_idx];
                    let hi = rpms[hi_idx];
                    let margin = (hi - lo).max(10.0) * 0.1;
                    (lo - margin, hi + margin)
                }
            };

            // KP events — вертикальные линии (точные тики с аппаратного таймера).
            let kp_times = kp_ticks_to_times(&vd.kp_ticks, vd.start_tick(), vd.end_tick());

            // KP flag spans — прямоугольники из KEYPHASOR_LEVEL_FLAG (ширина метки).
            // Рисуем step_line внизу 10% Y-range.
            let flag_spans = kp_flag_spans(samples, vd.start_tick());
            // KP-индикация привязана к экранным координатам (нижние 10% lane),
            // а не к мировым Y — чтобы при Y-зуме штрихи не уезжали.

            // KP flag spans храним как (t0, t1, level) где level = 0 (low) или 1 (high).
            // Конвертация в мировые Y координаты происходит в draw через screen_frac_to_y.
            let kp_flag_levels: Vec<(f64, f64, u8)> = {
                let mut out = Vec::new();
                let mut prev_end = 0.0_f64;
                for &(t0, t1) in &flag_spans {
                    if t0 > prev_end {
                        out.push((prev_end, t0, 0u8));
                    }
                    out.push((t0, t1, 1u8));
                    prev_end = t1;
                }
                let t_total = (vd.end_tick().saturating_sub(vd.start_tick())) as f64 / SYSTIMER_HZ;
                if prev_end < t_total {
                    out.push((prev_end, t_total, 0u8));
                }
                out
            };

            // Данные для step_line: (t_start, t_end, rpm).
            let step_data: Vec<(f64, f64, f64)> = steps
                .iter()
                .map(|s| (s.t_start, s.t_end, s.rpm))
                .collect();

            // Waveform ch0 (нормализован на PGA → LSB при PGA=1).
            let pga_f = vd.pga as f64;
            let ch0_sign = if self.invert_ch0 { -1.0 } else { 1.0 };
            // Waveform по фактическим hardware тикам — гарантирует совпадение
            // с KP-метками на оси X (обе системы координат используют одни тики).
            // Ранее использовался индекс i/sr, который уплывает если номинальный
            // sample_rate не совпадает с реальным периодом между DRDY↓.
            let start_tick_f = vd.start_tick() as f64;
            let wave_ch0: Vec<(f64, f64)> = samples
                .iter()
                .map(|s| (
                    (s.tick as f64 - start_tick_f) / SYSTIMER_HZ,
                    ch0_sign * s.ch0.as_f64() / pga_f,
                ))
                .collect();

            let (ch0_min, ch0_max) = if wave_ch0.is_empty() {
                (0.0, 1.0)
            } else {
                let mut lo = f64::MAX;
                let mut hi = f64::MIN;
                for &(_, y) in &wave_ch0 {
                    if y < lo { lo = y; }
                    if y > hi { hi = y; }
                }
                let margin = (hi - lo).max(1.0) * 0.05;
                (lo - margin, hi + margin)
            };

            // order_points уже вычислены в pre-compute блоке.

            let amp_line: Vec<(f64, f64)> = order_points
                .iter()
                .map(|p| (p.t_mid, p.vv_ch0_1x.amplitude))
                .collect();
            let phase_line: Vec<(f64, f64)> = order_points
                .iter()
                .map(|p| (p.t_mid, p.vv_ch0_1x.phase_deg()))
                .collect();
            let amp_2x_line: Vec<(f64, f64)> = order_points
                .iter()
                .map(|p| (p.t_mid, p.vv_ch0_2x.amplitude))
                .collect();
            let phase_2x_line: Vec<(f64, f64)> = order_points
                .iter()
                .map(|p| (p.t_mid, p.vv_ch0_2x.phase_deg()))
                .collect();

            // Y-range для амплитуды: всегда включает ноль.
            let (amp_min, amp_max) = if amp_line.is_empty() {
                (0.0, 1.0)
            } else {
                let mut hi = f64::MIN;
                for &(_, y) in &amp_line {
                    if y > hi { hi = y; }
                }
                let margin = hi.max(1.0) * 0.05;
                (0.0, hi + margin)
            };
            let (amp_2x_min, amp_2x_max) = if amp_2x_line.is_empty() {
                (0.0, 1.0)
            } else {
                let mut hi = f64::MIN;
                for &(_, y) in &amp_2x_line {
                    if y > hi { hi = y; }
                }
                let margin = hi.max(1.0) * 0.05;
                (0.0, hi + margin)
            };

            // Phase: -180..+180 (ноль по центру).
            let phase_min = -180.0;
            let phase_max = 180.0;

            // --- Lanes ---
            let rpm_color = egui::Color32::from_rgb(255, 200, 50);
            let kp_color = egui::Color32::from_rgb(255, 100, 100);
            let flag_color = egui::Color32::from_rgb(100, 180, 255);
            let ch0_color = egui::Color32::from_rgb(100, 200, 100);

            let pll_color = egui::Color32::from_rgb(50, 255, 150);
            let pll_kp_color = egui::Color32::from_rgb(50, 200, 255);

            let amp_color = egui::Color32::from_rgb(255, 150, 50);
            let phase_color = egui::Color32::from_rgb(150, 130, 255);
            let amp_2x_color = egui::Color32::from_rgb(255, 100, 120);
            let phase_2x_color = egui::Color32::from_rgb(120, 220, 255);

            let lanes = vec![
                LaneData {
                    label: "Tacho",
                    color: rpm_color,
                    y_range: Some((rpm_min, rpm_max)),
                    y_view_idx: 0,
                    draw: Box::new(move |lp| {
                        // KP flag/event Y позиции: нижние 10% экрана lane (привязка к пикселям).
                        let kp_y_lo = lp.screen_frac_to_y(0.0);
                        let kp_y_hi = lp.screen_frac_to_y(0.10);

                        // KP flag level — прямоугольники (ширина метки на валу).
                        let kp_flag_steps: Vec<(f64, f64, f64)> = kp_flag_levels
                            .iter()
                            .map(|&(t0, t1, lvl)| {
                                (t0, t1, if lvl == 0 { kp_y_lo } else { kp_y_hi })
                            })
                            .collect();
                        lp.step_line(&kp_flag_steps, flag_color, 0.5);
                        // KP events — сырые фронты.
                        lp.vlines(&kp_times, kp_y_lo, kp_y_hi, kp_color, 0.5);
                        // RPM ступеньки (сырые).
                        lp.step_line(&step_data, rpm_color, 0.5);
                        // PLL RPM — сглаженная линия.
                        lp.line(&pll_rpm_line, pll_color, 1.0);
                        // PLL-выровненные метки — 10px выше сырых, чтобы различать.
                        lp.vlines_offset(&pll_aligned_kp, kp_y_lo, kp_y_hi, pll_kp_color, 1.0, -10.0);
                        lp.legend(&[
                            ("RPM", rpm_color),
                            ("PLL", pll_color),
                            ("KP", kp_color),
                            ("PLL KP", pll_kp_color),
                            ("Flag", flag_color),
                        ]);
                    }),
                },
                LaneData {
                    label: "CH0",
                    color: ch0_color,
                    y_range: Some((ch0_min, ch0_max)),
                    y_view_idx: 1,
                    draw: Box::new(move |lp| {
                        lp.line(&wave_ch0, ch0_color, 1.0);
                    }),
                },
                LaneData {
                    label: "1x A",
                    color: amp_color,
                    y_range: Some((amp_min, amp_max)),
                    y_view_idx: 2,
                    draw: Box::new(move |lp| {
                        let zero_color = egui::Color32::from_rgb(60, 60, 65);
                        lp.hline(0.0, zero_color, 0.5);
                        lp.line(&amp_line, amp_color, 1.5);
                    }),
                },
                LaneData {
                    label: "1x Φ",
                    color: phase_color,
                    y_range: Some((phase_min, phase_max)),
                    y_view_idx: 3,
                    draw: Box::new(move |lp| {
                        let zero_color = egui::Color32::from_rgb(60, 60, 65);
                        lp.hline(0.0, zero_color, 0.5);
                        lp.line(&phase_line, phase_color, 1.5);
                    }),
                },
                LaneData {
                    label: "2x A",
                    color: amp_2x_color,
                    y_range: Some((amp_2x_min, amp_2x_max)),
                    y_view_idx: 4,
                    draw: Box::new(move |lp| {
                        let zero_color = egui::Color32::from_rgb(60, 60, 65);
                        lp.hline(0.0, zero_color, 0.5);
                        lp.line(&amp_2x_line, amp_2x_color, 1.5);
                    }),
                },
                LaneData {
                    label: "2x Φ",
                    color: phase_2x_color,
                    y_range: Some((phase_min, phase_max)),
                    y_view_idx: 5,
                    draw: Box::new(move |lp| {
                        let zero_color = egui::Color32::from_rgb(60, 60, 65);
                        lp.hline(0.0, zero_color, 0.5);
                        lp.line(&phase_2x_line, phase_2x_color, 1.5);
                    }),
                },
            ];

            // Фильтруем lane'ы по видимости.
            let visible_lanes: Vec<LaneData<'_>> = lanes
                .into_iter()
                .enumerate()
                .filter(|(i, _)| self.lane_visible[*i])
                .map(|(_, l)| l)
                .collect();
            show_strip_chart(ui, &mut self.chart, visible_lanes);
        });
    }
}

// ---------------------------------------------------------------------------
// Recording comments persistence
// ---------------------------------------------------------------------------

/// Путь к файлу с комментариями.
fn comments_path() -> std::path::PathBuf {
    std::path::PathBuf::from("recordings/comments.json")
}

/// Загрузить комментарии из JSON-файла. Возвращает пустую map если файл отсутствует.
fn load_comments() -> std::collections::HashMap<String, String> {
    let path = comments_path();
    std::fs::read_to_string(&path)
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default()
}

/// Сохранить комментарии в JSON-файл.
fn save_comments(comments: &std::collections::HashMap<String, String>) {
    let path = comments_path();
    if let Some(dir) = path.parent() {
        std::fs::create_dir_all(dir).unwrap();
    }
    let json = serde_json::to_string_pretty(comments).unwrap();
    std::fs::write(&path, json).unwrap();
}


// ---------------------------------------------------------------------------
// Phase → color mapping
// ---------------------------------------------------------------------------

/// Амплитуда в человекочитаемом формате: 7800 → "7.8k", 350 → "350".
fn format_amp_k(amp: f64) -> String {
    if amp.abs() >= 1000.0 {
        format!("{:.1}k", amp / 1000.0)
    } else {
        format!("{:.0}", amp)
    }
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

fn main() -> eframe::Result {
    unsafe { std::env::set_var("RUST_BACKTRACE", "full") };
    sound::init();

    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 720.0])
            .with_title("Vibrometer"),
        ..Default::default()
    };

    eframe::run_native(
        "Vibrometer",
        options,
        Box::new(|cc| {
            cc.egui_ctx.set_pixels_per_point(2.7);
            Ok(Box::new(VibroApp::new(cc)))
        }),
    )
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use vibro_protocol::Sample;
    use vibro_types::{
        AdcCount,
        SampleIndex,
        SampleRateHz,
    };

    use super::*;

    fn sample(ch0: i32, ch1: i32, keyphasor: bool) -> Sample {
        Sample {
            ch0: AdcCount(ch0),
            ch1: AdcCount(ch1),
            flags: if keyphasor { Sample::KEYPHASOR_FLAG } else { 0 },
            tick: 0,
        }
    }

    #[test]
    fn keyphasor_indices_extract_relative_positions() {
        let samples = vec![
            sample(0, 0, false),
            sample(1, 1, true),
            sample(2, 2, false),
            sample(3, 3, true),
        ];
        assert_eq!(
            crate::analysis::keyphasor_indices(&samples),
            vec![SampleIndex(1), SampleIndex(3)]
        );
    }

    #[test]
    fn tacho_steps_basic() {
        // Два keyphasor-фронта разделённые 0.1 с → 600 RPM.
        // SYSTIMER_HZ = 16_000_000. 0.1 с = 1_600_000 тиков.
        let start = 1_000_000u64;
        let kp_ticks = vec![start, start + 1_600_000];
        let end = start + 3_200_000;

        let steps = tacho_steps(&kp_ticks, start, end);
        assert_eq!(steps.len(), 1);
        assert!((steps[0].rpm - 600.0).abs() < 0.01);
        assert!((steps[0].t_start - 0.0).abs() < 1e-9);
        assert!((steps[0].t_end - 0.1).abs() < 1e-6);
    }

    #[test]
    fn tacho_steps_empty_without_keyphasor() {
        let steps = tacho_steps(&[], 0, 1_000_000);
        assert!(steps.is_empty());
    }

    #[test]
    fn continuous_recording_appends_samples_until_stop() {
        let mut shared = Shared::new();
        shared.recording = true;
        shared.record_mode = RecordMode::Continuous;

        crate::recording::process_recording_sample(&mut shared, sample(1, 2, false));
        crate::recording::process_recording_sample(&mut shared, sample(3, 4, true));

        assert_eq!(shared.rec_buf.len(), 2);
        assert!(shared.recording);
    }

    #[test]
    fn revolution_recording_waits_for_keyphasor_and_stops_after_target() {
        let mut shared = Shared::new();
        shared.sample_rate = SampleRateHz(2000);
        shared.recording = true;
        shared.record_mode = RecordMode::Revolutions;
        shared.record_revs = 1;
        shared.rec_waiting_kp = true;

        // Сэмпл до первого keyphasor — игнорируется (ждём фронта).
        crate::recording::process_recording_sample(&mut shared, sample(10, 20, false));
        assert!(shared.rec_buf.is_empty());
        assert!(shared.recordings.is_empty());

        // Keyphasor-фронт (приходит как отдельное событие) → начинаем запись.
        crate::recording::process_keyphasor(&mut shared);
        // Сэмплы оборота.
        crate::recording::process_recording_sample(&mut shared, sample(11, 21, false));
        crate::recording::process_recording_sample(&mut shared, sample(12, 22, false));
        // Следующий keyphasor-фронт → оборот завершён.
        crate::recording::process_keyphasor(&mut shared);

        assert!(!shared.recording);
        assert_eq!(shared.recordings.len(), 1);
        assert_eq!(shared.recordings[0].sample_rate, SampleRateHz(2000));
        assert_eq!(shared.recordings[0].samples.len(), 2);
    }
}
