use std::sync::{
    Arc,
    Mutex,
};

use vibro_analysis::math::complex::VibroVector;
use vibro_analysis::signal::average::time_synchronous_average;
use vibro_analysis::signal::fft::{
    Spectrum,
    compute_spectrum,
};
use vibro_analysis::signal::goertzel::vibro_goertzel_hanning;
use vibro_analysis::signal::keyphasor::{
    correct_phase,
    rpm_from_keyphasor,
};
use vibro_analysis::signal::pll::{
    PllPoint,
    interpolate_phase,
};
use vibro_protocol::Sample;
use vibro_types::{
    Hertz,
    Rpm,
    SampleIndex,
    SampleRateHz,
};

use crate::recording::Shared;

/// Результат FFT-анализа последнего блока (обновляется по кнопке или автоматически).
pub(crate) struct FftResult {
    pub(crate) spectrum_ch0:  Spectrum,
    pub(crate) spectrum_ch1:  Spectrum,
    /// Оценка RPM для текущего блока.
    pub(crate) rpm_estimated: Rpm,
    /// Источник оценки RPM/фазы: keyphasor или спектр.
    pub(crate) rpm_source:    &'static str,
    /// Вибровектор 1x ch0
    pub(crate) vv_ch0_1x:     VibroVector,
    /// Вибровектор 1x ch1
    pub(crate) vv_ch1_1x:     VibroVector,
}

pub(crate) fn keyphasor_indices(samples: &[Sample]) -> Vec<SampleIndex> {
    samples
        .iter()
        .enumerate()
        .filter_map(|(idx, sample)| sample.keyphasor().then_some(SampleIndex(idx)))
        .collect()
}

pub(crate) fn analyze_block(
    samples: &[Sample],
    sample_rate: SampleRateHz,
    pga: u8,
    rpm_search_min: Rpm,
    rpm_search_max: Rpm,
) -> Option<(Rpm, &'static str, VibroVector, VibroVector)> {
    if sample_rate.as_f64() < 1.0 || samples.len() < 64 {
        return None;
    }

    // Нормализация на PGA: raw / pga → LSB при PGA=1.
    let pga_f = pga as f64;
    let ch0: Vec<f64> = samples.iter().map(|s| s.ch0.as_f64() / pga_f).collect();
    let ch1: Vec<f64> = samples.iter().map(|s| s.ch1.as_f64() / pga_f).collect();
    let kp_indices = keyphasor_indices(samples);
    let sample_rate_hz = sample_rate.as_hz();

    if kp_indices.len() >= 3 {
        let rpm = rpm_from_keyphasor(&kp_indices, sample_rate_hz);
        let f_rot = rpm.as_hz();
        let points_per_rev = ((sample_rate_hz.as_f64() / f_rot.as_f64()).round() as usize).max(32);

        let tsa_ch0 = time_synchronous_average(&ch0, &kp_indices, points_per_rev);
        let tsa_ch1 = time_synchronous_average(&ch1, &kp_indices, points_per_rev);
        let virtual_sample_rate = Hertz::new(f_rot.as_f64() * points_per_rev as f64);

        let sp0 = compute_spectrum(&tsa_ch0.samples, virtual_sample_rate);
        let sp1 = compute_spectrum(&tsa_ch1.samples, virtual_sample_rate);

        Some((rpm, "tsa", sp0.vibro_at(f_rot), sp1.vibro_at(f_rot)))
    } else if kp_indices.len() >= 2 {
        let rpm = rpm_from_keyphasor(&kp_indices, sample_rate_hz);
        let f_rot = rpm.as_hz();
        let kp_sample = kp_indices[0];

        let vv_ch0 = correct_phase(
            vibro_goertzel_hanning(&ch0, f_rot, sample_rate_hz),
            kp_sample,
            f_rot,
            sample_rate_hz,
        );
        let vv_ch1 = correct_phase(
            vibro_goertzel_hanning(&ch1, f_rot, sample_rate_hz),
            kp_sample,
            f_rot,
            sample_rate_hz,
        );

        Some((rpm, "keyphasor", vv_ch0, vv_ch1))
    } else {
        let sp0 = compute_spectrum(&ch0, sample_rate_hz);
        let sp1 = compute_spectrum(&ch1, sample_rate_hz);
        let rpm = sp0.estimate_rpm(rpm_search_min, rpm_search_max);
        let f_rot = rpm.as_hz();
        Some((rpm, "spectrum", sp0.vibro_at(f_rot), sp1.vibro_at(f_rot)))
    }
}

/// Диапазон выбора в записи (секунды). None = весь диапазон.
pub(crate) type RecRange = Option<(f64, f64)>;

pub(crate) fn current_view_samples(
    shared: &Arc<Mutex<Shared>>,
    view_recording: Option<usize>,
    range: RecRange,
) -> Option<(Vec<Sample>, SampleRateHz, u8)> {
    let sh = shared.lock().unwrap();

    match view_recording {
        None => {
            let sample_rate = sh.sample_rate;
            if sample_rate.as_f64() < 1.0 {
                return None;
            }
            Some((sh.live_buf.iter().copied().collect(), sample_rate, sh.pga))
        }
        Some(idx) => {
            let rec = sh.recordings.get(idx)?;
            let sample_rate = rec.sample_rate;
            if sample_rate.as_f64() < 1.0 {
                return None;
            }
            let samples = match range {
                Some((from_s, to_s)) => {
                    let sr = sample_rate.as_f64();
                    let from_idx = (from_s * sr).round() as usize;
                    let to_idx = ((to_s * sr).round() as usize).min(rec.samples.len());
                    if from_idx < to_idx {
                        rec.samples[from_idx..to_idx].to_vec()
                    } else {
                        rec.samples.clone()
                    }
                }
                None => rec.samples.clone(),
            };
            Some((samples, sample_rate, rec.pga))
        }
    }
}

pub(crate) fn current_measurement(
    shared: &Arc<Mutex<Shared>>,
    view_recording: Option<usize>,
    rpm_search_min: Rpm,
    rpm_search_max: Rpm,
    range: RecRange,
) -> Option<(Rpm, &'static str, [VibroVector; 2])> {
    let (samples, sample_rate, pga) = current_view_samples(shared, view_recording, range)?;
    let (rpm, source, vv_ch0, vv_ch1) = analyze_block(&samples, sample_rate, pga, rpm_search_min, rpm_search_max)?;
    Some((rpm, source, [vv_ch0, vv_ch1]))
}

// ---------------------------------------------------------------------------
// IQ demodulation: 1x extraction with EMA low-pass filter
// ---------------------------------------------------------------------------

/// Результат IQ-демодуляции в одной точке (один оборот).
pub(crate) struct OrderPoint {
    /// Время середины оборота (секунды от start_tick).
    pub(crate) t_mid: f64,
    /// 1x вибровектор ch0 (после LPF).
    pub(crate) vv_ch0_1x: VibroVector,
    /// 1x вибровектор ch1 (после LPF).
    pub(crate) vv_ch1_1x: VibroVector,
    /// 2x вибровектор ch0 (после LPF).
    pub(crate) vv_ch0_2x: VibroVector,
    /// 2x вибровектор ch1 (после LPF).
    pub(crate) vv_ch1_2x: VibroVector,
    /// RPM этого оборота (из PLL smooth_period).
    pub(crate) rpm: f64,
}

/// IQ-демодуляция 1x/2x по оборотам.
///
/// Для каждого оборота [pll[i].aligned_tick, pll[i+1].aligned_tick):
///   I = (1/N) · Σ x[n] · cos(θ[n])
///   Q = (1/N) · Σ x[n] · sin(θ[n])
///   amplitude = 2·√(I²+Q²)    (×2 т.к. ‹cos²θ› = 0.5)
///   phase     = atan2(Q, I)
pub(crate) fn order_track(
    samples: &[Sample],
    pll: &[PllPoint],
    start_tick: u64,
    systimer_hz: f64,
    pga: u8,
    invert_ch0: bool,
    velocity_sensor: bool,
) -> Vec<OrderPoint> {
    if pll.len() < 2 || samples.len() < 4 {
        return Vec::new();
    }

    let pga_f = pga as f64;
    let mut results = Vec::with_capacity(pll.len());

    // Бинарный поиск первого сэмпла, попадающего в первый оборот.
    // Затем линейное продвижение курсора — O(N + M) вместо O(N × M).
    let first_tick_from = pll[0].aligned_tick;
    let mut si = samples.partition_point(|s| (s.tick as f64) < first_tick_from);

    for w in pll.windows(2) {
        let tick_from = w[0].aligned_tick;
        let tick_to = w[1].aligned_tick;

        let mut i0_1x = 0.0f64; let mut q0_1x = 0.0f64;
        let mut i1_1x = 0.0f64; let mut q1_1x = 0.0f64;
        let mut i0_2x = 0.0f64; let mut q0_2x = 0.0f64;
        let mut i1_2x = 0.0f64; let mut q1_2x = 0.0f64;
        let mut n = 0usize;

        // Пропускаем сэмплы до начала этого оборота (на случай если
        // aligned_tick'и перекрываются или есть зазоры).
        while si < samples.len() && (samples[si].tick as f64) < tick_from {
            si += 1;
        }

        // Итерируем только сэмплы внутри [tick_from, tick_to).
        let mut j = si;
        while j < samples.len() {
            let s = &samples[j];
            let t = s.tick as f64;
            if t >= tick_to {
                break;
            }
            let theta = interpolate_phase(pll, s.tick, systimer_hz).1;
            let x0_raw = s.ch0.as_f64() / pga_f;
            let x0 = if invert_ch0 { -x0_raw } else { x0_raw };
            let x1 = s.ch1.as_f64() / pga_f;
            let (sin1, cos1) = theta.sin_cos();
            let (sin2, cos2) = (2.0 * theta).sin_cos();

            i0_1x += x0 * cos1; q0_1x += x0 * sin1;
            i1_1x += x1 * cos1; q1_1x += x1 * sin1;
            i0_2x += x0 * cos2; q0_2x += x0 * sin2;
            i1_2x += x1 * cos2; q1_2x += x1 * sin2;
            n += 1;
            j += 1;
        }

        if n == 0 {
            continue;
        }
        let c = n as f64;
        let iq_to_vv = |i: f64, q: f64| {
            VibroVector::new(2.0 * ((i / c).hypot(q / c)), (q / c).atan2(i / c))
        };

        let mut vv_ch0_1x = iq_to_vv(i0_1x, q0_1x);
        let mut vv_ch1_1x = iq_to_vv(i1_1x, q1_1x);
        let mut vv_ch0_2x = iq_to_vv(i0_2x, q0_2x);
        let mut vv_ch1_2x = iq_to_vv(i1_2x, q1_2x);

        // Velocity transducer: velocity опережает displacement на π/2,
        // плюс зеркало направления отсчёта фазы.
        // Коррекция: corrected = −phase + π/2.
        if velocity_sensor {
            let correct = |vv: VibroVector| {
                VibroVector::new(vv.amplitude, -vv.phase + std::f64::consts::FRAC_PI_2)
            };
            vv_ch0_1x = correct(vv_ch0_1x);
            vv_ch1_1x = correct(vv_ch1_1x);
            vv_ch0_2x = correct(vv_ch0_2x);
            vv_ch1_2x = correct(vv_ch1_2x);
        }

        results.push(OrderPoint {
            t_mid:     (tick_from - start_tick as f64) / systimer_hz,
            rpm:       w[0].rpm(systimer_hz),
            vv_ch0_1x,
            vv_ch1_1x,
            vv_ch0_2x,
            vv_ch1_2x,
        });
    }

    results
}

/// Скользящее среднее 1x/2x-векторов по IQ.
///
/// window: количество оборотов для усреднения. 1 = без усреднения.
/// Усреднение происходит в пространстве IQ (Re/Im), а не по амплитуде/фазе —
/// это корректное усреднение комплексных сигналов, не смещает фазу при переходах 0°/360°.
pub(crate) fn smooth_order_points(pts: Vec<OrderPoint>, window: usize) -> Vec<OrderPoint> {
    if window <= 1 || pts.len() < 2 {
        return pts;
    }
    let n = pts.len();
    let mut out = Vec::with_capacity(n);

    for i in 0..n {
        // Симметричное окно вокруг точки i.
        let half = window / 2;
        let from = i.saturating_sub(half);
        let to = (i + window - half).min(n);
        let count = (to - from) as f64;

        let mut i0_1x = 0.0f64; let mut q0_1x = 0.0f64;
        let mut i1_1x = 0.0f64; let mut q1_1x = 0.0f64;
        let mut i0_2x = 0.0f64; let mut q0_2x = 0.0f64;
        let mut i1_2x = 0.0f64; let mut q1_2x = 0.0f64;

        for p in &pts[from..to] {
            i0_1x += p.vv_ch0_1x.amplitude * p.vv_ch0_1x.phase.cos();
            q0_1x += p.vv_ch0_1x.amplitude * p.vv_ch0_1x.phase.sin();
            i1_1x += p.vv_ch1_1x.amplitude * p.vv_ch1_1x.phase.cos();
            q1_1x += p.vv_ch1_1x.amplitude * p.vv_ch1_1x.phase.sin();
            i0_2x += p.vv_ch0_2x.amplitude * p.vv_ch0_2x.phase.cos();
            q0_2x += p.vv_ch0_2x.amplitude * p.vv_ch0_2x.phase.sin();
            i1_2x += p.vv_ch1_2x.amplitude * p.vv_ch1_2x.phase.cos();
            q1_2x += p.vv_ch1_2x.amplitude * p.vv_ch1_2x.phase.sin();
        }

        let avg = |i: f64, q: f64| {
            VibroVector::new((i / count).hypot(q / count), (q / count).atan2(i / count))
        };

        out.push(OrderPoint {
            t_mid:     pts[i].t_mid,
            rpm:       pts[i].rpm,
            vv_ch0_1x: avg(i0_1x, q0_1x),
            vv_ch1_1x: avg(i1_1x, q1_1x),
            vv_ch0_2x: avg(i0_2x, q0_2x),
            vv_ch1_2x: avg(i1_2x, q1_2x),
        });
    }
    out
}

/// Среднее 1x вибровектора (ch0) по order_points в диапазоне [t_from, t_to].
/// Усреднение в IQ-пространстве (корректно для фазы).
/// Возвращает None если в диапазоне нет точек.
pub(crate) fn mean_1x_in_range(pts: &[OrderPoint], t_from: f64, t_to: f64) -> Option<VibroVector> {
    let mut i_acc = 0.0f64;
    let mut q_acc = 0.0f64;
    let mut count = 0u32;
    for p in pts {
        if p.t_mid >= t_from && p.t_mid <= t_to {
            i_acc += p.vv_ch0_1x.amplitude * p.vv_ch0_1x.phase.cos();
            q_acc += p.vv_ch0_1x.amplitude * p.vv_ch0_1x.phase.sin();
            count += 1;
        }
    }
    if count == 0 { return None; }
    let n = count as f64;
    Some(VibroVector::new((i_acc / n).hypot(q_acc / n), (q_acc / n).atan2(i_acc / n)))
}
