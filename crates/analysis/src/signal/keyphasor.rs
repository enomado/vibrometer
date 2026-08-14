/// Обработка keyphasor-сигнала: RPM из периода между фронтами,
/// коррекция фазы FFT-бина на момент keyphasor-фронта.
///
/// Keyphasor — один импульс за оборот вала. Даёт:
///   - RPM = 60 / T_period (из периода между фронтами)
///   - Опорную фазу: момент фронта = 0° вала
///
/// Коррекция фазы FFT:
///   FFT-фаза привязана к началу блока данных, а не к ротору.
///   Зная позицию keyphasor-фронта в блоке (сэмпл k_kp), корректируем:
///     phi_rotor = phi_fft - 2*pi*f_rot * (k_kp / sample_rate)
///
///   Это переводит фазу из координат "начало буфера" в координаты "ротор".
use std::f64::consts::PI;

use vibro_types::{
    Hertz,
    Rpm,
    SampleCount,
    SampleIndex,
};

use crate::math::complex::VibroVector;

/// RPM из периодов между keyphasor-фронтами.
///
/// # Аргументы
/// - `kp_indices` — индексы сэмплов с keyphasor-фронтом (монотонно возрастающие)
/// - `sample_rate` — частота дискретизации (Гц)
///
/// Возвращает средний RPM. Использует все доступные периоды.
///
/// # Паникует
/// Если менее 2 фронтов (нужен хотя бы один полный оборот).
pub fn rpm_from_keyphasor<K>(kp_indices: &[K], sample_rate: impl Into<Hertz>) -> Rpm
where
    K: Copy + Into<SampleIndex>,
{
    let sample_rate = sample_rate.into();
    assert!(
        kp_indices.len() >= 2,
        "нужно минимум 2 keyphasor-фронта для определения RPM"
    );

    let mut sum_period = 0.0;
    let n_periods = kp_indices.len() - 1;
    for w in kp_indices.windows(2) {
        let dt = (w[1].into().as_usize() - w[0].into().as_usize()) as f64 / sample_rate.as_f64();
        sum_period += dt;
    }
    let avg_period = sum_period / n_periods as f64;
    Rpm::new(60.0 / avg_period)
}

/// Мгновенный RPM из последнего периода keyphasor.
///
/// Полезно для отслеживания нестабильности оборотов в реальном времени.
pub fn rpm_instantaneous<K>(kp_indices: &[K], sample_rate: impl Into<Hertz>) -> Rpm
where
    K: Copy + Into<SampleIndex>,
{
    let sample_rate = sample_rate.into();
    assert!(kp_indices.len() >= 2, "нужно минимум 2 фронта");
    let n = kp_indices.len();
    let dt = (kp_indices[n - 1].into().as_usize() - kp_indices[n - 2].into().as_usize()) as f64
        / sample_rate.as_f64();
    Rpm::new(60.0 / dt)
}

/// Стабильность RPM: стандартное отклонение мгновенного RPM.
///
/// Малое значение → обороты стабильны, можно балансировать.
/// Большое → обороты гуляют, нужно RPM-гейтирование или ждать стабилизации.
///
/// Возвращает (mean_rpm, std_rpm).
pub fn rpm_stability<K>(kp_indices: &[K], sample_rate: impl Into<Hertz>) -> (Rpm, Rpm)
where
    K: Copy + Into<SampleIndex>,
{
    let sample_rate = sample_rate.into();
    assert!(kp_indices.len() >= 3, "нужно минимум 3 фронта для статистики");

    let rpms: Vec<Rpm> = kp_indices
        .windows(2)
        .map(|w| {
            let dt = (w[1].into().as_usize() - w[0].into().as_usize()) as f64 / sample_rate.as_f64();
            Rpm::new(60.0 / dt)
        })
        .collect();

    let mean = rpms.iter().map(|r| r.as_f64()).sum::<f64>() / rpms.len() as f64;
    let variance = rpms.iter().map(|r| (r.as_f64() - mean).powi(2)).sum::<f64>() / rpms.len() as f64;
    (Rpm::new(mean), Rpm::new(variance.sqrt()))
}

/// Коррекция фазы вибровектора: из координат "начало FFT-блока" в координаты "ротор".
///
/// # Аргументы
/// - `vv` — вибровектор из FFT (фаза относительно начала блока)
/// - `kp_sample` — индекс keyphasor-фронта внутри FFT-блока
/// - `f_rot` — частота вращения (Гц)
/// - `sample_rate` — частота дискретизации (Гц)
///
/// # Формула
///   phi_rotor = phi_fft - 2*pi*f_rot * (kp_sample / sample_rate)
///
/// Keyphasor-фронт определяет 0° ротора. Вибровектор с исправленной фазой
/// показывает положение дисбаланса относительно метки на роторе.
pub fn correct_phase(
    vv: VibroVector,
    kp_sample: impl Into<SampleIndex>,
    f_rot: impl Into<Hertz>,
    sample_rate: impl Into<Hertz>,
) -> VibroVector {
    let kp_sample = kp_sample.into();
    let f_rot = f_rot.into();
    let sample_rate = sample_rate.into();
    let t_kp = kp_sample.as_f64() / sample_rate.as_f64();
    let phase_correction = 2.0 * PI * f_rot.as_f64() * t_kp;
    VibroVector::new(vv.amplitude, vv.phase - phase_correction)
}

/// Найти первый keyphasor-фронт в FFT-блоке.
///
/// `block_start` — абсолютный индекс начала блока в потоке сэмплов.
/// `kp_indices` — абсолютные индексы keyphasor-фронтов.
/// `block_len` — длина блока (N).
///
/// Возвращает относительный индекс внутри блока, или None если фронтов нет.
pub fn first_kp_in_block<K>(
    block_start: impl Into<SampleIndex>,
    block_len: impl Into<SampleCount>,
    kp_indices: &[K],
) -> Option<SampleIndex>
where
    K: Copy + Into<SampleIndex>,
{
    let block_start = block_start.into();
    let block_len = block_len.into();
    let block_end = block_start.as_usize() + block_len.as_usize();
    kp_indices
        .iter()
        .copied()
        .map(Into::into)
        .find(|idx: &SampleIndex| idx.as_usize() >= block_start.as_usize() && idx.as_usize() < block_end)
        .map(|idx| SampleIndex(idx.as_usize() - block_start.as_usize()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rpm_from_keyphasor() {
        // 50 Гц = 3000 RPM, sample_rate=2000, период = 40 сэмплов
        let kp = vec![0, 40, 80, 120, 160];
        let rpm = rpm_from_keyphasor(&kp, 2000.0);
        assert!((rpm - 3000.0).abs() < 0.01, "RPM: {rpm:.2}, expected 3000");
    }

    #[test]
    fn test_rpm_instantaneous() {
        // Последний период чуть короче: 38 сэмплов → RPM = 60 / (38/2000) = 3157.9
        let kp = vec![0, 40, 80, 118];
        let rpm = rpm_instantaneous(&kp, 2000.0);
        let expected = 60.0 / (38.0 / 2000.0);
        assert!(
            (rpm - expected).abs() < 0.1,
            "RPM: {rpm:.1}, expected {expected:.1}"
        );
    }

    #[test]
    fn test_rpm_stability_stable() {
        // Стабильные обороты: все периоды = 40 сэмплов
        let kp: Vec<usize> = (0..20).map(|i| i * 40).collect();
        let (mean, std) = rpm_stability(&kp, 2000.0);
        assert!((mean - 3000.0).abs() < 0.1, "mean: {mean:.1}");
        assert!(std < 0.1, "std: {std:.3}, expected ~0");
    }

    #[test]
    fn test_rpm_stability_unstable() {
        // Нестабильные: периоды гуляют ±2 сэмпла (38, 42, 39, 41, 40)
        let kp = vec![0, 38, 80, 119, 160, 200];
        let (mean, std) = rpm_stability(&kp, 2000.0);
        // Средний RPM ≈ 3000, std > 0
        assert!((mean - 3000.0).abs() < 100.0, "mean: {mean:.1}");
        assert!(std > 10.0, "std: {std:.1}, expected > 10");
    }

    #[test]
    fn test_correct_phase() {
        // FFT-вектор: 100∠45°. Keyphasor в сэмпле 20 при f_rot=50, fs=2000.
        // t_kp = 20/2000 = 0.01 с.
        // Коррекция = 2*pi*50*0.01 = pi рад = 180°.
        // phi_rotor = 45° - 180° = -135° → нормализованный = -135° = 225° (в (-π,π]: -135°)
        let vv = VibroVector::new(100.0, 45.0_f64.to_radians());
        let corrected = correct_phase(vv, 20, 50.0, 2000.0);
        assert!((corrected.amplitude - 100.0).abs() < 0.01);
        let expected_phase = 45.0_f64.to_radians() - PI;
        assert!(
            (corrected.phase - expected_phase).abs() < 0.01,
            "phase: {:.1}°, expected {:.1}°",
            corrected.phase.to_degrees(),
            expected_phase.to_degrees()
        );
    }

    #[test]
    fn test_first_kp_in_block() {
        let kp = vec![10, 50, 90, 130, 170];
        // Блок начинается с 40, длина 60 → [40, 100)
        // Фронты 50 и 90 попадают, первый = 50, относительный = 10
        assert_eq!(first_kp_in_block(40, 60, &kp), Some(SampleIndex(10)));
        // Блок [200, 260) → нет фронтов
        assert_eq!(first_kp_in_block(200, 60, &kp), None);
        // Блок [0, 20) → фронт 10, относительный = 10
        assert_eq!(first_kp_in_block(0, 20, &kp), Some(SampleIndex(10)));
    }

    #[test]
    fn test_phase_correction_roundtrip() {
        // Если keyphasor в начале блока (kp_sample=0), коррекция = 0:
        // фаза FFT уже привязана к началу = keyphasor.
        let vv = VibroVector::new(100.0, 1.2);
        let corrected = correct_phase(vv, 0, 50.0, 2000.0);
        assert!((corrected.phase - vv.phase).abs() < 1e-12);
    }
}
