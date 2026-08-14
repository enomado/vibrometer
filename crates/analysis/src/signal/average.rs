/// RPM-гейтированное усреднение вибровекторов.
///
/// При нестабильных оборотах каждый FFT-блок даёт свой RPM и вибровектор.
/// Простое усреднение некорректно: если RPM "гуляет" через резонанс,
/// амплитуда и фаза скачут -- среднее вектор будет бессмысленным.
///
/// Решение: принимать в усреднение только блоки, где RPM попадает
/// в узкое окно вокруг целевого значения.
///
/// Дополнительно: экспоненциальное скользящее среднее (EMA) для вектора,
/// чтобы отслеживать медленные изменения в реальном времени.
use num_complex::Complex64;
use vibro_types::{
    Rpm,
    SampleIndex,
};

use crate::math::complex::VibroVector;

/// Точка измерения: вибровектор + RPM в момент измерения.
#[derive(Debug, Clone, Copy)]
pub struct VectorSample {
    pub vector: VibroVector,
    pub rpm:    Rpm,
}

/// Результат усреднения.
#[derive(Debug, Clone, Copy)]
pub struct AverageResult {
    /// Усреднённый вибровектор.
    pub vector:     VibroVector,
    /// Средний RPM принятых блоков.
    pub rpm:        Rpm,
    /// Число блоков, вошедших в усреднение.
    pub n_accepted: usize,
    /// Число отброшенных блоков (RPM за пределами окна).
    pub n_rejected: usize,
}

/// Результат синхронного усреднения по оборотам (TSA).
#[derive(Debug, Clone)]
pub struct TsaResult {
    /// Усреднённый периодический сигнал на один оборот.
    pub samples:        Vec<f64>,
    /// Число оборотов, вошедших в усреднение.
    pub n_revolutions:  usize,
    /// Число точек на оборот после ресэмплинга.
    pub points_per_rev: usize,
}

fn lerp_sample(samples: &[f64], pos: f64) -> f64 {
    let i0 = pos.floor() as usize;
    let frac = pos - i0 as f64;
    let i1 = (i0 + 1).min(samples.len() - 1);
    samples[i0] * (1.0 - frac) + samples[i1] * frac
}

/// Синхронное усреднение по keyphasor.
///
/// Нарезает сигнал на полные обороты по соседним импульсам keyphasor,
/// ресэмплирует каждый оборот до одинаковой длины и усредняет поточечно.
/// Результат уже привязан к keyphasor: начало массива = 0° ротора.
pub fn time_synchronous_average<K>(samples: &[f64], kp_indices: &[K], points_per_rev: usize) -> TsaResult
where
    K: Copy + Into<SampleIndex>,
{
    assert!(points_per_rev >= 8, "points_per_rev должно быть >= 8");
    assert!(kp_indices.len() >= 3, "нужно минимум 3 keyphasor-фронта для TSA");
    assert!(samples.len() >= 2, "нужно хотя бы 2 сэмпла для TSA");

    let mut acc = vec![0.0; points_per_rev];
    let mut n_revolutions = 0usize;

    for window in kp_indices.windows(2) {
        let start = window[0].into().as_usize();
        let end = window[1].into().as_usize();
        if end <= start + 1 || end > samples.len() {
            continue;
        }

        let rev_len = (end - start) as f64;
        for (j, out) in acc.iter_mut().enumerate() {
            let pos = start as f64 + j as f64 * rev_len / points_per_rev as f64;
            *out += lerp_sample(samples, pos);
        }
        n_revolutions += 1;
    }

    assert!(n_revolutions > 0, "не найдено ни одного полного оборота для TSA");

    for out in &mut acc {
        *out /= n_revolutions as f64;
    }

    TsaResult {
        samples: acc,
        n_revolutions,
        points_per_rev,
    }
}

/// Усреднение набора измерений с RPM-гейтированием.
///
/// Принимает только блоки, где |rpm_i - rpm_target| < rpm_tolerance.
/// Усреднение -- арифметическое среднее комплексных представлений.
///
/// # Паникует
/// Если ни один блок не прошёл фильтр.
pub fn rpm_gated_average(
    samples: &[VectorSample],
    rpm_target: impl Into<Rpm>,
    rpm_tolerance: impl Into<Rpm>,
) -> AverageResult {
    let rpm_target = rpm_target.into();
    let rpm_tolerance = rpm_tolerance.into();
    let mut sum = Complex64::new(0.0, 0.0);
    let mut rpm_sum = 0.0;
    let mut n_accepted = 0_usize;

    for s in samples {
        if (s.rpm.as_f64() - rpm_target.as_f64()).abs() <= rpm_tolerance.as_f64() {
            sum += s.vector.to_complex();
            rpm_sum += s.rpm.as_f64();
            n_accepted += 1;
        }
    }

    assert!(
        n_accepted > 0,
        "ни один блок не попал в RPM-окно [{:.0}, {:.0}]",
        rpm_target.as_f64() - rpm_tolerance.as_f64(),
        rpm_target.as_f64() + rpm_tolerance.as_f64()
    );

    let avg_complex = sum / n_accepted as f64;
    let n_rejected = samples.len() - n_accepted;

    AverageResult {
        vector: VibroVector::from_complex(avg_complex),
        rpm: Rpm::new(rpm_sum / n_accepted as f64),
        n_accepted,
        n_rejected,
    }
}

/// Экспоненциальное скользящее среднее (EMA) для вибровектора.
///
/// Отслеживает медленные изменения вектора в реальном времени.
/// alpha = 1 -- нет сглаживания (текущее значение), alpha → 0 -- сильное сглаживание.
///
/// Формула: V_ema[n] = alpha * V[n] + (1 - alpha) * V_ema[n-1]
///
/// Типичные значения alpha:
///   0.1 -- сильное сглаживание (нужно ~20 блоков для отклика)
///   0.3 -- умеренное
///   0.5 -- слабое (быстрый отклик)
pub struct VectorEma {
    /// Текущее значение EMA (комплексное для корректного усреднения фаз).
    state:       Complex64,
    /// Коэффициент сглаживания (0, 1].
    alpha:       f64,
    /// Было ли хотя бы одно обновление (для инициализации).
    initialized: bool,
}

impl VectorEma {
    pub fn new(alpha: f64) -> Self {
        assert!(alpha > 0.0 && alpha <= 1.0, "alpha должна быть в (0, 1]");
        Self {
            state: Complex64::new(0.0, 0.0),
            alpha,
            initialized: false,
        }
    }

    /// Подать новый вектор. Возвращает текущее сглаженное значение.
    pub fn update(&mut self, v: VibroVector) -> VibroVector {
        let c = v.to_complex();
        if !self.initialized {
            self.state = c;
            self.initialized = true;
        } else {
            self.state = self.state * (1.0 - self.alpha) + c * self.alpha;
        }
        VibroVector::from_complex(self.state)
    }

    /// Текущее сглаженное значение.
    pub fn current(&self) -> VibroVector {
        VibroVector::from_complex(self.state)
    }

    /// Сбросить состояние (например, при смене режима).
    pub fn reset(&mut self) {
        self.state = Complex64::new(0.0, 0.0);
        self.initialized = false;
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use vibro_types::Rpm;

    use super::*;
    use crate::signal::testgen::{
        SignalParams,
        generate,
    };

    #[test]
    fn test_rpm_gated_all_accepted() {
        // Все блоки в окне -- среднее арифметическое.
        let samples = vec![
            VectorSample {
                vector: VibroVector::new(100.0, 0.0),
                rpm:    Rpm::new(3000.0),
            },
            VectorSample {
                vector: VibroVector::new(100.0, 0.0),
                rpm:    Rpm::new(3010.0),
            },
            VectorSample {
                vector: VibroVector::new(100.0, 0.0),
                rpm:    Rpm::new(2990.0),
            },
        ];
        let res = rpm_gated_average(&samples, 3000.0, 50.0);
        assert_eq!(res.n_accepted, 3);
        assert_eq!(res.n_rejected, 0);
        assert!((res.vector.amplitude - 100.0).abs() < 0.01);
        assert!((res.rpm - 3000.0).abs() < 10.1);
    }

    #[test]
    fn test_rpm_gated_some_rejected() {
        // Один блок с RPM далеко от цели -- должен быть отброшен.
        let samples = vec![
            VectorSample {
                vector: VibroVector::new(100.0, 0.0),
                rpm:    Rpm::new(3000.0),
            },
            VectorSample {
                vector: VibroVector::new(100.0, 0.0),
                rpm:    Rpm::new(3000.0),
            },
            // Этот блок с RPM=5000 -- вне окна, должен быть отброшен
            VectorSample {
                vector: VibroVector::new(500.0, PI),
                rpm:    Rpm::new(5000.0),
            },
        ];
        let res = rpm_gated_average(&samples, 3000.0, 50.0);
        assert_eq!(res.n_accepted, 2);
        assert_eq!(res.n_rejected, 1);
        // Без отклонённого блока амплитуда = 100, не 233 (если бы взяли все)
        assert!((res.vector.amplitude - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_rpm_gated_vector_averaging() {
        // Два вектора с разными фазами -- среднее должно быть корректным.
        // V1 = 100∠0°, V2 = 100∠90° → среднее = 50*(1+j) = 70.7∠45°
        let samples = vec![
            VectorSample {
                vector: VibroVector::new(100.0, 0.0),
                rpm:    Rpm::new(3000.0),
            },
            VectorSample {
                vector: VibroVector::new(100.0, PI / 2.0),
                rpm:    Rpm::new(3000.0),
            },
        ];
        let res = rpm_gated_average(&samples, 3000.0, 50.0);
        let expected_amp = 50.0 * 2.0_f64.sqrt(); // ~70.71
        assert!(
            (res.vector.amplitude - expected_amp).abs() < 0.1,
            "avg amp: {:.2}, expected: {expected_amp:.2}",
            res.vector.amplitude
        );
        assert!(
            (res.vector.phase - PI / 4.0).abs() < 0.01,
            "avg phase: {:.3}, expected: {:.3}",
            res.vector.phase,
            PI / 4.0
        );
    }

    #[test]
    #[should_panic(expected = "ни один блок")]
    fn test_rpm_gated_none_accepted() {
        let samples = vec![VectorSample {
            vector: VibroVector::new(100.0, 0.0),
            rpm:    Rpm::new(5000.0),
        }];
        rpm_gated_average(&samples, 3000.0, 50.0);
    }

    #[test]
    fn test_ema_first_value() {
        // Первое значение инициализирует EMA
        let mut ema = VectorEma::new(0.3);
        let v = VibroVector::new(100.0, 0.0);
        let result = ema.update(v);
        assert!((result.amplitude - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_ema_convergence() {
        // EMA должна сходиться к постоянному значению.
        let mut ema = VectorEma::new(0.3);
        // Инициализируем чем-то далёким
        ema.update(VibroVector::new(0.0, 0.0));
        // Подаём 50 одинаковых значений
        let target = VibroVector::new(100.0, PI / 4.0);
        let mut last = VibroVector::ZERO;
        for _ in 0..50 {
            last = ema.update(target);
        }
        assert!(
            (last.amplitude - 100.0).abs() < 0.1,
            "EMA converged to amp={:.2}, expected 100.0",
            last.amplitude
        );
        assert!(
            (last.phase - PI / 4.0).abs() < 0.01,
            "EMA converged to phase={:.3}, expected {:.3}",
            last.phase,
            PI / 4.0
        );
    }

    #[test]
    fn test_ema_smoothing_effect() {
        // С alpha=0.1 скачок амплитуды должен сглаживаться.
        let mut ema = VectorEma::new(0.1);
        // 10 блоков с amp=100
        for _ in 0..10 {
            ema.update(VibroVector::new(100.0, 0.0));
        }
        // Скачок до amp=200
        let after_jump = ema.update(VibroVector::new(200.0, 0.0));
        // EMA должна быть между 100 и 200, ближе к 100
        assert!(
            after_jump.amplitude > 100.0 && after_jump.amplitude < 150.0,
            "after jump: {:.1}, expected ~110",
            after_jump.amplitude
        );
    }

    #[test]
    fn test_ema_reset() {
        let mut ema = VectorEma::new(0.5);
        ema.update(VibroVector::new(100.0, 0.0));
        ema.reset();
        // После reset первое значение = входное
        let v = ema.update(VibroVector::new(50.0, PI));
        assert!((v.amplitude - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_tsa_preserves_1x_amplitude_and_phase() {
        let params = SignalParams {
            n_samples: 4000,
            amp_1x: 100.0,
            phase_1x: 0.7,
            amp_2x: 0.0,
            noise_rms: 0.0,
            ..SignalParams::default_2khz_50hz()
        };
        let sig = generate(params.clone());
        let points_per_rev = (params.sample_rate / params.f_rot).round() as usize;
        let tsa = time_synchronous_average(&sig.samples, &sig.keyphasor_indices, points_per_rev);
        let peak = tsa.samples.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        let expected_first = params.amp_1x * params.phase_1x.sin();

        assert!((peak - params.amp_1x).abs() < 2.0, "peak={peak:.2}");
        assert!(
            (tsa.samples[0] - expected_first).abs() < 2.0,
            "first={:.2} expected={expected_first:.2}",
            tsa.samples[0]
        );
    }

    #[test]
    fn test_tsa_reduces_noise_floor() {
        let params = SignalParams {
            n_samples: 4000,
            amp_1x: 100.0,
            phase_1x: 0.3,
            amp_2x: 0.0,
            noise_rms: 40.0,
            ..SignalParams::default_2khz_50hz()
        };
        let sig = generate(params.clone());
        let points_per_rev = (params.sample_rate / params.f_rot).round() as usize;
        let tsa = time_synchronous_average(&sig.samples, &sig.keyphasor_indices, points_per_rev);

        let raw_rms = (sig.samples.iter().map(|x| x * x).sum::<f64>() / sig.samples.len() as f64).sqrt();
        let tsa_rms = (tsa.samples.iter().map(|x| x * x).sum::<f64>() / tsa.samples.len() as f64).sqrt();

        assert!(tsa.n_revolutions >= 2);
        assert!(tsa_rms < raw_rms, "tsa_rms={tsa_rms:.2} raw_rms={raw_rms:.2}");
    }
}
