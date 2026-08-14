/// Генератор тестовых сигналов виброметра.
///
/// Синтетический сигнал для валидации математики без реального железа:
///   s(t) = A1 * sin(2π*f_rot*t + φ1)          — 1x (дисбаланс)
///          + A2 * sin(2π*2*f_rot*t + φ2)       — 2x (несоосность, опционально)
///          + noise(t)                           — белый шум (AWGN)
///
/// Keyphasor генерируется в момент t = k / f_rot (один раз за оборот).
///
/// Сигнал возвращается как Vec<f64> в условных единицах (мВ от датчика).
/// ADS1256 24-бит, диапазон ±2^23 ≈ ±8388607. Типичный уровень: 1000–100000 LSB.
use std::f64::consts::PI;

/// Параметры тестового сигнала.
#[derive(Debug, Clone)]
pub struct SignalParams {
    /// Частота вращения ротора (Гц). RPM = f_rot * 60.
    pub f_rot:       f64,
    /// Частота дискретизации ADC (Гц). Должна быть > 2 * f_rot (критерий Найквиста).
    pub sample_rate: f64,
    /// Число сэмплов для генерации.
    pub n_samples:   usize,
    /// Амплитуда 1x (АЦП единицы, half-peak). Задаёт дисбаланс.
    pub amp_1x:      f64,
    /// Начальная фаза 1x (рад). Угловое положение дисбаланса.
    pub phase_1x:    f64,
    /// Амплитуда 2x (АЦП единицы). 0 = нет 2x.
    pub amp_2x:      f64,
    /// Начальная фаза 2x (рад).
    pub phase_2x:    f64,
    /// Среднеквадратическое отклонение шума (АЦП единицы).
    /// SNR ≈ amp_1x / noise_rms. Типично: 100..5000.
    pub noise_rms:   f64,
    /// Смещение нуля сигнала (DC offset, АЦП единицы). Моделирует смещение усилителя.
    pub dc_offset:   f64,
}

impl SignalParams {
    /// Параметры по умолчанию: 3000 RPM = 50 Гц, 2 кГц ADC, 4096 точек.
    /// SNR ≈ 40 (хороший сигнал).
    pub fn default_2khz_50hz() -> Self {
        Self {
            f_rot:       50.0,
            sample_rate: 2000.0,
            n_samples:   4096,
            amp_1x:      100_000.0,
            phase_1x:    PI / 3.0, // 60°
            amp_2x:      0.0,
            phase_2x:    0.0,
            noise_rms:   2500.0,
            dc_offset:   0.0,
        }
    }
}

/// Результат генерации тестового сигнала.
pub struct TestSignal {
    /// Сигнал виброметра (АЦП единицы, f64).
    /// Длина = params.n_samples.
    pub samples:           Vec<f64>,
    /// Индексы сэмплов, в которых произошёл keyphasor-фронт.
    /// Каждый фронт = начало нового оборота (t = k / f_rot).
    pub keyphasor_indices: Vec<usize>,
    /// Параметры, с которыми генерировался сигнал.
    pub params:            SignalParams,
}

impl TestSignal {
    /// Число оборотов, вошедших в сигнал (может быть нецелым).
    pub fn n_revolutions(&self) -> f64 {
        self.params.n_samples as f64 / self.params.sample_rate * self.params.f_rot
    }
}

/// Генерация тестового сигнала с псевдослучайным шумом (LCG-генератор).
///
/// Не использует rand crate — детерминированный вывод, seed фиксирован.
/// Это важно: тесты должны быть воспроизводимыми.
pub fn generate(params: SignalParams) -> TestSignal {
    let dt = 1.0 / params.sample_rate;
    let mut samples = Vec::with_capacity(params.n_samples);
    let mut keyphasor_indices = Vec::new();

    // LCG для шума: простой, без зависимостей.
    // Параметры из Numerical Recipes.
    let mut rng_state: u64 = 12345;
    let lcg_next = |state: &mut u64| -> f64 {
        *state = state.wrapping_mul(1664525).wrapping_add(1013904223);
        // Нормируем в (-1, 1)
        (*state as f64) / (u64::MAX as f64) * 2.0 - 1.0
    };

    // Нормальный шум (Box-Muller transform).
    // Принимает пару uniform-чисел, возвращает одно нормальное.
    let box_muller = |u1: f64, u2: f64| -> f64 {
        // u1, u2 ∈ (0, 1) — нужны положительные
        let u1 = (u1.abs() + 1e-15).min(1.0);
        let u2 = (u2.abs() + 1e-15).min(1.0);
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    };

    // Keyphasor фронт: первый после t=0 каждого оборота.
    // Период оборота в сэмплах (может быть дробным):
    let rev_period_samples = params.sample_rate / params.f_rot;

    // Следующий keyphasor-индекс (первый оборот начинается в t=0)
    let mut next_kp = 0.0_f64;

    for i in 0..params.n_samples {
        let t = i as f64 * dt;

        // Keyphasor: фронт в начале каждого нового оборота
        if i as f64 >= next_kp {
            keyphasor_indices.push(i);
            next_kp += rev_period_samples;
        }

        // 1x компонента
        let s1x = params.amp_1x * (2.0 * PI * params.f_rot * t + params.phase_1x).sin();

        // 2x компонента (если задана)
        let s2x = if params.amp_2x > 0.0 {
            params.amp_2x * (2.0 * PI * 2.0 * params.f_rot * t + params.phase_2x).sin()
        } else {
            0.0
        };

        // Гауссов шум (Box-Muller из LCG)
        let u1 = lcg_next(&mut rng_state) * 0.5 + 0.5;
        let u2 = lcg_next(&mut rng_state) * 0.5 + 0.5;
        let noise = box_muller(u1, u2) * params.noise_rms;

        samples.push(params.dc_offset + s1x + s2x + noise);
    }

    TestSignal {
        samples,
        keyphasor_indices,
        params,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_length() {
        let p = SignalParams::default_2khz_50hz();
        let n = p.n_samples;
        let sig = generate(p);
        assert_eq!(sig.samples.len(), n);
    }

    #[test]
    fn test_keyphasor_count() {
        // При 50 Гц и 2 кГц за 4096 сэмплов = 2.048 с → ~102 оборота
        let p = SignalParams::default_2khz_50hz();
        let sig = generate(p);
        let expected_revs = sig.n_revolutions();
        let n_kp = sig.keyphasor_indices.len() as f64;
        // Число keyphasor-событий должно быть в пределах floor..ceil оборотов
        assert!(
            n_kp >= expected_revs.floor() && n_kp <= expected_revs.ceil() + 1.0,
            "keyphasor count={n_kp}, expected ~{expected_revs}"
        );
    }

    #[test]
    fn test_keyphasor_spacing() {
        // Расстояние между keyphasor-фронтами должно быть sample_rate / f_rot ± 1 сэмпл
        let p = SignalParams::default_2khz_50hz();
        let expected_spacing = p.sample_rate / p.f_rot; // 40.0 сэмплов
        let sig = generate(p);
        for w in sig.keyphasor_indices.windows(2) {
            let spacing = (w[1] - w[0]) as f64;
            assert!(
                (spacing - expected_spacing).abs() <= 1.0 + 1e-9,
                "spacing={spacing}, expected~{expected_spacing}"
            );
        }
    }

    #[test]
    fn test_no_noise_exact_amplitude() {
        // Без шума: максимум сигнала должен быть близок к amp_1x + dc_offset
        let p = SignalParams {
            noise_rms: 0.0,
            dc_offset: 0.0,
            amp_2x: 0.0,
            n_samples: 4000,
            ..SignalParams::default_2khz_50hz()
        };
        let amp = p.amp_1x;
        let sig = generate(p);
        let max_val = sig.samples.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (max_val - amp).abs() < amp * 0.01,
            "max={max_val}, expected ~{amp}"
        );
    }

    #[test]
    fn test_deterministic() {
        // Генератор детерминирован: два вызова с одинаковыми params дают одинаковый сигнал
        let p1 = SignalParams::default_2khz_50hz();
        let p2 = SignalParams::default_2khz_50hz();
        let s1 = generate(p1);
        let s2 = generate(p2);
        assert_eq!(s1.samples, s2.samples);
        assert_eq!(s1.keyphasor_indices, s2.keyphasor_indices);
    }
}
