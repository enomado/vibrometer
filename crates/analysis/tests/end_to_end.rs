/// Сквозной тест: от синтетического сигнала до корректирующего груза.
///
/// Модель:
///   - Ротор с известным дисбалансом U (масса × угол)
///   - Матрица влияния A (связь дисбаланс → вибрация на датчике)
///   - Генерация сигнала: вибрация = A * U → амплитуда и фаза → testgen
///   - FFT → вибровектор → influence → Correction
///   - Проверка: добавить коррекцию к дисбалансу → остаточная вибрация ≈ 0
///
/// Тестируем всю цепочку: testgen → fft → xy_pair → average → influence → verify.
use std::f64::consts::PI;

use num_complex::Complex64;
use vibro_analysis::balance::bode::{
    BodePlot,
    BodePoint,
    Regime,
};
use vibro_analysis::balance::influence::{
    TrialQuality,
    TrialWeight,
    TwoPlaneRuns,
    check_trial_response,
    solve_1plane,
    solve_2planes,
};
use vibro_analysis::math::complex::VibroVector;
use vibro_analysis::signal::average::{
    VectorSample,
    rpm_gated_average,
};
use vibro_analysis::signal::fft::compute_spectrum;
use vibro_analysis::signal::xy_pair::process_xy_pair;
use vibro_types::{
    Rpm,
};

// ─── Генератор шума ─────────────────────────────────────────────────────────
//
// Xorshift64 — качественный PRNG для тестов. В отличие от LCG с 32-битными
// константами на u64, даёт равномерно распределённые биты во всём диапазоне.

/// Xorshift64 PRNG (Marsaglia, 2003). Период 2^64 - 1.
struct Xorshift64(u64);

impl Xorshift64 {
    fn new(seed: u64) -> Self {
        // seed=0 недопустим для xorshift
        Self(seed | 1)
    }

    fn next_u64(&mut self) -> u64 {
        let mut x = self.0;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        self.0 = x;
        x
    }

    /// Uniform в [0, 1).
    fn uniform(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / ((1u64 << 53) as f64)
    }

    /// Нормальное распределение N(0, 1) методом Box-Muller.
    fn normal(&mut self) -> f64 {
        let u1 = self.uniform().max(1e-15);
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

// ─── Вспомогательные функции ────────────────────────────────────────────────

/// Модель влияния: вибрация на датчике = alpha * дисбаланс.
/// alpha — комплексный коэффициент (передаточная функция).
fn vibration_from_imbalance(alpha: Complex64, imbalance: Complex64) -> VibroVector {
    VibroVector::from_complex(alpha * imbalance)
}

/// Генерация X/Y пары сигналов для заданного вибровектора.
///
/// x(t) = amp * cos(2*pi*f_rot*t + phase)
/// y(t) = amp * sin(2*pi*f_rot*t + phase)
fn generate_xy_signals(
    vv: VibroVector,
    f_rot: f64,
    sample_rate: f64,
    n_samples: usize,
    noise_rms: f64,
    seed: u64,
) -> (Vec<f64>, Vec<f64>) {
    let dt = 1.0 / sample_rate;
    let mut rng = Xorshift64::new(seed);
    let mut x = Vec::with_capacity(n_samples);
    let mut y = Vec::with_capacity(n_samples);

    for i in 0..n_samples {
        let t = i as f64 * dt;
        let angle = 2.0 * PI * f_rot * t + vv.phase;
        x.push(vv.amplitude * angle.cos() + rng.normal() * noise_rms);
        y.push(vv.amplitude * angle.sin() + rng.normal() * noise_rms);
    }
    (x, y)
}

/// Генерация одноканального cos-сигнала.
///
/// Используем cos для согласования с комплексной конвенцией FFT:
///   s(t) = A * cos(2*pi*f*t + phi)
///   FFT-бин на частоте f: phase = phi
fn generate_single_channel(
    vv: VibroVector,
    f_rot: f64,
    sample_rate: f64,
    n_samples: usize,
    noise_rms: f64,
    seed: u64,
) -> Vec<f64> {
    let dt = 1.0 / sample_rate;
    let mut rng = Xorshift64::new(seed);

    (0..n_samples)
        .map(|i| {
            let t = i as f64 * dt;
            vv.amplitude * (2.0 * PI * f_rot * t + vv.phase).cos() + rng.normal() * noise_rms
        })
        .collect()
}

// ─── Тесты ──────────────────────────────────────────────────────────────────

/// Одноплоскостная балансировка через один датчик (FFT).
///
/// Цепочка: модель → testgen → FFT → vibro_at → solve_1plane → verify
#[test]
fn e2e_single_plane_single_sensor() {
    let f_rot = 50.0; // 3000 RPM
    let sample_rate = 2000.0;
    // N=4000: точное попадание на бин (50*4000/2000 = 100), минимальный leakage
    let n_samples = 4000;
    let noise_rms = 2500.0;

    // Коэффициент влияния: alpha = 10000∠15° (LSB/г)
    // При дисбалансе 8г → амплитуда вибрации = 80000 LSB, SNR = 80000/2500 = 32 (хорошо)
    let alpha = Complex64::from_polar(10000.0, 15.0_f64.to_radians());

    // Начальный дисбаланс: 8г @ 60°
    let imbalance = Complex64::from_polar(8.0, 60.0_f64.to_radians());

    // --- Пуск 0: начальный ---
    let v0_true = vibration_from_imbalance(alpha, imbalance);
    let signal_0 = generate_single_channel(v0_true, f_rot, sample_rate, n_samples, noise_rms, 111);
    let sp_0 = compute_spectrum(&signal_0, sample_rate);
    let v0 = sp_0.vibro_at(f_rot);

    // --- Пуск 1: пробный груз ---
    let trial = TrialWeight::new(5.0, 0.0); // 5г @ 0°
    let trial_c = trial.to_complex();
    // Новый дисбаланс = начальный + пробный
    let imbalance_with_trial = imbalance + trial_c;
    let v1_true = vibration_from_imbalance(alpha, imbalance_with_trial);
    let signal_1 = generate_single_channel(v1_true, f_rot, sample_rate, n_samples, noise_rms, 222);
    let sp_1 = compute_spectrum(&signal_1, sample_rate);
    let v1 = sp_1.vibro_at(f_rot);

    // --- Проверка качества пробного пуска ---
    let (ratio, quality) = check_trial_response(v0, v1);
    assert_eq!(quality, TrialQuality::Good, "trial response ratio: {ratio:.2}");

    // --- Решение ---
    let correction = solve_1plane(v0, v1, trial).unwrap();
    println!(
        "E2E 1-plane: imbalance={:.1}г@{:.0}°, correction={}",
        imbalance.norm(),
        imbalance.arg().to_degrees(),
        correction
    );

    // --- Верификация: добавляем коррекцию к начальному дисбалансу ---
    let w = Complex64::from_polar(correction.mass_g.as_f64(), correction.angle_rad.as_f64());
    let residual_imbalance = imbalance + w;
    let v_residual = vibration_from_imbalance(alpha, residual_imbalance);

    // Остаточная вибрация < 5% от начальной
    let reduction = v_residual.amplitude / v0_true.amplitude;
    assert!(
        reduction < 0.05,
        "остаточная вибрация: {:.1}% от начальной (V0={:.1}, Vres={:.1})",
        reduction * 100.0,
        v0_true.amplitude,
        v_residual.amplitude
    );
}

/// Одноплоскостная балансировка через X/Y пару.
///
/// Цепочка: модель → generate_xy → process_xy_pair → solve_1plane → verify
#[test]
fn e2e_single_plane_xy_pair() {
    let f_rot = 50.0;
    let sample_rate = 2000.0;
    let n_samples = 4000;
    let noise_rms = 2500.0;

    let alpha = Complex64::from_polar(10000.0, 20.0_f64.to_radians());
    let imbalance = Complex64::from_polar(12.0, 130.0_f64.to_radians());

    // --- Пуск 0 ---
    let v0_true = vibration_from_imbalance(alpha, imbalance);
    let (x0, y0) = generate_xy_signals(v0_true, f_rot, sample_rate, n_samples, noise_rms, 333);
    let res0 = process_xy_pair(&x0, &y0, sample_rate, 1000.0, 5000.0);
    let v0 = res0.vibro;

    // RPM должен определяться корректно
    assert!(
        (res0.rpm - 3000.0).abs() < 60.0,
        "RPM: {:.0}, expected 3000",
        res0.rpm
    );

    // --- Пуск 1: пробный груз ---
    let trial = TrialWeight::new(6.0, PI / 4.0); // 6г @ 45°
    let imbalance_trial = imbalance + trial.to_complex();
    let v1_true = vibration_from_imbalance(alpha, imbalance_trial);
    let (x1, y1) = generate_xy_signals(v1_true, f_rot, sample_rate, n_samples, noise_rms, 444);
    let res1 = process_xy_pair(&x1, &y1, sample_rate, 1000.0, 5000.0);
    let v1 = res1.vibro;

    // --- Решение ---
    let correction = solve_1plane(v0, v1, trial).unwrap();
    println!(
        "E2E 1-plane XY: imbalance={:.1}г@{:.0}°, correction={}",
        imbalance.norm(),
        imbalance.arg().to_degrees(),
        correction
    );

    // --- Верификация ---
    let w = Complex64::from_polar(correction.mass_g.as_f64(), correction.angle_rad.as_f64());
    let residual = imbalance + w;
    let v_res = vibration_from_imbalance(alpha, residual);
    let reduction = v_res.amplitude / v0_true.amplitude;
    assert!(
        reduction < 0.10,
        "XY остаток: {:.1}% (V0={:.1}, Vres={:.1})",
        reduction * 100.0,
        v0_true.amplitude,
        v_res.amplitude
    );
}

/// Двухплоскостная балансировка: 2 датчика, 2 плоскости, 3 пуска.
///
/// Цепочка: модель → generate_single × 2 датчика × 3 пуска → FFT → solve_2planes → verify
#[test]
fn e2e_two_plane() {
    let f_rot = 50.0;
    let sample_rate = 2000.0;
    let n_samples = 4000;
    let noise_rms = 2500.0;

    // Матрица влияния 2×2: aij связывает дисбаланс в плоскости j с вибрацией на датчике i
    // Масштаб LSB/г — реалистичный для ADS1256 + velocity transducer
    let a11 = Complex64::from_polar(8000.0, 10.0_f64.to_radians());
    let a12 = Complex64::from_polar(2000.0, -30.0_f64.to_radians());
    let a21 = Complex64::from_polar(1500.0, 45.0_f64.to_radians());
    let a22 = Complex64::from_polar(7000.0, -15.0_f64.to_radians());

    // Начальные дисбалансы в двух плоскостях
    let u1 = Complex64::from_polar(10.0, 40.0_f64.to_radians());
    let u2 = Complex64::from_polar(6.0, 200.0_f64.to_radians());

    // Функция: вибрация на двух датчиках от двух дисбалансов
    let vibro_2sensor = |d1: Complex64, d2: Complex64| -> [VibroVector; 2] {
        [
            VibroVector::from_complex(a11 * d1 + a12 * d2),
            VibroVector::from_complex(a21 * d1 + a22 * d2),
        ]
    };

    // Функция: генерация сигнала и извлечение вибровектора
    let measure = |vv: VibroVector, seed: u64| -> VibroVector {
        let sig = generate_single_channel(vv, f_rot, sample_rate, n_samples, noise_rms, seed);
        let sp = compute_spectrum(&sig, sample_rate);
        sp.vibro_at(f_rot)
    };

    // --- Пуск 0: начальный ---
    let [v0_s1_true, v0_s2_true] = vibro_2sensor(u1, u2);
    let v0_s1 = measure(v0_s1_true, 1001);
    let v0_s2 = measure(v0_s2_true, 1002);

    // --- Пуск 1: пробный груз в плоскости 1 ---
    let trial_1 = TrialWeight::new(5.0, 0.0);
    let [v1_s1_true, v1_s2_true] = vibro_2sensor(u1 + trial_1.to_complex(), u2);
    let v1_s1 = measure(v1_s1_true, 2001);
    let v1_s2 = measure(v1_s2_true, 2002);

    // --- Пуск 2: пробный груз в плоскости 2 ---
    let trial_2 = TrialWeight::new(4.0, PI / 3.0);
    let [v2_s1_true, v2_s2_true] = vibro_2sensor(u1, u2 + trial_2.to_complex());
    let v2_s1 = measure(v2_s1_true, 3001);
    let v2_s2 = measure(v2_s2_true, 3002);

    // --- Решение ---
    let runs = TwoPlaneRuns {
        v0: [v0_s1, v0_s2],
        v1: [v1_s1, v1_s2],
        v2: [v2_s1, v2_s2],
        trial_1,
        trial_2,
    };
    let [c1, c2] = solve_2planes(&runs).unwrap();
    println!(
        "E2E 2-plane:\n  U1={:.1}г@{:.0}°, U2={:.1}г@{:.0}°\n  C1={}, C2={}",
        u1.norm(),
        u1.arg().to_degrees(),
        u2.norm(),
        u2.arg().to_degrees(),
        c1,
        c2
    );

    // --- Верификация: дисбаланс + коррекция → остаток ---
    let w1 = Complex64::from_polar(c1.mass_g.as_f64(), c1.angle_rad.as_f64());
    let w2 = Complex64::from_polar(c2.mass_g.as_f64(), c2.angle_rad.as_f64());
    let [vr_s1, vr_s2] = vibro_2sensor(u1 + w1, u2 + w2);

    let red_s1 = vr_s1.amplitude / v0_s1_true.amplitude;
    let red_s2 = vr_s2.amplitude / v0_s2_true.amplitude;

    assert!(
        red_s1 < 0.05,
        "датчик 1 остаток: {:.1}% (V0={:.1}, Vres={:.1})",
        red_s1 * 100.0,
        v0_s1_true.amplitude,
        vr_s1.amplitude
    );
    assert!(
        red_s2 < 0.05,
        "датчик 2 остаток: {:.1}% (V0={:.1}, Vres={:.1})",
        red_s2 * 100.0,
        v0_s2_true.amplitude,
        vr_s2.amplitude
    );
}

/// Сквозной тест с RPM-гейтированным усреднением.
///
/// Модель: f_rot=50 Гц стабильный (попадает на бин), 10 блоков с разным шумом.
/// Усреднение подавляет шум (SNR растёт как sqrt(N)).
/// Также подмешиваем 2 блока с RPM=4800 (выброс) — должны быть отброшены.
#[test]
fn e2e_with_rpm_averaging() {
    let f_rot = 50.0; // 3000 RPM
    let sample_rate = 2000.0;
    let n_samples = 4000; // 50*4000/2000=100 — точное попадание на бин
    let noise_rms = 2500.0;

    let alpha = Complex64::from_polar(10000.0, 15.0_f64.to_radians());
    let imbalance = Complex64::from_polar(8.0, 60.0_f64.to_radians());
    let v_true = vibration_from_imbalance(alpha, imbalance);
    let rpm_target = 3000.0;

    // 10 хороших блоков с разными шумовыми реализациями
    let mut vector_samples = Vec::new();
    for i in 0..10 {
        let sig = generate_single_channel(v_true, f_rot, sample_rate, n_samples, noise_rms, 500 + i);
        let sp = compute_spectrum(&sig, sample_rate);
        let detected_rpm = sp.estimate_rpm(1000.0, 5000.0);
        let vv = sp.vibro_at(detected_rpm / 60.0);
        vector_samples.push(VectorSample {
            vector: vv,
            rpm:    detected_rpm,
        });
    }
    // 2 плохих блока: другая частота (выброс RPM при провале напряжения)
    for i in 0..2 {
        let sig = generate_single_channel(v_true, 80.0, sample_rate, n_samples, noise_rms, 900 + i);
        let sp = compute_spectrum(&sig, sample_rate);
        vector_samples.push(VectorSample {
            vector: sp.vibro_at(80.0),
            rpm:    Rpm::new(4800.0),
        });
    }

    // RPM-гейтированное усреднение: окно ±100 RPM вокруг 3000
    let avg = rpm_gated_average(&vector_samples, rpm_target, 100.0);
    assert_eq!(avg.n_accepted, 10, "10 хороших блоков принято");
    assert_eq!(avg.n_rejected, 2, "2 выброса отброшены");

    // Усреднение 10 блоков подавляет шум → амплитуда ≈ истинной
    let amp_err = (avg.vector.amplitude - v_true.amplitude).abs() / v_true.amplitude;
    assert!(
        amp_err < 0.03,
        "усреднённая amp: {:.1}, истинная: {:.1}, ошибка: {:.1}%",
        avg.vector.amplitude,
        v_true.amplitude,
        amp_err * 100.0
    );

    // Балансировка
    let v0 = avg.vector;

    let trial = TrialWeight::new(5.0, 0.0);
    let v_trial_true = vibration_from_imbalance(alpha, imbalance + trial.to_complex());

    let mut trial_samples = Vec::new();
    for i in 0..10 {
        let sig = generate_single_channel(v_trial_true, f_rot, sample_rate, n_samples, noise_rms, 700 + i);
        let sp = compute_spectrum(&sig, sample_rate);
        let detected_rpm = sp.estimate_rpm(1000.0, 5000.0);
        let vv = sp.vibro_at(detected_rpm / 60.0);
        trial_samples.push(VectorSample {
            vector: vv,
            rpm:    detected_rpm,
        });
    }

    let avg_trial = rpm_gated_average(&trial_samples, rpm_target, 100.0);
    let v1 = avg_trial.vector;

    let correction = solve_1plane(v0, v1, trial).unwrap();
    let w = Complex64::from_polar(correction.mass_g.as_f64(), correction.angle_rad.as_f64());
    let residual = imbalance + w;
    let v_res = vibration_from_imbalance(alpha, residual);
    let reduction = v_res.amplitude / v_true.amplitude;
    println!(
        "E2E avg: correction={}, residual={:.1}%",
        correction,
        reduction * 100.0
    );
    assert!(
        reduction < 0.05,
        "с усреднением остаток: {:.1}%",
        reduction * 100.0
    );
}

/// Сквозной тест Bode plot: разгон → определение режима → балансировка с учётом фазы.
///
/// Проверяем что Bode анализ корректно определяет критическую скорость
/// и режим работы при данных, полученных из модели.
#[test]
fn e2e_bode_regime_detection() {
    let rpm_cr = 3000.0;
    let zeta = 0.08;
    // Статический дисбаланс (m*e/M) — определяет масштаб амплитуды (в LSB)
    // На резонансе: amp = a0/(2*zeta) = 50000/(0.16) = 312500 LSB, SNR = 312500/2500 = 125
    let a0 = 50000.0;
    let sample_rate = 2000.0;
    let n_samples = 4000;
    let noise_rms = 2500.0;

    // Модель Джеффкотта: амплитуда и фаза vs RPM
    let jeffcott = |rpm: f64| -> VibroVector {
        let r = rpm / rpm_cr;
        let r2 = r * r;
        let amp = a0 * r2 / ((1.0 - r2).powi(2) + (2.0 * zeta * r).powi(2)).sqrt();
        let phase = (2.0 * zeta * r).atan2(1.0 - r2);
        VibroVector::new(amp, phase)
    };

    // Разгон: от 1000 до 5000 RPM с шагом 200
    let mut bode = BodePlot::new();
    for rpm in (1000..=5000).step_by(200) {
        let rpm = rpm as f64;
        let vv_true = jeffcott(rpm);
        let f_rot = rpm / 60.0;

        // Генерируем реальный сигнал, пропускаем через FFT
        let sig = generate_single_channel(vv_true, f_rot, sample_rate, n_samples, noise_rms, rpm as u64);
        let sp = compute_spectrum(&sig, sample_rate);
        let vv_measured = sp.vibro_at(f_rot);

        bode.push(BodePoint {
            rpm: Rpm::new(rpm),
            vector: vv_measured,
        });
    }

    // Определяем критическую скорость
    let detected_cr = bode.critical_rpm();
    assert!(
        (detected_cr - rpm_cr).abs() < 300.0,
        "critical RPM: {detected_cr:.0}, expected {rpm_cr:.0}"
    );

    // Режимы
    assert_eq!(bode.regime_at(1500.0), Regime::SubResonance);
    assert_eq!(bode.regime_at(3000.0), Regime::Resonance);
    assert_eq!(bode.regime_at(4500.0), Regime::SuperResonance);

    // Анализ
    let analysis = bode.analyze();
    println!(
        "E2E Bode: critical={:.0} RPM, zeta={:.3}, peak_amp={:.1}",
        analysis.critical_rpm, analysis.damping_ratio, analysis.peak_amplitude
    );

    // zeta должен быть близок к 0.08
    assert!(
        (analysis.damping_ratio - zeta).abs() < 0.03,
        "zeta: {:.3}, expected {zeta}",
        analysis.damping_ratio
    );
}

/// Двухплоскостная балансировка через X/Y пары (полная цепочка).
///
/// Цепочка: модель 2×2 → generate_xy × 2 опоры × 3 пуска → xy_pair → solve_2planes → verify
#[test]
fn e2e_two_plane_xy_pair() {
    let f_rot = 50.0;
    let sample_rate = 2000.0;
    let n_samples = 4000;
    let noise_rms = 2500.0;

    // Матрица влияния (LSB/г)
    let a11 = Complex64::from_polar(8000.0, 10.0_f64.to_radians());
    let a12 = Complex64::from_polar(2000.0, -30.0_f64.to_radians());
    let a21 = Complex64::from_polar(1500.0, 45.0_f64.to_radians());
    let a22 = Complex64::from_polar(7000.0, -15.0_f64.to_radians());

    let u1 = Complex64::from_polar(10.0, 40.0_f64.to_radians());
    let u2 = Complex64::from_polar(6.0, 200.0_f64.to_radians());

    let vibro_2 = |d1: Complex64, d2: Complex64| -> [VibroVector; 2] {
        [
            VibroVector::from_complex(a11 * d1 + a12 * d2),
            VibroVector::from_complex(a21 * d1 + a22 * d2),
        ]
    };

    // Измерение: генерируем X/Y пару для каждой опоры, пропускаем через process_xy_pair
    let measure_xy = |vv: VibroVector, seed: u64| -> VibroVector {
        let (x, y) = generate_xy_signals(vv, f_rot, sample_rate, n_samples, noise_rms, seed);
        process_xy_pair(&x, &y, sample_rate, 1000.0, 5000.0).vibro
    };

    // --- 3 пуска ---
    let [v0_1, v0_2] = vibro_2(u1, u2);
    let v0 = [measure_xy(v0_1, 4001), measure_xy(v0_2, 4002)];

    let trial_1 = TrialWeight::new(5.0, 0.0);
    let [v1_1, v1_2] = vibro_2(u1 + trial_1.to_complex(), u2);
    let v1 = [measure_xy(v1_1, 5001), measure_xy(v1_2, 5002)];

    let trial_2 = TrialWeight::new(4.0, PI / 3.0);
    let [v2_1, v2_2] = vibro_2(u1, u2 + trial_2.to_complex());
    let v2 = [measure_xy(v2_1, 6001), measure_xy(v2_2, 6002)];

    let runs = TwoPlaneRuns {
        v0,
        v1,
        v2,
        trial_1,
        trial_2,
    };
    let [c1, c2] = solve_2planes(&runs).unwrap();

    println!(
        "E2E 2-plane XY:\n  U1={:.1}@{:.0}°, U2={:.1}@{:.0}°\n  C1={}, C2={}",
        u1.norm(),
        u1.arg().to_degrees(),
        u2.norm(),
        u2.arg().to_degrees(),
        c1,
        c2
    );

    // Верификация
    let w1 = Complex64::from_polar(c1.mass_g.as_f64(), c1.angle_rad.as_f64());
    let w2 = Complex64::from_polar(c2.mass_g.as_f64(), c2.angle_rad.as_f64());
    let [vr1, vr2] = vibro_2(u1 + w1, u2 + w2);

    // X/Y пара вносит чуть больше шума → допуск 10%
    let red1 = vr1.amplitude / v0_1.amplitude;
    let red2 = vr2.amplitude / v0_2.amplitude;
    assert!(red1 < 0.10, "XY 2-plane опора 1 остаток: {:.1}%", red1 * 100.0);
    assert!(red2 < 0.10, "XY 2-plane опора 2 остаток: {:.1}%", red2 * 100.0);
}
