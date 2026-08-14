/// Алгоритм Goertzel: вычисление DFT на произвольной частоте.
///
/// В отличие от FFT, Goertzel вычисляет один DFT-бин на заданной частоте
/// без ограничения на целое число периодов в блоке. Это решает проблему
/// spectral leakage при нестабильных оборотах.
///
/// Сложность: O(N) на одну частоту (vs O(N*log(N)) для FFT на все частоты).
/// Для балансировки нам нужен ровно один бин (1x) → Goertzel оптимален.
///
/// Алгоритм (обобщённый Goertzel для произвольной частоты):
///   k = f_target * N / sample_rate  (может быть дробным!)
///   w = 2*pi*k/N
///   coeff = 2*cos(w)
///
///   s0 = s1 = s2 = 0
///   for each sample x[n]:
///     s0 = x[n] + coeff*s1 - s2
///     s2 = s1; s1 = s0
///
///   X(k) = s1 - s2 * e^(-j*w)
///
/// Результат: комплексное число X(k), из которого:
///   amplitude = |X(k)| * 2 / N  (нормировка как у FFT)
///   phase = arg(X(k))
use std::f64::consts::PI;

use num_complex::Complex64;
use vibro_types::{
    Hertz,
    SampleCount,
};

use crate::math::complex::VibroVector;

/// Результат Goertzel: комплексный DFT-бин на заданной частоте.
pub struct GoertzelResult {
    /// Комплексное значение DFT-бина (ненормированное).
    pub raw:       Complex64,
    /// Число сэмплов.
    pub n_samples: SampleCount,
}

impl GoertzelResult {
    /// Амплитуда (нормированная, как у FFT с Hanning).
    /// Без окна: amplitude = |raw| * 2 / N.
    pub fn amplitude(&self) -> f64 {
        self.raw.norm() * 2.0 / self.n_samples.as_f64()
    }

    /// Фаза (радианы).
    pub fn phase(&self) -> f64 {
        self.raw.arg()
    }

    /// Вибровектор.
    pub fn vibro(&self) -> VibroVector {
        VibroVector::new(self.amplitude(), self.phase())
    }
}

/// Goertzel на произвольной частоте, без оконной функции.
///
/// # Аргументы
/// - `samples` — входной сигнал
/// - `target_freq` — целевая частота (Гц)
/// - `sample_rate` — частота дискретизации (Гц)
///
/// Точная формула для дробного k: вычисляем один DFT-коэффициент
/// X(f) = sum_{n=0}^{N-1} x[n] * e^{-j*2*pi*f*n/fs}
/// через рекурсию Goertzel.
pub fn goertzel(samples: &[f64], target_freq: impl Into<Hertz>, sample_rate: impl Into<Hertz>) -> GoertzelResult {
    let target_freq = target_freq.into();
    let sample_rate = sample_rate.into();
    let n = samples.len();
    assert!(n >= 2, "нужно хотя бы 2 сэмпла");

    // Нормированная частота: k = f * N / fs (может быть дробной)
    let k = target_freq.as_f64() * n as f64 / sample_rate.as_f64();
    let w = 2.0 * PI * k / n as f64;
    let coeff = 2.0 * w.cos();

    // Рекурсия Goertzel: s[n] = x[n] + 2*cos(w)*s[n-1] - s[n-2]
    // s[-1] = s[-2] = 0
    // После N шагов (n=0..N-1): s1=s[N-1], s2=s[N-2]
    let mut s1 = 0.0; // s[n-1]
    let mut s2 = 0.0; // s[n-2]

    for &x in samples {
        let s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    // Финальный шаг: X[k] = s[N-1] - s[N-2] * exp(-j*w)
    // = s1 - s2*(cos(w) - j*sin(w))
    // = (s1 - s2*cos(w)) + j*(s2*sin(w))
    //
    // НО: это даёт X[k] = DFT * exp(-j*w) (сдвиг на один шаг).
    // Правильная формула без сдвига — нужен дополнительный шаг рекурсии
    // с x[N]=0, чтобы получить s[N]:
    let s_n = coeff * s1 - s2; // = 0 + coeff*s[N-1] - s[N-2], т.е. x[N]=0
    // Теперь X[k] = s[N] - s[N-1] * exp(-j*w)
    let real = s_n - s1 * w.cos();
    let imag = s1 * w.sin();
    let raw = Complex64::new(real, imag);

    GoertzelResult {
        raw,
        n_samples: SampleCount(n),
    }
}

/// Goertzel с оконной функцией Hanning.
///
/// Применяет окно перед Goertzel. Нужно для подавления spectral leakage
/// от других гармоник (2x, 3x), когда в сигнале есть несколько компонент.
///
/// Нормировка: amplitude = |raw| * 2 / N / coherent_gain
/// Для Hanning: coherent_gain = 0.5 → amplitude = |raw| * 4 / N.
pub fn goertzel_hanning(samples: &[f64], target_freq: impl Into<Hertz>, sample_rate: impl Into<Hertz>) -> GoertzelResult {
    let target_freq = target_freq.into();
    let sample_rate = sample_rate.into();
    let n = samples.len();
    assert!(n >= 2, "нужно хотя бы 2 сэмпла");

    let k = target_freq.as_f64() * n as f64 / sample_rate.as_f64();
    let w = 2.0 * PI * k / n as f64;
    let coeff = 2.0 * w.cos();

    let mut s1 = 0.0;
    let mut s2 = 0.0;

    for (i, &x) in samples.iter().enumerate() {
        // Hanning: w[i] = 0.5 * (1 - cos(2*pi*i/(N-1)))
        let win = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos());
        let s0 = x * win + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    let s_n = coeff * s1 - s2;
    let real = s_n - s1 * w.cos();
    let imag = s1 * w.sin();
    let raw = Complex64::new(real, imag);

    GoertzelResult {
        raw,
        n_samples: SampleCount(n),
    }
}

/// Вибровектор на заданной частоте через Goertzel (без окна).
///
/// Аналог `Spectrum::vibro_at`, но работает на произвольной (дробной) частоте.
pub fn vibro_goertzel(samples: &[f64], target_freq: impl Into<Hertz>, sample_rate: impl Into<Hertz>) -> VibroVector {
    goertzel(samples, target_freq, sample_rate).vibro()
}

/// Вибровектор через Goertzel с окном Hanning.
///
/// Нормировка учитывает Hanning coherent gain = 0.5.
pub fn vibro_goertzel_hanning(
    samples: &[f64],
    target_freq: impl Into<Hertz>,
    sample_rate: impl Into<Hertz>,
) -> VibroVector {
    let res = goertzel_hanning(samples, target_freq, sample_rate);
    // Hanning coherent gain = 0.5 → дополнительная компенсация ×2
    VibroVector::new(res.amplitude() * 2.0, res.phase())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::signal::fft::compute_spectrum;

    const AMP_TOL: f64 = 0.02; // 2% допуск

    /// Прямое вычисление DFT для проверки Goertzel.
    fn direct_dft(samples: &[f64], target_freq: f64, sample_rate: f64) -> Complex64 {
        let n = samples.len();
        let k = target_freq * n as f64 / sample_rate;
        let mut sum = Complex64::new(0.0, 0.0);
        for (i, &x) in samples.iter().enumerate() {
            let angle = -2.0 * PI * k * i as f64 / n as f64;
            sum += Complex64::new(x, 0.0) * Complex64::new(angle.cos(), angle.sin());
        }
        sum
    }

    #[test]
    fn test_goertzel_vs_direct_dft() {
        // Проверяем что Goertzel даёт тот же результат что и прямой DFT.
        let n = 4000;
        let sample_rate = 2000.0;
        let f = 50.0;
        let amp = 100.0;
        let phase = 0.8;

        let samples: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * f * i as f64 / sample_rate + phase).cos())
            .collect();

        let g = goertzel(&samples, f, sample_rate);
        let dft = direct_dft(&samples, f, sample_rate);

        // Goertzel raw должен совпадать с DFT
        let diff = (g.raw - dft).norm();
        assert!(
            diff < 1e-4,
            "goertzel raw={:.4}, dft={:.4}, diff={diff:.6}",
            g.raw,
            dft
        );
    }

    #[test]
    fn test_goertzel_exact_bin() {
        // Частота точно на бине: 50 Гц при N=4000, fs=2000 → k=100 (целое).
        let n = 4000;
        let sample_rate = 2000.0;
        let f = 50.0;
        let amp = 100.0;
        let phase = 0.8;

        let samples: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * f * i as f64 / sample_rate + phase).cos())
            .collect();

        // Прямой DFT как эталон
        let dft = direct_dft(&samples, f, sample_rate);
        let dft_amp = dft.norm() * 2.0 / n as f64;
        let dft_phase = dft.arg();
        println!(
            "DFT: amp={dft_amp:.2} phase={dft_phase:.4} ({:.1}°)",
            dft_phase.to_degrees()
        );

        let g = goertzel(&samples, f, sample_rate);
        assert!(
            (g.amplitude() - amp).abs() / amp < AMP_TOL,
            "goertzel amp: {:.2}, expected {amp}",
            g.amplitude()
        );
        assert!(
            (g.phase() - phase).abs() < 0.02,
            "goertzel phase: {:.3}, expected {phase}",
            g.phase()
        );
    }

    #[test]
    fn test_goertzel_fractional_bin() {
        // Частота НЕ на бине: 51.3 Гц при N=4096, fs=2000.
        // Без окна: Goertzel амплитуда точная, но фаза смещена из-за leakage
        // от зеркальной компоненты (cos → два пика в DFT).
        // С окном Hanning: и амплитуда, и фаза точные.
        let n = 4096;
        let sample_rate = 2000.0;
        let f = 51.3;
        let amp = 100.0;
        let phase = -0.5;

        let samples: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * f * i as f64 / sample_rate + phase).cos())
            .collect();

        // Без окна: амплитуда точная
        let g = goertzel(&samples, f, sample_rate);
        assert!(
            (g.amplitude() - amp).abs() / amp < AMP_TOL,
            "goertzel fractional amp: {:.2}, expected {amp}",
            g.amplitude()
        );

        // С окном Hanning: амплитуда точнее FFT
        let gh = vibro_goertzel_hanning(&samples, f, sample_rate);
        assert!(
            (gh.amplitude - amp).abs() / amp < 0.03,
            "goertzel+hanning amp: {:.2}, expected {amp}",
            gh.amplitude
        );

        // Goertzel+Hanning амплитуда точнее FFT при дробном бине
        let sp = compute_spectrum(&samples, sample_rate);
        let fft_amp = sp.vibro_at(f).amplitude;
        let fft_err = (fft_amp - amp).abs() / amp;
        let g_err = (gh.amplitude - amp).abs() / amp;
        assert!(
            g_err < fft_err + 0.001,
            "goertzel+hanning ({g_err:.4}) хуже FFT ({fft_err:.4})"
        );
    }

    #[test]
    fn test_goertzel_hanning_vs_fft() {
        // С окном Hanning: Goertzel и FFT должны давать одинаковый результат
        // на целом бине.
        let n = 4000;
        let sample_rate = 2000.0;
        let f = 50.0;
        let amp = 100.0;
        let phase = 1.0;

        let samples: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * f * i as f64 / sample_rate + phase).cos())
            .collect();

        let vv_g = vibro_goertzel_hanning(&samples, f, sample_rate);
        let sp = compute_spectrum(&samples, sample_rate);
        let vv_fft = sp.vibro_at(f);

        assert!(
            (vv_g.amplitude - vv_fft.amplitude).abs() / vv_fft.amplitude < 0.02,
            "hanning: goertzel={:.2} vs fft={:.2}",
            vv_g.amplitude,
            vv_fft.amplitude
        );
        assert!(
            (vv_g.phase - vv_fft.phase).abs() < 0.02,
            "hanning phase: goertzel={:.3} vs fft={:.3}",
            vv_g.phase,
            vv_fft.phase
        );
    }

    #[test]
    fn test_goertzel_rejects_other_harmonics() {
        // Сигнал: 1x (50 Гц) + 2x (100 Гц). Goertzel на 50 Гц
        // должен видеть только 1x.
        let n = 4000;
        let sample_rate = 2000.0;
        let f1x = 50.0;
        let amp1x = 100.0;
        let amp2x = 50.0;

        let samples: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                amp1x * (2.0 * PI * f1x * t).cos() + amp2x * (2.0 * PI * 2.0 * f1x * t + 0.7).cos()
            })
            .collect();

        let g = goertzel(&samples, f1x, sample_rate);
        assert!(
            (g.amplitude() - amp1x).abs() / amp1x < AMP_TOL,
            "1x amp: {:.2}, expected {amp1x}",
            g.amplitude()
        );
    }

    #[test]
    fn test_vibro_goertzel_unstable_rpm() {
        // Ключевой тест: при нестабильных оборотах (51.3 Гц вместо 50),
        // Goertzel на точной частоте даёт правильный результат,
        // а FFT с ближайшим бином — нет.
        let n = 4096;
        let sample_rate = 2000.0;
        let f_actual = 51.3; // реальная частота ротора
        let amp = 100.0;
        let phase = 0.3;

        let samples: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * f_actual * i as f64 / sample_rate + phase).cos())
            .collect();

        // Goertzel на точной частоте
        let vv_g = vibro_goertzel(&samples, f_actual, sample_rate);
        let g_err = (vv_g.amplitude - amp).abs() / amp;

        // FFT на ближайшем бине
        let sp = compute_spectrum(&samples, sample_rate);
        let vv_fft = sp.vibro_at(f_actual);
        let fft_err = (vv_fft.amplitude - amp).abs() / amp;

        println!(
            "unstable RPM: goertzel err={:.2}%, fft err={:.2}%",
            g_err * 100.0,
            fft_err * 100.0
        );

        assert!(g_err < 0.02, "goertzel error {:.1}% > 2%", g_err * 100.0);
        // FFT ошибка значительно больше при дробном бине
        assert!(
            g_err < fft_err,
            "goertzel ({:.2}%) не точнее FFT ({:.2}%)",
            g_err * 100.0,
            fft_err * 100.0
        );
    }
}
