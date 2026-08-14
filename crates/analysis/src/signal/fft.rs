/// FFT-анализ сигнала виброметра.
///
/// Основная задача: из сырого буфера сэмплов выделить 1x компоненту
/// (вибрацию на частоте вращения) и определить RPM из спектра.
///
/// Алгоритм (согласно docs/04_signal_processing.md):
///   1. Набрать блок N точек (N = степень двойки, 1024–4096)
///   2. Применить оконную функцию Hanning
///   3. FFT → спектр
///   4. Найти бин, ближайший к f_rot: bin = round(f_rot * N / f_sample)
///   5. Амплитуда = |X[bin]| * 2/N   (коррекция на оконную функцию и нормировку)
///   6. Фаза = arg(X[bin])
///
/// Для определения RPM без keyphasor: ищем доминирующий пик в диапазоне.
use std::f64::consts::PI;

use num_complex::Complex64;
use rustfft::FftPlanner;
use vibro_types::{
    Hertz,
    Rpm,
    SampleCount,
};

use crate::math::complex::VibroVector;

/// Спектральный анализ: результат FFT одного блока.
pub struct Spectrum {
    /// Комплексные бины FFT. Длина = n_samples / 2 (только положительные частоты).
    /// Нормировка: амплитуда реального синуса = |bins[k]| * 2 / n_samples.
    pub bins:        Vec<Complex64>,
    /// Частота дискретизации (Гц).
    pub sample_rate: Hertz,
    /// Число сэмплов в исходном блоке (N).
    pub n_samples:   SampleCount,
}

impl Spectrum {
    /// Частота бина k (Гц).
    pub fn bin_freq(&self, k: usize) -> Hertz {
        Hertz::new(k as f64 * self.sample_rate.as_f64() / self.n_samples.as_f64())
    }

    /// Разрешение по частоте (Гц/бин).
    pub fn freq_resolution(&self) -> Hertz {
        Hertz::new(self.sample_rate.as_f64() / self.n_samples.as_f64())
    }

    /// Амплитуда бина k (в единицах входного сигнала, half-peak).
    /// Применена коррекция нормировки FFT и оконной функции Hanning (коэфф. 0.5).
    ///
    /// Для k=0 (DC) коррекция другая — не делится на 2. DC нас обычно не интересует.
    pub fn amplitude(&self, k: usize) -> f64 {
        // Hanning window: коэффициент усиления = 0.5 → компенсируем умножением на 2.
        // Нормировка FFT: делим на n_samples.
        // Зеркальная симметрия: умножаем на 2 (берём только половину спектра).
        // Итого: |bins[k]| * 2 / n_samples * 2 = |bins[k]| * 4 / n_samples.
        // Но коэффициент окна 0.5 уже учтён в этих 4 — итоговая формула:
        // amplitude = |bins[k]| * 2 / n_samples / 0.5 = |bins[k]| * 4 / n_samples
        //
        // Для стандартного Hanning: COHERENT_GAIN = 0.5 → поправка = 1/0.5 = 2.
        // amplitude = |bins[k]| * 2 / n_samples * 2 (зеркало) / 1 (DC не удваивается)
        //           = |bins[k]| * 4 / n_samples
        //
        // На практике: сравниваем с testgen и подтверждаем в тестах.
        self.bins[k].norm() * 4.0 / self.n_samples.as_f64()
    }

    /// Фаза бина k (радианы).
    pub fn phase(&self, k: usize) -> f64 {
        self.bins[k].arg()
    }

    /// Индекс бина, ближайшего к заданной частоте (Гц).
    pub fn freq_to_bin(&self, freq_hz: impl Into<Hertz>) -> usize {
        let freq_hz = freq_hz.into();
        let k = (freq_hz.as_f64() * self.n_samples.as_f64() / self.sample_rate.as_f64()).round() as usize;
        k.min(self.bins.len() - 1)
    }

    /// Вибровектор на частоте f_rot (ближайший бин).
    pub fn vibro_at(&self, f_rot_hz: impl Into<Hertz>) -> VibroVector {
        let k = self.freq_to_bin(f_rot_hz);
        VibroVector::new(self.amplitude(k), self.phase(k))
    }

    /// Вибровектор с интерполированной амплитудой.
    ///
    /// При нестабильных оборотах частота не попадает точно на бин FFT →
    /// spectral leakage занижает амплитуду. `peak_interp` уточняет
    /// амплитуду параболической аппроксимацией пика.
    /// Фаза берётся из центрального бина (интерполяция фазы ненадёжна).
    pub fn vibro_at_interp(&self, f_rot_hz: impl Into<Hertz>) -> VibroVector {
        let k = self.freq_to_bin(f_rot_hz);
        let (_, amp_interp) = self.peak_interp(k);
        VibroVector::new(amp_interp, self.phase(k))
    }

    /// Поиск доминирующего пика в диапазоне частот [f_min, f_max] (Гц).
    ///
    /// Возвращает (частота пика, амплитуда) с точностью до одного бина.
    /// Для субдискретной точности использовать `dominant_peak_interp`.
    pub fn dominant_peak(&self, f_min: impl Into<Hertz>, f_max: impl Into<Hertz>) -> (Hertz, f64) {
        let f_min = f_min.into();
        let f_max = f_max.into();
        let k_min = self.freq_to_bin(f_min).max(1); // пропускаем DC (k=0)
        let k_max = self.freq_to_bin(f_max).min(self.bins.len() - 1);

        let mut best_k = k_min;
        let mut best_amp = 0.0_f64;
        for k in k_min..=k_max {
            let amp = self.amplitude(k);
            if amp > best_amp {
                best_amp = amp;
                best_k = k;
            }
        }
        (self.bin_freq(best_k), best_amp)
    }

    /// Параболическая интерполяция пика по трём бинам (k-1, k, k+1).
    ///
    /// Уточняет частоту и амплитуду пика с субдискретной точностью.
    /// Метод: аппроксимация log-амплитуд трёх соседних бинов параболой,
    /// вершина параболы = истинное положение пика.
    ///
    /// Формула (Gasior, Gonzalez, "Improving FFT Frequency Measurement Resolution"):
    ///   delta = 0.5 * (a_left - a_right) / (a_left - 2*a_center + a_right)
    ///   f_true = (k + delta) * sample_rate / N
    ///
    /// где a_left, a_center, a_right — амплитуды бинов k-1, k, k+1.
    ///
    /// Возвращает (уточнённая частота, уточнённая амплитуда).
    /// Если пик на краю спектра (k=0 или k=last) — возвращает без интерполяции.
    pub fn peak_interp(&self, k: usize) -> (Hertz, f64) {
        // Крайние бины: интерполяция невозможна
        if k == 0 || k >= self.bins.len() - 1 {
            return (self.bin_freq(k), self.amplitude(k));
        }

        let a_left = self.amplitude(k - 1);
        let a_center = self.amplitude(k);
        let a_right = self.amplitude(k + 1);

        // Знаменатель параболы. Если ≈0 — плоская вершина, дельта неопределена.
        let denom = a_left - 2.0 * a_center + a_right;
        if denom.abs() < 1e-30 {
            return (self.bin_freq(k), a_center);
        }

        // Смещение вершины параболы от центрального бина.
        // delta ∈ (-0.5, 0.5) при корректных данных.
        let delta = 0.5 * (a_left - a_right) / denom;

        let f_true = Hertz::new((k as f64 + delta) * self.sample_rate.as_f64() / self.n_samples.as_f64());

        // Уточнённая амплитуда: значение параболы в вершине.
        // A_interp = a_center - denom * delta^2 / 4
        // Но проще: для Hanning-окна поправка небольшая, берём a_center
        // как нижнюю оценку (leakage занижает пик). Точная формула:
        let a_interp = a_center - 0.25 * (a_left - a_right) * delta;

        (f_true, a_interp)
    }

    /// Поиск доминирующего пика с параболической интерполяцией.
    ///
    /// Сначала находит дискретный пик, затем уточняет через `peak_interp`.
    /// Возвращает (уточнённая частота, уточнённая амплитуда).
    pub fn dominant_peak_interp(&self, f_min: impl Into<Hertz>, f_max: impl Into<Hertz>) -> (Hertz, f64) {
        let f_min = f_min.into();
        let f_max = f_max.into();
        let k_min = self.freq_to_bin(f_min).max(1);
        let k_max = self.freq_to_bin(f_max).min(self.bins.len() - 1);

        let mut best_k = k_min;
        let mut best_amp = 0.0_f64;
        for k in k_min..=k_max {
            let amp = self.amplitude(k);
            if amp > best_amp {
                best_amp = amp;
                best_k = k;
            }
        }
        self.peak_interp(best_k)
    }

    /// Оценка RPM из спектра (без keyphasor).
    ///
    /// Ищет доминирующий пик в диапазоне [rpm_min, rpm_max] и возвращает RPM.
    /// Использует параболическую интерполяцию для субдискретной точности.
    /// Без интерполяции: Δf = sample_rate / N → ΔRPM ≈ 29 об/мин при N=4096, fs=2000.
    /// С интерполяцией: точность улучшается в ~10-20 раз.
    pub fn estimate_rpm(&self, rpm_min: impl Into<Rpm>, rpm_max: impl Into<Rpm>) -> Rpm {
        let rpm_min = rpm_min.into();
        let rpm_max = rpm_max.into();
        let (f_peak, _amp) = self.dominant_peak_interp(rpm_min.as_hz(), rpm_max.as_hz());
        f_peak.as_rpm()
    }
}

/// Оконная функция Hanning.
///
/// w[n] = 0.5 * (1 - cos(2π*n/(N-1)))
///
/// COHERENT_GAIN = 0.5 (среднее значение окна).
/// Применяется к сигналу поточечно перед FFT.
fn hanning_window(n: usize) -> Vec<f64> {
    (0..n)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos()))
        .collect()
}

/// Вычислить спектр блока сэмплов.
///
/// Предусловие: samples.len() >= 2, желательно степень двойки (1024, 2048, 4096).
/// Если длина не степень двойки — rustfft всё равно работает, но медленнее.
///
/// Применяет оконную функцию Hanning перед FFT.
pub fn compute_spectrum(samples: &[f64], sample_rate: impl Into<Hertz>) -> Spectrum {
    let sample_rate = sample_rate.into();
    let n = samples.len();
    assert!(n >= 2, "нужно хотя бы 2 сэмпла");

    let window = hanning_window(n);

    // Применяем окно и преобразуем в Complex64 для rustfft
    let mut buf: Vec<Complex64> = samples
        .iter()
        .zip(window.iter())
        .map(|(&s, &w)| Complex64::new(s * w, 0.0))
        .collect();

    let mut planner = FftPlanner::new();
    let fft = planner.plan_fft_forward(n);
    fft.process(&mut buf);

    // Берём только первую половину (положительные частоты)
    let half = n / 2;
    let bins = buf[..=half].to_vec();

    Spectrum {
        bins,
        sample_rate,
        n_samples: SampleCount(n),
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;
    use crate::signal::testgen::{
        SignalParams,
        generate,
    };

    /// Допустимая погрешность амплитуды (5%).
    /// При Hanning-окне и точном попадании на бин — погрешность < 1%.
    /// Небольшая погрешность из-за spectral leakage при нецелом числе периодов.
    const AMP_TOL: f64 = 0.05;

    #[test]
    fn test_spectrum_length() {
        let samples: Vec<f64> = (0..1024).map(|i| (i as f64).sin()).collect();
        let sp = compute_spectrum(&samples, 1000.0);
        assert_eq!(sp.bins.len(), 513); // N/2 + 1
    }

    #[test]
    fn test_single_sine_amplitude() {
        // Чистый синус без шума — амплитуда должна восстанавливаться точно.
        // Выбираем f_rot = 50 Гц, sample_rate = 2000, N = 4000.
        // 50 * 4000 / 2000 = 100 — точно попадаем на бин (целое число периодов).
        let n = 4000;
        let sample_rate = 2000.0;
        let f_rot = 50.0;
        let amp = 100_000.0;
        let phase = PI / 4.0;

        let samples: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * f_rot * i as f64 / sample_rate + phase).sin())
            .collect();

        let sp = compute_spectrum(&samples, sample_rate);
        let k = sp.freq_to_bin(f_rot);
        let recovered_amp = sp.amplitude(k);

        assert!(
            (recovered_amp - amp).abs() / amp < AMP_TOL,
            "амплитуда: восстановлено {recovered_amp:.1}, ожидалось {amp:.1}, ошибка {:.1}%",
            (recovered_amp - amp).abs() / amp * 100.0
        );
    }

    #[test]
    fn test_phase_recovery() {
        // Проверяем что фаза двух сигналов с разной фазой восстанавливается
        // стабильно и их разность соответствует ожидаемой.
        // Используем N=4000 (точное попадание на бин: 50*4000/2000=100).
        let n = 4000;
        let sample_rate = 2000.0;
        let f_rot = 50.0;
        let amp = 100_000.0;

        // Два сигнала с разностью фаз 90° (π/2)
        let phase_a = PI / 6.0; // 30°
        let phase_b = phase_a + PI / 2.0; // 120°

        let make = |phase: f64| -> Vec<f64> {
            (0..n)
                .map(|i| amp * (2.0 * PI * f_rot * i as f64 / sample_rate + phase).sin())
                .collect()
        };

        let sp_a = compute_spectrum(&make(phase_a), sample_rate);
        let sp_b = compute_spectrum(&make(phase_b), sample_rate);

        let vv_a = sp_a.vibro_at(f_rot);
        let vv_b = sp_b.vibro_at(f_rot);

        // Разность фаз должна быть π/2 (с точностью 0.05 рад)
        let phase_diff = {
            let d = (vv_b.phase - vv_a.phase).rem_euclid(2.0 * PI);
            if d > PI { d - 2.0 * PI } else { d }
        };
        assert!(
            (phase_diff - PI / 2.0).abs() < 0.05,
            "разность фаз: {:.3} рад ({:.1}°), ожидалось π/2={:.3} рад",
            phase_diff,
            phase_diff.to_degrees(),
            PI / 2.0
        );
    }

    #[test]
    fn test_estimate_rpm_no_noise() {
        // Без шума RPM должен определяться точно с точностью до разрешения FFT.
        // Δf = 2000/4096 ≈ 0.488 Гц → ΔRPM ≈ 29 об/мин.
        let p = SignalParams {
            noise_rms: 0.0,
            amp_2x: 0.0,
            ..SignalParams::default_2khz_50hz()
        };
        let f_rot = p.f_rot;
        let sr = p.sample_rate;
        let sig = generate(p);
        let sp = compute_spectrum(&sig.samples, sr);
        let rpm = sp.estimate_rpm(1000.0, 4000.0);
        let expected_rpm = f_rot * 60.0; // 3000
        // Допуск: 2 бина = 2 * (sr/n) * 60 ≈ 58 об/мин
        assert!(
            (rpm - expected_rpm).abs() < 60.0,
            "RPM: {rpm:.1}, ожидалось {expected_rpm:.1}"
        );
    }

    #[test]
    fn test_estimate_rpm_with_noise() {
        // При SNR ≈ 40 (amp=100k, noise_rms=2500) пик должен уверенно находиться.
        let p = SignalParams::default_2khz_50hz();
        let f_rot = p.f_rot;
        let sr = p.sample_rate;
        let sig = generate(p);
        let sp = compute_spectrum(&sig.samples, sr);
        let rpm = sp.estimate_rpm(1000.0, 4000.0);
        let expected_rpm = f_rot * 60.0;
        assert!(
            (rpm - expected_rpm).abs() < 60.0,
            "RPM с шумом: {rpm:.1}, ожидалось {expected_rpm:.1}"
        );
    }

    #[test]
    fn test_vibro_vector_from_spectrum() {
        // Проверяем что vibro_at возвращает разумный вибровектор.
        // N=4000 → точное попадание на бин, минимальный leakage.
        let p = SignalParams {
            noise_rms: 0.0,
            amp_2x: 0.0,
            n_samples: 4000,
            ..SignalParams::default_2khz_50hz()
        };
        let expected_amp = p.amp_1x;
        let sr = p.sample_rate;
        let f_rot = p.f_rot;
        let sig = generate(p);
        let sp = compute_spectrum(&sig.samples, sr);
        let vv = sp.vibro_at(f_rot);

        assert!(
            (vv.amplitude - expected_amp).abs() / expected_amp < AMP_TOL,
            "вибровектор: amp={:.1}, ожидалось {expected_amp:.1}",
            vv.amplitude
        );
    }

    #[test]
    fn test_peak_interp_exact_bin() {
        // Частота попадает точно на бин: интерполяция не должна сильно менять результат.
        // 50 Гц * 4000 / 2000 = 100 -- точно на бине.
        let n = 4000;
        let sample_rate = 2000.0;
        let f_rot = 50.0;
        let amp = 100_000.0;

        let samples: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * f_rot * i as f64 / sample_rate).sin())
            .collect();

        let sp = compute_spectrum(&samples, sample_rate);
        let k = sp.freq_to_bin(f_rot);
        let (f_interp, a_interp) = sp.peak_interp(k);

        // Частота должна остаться ~50 Гц
        assert!(
            (f_interp - f_rot).abs() < sp.freq_resolution() * 0.1,
            "interp freq: {f_interp:.3}, expected {f_rot}"
        );
        // Амплитуда должна быть близка к исходной
        assert!(
            (a_interp - amp).abs() / amp < AMP_TOL,
            "interp amp: {a_interp:.0}, expected {amp:.0}"
        );
    }

    #[test]
    fn test_peak_interp_off_bin() {
        // Частота НЕ попадает точно на бин -- интерполяция должна уточнить.
        // f_rot = 51.3 Гц при N=4096, fs=2000: bin = 51.3*4096/2000 = 105.1 (дробный).
        // Без интерполяции: f = 105 * 2000/4096 = 51.27 Гц (ошибка 0.03 Гц).
        // С интерполяцией: ошибка должна быть < 0.01 Гц.
        let n = 4096;
        let sample_rate = 2000.0;
        let f_rot = 51.3;
        let amp = 100_000.0;

        let samples: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * f_rot * i as f64 / sample_rate).sin())
            .collect();

        let sp = compute_spectrum(&samples, sample_rate);

        // Дискретный пик
        let (f_discrete, _) = sp.dominant_peak(40.0, 60.0);
        let err_discrete = (f_discrete - f_rot).abs();

        // Интерполированный пик
        let (f_interp, _) = sp.dominant_peak_interp(40.0, 60.0);
        let err_interp = (f_interp - f_rot).abs();

        // Интерполяция должна быть точнее дискретного бина
        assert!(
            err_interp < err_discrete + 0.001,
            "interp не улучшил: discrete_err={err_discrete:.4}, interp_err={err_interp:.4}"
        );
        // Ошибка интерполяции < 5% от разрешения FFT
        let df = sp.freq_resolution();
        assert!(
            err_interp < df * 0.05,
            "interp freq error: {err_interp:.4} Гц, resolution: {df:.4} Гц"
        );
    }

    #[test]
    fn test_estimate_rpm_interp_precision() {
        // С интерполяцией RPM определяется точнее: ошибка < 5 об/мин
        // вместо ~29 об/мин без интерполяции.
        let n = 4096;
        let sample_rate = 2000.0;
        let f_rot = 51.3; // 3078 RPM -- не попадает точно на бин
        let amp = 100_000.0;

        let samples: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * f_rot * i as f64 / sample_rate).sin())
            .collect();

        let sp = compute_spectrum(&samples, sample_rate);
        let rpm = sp.estimate_rpm(2000.0, 4000.0);
        let expected_rpm = f_rot * 60.0;

        assert!(
            (rpm - expected_rpm).abs() < 5.0,
            "RPM с интерполяцией: {rpm:.1}, ожидалось {expected_rpm:.1}, ошибка {:.1}",
            (rpm - expected_rpm).abs()
        );
    }
}
