use num_complex::Complex64;
use vibro_types::{
    Hertz,
    Rpm,
};

/// Восстановление вибровектора из пары ортогональных виброметров (X/Y).
///
/// Два виброметра установлены на одной опоре под 90 друг к другу.
/// Каждый измеряет проекцию вибрации:
///   x(t) = A * cos(w*t + phi)
///   y(t) = A * sin(w*t + phi)
///
/// Из FFT обоих каналов на частоте f_rot:
///   V_x = amplitude_x * e^(j * phase_x)   -- 1x компонента X-канала
///   V_y = amplitude_y * e^(j * phase_y)   -- 1x компонента Y-канала
///
/// Вибровектор: V = V_x + j * V_y
///
/// Фаза "плавает" (нет привязки к абсолютному углу вала), но для
/// балансировки это допустимо -- важна разность фаз между пусками.
use crate::math::complex::VibroVector;
use crate::signal::fft::{
    Spectrum,
    compute_spectrum,
};

/// Результат обработки X/Y пары на одной опоре.
pub struct XYResult {
    /// Вибровектор на частоте 1x: V = V_x + j*V_y.
    pub vibro:      VibroVector,
    /// RPM, определённый из спектра (среднее по X и Y каналам).
    pub rpm:        Rpm,
    /// Спектр X-канала (для дальнейшего анализа гармоник).
    pub spectrum_x: Spectrum,
    /// Спектр Y-канала.
    pub spectrum_y: Spectrum,
}

/// Обработать X/Y пару: из двух каналов сэмплов -> вибровектор + RPM.
///
/// # Аргументы
/// - `samples_x`, `samples_y` -- сырые буферы ADC (одинаковой длины, синхронные)
/// - `sample_rate` -- частота дискретизации (Гц)
/// - `rpm_min`, `rpm_max` -- диапазон ожидаемых оборотов для поиска 1x пика
///
/// RPM определяется по доминирующему пику в спектре X-канала, затем уточняется
/// усреднением с Y-каналом. Вибровектор извлекается на найденной частоте.
pub fn process_xy_pair(
    samples_x: &[f64],
    samples_y: &[f64],
    sample_rate: impl Into<Hertz>,
    rpm_min: impl Into<Rpm>,
    rpm_max: impl Into<Rpm>,
) -> XYResult {
    let sample_rate = sample_rate.into();
    let rpm_min = rpm_min.into();
    let rpm_max = rpm_max.into();
    assert_eq!(
        samples_x.len(),
        samples_y.len(),
        "X и Y каналы должны быть одной длины"
    );

    let sp_x = compute_spectrum(samples_x, sample_rate);
    let sp_y = compute_spectrum(samples_y, sample_rate);

    // RPM: берём среднее оценки из обоих каналов для устойчивости
    let rpm_x = sp_x.estimate_rpm(rpm_min, rpm_max);
    let rpm_y = sp_y.estimate_rpm(rpm_min, rpm_max);
    let rpm = Rpm::new((rpm_x.as_f64() + rpm_y.as_f64()) / 2.0);
    let f_rot = rpm.as_hz();

    // 1x компонента каждого канала -- комплексные FFT-бины.
    let vx = sp_x.vibro_at(f_rot);
    let vy = sp_y.vibro_at(f_rot);

    // Разложение эллиптической орбиты на forward (прямую) и backward (обратную)
    // компоненты вращения.
    //
    // X-канал измеряет: x(t) = Re[ V_x * e^(j*w*t) ]
    // Y-канал измеряет: y(t) = Re[ V_y * e^(j*w*t) ]
    //
    // Комплексная орбита: z(t) = x(t) + j*y(t) = V_fwd*e^(j*w*t) + V_bwd*e^(-j*w*t)
    //
    // Из FFT-компонент на частоте +w:
    //   V_fwd = (V_x + j*V_y) / 2    -- forward (дисбаланс)
    //   V_bwd = (V_x - j*V_y) / 2    -- backward (анизотропия опоры)
    //
    // Вывод: для круговой орбиты x=A*cos(wt+phi), y=A*sin(wt+phi):
    //   Vx = A*e^(j*phi), Vy = A*e^(j*(phi-pi/2))
    //   V_fwd = (Vx + j*Vy)/2 = A*e^(j*phi) -- правильная амплитуда и фаза
    //   V_bwd = 0 (нет обратной компоненты)
    //
    // Для балансировки нужна forward-компонента: она вызвана дисбалансом
    // и линейно зависит от массы/угла корректирующего груза.
    // Backward -- от анизотропии жёсткости опоры, не корректируется грузом.
    let cx = vx.to_complex();
    let cy = vy.to_complex();
    let j = Complex64::new(0.0, 1.0);
    let vibro_fwd = (cx + j * cy) / 2.0;
    let vibro = VibroVector::from_complex(vibro_fwd);

    XYResult {
        vibro,
        rpm,
        spectrum_x: sp_x,
        spectrum_y: sp_y,
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    /// Генерация X/Y пары: идеальный синус в двух проекциях.
    ///
    /// x(t) = A * cos(2*pi*f_rot*t + phi)
    /// y(t) = A * sin(2*pi*f_rot*t + phi)
    ///
    /// Это круговая орбита -- X и Y одинаковой амплитуды, сдвинуты на 90.
    fn generate_xy(
        f_rot: f64,
        amplitude: f64,
        phase: f64,
        sample_rate: f64,
        n_samples: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let dt = 1.0 / sample_rate;
        let mut x = Vec::with_capacity(n_samples);
        let mut y = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let t = i as f64 * dt;
            let angle = 2.0 * PI * f_rot * t + phase;
            x.push(amplitude * angle.cos());
            y.push(amplitude * angle.sin());
        }
        (x, y)
    }

    #[test]
    fn test_xy_pair_rpm() {
        // RPM должен определяться корректно из X/Y пары
        let f_rot = 50.0; // 3000 RPM
        let (x, y) = generate_xy(f_rot, 100_000.0, 0.0, 2000.0, 4000);
        let res = process_xy_pair(&x, &y, 2000.0, 1000.0, 5000.0);
        let expected_rpm = f_rot * 60.0;
        assert!(
            (res.rpm - expected_rpm).abs() < 60.0,
            "RPM: {}, expected {}",
            res.rpm,
            expected_rpm
        );
    }

    #[test]
    fn test_xy_pair_amplitude() {
        // Forward-компонента круговой орбиты.
        // x=A*cos(wt+phi), y=A*sin(wt+phi) → V_fwd = A*e^(j*phi), |V_fwd| = A.
        let amp = 100_000.0;
        let f_rot = 50.0;
        let (x, y) = generate_xy(f_rot, amp, 0.0, 2000.0, 4000);
        let res = process_xy_pair(&x, &y, 2000.0, 1000.0, 5000.0);

        let rel_err = (res.vibro.amplitude - amp).abs() / amp;
        assert!(
            rel_err < 0.10,
            "amplitude: {:.0}, expected: {:.0}, err: {:.1}%",
            res.vibro.amplitude,
            amp,
            rel_err * 100.0
        );
    }

    #[test]
    fn test_xy_pair_phase_difference() {
        // Главное для балансировки: разность фаз между двумя пусками
        // должна сохраняться. Меняем фазу на 90 -- вектор должен повернуться на 90.
        let f_rot = 50.0;
        let amp = 100_000.0;

        let (x1, y1) = generate_xy(f_rot, amp, 0.0, 2000.0, 4000);
        let (x2, y2) = generate_xy(f_rot, amp, PI / 2.0, 2000.0, 4000);

        let r1 = process_xy_pair(&x1, &y1, 2000.0, 1000.0, 5000.0);
        let r2 = process_xy_pair(&x2, &y2, 2000.0, 1000.0, 5000.0);

        // Разность фаз: должна быть ~pi/2
        let dphi = (r2.vibro.phase - r1.vibro.phase).rem_euclid(2.0 * PI);
        let dphi = if dphi > PI { dphi - 2.0 * PI } else { dphi };
        assert!(
            (dphi.abs() - PI / 2.0).abs() < 0.15,
            "phase diff: {:.3} rad ({:.1}), expected pi/2={:.3}",
            dphi,
            dphi.to_degrees(),
            PI / 2.0
        );
    }

    #[test]
    fn test_xy_pair_with_noise() {
        // С шумом амплитуда и фаза должны оставаться разумными
        use crate::signal::testgen::{
            SignalParams,
            generate,
        };
        let p_x = SignalParams {
            phase_1x: 0.0, // X = cos
            ..SignalParams::default_2khz_50hz()
        };
        let p_y = SignalParams {
            phase_1x: -PI / 2.0, // Y = sin = cos(x - pi/2)
            ..SignalParams::default_2khz_50hz()
        };
        let sig_x = generate(p_x);
        let sig_y = generate(p_y);

        let res = process_xy_pair(&sig_x.samples, &sig_y.samples, 2000.0, 1000.0, 5000.0);

        // Forward-компонента: амплитуда ~100_000
        let expected_amp = 100_000.0;
        let rel_err = (res.vibro.amplitude - expected_amp).abs() / expected_amp;
        assert!(
            rel_err < 0.15,
            "noisy amplitude: {:.0}, expected: {:.0}, err: {:.1}%",
            res.vibro.amplitude,
            expected_amp,
            rel_err * 100.0
        );
    }
}
