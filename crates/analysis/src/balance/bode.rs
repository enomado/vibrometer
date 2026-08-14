use vibro_types::{
    AngleRad,
    Rpm,
};

/// Bode plot и определение режима работы ротора.
///
/// Bode plot — зависимость амплитуды и фазы 1x вибрации от RPM.
/// Строится при разгоне/торможении (coast-up/coast-down) или
/// накоплением точек на разных оборотах.
///
/// Из Bode plot определяется:
///   - Критическая скорость w_cr (RPM с максимальной амплитудой)
///   - Текущий режим работы (дорезонанс / резонанс / зарезонанс)
///   - Коэффициент демпфирования zeta (ширина пика на уровне -3 dB)
///   - Фазовая коррекция для балансировки в разных режимах
///
/// Физика (из docs/02_regimes.md):
///   phi(r) = atan2(2*zeta*r, 1 - r^2),  r = w/w_cr
///   A(r)   = (m*e/M) * r^2 / sqrt((1-r^2)^2 + (2*zeta*r)^2)
use crate::math::complex::VibroVector;

/// Одна точка Bode plot: вибровектор при заданном RPM.
#[derive(Debug, Clone, Copy)]
pub struct BodePoint {
    pub rpm:    Rpm,
    pub vector: VibroVector,
}

/// Режим работы относительно критической скорости.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Regime {
    /// w << w_cr. Фаза ≈ 0°, амплитуда растёт с RPM².
    SubResonance,
    /// w ≈ w_cr. Фаза ≈ 90°, максимальная амплитуда.
    Resonance,
    /// w >> w_cr. Фаза ≈ 180°, амплитуда стабилизируется.
    SuperResonance,
}

impl std::fmt::Display for Regime {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Regime::SubResonance => write!(f, "дорезонансный (w << w_cr)"),
            Regime::Resonance => write!(f, "резонансный (w ≈ w_cr)"),
            Regime::SuperResonance => write!(f, "зарезонансный (w >> w_cr)"),
        }
    }
}

/// Результат анализа Bode plot.
#[derive(Debug, Clone)]
pub struct BodeAnalysis {
    /// Критическая скорость (RPM), найденная по пику амплитуды.
    pub critical_rpm:   Rpm,
    /// Амплитуда на резонансе (пиковое значение).
    pub peak_amplitude: f64,
    /// Оценка коэффициента демпфирования zeta (из ширины пика).
    pub damping_ratio:  f64,
}

/// Bode plot: коллекция точек (RPM, VibroVector).
///
/// Точки не обязаны быть отсортированы — сортируем при анализе.
/// Можно добавлять по одной в реальном времени.
pub struct BodePlot {
    points: Vec<BodePoint>,
}

impl BodePlot {
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Создать из готового набора точек.
    pub fn from_points(points: Vec<BodePoint>) -> Self {
        Self { points }
    }

    /// Добавить точку.
    pub fn push(&mut self, point: BodePoint) {
        self.points.push(point);
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    /// Точки, отсортированные по RPM.
    pub fn sorted(&self) -> Vec<BodePoint> {
        let mut pts = self.points.clone();
        pts.sort_by(|a, b| a.rpm.partial_cmp(&b.rpm).unwrap());
        pts
    }

    /// Найти критическую скорость: RPM с максимальной амплитудой 1x.
    ///
    /// Нужно минимум 3 точки для осмысленного результата.
    /// Для надёжного определения нужен прогон через резонанс (амплитуда
    /// должна расти, достичь пика и начать снижаться).
    pub fn critical_rpm(&self) -> Rpm {
        assert!(
            self.points.len() >= 3,
            "нужно минимум 3 точки Bode plot для определения w_cr"
        );

        self.points
            .iter()
            .max_by(|a, b| a.vector.amplitude.partial_cmp(&b.vector.amplitude).unwrap())
            .unwrap()
            .rpm
    }

    /// Определить режим работы при заданном RPM.
    ///
    /// Использует отношение r = rpm / rpm_cr:
    ///   r < 0.7  → дорезонанс
    ///   0.7 ≤ r ≤ 1.3 → резонанс
    ///   r > 1.3  → зарезонанс
    ///
    /// Границы выбраны из практики: в зоне 0.7-1.3 фаза меняется
    /// быстро (от ~30° до ~150°), влияние на балансировку максимально.
    pub fn regime_at(&self, current_rpm: impl Into<Rpm>) -> Regime {
        let rpm_cr = self.critical_rpm();
        let r = current_rpm.into().as_f64() / rpm_cr.as_f64();
        if r < 0.7 {
            Regime::SubResonance
        } else if r > 1.3 {
            Regime::SuperResonance
        } else {
            Regime::Resonance
        }
    }

    /// Полный анализ Bode plot: критическая скорость, демпфирование.
    ///
    /// Коэффициент демпфирования zeta оценивается методом полуширины пика (half-power bandwidth):
    ///   zeta = (rpm_high - rpm_low) / (2 * rpm_cr)
    ///
    /// rpm_high, rpm_low — RPM, где амплитуда = peak / sqrt(2) (уровень -3 dB).
    pub fn analyze(&self) -> BodeAnalysis {
        let sorted = self.sorted();
        assert!(sorted.len() >= 3, "нужно минимум 3 точки для анализа");

        // Пик
        let peak_idx = sorted
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.vector.amplitude.partial_cmp(&b.vector.amplitude).unwrap())
            .unwrap()
            .0;

        let peak_amp = sorted[peak_idx].vector.amplitude;
        let critical_rpm = sorted[peak_idx].rpm;

        // Уровень -3 dB = peak / sqrt(2)
        let half_power = peak_amp / 2.0_f64.sqrt();

        // Ищем пересечение уровня half_power слева и справа от пика.
        // Интерполяция линейная между соседними точками.
        let rpm_low = find_crossing(&sorted[..=peak_idx], half_power, Direction::Rising);
        let rpm_high = find_crossing(&sorted[peak_idx..], half_power, Direction::Falling);

        let damping_ratio = match (rpm_low, rpm_high) {
            (Some(lo), Some(hi)) => (hi - lo) / (2.0 * critical_rpm.as_f64()),
            // Если не нашли пересечение -- недостаточно данных.
            // Возвращаем типичное значение для жёстких роторов.
            _ => 0.05,
        };

        BodeAnalysis {
            critical_rpm,
            peak_amplitude: peak_amp,
            damping_ratio,
        }
    }

    /// Теоретическая фаза при заданном RPM, используя модель Джеффкотта.
    ///
    ///   phi(r) = atan2(2*zeta*r, 1 - r^2)
    ///
    /// Возвращает фазовый сдвиг в радианах [0, π].
    /// Для коррекции вибровектора: если балансировка проводится не в дорезонансе,
    /// нужно учесть этот сдвиг.
    pub fn phase_lag(&self, current_rpm: impl Into<Rpm>) -> AngleRad {
        let analysis = self.analyze();
        let r = current_rpm.into().as_f64() / analysis.critical_rpm.as_f64();
        let zeta = analysis.damping_ratio;
        AngleRad::new((2.0 * zeta * r).atan2(1.0 - r * r))
    }

    /// Амплитудный коэффициент усиления (magnification factor) при данном RPM.
    ///
    ///   M(r) = r^2 / sqrt((1-r^2)^2 + (2*zeta*r)^2)
    ///
    /// Нормированная амплитуда: A = (m*e/M) * M(r).
    /// На резонансе (r=1): M ≈ 1/(2*zeta).
    pub fn magnification(&self, current_rpm: impl Into<Rpm>) -> f64 {
        let analysis = self.analyze();
        let r = current_rpm.into().as_f64() / analysis.critical_rpm.as_f64();
        let zeta = analysis.damping_ratio;
        let r2 = r * r;
        r2 / ((1.0 - r2).powi(2) + (2.0 * zeta * r).powi(2)).sqrt()
    }
}

/// Направление поиска пересечения уровня.
#[derive(Clone, Copy)]
enum Direction {
    /// Амплитуда растёт (ищем пересечение снизу вверх).
    Rising,
    /// Амплитуда падает (ищем пересечение сверху вниз).
    Falling,
}

/// Линейная интерполяция: найти RPM, где амплитуда пересекает заданный уровень.
fn find_crossing(points: &[BodePoint], level: f64, dir: Direction) -> Option<f64> {
    for w in points.windows(2) {
        let (a0, a1) = (w[0].vector.amplitude, w[1].vector.amplitude);
        let crosses = match dir {
            Direction::Rising => a0 < level && a1 >= level,
            Direction::Falling => a0 >= level && a1 < level,
        };
        if crosses {
            // Линейная интерполяция
            let frac = (level - a0) / (a1 - a0);
            return Some(w[0].rpm.as_f64() + frac * (w[1].rpm.as_f64() - w[0].rpm.as_f64()));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    /// Генерация синтетического Bode plot по модели Джеффкотта.
    ///
    ///   A(r) = A0 * r^2 / sqrt((1-r^2)^2 + (2*zeta*r)^2)
    ///   phi(r) = atan2(2*zeta*r, 1 - r^2)
    ///
    /// A0 = m*e/M — статический дисбаланс / масса ротора.
    fn synthetic_bode(
        rpm_cr: f64,
        zeta: f64,
        a0: f64,
        rpm_range: std::ops::RangeInclusive<f64>,
        n_points: usize,
    ) -> BodePlot {
        let rpm_min = *rpm_range.start();
        let rpm_max = *rpm_range.end();
        let step = (rpm_max - rpm_min) / (n_points - 1) as f64;

        let mut points = Vec::with_capacity(n_points);
        for i in 0..n_points {
            let rpm = rpm_min + i as f64 * step;
            let r = rpm / rpm_cr;
            let r2 = r * r;
            let amp = a0 * r2 / ((1.0 - r2).powi(2) + (2.0 * zeta * r).powi(2)).sqrt();
            let phase = (2.0 * zeta * r).atan2(1.0 - r2);
            points.push(BodePoint {
                rpm:    Rpm::new(rpm),
                vector: VibroVector::new(amp, phase),
            });
        }
        BodePlot::from_points(points)
    }

    #[test]
    fn test_critical_rpm_detection() {
        // w_cr = 3000 RPM, zeta = 0.05 (слабое демпфирование, острый пик)
        let bode = synthetic_bode(3000.0, 0.05, 100.0, 1000.0..=5000.0, 100);
        let rpm_cr = bode.critical_rpm();
        // Допуск: ±1 шаг (40 RPM)
        assert!(
            (rpm_cr - 3000.0).abs() < 50.0,
            "critical RPM: {rpm_cr:.0}, expected 3000"
        );
    }

    #[test]
    fn test_damping_ratio() {
        // zeta = 0.1 — умеренное демпфирование.
        // Метод полуширины должен восстановить zeta ≈ 0.1.
        let bode = synthetic_bode(3000.0, 0.1, 100.0, 500.0..=5500.0, 200);
        let analysis = bode.analyze();
        assert!(
            (analysis.damping_ratio - 0.1).abs() < 0.02,
            "zeta: {:.3}, expected 0.1",
            analysis.damping_ratio
        );
    }

    #[test]
    fn test_regime_sub_resonance() {
        let bode = synthetic_bode(3000.0, 0.05, 100.0, 500.0..=5000.0, 100);
        assert_eq!(bode.regime_at(1500.0), Regime::SubResonance);
    }

    #[test]
    fn test_regime_resonance() {
        let bode = synthetic_bode(3000.0, 0.05, 100.0, 500.0..=5000.0, 100);
        assert_eq!(bode.regime_at(3000.0), Regime::Resonance);
        assert_eq!(bode.regime_at(2500.0), Regime::Resonance);
    }

    #[test]
    fn test_regime_super_resonance() {
        let bode = synthetic_bode(3000.0, 0.05, 100.0, 500.0..=5000.0, 100);
        assert_eq!(bode.regime_at(4500.0), Regime::SuperResonance);
    }

    #[test]
    fn test_phase_lag_values() {
        // Модель Джеффкотта:
        //   r << 1: phi → 0
        //   r = 1:  phi = pi/2
        //   r >> 1: phi → pi
        //
        // Допуск повышен: analyze() определяет zeta из дискретных точек,
        // что вносит погрешность ~5-10%.
        let bode = synthetic_bode(3000.0, 0.05, 100.0, 500.0..=5500.0, 200);

        let phi_low = bode.phase_lag(1000.0); // r = 0.33
        let phi_cr = bode.phase_lag(3000.0); // r = 1.0
        let phi_high = bode.phase_lag(5000.0); // r = 1.67

        assert!(phi_low < PI / 6.0, "phi(r=0.33): {:.2} > pi/6", phi_low);
        assert!(
            (phi_cr - PI / 2.0).abs() < 0.15,
            "phi(r=1.0): {:.3}, expected pi/2={:.3}",
            phi_cr,
            PI / 2.0
        );
        assert!(
            phi_high > 2.3,
            "phi(r=1.67): {:.2}, expected close to pi",
            phi_high
        );
    }

    #[test]
    fn test_magnification_at_resonance() {
        // На резонансе (r=1): M = 1/(2*zeta).
        // При zeta=0.1: M = 5. Допуск 10%: дискретизация Bode plot
        // вносит погрешность в определение zeta.
        let bode = synthetic_bode(3000.0, 0.1, 100.0, 500.0..=5500.0, 200);
        let mag = bode.magnification(3000.0);
        let expected = 1.0 / (2.0 * 0.1);
        assert!(
            (mag - expected).abs() / expected < 0.10,
            "magnification: {mag:.2}, expected {expected:.2}"
        );
    }

    #[test]
    fn test_incremental_building() {
        // Bode plot можно строить по одной точке (реальное время).
        let mut bode = BodePlot::new();
        assert!(bode.is_empty());

        for rpm in (1000..=5000).step_by(100) {
            let r = rpm as f64 / 3000.0;
            let zeta = 0.05;
            let amp = 100.0 * r * r / ((1.0 - r * r).powi(2) + (2.0 * zeta * r).powi(2)).sqrt();
            bode.push(BodePoint {
                rpm:    Rpm::new(rpm as f64),
                vector: VibroVector::new(amp, 0.0),
            });
        }

        assert_eq!(bode.len(), 41);
        let rpm_cr = bode.critical_rpm();
        assert!((rpm_cr - 3000.0).abs() < 100.0);
    }
}
