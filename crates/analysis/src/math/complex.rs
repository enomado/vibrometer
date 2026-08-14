/// Вибровектор — комплексное представление вибрации на частоте вращения.
///
/// Хранится в полярной форме (амплитуда + фаза), потому что именно эти
/// физические величины важны для балансировки. Для арифметики конвертируется
/// в декартову (re + j*im) через num-complex.
///
/// Единицы амплитуды — произвольные (мкм, мм/с, м/с²), зависят от датчика.
/// Фаза — радианы, относительно keyphasor (или "плавающий ноль" при X/Y паре).
use num_complex::Complex64;

/// Вибровектор: амплитуда + фаза.
/// Инварианты: amplitude >= 0, phase ∈ (-π, π].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VibroVector {
    /// Амплитуда (пик-пик / 2, т.е. half-peak). Всегда >= 0.
    pub amplitude: f64,
    /// Фаза в радианах. Нормализована в (-π, π].
    pub phase:     f64,
}

impl VibroVector {
    /// Нулевой вибровектор (нет вибрации).
    pub const ZERO: VibroVector = VibroVector {
        amplitude: 0.0,
        phase:     0.0,
    };

    /// Создать вибровектор из амплитуды и фазы.
    /// amplitude должна быть >= 0.
    pub fn new(amplitude: f64, phase_rad: f64) -> Self {
        debug_assert!(amplitude >= 0.0, "амплитуда должна быть >= 0");
        Self {
            amplitude,
            phase: normalize_angle(phase_rad),
        }
    }

    /// Создать вибровектор из декартовых координат (re + j*im).
    pub fn from_complex(c: Complex64) -> Self {
        Self {
            amplitude: c.norm(),
            phase:     c.arg(),
        }
    }

    /// Конвертировать в декартову форму для арифметики.
    pub fn to_complex(self) -> Complex64 {
        Complex64::from_polar(self.amplitude, self.phase)
    }

    /// Угол в градусах (удобно для отображения).
    pub fn phase_deg(self) -> f64 {
        self.phase.to_degrees()
    }
}

impl std::fmt::Display for VibroVector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:.3} ∠ {:.1}°", self.amplitude, self.phase_deg())
    }
}

/// Нормализация угла в (-π, π].
fn normalize_angle(a: f64) -> f64 {
    use std::f64::consts::PI;
    let mut r = a % (2.0 * PI);
    if r > PI {
        r -= 2.0 * PI;
    } else if r <= -PI {
        r += 2.0 * PI;
    }
    r
}

// ─── Арифметика ─────────────────────────────────────────────────────────────
//
// Все операции идут через Complex64, результат конвертируется обратно.
// Это гарантирует корректное сложение фаз.

impl std::ops::Add for VibroVector {
    type Output = VibroVector;
    fn add(self, rhs: VibroVector) -> VibroVector {
        VibroVector::from_complex(self.to_complex() + rhs.to_complex())
    }
}

impl std::ops::Sub for VibroVector {
    type Output = VibroVector;
    fn sub(self, rhs: VibroVector) -> VibroVector {
        VibroVector::from_complex(self.to_complex() - rhs.to_complex())
    }
}

/// Деление вибровектора на скаляр (нормировка).
impl std::ops::Div<f64> for VibroVector {
    type Output = VibroVector;
    fn div(self, rhs: f64) -> VibroVector {
        debug_assert!(rhs != 0.0, "деление на ноль");
        VibroVector::new(self.amplitude / rhs, self.phase)
    }
}

/// Умножение вибровектора на скаляр (масштаб).
impl std::ops::Mul<f64> for VibroVector {
    type Output = VibroVector;
    fn mul(self, rhs: f64) -> VibroVector {
        debug_assert!(
            rhs >= 0.0,
            "отрицательный скаляр изменит фазу на π — используй Complex64"
        );
        VibroVector::new(self.amplitude * rhs, self.phase)
    }
}

/// Деление двух вибровекторов (для вычисления influence coefficient: alpha = ΔV / T).
impl std::ops::Div<VibroVector> for VibroVector {
    type Output = Complex64;
    fn div(self, rhs: VibroVector) -> Complex64 {
        debug_assert!(rhs.amplitude != 0.0, "деление на нулевой вибровектор");
        self.to_complex() / rhs.to_complex()
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    #[test]
    fn test_zero() {
        let v = VibroVector::ZERO;
        assert_eq!(v.amplitude, 0.0);
        let c = v.to_complex();
        assert!((c.re).abs() < 1e-12);
        assert!((c.im).abs() < 1e-12);
    }

    #[test]
    fn test_roundtrip_polar() {
        let v = VibroVector::new(3.0, PI / 4.0);
        let c = v.to_complex();
        let v2 = VibroVector::from_complex(c);
        assert!((v2.amplitude - 3.0).abs() < 1e-12);
        assert!((v2.phase - PI / 4.0).abs() < 1e-12);
    }

    #[test]
    fn test_addition_phase() {
        // Два вектора с одинаковой фазой — амплитуды складываются
        let a = VibroVector::new(1.0, 0.0);
        let b = VibroVector::new(2.0, 0.0);
        let s = a + b;
        assert!((s.amplitude - 3.0).abs() < 1e-12);
        assert!(s.phase.abs() < 1e-12);
    }

    #[test]
    fn test_subtraction_opposite() {
        // Два вектора с противоположной фазой — вычитание даёт сумму амплитуд
        let a = VibroVector::new(3.0, 0.0);
        let b = VibroVector::new(1.0, PI);
        let d = a - b;
        // a - b = 3∠0 - 1∠π = 3 - (-1) = 4
        assert!((d.amplitude - 4.0).abs() < 1e-12);
        assert!(d.phase.abs() < 1e-12);
    }

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(3.0 * PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(-3.0 * PI) - PI).abs() < 1e-12);
        assert!((normalize_angle(0.0)).abs() < 1e-12);
        assert!((normalize_angle(PI)).abs() - PI < 1e-12);
    }

    #[test]
    fn test_influence_coeff_division() {
        // alpha = ΔV / T
        // если ΔV = 2∠30° и T = 1∠0°, то alpha = 2∠30°
        let dv = VibroVector::new(2.0, PI / 6.0);
        let t = VibroVector::new(1.0, 0.0);
        let alpha: Complex64 = dv / t;
        assert!((alpha.norm() - 2.0).abs() < 1e-12);
        assert!((alpha.arg() - PI / 6.0).abs() < 1e-12);
    }
}
