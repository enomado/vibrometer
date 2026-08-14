use std::f64::consts::PI;

/// Метод коэффициентов влияния (Influence Coefficient Method).
///
/// Универсальный метод балансировки: работает в любом режиме (до/резонанс/зарезонанс),
/// не требует модели ротора -- чисто экспериментальный.
///
/// ## Одноплоскостная балансировка (1 датчик, 1 плоскость, 2 пуска)
///
///   1. Начальный пуск: V0
///   2. Пробный груз T в плоскости коррекции: V1
///   3. Коэффициент влияния: alpha = (V1 - V0) / T
///   4. Корректирующий груз: W = -V0 / alpha
///
/// ## Двухплоскостная балансировка (2 датчика, 2 плоскости, 3 пуска)
///
///   1. Начальный пуск: V10, V20
///   2. Пробный груз T1 в плоскости 1: V11, V21
///   3. Пробный груз T2 в плоскости 2: V12, V22
///   4. Матрица коэффициентов влияния:
///        a11 = (V11 - V10) / T1
///        a12 = (V12 - V10) / T2
///        a21 = (V21 - V20) / T1
///        a22 = (V22 - V20) / T2
///   5. Корректирующие грузы: решение системы A*W = -V0
use num_complex::Complex64;
use vibro_types::{
    AngleDeg,
    AngleRad,
    MassGrams,
};

use crate::math::complex::VibroVector;
use crate::math::linalg::{
    Mat2x2,
    solve_2plane,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BalanceError {
    TrialResponseTooWeak,
    SingularMatrix,
}

impl std::fmt::Display for BalanceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BalanceError::TrialResponseTooWeak => {
                write!(
                    f,
                    "пробный груз не дал заметного изменения вибрации: груз слишком мал или обороты нестабильны"
                )
            }
            BalanceError::SingularMatrix => {
                write!(
                    f,
                    "матрица коэффициентов влияния вырождена: пробные грузы дали слабый или одинаковый отклик"
                )
            }
        }
    }
}

impl std::error::Error for BalanceError {
}

/// Пробный груз: масса + угол установки на роторе.
///
/// В комплексной форме: T = mass * e^(j*angle).
/// |T| = масса (граммы), arg(T) = угол (радианы).
#[derive(Debug, Clone, Copy)]
pub struct TrialWeight {
    /// Масса груза (граммы).
    pub mass_g:    MassGrams,
    /// Угол установки на роторе (радианы, от произвольного нуля).
    pub angle_rad: AngleRad,
}

impl TrialWeight {
    pub fn new(mass_g: impl Into<MassGrams>, angle_rad: impl Into<AngleRad>) -> Self {
        let mass_g = mass_g.into();
        let angle_rad = angle_rad.into();
        assert!(mass_g.as_f64() > 0.0, "масса пробного груза должна быть > 0");
        Self { mass_g, angle_rad }
    }

    /// Комплексное представление: T = mass * e^(j*angle).
    pub fn to_complex(self) -> Complex64 {
        Complex64::from_polar(self.mass_g.as_f64(), self.angle_rad.as_f64())
    }
}

/// Результат балансировки: корректирующий груз.
#[derive(Debug, Clone, Copy)]
pub struct Correction {
    /// Масса корректирующего груза (граммы).
    pub mass_g:    MassGrams,
    /// Угол установки (радианы).
    pub angle_rad: AngleRad,
}

impl Correction {
    /// Создать из комплексного числа (|W| = масса, arg(W) = угол).
    fn from_complex(c: Complex64) -> Self {
        Self {
            mass_g:    MassGrams::new(c.norm()),
            angle_rad: normalize_angle(AngleRad::new(c.arg())),
        }
    }

    /// Угол в градусах.
    pub fn angle_deg(&self) -> AngleDeg {
        self.angle_rad.to_degrees()
    }
}

impl std::fmt::Display for Correction {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} г @ {}", self.mass_g, self.angle_deg())
    }
}

/// Одноплоскостная балансировка.
///
/// # Аргументы
/// - `v0` -- начальная вибрация (без пробного груза)
/// - `v1` -- вибрация с пробным грузом
/// - `trial` -- параметры пробного груза
///
/// # Возвращает
/// Корректирующий груз (масса + угол).
///
pub fn solve_1plane(
    v0: VibroVector,
    v1: VibroVector,
    trial: TrialWeight,
) -> Result<Correction, BalanceError> {
    let dv = v1 - v0;
    if dv.amplitude <= 1e-10 {
        return Err(BalanceError::TrialResponseTooWeak);
    }

    // Коэффициент влияния: alpha = ΔV / T (комплексное деление)
    let alpha: Complex64 = dv / VibroVector::from_complex(trial.to_complex());

    // Корректирующий груз: W = -V0 / alpha
    let w = -v0.to_complex() / alpha;
    Ok(Correction::from_complex(w))
}

/// Данные трёх пусков для двухплоскостной балансировки.
///
/// Пуск 0: начальный (без грузов).
/// Пуск 1: пробный груз T1 в плоскости 1 (груз из пуска 0 убран).
/// Пуск 2: пробный груз T2 в плоскости 2 (груз из пуска 1 убран).
///
/// Каждый пуск содержит показания двух датчиков (по одному на опору).
pub struct TwoPlaneRuns {
    /// Начальная вибрация: [датчик_1, датчик_2].
    pub v0:      [VibroVector; 2],
    /// Вибрация с пробным грузом в плоскости 1: [датчик_1, датчик_2].
    pub v1:      [VibroVector; 2],
    /// Вибрация с пробным грузом в плоскости 2: [датчик_1, датчик_2].
    pub v2:      [VibroVector; 2],
    /// Пробный груз в плоскости 1.
    pub trial_1: TrialWeight,
    /// Пробный груз в плоскости 2.
    pub trial_2: TrialWeight,
}

/// Двухплоскостная балансировка.
///
/// Вычисляет матрицу 2x2 коэффициентов влияния из трёх пусков,
/// затем решает систему для корректирующих грузов.
///
/// Возвращает [коррекция_плоскость_1, коррекция_плоскость_2].
///
pub fn solve_2planes(runs: &TwoPlaneRuns) -> Result<[Correction; 2], BalanceError> {
    let t1 = runs.trial_1.to_complex();
    let t2 = runs.trial_2.to_complex();

    // Коэффициенты влияния: aij = (Vij - Vi0) / Tj
    // i = номер датчика (строка), j = номер плоскости (столбец)
    let a11 = (runs.v1[0] - runs.v0[0]).to_complex() / t1;
    let a12 = (runs.v2[0] - runs.v0[0]).to_complex() / t2;
    let a21 = (runs.v1[1] - runs.v0[1]).to_complex() / t1;
    let a22 = (runs.v2[1] - runs.v0[1]).to_complex() / t2;

    let a = Mat2x2::new(a11, a12, a21, a22);
    if a.det().norm() <= 1e-30 {
        return Err(BalanceError::SingularMatrix);
    }
    let v0 = [runs.v0[0].to_complex(), runs.v0[1].to_complex()];

    let [w1, w2] = solve_2plane(&a, v0);

    Ok([Correction::from_complex(w1), Correction::from_complex(w2)])
}

/// Диагностика 2-плоскостной балансировки.
#[derive(Debug, Clone, Copy)]
pub struct TwoPlaneDiagnostics {
    /// Отношение |ΔV|/|V0| для пробного груза в плоскости 1 на обоих датчиках.
    pub trial1_ratio_sensor1: f64,
    pub trial1_ratio_sensor2: f64,
    /// Отношение |ΔV|/|V0| для пробного груза в плоскости 2 на обоих датчиках.
    pub trial2_ratio_sensor1: f64,
    pub trial2_ratio_sensor2: f64,
    /// Приближённая обусловленность матрицы коэффициентов влияния.
    pub condition_number:     f64,
    /// Итоговая качественная оценка.
    pub quality:              TwoPlaneQuality,
}

/// Качество 2-плоскостного решения.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TwoPlaneQuality {
    Good,
    Marginal,
    Poor,
}

impl std::fmt::Display for TwoPlaneQuality {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TwoPlaneQuality::Good => write!(f, "OK"),
            TwoPlaneQuality::Marginal => write!(f, "на грани: решение чувствительно к шуму"),
            TwoPlaneQuality::Poor => write!(f, "плохо: слабая разделимость плоскостей"),
        }
    }
}

pub fn diagnose_2planes(runs: &TwoPlaneRuns) -> TwoPlaneDiagnostics {
    let t1 = runs.trial_1.to_complex();
    let t2 = runs.trial_2.to_complex();

    let dv11 = (runs.v1[0] - runs.v0[0]).amplitude;
    let dv21 = (runs.v1[1] - runs.v0[1]).amplitude;
    let dv12 = (runs.v2[0] - runs.v0[0]).amplitude;
    let dv22 = (runs.v2[1] - runs.v0[1]).amplitude;

    let trial1_ratio_sensor1 = if runs.v0[0].amplitude > 1e-10 {
        dv11 / runs.v0[0].amplitude
    } else {
        f64::INFINITY
    };
    let trial1_ratio_sensor2 = if runs.v0[1].amplitude > 1e-10 {
        dv21 / runs.v0[1].amplitude
    } else {
        f64::INFINITY
    };
    let trial2_ratio_sensor1 = if runs.v0[0].amplitude > 1e-10 {
        dv12 / runs.v0[0].amplitude
    } else {
        f64::INFINITY
    };
    let trial2_ratio_sensor2 = if runs.v0[1].amplitude > 1e-10 {
        dv22 / runs.v0[1].amplitude
    } else {
        f64::INFINITY
    };

    let a11 = (runs.v1[0] - runs.v0[0]).to_complex() / t1;
    let a12 = (runs.v2[0] - runs.v0[0]).to_complex() / t2;
    let a21 = (runs.v1[1] - runs.v0[1]).to_complex() / t1;
    let a22 = (runs.v2[1] - runs.v0[1]).to_complex() / t2;
    let a = Mat2x2::new(a11, a12, a21, a22);
    let condition_number = a.condition_frobenius_checked().unwrap_or(f64::INFINITY);

    let weak_response = [
        trial1_ratio_sensor1,
        trial1_ratio_sensor2,
        trial2_ratio_sensor1,
        trial2_ratio_sensor2,
    ]
    .iter()
    .any(|&r| r.is_finite() && r < 0.10);

    let quality = if weak_response || condition_number > 25.0 {
        TwoPlaneQuality::Poor
    } else if condition_number > 10.0 {
        TwoPlaneQuality::Marginal
    } else {
        TwoPlaneQuality::Good
    };

    TwoPlaneDiagnostics {
        trial1_ratio_sensor1,
        trial1_ratio_sensor2,
        trial2_ratio_sensor1,
        trial2_ratio_sensor2,
        condition_number,
        quality,
    }
}

/// Проверка адекватности пробного пуска: изменение вибрации
/// должно составлять 30-200% от начальной.
///
/// Возвращает (отношение |ΔV|/|V0|, рекомендация).
pub fn check_trial_response(v0: VibroVector, v_trial: VibroVector) -> (f64, TrialQuality) {
    let dv = (v_trial - v0).amplitude;
    if v0.amplitude < 1e-10 {
        // Начальная вибрация нулевая -- ротор уже сбалансирован?
        return (f64::INFINITY, TrialQuality::InitialTooLow);
    }
    let ratio = dv / v0.amplitude;
    let quality = if ratio < 0.15 {
        TrialQuality::TooWeak
    } else if ratio > 3.0 {
        TrialQuality::TooStrong
    } else {
        TrialQuality::Good
    };
    (ratio, quality)
}

/// Качество отклика на пробный груз.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TrialQuality {
    /// Хороший отклик (30-200% от начальной вибрации).
    Good,
    /// Слишком слабый отклик: увеличить массу пробного груза.
    TooWeak,
    /// Слишком сильный отклик: уменьшить массу.
    TooStrong,
    /// Начальная вибрация слишком мала (ротор уже сбалансирован).
    InitialTooLow,
}

impl std::fmt::Display for TrialQuality {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TrialQuality::Good => write!(f, "OK"),
            TrialQuality::TooWeak => write!(f, "слишком слабый отклик, увеличить груз"),
            TrialQuality::TooStrong => write!(f, "слишком сильный отклик, уменьшить груз"),
            TrialQuality::InitialTooLow => write!(f, "начальная вибрация ~0, ротор сбалансирован?"),
        }
    }
}

fn normalize_angle(a: AngleRad) -> AngleRad {
    let mut r = a.as_f64() % (2.0 * PI);
    if r < 0.0 {
        r += 2.0 * PI;
    }
    AngleRad::new(r)
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    #[test]
    fn test_solve_1plane_simple() {
        // Дисбаланс: 10г @ 0°. Пробный груз 5г @ 0° удваивает вибрацию.
        // V0 = 100∠0°, V1 = 200∠0° (груз в том же месте, что и дисбаланс).
        // alpha = (200∠0° - 100∠0°) / 5∠0° = 100/5 = 20 (мкм/г)
        // W = -100∠0° / 20 = -5∠0° = 5∠180°
        // Корректирующий груз: 5г @ 180° (напротив дисбаланса).
        let v0 = VibroVector::new(100.0, 0.0);
        let v1 = VibroVector::new(200.0, 0.0);
        let trial = TrialWeight::new(5.0, 0.0);

        let corr = solve_1plane(v0, v1, trial).unwrap();

        assert!(
            (corr.mass_g - 5.0).abs() < 0.01,
            "mass: {:.3}, expected 5.0",
            corr.mass_g
        );
        assert!(
            (corr.angle_rad - PI).abs() < 0.01,
            "angle: {:.3} ({:.1}°), expected π (180°)",
            corr.angle_rad,
            corr.angle_deg()
        );
    }

    #[test]
    fn test_solve_1plane_with_phase() {
        // Дисбаланс: 8г @ 45°. Пробный груз 4г @ 90°.
        // V0 = 80∠45° (дисбаланс создаёт вибрацию в своём направлении, дорезонанс).
        //
        // alpha = 10 мкм/г → вибрация от пробного = 4 * 10∠0° @ 90° = 40∠90°
        // V1 = V0 + alpha*T = 80∠45° + 40∠90°
        //
        // Корректирующий: W = -V0/alpha = -80∠45° / 10∠0° = 8∠(45°+180°) = 8∠225°
        let alpha = Complex64::from_polar(10.0, 0.0); // 10 мкм/г, фаза 0°
        let v0_c = Complex64::from_polar(80.0, PI / 4.0);
        let trial = TrialWeight::new(4.0, PI / 2.0);
        let v1_c = v0_c + alpha * trial.to_complex();

        let v0 = VibroVector::from_complex(v0_c);
        let v1 = VibroVector::from_complex(v1_c);

        let corr = solve_1plane(v0, v1, trial).unwrap();

        // Ожидаемый результат: 8г @ 225° (= 45° + 180°)
        assert!(
            (corr.mass_g - 8.0).abs() < 0.01,
            "mass: {:.3}, expected 8.0",
            corr.mass_g
        );
        let expected_angle = normalize_angle(AngleRad::new(PI / 4.0 + PI));
        assert!(
            (corr.angle_rad - expected_angle).abs() < 0.01,
            "angle: {:.3} ({:.1}°), expected {expected_angle:.3} ({:.1}°)",
            corr.angle_rad,
            corr.angle_deg(),
            expected_angle.to_degrees()
        );
    }

    #[test]
    fn test_solve_1plane_residual() {
        // Проверка через подстановку: после коррекции вибрация ≈ 0.
        // V0 + alpha * W должно быть ≈ 0.
        let v0 = VibroVector::new(150.0, 1.2);
        let alpha = Complex64::from_polar(3.5, -0.7);
        let trial = TrialWeight::new(10.0, 0.5);
        let v1_c = v0.to_complex() + alpha * trial.to_complex();
        let v1 = VibroVector::from_complex(v1_c);

        let corr = solve_1plane(v0, v1, trial).unwrap();

        // Проверяем: V0 + alpha * W = 0
        let w = Complex64::from_polar(corr.mass_g.as_f64(), corr.angle_rad.as_f64());
        let residual = (v0.to_complex() + alpha * w).norm();
        assert!(
            residual < 1e-8,
            "остаточная вибрация: {residual:.3e}, ожидалось ~0"
        );
    }

    #[test]
    fn test_solve_2planes() {
        // Моделируем двухплоскостную: задаём alpha-матрицу,
        // вычисляем V1, V2 из V0 + alpha*T, затем проверяем что коррекция
        // компенсирует V0.
        let a11 = Complex64::from_polar(2.0, 0.3);
        let a12 = Complex64::from_polar(0.5, -0.8);
        let a21 = Complex64::from_polar(0.4, 1.1);
        let a22 = Complex64::from_polar(1.5, -0.2);

        let v0 = [VibroVector::new(120.0, 0.5), VibroVector::new(80.0, -0.3)];

        let trial_1 = TrialWeight::new(5.0, 0.0);
        let trial_2 = TrialWeight::new(8.0, PI / 3.0);

        let t1 = trial_1.to_complex();
        let t2 = trial_2.to_complex();

        // V1 = V0 + A * T1 (пробный груз в плоскости 1)
        let v1 = [
            VibroVector::from_complex(v0[0].to_complex() + a11 * t1),
            VibroVector::from_complex(v0[1].to_complex() + a21 * t1),
        ];
        // V2 = V0 + A * T2 (пробный груз в плоскости 2)
        let v2 = [
            VibroVector::from_complex(v0[0].to_complex() + a12 * t2),
            VibroVector::from_complex(v0[1].to_complex() + a22 * t2),
        ];

        let runs = TwoPlaneRuns {
            v0,
            v1,
            v2,
            trial_1,
            trial_2,
        };
        let [c1, c2] = solve_2planes(&runs).unwrap();

        // Проверяем: V0 + A * W = 0 на обоих датчиках
        let w1 = Complex64::from_polar(c1.mass_g.as_f64(), c1.angle_rad.as_f64());
        let w2 = Complex64::from_polar(c2.mass_g.as_f64(), c2.angle_rad.as_f64());

        let r1 = v0[0].to_complex() + a11 * w1 + a12 * w2;
        let r2 = v0[1].to_complex() + a21 * w1 + a22 * w2;

        assert!(r1.norm() < 1e-6, "остаток на датчике 1: {:.3e}", r1.norm());
        assert!(r2.norm() < 1e-6, "остаток на датчике 2: {:.3e}", r2.norm());
    }

    #[test]
    fn test_check_trial_good() {
        let v0 = VibroVector::new(100.0, 0.0);
        let v_trial = VibroVector::new(150.0, 0.3); // |ΔV| ≈ 54, ratio ≈ 0.54
        let (ratio, quality) = check_trial_response(v0, v_trial);
        assert!(ratio > 0.15 && ratio < 3.0);
        assert_eq!(quality, TrialQuality::Good);
    }

    #[test]
    fn test_check_trial_too_weak() {
        let v0 = VibroVector::new(100.0, 0.0);
        let v_trial = VibroVector::new(101.0, 0.0); // |ΔV| = 1, ratio = 0.01
        let (_, quality) = check_trial_response(v0, v_trial);
        assert_eq!(quality, TrialQuality::TooWeak);
    }

    #[test]
    fn test_check_trial_too_strong() {
        let v0 = VibroVector::new(100.0, 0.0);
        let v_trial = VibroVector::new(500.0, 0.0); // |ΔV| = 400, ratio = 4.0
        let (_, quality) = check_trial_response(v0, v_trial);
        assert_eq!(quality, TrialQuality::TooStrong);
    }

    #[test]
    fn test_diagnose_2planes_good_matrix() {
        let runs = TwoPlaneRuns {
            v0:      [VibroVector::new(100.0, 0.0), VibroVector::new(80.0, 0.2)],
            v1:      [VibroVector::new(150.0, 0.1), VibroVector::new(100.0, 0.3)],
            v2:      [VibroVector::new(110.0, 0.5), VibroVector::new(145.0, -0.2)],
            trial_1: TrialWeight::new(5.0, 0.0),
            trial_2: TrialWeight::new(5.0, PI / 2.0),
        };

        let diag = diagnose_2planes(&runs);
        assert_eq!(diag.quality, TwoPlaneQuality::Good);
        assert!(diag.condition_number.is_finite());
        assert!(diag.condition_number < 10.0, "cond={:.2}", diag.condition_number);
    }

    #[test]
    fn test_diagnose_2planes_poor_matrix() {
        let runs = TwoPlaneRuns {
            v0:      [VibroVector::new(100.0, 0.0), VibroVector::new(100.0, 0.0)],
            v1:      [VibroVector::new(105.0, 0.0), VibroVector::new(105.0, 0.0)],
            v2:      [VibroVector::new(105.0, 0.0), VibroVector::new(105.0, 0.0)],
            trial_1: TrialWeight::new(5.0, 0.0),
            trial_2: TrialWeight::new(5.0, PI / 2.0),
        };

        let diag = diagnose_2planes(&runs);
        assert_eq!(diag.quality, TwoPlaneQuality::Poor);
        assert!(diag.condition_number > 20.0 || !diag.condition_number.is_finite());
    }
}
