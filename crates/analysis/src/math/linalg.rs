/// Линейная алгебра для двухплоскостной балансировки.
///
/// Нужно решить систему 2×2 с комплексными коэффициентами:
///
///   ┌ a11  a12 ┐   ┌ W1 ┐       ┌ V10 ┐
///   │          │ × │    │  = −   │     │
///   └ a21  a22 ┘   └ W2 ┘       └ V20 ┘
///
/// Коэффициенты aij — influence coefficients (Complex64),
/// Vi0 — начальные вибровекторы, Wi — корректирующие грузы.
use num_complex::Complex64;

/// Матрица 2×2 с комплексными элементами.
/// Индексация: [строка][столбец], строки = датчики, столбцы = плоскости.
pub struct Mat2x2 {
    pub data: [[Complex64; 2]; 2],
}

impl Mat2x2 {
    pub fn new(a11: Complex64, a12: Complex64, a21: Complex64, a22: Complex64) -> Self {
        Self {
            data: [[a11, a12], [a21, a22]],
        }
    }

    /// Определитель: det = a11*a22 - a12*a21.
    pub fn det(&self) -> Complex64 {
        self.data[0][0] * self.data[1][1] - self.data[0][1] * self.data[1][0]
    }

    /// Норма Фробениуса: sqrt(sum |a_ij|^2).
    pub fn frobenius_norm(&self) -> f64 {
        self.data
            .iter()
            .flat_map(|row| row.iter())
            .map(|c| c.norm_sqr())
            .sum::<f64>()
            .sqrt()
    }

    /// Обратная матрица. Паникует если det ≈ 0 (матрица вырождена).
    ///
    /// Предусловие: матрица должна быть невырожденной (|det| >> 0).
    /// Физически это означает: пробные грузы давали различимое изменение вибрации
    /// на обоих датчиках, и плоскости коррекции не имеют одинакового влияния.
    pub fn inverse(&self) -> Mat2x2 {
        let d = self.det();
        assert!(
            d.norm() > 1e-30,
            "матрица вырождена (det={d}): пробные грузы дали слабый или одинаковый отклик"
        );
        let inv_d = Complex64::new(1.0, 0.0) / d;
        Mat2x2::new(
            self.data[1][1] * inv_d,
            -self.data[0][1] * inv_d,
            -self.data[1][0] * inv_d,
            self.data[0][0] * inv_d,
        )
    }

    /// Приближённая обусловленность по норме Фробениуса.
    /// Чем больше значение, тем сильнее шум в измерениях раздувает решение.
    pub fn condition_frobenius(&self) -> f64 {
        let inv = self.inverse();
        self.frobenius_norm() * inv.frobenius_norm()
    }

    /// Безопасная версия condition number.
    /// Возвращает None для вырожденной или почти вырожденной матрицы.
    pub fn condition_frobenius_checked(&self) -> Option<f64> {
        if self.det().norm() <= 1e-30 {
            return None;
        }
        Some(self.condition_frobenius())
    }

    /// Умножение матрицы на вектор-столбец [v0, v1].
    pub fn mul_vec(&self, v: [Complex64; 2]) -> [Complex64; 2] {
        [
            self.data[0][0] * v[0] + self.data[0][1] * v[1],
            self.data[1][0] * v[0] + self.data[1][1] * v[1],
        ]
    }
}

/// Решение 2×2 системы для двухплоскостной балансировки.
///
/// Входные данные:
///   - `a`: матрица influence coefficients [[a11, a12], [a21, a22]]
///   - `v0`: начальные вибрации [V10, V20] (до пробных грузов)
///
/// Возвращает [W1, W2] — корректирующие грузы (комплексные).
/// |W| = масса (г), arg(W) = угол установки (рад).
///
/// Формула: W = -A⁻¹ × V0
pub fn solve_2plane(a: &Mat2x2, v0: [Complex64; 2]) -> [Complex64; 2] {
    let inv = a.inverse();
    let w = inv.mul_vec(v0);
    // Знак минус: корректирующий груз компенсирует вибрацию
    [-w[0], -w[1]]
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use super::*;

    #[test]
    fn test_identity_inverse() {
        let one = Complex64::new(1.0, 0.0);
        let zero = Complex64::new(0.0, 0.0);
        let m = Mat2x2::new(one, zero, zero, one);
        let inv = m.inverse();
        assert!((inv.data[0][0] - one).norm() < 1e-12);
        assert!((inv.data[1][1] - one).norm() < 1e-12);
        assert!(inv.data[0][1].norm() < 1e-12);
        assert!(inv.data[1][0].norm() < 1e-12);
    }

    #[test]
    fn test_solve_2plane_simple() {
        // Если матрица — диагональная: a11=2, a22=3, v0=[6, 9]
        // W1 = -6/2 = -3, W2 = -9/3 = -3
        let two = Complex64::new(2.0, 0.0);
        let three = Complex64::new(3.0, 0.0);
        let zero = Complex64::new(0.0, 0.0);
        let a = Mat2x2::new(two, zero, zero, three);
        let v0 = [Complex64::new(6.0, 0.0), Complex64::new(9.0, 0.0)];
        let [w1, w2] = solve_2plane(&a, v0);
        assert!((w1 - Complex64::new(-3.0, 0.0)).norm() < 1e-12);
        assert!((w2 - Complex64::new(-3.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn test_solve_2plane_complex_coeffs() {
        // Проверка через подстановку: A*W = -V0
        let a11 = Complex64::from_polar(2.0, PI / 4.0);
        let a12 = Complex64::from_polar(0.5, -PI / 6.0);
        let a21 = Complex64::from_polar(0.3, PI / 3.0);
        let a22 = Complex64::from_polar(1.8, -PI / 5.0);
        let a = Mat2x2::new(a11, a12, a21, a22);

        let v0 = [Complex64::from_polar(1.0, 0.2), Complex64::from_polar(0.7, -0.5)];

        let [w1, w2] = solve_2plane(&a, v0);

        // Проверяем: A * W должен быть равен -V0
        let check = a.mul_vec([w1, w2]);
        assert!((check[0] + v0[0]).norm() < 1e-10);
        assert!((check[1] + v0[1]).norm() < 1e-10);
    }

    #[test]
    #[should_panic(expected = "матрица вырождена")]
    fn test_singular_matrix_panics() {
        // Вырожденная матрица: строки пропорциональны
        let one = Complex64::new(1.0, 0.0);
        let two = Complex64::new(2.0, 0.0);
        let m = Mat2x2::new(one, two, one, two);
        let _ = m.inverse();
    }
}
