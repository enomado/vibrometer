/// Smoothing для keyphasor-тиков: Whittaker smoother (penalized least squares).
///
/// Альтернатива PLL: вместо каузальной фильтрации (PLL видит только прошлое),
/// Whittaker smoother использует ВСЮ запись — non-causal, batch processing.
///
/// Минимизирует:
///   Σ_i (z_i - y_i)² + λ · Σ_i (Δ²z_i)²
///
/// где Δ²z_i = z_{i+2} - 2·z_{i+1} + z_i — вторая разность.
///
/// Эквивалентно решению линейной системы:
///   (I + λ · D₂ᵀ · D₂) · z = y
///
/// D₂ — матрица вторых разностей (n-2)×n.
/// D₂ᵀ·D₂ — pentadiagonal, band=2.
///
/// Преимущества перед PLL:
/// - Нет settling time (видит будущее)
/// - Нет oscillation от Ki (PLL period oscillates при alternating jitter)
/// - На постоянной скорости: z ≈ a + b·i (линейная), интервалы идеально равны
/// - Один параметр λ: больше → глаже

/// Результат сглаживания для одного keyphasor-события.
#[derive(Clone, Debug)]
pub struct SplinePoint {
    /// Индекс оборота (0-based, соответствует kp_ticks[i]).
    pub rev: usize,
    /// Сырой tick этого KP-события.
    pub tick: u64,
    /// Сглаженный tick.
    pub smooth_tick: f64,
    /// Первая производная (тики/оборот = период).
    /// Вычислена как центральная разность smooth_tick.
    pub period: f64,
}

impl SplinePoint {
    /// Частота вращения (Hz).
    pub fn freq(&self, systimer_hz: f64) -> f64 {
        systimer_hz / self.period
    }

    /// RPM.
    pub fn rpm(&self, systimer_hz: f64) -> f64 {
        self.freq(systimer_hz) * 60.0
    }
}

/// Интерполировать фазу для произвольного tick через сглаженный результат.
///
/// Аналог interpolate_phase из pll.rs, но на smooth_tick.
///
/// Фазовая конвенция: points[0] = фаза 0, points[k] = фаза 2π·k.
/// (В отличие от PLL, где pll[0] соответствует kp[1], здесь points[0] = kp[0].)
///
/// Возвращает (freq_hz, phase_rad).
pub fn spline_interpolate_phase(
    points: &[SplinePoint],
    tick: u64,
    systimer_hz: f64,
) -> (f64, f64) {
    assert!(!points.is_empty(), "spline result is empty");

    let tick_f = tick as f64;
    let two_pi = 2.0 * std::f64::consts::PI;

    // До первой точки — экстраполяция.
    if tick_f <= points[0].smooth_tick {
        let freq = systimer_hz / points[0].period;
        let dt = (points[0].smooth_tick - tick_f) / systimer_hz;
        let phase = -two_pi * freq * dt;
        return (freq, phase);
    }

    // После последней — экстраполяция.
    let last = points.last().unwrap();
    if tick_f >= last.smooth_tick {
        let freq = systimer_hz / last.period;
        let dt = (tick_f - last.smooth_tick) / systimer_hz;
        let n_last = (points.len() - 1) as f64;
        let phase = two_pi * n_last + two_pi * freq * dt;
        return (freq, phase);
    }

    // Бинарный поиск интервала: points[j].smooth_tick <= tick_f < points[j+1].smooth_tick.
    let j = points
        .partition_point(|p| p.smooth_tick <= tick_f)
        .saturating_sub(1);
    let a = &points[j];
    let b = &points[j + 1];
    let dt_ab = b.smooth_tick - a.smooth_tick;
    if dt_ab.abs() < 1e-15 {
        let freq = systimer_hz / a.period;
        return (freq, two_pi * j as f64);
    }
    let frac = (tick_f - a.smooth_tick) / dt_ab;
    let sp = a.period + (b.period - a.period) * frac;
    let freq = systimer_hz / sp;
    // points[j] = фаза 2π·j, points[j+1] = фаза 2π·(j+1).
    let phase = two_pi * (j as f64 + frac);
    (freq, phase)
}

/// Конвертировать SplinePoint[] в PllPoint[] для совместимости с order_track.
///
/// aligned_tick = smooth_tick (Whittaker): граница оборота привязана к сглаженному
/// моменту, а не к сырому KP-фронту. Это убирает jitter с опорного сигнала IQ-демодуляции
/// и даёт корректную амплитуду 1x.
///
/// PLL пропускает kp[0] (pll[0] ↔ kp[1]), поэтому spline[0] тоже пропускается.
pub fn spline_to_pll(points: &[SplinePoint]) -> Vec<crate::signal::pll::PllPoint> {
    // Пропускаем points[0] — аналог PLL, который начинает с kp[1].
    points[1..]
        .iter()
        .map(|sp| crate::signal::pll::PllPoint {
            tick: sp.tick,
            // aligned_tick = smooth_tick: граница оборота из Whittaker, без jitter.
            // interpolate_phase() считает что в этой точке фаза = 2π·(j+1),
            // и интерполирует внутри оборота по smooth_period.
            aligned_tick: sp.smooth_tick,
            period: sp.period,
            smooth_period: sp.period, // Whittaker period уже сглажен
        })
        .collect()
}

/// Whittaker smoother по keyphasor-тикам.
///
/// Решает (I + λ·D₂ᵀ·D₂)·z = y через Cholesky pentadiagonal solver.
///
/// `lambda` — параметр сглаживания:
/// - lambda → 0: z ≈ y (без сглаживания)
/// - lambda → ∞: z ≈ линейная аппроксимация y
///
/// Для типичного виброметра с 30% jitter: lambda ≈ 10..1000.
///
/// # Паникует
/// Если `kp_ticks.len() < 3`.
pub fn smoothing_spline(kp_ticks: &[u64], lambda: f64) -> Vec<SplinePoint> {
    let n = kp_ticks.len();
    assert!(n >= 3, "smoothing spline needs at least 3 points");

    let y: Vec<f64> = kp_ticks.iter().map(|&t| t as f64).collect();

    // D₂ᵀ·D₂ для second-difference penalty.
    // D₂ — (n-2)×n матрица: строка k = [0..0, 1, -2, 1, 0..0] начиная с позиции k.
    //
    // D₂ᵀ·D₂ — pentadiagonal n×n матрица. Элементы:
    //   диаг  0: [1, 5, 6, 6, ..., 6, 5, 1]  (с учётом краёв)
    //   диаг ±1: [-2, -4, -4, ..., -4, -2]
    //   диаг ±2: [1, 1, 1, ..., 1]
    //
    // Матрица A = I + λ·D₂ᵀ·D₂.
    // Храним 3 диагонали (симметричная → верхний треугольник):
    //   d[i] — главная, e[i] — ±1, f[i] — ±2.

    let z = whittaker_solve(&y, lambda);

    // Первая производная: центральная разность для внутренних точек,
    // односторонняя на краях.
    let mut period = vec![0.0; n];
    period[0] = z[1] - z[0];
    period[n - 1] = z[n - 1] - z[n - 2];
    for i in 1..n - 1 {
        period[i] = (z[i + 1] - z[i - 1]) / 2.0;
    }

    (0..n)
        .map(|i| SplinePoint {
            rev: i,
            tick: kp_ticks[i],
            smooth_tick: z[i],
            period: period[i],
        })
        .collect()
}

/// Решение (I + λ·D₂ᵀ·D₂)·z = y.
///
/// Pentadiagonal Cholesky (LDLᵀ decomposition для band=2).
/// O(n) time, O(n) memory.
fn whittaker_solve(y: &[f64], lambda: f64) -> Vec<f64> {
    let n = y.len();
    assert!(n >= 3);

    // Строим главную и верхние диагонали A = I + λ·D₂ᵀ·D₂.
    // d[i] — главная диагональ A[i,i].
    // e[i] — A[i, i+1] (first superdiagonal).
    // f[i] — A[i, i+2] (second superdiagonal).
    let mut d = vec![1.0; n];  // I on diagonal
    let mut e = vec![0.0; n];  // off-diagonal starts at 0
    let mut f = vec![0.0; n];

    // D₂ᵀ·D₂ contributions.
    // Строка k матрицы D₂: позиции k, k+1, k+2 с коэффициентами [1, -2, 1].
    // Вклад в D₂ᵀ·D₂:
    //   (k,k)   += 1,  (k,k+1)   += -2, (k,k+2)   += 1
    //   (k+1,k+1) += 4, (k+1,k+2) += -2
    //   (k+2,k+2) += 1
    for k in 0..n - 2 {
        d[k] += lambda * 1.0;
        d[k + 1] += lambda * 4.0;
        d[k + 2] += lambda * 1.0;
        e[k] += lambda * (-2.0);
        e[k + 1] += lambda * (-2.0);
        f[k] += lambda * 1.0;
    }

    // Pentadiagonal LDLᵀ decomposition (in-place).
    // Для band=2 symmetric positive definite матрицы.
    // L — нижнетреугольная с единичной диагональю, bandwidth 2.
    // l1[i] = L[i+1, i], l2[i] = L[i+2, i].
    let mut l1 = vec![0.0; n]; // sub-diagonal 1
    let mut l2 = vec![0.0; n]; // sub-diagonal 2

    // Forward decomposition.
    for i in 0..n {
        // d[i] = A[i,i] - l1[i]*l1[i]*d[i-1] - l2[i]*l2[i]*d[i-2]
        if i >= 1 {
            d[i] -= l1[i] * l1[i] * d[i - 1];
        }
        if i >= 2 {
            d[i] -= l2[i] * l2[i] * d[i - 2];
        }

        // e[i] → l1[i+1], f[i] → l2[i+2]
        if i + 1 < n {
            // e[i] = A[i, i+1] - l2[i+1]*l1[i]*d[i-1]
            if i >= 1 {
                e[i] -= l2[i + 1] * l1[i] * d[i - 1];
            }
            l1[i + 1] = e[i] / d[i];
        }
        if i + 2 < n {
            l2[i + 2] = f[i] / d[i];
        }
    }

    // Solve L·D·Lᵀ·z = y.
    // Step 1: L·w = y (forward substitution).
    let mut w = y.to_vec();
    for i in 0..n {
        if i >= 1 {
            w[i] -= l1[i] * w[i - 1];
        }
        if i >= 2 {
            w[i] -= l2[i] * w[i - 2];
        }
    }

    // Step 2: D·v = w.
    let mut v = w;
    for i in 0..n {
        v[i] /= d[i];
    }

    // Step 3: Lᵀ·z = v (backward substitution).
    let mut z = v;
    for i in (0..n).rev() {
        if i + 1 < n {
            z[i] -= l1[i + 1] * z[i + 1];
        }
        if i + 2 < n {
            z[i] -= l2[i + 2] * z[i + 2];
        }
    }

    z
}

/// Вычислить aligned keyphasor-тики в секундах от start_tick.
pub fn spline_aligned_times(
    points: &[SplinePoint],
    start_tick: u64,
    systimer_hz: f64,
) -> Vec<f64> {
    points
        .iter()
        .map(|p| (p.smooth_tick - start_tick as f64) / systimer_hz)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const SYS_HZ: f64 = 16_000_000.0;

    fn uniform_kp(n_revs: usize, period_ticks: u64, start: u64) -> Vec<u64> {
        (0..=n_revs)
            .map(|i| start + i as u64 * period_ticks)
            .collect()
    }

    fn jittery_kp(
        n_revs: usize,
        period_ticks: u64,
        jitter_frac: f64,
        start: u64,
    ) -> Vec<u64> {
        let mut ticks = vec![start];
        let mut t = start;
        for i in 0..n_revs {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            let dt = (period_ticks as f64 * (1.0 + sign * jitter_frac)) as u64;
            t += dt;
            ticks.push(t);
        }
        ticks
    }

    fn cv_of(vals: &[f64]) -> f64 {
        let mean = vals.iter().sum::<f64>() / vals.len() as f64;
        let std = (vals.iter().map(|x| (x - mean).powi(2)).sum::<f64>()
            / vals.len() as f64)
            .sqrt();
        std / mean
    }

    #[test]
    fn whittaker_uniform_input_unchanged() {
        // Без jitter: сглаживание не должно менять равномерные точки.
        let period = 320_000u64; // 50 Hz
        let kp = uniform_kp(50, period, 0);
        let result = smoothing_spline(&kp, 10.0);

        assert_eq!(result.len(), kp.len());
        for (i, p) in result.iter().enumerate() {
            let expected = i as f64 * period as f64;
            assert!(
                (p.smooth_tick - expected).abs() < 1.0,
                "[{i}] smooth={:.0}, expected={:.0}, diff={:.6}",
                p.smooth_tick,
                expected,
                p.smooth_tick - expected
            );
        }
    }

    #[test]
    fn whittaker_smooths_jitter() {
        // ±30% alternating jitter — должен дать более равномерные интервалы.
        let period = 320_000u64;
        let kp = jittery_kp(200, period, 0.30, 0);
        let result = smoothing_spline(&kp, 100.0);

        let intervals: Vec<f64> = result[10..190]
            .windows(2)
            .map(|w| w[1].smooth_tick - w[0].smooth_tick)
            .collect();

        let cv = cv_of(&intervals);
        eprintln!("whittaker λ=100, jitter=30%: CV = {:.4}%", cv * 100.0);

        // Raw CV = 30%. Whittaker должен значительно уменьшить.
        assert!(
            cv < 0.05,
            "CV = {:.2}%, expected < 5% (raw CV = 30%)",
            cv * 100.0
        );

        // Средний период ≈ raw period (нет bias).
        let mean = intervals.iter().sum::<f64>() / intervals.len() as f64;
        let bias = (mean - period as f64).abs() / period as f64;
        assert!(bias < 0.01, "mean period bias = {:.2}%", bias * 100.0);
    }

    #[test]
    fn whittaker_monotonic() {
        let period = 320_000u64;
        let kp = jittery_kp(100, period, 0.25, 1_000_000);
        let result = smoothing_spline(&kp, 10.0);

        for w in result.windows(2) {
            assert!(
                w[1].smooth_tick > w[0].smooth_tick,
                "not monotonic: {:.0} -> {:.0}",
                w[0].smooth_tick,
                w[1].smooth_tick
            );
        }
    }

    #[test]
    fn whittaker_ramp_tracks() {
        // Разгон 25→50 Hz + 40 плато.
        let mut kp = vec![0u64];
        let mut t = 0u64;
        for i in 0..60 {
            let frac = i as f64 / 60.0;
            let freq = 25.0 + 25.0 * frac;
            let p = (SYS_HZ / freq) as u64;
            t += p;
            kp.push(t);
        }
        let plateau_period = (SYS_HZ / 50.0) as u64;
        for _ in 0..40 {
            t += plateau_period;
            kp.push(t);
        }

        let result = smoothing_spline(&kp, 10.0);

        let last = result.last().unwrap();
        let rpm = last.rpm(SYS_HZ);
        assert!(
            (rpm - 3000.0).abs() < 100.0,
            "final RPM = {:.1}, expected ~3000",
            rpm
        );
    }

    #[test]
    fn whittaker_lambda_effect() {
        // Больше lambda → меньше CV (больше сглаживание).
        let period = 320_000u64;
        let kp = jittery_kp(100, period, 0.30, 0);

        let cv_for = |lambda: f64| -> f64 {
            let result = smoothing_spline(&kp, lambda);
            let intervals: Vec<f64> = result[10..90]
                .windows(2)
                .map(|w| w[1].smooth_tick - w[0].smooth_tick)
                .collect();
            cv_of(&intervals)
        };

        let cv_small = cv_for(1.0);
        let cv_medium = cv_for(10.0);
        let cv_large = cv_for(100.0);

        eprintln!("λ=1:    CV = {:.4}%", cv_small * 100.0);
        eprintln!("λ=10:   CV = {:.4}%", cv_medium * 100.0);
        eprintln!("λ=100:  CV = {:.4}%", cv_large * 100.0);

        assert!(
            cv_large <= cv_medium + 1e-10,
            "larger lambda should give smaller CV: λ=100 CV={:.4}% > λ=10 CV={:.4}%",
            cv_large * 100.0,
            cv_medium * 100.0
        );
        assert!(
            cv_medium <= cv_small + 1e-10,
            "larger lambda should give smaller CV: λ=10 CV={:.4}% > λ=1 CV={:.4}%",
            cv_medium * 100.0,
            cv_small * 100.0
        );
    }

    #[test]
    fn whittaker_vs_pll_on_stable_plateau() {
        // Главный тест: на стабильном участке Whittaker даёт более равномерные
        // интервалы чем PLL (non-causal + global optimization).
        use crate::signal::pll::{PllParams, run_pll};

        let period = 320_000u64; // 50 Hz
        // Разгон 25→50 за 30 rev + 170 rev плато с ±30% jitter.
        let mut kp = vec![0u64];
        let mut t = 0u64;
        for i in 0..30 {
            let freq = 25.0 + 25.0 * (i as f64 / 30.0);
            let p = (SYS_HZ / freq) as u64;
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            t += (p as f64 * (1.0 + sign * 0.30)) as u64;
            kp.push(t);
        }
        for i in 0..170 {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            t += (period as f64 * (1.0 + sign * 0.30)) as u64;
            kp.push(t);
        }

        // PLL.
        let params = PllParams {
            smoothing: 20.0,
            damping: 0.707,
            systimer_hz: SYS_HZ,
        };
        let pll = run_pll(&kp, &params);

        // Whittaker.
        let spline = smoothing_spline(&kp, 100.0);

        // Сравниваем CV на плато (rev 80..180 — далеко от краёв и от разгона).
        // PLL: pll[i] соответствует kp[i+1] (pll пропускает kp[0]).
        // Spline: spline[i] соответствует kp[i].
        // Плато в kp: indices 31..200 (после 30 rev разгона).
        // Берём 80..180 в kp-индексации → pll[79..179], spline[80..180].
        let pll_intervals: Vec<f64> = pll[79..179]
            .windows(2)
            .map(|w| w[1].aligned_tick - w[0].aligned_tick)
            .collect();
        let spline_intervals: Vec<f64> = spline[80..180]
            .windows(2)
            .map(|w| w[1].smooth_tick - w[0].smooth_tick)
            .collect();

        let pll_cv = cv_of(&pll_intervals);
        let spline_cv = cv_of(&spline_intervals);

        eprintln!("PLL CV      = {:.4}% on plateau", pll_cv * 100.0);
        eprintln!("Whittaker CV = {:.4}% on plateau", spline_cv * 100.0);

        // Whittaker должен быть лучше (ниже CV) на плато.
        assert!(
            spline_cv < pll_cv,
            "spline CV ({:.4}%) >= PLL CV ({:.4}%)",
            spline_cv * 100.0,
            pll_cv * 100.0
        );

        // Оба CV должны быть разумными.
        assert!(pll_cv < 0.15, "PLL CV too high: {:.2}%", pll_cv * 100.0);
        assert!(
            spline_cv < 0.01,
            "spline CV too high: {:.2}%",
            spline_cv * 100.0
        );
    }

    #[test]
    fn whittaker_phase_interpolation_consistent() {
        let period = 320_000u64;
        let kp = uniform_kp(20, period, 0);
        let result = smoothing_spline(&kp, 10.0);

        // Середина между points[10] и points[11].
        let mid_tick = ((result[10].smooth_tick + result[11].smooth_tick) / 2.0) as u64;
        let (freq, phase) = spline_interpolate_phase(&result, mid_tick, SYS_HZ);

        assert!(
            (freq - 50.0).abs() < 2.0,
            "freq = {:.3}, expected ~50",
            freq
        );

        // points[10] = phase 2π·10, points[11] = phase 2π·11.
        // Середина → phase ≈ 2π·10.5.
        let two_pi = 2.0 * std::f64::consts::PI;
        let expected_mid = two_pi * 10.5;
        assert!(
            (phase - expected_mid).abs() < two_pi * 0.1,
            "phase = {:.3}, expected ~{:.3}",
            phase,
            expected_mid
        );
    }

    #[test]
    fn whittaker_no_drift_vs_raw() {
        // На постоянной скорости: aligned метки не должны дрейфовать от raw.
        let period = 320_000u64;
        let kp = jittery_kp(200, period, 0.30, 0);
        let result = smoothing_spline(&kp, 100.0);

        // Средний offset aligned vs raw на двух половинах плато.
        let first_half: f64 = (50..100)
            .map(|i| result[i].smooth_tick - kp[i] as f64)
            .sum::<f64>()
            / 50.0;
        let second_half: f64 = (150..200)
            .map(|i| result[i].smooth_tick - kp[i] as f64)
            .sum::<f64>()
            / 50.0;

        let drift = (second_half - first_half).abs();
        let drift_pct = drift / period as f64 * 100.0;

        eprintln!(
            "drift = {:.0} ticks ({:.2}% of period)",
            drift,
            drift_pct
        );
        assert!(
            drift_pct < 5.0,
            "drift = {:.0} ticks ({:.2}% of period)",
            drift,
            drift_pct
        );
    }

    /// Реалистичная симуляция записи виброметра: разгон → плато → выбег.
    ///
    /// Профиль как в реальных записях:
    /// - Разгон 0→3000 RPM (0→50 Hz) за ~100 оборотов
    /// - Плато 3000 RPM за ~300 оборотов
    /// - Выбег 3000→0 RPM за ~80 оборотов
    /// - Jitter ±35% (широкая метка ~35-40% duty)
    ///
    /// На плато проверяем:
    /// 1. CV интервалов Whittaker < CV PLL
    /// 2. Whittaker RPM bias < PLL RPM bias
    /// 3. Ни один из методов не дрейфует
    #[test]
    fn realistic_recording_profile() {
        use crate::signal::pll::{PllParams, run_pll};

        let jitter_frac = 0.35; // реалистичный jitter от широкой метки

        // LCG для pseudo-random jitter (не только alternating).
        let mut rng: u64 = 42;
        let mut next_jitter = || -> f64 {
            rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // Uniform в [-jitter_frac, +jitter_frac].
            let u = (rng >> 33) as f64 / (1u64 << 31) as f64; // [0, 1)
            (u * 2.0 - 1.0) * jitter_frac
        };

        let mut kp = vec![0u64];
        let mut t = 0u64;

        // Разгон: 5→50 Hz за 100 оборотов (линейный по частоте).
        for i in 0..100 {
            let frac = (i + 1) as f64 / 100.0;
            let freq = 5.0 + 45.0 * frac;
            let true_period = (SYS_HZ / freq) as u64;
            let jittered = (true_period as f64 * (1.0 + next_jitter())) as u64;
            t += jittered;
            kp.push(t);
        }

        // Плато: 50 Hz за 300 оборотов.
        let plateau_period = (SYS_HZ / 50.0) as u64;
        let plateau_start_idx = kp.len(); // kp index где начинается плато
        for _ in 0..300 {
            let jittered = (plateau_period as f64 * (1.0 + next_jitter())) as u64;
            t += jittered;
            kp.push(t);
        }
        let plateau_end_idx = kp.len() - 1;

        // Выбег: 50→5 Hz за 80 оборотов.
        for i in 0..80 {
            let frac = (i + 1) as f64 / 80.0;
            let freq = 50.0 - 45.0 * frac;
            let true_period = (SYS_HZ / freq.max(3.0)) as u64;
            let jittered = (true_period as f64 * (1.0 + next_jitter())) as u64;
            t += jittered;
            kp.push(t);
        }

        eprintln!(
            "total KP marks: {}, plateau: indices {}..{}",
            kp.len(),
            plateau_start_idx,
            plateau_end_idx
        );

        // PLL.
        let params = PllParams {
            smoothing: 20.0,
            damping: 0.707,
            systimer_hz: SYS_HZ,
        };
        let pll = run_pll(&kp, &params);

        // Whittaker с разными λ.
        let spline_10 = smoothing_spline(&kp, 10.0);
        let spline_100 = smoothing_spline(&kp, 100.0);
        let spline_1000 = smoothing_spline(&kp, 1000.0);

        // Анализируем середину плато (далеко от переходных процессов).
        // kp-индексы: plateau_start_idx + 50 .. plateau_end_idx - 50.
        let analyze_from = plateau_start_idx + 50;
        let analyze_to = plateau_end_idx - 50;

        // PLL: pll[i] ↔ kp[i+1], поэтому pll_from = analyze_from - 1.
        let pll_from = analyze_from - 1;
        let pll_to = analyze_to - 1;

        let pll_intervals: Vec<f64> = pll[pll_from..pll_to]
            .windows(2)
            .map(|w| w[1].aligned_tick - w[0].aligned_tick)
            .collect();

        let spline_intervals = |s: &[SplinePoint]| -> Vec<f64> {
            s[analyze_from..analyze_to]
                .windows(2)
                .map(|w| w[1].smooth_tick - w[0].smooth_tick)
                .collect()
        };

        let pll_cv = cv_of(&pll_intervals);
        let sp10_cv = cv_of(&spline_intervals(&spline_10));
        let sp100_cv = cv_of(&spline_intervals(&spline_100));
        let sp1000_cv = cv_of(&spline_intervals(&spline_1000));

        eprintln!("=== Plateau interval CV ===");
        eprintln!("PLL (causal):     {:.4}%", pll_cv * 100.0);
        eprintln!("Whittaker λ=10:   {:.4}%", sp10_cv * 100.0);
        eprintln!("Whittaker λ=100:  {:.4}%", sp100_cv * 100.0);
        eprintln!("Whittaker λ=1000: {:.4}%", sp1000_cv * 100.0);

        // Whittaker λ=100 должен быть значительно лучше PLL на плато.
        assert!(
            sp100_cv < pll_cv,
            "Whittaker λ=100 ({:.4}%) >= PLL ({:.4}%)",
            sp100_cv * 100.0,
            pll_cv * 100.0
        );

        // RPM accuracy на плато.
        let expected_rpm = 3000.0;

        let pll_rpms: Vec<f64> = pll[pll_from..pll_to]
            .iter()
            .map(|p| p.rpm(SYS_HZ))
            .collect();
        let pll_mean_rpm = pll_rpms.iter().sum::<f64>() / pll_rpms.len() as f64;
        let pll_rpm_bias = (pll_mean_rpm - expected_rpm).abs() / expected_rpm;

        let sp100_rpms: Vec<f64> = spline_100[analyze_from..analyze_to]
            .iter()
            .map(|p| p.rpm(SYS_HZ))
            .collect();
        let sp100_mean_rpm = sp100_rpms.iter().sum::<f64>() / sp100_rpms.len() as f64;
        let sp100_rpm_bias = (sp100_mean_rpm - expected_rpm).abs() / expected_rpm;

        eprintln!("\n=== Plateau RPM ===");
        eprintln!(
            "PLL:    mean={:.1} RPM, bias={:.3}%",
            pll_mean_rpm,
            pll_rpm_bias * 100.0
        );
        eprintln!(
            "Whittaker λ=100: mean={:.1} RPM, bias={:.3}%",
            sp100_mean_rpm,
            sp100_rpm_bias * 100.0
        );

        // Оба bias < 2%.
        // (С random jitter 35% есть небольшой bias из-за Jensen's inequality:
        // E[1/T] ≠ 1/E[T] для нелинейного преобразования период→частота.)
        assert!(
            pll_rpm_bias < 0.02,
            "PLL RPM bias = {:.3}%",
            pll_rpm_bias * 100.0
        );
        assert!(
            sp100_rpm_bias < 0.02,
            "Whittaker RPM bias = {:.3}%",
            sp100_rpm_bias * 100.0
        );

        // RPM std: Whittaker должен быть стабильнее.
        let rpm_std = |rpms: &[f64]| -> f64 {
            let mean = rpms.iter().sum::<f64>() / rpms.len() as f64;
            (rpms
                .iter()
                .map(|x| (x - mean).powi(2))
                .sum::<f64>()
                / rpms.len() as f64)
                .sqrt()
        };
        let pll_rpm_std = rpm_std(&pll_rpms);
        let sp100_rpm_std = rpm_std(&sp100_rpms);

        eprintln!("PLL RPM std:          {:.2}", pll_rpm_std);
        eprintln!("Whittaker λ=100 std:  {:.2}", sp100_rpm_std);

        assert!(
            sp100_rpm_std < pll_rpm_std,
            "Whittaker RPM std ({:.2}) >= PLL ({:.2})",
            sp100_rpm_std,
            pll_rpm_std
        );
    }

    /// Тест: при слишком большом λ Whittaker не "перегибает" разгон.
    ///
    /// На разгоне вторая производная f''(x) != 0 (ускорение).
    /// Слишком сильное сглаживание сплющит разгон → RPM bias.
    /// Проверяем что λ=100 не портит оценку скорости на разгоне.
    #[test]
    fn whittaker_ramp_accuracy() {
        let mut kp = vec![0u64];
        let mut t = 0u64;

        // Разгон 10→50 Hz за 80 оборотов, без jitter.
        for i in 0..80 {
            let frac = (i + 1) as f64 / 80.0;
            let freq = 10.0 + 40.0 * frac;
            let true_period = (SYS_HZ / freq) as u64;
            t += true_period;
            kp.push(t);
        }

        let result = smoothing_spline(&kp, 100.0);

        // Проверяем что RPM на середине разгона (i=40) ≈ ожидаемому.
        // freq(40) = 10 + 40 * 40/80 = 30 Hz → 1800 RPM.
        let mid_rpm = result[40].rpm(SYS_HZ);
        let expected_rpm = 30.0 * 60.0; // 1800

        eprintln!("ramp mid RPM: {:.1}, expected: {:.1}", mid_rpm, expected_rpm);

        // Допуск: smoothing сплющивает, но не больше 10%.
        let err = (mid_rpm - expected_rpm).abs() / expected_rpm;
        assert!(
            err < 0.10,
            "ramp mid RPM error = {:.1}% ({:.0} vs {:.0})",
            err * 100.0,
            mid_rpm,
            expected_rpm
        );

        // На конце: RPM ≈ 3000.
        let end_rpm = result[80].rpm(SYS_HZ);
        let end_err = (end_rpm - 3000.0).abs() / 3000.0;
        assert!(
            end_err < 0.05,
            "ramp end RPM error = {:.1}% ({:.0})",
            end_err * 100.0,
            end_rpm
        );
    }
}
