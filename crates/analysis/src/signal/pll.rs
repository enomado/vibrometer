/// Type-2 digital PLL по keyphasor-тикам.
///
/// Keyphasor даёт один импульс за оборот, но из-за широкой метки
/// момент фронта jitter'ит (±30%). PLL сглаживает мгновенную частоту
/// и оценивает "истинные" моменты оборотов.
///
/// Time-domain формулировка (без перевода в Hz/рад):
///   - Состояние: period (тики на оборот)
///   - На каждом KP[n]:
///       predicted_tick = KP[n-1] + period
///       time_error = KP[n] - predicted_tick        (тики)
///       period += Ki · time_error                   (integral: корректирует скорость)
///       aligned_tick = predicted_tick + Kp · time_error  (proportional: корректирует фазу)
///
///   - time_error линеен в dt → нет bias при alternating jitter.
///     (В формулировке через Hz: freq += Ki·error·freq/2π — нелинейно, bias ≠ 0.)
///
///   - aligned_tick — компромисс между prediction и measurement:
///       Kp=0 → aligned_tick = predicted (чистое предсказание, max фильтрация)
///       Kp=1 → aligned_tick = KP[n] (сырой фронт, нулевая фильтрация)
///
///   Gain mapping (Gardner 1980):
///     ωn  = 4 / (ζ · N), Kp = 2·ζ·ωn, Ki = ωn²
///     N = settling (обороты), ζ = damping (0.707 = Butterworth)
///
///   Стабильность: Ki < Kp ↔ N > 2/ζ².
const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

/// Параметры PLL.
#[derive(Clone, Debug)]
pub struct PllParams {
    /// Число оборотов на установку (settling time).
    pub smoothing: f64,
    /// Коэффициент демпфирования. 0.707 = Butterworth, 1.0 = критическое.
    pub damping: f64,
    /// Частота SystemTimer (Hz). 16 МГц для ESP32-C3.
    pub systimer_hz: f64,
}

impl PllParams {
    pub fn default_16mhz() -> Self {
        Self {
            smoothing: 20.0,
            damping: 0.707,
            systimer_hz: 16_000_000.0,
        }
    }
}

/// Kp и Ki из (smoothing N, damping ζ).
/// ωn = 4/(ζ·N), Kp = 2·ζ·ωn, Ki = ωn².
fn pll_gains(params: &PllParams) -> (f64, f64) {
    let zeta = params.damping.clamp(0.1, 2.0);
    let n = params.smoothing.max(2.0 / (zeta * zeta) + 0.1);
    let wn = 4.0 / (zeta * n);
    let kp = 2.0 * zeta * wn;
    let ki = wn * wn;
    debug_assert!(ki < kp, "PLL unstable: ki={ki} >= kp={kp}");
    (kp, ki)
}

/// Результат PLL для одного keyphasor-события.
#[derive(Clone, Debug)]
pub struct PllPoint {
    /// Сырой tick этого KP-события (из hardware timer).
    pub tick: u64,
    /// Оценённый "истинный" tick оборота (aligned).
    /// Компромисс между предсказанием PLL и сырым фронтом.
    pub aligned_tick: f64,
    /// Оценённый период (тики на оборот).
    /// При alternating jitter period oscillates.
    /// Для RPM используй smooth_period (среднее двух последних).
    pub period: f64,
    /// Сглаженный период — среднее текущего и предыдущего.
    /// Убирает Ki-oscillation при alternating jitter.
    pub smooth_period: f64,
}

impl PllPoint {
    /// Сглаженная частота вращения (Hz).
    pub fn freq(&self, systimer_hz: f64) -> f64 {
        systimer_hz / self.smooth_period
    }
    /// Сглаженный RPM.
    pub fn rpm(&self, systimer_hz: f64) -> f64 {
        self.freq(systimer_hz) * 60.0
    }
}

/// Прогнать PLL по keyphasor-тикам.
///
/// Time-domain Type-2 PLL. Все вычисления в тиках — нет перевода в Hz/рад,
/// нет нелинейных членов, нет bias.
///
/// На каждом KP[n]:
///   1. predicted = prev_aligned + period
///   2. time_error = KP[n] - predicted              (тики)
///   3. period += Ki · time_error                    (integral correction)
///   4. aligned = predicted + Kp · time_error        (proportional correction)
///
/// # Паникует
/// Если `kp_ticks.len() < 2`.
pub fn run_pll(kp_ticks: &[u64], params: &PllParams) -> Vec<PllPoint> {
    assert!(kp_ticks.len() >= 2, "PLL needs at least 2 keyphasor events");

    let (kp, ki) = pll_gains(params);
    const PERIOD_OBSERVER_BLEND: f64 = 0.12;

    // Инициализация period: среднее первых 2 интервалов если доступны.
    let mut period = if kp_ticks.len() >= 3 {
        (kp_ticks[2] - kp_ticks[0]) as f64 / 2.0
    } else {
        (kp_ticks[1] - kp_ticks[0]) as f64
    };

    // Первая точка: KP[1]. predicted = KP[0] + period.
    let predicted = kp_ticks[0] as f64 + period;
    let time_error = kp_ticks[1] as f64 - predicted;
    let prev_period = period;
    period += ki * time_error;
    let aligned = predicted + kp * time_error;
    if period < 1.0 { period = 1.0; }
    let smooth_period = (prev_period + period) / 2.0;

    let mut out = Vec::with_capacity(kp_ticks.len() - 1);
    out.push(PllPoint { tick: kp_ticks[1], aligned_tick: aligned, period, smooth_period });

    for i in 2..kp_ticks.len() {
        let raw_period = (kp_ticks[i] - kp_ticks[i - 1]) as f64;
        let prev_aligned = out.last().unwrap().aligned_tick;
        let predicted = prev_aligned + period;
        let time_error = kp_ticks[i] as f64 - predicted;

        let prev_period = period;
        // Обычный PI-контур корректирует phase error, а слабый observer
        // плавно подтягивает оценку периода к измеренному raw-интервалу.
        // Так пачка импульсов постепенно "утащит" PLL к новой скорости,
        // но без жёстких перескоков aligned-фазы.
        period += ki * time_error;
        period += PERIOD_OBSERVER_BLEND * (raw_period - period);
        let aligned = predicted + kp * time_error;

        if period < 1.0 { period = 1.0; }

        // smooth_period: среднее pre/post Ki-correction.
        // При alternating jitter period oscillates ±Ki·error.
        // Среднее двух соседних = точное значение (bias = 0).
        let smooth_period = (prev_period + period) / 2.0;

        out.push(PllPoint { tick: kp_ticks[i], aligned_tick: aligned, period, smooth_period });
    }

    out
}

/// Вычислить выровненные keyphasor-тики из PLL.
///
/// Возвращает aligned_tick каждого PLL point в секундах от start_tick.
pub fn aligned_kp_times(pll: &[PllPoint], start_tick: u64, systimer_hz: f64) -> Vec<f64> {
    pll.iter()
        .map(|p| (p.aligned_tick - start_tick as f64) / systimer_hz)
        .collect()
}

/// Интерполировать freq и phase для произвольного tick.
/// Возвращает (freq_hz, phase_rad).
pub fn interpolate_phase(pll: &[PllPoint], tick: u64, systimer_hz: f64) -> (f64, f64) {
    assert!(!pll.is_empty(), "PLL result is empty");

    let tick_f = tick as f64;

    // До первой точки — экстраполяция назад.
    if tick_f <= pll[0].aligned_tick {
        let freq = systimer_hz / pll[0].smooth_period;
        let dt = (pll[0].aligned_tick - tick_f) / systimer_hz;
        let phase = TWO_PI - TWO_PI * freq * dt;
        return (freq, phase);
    }

    // После последней — экстраполяция вперёд.
    let last = pll.last().unwrap();
    if tick_f >= last.aligned_tick {
        let freq = systimer_hz / last.smooth_period;
        let dt = (tick_f - last.aligned_tick) / systimer_hz;
        let phase = TWO_PI * pll.len() as f64 + TWO_PI * freq * dt;
        return (freq, phase);
    }

    // Бинарный поиск интервала.
    let j = pll.partition_point(|p| p.aligned_tick <= tick_f).saturating_sub(1);
    let a = &pll[j];
    let b = &pll[j + 1];
    let dt_ab = b.aligned_tick - a.aligned_tick;
    if dt_ab.abs() < 1e-15 {
        let freq = systimer_hz / a.smooth_period;
        return (freq, TWO_PI * (j + 1) as f64);
    }
    let frac = (tick_f - a.aligned_tick) / dt_ab;
    let sp = a.smooth_period + (b.smooth_period - a.smooth_period) * frac;
    let freq = systimer_hz / sp;
    // Фаза: a = (j+1)·2π, b = (j+2)·2π.
    let phase = TWO_PI * ((j + 1) as f64 + frac);
    (freq, phase)
}

/// Вычислить фазу PLL для каждого сэмпла.
///
/// Для resampling time→angle domain (order tracking).
pub fn sample_phases(
    pll: &[PllPoint],
    sample_ticks: &[u64],
    systimer_hz: f64,
) -> Vec<f64> {
    sample_ticks
        .iter()
        .map(|&tick| interpolate_phase(pll, tick, systimer_hz).1)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    const SYS_HZ: f64 = 16_000_000.0;

    fn uniform_kp(n_revs: usize, period_ticks: u64, start: u64) -> Vec<u64> {
        (0..=n_revs).map(|i| start + i as u64 * period_ticks).collect()
    }

    fn jittery_kp(n_revs: usize, period_ticks: u64, jitter_frac: f64, start: u64) -> Vec<u64> {
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

    #[test]
    fn stable_rpm_converges() {
        let period = 320_000u64; // 50 Hz
        let kp = uniform_kp(100, period, 0);
        let params = PllParams::default_16mhz();
        let pll = run_pll(&kp, &params);

        for p in &pll[5..] {
            let rpm = p.rpm(SYS_HZ);
            assert!(
                (rpm - 3000.0).abs() < 30.0,
                "rpm = {:.1}, expected ~3000",
                rpm
            );
        }
    }

    #[test]
    fn jitter_smoothed() {
        // ±30% alternating jitter.
        let period = 320_000u64;
        let kp = jittery_kp(200, period, 0.30, 0);
        let params = PllParams {
            smoothing: 20.0,
            damping: 0.707,
            systimer_hz: SYS_HZ,
        };
        let pll = run_pll(&kp, &params);

        // PLL RPM не должен скакать больше ±15%.
        for p in &pll[60..] {
            let err = (p.rpm(SYS_HZ) - 3000.0).abs() / 3000.0;
            assert!(
                err < 0.15,
                "PLL RPM = {:.1}, error = {:.1}%",
                p.rpm(SYS_HZ),
                err * 100.0
            );
        }
    }

    #[test]
    fn phase_monotonic() {
        let period = 320_000u64;
        let kp = jittery_kp(50, period, 0.25, 1_000_000);
        let params = PllParams::default_16mhz();
        let pll = run_pll(&kp, &params);

        // aligned_tick должен монотонно расти.
        for w in pll.windows(2) {
            assert!(
                w[1].aligned_tick > w[0].aligned_tick,
                "aligned_tick not monotonic: {} -> {}",
                w[0].aligned_tick,
                w[1].aligned_tick
            );
        }
    }

    #[test]
    fn ramp_up_tracks() {
        // Разгон 25→50 Hz за 60 оборотов + 40 оборотов плато.
        let mut kp = vec![0u64];
        let mut t = 0u64;
        for i in 0..60 {
            let frac = i as f64 / 60.0;
            let freq = 25.0 + 25.0 * frac;
            let period = (SYS_HZ / freq) as u64;
            t += period;
            kp.push(t);
        }
        let plateau_period = (SYS_HZ / 50.0) as u64;
        for _ in 0..40 {
            t += plateau_period;
            kp.push(t);
        }

        let params = PllParams::default_16mhz();
        let pll = run_pll(&kp, &params);

        let last = pll.last().unwrap();
        let last_freq = last.freq(SYS_HZ);
        assert!(
            (last_freq - 50.0).abs() < 1.0,
            "final freq = {:.1}, expected ~50.0",
            last_freq
        );
    }

    #[test]
    fn aligned_kp_differ_from_raw_on_jitter() {
        // При jitter: aligned метки должны быть РАВНОМЕРНЕЕ сырых.
        let period = 320_000u64;
        let start = 0u64;
        let kp = jittery_kp(100, period, 0.25, start);
        let params = PllParams {
            smoothing: 10.0,
            damping: 0.707,
            systimer_hz: SYS_HZ,
        };
        let pll = run_pll(&kp, &params);

        let aligned = aligned_kp_times(&pll, start, SYS_HZ);

        // Aligned интервалы после settling.
        let intervals: Vec<f64> = aligned.windows(2)
            .skip(20)
            .map(|w| w[1] - w[0])
            .collect();

        let mean = intervals.iter().sum::<f64>() / intervals.len() as f64;
        let std = (intervals.iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f64>() / intervals.len() as f64)
            .sqrt();
        let cv = std / mean;

        // Aligned CV < Kp * jitter_frac ≈ 0.4 * 25% = 10%.
        // С smoothing=10, Kp=0.8, CV до 20%. Ослабляем до 20%.
        // Главное: CV aligned < CV raw (25%).
        assert!(
            cv < 0.20,
            "aligned CV = {:.1}%, expected < 20% (raw CV = 25%)",
            cv * 100.0
        );
    }

    /// Главный тест: aligned метки совпадают по фазе с raw KP.
    ///
    /// На стабильном участке (const RPM + jitter) средняя ошибка
    /// между aligned и raw KP не должна дрейфовать.
    /// Т.е. aligned[i] ≈ raw_kp[i] (с точностью до jitter filtering).
    #[test]
    fn aligned_in_phase_with_raw_kp() {
        let period = 320_000u64; // 50 Hz
        let kp = jittery_kp(200, period, 0.30, 0);
        let params = PllParams {
            smoothing: 20.0,
            damping: 0.707,
            systimer_hz: SYS_HZ,
        };
        let pll = run_pll(&kp, &params);

        // Aligned в тиках.
        let aligned_ticks: Vec<f64> = pll.iter().map(|p| p.aligned_tick).collect();
        // Raw KP (без первого — pll[0] соответствует kp[1]).
        let raw_ticks: Vec<f64> = kp[1..].iter().map(|&t| t as f64).collect();

        assert_eq!(aligned_ticks.len(), raw_ticks.len());

        // После settling (>40 rev): drift между aligned и raw < 5% от period.
        // "Drift" = систематическое смещение aligned от raw.
        // Если PLL в lock, aligned[i] ≈ raw[i] ± jitter_correction.
        // Среднее (aligned - raw) за 10 оборотов должно быть < 5% period.
        let check_range = 100..190;
        let diffs: Vec<f64> = (check_range.clone())
            .map(|i| aligned_ticks[i] - raw_ticks[i])
            .collect();

        // Среднее diff не должно расти (т.е. нет drift).
        let first_half: f64 = diffs[..diffs.len()/2].iter().sum::<f64>()
            / (diffs.len()/2) as f64;
        let second_half: f64 = diffs[diffs.len()/2..].iter().sum::<f64>()
            / (diffs.len() - diffs.len()/2) as f64;
        let drift = (second_half - first_half).abs();

        assert!(
            drift < 0.05 * period as f64,
            "drift = {:.0} ticks ({:.1}% of period), expected < 5%",
            drift,
            drift / period as f64 * 100.0
        );

        // Абсолютный offset тоже не должен быть огромным (< 50% period).
        let mean_offset = diffs.iter().sum::<f64>() / diffs.len() as f64;
        assert!(
            mean_offset.abs() < 0.5 * period as f64,
            "mean offset = {:.0} ticks ({:.1}% of period)",
            mean_offset,
            mean_offset / period as f64 * 100.0
        );
    }

    #[test]
    fn phase_locks_on_stable_plateau() {
        // Разгон + 80 оборотов плато с ±30% jitter + выбег.
        let plateau_freq = 50.0;
        let plateau_period = (SYS_HZ / plateau_freq) as u64;

        let mut kp = vec![0u64];
        let mut t = 0u64;

        // Разгон: 25→50 Hz за 10 оборотов.
        for i in 0..10 {
            let freq = 25.0 + 25.0 * (i as f64 / 10.0);
            t += (SYS_HZ / freq) as u64;
            kp.push(t);
        }
        // Плато: 80 оборотов с ±30% jitter.
        for i in 0..80 {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            let jittered = (plateau_period as f64 * (1.0 + sign * 0.30)) as u64;
            t += jittered;
            kp.push(t);
        }
        // Выбег: 50→30 Hz за 10 оборотов.
        for i in 0..10 {
            let freq = 50.0 - 20.0 * (i as f64 / 10.0);
            t += (SYS_HZ / freq) as u64;
            kp.push(t);
        }

        let params = PllParams {
            smoothing: 20.0,
            damping: 0.707,
            systimer_hz: SYS_HZ,
        };
        let pll = run_pll(&kp, &params);

        // На конце плато (pll[69..87]) RPM должен быть ≈ 3000.
        let plateau = &pll[69..87];
        let expected_rpm = plateau_freq * 60.0;

        // Mean RPM bias < 1.5%.
        let mean_rpm: f64 = plateau.iter()
            .map(|p| p.rpm(SYS_HZ))
            .sum::<f64>() / plateau.len() as f64;
        let bias = (mean_rpm - expected_rpm).abs() / expected_rpm;
        assert!(
            bias < 0.015,
            "plateau mean RPM = {:.1}, bias = {:.2}%",
            mean_rpm,
            bias * 100.0
        );

        // Aligned метки на плато равномерны.
        let aligned: Vec<f64> = plateau.iter()
            .map(|p| p.aligned_tick / SYS_HZ)
            .collect();
        let intervals: Vec<f64> = aligned.windows(2)
            .map(|w| w[1] - w[0])
            .collect();
        if intervals.len() >= 3 {
            let mean = intervals.iter().sum::<f64>() / intervals.len() as f64;
            let std = (intervals.iter()
                .map(|x| (x - mean).powi(2))
                .sum::<f64>() / intervals.len() as f64)
                .sqrt();
            let cv = std / mean;
            // CV < Kp * jitter ≈ 0.4 * 30% = 12%.
            assert!(
                cv < 0.12,
                "plateau aligned CV = {:.2}%",
                cv * 100.0
            );
        }
    }

    #[test]
    fn abrupt_pulse_burst_blends_smoothly() {
        // Несколько редких импульсов, затем плотная пачка.
        // Реальный кейс: PLL не должен дёргаться, но должен плавно
        // утащиться к новому периоду по мере прихода пачки.
        let slow_period = (SYS_HZ / 10.0) as u64;
        let fast_period = (SYS_HZ / 50.0) as u64;

        let mut kp = vec![0u64];
        let mut t = 0u64;
        for _ in 0..6 {
            t += slow_period;
            kp.push(t);
        }
        for _ in 0..12 {
            t += fast_period;
            kp.push(t);
        }

        let params = PllParams::default_16mhz();
        let pll = run_pll(&kp, &params);

        // На пачке частота должна расти плавно, без возвратов назад.
        let burst_freqs: Vec<f64> = pll[6..]
            .iter()
            .map(|p| p.freq(SYS_HZ))
            .collect();
        for w in burst_freqs.windows(2) {
            assert!(
                w[1] >= w[0] - 1e-6,
                "freq regressed during burst: {:.2} Hz -> {:.2} Hz",
                w[0],
                w[1]
            );
        }

        // К концу пачки PLL должен заметно приблизиться к новому режиму.
        let last_freq = *burst_freqs.last().unwrap();
        assert!(
            last_freq > 40.0,
            "final burst freq = {:.2} Hz, expected smooth convergence toward 50 Hz",
            last_freq
        );
    }

    #[test]
    fn interpolation_between_points() {
        let period = 320_000u64;
        let kp = uniform_kp(10, period, 0);
        let params = PllParams::default_16mhz();
        let pll = run_pll(&kp, &params);

        let mid_tick = ((pll[4].aligned_tick + pll[5].aligned_tick) / 2.0) as u64;
        let (freq, phase) = interpolate_phase(&pll, mid_tick, SYS_HZ);

        assert!(
            (freq - 50.0).abs() < 1.0,
            "freq = {:.3}, expected ~50",
            freq
        );
        let phase_4 = TWO_PI * 5.0;
        let phase_5 = TWO_PI * 6.0;
        assert!(
            phase > phase_4 && phase < phase_5,
            "phase = {:.3}, expected between {:.3} and {:.3}",
            phase,
            phase_4,
            phase_5
        );
    }

    /// Тест: aligned marks 1:1 с raw KP, и на стабильном участке
    /// aligned ≈ raw (каждая aligned метка рядом с соответствующей raw).
    #[test]
    fn aligned_count_equals_raw_and_positions_match() {
        // Разгон 5→50 Hz (как реальная запись), 50 rev + 50 rev плато с ±30% jitter.
        let mut kp = vec![0u64];
        let mut t = 0u64;
        // Разгон.
        for i in 0..50 {
            let freq = 5.0 + 45.0 * (i as f64 / 50.0);
            let true_period = (SYS_HZ / freq) as u64;
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            t += (true_period as f64 * (1.0 + sign * 0.30)) as u64;
            kp.push(t);
        }
        // Плато 50 Hz.
        let plateau_period = (SYS_HZ / 50.0) as u64;
        for i in 0..50 {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            t += (plateau_period as f64 * (1.0 + sign * 0.30)) as u64;
            kp.push(t);
        }

        let start_tick = kp[0];
        let params = PllParams::default_16mhz();
        let pll = run_pll(&kp, &params);
        let aligned = aligned_kp_times(&pll, start_tick, SYS_HZ);

        // 1. Количество: aligned = kp.len() - 1 (pll пропускает kp[0]).
        assert_eq!(
            aligned.len(), kp.len() - 1,
            "aligned count {} != kp count - 1 = {}",
            aligned.len(), kp.len() - 1
        );

        // 2. На плато (последние 20 rev): aligned[i] ≈ raw_kp[i+1] ± 20% period.
        let plateau_start = kp.len() - 1 - 20; // pll index
        for i in plateau_start..pll.len() {
            let raw_sec = (kp[i + 1] - start_tick) as f64 / SYS_HZ;
            let ali_sec = aligned[i];
            let period_sec = plateau_period as f64 / SYS_HZ;
            let diff = (ali_sec - raw_sec).abs();
            assert!(
                diff < 0.20 * period_sec,
                "plateau[{}]: aligned={:.6}s, raw={:.6}s, diff={:.6}s ({:.1}% of period)",
                i, ali_sec, raw_sec, diff, diff / period_sec * 100.0
            );
        }

        // 3. Print для визуальной проверки (cargo test -- --nocapture).
        eprintln!("\n=== aligned vs raw (first 10 + last 10) ===");
        for i in 0..10.min(pll.len()) {
            let raw_sec = (kp[i + 1] - start_tick) as f64 / SYS_HZ;
            let ali_sec = aligned[i];
            eprintln!(
                "  [{:3}] raw={:.6}s  aligned={:.6}s  diff={:+.6}s",
                i, raw_sec, ali_sec, ali_sec - raw_sec
            );
        }
        if pll.len() > 20 {
            eprintln!("  ...");
            for i in (pll.len() - 10)..pll.len() {
                let raw_sec = (kp[i + 1] - start_tick) as f64 / SYS_HZ;
                let ali_sec = aligned[i];
                eprintln!(
                    "  [{:3}] raw={:.6}s  aligned={:.6}s  diff={:+.6}s",
                    i, raw_sec, ali_sec, ali_sec - raw_sec
                );
            }
        }
    }
}
