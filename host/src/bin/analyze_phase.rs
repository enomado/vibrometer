/// CLI для анализа фазы и амплитуды 1x из parquet-записей виброметра.
///
/// Алгоритм:
/// 1. Читаем kp_ticks из metadata файла (или детектируем фронты по flags).
/// 2. Запускаем PLL → список PllPoint (aligned_tick, период).
/// 3. Ищем непрерывный участок с постоянной скоростью (CV периода < threshold).
/// 4. IQ-демодуляция 1x по каждому обороту участка.
/// 5. Усреднение амплитуды + circular mean фазы.
use std::f64::consts::PI;
use std::path::{Path, PathBuf};

use arrow::array::{Int32Array, RecordBatchReader as _, UInt64Array, UInt8Array};
use clap::Parser;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use vibro_analysis::signal::pll::{PllParams, PllPoint, interpolate_phase, run_pll};

const SYSTIMER_HZ: f64 = 16_000_000.0;

// --------------------------------------------------------------------------- //
// CLI

#[derive(Parser)]
#[command(
    name = "analyze-phase",
    about = "Анализ фазы и амплитуды 1x из parquet-записей виброметра"
)]
struct Cli {
    /// Номера записей (например: 91 92 93 94). Без аргументов — все файлы.
    nums: Vec<u32>,

    /// Директория с записями
    #[arg(long, default_value = "recordings")]
    dir: PathBuf,

    /// Порог CV периода для поиска стационарного участка (доля, напр. 0.01 = 1%)
    #[arg(long, default_value_t = 0.01)]
    cv: f64,

    /// Минимальное число оборотов в стационарном участке
    #[arg(long, default_value_t = 10)]
    min_revs: usize,
}

// --------------------------------------------------------------------------- //
// Типы

struct Sample {
    ch0:   f64,
    tick:  u64,
    /// Флаги keyphasor (bit1 = KEYPHASOR_LEVEL_FLAG)
    flags: u8,
}

struct RecordingData {
    samples:  Vec<Sample>,
    /// kp_ticks из metadata (пустой → определяем по rising edge в flags)
    kp_ticks: Vec<u64>,
}

// --------------------------------------------------------------------------- //
// Загрузка parquet

fn load_parquet(path: &Path) -> RecordingData {
    let file = std::fs::File::open(path).unwrap();
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).unwrap();

    let kv = builder.metadata().file_metadata().key_value_metadata();
    let meta_str = |key: &str| -> Option<String> {
        kv?.iter()
            .find(|kv| kv.key == key)
            .and_then(|kv| kv.value.clone())
    };

    let pga: f64 = meta_str("pga")
        .and_then(|v| v.parse::<u8>().ok())
        .unwrap_or(1) as f64;

    // kp_ticks хранятся как JSON-массив "[123,456,...]"
    let kp_ticks: Vec<u64> = meta_str("kp_ticks")
        .map(|s| {
            s.trim_matches(|c| c == '[' || c == ']')
                .split(',')
                .filter(|t| !t.is_empty())
                .map(|t| t.trim().parse::<u64>().unwrap())
                .collect()
        })
        .unwrap_or_default();

    let reader = builder.build().unwrap();
    let has_tick = reader.schema().column_with_name("tick").is_some();

    let mut samples = Vec::new();
    for batch in reader {
        let batch = batch.unwrap();
        let ch0_arr   = batch.column(0).as_any().downcast_ref::<Int32Array>().unwrap();
        let flags_arr = batch.column(2).as_any().downcast_ref::<UInt8Array>().unwrap();

        if has_tick {
            let tick_arr = batch.column(3).as_any().downcast_ref::<UInt64Array>().unwrap();
            for i in 0..batch.num_rows() {
                samples.push(Sample {
                    ch0:   ch0_arr.value(i) as f64 / pga,
                    tick:  tick_arr.value(i),
                    flags: flags_arr.value(i),
                });
            }
        } else {
            // Старый формат без tick: номер сэмпла × ticks-per-sample
            let tps = (SYSTIMER_HZ / 7500.0) as u64; // fallback
            for i in 0..batch.num_rows() {
                samples.push(Sample {
                    ch0:   ch0_arr.value(i) as f64 / pga,
                    tick:  samples.len() as u64 * tps,
                    flags: flags_arr.value(i),
                });
            }
        }
    }

    RecordingData { samples, kp_ticks }
}

// --------------------------------------------------------------------------- //
// Keyphasor: rising edges из flags

/// Rising edge (0→2) по флагам keyphasor (bit1 = KEYPHASOR_LEVEL_FLAG = 0x02).
/// Используется когда kp_ticks в metadata отсутствуют.
fn detect_kp_ticks_from_flags(samples: &[Sample]) -> Vec<u64> {
    let mut ticks = Vec::new();
    let mut prev_level = false;
    for s in samples {
        let level = s.flags & 0x02 != 0;
        if level && !prev_level {
            ticks.push(s.tick);
        }
        prev_level = level;
    }
    ticks
}

// --------------------------------------------------------------------------- //
// Поиск стационарного участка

/// Возвращает (start_idx, end_idx) в массиве pll — диапазон оборотов с CV < threshold.
/// Ищет наибольший непрерывный участок.
fn find_steady_segment(
    pll: &[PllPoint],
    cv_threshold: f64,
    min_revs: usize,
) -> Option<(usize, usize)> {
    // periods[i] = pll[i+1].smooth_period
    let periods: Vec<f64> = pll.windows(2).map(|w| w[0].smooth_period).collect();
    let n = periods.len();

    let mut best: Option<(usize, usize)> = None;
    let mut best_len = 0;

    let mut i = 0;
    while i < n {
        let mut j = i + 1;
        while j <= n {
            let seg = &periods[i..j];
            let mean = seg.iter().copied().sum::<f64>() / seg.len() as f64;
            let std = (seg.iter().map(|&p| (p - mean).powi(2)).sum::<f64>() / seg.len() as f64)
                .sqrt();
            if std / mean < cv_threshold {
                j += 1;
            } else {
                break;
            }
        }
        let seg_len = j - 1 - i;
        if seg_len >= min_revs && seg_len > best_len {
            best_len = seg_len;
            // pll индексы: обороты i..i+seg_len, границы pll[i]..pll[i+seg_len]
            best = Some((i, i + seg_len));
        }
        i += 1;
    }
    best
}

// --------------------------------------------------------------------------- //
// IQ-демодуляция 1x

/// IQ-демодуляция 1x для одного оборота [pll[k]..pll[k+1]).
/// Возвращает (amplitude, phase_rad).
///
/// phase=0 → пик вибрации совпадает с keyphasor mark.
/// amplitude × 2 т.к. ‹cos²θ› = 0.5
fn iq_1x_rev(
    samples: &[Sample],
    pll: &[PllPoint],
    rev_k: usize,
) -> (f64, f64) {
    let tick_from = pll[rev_k].aligned_tick;
    let tick_to   = pll[rev_k + 1].aligned_tick;

    let mut i_sum = 0.0f64;
    let mut q_sum = 0.0f64;
    let mut n = 0usize;

    for s in samples {
        let t = s.tick as f64;
        if t < tick_from || t >= tick_to {
            continue;
        }
        let theta = interpolate_phase(pll, s.tick, SYSTIMER_HZ).1;
        let (sin_t, cos_t) = theta.sin_cos();
        i_sum += s.ch0 * cos_t;
        q_sum += s.ch0 * sin_t;
        n += 1;
    }

    if n == 0 {
        return (0.0, 0.0);
    }
    let c = n as f64;
    let amp = 2.0 * (i_sum / c).hypot(q_sum / c);
    let phase = (q_sum / c).atan2(i_sum / c);
    (amp, phase)
}

// --------------------------------------------------------------------------- //
// Circular mean фазы

fn circular_mean_deg(phases_rad: &[f64]) -> f64 {
    let sin_m: f64 = phases_rad.iter().map(|&p| p.sin()).sum::<f64>() / phases_rad.len() as f64;
    let cos_m: f64 = phases_rad.iter().map(|&p| p.cos()).sum::<f64>() / phases_rad.len() as f64;
    sin_m.atan2(cos_m).to_degrees().rem_euclid(360.0)
}

fn circular_std_deg(phases_rad: &[f64], mean_rad: f64) -> f64 {
    let variance: f64 = phases_rad
        .iter()
        .map(|&p| {
            let d = p - mean_rad;
            // Нормализация в [-π, π]
            let d = d - (d / (2.0 * PI)).round() * (2.0 * PI);
            d * d
        })
        .sum::<f64>()
        / phases_rad.len() as f64;
    variance.sqrt().to_degrees()
}

// --------------------------------------------------------------------------- //
// Анализ одной записи

struct AnalysisResult {
    num:       u32,
    rpm:       f64,
    revs:      usize,
    amp:       f64,
    phase_deg: f64,
    phase_std: f64,
    t_start_s: f64,
    t_end_s:   f64,
}

fn analyze(path: &Path, num: u32, cv: f64, min_revs: usize) -> Result<AnalysisResult, String> {
    let rec = load_parquet(path);
    if rec.samples.is_empty() {
        return Err("пустой файл".into());
    }

    // Keyphasor ticks: из metadata или детектируем из flags
    let kp_ticks = if !rec.kp_ticks.is_empty() {
        rec.kp_ticks.clone()
    } else {
        detect_kp_ticks_from_flags(&rec.samples)
    };

    if kp_ticks.len() < 4 {
        return Err(format!("слишком мало KP-событий: {}", kp_ticks.len()));
    }

    // PLL
    let pll = run_pll(&kp_ticks, &PllParams::default_16mhz());
    if pll.len() < min_revs + 1 {
        return Err(format!("мало оборотов: {}", pll.len()));
    }

    // Стационарный участок
    let (seg_start, seg_end) = find_steady_segment(&pll, cv, min_revs)
        // Смягчаем: CV×2, min_revs/2
        .or_else(|| find_steady_segment(&pll, cv * 2.0, min_revs / 2))
        .ok_or("стационарный участок не найден")?;

    let revs = seg_end - seg_start;
    let mean_period: f64 = pll[seg_start..seg_end]
        .iter()
        .map(|p| p.smooth_period)
        .sum::<f64>()
        / revs as f64;
    let rpm = 60.0 * SYSTIMER_HZ / mean_period;

    // IQ по каждому обороту участка
    let start_tick = rec.samples[0].tick;
    let mut amps   = Vec::with_capacity(revs);
    let mut phases = Vec::with_capacity(revs);
    for k in seg_start..seg_end {
        let (amp, phase) = iq_1x_rev(&rec.samples, &pll, k);
        amps.push(amp);
        phases.push(phase);
    }

    let mean_amp  = amps.iter().copied().sum::<f64>() / amps.len() as f64;
    let phase_deg = circular_mean_deg(&phases);
    let mean_rad  = phase_deg.to_radians();
    let phase_std = circular_std_deg(&phases, mean_rad);

    let t_start_s = (pll[seg_start].aligned_tick - start_tick as f64) / SYSTIMER_HZ;
    let t_end_s   = (pll[seg_end].aligned_tick   - start_tick as f64) / SYSTIMER_HZ;

    Ok(AnalysisResult { num, rpm, revs, amp: mean_amp, phase_deg, phase_std, t_start_s, t_end_s })
}

// --------------------------------------------------------------------------- //
// Загрузка comments.json

fn load_comments(dir: &Path) -> std::collections::HashMap<String, String> {
    let path = dir.join("comments.json");
    let Ok(text) = std::fs::read_to_string(path) else {
        return Default::default();
    };
    serde_json::from_str(&text).unwrap_or_default()
}

fn recording_files_sorted(dir: &Path) -> Vec<(u32, PathBuf)> {
    let Ok(rd) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut files: Vec<(u32, PathBuf)> = rd
        .filter_map(|e| e.ok())
        .filter_map(|e| {
            let name = e.file_name().into_string().ok()?;
            let stem = name.strip_suffix(".parquet")?;
            let num: u32 = stem.parse().ok()?;
            Some((num, e.path()))
        })
        .collect();
    files.sort_by_key(|(n, _)| *n);
    files
}

// --------------------------------------------------------------------------- //
// main

fn main() {
    let cli = Cli::parse();

    let comments = load_comments(&cli.dir);
    let all_files = recording_files_sorted(&cli.dir);

    let targets: Vec<(u32, PathBuf)> = if cli.nums.is_empty() {
        all_files
    } else {
        let wanted: std::collections::HashSet<u32> = cli.nums.iter().copied().collect();
        all_files.into_iter().filter(|(n, _)| wanted.contains(n)).collect()
    };

    if targets.is_empty() {
        eprintln!("Нет подходящих файлов в {:?}", cli.dir);
        std::process::exit(1);
    }

    println!(
        "{:>3}  {:<16}  {:>6}  {:>5}  {:>12}  {:>7}  {:>5}  {}",
        "#", "Комментарий", "RPM", "Обор", "Амп (LSB)", "Фаза°", "±σ°", "Участок, с"
    );
    println!("{}", "-".repeat(80));

    for (num, path) in targets {
        let key     = format!("{:03}.parquet", num);
        let comment = comments.get(&key).map(|s| s.as_str()).unwrap_or("—");

        match analyze(&path, num, cli.cv, cli.min_revs) {
            Ok(r) => println!(
                "{:>3}  {:<16}  {:>6.1}  {:>5}  {:>12.1}  {:>7.1}  {:>5.1}  {:.1}–{:.1} с",
                r.num, comment, r.rpm, r.revs, r.amp, r.phase_deg, r.phase_std,
                r.t_start_s, r.t_end_s,
            ),
            Err(e) => println!("{:>3}  {:<16}  ОШИБКА: {}", num, comment, e),
        }
    }
}
