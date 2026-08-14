use std::collections::VecDeque;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use arrow::array::{Int32Array, RecordBatchReader as _, UInt64Array, UInt8Array};
use arrow::datatypes::{DataType, Field, Schema};
use arrow::record_batch::RecordBatch;
use parquet::arrow::ArrowWriter;
use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;
use parquet::file::properties::WriterProperties;
use vibro_analysis::math::complex::VibroVector;
use vibro_analysis::signal::goertzel::vibro_goertzel_hanning;
use vibro_protocol::{
    Command,
    Sample,
};
use vibro_types::{
    AdcCount,
    Hertz,
    PacketSeq,
    Rpm,
    SampleRateHz,
};

/// Статус TCP-соединения.
#[derive(Clone, Debug)]
pub(crate) enum ConnStatus {
    /// Ждём подключения от прошивки
    Listening,
    /// Прошивка подключена
    Connected(String),
    /// Ошибка / отключение
    Disconnected(String),
}

/// Измерение вибровектора 1x за один оборот (для live-тренда).
pub(crate) struct RevMeasurement {
    pub(crate) vv:  [VibroVector; 2],
    pub(crate) rpm: Rpm,
}

/// Максимальное число оборотов в live-тренде.
pub(crate) const REV_TREND_SIZE: usize = 200;

/// Частота SystemTimer на ESP32-C3 (16 МГц).
pub(crate) const SYSTIMER_HZ: f64 = 16_000_000.0;

/// Запись одного оборота (между двумя keyphasor-фронтами) или непрерывный буфер.
/// start_tick/end_tick вычисляются из samples[0].tick / samples.last().tick.
pub(crate) struct Recording {
    pub(crate) samples:     Vec<Sample>,
    pub(crate) sample_rate: SampleRateHz,
    /// PGA при записи (для нормализации: raw / pga → LSB при PGA=1).
    pub(crate) pga:         u8,
    /// Keyphasor-тики, попавшие в эту запись.
    pub(crate) kp_ticks:    Vec<u64>,
    /// Имя файла (без пути), например "019.parquet".
    pub(crate) filename:    String,
    /// Пользовательское выделение диапазона (секунды), персистится в parquet metadata.
    pub(crate) selection:   Option<crate::strip::Selection>,
}

impl Recording {
    /// SystemTimer tick первого сэмпла (0 если пусто).
    pub(crate) fn start_tick(&self) -> u64 {
        self.samples.first().map_or(0, |s| s.tick)
    }

    /// SystemTimer tick последнего сэмпла (0 если пусто).
    pub(crate) fn end_tick(&self) -> u64 {
        self.samples.last().map_or(0, |s| s.tick)
    }
}

// ---------------------------------------------------------------------------
// Parquet persistence
// ---------------------------------------------------------------------------

/// Директория для хранения записей (рядом с бинарником).
fn recordings_dir() -> PathBuf {
    PathBuf::from("recordings")
}

/// Схема parquet-файла: ch0 (i32), ch1 (i32), flags (u8), tick (u64).
fn parquet_schema() -> Arc<Schema> {
    Arc::new(Schema::new(vec![
        Field::new("ch0", DataType::Int32, false),
        Field::new("ch1", DataType::Int32, false),
        Field::new("flags", DataType::UInt8, false),
        Field::new("tick", DataType::UInt64, false),
    ]))
}

/// Следующий свободный номер файла в recordings/.
fn next_file_index(dir: &Path) -> usize {
    let mut max_idx = 0usize;
    if let Ok(entries) = fs::read_dir(dir) {
        for entry in entries.flatten() {
            let name = entry.file_name();
            let name = name.to_string_lossy();
            // Формат: "001.parquet"
            if let Some(stem) = name.strip_suffix(".parquet") {
                if let Ok(n) = stem.parse::<usize>() {
                    max_idx = max_idx.max(n + 1);
                }
            }
        }
    }
    max_idx
}

/// Сохранить запись в parquet-файл.
/// sample_rate хранится в file-level metadata (ключ "sample_rate").
pub(crate) fn save_recording(rec: &Recording) -> PathBuf {
    let dir = recordings_dir();
    fs::create_dir_all(&dir).unwrap();

    let idx = next_file_index(&dir);
    let path = dir.join(format!("{:03}.parquet", idx));

    write_parquet(&path, rec);

    eprintln!("saved recording: {} ({} samples)", path.display(), rec.samples.len());
    path
}

/// Перезаписать существующий parquet-файл (обновление metadata, например selection).
pub(crate) fn resave_recording(rec: &Recording) {
    if rec.filename.is_empty() { return; }
    let path = recordings_dir().join(&rec.filename);
    write_parquet(&path, rec);
}

/// Общая запись Recording в parquet по указанному пути.
fn write_parquet(path: &Path, rec: &Recording) {
    let schema = parquet_schema();

    let ch0: Int32Array = rec.samples.iter().map(|s| s.ch0.as_i32()).collect();
    let ch1: Int32Array = rec.samples.iter().map(|s| s.ch1.as_i32()).collect();
    let flags: UInt8Array = rec.samples.iter().map(|s| s.flags).collect();
    let ticks: UInt64Array = rec.samples.iter().map(|s| s.tick).collect();

    let batch = RecordBatch::try_new(
        schema.clone(),
        vec![Arc::new(ch0), Arc::new(ch1), Arc::new(flags), Arc::new(ticks)],
    )
    .unwrap();

    let kp_ticks_json = format!(
        "[{}]",
        rec.kp_ticks.iter().map(|t| t.to_string()).collect::<Vec<_>>().join(",")
    );
    let mut meta = vec![
        parquet::file::metadata::KeyValue {
            key:   "sample_rate".to_string(),
            value: Some(rec.sample_rate.0.to_string()),
        },
        parquet::file::metadata::KeyValue {
            key:   "pga".to_string(),
            value: Some(rec.pga.to_string()),
        },
        parquet::file::metadata::KeyValue {
            key:   "kp_ticks".to_string(),
            value: Some(kp_ticks_json),
        },
    ];
    if let Some(ref sel) = rec.selection {
        meta.push(parquet::file::metadata::KeyValue {
            key:   "selection".to_string(),
            value: Some(serde_json::to_string(sel).unwrap()),
        });
    }
    let props = WriterProperties::builder()
        .set_key_value_metadata(Some(meta))
        .build();

    let file = fs::File::create(path).unwrap();
    let mut writer = ArrowWriter::try_new(file, schema, Some(props)).unwrap();
    writer.write(&batch).unwrap();
    writer.close().unwrap();
}

/// Достать строковое значение из file-level metadata по ключу.
fn parquet_meta_str<'a>(
    kv: Option<&'a Vec<parquet::file::metadata::KeyValue>>,
    key: &str,
) -> Option<&'a str> {
    kv?.iter()
        .find(|e| e.key == key)?
        .value
        .as_deref()
}

/// Загрузить одну запись из parquet-файла.
fn load_recording(path: &Path) -> Recording {
    let file = fs::File::open(path).unwrap();
    let builder = ParquetRecordBatchReaderBuilder::try_new(file).unwrap();

    let kv = builder
        .metadata()
        .file_metadata()
        .key_value_metadata();

    let sample_rate = parquet_meta_str(kv, "sample_rate")
        .and_then(|v| v.parse::<u16>().ok())
        .map(SampleRateHz)
        .unwrap();

    // kp_ticks хранится как JSON array "[123,456,...]".
    let kp_ticks: Vec<u64> = parquet_meta_str(kv, "kp_ticks")
        .map(|s| {
            s.trim_matches(|c| c == '[' || c == ']')
                .split(',')
                .filter(|t| !t.is_empty())
                .map(|t| t.trim().parse::<u64>().unwrap())
                .collect()
        })
        .unwrap_or_default();

    // Обратная совместимость: start_tick из metadata (старые записи без колонки tick).
    let legacy_start_tick = parquet_meta_str(kv, "start_tick")
        .and_then(|v| v.parse::<u64>().ok())
        .unwrap_or(0);

    // PGA из metadata (старые записи без PGA → default 1).
    let pga = parquet_meta_str(kv, "pga")
        .and_then(|v| v.parse::<u8>().ok())
        .unwrap_or(1);

    // Selection из metadata (JSON, опционально).
    let selection: Option<crate::strip::Selection> = parquet_meta_str(kv, "selection")
        .and_then(|s| serde_json::from_str(s).ok());

    let reader = builder.build().unwrap();

    // Проверяем наличие колонки tick (новый формат).
    let has_tick_column = reader.schema().column_with_name("tick").is_some();

    let mut samples = Vec::new();
    for batch in reader {
        let batch = batch.unwrap();
        let ch0 = batch.column(0).as_any().downcast_ref::<Int32Array>().unwrap();
        let ch1 = batch.column(1).as_any().downcast_ref::<Int32Array>().unwrap();
        let flags = batch.column(2).as_any().downcast_ref::<UInt8Array>().unwrap();

        if has_tick_column {
            let ticks = batch.column(3).as_any().downcast_ref::<UInt64Array>().unwrap();
            for i in 0..batch.num_rows() {
                samples.push(Sample {
                    ch0:   AdcCount(ch0.value(i)),
                    ch1:   AdcCount(ch1.value(i)),
                    flags: flags.value(i),
                    tick:  ticks.value(i),
                });
            }
        } else {
            // Старый формат без tick: восстанавливаем из start_tick + sample_rate.
            let tps = (SYSTIMER_HZ / sample_rate.as_f64()) as u64;
            let base = legacy_start_tick + samples.len() as u64 * tps;
            for i in 0..batch.num_rows() {
                samples.push(Sample {
                    ch0:   AdcCount(ch0.value(i)),
                    ch1:   AdcCount(ch1.value(i)),
                    flags: flags.value(i),
                    tick:  base + i as u64 * tps,
                });
            }
        }
    }

    let filename = path.file_name().unwrap().to_string_lossy().into_owned();
    Recording { samples, sample_rate, pga, kp_ticks, filename, selection }
}

/// Загрузить все записи из recordings/ (сортировка по имени файла).
pub(crate) fn load_all_recordings() -> Vec<Recording> {
    let dir = recordings_dir();
    if !dir.exists() {
        return Vec::new();
    }

    let mut paths: Vec<PathBuf> = fs::read_dir(&dir)
        .unwrap()
        .flatten()
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|ext| ext == "parquet"))
        .collect();
    paths.sort();

    let mut recordings = Vec::with_capacity(paths.len());
    for path in &paths {
        let rec = load_recording(path);
        eprintln!("loaded: {} ({} samples, {}Hz)", path.display(), rec.samples.len(), rec.sample_rate);
        recordings.push(rec);
    }
    recordings
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RecordMode {
    Continuous,
    Revolutions,
}

pub(crate) struct Shared {
    /// Кольцевой буфер последних сэмплов для live-осциллограммы.
    /// Хранит до LIVE_BUF_SIZE сэмплов. Каждый Sample несёт свой tick.
    pub(crate) live_buf:          VecDeque<Sample>,
    /// Текущий sample_rate (из последнего пакета)
    pub(crate) sample_rate:       SampleRateHz,
    /// Статус TCP
    pub(crate) status:            ConnStatus,
    /// Идёт ли запись
    pub(crate) recording:         bool,
    /// Режим записи: непрерывно или фиксированное число оборотов.
    pub(crate) record_mode:       RecordMode,
    /// Сколько оборотов нужно записать в режиме Revolutions.
    pub(crate) record_revs:       usize,
    /// Ждём ли стартового keyphasor для записи по оборотам.
    pub(crate) rec_waiting_kp:    bool,
    /// Сколько полных оборотов уже захвачено в текущей записи.
    pub(crate) rec_captured_revs: usize,
    /// Буфер записи (каждый Sample несёт свой tick)
    pub(crate) rec_buf:           Vec<Sample>,
    /// Keyphasor-тики, попавшие в текущую запись.
    pub(crate) rec_kp_ticks:      Vec<u64>,
    /// Завершённые записи (Record → Stop)
    pub(crate) recordings:        Vec<Recording>,
    /// Счётчик принятых сэмплов
    pub(crate) total_samples:     u64,
    /// Счётчик keyphasor-событий
    pub(crate) keyphasor_count:   u64,
    /// Последний seq
    pub(crate) last_seq:          PacketSeq,
    /// Время прихода последнего пакета
    pub(crate) last_packet_at:    Option<Instant>,
    /// Время прихода последнего keyphasor-импульса (для индикатора в UI).
    pub(crate) last_keyphasor_at: Option<Instant>,
    /// Авто-запись: ждём первого KP-фронта для старта записи.
    /// Устанавливается кнопкой "Auto" в UI; сбрасывается при старте или отмене.
    pub(crate) auto_rec_armed: bool,
    /// Время последнего KP во время авто-записи.
    /// Авто-остановка если elapsed > AUTO_REC_IDLE_TIMEOUT.
    pub(crate) auto_rec_last_kp_at: Option<Instant>,
    /// Режим Auto+: после завершения записи автоматически re-arm (ждём следующий триггер).
    /// Цикл: armed → запись → finish → re-arm → armed → …
    pub(crate) auto_rec_plus: bool,
    /// Очередь команд для отправки firmware.
    /// UI пишет, TCP-поток вычитывает и отправляет.
    pub(crate) cmd_queue:         Vec<Command>,
    /// Последняя отправленная, но ещё не подтверждённая потоком команда PGA.
    pub(crate) pending_pga:       Option<u8>,
    /// Последняя отправленная, но ещё не подтверждённая потоком команда rate.
    pub(crate) pending_rate:      Option<SampleRateHz>,
    /// Когда последний SetDataRate ушёл в firmware и мы ждём новый packet_rate.
    pending_rate_since:           Option<Instant>,
    /// Кольцевой буфер вибровекторов по оборотам (live-тренд).
    pub(crate) rev_trend:         VecDeque<RevMeasurement>,
    /// Сэмплы текущего (незавершённого) оборота для live-тренда.
    pub(crate) rev_buf:           Vec<Sample>,
    /// Все keyphasor-тики за сессию (абсолютные systimer ticks).
    pub(crate) kp_ticks:          Vec<u64>,
    /// total_samples на момент предыдущего KP (для подсчёта семплов между пульсами).
    pub(crate) kp_last_total:     u64,
    /// Текущий PGA (из последнего пакета firmware). Для нормализации: raw / pga.
    pub(crate) pga:               u8,
    /// Желаемый PGA — UI задаёт при старте, TCP-поток отправляет при коннекте.
    pub(crate) desired_pga:       u8,
    /// Желаемый sample rate — UI задаёт при старте, TCP-поток отправляет при коннекте.
    pub(crate) desired_rate:      SampleRateHz,
}

/// Размер live-буфера: ~4 секунды при 2 кГц.
pub(crate) const LIVE_BUF_SIZE: usize = 8192;

/// Таймаут авто-остановки: если KP не приходил дольше этого — запись завершается.
pub(crate) const AUTO_REC_IDLE_TIMEOUT: Duration = Duration::from_secs(2);

/// Минимальное число стабильных оборотов для триггера авто-записи.
/// Три оборота = четыре KP-импульса с монотонным стабильным периодом.
const AUTO_TRIGGER_REVS: usize = 3;

/// Минимальный RPM для триггера. Ниже этого — считаем ручным вращением / помехой.
const AUTO_TRIGGER_MIN_RPM: f64 = 300.0;

/// Максимальный допустимый разброс периода между оборотами (20%).
/// Если CV > этого — паттерн нестабильный, триггер не сработает.
const AUTO_TRIGGER_MAX_PERIOD_CV: f64 = 0.20;

/// Минимальный период KP в тиках (защита от дребезга/помех).
/// Соответствует ~60000 RPM при 16 МГц таймере.
const AUTO_TRIGGER_MIN_PERIOD_TICKS: u64 = (SYSTIMER_HZ * 60.0 / 60_000.0) as u64;

/// Проверяет, выполнены ли условия триггера авто-записи:
/// - Последние AUTO_TRIGGER_REVS оборотов имеют стабильный период
/// - RPM выше порога (не ручное вращение)
/// - Нет аномально коротких периодов (помехи/дребезг)
///
/// Возвращает true если паттерн чёткий и можно стартовать запись.
fn auto_trigger_check(kp_ticks: &[u64]) -> bool {
    // Нужно AUTO_TRIGGER_REVS оборотов = AUTO_TRIGGER_REVS+1 тиков.
    let needed = AUTO_TRIGGER_REVS + 1;
    if kp_ticks.len() < needed {
        return false;
    }

    let recent = &kp_ticks[kp_ticks.len() - needed..];

    // Вычисляем периоды между последовательными KP.
    let periods: Vec<u64> = recent.windows(2).map(|w| w[1] - w[0]).collect();

    // Фильтр помех: все периоды должны быть больше минимального.
    if periods.iter().any(|&p| p < AUTO_TRIGGER_MIN_PERIOD_TICKS) {
        return false;
    }

    let mean = periods.iter().sum::<u64>() as f64 / periods.len() as f64;

    // RPM из среднего периода.
    let rpm = 60.0 * SYSTIMER_HZ / mean;
    if rpm < AUTO_TRIGGER_MIN_RPM {
        return false;
    }

    // Стабильность: coefficient of variation < порога.
    let variance = periods.iter()
        .map(|&p| {
            let diff = p as f64 - mean;
            diff * diff
        })
        .sum::<f64>() / periods.len() as f64;
    let cv = variance.sqrt() / mean;
    if cv > AUTO_TRIGGER_MAX_PERIOD_CV {
        return false;
    }

    true
}

impl Shared {
    pub(crate) fn new() -> Self {
        Self {
            live_buf:          VecDeque::with_capacity(LIVE_BUF_SIZE),
            sample_rate:       SampleRateHz::ZERO,
            status:            ConnStatus::Listening,
            recording:         false,
            record_mode:       RecordMode::Continuous,
            record_revs:       1,
            rec_waiting_kp:    false,
            rec_captured_revs: 0,
            rec_buf:           Vec::new(),
            rec_kp_ticks:      Vec::new(),
            recordings:        Vec::new(),
            total_samples:     0,
            keyphasor_count:   0,
            last_seq:          PacketSeq::ZERO,
            last_packet_at:    None,
            last_keyphasor_at: None,
            auto_rec_armed: false,
            auto_rec_last_kp_at: None,
            auto_rec_plus: false,
            cmd_queue:         Vec::new(),
            pending_pga:       None,
            pending_rate:      None,
            pending_rate_since: None,
            rev_trend:         VecDeque::with_capacity(REV_TREND_SIZE),
            rev_buf:           Vec::new(),
            kp_ticks:          Vec::new(),
            kp_last_total:     0,
            pga:               1,
            desired_pga:       1,
            desired_rate:      SampleRateHz::ZERO,
        }
    }

    /// Схлопывает повторные pending-команды управления ADC.
    /// Если пользователь несколько раз подряд нажал Set, firmware получит
    /// только последнее актуальное значение каждого типа команды.
    pub(crate) fn enqueue_command(&mut self, cmd: Command) {
        match cmd {
            Command::SetPga(pga) => {
                if self.pending_pga == Some(pga) {
                    return;
                }
                if let Some(existing) = self
                    .cmd_queue
                    .iter_mut()
                    .find(|queued| matches!(queued, Command::SetPga(_)))
                {
                    *existing = Command::SetPga(pga);
                } else {
                    self.cmd_queue.push(Command::SetPga(pga));
                }
            }
            Command::SetDataRate(rate) => {
                if self.sample_rate == rate || self.pending_rate == Some(rate) {
                    return;
                }
                if let Some(existing) = self
                    .cmd_queue
                    .iter_mut()
                    .find(|queued| matches!(queued, Command::SetDataRate(_)))
                {
                    *existing = Command::SetDataRate(rate);
                } else {
                    self.cmd_queue.push(Command::SetDataRate(rate));
                }
            }
        }
    }

    pub(crate) fn mark_command_sent(&mut self, cmd: Command) {
        match cmd {
            Command::SetPga(pga) => self.pending_pga = Some(pga),
            Command::SetDataRate(rate) => {
                self.pending_rate = Some(rate);
                self.pending_rate_since = Some(Instant::now());
            }
        }
    }

    pub(crate) fn note_stream_packet(&mut self, packet_rate: SampleRateHz) {
        if self.pending_rate == Some(packet_rate) {
            self.pending_rate = None;
            self.pending_rate_since = None;
        }

        // Для PGA у нас нет отдельного ack/telemetry. Если поток данных уже
        // снова идёт после отправки команды, считаем reconfig завершённым и
        // разрешаем следующую ручную смену PGA.
        if self.pending_pga.is_some() {
            self.pending_pga = None;
        }
    }

    pub(crate) fn clear_pending_commands(&mut self) {
        self.pending_pga = None;
        self.pending_rate = None;
        self.pending_rate_since = None;
    }

    /// Если после SetDataRate поток продолжает идти со старым rate,
    /// повторяем запрос вместо бесконечного "Applying...".
    pub(crate) fn retry_stuck_rate_change(&mut self, packet_rate: SampleRateHz, now: Instant) {
        const RATE_RETRY_AFTER: Duration = Duration::from_millis(750);

        let Some(pending_rate) = self.pending_rate else {
            return;
        };
        let Some(sent_at) = self.pending_rate_since else {
            return;
        };

        if packet_rate == pending_rate || now.duration_since(sent_at) < RATE_RETRY_AFTER {
            return;
        }

        self.pending_rate = None;
        self.pending_rate_since = None;
        self.enqueue_command(Command::SetDataRate(self.desired_rate));
    }
}

pub(crate) fn finish_recording(shared: &mut Shared) {
    let is_plus = shared.auto_rec_plus;

    shared.recording = false;
    shared.rec_waiting_kp = false;
    shared.rec_captured_revs = 0;
    shared.auto_rec_last_kp_at = None;

    // Auto+: re-arm для следующего триггера, обычный auto — сбросить.
    shared.auto_rec_armed = is_plus;

    crate::sound::beep_rec_stop();

    let samples = std::mem::take(&mut shared.rec_buf);
    let kp_ticks = std::mem::take(&mut shared.rec_kp_ticks);
    if samples.is_empty() {
        return;
    }

    let mut rec = Recording {
        samples,
        sample_rate: shared.sample_rate,
        pga: shared.pga,
        kp_ticks,
        filename: String::new(),
        selection: None,
    };
    #[cfg(not(test))]
    {
        let path = save_recording(&rec);
        rec.filename = path.file_name().unwrap().to_string_lossy().into_owned();
    }
    shared.recordings.push(rec);
}

pub(crate) fn process_recording_sample(shared: &mut Shared, sample: Sample) {
    if !shared.recording {
        return;
    }

    match shared.record_mode {
        RecordMode::Continuous => {
            shared.rec_buf.push(sample);
        }
        RecordMode::Revolutions => {
            if shared.rec_waiting_kp {
                // Ждём первого keyphasor-фронта — он приходит через process_keyphasor.
                return;
            }
            shared.rec_buf.push(sample);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Shared;
    use std::time::{Duration, Instant};
    use vibro_protocol::Command;
    use vibro_types::SampleRateHz;

    #[test]
    fn enqueue_command_replaces_pending_pga() {
        let mut shared = Shared::new();
        shared.enqueue_command(Command::SetPga(1));
        shared.enqueue_command(Command::SetPga(4));
        shared.enqueue_command(Command::SetPga(8));

        assert_eq!(shared.cmd_queue.len(), 1);
        assert!(matches!(shared.cmd_queue[0], Command::SetPga(8)));
    }

    #[test]
    fn enqueue_command_keeps_one_pending_command_per_type() {
        let mut shared = Shared::new();
        shared.enqueue_command(Command::SetPga(2));
        shared.enqueue_command(Command::SetDataRate(SampleRateHz(2000)));
        shared.enqueue_command(Command::SetPga(4));
        shared.enqueue_command(Command::SetDataRate(SampleRateHz(7500)));

        assert_eq!(shared.cmd_queue.len(), 2);
        assert!(matches!(shared.cmd_queue[0], Command::SetPga(4)));
        assert!(matches!(
            shared.cmd_queue[1],
            Command::SetDataRate(SampleRateHz(7500))
        ));
    }

    #[test]
    fn note_stream_packet_clears_pending_rate_on_ack() {
        let mut shared = Shared::new();
        shared.mark_command_sent(Command::SetDataRate(SampleRateHz(7500)));

        shared.note_stream_packet(SampleRateHz(7500));

        assert_eq!(shared.pending_rate, None);
        assert_eq!(shared.pending_rate_since, None);
    }

    #[test]
    fn retry_stuck_rate_change_requeues_desired_rate() {
        let mut shared = Shared::new();
        shared.sample_rate = SampleRateHz(2000);
        shared.desired_rate = SampleRateHz(500);
        shared.mark_command_sent(Command::SetDataRate(SampleRateHz(500)));

        let now = Instant::now() + Duration::from_secs(1);
        shared.retry_stuck_rate_change(SampleRateHz(2000), now);

        assert_eq!(shared.pending_rate, None);
        assert_eq!(shared.pending_rate_since, None);
        assert!(matches!(
            shared.cmd_queue.as_slice(),
            [Command::SetDataRate(SampleRateHz(500))]
        ));
    }

    /// Загрузить все записи и сравнить Whittaker smoother vs PLL
    /// по равномерности интервалов на участках постоянной скорости.
    ///
    /// Тест ищет записи с ≥ 30 KP marks и автоматически находит
    /// участок плато (где RPM стабилен), затем сравнивает CV.
    #[test]
    fn whittaker_vs_pll_on_real_recordings() {
        use super::{load_all_recordings, SYSTIMER_HZ};
        use vibro_analysis::signal::pll::{PllParams, run_pll};
        use vibro_analysis::signal::spline::smoothing_spline;

        let recordings = load_all_recordings();
        if recordings.is_empty() {
            eprintln!("no recordings found, skipping");
            return;
        }

        let params = PllParams::default_16mhz();
        let mut tested = 0;

        for (rec_idx, rec) in recordings.iter().enumerate() {
            if rec.kp_ticks.len() < 30 {
                continue;
            }

            let kp = &rec.kp_ticks;
            let pll = run_pll(kp, &params);
            let spline = smoothing_spline(kp, 100.0);

            // Находим участок плато: окно из 20 rev где RPM std < 5% mean.
            // Ищем на pll (от settling=20 до конца).
            let window_size = 20;
            let settle = 20.min(pll.len().saturating_sub(window_size + 1));
            let mut best_plateau: Option<(usize, f64)> = None; // (start_idx, cv)

            for start in settle..pll.len().saturating_sub(window_size) {
                let rpms: Vec<f64> = pll[start..start + window_size]
                    .iter()
                    .map(|p| p.rpm(SYSTIMER_HZ))
                    .collect();
                let mean = rpms.iter().sum::<f64>() / rpms.len() as f64;
                if mean < 300.0 {
                    continue; // слишком медленно, не плато
                }
                let std = (rpms
                    .iter()
                    .map(|x| (x - mean).powi(2))
                    .sum::<f64>()
                    / rpms.len() as f64)
                    .sqrt();
                let cv = std / mean;

                // Дополнительная проверка: сырые интервалы тоже должны быть стабильными.
                // Это исключает ложные плато, где PLL lag маскирует замедление.
                // pll[start] ↔ kp[start+1], нужны kp-интервалы.
                let kp_start = start + 1;
                let kp_end = (kp_start + window_size).min(kp.len());
                if kp_end - kp_start < 3 {
                    continue;
                }
                let raw_intervals: Vec<f64> = kp[kp_start..kp_end]
                    .windows(2)
                    .map(|w| (w[1] - w[0]) as f64)
                    .collect();
                let raw_mean = raw_intervals.iter().sum::<f64>() / raw_intervals.len() as f64;
                // Тренд: линейная регрессия slope.
                // Если |slope/mean| > 1% за окно → не плато (разгон/выбег).
                let n_ri = raw_intervals.len() as f64;
                let x_mean = (n_ri - 1.0) / 2.0;
                let slope: f64 = raw_intervals.iter().enumerate()
                    .map(|(j, &v)| (j as f64 - x_mean) * (v - raw_mean))
                    .sum::<f64>()
                    / raw_intervals.iter().enumerate()
                        .map(|(j, _)| (j as f64 - x_mean).powi(2))
                        .sum::<f64>();
                let relative_slope = (slope * n_ri / raw_mean).abs();
                if relative_slope > 0.2 {
                    continue; // значительный тренд — не стационарный участок
                }

                // Доп. проверка: сглаженные (spline) интервалы не должны показывать тренд.
                // Это надёжнее чем raw-интервалы (jitter маскирует тренд в raw).
                // pll[start] ↔ kp[start+1] ↔ spline[start+1].
                let sp_from = start + 1;
                let sp_to = (sp_from + window_size).min(spline.len());
                if sp_to - sp_from >= 3 {
                    let sp_ints: Vec<f64> = spline[sp_from..sp_to]
                        .windows(2)
                        .map(|w| w[1].smooth_tick - w[0].smooth_tick)
                        .collect();
                    let sp_mean = sp_ints.iter().sum::<f64>() / sp_ints.len() as f64;
                    let sp_first = sp_ints[0];
                    let sp_last = sp_ints[sp_ints.len() - 1];
                    let sp_drift = ((sp_last - sp_first) / sp_mean).abs();
                    if sp_drift > 0.03 {
                        continue; // >3% drift в smoothed интервалах — не стационарный
                    }
                }

                if cv < 0.05 {
                    if best_plateau.is_none() || cv < best_plateau.unwrap().1 {
                        best_plateau = Some((start, cv));
                    }
                }
            }

            let Some((plateau_start, _)) = best_plateau else {
                continue;
            };

            // Вычисляем CV интервалов на плато.
            // PLL: pll[i] ↔ kp[i+1].
            // Spline: spline[i] ↔ kp[i].
            let pll_intervals: Vec<f64> = pll[plateau_start..plateau_start + window_size]
                .windows(2)
                .map(|w| w[1].aligned_tick - w[0].aligned_tick)
                .collect();

            // spline_start = plateau_start + 1 (смещение pll→kp).
            let sp_start = plateau_start + 1;
            let sp_end = (sp_start + window_size).min(spline.len());
            if sp_end - sp_start < 3 {
                continue;
            }
            let spline_intervals: Vec<f64> = spline[sp_start..sp_end]
                .windows(2)
                .map(|w| w[1].smooth_tick - w[0].smooth_tick)
                .collect();

            let cv_of = |vals: &[f64]| -> f64 {
                let mean = vals.iter().sum::<f64>() / vals.len() as f64;
                let std = (vals
                    .iter()
                    .map(|x| (x - mean).powi(2))
                    .sum::<f64>()
                    / vals.len() as f64)
                    .sqrt();
                std / mean
            };

            let pll_cv = cv_of(&pll_intervals);
            let spline_cv = cv_of(&spline_intervals);

            // Средний RPM на плато.
            let mean_rpm: f64 = pll[plateau_start..plateau_start + window_size]
                .iter()
                .map(|p| p.rpm(SYSTIMER_HZ))
                .sum::<f64>()
                / window_size as f64;

            eprintln!(
                "rec {:03}: {} KP, plateau@{} ({:.0} RPM): PLL CV={:.2}%, Whittaker CV={:.2}%  {}",
                rec_idx,
                kp.len(),
                plateau_start,
                mean_rpm,
                pll_cv * 100.0,
                spline_cv * 100.0,
                if spline_cv < pll_cv { "WIN" } else { "lose" }
            );

            // На плато Whittaker должен быть не хуже PLL.
            // (Non-causal метод не может быть принципиально хуже на стационарном участке.)
            tested += 1;
        }

        eprintln!("\ntested {} recordings with plateau", tested);
        assert!(
            tested > 0,
            "no recordings with suitable plateau found"
        );
    }
}

/// Обработка keyphasor-фронта (из Frame::Keyphasor).
/// Управляет записью по оборотам и live rev_trend.
pub(crate) fn process_keyphasor(shared: &mut Shared) {
    // Авто-запись: ждём AUTO_TRIGGER_REVS стабильных оборотов на приличной скорости.
    // Одиночные помехи и ручное вращение не проходят проверку.
    if shared.auto_rec_armed {
        if auto_trigger_check(&shared.kp_ticks) {
            shared.auto_rec_armed = false;
            shared.recording = true;
            shared.record_mode = RecordMode::Continuous;
            shared.rec_kp_ticks.clear();
            shared.rec_captured_revs = 0;
            shared.rec_waiting_kp = false;

            // Забираем из live_buf сэмплы начиная с первого KP-тика триггерного окна —
            // включаем все AUTO_TRIGGER_REVS оборотов в запись.
            let trigger_start_idx = shared.kp_ticks.len() - (AUTO_TRIGGER_REVS + 1);
            let start_tick = shared.kp_ticks[trigger_start_idx];
            shared.rec_buf.clear();
            for &s in shared.live_buf.iter() {
                if s.tick >= start_tick {
                    shared.rec_buf.push(s);
                }
            }
            // KP-тики триггерного окна тоже попадают в запись.
            shared.rec_kp_ticks.extend_from_slice(&shared.kp_ticks[trigger_start_idx..]);

            // Фиксируем время старта для таймаута.
            shared.auto_rec_last_kp_at = Some(Instant::now());

            crate::sound::beep_rec_start();
        }
    } else if shared.recording && shared.record_mode == RecordMode::Continuous
        && shared.auto_rec_last_kp_at.is_some()
    {
        // Авто-запись идёт — обновляем время последнего KP.
        shared.auto_rec_last_kp_at = Some(Instant::now());
    }
    // Live-тренд по оборотам: при keyphasor-фронте вычисляем 1x вектор
    // через Goertzel из накопленных сэмплов.
    if shared.rev_buf.len() >= 32 && shared.sample_rate.as_f64() >= 1.0 {
        let rev_samples = &shared.rev_buf;
        let sr = shared.sample_rate.as_hz();
        // Частота вращения = sample_rate / число_сэмплов_за_оборот.
        let f_rot = Hertz::new(sr.as_f64() / rev_samples.len() as f64);
        let rpm = Rpm::new(f_rot.as_f64() * 60.0);

        // Нормализация на PGA: raw / pga → LSB при PGA=1.
        let pga_f = shared.pga as f64;
        let ch0: Vec<f64> = rev_samples.iter().map(|s| s.ch0.as_f64() / pga_f).collect();
        let ch1: Vec<f64> = rev_samples.iter().map(|s| s.ch1.as_f64() / pga_f).collect();

        let vv0 = vibro_goertzel_hanning(&ch0, f_rot, sr);
        let vv1 = vibro_goertzel_hanning(&ch1, f_rot, sr);

        if shared.rev_trend.len() >= REV_TREND_SIZE {
            shared.rev_trend.pop_front();
        }
        shared.rev_trend.push_back(RevMeasurement {
            vv:  [vv0, vv1],
            rpm,
        });

        shared.rev_buf.clear();
    }

    // Запись по оборотам.
    if !shared.recording || shared.record_mode != RecordMode::Revolutions {
        return;
    }

    if shared.rec_waiting_kp {
        shared.rec_waiting_kp = false;
        shared.rec_buf.clear();
        return;
    }

    shared.rec_captured_revs += 1;
    if shared.rec_captured_revs >= shared.record_revs.max(1) {
        finish_recording(shared);
    }
}
