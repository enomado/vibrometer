//! Звуковые уведомления при старте/стопе записи.
//! WAV-файлы из assets/ (CC0, bigsoundbank.com).
//!
//! DeviceSink создаётся один раз (lazy) и переиспользуется —
//! это убирает задержку ~200ms на открытие аудио-устройства при каждом beep.

use std::io::Cursor;
use std::sync::OnceLock;
use rodio::{Decoder, DeviceSinkBuilder, mixer::Mixer, stream::MixerDeviceSink};

/// Встроенные WAV-файлы (include_bytes компилирует в бинарник).
static START_WAV: &[u8] = include_bytes!("../assets/rec_start.wav");
static STOP_WAV: &[u8] = include_bytes!("../assets/rec_stop.wav");

/// Глобальный аудио-выход. Создаётся один раз при первом вызове.
static SINK: OnceLock<MixerDeviceSink> = OnceLock::new();

fn global_mixer() -> &'static Mixer {
    SINK.get_or_init(|| {
        let mut sink = DeviceSinkBuilder::open_default_sink().unwrap();
        sink.log_on_drop(false);
        sink
    })
    .mixer()
}

/// Прогреть аудио-устройство при старте приложения.
/// Вызвать один раз до первого beep, чтобы убрать задержку ~500ms.
pub fn init() {
    global_mixer();
}

/// Проиграть встроенный WAV через глобальный mixer.
fn play_wav(data: &'static [u8]) {
    let cursor = Cursor::new(data);
    let source = Decoder::new(cursor).unwrap();
    global_mixer().add(source);
}

/// Звук начала записи.
pub fn beep_rec_start() {
    play_wav(START_WAV);
}

/// Звук конца записи.
pub fn beep_rec_stop() {
    play_wav(STOP_WAV);
}
