/// Задачи сбора данных:
/// - ISR (isr_capture): DRDY↓ и keyphasor edges → Priority2 ISR → Sample/KpEvent в SPSC.
///   Keyphasor tick захватывается прямо в ISR — точность на уровне ISR entry latency (~µs).
/// - adc_command_task: обработка команд (SetPga, SetDataRate) — забирает периферию из ISR.

use crate::ads1256::Ads1256;

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, Ordering};

use embassy_sync::{
    blocking_mutex::raw::CriticalSectionRawMutex,
    signal::Signal,
};
use embassy_time::{Duration, Timer};

use heapless::spsc::{Consumer, Producer, Queue};
use esp_hal::timer::systimer::SystemTimer;
use libm::sinf;
use static_cell::StaticCell;
use vibro_protocol::{Command, KpEvent, Sample};
use vibro_types::AdcCount;

/// Размер lock-free SPSC очереди сэмплов.
/// 4096 сэмплов ≈ 546 мс при 7500 SPS, 2 сек при 2000 SPS.
/// Большой буфер — WiFi/TCP stalls не теряют данные (~48KB RAM).
pub const SAMPLE_QUE_SIZE: usize = 4096;

/// Размер SPSC очереди keyphasor-событий.
/// Keyphasor — 1 раз за оборот: при 200 Гц (12000 RPM) = 200 событий/сек.
/// 32 слота — запас на ~160 мс при максимальных оборотах.
pub const KP_QUE_SIZE: usize = 32;

/// Lock-free SPSC очередь: adc_loop (producer) → stream_loop (consumer).
static SAMPLE_QUE: StaticCell<Queue<Sample, SAMPLE_QUE_SIZE>> = StaticCell::new();

/// Lock-free SPSC очередь: keyphasor_task (producer) → stream_loop (consumer).
static KP_QUE: StaticCell<Queue<KpEvent, KP_QUE_SIZE>> = StaticCell::new();

/// Сигнал команды от network → adc_loop.
/// Signal<_, Command> хранит последнее значение и будит задачу.
/// Если несколько команд пришло до обработки — применяется только последняя.
/// Это приемлемо: PGA/DataRate — идемпотентные настройки.
pub static CMD_SIGNAL: Signal<CriticalSectionRawMutex, Command> = Signal::new();
pub static CURRENT_SAMPLE_RATE: AtomicU16 = AtomicU16::new(2000);
pub static CURRENT_PGA: AtomicU8 = AtomicU8::new(1);

/// Флаг: reconfig в процессе. Пока true — stream_loop выбрасывает сэмплы из SPSC
/// не отправляя их: они могли накопиться со старым rate/PGA до take_peripherals.
/// Устанавливается перед take_peripherals, сбрасывается после give_back.
pub static RECONFIG_IN_PROGRESS: AtomicBool = AtomicBool::new(false);

/// Текущий уровень keyphasor GPIO. Обновляется keyphasor_task,
/// читается adc_loop для KEYPHASOR_LEVEL_FLAG (отладка).
pub static KP_LEVEL: AtomicBool = AtomicBool::new(false);


/// Инициализировать SPSC очереди и вернуть (sample_producer, sample_consumer, kp_producer, kp_consumer).
/// Вызывать один раз из main перед spawn задач.
pub fn init_queues() -> (
    Producer<'static, Sample>,
    Consumer<'static, Sample>,
    Producer<'static, KpEvent>,
    Consumer<'static, KpEvent>,
) {
    let sample_q = SAMPLE_QUE.init(Queue::new());
    let (sp, sc) = sample_q.split();
    let kp_q = KP_QUE.init(Queue::new());
    let (kp, kc) = kp_q.split();
    (sp, sc, kp, kc)
}

/// Заглушка: генерирует реалистичный виброметрический сигнал вместо реального ADC.
///
/// Моделирует:
///   ch0 = A1 * sin(2π * f_rot * t + φ0)    — 1x (дисбаланс), датчик X
///   ch1 = A1 * sin(2π * f_rot * t + φ1)    — 1x (дисбаланс), датчик Y (сдвиг 90°)
///
/// Параметры:
///   f_rot = 50 Гц (3000 RPM) — частота вращения
///   fs = 2000 Гц — частота дискретизации
///   A1 = 1_000_000 LSB (~12% от 24-бит диапазона) — амплитуда 1x
///   φ0 = 0, φ1 = π/2 — X/Y пара для восстановления вибровектора без keyphasor
///   Keyphasor каждые fs/f_rot = 40 сэмплов (1 раз за оборот).
///
/// ADS1256 24-бит: диапазон ±8_388_607. Типичный сигнал виброметра — тысячи LSB.
#[embassy_executor::task]
pub async fn adc_loop_stub(
    mut sample_tx: Producer<'static, Sample>,
    mut kp_tx: Producer<'static, KpEvent>,
) {
    // Период одного сэмпла: 500 мкс = 2 кГц
    const PERIOD_US: u64 = 500;

    // Частота вращения: 50 Гц (3000 RPM)
    const F_ROT_HZ: f32 = 50.0;
    // Частота дискретизации: 2 кГц
    const FS_HZ: f32 = 2000.0;
    // Амплитуда 1x: ~12% от 24-бит диапазона
    const AMP_1X: f32 = 1_000_000.0;
    // Фазовый сдвиг между каналами: 90° = π/2
    const PHASE_XY: f32 = core::f32::consts::FRAC_PI_2;

    // Угловой шаг за один сэмпл: 2π * f_rot / fs
    const OMEGA_STEP: f32 = 2.0 * core::f32::consts::PI * F_ROT_HZ / FS_HZ;

    // Keyphasor каждые fs/f_rot сэмплов = 40 сэмплов
    const KP_PERIOD: u32 = (FS_HZ / F_ROT_HZ) as u32; // 40

    let mut angle: f32 = 0.0;
    let mut count: u32 = 0;

    loop {
        if CMD_SIGNAL.signaled() {
            let _cmd = CMD_SIGNAL.wait().await;
        }

        CURRENT_SAMPLE_RATE.store(FS_HZ as u16, Ordering::Relaxed);

        let ch0 = (AMP_1X * sinf(angle)) as i32;
        let ch1 = (AMP_1X * sinf(angle + PHASE_XY)) as i32;

        // Keyphasor-событие — отдельный фрейм с точным timestamp.
        if count % KP_PERIOD == 0 {
            let tick = SystemTimer::unit_value(esp_hal::timer::systimer::Unit::Unit0);
            let _ = kp_tx.enqueue(KpEvent { tick });
        }

        let tick = SystemTimer::unit_value(esp_hal::timer::systimer::Unit::Unit0);
        let sample = Sample { ch0: AdcCount(ch0), ch1: AdcCount(ch1), flags: 0, tick };
        let _ = sample_tx.enqueue(sample);

        angle += OMEGA_STEP;
        if angle >= 2.0 * core::f32::consts::PI {
            angle -= 2.0 * core::f32::consts::PI;
        }
        count = count.wrapping_add(1);

        Timer::after(Duration::from_micros(PERIOD_US)).await;
    }
}


/// Диагностика ISR: печатает ISR calls/sec и drop count каждую секунду.
#[embassy_executor::task]
pub async fn isr_stats_task() {
    loop {
        Timer::after(Duration::from_secs(1)).await;
        let count = crate::isr_capture::ISR_COUNT.load(Ordering::Relaxed);
        crate::isr_capture::ISR_COUNT.store(0, Ordering::Relaxed);
        let drops = crate::isr_capture::ISR_DROP_COUNT.load(Ordering::Relaxed);
        crate::isr_capture::ISR_DROP_COUNT.store(0, Ordering::Relaxed);
        let kp_count = crate::isr_capture::KP_COUNT.load(Ordering::Relaxed);
        crate::isr_capture::KP_COUNT.store(0, Ordering::Relaxed);
        let kp_drops = crate::isr_capture::KP_DROP_COUNT.load(Ordering::Relaxed);
        crate::isr_capture::KP_DROP_COUNT.store(0, Ordering::Relaxed);
        esp_println::println!("ISR: {}/s  drops: {}  KP: {}/s  kp_drops: {}", count, drops, kp_count, kp_drops);
    }
}

/// Задача обработки команд (SetPga, SetDataRate).
/// Ждёт CMD_SIGNAL, забирает периферию из ISR, выполняет команду, возвращает.
/// Сам захват семплов — в ISR (isr_capture), эта задача только для reconfig.
#[embassy_executor::task]
pub async fn adc_command_task() {
    loop {
        let cmd = CMD_SIGNAL.wait().await;
        esp_println::println!("adc_command_task: got cmd {:?}, taking peripherals...", cmd);

        // Сигнализируем stream_loop: начинаем reconfig, старые сэмплы из очереди — мусор.
        RECONFIG_IN_PROGRESS.store(true, Ordering::Release);

        // Забираем периферию — ISR приостановлен.
        let (spi, cs, drdy, tx, kp_pin, kp_producer) = crate::isr_capture::take_peripherals();
        esp_println::println!("adc_command_task: peripherals taken");
        let mut adc = Ads1256::new(spi, cs, drdy);
        // Гарантируем command mode: ISR мог оставить SPI/чип в неопределённом состоянии.
        adc.ensure_command_mode();

        match cmd {
            Command::SetPga(v) => {
                let current = CURRENT_PGA.load(Ordering::Relaxed);
                if current == v {
                    esp_println::println!("ADS1256: PGA already {}, skip", v);
                } else {
                    esp_println::println!("ADS1256: set PGA={}", v);
                    if !adc.set_pga(v) {
                        esp_println::println!("ADS1256: set PGA failed");
                    } else {
                        CURRENT_PGA.store(v, Ordering::Relaxed);
                    }
                }
            }
            Command::SetDataRate(rate) => {
                let current = CURRENT_SAMPLE_RATE.load(Ordering::Relaxed);
                if current == rate.0 {
                    esp_println::println!("ADS1256: sample rate already {} SPS, skip", rate.0);
                } else {
                    esp_println::println!("ADS1256: set sample rate={} SPS", rate.0);
                    if adc.set_data_rate(rate.0) {
                        CURRENT_SAMPLE_RATE.store(rate.0, Ordering::Relaxed);
                    } else {
                        esp_println::println!("ADS1256: set sample rate failed");
                    }
                }
            }
        }

        // Возвращаем периферию в ISR — захват возобновляется.
        esp_println::println!("adc_command_task: returning peripherals...");
        let (spi, cs, drdy) = adc.into_parts();
        crate::isr_capture::give_back(spi, cs, drdy, tx, kp_pin, kp_producer);
        esp_println::println!("adc_command_task: give_back done");

        // Reconfig завершён — ISR уже пишет свежие сэмплы с новыми настройками.
        // Сбрасываем флаг: stream_loop перестаёт выбрасывать сэмплы.
        RECONFIG_IN_PROGRESS.store(false, Ordering::Release);
        esp_println::println!("adc_command_task: reconfig complete");
    }
}
