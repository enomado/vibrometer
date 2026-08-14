/// Interrupt-driven захват ADS1256 и keyphasor.
///
/// GPIO ISR по DRDY↓ → SystemTimer tick → blocking SPI RDATA (~25µs) → Sample в SPSC.
/// Та же ISR обрабатывает любой фронт keyphasor GPIO → точный tick → KpEvent в SPSC.
///
/// Оба события захватываются на Priority2 — никакая embassy-задача (Priority1, WiFi)
/// не может задержать ISR. Это даёт точность тика keyphasor на уровне ISR-latency
/// (~µs), а не embassy-scheduler-latency (~мс при активном WiFi).
///
/// # Synchronization
///
/// ISR_STATE: `Mutex<CriticalSectionRawMutex, RefCell<Option<IsrState>>>`.
/// Стандартный embedded-Rust паттерн (см. embassy/examples/interrupt.rs):
/// - `critical_section::with` отключает machine interrupts → exclusive access.
/// - `RefCell` даёт runtime borrow-checking (паники при двойном borrow нет,
///   т.к. ISR non-reentrant и async не может прервать ISR Priority2).
/// - ISR сам вызывает `critical_section::with` — на single-core RISC-V
///   повторное отключение прерываний из ISR — безопасный no-op.

use core::cell::RefCell;
use core::sync::atomic::{AtomicU16, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use esp_hal::gpio::{Event, Input, Output};
use esp_hal::ram;
use esp_hal::spi::master::SpiDma;
use esp_hal::timer::systimer::SystemTimer;
use heapless::spsc::Producer;
use vibro_protocol::{KpEvent, Sample};
use vibro_types::AdcCount;

use crate::ads1256::{CMD_RDATA, decode_i24, delay_us};
use crate::stream::KP_LEVEL;

// ---------------------------------------------------------------------------
// ISR-owned state
// ---------------------------------------------------------------------------

struct IsrState {
    spi:         SpiDma<'static, esp_hal::Blocking>,
    cs_pin:      Output<'static>,
    drdy:        Input<'static>,
    tx:          Producer<'static, Sample>,
    /// Keyphasor GPIO — любой фронт (rising + falling) для точного тика и KP_LEVEL.
    kp_pin:      Input<'static>,
    kp_producer: Producer<'static, KpEvent>,
}

static ISR_STATE: Mutex<CriticalSectionRawMutex, RefCell<Option<IsrState>>> =
    Mutex::new(RefCell::new(None));

/// Счётчик вызовов ISR — для диагностики потерь.
pub static ISR_COUNT: AtomicU16 = AtomicU16::new(0);
/// Счётчик неудачных enqueue (SPSC full).
pub static ISR_DROP_COUNT: AtomicU16 = AtomicU16::new(0);
/// Счётчик keyphasor-событий (rising edge) за секунду.
pub static KP_COUNT: AtomicU16 = AtomicU16::new(0);
/// Счётчик dropped KP-событий (SPSC full).
pub static KP_DROP_COUNT: AtomicU16 = AtomicU16::new(0);

// ---------------------------------------------------------------------------
// ISR handler
// ---------------------------------------------------------------------------

/// GPIO interrupt handler — вызывается аппаратно при любом GPIO-прерывании.
/// Priority2: выше WiFi/embassy (Priority1).
///
/// Обрабатывает два источника:
///   - DRDY↓ от ADS1256 → SPI RDATA → Sample в SPSC.
///   - Any edge от keyphasor GPIO → KpEvent в SPSC + KP_LEVEL update.
///
/// Tick для обоих захватывается в самом начале ISR — до SPI-задержек.
/// Это даёт точность keyphasor на уровне ISR entry latency (~µs).
#[esp_hal::handler(priority = esp_hal::interrupt::Priority::Priority2)]
#[ram]
fn drdy_isr() {
    // critical_section::with в ISR — на single-core RISC-V повторное отключение
    // прерываний безопасно. Весь ISR под одним cs — см. комментарий в оригинале.
    critical_section::with(|cs| {
        let mut guard = ISR_STATE.borrow(cs).borrow_mut();
        let Some(st) = guard.as_mut() else {
            return;
        };

        let drdy_fired = st.drdy.is_interrupt_set();
        let kp_fired   = st.kp_pin.is_interrupt_set();

        if !drdy_fired && !kp_fired {
            return;
        }

        // Захватываем tick немедленно — до любой обработки.
        // KP и DRDY используют один tick: они в одном ISR-вызове,
        // разница — единицы тактов (< 1 µs).
        let tick = SystemTimer::unit_value(esp_hal::timer::systimer::Unit::Unit0);

        // --- DRDY: читаем сэмпл ADS1256 ---
        if drdy_fired {
            st.drdy.clear_interrupt();

            // Blocking DMA RDATA: CS↓ → RDATA cmd → t6 delay → read 3 bytes → CS↑.
            // ~25µs при 1920kHz SPI clock.
            st.cs_pin.set_low();
            st.spi.write(&[CMD_RDATA]).unwrap();
            delay_us(7); // t6
            let mut buf = [0u8; 3];
            st.spi.read(&mut buf).unwrap();
            st.cs_pin.set_high();

            let ch0 = decode_i24(&buf);
            let flags = if KP_LEVEL.load(Ordering::Relaxed) {
                Sample::KEYPHASOR_LEVEL_FLAG
            } else {
                0
            };

            let sample = Sample {
                ch0: AdcCount(ch0),
                ch1: AdcCount(0),
                flags,
                tick,
            };

            ISR_COUNT.store(ISR_COUNT.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);

            if st.tx.enqueue(sample).is_err() {
                ISR_DROP_COUNT.store(ISR_DROP_COUNT.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
            }
        }

        // --- Keyphasor: любой фронт ---
        if kp_fired {
            st.kp_pin.clear_interrupt();
            let level = st.kp_pin.is_high();
            // Обновляем уровень — читается ISR при следующем DRDY для KEYPHASOR_LEVEL_FLAG.
            KP_LEVEL.store(level, Ordering::Relaxed);
            if level {
                // Rising edge — точный тик keyphasor.
                KP_COUNT.store(KP_COUNT.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
                if st.kp_producer.enqueue(KpEvent { tick }).is_err() {
                    KP_DROP_COUNT.store(KP_DROP_COUNT.load(Ordering::Relaxed).wrapping_add(1), Ordering::Relaxed);
                }
            }
        }
    });
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Передать периферию в ISR и включить прерывания DRDY и keyphasor.
/// Вызывать один раз из main после init ADS1256.
pub fn start(
    spi:         SpiDma<'static, esp_hal::Blocking>,
    cs_pin:      Output<'static>,
    drdy:        Input<'static>,
    tx:          Producer<'static, Sample>,
    kp_pin:      Input<'static>,
    kp_producer: Producer<'static, KpEvent>,
    io:          &mut esp_hal::gpio::Io<'_>,
) {
    io.set_interrupt_handler(drdy_isr);

    // Сначала state, потом listen — ISR при первом вызове гарантированно видит state = Some.
    // Обратный порядок (listen до state) → livelock: ISR на Priority2 при state=None не чистит
    // pending bit, re-triggers бесконечно, Priority1 код не может записать state.
    critical_section::with(|cs| {
        *ISR_STATE.borrow(cs).borrow_mut() = Some(IsrState { spi, cs_pin, drdy, tx, kp_pin, kp_producer });
    });

    // listen() после записи state — ISR сразу корректно обработает edge.
    critical_section::with(|cs| {
        let mut guard = ISR_STATE.borrow(cs).borrow_mut();
        let st = guard.as_mut().unwrap();
        st.drdy.listen(Event::FallingEdge);
        // AnyEdge: захватываем и rising (KpEvent) и falling (KP_LEVEL сброс).
        st.kp_pin.listen(Event::AnyEdge);
    });
}

/// Забрать периферию из ISR для reconfig.
/// critical_section отключает прерывания — ISR не может прервать take.
pub fn take_peripherals() -> (
    SpiDma<'static, esp_hal::Blocking>,
    Output<'static>,
    Input<'static>,
    Producer<'static, Sample>,
    Input<'static>,
    Producer<'static, KpEvent>,
) {
    esp_println::println!("take_peripherals: before cs");
    let st = critical_section::with(|cs| {
        let mut st = ISR_STATE.borrow(cs).borrow_mut().take().unwrap();
        // unlisten внутри cs — безопасно на single-core RISC-V.
        st.drdy.unlisten();
        st.kp_pin.unlisten();
        st
    });
    esp_println::println!("take_peripherals: done");
    (st.spi, st.cs_pin, st.drdy, st.tx, st.kp_pin, st.kp_producer)
}

/// Вернуть периферию в ISR после reconfig и снова включить прерывания.
pub fn give_back(
    spi:         SpiDma<'static, esp_hal::Blocking>,
    cs_pin:      Output<'static>,
    drdy:        Input<'static>,
    tx:          Producer<'static, Sample>,
    kp_pin:      Input<'static>,
    kp_producer: Producer<'static, KpEvent>,
) {
    let mut spi    = spi;
    let mut cs_pin = cs_pin;
    let mut drdy   = drdy;
    let mut kp_pin = kp_pin;

    // Если во время reconfig/skip DRDY уже успел упасть в LOW, то одного
    // `listen(FallingEdge)` недостаточно: нового falling edge может не быть,
    // пока мастер не заберёт pending sample. Прочитываем его вручную и
    // выбрасываем, чтобы вернуть ADS1256 к следующей конверсии.
    let drdy_low = drdy.is_low();
    esp_println::println!("give_back: DRDY={}", if drdy_low { "LOW" } else { "HIGH" });
    if drdy_low {
        esp_println::println!("give_back: dummy RDATA start");
        cs_pin.set_low();
        spi.write(&[CMD_RDATA]).unwrap();
        delay_us(7);
        let mut buf = [0u8; 3];
        spi.read(&mut buf).unwrap();
        cs_pin.set_high();
        esp_println::println!("give_back: dummy RDATA done");
    }

    // ВАЖНО: сначала записываем state, потом listen.
    // Если listen() до записи — ISR при state=None не чистит pending bit,
    // Priority2 ISR re-triggers бесконечно, livelock: give_back (Priority1) не может продвинуться.
    drdy.clear_interrupt();
    kp_pin.clear_interrupt();

    critical_section::with(|cs| {
        *ISR_STATE.borrow(cs).borrow_mut() = Some(IsrState { spi, cs_pin, drdy, tx, kp_pin, kp_producer });
    });

    // listen() включает прерывания — ISR теперь гарантированно видит state = Some.
    // Но drdy и kp_pin уже moved в ISR_STATE — нужно включать listen внутри CS.
    critical_section::with(|cs| {
        let mut guard = ISR_STATE.borrow(cs).borrow_mut();
        let st = guard.as_mut().unwrap();
        st.drdy.listen(Event::FallingEdge);
        st.kp_pin.listen(Event::AnyEdge);
    });
    esp_println::println!("give_back: ISR state restored, listen enabled");
}
