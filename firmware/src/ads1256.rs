/// Драйвер ADS1256 (24-бит sigma-delta ADC, SPI).
///
/// SPI master работает через DMA, а `DRDY` захватывается GPIO interrupt.
/// Последовательность команд остаётся blocking: write/read завершаются до
/// возврата, поэтому тайминги CS и t6 остаются явными и локальными.
///
/// Протокол ADS1256:
/// - Команды: WREG, RREG, RDATA, RDATAC, SYNC, WAKEUP, MUX select
/// - DRDY пин: low когда данные готовы
/// - После переключения MUX нужен SYNC + WAKEUP + ожидание DRDY

use esp_hal::timer::systimer::SystemTimer;
use esp_hal::gpio::{Input, Output};
use esp_hal::spi::master::SpiDma;

// Регистры ADS1256
const REG_STATUS: u8 = 0x00;
const REG_MUX: u8 = 0x01;
const REG_ADCON: u8 = 0x02;
const REG_DRATE: u8 = 0x03;

// Команды
const CMD_RREG: u8 = 0x10;  // Read register:  0x10 | reg, 0x00
const CMD_WREG: u8 = 0x50;  // Write register: 0x50 | reg, 0x00, data
pub(crate) const CMD_RDATA: u8 = 0x01; // Read data once
const CMD_SDATAC: u8 = 0x0F; // Stop read data continuously
const CMD_SYNC: u8 = 0xFC;  // Sync
const CMD_WAKEUP: u8 = 0x00; // Wakeup (also: 0xFF)
const CMD_SELFCAL: u8 = 0xF0;
const CMD_RESET: u8 = 0xFE;

// MUX: psel[7:4] | nsel[3:0]
const MUX_DIFF_CH0: u8 = 0x01;
const MUX_DIFF_CH1: u8 = 0x23;

const STATUS_DEFAULT: u8 = 0x00;
const ADCON_DEFAULT: u8 = 0x00;
const DRATE_DEFAULT: u8 = 0x92;

fn encode_pga(pga: u8) -> u8 {
    match pga {
        1 => 0,
        2 => 1,
        4 => 2,
        8 => 3,
        16 => 4,
        32 => 5,
        64 => 6,
        _ => 0,
    }
}

fn drate_from_sps(sps: u16) -> Option<u8> {
    match sps {
        30000 => Some(0xF0),
        15000 => Some(0xE0),
        7500 => Some(0xD0),
        3750 => Some(0xC0),
        2000 => Some(0xB0),
        1000 => Some(0xA1),
        500 => Some(0x92),
        100 => Some(0x82),
        60 => Some(0x72),
        50 => Some(0x63),
        30 => Some(0x53),
        25 => Some(0x43),
        15 => Some(0x33),
        10 => Some(0x20),
        5 => Some(0x13),
        _ => None,
    }
}

/// Декодировать 24-бит two's complement из 3 байт MSB-first.
pub(crate) fn decode_i24(buf: &[u8; 3]) -> i32 {
    let raw = ((buf[0] as u32) << 16) | ((buf[1] as u32) << 8) | (buf[2] as u32);
    if raw & 0x800000 != 0 {
        (raw | 0xFF000000) as i32
    } else {
        raw as i32
    }
}

/// Задержка через busy-wait. Для коротких пауз (t6 и т.п.) внутри SPI транзакции,
/// где async Timer неприемлем — нельзя отдавать executor между CS↓ и CS↑.
pub(crate) fn delay_us(us: u32) {
    let start = esp_hal::time::Instant::now();
    let target = esp_hal::time::Duration::from_micros(us as u64);
    while start.elapsed() < target {}
}

pub struct Ads1256<'a> {
    spi: SpiDma<'a, esp_hal::Blocking>,
    cs: Output<'a>,
    drdy: Input<'a>,
}

impl<'a> Ads1256<'a> {
    pub fn new(spi: SpiDma<'a, esp_hal::Blocking>, cs: Output<'a>, drdy: Input<'a>) -> Self {
        Self { spi, cs, drdy }
    }

    /// Гарантировать command mode после передачи SPI из ISR.
    /// ISR использует RDATA (single read), но если последний read не завершился
    /// полностью (SPI FIFO residual) — чип может быть в data output state.
    /// SDATAC безопасен: если уже в command mode — no-op.
    pub fn ensure_command_mode(&mut self) {
        self.cs_low();
        // Если чип отдаёт данные — забираем 3 байта (24-бит conversion).
        let mut dummy = [0u8; 3];
        self.spi_read(&mut dummy);
        // Затем SDATAC — переводит в command mode.
        self.spi_write(&[CMD_SDATAC]);
        self.cs_high();
        delay_us(10);
    }

    /// Разобрать на составные части (для передачи в ISR после init).
    pub fn into_parts(self) -> (SpiDma<'a, esp_hal::Blocking>, Output<'a>, Input<'a>) {
        (self.spi, self.cs, self.drdy)
    }

    fn cs_low(&mut self) {
        self.cs.set_low();
    }

    fn cs_high(&mut self) {
        self.cs.set_high();
    }

    /// Blocking DMA write: функция возвращается только после завершения transfer.
    fn spi_write(&mut self, data: &[u8]) {
        self.spi.write(data).unwrap();
    }

    /// Blocking DMA read: функция возвращается только после завершения transfer.
    fn spi_read(&mut self, buf: &mut [u8]) {
        self.spi.read(buf).unwrap();
    }

    fn send_command(&mut self, cmd: u8) {
        self.cs_low();
        self.spi_write(&[cmd]);
        self.cs_high();
    }

    fn wait_drdy_low(
        &mut self,
        timeout_ms: u32,
        context: &str,
    ) -> Option<u64> {
        let initial_state = if self.drdy.is_high() { "HIGH" } else { "LOW" };
        esp_println::println!("ADS1256: wait_drdy_low({}) start, DRDY={}", context, initial_state);
        let start = esp_hal::time::Instant::now();
        let timeout = esp_hal::time::Duration::from_millis(timeout_ms as u64);
        while self.drdy.is_high() {
            if start.elapsed() > timeout {
                esp_println::println!("ADS1256: DRDY timeout during {}", context);
                return None;
            }
        }
        let elapsed_us = start.elapsed().as_micros();
        esp_println::println!("ADS1256: DRDY went LOW in {}µs ({})", elapsed_us, context);
        Some(SystemTimer::unit_value(esp_hal::timer::systimer::Unit::Unit0))
    }

    fn recover_from_drdy_timeout(&mut self) -> bool {
        esp_println::println!("ADS1256: DRDY timeout (poll), resetting...");

        self.send_command(CMD_RESET);
        delay_us(5000);

        if self.wait_drdy_low(200, "RESET").is_none() {
            esp_println::println!("ADS1256: reset did not bring DRDY low");
            return false;
        }

        // После RESET чип в RDATAC mode. Если DRDY уже low, забираем
        // текущие 24 бита и только потом переводим в SDATAC.
        self.cs_low();
        let mut dummy = [0u8; 3];
        self.spi_read(&mut dummy);
        self.spi_write(&[CMD_SDATAC]);
        self.cs_high();
        delay_us(10);

        self.write_reg(REG_STATUS, STATUS_DEFAULT);
        self.write_reg(REG_ADCON, ADCON_DEFAULT);
        self.write_reg(REG_DRATE, DRATE_DEFAULT);
        self.write_reg(REG_MUX, MUX_DIFF_CH0);

        self.recalibrate_sync();
        if self.wait_drdy_low(200, "SELFCAL after RESET").is_none() {
            esp_println::println!("ADS1256: SELFCAL after reset did not complete");
            return false;
        }

        esp_println::println!("ADS1256: reset done");
        true
    }

    /// Ждём DRDY ↓ (данные готовы). Polling — не зависит от async GPIO.
    /// Используется только при init/reconfig когда ISR выключен.
    /// Возвращает SystemTimer tick момента DRDY↓.
    fn wait_drdy_poll(&mut self) -> Option<u64> {
        if let Some(tick) = self.wait_drdy_low(200, "poll") {
            return Some(tick);
        }

        if !self.recover_from_drdy_timeout() {
            return None;
        }

        self.wait_drdy_low(200, "post-reset poll")
    }

    fn recalibrate_sync(&mut self) {
        self.send_command(CMD_SELFCAL);
    }

    fn recalibrate(&mut self) -> bool {
        esp_println::println!("ADS1256: SELFCAL start, DRDY={}", if self.drdy.is_high() { "HIGH" } else { "LOW" });
        self.recalibrate_sync();
        if self.wait_drdy_poll().is_none() {
            esp_println::println!("ADS1256: SELFCAL failed");
            return false;
        }
        esp_println::println!("ADS1256: SELFCAL done");
        true
    }

    /// Записать один регистр.
    fn write_reg(&mut self, reg: u8, value: u8) {
        self.cs_low();
        self.spi_write(&[CMD_WREG | reg, 0x00, value]);
        self.cs_high();
        delay_us(10);
    }

    /// Прочитать один регистр.
    fn read_reg(&mut self, reg: u8) -> u8 {
        self.cs_low();
        self.spi_write(&[CMD_RREG | reg, 0x00]);
        delay_us(10); // t6 задержка перед чтением
        let mut data = [0u8; 1];
        self.spi_read(&mut data);
        self.cs_high();
        data[0]
    }

    /// Записать регистр и проверить readback с ретраями.
    ///
    /// SPI MISO (DOUT ADS1256) подвержен шуму от WiFi EMI при 1920kHz
    /// (максимальная частота SCLK для 7.68MHz CLKIN). MOSI чист (ESP32
    /// strong driver), поэтому WREG почти всегда успешен — но RREG readback
    /// может вернуть мусор. Повторяем readback до совпадения.
    /// Каждые 4 попытки перезаписываем регистр на случай если WREG тоже
    /// попал под помеху.
    fn write_verify_reg(&mut self, reg: u8, value: u8) -> bool {
        const MAX_ATTEMPTS: u8 = 12;
        self.write_reg(reg, value);
        for attempt in 0..MAX_ATTEMPTS {
            let actual = self.read_reg(reg);
            if actual == value {
                if attempt > 0 {
                    esp_println::println!(
                        "ADS1256: reg 0x{:02x} verify ok on attempt {}",
                        reg, attempt + 1
                    );
                }
                return true;
            }
            esp_println::println!(
                "ADS1256: reg 0x{:02x} verify mismatch (wrote=0x{:02x} read=0x{:02x}), retry {}",
                reg, value, actual, attempt + 1
            );
            if attempt % 4 == 3 {
                // перезапись регистра на случай если и WREG был повреждён
                self.write_reg(reg, value);
            }
            delay_us(50);
        }
        esp_println::println!(
            "ADS1256 ERROR: reg 0x{:02x} write failed after {} attempts",
            reg, MAX_ATTEMPTS
        );
        false
    }

    /// Прочитать ключевые регистры для отладки.
    pub fn debug_registers(&mut self) -> (u8, u8, u8, u8) {
        let status = self.read_reg(REG_STATUS);
        let mux = self.read_reg(REG_MUX);
        let adcon = self.read_reg(REG_ADCON);
        let drate = self.read_reg(REG_DRATE);
        (status, mux, adcon, drate)
    }

    /// Инициализация: сброс, настройка регистров, автокалибровка.
    /// Полностью blocking (polling DRDY) — не зависит от async GPIO.
    pub fn init(&mut self) -> bool {
        self.cs_high();
        delay_us(5000);
        if self.wait_drdy_poll().is_none() {
            return false;
        }

        self.send_command(CMD_RESET);
        delay_us(5000);
        if self.wait_drdy_poll().is_none() {
            return false;
        }

        // После RESET чип в RDATAC mode. При DRDY↓ он ожидает что мастер
        // заберёт 24 бита данных. SDATAC во время data output игнорируется.
        // Поэтому: сначала dummy read (забрать данные), потом SDATAC.
        self.cs_low();
        let mut dummy = [0u8; 3];
        self.spi_read(&mut dummy);
        self.spi_write(&[CMD_SDATAC]);
        self.cs_high();
        delay_us(10);

        self.write_reg(REG_STATUS, STATUS_DEFAULT);
        self.write_reg(REG_ADCON, ADCON_DEFAULT);
        self.write_reg(REG_DRATE, DRATE_DEFAULT);
        self.write_reg(REG_MUX, MUX_DIFF_CH0);

        self.recalibrate()
    }

    /// Инициализация для одного дифференциального канала.
    pub fn init_single_channel(&mut self, mux: u8) -> bool {
        if !self.init() {
            return false;
        }
        self.write_reg(REG_MUX, mux);
        if !self.recalibrate() {
            return false;
        }
        // Drain settling: 3 сэмпла переходного процесса sinc³ фильтра.
        for _ in 0..3 {
            if self.read_single_channel().is_none() {
                return false;
            }
        }
        true
    }

    /// Переключить MUX на канал и дождаться готовности.
    fn select_channel(&mut self, mux: u8) {
        self.write_reg(REG_MUX, mux);
        self.send_command(CMD_SYNC);
        delay_us(4);
        self.send_command(CMD_WAKEUP);
        let _ = self.wait_drdy_poll();
    }

    /// DRDY↓ → CS↓ → RDATA → t6 → read 3 bytes → CS↑.
    /// Возвращает (value, tick). tick — момент DRDY↓.
    pub fn read_single_channel(&mut self) -> Option<(i32, u64)> {
        let tick = self.wait_drdy_poll()?;
        self.cs_low();
        self.spi_write(&[CMD_RDATA]);
        delay_us(7); // t6
        let mut buf = [0u8; 3];
        self.spi_read(&mut buf);
        self.cs_high();
        Some((decode_i24(&buf), tick))
    }

    pub fn set_pga(&mut self, pga: u8) -> bool {
        let pga_bits = encode_pga(pga);
        esp_println::println!("ADS1256: set PGA={}", pga);
        if !self.write_verify_reg(REG_ADCON, pga_bits) {
            return false;
        }
        if !self.recalibrate() {
            return false;
        }
        esp_println::println!("ADS1256: draining 3 samples after PGA change...");
        for i in 0..3u8 {
            esp_println::println!("ADS1256: drain sample {}/3, DRDY={}", i + 1,
                if self.drdy.is_high() { "HIGH" } else { "LOW" });
            if self.read_single_channel().is_none() {
                esp_println::println!("ADS1256: drain after PGA change failed at sample {}/3", i + 1);
                return false;
            }
            esp_println::println!("ADS1256: drain sample {}/3 done", i + 1);
        }
        esp_println::println!("ADS1256: set_pga({}) done", pga);
        true
    }

    pub fn set_data_rate(&mut self, sps: u16) -> bool {
        let Some(drate) = drate_from_sps(sps) else {
            esp_println::println!("ADS1256: unknown SPS {}", sps);
            return false;
        };
        esp_println::println!("ADS1256: set sample rate={} SPS", sps);
        if !self.write_verify_reg(REG_DRATE, drate) {
            return false;
        }
        if !self.recalibrate() {
            return false;
        }
        esp_println::println!("ADS1256: draining 3 samples...");
        for _ in 0..3 {
            if self.read_single_channel().is_none() {
                esp_println::println!("ADS1256: drain after data rate change failed");
                return false;
            }
        }
        esp_println::println!("ADS1256: set_data_rate done");
        true
    }

    /// Прочитать оба дифференциальных канала (с переключением MUX).
    pub fn read_both_channels(&mut self) -> (i32, i32) {
        self.select_channel(MUX_DIFF_CH0);
        let ch0 = self.read_data();

        self.select_channel(MUX_DIFF_CH1);
        let ch1 = self.read_data();

        (ch0, ch1)
    }

    /// Прочитать 24-бит результат (RDATA, для read_both_channels).
    fn read_data(&mut self) -> i32 {
        self.cs_low();
        self.spi_write(&[CMD_RDATA]);
        delay_us(7);
        let mut buf = [0u8; 3];
        self.spi_read(&mut buf);
        self.cs_high();
        decode_i24(&buf)
    }
}
