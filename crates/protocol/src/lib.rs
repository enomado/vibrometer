#![no_std]

use serde::{
    Deserialize,
    Serialize,
};
use vibro_types::{
    AdcCount,
    PacketSeq,
    SampleRateHz,
};

pub const PACKED_CH0_SAMPLE_SIZE: usize = 6;

/// Один сэмпл с АЦП.
/// ADS1256 24-бит, дифф. режим: 2 канала velocity transducers.
/// Keyphasor — бит-флаг (фронт в этом сэмпле).
/// tick — SystemTimer (16 МГц) момент DRDY↓ (реальный момент конверсии).
/// Единые часы с KpEvent.tick → нулевой фазный дрейф между каналами.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct Sample {
    /// Канал 0 (AIN0-AIN1), дифф., 24-бит знаковый.
    /// Сырое значение АЦП, без преобразования в физ. единицы.
    pub ch0:   AdcCount,
    /// Канал 1 (AIN2-AIN3), дифф., 24-бит знаковый.
    pub ch1:   AdcCount,
    /// bit 0: keyphasor фронт обнаружен в этом сэмпле
    pub flags: u8,
    /// SystemTimer tick момента DRDY↓ (62.5 нс, абсолютный от старта МК).
    /// Тот же клок что KpEvent.tick — фазный дрейф между ADC и keyphasor = 0.
    pub tick:  u64,
}

impl Sample {
    /// Нарастающий фронт keyphasor в этом сэмпле.
    pub const KEYPHASOR_FLAG: u8 = 0x01;
    /// Текущий уровень keyphasor GPIO (high = метка под датчиком).
    pub const KEYPHASOR_LEVEL_FLAG: u8 = 0x02;

    pub fn keyphasor(&self) -> bool {
        self.flags & Self::KEYPHASOR_FLAG != 0
    }

    pub fn keyphasor_level(&self) -> bool {
        self.flags & Self::KEYPHASOR_LEVEL_FLAG != 0
    }
}

/// Keyphasor-событие: нарастающий фронт с точным аппаратным timestamp.
/// tick — абсолютное значение SystemTimer (16 МГц, 62.5 нс) от старта МК.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct KpEvent {
    pub tick: u64,
}

/// Пакет для сериализации (firmware → wire).
/// Использует &[Sample] — zero-copy на стороне отправки.
#[derive(Debug, Clone, Serialize)]
pub struct Packet<'a> {
    /// Порядковый номер пакета (монотонно растёт)
    pub seq:         PacketSeq,
    /// Частота дискретизации АЦП (Гц)
    pub sample_rate: SampleRateHz,
    /// Текущее усиление PGA (1, 2, 4, 8, 16, 32, 64).
    /// Нужно для нормализации: raw_LSB / pga → LSB при PGA=1.
    pub pga:         u8,
    /// Сэмплы в этом пакете (каждый несёт свой tick)
    pub samples:     &'a [Sample],
}

/// Плотный one-channel пакет для high-rate streaming.
/// На проводе каждый sample кодируется как:
/// - ch0: 24-bit signed big-endian
/// - flags: u8
/// - dt: u16 big-endian (delta tick from previous sample; first sample = 0)
#[derive(Debug, Clone, Serialize)]
pub struct PackedPacket<'a> {
    pub seq:         PacketSeq,
    pub sample_rate: SampleRateHz,
    pub pga:         u8,
    pub base_tick:   u64,
    pub samples:     &'a [u8],
}

/// Пакет для десериализации (wire → receiver).
/// Owned-версия: Vec вместо &[] — нужен alloc.
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, Deserialize)]
pub struct PacketOwned {
    pub seq:         PacketSeq,
    pub sample_rate: SampleRateHz,
    pub pga:         u8,
    pub samples:     alloc::vec::Vec<Sample>,
}

#[cfg(feature = "alloc")]
#[derive(Debug, Clone, Deserialize)]
pub struct PackedPacketOwned {
    pub seq:         PacketSeq,
    pub sample_rate: SampleRateHz,
    pub pga:         u8,
    pub base_tick:   u64,
    pub samples:     alloc::vec::Vec<u8>,
}

/// Фрейм на проводе firmware → receiver.
/// Framing: [4 байта BE длина] + postcard payload.
/// Тег enum кодируется postcard (первый байт = variant index).
#[derive(Debug, Clone, Serialize)]
pub enum Frame<'a> {
    /// Батч ADC-сэмплов.
    Data(Packet<'a>),
    /// Плотный батч one-channel ADC-сэмплов.
    DataPacked(PackedPacket<'a>),
    /// Keyphasor-фронт с точным аппаратным timestamp.
    Keyphasor(KpEvent),
}

/// Owned-версия Frame для десериализации на receiver.
#[cfg(feature = "alloc")]
#[derive(Debug, Clone, Deserialize)]
pub enum FrameOwned {
    Data(PacketOwned),
    DataPacked(PackedPacketOwned),
    Keyphasor(KpEvent),
}

#[cfg(feature = "alloc")]
extern crate alloc;

/// Команда от receiver → firmware.
///
/// Framing: тот же формат что и Packet — [4 байта BE длина] + postcard payload.
/// Firmware читает из rx_buffer того же TCP-сокета.
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum Command {
    /// Установить PGA (усиление) ADS1256.
    /// Допустимые значения: 1, 2, 4, 8, 16, 32, 64.
    SetPga(u8),
    /// Установить частоту дискретизации АЦП (SPS).
    /// Допустимые значения: 30000, 15000, 7500, 3750, 2000, 1000, 500, 100, 60, 50, 30, 25, 15, 10, 5.
    SetDataRate(SampleRateHz),
}

/// Формат на проводе: [4 байта BE длина payload] + [postcard payload]
pub const HEADER_SIZE: usize = 4;

/// Максимальный размер пакета (сэмплов).
/// 250 сэмплов @ 15 кГц = ~16.7 мс батч.
/// Более крупный батч заметно снижает packets/sec и TCP/postcard overhead
/// на ESP32-C3 при высоких sample rate.
pub const MAX_BATCH_SIZE: usize = 250;
pub const MAX_PACKED_BATCH_SIZE: usize = MAX_BATCH_SIZE * PACKED_CH0_SAMPLE_SIZE;
