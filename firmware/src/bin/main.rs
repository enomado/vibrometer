#![no_std]
#![no_main]

use esp_hal::clock::CpuClock;
use esp_hal::{dma_rx_buffer, dma_tx_buffer};
use esp_hal::gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull};
use esp_hal::interrupt::software::SoftwareInterruptControl;
use esp_hal::rng::Rng;
use esp_hal::spi::{
    Mode,
    master::{Config as SpiConfig, Spi},
};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;

use embassy_executor::Spawner;
use embassy_net::StackResources;
use embassy_time::{Duration, Timer};

use esp_println as _;
use esp_backtrace as _;
use esp_alloc as _;
use esp_rtos as _;

use esp_radio::wifi::{Config, ControllerConfig, sta::StationConfig};
#[allow(unused_imports)]
use esp_radio::ble::controller::BleConnector;
#[allow(unused_imports)]
use trouble_host::prelude::*;

esp_bootloader_esp_idf::esp_app_desc!();

use esp_hal::gpio::Io;

use vibrometer_fw::mk_static;
use vibrometer_fw::ads1256::Ads1256;
#[allow(unused_imports)]
use vibrometer_fw::ble;
use vibrometer_fw::isr_capture;
use vibrometer_fw::network::{connection, net_task, stream_loop};
use vibrometer_fw::stream::{CURRENT_PGA, CURRENT_SAMPLE_RATE, adc_command_task, init_queues, isr_stats_task};

include!(concat!(env!("OUT_DIR"), "/constants.rs"));

#[allow(dead_code)]
#[embassy_executor::task]
async fn ble_task(controller: ExternalController<BleConnector<'static>, 1>) -> ! {
    ble::run(controller).await;
    unreachable!()
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::default());
    let peripherals = esp_hal::init(config);

    // WiFi + BLE coex: внутренние задачи esp-radio аллоцируют стеки из heap.
    // 72 KB мало для WiFi+BLE, 116 KB съедает main stack. 96 KB — компромисс.
    esp_alloc::heap_allocator!(size: 96 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, software_interrupt.software_interrupt0);

    let rng = Rng::new();

    // --- WiFi ---
    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(WIFI_SSID)
            .with_password(WIFI_PASSWD.into()),
    );

    // esp-radio: контроллер и интерфейсы теперь создаются раздельно —
    // `wifi::new()` больше нет, вместо него WifiController::new + Interface::station().
    let controller = esp_radio::wifi::WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();

    let wifi_interface = esp_radio::wifi::Interface::station();

    let config = embassy_net::Config::dhcpv4(Default::default());
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        config,
        mk_static!(StackResources<7>, StackResources::<7>::new()),
        seed,
    );

    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(connection(controller).unwrap());

    // --- BLE (disabled: проверка влияния на ISR jitter при 7500 SPS) ---
    // let connector = BleConnector::new(peripherals.BT, Default::default()).unwrap();
    // let ble_controller: ExternalController<_, 1> = ExternalController::new(connector);
    // spawner.spawn(ble_task(ble_controller).unwrap());

    // --- ADS1256 ---
    // SPI2 + DMA: DRDY остаётся в Priority2 ISR, SPI transfers завершаются blocking.
    let spi = Spi::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_khz(1920))
            .with_mode(Mode::_1),
    )
    .unwrap()
    .with_sck(peripherals.GPIO4)
    .with_mosi(peripherals.GPIO6)
    .with_miso(peripherals.GPIO5)
    .with_dma(peripherals.DMA_CH0);

    // esp-hal: DmaRxBuf/DmaTxBuf теперь требуют выровненные буферы (DmaAlignedMut),
    // так что собираем их макросами dma_{rx,tx}_buffer! вместо ручного dma_buffers!.
    let dma_rx_buf = dma_rx_buffer!(4).unwrap();
    let dma_tx_buf = dma_tx_buffer!(4).unwrap();
    let spi = spi.with_buffers(dma_rx_buf, dma_tx_buf);

    let cs = Output::new(peripherals.GPIO7, Level::High, OutputConfig::default());
    let drdy = Input::new(
        peripherals.GPIO10,
        InputConfig::default().with_pull(Pull::None),
    );
    let keyphasor = Input::new(
        peripherals.GPIO1,
        InputConfig::default().with_pull(Pull::None),
    );

    esp_println::println!(
        "ADS1256 pins: IO4=SCLK IO6=DIN IO5=DOUT IO7=CS IO10=DRDY PDWN=3V3 | keyphasor: IO1=TCRT5000 D0"
    );

    // Init ADS1256 (async — нужен wait_drdy для калибровки).
    let mut adc = Ads1256::new(spi, cs, drdy);
    CURRENT_SAMPLE_RATE.store(500, core::sync::atomic::Ordering::Relaxed);
    CURRENT_PGA.store(1, core::sync::atomic::Ordering::Relaxed);
    esp_println::println!("ADS1256: init diff AIN0-AIN1...");
    if !adc.init_single_channel(0x01) {
        panic!("ADS1256 init failed");
    }

    let (status, mux, adcon, drate) = adc.debug_registers();
    esp_println::println!(
        "ADS1256: init ok STATUS=0x{:02x} MUX=0x{:02x} ADCON=0x{:02x} DRATE=0x{:02x}",
        status, mux, adcon, drate
    );

    // Передать периферию в ISR — захват по DRDY↓ interrupt.
    let (spi, cs, drdy) = adc.into_parts();
    let (sample_tx, sample_rx, kp_tx, kp_rx) = init_queues();

    let mut io = Io::new(peripherals.IO_MUX);
    isr_capture::start(spi, cs, drdy, sample_tx, keyphasor, kp_tx, &mut io);
    esp_println::println!("ADS1256: ISR capture started");

    // --- Запуск задач ---
    spawner.spawn(adc_command_task().unwrap());
    spawner.spawn(isr_stats_task().unwrap());
    spawner.spawn(stream_loop(stack, sample_rx, kp_rx).unwrap());

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}
