/// BLE GATT-сервер для vibrometer.
///
/// Поднимает peripheral с именем "Vibrometer", advertises один сервис
/// с характеристикой status (read/notify). Пока просто шлёт счётчик
/// каждые 2 секунды — заглушка для проверки связи с Android.

use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_time::Timer;
use trouble_host::prelude::*;

const CONNECTIONS_MAX: usize = 1;
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + ATT

/// Кастомный UUID для vibrometer-сервиса.
#[gatt_server]
struct Server {
    vibrometer_service: VibrometerService,
}

#[gatt_service(uuid = "12345678-1234-5678-1234-56789abcdef0")]
struct VibrometerService {
    /// Текущий статус/счётчик — read + notify
    #[characteristic(uuid = "12345678-1234-5678-1234-56789abcdef1", read, notify)]
    status: u8,
}

/// Точка входа BLE-подсистемы. Блокирует навсегда (advertise loop).
pub async fn run<C: Controller>(controller: C) {
    // Фиксированный random address — для отладки (всегда один и тот же MAC).
    let address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xaa]);

    // trouble-host: HostResources больше не параметризован контроллером —
    // первый параметр теперь сам PacketPool.
    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources)
        .set_random_address(address)
        .build();
    let runner = stack.runner();
    let mut peripheral = stack.peripheral();

    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Vibrometer",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    esp_println::println!("BLE: GATT server started, advertising as 'Vibrometer'");

    // runner — фоновый BLE-стек, advertise loop — логика подключений.
    let _ = join(ble_runner(runner), async {
        loop {
            match advertise(&mut peripheral, &server).await {
                Ok(conn) => {
                    esp_println::println!("BLE: client connected");
                    let a = gatt_events(&server, &conn);
                    let b = notify_loop(&server, &conn);
                    select(a, b).await;
                    esp_println::println!("BLE: client disconnected");
                }
                Err(_e) => {
                    esp_println::println!("BLE: advertise error");
                }
            }
        }
    })
    .await;
}

async fn ble_runner<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(_e) = runner.run().await {
            esp_println::println!("BLE: runner error");
        }
    }
}

async fn advertise<'v, 's, C: Controller>(
    peripheral: &mut Peripheral<'v, C, DefaultPacketPool>,
    server: &'s Server<'v>,
) -> Result<GattConnection<'v, 's, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut adv_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::CompleteLocalName(b"Vibrometer"),
        ],
        &mut adv_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &adv_data[..len],
                scan_data: &[],
            },
        )
        .await?;
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    Ok(conn)
}

/// Обрабатывает GATT read/write events до отключения.
async fn gatt_events<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { .. } => return Ok(()),
            GattConnectionEvent::Gatt { event } => {
                // accept() отправляет ответ на read/write request
                match event.accept() {
                    Ok(reply) => reply.send().await,
                    Err(_) => {}
                };
            }
            _ => {}
        }
    }
}

/// Шлёт notify со счётчиком каждые 2 секунды — заглушка.
async fn notify_loop<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) {
    let status = server.vibrometer_service.status;
    let mut tick: u8 = 0;
    loop {
        tick = tick.wrapping_add(1);
        // Третий аргумент `store` появился в новом trouble-host; раньше значение
        // писалось в attribute table всегда → передаём true, чтобы сохранить поведение.
        if status.notify(conn, &tick, true).await.is_err() {
            break;
        }
        Timer::after_secs(2).await;
    }
}
