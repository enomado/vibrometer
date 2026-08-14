/// WiFi подключение и TCP стриминг сэмплов на receiver.

use core::sync::atomic::Ordering;

use embassy_net::{Runner, Stack, tcp::TcpSocket};
use embassy_time::{Duration, Timer};
use esp_println::println;
use esp_radio::wifi::{Interface, WifiController};

use heapless::spsc::Consumer;
use vibro_protocol::{
    Command,
    Frame,
    HEADER_SIZE,
    KpEvent,
    MAX_BATCH_SIZE,
    MAX_PACKED_BATCH_SIZE,
    PackedPacket,
    Sample,
};
use vibro_types::{PacketSeq, SampleRateHz};

use crate::stream::{CMD_SIGNAL, CURRENT_PGA, CURRENT_SAMPLE_RATE, RECONFIG_IN_PROGRESS};

include!(concat!(env!("OUT_DIR"), "/constants.rs"));

#[embassy_executor::task]
pub async fn connection(mut controller: WifiController<'static>) {
    loop {
        println!("wifi: connecting...");
        match controller.connect_async().await {
            Ok(info) => {
                println!("wifi: connected {:?}", info);
                let info = controller.wait_for_disconnect_async().await.ok();
                println!("wifi: disconnected {:?}", info);
            }
            Err(e) => {
                println!("wifi: error {:?}", e);
            }
        }
        Timer::after(Duration::from_millis(5000)).await;
    }
}

#[embassy_executor::task]
// esp-radio: `Interface` больше не параметризован лайфтаймом.
pub async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await;
}

/// Забрать из очереди до MAX_BATCH_SIZE сэмплов (lock-free).
fn drain_samples(
    rx: &mut Consumer<'static, Sample>,
    buf: &mut heapless::Vec<Sample, MAX_BATCH_SIZE>,
) {
    while !buf.is_full() {
        if let Some(s) = rx.dequeue() {
            buf.push(s).unwrap();
        } else {
            break;
        }
    }
}

fn encode_i24_be(value: i32, out: &mut [u8; 3]) {
    out[0] = ((value >> 16) & 0xFF) as u8;
    out[1] = ((value >> 8) & 0xFF) as u8;
    out[2] = (value & 0xFF) as u8;
}

fn pack_samples_ch0(
    samples: &[Sample],
    packed: &mut heapless::Vec<u8, MAX_PACKED_BATCH_SIZE>,
) -> u64 {
    packed.clear();
    let base_tick = samples.first().map(|s| s.tick).unwrap_or(0);
    let mut prev_tick = base_tick;

    for sample in samples {
        let mut ch0 = [0u8; 3];
        encode_i24_be(sample.ch0.as_i32(), &mut ch0);
        let dt = sample.tick.wrapping_sub(prev_tick);
        let dt = u16::try_from(dt).unwrap_or(u16::MAX);

        packed.extend_from_slice(&ch0).unwrap();
        packed.push(sample.flags).unwrap();
        packed.extend_from_slice(&dt.to_be_bytes()).unwrap();

        prev_tick = sample.tick;
    }

    base_tick
}

/// Попытаться прочитать и обработать одну команду из rx_buffer сокета.
///
/// Возвращает true если команда была обработана, false если данных нет.
/// Framing: [4 байта BE длина] + postcard payload.
///
/// Важно: `can_recv()` у embassy-net означает только "в буфере есть хоть какие-то
/// байты", а не "там уже лежит целый framed packet". Поэтому нельзя делать
/// `read_exact()` после `can_recv()` — если пришёл только кусок заголовка/тела,
/// stream_loop зависнет в ожидании остатка и перестанет отправлять data frames.
///
/// Здесь мы накапливаем байты в локальном parser-buffer и разбираем команду
/// только когда в буфере уже есть весь framed packet.
async fn try_recv_command(
    socket: &mut TcpSocket<'_>,
    buf: &mut [u8; 256],
    used: &mut usize,
) -> bool {
    if !socket.can_recv() {
        return false;
    }

    if *used >= buf.len() {
        *used = 0;
    }

    let free = buf.len() - *used;
    let read = match socket
        .read_with(|chunk| {
            let n = chunk.len().min(free);
            if n > 0 {
                buf[*used..*used + n].copy_from_slice(&chunk[..n]);
            }
            (n, n)
        })
        .await
    {
        Ok(n) => n,
        Err(_) => return false,
    };

    *used += read;

    if *used < HEADER_SIZE {
        return false;
    }

    let payload_len = u32::from_be_bytes(buf[..HEADER_SIZE].try_into().unwrap()) as usize;
    if payload_len == 0 || payload_len > buf.len() - HEADER_SIZE {
        esp_println::println!("cmd recv: invalid payload_len={}", payload_len);
        *used = 0;
        return false;
    }

    let frame_len = HEADER_SIZE + payload_len;
    if *used < frame_len {
        return false;
    }

    let cmd = match postcard::from_bytes::<Command>(&buf[HEADER_SIZE..frame_len]) {
        Ok(cmd) => {
            println!("cmd recv: {:?}", cmd);
            CMD_SIGNAL.signal(cmd);
        }
        Err(_) => {
            esp_println::println!("cmd recv: postcard decode failed");
            *used = 0;
            return false;
        }
    };

    let remaining = *used - frame_len;
    if remaining > 0 {
        buf.copy_within(frame_len..*used, 0);
    }
    *used = remaining;
    let _ = cmd;
    true
}

async fn write_all(socket: &mut TcpSocket<'_>, mut data: &[u8]) -> Result<(), embassy_net::tcp::Error> {
    while !data.is_empty() {
        let written = socket.write(data).await?;
        if written == 0 {
            return Err(embassy_net::tcp::Error::ConnectionReset);
        }
        data = &data[written..];
    }
    Ok(())
}

/// Сериализовать Frame в body_buf (с 4-байтным заголовком длины) и отправить.
/// Возвращает Err при ошибке TCP.
async fn send_frame(
    socket: &mut TcpSocket<'_>,
    frame: &Frame<'_>,
    body_buf: &mut [u8; 4096],
) -> Result<(), embassy_net::tcp::Error> {
    let payload = postcard::to_slice(frame, &mut body_buf[HEADER_SIZE..]).unwrap();
    let payload_len = payload.len();
    body_buf[..HEADER_SIZE].copy_from_slice(&(payload_len as u32).to_be_bytes());
    write_all(socket, &body_buf[..HEADER_SIZE + payload_len]).await
}

#[embassy_executor::task]
pub async fn stream_loop(
    stack: Stack<'static>,
    mut sample_rx: Consumer<'static, Sample>,
    mut kp_rx: Consumer<'static, KpEvent>,
) {
    let mut rx_buffer = [0; 512];
    let mut tx_buffer = [0; 4096];

    stack.wait_link_up().await;
    stack.wait_config_up().await;
    println!("net: link up");

    let remote = (SERVER_IP, SERVER_PORT);
    let mut seq: u32 = 0;
    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(Duration::from_secs(30)));

        println!("tcp: connecting to {}:{}", SERVER_IP, SERVER_PORT);
        if let Err(e) = socket.connect(remote).await {
            println!("tcp: connect error {:?}", e);
            Timer::after(Duration::from_millis(3000)).await;
            continue;
        }
        println!("tcp: connected");

        let mut cmd_buf = [0u8; 256];
        let mut cmd_buf_used = 0usize;
        let mut body_buf = [0u8; 4096];
        let mut packed_samples = heapless::Vec::<u8, MAX_PACKED_BATCH_SIZE>::new();

        'connected: loop {
            let had_cmd = try_recv_command(&mut socket, &mut cmd_buf, &mut cmd_buf_used).await;
            if had_cmd {
                esp_println::println!("stream_loop: cmd dispatched, yielding to adc_command_task");
                // Явный yield — даём adc_command_task выполниться до того, как
                // stream_loop уйдёт в send_frame и начнёт монополизировать TCP write.
                // Timer(0) = немедленный yield без реальной задержки.
                Timer::after(Duration::from_millis(0)).await;
            }
            while try_recv_command(&mut socket, &mut cmd_buf, &mut cmd_buf_used).await {}

            // Keyphasor-события — отправляем первыми (они привязаны ко времени).
            while let Some(kp) = kp_rx.dequeue() {
                let frame = Frame::Keyphasor(kp);
                if let Err(e) = send_frame(&mut socket, &frame, &mut body_buf).await {
                    println!("tcp: write error {:?}", e);
                    break 'connected;
                }
            }

            // ADC-сэмплы.
            let mut samples = heapless::Vec::<Sample, MAX_BATCH_SIZE>::new();
            drain_samples(&mut sample_rx, &mut samples);

            // Пока reconfig — выбрасываем сэмплы: они накопились до take_peripherals
            // и содержат данные со старым rate/PGA. ISR возобновится после give_back,
            // флаг будет снят, и следующие сэмплы уже будут с новыми настройками.
            //
            // ВАЖНО: обязательно yield (.await) перед continue — иначе embassy не может
            // переключиться на adc_command_task, и получается дедлок: stream_loop крутится
            // без .await, не давая выполниться задаче reconfig, которая должна снять флаг.
            if RECONFIG_IN_PROGRESS.load(Ordering::Acquire) {
                Timer::after(Duration::from_millis(1)).await;
                continue;
            }

            if samples.is_empty() {
                Timer::after(Duration::from_millis(10)).await;
                continue;
            }

            seq += 1;

            let base_tick = pack_samples_ch0(&samples, &mut packed_samples);

            let frame = Frame::DataPacked(PackedPacket {
                seq: PacketSeq(seq),
                sample_rate: SampleRateHz(CURRENT_SAMPLE_RATE.load(Ordering::Relaxed)),
                pga: CURRENT_PGA.load(Ordering::Relaxed),
                base_tick,
                samples: &packed_samples,
            });

            match send_frame(&mut socket, &frame, &mut body_buf).await {
                Ok(()) => {}
                Err(e) => {
                    println!("tcp: write error {:?}", e);
                    break 'connected;
                }
            }
        }

        Timer::after(Duration::from_millis(3000)).await;
    }
}
