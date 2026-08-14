use std::io::{
    Read,
    Write,
};
use std::net::{
    TcpListener,
    TcpStream,
};
use std::sync::{
    Arc,
    Mutex,
};
use std::time::Instant;

use eframe::egui;
use vibro_protocol::{
    Command,
    FrameOwned,
    HEADER_SIZE,
    MAX_BATCH_SIZE,
    PACKED_CH0_SAMPLE_SIZE,
    Sample,
};
use vibro_types::SampleRateHz;
use vibro_types::AdcCount;

use crate::recording::{
    AUTO_REC_IDLE_TIMEOUT,
    ConnStatus,
    LIVE_BUF_SIZE,
    SYSTIMER_HZ,
    Shared,
    finish_recording,
    process_keyphasor,
    process_recording_sample,
};

/// Максимальный допустимый payload (байт).
/// postcard varint: Sample ≈ 12 байт макс, + seq(5) + sample_rate(3) + vec_len(3) → ~1200+11.
/// Берём с запасом.
const MAX_PAYLOAD: usize = MAX_BATCH_SIZE * 3 * size_of::<Sample>() + 64;
const DIAG_PERIOD: std::time::Duration = std::time::Duration::from_secs(1);
const REPAINT_PERIOD: std::time::Duration = std::time::Duration::from_millis(33);

struct SessionDiag {
    started_at: Instant,
    last_print_at: Instant,
    last_repaint_at: Instant,
    packets: u64,
    samples: u64,
    keyphasors: u64,
    seq_gaps: u64,
}

impl SessionDiag {
    fn new(now: Instant) -> Self {
        Self {
            started_at: now,
            last_print_at: now,
            last_repaint_at: now,
            packets: 0,
            samples: 0,
            keyphasors: 0,
            seq_gaps: 0,
        }
    }

    fn note_packet(&mut self, sample_count: usize) {
        self.packets += 1;
        self.samples += sample_count as u64;
    }

    fn note_keyphasor(&mut self) {
        self.keyphasors += 1;
    }

    fn note_seq_gap(&mut self, gap: u32) {
        self.seq_gaps += gap as u64;
    }

    fn maybe_print(&mut self, peer: &str) {
        let now = Instant::now();
        let dt = now.duration_since(self.last_print_at);
        if dt < DIAG_PERIOD {
            return;
        }

        let dt_s = dt.as_secs_f64().max(1e-6);
        let uptime_s = now.duration_since(self.started_at).as_secs_f64();
        println!(
            "tcp {peer}: {:.1}s uptime | {:.1} pkt/s | {:.0} samp/s | {:.1} kp/s | seq_gaps={}",
            uptime_s,
            self.packets as f64 / dt_s,
            self.samples as f64 / dt_s,
            self.keyphasors as f64 / dt_s,
            self.seq_gaps,
        );

        self.last_print_at = now;
        self.packets = 0;
        self.samples = 0;
        self.keyphasors = 0;
    }

    fn maybe_repaint(&mut self, ctx: &egui::Context) {
        let now = Instant::now();
        if now.duration_since(self.last_repaint_at) >= REPAINT_PERIOD {
            ctx.request_repaint();
            self.last_repaint_at = now;
        }
    }
}

fn read_frame(
    stream: &mut TcpStream,
    recv_buf: &mut Vec<u8>,
) -> anyhow::Result<FrameOwned> {
    let mut len_buf = [0u8; HEADER_SIZE];
    stream.read_exact(&mut len_buf)?;
    let payload_len = u32::from_be_bytes(len_buf) as usize;

    // Защита от рассинхронизации фрейминга: если payload_len явно мусор,
    // не аллоцируем гигабайт, а сразу обрываем соединение.
    if payload_len > MAX_PAYLOAD {
        anyhow::bail!(
            "payload_len={payload_len} exceeds MAX_PAYLOAD={MAX_PAYLOAD}, framing lost"
        );
    }

    recv_buf.resize(payload_len, 0);
    stream.read_exact(&mut recv_buf[..payload_len])?;

    let frame: FrameOwned = postcard::from_bytes(&recv_buf[..payload_len])?;
    Ok(frame)
}

/// Отправить команду firmware через TCP.
/// Framing: [4 байта BE длина] + postcard payload — тот же формат что и Packet.
fn send_command(stream: &mut TcpStream, cmd: Command) -> anyhow::Result<()> {
    let mut body_buf = [0u8; 64];
    let payload = postcard::to_slice(&cmd, &mut body_buf[HEADER_SIZE..])?;
    let payload_len = payload.len();
    body_buf[..HEADER_SIZE].copy_from_slice(&(payload_len as u32).to_be_bytes());
    stream.write_all(&body_buf[..HEADER_SIZE + payload_len])?;
    Ok(())
}

fn decode_i24_be(bytes: &[u8]) -> i32 {
    let raw = ((bytes[0] as u32) << 16) | ((bytes[1] as u32) << 8) | (bytes[2] as u32);
    if raw & 0x800000 != 0 {
        (raw | 0xFF00_0000) as i32
    } else {
        raw as i32
    }
}

pub(crate) fn tcp_listener_thread(shared: Arc<Mutex<Shared>>, ctx: egui::Context) {
    let addr = "0.0.0.0:7100";
    let listener = TcpListener::bind(addr).unwrap();
    println!("listening on {addr}");

    loop {
        {
            shared.lock().unwrap().status = ConnStatus::Listening;
            ctx.request_repaint();
        }

        let (mut stream, peer) = match listener.accept() {
            Ok(v) => v,
            Err(e) => {
                shared.lock().unwrap().status = ConnStatus::Disconnected(format!("accept error: {e}"));
                ctx.request_repaint();
                continue;
            }
        };

        let peer_str = peer.to_string();
        println!("connected: {peer_str}");

        // Таймаут на чтение: если firmware умерла без TCP FIN (WiFi пропал, power cycle),
        // read_exact повиснет на минуты (TCP retransmit timeout).
        // 5 секунд — при 2 кГц пакеты идут каждые ~50 мс, запас огромный.
        stream
            .set_read_timeout(Some(std::time::Duration::from_secs(5)))
            .unwrap();
        stream
            .set_write_timeout(Some(std::time::Duration::from_secs(5)))
            .unwrap();
        stream.set_nodelay(true).unwrap();

        {
            shared.lock().unwrap().status = ConnStatus::Connected(peer_str.clone());
            ctx.request_repaint();
        }

        // При коннекте отправляем желаемые настройки ADC — синхронизация UI → firmware.
        {
            let sh = shared.lock().unwrap();
            let pga = sh.desired_pga;
            let rate = sh.desired_rate;
            drop(sh);

            if let Err(e) = send_command(&mut stream, Command::SetPga(pga)) {
                println!("initial SetPga send error: {e}");
                shared.lock().unwrap().status = ConnStatus::Disconnected(format!("{peer_str}: {e}"));
                ctx.request_repaint();
                continue;
            }
            if rate != SampleRateHz::ZERO {
                if let Err(e) = send_command(&mut stream, Command::SetDataRate(rate)) {
                    println!("initial SetDataRate send error: {e}");
                    shared.lock().unwrap().status = ConnStatus::Disconnected(format!("{peer_str}: {e}"));
                    ctx.request_repaint();
                    continue;
                }
            }
            println!("tcp: sent initial config pga={pga} rate={rate:?}");
        }

        // Переиспользуемый буфер для чтения payload — без аллокации каждый пакет.
        let mut recv_buf = Vec::with_capacity(MAX_PAYLOAD);
        let mut expected_seq: Option<u32> = None;
        let mut diag = SessionDiag::new(Instant::now());

        loop {
            // Чтение фрейма без удержания mutex — IO не блокирует UI.
            let frame = match read_frame(&mut stream, &mut recv_buf) {
                Ok(v) => v,
                Err(e) => {
                    println!("disconnect ({peer_str}): {e}");
                    let mut sh = shared.lock().unwrap();
                    sh.clear_pending_commands();
                    sh.status = ConnStatus::Disconnected(format!("{peer_str}: {e}"));
                    ctx.request_repaint();
                    break;
                }
            };

            match frame {
                FrameOwned::Data(pkt) => {
                    // Детект пропущенных пакетов по seq.
                    let seq = pkt.seq.as_u32();
                    if let Some(exp) = expected_seq {
                        if seq != exp {
                            let gap = seq.wrapping_sub(exp);
                            diag.note_seq_gap(gap);
                            println!("WARNING: seq gap! expected={exp} got={seq} (lost {gap} packets)");
                        }
                    }
                    expected_seq = Some(seq.wrapping_add(1));
                    diag.note_packet(pkt.samples.len());

                    let mut sh = shared.lock().unwrap();

                    // Смена sample_rate → старые сэмплы в live_buf невалидны.
                    if sh.sample_rate != pkt.sample_rate && sh.sample_rate != SampleRateHz::ZERO {
                        sh.live_buf.clear();
                    }
                    sh.sample_rate = pkt.sample_rate;
                    sh.pga = pkt.pga;
                    sh.note_stream_packet(pkt.sample_rate);
                    sh.retry_stuck_rate_change(pkt.sample_rate, Instant::now());
                    sh.last_seq = pkt.seq;
                    sh.last_packet_at = Some(Instant::now());

                    for sample in &pkt.samples {
                        if sh.live_buf.len() >= LIVE_BUF_SIZE {
                            sh.live_buf.pop_front();
                        }
                        sh.live_buf.push_back(*sample);
                        sh.rev_buf.push(*sample);

                        process_recording_sample(&mut sh, *sample);
                    }

                    sh.total_samples += pkt.samples.len() as u64;

                    // Авто-остановка: если запись идёт в авто-режиме и KP не было >2с.
                    if sh.recording
                        && sh.auto_rec_last_kp_at.is_some()
                        && sh.auto_rec_last_kp_at.unwrap().elapsed() > AUTO_REC_IDLE_TIMEOUT
                    {
                        finish_recording(&mut sh);
                    }

                    let cmds: Vec<Command> = sh.cmd_queue.drain(..).collect();
                    drop(sh);

                    let mut cmd_send_failed = false;
                    for cmd in cmds {
                        // mark_command_sent ПЕРЕД send_command: иначе race — firmware успевает
                        // ответить пакетом с новым rate до того как pending_rate выставлен,
                        // note_stream_packet не сбрасывает его, и UI зависает в "Applying...".
                        shared.lock().unwrap().mark_command_sent(cmd);
                        if let Err(e) = send_command(&mut stream, cmd) {
                            println!("cmd send error: {e}");
                            let mut sh = shared.lock().unwrap();
                            sh.clear_pending_commands();
                            sh.status = ConnStatus::Disconnected(format!("{peer_str}: {e}"));
                            ctx.request_repaint();
                            cmd_send_failed = true;
                            break;
                        }
                        println!("cmd sent: {cmd:?}");
                    }
                    if cmd_send_failed {
                        break;
                    }
                }

                FrameOwned::DataPacked(pkt) => {
                    let seq = pkt.seq.as_u32();
                    if let Some(exp) = expected_seq {
                        if seq != exp {
                            let gap = seq.wrapping_sub(exp);
                            diag.note_seq_gap(gap);
                            println!("WARNING: seq gap! expected={exp} got={seq} (lost {gap} packets)");
                        }
                    }
                    expected_seq = Some(seq.wrapping_add(1));

                    if pkt.samples.len() % PACKED_CH0_SAMPLE_SIZE != 0 {
                        println!(
                            "disconnect ({peer_str}): packed payload len={} is not multiple of {}",
                            pkt.samples.len(),
                            PACKED_CH0_SAMPLE_SIZE
                        );
                        let mut sh = shared.lock().unwrap();
                        sh.clear_pending_commands();
                        sh.status = ConnStatus::Disconnected(format!(
                            "{peer_str}: invalid packed payload len={}",
                            pkt.samples.len()
                        ));
                        ctx.request_repaint();
                        break;
                    }

                    let sample_count = pkt.samples.len() / PACKED_CH0_SAMPLE_SIZE;
                    diag.note_packet(sample_count);

                    let mut sh = shared.lock().unwrap();

                    if sh.sample_rate != pkt.sample_rate && sh.sample_rate != SampleRateHz::ZERO {
                        sh.live_buf.clear();
                    }
                    sh.sample_rate = pkt.sample_rate;
                    sh.pga = pkt.pga;
                    sh.note_stream_packet(pkt.sample_rate);
                    sh.retry_stuck_rate_change(pkt.sample_rate, Instant::now());
                    sh.last_seq = pkt.seq;
                    sh.last_packet_at = Some(Instant::now());

                    let mut tick = pkt.base_tick;
                    for chunk in pkt.samples.chunks_exact(PACKED_CH0_SAMPLE_SIZE) {
                        let dt = u16::from_be_bytes([chunk[4], chunk[5]]) as u64;
                        tick = tick.wrapping_add(dt);
                        let sample = Sample {
                            ch0: AdcCount(decode_i24_be(&chunk[..3])),
                            ch1: AdcCount::ZERO,
                            flags: chunk[3],
                            tick,
                        };

                        if sh.live_buf.len() >= LIVE_BUF_SIZE {
                            sh.live_buf.pop_front();
                        }
                        sh.live_buf.push_back(sample);
                        sh.rev_buf.push(sample);
                        process_recording_sample(&mut sh, sample);
                    }

                    sh.total_samples += sample_count as u64;

                    if sh.recording
                        && sh.auto_rec_last_kp_at.is_some()
                        && sh.auto_rec_last_kp_at.unwrap().elapsed() > AUTO_REC_IDLE_TIMEOUT
                    {
                        finish_recording(&mut sh);
                    }

                    let cmds: Vec<Command> = sh.cmd_queue.drain(..).collect();
                    drop(sh);

                    let mut cmd_send_failed = false;
                    for cmd in cmds {
                        shared.lock().unwrap().mark_command_sent(cmd);
                        if let Err(e) = send_command(&mut stream, cmd) {
                            println!("cmd send error: {e}");
                            let mut sh = shared.lock().unwrap();
                            sh.clear_pending_commands();
                            sh.status = ConnStatus::Disconnected(format!("{peer_str}: {e}"));
                            ctx.request_repaint();
                            cmd_send_failed = true;
                            break;
                        }
                        println!("cmd sent: {cmd:?}");
                    }
                    if cmd_send_failed {
                        break;
                    }
                }

                FrameOwned::Keyphasor(kp) => {
                    diag.note_keyphasor();
                    let mut sh = shared.lock().unwrap();
                    sh.keyphasor_count += 1;
                    sh.last_keyphasor_at = Some(Instant::now());

                    // Семплов между этим и предыдущим KP.
                    let samples_between = sh.total_samples - sh.kp_last_total;
                    sh.kp_last_total = sh.total_samples;

                    // RPM из двух последних kp_ticks.
                    if let Some(&prev) = sh.kp_ticks.last() {
                        let dt = (kp.tick - prev) as f64 / SYSTIMER_HZ;
                        let _rpm = 60.0 / dt;
                    } else {
                        let _samples_between = samples_between;
                    }

                    sh.kp_ticks.push(kp.tick);
                    if sh.recording {
                        sh.rec_kp_ticks.push(kp.tick);
                    }
                    process_keyphasor(&mut sh);
                }
            }

            diag.maybe_print(&peer_str);
            diag.maybe_repaint(&ctx);
        }
    }
}
