# vibrometer

A laser vibrometer / rotor balancing rig: ESP32-C3 firmware that acquires a 24-bit
vibration signal, and a desktop app that receives it over WiFi, visualizes it live,
records sessions and does the rotor-dynamics math.

Both halves live in this repository because they share the wire protocol and the
domain types — `crates/protocol` and `crates/types` are `no_std` and are compiled
into the firmware as well as into the host app.

> Code comments and `docs/` are in Russian.

## Layout

```
crates/
  protocol/     wire protocol: frames, commands, packing        (no_std, shared)
  types/        domain newtypes: AdcCount, Rpm, AngleRad, …     (no_std, shared)
  analysis/     DSP: FFT, Goertzel, PLL, spline, order tracking, balancing
host/           desktop receiver — egui app (vibro-receiver)
firmware/       ESP32-C3 firmware — embassy + esp-hal           (own workspace)
docs/           physics, methods, signal processing, hardware, decisions
```

## Host app

Receives ADC samples and keyphasor events over TCP, shows them live and stores them.

- 24-bit ADS1256 samples + keyphasor (shaft-revolution) events over TCP:7100
- live strip chart, FFT, order tracking, polar plots
- sessions recorded to Parquet/Arrow
- remote ADC control: PGA gain, sample rate
- rotor balancing: influence coefficients, Bode diagrams

```
host/src/
  main.rs        app state, UI panels
  tcp.rs         TCP listener, frame deserialization
  recording.rs   Parquet save/load, session management
  analysis.rs    order tracking, DSP integration
  strip.rs       strip chart rendering
  ui_polar.rs    polar plot widget
  sound.rs       audio feedback
```

Run it from the repository root (recordings are written to `./recordings`):

```sh
cargo run --release
```

Rust edition 2024.

## Firmware

ESP32-C3, `no_std`, embassy + esp-hal. Reads the differential ADC channel, timestamps
keyphasor pulses in hardware and streams both to the host.

- differential channel `AIN0`–`AIN1`, up to 30 kSPS
- keyphasor pulses captured by GPIO interrupt
- accepts `SetPga` / `SetDataRate` commands from the host
- BLE GATT server for auxiliary status (currently disabled in `src/bin/main.rs`
  while sample retention at 7500 SPS is being evaluated)

Architecture:

- **ISR (Priority2)** — the `DRDY` interrupt reads the sample over SPI2+DMA; the
  keyphasor edge latches a `SystemTimer` tick, so ADC and keyphasor share one clock
  and the phase does not drift.
- **Lock-free SPSC queues** (4096 samples, 32 keyphasor events) between the ISR and
  the async tasks.
- **96 KB heap**, sized for WiFi + BLE coexistence.

Dropped samples on the ESP32-C3 turned out to come from missed `DRDY` edges under
cooperative scheduling rather than from blocking SPI time — interrupt-driven capture
on `DRDY↓` is the robust baseline, and DMA only shortens the ISR on top of that.
See [docs/freeze_bug_postmortem.md](docs/freeze_bug_postmortem.md).

### Configuration

WiFi credentials and the host address are build-time parameters: `build.rs` parses
`firmware/config.toml` and generates constants from it. That file is **not** in the
repository — copy the template and fill in your own values:

```sh
cd firmware
cp config.toml.template config.toml
```

```toml
[firmware]
wifi_ssid   = "my-network"
wifi_passwd = "my-password"
server_ip   = "192.168.1.100"
server_port = 7100
```

`./update-ip.sh` rewrites `server_ip` to this machine's current address, which is
handy when the host sits behind DHCP.

### Build & flash

```sh
cd firmware
cargo run --release
```

Needs the RISC-V bare-metal target (`riscv32imc-unknown-none-elf`); `runner` in
`.cargo/config.toml` flashes and opens a `defmt` monitor via `espflash`.

The firmware is a separate cargo workspace — a different target, toolchain and
lockfile — so it is `exclude`d from the root workspace and built from `firmware/`.

## Hardware

MCU: `ESP-C3-32S-Kit`. ADC: `LC Technology ADS1256` module.

| Signal | GPIO | Notes |
|--------|------|-------|
| SCLK | IO4 | SPI clock (1920 kHz) |
| DIN | IO6 | SPI MOSI |
| DOUT | IO5 | SPI MISO |
| CS | IO7 | SPI chip select |
| DRDY | IO10 | ADC data ready (interrupt) |
| Keyphasor | IO1 | TCRT5000 proximity sensor |
| PDWN | 3.3V | tied high, not GPIO-controlled |

Analog input: `AIN0` → signal `+`, `AIN1` → signal `−`.

**Do not use IO8** — it interferes with boot/flashing.

ADS1256 notes: SPI Mode 1 with manual CS; init sends RESET + SDATAC and then RDATAC
for continuous streaming; a `SELFCAL` is required after a MUX change, `SYNC+WAKEUP`
alone is not enough. A healthy init logs
`ADS1256: init ok STATUS=0x30 MUX=0x01 ADCON=0x00 DRATE=0xb0`.

## Patches

`firmware/patches/esp-phy` — vendored fix for a PHY clock ref-count leak in
`disable_phy()`. Without it every failed TCP connect leaks ~28 refs from a `u8`
counter, which panics on overflow after ~9 attempts.
