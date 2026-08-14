use std::{env, fs, path::PathBuf};

mod build_helpers;

fn main() {
    // config.toml не в репозитории (в нём wifi-пароль) — его заводит каждый у себя
    // из config.toml.template. Отсутствие файла — самая частая ошибка первой сборки,
    // поэтому вместо голого unwrap-паникa даём внятную инструкцию.
    println!("cargo:rerun-if-changed=config.toml");
    println!("cargo:rerun-if-changed=config.toml.template");

    let toml_str = match fs::read_to_string("config.toml") {
        Ok(s) => s,
        Err(e) => panic!(
            "не могу прочитать firmware/config.toml ({e}).\n\
             Скопируйте шаблон и впишите свои параметры:\n\
             \n    cp config.toml.template config.toml\n"
        ),
    };
    let fw = build_helpers::parse_config(&toml_str);
    let ip_lit = build_helpers::ipv4_literal(fw.server_ip);

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());

    let contents = format!(
        r#"
pub const WIFI_SSID: &str = "{ssid}";
pub const WIFI_PASSWD: &str = "{passwd}";
pub const SERVER_IP: core::net::Ipv4Addr = {ip};
pub const SERVER_PORT: u16 = {port};
"#,
        ssid = fw.wifi_ssid,
        passwd = fw.wifi_passwd,
        ip = ip_lit,
        port = fw.server_port,
    );

    fs::write(out_dir.join("constants.rs"), contents).unwrap();

    // Пароль сюда сознательно не печатаем: cargo-warning'и оседают в CI-логах.
    println!(
        "cargo:warning=WiFi: {} -> {}:{}",
        fw.wifi_ssid, fw.server_ip, fw.server_port
    );
}
