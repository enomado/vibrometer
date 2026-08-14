use std::net::Ipv4Addr;

use toml::Value;

pub struct FirmwareConfig {
    pub wifi_ssid: String,
    pub wifi_passwd: String,
    pub server_ip: Ipv4Addr,
    pub server_port: u16,
}

pub fn parse_config(toml_str: &str) -> FirmwareConfig {
    let doc: Value = toml::from_str(toml_str).unwrap();
    let fw = &doc["firmware"];

    FirmwareConfig {
        wifi_ssid: fw["wifi_ssid"].as_str().unwrap().to_string(),
        wifi_passwd: fw["wifi_passwd"].as_str().unwrap().to_string(),
        server_ip: fw["server_ip"].as_str().unwrap().parse().unwrap(),
        server_port: fw["server_port"].as_integer().unwrap() as u16,
    }
}

pub fn ipv4_literal(ip: Ipv4Addr) -> String {
    let o = ip.octets();
    format!("core::net::Ipv4Addr::new({}, {}, {}, {})", o[0], o[1], o[2], o[3])
}
