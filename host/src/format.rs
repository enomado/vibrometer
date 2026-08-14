pub(crate) fn format_compact_value(v: f64) -> String {
    let abs = v.abs();
    if abs >= 1_000_000.0 {
        format!("{:.1}M", v / 1_000_000.0)
    } else if abs >= 1_000.0 {
        format!("{:.1}k", v / 1_000.0)
    } else if abs >= 1.0 {
        format!("{:.0}", v)
    } else if abs >= 0.01 {
        format!("{:.2}", v)
    } else {
        format!("{:.1e}", v)
    }
}

#[cfg(test)]
mod tests {
    use super::format_compact_value;

    #[test]
    fn format_value_compact() {
        assert_eq!(format_compact_value(1_500_000.0), "1.5M");
        assert_eq!(format_compact_value(2500.0), "2.5k");
        assert_eq!(format_compact_value(42.0), "42");
        assert_eq!(format_compact_value(0.05), "0.05");
    }
}
