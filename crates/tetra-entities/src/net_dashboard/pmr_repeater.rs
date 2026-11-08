//! Dashboard persistence for the PMR / UHF FM repeater switch.

use tetra_config::bluestation::PmrRepeaterRuntimeOverride;

/// Rewrite (or insert) the `[pmr_repeater]` section in the live config file.
pub fn write_pmr_repeater_to_toml(config_path: &str, ov: &PmrRepeaterRuntimeOverride) -> std::io::Result<()> {
    let original = std::fs::read_to_string(config_path)?;
    let mode_escaped = ov.mode.replace('\\', "\\\\").replace('"', "\\\"");
    let section = format!(
        "[pmr_repeater]\n\
         enabled = {}\n\
         mode = \"{}\"\n\
         tx_channel = {}\n\
         rx_channel = {}\n\
         tx_hz = {}\n\
         rx_hz = {}\n\
         ctcss_hz = {}\n\
         hangtime_ms = {}\n\
         min_duplex_split_hz = {}\n\
         exclusive_rf = {}",
        ov.enabled,
        mode_escaped,
        ov.tx_channel,
        ov.rx_channel,
        ov.tx_hz,
        ov.rx_hz,
        ov.ctcss_hz,
        ov.hangtime_ms,
        ov.min_duplex_split_hz,
        ov.exclusive_rf
    );

    let lines: Vec<&str> = original.lines().collect();
    let mut out: Vec<String> = Vec::with_capacity(lines.len() + 14);
    let mut i = 0;
    let mut replaced = false;

    while i < lines.len() {
        let trimmed = lines[i].trim_start();
        if trimmed.starts_with("[pmr_repeater]") {
            out.push(section.clone());
            replaced = true;
            i += 1;
            while i < lines.len() {
                let t = lines[i].trim_start();
                if t.starts_with('[') && t.contains(']') {
                    break;
                }
                i += 1;
            }
            continue;
        }
        out.push(lines[i].to_string());
        i += 1;
    }

    if !replaced {
        if !out.is_empty() && !out.last().map(|l| l.is_empty()).unwrap_or(true) {
            out.push(String::new());
        }
        out.push(section);
    }

    let mut new_content = out.join("\n");
    if original.ends_with('\n') {
        new_content.push('\n');
    }

    let backup = format!("{config_path}.pmr.bak");
    let _ = std::fs::copy(config_path, &backup);
    std::fs::write(config_path, new_content)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_toml_replace_section() {
        let cfg = "[cell]\nfoo = 1\n\n[pmr_repeater]\nenabled = false\ntx_channel = 1\n\n[security]\nbar = 2\n";
        let dir = std::env::temp_dir();
        let path = dir.join("fs_pmr_test_replace.toml");
        std::fs::write(&path, cfg).unwrap();
        let ov = PmrRepeaterRuntimeOverride {
            enabled: true,
            mode: "simplex_relay".into(),
            tx_channel: 1,
            rx_channel: 2,
            tx_hz: 0,
            rx_hz: 0,
            ctcss_hz: 0.0,
            hangtime_ms: 1000,
            min_duplex_split_hz: 100_000,
            exclusive_rf: true,
        };
        write_pmr_repeater_to_toml(path.to_str().unwrap(), &ov).unwrap();
        let out = std::fs::read_to_string(&path).unwrap();
        assert!(out.contains("enabled = true"));
        assert!(out.contains("rx_channel = 2"));
        assert!(out.contains("[cell]"));
        assert!(out.contains("[security]"));
        let _ = std::fs::remove_file(&path);
    }
}
