//! Dashboard persistence and install helpers for the SIP / PBX client.
//!
//! Rewrites only the `[asterisk]` section of the live config file, and exposes
//! the checks used by the Install button (native tetra-codec + compiled-in
//! `--features asterisk` marker).

use tetra_config::bluestation::CfgAsterisk;

/// Filename written next to the source tree after a successful SIP install so
/// later OTA rebuilds keep `--features asterisk` instead of dropping the bridge.
pub const SIP_CLIENT_MARKER: &str = ".flowstation-sip-client";

/// Mask a secret for display. Returns an empty string for an empty value.
pub fn mask_secret(secret: &str) -> String {
    crate::net_dashboard::dapnet::mask_secret(secret)
}

fn toml_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

fn toml_string_list(items: &[String]) -> String {
    items
        .iter()
        .map(|s| format!("\"{}\"", toml_escape(s)))
        .collect::<Vec<_>>()
        .join(", ")
}

/// Rewrite (or insert) the `[asterisk]` section. A `.asterisk.bak` backup is made.
pub fn write_asterisk_to_toml(config_path: &str, cfg: &CfgAsterisk) -> std::io::Result<()> {
    let original = std::fs::read_to_string(config_path)?;
    let section = format!(
        "[asterisk]\n\
         enabled = {}\n\
         outbound_prefix = \"{}\"\n\
         strip_outbound_prefix = {}\n\
         inbound_prefix = \"{}\"\n\
         register = {}\n\
         codec = \"{}\"\n\
         service_numbers = [{}]\n\
         rtp_port_min = {}\n\
         rtp_port_max = {}\n\
         bind_addr = \"{}\"\n\
         bind_port = {}\n\
         remote_host = \"{}\"\n\
         remote_port = {}\n\
         outbound_proxy_host = \"{}\"\n\
         outbound_proxy_port = {}\n\
         contact_host = \"{}\"\n\
         from_domain = \"{}\"\n\
         local_user = \"{}\"\n\
         auth_user = \"{}\"\n\
         password = \"{}\"\n\
         realm = \"{}\"\n\
         options_interval_secs = {}\n\
         ptime_ms = {}\n\
         dl_jitter_ms = {}\n\
         allow_from = [{}]\n\
         max_pending_dialogs = {}\n\
         max_invites_per_minute = {}",
        cfg.enabled,
        toml_escape(&cfg.outbound_prefix),
        cfg.strip_outbound_prefix,
        toml_escape(&cfg.inbound_prefix),
        cfg.register,
        toml_escape(&cfg.codec),
        toml_string_list(&cfg.service_numbers),
        cfg.rtp_port_min,
        cfg.rtp_port_max,
        toml_escape(&cfg.bind_addr),
        cfg.bind_port,
        toml_escape(&cfg.remote_host),
        cfg.remote_port,
        toml_escape(&cfg.outbound_proxy_host),
        cfg.outbound_proxy_port,
        toml_escape(&cfg.contact_host),
        toml_escape(&cfg.from_domain),
        toml_escape(&cfg.local_user),
        toml_escape(&cfg.auth_user),
        toml_escape(cfg.password.as_ref()),
        toml_escape(&cfg.realm),
        cfg.options_interval_secs,
        cfg.ptime_ms,
        cfg.dl_jitter_ms,
        toml_string_list(&cfg.allow_from),
        cfg.max_pending_dialogs,
        cfg.max_invites_per_minute,
    );

    let lines: Vec<&str> = original.lines().collect();
    let mut out: Vec<String> = Vec::with_capacity(lines.len() + 40);
    let mut i = 0;
    let mut replaced = false;

    while i < lines.len() {
        let trimmed = lines[i].trim_start();
        if trimmed.starts_with("[asterisk]") {
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

    let backup = format!("{config_path}.asterisk.bak");
    let _ = std::fs::copy(config_path, &backup);
    std::fs::write(config_path, new_content)
}

/// True when this binary was compiled with `--features asterisk`.
pub fn sip_client_compiled_in() -> bool {
    cfg!(feature = "asterisk")
}

/// True when `pkg-config` can see the native TETRA ACELP library used by the SIP audio bridge.
pub fn tetra_codec_installed() -> bool {
    if std::process::Command::new("pkg-config")
        .args(["--exists", "tetra-codec"])
        .status()
        .map(|s| s.success())
        .unwrap_or(false)
    {
        return true;
    }
    // Fallback for installs that skipped the .pc file.
    [
        "/usr/local/lib/libtetra-codec.so",
        "/usr/lib/libtetra-codec.so",
        "/usr/local/lib/libtetra-codec.a",
    ]
    .iter()
    .any(|p| std::path::Path::new(p).exists())
}

pub fn sip_client_marker_path(src_dir: &std::path::Path) -> std::path::PathBuf {
    src_dir.join(SIP_CLIENT_MARKER)
}

/// Whether a later `cargo build --release` must keep `--features asterisk`.
///
/// Uses the marker written by a successful Install, or the fact that *this*
/// running binary already has the feature. Config `enabled = true` alone is
/// not enough — that would make a later OTA fail on a machine without tetra-codec.
pub fn sip_client_should_build(src_dir: &std::path::Path) -> bool {
    sip_client_compiled_in() || sip_client_marker_path(src_dir).is_file()
}

pub fn write_sip_client_marker(src_dir: &std::path::Path) -> std::io::Result<()> {
    std::fs::write(
        sip_client_marker_path(src_dir),
        "FlowStation SIP client installed. Keep cargo --features asterisk on rebuild.\n",
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use tetra_config::bluestation::sec_asterisk::{CfgAsteriskDto, apply_asterisk_patch};

    fn sample_cfg() -> CfgAsterisk {
        apply_asterisk_patch(CfgAsteriskDto {
            enabled: true,
            outbound_prefix: "91".into(),
            remote_host: "pbx.example.com".into(),
            outbound_proxy_host: "proxy.example.com".into(),
            outbound_proxy_port: 5060,
            local_user: "1001".into(),
            auth_user: "1001".into(),
            password: "s3cret".into(),
            from_domain: "pbx.example.com".into(),
            service_numbers: vec!["600".into(), "601".into()],
            extra: Default::default(),
            ..CfgAsteriskDto::default()
        })
        .unwrap()
    }

    #[test]
    fn write_inserts_asterisk_section_and_round_trips() {
        let dir = std::env::temp_dir();
        let path = dir.join(format!("fs_ast_write_{}.toml", std::process::id()));
        std::fs::write(&path, "config_version = \"0.6\"\nstack_mode = \"Bs\"\n").unwrap();
        write_asterisk_to_toml(path.to_str().unwrap(), &sample_cfg()).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains("[asterisk]"));
        assert!(text.contains("remote_host = \"pbx.example.com\""));
        assert!(text.contains("outbound_proxy_host = \"proxy.example.com\""));
        assert!(text.contains("local_user = \"1001\""));
        assert!(text.contains("password = \"s3cret\""));
        assert!(text.contains("\"600\""));
        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(format!("{}.asterisk.bak", path.display()));
    }

    #[test]
    fn write_replaces_existing_asterisk_section_only() {
        let dir = std::env::temp_dir();
        let path = dir.join(format!("fs_ast_replace_{}.toml", std::process::id()));
        std::fs::write(
            &path,
            "config_version = \"0.6\"\n\n[net_info]\nmcc = 204\nmnc = 1\n\n[asterisk]\nenabled = false\n\n[dapnet]\nenabled = false\n",
        )
        .unwrap();
        write_asterisk_to_toml(path.to_str().unwrap(), &sample_cfg()).unwrap();
        let text = std::fs::read_to_string(&path).unwrap();
        assert!(text.contains("mcc = 204"));
        assert!(text.contains("[dapnet]"));
        assert_eq!(text.matches("[asterisk]").count(), 1);
        assert!(text.contains("enabled = true"));
        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_file(format!("{}.asterisk.bak", path.display()));
    }

    #[test]
    fn marker_round_trip() {
        let dir = std::env::temp_dir().join(format!("fs_ast_marker_{}", std::process::id()));
        let _ = std::fs::create_dir_all(&dir);
        assert!(!sip_client_marker_path(&dir).is_file());
        write_sip_client_marker(&dir).unwrap();
        assert!(sip_client_marker_path(&dir).is_file());
        assert!(sip_client_should_build(&dir));
        let _ = std::fs::remove_dir_all(&dir);
    }
}
