use serde::Deserialize;
use toml::Value;

/// Analog PMR / UHF FM repeater settings (SXceiver 70 cm window).
///
/// Default: **parrot on PMR446 channel 2** — listen on ch2, when the carrier
/// drops re-transmit the recording on the **same** channel (store-and-forward
/// “loro”). Same antenna, time-separated RX then TX. Also supports
/// `simplex_relay` with different RX/TX channels, and `duplex` with a wide split.
#[derive(Debug, Clone)]
pub struct CfgPmrRepeater {
    pub enabled: bool,
    /// `parrot` (same channel store-and-forward), `simplex_relay` (RX then TX,
    /// channels may differ), or `duplex` (requires `|tx-rx| >= min_duplex_split_hz`).
    pub mode: String,
    /// PMR446 channel number 1..=16 used for TX ("talk"), when `tx_hz` is 0.
    pub tx_channel: u8,
    /// PMR446 channel number 1..=16 used for RX, when `rx_hz` is 0.
    pub rx_channel: u8,
    /// Explicit TX frequency in Hz (0 = derive from `tx_channel`).
    pub tx_hz: u32,
    /// Explicit RX frequency in Hz (0 = derive from `rx_channel`).
    pub rx_hz: u32,
    /// CTCSS tone Hz; 0.0 disables tone.
    pub ctcss_hz: f32,
    pub hangtime_ms: u32,
    /// Minimum duplex split enforced when `mode = "duplex"`.
    pub min_duplex_split_hz: u32,
    /// When true, enabling PMR takes exclusive RF (mute TETRA TX until PMR is off).
    pub exclusive_rf: bool,
}

impl Default for CfgPmrRepeater {
    fn default() -> Self {
        apply_pmr_repeater_patch(CfgPmrRepeaterDto::default())
            .expect("default pmr_repeater config must be valid")
    }
}

#[derive(Deserialize)]
pub struct CfgPmrRepeaterDto {
    #[serde(default)]
    pub enabled: bool,
    #[serde(default = "default_mode")]
    pub mode: String,
    #[serde(default = "default_tx_channel")]
    pub tx_channel: u8,
    #[serde(default = "default_rx_channel")]
    pub rx_channel: u8,
    #[serde(default)]
    pub tx_hz: u32,
    #[serde(default)]
    pub rx_hz: u32,
    #[serde(default)]
    pub ctcss_hz: f32,
    #[serde(default = "default_hangtime_ms")]
    pub hangtime_ms: u32,
    #[serde(default = "default_min_duplex_split_hz")]
    pub min_duplex_split_hz: u32,
    #[serde(default = "default_exclusive_rf")]
    pub exclusive_rf: bool,

    #[serde(flatten)]
    pub extra: std::collections::HashMap<String, Value>,
}

impl Default for CfgPmrRepeaterDto {
    fn default() -> Self {
        Self {
            enabled: false,
            mode: default_mode(),
            tx_channel: default_tx_channel(),
            rx_channel: default_rx_channel(),
            tx_hz: 0,
            rx_hz: 0,
            ctcss_hz: 0.0,
            hangtime_ms: default_hangtime_ms(),
            min_duplex_split_hz: default_min_duplex_split_hz(),
            exclusive_rf: default_exclusive_rf(),
            extra: Default::default(),
        }
    }
}

fn default_mode() -> String {
    "parrot".to_string()
}
fn default_tx_channel() -> u8 {
    2
}
fn default_rx_channel() -> u8 {
    2
}
fn default_hangtime_ms() -> u32 {
    1000
}
fn default_min_duplex_split_hz() -> u32 {
    100_000
}
fn default_exclusive_rf() -> bool {
    true
}

/// Classic 8-channel PMR446 (12.5 kHz). Channels 9..=16 use the 6.25 kHz raster
/// starting at 446.003125 MHz (common “digital/expanded” numbering).
pub fn pmr446_channel_hz(channel: u8) -> Option<u32> {
    match channel {
        1..=8 => Some(446_006_250 + u32::from(channel - 1) * 12_500),
        9..=16 => Some(446_003_125 + u32::from(channel - 9) * 6_250),
        _ => None,
    }
}

impl CfgPmrRepeater {
    pub fn resolved_tx_hz(&self) -> Option<u32> {
        if self.tx_hz != 0 {
            Some(self.tx_hz)
        } else {
            pmr446_channel_hz(self.tx_channel)
        }
    }

    pub fn resolved_rx_hz(&self) -> Option<u32> {
        if self.rx_hz != 0 {
            Some(self.rx_hz)
        } else {
            pmr446_channel_hz(self.rx_channel)
        }
    }

    pub fn split_hz(&self) -> Option<u32> {
        let tx = self.resolved_tx_hz()?;
        let rx = self.resolved_rx_hz()?;
        Some(tx.abs_diff(rx))
    }

    /// Exclusive PMR is active — TETRA downlink must stay muted.
    pub fn holds_tetra_rf(&self) -> bool {
        self.enabled && self.exclusive_rf
    }
}

pub fn apply_pmr_repeater_patch(src: CfgPmrRepeaterDto) -> Result<CfgPmrRepeater, String> {
    if !src.extra.is_empty() {
        return Err(format!(
            "pmr_repeater: unrecognized fields {:?}",
            src.extra.keys().collect::<Vec<_>>()
        ));
    }

    let mode = src.mode.trim().to_ascii_lowercase();
    if mode != "parrot" && mode != "simplex_relay" && mode != "duplex" {
        return Err(
            "pmr_repeater: mode must be \"parrot\", \"simplex_relay\" or \"duplex\"".to_string(),
        );
    }

    if src.tx_hz == 0 && !(1..=16).contains(&src.tx_channel) {
        return Err("pmr_repeater: tx_channel must be 1..=16 when tx_hz is 0".to_string());
    }
    if src.rx_hz == 0 && !(1..=16).contains(&src.rx_channel) {
        return Err("pmr_repeater: rx_channel must be 1..=16 when rx_hz is 0".to_string());
    }

    let cfg = CfgPmrRepeater {
        enabled: src.enabled,
        mode,
        tx_channel: src.tx_channel,
        rx_channel: src.rx_channel,
        tx_hz: src.tx_hz,
        rx_hz: src.rx_hz,
        ctcss_hz: src.ctcss_hz.max(0.0),
        hangtime_ms: src.hangtime_ms,
        min_duplex_split_hz: src.min_duplex_split_hz.max(25_000),
        exclusive_rf: src.exclusive_rf,
    };

    let Some(tx) = cfg.resolved_tx_hz() else {
        return Err("pmr_repeater: could not resolve TX frequency".to_string());
    };
    let Some(rx) = cfg.resolved_rx_hz() else {
        return Err("pmr_repeater: could not resolve RX frequency".to_string());
    };
    // SXceiver useful window is UHF (~360–550 MHz); keep a soft guard.
    for (name, hz) in [("tx", tx), ("rx", rx)] {
        if !(360_000_000..=550_000_000).contains(&hz) {
            return Err(format!(
                "pmr_repeater: {name} frequency {hz} Hz is outside the SXceiver UHF window"
            ));
        }
    }

    let split = tx.abs_diff(rx);
    if cfg.mode == "parrot" && split != 0 {
        return Err(
            "pmr_repeater: parrot mode needs the same TX and RX channel/frequency (store-and-forward on one channel)"
                .to_string(),
        );
    }
    if cfg.mode == "duplex" {
        if split == 0 {
            return Err(
                "pmr_repeater: duplex needs different TX/RX — use mode = \"parrot\" for same-channel loro"
                    .to_string(),
            );
        }
        if split < cfg.min_duplex_split_hz {
            return Err(format!(
                "pmr_repeater: duplex needs ≥{} Hz split (got {} Hz). \
                 Use mode = \"parrot\" (same channel) or \"simplex_relay\"",
                cfg.min_duplex_split_hz, split
            ));
        }
    }

    Ok(cfg)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pmr446_channel_table() {
        assert_eq!(pmr446_channel_hz(1), Some(446_006_250));
        assert_eq!(pmr446_channel_hz(2), Some(446_018_750));
        assert_eq!(pmr446_channel_hz(8), Some(446_093_750));
        assert!(pmr446_channel_hz(0).is_none());
        assert!(pmr446_channel_hz(17).is_none());
    }

    #[test]
    fn default_is_parrot_ch2() {
        let cfg = CfgPmrRepeater::default();
        assert!(!cfg.enabled);
        assert_eq!(cfg.mode, "parrot");
        assert_eq!(cfg.tx_channel, 2);
        assert_eq!(cfg.rx_channel, 2);
        assert_eq!(cfg.resolved_tx_hz(), Some(446_018_750));
        assert_eq!(cfg.resolved_rx_hz(), Some(446_018_750));
        assert_eq!(cfg.split_hz(), Some(0));
    }

    #[test]
    fn parrot_rejects_different_channels() {
        let mut dto = CfgPmrRepeaterDto::default();
        dto.mode = "parrot".into();
        dto.tx_channel = 1;
        dto.rx_channel = 2;
        let err = apply_pmr_repeater_patch(dto).unwrap_err();
        assert!(err.contains("parrot"));
    }

    #[test]
    fn simplex_relay_allows_same_or_split() {
        let mut same = CfgPmrRepeaterDto::default();
        same.mode = "simplex_relay".into();
        same.tx_channel = 2;
        same.rx_channel = 2;
        assert!(apply_pmr_repeater_patch(same).is_ok());

        let mut split = CfgPmrRepeaterDto::default();
        split.mode = "simplex_relay".into();
        split.tx_channel = 1;
        split.rx_channel = 8;
        assert!(apply_pmr_repeater_patch(split).is_ok());
    }

    #[test]
    fn adjacent_channels_reject_duplex() {
        let mut dto = CfgPmrRepeaterDto::default();
        dto.mode = "duplex".into();
        dto.tx_channel = 1;
        dto.rx_channel = 2;
        let err = apply_pmr_repeater_patch(dto).unwrap_err();
        assert!(err.contains("split") || err.contains("parrot"));
    }

    #[test]
    fn duplex_ok_with_wide_split() {
        let mut dto = CfgPmrRepeaterDto::default();
        dto.mode = "duplex".into();
        dto.tx_hz = 438_225_000;
        dto.rx_hz = 430_625_000;
        let cfg = apply_pmr_repeater_patch(dto).unwrap();
        assert_eq!(cfg.mode, "duplex");
        assert_eq!(cfg.split_hz(), Some(7_600_000));
    }

    #[test]
    fn holds_tetra_rf_requires_enabled_and_exclusive() {
        let mut cfg = CfgPmrRepeater::default();
        assert!(!cfg.holds_tetra_rf());
        cfg.enabled = true;
        cfg.exclusive_rf = true;
        assert!(cfg.holds_tetra_rf());
        cfg.exclusive_rf = false;
        assert!(!cfg.holds_tetra_rf());
    }
}
