//! PMR446 / UHF NBFM store-and-forward parrot (“loro”).
//!
//! While exclusive PMR holds the SXceiver, this engine:
//! 1. Listens on one channel (default PMR446 ch2 = 446.01875 MHz)
//! 2. Records demodulated audio while a carrier is present
//! 3. After hangtime, re-transmits the recording on the same channel
//!
//! Runs on the existing PHY thread using the live Soapy streams (no second
//! process). IQ stays at the SDR sample rate; audio is stored at ~12.5 kHz.

use std::f32::consts::TAU;
use std::time::{Duration, Instant};

use tetra_config::bluestation::CfgPmrRepeater;
use tetra_pdus::phy::traits::rxtx_dev::RxTxDevError;

use super::dsp_types::*;
use super::soapyio::SoapyIo;

/// Peak FM deviation used for PMR446-class NBFM (±2.5 kHz).
const DEVIATION_HZ: f64 = 2_500.0;
/// Target audio storage rate (IQ is decimated down to this).
const AUDIO_FS: f64 = 12_500.0;
/// Refuse to record forever if someone holds PTT open.
const MAX_RECORD_SECS: f32 = 45.0;
/// After this many seconds of recording, the next hang/replay is preceded by
/// a double beep (simplex: heard when they unkey / when we force-cut).
const WARN_RECORD_SECS: f32 = 40.0;
/// Soft TX amplitude (full-scale would slam the SXceiver PA).
const TX_AMPLITUDE: f32 = 0.45;
/// Timeout courtesy tone.
const BEEP_HZ: f64 = 1_000.0;
const BEEP_ON_MS: u32 = 120;
const BEEP_OFF_MS: u32 = 80;
const BEEP_COUNT: u32 = 2;
/// Minimum recording length before a replay is worth airing (~150 ms).
const MIN_RECORD_SAMPLES: usize = (AUDIO_FS * 0.15) as usize;
/// Open squelch this many dB above the tracked noise floor.
const SQUELCH_OPEN_DB: f32 = 6.0;
/// Close squelch this many dB above the noise floor (hysteresis).
const SQUELCH_CLOSE_DB: f32 = 2.5;
/// IQ blocks to observe before allowing carrier (learn ambient floor).
const SQUELCH_WARMUP_BLOCKS: u32 = 400;
/// Audio high-pass — strips slow discriminator bias / rumble.
const AUDIO_HPF_HZ: f64 = 500.0;
/// Ignore RX carrier this long after replay (own TX bleed into RX).
const POST_REPLAY_COOLDOWN: Duration = Duration::from_millis(750);
/// Use only the first N audio samples of a take to estimate AFC (avoid voice bias).
const AFC_ESTIMATE_AUDIO_SAMPLES: usize = (AUDIO_FS * 0.25) as usize;
/// IQ DC tracker time-constant (Hz). Kills LO leakage birdies before FM demod.
const IQ_DC_HZ: f64 = 30.0;
/// Search band for the constant on-air birdie (user capture ≈ 623 Hz).
const BIRDIE_SEARCH_LO_HZ: f64 = 350.0;
const BIRDIE_SEARCH_HI_HZ: f64 = 1_200.0;
/// Notch Q — narrow enough to spare voice formants.
const BIRDIE_NOTCH_Q: f64 = 12.0;
/// Require birdie peak this much above band median before notching.
const BIRDIE_PEAK_RATIO: f64 = 4.0;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Listen,
    Record,
    Hang,
    /// Double beep before replay when the take hit the time warning / max.
    TimeoutBeep,
    Replay,
}

pub struct PmrParrot {
    fs: f64,
    decim: usize,
    hangtime: Duration,
    ctcss_hz: f32,
    phase: Phase,
    /// Previous IQ for FM discriminator.
    prev: ComplexSample,
    /// Slow magnitude EMA (linear) for logging / fallback.
    mag_ema: f32,
    /// Noise floor in dBFS (20·log10(mag)), tracked with fast-down / slow-up.
    noise_db: f32,
    /// Blocks seen while learning the ambient floor (no opens yet).
    squelch_warmup: u32,
    carrier: bool,
    /// 1-pole HPF state (DC block on demod audio).
    hpf_x1: f32,
    hpf_y1: f32,
    hpf_alpha: f32,
    /// Running mean of raw (pre-HPF) audio while recording — LO/discriminator offset.
    afc_raw_sum: f64,
    afc_raw_n: usize,
    /// Slow IQ DC estimate (LO leakage) removed before demod.
    iq_dc: ComplexSample,
    iq_dc_alpha: f32,
    /// IQ low-pass state before decimated FM demod (anti-alias).
    iq_lp: ComplexSample,
    iq_lp_alpha: f32,
    iq_lp_n: usize,
    /// Recorded baseband audio at AUDIO_FS.
    recording: Vec<f32>,
    hang_deadline: Option<Instant>,
    /// After replay, ignore carriers until this instant.
    listen_after: Option<Instant>,
    /// Play double-beep before replay (timeout / long take).
    play_timeout_beep: bool,
    /// Logged once when recording crosses WARN_RECORD_SECS.
    warn_logged: bool,
    /// Sample cursor while generating timeout beeps (at IQ rate).
    beep_i: usize,
    /// Total IQ samples for the beep pattern.
    beep_len: usize,
    /// Replay cursor into `recording`.
    replay_i: usize,
    /// Continuous FM modulator phase (radians).
    tx_phase: f64,
    /// CTCSS oscillator phase while transmitting.
    ctcss_phase: f64,
    /// Scratch buffers reused every tick (avoid per-block alloc).
    rx_scratch: Vec<ComplexSample>,
    tx_scratch: Vec<ComplexSample>,
    /// Block size matched to Soapy period (~1.5 ms at 600 kHz → 900).
    block_len: usize,
    stats_rx_blocks: u64,
    stats_tx_blocks: u64,
    last_log: Instant,
}

impl PmrParrot {
    pub fn new(fs: f64, cfg: &CfgPmrRepeater) -> Self {
        let fs = if fs.is_finite() && fs > 0.0 { fs } else { 600_000.0 };
        let decim = ((fs / AUDIO_FS).round() as usize).max(1);
        // Match SXceiver period ≈ 1.5 ms.
        let block_len = ((fs * 0.0015).round() as usize).clamp(256, 4096);
        let hangtime = Duration::from_millis(u64::from(cfg.hangtime_ms.max(100)));
        let audio_fs = fs / decim as f64;
        // y[n] = α·(y[n-1] + x[n] - x[n-1]), α = RC/(RC+dt), RC = 1/(2πf)
        let rc = 1.0 / (TAU as f64 * AUDIO_HPF_HZ);
        let dt = 1.0 / audio_fs;
        let hpf_alpha = (rc / (rc + dt)) as f32;
        // 1-pole EMA ≈ low-pass at IQ_DC_HZ on the IQ stream.
        let iq_dc_alpha = (TAU as f64 * IQ_DC_HZ / fs).clamp(1.0e-6, 0.05) as f32;
        tracing::info!(
            "PMR parrot: fs={:.0} Hz audio≈{:.0} Hz (decim={}) hang={}ms ctcss={:.1} block={} hpf={:.0}Hz iq_dc={:.0}Hz",
            fs,
            audio_fs,
            decim,
            hangtime.as_millis(),
            cfg.ctcss_hz,
            block_len,
            AUDIO_HPF_HZ,
            IQ_DC_HZ
        );
        Self {
            fs,
            decim,
            hangtime,
            ctcss_hz: cfg.ctcss_hz.max(0.0),
            phase: Phase::Listen,
            prev: ComplexSample::ZERO,
            mag_ema: 0.0,
            noise_db: -60.0,
            squelch_warmup: 0,
            carrier: false,
            hpf_x1: 0.0,
            hpf_y1: 0.0,
            hpf_alpha,
            afc_raw_sum: 0.0,
            afc_raw_n: 0,
            iq_dc: ComplexSample::ZERO,
            iq_dc_alpha,
            iq_lp: ComplexSample::ZERO,
            iq_lp_alpha: 2.0 / (decim as f32 + 1.0),
            iq_lp_n: 0,
            recording: Vec::with_capacity((AUDIO_FS * 8.0) as usize),
            hang_deadline: None,
            listen_after: None,
            play_timeout_beep: false,
            warn_logged: false,
            beep_i: 0,
            beep_len: 0,
            replay_i: 0,
            tx_phase: 0.0,
            ctcss_phase: 0.0,
            rx_scratch: vec![ComplexSample::ZERO; block_len],
            tx_scratch: vec![ComplexSample::ZERO; block_len],
            block_len,
            stats_rx_blocks: 0,
            stats_tx_blocks: 0,
            last_log: Instant::now(),
        }
    }

    pub fn phase_name(&self) -> &'static str {
        match self.phase {
            Phase::Listen => "listen",
            Phase::Record => "record",
            Phase::Hang => "hang",
            Phase::TimeoutBeep => "timeout_beep",
            Phase::Replay => "replay",
        }
    }

    fn warn_samples(&self) -> usize {
        (WARN_RECORD_SECS * (self.fs / self.decim as f64) as f32) as usize
    }

    fn max_samples(&self) -> usize {
        (MAX_RECORD_SECS * (self.fs / self.decim as f64) as f32) as usize
    }

    fn beep_pattern_len_iq(&self) -> usize {
        let cycle_ms = BEEP_ON_MS + BEEP_OFF_MS;
        let total_ms = cycle_ms * BEEP_COUNT;
        ((self.fs * f64::from(total_ms) / 1000.0).round() as usize).max(1)
    }

    /// One PHY tick: RX always (keeps Soapy timestamps alive); TX only while replaying.
    pub fn tick(&mut self, sdr: &mut SoapyIo) -> Result<(), RxTxDevError> {
        if self.rx_scratch.len() != self.block_len {
            self.rx_scratch.resize(self.block_len, ComplexSample::ZERO);
        }

        let rx = match sdr.receive(&mut self.rx_scratch) {
            Ok(r) => r,
            Err(RxTxDevError::RxReadError) => {
                // Recoverable timeout/overflow — skip this tick.
                return Ok(());
            }
            Err(e) => return Err(e),
        };
        let n = rx.len.min(self.rx_scratch.len());
        if n == 0 {
            return Ok(());
        }
        self.stats_rx_blocks += 1;

        let (mag, audio_chunk) = self.demod_block_n(n);
        self.update_squelch(mag);

        match self.phase {
            Phase::Listen => {
                let cooled = self
                    .listen_after
                    .map(|t| Instant::now() >= t)
                    .unwrap_or(true);
                if self.carrier && cooled {
                    self.recording.clear();
                    self.hpf_x1 = 0.0;
                    self.hpf_y1 = 0.0;
                    self.afc_raw_sum = 0.0;
                    self.afc_raw_n = 0;
                    self.iq_dc = ComplexSample::ZERO;
                    self.iq_lp = ComplexSample::ZERO;
                    self.iq_lp_n = 0;
                    self.play_timeout_beep = false;
                    self.warn_logged = false;
                    self.phase = Phase::Record;
                    tracing::info!("PMR parrot: carrier — recording");
                    self.append_audio(&audio_chunk);
                }
            }
            Phase::Record => {
                self.append_audio(&audio_chunk);
                let warn_samp = self.warn_samples();
                let max_samp = self.max_samples();
                if !self.warn_logged && self.recording.len() >= warn_samp {
                    self.warn_logged = true;
                    self.play_timeout_beep = true;
                    tracing::warn!(
                        "PMR parrot: approaching max record ({:.0}/{:.0} s) — will beep-beep before replay",
                        WARN_RECORD_SECS,
                        MAX_RECORD_SECS
                    );
                }
                let force_end = self.recording.len() >= max_samp;
                if !self.carrier || force_end {
                    if force_end {
                        self.play_timeout_beep = true;
                        tracing::warn!(
                            "PMR parrot: max record length — forcing end ({} samples)",
                            self.recording.len()
                        );
                        self.carrier = false;
                    }
                    if self.recording.len() >= MIN_RECORD_SAMPLES {
                        self.finalize_recording_afc();
                        self.hang_deadline = Some(Instant::now() + self.hangtime);
                        self.phase = Phase::Hang;
                        let afc_hz = if self.afc_raw_n > 0 {
                            (self.afc_raw_sum / self.afc_raw_n as f64) * DEVIATION_HZ
                        } else {
                            0.0
                        };
                        tracing::info!(
                            "PMR parrot: unkey — hang {} ms ({} audio samples, rx_offset≈{:.0} Hz, timeout_beep={})",
                            self.hangtime.as_millis(),
                            self.recording.len(),
                            afc_hz,
                            self.play_timeout_beep
                        );
                    } else {
                        tracing::debug!("PMR parrot: drop short take ({} samples)", self.recording.len());
                        self.recording.clear();
                        self.phase = Phase::Listen;
                    }
                }
            }
            Phase::Hang => {
                // Ignore carriers while hanging (own TX not up yet; avoid noise takes).
                if self.hang_deadline.map(|d| Instant::now() >= d).unwrap_or(true) {
                    if self.recording.len() >= MIN_RECORD_SAMPLES && sdr.tx_possible() {
                        self.tx_phase = 0.0;
                        self.ctcss_phase = 0.0;
                        if self.play_timeout_beep {
                            self.beep_i = 0;
                            self.beep_len = self.beep_pattern_len_iq();
                            self.phase = Phase::TimeoutBeep;
                            tracing::warn!(
                                "PMR parrot: timeout beep-beep ({} IQ samples) then replay",
                                self.beep_len
                            );
                        } else {
                            self.replay_i = 0;
                            self.phase = Phase::Replay;
                            tracing::info!("PMR parrot: replaying {} samples", self.recording.len());
                        }
                    } else {
                        self.recording.clear();
                        self.phase = Phase::Listen;
                    }
                }
            }
            Phase::TimeoutBeep => {
                if !sdr.tx_possible() {
                    return Ok(());
                }
                self.modulate_beep_block(n);
                let count = tx_schedule_count(sdr, n)?;
                sdr.transmit(&self.tx_scratch[..n], Some(count))?;
                self.stats_tx_blocks += 1;
                self.beep_i = self.beep_i.saturating_add(n);
                if self.beep_i >= self.beep_len {
                    self.play_timeout_beep = false;
                    self.replay_i = 0;
                    self.tx_phase = 0.0;
                    self.phase = Phase::Replay;
                    tracing::info!("PMR parrot: replaying {} samples", self.recording.len());
                }
            }
            Phase::Replay => {
                if !sdr.tx_possible() {
                    // Wait for timestamp anchor.
                    return Ok(());
                }
                self.modulate_tx_block(n);
                let count = tx_schedule_count(sdr, n)?;
                sdr.transmit(&self.tx_scratch[..n], Some(count))?;
                self.stats_tx_blocks += 1;
                if self.replay_i >= self.recording.len() {
                    tracing::info!("PMR parrot: replay done — listening again");
                    self.recording.clear();
                    self.listen_after = Some(Instant::now() + POST_REPLAY_COOLDOWN);
                    self.carrier = false;
                    self.phase = Phase::Listen;
                }
            }
        }

        if self.last_log.elapsed() >= Duration::from_secs(5) {
            let mag_db = 20.0 * self.mag_ema.max(1e-9).log10();
            tracing::warn!(
                "PMR parrot: phase={} carrier={} rec={} rx={} tx={} mag={:.4} ({:.1} dB) noise={:.1} dB open≥{:.1}",
                self.phase_name(),
                self.carrier,
                self.recording.len(),
                self.stats_rx_blocks,
                self.stats_tx_blocks,
                self.mag_ema,
                mag_db,
                self.noise_db,
                self.noise_db + SQUELCH_OPEN_DB
            );
            self.last_log = Instant::now();
        }

        Ok(())
    }

    fn demod_block_n(&mut self, n: usize) -> (f32, Vec<f32>) {
        let mut mag_sum = 0.0f32;
        let mut out = Vec::with_capacity(n / self.decim + 1);
        // After IQ decimation, discriminator runs at ~AUDIO_FS.
        let audio_fs = self.fs / self.decim as f64;
        let scale = (audio_fs / (TAU as f64) / DEVIATION_HZ) as f32;
        let a = self.iq_lp_alpha;

        for i in 0..n {
            let mut s = self.rx_scratch[i];
            mag_sum += s.norm();
            // Strip LO/IQ DC leakage (common source of a fixed mid-band birdie).
            self.iq_dc.re += self.iq_dc_alpha * (s.re - self.iq_dc.re);
            self.iq_dc.im += self.iq_dc_alpha * (s.im - self.iq_dc.im);
            s.re -= self.iq_dc.re;
            s.im -= self.iq_dc.im;
            // Anti-alias: EMA low-pass on IQ, then demod at decimated rate.
            self.iq_lp.re += a * (s.re - self.iq_lp.re);
            self.iq_lp.im += a * (s.im - self.iq_lp.im);
            self.iq_lp_n += 1;
            if self.iq_lp_n < self.decim {
                continue;
            }
            self.iq_lp_n = 0;
            let s_lp = self.iq_lp;
            let prod = s_lp * self.prev.conj();
            let dphi = prod.im.atan2(prod.re);
            self.prev = s_lp;
            let raw = (dphi * scale).clamp(-1.5, 1.5);

            if self.phase == Phase::Record && self.afc_raw_n < AFC_ESTIMATE_AUDIO_SAMPLES {
                self.afc_raw_sum += f64::from(raw);
                self.afc_raw_n += 1;
            }
            let afc = if self.afc_raw_n > 0 {
                (self.afc_raw_sum / self.afc_raw_n as f64) as f32
            } else {
                0.0
            };
            let centered = raw - afc;
            let y = self.hpf_alpha * (self.hpf_y1 + centered - self.hpf_x1);
            self.hpf_x1 = centered;
            self.hpf_y1 = y;
            out.push(y.clamp(-1.0, 1.0));
        }
        let mag = mag_sum / n.max(1) as f32;
        (mag, out)
    }

    fn update_squelch(&mut self, mag: f32) {
        self.mag_ema = if self.mag_ema <= 0.0 {
            mag
        } else {
            0.85 * self.mag_ema + 0.15 * mag
        };
        let mag_db = 20.0 * self.mag_ema.max(1e-9).log10();

        // Warm-up: learn the real ambient floor before allowing opens.
        // On SXceiver ambient was ~-59 dB; starting at -80 made carrier stick open forever.
        if self.squelch_warmup < SQUELCH_WARMUP_BLOCKS {
            if self.squelch_warmup == 0 {
                self.noise_db = mag_db;
            } else if mag_db < self.noise_db {
                self.noise_db = 0.5 * self.noise_db + 0.5 * mag_db;
            } else {
                self.noise_db = 0.9 * self.noise_db + 0.1 * mag_db;
            }
            self.squelch_warmup += 1;
            if self.squelch_warmup == SQUELCH_WARMUP_BLOCKS {
                tracing::warn!(
                    "PMR parrot: squelch ready — ambient≈{:.1} dB (open≥{:.1})",
                    self.noise_db,
                    self.noise_db + SQUELCH_OPEN_DB
                );
            }
            self.carrier = false;
            return;
        }

        if !self.carrier {
            // Fast down / moderate up so ambient is tracked, not a stale -80 dB floor.
            if mag_db < self.noise_db {
                self.noise_db = 0.5 * self.noise_db + 0.5 * mag_db;
            } else {
                self.noise_db = 0.97 * self.noise_db + 0.03 * mag_db;
            }
        }
        let open_th = self.noise_db + SQUELCH_OPEN_DB;
        let close_th = self.noise_db + SQUELCH_CLOSE_DB;
        let was = self.carrier;
        if !self.carrier && mag_db >= open_th {
            self.carrier = true;
        } else if self.carrier && mag_db < close_th {
            self.carrier = false;
        }
        if self.carrier != was {
            tracing::warn!(
                "PMR parrot: carrier {} (mag={:.4} / {:.1} dB, noise={:.1} dB)",
                if self.carrier { "OPEN" } else { "CLOSE" },
                self.mag_ema,
                mag_db,
                self.noise_db
            );
        }
    }

    fn append_audio(&mut self, chunk: &[f32]) {
        self.recording.extend_from_slice(chunk);
    }

    /// Strip residual DC and any strong constant birdie before replay.
    fn finalize_recording_afc(&mut self) {
        if self.recording.is_empty() {
            return;
        }
        let mean = self.recording.iter().sum::<f32>() / self.recording.len() as f32;
        if mean.abs() >= 1e-6 {
            for s in &mut self.recording {
                *s = (*s - mean).clamp(-1.0, 1.0);
            }
        }
        if let Some(hz) = find_birdie_hz(&self.recording, self.fs / self.decim as f64) {
            tracing::warn!("PMR parrot: notching constant birdie ≈ {:.0} Hz before replay", hz);
            notch_inplace(&mut self.recording, self.fs / self.decim as f64, hz, BIRDIE_NOTCH_Q);
        }
    }

    fn modulate_beep_block(&mut self, n: usize) {
        if self.tx_scratch.len() < n {
            self.tx_scratch.resize(n, ComplexSample::ZERO);
        }
        // Peak deviation for a clear courtesy tone (~1.5 kHz).
        let tone_dev = TAU as f64 * 1_500.0 / self.fs;
        let on_iq = ((self.fs * f64::from(BEEP_ON_MS) / 1000.0).round() as usize).max(1);
        let off_iq = ((self.fs * f64::from(BEEP_OFF_MS) / 1000.0).round() as usize).max(1);
        let cycle = on_iq + off_iq;

        for i in 0..n {
            let pos = self.beep_i + i;
            let in_beep = (pos / cycle) < BEEP_COUNT as usize && (pos % cycle) < on_iq;
            if in_beep {
                let t = pos as f64 / self.fs;
                let audio = (TAU as f64 * BEEP_HZ * t).sin();
                self.tx_phase += tone_dev * audio;
                if self.tx_phase > TAU as f64 {
                    self.tx_phase -= TAU as f64;
                } else if self.tx_phase < -TAU as f64 {
                    self.tx_phase += TAU as f64;
                }
                self.tx_scratch[i] = ComplexSample::from_polar(TX_AMPLITUDE, self.tx_phase as f32);
            } else {
                self.tx_scratch[i] = ComplexSample::ZERO;
            }
        }
    }

    fn modulate_tx_block(&mut self, n: usize) {
        if self.tx_scratch.len() < n {
            self.tx_scratch.resize(n, ComplexSample::ZERO);
        }
        let dphi_dev = TAU as f64 * DEVIATION_HZ / self.fs;
        let ctcss = self.ctcss_hz as f64;
        let dphi_ctcss = if ctcss > 0.0 {
            TAU as f64 * ctcss / self.fs
        } else {
            0.0
        };
        // Audio sample duration in IQ samples.
        let audio_step = self.decim as f64;

        for i in 0..n {
            let audio_pos = self.replay_i as f64 + (i as f64) / audio_step;
            let ai = audio_pos as usize;
            let audio = if ai < self.recording.len() {
                self.recording[ai].clamp(-1.0, 1.0) as f64
            } else {
                0.0
            };
            let mut inst = dphi_dev * audio;
            if dphi_ctcss > 0.0 {
                // ~15% deviation for CTCSS (standard sub-audible level).
                inst += dphi_ctcss * 0.15 * (self.ctcss_phase).sin();
                self.ctcss_phase += dphi_ctcss;
                if self.ctcss_phase > TAU as f64 {
                    self.ctcss_phase -= TAU as f64;
                }
            }
            self.tx_phase += inst;
            if self.tx_phase > TAU as f64 {
                self.tx_phase -= TAU as f64;
            } else if self.tx_phase < -TAU as f64 {
                self.tx_phase += TAU as f64;
            }
            self.tx_scratch[i] = ComplexSample::from_polar(TX_AMPLITUDE, self.tx_phase as f32);
        }
        // Advance audio cursor by the block's worth of audio samples.
        let advance = (n + self.decim - 1) / self.decim;
        self.replay_i = self.replay_i.saturating_add(advance);
    }
}

fn tx_schedule_count(sdr: &SoapyIo, block_len: usize) -> Result<SampleCount, RxTxDevError> {
    // Schedule slightly ahead of "now" so the write is not late.
    let now = sdr.tx_current_count()?;
    Ok(now + block_len as SampleCount * 4)
}

/// Autocorr + Goertzel confirm for a stable mid-band birdie (user capture ≈ 623 Hz).
fn find_birdie_hz(audio: &[f32], audio_fs: f64) -> Option<f64> {
    if audio.len() < 128 || !(audio_fs.is_finite() && audio_fs > 0.0) {
        return None;
    }
    // Prefer early audio (tone is already present; less vowel bias than mid-take).
    let n = audio.len().min((audio_fs * 0.6) as usize).max(128);
    let slice = &audio[..n];
    let lo = BIRDIE_SEARCH_LO_HZ;
    let hi = BIRDIE_SEARCH_HI_HZ.min(audio_fs * 0.45);
    if hi <= lo + 20.0 {
        return None;
    }

    // Remove DC for a cleaner autocorr peak.
    let mean = slice.iter().map(|x| f64::from(*x)).sum::<f64>() / n as f64;
    let centered: Vec<f64> = slice.iter().map(|x| f64::from(*x) - mean).collect();
    let energy = centered.iter().map(|x| x * x).sum::<f64>() / n as f64;
    if energy < 1e-8 {
        return None;
    }

    let lag_lo = ((audio_fs / hi).floor() as usize).max(2);
    let lag_hi = ((audio_fs / lo).ceil() as usize).min(n / 3);
    if lag_hi <= lag_lo {
        return None;
    }

    let mut best_lag = lag_lo;
    let mut best_c = f64::NEG_INFINITY;
    for lag in lag_lo..=lag_hi {
        let mut c = 0.0;
        let mut i = 0;
        while i + lag < n {
            c += centered[i] * centered[i + lag];
            i += 1;
        }
        c /= (n - lag) as f64;
        if c > best_c {
            best_c = c;
            best_lag = lag;
        }
    }
    // Require a meaningful periodic component (vs broadband voice/noise).
    if best_c < energy * 0.12 {
        return None;
    }
    let f_ac = audio_fs / best_lag as f64;

    // Refine with Goertzel around the autocorr estimate; demand a clear spectral peak.
    let step = 2.0;
    let mut best_f = f_ac;
    let mut best_p = 0.0f64;
    let mut powers = Vec::new();
    let mut f = (f_ac - 40.0).max(lo);
    let f_end = (f_ac + 40.0).min(hi);
    while f <= f_end {
        let p = goertzel_power(slice, audio_fs, f);
        powers.push(p);
        if p > best_p {
            best_p = p;
            best_f = f;
        }
        f += step;
    }
    // Band median as reference (wider search).
    let mut band_powers = Vec::new();
    let mut f = lo;
    while f <= hi {
        band_powers.push(goertzel_power(slice, audio_fs, f));
        f += 10.0;
    }
    band_powers.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = band_powers[band_powers.len() / 2];
    if median <= 1e-18 || best_p < BIRDIE_PEAK_RATIO * median {
        return None;
    }
    Some(best_f)
}

fn goertzel_power(x: &[f32], fs: f64, freq: f64) -> f64 {
    let w = TAU as f64 * freq / fs;
    let coeff = 2.0 * w.cos();
    let mut s1 = 0.0;
    let mut s2 = 0.0;
    for &sample in x {
        let s0 = f64::from(sample) + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    // Standard Goertzel power (avoids unused-assignment on the last s0).
    s1 * s1 + s2 * s2 - s1 * s2 * coeff
}

/// In-place biquad notch (RBJ cookbook).
fn notch_inplace(audio: &mut [f32], audio_fs: f64, freq: f64, q: f64) {
    if audio.is_empty() || !(audio_fs.is_finite() && audio_fs > 0.0) {
        return;
    }
    let f0 = freq.clamp(50.0, audio_fs * 0.45);
    let q = q.clamp(1.0, 50.0);
    let w0 = TAU as f64 * f0 / audio_fs;
    let alpha = w0.sin() / (2.0 * q);
    let cos_w0 = w0.cos();
    let b0 = 1.0;
    let b1 = -2.0 * cos_w0;
    let b2 = 1.0;
    let a0 = 1.0 + alpha;
    let a1 = -2.0 * cos_w0;
    let a2 = 1.0 - alpha;
    let b0a = b0 / a0;
    let b1a = b1 / a0;
    let b2a = b2 / a0;
    let a1a = a1 / a0;
    let a2a = a2 / a0;
    let mut x1 = 0.0f64;
    let mut x2 = 0.0f64;
    let mut y1 = 0.0f64;
    let mut y2 = 0.0f64;
    for s in audio.iter_mut() {
        let x0 = f64::from(*s);
        let y0 = b0a * x0 + b1a * x1 + b2a * x2 - a1a * y1 - a2a * y2;
        x2 = x1;
        x1 = x0;
        y2 = y1;
        y1 = y0;
        *s = (y0 as f32).clamp(-1.0, 1.0);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parrot_defaults_from_pmr_config() {
        let cfg = CfgPmrRepeater::default();
        let p = PmrParrot::new(600_000.0, &cfg);
        assert_eq!(p.decim, 48);
        assert_eq!(p.phase, Phase::Listen);
        assert!(p.hangtime >= Duration::from_millis(100));
    }

    #[test]
    fn discriminator_sees_tone() {
        let cfg = CfgPmrRepeater::default();
        let mut p = PmrParrot::new(48_000.0, &cfg);
        // Generate a 1 kHz tone FM at 48 kHz IQ.
        let mut iq = Vec::new();
        let mut phase = 0.0f64;
        let tone = 1_000.0;
        for i in 0..4800 {
            let t = i as f64 / 48_000.0;
            let audio = (TAU as f64 * tone * t).sin();
            phase += TAU as f64 * DEVIATION_HZ * audio / 48_000.0;
            iq.push(ComplexSample::from_polar(0.5, phase as f32));
        }
        p.rx_scratch = iq;
        let (mag, audio) = p.demod_block_n(4800);
        assert!(mag > 0.1);
        assert!(audio.len() > 50);
        // Energy should be present in the demodulated audio.
        let energy: f32 = audio.iter().map(|x| x * x).sum::<f32>() / audio.len() as f32;
        assert!(energy > 0.01, "energy={energy}");
    }

    #[test]
    fn finds_and_notches_constant_birdie() {
        let fs = 12_500.0;
        let n = (fs * 0.5) as usize;
        let birdie = 623.0;
        let mut audio: Vec<f32> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                // Constant birdie + quieter voice-ish tone.
                (0.55 * (TAU as f64 * birdie * t).sin() + 0.15 * (TAU as f64 * 900.0 * t).sin())
                    as f32
            })
            .collect();
        let found = find_birdie_hz(&audio, fs).expect("birdie");
        assert!(
            (found - birdie).abs() < 10.0,
            "found={found} expected≈{birdie}"
        );
        let before = goertzel_power(&audio, fs, birdie);
        notch_inplace(&mut audio, fs, found, BIRDIE_NOTCH_Q);
        let after = goertzel_power(&audio, fs, birdie);
        assert!(
            after < before * 0.15,
            "before={before} after={after} — notch should crush birdie"
        );
    }
}
