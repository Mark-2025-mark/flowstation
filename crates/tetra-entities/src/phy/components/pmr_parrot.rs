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
/// Soft TX amplitude (full-scale would slam the SXceiver PA).
const TX_AMPLITUDE: f32 = 0.45;
/// Minimum recording length before a replay is worth airing (~150 ms).
const MIN_RECORD_SAMPLES: usize = (AUDIO_FS * 0.15) as usize;
/// Open squelch this many dB above the tracked noise floor.
const SQUELCH_OPEN_DB: f32 = 6.0;
/// Close squelch this many dB above the noise floor (hysteresis).
const SQUELCH_CLOSE_DB: f32 = 2.5;
/// IQ blocks to observe before allowing carrier (learn ambient floor).
const SQUELCH_WARMUP_BLOCKS: u32 = 400;
/// Audio high-pass (~300 Hz) removes discriminator DC — that DC remodulates as a
/// constant electrical beep under the voice.
const AUDIO_HPF_HZ: f64 = 300.0;
/// Ignore RX carrier this long after replay (own TX bleed into RX).
const POST_REPLAY_COOLDOWN: Duration = Duration::from_millis(750);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    Listen,
    Record,
    Hang,
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
    /// Decimation accumulator for audio.
    audio_acc: f32,
    audio_n: usize,
    /// 1-pole HPF state (DC block on demod audio).
    hpf_x1: f32,
    hpf_y1: f32,
    hpf_alpha: f32,
    /// Recorded baseband audio at AUDIO_FS.
    recording: Vec<f32>,
    hang_deadline: Option<Instant>,
    /// After replay, ignore carriers until this instant.
    listen_after: Option<Instant>,
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
        tracing::info!(
            "PMR parrot: fs={:.0} Hz audio≈{:.0} Hz (decim={}) hang={}ms ctcss={:.1} block={} hpf={:.0}Hz",
            fs,
            audio_fs,
            decim,
            hangtime.as_millis(),
            cfg.ctcss_hz,
            block_len,
            AUDIO_HPF_HZ
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
            audio_acc: 0.0,
            audio_n: 0,
            hpf_x1: 0.0,
            hpf_y1: 0.0,
            hpf_alpha,
            recording: Vec::with_capacity((AUDIO_FS * 8.0) as usize),
            hang_deadline: None,
            listen_after: None,
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
            Phase::Replay => "replay",
        }
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
                    self.audio_acc = 0.0;
                    self.audio_n = 0;
                    self.hpf_x1 = 0.0;
                    self.hpf_y1 = 0.0;
                    self.phase = Phase::Record;
                    tracing::info!("PMR parrot: carrier — recording");
                    self.append_audio(&audio_chunk);
                }
            }
            Phase::Record => {
                self.append_audio(&audio_chunk);
                let max_samp = (MAX_RECORD_SECS * (self.fs / self.decim as f64) as f32) as usize;
                let force_end = self.recording.len() >= max_samp;
                if !self.carrier || force_end {
                    if force_end {
                        tracing::warn!(
                            "PMR parrot: max record length — forcing end ({} samples)",
                            self.recording.len()
                        );
                        self.carrier = false;
                    }
                    if self.recording.len() >= MIN_RECORD_SAMPLES {
                        self.normalize_recording_dc();
                        self.hang_deadline = Some(Instant::now() + self.hangtime);
                        self.phase = Phase::Hang;
                        tracing::info!(
                            "PMR parrot: unkey — hang {} ms ({} audio samples)",
                            self.hangtime.as_millis(),
                            self.recording.len()
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
                        self.replay_i = 0;
                        self.tx_phase = 0.0;
                        self.ctcss_phase = 0.0;
                        self.phase = Phase::Replay;
                        tracing::info!("PMR parrot: replaying {} samples", self.recording.len());
                    } else {
                        self.recording.clear();
                        self.phase = Phase::Listen;
                    }
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
        let scale = (self.fs / (TAU as f64) / DEVIATION_HZ) as f32;

        for i in 0..n {
            let s = self.rx_scratch[i];
            mag_sum += s.norm();
            // Quadrature discriminator: Δφ between consecutive samples.
            let prod = s * self.prev.conj();
            let dphi = prod.im.atan2(prod.re);
            self.prev = s;
            let audio = (dphi * scale).clamp(-1.5, 1.5);
            self.audio_acc += audio;
            self.audio_n += 1;
            if self.audio_n >= self.decim {
                let raw = self.audio_acc / self.audio_n as f32;
                self.audio_acc = 0.0;
                self.audio_n = 0;
                // DC-block: without this, LO/discriminator bias becomes a constant beep on TX.
                let y = self.hpf_alpha * (self.hpf_y1 + raw - self.hpf_x1);
                self.hpf_x1 = raw;
                self.hpf_y1 = y;
                out.push(y.clamp(-1.0, 1.0));
            }
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

    /// Extra safety: strip residual DC from the whole take before replay.
    fn normalize_recording_dc(&mut self) {
        if self.recording.is_empty() {
            return;
        }
        let mean = self.recording.iter().sum::<f32>() / self.recording.len() as f32;
        if mean.abs() < 1e-6 {
            return;
        }
        for s in &mut self.recording {
            *s = (*s - mean).clamp(-1.0, 1.0);
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
}
