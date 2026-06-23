mod adpcm;
mod disk;
mod flash;
mod proto;

use std::path::PathBuf;

use anyhow::{bail, Context, Result};
use clap::{Parser, Subcommand};
use indicatif::{ProgressBar, ProgressStyle};

// ── CLI ───────────────────────────────────────────────────────────────────────

#[derive(Parser)]
#[command(name = "rome", about = "SP-1 toolkit — firmware flasher and song manager")]
struct Cli {
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand)]
enum Cmd {
    /// Flash firmware .bin to the device via the bootloader
    Flash {
        #[arg(short, long)]
        port: Option<String>,
        firmware: Option<PathBuf>,
        #[arg(short, long)]
        list: bool,
        #[arg(short, long)]
        verbose: bool,
    },

    /// Show device info (disk header + song list)
    Info {
        #[arg(short, long)]
        port: Option<String>,
    },

    /// Format the eMMC disk (clears all songs)
    Format {
        #[arg(short, long)]
        port: Option<String>,
        #[arg(long)]
        yes: bool,
    },

    /// Song management
    Song {
        #[command(subcommand)]
        cmd: SongCmd,
    },

    /// Dump the eMMC EXT_CSD register (diagnostics)
    Extcsd {
        #[arg(short, long)]
        port: Option<String>,
    },

    /// Read audio codec bring-up diagnostics (CS42L42 + TAS2505)
    Codec {
        #[arg(short, long)]
        port: Option<String>,
    },

    /// Feed-thread health (underrun recoveries, eMMC read times). Poll while playing.
    Audio {
        #[arg(short, long)]
        port: Option<String>,
        /// Sample N times, 1s apart, to watch counters climb
        #[arg(short, long, default_value_t = 1)]
        count: u32,
    },

    /// Dump a raw 512-byte eMMC block (diagnostics)
    Dump {
        #[arg(short, long)]
        port: Option<String>,
        /// Block address (decimal)
        block: u32,
    },

    /// Write+verify a test pattern to specific blocks (maps writable region). DESTRUCTIVE.
    Probe {
        #[arg(short, long)]
        port: Option<String>,
        /// Block addresses to probe (space separated)
        blocks: Vec<u32>,
    },

    /// Stress-write N consecutive blocks via CMD24 (no USB/block) to find cumulative-write wall. DESTRUCTIVE.
    Stress {
        #[arg(short, long)]
        port: Option<String>,
        count: u32,
    },

    /// Decode N blocks on the host (mirrors firmware) and print amplitude envelope
    Decode {
        #[arg(short, long)]
        port: Option<String>,
        /// First block address (decimal)
        start: u32,
        /// Number of blocks to decode
        count: u32,
    },
}

#[derive(Subcommand)]
enum SongCmd {
    /// Upload a song (4 stereo WAV stems → 8-channel IMA-ADPCM)
    Add {
        /// Serial port (auto-detected if omitted)
        #[arg(short, long)]
        port: Option<String>,

        /// Song name (max 23 chars)
        name: String,

        /// Stem 1 WAV (e.g. drums) — 48 kHz, 16-bit, stereo
        stem1: PathBuf,

        /// Stem 2 WAV (e.g. bass)
        stem2: PathBuf,

        /// Stem 3 WAV (e.g. vocals)
        stem3: PathBuf,

        /// Stem 4 WAV (e.g. other)
        stem4: PathBuf,
    },

    /// Remove a song from the library by catalog index
    Rm {
        #[arg(short, long)]
        port: Option<String>,
        /// Catalog index shown by `rome info`
        idx: u16,
    },

    /// List all songs on the device
    List {
        #[arg(short, long)]
        port: Option<String>,
    },
}

// ── Audio loading ─────────────────────────────────────────────────────────────

use symphonia::core::audio::SampleBuffer;
use symphonia::core::codecs::DecoderOptions;
use symphonia::core::errors::Error as SymphoniaError;
use symphonia::core::formats::FormatOptions;
use symphonia::core::io::MediaSourceStream;
use symphonia::core::meta::MetadataOptions;
use symphonia::core::probe::Hint;

/// Linear interpolation resample from src_rate to 48000 Hz.
fn resample_to_48k(samples: &[i16], src_rate: u32) -> Vec<i16> {
    const DST_RATE: u32 = 48000;
    if src_rate == DST_RATE { return samples.to_vec(); }
    let ratio = DST_RATE as f64 / src_rate as f64;
    let dst_len = (samples.len() as f64 * ratio).ceil() as usize;
    let mut out = Vec::with_capacity(dst_len);
    for i in 0..dst_len {
        let src_pos = i as f64 / ratio;
        let idx = src_pos as usize;
        let frac = src_pos - idx as f64;
        let s0 = samples.get(idx).copied().unwrap_or(0) as f64;
        let s1 = samples.get(idx + 1).copied().unwrap_or(0) as f64;
        let v = (s0 + frac * (s1 - s0)).round() as i32;
        out.push(v.clamp(-32768, 32767) as i16);
    }
    out
}

/// Load any audio file (WAV, FLAC, MP3, OGG…) and return (left, right) at 48 kHz.
/// Mono files are duplicated to stereo. Multi-channel files use ch 0 + ch 1.
fn load_audio_stereo(path: &PathBuf) -> Result<(Vec<i16>, Vec<i16>)> {
    let file = std::fs::File::open(path)
        .with_context(|| format!("cannot open {}", path.display()))?;
    let mss = MediaSourceStream::new(Box::new(file), Default::default());

    let mut hint = Hint::new();
    if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
        hint.with_extension(ext);
    }

    let probed = symphonia::default::get_probe()
        .format(&hint, mss, &FormatOptions::default(), &MetadataOptions::default())
        .with_context(|| format!("cannot probe {}", path.display()))?;

    let mut format = probed.format;
    let track = format.default_track()
        .ok_or_else(|| anyhow::anyhow!("{}: no audio track", path.display()))?;

    let sample_rate = track.codec_params.sample_rate
        .ok_or_else(|| anyhow::anyhow!("{}: unknown sample rate", path.display()))?;
    let n_channels = track.codec_params.channels
        .map(|c| c.count())
        .unwrap_or(2);
    let track_id = track.id;

    let mut decoder = symphonia::default::get_codecs()
        .make(&track.codec_params, &DecoderOptions::default())
        .with_context(|| format!("{}: unsupported codec", path.display()))?;

    let mut left_raw: Vec<i16> = Vec::new();
    let mut right_raw: Vec<i16> = Vec::new();

    loop {
        let packet = match format.next_packet() {
            Ok(p) => p,
            Err(SymphoniaError::IoError(e)) if e.kind() == std::io::ErrorKind::UnexpectedEof => break,
            Err(SymphoniaError::ResetRequired) => continue,
            Err(e) => return Err(e).context("decode error"),
        };
        if packet.track_id() != track_id { continue; }

        let decoded = match decoder.decode(&packet) {
            Ok(d) => d,
            Err(SymphoniaError::IoError(_)) => break,
            Err(SymphoniaError::DecodeError(_)) => continue,
            Err(e) => return Err(e).context("decode error"),
        };

        let spec = *decoded.spec();
        let mut buf: SampleBuffer<i16> = SampleBuffer::new(decoded.capacity() as u64, spec);
        buf.copy_interleaved_ref(decoded);
        let samples = buf.samples();

        let ch = n_channels.max(1);
        for frame in samples.chunks(ch) {
            left_raw.push(frame[0]);
            right_raw.push(if ch >= 2 { frame[1] } else { frame[0] });
        }
    }

    if left_raw.is_empty() {
        bail!("{}: decoded no samples", path.display());
    }

    if sample_rate != 48000 {
        eprintln!("    resampling {} Hz → 48000 Hz", sample_rate);
    }
    let left  = resample_to_48k(&left_raw, sample_rate);
    let right = resample_to_48k(&right_raw, sample_rate);
    Ok((left, right))
}

// ── Device helpers ────────────────────────────────────────────────────────────

fn open_dev(port: Option<&str>) -> Result<proto::DeviceConn> {
    match port {
        Some(p) => proto::DeviceConn::open(p),
        None    => proto::DeviceConn::open_auto(),
    }
}

fn progress_bar(total_blocks: u64) -> ProgressBar {
    let pb = ProgressBar::new(total_blocks * 512);
    pb.set_style(
        ProgressStyle::with_template(
            "  [{bar:40.cyan/blue}] {pos_block}/{len_block} blocks  {binary_bytes_per_sec}  eta {eta}"
        )
        .unwrap()
        .with_key("pos_block", |s: &indicatif::ProgressState, w: &mut dyn std::fmt::Write| {
            write!(w, "{}", s.pos() / 512).unwrap()
        })
        .with_key("len_block", |s: &indicatif::ProgressState, w: &mut dyn std::fmt::Write| {
            write!(w, "{}", s.len().unwrap_or(0) / 512).unwrap()
        })
        .progress_chars("=>-"),
    );
    pb
}

// ── Commands ──────────────────────────────────────────────────────────────────

fn cmd_info(port: Option<&str>) -> Result<()> {
    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;

    let raw = dev.disk_info()?;
    let hdr = disk::DiskHeader::from_block(&raw);

    if !hdr.is_valid() {
        println!("Disk: NOT FORMATTED");
        println!("  raw[0..16]: {:02X?}", &raw[0..16]);
        println!("  magic: 0x{:08X} (want 0x{:08X})", hdr.magic, disk::DISK_MAGIC);
        println!("  version: {} (want {})", hdr.version, disk::DISK_VERSION);
        println!("Run `rome format` to initialize.");
        return Ok(());
    }

    println!("Disk: v{}  songs: {}  next free block: {}",
        hdr.version, hdr.song_count, hdr.next_free_block);

    if hdr.song_count == 0 {
        println!("Songs: (none)");
        return Ok(());
    }

    let catalog = dev.catalog_read()?;
    let songs = disk::parse_catalog(&catalog);

    println!("Songs:");
    for (i, song) in songs.iter().enumerate().take(hdr.song_count as usize) {
        if song.is_free() {
            println!("  [{i:3}] (deleted)");
        } else {
            let secs = song.block_count as f64 * adpcm::SAMPLES_PER_BLOCK as f64 / 48000.0;
            println!("  [{i:3}] \"{}\"  {} blocks  ({:.0}m{:.0}s)",
                song.name_str(), song.block_count,
                (secs / 60.0).floor(), secs % 60.0);
        }
    }
    Ok(())
}

fn cmd_format(port: Option<&str>, confirmed: bool) -> Result<()> {
    if !confirmed {
        eprint!("This will erase all songs. Type 'yes' to confirm: ");
        let mut line = String::new();
        std::io::stdin().read_line(&mut line)?;
        if line.trim() != "yes" {
            bail!("aborted");
        }
    }
    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;
    dev.disk_format()?;
    eprintln!("rome: disk formatted");
    Ok(())
}

fn cmd_song_add(
    port: Option<&str>,
    name: &str,
    stems: [&PathBuf; 4],
) -> Result<()> {
    if name.len() > 23 {
        bail!("song name too long (max 23 chars)");
    }

    eprintln!("rome: loading stems...");
    let mut channel_pcm: [Vec<i16>; adpcm::CHANNELS] = std::array::from_fn(|_| Vec::new());

    for (stem_idx, path) in stems.iter().enumerate() {
        let (left, right) = load_audio_stereo(path)?;
        let n = left.len();
        // Pad all channels to same length
        let target = channel_pcm[0].len().max(n);
        for ch in channel_pcm.iter_mut() {
            ch.resize(target, 0);
        }
        let ch_l = stem_idx * 2;
        let ch_r = stem_idx * 2 + 1;
        channel_pcm[ch_l] = left;
        channel_pcm[ch_l].resize(target, 0);
        channel_pcm[ch_r] = right;
        channel_pcm[ch_r].resize(target, 0);
        eprintln!("  stem {}: {} ({} frames)", stem_idx + 1, path.display(), n);
    }

    // Pad all channels to the same length
    let max_len = channel_pcm.iter().map(|c| c.len()).max().unwrap_or(0);
    for ch in channel_pcm.iter_mut() {
        ch.resize(max_len, 0);
    }

    eprintln!("rome: encoding {} frames → 8ch IMA-ADPCM...", max_len);
    let blocks = adpcm::encode_8ch(&channel_pcm);
    let secs = max_len as f64 / 48000.0;
    eprintln!("rome: {} blocks, {:.0}m{:.0}s, {:.1} MB",
        blocks.len(), (secs / 60.0).floor(), secs % 60.0,
        blocks.len() as f64 * 512.0 / 1_048_576.0);

    // Bake decimated per-stem VU levels (4 bytes/decim), appended after audio.
    let levels = adpcm::bake_stem_levels(&blocks);
    let level_blocks = adpcm::pack_levels(&levels);
    eprintln!("rome: baked {} per-stem VU bytes → {} level blocks", levels.len(), level_blocks.len());
    let mut stream: Vec<[u8; 512]> = Vec::with_capacity(blocks.len() + level_blocks.len());
    stream.extend_from_slice(&blocks);
    stream.extend_from_slice(&level_blocks);

    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;

    let mut name_bytes = [0u8; 24];
    let nb = name.as_bytes();
    name_bytes[..nb.len().min(23)].copy_from_slice(&nb[..nb.len().min(23)]);

    eprintln!("rome: uploading \"{}\" ({} blocks)...", name, blocks.len());
    let song_idx = dev.song_begin(&name_bytes, blocks.len() as u32, level_blocks.len() as u32)?;

    const BATCH: usize = 96;
    let pb = progress_bar(stream.len() as u64);
    let mut sent = 0usize;
    for chunk in stream.chunks(BATCH) {
        if let Err(e) = dev.song_multiblock(chunk) {
            pb.finish_and_clear();
            eprintln!("rome: FAILED after {} blocks ok; failing batch = blocks [{}..{}]",
                      sent, sent, sent + chunk.len());
            return Err(e);
        }
        sent += chunk.len();
        pb.inc(chunk.len() as u64 * 512);
    }
    pb.finish_and_clear();

    dev.song_commit()?;
    eprintln!("rome: upload complete — catalog index {song_idx}");
    Ok(())
}

fn cmd_song_rm(port: Option<&str>, idx: u16) -> Result<()> {
    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;
    dev.song_remove(idx)?;
    eprintln!("rome: removed song {idx}");
    Ok(())
}

fn cmd_song_list(port: Option<&str>) -> Result<()> {
    cmd_info(port)
}

fn cmd_codec(port: Option<&str>) -> Result<()> {
    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;
    let d = dev.codec_diag()?;

    let ok = |b: u8| if b != 0 { "yes" } else { "NO" };
    println!("Codec bring-up diagnostics:");
    println!("  init_ok            = {}", ok(d[0]));
    println!("  i2c_errors         = {}", d[12]);
    println!();
    println!("  CS42L42 (headphone codec + LRCLK source):");
    println!("    DEVID AB         = 0x{:02X} (expect 0x42)  {}", d[1],
             if d[1] == 0x42 { "OK" } else { "MISMATCH" });
    println!("    DEVID CD         = 0x{:02X} (expect 0xA4)  {}", d[2],
             if d[2] == 0xA4 { "OK" } else { "MISMATCH" });
    println!("    DEVID E          = 0x{:02X}", d[3]);
    println!("    PLL_LOCK_STATUS  = 0x{:02X}  ({})", d[4],
             if d[4] & 1 != 0 { "LOCKED" } else { "not locked" });
    println!("    CLOCK_CTL [1007] = 0x{:02X}", d[5]);
    println!("    PWR_CTL1  [1101] = 0x{:02X}", d[6]);
    println!("    HP_CTL    [2001] = 0x{:02X}", d[7]);
    println!();
    println!("  TAS2505 (speaker amp) — at init:");
    println!("    P0/R2            = 0x{:02X}", d[8]);
    println!("    P1/R46 (spk vol) = 0x{:02X}", d[9]);
    println!("    P1/R45 (amp ctl) = 0x{:02X}", d[10]);
    println!("    P0/R63 (dac)     = 0x{:02X}", d[11]);
    println!();
    println!("  AUDIO init:");
    println!("    stage reached    = {} (1=enter 2=dev-not-ready 3=after-i2s-cfg 4=done)", d[21]);
    println!("    i2s_configure ret= {}", d[22] as i8);
    println!();
    println!("  LIVE now:");
    println!("    audio_running    = {}", ok(d[13]));
    println!("    CS42L42 PLL lock = 0x{:02X}  ({})", d[14],
             if d[14] & 1 != 0 { "LOCKED" } else { "NOT LOCKED" });
    println!("    TAS P1/R46 vol   = 0x{:02X}  (0x00=max 0x7F=mute)", d[15]);
    println!("    TAS P1/R45 amp   = 0x{:02X}  (expect 0x02 unmuted)", d[16]);
    println!("    TAS P0/R63 dac   = 0x{:02X}  (expect 0xB0 on)", d[17]);
    println!("    TAS P0/R64 dacv  = 0x{:02X}", d[18]);
    println!("    TAS P0/R65 lvol  = 0x{:02X}", d[19]);
    println!("    TAS P0/R25 clk   = 0x{:02X}", d[20]);
    println!("    CS osc-switch    = 0x{:02X}  ({})", d[23],
             if d[23] == 0x02 { "SCLK→MCLK OK" } else { "NOT switched (DAC on RCO → clicks)" });
    println!("    CS HP_CTL now    = 0x{:02X}  (0x01=unmuted 0x0D=muted)", d[24]);
    let cur_block = u32::from_le_bytes([d[23], d[24], d[25], d[26]]);
    let read_us = u32::from_le_bytes([d[27], d[28], d[29], d[30]]);
    println!("    audio cur_block  = {} (relative; loud audio starts ~block 26)", cur_block);
    println!("    eMMC read/block  = {} µs (budget 2670 µs/block for realtime)", read_us);
    let ain1 = u16::from_le_bytes([d[29], d[30]]);
    println!("    AIN1 ladder raw  = {} (vol+/vol-/fwd/rwd; hold one while running)", ain1);
    let ul_block = u32::from_le_bytes([d[23], d[24], d[25], d[26]]);
    let ul_fail = d[28];
    let fail_str = match ul_fail { 0 => "none", 1 => "USB-READ-TIMEOUT", 2 => "WRITE-FAIL", _ => "?" };
    println!("    upload fail      = block {} ({})", ul_block, fail_str);
    let busy_us = u16::from_le_bytes([d[29], d[30]]);
    println!("    last-block busy  = {} µs (NAND program time per block)", busy_us);
    Ok(())
}

fn cmd_audio(port: Option<&str>, count: u32) -> Result<()> {
    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;
    println!("{:>10} {:>11} {:>11} {:>11} {:>10} {:>10} {:>10}",
        "recover", "write_fail", "max_rd_us", "last_rd_us", "cur_blk", "blks_fed", "crc_err");
    for i in 0..count.max(1) {
        let d = dev.audio_diag()?;
        println!("{:>10} {:>11} {:>11} {:>11} {:>10} {:>10} {:>10}",
            d[0], d[1], d[2], d[3], d[4], d[5], d[6]);
        if i + 1 < count {
            std::thread::sleep(std::time::Duration::from_secs(1));
        }
    }
    println!("\nbudget: max_rd_us must stay < 2670 for realtime. recover/write_fail > 0 = TX underran.");
    Ok(())
}

fn cmd_extcsd(port: Option<&str>) -> Result<()> {
    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;
    let e = dev.extcsd_dump()?;

    let cache_size = u32::from_le_bytes([e[168], e[169], e[170], e[171]]);
    let sec_count = u32::from_le_bytes([e[212], e[213], e[214], e[215]]);
    println!("EXT_CSD key fields:");
    println!("  [212] SEC_COUNT       = {} blocks = {:.1} MB ({:.2} GB)  [fail block 18793]",
             sec_count, sec_count as f64 * 512.0 / 1e6, sec_count as f64 * 512.0 / 1e9);
    println!("  [33]  CACHE_CTRL      = 0x{:02X} (cache {})",
             e[33], if e[33] & 1 != 0 { "ON" } else { "off" });
    println!("  [166] WR_REL_PARAM    = 0x{:02X} (EN_REL_WR={}, HS_CTRL_REL={})",
             e[166], (e[166] >> 2) & 1, e[166] & 1);
    println!("  [167] WR_REL_SET      = 0x{:02X} (user-area reliable-write {})",
             e[167], if e[167] & 1 != 0 { "ON" } else { "off" });
    println!("  [168] CACHE_SIZE      = {} KiB ({} bytes)", cache_size, cache_size * 1024);
    println!("  [181] CACHE_FLUSH_PLCY (BKOPS)?");
    println!("  [183] BUS_WIDTH       = 0x{:02X}", e[183]);
    println!("  [185] HS_TIMING       = 0x{:02X}", e[185]);
    println!("  [196] DEVICE_TYPE     = 0x{:02X}", e[196]);
    println!("  [231] SEC_FEATURE_SUPPORT = 0x{:02X}", e[231]);
    println!("  [232] TRIM_MULT       = {}", e[232]);
    println!();
    print!("full 512-byte dump:");
    for (i, b) in e.iter().enumerate() {
        if i % 16 == 0 { print!("\n  {i:3}: "); }
        print!("{b:02X} ");
    }
    println!();
    Ok(())
}

// ── Entry point ───────────────────────────────────────────────────────────────

fn main() -> Result<()> {
    let cli = Cli::parse();

    match cli.cmd {
        Cmd::Flash { port, firmware, list, verbose } => {
            flash::run(port.as_deref(), firmware.as_deref(), list, verbose)
        }
        Cmd::Info   { port }       => cmd_info(port.as_deref()),
        Cmd::Format { port, yes }  => cmd_format(port.as_deref(), yes),
        Cmd::Song   { cmd }        => match cmd {
            SongCmd::Add { port, name, stem1, stem2, stem3, stem4 } => {
                cmd_song_add(port.as_deref(), &name, [&stem1, &stem2, &stem3, &stem4])
            }
            SongCmd::Rm   { port, idx } => cmd_song_rm(port.as_deref(), idx),
            SongCmd::List { port }      => cmd_song_list(port.as_deref()),
        },
        Cmd::Extcsd { port }       => cmd_extcsd(port.as_deref()),
        Cmd::Codec  { port }       => cmd_codec(port.as_deref()),
        Cmd::Audio  { port, count } => cmd_audio(port.as_deref(), count),
        Cmd::Dump   { port, block } => cmd_dump(port.as_deref(), block),
        Cmd::Decode { port, start, count } => cmd_decode(port.as_deref(), start, count),
        Cmd::Probe  { port, blocks } => cmd_probe(port.as_deref(), &blocks),
        Cmd::Stress { port, count } => cmd_stress(port.as_deref(), count),
    }
}

fn cmd_stress(port: Option<&str>, count: u32) -> Result<()> {
    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;
    eprintln!("rome: stress-writing {count} blocks (CMD24) from block 9...");
    let ff = dev.write_stress(count)?;
    if ff == 0xFFFFFFFF {
        println!("all {count} blocks written ok");
    } else {
        println!("FIRST FAILURE at block index {} (absolute eMMC block {})", ff, ff + 9);
    }
    Ok(())
}

fn cmd_probe(port: Option<&str>, blocks: &[u32]) -> Result<()> {
    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;
    for &b in blocks {
        let r = dev.write_probe(b)?;
        let s = match r { 0 => "WRITE-FAIL", 1 => "readback-fail", 2 => "MISMATCH", 3 => "ok", _ => "?" };
        println!("block {:>8}: {} ({})", b, r, s);
    }
    Ok(())
}

fn cmd_decode(port: Option<&str>, start: u32, count: u32) -> Result<()> {
    use adpcm::STEP_TABLE;
    const INDEX_TABLE: [i32; 16] = [-1,-1,-1,-1,2,4,6,8,-1,-1,-1,-1,2,4,6,8];
    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;

    // 8 channel decoder states (predictor, step_index), carried across blocks.
    let mut pred = [0i32; 8];
    let mut sidx = [0i32; 8];
    let step = |s: &mut i32, p: &mut i32, n: u8| -> i32 {
        let st = STEP_TABLE[*s as usize];
        let mut d = st >> 3;
        if n & 4 != 0 { d += st; }
        if n & 2 != 0 { d += st >> 1; }
        if n & 1 != 0 { d += st >> 2; }
        if n & 8 != 0 { d = -d; }
        *p = (*p + d).clamp(-32768, 32767);
        *s = (*s + INDEX_TABLE[(n & 0xF) as usize]).clamp(0, 88);
        *p
    };

    println!("block  Lpeak  Rpeak  idx(8ch)");
    for blk in 0..count {
        let b = dev.read_block(start + blk)?;
        let (mut lpk, mut rpk) = (0i32, 0i32);
        for i in 0..128usize {
            let (mut l, mut r) = (0i32, 0i32);
            for stem in 0..4 {
                let bl = b[stem * 128 + i / 2];
                let br = b[stem * 128 + 64 + i / 2];
                let nl = if i & 1 == 1 { bl >> 4 } else { bl & 0xF };
                let nr = if i & 1 == 1 { br >> 4 } else { br & 0xF };
                l += step(&mut sidx[stem*2],   &mut pred[stem*2],   nl);
                r += step(&mut sidx[stem*2+1], &mut pred[stem*2+1], nr);
            }
            lpk = lpk.max((l >> 1).abs());
            rpk = rpk.max((r >> 1).abs());
        }
        println!("{:5}  {:5}  {:5}  {:?}", start + blk, lpk, rpk, sidx);
    }
    Ok(())
}

fn cmd_dump(port: Option<&str>, block: u32) -> Result<()> {
    let mut dev = open_dev(port)?;
    dev.ping().context("ping failed")?;
    let b = dev.read_block(block)?;

    // Entropy/stats: ADPCM audio data should be high-entropy and non-zero.
    let zeros = b.iter().filter(|&&x| x == 0).count();
    let ff = b.iter().filter(|&&x| x == 0xFF).count();
    let sum: u32 = b.iter().map(|&x| x as u32).sum();
    println!("Block {block}: {} zero bytes, {} 0xFF bytes, byte-avg {:.1}",
             zeros, ff, sum as f64 / 512.0);
    println!();
    // Per-stem nonzero stats (each stem = 128 bytes: 64 L + 64 R).
    for stem in 0..4 {
        let s = &b[stem * 128..stem * 128 + 128];
        let nz = s.iter().filter(|&&x| x != 0).count();
        let avg: f64 = s.iter().map(|&x| x as f64).sum::<f64>() / 128.0;
        println!("  stem{stem} (bytes {:3}..{:3}): {:3} nonzero, avg {:.1}",
                 stem * 128, stem * 128 + 128, nz, avg);
    }
    println!();
    for (i, chunk) in b.chunks(16).enumerate() {
        print!("{:04X}: ", i * 16);
        for byte in chunk { print!("{:02X} ", byte); }
        println!();
    }
    Ok(())
}
