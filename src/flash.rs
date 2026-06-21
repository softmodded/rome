use std::io::{Read, Write};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use indicatif::{ProgressBar, ProgressStyle};

const FW_PAGE_SIZE: usize = 4096;
const FW_CHUNK_SIZE: usize = 240;
const FLASH_START: u32 = 0x0002_0000;
const FLASH_END: u32 = 0x000F_F000;
const FIRST_PAGE: u32 = 0x20;
const BAUD_RATE: u32 = 115_200;

// ── CRC-8 ────────────────────────────────────────────────────────────────────

#[rustfmt::skip]
const CRC8_TABLE: [u8; 256] = [
    0xea,0xd4,0x96,0xa8,0x12,0x2c,0x6e,0x50,0x7f,0x41,0x03,0x3d,0x87,0xb9,0xfb,0xc5,
    0xa5,0x9b,0xd9,0xe7,0x5d,0x63,0x21,0x1f,0x30,0x0e,0x4c,0x72,0xc8,0xf6,0xb4,0x8a,
    0x74,0x4a,0x08,0x36,0x8c,0xb2,0xf0,0xce,0xe1,0xdf,0x9d,0xa3,0x19,0x27,0x65,0x5b,
    0x3b,0x05,0x47,0x79,0xc3,0xfd,0xbf,0x81,0xae,0x90,0xd2,0xec,0x56,0x68,0x2a,0x14,
    0xb3,0x8d,0xcf,0xf1,0x4b,0x75,0x37,0x09,0x26,0x18,0x5a,0x64,0xde,0xe0,0xa2,0x9c,
    0xfc,0xc2,0x80,0xbe,0x04,0x3a,0x78,0x46,0x69,0x57,0x15,0x2b,0x91,0xaf,0xed,0xd3,
    0x2d,0x13,0x51,0x6f,0xd5,0xeb,0xa9,0x97,0xb8,0x86,0xc4,0xfa,0x40,0x7e,0x3c,0x02,
    0x62,0x5c,0x1e,0x20,0x9a,0xa4,0xe6,0xd8,0xf7,0xc9,0x8b,0xb5,0x0f,0x31,0x73,0x4d,
    0x58,0x66,0x24,0x1a,0xa0,0x9e,0xdc,0xe2,0xcd,0xf3,0xb1,0x8f,0x35,0x0b,0x49,0x77,
    0x17,0x29,0x6b,0x55,0xef,0xd1,0x93,0xad,0x82,0xbc,0xfe,0xc0,0x7a,0x44,0x06,0x38,
    0xc6,0xf8,0xba,0x84,0x3e,0x00,0x42,0x7c,0x53,0x6d,0x2f,0x11,0xab,0x95,0xd7,0xe9,
    0x89,0xb7,0xf5,0xcb,0x71,0x4f,0x0d,0x33,0x1c,0x22,0x60,0x5e,0xe4,0xda,0x98,0xa6,
    0x01,0x3f,0x7d,0x43,0xf9,0xc7,0x85,0xbb,0x94,0xaa,0xe8,0xd6,0x6c,0x52,0x10,0x2e,
    0x4e,0x70,0x32,0x0c,0xb6,0x88,0xca,0xf4,0xdb,0xe5,0xa7,0x99,0x23,0x1d,0x5f,0x61,
    0x9f,0xa1,0xe3,0xdd,0x67,0x59,0x1b,0x25,0x0a,0x34,0x76,0x48,0xf2,0xcc,0x8e,0xb0,
    0xd0,0xee,0xac,0x92,0x28,0x16,0x54,0x6a,0x45,0x7b,0x39,0x07,0xbd,0x83,0xc1,0xff,
];

fn crc8(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |crc, &b| CRC8_TABLE[(crc ^ b) as usize])
}

// ── COBS ─────────────────────────────────────────────────────────────────────

fn cobs_encode(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(data.len() + 2);
    let mut idx = 0;
    loop {
        let run = data[idx..].iter().take_while(|&&b| b != 0).count();
        let end = idx + run;
        out.push((run + 1) as u8);
        out.extend_from_slice(&data[idx..end]);
        if end < data.len() {
            idx = end + 1;
        } else {
            break;
        }
    }
    out.push(0x00);
    out
}

fn cobs_decode(raw: &[u8]) -> Vec<u8> {
    let data = if raw.last() == Some(&0) { &raw[..raw.len() - 1] } else { raw };
    let mut out = Vec::new();
    let mut idx = 0;
    while idx < data.len() {
        let code = data[idx] as usize;
        idx += 1;
        if code == 0 { break; }
        let end = (idx + code - 1).min(data.len());
        out.extend_from_slice(&data[idx..end]);
        idx = end;
        if code < 0xFF && idx < data.len() { out.push(0); }
    }
    out
}

// ── Packet framing ────────────────────────────────────────────────────────────

pub struct Response {
    pub cmd: u8,
    pub payload: Vec<u8>,
    pub crc_ok: bool,
}

fn build_packet(cmd: u8, payload: &[u8], seq: u8) -> Vec<u8> {
    let mut body = vec![0x51, seq, cmd, payload.len() as u8];
    body.extend_from_slice(payload);
    body.push(crc8(&body));
    cobs_encode(&body)
}

fn parse_response(data: &[u8]) -> Option<Response> {
    let decoded = cobs_decode(data);
    if decoded.len() < 5 { return None; }
    let cmd = decoded[2];
    let plen = decoded[3] as usize;
    if decoded.len() < 5 + plen { return None; }
    let payload = decoded[4..4 + plen].to_vec();
    let crc_ok = decoded[4 + plen] == crc8(&decoded[..4 + plen]);
    Some(Response { cmd, payload, crc_ok })
}

// ── Serial I/O ───────────────────────────────────────────────────────────────

pub struct Flasher {
    pub port: Box<dyn serialport::SerialPort>,
    rx_buf: Vec<u8>,
}

impl Flasher {
    pub fn new(port: Box<dyn serialport::SerialPort>) -> Self {
        Self { port, rx_buf: Vec::new() }
    }

    fn drain(&mut self) { self.rx_buf.clear(); }

    pub fn read_until_zero(&mut self, timeout: Duration) -> Result<Option<Vec<u8>>> {
        let deadline = Instant::now() + timeout;
        let mut tmp = [0u8; 64];
        loop {
            if let Some(pos) = self.rx_buf.iter().position(|&b| b == 0) {
                return Ok(Some(self.rx_buf.drain(..=pos).collect()));
            }
            if Instant::now() >= deadline { return Ok(None); }
            let rem = deadline.saturating_duration_since(Instant::now());
            self.port.set_timeout(rem.min(Duration::from_millis(50)))?;
            match self.port.read(&mut tmp) {
                Ok(0) => {}
                Ok(n) => self.rx_buf.extend_from_slice(&tmp[..n]),
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                Err(e) => return Err(e).context("serial read"),
            }
        }
    }

    pub fn send_cmd(
        &mut self,
        cmd: u8,
        payload: &[u8],
        seq: u8,
        timeout: Duration,
    ) -> Result<Option<Response>> {
        self.drain();
        self.port.write_all(&build_packet(cmd, payload, seq)).context("serial write")?;
        Ok(self.read_until_zero(timeout)?.and_then(|r| parse_response(&r)))
    }

    pub fn send_no_reply(&mut self, cmd: u8, payload: &[u8], seq: u8) -> Result<()> {
        self.port.write_all(&build_packet(cmd, payload, seq)).context("serial write")
    }
}

pub fn open_port(name: &str) -> Result<Box<dyn serialport::SerialPort>> {
    serialport::new(name, BAUD_RATE)
        .timeout(Duration::from_millis(50))
        .open()
        .with_context(|| format!("failed to open {name}"))
}

// ── Flash sequence ────────────────────────────────────────────────────────────

macro_rules! vlog {
    ($v:expr, $pb:expr, $($arg:tt)*) => {
        if $v { $pb.println(format!($($arg)*)); }
    };
}

fn ensure_bootloader(
    port_name: &str,
    verbose: bool,
    pb: &ProgressBar,
    seq: &mut u8,
) -> Result<Flasher> {
    pb.set_message("checking device...");
    vlog!(verbose, pb, "phase 0: status (R)");

    let mut f = Flasher::new(open_port(port_name)?);
    let r0 = f
        .send_cmd(0x52, &[], *seq, Duration::from_millis(3000))?
        .context("device did not respond to R. check connection and bootloader mode.")?;
    *seq = seq.wrapping_add(1);

    if !r0.crc_ok || r0.cmd != 0x53 {
        bail!("bad status response cmd={:#04x} crc_ok={}", r0.cmd, r0.crc_ok);
    }
    if verbose && r0.payload.len() >= 4 {
        let pages = u32::from_le_bytes(r0.payload[..4].try_into().unwrap());
        vlog!(verbose, pb, "  pages: {pages}");
    }

    let payload_str: String = r0.payload.iter().map(|b| b.to_string()).collect();
    if payload_str != "10510" {
        return Ok(f);
    }

    pb.set_message("resetting to bootloader...");
    vlog!(verbose, pb, "  device in transfer mode, sending Q");
    let _ = f.send_no_reply(0x51, &[], *seq);
    *seq = seq.wrapping_add(1);
    drop(f);

    pb.set_message("waiting for device...");
    for attempt in 0..20 {
        thread::sleep(Duration::from_millis(200));
        match open_port(port_name) {
            Ok(port) => return Ok(Flasher::new(port)),
            Err(_) => vlog!(verbose, pb, "  waiting... attempt {}", attempt + 1),
        }
    }
    bail!("device did not re-enumerate after reset");
}

fn flash_fw(port_name: &str, fw: &[u8], verbose: bool, pb: &ProgressBar) -> Result<()> {
    let fw_size = fw.len();
    let num_pages = fw_size.div_ceil(FW_PAGE_SIZE);
    let mut seq: u8 = 1;

    let mut f = ensure_bootloader(port_name, verbose, pb, &mut seq)?;

    pb.set_message("erasing...");
    pb.set_position(2);
    vlog!(verbose, pb, "phase 1: format (F)");
    let r1 = f
        .send_cmd(0x46, &[], seq, Duration::from_millis(5000))?
        .context("no response to format (F)")?;
    seq = seq.wrapping_add(1);
    if !r1.crc_ok || r1.cmd != 0x47 {
        bail!("format failed cmd={:#04x} crc_ok={}", r1.cmd, r1.crc_ok);
    }
    vlog!(verbose, pb, "  format ok");

    let total_chunks: usize = (0..num_pages)
        .map(|pg| {
            let page_len = FW_PAGE_SIZE.min(fw_size - pg * FW_PAGE_SIZE);
            page_len.div_ceil(FW_CHUNK_SIZE)
        })
        .sum();
    vlog!(verbose, pb, "phase 2: writing {fw_size} bytes, {num_pages} pages, {total_chunks} chunks");

    let mut e_seq: u32 = 0;
    let mut chunks_sent: usize = 0;
    for pg_idx in 0..num_pages {
        let page_base = pg_idx * FW_PAGE_SIZE;
        let page_len = FW_PAGE_SIZE.min(fw_size - page_base);
        let mut offset = 0;
        while offset < page_len {
            e_seq += 1;
            let chunk_len = FW_CHUNK_SIZE.min(page_len - offset);
            let addr = FLASH_START + page_base as u32 + offset as u32;
            let chunk = &fw[page_base + offset..page_base + offset + chunk_len];
            let mut payload = Vec::with_capacity(8 + chunk_len);
            payload.extend_from_slice(&e_seq.to_le_bytes());
            payload.extend_from_slice(&addr.to_le_bytes());
            payload.extend_from_slice(chunk);
            f.send_no_reply(0x45, &payload, seq)?;
            seq = seq.wrapping_add(1);
            chunks_sent += 1;
            let pct = 5 + chunks_sent * 85 / total_chunks;
            pb.set_position(pct as u64);
            pb.set_message(format!("writing {}/{}", pg_idx + 1, num_pages));
            if offset == 0 && pg_idx > 0 {
                thread::sleep(Duration::from_millis(100));
            } else {
                thread::sleep(Duration::from_millis(5));
            }
            offset += FW_CHUNK_SIZE;
        }
    }
    vlog!(verbose, pb, "  sent {e_seq} chunks");

    pb.set_message("committing...");
    pb.set_position(92);
    vlog!(verbose, pb, "phase 3: commit (H)");
    thread::sleep(Duration::from_millis(150));
    let last_page = FIRST_PAGE + num_pages as u32 - 1;
    let r3 = f
        .send_cmd(0x48, &last_page.to_le_bytes(), seq, Duration::from_millis(5000))?
        .context("no response to commit (H)")?;
    seq = seq.wrapping_add(1);
    if !r3.crc_ok || r3.cmd != 0x49 {
        bail!("commit failed cmd={:#04x} crc_ok={}", r3.cmd, r3.crc_ok);
    }
    let counter = if r3.payload.len() >= 4 {
        u32::from_le_bytes(r3.payload[..4].try_into().unwrap())
    } else {
        e_seq
    };
    vlog!(verbose, pb, "  counter: {counter}");

    pb.set_message("finalizing...");
    pb.set_position(96);
    vlog!(verbose, pb, "phase 4: finalize (H)");
    let r4 = f
        .send_cmd(0x48, &counter.to_le_bytes(), seq, Duration::from_millis(5000))?
        .context("no response to finalize (H)")?;
    seq = seq.wrapping_add(1);
    if !r4.crc_ok { bail!("finalize CRC error"); }
    if r4.cmd != 0x49 { bail!("unexpected finalize response: {:#04x}", r4.cmd); }
    if verbose && r4.payload.len() >= 4 {
        let fc = u32::from_le_bytes(r4.payload[..4].try_into().unwrap());
        vlog!(verbose, pb, "  finalize counter: {fc}");
    }

    let _ = f.send_cmd(0x50, &[], seq, Duration::from_millis(3000));
    pb.set_position(100);
    Ok(())
}

// ── Public entry point ────────────────────────────────────────────────────────

pub fn run(
    port: Option<&str>,
    firmware: Option<&std::path::Path>,
    list: bool,
    verbose: bool,
) -> Result<()> {
    if list {
        let ports = serialport::available_ports().context("failed to enumerate ports")?;
        if ports.is_empty() {
            println!("no serial ports found");
        } else {
            for p in &ports { println!("{}", p.port_name); }
        }
        return Ok(());
    }

    let port_name = port.context("--port required. run with --list to see available ports.")?;
    let fw_path: &std::path::Path = firmware.context("firmware path required")?;
    let fw = std::fs::read(fw_path)
        .with_context(|| format!("failed to read {}", fw_path.display()))?;

    let max_size = (FLASH_END - FLASH_START) as usize;
    if fw.is_empty() { bail!("firmware file is empty"); }
    if fw.len() > max_size {
        bail!(
            "firmware too large: {} bytes (max {} bytes / {:.0} KB)",
            fw.len(), max_size, max_size as f64 / 1024.0
        );
    }

    let num_pages = fw.len().div_ceil(FW_PAGE_SIZE);
    eprintln!(
        "rome: {} ({:.1} KB, {} pages) → {}",
        fw_path.display(),
        fw.len() as f64 / 1024.0,
        num_pages,
        port_name
    );

    let pb = ProgressBar::new(100);
    pb.set_style(
        ProgressStyle::with_template("{msg:26} [{bar:40.cyan/blue}] {pos:>3}%")
            .unwrap()
            .progress_chars("=>-"),
    );

    match flash_fw(port_name, &fw, verbose, &pb) {
        Ok(()) => {
            pb.finish_with_message("flash complete");
            eprintln!("device will restart now.");
            Ok(())
        }
        Err(e) => {
            pb.abandon_with_message("failed");
            Err(e)
        }
    }
}

// ── Tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc8_empty() { assert_eq!(crc8(&[]), 0); }

    #[test]
    fn cobs_roundtrip_with_zero() {
        let data = [0x51, 0x01, 0x52, 0x00u8];
        let encoded = cobs_encode(&data);
        assert!(!encoded[..encoded.len() - 1].contains(&0));
        assert_eq!(encoded.last(), Some(&0));
        assert_eq!(cobs_decode(&encoded), data);
    }

    #[test]
    fn cobs_roundtrip_all_nonzero() {
        let data = [1u8, 2, 3, 4, 5];
        assert_eq!(cobs_decode(&cobs_encode(&data)), data);
    }

    #[test]
    fn cobs_roundtrip_leading_zero() {
        let data = [0u8, 1, 2];
        assert_eq!(cobs_decode(&cobs_encode(&data)), data);
    }

    #[test]
    fn packet_parses_back() {
        let pkt = build_packet(0x53, &[0x01, 0x00, 0x00, 0x00], 1);
        let resp = parse_response(&pkt).expect("parse failed");
        assert_eq!(resp.cmd, 0x53);
        assert!(resp.crc_ok);
        assert_eq!(resp.payload, &[0x01, 0x00, 0x00, 0x00]);
    }
}
