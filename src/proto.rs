use anyhow::{anyhow, bail, Context, Result};
use serialport::SerialPortType;
use std::collections::VecDeque;
use std::time::Duration;
use rusb::{Direction, TransferType, UsbContext};

pub const CMD_PING: u8         = 0x01;
pub const CMD_DISK_INFO: u8    = 0x02;
pub const CMD_DISK_FORMAT: u8  = 0x03;
pub const CMD_SONG_BEGIN: u8      = 0x04;
pub const CMD_SONG_BLOCK: u8      = 0x05;
pub const CMD_SONG_COMMIT: u8     = 0x06;
pub const CMD_SONG_REMOVE: u8     = 0x07;
pub const CMD_CATALOG_READ: u8    = 0x08;
pub const CMD_SONG_MULTIBLOCK: u8 = 0x09;
pub const CMD_EXTCSD_DUMP: u8     = 0x0A;
pub const CMD_CODEC_DIAG: u8      = 0x0B;
pub const CMD_READ_BLOCK: u8      = 0x0C;
pub const CMD_WRITE_PROBE: u8     = 0x0D;
pub const CMD_WRITE_STRESS: u8    = 0x0E;

const STATUS_OK: u8  = 0x00;
const STATUS_ERR: u8 = 0xFF;

pub const DEVICE_VID: u16 = 0x2FE3;
pub const DEVICE_PID: u16 = 0x0101;

pub fn find_device_port() -> Option<String> {
    serialport::available_ports().ok()?.into_iter().find_map(|info| {
        if let SerialPortType::UsbPort(usb) = info.port_type {
            if usb.vid == DEVICE_VID && usb.pid == DEVICE_PID {
                return Some(info.port_name);
            }
        }
        None
    })
}

pub fn list_ports() -> Result<()> {
    let ports = serialport::available_ports().context("cannot list serial ports")?;
    if ports.is_empty() {
        println!("No serial ports found.");
        return Ok(());
    }
    for info in &ports {
        let tag = match &info.port_type {
            SerialPortType::UsbPort(usb) if usb.vid == DEVICE_VID && usb.pid == DEVICE_PID => {
                " ← SP-1 Stem Player"
            }
            _ => "",
        };
        println!("{}{}", info.port_name, tag);
    }
    Ok(())
}

pub struct DeviceConn {
    handle: rusb::DeviceHandle<rusb::Context>,
    ep_in: u8,
    ep_out: u8,
    timeout: Duration,
    rbuf: VecDeque<u8>,
}

impl DeviceConn {
    /// Port name is ignored — we bind directly to the USB device by VID/PID and
    /// drive its CDC-ACM bulk endpoints raw (bypasses the kernel cdc-acm tty,
    /// which caps throughput at ~180 KB/s).
    pub fn open(_port_name: &str) -> Result<Self> { Self::open_auto() }

    pub fn open_auto() -> Result<Self> {
        let ctx = rusb::Context::new().context("init libusb")?;
        for dev in ctx.devices().context("list usb devices")?.iter() {
            let desc = match dev.device_descriptor() { Ok(d) => d, Err(_) => continue };
            if desc.vendor_id() != DEVICE_VID || desc.product_id() != DEVICE_PID { continue; }

            // Find the CDC-Data interface (class 0x0A) and its bulk IN/OUT endpoints.
            let cfg = dev.active_config_descriptor().context("read config descriptor")?;
            let mut found: Option<(u8, u8, u8)> = None;
            for iface in cfg.interfaces() {
                for id in iface.descriptors() {
                    if id.class_code() != 0x0A { continue; }
                    let (mut ep_in, mut ep_out) = (0u8, 0u8);
                    for ep in id.endpoint_descriptors() {
                        if ep.transfer_type() != TransferType::Bulk { continue; }
                        match ep.direction() {
                            Direction::In  => ep_in = ep.address(),
                            Direction::Out => ep_out = ep.address(),
                        }
                    }
                    if ep_in != 0 && ep_out != 0 {
                        found = Some((iface.number(), ep_in, ep_out));
                    }
                }
            }
            let (iface, ep_in, ep_out) =
                found.ok_or_else(|| anyhow!("SP-1 has no CDC-data bulk interface"))?;

            let mut handle = dev.open().context(
                "open SP-1 USB device (permission denied? add a udev rule for \
                 2fe3:0101 or run with sudo)")?;
            handle.set_auto_detach_kernel_driver(true).ok();
            handle.claim_interface(iface).context("claim CDC-data interface")?;
            eprintln!("rome: found SP-1 (USB {:04x}:{:04x}, bulk in 0x{:02x}/out 0x{:02x})",
                      DEVICE_VID, DEVICE_PID, ep_in, ep_out);
            return Ok(Self {
                handle, ep_in, ep_out,
                timeout: Duration::from_secs(10),
                rbuf: VecDeque::new(),
            });
        }
        bail!("SP-1 not found (USB {DEVICE_VID:04x}:{DEVICE_PID:04x}). Is the firmware running?");
    }

    fn write_all(&mut self, data: &[u8]) -> Result<()> {
        let mut off = 0;
        while off < data.len() {
            let n = self.handle.write_bulk(self.ep_out, &data[off..], self.timeout)
                .context("bulk write")?;
            if n == 0 { bail!("bulk write returned 0"); }
            off += n;
        }
        Ok(())
    }

    fn read_exact(&mut self, buf: &mut [u8]) -> Result<()> {
        let mut i = 0;
        while i < buf.len() {
            if let Some(b) = self.rbuf.pop_front() {
                buf[i] = b; i += 1; continue;
            }
            let mut tmp = [0u8; 4096];
            let n = self.handle.read_bulk(self.ep_in, &mut tmp, self.timeout)
                .context("bulk read (timeout?)")?;
            self.rbuf.extend(&tmp[..n]);
        }
        Ok(())
    }

    fn set_timeout(&mut self, t: Duration) { self.timeout = t; }

    fn send(&mut self, cmd: u8, payload: &[u8]) -> Result<()> {
        let len = payload.len() as u32;
        let mut hdr = [0u8; 5];
        hdr[0] = cmd;
        hdr[1..5].copy_from_slice(&len.to_le_bytes());
        self.write_all(&hdr).context("write header")?;
        if !payload.is_empty() {
            self.write_all(payload).context("write payload")?;
        }
        Ok(())
    }

    fn recv(&mut self) -> Result<Vec<u8>> {
        let mut hdr = [0u8; 5];
        self.read_exact(&mut hdr).context("read response header (timeout?)")?;
        let status = hdr[0];
        let len = u32::from_le_bytes(hdr[1..5].try_into().unwrap()) as usize;
        let mut payload = vec![0u8; len];
        if len > 0 {
            self.read_exact(&mut payload).context("read response payload")?;
        }
        if status == STATUS_ERR {
            let msg = String::from_utf8_lossy(&payload).to_string();
            bail!("device error: {}", if msg.is_empty() { "ERR" } else { &msg });
        }
        if status != STATUS_OK {
            bail!("unexpected status byte: 0x{status:02X}");
        }
        Ok(payload)
    }

    fn cmd(&mut self, cmd: u8, payload: &[u8]) -> Result<Vec<u8>> {
        self.send(cmd, payload)?;
        self.recv()
    }

    pub fn ping(&mut self) -> Result<()> {
        self.cmd(CMD_PING, &[])?;
        Ok(())
    }

    pub fn disk_info(&mut self) -> Result<[u8; 512]> {
        let data = self.cmd(CMD_DISK_INFO, &[])?;
        if data.len() < 512 {
            bail!("disk_info: short response ({} bytes)", data.len());
        }
        let mut buf = [0u8; 512];
        buf.copy_from_slice(&data[..512]);
        Ok(buf)
    }

    pub fn disk_format(&mut self) -> Result<()> {
        self.cmd(CMD_DISK_FORMAT, &[])?;
        Ok(())
    }

    pub fn extcsd_dump(&mut self) -> Result<[u8; 512]> {
        let data = self.cmd(CMD_EXTCSD_DUMP, &[])?;
        if data.len() < 512 {
            bail!("extcsd_dump: short response ({} bytes)", data.len());
        }
        let mut buf = [0u8; 512];
        buf.copy_from_slice(&data[..512]);
        Ok(buf)
    }

    pub fn read_block(&mut self, addr: u32) -> Result<[u8; 512]> {
        let data = self.cmd(CMD_READ_BLOCK, &addr.to_le_bytes())?;
        if data.len() < 512 {
            bail!("read_block: short response ({} bytes)", data.len());
        }
        let mut buf = [0u8; 512];
        buf.copy_from_slice(&data[..512]);
        Ok(buf)
    }

    pub fn write_probe(&mut self, addr: u32) -> Result<u8> {
        let data = self.cmd(CMD_WRITE_PROBE, &addr.to_le_bytes())?;
        if data.is_empty() { bail!("write_probe: empty response"); }
        Ok(data[0])
    }

    pub fn write_stress(&mut self, count: u32) -> Result<u32> {
        self.set_timeout(Duration::from_secs(300));
        let r = self.cmd(CMD_WRITE_STRESS, &count.to_le_bytes());
        self.set_timeout(Duration::from_secs(10));
        let data = r?;
        if data.len() < 4 { bail!("write_stress: short response"); }
        Ok(u32::from_le_bytes([data[0], data[1], data[2], data[3]]))
    }

    pub fn codec_diag(&mut self) -> Result<[u8; 32]> {
        let data = self.cmd(CMD_CODEC_DIAG, &[])?;
        if data.len() < 32 {
            bail!("codec_diag: short response ({} bytes)", data.len());
        }
        let mut buf = [0u8; 32];
        buf.copy_from_slice(&data[..32]);
        Ok(buf)
    }

    /// Begin uploading a song. Returns catalog index.
    pub fn song_begin(&mut self, name: &[u8; 24], nblocks: u32) -> Result<u16> {
        let mut payload = [0u8; 28];
        payload[0..24].copy_from_slice(name);
        payload[24..28].copy_from_slice(&nblocks.to_le_bytes());
        let resp = self.cmd(CMD_SONG_BEGIN, &payload)?;
        if resp.len() < 2 {
            bail!("song_begin: short response");
        }
        Ok(u16::from_le_bytes(resp[0..2].try_into().unwrap()))
    }

    pub fn song_block(&mut self, block: &[u8; 512]) -> Result<()> {
        self.cmd(CMD_SONG_BLOCK, block.as_ref())?;
        Ok(())
    }

    /// Send N blocks in one round trip.
    /// Protocol: CMD_SONG_MULTIBLOCK + count(2 LE) header, then raw N×512 bytes,
    /// then single ACK from firmware after all blocks written.
    pub fn song_multiblock(&mut self, blocks: &[[u8; 512]]) -> Result<()> {
        if blocks.is_empty() { return Ok(()); }
        let count = blocks.len() as u16;
        // Send command header (cmd + len=2 + count[2])
        let mut hdr = [0u8; 7];
        hdr[0] = CMD_SONG_MULTIBLOCK;
        hdr[1] = 2; // payload len LE: 2 bytes (just the count)
        hdr[5] = count as u8;
        hdr[6] = (count >> 8) as u8;
        // Coalesce header + all block data into ONE write so the host USB stack
        // pipelines full-size bulk transfers instead of 512-byte dribbles.
        let mut buf = Vec::with_capacity(hdr.len() + blocks.len() * 512);
        buf.extend_from_slice(&hdr);
        for block in blocks {
            buf.extend_from_slice(block.as_ref());
        }
        self.write_all(&buf).context("write multiblock")?;
        // One ACK for all blocks
        self.recv().context("multiblock ack")?;
        Ok(())
    }

    pub fn song_commit(&mut self) -> Result<()> {
        self.cmd(CMD_SONG_COMMIT, &[])?;
        Ok(())
    }

    pub fn song_remove(&mut self, idx: u16) -> Result<()> {
        self.cmd(CMD_SONG_REMOVE, &idx.to_le_bytes())?;
        Ok(())
    }

    /// Read catalog (4096 bytes = 8 × 512-byte catalog blocks).
    pub fn catalog_read(&mut self) -> Result<Vec<u8>> {
        // Use longer timeout — 4096 bytes at CDC ACM speeds takes a moment
        self.set_timeout(Duration::from_secs(30));
        let data = self.cmd(CMD_CATALOG_READ, &[])?;
        self.set_timeout(Duration::from_secs(10));
        if data.len() < 4096 {
            bail!("catalog_read: short response ({} bytes)", data.len());
        }
        Ok(data)
    }
}
