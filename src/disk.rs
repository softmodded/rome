/// Rust mirror of app/src/disk.h — SP1F eMMC disk format.

pub const DISK_MAGIC: u32 = 0x5350_3146; // "SP1F"
pub const DISK_VERSION: u8 = 3; // v3: per-stem baked VU levels (4 bytes/decim) after audio
pub const DISK_CATALOG_BLOCKS: usize = 8;
pub const DISK_SONGS_PER_BLOCK: usize = 16;
pub const DISK_MAX_SONGS: usize = DISK_CATALOG_BLOCKS * DISK_SONGS_PER_BLOCK; // 128

/// Master header — 512 bytes, matches C `disk_header_t`.
#[derive(Clone, Debug)]
pub struct DiskHeader {
    pub magic: u32,
    pub version: u8,
    pub song_count: u16,
    pub next_free_block: u32,
}

impl Default for DiskHeader {
    fn default() -> Self {
        Self {
            magic: DISK_MAGIC,
            version: DISK_VERSION,
            song_count: 0,
            next_free_block: 9, // DISK_DATA_START_BLOCK
        }
    }
}

impl DiskHeader {
    pub fn is_valid(&self) -> bool {
        self.magic == DISK_MAGIC && self.version == DISK_VERSION
    }

    pub fn from_block(block: &[u8; 512]) -> Self {
        Self {
            magic:            u32::from_le_bytes(block[0..4].try_into().unwrap()),
            version:          block[4],
            song_count:       u16::from_le_bytes(block[8..10].try_into().unwrap()),
            next_free_block:  u32::from_le_bytes(block[12..16].try_into().unwrap()),
        }
    }
}

/// One song entry — 32 bytes, matches C `disk_song_entry_t`.
#[derive(Clone, Debug, Default)]
pub struct SongEntry {
    pub name: [u8; 24],
    pub block_start: u32,
    pub block_count: u32,
}

impl SongEntry {
    pub fn is_free(&self) -> bool {
        self.name[0] == 0
    }

    pub fn name_str(&self) -> &str {
        let end = self.name.iter().position(|&b| b == 0).unwrap_or(24);
        std::str::from_utf8(&self.name[..end]).unwrap_or("<invalid>")
    }

    pub fn set_name(&mut self, s: &str) {
        self.name = [0u8; 24];
        let bytes = s.as_bytes();
        let len = bytes.len().min(23);
        self.name[..len].copy_from_slice(&bytes[..len]);
    }

    pub fn from_bytes(b: &[u8; 32]) -> Self {
        let mut e = Self::default();
        e.name.copy_from_slice(&b[0..24]);
        e.block_start = u32::from_le_bytes(b[24..28].try_into().unwrap());
        e.block_count = u32::from_le_bytes(b[28..32].try_into().unwrap());
        e
    }
}

/// Parse the 4096-byte catalog payload (8 blocks × 512 bytes) into song entries.
pub fn parse_catalog(data: &[u8]) -> Vec<SongEntry> {
    assert!(data.len() >= DISK_CATALOG_BLOCKS * 512);
    let mut songs = Vec::new();
    for i in 0..DISK_MAX_SONGS {
        let off = i * 32;
        let mut raw = [0u8; 32];
        raw.copy_from_slice(&data[off..off + 32]);
        songs.push(SongEntry::from_bytes(&raw));
    }
    songs
}
