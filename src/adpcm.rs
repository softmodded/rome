/// IMA-ADPCM encoder for the SP1F 8-channel format.
///
/// Block layout (512 bytes, 8 channels, 128 samples/channel):
///   bytes   0– 63  channel 0 (stem 0 L)  — 128 nibbles, low nibble first
///   bytes  64–127  channel 1 (stem 0 R)
///   bytes 128–191  channel 2 (stem 1 L)
///   bytes 192–255  channel 3 (stem 1 R)
///   bytes 256–319  channel 4 (stem 2 L)
///   bytes 320–383  channel 5 (stem 2 R)
///   bytes 384–447  channel 6 (stem 3 L)
///   bytes 448–511  channel 7 (stem 3 R)
///
/// ADPCM state carries over between blocks.
/// Initial state: predictor=0, step_index=0 for all channels.

#[rustfmt::skip]
pub const STEP_TABLE: [i32; 89] = [
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41,
    45, 50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190,
    209, 230, 253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749,
    3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630,
    9493, 10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623,
    27086, 29794, 32767,
];

#[rustfmt::skip]
const INDEX_TABLE: [i32; 16] = [
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8,
];

pub const SAMPLES_PER_BLOCK: usize = 128;
pub const BLOCK_SIZE: usize = 512;
pub const CHANNELS: usize = 8;

pub struct Channel {
    pub predictor: i32,
    pub step_index: i32,
}

impl Default for Channel {
    fn default() -> Self { Self { predictor: 0, step_index: 0 } }
}

impl Channel {
    pub fn encode(&mut self, sample: i16) -> u8 {
        let step = STEP_TABLE[self.step_index as usize];
        let diff = sample as i32 - self.predictor;

        let mut nibble: i32 = if diff < 0 { 8 } else { 0 };
        let mut abs_diff = diff.unsigned_abs() as i32;

        if abs_diff >= step      { nibble |= 4; abs_diff -= step; }
        if abs_diff >= step >> 1 { nibble |= 2; abs_diff -= step >> 1; }
        if abs_diff >= step >> 2 { nibble |= 1; }
        let _ = abs_diff;

        let mut diffq = step >> 3;
        if nibble & 4 != 0 { diffq += step; }
        if nibble & 2 != 0 { diffq += step >> 1; }
        if nibble & 1 != 0 { diffq += step >> 2; }

        if nibble & 8 != 0 { self.predictor -= diffq; } else { self.predictor += diffq; }
        self.predictor = self.predictor.clamp(-32768, 32767);
        self.step_index =
            (self.step_index + INDEX_TABLE[(nibble & 0xF) as usize]).clamp(0, 88);

        (nibble & 0xF) as u8
    }
}

/// Encode one 512-byte 8-channel block. State persists across calls.
/// Each channel slice should have SAMPLES_PER_BLOCK samples; shorter slices
/// are zero-padded.
pub fn encode_block_8ch(
    channels: &[&[i16]; CHANNELS],
    states: &mut [Channel; CHANNELS],
) -> [u8; BLOCK_SIZE] {
    let mut block = [0u8; BLOCK_SIZE];
    for (ch, (samples, state)) in channels.iter().zip(states.iter_mut()).enumerate() {
        let base = ch * 64;
        for b in 0..64 {
            let s0 = samples.get(b * 2).copied().unwrap_or(0);
            let s1 = samples.get(b * 2 + 1).copied().unwrap_or(0);
            let n0 = state.encode(s0);
            let n1 = state.encode(s1);
            block[base + b] = (n1 << 4) | n0;
        }
    }
    block
}

/// Encode 8 channels of mono PCM samples into 512-byte ADPCM blocks.
/// All channels must have the same sample count.
pub fn encode_8ch(channel_pcm: &[Vec<i16>; CHANNELS]) -> Vec<[u8; BLOCK_SIZE]> {
    let frame_count = channel_pcm[0].len();
    let block_count = frame_count.div_ceil(SAMPLES_PER_BLOCK);
    let mut states: [Channel; CHANNELS] = std::array::from_fn(|_| Channel::default());
    let mut blocks = Vec::with_capacity(block_count);

    for b in 0..block_count {
        let start = b * SAMPLES_PER_BLOCK;
        let end = (start + SAMPLES_PER_BLOCK).min(frame_count);
        let slices: [&[i16]; CHANNELS] =
            std::array::from_fn(|ch| &channel_pcm[ch][start..end]);
        blocks.push(encode_block_8ch(&slices, &mut states));
    }

    blocks
}

/// Audio blocks per baked VU-level byte. MUST match firmware `LVL_DECIM`.
pub const LVL_DECIM: usize = 16;

/// Bake one VU-level byte per `LVL_DECIM` audio blocks. Decodes all 8 channels
/// (4 stereo stems) and sums them — UN-clipped — so the meter reflects the
/// combined energy of every stem, not just the loudest. Peak of |sumL|,|sumR|
/// over the group, quantized >> 9 → 0..255 (4 full stems ≈ 255, single ≈ 63).
pub fn bake_levels(blocks: &[[u8; BLOCK_SIZE]]) -> Vec<u8> {
    let mut pred = [0i32; CHANNELS];
    let mut sidx = [0i32; CHANNELS];

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

    let mut levels = Vec::with_capacity(blocks.len().div_ceil(LVL_DECIM));
    let mut group_peak = 0i32;
    for (bi, b) in blocks.iter().enumerate() {
        for i in 0..SAMPLES_PER_BLOCK {
            // Sum all 4 stems (8 channels) without clipping so each contributes.
            let (mut l, mut r) = (0i32, 0i32);
            for stem in 0..4 {
                let bl = b[stem * 128 + i / 2];
                let br = b[stem * 128 + 64 + i / 2];
                let nl = if i & 1 == 1 { bl >> 4 } else { bl & 0xF };
                let nr = if i & 1 == 1 { br >> 4 } else { br & 0xF };
                l += step(&mut sidx[stem * 2],     &mut pred[stem * 2],     nl);
                r += step(&mut sidx[stem * 2 + 1], &mut pred[stem * 2 + 1], nr);
            }
            group_peak = group_peak.max(l.abs()).max(r.abs());
        }
        if bi % LVL_DECIM == LVL_DECIM - 1 || bi == blocks.len() - 1 {
            // >> 9: 4 stems at full scale (~131068) → 255, single stem → ~63.
            levels.push((group_peak >> 9).min(255) as u8);
            group_peak = 0;
        }
    }
    levels
}

/// Pack baked level bytes into 512-byte blocks (last block zero-padded).
pub fn pack_levels(levels: &[u8]) -> Vec<[u8; BLOCK_SIZE]> {
    levels
        .chunks(BLOCK_SIZE)
        .map(|c| {
            let mut blk = [0u8; BLOCK_SIZE];
            blk[..c.len()].copy_from_slice(c);
            blk
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn block_is_512_bytes() {
        let empty = vec![0i16; 0];
        let ch: [&[i16]; CHANNELS] = std::array::from_fn(|_| empty.as_slice());
        let mut states: [Channel; CHANNELS] = std::array::from_fn(|_| Channel::default());
        let b = encode_block_8ch(&ch, &mut states);
        assert_eq!(b.len(), 512);
    }

    #[test]
    fn encode_8ch_block_count() {
        let pcm: Vec<i16> = vec![0i16; 128];
        let channels: [Vec<i16>; CHANNELS] = std::array::from_fn(|_| pcm.clone());
        assert_eq!(encode_8ch(&channels).len(), 1);
        let pcm2: Vec<i16> = vec![0i16; 129];
        let channels2: [Vec<i16>; CHANNELS] = std::array::from_fn(|_| pcm2.clone());
        assert_eq!(encode_8ch(&channels2).len(), 2);
    }
}
