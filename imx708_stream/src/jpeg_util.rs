//! CPU path: unpack packed 10-bit CSI-2 Bayer → demosaic → scale → JPEG.

use anyhow::{bail, Result};
use image::{codecs::jpeg::JpegEncoder, ExtendedColorType, ImageEncoder};

/// Bayer CFA order, parsed from the resolved libcamera format string
/// (e.g. `SRGGB10_CSI2P` on the Pi 5 IMX708, `SBGGR10_CSI2P` elsewhere).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BayerOrder {
    Rggb,
    Bggr,
    Grbg,
    Gbrg,
}

impl BayerOrder {
    pub fn parse(format: &str) -> Option<Self> {
        if format.starts_with("SRGGB") {
            Some(Self::Rggb)
        } else if format.starts_with("SBGGR") {
            Some(Self::Bggr)
        } else if format.starts_with("SGRBG") {
            Some(Self::Grbg)
        } else if format.starts_with("SGBRG") {
            Some(Self::Gbrg)
        } else {
            None
        }
    }

    /// Offsets within a 2x2 CFA block: ((ry,rx), (gy,gx), (by,bx)).
    fn offsets(self) -> ((usize, usize), (usize, usize), (usize, usize)) {
        match self {
            Self::Rggb => ((0, 0), (0, 1), (1, 1)),
            Self::Bggr => ((1, 1), (0, 1), (0, 0)),
            Self::Grbg => ((0, 1), (0, 0), (1, 0)),
            Self::Gbrg => ((1, 0), (0, 0), (0, 1)),
        }
    }
}

/// True when the CPU pipeline can demosaic this resolved format.
pub fn cpu_demosaicable(pixel_format: &str) -> bool {
    BayerOrder::parse(pixel_format).is_some()
        && (pixel_format.contains("10_CSI2P") || pixel_format.ends_with("16"))
}

/// 16-bit-per-sample Bayer (e.g. SRGGB16): little-endian u16 per pixel.
pub fn unpack_bayer16_le(src: &[u8], width: u32, height: u32, stride: u32) -> Result<Vec<u16>> {
    let width = width as usize;
    let height = height as usize;
    let stride = stride as usize;
    let row_bytes = width * 2;
    if stride < row_bytes {
        bail!("stride {stride} < 16-bit row {row_bytes}");
    }
    if src.len() < stride * height {
        bail!("buffer too small: {} < {}", src.len(), stride * height);
    }
    let mut out = vec![0u16; width * height];
    for y in 0..height {
        let row = &src[y * stride..y * stride + row_bytes];
        for x in 0..width {
            out[y * width + x] = u16::from_le_bytes([row[x * 2], row[x * 2 + 1]]);
        }
    }
    Ok(out)
}

/// Right-shift that maps the observed sample range onto 8 bits, so 10-bit
/// values low-aligned in a 16-bit container don't come out black.
fn shift_to_8bit(bayer: &[u16]) -> u32 {
    let max = bayer.iter().copied().max().unwrap_or(0) as u32;
    let mut bits = 8u32;
    while (max >> bits) != 0 {
        bits += 1;
    }
    bits - 8
}

/// CSI-2 packed 10-bit: 4 samples in 5 bytes.
/// Layout: A[9:2], B[9:2], C[9:2], D[9:2], (D[1:0]<<6|C[1:0]<<4|B[1:0]<<2|A[1:0]).
pub fn unpack_bayer10_csi2p(src: &[u8], width: u32, height: u32, stride: u32) -> Result<Vec<u16>> {
    let width = width as usize;
    let height = height as usize;
    let stride = stride as usize;
    if width == 0 || height == 0 || width % 2 != 0 || height % 2 != 0 {
        bail!("invalid bayer size {width}x{height}");
    }
    let packed_row = (width * 5).div_ceil(4);
    if stride < packed_row {
        bail!("stride {stride} < packed row {packed_row}");
    }
    if src.len() < stride * height {
        bail!("buffer too small: {} < {}", src.len(), stride * height);
    }

    let mut out = vec![0u16; width * height];
    for y in 0..height {
        let row = &src[y * stride..y * stride + packed_row];
        let mut x = 0usize;
        let mut i = 0usize;
        while x + 3 < width && i + 4 < row.len() {
            let a = row[i] as u16;
            let b = row[i + 1] as u16;
            let c = row[i + 2] as u16;
            let d = row[i + 3] as u16;
            let l = row[i + 4] as u16;
            out[y * width + x] = (a << 2) | (l & 0x3);
            out[y * width + x + 1] = (b << 2) | ((l >> 2) & 0x3);
            out[y * width + x + 2] = (c << 2) | ((l >> 4) & 0x3);
            out[y * width + x + 3] = (d << 2) | ((l >> 6) & 0x3);
            x += 4;
            i += 5;
        }
    }
    Ok(out)
}

/// Nearest-neighbor demosaic → RGB8 (preview-grade). Each 2x2 CFA block's
/// R/G/B samples are replicated to all four pixels of the block.
pub fn demosaic_bayer_nearest(bayer: &[u16], width: u32, height: u32, order: BayerOrder) -> Vec<u8> {
    let w = width as usize;
    let h = height as usize;
    let mut rgb = vec![0u8; w * h * 3];
    let ((ry, rx), (gy, gx), (by, bx)) = order.offsets();
    let shift = shift_to_8bit(bayer);
    let sample = |x: usize, y: usize| -> u8 {
        let v = bayer[y.min(h - 1) * w + x.min(w - 1)];
        (v >> shift).min(255) as u8
    };
    for y in (0..h).step_by(2) {
        for x in (0..w).step_by(2) {
            let r = sample(x + rx, y + ry);
            let g = sample(x + gx, y + gy);
            let b = sample(x + bx, y + by);
            for dy in 0..2usize {
                for dx in 0..2usize {
                    if y + dy < h && x + dx < w {
                        let i = ((y + dy) * w + (x + dx)) * 3;
                        rgb[i] = r;
                        rgb[i + 1] = g;
                        rgb[i + 2] = b;
                    }
                }
            }
        }
    }
    rgb
}

pub fn scale_rgb_nearest(
    rgb: &[u8],
    width: u32,
    height: u32,
    out_w: u32,
    out_h: u32,
) -> Result<Vec<u8>> {
    if out_w == 0 || out_h == 0 {
        bail!("invalid scale target {out_w}x{out_h}");
    }
    if out_w == width && out_h == height {
        return Ok(rgb.to_vec());
    }
    let mut out = vec![0u8; (out_w * out_h * 3) as usize];
    for y in 0..out_h {
        let sy = (y as u64 * height as u64 / out_h as u64) as u32;
        for x in 0..out_w {
            let sx = (x as u64 * width as u64 / out_w as u64) as u32;
            let si = ((sy * width + sx) * 3) as usize;
            let di = ((y * out_w + x) * 3) as usize;
            out[di] = rgb[si];
            out[di + 1] = rgb[si + 1];
            out[di + 2] = rgb[si + 2];
        }
    }
    Ok(out)
}

pub fn encode_jpeg_rgb(rgb: &[u8], width: u32, height: u32, quality: u8) -> Result<Vec<u8>> {
    let q = quality.clamp(1, 100);
    let mut out = Vec::new();
    let enc = JpegEncoder::new_with_quality(&mut out, q);
    enc.write_image(rgb, width, height, ExtendedColorType::Rgb8)
        .map_err(|e| anyhow::anyhow!("jpeg encode: {e}"))?;
    Ok(out)
}

pub fn jpeg_scale_dims(width: u32, height: u32, scale: u8) -> (u32, u32) {
    let div = match scale {
        0 => 1,
        1 => 2,
        2 => 4,
        _ => 8,
    };
    let w = (width / div).max(2) & !1;
    let h = (height / div).max(2) & !1;
    (w, h)
}

pub fn raw_to_jpeg(
    packed: &[u8],
    width: u32,
    height: u32,
    stride: u32,
    pixel_format: &str,
    scale: u8,
    quality: u8,
) -> Result<Vec<u8>> {
    let order = BayerOrder::parse(pixel_format)
        .ok_or_else(|| anyhow::anyhow!("not a demosaicable Bayer format: {pixel_format}"))?;
    let bayer = if pixel_format.contains("10_CSI2P") {
        unpack_bayer10_csi2p(packed, width, height, stride)?
    } else if pixel_format.ends_with("16") {
        unpack_bayer16_le(packed, width, height, stride)?
    } else {
        bail!("unsupported raw packing for CPU demosaic: {pixel_format}");
    };
    let rgb = demosaic_bayer_nearest(&bayer, width, height, order);
    let (ow, oh) = jpeg_scale_dims(width, height, scale);
    let scaled = scale_rgb_nearest(&rgb, width, height, ow, oh)?;
    encode_jpeg_rgb(&scaled, ow, oh, quality)
}
