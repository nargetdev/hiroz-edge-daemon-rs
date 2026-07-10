//! CPU path: unpack packed SBGGR10_CSI2P → demosaic → scale → JPEG.

use anyhow::{bail, Result};
use image::{codecs::jpeg::JpegEncoder, ExtendedColorType, ImageEncoder};

/// CSI-2 packed 10-bit: 4 samples in 5 bytes.
/// Layout: A[9:2], B[9:2], C[9:2], D[9:2], (D[1:0]<<6|C[1:0]<<4|B[1:0]<<2|A[1:0]).
pub fn unpack_sbggr10_csi2p(src: &[u8], width: u32, height: u32, stride: u32) -> Result<Vec<u16>> {
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

/// Nearest-neighbor SBGGR demosaic → RGB8 (preview-grade).
pub fn demosaic_sbggr_nearest(bayer: &[u16], width: u32, height: u32) -> Vec<u8> {
    let w = width as usize;
    let h = height as usize;
    let mut rgb = vec![0u8; w * h * 3];
    let sample = |x: usize, y: usize| -> u8 {
        let v = bayer[y * w + x];
        (v >> 2).min(255) as u8
    };
    for y in 0..h {
        for x in 0..w {
            let (r, g, b) = match (y & 1, x & 1) {
                // SBGGR: even row B G; odd row G R
                (0, 0) => {
                    let bb = sample(x, y);
                    let g = sample(x.saturating_add(1).min(w - 1), y);
                    let rr = sample(
                        x.saturating_add(1).min(w - 1),
                        y.saturating_add(1).min(h - 1),
                    );
                    (rr, g, bb)
                }
                (0, 1) => {
                    let g = sample(x, y);
                    let bb = sample(x.saturating_sub(1), y);
                    let rr = sample(x, y.saturating_add(1).min(h - 1));
                    (rr, g, bb)
                }
                (1, 0) => {
                    let g = sample(x, y);
                    let rr = sample(x.saturating_add(1).min(w - 1), y);
                    let bb = sample(x, y.saturating_sub(1));
                    (rr, g, bb)
                }
                _ => {
                    let rr = sample(x, y);
                    let g = sample(x.saturating_sub(1), y);
                    let bb = sample(x.saturating_sub(1), y.saturating_sub(1));
                    (rr, g, bb)
                }
            };
            let i = (y * w + x) * 3;
            rgb[i] = r;
            rgb[i + 1] = g;
            rgb[i + 2] = b;
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
    let mut enc = JpegEncoder::new_with_quality(&mut out, q);
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
    scale: u8,
    quality: u8,
) -> Result<Vec<u8>> {
    let bayer = unpack_sbggr10_csi2p(packed, width, height, stride)?;
    let rgb = demosaic_sbggr_nearest(&bayer, width, height);
    let (ow, oh) = jpeg_scale_dims(width, height, scale);
    let scaled = scale_rgb_nearest(&rgb, width, height, ow, oh)?;
    encode_jpeg_rgb(&scaled, ow, oh, quality)
}
