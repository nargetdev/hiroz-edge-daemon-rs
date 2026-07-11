//! libcamera open, mode table, capture session.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Context, Result};
use libcamera::camera::{ActiveCamera, Camera, CameraConfigurationStatus};
use libcamera::camera_manager::CameraManager;
use libcamera::control::ControlList;
use libcamera::controls::{
    AeEnable, AfMode, AnalogueGain, AwbEnable, ExposureTime, HdrMode, LensPosition,
};
use libcamera::framebuffer::AsFrameBuffer;
use libcamera::framebuffer_allocator::{FrameBuffer, FrameBufferAllocator};
use libcamera::framebuffer_map::MemoryMappedFrameBuffer;
use libcamera::geometry::Size;
use libcamera::pixel_format::PixelFormat;
use libcamera::properties;
use libcamera::request::{Request, ReuseFlag};
use libcamera::stream::{Stream, StreamRole};

use crate::jpeg_util::{cpu_demosaicable, jpeg_scale_dims, raw_to_jpeg, BayerOrder};
use crate::{JpegPipeline, StreamSettings};

/// Preferred raw format; the pipeline may resolve to another Bayer order
/// (e.g. SRGGB10_CSI2P on Pi 5 IMX708). The resolved format is reported.
pub const PREFERRED_RAW_FORMAT: &str = "SBGGR10_CSI2P";

#[derive(Debug, Clone)]
pub struct SensorMode {
    pub index: u8,
    pub width: u32,
    pub height: u32,
    pub pixel_format: String,
    pub max_fps_hint: Option<f64>,
}

impl SensorMode {
    pub fn label(&self) -> String {
        match self.max_fps_hint {
            Some(fps) => format!(
                "{}x{}_{}_{:.2}fps",
                self.width, self.height, self.pixel_format, fps
            ),
            None => format!("{}x{}_{}", self.width, self.height, self.pixel_format),
        }
    }
}

#[derive(Debug, Clone)]
pub struct CameraInfo {
    pub index: usize,
    pub id: String,
    pub model: String,
    pub modes: Vec<SensorMode>,
}

#[derive(Debug, Clone)]
pub struct CapturedFrame {
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    /// Resolved libcamera pixel format (used as Image.encoding).
    pub encoding: String,
    pub raw: Vec<u8>,
    pub jpeg: Option<Vec<u8>>,
    pub stamp_ns: u128,
    pub sequence: u32,
}

pub fn list_cameras() -> Result<Vec<CameraInfo>> {
    let mgr = CameraManager::new().context("CameraManager::new")?;
    let cameras = mgr.cameras();
    let mut out = Vec::new();
    for (index, cam) in cameras.iter().enumerate() {
        let id = cam.id().to_string();
        let model = cam
            .properties()
            .get::<properties::Model>()
            .map(|m| m.0.clone())
            .unwrap_or_else(|_| "unknown".to_string());
        let modes = discover_modes(&cam)?;
        out.push(CameraInfo {
            index,
            id,
            model,
            modes,
        });
    }
    Ok(out)
}

fn discover_modes(cam: &Camera<'_>) -> Result<Vec<SensorMode>> {
    let config = cam
        .generate_configuration(&[StreamRole::Raw])
        .context("generate raw configuration for mode discovery")?;
    let cfg = config
        .get(0)
        .ok_or_else(|| anyhow!("raw config missing stream 0"))?;
    let formats = cfg.formats();
    let mut modes = Vec::new();
    let mut index = 0u8;
    for pf in formats.pixel_formats().into_iter() {
        let name = pf.to_string();
        for size in formats.sizes(pf) {
            modes.push(SensorMode {
                index,
                width: size.width,
                height: size.height,
                pixel_format: name.clone(),
                max_fps_hint: None,
            });
            index = index.saturating_add(1);
        }
    }
    if modes.is_empty() {
        bail!("no raw modes reported by libcamera");
    }
    Ok(modes)
}

pub fn hdr_fps_warning(modes: &[SensorMode]) -> Option<String> {
    if modes.len() < 2 {
        return None;
    }
    let sizes: Vec<_> = modes.iter().map(|m| (m.width, m.height)).collect();
    if sizes.contains(&(1536, 864))
        && sizes.contains(&(2304, 1296))
        && sizes.contains(&(4608, 2592))
    {
        Some(
            "host may be in HDR profile (all discrete sizes present); \
             high-FPS crop (e.g. 1536x864@~120) may be hidden until HDR is disabled at the pipeline"
                .to_string(),
        )
    } else {
        None
    }
}

pub fn doctor_single_request(camera_index: usize) -> Result<()> {
    let mgr = CameraManager::new().context("CameraManager::new")?;
    let cameras = mgr.cameras();
    let cam = cameras
        .get(camera_index)
        .ok_or_else(|| anyhow!("camera_index={camera_index} out of range (n={})", cameras.len()))?;
    let mut cam = cam.acquire().map_err(|e| {
        anyhow!("acquire camera {camera_index}: {e} (is another process holding it?)")
    })?;

    let mut cfgs = cam
        .generate_configuration(&[StreamRole::Raw])
        .context("generate raw configuration")?;
    {
        let mut stream = cfgs
            .get_mut(0)
            .ok_or_else(|| anyhow!("missing raw stream config"))?;
        if let Some(pf) = PixelFormat::parse(PREFERRED_RAW_FORMAT) {
            stream.set_pixel_format(pf);
        }
        stream.set_size(Size::new(1536, 864));
    }
    match cfgs.validate() {
        CameraConfigurationStatus::Valid | CameraConfigurationStatus::Adjusted => {}
        CameraConfigurationStatus::Invalid => bail!("doctor raw configuration invalid"),
    }
    cam.configure(&mut cfgs).context("configure camera")?;
    let stream = cfgs
        .get(0)
        .and_then(|c| c.stream())
        .ok_or_else(|| anyhow!("no configured raw stream"))?;
    let stride = cfgs.get(0).map(|c| c.get_stride()).unwrap_or(0);
    let size = cfgs
        .get(0)
        .map(|c| c.get_size())
        .unwrap_or(Size::new(0, 0));

    let mut alloc = FrameBufferAllocator::new(&cam);
    let buffers = alloc.alloc(&stream).context("allocate framebuffers")?;
    let mut buffers: Vec<_> = buffers
        .into_iter()
        .map(|buf| MemoryMappedFrameBuffer::new(buf).context("mmap framebuffer"))
        .collect::<Result<Vec<_>>>()?;
    let buf = buffers
        .pop()
        .ok_or_else(|| anyhow!("no buffers allocated"))?;

    let mut req = cam
        .create_request(None)
        .ok_or_else(|| anyhow!("create request failed"))?;
    req.add_buffer(&stream, buf).context("add buffer")?;

    let rx = cam.subscribe_request_completed();
    cam.start(None).context("start camera")?;
    cam.queue_request(req)
        .map_err(|(_, e)| anyhow!("queue request: {e}"))?;

    let req = rx
        .recv_timeout(Duration::from_secs(5))
        .context("doctor request timeout")?;
    let fb: &MemoryMappedFrameBuffer<FrameBuffer> = req
        .buffer(&stream)
        .ok_or_else(|| anyhow!("completed request missing buffer"))?;
    let planes = fb.data();
    let plane0 = planes
        .first()
        .ok_or_else(|| anyhow!("framebuffer has no planes"))?;
    let bytes_used = fb
        .metadata()
        .and_then(|m| m.planes().get(0))
        .map(|p| p.bytes_used as usize)
        .unwrap_or(plane0.len())
        .min(plane0.len());

    println!(
        "DOCTOR_REQUEST_GREEN camera_index={} size={}x{} stride={} bytes_used={}",
        camera_index, size.width, size.height, stride, bytes_used
    );
    let _ = cam.stop();
    Ok(())
}

fn apply_controls(list: &mut ControlList, settings: &StreamSettings) {
    if let Err(e) = list.set(AeEnable(settings.ae_enable)) {
        eprintln!("IMX708_CONTROL_WARN control=AeEnable error={e:?}");
    }
    if !settings.ae_enable {
        if let Err(e) = list.set(ExposureTime(settings.exposure_time_us as i32)) {
            eprintln!("IMX708_CONTROL_WARN control=ExposureTime error={e:?}");
        }
        if let Err(e) = list.set(AnalogueGain(settings.analogue_gain as f32)) {
            eprintln!("IMX708_CONTROL_WARN control=AnalogueGain error={e:?}");
        }
    }
    if let Err(e) = list.set(AwbEnable(settings.awb_enable)) {
        eprintln!("IMX708_CONTROL_WARN control=AwbEnable error={e:?}");
    }

    let af = match settings.af_mode {
        0 => AfMode::Auto,
        1 => AfMode::Manual,
        2 => AfMode::Continuous,
        _ => AfMode::Auto,
    };
    if let Err(e) = list.set(af) {
        eprintln!("IMX708_CONTROL_WARN control=AfMode error={e:?}");
    } else if settings.af_mode == 1 {
        if let Err(e) = list.set(LensPosition(settings.lens_position as f32)) {
            eprintln!("IMX708_CONTROL_WARN control=LensPosition error={e:?}");
        }
    }

    let hdr = if settings.hdr_enable {
        HdrMode::MultiExposure
    } else {
        HdrMode::Off
    };
    if let Err(e) = list.set(hdr) {
        eprintln!("IMX708_CONTROL_WARN control=HdrMode error={e:?}");
    }
}

fn apply_controls_to_request(req: &mut Request, settings: &StreamSettings) {
    apply_controls(req.controls_mut(), settings);
}

/// Blocking capture loop. Sends frames on `tx` until `stop` is set or channel closes.
pub fn run_capture_loop(
    camera_index: usize,
    settings: StreamSettings,
    tx: mpsc::SyncSender<CapturedFrame>,
    stop: Arc<AtomicBool>,
) -> Result<()> {
    let mgr = CameraManager::new().context("CameraManager::new")?;
    let cameras = mgr.cameras();
    let cam_ref = cameras
        .get(camera_index)
        .ok_or_else(|| anyhow!("camera_index={camera_index} out of range"))?;
    let modes = discover_modes(&cam_ref)?;
    let mode = modes
        .iter()
        .find(|m| m.index == settings.sensor_mode)
        .cloned()
        .ok_or_else(|| {
            anyhow!(
                "sensor_mode={} out of range 0..{}",
                settings.sensor_mode,
                modes.len().saturating_sub(1)
            )
        })?;

    let mut cam = cam_ref.acquire().map_err(|e| anyhow!("acquire camera: {e}"))?;

    let roles: Vec<StreamRole> = match settings.jpeg_pipeline {
        JpegPipeline::CpuFromRaw => vec![StreamRole::Raw],
        JpegPipeline::IspProcessed => vec![StreamRole::Raw, StreamRole::ViewFinder],
    };

    let mut cfgs = cam
        .generate_configuration(&roles)
        .context("generate configuration")?;

    {
        let mut raw = cfgs
            .get_mut(0)
            .ok_or_else(|| anyhow!("missing raw stream"))?;
        let pf = PixelFormat::parse(&mode.pixel_format)
            .or_else(|| PixelFormat::parse(PREFERRED_RAW_FORMAT))
            .ok_or_else(|| anyhow!("cannot parse pixel format {}", mode.pixel_format))?;
        raw.set_pixel_format(pf);
        raw.set_size(Size::new(mode.width, mode.height));
    }

    if matches!(settings.jpeg_pipeline, JpegPipeline::IspProcessed) {
        let (jw, jh) = jpeg_scale_dims(mode.width, mode.height, settings.jpeg_scale);
        let mut vf = cfgs
            .get_mut(1)
            .ok_or_else(|| anyhow!("isp_processed missing ViewFinder stream"))?;
        if let Some(pf) = PixelFormat::parse("RGB888") {
            vf.set_pixel_format(pf);
        }
        vf.set_size(Size::new(jw, jh));
    }

    match cfgs.validate() {
        CameraConfigurationStatus::Valid => {}
        CameraConfigurationStatus::Adjusted => {
            eprintln!("IMX708_CONFIG_ADJUSTED cfg={cfgs:?}");
        }
        CameraConfigurationStatus::Invalid => {
            bail!(
                "camera configuration invalid for pipeline={:?} mode={}",
                settings.jpeg_pipeline,
                mode.label()
            );
        }
    }

    cam.configure(&mut cfgs).context("configure camera")?;

    let mut resolved_raw_format = cfgs
        .get(0)
        .ok_or_else(|| anyhow!("raw cfg missing after configure"))?
        .get_pixel_format()
        .to_string();

    // Spec (cpu_from_raw): if the pipeline resolves the raw stream to something
    // the CPU cannot demosaic (e.g. PiSP compressed raw), fall back to 16-bit
    // uncompressed Bayer — still single-stream. Otherwise JPEG hard-fails with
    // a clear reason.
    if matches!(settings.jpeg_pipeline, JpegPipeline::CpuFromRaw)
        && settings.publish_jpeg
        && !cpu_demosaicable(&resolved_raw_format)
    {
        let fallback_name = BayerOrder::parse(&mode.pixel_format).map(|o| match o {
            BayerOrder::Rggb => "SRGGB16",
            BayerOrder::Bggr => "SBGGR16",
            BayerOrder::Grbg => "SGRBG16",
            BayerOrder::Gbrg => "SGBRG16",
        });
        if let Some(pf) = fallback_name.and_then(PixelFormat::parse) {
            {
                let mut raw = cfgs
                    .get_mut(0)
                    .ok_or_else(|| anyhow!("missing raw stream for 16-bit fallback"))?;
                raw.set_pixel_format(pf);
                raw.set_size(Size::new(mode.width, mode.height));
            }
            match cfgs.validate() {
                CameraConfigurationStatus::Invalid => {
                    eprintln!("IMX708_RAW_FALLBACK_INVALID requested={fallback_name:?}");
                }
                _ => {
                    cam.configure(&mut cfgs)
                        .context("configure camera (16-bit fallback)")?;
                    resolved_raw_format = cfgs
                        .get(0)
                        .ok_or_else(|| anyhow!("raw cfg missing after fallback configure"))?
                        .get_pixel_format()
                        .to_string();
                    println!(
                        "IMX708_RAW_FALLBACK_16BIT requested={} resolved={}",
                        fallback_name.unwrap_or("?"),
                        resolved_raw_format
                    );
                }
            }
        }
        if !cpu_demosaicable(&resolved_raw_format) {
            eprintln!(
                "IMX708_JPEG_UNSUPPORTED reason=resolved raw format {resolved_raw_format} not CPU-demosaicable and 16-bit fallback unavailable; JPEG will fail"
            );
        }
    }

    let raw_cfg = cfgs
        .get(0)
        .ok_or_else(|| anyhow!("raw cfg missing after configure"))?;
    let raw_stream = raw_cfg
        .stream()
        .ok_or_else(|| anyhow!("raw stream handle missing"))?;
    let raw_stride = raw_cfg.get_stride();
    let raw_size = raw_cfg.get_size();
    let raw_format = resolved_raw_format;
    println!(
        "IMX708_RAW_FORMAT_RESOLVED format={} size={}x{} stride={}",
        raw_format, raw_size.width, raw_size.height, raw_stride
    );

    let vf_stream = if matches!(settings.jpeg_pipeline, JpegPipeline::IspProcessed) {
        Some(
            cfgs.get(1)
                .and_then(|c| c.stream())
                .ok_or_else(|| anyhow!("viewfinder stream missing after configure"))?,
        )
    } else {
        None
    };
    let vf_stride = cfgs.get(1).map(|c| c.get_stride());
    let vf_size = cfgs.get(1).map(|c| c.get_size());

    let mut alloc = FrameBufferAllocator::new(&cam);
    let raw_buffers = alloc.alloc(&raw_stream).context("alloc raw buffers")?;
    let raw_buffers: Vec<_> = raw_buffers
        .into_iter()
        .map(|b| MemoryMappedFrameBuffer::new(b).context("mmap raw"))
        .collect::<Result<_>>()?;

    let vf_buffers = if let Some(stream) = &vf_stream {
        let bufs = alloc.alloc(stream).context("alloc viewfinder buffers")?;
        Some(
            bufs.into_iter()
                .map(|b| MemoryMappedFrameBuffer::new(b).context("mmap viewfinder"))
                .collect::<Result<Vec<_>>>()?,
        )
    } else {
        None
    };

    let n = match &vf_buffers {
        Some(v) => raw_buffers.len().min(v.len()),
        None => raw_buffers.len(),
    };
    if n == 0 {
        bail!("no framebuffers allocated");
    }

    let mut raw_iter = raw_buffers.into_iter();
    let mut vf_iter = vf_buffers.map(|v| v.into_iter());
    let mut initial = Vec::with_capacity(n);
    for i in 0..n {
        let mut req = cam
            .create_request(Some(i as u64))
            .ok_or_else(|| anyhow!("create_request failed"))?;
        let raw_buf = raw_iter
            .next()
            .ok_or_else(|| anyhow!("raw buffer underrun"))?;
        req.add_buffer(&raw_stream, raw_buf)
            .context("add raw buffer")?;
        if let (Some(stream), Some(iter)) = (&vf_stream, vf_iter.as_mut()) {
            let vf_buf = iter
                .next()
                .ok_or_else(|| anyhow!("viewfinder buffer underrun"))?;
            req.add_buffer(stream, vf_buf)
                .context("add viewfinder buffer")?;
        }
        apply_controls_to_request(&mut req, &settings);
        initial.push(req);
    }

    let rx = cam.subscribe_request_completed();
    cam.start(None).context("start camera")?;

    for req in initial {
        cam.queue_request(req)
            .map_err(|(_, e)| anyhow!("queue_request: {e}"))?;
    }

    let mut last_jpeg = Instant::now()
        .checked_sub(Duration::from_secs(3600))
        .unwrap_or_else(Instant::now);
    let jpeg_period = if settings.jpeg_fps > 0.0 {
        Duration::from_secs_f64(1.0 / settings.jpeg_fps)
    } else {
        Duration::ZERO
    };

    while !stop.load(Ordering::Relaxed) {
        let req = match rx.recv_timeout(Duration::from_millis(500)) {
            Ok(r) => r,
            Err(mpsc::RecvTimeoutError::Timeout) => continue,
            Err(mpsc::RecvTimeoutError::Disconnected) => break,
        };

        let frame = match extract_frame(
            &req,
            &raw_stream,
            raw_size,
            raw_stride,
            &raw_format,
            vf_stream.as_ref(),
            vf_size,
            vf_stride,
            &settings,
            &mut last_jpeg,
            jpeg_period,
        ) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("IMX708_FRAME_RED error={e:#}");
                requeue(&cam, req, &settings);
                continue;
            }
        };

        match tx.try_send(frame) {
            Ok(()) => {}
            Err(mpsc::TrySendError::Full(_)) => {
                // depth-1 full: drop this frame (publisher lagging)
            }
            Err(mpsc::TrySendError::Disconnected(_)) => break,
        }

        requeue(&cam, req, &settings);
    }

    let _ = cam.stop();
    Ok(())
}

fn requeue(cam: &ActiveCamera<'_>, mut req: Request, settings: &StreamSettings) {
    req.reuse(ReuseFlag::REUSE_BUFFERS);
    apply_controls_to_request(&mut req, settings);
    if let Err((_, e)) = cam.queue_request(req) {
        eprintln!("IMX708_REQUEUE_RED error={e}");
    }
}

fn extract_frame(
    req: &Request,
    raw_stream: &Stream,
    raw_size: Size,
    raw_stride: u32,
    raw_format: &str,
    vf_stream: Option<&Stream>,
    vf_size: Option<Size>,
    vf_stride: Option<u32>,
    settings: &StreamSettings,
    last_jpeg: &mut Instant,
    jpeg_period: Duration,
) -> Result<CapturedFrame> {
    let fb: &MemoryMappedFrameBuffer<FrameBuffer> = req
        .buffer(raw_stream)
        .ok_or_else(|| anyhow!("missing raw buffer"))?;
    let planes = fb.data();
    let plane0 = *planes
        .first()
        .ok_or_else(|| anyhow!("raw framebuffer has no planes"))?;
    let bytes_used = fb
        .metadata()
        .and_then(|m| m.planes().get(0))
        .map(|p| p.bytes_used as usize)
        .unwrap_or(plane0.len())
        .min(plane0.len());
    let raw = plane0[..bytes_used].to_vec();
    let stamp_ns = fb
        .metadata()
        .map(|m| m.timestamp() as u128)
        .filter(|t| *t > 0)
        .unwrap_or_else(crate::unix_time_ns);
    let sequence = fb.metadata().map(|m| m.sequence()).unwrap_or(0);

    let want_jpeg =
        settings.publish_jpeg && (jpeg_period.is_zero() || last_jpeg.elapsed() >= jpeg_period);

    let jpeg = if want_jpeg {
        match settings.jpeg_pipeline {
            JpegPipeline::CpuFromRaw => {
                match raw_to_jpeg(
                    &raw,
                    raw_size.width,
                    raw_size.height,
                    raw_stride,
                    raw_format,
                    settings.jpeg_scale,
                    settings.jpeg_quality,
                ) {
                    Ok(j) => {
                        *last_jpeg = Instant::now();
                        Some(j)
                    }
                    Err(e) => {
                        eprintln!("IMX708_JPEG_RED error={e:#}");
                        None
                    }
                }
            }
            JpegPipeline::IspProcessed => {
                let jpeg = encode_from_viewfinder(
                    req,
                    vf_stream,
                    vf_size,
                    vf_stride,
                    settings.jpeg_quality,
                );
                if jpeg.is_some() {
                    *last_jpeg = Instant::now();
                }
                jpeg
            }
        }
    } else {
        None
    };

    Ok(CapturedFrame {
        width: raw_size.width,
        height: raw_size.height,
        stride: raw_stride,
        encoding: raw_format.to_string(),
        raw,
        jpeg,
        stamp_ns,
        sequence,
    })
}

fn encode_from_viewfinder(
    req: &Request,
    stream: Option<&Stream>,
    size: Option<Size>,
    stride: Option<u32>,
    quality: u8,
) -> Option<Vec<u8>> {
    let stream = stream?;
    let size = size?;
    let stride = stride?;
    let fb: &MemoryMappedFrameBuffer<FrameBuffer> = req.buffer(stream)?;
    let planes = fb.data();
    let plane0 = *planes.first()?;
    let bytes_used = fb
        .metadata()
        .and_then(|m| m.planes().get(0))
        .map(|p| p.bytes_used as usize)
        .unwrap_or(plane0.len())
        .min(plane0.len());
    let data = &plane0[..bytes_used];

    if data.len() > 3 && data[0] == 0xFF && data[1] == 0xD8 {
        return Some(data.to_vec());
    }

    let row_rgb = (size.width * 3) as usize;
    if stride as usize >= row_rgb && data.len() >= stride as usize * size.height as usize {
        let mut rgb = vec![0u8; row_rgb * size.height as usize];
        for y in 0..size.height as usize {
            let src = &data[y * stride as usize..y * stride as usize + row_rgb];
            rgb[y * row_rgb..(y + 1) * row_rgb].copy_from_slice(src);
        }
        return crate::jpeg_util::encode_jpeg_rgb(&rgb, size.width, size.height, quality).ok();
    }
    None
}
