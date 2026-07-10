//! Hiroz continuous IMX708 stream node (libcamera-rs).
//! See SPEC_imx708_svc.md.

mod camera;
mod jpeg_util;

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, bail, Context, Result};
use hiroz::context::ZContextBuilder;
use hiroz::node::ZNode;
use hiroz::parameter::{
    IntegerRange, Parameter, ParameterDescriptor, ParameterType, ParameterValue, SetParametersResult,
};
use hiroz::pubsub::ZPub;
use hiroz::{Builder, ZBuf};
use hiroz_msgs::builtin_interfaces::Time as RosTime;
use hiroz_msgs::sensor_msgs::{CompressedImage, Image};
use hiroz_msgs::std_msgs::{Header, String as RosString};
use hiroz_msgs::ZMessage;

use camera::{
    doctor_single_request, hdr_fps_warning, list_cameras, run_capture_loop, CapturedFrame,
    RAW_ENCODING,
};

const DEFAULT_ZENOH_ENDPOINT: &str = "tcp/172.31.1.252:7447";
const COMPRESSED_IMAGE_FORMAT: &str = "bgr8; jpeg compressed bgr8";
const JPEG_SCALES: &[&str] = &["1/1", "1/2", "1/4", "1/8"];
const AF_MODES: &[&str] = &["auto", "manual", "continuous"];

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JpegPipeline {
    CpuFromRaw,
    IspProcessed,
}

impl JpegPipeline {
    fn as_str(self) -> &'static str {
        match self {
            Self::CpuFromRaw => "cpu_from_raw",
            Self::IspProcessed => "isp_processed",
        }
    }

    fn parse(s: &str) -> Result<Self> {
        match s {
            "cpu_from_raw" => Ok(Self::CpuFromRaw),
            "isp_processed" => Ok(Self::IspProcessed),
            other => bail!("jpeg_pipeline must be cpu_from_raw|isp_processed, got {other}"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct StreamSettings {
    pub camera_name: String,
    pub camera_index: usize,
    pub sensor_mode: u8,
    pub hdr_enable: bool,
    pub publish_raw: bool,
    pub publish_jpeg: bool,
    pub jpeg_scale: u8,
    pub jpeg_fps: f64,
    pub jpeg_quality: u8,
    pub jpeg_pipeline: JpegPipeline,
    pub frame_id: String,
    pub status_period_sec: f64,
    pub ae_enable: bool,
    pub exposure_time_us: i64,
    pub analogue_gain: f64,
    pub awb_enable: bool,
    pub af_mode: u8,
    pub lens_position: f64,
}

impl StreamSettings {
    fn defaults(camera_name: String, camera_index: usize) -> Self {
        Self {
            camera_name: camera_name.clone(),
            camera_index,
            sensor_mode: 0,
            hdr_enable: false,
            publish_raw: false,
            publish_jpeg: true,
            jpeg_scale: 2,
            jpeg_fps: 0.0,
            jpeg_quality: 80,
            jpeg_pipeline: JpegPipeline::CpuFromRaw,
            frame_id: format!("{camera_name}/optical_frame"),
            status_period_sec: 5.0,
            ae_enable: true,
            exposure_time_us: 10_000,
            analogue_gain: 1.0,
            awb_enable: true,
            af_mode: 0,
            lens_position: 1.0,
        }
    }
}

#[derive(Debug, Clone)]
struct Topics {
    image_raw: String,
    image_jpg: String,
    status: String,
    capture_reserved: String,
}

#[derive(Debug, Clone)]
struct StatusState {
    state: &'static str,
    settings: StreamSettings,
    frame_count: u64,
    raw_pub_ok: u64,
    raw_pub_drop: u64,
    jpeg_pub_ok: u64,
    jpeg_pub_drop: u64,
    last_frame_unix_ns: Option<u128>,
    sensor_mode_label: String,
    image_raw_topic: String,
    image_jpg_topic: String,
}

type RawPublisher = ZPub<Image, <Image as ZMessage>::Serdes>;
type JpgPublisher = ZPub<CompressedImage, <CompressedImage as ZMessage>::Serdes>;
type StatusPublisher = ZPub<RosString, <RosString as ZMessage>::Serdes>;

#[tokio::main]
async fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let command = args.next().unwrap_or_else(|| "help".to_string());
    match command.as_str() {
        "doctor" => doctor(args.collect()),
        "stream" => stream(args.collect()).await,
        "-h" | "--help" | "help" => {
            print_help();
            Ok(())
        }
        other => {
            print_help();
            bail!("unknown command: {other}");
        }
    }
}

fn print_help() {
    println!("imx708_stream — Hiroz IMX708 continuous stream (libcamera-rs)");
    println!("  cargo run -p imx708_stream -- doctor [--zenoh] [--camera-index N]");
    println!(
        "  cargo run -p imx708_stream -- stream --camera-name NAME [--camera-index N] [flags]"
    );
    println!("Flags (stream):");
    println!("  --publish-raw / --no-publish-raw");
    println!("  --publish-jpeg / --no-publish-jpeg");
    println!("  --sensor-mode N");
    println!("  --jpeg-pipeline cpu_from_raw|isp_processed");
    println!("  --jpeg-scale 0..3   --jpeg-fps F   --jpeg-quality 0..100");
    println!("  --hdr / --no-hdr");
}

fn doctor(args: Vec<String>) -> Result<()> {
    let mut camera_index = 0usize;
    let mut zenoh = false;
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--camera-index" => {
                i += 1;
                camera_index = args
                    .get(i)
                    .ok_or_else(|| anyhow!("--camera-index needs value"))?
                    .parse()
                    .context("parse --camera-index")?;
            }
            "--zenoh" => zenoh = true,
            other => bail!("unknown doctor arg: {other}"),
        }
        i += 1;
    }

    let cameras = list_cameras().context("list cameras")?;
    if cameras.is_empty() {
        bail!("no libcamera cameras found");
    }
    println!("CAMERAS_GREEN count={}", cameras.len());
    for cam in &cameras {
        println!(
            "camera index={} id={} model={}",
            cam.index, cam.id, cam.model
        );
        for mode in &cam.modes {
            println!(
                "  mode {} => {}x{} {}",
                mode.index, mode.width, mode.height, mode.pixel_format
            );
        }
        if let Some(warn) = hdr_fps_warning(&cam.modes) {
            println!("HDR_FPS_WARN camera_index={} {warn}", cam.index);
        }
    }

    let selected = cameras
        .iter()
        .find(|c| c.index == camera_index)
        .ok_or_else(|| anyhow!("camera_index={camera_index} not found"))?;
    println!(
        "CAMERA_SELECT_GREEN index={} id={} modes={}",
        selected.index,
        selected.id,
        selected.modes.len()
    );

    match doctor_single_request(camera_index) {
        Ok(()) => {}
        Err(e) => {
            println!("DOCTOR_REQUEST_RED error={e:#}");
            // Still useful if sensors are held by other services.
        }
    }

    if zenoh {
        let ctx = zenoh_context_builder()
            .build()
            .map_err(|e| anyhow!("zenoh/hiroz context: {e}"))?;
        let _node = ctx
            .create_node("imx708_stream_doctor")
            .build()
            .map_err(|e| anyhow!("create doctor node: {e}"))?;
        println!("ZENOH_GREEN endpoint_default={DEFAULT_ZENOH_ENDPOINT}");
    }

    println!("DOCTOR_DONE");
    Ok(())
}

struct CliOverrides {
    camera_name: Option<String>,
    camera_index: Option<usize>,
    settings_patch: StreamSettingsPatch,
}

#[derive(Default)]
struct StreamSettingsPatch {
    sensor_mode: Option<u8>,
    hdr_enable: Option<bool>,
    publish_raw: Option<bool>,
    publish_jpeg: Option<bool>,
    jpeg_scale: Option<u8>,
    jpeg_fps: Option<f64>,
    jpeg_quality: Option<u8>,
    jpeg_pipeline: Option<JpegPipeline>,
    frame_id: Option<String>,
    status_period_sec: Option<f64>,
}

fn parse_stream_args(args: Vec<String>) -> Result<CliOverrides> {
    let mut out = CliOverrides {
        camera_name: None,
        camera_index: None,
        settings_patch: StreamSettingsPatch::default(),
    };
    let mut i = 0;
    while i < args.len() {
        match args[i].as_str() {
            "--camera-name" => {
                i += 1;
                out.camera_name = Some(
                    args.get(i)
                        .ok_or_else(|| anyhow!("--camera-name needs value"))?
                        .clone(),
                );
            }
            "--camera-index" => {
                i += 1;
                out.camera_index = Some(
                    args.get(i)
                        .ok_or_else(|| anyhow!("--camera-index needs value"))?
                        .parse()
                        .context("parse --camera-index")?,
                );
            }
            "--sensor-mode" => {
                i += 1;
                out.settings_patch.sensor_mode = Some(
                    args.get(i)
                        .ok_or_else(|| anyhow!("--sensor-mode needs value"))?
                        .parse()
                        .context("parse --sensor-mode")?,
                );
            }
            "--jpeg-pipeline" => {
                i += 1;
                out.settings_patch.jpeg_pipeline = Some(JpegPipeline::parse(
                    args.get(i)
                        .ok_or_else(|| anyhow!("--jpeg-pipeline needs value"))?,
                )?);
            }
            "--jpeg-scale" => {
                i += 1;
                out.settings_patch.jpeg_scale = Some(
                    args.get(i)
                        .ok_or_else(|| anyhow!("--jpeg-scale needs value"))?
                        .parse()
                        .context("parse --jpeg-scale")?,
                );
            }
            "--jpeg-fps" => {
                i += 1;
                out.settings_patch.jpeg_fps = Some(
                    args.get(i)
                        .ok_or_else(|| anyhow!("--jpeg-fps needs value"))?
                        .parse()
                        .context("parse --jpeg-fps")?,
                );
            }
            "--jpeg-quality" => {
                i += 1;
                out.settings_patch.jpeg_quality = Some(
                    args.get(i)
                        .ok_or_else(|| anyhow!("--jpeg-quality needs value"))?
                        .parse()
                        .context("parse --jpeg-quality")?,
                );
            }
            "--frame-id" => {
                i += 1;
                out.settings_patch.frame_id = Some(
                    args.get(i)
                        .ok_or_else(|| anyhow!("--frame-id needs value"))?
                        .clone(),
                );
            }
            "--status-period-sec" => {
                i += 1;
                out.settings_patch.status_period_sec = Some(
                    args.get(i)
                        .ok_or_else(|| anyhow!("--status-period-sec needs value"))?
                        .parse()
                        .context("parse --status-period-sec")?,
                );
            }
            "--publish-raw" => out.settings_patch.publish_raw = Some(true),
            "--no-publish-raw" => out.settings_patch.publish_raw = Some(false),
            "--publish-jpeg" => out.settings_patch.publish_jpeg = Some(true),
            "--no-publish-jpeg" => out.settings_patch.publish_jpeg = Some(false),
            "--hdr" => out.settings_patch.hdr_enable = Some(true),
            "--no-hdr" => out.settings_patch.hdr_enable = Some(false),
            other => bail!("unknown stream arg: {other}"),
        }
        i += 1;
    }
    Ok(out)
}

fn apply_patch(settings: &mut StreamSettings, patch: &StreamSettingsPatch) {
    if let Some(v) = patch.sensor_mode {
        settings.sensor_mode = v;
    }
    if let Some(v) = patch.hdr_enable {
        settings.hdr_enable = v;
    }
    if let Some(v) = patch.publish_raw {
        settings.publish_raw = v;
    }
    if let Some(v) = patch.publish_jpeg {
        settings.publish_jpeg = v;
    }
    if let Some(v) = patch.jpeg_scale {
        settings.jpeg_scale = v;
    }
    if let Some(v) = patch.jpeg_fps {
        settings.jpeg_fps = v;
    }
    if let Some(v) = patch.jpeg_quality {
        settings.jpeg_quality = v;
    }
    if let Some(v) = patch.jpeg_pipeline {
        settings.jpeg_pipeline = v;
    }
    if let Some(v) = &patch.frame_id {
        settings.frame_id = v.clone();
    }
    if let Some(v) = patch.status_period_sec {
        settings.status_period_sec = v;
    }
}

async fn stream(args: Vec<String>) -> Result<()> {
    let cli = parse_stream_args(args)?;
    let camera_name = cli
        .camera_name
        .ok_or_else(|| anyhow!("--camera-name is required"))?;
    let camera_name = sanitize_topic_segment(&camera_name);
    if camera_name.is_empty() {
        bail!("camera_name sanitizes to empty");
    }
    let camera_index = cli.camera_index.unwrap_or(0);
    let mut settings = StreamSettings::defaults(camera_name.clone(), camera_index);
    apply_patch(&mut settings, &cli.settings_patch);
    validate_settings(&settings)?;

    let hostname = sanitize_topic_segment(
        &hostname().unwrap_or_else(|_| "unknown_host".to_string()),
    );
    let topics = topics_for(&hostname, &settings.camera_name);
    let node_name = format!("imx708_stream_{}", settings.camera_name);

    let cameras = list_cameras().context("list cameras")?;
    let cam_info = cameras
        .iter()
        .find(|c| c.index == settings.camera_index)
        .ok_or_else(|| anyhow!("camera_index={} not found", settings.camera_index))?;
    let mode_label = cam_info
        .modes
        .iter()
        .find(|m| m.index == settings.sensor_mode)
        .map(|m| m.label())
        .unwrap_or_else(|| format!("mode_{}", settings.sensor_mode));

    let ctx = zenoh_context_builder()
        .build()
        .map_err(|e| anyhow!("create Hiroz context: {e}"))?;
    let node = ctx
        .create_node(&node_name)
        .build()
        .map_err(|e| anyhow!("create Hiroz node {node_name}: {e}"))?;

    declare_parameters(&node, &settings)?;

    let raw_pub: RawPublisher = node
        .create_pub::<Image>(&topics.image_raw)
        .build()
        .map_err(|e| anyhow!("create raw publisher {}: {e}", topics.image_raw))?;
    let jpg_pub: JpgPublisher = node
        .create_pub::<CompressedImage>(&topics.image_jpg)
        .build()
        .map_err(|e| anyhow!("create jpeg publisher {}: {e}", topics.image_jpg))?;
    let status_pub: StatusPublisher = node
        .create_pub::<RosString>(&topics.status)
        .build()
        .map_err(|e| anyhow!("create status publisher {}: {e}", topics.status))?;

    let status = StatusState {
        state: "streaming",
        settings: settings.clone(),
        frame_count: 0,
        raw_pub_ok: 0,
        raw_pub_drop: 0,
        jpeg_pub_ok: 0,
        jpeg_pub_drop: 0,
        last_frame_unix_ns: None,
        sensor_mode_label: mode_label,
        image_raw_topic: topics.image_raw.clone(),
        image_jpg_topic: topics.image_jpg.clone(),
    };

    println!("IMX708_STREAM_READY node={node_name}");
    println!("IMX708_TOPIC_RAW topic={}", topics.image_raw);
    println!("IMX708_TOPIC_JPG topic={}", topics.image_jpg);
    println!("IMX708_TOPIC_STATUS topic={}", topics.status);
    println!(
        "IMX708_CAPTURE_RESERVED topic={} (not implemented)",
        topics.capture_reserved
    );

    run_publish_loop(
        node,
        settings,
        topics,
        status,
        raw_pub,
        jpg_pub,
        status_pub,
        cameras,
    )
    .await
}

/// Publish loop with capture thread + param soft updates.
async fn run_publish_loop(
    node: ZNode,
    mut settings: StreamSettings,
    topics: Topics,
    mut status: StatusState,
    raw_pub: RawPublisher,
    jpg_pub: JpgPublisher,
    status_pub: StatusPublisher,
    cameras: Vec<camera::CameraInfo>,
) -> Result<()> {
    let stop = Arc::new(AtomicBool::new(false));
    let stop_thread = stop.clone();
    let (frame_tx, frame_rx) = mpsc::sync_channel::<CapturedFrame>(1);
    let capture_settings = settings.clone();
    let camera_index = settings.camera_index;
    let join = std::thread::Builder::new()
        .name("imx708-capture".into())
        .spawn(move || run_capture_loop(camera_index, capture_settings, frame_tx, stop_thread))
        .context("spawn capture thread")?;

    let mut heartbeat = tokio::time::interval(Duration::from_secs_f64(
        settings.status_period_sec.max(0.1),
    ));
    let mut reconnect_left = 3u32;

    loop {
        tokio::select! {
            _ = heartbeat.tick() => {
                if let Ok(s) = settings_from_params(&node, &settings) {
                    // Soft-apply only (publish/jpeg/AE/frame_id). Mode/hdr/pipeline logged.
                    if settings_need_reconfigure(&settings, &s) {
                        eprintln!(
                            "IMX708_RECONFIG_NOTE sensor_mode/hdr/jpeg_pipeline changes require process restart in v1"
                        );
                    }
                    settings = s;
                    status.settings = settings.clone();
                    status.sensor_mode_label = mode_label_for(&cameras, &settings);
                }
                publish_status(&status_pub, &status).await?;
            }
            _ = tokio::time::sleep(Duration::from_millis(5)) => {
                match frame_rx.try_recv() {
                    Ok(frame) => {
                        status.frame_count = status.frame_count.saturating_add(1);
                        status.last_frame_unix_ns = Some(unix_time_ns());
                        if let Err(e) = publish_frame(
                            &raw_pub,
                            &jpg_pub,
                            &topics,
                            &settings,
                            &frame,
                            &mut status,
                        ).await {
                            eprintln!("IMX708_PUBLISH_RED error={e:#}");
                        }
                    }
                    Err(mpsc::TryRecvError::Empty) => {}
                    Err(mpsc::TryRecvError::Disconnected) => {
                        status.state = "error";
                        let _ = publish_status(&status_pub, &status).await;
                        if reconnect_left == 0 {
                            stop.store(true, Ordering::Relaxed);
                            let _ = join.join();
                            bail!("capture thread ended");
                        }
                        reconnect_left -= 1;
                        eprintln!(
                            "IMX708_RECONNECT attempt_remaining={reconnect_left}"
                        );
                        stop.store(true, Ordering::Relaxed);
                        let _ = join.join();
                        // For v1 exit after disconnect; outer reconnect is best-effort via process supervisor.
                        bail!("capture thread disconnected");
                    }
                }
            }
        }
    }
}

async fn publish_frame(
    raw_pub: &RawPublisher,
    jpg_pub: &JpgPublisher,
    topics: &Topics,
    settings: &StreamSettings,
    frame: &CapturedFrame,
    status: &mut StatusState,
) -> Result<()> {
    let stamp = ros_time_from_ns(frame.stamp_ns);

    if settings.publish_raw {
        let msg = Image {
            header: Header {
                stamp: stamp.clone(),
                frame_id: settings.frame_id.clone(),
            },
            height: frame.height,
            width: frame.width,
            encoding: RAW_ENCODING.to_string(),
            is_bigendian: 0,
            step: frame.stride,
            data: ZBuf::from(frame.raw.clone()),
        };
        match raw_pub.async_publish(&msg).await {
            Ok(()) => {
                status.raw_pub_ok = status.raw_pub_ok.saturating_add(1);
                if status.raw_pub_ok % 30 == 1 {
                    println!(
                        "IMX708_PUBLISH_RAW_GREEN topic={} bytes={} seq={}",
                        topics.image_raw,
                        frame.raw.len(),
                        frame.sequence
                    );
                }
            }
            Err(e) => {
                status.raw_pub_drop = status.raw_pub_drop.saturating_add(1);
                eprintln!("IMX708_PUBLISH_RAW_DROP error={e}");
            }
        }
    }

    if settings.publish_jpeg {
        if let Some(jpeg) = &frame.jpeg {
            let msg = CompressedImage {
                header: Header {
                    stamp,
                    frame_id: settings.frame_id.clone(),
                },
                format: COMPRESSED_IMAGE_FORMAT.to_string(),
                data: ZBuf::from(jpeg.clone()),
            };
            match jpg_pub.async_publish(&msg).await {
                Ok(()) => {
                    status.jpeg_pub_ok = status.jpeg_pub_ok.saturating_add(1);
                    if status.jpeg_pub_ok % 30 == 1 {
                        println!(
                            "IMX708_PUBLISH_JPG_GREEN topic={} bytes={} seq={}",
                            topics.image_jpg,
                            jpeg.len(),
                            frame.sequence
                        );
                    }
                }
                Err(e) => {
                    status.jpeg_pub_drop = status.jpeg_pub_drop.saturating_add(1);
                    eprintln!("IMX708_PUBLISH_JPG_DROP error={e}");
                }
            }
        }
    }
    Ok(())
}

async fn publish_status(publisher: &StatusPublisher, status: &StatusState) -> Result<()> {
    let s = &status.settings;
    let line = format!(
        "state={} camera_name={} camera_index={} sensor_mode={} sensor_mode_label={} hdr_enable={} publish_raw={} publish_jpeg={} jpeg_scale={} jpeg_fps={} jpeg_quality={} jpeg_pipeline={} frame_count={} raw_pub_ok={} raw_pub_drop={} jpeg_pub_ok={} jpeg_pub_drop={} last_frame_unix_ns={} image_raw_topic={} image_jpg_topic={} heartbeat_unix_ns={}",
        status.state,
        s.camera_name,
        s.camera_index,
        s.sensor_mode,
        status.sensor_mode_label,
        s.hdr_enable as u8,
        s.publish_raw as u8,
        s.publish_jpeg as u8,
        s.jpeg_scale,
        s.jpeg_fps,
        s.jpeg_quality,
        s.jpeg_pipeline.as_str(),
        status.frame_count,
        status.raw_pub_ok,
        status.raw_pub_drop,
        status.jpeg_pub_ok,
        status.jpeg_pub_drop,
        status
            .last_frame_unix_ns
            .map(|v| v.to_string())
            .unwrap_or_default(),
        status.image_raw_topic,
        status.image_jpg_topic,
        unix_time_ns(),
    );
    let msg = RosString { data: line };
    publisher
        .async_publish(&msg)
        .await
        .map_err(|e| anyhow!("publish status: {e}"))?;
    println!("IMX708_STATUS_GREEN data={:?}", msg.data);
    Ok(())
}

fn settings_need_reconfigure(old: &StreamSettings, new: &StreamSettings) -> bool {
    old.sensor_mode != new.sensor_mode
        || old.hdr_enable != new.hdr_enable
        || old.jpeg_pipeline != new.jpeg_pipeline
        || old.camera_index != new.camera_index
        || old.camera_name != new.camera_name
}

fn mode_label_for(cameras: &[camera::CameraInfo], settings: &StreamSettings) -> String {
    cameras
        .iter()
        .find(|c| c.index == settings.camera_index)
        .and_then(|c| c.modes.iter().find(|m| m.index == settings.sensor_mode))
        .map(|m| m.label())
        .unwrap_or_else(|| format!("mode_{}", settings.sensor_mode))
}

fn declare_parameters(node: &ZNode, settings: &StreamSettings) -> Result<()> {
    declare_string(node, "camera_name", &settings.camera_name, "Topic segment")?;
    declare_int(
        node,
        "camera_index",
        settings.camera_index as i64,
        0,
        16,
        "libcamera index",
    )?;
    declare_int(
        node,
        "sensor_mode",
        settings.sensor_mode as i64,
        0,
        64,
        "Index into live mode table",
    )?;
    declare_bool(node, "hdr_enable", settings.hdr_enable, "HDR enable")?;
    declare_bool(node, "publish_raw", settings.publish_raw, "Publish raw Image")?;
    declare_bool(
        node,
        "publish_jpeg",
        settings.publish_jpeg,
        "Publish CompressedImage",
    )?;
    declare_int(
        node,
        "jpeg_scale",
        settings.jpeg_scale as i64,
        0,
        3,
        &format!("Scale enum {:?}", JPEG_SCALES),
    )?;
    declare_double(node, "jpeg_fps", settings.jpeg_fps, "0 = every frame")?;
    declare_int(
        node,
        "jpeg_quality",
        settings.jpeg_quality as i64,
        0,
        100,
        "JPEG quality",
    )?;
    declare_string(
        node,
        "jpeg_pipeline",
        settings.jpeg_pipeline.as_str(),
        "cpu_from_raw|isp_processed",
    )?;
    declare_string(node, "frame_id", &settings.frame_id, "header.frame_id")?;
    declare_double(
        node,
        "status_period_sec",
        settings.status_period_sec,
        "Status heartbeat period",
    )?;
    declare_bool(node, "ae_enable", settings.ae_enable, "Auto exposure")?;
    declare_int(
        node,
        "exposure_time_us",
        settings.exposure_time_us,
        1,
        10_000_000,
        "Manual exposure us",
    )?;
    declare_double(node, "analogue_gain", settings.analogue_gain, "Manual gain")?;
    declare_bool(node, "awb_enable", settings.awb_enable, "Auto white balance")?;
    declare_int(
        node,
        "af_mode",
        settings.af_mode as i64,
        0,
        2,
        &format!("AF {:?}", AF_MODES),
    )?;
    declare_double(
        node,
        "lens_position",
        settings.lens_position,
        "Manual lens position (dioptres)",
    )?;

    node.on_set_parameters(|params| {
        for param in params {
            if let Err(reason) = validate_param(param) {
                return SetParametersResult::failure(reason);
            }
        }
        SetParametersResult::success()
    });

    println!(
        "IMX708_PARAMETERS_GREEN node_params=camera_name,camera_index,sensor_mode,hdr_enable,publish_raw,publish_jpeg,jpeg_* ,ae_*,af_*,awb_enable,lens_position,frame_id,status_period_sec"
    );
    Ok(())
}

fn validate_param(param: &Parameter) -> std::result::Result<(), String> {
    match param.name.as_str() {
        "sensor_mode" | "camera_index" | "jpeg_scale" | "jpeg_quality" | "af_mode"
        | "exposure_time_us" => match &param.value {
            ParameterValue::Integer(v) => {
                let (lo, hi) = match param.name.as_str() {
                    "jpeg_scale" => (0, 3),
                    "jpeg_quality" => (0, 100),
                    "af_mode" => (0, 2),
                    "camera_index" => (0, 16),
                    "sensor_mode" => (0, 64),
                    "exposure_time_us" => (1, 10_000_000),
                    _ => (i64::MIN, i64::MAX),
                };
                if *v < lo || *v > hi {
                    Err(format!("{}={} out of range {lo}..{hi}", param.name, v))
                } else {
                    Ok(())
                }
            }
            _ => Err(format!("{} must be integer", param.name)),
        },
        "jpeg_pipeline" => match &param.value {
            ParameterValue::String(s) if s == "cpu_from_raw" || s == "isp_processed" => Ok(()),
            ParameterValue::String(s) => Err(format!("jpeg_pipeline invalid: {s}")),
            _ => Err("jpeg_pipeline must be string".into()),
        },
        "hdr_enable" | "publish_raw" | "publish_jpeg" | "ae_enable" | "awb_enable" => {
            match &param.value {
                ParameterValue::Bool(_) => Ok(()),
                _ => Err(format!("{} must be bool", param.name)),
            }
        }
        "jpeg_fps" | "analogue_gain" | "lens_position" | "status_period_sec" => match &param.value
        {
            ParameterValue::Double(v) if *v >= 0.0 => Ok(()),
            ParameterValue::Double(v) => Err(format!("{}={v} must be >= 0", param.name)),
            ParameterValue::Integer(v) if *v >= 0 => Ok(()),
            _ => Err(format!("{} must be non-negative number", param.name)),
        },
        "camera_name" | "frame_id" => match &param.value {
            ParameterValue::String(s) if !s.is_empty() => Ok(()),
            _ => Err(format!("{} must be non-empty string", param.name)),
        },
        _ => Ok(()),
    }
}

fn settings_from_params(node: &ZNode, base: &StreamSettings) -> Result<StreamSettings> {
    let mut s = base.clone();
    s.camera_name = param_string(node, "camera_name", &base.camera_name)?;
    s.camera_index = param_int(node, "camera_index", base.camera_index as i64)? as usize;
    s.sensor_mode = param_int(node, "sensor_mode", base.sensor_mode as i64)? as u8;
    s.hdr_enable = param_bool(node, "hdr_enable", base.hdr_enable)?;
    s.publish_raw = param_bool(node, "publish_raw", base.publish_raw)?;
    s.publish_jpeg = param_bool(node, "publish_jpeg", base.publish_jpeg)?;
    s.jpeg_scale = param_int(node, "jpeg_scale", base.jpeg_scale as i64)? as u8;
    s.jpeg_fps = param_double(node, "jpeg_fps", base.jpeg_fps)?;
    s.jpeg_quality = param_int(node, "jpeg_quality", base.jpeg_quality as i64)? as u8;
    let pipe = param_string(node, "jpeg_pipeline", base.jpeg_pipeline.as_str())?;
    s.jpeg_pipeline = JpegPipeline::parse(&pipe)?;
    s.frame_id = param_string(node, "frame_id", &base.frame_id)?;
    s.status_period_sec = param_double(node, "status_period_sec", base.status_period_sec)?;
    s.ae_enable = param_bool(node, "ae_enable", base.ae_enable)?;
    s.exposure_time_us = param_int(node, "exposure_time_us", base.exposure_time_us)?;
    s.analogue_gain = param_double(node, "analogue_gain", base.analogue_gain)?;
    s.awb_enable = param_bool(node, "awb_enable", base.awb_enable)?;
    s.af_mode = param_int(node, "af_mode", base.af_mode as i64)? as u8;
    s.lens_position = param_double(node, "lens_position", base.lens_position)?;
    validate_settings(&s)?;
    Ok(s)
}

fn validate_settings(s: &StreamSettings) -> Result<()> {
    if s.camera_name.is_empty() {
        bail!("camera_name empty");
    }
    if s.jpeg_scale > 3 {
        bail!("jpeg_scale out of range");
    }
    if s.jpeg_quality > 100 {
        bail!("jpeg_quality out of range");
    }
    if s.af_mode > 2 {
        bail!("af_mode out of range");
    }
    if s.jpeg_fps < 0.0 || s.status_period_sec <= 0.0 {
        bail!("invalid fps/period");
    }
    Ok(())
}

fn declare_string(node: &ZNode, name: &str, value: &str, description: &str) -> Result<()> {
    let mut d = ParameterDescriptor::new(name, ParameterType::String);
    d.description = description.to_string();
    node.declare_parameter(name, ParameterValue::String(value.to_string()), d)
        .map_err(|e| anyhow!("declare {name}: {e}"))?;
    Ok(())
}

fn declare_bool(node: &ZNode, name: &str, value: bool, description: &str) -> Result<()> {
    let mut d = ParameterDescriptor::new(name, ParameterType::Bool);
    d.description = description.to_string();
    node.declare_parameter(name, ParameterValue::Bool(value), d)
        .map_err(|e| anyhow!("declare {name}: {e}"))?;
    Ok(())
}

fn declare_int(
    node: &ZNode,
    name: &str,
    value: i64,
    min: i64,
    max: i64,
    description: &str,
) -> Result<()> {
    let mut d = ParameterDescriptor::new(name, ParameterType::Integer);
    d.description = description.to_string();
    d.integer_range = Some(IntegerRange {
        from_value: min,
        to_value: max,
        step: 1,
    });
    node.declare_parameter(name, ParameterValue::Integer(value), d)
        .map_err(|e| anyhow!("declare {name}: {e}"))?;
    Ok(())
}

fn declare_double(node: &ZNode, name: &str, value: f64, description: &str) -> Result<()> {
    let mut d = ParameterDescriptor::new(name, ParameterType::Double);
    d.description = description.to_string();
    node.declare_parameter(name, ParameterValue::Double(value), d)
        .map_err(|e| anyhow!("declare {name}: {e}"))?;
    Ok(())
}

fn param_string(node: &ZNode, name: &str, default: &str) -> Result<String> {
    match node.get_parameter(name) {
        Some(ParameterValue::String(s)) => Ok(s),
        Some(other) => bail!("{name} must be string, got {other:?}"),
        None => Ok(default.to_string()),
    }
}

fn param_bool(node: &ZNode, name: &str, default: bool) -> Result<bool> {
    match node.get_parameter(name) {
        Some(ParameterValue::Bool(v)) => Ok(v),
        Some(other) => bail!("{name} must be bool, got {other:?}"),
        None => Ok(default),
    }
}

fn param_int(node: &ZNode, name: &str, default: i64) -> Result<i64> {
    match node.get_parameter(name) {
        Some(ParameterValue::Integer(v)) => Ok(v),
        Some(other) => bail!("{name} must be integer, got {other:?}"),
        None => Ok(default),
    }
}

fn param_double(node: &ZNode, name: &str, default: f64) -> Result<f64> {
    match node.get_parameter(name) {
        Some(ParameterValue::Double(v)) => Ok(v),
        Some(ParameterValue::Integer(v)) => Ok(v as f64),
        Some(other) => bail!("{name} must be double, got {other:?}"),
        None => Ok(default),
    }
}

fn topics_for(hostname: &str, camera_name: &str) -> Topics {
    let base = format!("/pgwaam/{hostname}/{camera_name}");
    Topics {
        image_raw: format!("{base}/image_raw"),
        image_jpg: format!("{base}/image_jpg/compressed"),
        status: format!("{base}/status"),
        capture_reserved: format!("{base}/capture"),
    }
}

fn sanitize_topic_segment(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    for ch in value.trim().chars() {
        if ch.is_ascii_alphanumeric() || ch == '_' {
            out.push(ch);
        } else if !out.ends_with('_') {
            out.push('_');
        }
    }
    out.trim_matches('_').to_string()
}

fn hostname() -> Result<String> {
    let out = std::process::Command::new("hostname")
        .output()
        .context("hostname")?;
    if !out.status.success() {
        bail!("hostname failed");
    }
    Ok(String::from_utf8_lossy(&out.stdout).trim().to_string())
}

fn zenoh_context_builder() -> ZContextBuilder {
    match std::env::var("ZENOH_CONFIG_OVERRIDE") {
        Ok(value) => {
            println!("ZENOH_CONFIG_GREEN source=env override={value:?}");
            ZContextBuilder::default()
        }
        Err(_) => {
            println!(
                "ZENOH_CONFIG_GREEN source=builtin mode=client endpoint={DEFAULT_ZENOH_ENDPOINT}"
            );
            ZContextBuilder::default()
                .with_json("mode", "client")
                .with_json("connect/endpoints", [DEFAULT_ZENOH_ENDPOINT])
        }
    }
}

pub fn unix_time_ns() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0)
}

fn ros_time_from_ns(ns: u128) -> RosTime {
    // libcamera timestamps are often monotonic ns; if huge/host-like use directly.
    let host = unix_time_ns();
    let use_ns = if ns > 1_000_000_000_000 {
        // likely monotonic; prefer host now for ROS graph coherence
        host
    } else if ns > 0 {
        ns
    } else {
        host
    };
    RosTime {
        sec: (use_ns / 1_000_000_000) as i32,
        nanosec: (use_ns % 1_000_000_000) as u32,
    }
}
