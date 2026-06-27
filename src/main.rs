use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, bail, Context as AnyhowContext, Result};
use gphoto2::{camera::CameraEvent, widget::RadioWidget, Context as GPhotoContext};
use hiroz::{
    context::ZContextBuilder,
    msg::{SerdeCdrSerdes, ZMessage, ZService},
    node::ZNode,
    parameter::{
        IntegerRange, Parameter, ParameterDescriptor, ParameterType, ParameterValue,
        SetParametersResult,
    },
    pubsub::ZPub,
    Builder, ServiceTypeInfo, TypeHash, TypeInfo, ZBuf,
};
use hiroz_msgs::{
    builtin_interfaces::Time as RosTime,
    sensor_msgs::CompressedImage,
    std_msgs::{ByteMultiArray, Header, MultiArrayLayout, String as RosString},
};

const DEFAULT_CAPTURE_DIR: &str = "captures";
const CHATTER_TOPIC: &str = "/chatter";
const DEFAULT_ZENOH_ENDPOINT: &str = "tcp/172.31.1.252:7447";
const DEFAULT_CHATTER_TIMEOUT_SECS: u64 = 15;
const DEFAULT_SERVICE_TIMEOUT_SECS: u64 = 10;
const DSLR_STATUS_HEARTBEAT_SECS: u64 = 5;
const COMPRESSED_IMAGE_FORMAT: &str = "bgr8; jpeg compressed bgr8";
const DEFAULT_SHUTTERSPEED: u8 = 30;
const DEFAULT_ISO: u8 = 7;
const DEFAULT_APERTURE: u8 = 9;
const DEFAULT_IMAGEFORMAT: u8 = 24;
const DSLR_NODE_NAME: &str = "gphoto2_rs_dslr_capture";

const SHUTTERSPEEDS: &[&str] = &[
    "30", "25", "20", "15", "13", "10.3", "8", "6.3", "5", "4", "3.2", "2.5", "2", "1.6", "1.3",
    "1", "0.8", "0.6", "0.5", "0.4", "0.3", "1/4", "1/5", "1/6", "1/8", "1/10", "1/13", "1/15",
    "1/20", "1/25", "1/30", "1/40", "1/50", "1/60", "1/80", "1/100", "1/125", "1/160", "1/200",
    "1/250", "1/320", "1/400", "1/500", "1/640", "1/800", "1/1000", "1/1250", "1/1600", "1/2000",
    "1/2500", "1/3200", "1/4000",
];
const ISOS: &[&str] = &[
    "Auto", "100", "125", "160", "200", "250", "320", "400", "500", "640", "800", "1000", "1250",
    "1600", "2000", "2500", "3200", "4000", "5000", "6400",
];
const APERTURES: &[&str] = &[
    "2.8", "3.2", "3.5", "4", "4.5", "5", "5.6", "6.3", "7.1", "8", "9", "10", "11", "13", "14",
    "16", "18", "20", "22", "25", "29", "32",
];
const IMAGEFORMATS: &[&str] = &[
    "Large Fine JPEG",
    "Large Normal JPEG",
    "Medium Fine JPEG",
    "Medium Normal JPEG",
    "Small Fine JPEG",
    "Small Normal JPEG",
    "Smaller JPEG",
    "Tiny JPEG",
    "RAW + Large Fine JPEG",
    "RAW + Large Normal JPEG",
    "RAW + Medium Fine JPEG",
    "RAW + Medium Normal JPEG",
    "RAW + Small Fine JPEG",
    "RAW + Small Normal JPEG",
    "RAW + Smaller JPEG",
    "RAW + Tiny JPEG",
    "mRAW + Large Fine JPEG",
    "mRAW + Large Normal JPEG",
    "mRAW + Medium Fine JPEG",
    "mRAW + Medium Normal JPEG",
    "mRAW + Small Fine JPEG",
    "mRAW + Small Normal JPEG",
    "mRAW + Smaller JPEG",
    "mRAW + Tiny JPEG",
    "sRAW + Large Fine JPEG",
    "sRAW + Large Normal JPEG",
    "sRAW + Medium Fine JPEG",
    "sRAW + Medium Normal JPEG",
    "sRAW + Small Fine JPEG",
    "sRAW + Small Normal JPEG",
    "sRAW + Smaller JPEG",
    "sRAW + Tiny JPEG",
    "RAW",
    "mRAW",
    "sRAW",
];

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct CaptureDslrImageRequest {}

impl ZMessage for CaptureDslrImageRequest {
    type Serdes = SerdeCdrSerdes<Self>;
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
struct CaptureDslrImageResponse {
    ack_msg: String,
}

impl ZMessage for CaptureDslrImageResponse {
    type Serdes = SerdeCdrSerdes<Self>;
}

struct CaptureDslrImage;

impl ServiceTypeInfo for CaptureDslrImage {
    fn service_type_info() -> TypeInfo {
        TypeInfo::new(
            "pgwaam_msgs::srv::dds_::CaptureDslrImage_",
            TypeHash::new(
                1,
                [
                    0x27, 0xcd, 0x3e, 0x79, 0x62, 0x9c, 0x9d, 0x45, 0x78, 0xa4, 0x46, 0x3c,
                    0xe9, 0x79, 0xf6, 0x54, 0xa9, 0xf9, 0x2b, 0x15, 0x0f, 0x92, 0x53, 0x7f,
                    0x59, 0xc2, 0xf3, 0xac, 0xec, 0x7b, 0x43, 0x73,
                ],
            ),
        )
    }
}

impl ZService for CaptureDslrImage {
    type Request = CaptureDslrImageRequest;
    type Response = CaptureDslrImageResponse;
}

#[derive(Debug, Clone)]
struct DslrIdentity {
    hostname: String,
    dslr_name: String,
}

#[derive(Debug, Clone)]
struct DslrTopics {
    service: String,
    image_cr2: String,
    image_jpg: String,
    status: String,
}

#[derive(Debug, Clone)]
struct DslrCaptureSettings {
    shutterspeed: u8,
    iso: u8,
    aperture: u8,
    imageformat: u8,
}

type Cr2Publisher = ZPub<ByteMultiArray, <ByteMultiArray as ZMessage>::Serdes>;
type JpgPublisher = ZPub<CompressedImage, <CompressedImage as ZMessage>::Serdes>;
type StatusPublisher = ZPub<RosString, <RosString as ZMessage>::Serdes>;

#[derive(Debug)]
struct DslrPublishers {
    image_cr2: Cr2Publisher,
    image_jpg: JpgPublisher,
    status: StatusPublisher,
}

#[derive(Debug, Clone)]
struct DslrServiceStatus {
    state: &'static str,
    settings: DslrCaptureSettings,
    last_request_id: Option<String>,
    last_capture_unix_ns: Option<u128>,
    last_cr2_bytes: Option<usize>,
    last_jpg_bytes: Option<usize>,
    image_cr2_topic: String,
    image_jpg_topic: String,
}

#[derive(Debug, Clone)]
struct CameraPath {
    folder: String,
    name: String,
}

#[derive(Debug)]
struct DslrCaptureFiles {
    cr2_path: PathBuf,
    jpg_path: PathBuf,
    cr2_bytes: Vec<u8>,
    jpg_bytes: Vec<u8>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let command = args.next().unwrap_or_else(|| "capture".to_string());

    match command.as_str() {
        "capture" => {
            let output = args.next().map(PathBuf::from);
            let path = capture_one(output.as_deref())?;
            println!("CAPTURE_GREEN path={}", path.display());
            Ok(())
        }
        "doctor" => doctor(),
        "source" => {
            println!("{}", gphoto2_source_path()?.display());
            Ok(())
        }
        "chatter-listen" => {
            let timeout = args
                .next()
                .map(|s| parse_timeout(&s))
                .transpose()?
                .unwrap_or_else(|| Duration::from_secs(DEFAULT_CHATTER_TIMEOUT_SECS));
            listen_chatter(timeout).await
        }
        "chatter-publish" => {
            let message = args.next().unwrap_or_else(chirp_message);
            publish_chatter(message).await
        }
        "dslr-service" => run_dslr_service(false).await,
        "dslr-service-once" => run_dslr_service(true).await,
        "dslr-capture-request" => {
            let request = parse_capture_request(args.collect())?;
            request_dslr_capture(request).await
        }
        "-h" | "--help" | "help" => {
            print_help();
            Ok(())
        }
        other => {
            print_help();
            bail!("unknown command '{other}'");
        }
    }
}

fn print_help() {
    println!("Usage:");
    println!("  cargo run -- doctor");
    println!("  cargo run -- capture [output-path]");
    println!("  cargo run -- source");
    println!("  cargo run -- chatter-listen [timeout-seconds]");
    println!("  cargo run -- chatter-publish [message]");
    println!("  cargo run -- dslr-service | dslr-service-once");
    println!(
        "  cargo run -- dslr-capture-request [shutterspeed-index] [iso-index] [aperture-index] [imageformat-index] [request-id]"
    );
}

fn doctor() -> Result<()> {
    let hostname = run_text("hostname", &[])?;
    let hostname = hostname.trim();
    if hostname == "id2-rpi4" {
        println!("HOST_GREEN hostname={hostname}");
    } else {
        println!("HOST_RED hostname={hostname}");
    }

    println!("GPHOTO2_RS_GREEN dependency=gphoto2 version=3.4");
    println!(
        "GPHOTO2_RS_SOURCE_GREEN path={}",
        gphoto2_source_path()?.display()
    );

    let detected = run_text("gphoto2", &["--auto-detect"])?;
    print!("{detected}");
    if detected.contains("Canon EOS 6D") {
        println!("CAMERA_GREEN model=\"Canon EOS 6D\"");
    } else {
        println!("CAMERA_RED model=\"Canon EOS 6D\" not found by gphoto2 --auto-detect");
    }

    Ok(())
}

fn zenoh_context_builder() -> ZContextBuilder {
    match std::env::var("ZENOH_CONFIG_OVERRIDE") {
        Ok(value) => {
            println!("ZENOH_CONFIG_GREEN source=env override={value:?}");
            ZContextBuilder::default()
        }
        Err(_) => {
            println!("ZENOH_CONFIG_GREEN source=builtin mode=client endpoint={DEFAULT_ZENOH_ENDPOINT}");
            ZContextBuilder::default()
                .with_json("mode", "client")
                .with_json("connect/endpoints", [DEFAULT_ZENOH_ENDPOINT])
        }
    }
}

fn capture_one(output: Option<&Path>) -> Result<PathBuf> {
    let output = match output {
        Some(path) => path.to_path_buf(),
        None => default_capture_path()?,
    };
    if let Some(parent) = output.parent().filter(|p| !p.as_os_str().is_empty()) {
        fs::create_dir_all(parent)
            .with_context(|| format!("create capture directory {}", parent.display()))?;
    }

    let context = GPhotoContext::new().context("create gphoto2 context")?;
    let camera = context
        .autodetect_camera()
        .wait()
        .context("autodetect camera; is the Canon EOS 6D attached and unclaimed?")?;

    let format = select_jpeg(&camera).context("select JPEG image format")?;
    println!("camera_format={format}");

    let file = camera.capture_image().wait().context("capture image")?;
    println!(
        "captured_on_camera name={} folder={}",
        file.name(),
        file.folder()
    );

    camera
        .fs()
        .download_to(&file.folder(), &file.name(), &output)
        .wait()
        .with_context(|| format!("download {} to {}", file.name(), output.display()))?;

    let metadata = fs::metadata(&output).with_context(|| format!("stat {}", output.display()))?;
    if metadata.len() == 0 {
        bail!("downloaded image is empty: {}", output.display());
    }
    println!(
        "downloaded bytes={} path={}",
        metadata.len(),
        output.display()
    );

    Ok(output)
}

async fn listen_chatter(timeout: Duration) -> Result<()> {
    let ctx = zenoh_context_builder()
        .build()
        .map_err(|e| anyhow!("build hiroz context: {e}"))?;
    let node = ctx
        .create_node("gphoto2_rs_chatter_listener")
        .build()
        .map_err(|e| anyhow!("create hiroz listener node: {e}"))?;
    let sub = node
        .create_sub::<RosString>(CHATTER_TOPIC)
        .build()
        .map_err(|e| anyhow!("subscribe to {CHATTER_TOPIC}: {e}"))?;

    println!(
        "CHATTER_LISTEN waiting topic={} timeout_secs={}",
        CHATTER_TOPIC,
        timeout.as_secs()
    );

    let message = tokio::time::timeout(timeout, sub.async_recv())
        .await
        .map_err(|_| anyhow!("timed out waiting for {CHATTER_TOPIC} after {timeout:?}"))?
        .map_err(|e| anyhow!("receive {CHATTER_TOPIC}: {e}"))?;

    println!("CHATTER_SUBSCRIBE_GREEN data={:?}", message.data);
    Ok(())
}

async fn publish_chatter(message: String) -> Result<()> {
    let ctx = zenoh_context_builder()
        .build()
        .map_err(|e| anyhow!("build hiroz context: {e}"))?;
    let node = ctx
        .create_node("gphoto2_rs_chatter_talker")
        .build()
        .map_err(|e| anyhow!("create hiroz talker node: {e}"))?;
    let publisher = node
        .create_pub::<RosString>(CHATTER_TOPIC)
        .build()
        .map_err(|e| anyhow!("create publisher for {CHATTER_TOPIC}: {e}"))?;

    let msg = RosString { data: message };
    publisher
        .async_publish(&msg)
        .await
        .map_err(|e| anyhow!("publish {CHATTER_TOPIC}: {e}"))?;

    println!("CHATTER_PUBLISH_GREEN data={:?}", msg.data);
    Ok(())
}

fn declare_dslr_parameters(node: &ZNode) -> Result<()> {
    declare_enum_parameter(
        node,
        "shutterspeed",
        DEFAULT_SHUTTERSPEED,
        SHUTTERSPEEDS,
        "Canon camera shutter speed enum index",
    )?;
    declare_enum_parameter(
        node,
        "iso",
        DEFAULT_ISO,
        ISOS,
        "Canon camera ISO enum index",
    )?;
    declare_enum_parameter(
        node,
        "aperture",
        DEFAULT_APERTURE,
        APERTURES,
        "Canon camera aperture enum index",
    )?;
    declare_enum_parameter(
        node,
        "imageformat",
        DEFAULT_IMAGEFORMAT,
        IMAGEFORMATS,
        "Canon camera image format enum index",
    )?;

    node.on_set_parameters(|params| {
        for param in params {
            let valid = match param.name.as_str() {
                "shutterspeed" => validate_parameter_index(param, SHUTTERSPEEDS),
                "iso" => validate_parameter_index(param, ISOS),
                "aperture" => validate_parameter_index(param, APERTURES),
                "imageformat" => validate_parameter_index(param, IMAGEFORMATS),
                _ => Ok(()),
            };
            if let Err(reason) = valid {
                return SetParametersResult::failure(reason);
            }
        }
        SetParametersResult::success()
    });

    println!(
        "DSLR_PARAMETERS_GREEN node={} params=shutterspeed,iso,aperture,imageformat",
        DSLR_NODE_NAME
    );
    Ok(())
}

fn declare_enum_parameter(
    node: &ZNode,
    name: &str,
    default: u8,
    values: &[&str],
    description: &str,
) -> Result<()> {
    let mut descriptor = ParameterDescriptor::new(name, ParameterType::Integer);
    descriptor.description = description.to_string();
    descriptor.additional_constraints = values
        .iter()
        .enumerate()
        .map(|(index, value)| format!("{index}:{value}"))
        .collect::<Vec<_>>()
        .join(" ");
    descriptor.integer_range = Some(IntegerRange {
        from_value: 0,
        to_value: values.len() as i64 - 1,
        step: 1,
    });
    node.declare_parameter(name, ParameterValue::Integer(default as i64), descriptor)
        .map_err(|e| anyhow!("declare DSLR parameter {name}: {e}"))?;
    Ok(())
}

fn validate_parameter_index(param: &Parameter, values: &[&str]) -> std::result::Result<(), String> {
    match &param.value {
        ParameterValue::Integer(value) if *value >= 0 && (*value as usize) < values.len() => Ok(()),
        ParameterValue::Integer(value) => Err(format!(
            "{}={} is out of range 0..{}",
            param.name,
            value,
            values.len().saturating_sub(1)
        )),
        other => Err(format!(
            "{} must be an integer enum index, got {:?}",
            param.name, other
        )),
    }
}

fn dslr_settings_from_parameters(node: &ZNode) -> Result<DslrCaptureSettings> {
    Ok(DslrCaptureSettings {
        shutterspeed: parameter_u8(node, "shutterspeed", DEFAULT_SHUTTERSPEED, SHUTTERSPEEDS)?,
        iso: parameter_u8(node, "iso", DEFAULT_ISO, ISOS)?,
        aperture: parameter_u8(node, "aperture", DEFAULT_APERTURE, APERTURES)?,
        imageformat: parameter_u8(node, "imageformat", DEFAULT_IMAGEFORMAT, IMAGEFORMATS)?,
    })
}

fn parameter_u8(node: &ZNode, name: &str, default: u8, values: &[&str]) -> Result<u8> {
    let value = match node.get_parameter(name) {
        Some(ParameterValue::Integer(value)) => value,
        Some(other) => bail!("DSLR parameter {name} must be integer, got {other:?}"),
        None => default as i64,
    };
    if value < 0 || value as usize >= values.len() {
        bail!(
            "DSLR parameter {name}={value} out of range 0..{}",
            values.len().saturating_sub(1)
        );
    }
    Ok(value as u8)
}

async fn run_dslr_service(once: bool) -> Result<()> {
    let identity = dslr_identity()?;
    let topics = dslr_topics(&identity);
    let ctx = zenoh_context_builder()
        .build()
        .map_err(|e| anyhow!("build hiroz context: {e}"))?;
    let node = ctx
        .create_node(DSLR_NODE_NAME)
        .build()
        .map_err(|e| anyhow!("create Hiroz DSLR node: {e}"))?;
    declare_dslr_parameters(&node)?;
    let mut service = node
        .create_service::<CaptureDslrImage>(&topics.service)
        .build()
        .map_err(|e| anyhow!("create DSLR capture service {}: {e}", topics.service))?;
    let publishers = DslrPublishers {
        image_cr2: node
            .create_pub::<ByteMultiArray>(&topics.image_cr2)
            .build()
            .map_err(|e| anyhow!("create CR2 publisher {}: {e}", topics.image_cr2))?,
        image_jpg: node
            .create_pub::<CompressedImage>(&topics.image_jpg)
            .build()
            .map_err(|e| anyhow!("create JPEG publisher {}: {e}", topics.image_jpg))?,
        status: node
            .create_pub::<RosString>(&topics.status)
            .build()
            .map_err(|e| anyhow!("create status publisher {}: {e}", topics.status))?,
    };
    let mut status = DslrServiceStatus::new(&topics, dslr_settings_from_parameters(&node)?);
    let mut heartbeat = tokio::time::interval(Duration::from_secs(DSLR_STATUS_HEARTBEAT_SECS));

    println!("DSLR_SERVICE_READY service={}", topics.service);
    println!("DSLR_TOPIC_CR2 topic={}", topics.image_cr2);
    println!("DSLR_TOPIC_JPG topic={}", topics.image_jpg);
    println!("DSLR_TOPIC_STATUS topic={}", topics.status);

    loop {
        let request = tokio::select! {
            _ = heartbeat.tick() => {
                status.settings = dslr_settings_from_parameters(&node)?;
                publish_dslr_status(&publishers.status, &status).await?;
                continue;
            }
            request = service.async_take_request() => {
                request.map_err(|e| anyhow!("take DSLR capture request: {e}"))?
            }
        };
        let settings = dslr_settings_from_parameters(&node)?;
        let request_id = default_request_id();
        let response = capture_ack(&request_id, &settings);

        request
            .reply(&response)
            .await
            .map_err(|e| anyhow!("reply to DSLR capture request: {e}"))?;
        println!(
            "DSLR_ACK_GREEN request_id={} shutterspeed={} iso={} aperture={} imageformat={}",
            request_id,
            enum_label(SHUTTERSPEEDS, settings.shutterspeed, "shutterspeed").unwrap_or("invalid"),
            enum_label(ISOS, settings.iso, "iso").unwrap_or("invalid"),
            enum_label(APERTURES, settings.aperture, "aperture").unwrap_or("invalid"),
            enum_label(IMAGEFORMATS, settings.imageformat, "imageformat").unwrap_or("invalid")
        );
        status.settings = settings.clone();
        status.mark_capturing(&request_id);
        publish_dslr_status(&publishers.status, &status).await?;

        match capture_dslr_pair(&request_id, &settings, &identity) {
            Ok(files) => {
                let cr2_len = files.cr2_bytes.len();
                let jpg_len = files.jpg_bytes.len();
                publish_dslr_images(&publishers, &topics, &identity, files).await?;
                status.mark_idle_after_capture(cr2_len, jpg_len);
                publish_dslr_status(&publishers.status, &status).await?;
            }
            Err(err) => {
                status.mark_error();
                let _ = publish_dslr_status(&publishers.status, &status).await;
                println!("DSLR_CAPTURE_RED request_id={} error={err:#}", request_id);
                return Err(err);
            }
        }

        if once {
            break;
        }
    }

    Ok(())
}

async fn request_dslr_capture(_request: CaptureDslrImageRequest) -> Result<()> {
    let identity = dslr_identity()?;
    let topics = dslr_topics(&identity);
    let ctx = zenoh_context_builder()
        .build()
        .map_err(|e| anyhow!("build hiroz context: {e}"))?;
    let node = ctx
        .create_node("gphoto2_rs_dslr_capture_client")
        .build()
        .map_err(|e| anyhow!("create Hiroz DSLR client node: {e}"))?;
    let client = node
        .create_client::<CaptureDslrImage>(&topics.service)
        .build()
        .map_err(|e| anyhow!("create DSLR capture client {}: {e}", topics.service))?;

    println!(
        "DSLR_REQUEST service={} request=empty",
        topics.service
    );

    let response = client
        .call_with_timeout(
            &CaptureDslrImageRequest {},
            Duration::from_secs(DEFAULT_SERVICE_TIMEOUT_SECS),
        )
        .await
        .map_err(|e| anyhow!("call DSLR capture service {}: {e}", topics.service))?;

    println!("DSLR_ACK_RECEIVED_GREEN ack_msg={}", response.ack_msg);
    Ok(())
}

fn parse_capture_request(args: Vec<String>) -> Result<CaptureDslrImageRequest> {
    if !args.is_empty() {
        println!("DSLR_REQUEST_ARGS_IGNORED reason=service request is now empty; use ROS parameters on {DSLR_NODE_NAME} for camera settings");
    }
    Ok(CaptureDslrImageRequest {})
}

fn capture_ack(request_id: &str, settings: &DslrCaptureSettings) -> CaptureDslrImageResponse {
    CaptureDslrImageResponse {
        ack_msg: format!(
            "ACK capture accepted request_id={} shutterspeed={} iso={} aperture={} imageformat={}",
            request_id,
            enum_label(SHUTTERSPEEDS, settings.shutterspeed, "shutterspeed").unwrap_or("invalid"),
            enum_label(ISOS, settings.iso, "iso").unwrap_or("invalid"),
            enum_label(APERTURES, settings.aperture, "aperture").unwrap_or("invalid"),
            enum_label(IMAGEFORMATS, settings.imageformat, "imageformat").unwrap_or("invalid")
        ),
    }
}

fn capture_dslr_pair(
    request_id: &str,
    settings: &DslrCaptureSettings,
    identity: &DslrIdentity,
) -> Result<DslrCaptureFiles> {
    let output_dir = PathBuf::from(DEFAULT_CAPTURE_DIR).join(request_id);
    fs::create_dir_all(&output_dir)
        .with_context(|| format!("create capture directory {}", output_dir.display()))?;

    let context = GPhotoContext::new().context("create gphoto2 context")?;
    let camera = context
        .autodetect_camera()
        .wait()
        .context("autodetect camera; is the Canon EOS 6D attached and unclaimed?")?;

    let actual_shutter = set_radio_choice_by_index(
        &camera,
        &["shutterspeed"],
        settings.shutterspeed,
        SHUTTERSPEEDS,
        "shutterspeed",
    )?;
    let actual_iso = set_radio_choice_by_index(&camera, &["iso"], settings.iso, ISOS, "iso")?;
    let actual_aperture = set_radio_choice_by_index(
        &camera,
        &["aperture", "f-number"],
        settings.aperture,
        APERTURES,
        "aperture",
    )?;
    let actual_format = set_radio_choice_by_index(
        &camera,
        &["imageformat", "imageformatsd", "imageformatcf"],
        settings.imageformat,
        IMAGEFORMATS,
        "imageformat",
    )?;

    println!(
        "DSLR_CAMERA_CONFIG_GREEN shutterspeed={} iso={} aperture={} imageformat={}",
        actual_shutter, actual_iso, actual_aperture, actual_format
    );

    let first = camera.capture_image().wait().context("capture image")?;
    let mut camera_paths = vec![CameraPath::from_gphoto(&first)];
    let started = Instant::now();
    let mut saw_complete = false;

    while started.elapsed() < Duration::from_secs(15) {
        match camera
            .wait_event(Duration::from_millis(1000))
            .wait()
            .context("wait for camera file events")?
        {
            CameraEvent::NewFile(file) => camera_paths.push(CameraPath::from_gphoto(&file)),
            CameraEvent::CaptureComplete => {
                saw_complete = true;
                if camera_paths.len() >= 2 {
                    break;
                }
            }
            CameraEvent::Timeout if saw_complete || camera_paths.len() >= 2 => break,
            CameraEvent::Timeout => {}
            other => println!("DSLR_CAMERA_EVENT event={other:?}"),
        }
    }

    camera_paths.sort_by(|a, b| a.name.cmp(&b.name).then(a.folder.cmp(&b.folder)));
    camera_paths.dedup_by(|a, b| a.folder == b.folder && a.name == b.name);

    let capture_stamp = capture_file_stamp();
    let mut cr2_path = None;
    let mut jpg_path = None;
    for (index, camera_path) in camera_paths.into_iter().enumerate() {
        let local_path = output_dir.join(local_capture_filename(
            &request_id,
            &capture_stamp,
            index,
            &camera_path.name,
        ));
        camera
            .fs()
            .download_to(&camera_path.folder, &camera_path.name, &local_path)
            .wait()
            .with_context(|| {
                format!(
                    "download camera file {}/{} to {}",
                    camera_path.folder,
                    camera_path.name,
                    local_path.display()
                )
            })?;
        println!(
            "DSLR_DOWNLOAD_GREEN file={} path={}",
            camera_path.name,
            local_path.display()
        );

        match extension_lower(&local_path).as_deref() {
            Some("cr2") => cr2_path = Some(local_path),
            Some("jpg") | Some("jpeg") => jpg_path = Some(local_path),
            _ => {}
        }
    }

    let cr2_path = cr2_path.ok_or_else(|| anyhow!("capture did not produce a .cr2 file"))?;
    let jpg_path = jpg_path.ok_or_else(|| anyhow!("capture did not produce a .jpg file"))?;
    let cr2_bytes = fs::read(&cr2_path).with_context(|| format!("read {}", cr2_path.display()))?;
    let jpg_bytes = fs::read(&jpg_path).with_context(|| format!("read {}", jpg_path.display()))?;

    if cr2_bytes.is_empty() {
        bail!("downloaded CR2 is empty: {}", cr2_path.display());
    }
    if jpg_bytes.is_empty() {
        bail!("downloaded JPEG is empty: {}", jpg_path.display());
    }

    println!(
        "DSLR_CAPTURE_GREEN camera={} cr2_bytes={} jpg_bytes={} cr2_path={} jpg_path={}",
        identity.dslr_name,
        cr2_bytes.len(),
        jpg_bytes.len(),
        cr2_path.display(),
        jpg_path.display()
    );

    Ok(DslrCaptureFiles {
        cr2_path,
        jpg_path,
        cr2_bytes,
        jpg_bytes,
    })
}

async fn publish_dslr_images(
    publishers: &DslrPublishers,
    topics: &DslrTopics,
    identity: &DslrIdentity,
    files: DslrCaptureFiles,
) -> Result<()> {
    let cr2_len = files.cr2_bytes.len();
    let jpg_len = files.jpg_bytes.len();
    let cr2_msg = ByteMultiArray {
        layout: MultiArrayLayout {
            dim: Vec::new(),
            data_offset: 0,
        },
        data: ZBuf::from(files.cr2_bytes),
    };
    let jpg_msg = CompressedImage {
        header: Header {
            stamp: ros_time_now(),
            frame_id: format!("{}/{}/optical_frame", identity.hostname, identity.dslr_name),
        },
        format: COMPRESSED_IMAGE_FORMAT.to_string(),
        data: ZBuf::from(files.jpg_bytes),
    };

    publishers
        .image_cr2
        .async_publish(&cr2_msg)
        .await
        .map_err(|e| anyhow!("publish CR2 image to {}: {e}", topics.image_cr2))?;
    println!(
        "DSLR_PUBLISH_CR2_GREEN topic={} bytes={} source={}",
        topics.image_cr2,
        cr2_len,
        files.cr2_path.display()
    );

    publishers
        .image_jpg
        .async_publish(&jpg_msg)
        .await
        .map_err(|e| anyhow!("publish JPEG image to {}: {e}", topics.image_jpg))?;
    println!(
        "DSLR_PUBLISH_JPG_GREEN topic={} bytes={} source={}",
        topics.image_jpg,
        jpg_len,
        files.jpg_path.display()
    );

    Ok(())
}

async fn publish_dslr_status(
    publisher: &StatusPublisher,
    status: &DslrServiceStatus,
) -> Result<()> {
    let data = status.to_status_line();
    let msg = RosString { data };
    publisher
        .async_publish(&msg)
        .await
        .map_err(|e| anyhow!("publish DSLR status heartbeat: {e}"))?;
    println!("DSLR_STATUS_GREEN data={:?}", msg.data);
    Ok(())
}

impl DslrServiceStatus {
    fn new(topics: &DslrTopics, settings: DslrCaptureSettings) -> Self {
        Self {
            state: "idle",
            settings,
            last_request_id: None,
            last_capture_unix_ns: None,
            last_cr2_bytes: None,
            last_jpg_bytes: None,
            image_cr2_topic: topics.image_cr2.clone(),
            image_jpg_topic: topics.image_jpg.clone(),
        }
    }

    fn mark_capturing(&mut self, request_id: &str) {
        self.state = "capturing";
        self.last_request_id = Some(request_id.to_string());
    }

    fn mark_idle_after_capture(&mut self, cr2_bytes: usize, jpg_bytes: usize) {
        self.state = "idle";
        self.last_capture_unix_ns = Some(unix_time_ns());
        self.last_cr2_bytes = Some(cr2_bytes);
        self.last_jpg_bytes = Some(jpg_bytes);
    }

    fn mark_error(&mut self) {
        self.state = "error";
    }

    fn to_status_line(&self) -> String {
        format!(
            "state={} heartbeat_unix_ns={} shutterspeed={} iso={} aperture={} imageformat={} last_request_id={} last_capture_unix_ns={} last_cr2_bytes={} last_jpg_bytes={} image_cr2_topic={} image_jpg_topic={}",
            self.state,
            unix_time_ns(),
            enum_label(SHUTTERSPEEDS, self.settings.shutterspeed, "shutterspeed")
                .unwrap_or("invalid"),
            enum_label(ISOS, self.settings.iso, "iso").unwrap_or("invalid"),
            enum_label(APERTURES, self.settings.aperture, "aperture").unwrap_or("invalid"),
            enum_label(IMAGEFORMATS, self.settings.imageformat, "imageformat")
                .unwrap_or("invalid"),
            self.last_request_id.as_deref().unwrap_or(""),
            self.last_capture_unix_ns
                .map(|value| value.to_string())
                .unwrap_or_default(),
            self.last_cr2_bytes
                .map(|value| value.to_string())
                .unwrap_or_default(),
            self.last_jpg_bytes
                .map(|value| value.to_string())
                .unwrap_or_default(),
            self.image_cr2_topic,
            self.image_jpg_topic
        )
    }
}

impl CameraPath {
    fn from_gphoto(path: &gphoto2::file::CameraFilePath) -> Self {
        Self {
            folder: path.folder().into_owned(),
            name: path.name().into_owned(),
        }
    }
}

fn dslr_identity() -> Result<DslrIdentity> {
    let hostname = run_text("hostname", &[])?.trim().to_string();
    let detected = run_text("gphoto2", &["--auto-detect"])?;
    let dslr_model = detected
        .lines()
        .find_map(|line| {
            if line.contains("usb:") && !line.starts_with("Model") {
                line.split("usb:")
                    .next()
                    .map(str::trim)
                    .filter(|s| !s.is_empty())
            } else {
                None
            }
        })
        .unwrap_or("Canon EOS 6D");

    Ok(DslrIdentity {
        hostname: sanitize_topic_segment(&hostname),
        dslr_name: sanitize_topic_segment(dslr_model),
    })
}

fn dslr_topics(identity: &DslrIdentity) -> DslrTopics {
    let base = format!("/pgwaam/{}/{}", identity.hostname, identity.dslr_name);
    DslrTopics {
        service: format!("{base}/capture"),
        image_cr2: format!("{base}/image_cr2"),
        image_jpg: format!("{base}/image_jpg/compressed"),
        status: format!("{base}/status"),
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
    let out = out.trim_matches('_').to_string();
    if out.is_empty() {
        "unknown".to_string()
    } else {
        out
    }
}

fn enum_label<'a>(values: &'a [&str], index: u8, name: &str) -> Result<&'a str> {
    values.get(index as usize).copied().ok_or_else(|| {
        anyhow!(
            "{name} enum index {index} is out of range 0..{}",
            values.len() - 1
        )
    })
}

fn set_radio_choice_by_index(
    camera: &gphoto2::Camera,
    keys: &[&str],
    index: u8,
    expected_values: &[&str],
    label: &str,
) -> Result<String> {
    let expected = enum_label(expected_values, index, label)?;

    for key in keys {
        let Ok(widget) = camera.config_key::<RadioWidget>(key).wait() else {
            continue;
        };
        let choices = widget.choices_iter().collect::<Vec<_>>();
        let choice = choices
            .get(index as usize)
            .cloned()
            .or_else(|| {
                let normalized_expected = normalize_choice(expected);
                choices
                    .iter()
                    .find(|choice| normalize_choice(choice) == normalized_expected)
                    .cloned()
            })
            .ok_or_else(|| {
                anyhow!(
                    "{label} index {index} ({expected}) is not available in camera config key {key}; choices={choices:?}"
                )
            })?;

        widget
            .set_choice(&choice)
            .with_context(|| format!("set {key} to {choice}"))?;
        camera
            .set_config(&widget)
            .wait()
            .with_context(|| format!("apply {key}"))?;
        return Ok(choice);
    }

    bail!("no camera config key found for {label}: tried {keys:?}");
}

fn normalize_choice(value: &str) -> String {
    value
        .chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .flat_map(char::to_lowercase)
        .collect()
}

fn extension_lower(path: &Path) -> Option<String> {
    path.extension()
        .and_then(|ext| ext.to_str())
        .map(|ext| ext.to_ascii_lowercase())
}

fn local_capture_filename(
    request_id: &str,
    capture_stamp: &str,
    index: usize,
    camera_filename: &str,
) -> String {
    let extension = Path::new(camera_filename)
        .extension()
        .and_then(|ext| ext.to_str())
        .map(|ext| sanitize_filename_segment(ext).to_ascii_lowercase())
        .filter(|ext| !ext.is_empty())
        .unwrap_or_else(|| "bin".to_string());
    let remote_stem = Path::new(camera_filename)
        .file_stem()
        .and_then(|stem| stem.to_str())
        .map(sanitize_filename_segment)
        .filter(|stem| !stem.is_empty())
        .unwrap_or_else(|| "camera-file".to_string());

    format!(
        "{}_{}_{}_{remote_stem}.{extension}",
        sanitize_filename_segment(request_id),
        capture_stamp,
        index + 1
    )
}

fn sanitize_filename_segment(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    for ch in value.trim().chars() {
        if ch.is_ascii_alphanumeric() || ch == '-' || ch == '_' || ch == '.' {
            out.push(ch);
        } else {
            out.push('_');
        }
    }
    out.trim_matches('.').trim_matches('_').to_string()
}

fn capture_file_stamp() -> String {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    format!("{}{:09}", now.as_secs(), now.subsec_nanos())
}

fn unix_time_ns() -> u128 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_nanos())
        .unwrap_or_default()
}

fn ros_time_now() -> RosTime {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap_or_default();
    RosTime {
        sec: now.as_secs().min(i32::MAX as u64) as i32,
        nanosec: now.subsec_nanos(),
    }
}

fn default_request_id() -> String {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs())
        .unwrap_or_default();
    format!("dslr-{now}")
}

fn parse_timeout(s: &str) -> Result<Duration> {
    let secs = s
        .parse::<u64>()
        .with_context(|| format!("parse timeout seconds from '{s}'"))?;
    Ok(Duration::from_secs(secs))
}

fn chirp_message() -> String {
    let datetime = run_text("date", &["--iso-8601=seconds"])
        .map(|s| s.trim().to_string())
        .unwrap_or_else(|_| {
            let secs = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or_default();
            format!("unix:{secs}")
        });
    format!("CHIRP CHIRP! From gphoto2-rs :: {datetime}")
}

fn select_jpeg(camera: &gphoto2::Camera) -> Result<String> {
    for key in ["imageformat", "imageformatsd", "imageformatcf"] {
        let Ok(widget) = camera.config_key::<RadioWidget>(key).wait() else {
            continue;
        };
        let choice = widget
            .choices_iter()
            .find(|choice| {
                let label = choice.to_ascii_lowercase();
                label.contains("jpeg") && !label.contains("raw")
            })
            .ok_or_else(|| anyhow!("no JPEG-only choice in {key}"))?;

        widget
            .set_choice(&choice)
            .with_context(|| format!("set {key} to {choice}"))?;
        camera
            .set_config(&widget)
            .wait()
            .with_context(|| format!("apply {key}"))?;
        return Ok(choice);
    }

    bail!("no imageformat* config key exposes JPEG choices");
}

fn default_capture_path() -> Result<PathBuf> {
    let now = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|err| anyhow!("system clock is before UNIX_EPOCH: {err}"))?;
    Ok(PathBuf::from(DEFAULT_CAPTURE_DIR).join(format!("canon-eos-6d-{}.jpg", now.as_secs())))
}

fn gphoto2_source_path() -> Result<PathBuf> {
    let home = std::env::var("HOME").context("HOME is not set")?;
    let registry_src = Path::new(&home).join(".cargo/registry/src");
    let entries = fs::read_dir(&registry_src)
        .with_context(|| format!("read cargo registry source dir {}", registry_src.display()))?;

    for entry in entries {
        let entry = entry?;
        let candidate = entry.path().join("gphoto2-3.4.1");
        if candidate.join("src/lib.rs").is_file() {
            return Ok(candidate);
        }
    }

    bail!(
        "could not find gphoto2-3.4.1 source under {}",
        registry_src.display()
    );
}

fn run_text(program: &str, args: &[&str]) -> Result<String> {
    let output = Command::new(program)
        .args(args)
        .output()
        .with_context(|| format!("run {program} {}", args.join(" ")))?;

    let mut text = String::from_utf8_lossy(&output.stdout).to_string();
    text.push_str(&String::from_utf8_lossy(&output.stderr));

    if !output.status.success() {
        bail!("{program} {} failed: {text}", args.join(" "));
    }

    Ok(text)
}
