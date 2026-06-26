use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, bail, Context as AnyhowContext, Result};
use gphoto2::{widget::RadioWidget, Context as GPhotoContext};
use hiroz::{context::ZContextBuilder, Builder};
use hiroz_msgs::std_msgs::String as RosString;

const DEFAULT_CAPTURE_DIR: &str = "captures";
const CHATTER_TOPIC: &str = "/chatter";
const DEFAULT_CHATTER_TIMEOUT_SECS: u64 = 15;

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
    let ctx = ZContextBuilder::default()
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
    let ctx = ZContextBuilder::default()
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
