use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{anyhow, bail, Context as AnyhowContext, Result};
use gphoto2::widget::RadioWidget;
use gphoto2::Context;

const DEFAULT_CAPTURE_DIR: &str = "captures";

fn main() -> Result<()> {
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

    let context = Context::new().context("create gphoto2 context")?;
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
