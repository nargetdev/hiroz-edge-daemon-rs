# imx708_stream node

Continuous camera streaming node built on
[libcamera-rs](https://github.com/lit-robotics/libcamera-rs) and Hiroz.
It joins the ROS 2 graph as a normal node over the Zenoh router, so
everything below works with stock `ros2` tooling running rmw_zenoh.

**One process owns one camera.** A host with two camera modules runs two
instances with different `--camera-name` / `--camera-index`.

Validated on `blackfinfive` (Pi 5, PiSP): imx708_wide at camera index 1,
ov5647 at index 0.

## Build

```sh
# on the camera host (needs libcamera-dev, clang, pkg-config)
cargo build --release -p imx708_stream
```

## CLI

### `doctor` — health check

Lists cameras and their raw mode tables, warns when the mode table looks
HDR-skewed, and (when the camera is free) performs one configure → capture
round-trip:

```sh
./target/release/imx708_stream doctor [--camera-index N] [--zenoh]
```

`--zenoh` additionally brings up a Hiroz node to prove router
connectivity. Green-path markers: `CAMERAS_GREEN`, `CAMERA_SELECT_GREEN`,
`DOCTOR_REQUEST_GREEN`, `DOCTOR_DONE`.

### `stream` — long-running node

```sh
./target/release/imx708_stream stream \
  --camera-name front_wide \
  --camera-index 1 \
  [--sensor-mode N] \
  [--jpeg-pipeline cpu_from_raw|isp_processed] \
  [--jpeg-scale 0..3]        # 0=1/1, 1=1/2, 2=1/4 (default), 3=1/8
  [--jpeg-fps F]             # 0 = every frame
  [--jpeg-quality 0..100] \
  [--publish-raw|--no-publish-raw] \
  [--publish-jpeg|--no-publish-jpeg] \
  [--hdr|--no-hdr] \
  [--frame-id ID] [--status-period-sec S]
```

`--camera-name` is required and becomes a topic segment. `camera_name`
and `camera_index` are restart-only; most other settings are also ROS
parameters (see below).

### Zenoh session

By default the node connects as a Zenoh client to
`tcp/172.31.1.252:7447`. Override with:

```sh
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/HOST:7447"]'
```

## Graph identity

Node name: `imx708_stream_{camera_name}`. Topics:

| Topic | Type | Notes |
|---|---|---|
| `/pgwaam/{hostname}/{camera_name}/image_jpg/compressed` | `sensor_msgs/CompressedImage` | on by default; `format` = `bgr8; jpeg compressed bgr8` |
| `/pgwaam/{hostname}/{camera_name}/image_raw` | `sensor_msgs/Image` | only when `publish_raw` is true; `encoding` = resolved libcamera format |
| `/pgwaam/{hostname}/{camera_name}/status` | `std_msgs/String` | key=value heartbeat every `status_period_sec` (default 5 s) |
| `/pgwaam/{hostname}/{camera_name}/capture` | — | reserved, not implemented in v1 |

Example (blackfinfive):

```text
/pgwaam/blackfinfive/front_wide/image_jpg/compressed
/pgwaam/blackfinfive/front_wide/image_raw
/pgwaam/blackfinfive/front_wide/status
```

## Raw formats: what `encoding` will actually say

The node always opens a single raw sensor stream and reports the format
the pipeline **resolves**, not the one requested:

- The Pi 5 imx708_wide advertises `SRGGB10_CSI2P` (not `SBGGR10_CSI2P`).
- PiSP resolves explicit 10-bit CSI2P requests to compressed raw
  (`BGGR_PISP_COMP1`), which cannot be demosaiced on the CPU. The
  `cpu_from_raw` pipeline detects this and falls back to 16-bit
  uncompressed Bayer — on blackfinfive that lands on `SBGGR16`
  (width×2 stride, e.g. 1536×864 → step 3072, 2 654 208 bytes/frame).
- Demosaic CFA order (RGGB/BGGR/GRBG/GBRG) always follows the resolved
  format, so `image_raw.encoding` is trustworthy.

## JPEG pipelines

| `jpeg_pipeline` | Behavior |
|---|---|
| `cpu_from_raw` (default) | Single raw stream; CPU unpack → nearest-neighbor demosaic → scale → JPEG. Preview-grade: no white balance, expect a green cast. Cheapest on the ISP. |
| `isp_processed` | Dual stream: raw + ViewFinder `RGB888` at the JPEG size. The ISP applies AWB/CCM, so colors are correct. Hard-fails at startup if the dual-stream configuration doesn't validate. |

## ROS parameters

Runtime-settable via `ros2 param set /imx708_stream_{camera_name} ...`.
Invalid values are rejected (soft-reject: the running stream is kept).

| Param | Type | Default | Notes |
|---|---|---|---|
| `camera_name` | string | *(launch)* | restart-only |
| `camera_index` | int | 0 | restart-only |
| `sensor_mode` | int | 0 | index into the live mode table |
| `hdr_enable` | bool | false | restart to take effect in v1 |
| `publish_raw` | bool | false | applies live |
| `publish_jpeg` | bool | true | applies live |
| `jpeg_pipeline` | string | `cpu_from_raw` | restart to take effect in v1 |
| `jpeg_scale` | int | 2 | 0..3 → 1/1, 1/2, 1/4, 1/8 |
| `jpeg_fps` | double | 0.0 | 0 = every frame |
| `jpeg_quality` | int | 80 | 0–100; out-of-range rejected |
| `frame_id` | string | `{camera_name}/optical_frame` | |
| `status_period_sec` | double | 5.0 | |
| `ae_enable` | bool | true | |
| `exposure_time_us` | int | 10000 | when AE off |
| `analogue_gain` | double | 1.0 | when AE off |
| `awb_enable` | bool | true | |
| `af_mode` | int | 0 | 0=auto, 1=manual, 2=continuous |
| `lens_position` | double | 1.0 | dioptres, when `af_mode=1` |

Example session:

```sh
ros2 param list /imx708_stream_front_wide
ros2 param set  /imx708_stream_front_wide jpeg_quality 40    # applied live
ros2 param set  /imx708_stream_front_wide publish_raw true   # raw topic starts
ros2 param set  /imx708_stream_front_wide jpeg_quality 400   # rejected, out of range
```

## Status heartbeat

One `key=value` line per period on `.../status`:

```text
state=streaming camera_name=front_wide camera_index=1 sensor_mode=0
sensor_mode_label=1536x864_SRGGB10_CSI2P hdr_enable=0 publish_raw=0
publish_jpeg=1 jpeg_scale=2 jpeg_fps=0 jpeg_quality=80
jpeg_pipeline=cpu_from_raw frame_count=114 raw_pub_ok=0 raw_pub_drop=0
jpeg_pub_ok=114 jpeg_pub_drop=0 last_frame_unix_ns=…
image_raw_topic=… image_jpg_topic=… heartbeat_unix_ns=…
```

`state` is one of `idle|streaming|reconfiguring|error`. Overruns use a
depth-1 latest-wins queue; drops show up in `*_pub_drop`.

## Run at boot (systemd)

Units for both blackfinfive cameras live in `imx708_stream/systemd/`
(JPEG at 1/8 scale):

```sh
cargo build --release -p imx708_stream
sudo cp imx708_stream/systemd/pgwaam-camera-*.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now pgwaam-camera-front_wide pgwaam-camera-ov5647
journalctl -u pgwaam-camera-front_wide -f
```

## Verifying from another machine

```sh
# ROS 2 (rmw_zenoh)
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/172.31.1.252:7447"]'
ros2 node list                      # → /imx708_stream_front_wide
ros2 topic list | grep front_wide
ros2 topic hz /pgwaam/blackfinfive/front_wide/image_jpg/compressed
```

## Troubleshooting

- **`IMX708_JPEG_RED … not a demosaicable Bayer format`** — the pipeline
  resolved compressed raw and the 16-bit fallback also failed; check the
  `IMX708_RAW_FALLBACK_*` / `IMX708_RAW_FORMAT_RESOLVED` log lines.
- **`HDR_FPS_WARN` from doctor** — the host may be in an HDR profile and
  hiding the high-FPS crop (1536×864@~120 needs HDR off).
- **`acquire camera … is another process holding it?`** — one process per
  camera; stop the systemd unit before running ad-hoc.
- **Building on a 4 GB Pi** — limit parallelism (`cargo build --release
  -j 2`); a full-parallel release build can OOM-hang the host.
