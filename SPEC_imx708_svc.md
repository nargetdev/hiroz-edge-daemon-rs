# IMX708 Hiroz stream service manifest

This goal runs from the Raspberry Pi itself and uses local Sony IMX708
module(s) via [libcamera-rs](https://github.com/lit-robotics/libcamera-rs)
plus Hiroz over the configured Zenoh client session:

```sh
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/172.31.1.252:7447"]'
```

Host validation (this Pi):

```text
0 : imx708_wide … Modes: 'SBGGR10_CSI2P' : 1536x864 / 2304x1296 / 4608x2592
1 : imx708_wide … same modes
```

When the sensor is in HDR mode, libcamera may report **30 fps for every
mode**. High-FPS crops (e.g. ~120 at 1536×864) require host/pipeline HDR
off; the node does not invent FPS the pipeline does not expose. `doctor`
warns when all listed modes share the same low FPS.

## Binary / crate

Workspace member package: **`imx708_stream`**

```sh
cargo run -p imx708_stream -- doctor
cargo run -p imx708_stream -- stream --camera-name front_wide --camera-index 0
```

One process owns **one** camera. Two IMX708s ⇒ two process instances.

## Graph identity

Node name: `imx708_stream_{camera_name}`

Topics (v1 stream focus):

```text
/pgwaam/{hostname}/{camera_name}/image_raw
/pgwaam/{hostname}/{camera_name}/image_jpg/compressed
/pgwaam/{hostname}/{camera_name}/status
```

Reserved still service (documented only; **not implemented in v1**):

```text
/pgwaam/{hostname}/{camera_name}/capture
```

Still frames (future) reuse the same image topics; status may show
`state=capturing_still`.

`camera_name` is required (topic segment). `camera_index` selects the
libcamera enumeration index. Both are **restart-only**.

Example for this host:

```text
/pgwaam/id2_rpi4/front_wide/image_raw
/pgwaam/id2_rpi4/front_wide/image_jpg/compressed
/pgwaam/id2_rpi4/front_wide/status
```

## Capture model

1. Always open and run a **raw** sensor stream at the selected `sensor_mode`.
2. Prefer pixel format **`SBGGR10_CSI2P`**. After `validate()`, use the
   **actual** format the pipeline settles on (PiSP may adjust to e.g.
   `RGGB_PISP_COMP1` or a 16-bit Bayer). Status and `Image.encoding` report
   the resolved format string.
3. `publish_raw` (default **false**) gates publishing packed/raw bytes as
   `sensor_msgs/Image` (pass-through; `step` from buffer metadata).
4. `publish_jpeg` (default **true**) gates JPEG on
   `sensor_msgs/CompressedImage`.
5. Overrun policy: queue depth **1**, latest-wins; count drops on status.
6. **No disk writes** in `stream` mode.

### JPEG pipeline (`jpeg_pipeline`)

| Value | Behavior |
|---|---|
| `cpu_from_raw` (**default**) | Prefer single raw stream; software demosaic/scale/JPEG on the Pi CPU. If the resolved raw format is not demosaicable (e.g. PiSP compressed raw), fall back to `SRGGB16` raw (still single-stream) for CPU demosaic, or hard-fail JPEG with a clear status reason if neither works. |
| `isp_processed` | Dual stream: raw + ViewFinder `RGB888` at JPEG size; JPEG-encode ISP RGB. Hard-fail startup/reconfigure if dual-stream validate fails (no silent fallback to CPU). |

JPEG `CompressedImage.format` matches the DSLR service:
`bgr8; jpeg compressed bgr8` (BGR order before encode when source is RGB).

## Parameters (Hiroz node params)

| Param | Type | Default | Notes |
|---|---|---|---|
| `camera_name` | string | *(required at launch)* | restart-only |
| `camera_index` | int | `0` | restart-only |
| `sensor_mode` | int | `0` | index into live mode table from open camera |
| `hdr_enable` | bool | `false` | maps to libcamera `HdrMode` Off/MultiExposure; reconfigure |
| `publish_raw` | bool | `false` | |
| `publish_jpeg` | bool | `true` | |
| `jpeg_pipeline` | string | `cpu_from_raw` | or `isp_processed` (also CLI) |
| `jpeg_scale` | int | `2` | `0=1/1, 1=1/2, 2=1/4, 3=1/8` of active mode |
| `jpeg_fps` | double | `0.0` | `0` = every frame; else cap |
| `jpeg_quality` | int | `80` | 0–100 |
| `frame_id` | string | `{camera_name}/optical_frame` | |
| `status_period_sec` | double | `5.0` | |
| `ae_enable` | bool | `true` | |
| `exposure_time_us` | int | `10000` | used when AE off |
| `analogue_gain` | double | `1.0` | used when AE off |
| `awb_enable` | bool | `true` | |
| `af_mode` | int | `0` | `0=manual, 1=auto, 2=continuous` |
| `lens_position` | double | `1.0` | dioptres; when `af_mode=manual` |

Runtime-reconfigure: mode, hdr, publish_*, jpeg_*, AE/AF set (tear down stream,
rebuild). Invalid sets are rejected; previous stream kept.

## Image messages

### `image_raw` — `sensor_msgs/msg/Image`

- `encoding`: resolved libcamera format name (prefer `SBGGR10_CSI2P`)
- `is_bigendian`: 0
- `width` / `height`: active mode
- `step`: from stream/buffer (not guessed)
- `data`: frame bytes pass-through
- `header.stamp`: libcamera frame metadata timestamp when available, else host time
- `header.frame_id`: `frame_id` param

### `image_jpg/compressed` — `sensor_msgs/msg/CompressedImage`

- `format`: `bgr8; jpeg compressed bgr8`
- `data`: JPEG bytes
- stamp / frame_id as above

## Status heartbeat

`std_msgs/msg/String` every `status_period_sec`, key=value line:

```text
state=idle|streaming|reconfiguring|error
camera_name=...
camera_index=...
sensor_mode=...
sensor_mode_label=...
hdr_enable=0|1
publish_raw=0|1
publish_jpeg=0|1
jpeg_pipeline=...
jpeg_scale=...
jpeg_fps=...
jpeg_quality=...
frame_count=...
raw_pub_ok=...
raw_pub_drop=...
jpeg_pub_ok=...
jpeg_pub_drop=...
last_frame_unix_ns=...
image_raw_topic=...
image_jpg_topic=...
heartbeat_unix_ns=...
```

## Lifecycle

- libcamera request loop on a **dedicated OS thread**
- depth-1 channels into async Hiroz publish
- camera disconnect: reopen up to **3** times with backoff, then exit non-zero
- publish failures: soft (drop counters)
- bad param set: soft reject

## CLI

- `doctor` — list cameras + mode table; warn on HDR-skewed FPS; attempt one
  configure/alloc/single request when camera free; optional `--zenoh`
- `stream` — long-running Hiroz node (default command)

## V1 acceptance

1. Workspace builds `imx708_stream` beside the DSLR package
2. `doctor` lists both IMX708s and mode table
3. `stream` publishes status + default JPEG; raw only when `publish_raw:=true`
4. Params reconfigure or soft-reject
5. `jpeg_pipeline=cpu_from_raw` default works; `isp_processed` works or hard-fails clearly
6. Drops counted on status
7. Still `/capture` documented only

## Non-goals (v1)

- Still capture service implementation
- On-disk circular buffer / recording
- Multi-camera in one process
- Full libcamera control surface
- Hardware JPEG encoder binding
- Non-Lyrical ROS
- Shared `pgwaam_common` crate
- Inventing FPS the host pipeline does not expose
