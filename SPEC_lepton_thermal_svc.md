# Lepton 3.5 (PureThermal) Hiroz thermal stream service manifest

Goal: replace the current three-hop thermal pipeline
(`libuvc config script` → `gst-launch UDP/RTP` → `gscam2` on the receiver)
with a **single edge process on the Pi** that configures the Lepton for
radiometry, grabs Y16 frames, and publishes `sensor_msgs/msg/Image` directly
over Hiroz/Zenoh — no GStreamer, no gscam2, no native ROS 2 install on
either end.

Two deliverables, in order:

1. **Prototype (Python, runs on the Pi today)** — quickest path to frames
   on the wire; may reuse the existing uv project on the Pi.
2. **Production (Rust)** — new workspace member crate in this repo,
   modeled on `imx708_stream`.

## Host / network context

| Thing | Value |
|---|---|
| Edge Pi (camera host) | `id1-cm5@id1-cm5` (Raspberry Pi CM5) |
| Zenoh router | `tcp/172.31.1.252:7447` (client mode, same as other services) |
| Legacy UDP receiver (obsolete after this work) | `172.31.1.253:5600` |
| Camera | FLIR Lepton 3.5 on GroupGets PureThermal board, USB VID:PID `0x1e4e:0x0100`, also enumerates as V4L2 `/dev/video0` |
| Native frame | 160×120, ~9 fps, Y16 / `GRAY16_LE` little-endian |

```sh
export ZENOH_CONFIG_OVERRIDE='mode="client";connect/endpoints=["tcp/172.31.1.252:7447"]'
```

Existing prototyping material **on the Pi** (reference, do not depend on):

- `~/read_lepton3.5_linear_uvc/` — uv project; `set linear/sanity_check.py`
  + `uvctypes.py` (libuvc ctypes bindings), `gst_launch_lepton.sh`,
  `read_lepton_TLinear.py`
- `~/lepton3.5_pureThermal_for_WAAM/` — earlier gain-mode experiments

Reference copies in the main repo (PhotogrammetryWAAM-Blender-UX):
`ros2_ws/edge/SENSORS/PureThermal_Lepton3.5/Lepton_3.5_UVC/read_lepton3.5_linear_uvc/`
and the colorizer at `ros2_ws/src/thermal_colorizer_pkg/`.

## Legacy pipeline being replaced (for understanding, not reimplementation)

1. **Radiometry config** — `sanity_check.py` via libuvc:
   disable AGC, gain mode LOW, TLinear ON, TLinear resolution 0.01 K.
   Observed on the Pi: all four XU writes succeed (readbacks confirm) but
   `uvc_start_streaming` fails with `-2` — the `uvcvideo` kernel driver owns
   the device. **Conclusion: libuvc streaming is a dead end on this host;
   use it (or raw ioctls) only for the CCI/XU control writes and capture via
   V4L2 instead.** The gst step below already proves V4L2 capture works
   while the XU-written radiometry settings stick.
2. **Transport** —
   `gst-launch-1.0 v4l2src device=/dev/video0 ! video/x-raw,format=GRAY16_LE,width=160,height=120,framerate=9/1 ! rtpgstpay config-interval=1 ! udpsink host=172.31.1.253 port=5600 sync=false`
3. **Receive/bridge** — `gscam2` (`udpsrc ! rtpgstdepay ! videoflip method=rotate-180`,
   `image_encoding:=mono16`, `frame_id:=therm_optical`, remapped to
   `/therm/image_raw`). gscam2 exists **only** to turn the RTP stream back
   into a ROS Image. Once the edge publishes Image directly, steps 2–3 and
   the gscam2 checkout are unnecessary.
4. **Visualization** — `thermal_colorizer_node`: mono16 in →
   window/clip → colormap (default inferno, fixed 15–50 °C window,
   optional percentile auto-window) → rgb8 out on `/therm/image_color`.

## Radiometry configuration contract (both phases MUST do this at startup)

Lepton CCI over UVC Extension Units. Selector IDs (from pt1.xml/ctrl_gen.py;
all 4-byte little-endian u32, write then read back to verify):

| Register | Unit | Selector | Value | Meaning |
|---|---|---|---|---|
| `LEP_AGC_ENABLE_STATE` | AGC | 1 | `0` | AGC off — raw radiometric counts |
| `LEP_SYS_GAIN_MODE` | SYS | 19 | `1` | LOW gain (WAAM: wide temp range, weld-pool capable) |
| `LEP_RAD_TLINEAR_ENABLE_STATE` | RAD | 49 | `1` | TLinear on — pixel = temperature |
| `LEP_RAD_TLINEAR_RESOLUTION` | RAD | 50 | `1` | 0.01 K/count (so `pixel/100.0 = Kelvin`) |

Unit IDs (`AGC_UNIT_ID`, `SYS_UNIT_ID`, `RAD_UNIT_ID`) come from
`uvctypes.py` on the Pi. Any readback mismatch ⇒ hard startup failure with
a clear message; never stream non-radiometric data silently.

**Pixel semantics after config:** uint16, `temp_K = value / 100.0`,
`temp_C = value / 100.0 - 273.15`. Sanity: room-temp scene averages
~30 000 counts (≈300 K). This is the invariant the whole WAAM stack
depends on — the raw topic stays lossless.

### Two implementation routes for the XU writes

- **Python prototype:** reuse `uvctypes.py` + `set_extension_unit` /
  `call_extension_unit` exactly as `sanity_check.py` does (proven). Do NOT
  call `uvc_start_streaming`; close/unref the libuvc handle after config so
  V4L2 capture is unencumbered.
- **Rust:** prefer the pure-kernel path — `UVCIOC_CTRL_QUERY` ioctl on
  `/dev/video0` (`uvc_xu_control_query` struct, `UVC_SET_CUR`/`UVC_GET_CUR`)
  — no libuvc dependency. Fall back to a libuvc-sys binding only if the
  ioctl path proves unworkable.

## Capture model

- Open `/dev/video0` via V4L2, request `Y16 ` fourcc (GRAY16_LE),
  160×120 @ 9 fps, mmap streaming.
- Device discovery: don't hardcode `video0` — enumerate `/dev/video*` and
  match on card/driver name containing `PureThermal` (or USB VID:PID
  `1e4e:0100` via sysfs). Param override available.
- Queue depth 1, latest-wins; count drops on status (same policy as
  `imx708_stream`).
- Optional `rotate_180` (default **true**, matching the legacy gscam2
  `videoflip method=rotate-180`) applied before publish to both topics.
- No disk writes.

## Graph identity

Node name: `lepton_thermal_{camera_name}` (default `camera_name=therm`).

Topics (follow the pgwaam scheme):

```text
/pgwaam/{hostname}/{camera_name}/image_raw          sensor_msgs/msg/Image   mono16 (source of truth)
/pgwaam/{hostname}/{camera_name}/image_color        sensor_msgs/msg/Image   rgb8   (operator eyes)
/pgwaam/{hostname}/{camera_name}/status             std_msgs/msg/String     heartbeat
```

Example for this host: `/pgwaam/id1_cm5/therm/image_raw`, `.../image_color`,
`.../status`. If downstream consumers still expect the legacy flat names
(`/therm/image_raw`, `/therm/image_color`), expose `topic_prefix` as a
param so the prototype can match them 1:1 during cutover.

## Message shapes

### `image_raw` — `sensor_msgs/msg/Image`

- `encoding`: `mono16`
- `is_bigendian`: 0
- `width`=160, `height`=120, `step`=320
- `data`: 38 400 bytes, verbatim TLinear counts (after optional rotation)
- `header.stamp`: capture time (V4L2 buffer timestamp when available, else
  host time); `header.frame_id`: `frame_id` param (default `therm_optical`)

### `image_color` — `sensor_msgs/msg/Image`

- `encoding`: `rgb8`, `step`=480
- Colorization = port of `thermal_colorizer_node.py`:
  clip raw counts to window → scale to u8 → colormap → rgb8.
  - Fixed window: `min_temp_c`/`max_temp_c` converted via
    `counts = (C + 273.15) * 100`
  - Auto window: per-frame percentiles `auto_low_pct`/`auto_high_pct`
  - Degenerate-window guard: `hi = lo + 1` if `hi - lo < 1`
  - Colormap default `inferno` (closest to FLIR ironbow)
- `header` copied from the raw frame (same stamp + frame_id).

### `status` — `std_msgs/msg/String`, key=value lines every `status_period_sec`

```text
state=idle|streaming|error
camera_name=... device=/dev/videoN
radiometry_ok=0|1 agc=0 gain_mode=1 tlinear=1 tlinear_res=1
frame_count=... raw_pub_ok=... raw_pub_drop=... color_pub_ok=... color_pub_drop=...
last_frame_unix_ns=... heartbeat_unix_ns=...
image_raw_topic=... image_color_topic=...
```

## Parameters

| Param | Type | Default | Notes |
|---|---|---|---|
| `camera_name` | string | `therm` | topic segment; restart-only |
| `device` | string | *(auto-detect)* | `/dev/videoN` override; restart-only |
| `frame_id` | string | `therm_optical` | |
| `rotate_180` | bool | `true` | matches legacy videoflip |
| `publish_raw` | bool | `true` | |
| `publish_color` | bool | `true` | |
| `colormap` | string | `inferno` | inferno\|magma\|plasma\|viridis\|jet\|turbo\|hot (prototype may support fewer; document) |
| `auto_window` | bool | `false` | |
| `min_temp_c` / `max_temp_c` | double | `15.0` / `50.0` | fixed window; weld range e.g. 200–1500 |
| `auto_low_pct` / `auto_high_pct` | double | `1.0` / `99.0` | percentile AGC |
| `gain_mode` | int | `1` | 0=HIGH 1=LOW 2=AUTO; applied at startup |
| `status_period_sec` | double | `5.0` | |

Runtime-reconfigure (Rust phase): windowing, colormap, publish gates.
Invalid sets soft-reject. Prototype may take everything as CLI flags/env.

## Phase 1 — Python prototype (on `id1-cm5`)

Purpose: prove frames-on-Zenoh end to end this week. Lives beside the
existing uv project on the Pi (`uv` runtime per house rules).

1. XU radiometry config via `uvctypes.py` (readback-verified), then release
   the libuvc handle.
2. V4L2 Y16 capture — simplest proven options, in preference order:
   a. `v4l2py` / direct `ioctl` mmap loop (no GStreamer dep), or
   b. OpenCV `VideoCapture` with `CAP_V4L2` + `CONVERT_RGB=0` +fourcc `Y16 `, or
   c. keep gst `v4l2src ! appsink` if (a)/(b) fight the driver.
3. Publish over Zenoh with **zenoh-python**, encoding
   `sensor_msgs/msg/Image` CDR by hand (little-endian CDR, XCDR1: 4-byte
   encapsulation header `00 01 00 00`, then Header{stamp{sec:i32,
   nanosec:u32}, frame_id:string}, height:u32, width:u32, encoding:string,
   is_bigendian:u8, step:u32, data:sequence<u8>). Key expressions and
   liveliness must match what Hiroz/`rmw_zenoh` expects — crib from this
   repo's `src/main.rs`, which already publishes Hiroz messages; mirror its
   keyexpr/attachment scheme rather than inventing one.
4. Acceptance: `zenoh_subscribe` (or the hiroz CLI here) on
   `.../therm/image_raw` shows decodable mono16 Images at ~9 fps with
   room-temp scenes averaging ≈30 000 counts; `image_color` renders sanely.

## Phase 2 — Rust crate

Workspace member **`lepton_thermal_stream`** beside `imx708_stream`:

```sh
cargo run -p lepton_thermal_stream -- doctor   # find device, XU readbacks, one test frame, scene-average sanity line
cargo run -p lepton_thermal_stream -- stream   # long-running Hiroz node (default)
```

- Capture: `v4l` (a.k.a. `v4l2`) crate, mmap streaming, dedicated OS thread;
  depth-1 channel into async Hiroz publish (same lifecycle pattern as
  `imx708_stream`).
- XU config: `UVCIOC_CTRL_QUERY` ioctls (nix crate), readback-verified,
  hard-fail on mismatch.
- Colorizer: small fixed LUTs (256×3) for the supported colormaps; no
  OpenCV dependency for a 160×120 image.
- Device disconnect: reopen up to 3 times with backoff, then exit non-zero.
- `doctor` prints the ALL-CAPS room-temp verdict line like `sanity_check.py`
  (`AVERAGE TEMPERATURE WITHIN +/-20% OF ROOM TEMPERATURE`).

## Acceptance (overall)

1. One process on the Pi replaces sanity_check + gst-launch + gscam2; the
   receiver needs nothing but a Zenoh/Hiroz subscriber.
2. `image_raw` is bit-faithful TLinear mono16 (0.01 K/count) — verified by
   scene-average ≈ room temp and by a hot object reading plausibly.
3. `image_color` visually matches `thermal_colorizer_node` output for the
   same window/colormap.
4. Radiometry config is readback-verified at startup; failure is loud.
5. Status heartbeat keeps the namespace visible to monitors.
6. Drops counted, never blocking capture.

## Non-goals

- RTP/UDP/GStreamer transport (retired)
- gscam2 (retired for this sensor)
- CompressedImage/JPEG topics (mono16 doesn't JPEG; revisit with PNG if
  bandwidth ever matters — 38 kB × 9 fps ≈ 350 kB/s is fine)
- FFC control, spotmeter, ROI stats, Y8/RGB UVC formats
- Multi-camera in one process
- Keeping the legacy flat `/therm/*` names beyond the cutover window
