#!/usr/bin/env python3
"""Phase 1 prototype: Lepton 3.5 -> radiometric mono16 Image over Zenoh.

Replaces sanity_check + gst-launch + gscam2 with one process.
See SPEC_lepton_thermal_svc.md. Usage:

    uv run lepton_proto.py doctor
    uv run lepton_proto.py stream [--camera-name therm] [--no-rotate-180] ...
"""

import argparse
import socket
import sys
import time

import numpy as np
import zenoh
from linuxpy.video.device import Device, VideoCapture

import colormaps
import rmw_wire
from radiometry import configure_radiometry

NATIVE_W, NATIVE_H = 160, 120
DEFAULT_ENDPOINT = "tcp/172.31.1.252:7447"
FFC_STALL_TOLERANCE_S = 5.0  # auto-FFC freezes frames ~1s; not a device death


def find_device() -> str:
    """Pick the PureThermal node that actually offers Y16 capture."""
    from linuxpy.video.device import iter_video_capture_devices

    for dev in iter_video_capture_devices():
        try:
            with dev:
                if "PureThermal" not in dev.info.card:
                    continue
                for fmt in dev.info.formats:
                    if getattr(fmt.pixel_format, "name", str(fmt.pixel_format)) == "Y16":
                        return dev.filename.as_posix() if hasattr(dev.filename, "as_posix") else str(dev.filename)
        except OSError:
            continue
    raise RuntimeError("no PureThermal device with Y16 capture found")


def colorize(raw: np.ndarray, lut: np.ndarray, lo: float, hi: float) -> np.ndarray:
    if hi - lo < 1:
        hi = lo + 1
    idx = np.clip((raw.astype(np.float32) - lo) * (255.0 / (hi - lo)), 0, 255).astype(np.uint8)
    return lut[idx]


def sanitize(seg: str) -> str:
    return "".join(c if c.isalnum() or c == "_" else "_" for c in seg.strip()).strip("_")


class Publisher:
    def __init__(self, session, domain, topic, type_with_hash):
        self.key = rmw_wire.topic_keyexpr(domain, topic, type_with_hash)
        self.pub = session.declare_publisher(
            self.key,
            reliability=zenoh.Reliability.RELIABLE,
            congestion_control=zenoh.CongestionControl.DROP,
        )
        self.att = rmw_wire.Attachments()
        self.ok = 0
        self.drop = 0

    def put(self, payload: bytes):
        try:
            self.pub.put(zenoh.ZBytes(payload), attachment=zenoh.ZBytes(self.att.next()))
            self.ok += 1
        except Exception as e:
            self.drop += 1
            print(f"PUB_DROP key={self.key} err={e}", file=sys.stderr)


def open_session(endpoint):
    conf = zenoh.Config()
    conf.insert_json5("mode", '"client"')
    conf.insert_json5("connect/endpoints", f'["{endpoint}"]')
    return zenoh.open(conf)


def capture_frames(device_path):
    """Yield (unix_ns, np.uint16 HxW) frames; tolerate FFC stalls.

    Opening right after the libuvc config handle closes can transiently
    EACCES while uvcvideo re-binds; retry briefly.
    """
    for attempt in range(5):
        try:
            dev = Device(device_path)
            dev.open()
            break
        except OSError:
            if attempt == 4:
                raise
            time.sleep(1.0)
    with dev:
        cap = VideoCapture(dev)
        cap.set_format(NATIVE_W, NATIVE_H, "Y16 ")
        with cap:
            for frame in cap:
                if len(bytes(frame)) < NATIVE_W * NATIVE_H * 2:
                    continue
                arr = np.frombuffer(bytes(frame), dtype=np.uint16)[: NATIVE_W * NATIVE_H]
                yield time.time_ns(), arr.reshape(NATIVE_H, NATIVE_W)


def cmd_doctor(args):
    dev = find_device()
    print(f"DEVICE_GREEN {dev}")
    rb = configure_radiometry(args.gain_mode)
    print(f"RADIOMETRY_GREEN {rb}")
    avgs = []
    gen = capture_frames(dev)
    for _ in range(10):
        _, arr = next(gen)
        avgs.append(float(arr.mean()))
    k = sum(avgs) / len(avgs) / 100.0
    print(f"SCENE_AVG {k:.2f} K ({k - 273.15:.2f} C) over {len(avgs)} frames")
    if 240 <= k <= 360:
        print("AVERAGE TEMPERATURE WITHIN +/-20% OF ROOM TEMPERATURE")
    else:
        print("AVERAGE TEMPERATURE OUTSIDE +/-20% OF ROOM TEMPERATURE")
    print("DOCTOR_DONE")


def cmd_stream(args):
    name = sanitize(args.camera_name)
    host = sanitize(socket.gethostname())
    prefix = args.topic_prefix.rstrip("/") if args.topic_prefix else f"/pgwaam/{host}/{name}"
    t_raw, t_color, t_status = f"{prefix}/image_raw", f"{prefix}/image_color", f"{prefix}/status"
    node = f"lepton_thermal_{name}"

    device = args.device or find_device()
    rb = configure_radiometry(args.gain_mode)
    print(f"RADIOMETRY_GREEN {rb}")

    session = open_session(args.endpoint)
    zid = str(session.info.zid())
    domain = 0
    pubs = {
        "raw": Publisher(session, domain, t_raw, rmw_wire.IMAGE_TYPE),
        "color": Publisher(session, domain, t_color, rmw_wire.IMAGE_TYPE),
        "status": Publisher(session, domain, t_status, rmw_wire.STRING_TYPE),
    }
    # Liveliness tokens make the node/publishers visible to ros2 graph discovery.
    tokens = [session.liveliness().declare_token(rmw_wire.node_token(domain, zid, node))]
    for i, (topic, th) in enumerate(
        [(t_raw, rmw_wire.IMAGE_TYPE), (t_color, rmw_wire.IMAGE_TYPE), (t_status, rmw_wire.STRING_TYPE)],
        start=1,
    ):
        tokens.append(
            session.liveliness().declare_token(
                rmw_wire.pub_token(domain, zid, i, node, topic, th)
            )
        )

    lut = colormaps.lut(args.colormap)
    lo_fixed = (args.min_temp_c + 273.15) * 100.0
    hi_fixed = (args.max_temp_c + 273.15) * 100.0

    print(f"LEPTON_STREAM_READY node={node} device={device}")
    for t in (t_raw, t_color, t_status):
        print(f"LEPTON_TOPIC {t}")

    frame_count = 0
    last_status = 0.0
    last_frame_wall = time.monotonic()

    def status_line(state):
        return (
            f"state={state} camera_name={name} device={device} radiometry_ok=1 "
            f"agc={rb['agc']} gain_mode={rb['gain_mode']} tlinear={rb['tlinear']} tlinear_res={rb['tlinear_res']} "
            f"frame_count={frame_count} raw_pub_ok={pubs['raw'].ok} raw_pub_drop={pubs['raw'].drop} "
            f"color_pub_ok={pubs['color'].ok} color_pub_drop={pubs['color'].drop} "
            f"last_frame_unix_ns={time.time_ns()} heartbeat_unix_ns={time.time_ns()} "
            f"image_raw_topic={t_raw} image_color_topic={t_color}"
        )

    try:
        for stamp_ns, arr in capture_frames(device):
            last_frame_wall = time.monotonic()
            frame_count += 1
            if args.rotate_180:
                arr = arr[::-1, ::-1]

            if args.publish_raw:
                pubs["raw"].put(
                    rmw_wire.encode_image(
                        stamp_ns, args.frame_id, NATIVE_H, NATIVE_W, "mono16", NATIVE_W * 2,
                        np.ascontiguousarray(arr).tobytes(),
                    )
                )
            if args.publish_color:
                if args.auto_window:
                    lo = float(np.percentile(arr, args.auto_low_pct))
                    hi = float(np.percentile(arr, args.auto_high_pct))
                else:
                    lo, hi = lo_fixed, hi_fixed
                rgb = colorize(arr, lut, lo, hi)
                pubs["color"].put(
                    rmw_wire.encode_image(
                        stamp_ns, args.frame_id, NATIVE_H, NATIVE_W, "rgb8", NATIVE_W * 3,
                        rgb.tobytes(),
                    )
                )

            now = time.monotonic()
            if now - last_status >= args.status_period_sec:
                last_status = now
                pubs["status"].put(rmw_wire.encode_string(status_line("streaming")))
                if frame_count % 1 == 0:
                    k = float(arr.mean()) / 100.0
                    print(
                        f"LEPTON_STATUS frames={frame_count} scene_avg={k:.1f}K "
                        f"raw_ok={pubs['raw'].ok} color_ok={pubs['color'].ok}"
                    )
    finally:
        pubs["status"].put(rmw_wire.encode_string(status_line("error")))
        for t in tokens:
            t.undeclare()
        session.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest="cmd", required=True)
    d = sub.add_parser("doctor")
    d.add_argument("--gain-mode", type=int, default=1)
    s = sub.add_parser("stream")
    s.add_argument("--camera-name", default="therm")
    s.add_argument("--device", default=None)
    s.add_argument("--endpoint", default=DEFAULT_ENDPOINT)
    s.add_argument("--topic-prefix", default=None, help="override e.g. /therm for legacy names")
    s.add_argument("--frame-id", default="therm_optical")
    s.add_argument("--gain-mode", type=int, default=1)
    s.add_argument("--rotate-180", action=argparse.BooleanOptionalAction, default=True)
    s.add_argument("--publish-raw", action=argparse.BooleanOptionalAction, default=True)
    s.add_argument("--publish-color", action=argparse.BooleanOptionalAction, default=True)
    s.add_argument("--colormap", default="inferno")
    s.add_argument("--auto-window", action=argparse.BooleanOptionalAction, default=False)
    s.add_argument("--min-temp-c", type=float, default=15.0)
    s.add_argument("--max-temp-c", type=float, default=50.0)
    s.add_argument("--auto-low-pct", type=float, default=1.0)
    s.add_argument("--auto-high-pct", type=float, default=99.0)
    s.add_argument("--status-period-sec", type=float, default=5.0)
    args = p.parse_args()
    if args.cmd == "doctor":
        cmd_doctor(args)
    else:
        cmd_stream(args)


if __name__ == "__main__":
    main()
