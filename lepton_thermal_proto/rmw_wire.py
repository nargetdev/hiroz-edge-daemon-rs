"""rmw_zenoh wire format: CDR encoding, keyexprs, liveliness, attachments.

Formats cloned byte-for-byte from a live rmw_zenoh/Hiroz publisher on this
graph (imx708_stream, ROS 2 kilted rmw_zenoh) — see SPEC_lepton_thermal_svc.md
Phase 1 step 3.
"""

import os
import struct
import time

IMAGE_TYPE = (
    "sensor_msgs::msg::dds_::Image_/"
    "RIHS01_d31d41a9a4c4bc8eae9be757b0beed306564f7526c88ea6a4588fb9582527d47"
)
STRING_TYPE = (
    "std_msgs::msg::dds_::String_/"
    "RIHS01_df668c740482bbd48fb39d76a70dfd4bd59db1288021743503259e948f6b1a18"
)
QOS_DEFAULT = "::,10:,:,:,,"  # reliable, keep-last 10 (as observed)


class _Cdr:
    """Little-endian XCDR1 writer; alignment origin is after the 4-byte header."""

    def __init__(self):
        self.b = bytearray(b"\x00\x01\x00\x00")

    def _align(self, n):
        pad = (-(len(self.b) - 4)) % n
        self.b += b"\x00" * pad

    def u8(self, v):
        self.b += struct.pack("<B", v)

    def u32(self, v):
        self._align(4)
        self.b += struct.pack("<I", v)

    def i32(self, v):
        self._align(4)
        self.b += struct.pack("<i", v)

    def string(self, s):
        raw = s.encode() + b"\x00"
        self.u32(len(raw))
        self.b += raw

    def bytes_seq(self, data):
        self.u32(len(data))
        self.b += data


def encode_image(stamp_ns, frame_id, height, width, encoding, step, data) -> bytes:
    c = _Cdr()
    c.i32(stamp_ns // 1_000_000_000)
    c.u32(stamp_ns % 1_000_000_000)
    c.string(frame_id)
    c.u32(height)
    c.u32(width)
    c.string(encoding)
    c.u8(0)  # is_bigendian
    c.u32(step)
    c.bytes_seq(bytes(data))
    return bytes(c.b)


def encode_string(s) -> bytes:
    c = _Cdr()
    c.string(s)
    return bytes(c.b)


class Attachments:
    """Per-publisher rmw_zenoh attachment: seq u64 LE | source_ts u64 LE | gid."""

    def __init__(self):
        self.seq = 0
        self.gid = os.urandom(16)

    def next(self) -> bytes:
        self.seq += 1
        return (
            struct.pack("<Q", self.seq)
            + struct.pack("<Q", time.time_ns())
            + bytes([16])
            + self.gid
        )


def topic_keyexpr(domain, topic, type_with_hash):
    return f"{domain}/{topic.lstrip('/')}/{type_with_hash}"


def _mangle(topic):
    return topic.replace("/", "%")


def node_token(domain, zid, node):
    return f"@ros2_lv/{domain}/{zid}/0/0/NN/%/%/{node}"


def pub_token(domain, zid, entity_id, node, topic, type_with_hash, qos=QOS_DEFAULT):
    return (
        f"@ros2_lv/{domain}/{zid}/0/{entity_id}/MP/%/%/{node}/"
        f"{_mangle(topic)}/{type_with_hash}/{qos}"
    )
