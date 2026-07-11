"""Lepton CCI radiometry config via libuvc XU writes (readback-verified).

Config-only: never starts libuvc streaming (uvcvideo owns the device);
the handle is closed after config so V4L2 capture is unencumbered.
"""

import struct
import sys
from ctypes import POINTER, byref, create_string_buffer

from uvctypes import (
    AGC_UNIT_ID,
    RAD_UNIT_ID,
    SYS_UNIT_ID,
    PT_USB_VID,
    PT_USB_PID,
    call_extension_unit,
    set_extension_unit,
    libuvc,
    uvc_context,
    uvc_device,
    uvc_device_handle,
)

LEP_AGC_ENABLE_STATE = 1
LEP_SYS_GAIN_MODE = 19
LEP_RAD_TLINEAR_ENABLE_STATE = 49
LEP_RAD_TLINEAR_RESOLUTION = 50


def _set_u32(devh, unit, sel, value):
    buf = create_string_buffer(4)
    struct.pack_into("<I", buf, 0, value)
    return set_extension_unit(devh, unit, sel, buf, 4)


def _get_u32(devh, unit, sel):
    buf = create_string_buffer(4)
    if call_extension_unit(devh, unit, sel, buf, 4) < 0:
        return None
    return struct.unpack_from("<I", buf, 0)[0]


def configure_radiometry(gain_mode: int = 1) -> dict:
    """AGC off, gain mode, TLinear on @ 0.01K. Returns readbacks.

    Raises RuntimeError on any readback mismatch (spec: never stream
    non-radiometric data silently).
    """
    ctx = POINTER(uvc_context)()
    dev = POINTER(uvc_device)()
    devh = POINTER(uvc_device_handle)()

    if libuvc.uvc_init(byref(ctx), 0) < 0:
        raise RuntimeError("uvc_init failed")
    try:
        if libuvc.uvc_find_device(ctx, byref(dev), PT_USB_VID, PT_USB_PID, 0) < 0:
            raise RuntimeError("PureThermal (1e4e:0100) not found on USB")
        try:
            if libuvc.uvc_open(dev, byref(devh)) < 0:
                raise RuntimeError("uvc_open failed (udev permissions?)")
            want = {
                ("agc", AGC_UNIT_ID, LEP_AGC_ENABLE_STATE): 0,
                ("gain_mode", SYS_UNIT_ID, LEP_SYS_GAIN_MODE): gain_mode,
                ("tlinear", RAD_UNIT_ID, LEP_RAD_TLINEAR_ENABLE_STATE): 1,
                ("tlinear_res", RAD_UNIT_ID, LEP_RAD_TLINEAR_RESOLUTION): 1,
            }
            got = {}
            for (name, unit, sel), value in want.items():
                _set_u32(devh, unit, sel, value)
                rb = _get_u32(devh, unit, sel)
                got[name] = rb
                if rb != value:
                    raise RuntimeError(
                        f"radiometry readback mismatch: {name} wrote {value} read {rb}"
                    )
            libuvc.uvc_close(devh)
            return got
        finally:
            libuvc.uvc_unref_device(dev)
    finally:
        libuvc.uvc_exit(ctx)


if __name__ == "__main__":
    print("RADIOMETRY", configure_radiometry(int(sys.argv[1]) if len(sys.argv) > 1 else 1))
