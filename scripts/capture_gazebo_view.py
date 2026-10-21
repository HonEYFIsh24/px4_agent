#!/usr/bin/env python3
"""Read one native Gazebo window's compositor pixmap, without reading the desktop.

No focus, window movement, scene control, redirection, or flight command occurs.
Requires an already-composited X11 Gazebo window and the existing X11 libraries.
"""
import argparse
import ctypes
import ctypes.util
import datetime
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import zlib


class XImage(ctypes.Structure):
    _fields_ = [("width", ctypes.c_int), ("height", ctypes.c_int), ("xoffset", ctypes.c_int),
                ("format", ctypes.c_int), ("data", ctypes.c_void_p), ("byte_order", ctypes.c_int),
                ("bitmap_unit", ctypes.c_int), ("bitmap_bit_order", ctypes.c_int),
                ("bitmap_pad", ctypes.c_int), ("depth", ctypes.c_int), ("bytes_per_line", ctypes.c_int),
                ("bits_per_pixel", ctypes.c_int), ("red_mask", ctypes.c_ulong),
                ("green_mask", ctypes.c_ulong), ("blue_mask", ctypes.c_ulong),
                ("obdata", ctypes.c_void_p), ("functions", ctypes.c_void_p * 6)]


class XErrorEvent(ctypes.Structure):
    _fields_ = [("type", ctypes.c_int), ("display", ctypes.c_void_p),
                ("resourceid", ctypes.c_ulong), ("serial", ctypes.c_ulong),
                ("error_code", ctypes.c_ubyte), ("request_code", ctypes.c_ubyte),
                ("minor_code", ctypes.c_ubyte)]


class Visual(ctypes.Structure):
    _fields_ = [("ext_data", ctypes.c_void_p), ("visualid", ctypes.c_ulong), ("class_type", ctypes.c_int),
                ("red_mask", ctypes.c_ulong), ("green_mask", ctypes.c_ulong), ("blue_mask", ctypes.c_ulong),
                ("bits_per_rgb", ctypes.c_int), ("map_entries", ctypes.c_int)]


class XWindowAttributes(ctypes.Structure):
    _fields_ = [("x", ctypes.c_int), ("y", ctypes.c_int), ("width", ctypes.c_int), ("height", ctypes.c_int),
                ("border_width", ctypes.c_int), ("depth", ctypes.c_int), ("visual", ctypes.POINTER(Visual)),
                ("root", ctypes.c_ulong), ("class_type", ctypes.c_int), ("bit_gravity", ctypes.c_int),
                ("win_gravity", ctypes.c_int), ("backing_store", ctypes.c_int), ("backing_planes", ctypes.c_ulong),
                ("backing_pixel", ctypes.c_ulong), ("save_under", ctypes.c_int), ("colormap", ctypes.c_ulong),
                ("map_installed", ctypes.c_int), ("map_state", ctypes.c_int), ("all_event_masks", ctypes.c_long),
                ("your_event_mask", ctypes.c_long), ("do_not_propagate_mask", ctypes.c_long),
                ("override_redirect", ctypes.c_int), ("screen", ctypes.c_void_p)]


def verified_window(requested=None):
    if requested is None:
        tree = subprocess.check_output(["xwininfo", "-root", "-tree"], text=True)
        candidates = []
        pattern = r'^\s*(0x[0-9a-f]+) "Gazebo": \("gazebo" "gazebo"\)\s+(\d+)x(\d+)'
        for line in tree.splitlines():
            match = re.search(pattern, line)
            if match and int(match[2]) >= 600 and int(match[3]) >= 400:
                candidates.append((int(match[2]) * int(match[3]), int(match[1], 16)))
        if not candidates:
            raise RuntimeError("no native Gazebo main window is available")
        window = max(candidates)[1]
    else:
        window = int(requested, 0)
    properties = subprocess.check_output(["xprop", "-id", hex(window), "_NET_WM_PID", "WM_CLASS"], text=True)
    if 'WM_CLASS(STRING) = "gazebo", "gazebo"' not in properties:
        raise RuntimeError("refusing a window that is not the native Gazebo application")
    match = re.search(r'_NET_WM_PID\(CARDINAL\) = (\d+)', properties)
    if match is None:
        raise RuntimeError("cannot verify the Gazebo window process")
    pid = int(match[1])
    command = Path("/proc/{}/cmdline".format(pid)).read_bytes().split(b"\0")[0]
    if os.path.basename(os.fsdecode(command)) != "gzclient":
        raise RuntimeError("refusing a window whose owner is not gzclient")
    return window, pid


def bind(library, name, arguments, result):
    function = getattr(library, name)
    function.argtypes, function.restype = arguments, result
    return function


def compositor_frame(client):
    information = subprocess.check_output(["xwininfo", "-id", hex(client), "-tree"], text=True)
    root_match = re.search(r'Root window id: (0x[0-9a-f]+)', information)
    parent_match = re.search(r'Parent window id: (0x[0-9a-f]+)', information)
    if root_match is None or parent_match is None:
        raise RuntimeError("cannot verify the Gazebo window frame ancestry")
    root, parent = int(root_match[1], 16), int(parent_match[1], 16)
    if parent == root:
        return client
    frame_info = subprocess.check_output(["xwininfo", "-id", hex(parent), "-tree"], text=True)
    frame_parent = re.search(r'Parent window id: (0x[0-9a-f]+)', frame_info)
    # Mutter redirects its decoration frame rather than the reparented Qt
    # child. Accept only a direct-root frame with exactly this one app child.
    if (frame_parent is None or int(frame_parent[1], 16) != root
            or re.search(r'^\s+1 child:\s*\n\s+' + re.escape(hex(client)) + r' "Gazebo":', frame_info, re.M) is None):
        raise RuntimeError("refusing an unverified compositor ancestor; desktop fallback is forbidden")
    return parent


def read_pixmap(window):
    client_window, window = window, compositor_frame(window)
    x11 = ctypes.CDLL(ctypes.util.find_library("X11") or "libX11.so.6")
    composite = ctypes.CDLL(ctypes.util.find_library("Xcomposite") or "libXcomposite.so.1")
    display_type, drawable_type = ctypes.c_void_p, ctypes.c_ulong
    open_display = bind(x11, "XOpenDisplay", [ctypes.c_char_p], display_type)
    close_display = bind(x11, "XCloseDisplay", [display_type], ctypes.c_int)
    sync = bind(x11, "XSync", [display_type, ctypes.c_int], ctypes.c_int)
    query = bind(composite, "XCompositeQueryExtension", [display_type, ctypes.POINTER(ctypes.c_int),
                 ctypes.POINTER(ctypes.c_int)], ctypes.c_int)
    name_pixmap = bind(composite, "XCompositeNameWindowPixmap", [display_type, drawable_type], drawable_type)
    attributes = bind(x11, "XGetWindowAttributes", [display_type, drawable_type,
                      ctypes.POINTER(XWindowAttributes)], ctypes.c_int)
    geometry = bind(x11, "XGetGeometry", [display_type, drawable_type, ctypes.POINTER(drawable_type),
                    ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
                    ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint),
                    ctypes.POINTER(ctypes.c_uint), ctypes.POINTER(ctypes.c_uint)], ctypes.c_int)
    get_image = bind(x11, "XGetImage", [display_type, drawable_type, ctypes.c_int, ctypes.c_int,
                     ctypes.c_uint, ctypes.c_uint, ctypes.c_ulong, ctypes.c_int], ctypes.POINTER(XImage))
    destroy_image = bind(x11, "XDestroyImage", [ctypes.POINTER(XImage)], ctypes.c_int)
    free_pixmap = bind(x11, "XFreePixmap", [display_type, drawable_type], ctypes.c_int)
    callback_type = ctypes.CFUNCTYPE(ctypes.c_int, display_type, ctypes.POINTER(XErrorEvent))
    error_codes = []

    @callback_type
    def on_error(display, error):
        error_codes.append(int(error.contents.error_code))
        return 0

    set_error = bind(x11, "XSetErrorHandler", [callback_type], ctypes.c_void_p)
    set_error(on_error)
    display, pixmap, captured = open_display(None), None, None
    if not display:
        raise RuntimeError("cannot open the configured X11 display")
    try:
        event, error = ctypes.c_int(), ctypes.c_int()
        if not query(display, ctypes.byref(event), ctypes.byref(error)):
            raise RuntimeError("XComposite is unavailable; desktop fallback is forbidden")
        window_attributes = XWindowAttributes()
        if not attributes(display, window, ctypes.byref(window_attributes)) or not window_attributes.visual:
            raise RuntimeError("cannot read the Gazebo window visual")
        pixmap = name_pixmap(display, window)
        sync(display, False)
        if error_codes or not pixmap:
            raise RuntimeError("Gazebo window has no compositor pixmap: {}".format(error_codes))
        root = drawable_type()
        x, y, width, height, border, depth = ctypes.c_int(), ctypes.c_int(), ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint(), ctypes.c_uint()
        if not geometry(display, pixmap, ctypes.byref(root), ctypes.byref(x), ctypes.byref(y),
                        ctypes.byref(width), ctypes.byref(height), ctypes.byref(border), ctypes.byref(depth)):
            raise RuntimeError("cannot read the Gazebo compositor pixmap dimensions")
        if not 100 <= width.value <= 5000 or not 100 <= height.value <= 5000:
            raise RuntimeError("invalid Gazebo pixmap dimensions")
        captured = get_image(display, pixmap, 0, 0, width.value, height.value, ctypes.c_ulong(-1).value, 2)
        sync(display, False)
        if error_codes or not captured:
            raise RuntimeError("cannot read the Gazebo window pixmap: {}".format(error_codes))
        image = captured.contents
        if image.bits_per_pixel not in (24, 32):
            raise RuntimeError("unsupported Gazebo window pixel format")
        bytes_per_pixel = image.bits_per_pixel // 8
        offsets = []
        visual = window_attributes.visual.contents
        # XGetImage's channel masks are not specified for a pixmap drawable.
        # Use the verified source window's actual Visual, not assumed RGB masks.
        for mask in (visual.red_mask, visual.green_mask, visual.blue_mask):
            for offset in range(bytes_per_pixel):
                bit_offset = 8 * (offset if image.byte_order == 0 else bytes_per_pixel - 1 - offset)
                if mask == 255 << bit_offset:
                    offsets.append(offset)
                    break
            else:
                raise RuntimeError("unsupported Gazebo color channel mask")
        raw = ctypes.string_at(image.data, image.bytes_per_line * image.height)
        scanlines = []
        for row in range(image.height):
            pixels = raw[row * image.bytes_per_line:row * image.bytes_per_line + image.width * bytes_per_pixel]
            rgb = bytearray(image.width * 3)
            for channel, offset in enumerate(offsets):
                rgb[channel::3] = pixels[offset::bytes_per_pixel]
            scanlines.append(b"\0" + rgb)
        return width.value, height.value, b"".join(scanlines), {"depth": image.depth, "bits_per_pixel": image.bits_per_pixel,
                "captured_frame_id": hex(window), "verified_client_id": hex(client_window)}
    finally:
        if captured:
            destroy_image(captured)
        if pixmap and not error_codes:
            free_pixmap(display, pixmap)
        close_display(display)


def chunk(kind, data):
    return struct.pack("!I", len(data)) + kind + data + struct.pack("!I", zlib.crc32(kind + data) & 0xffffffff)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window-id", help="explicit native Gazebo main-window XID")
    parser.add_argument("--output", default="logs/avoidance/gazebo_overview.png")
    args = parser.parse_args()
    window, pid = verified_window(args.window_id)
    width, height, data, pixel_format = read_pixmap(window)
    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack("!IIBBBBB", width, height, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(data, 6)) + chunk(b"IEND", b""))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(png)
    metadata = {"captured_at": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "capture_method": "XCompositeNameWindowPixmap + XGetImage (single Gazebo window)",
                "desktop_read": False, "window_id": hex(window), "gzclient_pid": pid,
                "width": width, "height": height, "sha256": hashlib.sha256(png).hexdigest(),
                "pixel_format": pixel_format}
    output.with_suffix(".json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2))
    print(json.dumps({"output": str(output), **metadata}, ensure_ascii=False))


if __name__ == "__main__":
    main()
