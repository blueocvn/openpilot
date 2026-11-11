#!/usr/bin/env python3
"""Replace the CARLA logo on the pedestrian/rider clothes of the CARLA motorcycles with the BlueOC logo
(icon + "BlueOC" wordmark).

Patched cooked UE4.26 textures (all DXT1, 2048x2048, 12 mips):

  Materials/Male/T_Male1Cloth_v1_d      white raglan T-shirt of the male rider (logo on front and back)
  AfroW02_/Textures/T_Bomber_Red/T_BomRed_d   red bomber jacket (logo on the shirt panel)

For every logo box the area is cleared to the surrounding cloth colour and the icon is composited on it (icon above, wordmark below).
Only the DXT blocks that overlap a logo are re-encoded, every other block is kept byte for byte. The
.uasset files are untouched (mip sizes do not change), so only .uexp/.ubulk are written. Output goes to
Content/..., which start_carla.sh bind-mounts over the same path inside the CARLA container.

  python3 build_rider_logo.py [--icon BlueOC-icon.png] [--src DIR] [--preview-dir DIR]

Without --src the originals are copied out of the carla-sim:0.9.12 image; with --src they are read from
DIR/<texture name>.uexp (+ .ubulk when the texture has one).
"""
import argparse
import io
import os
import re
import struct
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
IMAGE = "carla-sim:0.9.12"
PED = "Content/Carla/Static/Pedestrian"

# boxes: logo areas (x0, y0, x1, y1; exclusive) in full-resolution texels. The new logo (icon on top,
# "BlueOC" wordmark below, like the CARLA lockup) fills each box.
# icon_value / icon_saturation / word_color override the defaults: the white shirt needs a deep icon and the
# navy wordmark of the source logo, the black jacket panel keeps the bright icon and a light wordmark.
TEXTURES = [
    dict(name="T_Male1Cloth_v1_d", rel=f"{PED}/Materials/Male",
         boxes=[(1233, 213, 1554, 583), (1233, 1356, 1554, 1726)]),
    dict(name="T_BomRed_d", rel=f"{PED}/AfroW02_/Textures/T_Bomber_Red",
         boxes=[(337, 58, 398, 128)], icon_value=1.0, icon_saturation=1.0, word_color=(235, 240, 255)),
]
WORDMARK_SRC = "BlueOC-Tech-logo.png"   # full BlueOC TECH logo; only the "BlueOC" part is used
WORDMARK_BOX = (199, 85, 501, 149)      # "BlueOC" in WORDMARK_SRC (excludes the icon and "TECH")
GAP_FRAC = 0.065                        # icon-to-wordmark gap, fraction of the box height
BLOCK = 4
CLEAR_PAD = 4
ICON_VALUE = 0.55        # brightness multiplier for the icon (lower = darker, more contrast on white cloth)
ICON_SATURATION = 1.25   # saturation multiplier for the icon


def parse_mips(ux):
    """Mip list of a cooked Texture2D: [{sx, where, pos, size}], pos indexes into the .uexp or .ubulk."""
    m = re.search(rb"PF_[A-Za-z0-9_]+\x00", ux)
    pf = m.group(0)[:-1].decode()
    p = m.end()
    _first, nmips = struct.unpack_from("<ii", ux, p)
    p += 8
    mips = []
    for _ in range(nmips):
        _cooked, flags, _elem, size, offset = struct.unpack_from("<IIiiq", ux, p)
        p += 24
        if flags & 0x100:   # payload lives in the .ubulk
            e = {"where": "ubulk", "pos": offset, "size": size}
        else:               # inline in the .uexp, right after the header
            e = {"where": "uexp", "pos": p, "size": size}
            p += size
        e["sx"], _sy, _sz = struct.unpack_from("<iii", ux, p)
        p += 12
        mips.append(e)
    return pf, mips


def dxt1_decode(data, w, h):
    hdr = (b"DDS " + struct.pack("<7I", 124, 0x1007 | 0x80000, h, w, len(data), 0, 1) + bytes(44)
           + struct.pack("<2I4s5I", 32, 4, b"DXT1", 0, 0, 0, 0, 0) + struct.pack("<5I", 0x1000, 0, 0, 0, 0))
    im = Image.open(io.BytesIO(hdr + data))
    im.load()
    return im.convert("RGB")


def dxt1_encode(img):
    buf = io.BytesIO()
    img.convert("RGB").save(buf, format="DDS", pixel_format="DXT1")
    raw = buf.getvalue()
    assert raw[84:88] == b"DXT1", raw[84:88]
    return raw[128:]


def patch_blocks(data, w, new_img, box):
    """Re-encode the 4x4 blocks of `data` (DXT1, width w) that overlap `box` from `new_img` (same size)."""
    x0, y0, x1, y1 = box
    bx0, by0 = x0 // BLOCK, y0 // BLOCK
    bx1, by1 = -(-x1 // BLOCK), -(-y1 // BLOCK)
    enc = dxt1_encode(new_img.crop((bx0 * BLOCK, by0 * BLOCK, bx1 * BLOCK, by1 * BLOCK)))
    nbx, nby = bx1 - bx0, by1 - by0
    out = bytearray(data)
    bpr = max(1, w // BLOCK)
    for j in range(nby):
        dst = ((by0 + j) * bpr + bx0) * 8
        out[dst:dst + nbx * 8] = enc[j * nbx * 8:(j + 1) * nbx * 8]
    return bytes(out)


def load_icon(path, height, value=ICON_VALUE, saturation=ICON_SATURATION):
    """Icon cropped to its alpha bbox and scaled to `height` texels. `value` (< 1 darkens) and `saturation`
    (> 1 intensifies) deepen the colours: the cloth is lit and the game camera is strongly exposed, so the
    bright cyan of the original icon washes out on a white shirt."""
    ic = Image.open(path).convert("RGBA")
    ic = ic.crop(ic.getchannel("A").getbbox())
    w = round(height * ic.width / ic.height)
    ic = ic.resize((w * 4, height * 4), Image.LANCZOS)      # supersample, then reduce for clean edges
    ic = ic.resize((w, height), Image.LANCZOS)
    r, g, b, alpha = ic.split()
    h, s, v = Image.merge("RGB", (r, g, b)).convert("HSV").split()
    s = s.point(lambda x: min(255, round(x * saturation)))
    v = v.point(lambda x: min(255, round(x * value)))
    rgb = Image.merge("HSV", (h, s, v)).convert("RGB")
    return Image.merge("RGBA", (*rgb.split(), alpha))


def load_wordmark(path, width, color=None):
    """The "BlueOC" wordmark as an RGBA layer `width` texels wide. The source is dark text on white, so the
    coverage is recovered from the darkness; `color` recolours it (default: the navy of the source)."""
    src = Image.open(path).convert("RGB").crop(WORDMARK_BOX)
    g = np.array(src.convert("L"), dtype=np.float32)
    ink = np.array(src, dtype=np.float32)[g < 60].mean(axis=0)               # solid text colour
    cov = np.clip((255 - g) / (255 - g[g < 60].mean()), 0, 1)
    h = round(width * src.height / src.width)
    mask = Image.fromarray((cov * 255).astype(np.uint8)).resize((width * 4, h * 4), Image.LANCZOS)
    mask = mask.resize((width, h), Image.LANCZOS)
    rgb = tuple(int(c) for c in (color or ink))
    return Image.merge("RGBA", (*Image.new("RGB", mask.size, rgb).split(), mask))


def fill_from_edges(arr, x0, y0, x1, y1, strip=8, gap=2):
    """Cloth colour for the box: horizontal and vertical interpolation between the strips just outside it,
    so the cleared area follows the cloth's shading instead of showing up as a flat rectangle."""
    a = arr.astype(np.float32)
    left = a[y0:y1, x0 - gap - strip:x0 - gap].mean(axis=1)       # (h, 3)
    right = a[y0:y1, x1 + gap:x1 + gap + strip].mean(axis=1)
    top = a[y0 - gap - strip:y0 - gap, x0:x1].mean(axis=0)        # (w, 3)
    bottom = a[y1 + gap:y1 + gap + strip, x0:x1].mean(axis=0)
    h, w = y1 - y0, x1 - x0
    tx = np.linspace(0, 1, w, dtype=np.float32)[None, :, None]
    ty = np.linspace(0, 1, h, dtype=np.float32)[:, None, None]
    horiz = left[:, None, :] * (1 - tx) + right[:, None, :] * tx
    vert = top[None, :, :] * (1 - ty) + bottom[None, :, :] * ty
    return np.clip((horiz + vert) / 2, 0, 255).astype(np.uint8)


def fetch_originals(tex, dst):
    cid = subprocess.check_output(["docker", "create", IMAGE]).decode().strip()
    try:
        for ext in ("uexp", "ubulk"):
            r = subprocess.call(["docker", "cp", f"{cid}:/home/carla/CarlaUE4/{tex['rel']}/{tex['name']}.{ext}",
                                 os.path.join(dst, f"{tex['name']}.{ext}")], stderr=subprocess.DEVNULL)
            if r != 0 and ext == "uexp":
                raise SystemExit(f"{tex['name']}.uexp not found in {IMAGE}")
    finally:
        subprocess.call(["docker", "rm", cid], stdout=subprocess.DEVNULL)


def patch_texture(tex, src, out_root, icon_path, preview_dir, value, saturation):
    name = tex["name"]
    ux = open(os.path.join(src, name + ".uexp"), "rb").read()
    ubp = os.path.join(src, name + ".ubulk")
    ub = open(ubp, "rb").read() if os.path.exists(ubp) else b""
    pf, mips = parse_mips(ux)
    assert pf == "PF_DXT1" and mips[0]["sx"] == 2048, (name, pf, mips[0])
    buf0 = ub if mips[0]["where"] == "ubulk" else ux
    base = mips[0]
    full = dxt1_decode(buf0[base["pos"]:base["pos"] + base["size"]], 2048, 2048)

    # 1. new full-resolution image: clear each logo to the cloth colour, composite the icon
    arr = np.array(full)
    value, saturation = tex.get("icon_value", value), tex.get("icon_saturation", saturation)
    boxes, layers = [], []
    for x0, y0, x1, y1 in tex["boxes"]:
        word = load_wordmark(os.path.join(HERE, WORDMARK_SRC), x1 - x0, tex.get("word_color"))
        icon = load_icon(icon_path, (y1 - y0) - word.height - round(GAP_FRAC * (y1 - y0)), value, saturation)
        arr[y0 - CLEAR_PAD:y1 + CLEAR_PAD, x0 - CLEAR_PAD:x1 + CLEAR_PAD] = fill_from_edges(
            arr, x0 - CLEAR_PAD, y0 - CLEAR_PAD, x1 + CLEAR_PAD, y1 + CLEAR_PAD)
        boxes.append((x0 - CLEAR_PAD, y0 - CLEAR_PAD, x1 + CLEAR_PAD, y1 + CLEAR_PAD))
        cx = (x0 + x1) // 2
        layers += [(icon, (cx - icon.width // 2, y0)), (word, (cx - word.width // 2, y1 - word.height))]
    new = Image.fromarray(arr)
    for layer, pos in layers:
        new.paste(layer, pos, layer)

    # 2. patch every mip level that is at least 8 texels wide (4x4 and below are left as they are)
    ux_out, ub_out = bytearray(ux), bytearray(ub)
    for m in mips:
        w = m["sx"]
        if w < 8:
            continue
        s = 2048 // w
        lvl = new if s == 1 else new.resize((w, w), Image.BOX)
        buf = ub_out if m["where"] == "ubulk" else ux_out
        data = bytes(buf[m["pos"]:m["pos"] + m["size"]])
        for b in boxes:
            data = patch_blocks(data, w, lvl, (b[0] // s, b[1] // s, -(-b[2] // s), -(-b[3] // s)))
        buf[m["pos"]:m["pos"] + m["size"]] = data

    assert len(ux_out) == len(ux) and len(ub_out) == len(ub)
    out = os.path.join(out_root, tex["rel"])
    os.makedirs(out, exist_ok=True)
    open(os.path.join(out, name + ".uexp"), "wb").write(ux_out)
    if ub:
        open(os.path.join(out, name + ".ubulk"), "wb").write(ub_out)
    print(f"{name}: wrote {out}")
    if preview_dir:
        os.makedirs(preview_dir, exist_ok=True)
        b2 = ub_out if base["where"] == "ubulk" else ux_out
        dxt1_decode(bytes(b2[base["pos"]:base["pos"] + base["size"]]), 2048, 2048).save(
            os.path.join(preview_dir, name + ".png"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--icon", default=os.path.join(HERE, "BlueOC-icon.png"))
    ap.add_argument("--src", help="directory holding the original <texture>.uexp/.ubulk files")
    ap.add_argument("--out", default=HERE, help="output root (default: this directory)")
    ap.add_argument("--icon-value", type=float, default=ICON_VALUE, help="icon brightness multiplier (default %(default)s)")
    ap.add_argument("--icon-saturation", type=float, default=ICON_SATURATION,
                    help="icon saturation multiplier (default %(default)s)")
    ap.add_argument("--preview-dir", help="also write PNG previews of the patched textures here")
    args = ap.parse_args()

    tmp = tempfile.TemporaryDirectory()
    src = args.src or tmp.name
    for tex in TEXTURES:
        if args.src is None:
            fetch_originals(tex, src)
        patch_texture(tex, src, args.out, args.icon, args.preview_dir, args.icon_value, args.icon_saturation)


if __name__ == "__main__":
    sys.exit(main())
