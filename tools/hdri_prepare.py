"""Prepare a Poly Haven pure-sky HDRI to be the Unity skybox.

Two changes, both because Unity derives the ambient light and the default
reflection cubemap from the skybox:

1. The sun comes out. Unity lights the scene with a directional light for the
   sun; if the sun also stays in the sky texture, its energy lands in the
   ambient spherical harmonics as well and every sun-facing surface gets the
   sun twice (in this sky the sun is 70% of the light on flat ground). Pixels
   near the sun are clamped to SUN_CLAMP, which keeps a bright disc for bloom
   and removes over 99% of the sun's energy.

2. The ground goes in. Pure-sky HDRIs fill the lower hemisphere with a
   blurred, dimmed mirror of the sky, which would light everything from below
   with blue and put sky in the bottom half of every metal reflection. It is
   replaced by what a farm floor under this sky reflects:

     E        = irradiance on flat ground from the upper hemisphere, sun included
     L_ground = albedo * E / pi         (Lambertian ground)

   The albedo is the coverage-weighted linear mean of the four terrain layers,
   measured from Assets/Textures/Ground. The fill blends in over the first few
   degrees below the horizon so there is no seam where fog meets the sky.

    python tools/hdri_prepare.py in.hdr out.hdr [--half] [--albedo r g b]

Run tools/hdri_sun.py on the ORIGINAL file for the sun's direction, colour and
intensity; after this script the sun is no longer in the image.
--half writes a 2x box-downsampled image (4k -> 2k) for secondary skies.
Source HDRIs: CC0, polyhaven.com (see Assets/Textures/Sky/README.md).
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from hdri_sun import read_hdr  # noqa: E402

# linear mean albedo of the terrain layers (measured 2026-09-26):
# grass .048 .086 .018, dry grass .099 .126 .026, dirt .144 .102 .061,
# soil .077 .042 .029, weighted 60/20/10/10 by how much of the farm each covers
DEFAULT_ALBEDO = (0.071, 0.091, 0.025)
BLEND_DEGREES = 4.0
# luminance cap for the sun's neighbourhood: far above any cloud (the 99th
# percentile of this sky is about 2) so the disc still blooms, far below the
# real sun (about 75000) so its energy leaves the ambient
SUN_CLAMP = 30.0
SUN_RADIUS_DEGREES = 4.0

LUMA = np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)


def solid_angles(h, w):
    lat = (0.5 - (np.arange(h) + 0.5) / h) * np.pi
    return lat, np.cos(lat) * (np.pi / h) * (2.0 * np.pi / w)


def irradiance_up(rgb):
    h, w, _ = rgb.shape
    lat, domega = solid_angles(h, w)
    wrow = np.clip(np.sin(lat), 0.0, None) * domega
    return (rgb * wrow[:, None, None]).sum(axis=(0, 1))


def remove_sun(rgb):
    h, w, _ = rgb.shape
    lum = rgb @ LUMA
    y, x = np.unravel_index(np.argmax(lum), lum.shape)
    lat, domega = solid_angles(h, w)
    lon = ((np.arange(w) + 0.5) / w - 0.5) * 2.0 * np.pi
    # angular distance of every pixel from the sun pixel
    sy, cy = math.sin(lat[y]), math.cos(lat[y])
    cosd = np.sin(lat)[:, None] * sy + np.cos(lat)[:, None] * cy * np.cos(lon[None, :] - lon[x])
    near = cosd > math.cos(math.radians(SUN_RADIUS_DEGREES))
    over = near & (lum > SUN_CLAMP)
    scale = np.where(over, SUN_CLAMP / np.maximum(lum, 1e-6), 1.0).astype(np.float32)
    before = (rgb * (domega[:, None] * near)[:, :, None]).sum(axis=(0, 1))
    out = rgb * scale[:, :, None]
    after = (out * (domega[:, None] * near)[:, :, None]).sum(axis=(0, 1))
    return out, before, after, int(over.sum())


def ground_fill(rgb, albedo, e):
    h = rgb.shape[0]
    ground = np.asarray(albedo, dtype=np.float32) * e / np.pi
    lat_deg = (0.5 - (np.arange(h) + 0.5) / h) * 180.0
    # 0 at and above the horizon, 1 from BLEND_DEGREES below it, smoothstep between
    t = np.clip(-lat_deg / BLEND_DEGREES, 0.0, 1.0)
    t = t * t * (3.0 - 2.0 * t)
    out = rgb * (1.0 - t)[:, None, None] + ground[None, None, :] * t[:, None, None]
    return out.astype(np.float32), ground


def to_rgbe(rgb):
    m = rgb.max(axis=2)
    mant, expo = np.frexp(m)
    scale = np.where(m > 1e-32, mant * 256.0 / np.maximum(m, 1e-32), 0.0)
    out = np.zeros(rgb.shape[:2] + (4,), dtype=np.uint8)
    out[:, :, :3] = np.clip(rgb * scale[:, :, None], 0, 255).astype(np.uint8)
    out[:, :, 3] = np.where(m > 1e-32, expo + 128, 0).astype(np.uint8)
    return out


def rle_row(a):
    """New-style Radiance RLE for one channel of one scanline."""
    out = bytearray()
    n = len(a)
    # run boundaries, vectorised; the loop below only walks runs
    change = np.flatnonzero(np.diff(a.astype(np.int16)) != 0) + 1
    starts = np.concatenate(([0], change))
    ends = np.concatenate((change, [n]))
    lit_start = 0
    for s, e in zip(starts.tolist(), ends.tolist()):
        if e - s < 4:
            continue
        i = lit_start
        while i < s:
            k = min(128, s - i)
            out.append(k)
            out += a[i:i + k].tobytes()
            i += k
        i = s
        while i < e:
            k = min(127, e - i)
            out.append(128 + k)
            out.append(int(a[s]))
            i += k
        lit_start = e
    i = lit_start
    while i < n:
        k = min(128, n - i)
        out.append(k)
        out += a[i:i + k].tobytes()
        i += k
    return out


def write_hdr(path, rgb):
    h, w, _ = rgb.shape
    rgbe = to_rgbe(rgb)
    with open(path, "wb") as f:
        f.write(b"#?RADIANCE\n# prepared for Unity by FarmSimulator tools/hdri_prepare.py\n")
        f.write(b"FORMAT=32-bit_rle_rgbe\n\n")
        f.write(("-Y %d +X %d\n" % (h, w)).encode("ascii"))
        for y in range(h):
            f.write(bytes((2, 2, w >> 8, w & 255)))
            for c in range(4):
                f.write(rle_row(rgbe[y, :, c]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--half", action="store_true")
    ap.add_argument("--albedo", type=float, nargs=3, default=DEFAULT_ALBEDO)
    args = ap.parse_args()

    rgb = read_hdr(args.src)
    if args.half:
        h, w, _ = rgb.shape
        rgb = rgb[: h // 2 * 2, : w // 2 * 2].reshape(h // 2, 2, w // 2, 2, 3).mean(axis=(1, 3))

    # the ground is lit by the real sun, so measure before taking it out
    e = irradiance_up(rgb)
    rgb, sun_before, sun_after, clamped = remove_sun(rgb)
    out, ground = ground_fill(rgb, args.albedo, e)
    write_hdr(args.dst, out)

    kept = 100.0 * float(sun_after @ LUMA) / max(float(sun_before @ LUMA), 1e-9)
    print("%s: %dx%d" % (os.path.basename(args.src), out.shape[1], out.shape[0]))
    print("  sun: %d pixels clamped to %.0f, %.2f%% of the light near the sun kept" % (clamped, SUN_CLAMP, kept))
    print("  flat-ground irradiance %.3f %.3f %.3f -> ground radiance %.4f %.4f %.4f" % (
        e[0], e[1], e[2], ground[0], ground[1], ground[2]))
    print("  -> %s (%.1f MB)" % (args.dst, os.path.getsize(args.dst) / 1e6))


if __name__ == "__main__":
    main()
