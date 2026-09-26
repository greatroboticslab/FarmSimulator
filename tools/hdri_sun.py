"""Find the sun in an equirectangular Radiance .hdr and describe the sky.

Used to line the scene up with an HDRI sky: the directional light's direction,
colour and intensity, the fog colour at the horizon, and how bright the sky is
overall. Prints:

  - the sun's elevation and image azimuth
  - the sun's colour and its irradiance on a surface facing it (E_sun)
  - the sky's irradiance on a horizontal surface without the sun (E_sky)
  - the mean horizon colour (0 to 3 degrees up), for fog
  - the mean sky radiance above the horizon

Unity's built-in shading computes albedo * (light * N.L + ambient), with the
skybox-derived ambient equal to the sky's radiance, so a directional light
matching the HDRI has intensity E_sun / pi in the same units as the sky.

Azimuth is reported in image terms: degrees from the image centre, increasing
to the right. Turning that into a Unity light rotation depends on how the
importer wraps the lat-long image onto the cubemap, so the caller converts it
and checks the result with a render.

    python tools/hdri_sun.py sky.hdr [preview.png]
"""
import math
import sys

import numpy as np


def read_hdr(path):
    with open(path, "rb") as f:
        data = f.read()
    # header ends at the first empty line; the resolution line follows
    head_end = data.index(b"\n\n")
    header = data[:head_end].decode("latin-1")
    if "32-bit_rle_rgbe" not in header:
        raise SystemExit("not an RGBE file: " + header[:80])
    rest = data[head_end + 2:]
    nl = rest.index(b"\n")
    res = rest[:nl].decode("latin-1").split()
    if res[0] != "-Y" or res[2] != "+X":
        raise SystemExit("unsupported orientation " + " ".join(res))
    h, w = int(res[1]), int(res[3])
    buf = np.frombuffer(rest[nl + 1:], dtype=np.uint8)

    out = np.empty((h, w, 4), dtype=np.uint8)
    pos = 0
    for y in range(h):
        # new-style RLE scanline: 2 2 hi lo
        # (int() first: shifting a numpy uint8 left by 8 overflows to zero)
        if not (buf[pos] == 2 and buf[pos + 1] == 2 and (int(buf[pos + 2]) << 8 | int(buf[pos + 3])) == w):
            raise SystemExit("flat or old-style RLE scanlines are not handled")
        pos += 4
        for c in range(4):
            x = 0
            row = out[y, :, c]
            while x < w:
                n = int(buf[pos])
                pos += 1
                if n > 128:
                    n -= 128
                    row[x:x + n] = buf[pos]
                    pos += 1
                else:
                    row[x:x + n] = buf[pos:pos + n]
                    pos += n
                x += n
    e = out[:, :, 3].astype(np.int32)
    scale = np.where(e > 0, np.ldexp(1.0, e - 136), 0.0).astype(np.float32)
    rgb = out[:, :, :3].astype(np.float32) * scale[:, :, None]
    return rgb


def describe(rgb):
    h, w, _ = rgb.shape
    lum = rgb @ np.array([0.2126, 0.7152, 0.0722], dtype=np.float32)
    lat = (0.5 - (np.arange(h) + 0.5) / h) * np.pi
    domega = (np.cos(lat) * (np.pi / h) * (2.0 * np.pi / w))[:, None]

    y, x = np.unravel_index(np.argmax(lum), lum.shape)
    mask = lum > lum[y, x] * 0.5
    ys, xs = np.nonzero(mask)
    cy, cx = ys.mean(), xs.mean()
    elev = (0.5 - (cy + 0.5) / h) * 180.0
    az = ((cx + 0.5) / w - 0.5) * 360.0

    # the sun: pixels within 3 degrees of the peak that are well above the sky
    r = int(math.ceil(3.0 / 180.0 * h))
    near = np.zeros(lum.shape, dtype=bool)
    near[max(0, y - r):y + r + 1, max(0, x - r):x + r + 1] = True
    sky_level = np.median(lum[: h // 2])
    sunmask = near & (lum > sky_level * 20.0)
    e_sun = (rgb * (domega * sunmask)[:, :, None]).sum(axis=(0, 1))

    # sky irradiance on a horizontal surface, sun removed
    cos_zenith = np.clip(np.sin(lat), 0.0, None)[:, None]
    sky_only = rgb * (~sunmask)[:, :, None]
    e_sky = (sky_only * (domega * cos_zenith)[:, :, None]).sum(axis=(0, 1))

    band = (lat >= 0) & (lat <= math.radians(3.0))
    horizon = rgb[band].reshape(-1, 3).mean(axis=0)
    upper = lat > 0
    # domega is per row; every column of a row has the same solid angle
    sky_mean = (rgb[upper] * domega[upper][:, :, None]).sum(axis=(0, 1)) / (domega[upper].sum() * w)

    return {
        "size": (w, h), "sun_px": (x, y), "elev": elev, "az": az,
        "peak": float(lum[y, x]), "e_sun": e_sun, "e_sky": e_sky,
        "horizon": horizon, "sky_mean": sky_mean,
    }


def main():
    rgb = read_hdr(sys.argv[1])
    d = describe(rgb)
    e_sun, e_sky = d["e_sun"], d["e_sky"]
    print("size %dx%d" % d["size"])
    print("sun at pixel %d,%d  elevation %.2f deg  image azimuth %.2f deg  peak luminance %.0f" % (
        d["sun_px"] + (d["elev"], d["az"], d["peak"])))
    print("E_sun (facing the sun)   %.3f %.3f %.3f" % tuple(e_sun))
    print("E_sky (horizontal, no sun) %.3f %.3f %.3f" % tuple(e_sky))
    lum = lambda c: 0.2126 * c[0] + 0.7152 * c[1] + 0.0722 * c[2]
    e_sun_h = e_sun * math.sin(math.radians(d["elev"]))
    print("direct share of horizontal irradiance %.0f%%" % (100 * lum(e_sun_h) / (lum(e_sun_h) + lum(e_sky))))
    light = e_sun / math.pi
    print("Unity light intensity at sky exposure 1: %.3f, colour %.3f %.3f %.3f" % (
        light.max(), *(light / light.max())))
    print("horizon mean (0-3 deg) %.3f %.3f %.3f" % tuple(d["horizon"]))
    print("sky mean radiance      %.3f %.3f %.3f" % tuple(d["sky_mean"]))

    if len(sys.argv) > 2:
        from PIL import Image
        t = rgb / (1.0 + rgb)
        t = np.clip(t ** (1 / 2.2), 0, 1)
        Image.fromarray((t * 255).astype(np.uint8)).resize((1024, 512)).save(sys.argv[2])


if __name__ == "__main__":
    main()
