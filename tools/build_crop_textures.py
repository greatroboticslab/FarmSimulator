"""Texture atlases for the generated crop plants (tools/crop_gen.py).

    python tools/build_crop_textures.py

Soybean, 1024x1024, four 512 cells:
  0, 1, 2  a leaflet each, cut from ambientCG LeafSet022 (CC0): ovate,
           smooth-edged, mid-green, the closest CC0 match to a soybean
           leaflet. Tip up, base at the cell's bottom centre (the card pivot).
  3        a pod on the left three quarters, drawn here: soybean pods are
           flattened, bulge over two or three seeds and are covered in tawny
           hairs, which is what gives them their fuzzy outline; a stem strip
           on the right eighth.
Healthy and senesced (dead) versions share the layout, so the dead plant model
uses the same UVs. Normal maps: the leaf scan's own, a dome for the pod.

Needs tools/acg/LeafSet022_1K-PNG.zip (or ACG_DIR), not in the repo:
    curl -sL "https://ambientcg.com/get?file=LeafSet022_1K-PNG.zip" -o acg/LeafSet022_1K-PNG.zip
"""
import math
import os
import random
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_vegetation_textures import CELL, dilate_colour, load_leaves, new_cell, paste, transform_sprite  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "Assets", "Textures", "Crops")


def leaflet_cell(sprite):
    cell = new_cell()
    h = sprite[1].shape[0]
    scale = CELL * 0.92 / h
    tr = transform_sprite(sprite, 0.0, scale, False)
    paste(cell, tr, CELL / 2, CELL - 6 - tr[1].shape[0] / 2)
    return cell


def pod_cell(rng):
    """Two pods side by side on the left, a stem strip on the right."""
    cell = new_cell()
    crgb, ca, cn = cell
    ss = 2
    for k, x0 in enumerate((CELL * 0.2, CELL * 0.52)):
        size = (CELL * ss, CELL * ss)
        shape = Image.new("L", size, 0)
        shade = Image.new("L", size, 0)
        d, ds = ImageDraw.Draw(shape), ImageDraw.Draw(shade)
        length = CELL * rng.uniform(0.8, 0.9) * ss
        top = (CELL * ss - length) / 2
        half = CELL * 0.085 * ss
        seeds = rng.choice((2, 3, 3))
        left, right = [], []
        steps = 60
        for i in range(steps + 1):
            t = i / steps
            taper = math.sin(math.pi * min(max(t, 0.0), 1.0)) ** 0.45
            bulge = 0.78 + 0.22 * abs(math.sin(math.pi * seeds * t))
            w = half * taper * bulge
            bend = (t - 0.5) ** 2 * CELL * 0.08 * ss * (1 if k == 0 else -1)
            y = top + t * length
            left.append((x0 * ss + bend - w, y))
            right.append((x0 * ss + bend + w, y))
        d.polygon(left + right[::-1], fill=255)
        # a soft rise over each seed (the first version drew hard ellipses,
        # which read as stripes at plant scale)
        for s in range(seeds):
            cy = top + (s + 0.5) / seeds * length
            r = half * 0.7
            ds.ellipse([x0 * ss - r, cy - r * 1.8, x0 * ss + r, cy + r * 1.8], fill=120)
        # fine hairs along the outline, a thin fuzz rather than a fringe
        for (px, py), sign in [(p, -1) for p in left] + [(p, 1) for p in right]:
            for _ in range(3):
                ln = rng.uniform(3, 7) * ss
                ang = rng.uniform(-0.6, 0.6)
                ex = px + sign * ln * math.cos(ang)
                ey = py + ln * math.sin(ang) * 0.6
                d.line([(px, py + rng.uniform(-3, 3)), (ex, ey)], fill=110, width=1)
        a = np.asarray(shape.resize((CELL, CELL), Image.LANCZOS), np.float32) / 255.0
        sh = ndimage.gaussian_filter(np.asarray(shade.resize((CELL, CELL), Image.LANCZOS), np.float32) / 255.0, 14)
        base = np.array([0.44, 0.52, 0.24], np.float32)
        hair = np.array([0.55, 0.55, 0.34], np.float32)
        body = (a > 0.9)[:, :, None]
        rgb = np.where(body, base * (0.82 + 0.35 * sh[:, :, None]), hair)
        noise = ndimage.gaussian_filter(np.random.RandomState(rng.randrange(1 << 30)).rand(CELL, CELL), 1.2)
        rgb = rgb * (0.9 + 0.2 * noise[:, :, None])
        # domed normal from the shading
        gy, gx = np.gradient(ndimage.gaussian_filter(sh, 3))
        n = np.dstack([-gx * 6, gy * 6, np.ones_like(gx)])
        n /= np.linalg.norm(n, axis=2, keepdims=True)
        m = a[:, :, None]
        crgb[:] = rgb * m + crgb * (1 - m)
        cn[:] = n * m + cn * (1 - m)
        ca[:] = a + ca * (1 - a)
    # stem strip: the right eighth of the cell, solid, faint vertical fibres
    x0 = int(CELL * 7 / 8)
    fibre = ndimage.gaussian_filter(np.random.RandomState(7).rand(CELL, CELL - x0), (8, 0.6))
    crgb[:, x0:] = np.array([0.33, 0.42, 0.20], np.float32) * (0.85 + 0.3 * fibre[:, :, None])
    ca[:, x0:] = 1.0
    cn[:, x0:] = (0, 0, 1)
    return cell


def senesce(cell, rng):
    """The same cell as a dead plant: leaves dried to tan with darker blotches,
    pods and stems gone grey-brown."""
    crgb, ca, cn = cell
    lum = crgb @ np.array([0.3, 0.59, 0.11], np.float32)
    blotch = ndimage.gaussian_filter(np.random.RandomState(rng.randrange(1 << 30)).rand(*lum.shape), 10)
    blotch = np.clip((blotch - 0.45) * 6, 0, 1)
    tan = np.array([0.66, 0.52, 0.33], np.float32)
    dark = np.array([0.36, 0.27, 0.18], np.float32)
    rgb = lum[:, :, None] * 1.5 * (tan * (1 - blotch[:, :, None]) + dark * blotch[:, :, None])
    return np.clip(rgb, 0, 1), ca.copy(), cn.copy()


def save(name, cells):
    rgb = np.zeros((CELL * 2, CELL * 2, 3), np.float32)
    a = np.zeros((CELL * 2, CELL * 2), np.float32)
    n = np.zeros((CELL * 2, CELL * 2, 3), np.float32)
    for i, (crgb, ca, cn) in enumerate(cells):
        y, x = (i // 2) * CELL, (i % 2) * CELL
        rgb[y:y + CELL, x:x + CELL] = crgb
        a[y:y + CELL, x:x + CELL] = ca
        n[y:y + CELL, x:x + CELL] = cn
    rgb = dilate_colour(rgb, a)
    Image.fromarray((np.clip(np.dstack([rgb, a]), 0, 1) * 255 + 0.5).astype(np.uint8), "RGBA").save(
        os.path.join(OUT, name + "_Albedo.png"), optimize=True)
    n[a < 0.05] = (0, 0, 1)
    Image.fromarray(((n * 0.5 + 0.5) * 255 + 0.5).clip(0, 255).astype(np.uint8), "RGB").save(
        os.path.join(OUT, name + "_Normal.png"), optimize=True)
    print("%-12s coverage %.2f  mean colour %s" % (name, float(a.mean()), np.round(rgb[a > 0.5].mean(0), 3)))


def main():
    os.makedirs(OUT, exist_ok=True)
    rng = random.Random(1926)
    leaves = load_leaves("LeafSet022_1K-PNG")
    print("leaflets: %d" % len(leaves))

    # the three least damaged: fewest dark (necrotic) pixels inside the leaf
    def damage(s):
        rgb, a, _ = s
        inside = a > 0.5
        lum = rgb @ np.array([0.3, 0.59, 0.11], np.float32)
        return float((lum[inside] < 0.22).mean())
    leaves.sort(key=damage)
    # LeafSet022 is spring-light; a soybean canopy in the growing season is a
    # deeper, less yellow green
    summer = np.array([0.74, 0.82, 0.72], np.float32)
    leaves = [(np.clip(rgb * summer, 0, 1), a, n) for rgb, a, n in leaves[:3]]
    cells = [leaflet_cell(s) for s in leaves] + [pod_cell(rng)]
    save("Soybean", cells)
    save("SoybeanDead", [senesce(c, rng) for c in cells])


if __name__ == "__main__":
    main()
