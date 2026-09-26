"""Build the leaf-card and bark textures for the generated trees.

Leaf cards: each species gets a 1024x1024 atlas of four 512x512 cells, each
cell a twig with leaves arranged on it, so a tree needs only a few hundred
cards instead of thousands of single leaves.

  Oak   - leaves cut from ambientCG LeafSet016 (six oak leaves)
  Elm   - leaves cut from ambientCG LeafSet005 (serrated leaves on a twig)
  Cedar - drawn here: scale-leaf sprays like Eastern red cedar, the most common
          evergreen along Middle Tennessee fence rows. No CC0 photo set of that
          foliage exists on ambientCG; at the distances trees are seen from, a
          dense, dark, branching spray reads correctly.

For every leaf pasted onto a cell the colour, alpha and normal map are rotated
and scaled together, and the normal vectors are rotated by the same angle. A
rotated normal map whose vectors are not rotated lights every leaf as if it
still pointed straight up.

Colour is dilated into the transparent areas, so mipmaps average leaf green
into the edges instead of black, which is what gives alpha-tested foliage dark
fringes at a distance.

Bark: ambientCG Bark012 (oak) and Bark015, colour with half-strength ambient
occlusion multiplied in, plus the GL normal map.

Needs the source zips in tools/acg (or ACG_DIR); they are not in the repo:

    curl -sL "https://ambientcg.com/get?file=<AssetID>_1K-PNG.zip" -o acg/<AssetID>_1K-PNG.zip
    (LeafSet016, LeafSet005; Bark012 and Bark015 as _1K-JPG)

Everything from ambientCG is CC0.
"""
import io
import math
import os
import random
import zipfile

import numpy as np
from PIL import Image, ImageDraw, ImageFilter
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
ACG = os.environ.get("ACG_DIR", os.path.join(HERE, "acg"))
OUT = os.path.join(os.path.dirname(HERE), "Assets", "Textures", "Vegetation")
CELL = 512
AO_STRENGTH = 0.5


# ----------------------------------------------------------------------------
# loading

def zip_image(asset, suffix, mode):
    z = zipfile.ZipFile(os.path.join(ACG, asset + ".zip"))
    for n in z.namelist():
        if n.endswith(suffix):
            return Image.open(io.BytesIO(z.read(n))).convert(mode)
    raise KeyError(suffix + " not in " + asset)


def load_leaves(asset, min_area=2000):
    """Cut every separate leaf (connected opaque region) out of a leaf set.

    Returns sprites as float arrays: rgb (h,w,3) 0..1 sRGB, alpha (h,w) 0..1,
    normal (h,w,3) -1..1 tangent space, GL convention (y up).
    """
    col = np.asarray(zip_image(asset, "_Color.png", "RGB"), dtype=np.float32) / 255.0
    op = np.asarray(zip_image(asset, "_Opacity.png", "L"), dtype=np.float32) / 255.0
    nrm = np.asarray(zip_image(asset, "_NormalGL.png", "RGB"), dtype=np.float32) / 255.0 * 2.0 - 1.0
    labels, n = ndimage.label(op > 0.5)
    sprites = []
    for i, sl in enumerate(ndimage.find_objects(labels)):
        if sl is None:
            continue
        mask = labels[sl] == i + 1
        if mask.sum() < min_area:
            continue
        # a few pixels of margin so rotation has room for the soft edge
        y0, y1 = max(0, sl[0].start - 4), min(op.shape[0], sl[0].stop + 4)
        x0, x1 = max(0, sl[1].start - 4), min(op.shape[1], sl[1].stop + 4)
        own = (labels[y0:y1, x0:x1] == i + 1) | (labels[y0:y1, x0:x1] == 0)
        a = op[y0:y1, x0:x1] * own
        sprites.append((col[y0:y1, x0:x1], a, nrm[y0:y1, x0:x1]))
    return sprites


# ----------------------------------------------------------------------------
# compositing

def transform_sprite(sprite, angle_deg, scale, mirror):
    """Rotate (counter-clockwise, as displayed), scale and optionally mirror a
    sprite, rotating its normal vectors to match."""
    rgb, a, nrm = sprite
    h, w = a.shape
    size = (max(2, int(w * scale)), max(2, int(h * scale)))

    def tf(arr, resample):
        im = Image.fromarray(arr.astype(np.float32), mode="F")
        if mirror:
            im = im.transpose(Image.FLIP_LEFT_RIGHT)
        im = im.resize(size, resample)
        im = im.rotate(angle_deg, resample=Image.BICUBIC, expand=True)
        return np.asarray(im, dtype=np.float32)

    out_rgb = np.stack([tf(rgb[:, :, c], Image.LANCZOS) for c in range(3)], axis=2)
    out_a = np.clip(tf(a, Image.LANCZOS), 0.0, 1.0)
    nx = tf(nrm[:, :, 0], Image.BILINEAR)
    ny = tf(nrm[:, :, 1], Image.BILINEAR)
    nz = tf(nrm[:, :, 2], Image.BILINEAR)
    if mirror:
        nx = -nx
    t = math.radians(angle_deg)
    c, s = math.cos(t), math.sin(t)
    rx = nx * c - ny * s
    ry = nx * s + ny * c
    out_n = np.stack([rx, ry, nz], axis=2)
    ln = np.linalg.norm(out_n, axis=2, keepdims=True)
    out_n = np.where(ln > 1e-4, out_n / np.maximum(ln, 1e-4), np.array([0, 0, 1], dtype=np.float32))
    return np.clip(out_rgb, 0, 1), out_a, out_n


def paste(cell, sprite, cx, cy, tint=1.0):
    """Alpha-over a transformed sprite so its centre lands at (cx, cy)."""
    rgb, a, nrm = sprite
    crgb, ca, cn = cell
    h, w = a.shape
    x0, y0 = int(round(cx - w / 2)), int(round(cy - h / 2))
    sx0, sy0 = max(0, -x0), max(0, -y0)
    dx0, dy0 = max(0, x0), max(0, y0)
    dx1, dy1 = min(CELL, x0 + w), min(CELL, y0 + h)
    if dx1 <= dx0 or dy1 <= dy0:
        return
    sw, sh = dx1 - dx0, dy1 - dy0
    sa = a[sy0:sy0 + sh, sx0:sx0 + sw][:, :, None]
    crgb[dy0:dy1, dx0:dx1] = rgb[sy0:sy0 + sh, sx0:sx0 + sw] * tint * sa + crgb[dy0:dy1, dx0:dx1] * (1 - sa)
    cn[dy0:dy1, dx0:dx1] = nrm[sy0:sy0 + sh, sx0:sx0 + sw] * sa + cn[dy0:dy1, dx0:dx1] * (1 - sa)
    ca[dy0:dy1, dx0:dx1] = sa[:, :, 0] + ca[dy0:dy1, dx0:dx1] * (1 - sa[:, :, 0])


def new_cell():
    return (np.zeros((CELL, CELL, 3), np.float32), np.zeros((CELL, CELL), np.float32),
            np.tile(np.array([0, 0, 1], np.float32), (CELL, CELL, 1)))


def draw_twig(cell, pts, w0, w1, colour):
    """A tapering twig through pts, drawn supersampled; flat normals."""
    ss = 4
    im = Image.new("L", (CELL * ss, CELL * ss), 0)
    d = ImageDraw.Draw(im)
    for i in range(len(pts) - 1):
        t = i / max(1, len(pts) - 2)
        width = max(1, int((w0 + (w1 - w0) * t) * ss))
        a = (pts[i][0] * ss, pts[i][1] * ss)
        b = (pts[i + 1][0] * ss, pts[i + 1][1] * ss)
        d.line([a, b], fill=255, width=width)
        d.ellipse([b[0] - width / 2, b[1] - width / 2, b[0] + width / 2, b[1] + width / 2], fill=255)
    m = np.asarray(im.resize((CELL, CELL), Image.LANCZOS), np.float32) / 255.0
    crgb, ca, cn = cell
    col = np.array(colour, np.float32)
    crgb[:] = col * m[:, :, None] + crgb * (1 - m[:, :, None])
    ca[:] = m + ca * (1 - m)


def curve(p0, p1, bend, n=12):
    """Quadratic curve from p0 to p1 bowed sideways by `bend` pixels."""
    mx, my = (p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    ln = math.hypot(dx, dy) or 1.0
    cx, cy = mx - dy / ln * bend, my + dx / ln * bend
    pts = []
    for i in range(n + 1):
        t = i / n
        x = (1 - t) ** 2 * p0[0] + 2 * (1 - t) * t * cx + t * t * p1[0]
        y = (1 - t) ** 2 * p0[1] + 2 * (1 - t) * t * cy + t * t * p1[1]
        pts.append((x, y))
    return pts


def point_on(pts, t):
    f = t * (len(pts) - 1)
    i = min(int(f), len(pts) - 2)
    u = f - i
    x = pts[i][0] + (pts[i + 1][0] - pts[i][0]) * u
    y = pts[i][1] + (pts[i + 1][1] - pts[i][1]) * u
    dx, dy = pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1]
    return x, y, math.degrees(math.atan2(-dy, dx))    # angle measured y-up


def leaf_cluster(sprites, rng, n_leaves, leaf_len, twig_colour, hue_jitter, dark_back, colour=(1.0, 1.0, 1.0)):
    """One cell: a twig rising from the bottom centre, forking into side
    twigs, with leaves along all of them.

    The card's pivot is the bottom centre, where the twig meets the branch.
    Side twigs make the cluster wide rather than a single column, so a crown
    fills with fewer cards."""
    cell = new_cell()
    base = (CELL / 2 + rng.uniform(-15, 15), CELL - 4)
    tip = (CELL / 2 + rng.uniform(-40, 40), CELL * rng.uniform(0.16, 0.24))
    main = curve(base, tip, rng.uniform(-35, 35))
    twigs = [(main, 1.0)]
    for side in (-1, 1):
        t0 = rng.uniform(0.28, 0.5)
        x, y, along = point_on(main, t0)
        ang = math.radians(along + side * rng.uniform(38, 55))
        length = CELL * rng.uniform(0.34, 0.44)
        end = (x + math.cos(ang) * length, y - math.sin(ang) * length)
        end = (min(max(end[0], 40), CELL - 40), min(max(end[1], 40), CELL - 40))
        twigs.append((curve((x, y), end, rng.uniform(-25, 25)), 0.6))
    for pts, w in twigs:
        draw_twig(cell, pts, 7 * w, 2, twig_colour)

    leaves = []
    per = [int(round(n_leaves * 0.44)), int(round(n_leaves * 0.28)), n_leaves]
    per[2] = max(2, n_leaves - per[0] - per[1])
    for (pts, w), count in zip(twigs, per):
        for k in range(count):
            t = 0.2 + 0.8 * (k + rng.uniform(-0.3, 0.3)) / max(1, count - 1)
            t = min(max(t, 0.15), 1.0)
            x, y, along = point_on(pts, t)
            side = 1 if k % 2 == 0 else -1
            # leaves near a tip point along the twig, lower ones splay out wide
            splay = (1.0 - t) * rng.uniform(45, 80) + t * rng.uniform(10, 35)
            angle = along + side * splay
            spr = sprites[rng.randrange(len(sprites))]
            h = spr[1].shape[0]
            scale = leaf_len * rng.uniform(0.8, 1.15) / h
            # sprites are stored tip-up (90 deg); rotate from there
            leaves.append((rng.uniform(0, 1), spr, angle - 90.0, scale, rng.random() < 0.5, x, y))

    # back to front: the ones drawn first sit behind, and are a touch darker
    leaves.sort(key=lambda l: l[0])
    for depth, spr, angle, scale, mirror, x, y in leaves:
        tr = transform_sprite(spr, angle, scale, mirror)
        # push the leaf outward from its attachment point along its own axis
        a = math.radians(angle + 90.0)
        off = spr[1].shape[0] * scale * 0.45
        tint = (1.0 - dark_back * (1.0 - depth)) * (1.0 + rng.uniform(-hue_jitter, hue_jitter))
        tr = (tr[0] * np.array(colour, np.float32), tr[1], tr[2])
        paste(cell, tr, x + math.cos(a) * off, y - math.sin(a) * off, tint)
    return cell


def cedar_spray(rng):
    """Eastern red cedar style foliage: dense, rope-like sprays of tiny scale
    leaves. Branching shoots, each drawn as a chain of overlapping scale blobs,
    supersampled; everything stays inside the cell so no edge gets cut off."""
    ss = 3
    size = CELL * ss
    margin = 10 * ss
    alpha = Image.new("L", (size, size), 0)
    shade = Image.new("L", (size, size), 0)
    da, ds = ImageDraw.Draw(alpha), ImageDraw.Draw(shade)

    def inside(x, y):
        return margin < x < size - margin and margin < y < size - margin

    def shoot(x, y, ang, length, radius, depth):
        steps = max(3, int(length / (radius * 1.1)))
        for s in range(steps):
            nx = x + math.cos(math.radians(ang)) * length / steps
            ny = y - math.sin(math.radians(ang)) * length / steps
            if not inside(nx, ny):
                return
            r = radius * (1.0 - 0.45 * s / steps)
            light = int(70 + 110 * (s / steps) + rng.uniform(-25, 25))
            for _ in range(2):
                jx, jy = rng.uniform(-r, r) * 0.5, rng.uniform(-r, r) * 0.5
                box = [nx + jx - r, ny + jy - r * 0.8, nx + jx + r, ny + jy + r * 0.8]
                da.ellipse(box, fill=255)
                ds.ellipse(box, fill=max(0, min(255, light)))
            if depth > 0 and s % 3 == 1 and s < steps - 2:
                # side shoots shorten toward the tip, giving the spray its
                # pointed, feather-like outline
                taper = 1.0 - s / steps
                for side in (-1, 1):
                    if rng.random() < 0.7:
                        shoot(nx, ny, ang + side * rng.uniform(40, 62), length * taper * rng.uniform(0.35, 0.5),
                              radius * 0.8, depth - 1)
            ang += rng.uniform(-7, 7)
            x, y = nx, ny

    base = (size / 2 + rng.uniform(-20, 20) * ss, size - margin - 2 * ss)
    shoot(base[0], base[1], 90 + rng.uniform(-10, 10), size * 0.8, 4.0 * ss, 2)
    a = np.asarray(alpha.resize((CELL, CELL), Image.LANCZOS), np.float32) / 255.0
    sh = np.asarray(shade.resize((CELL, CELL), Image.LANCZOS), np.float32) / 255.0
    sh = sh / np.maximum(a, 1e-3)
    # dark blue-green, lighter new growth at the tips (brighter strokes)
    lo = np.array([0.12, 0.20, 0.11], np.float32)
    hi = np.array([0.28, 0.38, 0.20], np.float32)
    t = np.clip((sh - 0.3) / 0.4, 0, 1)[:, :, None]
    rgb = lo * (1 - t) + hi * t
    noise = ndimage.gaussian_filter(np.random.RandomState(rng.randrange(1 << 30)).rand(CELL, CELL), 2)
    rgb *= (0.85 + 0.3 * noise)[:, :, None]
    # a gently domed normal so the spray is not lit like a flat sheet
    yy, xx = np.mgrid[0:CELL, 0:CELL].astype(np.float32) / CELL - 0.5
    n = np.stack([xx * 0.6, -yy * 0.6, np.ones_like(xx)], axis=2)
    n /= np.linalg.norm(n, axis=2, keepdims=True)
    return np.clip(rgb, 0, 1), np.clip(a, 0, 1), n


def load_blades(asset, base_at_left):
    """Single grass blades from an ambientCG foliage set, turned tip-up."""
    blades = load_leaves(asset, min_area=1500)
    out = []
    for rgb, a, n in blades:
        h, w = a.shape
        # keep long thin shapes only (a blade, not a stray fragment)
        if max(h, w) < 4 * min(h, w):
            continue
        if base_at_left:
            rgb, a, n = transform_sprite((rgb, a, n), 90.0, 1.0, False)
        # the scanned blades are broad; pasted at tuft scale they read as
        # daylily leaves, so they are narrowed to half width
        h, w = a.shape
        nw = max(3, w // 2)
        rgb = np.stack([np.asarray(Image.fromarray(rgb[:, :, c], mode="F").resize((nw, h), Image.LANCZOS)) for c in range(3)], axis=2)
        a = np.asarray(Image.fromarray(a, mode="F").resize((nw, h), Image.LANCZOS))
        n = np.stack([np.asarray(Image.fromarray(n[:, :, c], mode="F").resize((nw, h), Image.BILINEAR)) for c in range(3)], axis=2)
        out.append((np.clip(rgb, 0, 1), np.clip(a, 0, 1), n))
    return out


def grass_tuft(blades, rng, count, dry_fraction, straw):
    """A 512 px tuft card: blades fanning up from the bottom centre, taller in
    the middle, the outer ones leaning further; a few dead blades; darker at
    the root where the tuft shades itself."""
    cell = new_cell()
    base_x = CELL / 2
    order = []
    for k in range(count):
        spread = rng.gauss(0, 0.45)
        lean = max(-42.0, min(42.0, spread * 38 + rng.uniform(-6, 6)))
        height = CELL * rng.uniform(0.5, 0.95) * (1.0 - 0.35 * min(1.0, abs(spread)))
        order.append((rng.random(), lean, height, base_x + spread * CELL * 0.12 + rng.uniform(-10, 10),
                      rng.random() < dry_fraction))
    order.sort(key=lambda o: o[0])
    for depth, lean, height, bx, dry in order:
        spr = blades[rng.randrange(len(blades))]
        scale = height / spr[1].shape[0]
        tr = transform_sprite(spr, -lean, scale, rng.random() < 0.5)
        rgb = tr[0]
        if dry:
            lum = rgb @ np.array([0.3, 0.59, 0.11], np.float32)
            rgb = np.clip(lum[:, :, None] * np.array(straw, np.float32) * 1.5, 0, 1)
        # bottom of the rotated blade sits at the base point
        h, w = tr[1].shape
        a = math.radians(lean)
        cx = bx + math.sin(a) * height * 0.5
        cy = CELL - 2 - math.cos(a) * height * 0.5
        tint = 0.8 + 0.3 * depth
        paste(cell, (rgb, tr[1], tr[2]), cx, cy, tint)
    crgb, ca, cn = cell
    rows = np.linspace(0, 1, CELL, dtype=np.float32)[:, None, None]
    crgb *= 0.55 + 0.45 * (1.0 - rows) ** 0.6 + 0.0 * rows
    return cell


def save_tuft(path, cell):
    crgb, ca, cn = cell
    rgb = dilate_colour(crgb, ca)
    rgba = np.dstack([rgb, ca])
    Image.fromarray((np.clip(rgba, 0, 1) * 255 + 0.5).astype(np.uint8), "RGBA").save(path, optimize=True)
    print("%-14s coverage %.2f  mean colour %s" % (os.path.basename(path), float(ca.mean()),
                                                    np.round(rgb[ca > 0.5].mean(0), 3)))


# ----------------------------------------------------------------------------
# output

def dilate_colour(rgb, alpha):
    """Fill transparent pixels with nearby leaf colour (multi-scale push-pull)."""
    filled = rgb.copy()
    known = alpha > 0.05
    acc_w = np.zeros(alpha.shape, np.float32)
    acc = np.zeros(rgb.shape, np.float32)
    for sigma in (2, 6, 18, 60):
        w = ndimage.gaussian_filter(known.astype(np.float32), sigma)
        c = np.stack([ndimage.gaussian_filter(rgb[:, :, i] * known, sigma) for i in range(3)], axis=2)
        take = (acc_w < 1e-3) & (w > 1e-3)
        acc[take] = c[take] / w[take][:, None]
        acc_w[take] = 1.0
    filled[~known] = acc[~known]
    return filled


def save_atlas(name, cells):
    rgb = np.zeros((CELL * 2, CELL * 2, 3), np.float32)
    a = np.zeros((CELL * 2, CELL * 2), np.float32)
    n = np.zeros((CELL * 2, CELL * 2, 3), np.float32)
    for i, (crgb, ca, cn) in enumerate(cells):
        y, x = (i // 2) * CELL, (i % 2) * CELL
        rgb[y:y + CELL, x:x + CELL] = crgb
        a[y:y + CELL, x:x + CELL] = ca
        n[y:y + CELL, x:x + CELL] = cn
    rgb = dilate_colour(rgb, a)
    rgba = np.dstack([rgb, a])
    Image.fromarray((np.clip(rgba, 0, 1) * 255 + 0.5).astype(np.uint8), "RGBA").save(
        os.path.join(OUT, name + "_Albedo.png"), optimize=True)
    # normals outside the leaves: straight up, so bilinear filtering at the
    # alpha edge does not pull in garbage
    n[a < 0.05] = (0, 0, 1)
    Image.fromarray(((n * 0.5 + 0.5) * 255 + 0.5).clip(0, 255).astype(np.uint8), "RGB").save(
        os.path.join(OUT, name + "_Normal.png"), optimize=True)
    cover = float(a.mean())
    print("%-14s coverage %.2f  mean leaf colour %s" % (name, cover, np.round(rgb[a > 0.5].mean(0), 3)))


def save_bark(name, asset, tint):
    col = zip_image(asset, "_Color.jpg", "RGB")
    ao = zip_image(asset, "_AmbientOcclusion.jpg", "L").resize(col.size, Image.LANCZOS)
    nrm = zip_image(asset, "_NormalGL.jpg", "RGB")
    c = np.asarray(col, np.float32) / 255.0
    o = np.asarray(ao, np.float32)[:, :, None] / 255.0
    c = c * (1.0 - (1.0 - o) * AO_STRENGTH) * np.array(tint, np.float32)
    Image.fromarray((np.clip(c, 0, 1) * 255 + 0.5).astype(np.uint8)).save(os.path.join(OUT, name + "_Albedo.png"), optimize=True)
    nrm.save(os.path.join(OUT, name + "_Normal.png"), optimize=True)
    print("%-14s from %s" % (name, asset))


def main():
    os.makedirs(OUT, exist_ok=True)
    rng = random.Random(20260926)

    oak = load_leaves("LeafSet016_1K-PNG")
    elm = load_leaves("LeafSet005_1K-PNG", min_area=6000)
    # LeafSet005's first object is a whole twig carrying three leaves; keep the
    # single leaves, which are all far smaller than it
    areas = sorted(float((s[1] > 0.5).sum()) for s in elm)
    median = areas[len(areas) // 2]
    elm = [s for s in elm if float((s[1] > 0.5).sum()) < 1.8 * median]
    print("sprites: oak %d, elm %d" % (len(oak), len(elm)))

    # LeafSet016 is spring oak, yellow-green; summer foliage in the growing
    # season is darker and less yellow
    save_atlas("OakLeaves", [leaf_cluster(oak, rng, rng.randint(15, 19), 110, (0.23, 0.17, 0.11), 0.06, 0.35,
                                          colour=(0.72, 0.82, 0.74))
                             for _ in range(4)])
    save_atlas("ElmLeaves", [leaf_cluster(elm, rng, rng.randint(16, 20), 118, (0.20, 0.15, 0.10), 0.05, 0.35)
                             for _ in range(4)])
    save_atlas("CedarFoliage", [cedar_spray(rng) for _ in range(4)])

    # Bark012 photographs pale and olive; oak bark on a farm tree reads darker
    # and greyer against the foliage
    save_bark("OakBark", "Bark012_1K-JPG", (0.74, 0.70, 0.66))
    # Eastern red cedar bark is reddish and stringy; pull Bark015 toward it
    save_bark("CedarBark", "Bark015_1K-JPG", (0.80, 0.62, 0.52))

    # terrain grass tufts: the same file names the terrain builder's tuft
    # prefabs already use, so no asset reference changes
    blades = load_blades("Foliage001_1K-PNG", False) + load_blades("Foliage008_1K-PNG", True)
    print("grass blades: %d" % len(blades))
    ground = os.path.join(os.path.dirname(OUT), "Ground")
    straw = (1.0, 0.86, 0.55)
    save_tuft(os.path.join(ground, "GrassBlades.png"), grass_tuft(blades, rng, 75, 0.12, straw))
    save_tuft(os.path.join(ground, "DryBlades.png"), grass_tuft(blades, rng, 60, 0.8, straw))


if __name__ == "__main__":
    main()
