"""Bleed colour into the transparent parts of the tree impostor renders.

tree_gen.py renders the billboards in Blender, whose Python has no scipy. The
renders come out with black in every fully transparent pixel, and mipmapping
averages that black into the silhouette, so distant trees get dark fringes.
This fills the transparent area with nearby foliage colour, keeping alpha.

    python tools/finish_impostors.py
"""
import glob
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_vegetation_textures import dilate_colour, OUT  # noqa: E402


def main():
    paths = sorted(glob.glob(os.path.join(OUT, "*_Impostor.png")))
    for p in paths:
        im = np.asarray(Image.open(p).convert("RGBA"), np.float32) / 255.0
        rgb, a = im[:, :, :3], im[:, :, 3]
        # Blender writes straight alpha; undo any premultiplied darkening at the
        # soft edge before bleeding
        rgb = np.where(a[:, :, None] > 0.02, rgb, 0.0)
        filled = dilate_colour(rgb, a)
        out = np.dstack([filled, a])
        Image.fromarray((np.clip(out, 0, 1) * 255 + 0.5).astype(np.uint8), "RGBA").save(p, optimize=True)
        print("%s  coverage %.2f" % (os.path.basename(p), float((a > 0.4).mean())))


if __name__ == "__main__":
    main()
