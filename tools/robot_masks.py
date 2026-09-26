"""Combine the robot bakes (tools/robot_bake.py) into one detail mask per part.

    python tools/robot_masks.py

Mask channels, read by FarmSim/RobotSurface. White means "no effect" in every
channel, so an unset mask leaves a material exactly as it was:
  R  ambient occlusion
  G  1 - edge wear: convex edges only (bevelled and unoccluded), broken into
     patches by noise, the way paint chips off a working machine rather than
     wearing evenly along every edge
  B  1 - grime: collects where the part is occluded, plus a faint, very low
     frequency film of dust everywhere
Writes Assets/Textures/Robots/<file>__<object>_Mask.png.
"""
import glob
import os

import numpy as np
from PIL import Image
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
BAKES = os.environ.get("ROBOT_BAKE_OUT", os.path.join(HERE, "robot_bake_out"))
OUT = os.path.join(REPO, "Assets", "Textures", "Robots")


def smooth_noise(shape, sigma, seed):
    n = np.random.RandomState(seed).rand(*shape).astype(np.float32)
    n = ndimage.gaussian_filter(n, sigma)
    n -= n.min()
    return n / max(n.max(), 1e-6)


def smoothstep(e0, e1, x):
    t = np.clip((x - e0) / (e1 - e0), 0, 1)
    return t * t * (3 - 2 * t)


def main():
    os.makedirs(OUT, exist_ok=True)
    for i, ao_path in enumerate(sorted(glob.glob(os.path.join(BAKES, "*_ao.png")))):
        name = os.path.basename(ao_path)[:-len("_ao.png")]
        ao = np.asarray(Image.open(ao_path).convert("L"), np.float32) / 255.0
        edge = np.asarray(Image.open(ao_path.replace("_ao.png", "_edge.png")).convert("L"), np.float32) / 255.0
        h, w = ao.shape
        ao = ndimage.gaussian_filter(ao, 0.7)

        # convex edges stay unoccluded; concave ones (seams, inside corners)
        # are dark in the AO and belong to grime, not wear
        convex = edge * smoothstep(0.55, 0.85, ao)
        chips = smoothstep(0.45, 0.7, smooth_noise((h, w), max(1.5, w / 160), 1000 + i))
        wear = np.clip(ndimage.gaussian_filter(convex, 0.6) * 1.4, 0, 1) * (0.25 + 0.75 * chips)

        dust = smooth_noise((h, w), max(4, w / 24), 2000 + i)
        # a field machine is never clean: a film of dust over everything, thicker
        # in the occluded places
        grime = np.clip((1.0 - ao) ** 0.7 * (0.55 + 0.45 * dust) + 0.1 + 0.15 * dust, 0, 1)

        mask = np.dstack([ao, 1.0 - wear, 1.0 - grime])
        Image.fromarray((np.clip(mask, 0, 1) * 255 + 0.5).astype(np.uint8), "RGB").save(
            os.path.join(OUT, name + "_Mask.png"), optimize=True)
        print("%-50s %4dpx  ao %.2f  wear %.3f  grime %.3f" % (name, w, ao.mean(), wear.mean(), grime.mean()))


if __name__ == "__main__":
    main()
