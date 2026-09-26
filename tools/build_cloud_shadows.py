"""Cloud shadows for the sun (the Midday sky preset is partly cloudy).

    python tools/build_cloud_shadows.py

Writes Assets/Textures/Sky/CloudShadows.png: 512x512 greyscale, tiles seamlessly.
White is full sun, darker is under a cloud. Soft-edged blobs from blurred noise
at three scales (wrap-around filtering keeps it tileable), about 30% cover like
the midday sky, and a cloud cuts the sun to 45% rather than to nothing: cumulus
this size still pass some light and the sky keeps lighting the shadowed ground.
Assets/Shaders/FarmScreenSpaceShadows.shader multiplies it into the sun's
shadow, projected along the sun over 600 m; SkyPresets drifts it with the wind.
"""
import os

import numpy as np
from PIL import Image
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), "Assets", "Textures", "Sky", "CloudShadows.png")
SIZE = 512
COVER = 0.30
SHADOW = 0.45


def main():
    rng = np.random.RandomState(48)
    field = np.zeros((SIZE, SIZE), np.float32)
    for sigma, weight in ((40, 1.0), (16, 0.45), (6, 0.2)):
        n = ndimage.gaussian_filter(rng.rand(SIZE, SIZE).astype(np.float32), sigma, mode="wrap")
        n = (n - n.mean()) / (n.std() + 1e-6)
        field += n * weight
    # threshold at the cover fraction, soft edges
    t = np.quantile(field, 1.0 - COVER)
    soft = field.std() * 0.35
    cloud = np.clip((field - t) / soft + 0.5, 0, 1)
    cloud = cloud * cloud * (3 - 2 * cloud)
    light = 1.0 - (1.0 - SHADOW) * cloud
    # clouds are big and soft-edged, and so are their shadows
    light = ndimage.gaussian_filter(light, 3, mode="wrap")
    Image.fromarray((light * 255 + 0.5).astype(np.uint8), "L").save(OUT, optimize=True)
    print("CloudShadows.png: %.0f%% under cloud, mean light %.2f" % (100 * (cloud > 0.5).mean(), light.mean()))


if __name__ == "__main__":
    main()
