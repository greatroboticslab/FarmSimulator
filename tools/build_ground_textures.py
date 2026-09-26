"""Turn the ambientCG 2K CC0 sets into the four terrain layers the project expects.

For each layer:
  albedo = Color * softened AmbientOcclusion, resized to 1024, saved as PNG
  normal = NormalGL (Unity uses the OpenGL green-up convention), resized to 1024
  smoothness = 1 - mean(Roughness), reported only: the four scans' roughness
      maps are not calibrated against each other, so FarmTerrainBuilder uses
      hand-set per-layer values instead (see BuildLayers there)

FarmTerrainBuilder loads these by exact filename from Assets/Textures/Ground,
so the names must not change.

Re-running this needs the source zips in tools/acg (or the folder named by
ACG_DIR). They are NOT in the repo (about 145 MB). Fetch them again with, for
each of Grass001, Grass004, Ground103 and Ground048:

    curl -sL "https://ambientcg.com/get?file=<AssetID>_2K-JPG.zip" -o acg/<AssetID>.zip

Everything from ambientCG is CC0, no attribution required.
"""
import io
import json
import os
import zipfile

from PIL import Image, ImageChops

SCRATCH = os.path.dirname(os.path.abspath(__file__))
ACG = os.environ.get("ACG_DIR", os.path.join(SCRATCH, "acg"))
OUT = os.path.join(os.path.dirname(SCRATCH), "Assets", "Textures", "Ground")
SIZE = 1024
# how much baked ambient occlusion to multiply into the albedo. 1.0 is the raw
# AO and crushes the shadows on a surface that also receives real-time shading;
# half of it reads as depth without going muddy.
AO_STRENGTH = 0.5

# layer name in the project -> ambientCG asset that fills it
LAYERS = [
    ("Grass",    "Grass001"),
    ("DryGrass", "Grass004"),
    ("Dirt",     "Ground103"),
    ("Soil",     "Ground048"),
]


def member(zf, suffix):
    for n in zf.namelist():
        if n.endswith(suffix):
            return n
    raise KeyError(suffix + " not in " + zf.filename)


def load(zf, suffix, mode="RGB"):
    with zf.open(member(zf, suffix)) as fh:
        return Image.open(io.BytesIO(fh.read())).convert(mode)


def main():
    if not os.path.isdir(OUT):
        raise SystemExit("missing output dir: " + OUT)

    report = {}
    for layer, asset in LAYERS:
        zf = zipfile.ZipFile(os.path.join(ACG, asset + ".zip"))

        color = load(zf, "_Color.jpg")
        ao = load(zf, "_AmbientOcclusion.jpg", "L")
        normal = load(zf, "_NormalGL.jpg")
        rough = load(zf, "_Roughness.jpg", "L")

        # bake softened AO into the albedo: contact shadowing that costs nothing
        # at runtime, which is the entire point of doing this offline
        if ao.size != color.size:
            ao = ao.resize(color.size, Image.LANCZOS)
        faded = ao.point(lambda v: int(255 - (255 - v) * AO_STRENGTH))
        color = ImageChops.multiply(color, Image.merge("RGB", (faded, faded, faded)))

        color = color.resize((SIZE, SIZE), Image.LANCZOS)
        normal = normal.resize((SIZE, SIZE), Image.LANCZOS)

        albedo_path = os.path.join(OUT, layer + "Albedo.png")
        normal_path = os.path.join(OUT, layer + "Normal.png")
        color.save(albedo_path, optimize=True)
        normal.save(normal_path, optimize=True)

        hist = rough.histogram()
        mean = sum(i * c for i, c in enumerate(hist)) / float(sum(hist)) / 255.0
        report[layer] = {
            "asset": asset,
            "smoothness": round(1.0 - mean, 3),
            "albedo_kb": os.path.getsize(albedo_path) // 1024,
            "normal_kb": os.path.getsize(normal_path) // 1024,
        }
        print("%-9s <- %-10s smoothness %.3f  albedo %5d KB  normal %5d KB" % (
            layer, asset, report[layer]["smoothness"],
            report[layer]["albedo_kb"], report[layer]["normal_kb"]))

    with open(os.path.join(ACG, "ground_report.json"), "w") as fh:
        json.dump(report, fh, indent=2)
    total = sum(v["albedo_kb"] + v["normal_kb"] for v in report.values())
    print("total added to repo: %.1f MB" % (total / 1024.0))


if __name__ == "__main__":
    main()
