"""Turn Poly Haven rock scans into game rocks with three LODs.

    blender --background --python tools/rock_gen.py -- <folder with the glTF downloads>

The scans are 23k to 124k triangles, and the farm scatters over a thousand
rocks. Each becomes a collapse-decimated mesh at about 2400 / 600 / 150
triangles. Collapse decimation keeps the UVs, so the scan's own CC0 textures
still fit: the colour map gets half-strength ambient occlusion from the ARM
map multiplied in (contact shading for free), the GL normal map is used as is,
and the fine detail the decimation removes is mostly what that normal map
already carries.

Writes Assets/Models/Rocks/<Name>.fbx (<Name>_LOD0/1/2) and
Assets/Textures/Rocks/<Name>_Albedo.png, <Name>_Normal.png.
Sources: CC0, polyhaven.com (rock_07, rock_09, stone_01, boulder_01).
"""
import glob
import os
import sys

import bmesh
import bpy
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
MODELS = os.path.join(REPO, "Assets", "Models", "Rocks")
TEX = os.path.join(REPO, "Assets", "Textures", "Rocks")
TARGETS = (2400, 600, 150)
AO_STRENGTH = 0.5
SATURATION = 0.35
COOL = (0.97, 1.0, 1.04)

#asset -> (name, longest side in metres). The scans are true to scale, and
#three of them are pebbles (14 to 32 cm); on a farm the visible stones are
#field stones and boulders, so they are scaled up. A 1k texture on a 1 m rock
#is still a millimetre a texel.
ROCKS = {
    "rock_07": ("Rock07", 0.9),
    "rock_09": ("Rock09", 0.65),
    "stone_01": ("Stone01", 0.75),
    "boulder_01": ("Boulder01", None),
}


def tri_count(obj):
    return sum(len(p.vertices) - 2 for p in obj.data.polygons)


def import_rock(folder, asset, longest):
    before = set(bpy.data.objects)
    gltf = glob.glob(os.path.join(folder, asset, "*.gltf"))[0]
    # merged vertices: glTF stores a vertex per UV seam side, and collapse
    # decimation will not cross those boundaries (the boulder stalled at 26k)
    bpy.ops.import_scene.gltf(filepath=gltf, merge_vertices=True)
    meshes = [o for o in bpy.data.objects if o not in before and o.type == "MESH"]
    bpy.ops.object.select_all(action="DESELECT")
    for o in meshes:
        o.select_set(True)
    bpy.context.view_layer.objects.active = meshes[0]
    if len(meshes) > 1:
        bpy.ops.object.join()
    obj = bpy.context.view_layer.objects.active
    # glTF imports can arrive parented to an empty with a rotation; bake it all
    bpy.ops.object.parent_clear(type="CLEAR_KEEP_TRANSFORM")
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    for o in list(bpy.data.objects):
        if o not in before and o is not obj:
            bpy.data.objects.remove(o)
    # Weld by position. boulder_01 arrives as a near triangle soup (111k of its
    # 155k edges are boundary edges), which collapse decimation cannot touch.
    # UVs live on face corners and survive the weld; the scan's custom normals
    # do not, and are replaced by smooth ones (the normal map carries detail).
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=0.0005)
    bm.to_mesh(obj.data)
    bm.free()
    if obj.data.has_custom_normals:
        bpy.context.view_layer.objects.active = obj
        bpy.ops.mesh.customdata_custom_splitnormals_clear()
    # pivot at the bottom centre, so the rock stands on the ground at 0
    vs = np.empty(len(obj.data.vertices) * 3, dtype=np.float32)
    obj.data.vertices.foreach_get("co", vs)
    vs = vs.reshape(-1, 3)
    lo, hi = vs.min(0), vs.max(0)
    shift = np.array([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]], np.float32)
    vs = vs + shift
    scale = 1.0 if longest is None else longest / float((hi - lo).max())
    obj.data.vertices.foreach_set("co", (vs * scale).ravel())
    obj.data.update()
    return obj, (hi - lo) * scale


def lod_copy(src, name, target):
    obj = src.copy()
    obj.data = src.data.copy()
    obj.name = name
    obj.data.name = name
    bpy.context.scene.collection.objects.link(obj)
    ratio = min(1.0, target / max(1, tri_count(src)))
    if ratio < 1.0:
        mod = obj.modifiers.new("Decimate", "DECIMATE")
        mod.decimate_type = "COLLAPSE"
        mod.ratio = ratio
        mod.use_collapse_triangulate = True
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)
    obj.data.polygons.foreach_set("use_smooth", [True] * len(obj.data.polygons))
    return obj


def textures(folder, asset, name):
    def load(pattern):
        path = glob.glob(os.path.join(folder, asset, "textures", pattern))[0]
        img = bpy.data.images.load(path)
        img.colorspace_settings.name = "Non-Color"   # read raw values
        w, h = img.size
        px = np.empty(w * h * 4, dtype=np.float32)
        img.pixels.foreach_get(px)
        return img, px.reshape(h, w, 4)

    diff_img, diff = load("*_diff_*.jpg")
    _, arm = load("*_arm_*.jpg")
    _, nor = load("*_nor_gl_*.jpg")
    # diff and arm are the same size in these downloads; the colour is sRGB, the
    # AO is data, so multiply in sRGB space the way a painter would
    ao = arm[:, :, 0:1]
    col = diff.copy()
    rgb = diff[:, :, :3] * (1.0 - (1.0 - ao) * AO_STRENGTH)
    # All four scans are warm sandstone and granite (mean sRGB around .40 .32
    # .24). Middle Tennessee field stone is grey limestone: keep the luminance
    # and the scan's texture, keep a third of its colour, cool it slightly.
    lum = (rgb @ np.array([0.2126, 0.7152, 0.0722], np.float32))[:, :, None]
    rgb = (lum + (rgb - lum) * SATURATION) * np.array(COOL, np.float32)
    col[:, :, :3] = rgb

    def save(arr, path):
        h, w, _ = arr.shape
        out = bpy.data.images.new(os.path.basename(path), w, h, alpha=False)
        out.colorspace_settings.name = "Non-Color"
        out.pixels.foreach_set(np.clip(arr, 0, 1).ravel())
        out.filepath_raw = path
        out.file_format = "PNG"
        out.save()

    save(col, os.path.join(TEX, name + "_Albedo.png"))
    save(nor, os.path.join(TEX, name + "_Normal.png"))


def export(objs, path):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    bpy.ops.export_scene.fbx(filepath=path, use_selection=True, object_types={"MESH"},
                             axis_forward="-Z", axis_up="Y", bake_space_transform=True,
                             apply_scale_options="FBX_SCALE_ALL", mesh_smooth_type="FACE",
                             use_mesh_modifiers=False, add_leaf_bones=False, path_mode="STRIP")


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    folder = argv[0]
    os.makedirs(MODELS, exist_ok=True)
    os.makedirs(TEX, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    for asset, (name, longest) in ROCKS.items():
        src, dims = import_rock(folder, asset, longest)
        full = tri_count(src)
        mat = bpy.data.materials.new(name + "Rock")
        src.data.materials.clear()
        src.data.materials.append(mat)
        lods = [lod_copy(src, "%s_LOD%d" % (name, i), t) for i, t in enumerate(TARGETS)]
        bpy.data.objects.remove(src)
        textures(folder, asset, name)
        export(lods, os.path.join(MODELS, name + ".fbx"))
        print("ROCK %s from %s: %.2f x %.2f x %.2f m, %d triangles -> %s" % (
            name, asset, dims[0], dims[1], dims[2], full, " / ".join(str(tri_count(o)) for o in lods)))
        for o in lods:
            bpy.data.objects.remove(o)
    print("DONE")


main()
