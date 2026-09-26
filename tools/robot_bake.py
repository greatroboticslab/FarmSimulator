"""Bake per-part detail maps for the polished robot meshes (step 3 of the
robot pass), in headless Blender with Cycles.

    blender --background --python tools/robot_bake.py

For every mesh in Assets/Models/Robots/*_polished.* that has UVs, two bakes
at a texel density of about 700 per metre (256 to 1024 px):
  AO    ambient occlusion within 8 cm: the crevices where grime collects
  edge  convex curvature (Cycles pointiness through a ramp): the bevelled
        edges that wear through first
written as <file>__<object>_ao.png / _edge.png into a scratch folder given by
ROBOT_BAKE_OUT (default tools/robot_bake_out). tools/robot_masks.py combines
them into the mask textures the Unity shader reads. The shape-keyed parts
(snipper, grippers) have no UVs and are skipped.
"""
import math
import os

import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SRC = os.path.join(REPO, "Assets", "Models", "Robots")
OUT = os.environ.get("ROBOT_BAKE_OUT", os.path.join(HERE, "robot_bake_out"))
DENSITY = 700.0


def import_file(path):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    if path.endswith(".obj"):
        bpy.ops.wm.obj_import(filepath=path)
    else:
        bpy.ops.import_scene.fbx(filepath=path)


def setup_cycles():
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = 48
    sc.render.bake.margin = 8
    world = bpy.data.worlds.new("bake")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[1].default_value = 1.0
    sc.world = world
    world.light_settings.distance = 0.08


def bake_material(image, kind):
    m = bpy.data.materials.new("bake_" + kind)
    m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = image
    nt.nodes.active = tex                     # the bake target
    if kind == "edge":
        # Edges from the Bevel node: how far a normal rounded over 1.2 cm
        # turns away from the true face normal. Pointiness was tried first and
        # is per vertex, so on big low-poly panels it smeared the bevelled
        # corners' value across whole faces (long_rover came out 75% "edge").
        geo = nt.nodes.new("ShaderNodeNewGeometry")
        bev = nt.nodes.new("ShaderNodeBevel")
        bev.inputs["Radius"].default_value = 0.012
        bev.samples = 16
        dot = nt.nodes.new("ShaderNodeVectorMath")
        dot.operation = "DOT_PRODUCT"
        nt.links.new(bev.outputs["Normal"], dot.inputs[0])
        nt.links.new(geo.outputs["True Normal"], dot.inputs[1])
        inv = nt.nodes.new("ShaderNodeMath")
        inv.operation = "SUBTRACT"
        inv.inputs[0].default_value = 1.0
        nt.links.new(dot.outputs["Value"], inv.inputs[1])
        gain = nt.nodes.new("ShaderNodeMath")
        gain.operation = "MULTIPLY"
        gain.inputs[1].default_value = 12.0
        gain.use_clamp = True
        nt.links.new(inv.outputs[0], gain.inputs[0])
        em = nt.nodes.new("ShaderNodeEmission")
        nt.links.new(gain.outputs[0], em.inputs["Color"])
        nt.links.new(em.outputs[0], out.inputs["Surface"])
    else:
        bsdf = nt.nodes.new("ShaderNodeBsdfDiffuse")
        nt.links.new(bsdf.outputs[0], out.inputs["Surface"])
    return m


def bake(obj, base):
    corners = [obj.matrix_world @ v.co for v in obj.data.vertices] if len(obj.data.vertices) < 50000 else []
    if corners:
        ext = [max(c[i] for c in corners) - min(c[i] for c in corners) for i in range(3)]
    else:
        ext = list(obj.dimensions)
    longest = max(ext) if ext else 0.5
    size = int(2 ** round(math.log2(max(256, min(1024, longest * DENSITY)))))

    results = []
    for kind in ("ao", "edge"):
        img = bpy.data.images.new("%s__%s_%s" % (base, obj.name, kind), size, size, alpha=False, float_buffer=False)
        img.colorspace_settings.name = "Non-Color"
        mat = bake_material(img, kind)
        slots = len(obj.data.materials)
        if slots == 0:
            obj.data.materials.append(mat)
        else:
            for i in range(slots):
                obj.data.materials[i] = mat
        bpy.ops.object.select_all(action="DESELECT")
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.bake(type="AO" if kind == "ao" else "EMIT", use_clear=True)
        path = os.path.join(OUT, "%s__%s_%s.png" % (base, obj.name, kind))
        img.filepath_raw = path
        img.file_format = "PNG"
        img.save()
        results.append(path)
    return size, results


def main():
    os.makedirs(OUT, exist_ok=True)
    done = set()
    for name in sorted(os.listdir(SRC)):
        if "_polished" not in name or not name.endswith((".obj", ".fbx")):
            continue
        base = os.path.splitext(name)[0].replace("_polished_single", "").replace("_polished", "")
        key = base + os.path.splitext(name)[1]
        if key in done:
            continue            # the single-material copy has the same UVs
        done.add(key)
        import_file(os.path.join(SRC, name))
        setup_cycles()
        for obj in [o for o in bpy.context.scene.objects if o.type == "MESH"]:
            if len(obj.data.uv_layers) == 0:
                print("BAKE skip %s/%s (no UVs)" % (key, obj.name))
                continue
            size, paths = bake(obj, key.replace(".", "_"))
            print("BAKE %s/%s %dpx" % (key, obj.name, size))
    print("DONE")


main()
