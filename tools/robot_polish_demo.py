"""Demonstrate the cheap hard-surface pass on one robot mesh.

Renders the mesh as-imported, then again after:
  1. smooth shading by angle  (fixes 102 faces sharing 6 normals)
  2. weighted normals         (keeps flat panels flat while smoothing transitions)
  3. a small bevel            (every edge catches a highlight, which is what
                               makes a shape read as manufactured metal)

No retopology, no remodelling. Run with:
  blender --background --python robot_polish_demo.py
"""
import os
import sys

import bpy

OUT = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(os.path.dirname(OUT), "Assets", "Models", "long_rover.obj")
RES = 560


def clear():
    bpy.ops.wm.read_factory_settings(use_empty=True)


def import_obj(path):
    if hasattr(bpy.ops.wm, "obj_import"):
        bpy.ops.wm.obj_import(filepath=path)
    else:
        bpy.ops.import_scene.obj(filepath=path)
    objs = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    if not objs:
        raise SystemExit("no mesh imported from " + path)
    # join everything into one object so framing is simple
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    if len(objs) > 1:
        bpy.ops.object.join()
    return bpy.context.view_layer.objects.active


def metal_material(obj):
    mat = bpy.data.materials.new("RobotMetal")
    mat.use_nodes = True
    bsdf = mat.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (0.32, 0.34, 0.38, 1.0)
    bsdf.inputs["Metallic"].default_value = 0.85
    bsdf.inputs["Roughness"].default_value = 0.35
    obj.data.materials.clear()
    obj.data.materials.append(mat)


def polish(obj):
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)

    # 1 + 2: smooth by angle, then weighted normals so flat panels stay flat.
    # Blender 4.1 removed mesh.use_auto_smooth in favour of this operator.
    if hasattr(bpy.ops.object, "shade_auto_smooth"):
        bpy.ops.object.shade_auto_smooth(angle=0.610865)   # 35 degrees
    else:
        bpy.ops.object.shade_smooth()
        obj.data.use_auto_smooth = True
        obj.data.auto_smooth_angle = 0.610865

    wn = obj.modifiers.new("WeightedNormal", "WEIGHTED_NORMAL")
    wn.keep_sharp = True

    # 3: the bevel. Small, two segments, angle limited so only real edges get it.
    bev = obj.modifiers.new("Bevel", "BEVEL")
    bev.width = 0.012
    bev.segments = 2
    bev.limit_method = "ANGLE"
    bev.angle_limit = 0.523599                              # 30 degrees
    bev.harden_normals = True
    # bevel must run before weighted normals
    obj.modifiers.move(obj.modifiers.find("Bevel"), 0)


def setup_scene(obj):
    scene = bpy.context.scene
    scene.render.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in \
        [i.identifier for i in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items] \
        else "BLENDER_EEVEE"
    scene.render.resolution_x = RES
    scene.render.resolution_y = RES
    scene.render.film_transparent = False

    # studio-ish world so metal has something to reflect, which is most of
    # what sells a metal surface
    world = bpy.data.worlds.new("W")
    scene.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.42, 0.46, 0.52, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 1.2

    # frame the object
    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    d = max(obj.dimensions) or 1.0

    import mathutils
    # aim at the mesh's real centre, not its origin, which is often at 0,0,0
    bb = [obj.matrix_world @ mathutils.Vector(v) for v in obj.bound_box]
    centre = sum(bb, mathutils.Vector()) / 8.0
    print("dims=%.2f x %.2f x %.2f  centre=%.2f %.2f %.2f" % (
        obj.dimensions.x, obj.dimensions.y, obj.dimensions.z,
        centre.x, centre.y, centre.z))

    target = bpy.data.objects.new("Target", None)
    target.location = centre
    scene.collection.objects.link(target)

    cam_data = bpy.data.cameras.new("Cam")
    cam = bpy.data.objects.new("Cam", cam_data)
    scene.collection.objects.link(cam)
    # explicit three quarter view so the silhouette reads, rather than staring
    # straight down one face of a boxy mesh
    cam.location = centre + mathutils.Vector((d * 1.9, -d * 2.3, d * 1.4))
    con = cam.constraints.new("TRACK_TO")
    con.target = target
    con.track_axis = "TRACK_NEGATIVE_Z"
    con.up_axis = "UP_Y"
    scene.camera = cam

    key = bpy.data.objects.new("Key", bpy.data.lights.new("Key", "AREA"))
    scene.collection.objects.link(key)
    key.data.energy = 900 * d
    key.data.size = d * 2
    key.location = (centre.x + d * 2, centre.y - d * 2, centre.z + d * 3)
    key.rotation_euler = (0.7, 0.2, 0.7)


def render(path):
    bpy.context.scene.render.filepath = path
    bpy.ops.render.render(write_still=True)


def run(polished, out_name):
    clear()
    obj = import_obj(SRC)
    metal_material(obj)
    if polished:
        polish(obj)
    setup_scene(obj)
    render(os.path.join(OUT, out_name))
    print("faces=%d  polished=%s" % (len(obj.data.polygons), polished))


run(False, "robot_before.png")
run(True, "robot_after.png")
print("DONE")
