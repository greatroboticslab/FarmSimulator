"""Bevel, weighted normals and UVs for every robot mesh, in headless Blender.

    blender --background --python tools/robot_polish.py

The robots stay the lab's machines: same silhouettes, same parts, same names.
For each source model in Assets/Models this writes a polished copy to
Assets/Models/Robots/<name>_polished.<obj|fbx>; the originals are untouched,
and RobotPolishSetup swaps the prefabs' MeshFilters over (colliders keep the
original meshes, so physics does not change).

Per mesh, the recipe proven on long_rover.obj (tools/robot_polish_demo.py):
  1. weld coincident vertices, smooth by angle (35 deg, sharp edges kept)
  2. bevel: 0.012 m, 2 segments, only edges sharper than 30 deg, hardened
     normals; capped at 6% of the part's smallest dimension so thin parts (the
     snipper blade is 1 cm) keep their shape
  3. weighted normals (face area, keep sharp) after the bevel
  4. UVs: Smart UV Project where the mesh has none (jaguar_wheel, basket,
     arm_basket, basketRoverSuspension, the arms, uv_body, uv_cone,
     swivel_wheel); meshes that have UVs keep them

Coordinates: OBJ sources go back out through Blender's OBJ exporter with the
importer's default axes, which round-trips exactly. FBX sources were written by
Blender 2.79's FBX exporter at its defaults and go back out with the same
settings (no baked space transform, unit scale applied), so Unity sees the same
local mesh space. RobotPolishSetup compares bounds before swapping anyway.
"""
import math
import os

import bmesh
import bpy
from mathutils import Vector

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SRC = os.path.join(REPO, "Assets", "Models")
OUT = os.path.join(REPO, "Assets", "Models", "Robots")

SOURCES = [
    "jaguar_wheel.obj", "jaguar_body.obj", "jaguar_body.fbx", "long_rover.obj",
    "basketroverbody.obj", "basketRoverSuspension.obj", "basket.obj", "arm_basket.obj",
    "robot_arm_snipper.fbx", "robot_arm.fbx", "lamp_rover.fbx", "swivel_wheel.fbx",
    "uv_body.obj", "uv_cone.obj",
]

BEVEL_WIDTH = 0.012
BEVEL_CAP = 0.06          # fraction of the smallest dimension
SMOOTH_ANGLE = math.radians(35)
BEVEL_ANGLE = math.radians(30)


def tri_count(me):
    return sum(len(p.vertices) - 2 for p in me.polygons)


def polish(obj):
    me = obj.data
    had_uv = len(me.uv_layers) > 0
    before = tri_count(me)
    if me.shape_keys is not None:
        # RobotArm.cs drives blend shape 0 on these (the snipper opening); a
        # modifier cannot be applied through shape keys, so the part stays as is
        return before, before, 0.0, had_uv

    bm = bmesh.new()
    bm.from_mesh(me)
    bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=1e-5)
    bm.to_mesh(me)
    bm.free()

    bpy.ops.object.select_all(action="DESELECT")
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    if me.has_custom_normals:
        bpy.ops.mesh.customdata_custom_splitnormals_clear()
    bpy.ops.object.shade_smooth_by_angle(angle=SMOOTH_ANGLE, keep_sharp_edges=True)

    # Size the bevel in world metres, then convert: the FBX parts carry object
    # scales (and parent scales) from Blender 2.79's unit handling, and the
    # bevel modifier works in mesh units.
    sc = obj.matrix_world.to_scale()
    mean_scale = (abs(sc.x) + abs(sc.y) + abs(sc.z)) / 3.0 or 1.0
    corners = [obj.matrix_world @ Vector(c) for c in obj.bound_box]
    extent = [max(c[i] for c in corners) - min(c[i] for c in corners) for i in range(3)]
    world_dims = [d for d in extent if d > 1e-5]
    smallest = min(world_dims) if world_dims else 1e-3
    width = min(BEVEL_WIDTH, smallest * BEVEL_CAP)

    bev = obj.modifiers.new("Bevel", "BEVEL")
    bev.width = width / mean_scale
    bev.segments = 2
    bev.limit_method = "ANGLE"
    bev.angle_limit = BEVEL_ANGLE
    bev.harden_normals = True
    wn = obj.modifiers.new("WeightedNormal", "WEIGHTED_NORMAL")
    wn.keep_sharp = True
    wn.mode = "FACE_AREA"
    for m in list(obj.modifiers):
        bpy.ops.object.modifier_apply(modifier=m.name)

    if not had_uv:
        bpy.ops.object.mode_set(mode="EDIT")
        bpy.ops.mesh.select_all(action="SELECT")
        bpy.ops.uv.smart_project(angle_limit=math.radians(66), island_margin=0.02, scale_to_bounds=False)
        bpy.ops.object.mode_set(mode="OBJECT")

    return before, tri_count(me), width, had_uv


def run(name):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    path = os.path.join(SRC, name)
    base, ext = os.path.splitext(name)
    if ext == ".obj":
        bpy.ops.wm.obj_import(filepath=path)
    else:
        bpy.ops.import_scene.fbx(filepath=path)

    report = []
    for obj in [o for o in bpy.context.scene.objects if o.type == "MESH"]:
        keyed = obj.data.shape_keys is not None
        before, after, width, had_uv = polish(obj)
        if keyed:
            report.append("%s left as is (shape keys drive it)" % obj.name)
        else:
            report.append("%s %d->%d tris, bevel %.4f m, uv %s" % (obj.name, before, after, width,
                                                                    "kept" if had_uv else "new"))

    out = os.path.join(OUT, base + "_polished" + ext)
    bpy.ops.object.select_all(action="SELECT")
    if ext == ".obj":
        bpy.ops.wm.obj_export(filepath=out, export_selected_objects=False, export_uv=True,
                              export_normals=True, export_materials=True, export_triangulated_mesh=False,
                              apply_modifiers=True, path_mode="STRIP")
        # Some prefabs render a multi-material OBJ through a single-submesh mesh
        # (jaguar_body.obj imports that way in some of them). Also write a copy
        # with every face on one material; RobotPolishSetup picks whichever
        # copy has the submesh count the prefab's mesh has.
        multi = [o for o in bpy.context.scene.objects if o.type == "MESH" and len(o.data.materials) > 1]
        if multi:
            for o in multi:
                o.data.polygons.foreach_set("material_index", [0] * len(o.data.polygons))
                while len(o.data.materials) > 1:
                    o.data.materials.pop()
            bpy.ops.wm.obj_export(filepath=os.path.join(OUT, base + "_polished_single" + ext),
                                  export_selected_objects=False, export_uv=True, export_normals=True,
                                  export_materials=True, export_triangulated_mesh=False,
                                  apply_modifiers=True, path_mode="STRIP")
    else:
        bpy.ops.export_scene.fbx(filepath=out, use_selection=False, object_types={"MESH", "EMPTY"},
                                 axis_forward="-Z", axis_up="Y", bake_space_transform=False,
                                 apply_unit_scale=True, apply_scale_options="FBX_SCALE_NONE",
                                 mesh_smooth_type="OFF", use_mesh_modifiers=True, add_leaf_bones=False,
                                 path_mode="STRIP", use_tspace=False)
    print("POLISHED %s -> %s: %s" % (name, os.path.basename(out), "; ".join(report)))


def main():
    os.makedirs(OUT, exist_ok=True)
    for name in SOURCES:
        run(name)
    print("DONE")


main()
