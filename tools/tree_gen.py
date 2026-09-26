"""Generate farm trees for Unity in headless Blender.

    blender --background --python tools/tree_gen.py -- [--species Oak,Elm,Cedar]
            [--variants 3] [--preview DIR]

For every species and variant this writes
  Assets/Models/Vegetation/<Species>_<V>.fbx     objects <Species>_<V>_LOD0/1/2
  Assets/Textures/Vegetation/<Species>_<V>_Impostor.png   (raw, run
      tools/finish_impostors.py afterwards to bleed colour into the transparency)
and with --preview a lit beauty render per variant into DIR, for judging the
shape without opening Unity.

Why generated and not downloaded: the CC0 trees available (Poly Haven) are
0.3 to 17 million triangles each with no LODs, and the farm scatters a few
thousand trees. These are a few thousand triangles up close and four in the
distance.

Structure of a tree:
  - trunk and branches are tubes along grown curves, bark UVs in metres
  - leaves are cluster cards (a twig with leaves, see
    build_vegetation_textures.py) hung on the outer branches
  - LOD0: everything; LOD1: coarser tubes, the thinnest branches dropped, 40%
    of the cards at 1.55x size; LOD2: two crossed quads showing an albedo
    render of LOD0 from the front and the side

Vertex colours carry data for FarmSim/Foliage and FarmSim/Bark:
  R = ambient occlusion (inner crown darker)
  G = flexibility for wind (0 at the trunk, 1 at card tips)
  B = a per-card phase
Card normals are bent 70% toward "out of the crown" so a crown lights as a
volume instead of as a pile of flat cards.

Blender is Z-up; the FBX export converts to Unity's Y-up.
"""
import math
import os
import random
import sys

import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
MODELS = os.path.join(REPO, "Assets", "Models", "Vegetation")
TEX = os.path.join(REPO, "Assets", "Textures", "Vegetation")

UP = Vector((0.0, 0.0, 1.0))
BARK_TILE = 1.2          # metres of bark texture along a branch
IMPOSTOR_RES = 512


# ----------------------------------------------------------------------------
# species

SPECIES = {
    # white-oak-like: short trunk, a few heavy limbs spreading wide, round crown
    "Oak": dict(
        leaves="OakLeaves", bark="OakBark",
        height=(11.0, 15.0), trunk_r=(0.30, 0.42), leader=0.8,
        limbs=(6, 8), limb_zone=(0.24, 0.62), limb_angle=(42, 70), limb_len=(0.42, 0.55),
        limb_up=0.14, limb_droop=0.10,
        subs=(7, 10), sub_angle=(35, 60), sub_len=(0.32, 0.5),
        twigs=(3, 5), twig_len=(1.0, 1.8),
        gnarl=0.16, card=(1.0, 1.3), card_step=0.3, crown=(0.52, 0.5), cone=False,
    ),
    # American-elm-like: tall, the trunk splits low into upright leaders that arch
    # out into a vase with drooping tips
    "Elm": dict(
        leaves="ElmLeaves", bark="OakBark",
        height=(15.0, 19.0), trunk_r=(0.30, 0.40), leader=0.34,
        limbs=(3, 4), limb_zone=(0.26, 0.36), limb_angle=(18, 32), limb_len=(0.62, 0.74),
        limb_up=0.04, limb_droop=0.18,
        subs=(11, 14), sub_angle=(38, 62), sub_len=(0.26, 0.42),
        twigs=(3, 4), twig_len=(0.9, 1.5),
        gnarl=0.12, card=(1.0, 1.25), card_step=0.34, crown=(0.40, 0.6), cone=False,
    ),
    # Eastern red cedar: one straight trunk to the top, many short ascending
    # branches from near the ground, dense conical foliage
    "Cedar": dict(
        leaves="CedarFoliage", bark="CedarBark",
        height=(7.0, 10.5), trunk_r=(0.13, 0.19), leader=1.0,
        limbs=(34, 44), limb_zone=(0.05, 0.9), limb_angle=(52, 68), limb_len=(0.22, 0.30),
        limb_up=0.10, limb_droop=0.0,
        subs=(2, 4), sub_angle=(30, 50), sub_len=(0.35, 0.55),
        twigs=(0, 0), twig_len=(0.0, 0.0),
        gnarl=0.10, card=(0.8, 1.0), card_step=0.26, crown=(0.24, 0.5), cone=True,
    ),
}


# ----------------------------------------------------------------------------
# growth

class Branch:
    def __init__(self, pts, radii, level, flex0, flex1):
        self.pts = pts
        self.radii = radii
        self.level = level
        self.flex0 = flex0
        self.flex1 = flex1

    def length(self):
        return sum((self.pts[i + 1] - self.pts[i]).length for i in range(len(self.pts) - 1))

    def at(self, t):
        """Point, direction and radius at fraction t of the length."""
        total = self.length()
        want = t * total
        acc = 0.0
        for i in range(len(self.pts) - 1):
            seg = (self.pts[i + 1] - self.pts[i]).length
            if acc + seg >= want or i == len(self.pts) - 2:
                u = 0.0 if seg < 1e-6 else min(1.0, (want - acc) / seg)
                p = self.pts[i].lerp(self.pts[i + 1], u)
                d = (self.pts[i + 1] - self.pts[i]).normalized()
                r = self.radii[i] + (self.radii[i + 1] - self.radii[i]) * u
                return p, d, r
            acc += seg
        return self.pts[-1], (self.pts[-1] - self.pts[-2]).normalized(), self.radii[-1]


def jitter(rng, s):
    return Vector((rng.gauss(0, s), rng.gauss(0, s), rng.gauss(0, s)))


def grow(rng, start, direction, length, r0, r1, segments, gnarl, up, droop):
    pts, radii = [start.copy()], [r0]
    d = direction.normalized()
    p = start.copy()
    step = length / segments
    for i in range(1, segments + 1):
        f = i / segments
        d = (d + jitter(rng, gnarl) + UP * (up - droop * f)).normalized()
        p = p + d * step
        pts.append(p.copy())
        radii.append(r0 + (r1 - r0) * f)
    return pts, radii


def perpendicular(d, rng):
    a = Vector((rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1)))
    a = a - d * a.dot(d)
    if a.length < 1e-4:
        a = d.orthogonal()
    return a.normalized()


def rotate_about(v, axis, angle_deg):
    return (Matrix.Rotation(math.radians(angle_deg), 3, axis) @ v).normalized()


def build_tree(spec, rng):
    H = rng.uniform(*spec["height"])
    r_trunk = rng.uniform(*spec["trunk_r"])
    branches = []

    lean = Vector((rng.uniform(-0.04, 0.04), rng.uniform(-0.04, 0.04), 1.0)).normalized()
    trunk_len = H * spec["leader"]
    trunk_pts, trunk_r = grow(rng, Vector((0, 0, -0.3)), lean, trunk_len + 0.3,
                              r_trunk, r_trunk * (0.12 if spec["cone"] else 0.35),
                              max(6, int(trunk_len / 0.8)), spec["gnarl"] * 0.35, 0.05, 0.0)
    # root flare: the bottom metre widens
    for i, p in enumerate(trunk_pts):
        z = p.z
        if z < 1.0:
            trunk_r[i] *= 1.0 + 0.45 * (1.0 - max(0.0, z))
    trunk = Branch(trunk_pts, trunk_r, 0, 0.0, 0.08)
    branches.append(trunk)

    n_limbs = rng.randint(*spec["limbs"])
    golden = 137.508
    az0 = rng.uniform(0, 360)
    limbs = []
    for k in range(n_limbs):
        t = spec["limb_zone"][0] + (spec["limb_zone"][1] - spec["limb_zone"][0]) * (k + rng.uniform(0.1, 0.9)) / n_limbs
        p, d, r = trunk.at(min(t / spec["leader"], 0.98) if spec["leader"] < 1 else t)
        az = math.radians(az0 + k * golden + rng.uniform(-18, 18))
        horiz = Vector((math.cos(az), math.sin(az), 0.0))
        tilt = math.radians(rng.uniform(*spec["limb_angle"]))
        direction = (UP * math.cos(tilt) + horiz * math.sin(tilt)).normalized()
        if spec["cone"]:
            # cedar: branch length follows the cone, long at the bottom, short at the top
            height_frac = p.z / H
            length = H * rng.uniform(*spec["limb_len"]) * max(0.12, 1.0 - height_frac) * 1.35
        else:
            length = H * rng.uniform(*spec["limb_len"])
        r0 = min(r * rng.uniform(0.5, 0.65), r_trunk * 0.55)
        pts, radii = grow(rng, p, direction, length, r0, r0 * 0.18, max(5, int(length / 0.7)),
                          spec["gnarl"], spec["limb_up"], spec["limb_droop"])
        limb = Branch(pts, radii, 1, 0.08, 0.45)
        limbs.append(limb)
        branches.append(limb)

    subs = []
    # broadleaf leaders carry side branches too, or the top of the crown is empty
    parents = limbs + ([] if spec["cone"] else [trunk])
    for limb in parents:
        n_sub = rng.randint(*spec["subs"])
        L = limb.length()
        lo_t = 0.22 if limb is not trunk else spec["limb_zone"][0] / spec["leader"]
        for k in range(n_sub):
            t = lo_t + (1.0 - lo_t) * (k + rng.uniform(0.0, 0.8)) / n_sub
            p, d, r = limb.at(t)
            side = perpendicular(d, rng)
            direction = rotate_about(d, side, rng.uniform(*spec["sub_angle"]))
            # a little upward preference: outer branches reach for light
            direction = (direction + UP * 0.25).normalized()
            # the leader is much longer than a limb; its side branches are not
            reach = L if limb is not trunk else L * 0.45
            length = reach * rng.uniform(*spec["sub_len"]) * (1.0 - 0.45 * t)
            r0 = max(0.012, r * rng.uniform(0.45, 0.65))
            pts, radii = grow(rng, p, direction, length, r0, 0.006, max(3, int(length / 0.5)),
                              spec["gnarl"] * 1.3, 0.08, spec["limb_droop"] * 0.8)
            sub = Branch(pts, radii, 2, 0.45, 0.8)
            subs.append(sub)
            branches.append(sub)

    twigs = []
    lo, hi = spec["twigs"]
    for sub in subs:
        if hi <= 0:
            break
        for k in range(rng.randint(lo, hi)):
            t = 0.35 + 0.6 * (k + rng.uniform(0, 0.8)) / max(1, hi)
            p, d, r = sub.at(min(t, 0.95))
            direction = rotate_about(d, perpendicular(d, rng), rng.uniform(30, 55))
            length = rng.uniform(*spec["twig_len"])
            pts, radii = grow(rng, p, direction, length, max(0.008, r * 0.6), 0.004, 3,
                              spec["gnarl"] * 1.5, 0.1, 0.05)
            tw = Branch(pts, radii, 3, 0.8, 0.95)
            twigs.append(tw)
            branches.append(tw)

    # leaf cards along the outer branches, and on the outer part of every limb
    # so no limb ends in a bare stick
    carriers = (twigs + subs) if twigs else (subs + limbs)
    if twigs:
        carriers = carriers + limbs
    cards = []
    for br in carriers:
        L = br.length()
        start = 0.25 if br.level >= 2 else (0.35 if not twigs else 0.7)
        n = max(1, int(L * (1.0 - start) / spec["card_step"]))
        for k in range(n + 1):
            t = start + (1.0 - start) * k / max(1, n)
            p, d, r = br.at(min(t, 1.0))
            cards.append((p, d, br.flex0 + (br.flex1 - br.flex0) * t))
    if spec["cone"]:
        # cedars are clothed right up the trunk; add cards around the stem
        for k in range(int(H * 14)):
            z = rng.uniform(0.6, H * 0.97)
            p, d, r = trunk.at(min(z / H, 1.0))
            out = Vector((rng.uniform(-1, 1), rng.uniform(-1, 1), 0.0)).normalized()
            cards.append((p + out * 0.1, (out + UP * 0.9).normalized(), 0.3))

    crown_shape(spec, H, branches, cards)
    return H, branches, cards


def crown_shape(spec, H, branches, cards):
    """Trim cards far outside the intended crown envelope, so every variant keeps
    a believable silhouette whatever the random growth did."""
    rx, zc = spec["crown"]
    if spec["cone"]:
        keep = []
        for c in cards:
            p = c[0]
            allowed = H * rx * max(0.05, 1.0 - p.z / (H * 1.02)) + 0.4
            if math.hypot(p.x, p.y) <= allowed * 1.25:
                keep.append(c)
        cards[:] = keep
        return
    centre = Vector((0, 0, H * zc + H * 0.2))
    radius_xy = H * rx
    radius_z = H * 0.42
    keep = []
    for c in cards:
        d = c[0] - centre
        e = (d.x / radius_xy) ** 2 + (d.y / radius_xy) ** 2 + (d.z / radius_z) ** 2
        if e <= 1.35:
            keep.append(c)
    cards[:] = keep


# ----------------------------------------------------------------------------
# mesh building

class MeshData:
    def __init__(self):
        self.v, self.n, self.uv, self.col, self.faces, self.mat = [], [], [], [], [], []
        self.loop_uv = []

    def add_tri(self, a, b, c, mat, uvs):
        self.faces.append((a, b, c))
        self.mat.append(mat)
        self.loop_uv.extend(uvs)


def add_tube(md, br, sides, mat, ao_fn, phase):
    pts, radii = br.pts, br.radii
    if len(pts) < 2:
        return
    # parallel-transport frames, so the tube does not twist
    t_prev = (pts[1] - pts[0]).normalized()
    n_prev = t_prev.orthogonal().normalized()
    frames = []
    for i in range(len(pts)):
        if i == 0:
            t = t_prev
        elif i == len(pts) - 1:
            t = (pts[i] - pts[i - 1]).normalized()
        else:
            t = ((pts[i] - pts[i - 1]).normalized() + (pts[i + 1] - pts[i]).normalized()).normalized()
        if i > 0:
            axis = t_prev.cross(t)
            if axis.length > 1e-6:
                ang = math.acos(max(-1.0, min(1.0, t_prev.dot(t))))
                n_prev = (Matrix.Rotation(ang, 3, axis.normalized()) @ n_prev).normalized()
        b = t.cross(n_prev).normalized()
        frames.append((t, n_prev.copy(), b))
        t_prev = t

    around = 2 if br.level == 0 else 1
    base = len(md.v)
    vlen = 0.0
    ring_v = []
    for i, p in enumerate(pts):
        if i > 0:
            vlen += (pts[i] - pts[i - 1]).length
        t, n, b = frames[i]
        f = i / (len(pts) - 1)
        flex = br.flex0 + (br.flex1 - br.flex0) * f
        ring_v.append(vlen / BARK_TILE)
        for j in range(sides):
            a = 2.0 * math.pi * j / sides
            off = n * math.cos(a) + b * math.sin(a)
            md.v.append(p + off * radii[i])
            md.n.append(off)
            md.col.append((ao_fn(p), flex, phase, 1.0))
    for i in range(len(pts) - 1):
        for j in range(sides):
            j2 = (j + 1) % sides
            a = base + i * sides + j
            b_ = base + i * sides + j2
            c = base + (i + 1) * sides + j2
            d = base + (i + 1) * sides + j
            u0 = around * j / sides
            u1 = around * (j + 1) / sides
            v0, v1 = ring_v[i], ring_v[i + 1]
            md.add_tri(a, b_, c, mat, [(u0, v0), (u1, v0), (u1, v1)])
            md.add_tri(a, c, d, mat, [(u0, v0), (u1, v1), (u0, v1)])


def add_card(md, rng, p, d, flex, size, mat, crown_centre, ao, cell):
    # the card rises roughly along the branch, turned outward and upward
    out = Vector((p.x - crown_centre.x, p.y - crown_centre.y, 0.0))
    out = out.normalized() if out.length > 1e-4 else Vector((1, 0, 0))
    up_axis = (d + out * 0.45 + UP * 0.35 + jitter(rng, 0.35)).normalized()
    normal = perpendicular(up_axis, rng)
    side = up_axis.cross(normal).normalized()
    base = p - up_axis * (size * 0.06)
    corners = [base - side * size * 0.5, base + side * size * 0.5,
               base + side * size * 0.5 + up_axis * size, base - side * size * 0.5 + up_axis * size]
    col, row = cell % 2, cell // 2
    u0, u1 = 0.5 * col, 0.5 * col + 0.5
    v0, v1 = 1.0 - 0.5 * (row + 1), 1.0 - 0.5 * row
    uvs = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
    phase = rng.random()
    i0 = len(md.v)
    for k, c in enumerate(corners):
        radial = (c - crown_centre)
        radial = radial.normalized() if radial.length > 1e-4 else UP
        # 70% "out of the crown": the crown lights as a volume
        nrm = (normal * 0.3 + radial * 0.7).normalized()
        md.v.append(c)
        md.n.append(nrm)
        tip = 1.0 if k >= 2 else flex
        md.col.append((ao, tip, phase, 1.0))
    md.add_tri(i0, i0 + 1, i0 + 2, mat, [uvs[0], uvs[1], uvs[2]])
    md.add_tri(i0, i0 + 2, i0 + 3, mat, [uvs[0], uvs[2], uvs[3]])


def crown_ao_fn(spec, H, cards):
    if not cards:
        return (lambda p: 1.0), Vector((0, 0, H * 0.6))
    cx = sum(c[0].x for c in cards) / len(cards)
    cy = sum(c[0].y for c in cards) / len(cards)
    cz = sum(c[0].z for c in cards) / len(cards)
    centre = Vector((cx, cy, cz))
    rmax = max((c[0] - centre).length for c in cards) or 1.0

    def ao(p):
        d = min(1.0, (p - centre).length / rmax)
        s = max(0.0, min(1.0, (d - 0.2) / 0.8))
        s = s * s * (3 - 2 * s)
        height = max(0.0, min(1.0, p.z / H))
        return max(0.25, min(1.0, (0.38 + 0.62 * s) * (0.82 + 0.18 * height)))
    return ao, centre


def build_mesh_data(spec, rng, H, branches, cards, lod):
    md = MeshData()
    ao_fn, centre = crown_ao_fn(spec, H, cards)
    sides_by_level = {0: (12, 7), 1: (8, 5), 2: (5, 3), 3: (3, 0)}
    for br in branches:
        sides = sides_by_level[br.level][lod]
        if sides < 3:
            continue
        add_tube(md, br, sides, 0, lambda p, f=ao_fn: 0.55 + 0.45 * f(p), rng.random() * 0.2)
    size_lo, size_hi = spec["card"]
    for i, (p, d, flex) in enumerate(cards):
        if lod == 1 and i % 5 not in (0, 2):
            continue
        size = rng.uniform(size_lo, size_hi) * (1.55 if lod == 1 else 1.0)
        add_card(md, rng, p, d, flex, size, 1, centre, ao_fn(p), rng.randrange(4))
    return md, centre


def make_object(name, md, materials):
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata([tuple(v) for v in md.v], [], md.faces)
    mesh.update()
    for m in materials:
        mesh.materials.append(m)
    mesh.polygons.foreach_set("material_index", md.mat)
    mesh.polygons.foreach_set("use_smooth", [True] * len(md.faces))
    uv = mesh.uv_layers.new(name="UVMap")
    flat = [c for pair in md.loop_uv for c in pair]
    uv.data.foreach_set("uv", flat)
    col = mesh.color_attributes.new(name="Col", type="FLOAT_COLOR", domain="POINT")
    col.data.foreach_set("color", [c for rgba in md.col for c in rgba])
    mesh.color_attributes.active_color = col
    mesh.normals_split_custom_set_from_vertices([tuple(n) for n in md.n])
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    return obj


# ----------------------------------------------------------------------------
# materials (Blender side: only for the impostor and preview renders; Unity
# gets its own materials by name)

def image(name):
    path = os.path.join(TEX, name + ".png")
    img = bpy.data.images.get(os.path.basename(path)) or bpy.data.images.load(path)
    return img


def material(name, tex, leaf, emission_only):
    m = bpy.data.materials.get(name + ("_emit" if emission_only else ""))
    if m:
        return m
    m = bpy.data.materials.new(name + ("_emit" if emission_only else ""))
    m.use_nodes = True
    nt = m.node_tree
    for n in list(nt.nodes):
        nt.nodes.remove(n)
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    tx = nt.nodes.new("ShaderNodeTexImage")
    tx.image = image(tex + "_Albedo")
    vc = nt.nodes.new("ShaderNodeVertexColor")
    vc.layer_name = "Col"
    sep = nt.nodes.new("ShaderNodeSeparateColor")
    nt.links.new(vc.outputs["Color"], sep.inputs["Color"])
    mul = nt.nodes.new("ShaderNodeMixRGB")
    mul.blend_type = "MULTIPLY"
    mul.inputs["Fac"].default_value = 1.0
    nt.links.new(tx.outputs["Color"], mul.inputs[1])
    nt.links.new(sep.outputs["Red"], mul.inputs[2])
    if emission_only:
        sh = nt.nodes.new("ShaderNodeEmission")
        nt.links.new(mul.outputs["Color"], sh.inputs["Color"])
    else:
        sh = nt.nodes.new("ShaderNodeBsdfPrincipled")
        nt.links.new(mul.outputs["Color"], sh.inputs["Base Color"])
        sh.inputs["Roughness"].default_value = 0.7
    if leaf:
        # hard alpha test at 0.35 like the Unity material
        gt = nt.nodes.new("ShaderNodeMath")
        gt.operation = "GREATER_THAN"
        gt.inputs[1].default_value = 0.35
        nt.links.new(tx.outputs["Alpha"], gt.inputs[0])
        tr = nt.nodes.new("ShaderNodeBsdfTransparent")
        mix = nt.nodes.new("ShaderNodeMixShader")
        nt.links.new(gt.outputs[0], mix.inputs["Fac"])
        nt.links.new(tr.outputs[0], mix.inputs[1])
        nt.links.new(sh.outputs[0], mix.inputs[2])
        nt.links.new(mix.outputs[0], out.inputs["Surface"])
        if hasattr(m, "surface_render_method"):
            m.surface_render_method = "DITHERED"
        if hasattr(m, "blend_method"):
            m.blend_method = "CLIP"
        m.use_backface_culling = False
    else:
        nt.links.new(sh.outputs[0], out.inputs["Surface"])
    return m


def set_materials(obj, mats):
    # assign slot by slot: materials.clear() resets every polygon's material
    # index to 0, which silently put bark on all the leaf cards
    for i, m in enumerate(mats):
        obj.data.materials[i] = m


# ----------------------------------------------------------------------------
# impostor

def render_setup(res_x, res_y, transparent):
    sc = bpy.context.scene
    engines = [e.identifier for e in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items]
    sc.render.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in engines else "BLENDER_EEVEE"
    sc.render.resolution_x = res_x
    sc.render.resolution_y = res_y
    sc.render.resolution_percentage = 100
    sc.render.film_transparent = transparent
    sc.render.image_settings.file_format = "PNG"
    sc.render.image_settings.color_mode = "RGBA"
    sc.view_settings.view_transform = "Standard"
    sc.view_settings.look = "None"
    if hasattr(sc, "eevee"):
        if hasattr(sc.eevee, "taa_render_samples"):
            sc.eevee.taa_render_samples = 16


def render_impostor(tree_obj, H, width, path):
    """Albedo x AO of LOD0 from the front (looking along +Y) and the side
    (looking along -X), side by side in one 2:1 image."""
    sc = bpy.context.scene
    render_setup(IMPOSTOR_RES, IMPOSTOR_RES, True)
    world = bpy.data.worlds.new("black")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.0
    sc.world = world

    size = max(H, width) * 1.04
    centre_z = size / 2.0 - 0.2
    cam_data = bpy.data.cameras.new("imp")
    cam_data.type = "ORTHO"
    cam_data.ortho_scale = size
    cam = bpy.data.objects.new("imp", cam_data)
    sc.collection.objects.link(cam)
    sc.camera = cam

    shots = []
    for k, (loc, rot) in enumerate([
            (Vector((0, -60, centre_z)), (math.radians(90), 0, 0)),
            (Vector((60, 0, centre_z)), (math.radians(90), 0, math.radians(90)))]):
        cam.location = loc
        cam.rotation_euler = rot
        p = path.replace(".png", "_view%d.png" % k)
        sc.render.filepath = p
        bpy.ops.render.render(write_still=True)
        shots.append(p)

    # stitch the two views: front on the left, side on the right
    a = bpy.data.images.load(shots[0])
    b = bpy.data.images.load(shots[1])
    w, h = a.size
    out = bpy.data.images.new("impostor", w * 2, h, alpha=True)
    pa, pb = list(a.pixels), list(b.pixels)
    px = [0.0] * (w * 2 * h * 4)
    for y in range(h):
        row_a = pa[y * w * 4:(y + 1) * w * 4]
        row_b = pb[y * w * 4:(y + 1) * w * 4]
        px[y * w * 8:y * w * 8 + w * 4] = row_a
        px[y * w * 8 + w * 4:(y + 1) * w * 8] = row_b
    out.pixels = px
    out.filepath_raw = path
    out.file_format = "PNG"
    out.save()
    for p in shots:
        os.remove(p)
    bpy.data.objects.remove(cam)
    return size, centre_z


def impostor_mesh(name, size, centre_z, mat_index):
    md = MeshData()
    half = size / 2.0
    z0, z1 = centre_z - half, centre_z + half
    # quad A: the XZ plane, seen along +Y, shows the front view (left half)
    # quad B: the YZ plane, seen along -X, shows the side view (right half)
    quads = [
        ([Vector((-half, 0, z0)), Vector((half, 0, z0)), Vector((half, 0, z1)), Vector((-half, 0, z1))], 0.0),
        ([Vector((0, -half, z0)), Vector((0, half, z0)), Vector((0, half, z1)), Vector((0, -half, z1))], 0.5),
    ]
    for corners, u_off in quads:
        i0 = len(md.v)
        for c in corners:
            radial = Vector((c.x, c.y, (c.z - (z0 + (z1 - z0) * 0.55)) * 0.6))
            nrm = (radial.normalized() * 0.7 + UP * 0.3).normalized()
            md.v.append(c)
            md.n.append(nrm)
            h = (c.z - z0) / (z1 - z0)
            md.col.append((1.0, 0.35 * h, 0.0, 1.0))
        uvs = [(u_off, 0.0), (u_off + 0.5, 0.0), (u_off + 0.5, 1.0), (u_off, 1.0)]
        md.add_tri(i0, i0 + 1, i0 + 2, mat_index, [uvs[0], uvs[1], uvs[2]])
        md.add_tri(i0, i0 + 2, i0 + 3, mat_index, [uvs[0], uvs[2], uvs[3]])
    return md


# ----------------------------------------------------------------------------
# preview

def render_preview(objs, H, path):
    sc = bpy.context.scene
    render_setup(640, 640, False)
    world = bpy.data.worlds.new("sky")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.42, 0.55, 0.78, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.9
    sc.world = world
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.5
    sun.rotation_euler = (math.radians(45), 0, math.radians(35))
    sc.collection.objects.link(sun)
    ground = bpy.data.meshes.new("ground")
    ground.from_pydata([(-40, -40, 0), (40, -40, 0), (40, 40, 0), (-40, 40, 0)], [], [(0, 1, 2, 3)])
    gobj = bpy.data.objects.new("ground", ground)
    gm = bpy.data.materials.new("groundmat")
    gm.use_nodes = True
    gm.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.08, 0.1, 0.04, 1)
    ground.materials.append(gm)
    sc.collection.objects.link(gobj)
    cam_data = bpy.data.cameras.new("pv")
    cam = bpy.data.objects.new("pv", cam_data)
    sc.collection.objects.link(cam)
    sc.camera = cam
    cam_data.lens = 35
    cam.location = Vector((H * 1.25, -H * 1.55, H * 0.55))
    target = Vector((0, 0, H * 0.48))
    cam.rotation_euler = (target - cam.location).to_track_quat("-Z", "Y").to_euler()
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)
    for o in (sun, gobj, cam):
        bpy.data.objects.remove(o)


# ----------------------------------------------------------------------------

def export_fbx(objs, path):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    bpy.context.view_layer.objects.active = objs[0]
    kwargs = dict(filepath=path, use_selection=True, object_types={"MESH"},
                  axis_forward="-Z", axis_up="Y", bake_space_transform=True,
                  apply_scale_options="FBX_SCALE_ALL", mesh_smooth_type="OFF",
                  use_mesh_modifiers=False, add_leaf_bones=False, path_mode="STRIP",
                  use_tspace=False)
    try:
        bpy.ops.export_scene.fbx(colors_type="LINEAR", **kwargs)
    except TypeError:
        bpy.ops.export_scene.fbx(**kwargs)


def generate(species, variant, preview_dir):
    spec = SPECIES[species]
    # a stable seed: hash() of a str is salted per process, so reruns would
    # produce different trees
    rng = random.Random(sum(ord(c) * (i + 1) for i, c in enumerate(species)) * 1000 + variant * 7919 + 17)
    H, branches, cards = build_tree(spec, rng)
    name = "%s_%s" % (species, "ABCDEFGH"[variant])

    bark_m = material(species + "Bark", spec["bark"], False, False)
    leaf_m = material(species + "Leaves", spec["leaves"], True, False)
    objs = []
    stats = []
    for lod in (0, 1):
        md, centre = build_mesh_data(spec, random.Random(rng.random()), H, branches, cards, lod)
        obj = make_object("%s_LOD%d" % (name, lod), md, [bark_m, leaf_m])
        objs.append(obj)
        stats.append(len(md.faces))

    width = 2.0 * max(max(abs(c[0].x), abs(c[0].y)) for c in cards) + 1.2 if cards else H * 0.5
    imp_path = os.path.join(TEX, name + "_Impostor.png")
    # albedo render: emission-only copies of the materials on LOD0
    objs[1].hide_render = True
    set_materials(objs[0], [material(species + "Bark", spec["bark"], False, True),
                            material(species + "Leaves", spec["leaves"], True, True)])
    size, centre_z = render_impostor(objs[0], H, width, imp_path)
    set_materials(objs[0], [bark_m, leaf_m])
    set_materials(objs[1], [bark_m, leaf_m])
    objs[1].hide_render = False

    imp_m = bpy.data.materials.new(name + "_Impostor")
    md2 = impostor_mesh(name + "_LOD2", size, centre_z, 0)
    objs.append(make_object(name + "_LOD2", md2, [imp_m]))
    stats.append(len(md2.faces))

    if preview_dir:
        objs[1].hide_render = True
        objs[2].hide_render = True
        render_preview(objs, H, os.path.join(preview_dir, name + "_preview.png"))
        objs[1].hide_render = False
        objs[2].hide_render = False

    export_fbx(objs, os.path.join(MODELS, name + ".fbx"))
    print("TREE %s height %.1f m, %d branches, %d cards, triangles LOD0 %d LOD1 %d LOD2 %d, impostor %.1f m" % (
        name, H, len(branches), len(cards), stats[0], stats[1], stats[2], size))
    for o in objs:
        bpy.data.objects.remove(o)


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    species = list(SPECIES)
    variants = 3
    preview = None
    for i, a in enumerate(argv):
        if a == "--species":
            species = argv[i + 1].split(",")
        elif a == "--variants":
            variants = int(argv[i + 1])
        elif a == "--preview":
            preview = argv[i + 1]
    os.makedirs(MODELS, exist_ok=True)
    if preview:
        os.makedirs(preview, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    for s in species:
        for v in range(variants):
            generate(s, v, preview)
    print("DONE")


main()
