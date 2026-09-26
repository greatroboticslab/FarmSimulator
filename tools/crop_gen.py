"""Generate crop plants for the plots, in headless Blender.

    blender --background --python tools/crop_gen.py

Writes Assets/Models/Crops/Soybean.fbx with two meshes, "Soybean" (the plant at
pod fill, R5-R6) and "SoybeanDead" (senesced), textured from the atlases
tools/build_crop_textures.py makes. Same footprint as the soybean.obj they
replace (0.66 m tall, base at 0, Y up in Unity), so plot spacing, colliders and
the Plant prefab's layout do not change.

A soybean plant as it grows in a row: one main stem that zig-zags slightly at
each node; alternate trifoliate leaves on long petioles (a terminal leaflet and
two laterals, ovate, 7 to 11 cm); two side branches from the lower nodes; pods
held in small clusters at the middle nodes. Leaflets are cards folded along
the midrib, so they catch light like a real leaf rather than a flat sheet.
About 750 triangles, fewer than the 600-face (mostly quad) original.

Vertex colour, as for the trees: R ambient occlusion, G wind flexibility, B
phase.
"""
import math
import os
import random

import bpy
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
OUT = os.path.join(REPO, "Assets", "Models", "Crops")
UP = Vector((0, 0, 1))

# atlas layout (see build_crop_textures.py): cells 0-2 leaflets, cell 3 pods + stem
LEAF_CELLS = [(0.0, 0.5, 0.5, 1.0), (0.5, 1.0, 0.5, 1.0), (0.0, 0.5, 0.0, 0.5)]   # u0 u1 v0 v1
POD_UV = [(0.545, 0.655, 0.03, 0.47), (0.705, 0.815, 0.03, 0.47)]
STEM_UV = (0.94, 0.995, 0.0, 0.5)


class Mesh:
    def __init__(self):
        self.v, self.n, self.col, self.faces, self.uv = [], [], [], [], []

    def vert(self, p, n, col):
        self.v.append(p.copy())
        self.n.append(n.normalized())
        self.col.append(col)
        return len(self.v) - 1

    def tri(self, a, b, c, uvs):
        self.faces.append((a, b, c))
        self.uv.extend(uvs)


def jitter(rng, s):
    return Vector((rng.gauss(0, s), rng.gauss(0, s), rng.gauss(0, s)))


def tube(m, pts, r0, r1, sides, flex0, flex1, ao, phase, uv=None, v_len=0.6):
    """A thin tube along pts; the strip's UVs run over v_len metres."""
    if len(pts) < 2:
        return
    u0, u1, v0, v1 = uv or STEM_UV
    t_prev = (pts[1] - pts[0]).normalized()
    n_prev = t_prev.orthogonal().normalized()
    rings = []
    length = 0.0
    for i, p in enumerate(pts):
        if i > 0:
            length += (pts[i] - pts[i - 1]).length
            t = (pts[i] - pts[i - 1]).normalized()
            axis = t_prev.cross(t)
            if axis.length > 1e-6:
                ang = math.acos(max(-1, min(1, t_prev.dot(t))))
                n_prev = (Matrix.Rotation(ang, 3, axis.normalized()) @ n_prev).normalized()
            t_prev = t
        b = t_prev.cross(n_prev).normalized()
        f = i / (len(pts) - 1)
        r = r0 + (r1 - r0) * f
        ring = []
        for j in range(sides):
            a = 2 * math.pi * j / sides
            off = n_prev * math.cos(a) + b * math.sin(a)
            ring.append(m.vert(p + off * r, off, (ao, flex0 + (flex1 - flex0) * f, phase, 1)))
        rings.append((ring, v0 + (v1 - v0) * min(1.0, length / v_len)))
    for i in range(len(rings) - 1):
        (ra, va), (rb, vb) = rings[i], rings[i + 1]
        for j in range(sides):
            k = (j + 1) % sides
            ua = u0 + (u1 - u0) * j / sides
            ub = u0 + (u1 - u0) * (j + 1) / sides
            m.tri(ra[j], ra[k], rb[k], [(ua, va), (ub, va), (ub, vb)])
            m.tri(ra[j], rb[k], rb[j], [(ua, va), (ub, vb), (ua, vb)])


def grow(start, direction, length, segments, bend, rng, wobble=0.04):
    pts = [start.copy()]
    d = direction.normalized()
    p = start.copy()
    for i in range(segments):
        d = (d + bend / segments + jitter(rng, wobble)).normalized()
        p = p + d * (length / segments)
        pts.append(p.copy())
    return pts


def leaflet(m, base, direction, normal_hint, length, fold, cell, flex0, ao, phase):
    """A leaflet card folded along its midrib: 6 vertices, 4 triangles."""
    u0, u1, v0, v1 = LEAF_CELLS[cell]
    um = (u0 + u1) / 2
    fwd = direction.normalized()
    side = fwd.cross(normal_hint).normalized()
    if side.length < 1e-4:
        side = fwd.orthogonal().normalized()
    nrm = side.cross(fwd).normalized()
    if nrm.dot(UP) < 0:
        nrm = -nrm
        side = -side
    h = length / 0.92                       # the leaf fills 92% of its cell
    w = h                                   # cells are square
    lift = nrm * (w / 2) * math.sin(fold)   # edges rise: a shallow V
    inward = side * (w / 2) * (1 - math.cos(fold))
    b = base - fwd * (h * 0.02)
    tip = b + fwd * h
    soft = (nrm * 0.7 + UP * 0.3).normalized()
    col_b = (ao, flex0, phase, 1)
    col_t = (ao, 1.0, phase, 1)
    bm = m.vert(b, soft, col_b)
    tm = m.vert(tip, soft, col_t)
    bl = m.vert(b - side * (w / 2) + inward + lift, (soft - side * 0.35), col_b)
    tl = m.vert(tip - side * (w / 2) + inward + lift, (soft - side * 0.35), col_t)
    br = m.vert(b + side * (w / 2) - inward + lift, (soft + side * 0.35), col_b)
    tr = m.vert(tip + side * (w / 2) - inward + lift, (soft + side * 0.35), col_t)
    m.tri(bl, bm, tm, [(u0, v0), (um, v0), (um, v1)])
    m.tri(bl, tm, tl, [(u0, v0), (um, v1), (u0, v1)])
    m.tri(bm, br, tr, [(um, v0), (u1, v0), (u1, v1)])
    m.tri(bm, tr, tm, [(um, v0), (u1, v1), (um, v1)])


def pod(m, base, direction, rng, ao, phase):
    """Crossed quads showing a pod sprite."""
    length = rng.uniform(0.042, 0.055)
    width = length * 0.26
    fwd = direction.normalized()
    s1 = fwd.orthogonal().normalized()
    s2 = fwd.cross(s1).normalized()
    u0, u1, v0, v1 = POD_UV[rng.randrange(2)]
    for s in (s1, s2):
        n = s.cross(fwd).normalized()
        a = m.vert(base - s * width / 2, n, (ao, 0.35, phase, 1))
        b = m.vert(base + s * width / 2, n, (ao, 0.35, phase, 1))
        c = m.vert(base + s * width / 2 + fwd * length, n, (ao, 0.5, phase, 1))
        d = m.vert(base - s * width / 2 + fwd * length, n, (ao, 0.5, phase, 1))
        m.tri(a, b, c, [(u0, v0), (u1, v0), (u1, v1)])
        m.tri(a, c, d, [(u0, v0), (u1, v1), (u0, v1)])


def trifoliate(m, rng, attach, direction, petiole_len, leaf_len, dead, ao, phase):
    """Petiole plus three leaflets; returns nothing, adds to m."""
    droop = -0.9 if dead else -0.25
    pts = grow(attach, direction, petiole_len, 4, UP * droop, rng, 0.03)
    tube(m, pts, 0.0019, 0.0012, 3, 0.3, 0.6, ao, phase)
    tip = pts[-1]
    d = (pts[-1] - pts[-2]).normalized()
    hd = Vector((d.x, d.y, 0.0))
    hd = hd.normalized() if hd.length > 1e-4 else Vector((1, 0, 0))
    # living leaflets tilt every way toward the light; held flat they are
    # edge-on to a camera at plant height and the plant reads as bare stems
    pitch = rng.uniform(-0.55, -0.9) if dead else rng.uniform(-0.2, 0.65)
    fold = math.radians(rng.uniform(32, 45) if dead else rng.uniform(10, 22))
    for k, yaw in enumerate((0.0, 72.0, -72.0)):
        if dead and rng.random() < 0.35:
            continue                        # dropped leaflets
        a = (Matrix.Rotation(math.radians(yaw + rng.uniform(-10, 10)), 3, UP) @ hd)
        leaf_dir = (a + UP * pitch).normalized()
        base = tip + a * (0.012 if k == 0 else 0.003)
        size = leaf_len * 1.15 * (1.0 if k == 0 else 0.86) * rng.uniform(0.92, 1.08)
        hint = (UP + jitter(rng, 0.6)).normalized()
        leaflet(m, base, leaf_dir, hint, size, fold, rng.randrange(3), 0.6, ao, phase)


def soybean(rng, dead):
    m = Mesh()
    H = rng.uniform(0.58, 0.66)
    n_nodes = 13
    # main stem, a slight zig-zag at the nodes
    stem = [Vector((0, 0, -0.01))]
    lean = Vector((rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05), 1)).normalized()
    for k in range(1, 13):
        z = H * k / 12
        zig = 0.006 * (1 if k % 2 else -1)
        stem.append(Vector((lean.x * z + zig, lean.y * z - zig * 0.5, z)))
    tube(m, stem, 0.0052, 0.0024, 6, 0.0, 0.3, 0.7, 0.0)

    az0 = rng.uniform(0, 360)
    for k in range(n_nodes):
        t = k / (n_nodes - 1)
        # internodes shorten toward the top, where the newest leaves crowd
        z = H * (0.1 + 0.9 * (1.0 - (1.0 - t) ** 1.35))
        at = Vector((lean.x * z, lean.y * z, z))
        az = math.radians(az0 + k * 180 + rng.uniform(-28, 28))
        horiz = Vector((math.cos(az), math.sin(az), 0))
        elev = math.radians(rng.uniform(26, 40) + 26 * t)
        petiole = (0.19 - 0.1 * t) * rng.uniform(0.85, 1.15)
        leaf = (0.125 - 0.04 * t) * rng.uniform(0.9, 1.1)
        ao = 0.5 + 0.5 * t
        if dead and rng.random() < 0.25:
            continue                        # whole leaves already fallen
        trifoliate(m, rng, at, horiz * math.cos(elev) + UP * math.sin(elev), petiole, leaf, dead, ao, rng.random())

        # pods at the middle nodes, in clusters of two to four
        if 2 <= k <= 9:
            for _ in range(rng.randint(2, 4)):
                pa = math.radians(rng.uniform(0, 360))
                pd = Vector((math.cos(pa), math.sin(pa), 0)) * 0.8 + UP * rng.uniform(-0.2, 0.55)
                pod(m, at + Vector((math.cos(pa), math.sin(pa), 0)) * 0.012, pd, rng, 0.45 + 0.4 * t, rng.random())

        # side branches from the lower nodes, each with leaves of their own:
        # most of a soybean plant's bulk
        if k in (1, 2, 3):
            bd = (horiz * 0.75 + UP).normalized()
            bpts = grow(at, bd, H * rng.uniform(0.32, 0.42), 5, UP * 0.3, rng)
            tube(m, bpts, 0.0035, 0.0018, 4, 0.1, 0.4, 0.6, 0.0)
            for j in range(1, 5):
                p = bpts[min(len(bpts) - 1, j + 1)]
                baz = math.radians(math.degrees(az) + (60 if j % 2 else -60) + rng.uniform(-20, 20))
                bh = Vector((math.cos(baz), math.sin(baz), 0))
                trifoliate(m, rng, p, bh * 0.8 + UP * 0.55, 0.12, 0.1, dead, 0.65, rng.random())

    # a young leaf at the apex, so the stem never ends in a bare stick
    top = stem[-1]
    trifoliate(m, rng, top, (Vector((rng.uniform(-1, 1), rng.uniform(-1, 1), 0)).normalized() * 0.4 + UP).normalized(),
               0.05, 0.07, dead, 1.0, rng.random())
    return m


# ---------------------------------------------------------------------------
# corn (atlas from build_crop_textures.py: leaf strips on the left half, husk top
# right, stalk / tassel / silk strips bottom right)

CORN_LEAF_U = [(k * 170 + 1) / 1024.0 for k in range(3)]
CORN_LEAF_W = 168 / 1024.0
CORN_LEAF_V = (4 / 1024.0, 1020 / 1024.0)
CORN_HUSK = (0.5, 1.0, 0.5, 1.0)
CORN_STALK = (0.5, 0.625, 0.0, 0.5)
CORN_TASSEL = (0.625, 0.75, 0.0, 0.5)
CORN_SILK = (0.75, 0.875, 0.0, 0.5)
# the drawn leaf is at most 42% as wide as its strip
CORN_LEAF_FILL = 0.42


def corn_leaf(m, rng, attach, azimuth, length, width, rise, ao, phase):
    """A leaf as a V-folded ribbon: up along the stalk for its sheath, then out
    and over, drooping toward the tip. The drawn leaf's alpha does the taper."""
    horiz = Vector((math.cos(azimuth), math.sin(azimuth), 0))
    ribbon = width / CORN_LEAF_FILL
    k = rng.randrange(3)
    u0 = CORN_LEAF_U[k]
    u1 = u0 + CORN_LEAF_W
    um = (u0 + u1) / 2
    v0, v1 = CORN_LEAF_V
    segs = 10
    out_angle = math.radians(rng.uniform(28, 44))
    droop = rng.uniform(0.9, 1.5)
    twist = math.radians(rng.uniform(-25, 25))
    rows = []
    p = attach.copy()
    for i in range(segs + 1):
        t = i / segs
        # direction: starts up the stalk, bends out, then down past horizontal
        tilt = out_angle * min(1.0, t * 4) + droop * max(0.0, t - 0.3) ** 1.3
        d = (UP * math.cos(tilt) + horiz * math.sin(tilt)).normalized()
        if i > 0:
            p = p + d * (length / segs)
        side = horiz.cross(UP).normalized()
        side = (Matrix.Rotation(twist * t, 3, d) @ side).normalized()
        nrm = side.cross(d).normalized()
        if nrm.dot(UP) < 0 and t > 0.2:
            nrm = -nrm
        fold = ribbon * 0.12
        flex = 0.35 + 0.65 * t
        col = (ao * (0.8 + 0.2 * t), flex, phase, 1)
        left = m.vert(p - side * ribbon / 2 + nrm * fold, (nrm - side * 0.3), col)
        mid = m.vert(p, nrm, col)
        right = m.vert(p + side * ribbon / 2 + nrm * fold, (nrm + side * 0.3), col)
        rows.append((left, mid, right, v0 + (v1 - v0) * t))
    for i in range(segs):
        (l0, m0, r0, va), (l1, m1, r1, vb) = rows[i], rows[i + 1]
        m.tri(l0, m0, m1, [(u0, va), (um, va), (um, vb)])
        m.tri(l0, m1, l1, [(u0, va), (um, vb), (u0, vb)])
        m.tri(m0, r0, r1, [(um, va), (u1, va), (u1, vb)])
        m.tri(m0, r1, m1, [(um, va), (u1, vb), (um, vb)])


def card_pair(m, base, direction, height, width, uv, flex0, flex1, ao, phase, rot=0.0):
    """Two crossed quads standing on base along direction (tassel, silk)."""
    u0, u1, v0, v1 = uv
    d = direction.normalized()
    s1 = (Matrix.Rotation(rot, 3, d) @ d.orthogonal()).normalized()
    s2 = d.cross(s1).normalized()
    for s in (s1, s2):
        n = s.cross(d).normalized()
        a = m.vert(base - s * width / 2, n, (ao, flex0, phase, 1))
        b = m.vert(base + s * width / 2, n, (ao, flex0, phase, 1))
        c = m.vert(base + s * width / 2 + d * height, n, (ao, flex1, phase, 1))
        e = m.vert(base - s * width / 2 + d * height, n, (ao, flex1, phase, 1))
        m.tri(a, b, c, [(u0, v0), (u1, v0), (u1, v1)])
        m.tri(a, c, e, [(u0, v0), (u1, v1), (u0, v1)])


def corn_stalk(rng):
    m = Mesh()
    H = rng.uniform(2.3, 2.45)
    lean = Vector((rng.uniform(-0.03, 0.03), rng.uniform(-0.03, 0.03), 1)).normalized()
    pts = [Vector((0, 0, -0.1))]
    for k in range(1, 13):
        z = H * k / 12
        pts.append(Vector((lean.x * z, lean.y * z, z)))
    # the stalk strip runs once over the whole stalk
    tube(m, pts, 0.024, 0.009, 8, 0.0, 0.5, 0.75, 0.0, uv=CORN_STALK, v_len=H + 0.1)

    n_leaves = rng.randint(12, 14)
    az0 = rng.uniform(0, 2 * math.pi)
    for k in range(n_leaves):
        t = k / (n_leaves - 1)
        z = H * (0.1 + 0.68 * t)
        at = Vector((lean.x * z, lean.y * z, z))
        # distichous: two ranks, 180 degrees apart, the plane drifting slowly
        az = az0 + (math.pi if k % 2 else 0.0) + t * 0.6 + rng.uniform(-0.25, 0.25)
        # middle leaves are the longest
        length = (0.55 + 0.42 * math.sin(math.pi * min(1.0, 0.15 + t))) * rng.uniform(0.92, 1.08)
        width = 0.085 * rng.uniform(0.9, 1.1) * (0.8 + 0.3 * math.sin(math.pi * t))
        corn_leaf(m, rng, at, az, length, width, 0.0, 0.55 + 0.45 * t, rng.random())

    # tassel at the top
    top = pts[-1]
    for k in range(3):
        d = (UP + Vector((rng.uniform(-0.3, 0.3), rng.uniform(-0.3, 0.3), 0))).normalized()
        card_pair(m, top - UP * 0.05, d, rng.uniform(0.32, 0.4), 0.14, CORN_TASSEL, 0.5, 0.8, 1.0, rng.random(),
                  rot=k * 0.6)
    return m


def corn_ear(rng):
    """A husked ear with silk, long axis Z, matching corn_cob.obj's extent
    (29 cm long, centred a little above 0)."""
    m = Mesh()
    z0, z1 = -0.13, 0.16
    profile = []
    segs = 10
    for i in range(segs + 1):
        t = i / segs
        r = 0.012 + 0.024 * math.sin(math.pi * min(1.0, t * 0.95 + 0.05)) ** 0.8
        profile.append((z0 + (z1 - z0) * t, r))
    u0, u1, v0, v1 = CORN_HUSK
    sides = 9
    rings = []
    for i, (z, r) in enumerate(profile):
        ring = []
        for j in range(sides + 1):
            a = 2 * math.pi * j / sides
            off = Vector((math.cos(a), math.sin(a), 0))
            wob = 1.0 + 0.06 * math.sin(a * 3 + z * 40)
            ring.append(m.vert(Vector((0, 0, z)) + off * r * wob, off, (0.8, 0.4, 0.3, 1)))
        rings.append((ring, v0 + (v1 - v0) * i / segs))
    for i in range(segs):
        (ra, va), (rb, vb) = rings[i], rings[i + 1]
        for j in range(sides):
            ua = u0 + (u1 - u0) * j / sides
            ub = u0 + (u1 - u0) * (j + 1) / sides
            m.tri(ra[j], ra[j + 1], rb[j + 1], [(ua, va), (ub, va), (ub, vb)])
            m.tri(ra[j], rb[j + 1], rb[j], [(ua, va), (ub, vb), (ua, vb)])
    # silk spilling from the tip: the strip hangs from its top edge, so the
    # cards are placed tip-down from just above the husk
    card_pair(m, Vector((0, 0, z1 + 0.06)), Vector((0.2, 0, -1)), 0.09, 0.07,
              (CORN_SILK[0], CORN_SILK[1], CORN_SILK[3], CORN_SILK[2]), 0.5, 0.5, 0.9, 0.0)
    return m


def make_object(name, m, material):
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in m.v], [], m.faces)
    me.update()
    me.materials.append(material)
    uv = me.uv_layers.new(name="UVMap")
    uv.data.foreach_set("uv", [c for p in m.uv for c in p])
    col = me.color_attributes.new(name="Col", type="FLOAT_COLOR", domain="POINT")
    col.data.foreach_set("color", [c for rgba in m.col for c in rgba])
    me.color_attributes.active_color = col
    me.polygons.foreach_set("use_smooth", [True] * len(m.faces))
    me.normals_split_custom_set_from_vertices([tuple(n) for n in m.n])
    obj = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(obj)
    return obj


def preview_material(name, atlas):
    m = bpy.data.materials.new(name + "_preview")
    m.use_nodes = True
    nt = m.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = bpy.data.images.load(os.path.join(REPO, "Assets", "Textures", "Crops", atlas + "_Albedo.png"))
    vc = nt.nodes.new("ShaderNodeVertexColor")
    vc.layer_name = "Col"
    sep = nt.nodes.new("ShaderNodeSeparateColor")
    mul = nt.nodes.new("ShaderNodeMixRGB")
    mul.blend_type = "MULTIPLY"
    mul.inputs["Fac"].default_value = 1.0
    nt.links.new(vc.outputs["Color"], sep.inputs["Color"])
    nt.links.new(tex.outputs["Color"], mul.inputs[1])
    nt.links.new(sep.outputs["Red"], mul.inputs[2])
    nt.links.new(mul.outputs["Color"], bsdf.inputs["Base Color"])
    gt = nt.nodes.new("ShaderNodeMath")
    gt.operation = "GREATER_THAN"
    gt.inputs[1].default_value = 0.35
    nt.links.new(tex.outputs["Alpha"], gt.inputs[0])
    nt.links.new(gt.outputs[0], bsdf.inputs["Alpha"])
    return m


def render_preview(objs, path):
    """A small plot: 3 rows 0.7 m apart, plants every 0.3 m, healthy in front."""
    sc = bpy.context.scene
    engines = [e.identifier for e in bpy.types.RenderSettings.bl_rna.properties["engine"].enum_items]
    sc.render.engine = "BLENDER_EEVEE_NEXT" if "BLENDER_EEVEE_NEXT" in engines else "BLENDER_EEVEE"
    sc.render.resolution_x, sc.render.resolution_y = 960, 540
    sc.view_settings.view_transform = "Standard"
    world = bpy.data.worlds.new("sky")
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (0.45, 0.58, 0.8, 1)
    sc.world = world
    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 3.5
    sun.rotation_euler = (math.radians(42), 0, math.radians(30))
    sc.collection.objects.link(sun)
    gm = bpy.data.meshes.new("g")
    gm.from_pydata([(-5, -5, 0), (5, -5, 0), (5, 5, 0), (-5, 5, 0)], [], [(0, 1, 2, 3)])
    g = bpy.data.objects.new("g", gm)
    gmat = bpy.data.materials.new("soil")
    gmat.use_nodes = True
    gmat.node_tree.nodes["Principled BSDF"].inputs["Base Color"].default_value = (0.09, 0.06, 0.04, 1)
    gm.materials.append(gmat)
    sc.collection.objects.link(g)
    for o, atlas in zip(objs, ("Soybean", "SoybeanDead")):
        o.data.materials[0] = preview_material(o.name, atlas)
    rng = random.Random(3)
    for row in range(3):
        for i in range(8):
            src = objs[1] if row == 2 else objs[0]
            inst = bpy.data.objects.new("p", src.data)
            inst.location = (i * 0.3 - 1.05 + rng.uniform(-0.03, 0.03), row * 0.7, 0)
            inst.rotation_euler = (0, 0, rng.uniform(0, 6.28))
            s = rng.uniform(0.9, 1.1)
            inst.scale = (s, s, s)
            sc.collection.objects.link(inst)
    for o in objs:
        o.hide_render = True
    cam = bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    cam.data.lens = 32
    sc.collection.objects.link(cam)
    sc.camera = cam
    cam.location = Vector((0.9, -1.4, 0.95))
    target = Vector((0, 0.6, 0.3))
    cam.rotation_euler = (target - cam.location).to_track_quat("-Z", "Y").to_euler()
    sc.render.filepath = path
    bpy.ops.render.render(write_still=True)


def main():
    os.makedirs(OUT, exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    objs = []
    for name, dead, seed in (("Soybean", False, 26), ("SoybeanDead", True, 26)):
        m = soybean(random.Random(seed), dead)
        mat = bpy.data.materials.new(name)
        objs.append(make_object(name, m, mat))
        zs = [v.z for v in m.v]
        print("CROP %s: %d triangles, height %.2f m" % (name, len(m.faces), max(zs) - min(zs)))
    preview = os.environ.get("CROP_PREVIEW")
    if preview:
        mats = [o.data.materials[0] for o in objs]
        render_preview(objs, preview)
        for o, mat in zip(objs, mats):
            o.data.materials[0] = mat
            o.hide_render = False
        for o in list(bpy.data.objects):
            if o not in objs:
                bpy.data.objects.remove(o)
    export(objs, "Soybean.fbx")
    for o in objs:
        bpy.data.objects.remove(o)

    corn = []
    for name, build in (("CornStalk", corn_stalk), ("CornEar", corn_ear)):
        m = build(random.Random(7))
        corn.append(make_object(name, m, bpy.data.materials.new(name)))
        zs = [v.z for v in m.v]
        print("CROP %s: %d triangles, height %.2f m" % (name, len(m.faces), max(zs) - min(zs)))
    export(corn, "Corn.fbx")
    print("DONE")


def export(objs, filename):
    bpy.ops.object.select_all(action="DESELECT")
    for o in objs:
        o.select_set(True)
    kwargs = dict(filepath=os.path.join(OUT, filename), use_selection=True, object_types={"MESH"},
                  axis_forward="-Z", axis_up="Y", bake_space_transform=True,
                  apply_scale_options="FBX_SCALE_ALL", mesh_smooth_type="OFF",
                  use_mesh_modifiers=False, add_leaf_bones=False, path_mode="STRIP")
    try:
        bpy.ops.export_scene.fbx(colors_type="LINEAR", **kwargs)
    except TypeError:
        bpy.ops.export_scene.fbx(**kwargs)


main()
