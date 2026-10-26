"""Three form directions for the cortex head, as looks-like mockups at one envelope, rendered photoreal side by side.

    python3 directions.py [spp]      # -> directions-lineup.png, dir-<A|B|C>-34.png, dir-<A|B|C>-front.png

A  soft capsule     -- squircle prism, flat edge-to-edge black glass face, aluminium base band
B  visor egg        -- ovoid skull, black visor band wrapping the face
C  capsule + ears   -- portrait squircle, single-curvature black face, round ear discs (Braun dot grid) with an
                       orange accent ring, aluminium neck band
Form studies only (smooth parametric surfaces via manifold3d, no internals); the chosen one is rebuilt in cortex.loon.
Envelope 140 W x 135 D x 165 H (portrait, mini class), the 4" square (Waveshare 4inch HDMI LCD (C), 720x720) face.
"""
import math, os, subprocess, sys, tempfile
import numpy as np
import manifold3d as mf
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.dirname(HERE)          # cortex/: the loon source dir (its stl/ proxies)
T = os.path.join(os.environ.get("CARGO_TARGET_DIR", os.path.expanduser("~/Developer/vcad/target")), "release")
R = os.environ.get("VCAD_RENDER", os.path.join(T, "vcad-render"))
SPP = int(sys.argv[1]) if len(sys.argv) > 1 else 96
EMISSIVE = os.environ.get("EMISSIVE", "1") == "1"        # needs vcad-render with :emissive-map
# two sizes, one head language. A human head is ~155 W x 195 D x 230 H; the 7" (166 wide) forces the full to ~1.3x.
SIZES = {
    # 4" square (Waveshare 4inch HDMI LCD (C), 720x720, active ~72 x 72)
    "mini": dict(W=128.0, D=150.0, H=172.0, ACT_W=72.0, ACT_H=72.0, UI="ui-eyes-720x720.png"),
    # 7" (Waveshare 7inch HDMI LCD (C), 1024x600, active 154.21 x 85.92; outline 164.9 x 107)
    "full": dict(W=200.0, D=215.0, H=235.0, ACT_W=154.21, ACT_H=85.92, UI="ui-eyes-1024x600.png"),
}


def use_size(name):
    """rebind the envelope the option builders read (they read these module globals at call time)."""
    globals().update(SIZES[name])


use_size("mini")


def man(tm):
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(tm.vertices, np.float32),
                               tri_verts=np.asarray(tm.faces, np.uint32)))


def tm(m):
    g = m.to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts))


def sgnpow(c, e):
    return np.sign(c) * np.abs(c) ** e


def superellipsoid(a, b, c, e1, e2, n=380, taper=None):
    """a closed superellipsoid mesh on a structured grid (smooth for gloss). Each exponent may be a pair:
    e1 = (bottom, top) roundness in elevation, e2 = (front, back) roundness in plan -- a head is a flat face on a
    round skull, a dome on a flat base. a may be a pair too (front depth, back depth). taper(z01) scales (x, y)."""
    e1b, e1t = e1 if isinstance(e1, tuple) else (e1, e1)
    e2f, e2b = e2 if isinstance(e2, tuple) else (e2, e2)
    af, ab = a if isinstance(a, tuple) else (a, a)
    nu, nv = n, 2 * n
    u = np.linspace(-math.pi / 2, math.pi / 2, nu + 1)[1:-1]          # poles added separately
    v = np.linspace(-math.pi, math.pi, nv, endpoint=False)
    U, V = np.meshgrid(u, v, indexing="ij")
    E1 = np.where(U >= 0, e1t, e1b)
    cv = np.cos(V)
    E2 = np.where(cv >= 0, e2f, e2b)
    A = np.where(cv >= 0, af, ab)
    x = A * sgnpow(np.cos(U), E1) * sgnpow(cv, E2)
    y = b * sgnpow(np.cos(U), E1) * sgnpow(np.sin(V), E2)
    z = c * sgnpow(np.sin(U), E1)
    if taper:
        k = taper((z / c + 1) / 2)
        x, y = x * k[0], y * k[1]
    P = np.stack([x, y, z], -1).reshape(-1, 3)
    P = np.vstack([P, [[0, 0, -c], [0, 0, c]]])
    bot, top = len(P) - 2, len(P) - 1
    F = []
    idx = lambda i, j: i * nv + (j % nv)
    for i in range(nu - 2):
        for j in range(nv):
            F += [[idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)], [idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)]]
    for j in range(nv):
        F.append([bot, idx(0, j + 1), idx(0, j)])
        F.append([top, idx(nu - 2, j), idx(nu - 2, j + 1)])
    m = trimesh.Trimesh(P, np.array(F), process=True)
    trimesh.repair.fix_normals(m)                         # the pole fans and the grid agree on winding, outward
    return man(m)


def box(x0, y0, z0, x1, y1, z1):
    return mf.Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def rbox(x0, y0, z0, x1, y1, z1, r):
    s = [mf.Manifold.sphere(r, 24).translate((x, y, z)) for x in (x0 + r, x1 - r) for y in (y0 + r, y1 - r)
         for z in (z0 + r, z1 - r)]
    return mf.Manifold.batch_hull(s)


def cyl_y(x, y0, y1, z, r, seg=96):
    return mf.Manifold.cylinder(y1 - y0, r, r, seg).rotate((-90, 0, 0)).translate((x, y0, z))


BIG = 1000.0


def base_form(e1, e2, sink, taper=None, a=None):
    """the head volume, sitting flat on z=0 with its bottom cut off `sink` above the shape's lowest point."""
    c = (H + sink) / 2
    f = superellipsoid(a or D / 2, W / 2, c, e1, e2, taper=taper).translate((0, 0, c - sink))
    return f ^ box(-BIG, -BIG, 0, BIG, BIG, BIG)


def option_A():
    # soft capsule: nearly symmetric, a slightly domed crown
    form = base_form((0.34, 0.48), (0.26, 0.40), 14.0)
    xf = D / 2 - 6.0                                     # the flat face plane; the glass is the next 5 mm of the form
    form = form ^ box(-BIG, -BIG, -BIG, xf + 5.0, BIG, BIG)
    band = form ^ box(-BIG, -BIG, 0, xf - 0.4, BIG, 6.0)
    body = form ^ box(-BIG, -BIG, 6.4, xf - 0.4, BIG, BIG)
    glass = form ^ box(xf, -BIG, 6.4, BIG, BIG, BIG)
    return dict(body=body, glass=glass, band=band), 0.50 * H


def option_B():
    # egg: a round skull that runs back past the face, a domed crown, a visor band wrapping the face
    taper = lambda t: (0.92 + 0.10 * t, 0.93 + 0.07 * t)
    form = base_form((0.55, 0.80), (0.50, 0.90), 22.0, taper=taper, a=(0.44 * D, 0.56 * D))
    lo, hi = (0.28 * H, 0.74 * H) if H < 200 else (0.25 * H, 0.76 * H)
    glass = form ^ rbox(8.0, -W, lo, BIG, W, hi, 14.0)
    body = form - rbox(7.4, -W - 1, lo - 0.6, BIG, W + 1, hi + 0.6, 14.6)
    band = form ^ box(-BIG, -BIG, 0, BIG, BIG, 5.0)
    body = body - box(-BIG, -BIG, -1, BIG, BIG, 5.4)
    return dict(body=body, glass=glass, band=band), 0.51 * H


def option_C():
    # capsule + ears: flat-ish face on a round skull, domed crown; the face a single curve about the vertical
    # (heat-bent acrylic), round ear discs on the sides with a Braun dot grid and an orange ring
    a = (0.44 * D, 0.56 * D)
    form = base_form((0.32, 0.62), (0.22, 0.75), 14.0, a=a)
    Rf, xf = 260.0 * W / 128.0, a[0] - 6.0             # the same bow at every size
    cyl = lambda r: mf.Manifold.cylinder(2 * BIG, r, r, 2048).translate((xf - Rf, 0, -BIG))
    form = form ^ cyl(Rf + 5.0)
    glass = (form - cyl(Rf)) - box(-BIG, -BIG, -1, BIG, BIG, 6.4)
    body = (form ^ cyl(Rf - 0.4)) - box(-BIG, -BIG, -1, BIG, BIG, 6.4)
    band = form ^ box(-BIG, -BIG, 0, BIG, BIG, 6.0)
    ex, ez, er = -10.0 * W / 128.0, 0.50 * H, 23.0 * W / 128.0
    side = W / 2
    ears, rings, holes = [], [], []
    for sg in (1, -1):
        y0, y1 = (side - 8.0, side + 1.2) if sg > 0 else (-side - 1.2, -side + 8.0)
        ears.append(cyl_y(ex, y0, y1, ez, er))
        rings.append(cyl_y(ex, y0, y1 - 0.6 if sg > 0 else y1, ez, er + 2.2) - cyl_y(ex, y0 - 1, y1 + 1, ez, er + 0.3))
        ni, nk = int(er / 2.2) + 1, int(er / 1.905) + 1
        for i in range(-ni, ni + 1):
            for k in range(-nk, nk + 1):
                py, pz = 2.2 * (i + 0.5 * (k % 2)), 1.905 * k
                if py * py + pz * pz < (er - 3.0) ** 2:
                    yy = (y1 - 1.5, y1 + 1) if sg > 0 else (y0 - 1, y0 + 1.5)
                    holes.append(cyl_y(ex + py, yy[0], yy[1], ez + pz, 0.7, 10))
        body = body - cyl_y(ex, y0 - 1, y1 + 1, ez, er + 2.5)
    ear = mf.Manifold.batch_boolean(ears, mf.OpType.Add) - mf.Manifold.batch_boolean(holes, mf.OpType.Add)
    ring = mf.Manifold.batch_boolean(rings, mf.OpType.Add)
    return dict(body=body, glass=glass, band=band, ear=ear, ring=ring), 0.50 * H


def option_D():
    # B + C: B's egg skull and visor face, C's ears -- round caps on the skull's own curve, a Braun dot grid and an
    # orange ring, set back behind the visor (where ears are)
    parts, zc = option_B()
    form = parts["body"] + parts["glass"] + parts["band"]
    bb = form.bounding_box()
    ctr = ((bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2)

    def grown(t):                                        # the skull offset outward ~t mm (an anisotropic scale)
        k = (1 + 2 * t / (bb[3] - bb[0]), 1 + 2 * t / (bb[4] - bb[1]), 1 + 2 * t / (bb[5] - bb[2]))
        return form.translate(tuple(-c for c in ctr)).scale(k).translate(ctr)

    ex, ez, er = -22.0 * W / 128.0, 0.50 * H, 22.0 * W / 128.0
    cap_out, ring_out = grown(1.5), grown(0.9)
    ears, rings, holes = [], [], []
    for sg in (1, -1):
        yy = (0.0, BIG) if sg > 0 else (-BIG, 0.0)
        ears.append((cap_out - form) ^ cyl_y(ex, yy[0], yy[1], ez, er))
        rings.append(((ring_out - form) ^ cyl_y(ex, yy[0], yy[1], ez, er + 2.4)) - cyl_y(ex, yy[0], yy[1], ez, er + 0.3))
        ni, nk = int(er / 2.2) + 1, int(er / 1.905) + 1
        for i in range(-ni, ni + 1):
            for k in range(-nk, nk + 1):
                py, pz = 2.2 * (i + 0.5 * (k % 2)), 1.905 * k
                if py * py + pz * pz < (er - 3.0) ** 2:
                    holes.append(cyl_y(ex + py, yy[0], yy[1], ez + pz, 0.7, 10))
    hole = mf.Manifold.batch_boolean(holes, mf.OpType.Add)
    parts["ear"] = mf.Manifold.batch_boolean(ears, mf.OpType.Add) - hole
    parts["ring"] = mf.Manifold.batch_boolean(rings, mf.OpType.Add)
    return parts, zc


def option_E():
    # D's egg + ears, but the visor is a GOGGLE LENS: a real double-curve polycarbonate lens (a ski/MTB
    # replacement lens, factory gloss on both faces) trimmed to the visor window and bonded behind the printed
    # bezel with VHB. The lens fills D's whole visor opening; the eyes light through it. Modelled as a 1.6 mm
    # skin of the skull's own surface within that window -- stand-in geometry for the real toric lens (same
    # thickness and wrap class); the actual part gets measured around the lens Cam buys.
    parts, zc = option_D()
    form = parts["body"] + parts["glass"] + parts["band"]
    bb = form.bounding_box()
    ctr = ((bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2)
    lo, hi = (0.28 * (bb[5] - bb[2]), 0.74 * (bb[5] - bb[2]))
    xv = 8.0 * D / 150.0
    win = rbox(bb[3] - 60.0, -W, lo, bb[3] + 30.0, W, hi, 14.0)
    def grown(k):
        kk = (1 + k / (bb[3] - bb[0]), 1 + k / (bb[4] - bb[1]), 1 + k / (bb[5] - bb[2]))
        return form.translate(tuple(-c for c in ctr)).scale(kk).translate(ctr)
    # flush: outer face = the skull's own surface, 1.6 thick, bonded in D's 0.6 reveal (a goggle in a bezel)
    parts["glass"] = (grown(0.6) - grown(-1.0)) ^ win
    return parts, (lo + hi) / 2


def option_F():
    # the printable MVP of E: no lens, no formed part. A single-curve visor band (a vertical-axis cylinder --
    # printable standing in spiral-vase mode, glossy PETG) wrapped over the skull's front with the 4-inch
    # panel's glass flush in a window at its centre as the dead front. Skull + ears = D. The band's radius is
    # fitted to the skull's measured profile, then the band is kept where it stays within 6 mm of the skull.
    parts, zc = option_D()
    body = parts["body"] + parts["glass"] + parts["band"] - parts["ear"] - parts["ring"]
    form = parts["body"] + parts["glass"] + parts["band"]
    bb = form.bounding_box()
    ctr = ((bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2)
    zc = (bb[2] + bb[5]) / 2 + 0.02 * (bb[5] - bb[2])
    lo, hi = 0.28 * (bb[5] - bb[2]), 0.74 * (bb[5] - bb[2])
    def grown(k):
        kk = (1 + k / (bb[3] - bb[0]), 1 + k / (bb[4] - bb[1]), 1 + k / (bb[5] - bb[2]))
        return form.translate(tuple(-c for c in ctr)).scale(kk).translate(ctr)
    skin_out = grown(0.6)
    xf = skin_out.bounding_box()[3]
    Rf = 240.0 * W / 128.0
    cyc = lambda r: mf.Manifold.cylinder(2 * BIG, r, r, 2048).translate((xf - Rf, 0, -BIG))
    win = rbox(bb[3] - 60.0, -W, lo, bb[3] + 30.0, W, hi, 14.0)
    strip = ((skin_out - skin_out.translate((-1.6, 0, 0))) ^ win) ^ cyc(Rf + 5.0)   # the visor skin, single-curved
    band = strip ^ box(-BIG, -BIG, lo, BIG, BIG, hi)
    band = band - (band.translate((2.6, 0, 0)) ^ (form ^ win))       # keep only where it stands off the skull <2.6
    aw, ah = ACT_W / 2 + 1.5, ACT_H / 2 + 1.5
    pocket = box(bb[3] - 40.0, -aw, zc - ah, xf + 5.0, aw, zc + ah)   # through the band and the skull front
    band = band - pocket                                             # the window: the band is the bezel
    # the display recessed inside the window: its front face is the dead front (the emissive pane), the body of
    # the display behind it. The skull stays intact behind; the panel hides its own pocket.
    glass = box(xf - 4.4, -ACT_W / 2, zc - ACT_H / 2, xf - 4.2, ACT_W / 2, zc + ACT_H / 2)
    panel = box(xf - 15.0, -ACT_W / 2 - 0.5, zc - ACT_H / 2 - 0.5, xf - 4.4, ACT_W / 2 + 0.5, zc + ACT_H / 2 + 0.5)
    body = body - pocket                                             # the window goes through the skull
    return dict(body=body, glass=glass, band=band, ear=parts["ear"], ring=parts["ring"], panel=panel), zc


def option_G():
    # one-material premium: ONE volume (D's egg + ears), the face a RECESS in it: the visor window is carved
    # 2 mm into the skull on an offset surface (a 0.8 mm chamfer lip at the rim), and the recess FLOOR is the
    # dead front -- a 0.2 mm glass skin carrying the lit eyes, with the display body behind it. Same trick as
    # E/F (offset shells), so the rim wraps the skull exactly and the eyes project straight onto the floor.
    parts, zc = option_D()
    form = parts["body"] + parts["glass"] + parts["band"]
    bb = form.bounding_box()
    ctr = ((bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2)
    zc = (bb[2] + bb[5]) / 2 + 0.02 * (bb[5] - bb[2])
    lo, hi = 0.30 * (bb[5] - bb[2]), 0.72 * (bb[5] - bb[2])
    def grown(k):
        kk = (1 + k / (bb[3] - bb[0]), 1 + k / (bb[4] - bb[1]), 1 + k / (bb[5] - bb[2]))
        return form.translate(tuple(-c for c in ctr)).scale(kk).translate(ctr)
    yw = ACT_W / 2 + 14.0                                # the visor's width: the window stops here
    win = rbox(bb[3] - 60.0, -yw, lo, bb[3] + 30.0, yw, hi, 14.0)
    shell = lambda a, b: grown(a) - grown(b)
    recess = win ^ shell(0.8, -2.0)                      # the window, 2 mm deep, 0.8 chamfer lip at the rim
    skull = form - recess
    glass = win ^ shell(-1.2, -1.4)                      # the dead front: a 0.2 mm glass skin, outermost in the recess
    panel = win ^ shell(-1.4, -2.2)                      # the display body behind it
    skull = skull - panel
    return dict(body=skull, glass=glass, panel=panel, ear=parts["ear"], ring=parts["ring"]), zc


def option_H():
    # F kept, but every joint gets a designed reveal (one material, two prints): a 1.2 mm tall, 1 mm deep step
    # at the band's top and bottom edges (a shadow line instead of a raw edge), the band-skull seam offset so the
    # band reads as its own part, and the ear ring kept as a separate colour-free piece (same filament, glossy).
    parts, zc = option_F()
    band = parts["band"]
    bb = band.bounding_box()
    t, deep = 1.2, 1.0
    band = band - box(-BIG, -BIG, bb[5] - t, BIG, BIG, bb[5] + 1.0)     # top step
    band = band - box(-BIG, -BIG, bb[2] - 1.0, BIG, BIG, bb[2] + t)     # bottom step
    parts["band"] = band
    return parts, zc


def explode(tag, fn, d):
    # an exploded view: shell parts spread apart along sensible axes, the electronics stack behind (the mini's
    # bought-part proxies from cortex.loon), so the insides and the assembly order are visible.
    parts, zc = fn()
    use = {"body": (0, 0, 95), "glass": (60, 0, 0), "panel": (40, 0, 0), "eye": (75, 0, 15),
           "ear_l": (0, 100, 0), "ear_r": (0, -100, 0), "ring_l": (0, 140, 0), "ring_r": (0, -140, 0),
           "band": (0, 0, -95), "lens": (30, 55, 0)}
    roots = []
    import shutil
    for k, off in use.items():
        base = k[:-2] if k.endswith("_l") or k.endswith("_r") else k
        if base not in parts:
            continue
        m = parts[base]
        if base != k:                                          # a pair part: split the mesh along y=0
            m = mf.Manifold.batch_boolean([
                m, mf.Manifold.cube((4 * BIG, 2 * BIG, 4 * BIG)).translate(
                    (-BIG, 0 if k.endswith("_l") else -2 * BIG, -BIG))], mf.OpType.Intersect)
        f = f"{tag}-x-{k}.stl"
        tm(m.translate(off)).export(os.path.join(d, f))
        roots.append(f'[root [import-mesh-scaled 0.001 0.001 0.001 "{f}"] "{MAT.get(base, "default")}"]')
    # the electronics stack, BEHIND the skull (the head's back is -x): battery low, compute mid, heatsink high
    ex, ey, ez = -95.0, 0.0, 86.0
    stack = [("cortex-battery", (ex, ey, ez - 30.0), "cell_pack"),
             ("cortex-compute", (ex - 55.0, ey, ez - 8.0), "pcb_green"),
             ("cortex-heatsink", (ex - 105.0, ey, ez + 6.0), "alu"),
             ("cortex-speakers", (ex - 40.0, 55.0, ez - 60.0), "ear_grey"),
             ("cortex-speakers", (ex - 40.0, -55.0, ez - 60.0), "ear_grey")]
    for si, (name, (x, y, z), mat) in enumerate(stack):
        p = os.path.join(SRC, "stl", name + ".stl")
        if not os.path.exists(p):
            continue
        rel = name if si < 3 else f"{name}-{si}"
        shutil.copy(p, os.path.join(d, rel + ".stl"))
        roots.append(f'[root [translate {x} {y} {z} [import-mesh-scaled 0.001 0.001 0.001 "{rel}.stl"]] "{mat}"]')
    return roots


def option_I():
    # no visor: the skull IS the face (Cam: "drop the visor, make the skull beautiful"). The two real cameras
    # are the EYES: recessed lens windows high on the skull at +-33 (the 66 mm baseline), dark glass discs in
    # them. The screen sits low as nose/mouth, flush in a chamfered rectangular recess, its front 3 mm behind
    # the rim (a shadow reveal, no bezel). The ears sit on the skull's own curve, the ring a recessed groove.
    sink = 14.0
    a_pair = (0.44 * D, 0.56 * D)
    skull = base_form((0.30, 0.66), (0.20, 0.78), sink, a=a_pair)
    bb = skull.bounding_box()
    ctr = ((bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2)

    def grown2(k):
        kk = (1 + k / (bb[3] - bb[0]), 1 + k / (bb[4] - bb[1]), 1 + k / (bb[5] - bb[2]))
        return skull.translate(tuple(-c for c in ctr)).scale(kk).translate(ctr)

    z_cam = 0.60 * H                                            # the camera eyes, high on the skull
    zc = 0.34 * H                                               # the screen: nose/mouth, well under the eyes
    skin = grown2(0.6)
    xf = skin.bounding_box()[3]
    gtm = tm(skin)
    def surf_x(y, z):
        loc, _, _ = gtm.ray.intersects_location([[bb[3] + 40.0, y, z]], [[-1.0, 0, 0]])
        return loc[:, 0].max() if len(loc) else xf
    x_edge = surf_x(ACT_W / 2 + 2.0, zc)                        # the skull's surface at the screen's side edge
    # the screen recess: a chamfered rectangular opening through the front wall
    aw, ah = ACT_W / 2, ACT_H / 2
    rec = None
    for k, (x, hw, hh) in enumerate([(x_edge + 4.0, aw + 3.0, ah + 3.0), (x_edge - 1.0, aw + 2.0, ah + 2.0),
                                     (x_edge - 3.0, aw + 0.4, ah + 0.4)]):
        r = rbox(x, -hw, zc - hh, BIG, hw, zc + hh, 3.0)        # from x BACK to +inf: an opening, not a slab
        rec = r if rec is None else rec + r
    skull = skull - rec
    xg = x_edge - 3.0                                           # the pane's front face: 3 behind the rim plane
    glass = box(xg - 0.2, -aw, zc - ah, xg, aw, zc + ah)
    panel = box(x_edge - 16.0, -aw - 0.5, zc - ah - 0.5, xg, aw + 0.5, zc + ah + 0.5)
    skull = skull - panel
    # the camera eyes: Ø9.5 windows through the skull at +-33, dark glass discs seated inside
    cam_y, cam_r = 33.0, 4.75
    cam_win = None
    for sg in (1, -1):
        w = cyl_y(bb[0] - 10.0, sg * cam_y - cam_r, sg * cam_y + cam_r, z_cam, cam_r, 64)
        cam_win = w if cam_win is None else cam_win + w
    skull = skull - cam_win
    # the lens discs: dark glass, a 1 mm disc perpendicular to x, seated 2.5 mm inside the rim
    lens = mf.Manifold.batch_boolean([
        mf.Manifold.cylinder(1.0, cam_r - 0.1, cam_r - 0.1, 64).rotate((0, 90, 0)).translate((x_edge - 2.5, sg * cam_y, z_cam))
        for sg in (1, -1)], mf.OpType.Add)
    # the ears: on the skull's own curve, the ring a recessed groove
    ex, ez, er = -12.0 * W / 128.0, 0.50 * H, 22.0 * W / 128.0
    cap_out, ring_out = grown2(1.2), grown2(0.5)
    ears, rings, holes = [], [], []
    for sg in (1, -1):
        yy = (0.0, BIG) if sg > 0 else (-BIG, 0.0)
        ears.append((cap_out - skull) ^ cyl_y(ex, yy[0], yy[1], ez, er))
        rings.append(((ring_out - skull) ^ cyl_y(ex, yy[0], yy[1], ez, er + 1.8)) - cyl_y(ex, yy[0], yy[1], ez, er + 0.2))
        ni, nk = int(er / 2.2) + 1, int(er / 1.905) + 1
        for i in range(-ni, ni + 1):
            for k in range(-nk, nk + 1):
                py, pz = 2.2 * (i + 0.5 * (k % 2)), 1.905 * k
                if py * py + pz * pz < (er - 3.0) ** 2:
                    holes.append(cyl_y(ex + py, yy[0], yy[1], ez + pz, 0.7, 10))
    hole = mf.Manifold.batch_boolean(holes, mf.OpType.Add)
    ear = mf.Manifold.batch_boolean(ears, mf.OpType.Add) - hole
    ring = mf.Manifold.batch_boolean(rings, mf.OpType.Add)
    return dict(body=skull, glass=glass, panel=panel, lens=lens, ear=ear, ring=ring), zc


def option_J():
    # the Figure lesson: no literal features; one dark glass face; the off-state a monolith. The skull gets a
    # FLAT FACE FACET (cut with a plane ~9 mm behind its crown front): the silhouette stays an egg from the
    # side, the face is one calm flat field, and the panel sits flush in it -- no stuck-on card, no curvature
    # fight. The eyes are soft emissive domes high on the pane (baby-schema sizing); the cameras hide behind.
    # Ears + recessed groove rings carry the character; the crown stays clean.
    sink = 14.0
    a_pair = (0.44 * D, 0.56 * D)
    skull = base_form((0.30, 0.66), (0.20, 0.78), sink, a=a_pair)
    bb = skull.bounding_box()
    ctr = ((bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, (bb[2] + bb[5]) / 2)

    def grown2(k):
        kk = (1 + k / (bb[3] - bb[0]), 1 + k / (bb[4] - bb[1]), 1 + k / (bb[5] - bb[2]))
        return skull.translate(tuple(-c for c in ctr)).scale(kk).translate(ctr)

    skin = grown2(0.6)
    xf = skin.bounding_box()[3]
    zc = 0.52 * H                                              # the face panel: centred, slightly high
    x_face = xf - 9.0                                          # the flat face plane
    skull = skull ^ box(-BIG, -BIG, -BIG, x_face, BIG, BIG)    # the facet cut (the crown keeps its dome)
    # the panel: flush in the facet, 1.2 mm reveal all round. TALL (84 x 104): it covers nose-to-chin, so the
    # drawn face -- eyes, waveform mouth, all of it -- lives on the one screen (the Figure/EVE approach); the
    # physical domes overlay the drawn eyes. The mouth is animated (waveform + mouth shapes), which the research
    # and Cam both point at: a face without a mouth is unsettling, a drawn mouth is not.
    pw, ph = 42.0, 52.0
    glass = box(x_face - 0.25, -pw, zc - ph, x_face - 0.05, pw, zc + ph)
    panel = box(x_face - 13.0, -pw - 0.5, zc - ph - 0.5, x_face - 0.25, pw + 0.5, zc + ph + 0.5)
    skull = skull - panel
    # the eyes: three nested emissive domes per eye (halo, mid, core) -- a cheap bloom, since kosm-render has
    # no per-hit image emission yet (the real soft edges come from the UI image when it does)
    ew, eh, gap = 19.0, 25.0, 1.5 * 19.0
    # the eyes: glossy dark glass domes (cabochons -- 22 mm acrylic/glass dome halves, off the shelf) over the
    # pane's drawn eyes. Their emissive cores sit BEHIND the pane plane (the screen's own pixels are the light).
    ew, eh, gap = 21.0, 26.0, 1.45 * 21.0
    xg = x_face - 0.05                                        # the pane's front plane
    dome_l, core = None, None
    for sg in (1, -1):
        yc = sg * (0.5 * gap + 0.5 * ew)                      # the eyes straddle y = 0, on the pane's upper half
        d = mf.Manifold.sphere(1.0, 96).scale((3.0, ew / 2, eh / 2)).translate((xg + 2.2 - 3.0, yc, zc + 14.0))
        dome_l = d if dome_l is None else dome_l + d
        # the lit pixels: an emissive disc on the pane under each dome, seen THROUGH the translucent dome
        c = box(xg - 0.02, yc - ew / 2 * 0.8, zc + 14.0 - eh / 2 * 0.8, xg + 0.02, yc + ew / 2 * 0.8,
                zc + 14.0 + eh / 2 * 0.8)
        core = c if core is None else core + c
    eye = dome_l
    # the mouth: a soft emissive bar low on the pane -- the waveform/mouth animation lives here in firmware
    # (dim glow = speaking; off = dark). A stadium: the hull of two spheres along y.
    mw, mh, mz = 26.0, 7.0, zc - 16.0
    mouth = mf.Manifold.batch_hull([
        mf.Manifold.sphere(mh / 2, 64).translate((xg + 0.02, -mw / 2 + mh / 2, mz)),
        mf.Manifold.sphere(mh / 2, 64).translate((xg + 0.02, mw / 2 - mh / 2, mz))])
    # the ears: on the skull's own curve, the ring a recessed groove
    ex, ez, er = -12.0 * W / 128.0, 0.50 * H, 22.0 * W / 128.0
    cap_out, ring_out = grown2(1.2), grown2(0.5)
    ears, rings, holes = [], [], []
    for sg in (1, -1):
        yy = (0.0, BIG) if sg > 0 else (-BIG, 0.0)
        ears.append((cap_out - skull) ^ cyl_y(ex, yy[0], yy[1], ez, er))
        rings.append(((ring_out - skull) ^ cyl_y(ex, yy[0], yy[1], ez, er + 1.8)) - cyl_y(ex, yy[0], yy[1], ez, er + 0.2))
        ni, nk = int(er / 2.2) + 1, int(er / 1.905) + 1
        for i in range(-ni, ni + 1):
            for k in range(-nk, nk + 1):
                py, pz = 2.2 * (i + 0.5 * (k % 2)), 1.905 * k
                if py * py + pz * pz < (er - 3.0) ** 2:
                    holes.append(cyl_y(ex + py, yy[0], yy[1], ez + pz, 0.7, 10))
    hole = mf.Manifold.batch_boolean(holes, mf.OpType.Add)
    ear = mf.Manifold.batch_boolean(ears, mf.OpType.Add) - hole
    ring = mf.Manifold.batch_boolean(rings, mf.OpType.Add)
    return dict(body=skull, glass=glass, panel=panel, eye=dome_l, mouth=mouth, ear=ear, ring=ring), zc


MATS = """[material "soft_black" 0.028 0.029 0.032 0.0 0.62]
[material "gloss_black" 0.006 0.006 0.008 0.0 0.04]
[material "alu" 0.80 0.80 0.81 1.0 0.28]
[material "ear_grey" 0.11 0.115 0.12 0.4 0.45]
[material "accent" 1.0 0.36 0.06 0.0 0.35]
[material "lens" 0.0 0.0 0.0 0.0 0.08]
[material "banana" 0.86 0.66 0.10 0.0 0.55]
[material "banana_stem" 0.30 0.32 0.10 0.0 0.7]
[material "banana_tip" 0.12 0.08 0.04 0.0 0.7]
[material "cell_pack" 0.13 0.22 0.42 0.1 0.5]
[material-emissive "dome_smoke" 0.72 0.80 0.92 0.0 0.10 1.0 1.0 1.0 1.05 "" 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0 0.0]   ; the lens glows (the screen behind it, seen through smoked glass)
[material-emissive "eye_core" 0.92 0.95 1.0 0.0 0.3 1.0 1.0 1.0 2.4 "" 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0 0.0]
[material-emissive "mouth_glow" 0.75 0.80 0.88 0.0 0.3 1.0 1.0 1.0 1.1 "" 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0 0.0]
[material-emissive "eye_core" 0.93 0.95 0.98 0.0 0.22 1.0 1.0 1.0 2.6 "" 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1.0 0.0]
"""
MAT = dict(body="soft_black", glass="gloss_black", band="gloss_black", ear="ear_grey", ring="accent",
           panel="soft_black", lens="lens", eye="dome_smoke", core="eye_core", mouth="mouth_glow")


def build(tag, fn, d, dy=0.0):
    # the option's parts as STLs + roots; the face glass carries the emissive map (a true dead front).
    parts, zc = fn()
    g = parts["glass"]
    hi_x = g.bounding_box()[3]
    aw, ah = ACT_W / 2, ACT_H / 2
    mname = f"face_{tag}"
    if EMISSIVE:
        # looking down -x with z up, the viewer's left is -y: the image's top-left goes at -y, its width runs +y
        mat = f'[material "{mname}" 0.010 0.010 0.013 0.0 0.10]'   # pane: a lighter gloss than the skull, reads as glass
    else:
        mat = f'[material "{mname}" 0.006 0.006 0.008 0.0 0.04]'
    roots = []
    for k, m in parts.items():
        f = f"{tag}-{k}.stl"
        tm(m.translate((0, dy, 0))).export(os.path.join(d, f))
        roots.append(f'[root [import-mesh-scaled 0.001 0.001 0.001 "{f}"] "{mname if k == "glass" else MAT[k]}"]')
    return mat, roots


def banana():
    """a Cavendish for scale: ~195 mm along a gentle arc, ~36 mm across a rounded-pentagon section, a stem at one
    end and a dark tip at the other, lying on the desk. Returns (fruit, stem, tip) manifolds, centred at the origin."""
    R0, sweep, N, K = 160.0, math.radians(70.0), 90, 40
    rings = []
    for i in range(N + 1):
        t = i / N
        th = -sweep / 2 + sweep * t
        cx, cy = R0 * math.sin(th), R0 * (1 - math.cos(th))       # the spine in the desk plane, bowed toward +y
        tx, ty = math.cos(th), math.sin(th)                       # tangent
        nx, ny = -ty, tx                                          # in-plane normal
        r = 18.0 * max(math.sin(math.pi * t), 0.0) ** 0.42 + 2.2   # fat middle, tapered ends
        ring = []
        for k in range(K):
            ph = 2 * math.pi * k / K
            facet = 1.0 + 0.045 * math.cos(5 * ph)                 # the five ribs
            u, w = r * facet * math.cos(ph), r * facet * math.sin(ph)
            ring.append((cx + nx * u, cy + ny * u, w))
        rings.append(ring)
    P = [p for ring in rings for p in ring]
    F = []
    for i in range(N):
        for k in range(K):
            a, b = i * K + k, i * K + (k + 1) % K
            c, d = a + K, b + K
            F += [[a, c, d], [a, d, b]]
    P.append(rings[0][0][:2] + (0.0,)); P[-1] = (sum(p[0] for p in rings[0]) / K, sum(p[1] for p in rings[0]) / K, 0.0)
    P.append((sum(p[0] for p in rings[-1]) / K, sum(p[1] for p in rings[-1]) / K, 0.0))
    s0, s1 = len(P) - 2, len(P) - 1
    for k in range(K):
        F.append([s0, k, (k + 1) % K])
        F.append([s1, N * K + (k + 1) % K, N * K + k])
    m = trimesh.Trimesh(np.array(P), np.array(F), process=True)
    trimesh.repair.fix_normals(m)
    fruit = man(m)
    # stem off the last ring, along the tangent; a dark nub at the first
    th1 = sweep / 2
    e = np.array(rings[-1]).mean(0)
    stem = mf.Manifold.cylinder(26.0, 3.6, 3.0, 24).rotate((0, 90, 0)).rotate((0, 0, math.degrees(th1))).translate(tuple(e))
    e0 = np.array(rings[0]).mean(0)
    tip = mf.Manifold.sphere(3.4, 24).translate(tuple(e0))
    lift = 18.0 * 1.045 + 2.2                                     # rest it on the desk
    bb = fruit.bounding_box()
    c = ((bb[0] + bb[3]) / 2, (bb[1] + bb[4]) / 2, -lift)
    return [x.translate((-c[0], -c[1], -c[2])) for x in (fruit, stem, tip)]


FULL = ["above", "beside", "corner"]
UI7 = "ui-face-1024x600.png"


def full_head(opt, d):
    """a 7-inch full mockup from cortex.loon itself (its full-<opt>.loon as size.loon), watertight via print_build,
    in its own subfolder of d. Returns ({part: relpath}, screen plane (o, u, v), y half-width)."""
    import shutil
    sys.path.insert(0, HERE)
    import photoreal as PH
    sub = os.path.join(d, opt)
    os.makedirs(sub, exist_ok=True)
    shutil.copy(os.path.join(os.path.dirname(HERE), "cortex.loon"), sub)
    shutil.copy(os.path.join(HERE, f"full-{opt}.loon"), os.path.join(sub, "size.loon"))
    names = PH.meshes(os.path.join(sub, "size.loon"), sub)
    s = PH.size_of(os.path.join(HERE, f"full-{opt}.loon"))
    o, u, v, _, _ = PH.screen_plane(s)
    return {k: f"{opt}/{f}" for k, f in names.items()}, (o, u, v), s["hw"]


def main():
    d = tempfile.mkdtemp(prefix="cortex-dirs-")
    import shutil
    for sz in SIZES.values():
        shutil.copy(os.path.join(HERE, sz["UI"]), d)
    shutil.copy(os.path.join(HERE, UI7), d)
    allopts = dict(A=option_A, B=option_B, C=option_C, D=option_D, E=option_E, F=option_F, G=option_G, H=option_H, I=option_I, J=option_J)
    opts = [(t, allopts[t]) for t in os.environ.get("OPTS", "A,B,C,D,E,F,G,H,I,J").split(",")]
    mats, rootsall = [], []
    gap, y = 45.0, 0.0
    y_banana = None
    for sz in ("mini", "full"):
        use_size(sz)
        for tag0, fn in opts:
            tag = tag0 if sz == "mini" else tag0 + "7"
            mat, roots = build(tag, fn, d)
            with open(os.path.join(d, f"{tag}.loon"), "w") as f:
                f.write(MATS + mat + "\n" + "\n".join(roots) + "\n")
            half = W / 2 + 3.0
            y += half
            mat2, roots2 = build(tag + "L", fn, d, dy=y)
            y += half + gap
            mats.append(mat2); rootsall += roots2
            print(tag, "built")
        if y_banana is None:
            y_banana = y - gap / 2 + 20.0                 # between the two sizes, out in front
            y += 50.0
    if os.environ.get("OLD7"):                            # the earlier boxy 7-inch heads from cortex.loon
        fullmat = {"cortex-tub-full": "soft_black", "cortex-floor": "soft_black", "cortex-ear-grilles": "soft_black",
                   "cortex-exhaust-grille": "soft_black", "cortex-back-grille": "soft_black", "d7-cams": "lens"}
        for opt in FULL:
            names, (o, u, v), hw = full_head(opt, d)
            y += hw + 3.0
            mname = f"face7_{opt}"
            mats.append(f'[material-emissive "{mname}" 0.006 0.006 0.008 0.0 0.04 1.0 1.0 1.0 3.0 "{UI7}" '
                        f'{o[0]:.3f} {o[1] + y:.3f} {o[2]:.3f} {u[0]:.4f} {u[1]:.4f} {u[2]:.4f} {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}]')
            for k, f in names.items():
                m = mname if k == "d7-glass" else fullmat[k]
                rootsall.append(f'[root [translate 0.0 {y:.3f} 0.0 [import-mesh-scaled 0.001 0.001 0.001 "{f}"]] "{m}"]')
            y += hw + 3.0 + gap
            print(opt, "built")
    # banana for scale, on the desk in front of the last head
    fruit, stem, tip = banana()
    for nm, m, mt in (("banana", fruit, "banana"), ("banana-stem", stem, "banana_stem"), ("banana-tip", tip, "banana_tip")):
        tm(m.rotate((0, 0, 84)).translate((150.0, y_banana, 0))).export(os.path.join(d, nm + ".stl"))
        rootsall.append(f'[root [import-mesh-scaled 0.001 0.001 0.001 "{nm}.stl"] "{mt}"]')
    with open(os.path.join(d, "lineup.loon"), "w") as f:
        f.write(MATS + "\n".join(mats) + "\n" + "\n".join(rootsall) + "\n")
    common = ["--photoreal", "--spp", str(SPP), "--backdrop", "studio", "--env", "studio", "--aperture", "0.01"]
    # the lineup straight on with a long lens, so sizes compare fairly
    jobs = [("lineup.loon", "directions-lineup.png", ["--size", "3000x1000", "--azimuth", "0", "--elevation", "12",
                                                      "--fov", "9", "--fill", "0.94"])]
    for tag in [t for t, _ in opts] + [t + "7" for t, _ in opts]:
        jobs.append((f"{tag}.loon", f"dir-{tag}-34.png", ["--size", "1200x1200", "--azimuth", "32", "--elevation", "10", "--fov", "22", "--fill", "0.8"]))
        jobs.append((f"{tag}.loon", f"dir-{tag}-front.png", ["--size", "1200x1200", "--azimuth", "0", "--elevation", "4", "--fov", "22", "--fill", "0.8"]))
    if os.environ.get("EXPLODE"):
        for tag0, fn in opts:
            exp = explode(tag0, fn, d)
            with open(os.path.join(d, f"{tag0}-x.loon"), "w") as f:
                f.write(MATS + "\n".join(exp) + "\n")
            jobs.append((f"{tag0}-x.loon", f"{tag0}-exploded.png",
                         ["--size", "1800x1400", "--azimuth=-58", "--elevation", "14", "--fov", "20",
                          "--fill", "0.9", "--aperture", "0.02"]))
    if os.environ.get("ONLY"):
        jobs = [j for j in jobs if os.environ["ONLY"] in j[1]]
    for src, out, extra in jobs:
        r = subprocess.run([R, os.path.join(d, src)] + common + extra + ["-o", os.path.join(HERE, out)],
                           capture_output=True, text=True)
        print(out, "ok" if r.returncode == 0 else r.stderr[-500:])
    print(d)


if __name__ == "__main__":
    main()
