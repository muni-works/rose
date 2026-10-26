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
T = os.path.join(os.environ.get("CARGO_TARGET_DIR", os.path.expanduser("~/Developer/vcad/target")), "release")
R = os.environ.get("VCAD_RENDER", os.path.join(T, "vcad-render"))
SPP = int(sys.argv[1]) if len(sys.argv) > 1 else 96
EMISSIVE = os.environ.get("EMISSIVE", "1") == "1"        # needs vcad-render with :emissive-map
# two sizes, one head language. A human head is ~155 W x 195 D x 230 H; the 7" (166 wide) forces the full to ~1.3x.
SIZES = {
    # 4" square (Waveshare 4inch HDMI LCD (C), 720x720, active ~72 x 72)
    "mini": dict(W=128.0, D=150.0, H=172.0, ACT_W=72.0, ACT_H=72.0, UI="ui-face-720x720.png"),
    # 7" (Waveshare 7inch HDMI LCD (C), 1024x600, active 154.21 x 85.92; outline 164.9 x 107)
    "full": dict(W=200.0, D=215.0, H=235.0, ACT_W=154.21, ACT_H=85.92, UI="ui-face-1024x600.png"),
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


MATS = """[material "soft_black" 0.028 0.029 0.032 0.0 0.62]
[material "gloss_black" 0.006 0.006 0.008 0.0 0.04]
[material "alu" 0.80 0.80 0.81 1.0 0.28]
[material "ear_grey" 0.11 0.115 0.12 0.4 0.45]
[material "accent" 1.0 0.36 0.06 0.0 0.35]
[material "lens" 0.0 0.0 0.0 0.0 0.08]
[material "banana" 0.86 0.66 0.10 0.0 0.55]
[material "banana_stem" 0.30 0.32 0.10 0.0 0.7]
[material "banana_tip" 0.12 0.08 0.04 0.0 0.7]
"""
MAT = dict(body="soft_black", glass="gloss_black", band="alu", ear="ear_grey", ring="accent")


def build(tag, fn, d, dy=0.0):
    """the option's parts as STLs + roots. The face glass itself carries the emissive map over the active area: a
    true dead front, one pane, the screen invisible until it lights."""
    parts, zc = fn()
    g = parts["glass"]
    hi_x = g.bounding_box()[3]
    aw, ah = ACT_W / 2, ACT_H / 2
    mname = f"face_{tag}"
    if EMISSIVE:
        # looking down -x with z up, the viewer's left is -y: the image's top-left goes at -y, its width runs +y
        mat = (f'[material-emissive "{mname}" 0.006 0.006 0.008 0.0 0.04 1.0 1.0 1.0 3.0 "{UI}" '
               f'{hi_x:.3f} {dy - aw:.3f} {zc + ah:.3f} 0.0 {2 * aw:.3f} 0.0 0.0 0.0 {-2 * ah:.3f}]')
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
    allopts = dict(A=option_A, B=option_B, C=option_C, D=option_D)
    opts = [(t, allopts[t]) for t in os.environ.get("OPTS", "A,B,C").split(",")]
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
    if os.environ.get("ONLY"):
        jobs = [j for j in jobs if os.environ["ONLY"] in j[1]]
    for src, out, extra in jobs:
        r = subprocess.run([R, os.path.join(d, src)] + common + extra + ["-o", os.path.join(HERE, out)],
                           capture_output=True, text=True)
        print(out, "ok" if r.returncode == 0 else r.stderr[-500:])
    print(d)


if __name__ == "__main__":
    main()
