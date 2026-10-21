"""lymb v10 skin: the quad00 humanoid's design language over the fork arm, RENDER ONLY.

Port of quad00/skin.py's approach (read-only there): each segment's shell is a hull of superellipse
sections taken along the segment's own axis, each section the structure's extent in that slice plus a
2.5 mm standoff, with domed ends; every pitch joint whose axis crosses the shells gets a flush graphite
puck (r = 0.78 x the motor's), every roll joint a thin gunmetal ring at its parting plane, and the tool
roll the one orange accent ring - the humanoid's palette (satin-black covers, graphite pucks, gunmetal
trim, orange accent, aluminium plate). Nothing here changes a plate: the plates stay the structure, the
shells are stand-ins for printed (MJF PA12) covers on them, and they are not in any check.

Segments (link frame, v10):
  base      a drum over the J1 RS04 and its collar        pedestal  a square shell over the desk tube
  turret    over the disc and tower plates (the shoulder) upper     J2 lobe -> the J3 cartridge (a bicep)
  upperd    J3 output -> the elbow axis                    fore      the elbow axis -> the wrist axis (+ yoke)
  wrist     the wrist axis -> the tool flange (over J7)

Used by render_v10.py:  frag(q, at, R0, prefix, outdir) -> loon text (the arm + its skin, no header).
Needs numpy, trimesh, manifold3d (run under uv like quad00).
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import numpy as np
import trimesh
from manifold3d import Manifold
import rose_fork as F
import rose_arm_geom as Gm

MARGIN = 2.5          # mm, shell standoff (quad00 skin.MARGIN)
SKIN_MATS = '\n'.join([
    '[material "skin" 0.016 0.012 0.009 0.0 0.45]',       # warm black #1e1813 (sRGB -> linear), satin (cortex CMF)
    '[material "puck" 0.06 0.062 0.066 0.6 0.38]',         # graphite
    '[material "trim" 0.30 0.32 0.34 0.85 0.32]',          # gunmetal
    '[material "accent" 0.224 1.0 0.078 0.0 0.4]',         # Make green #39ff14 - the one neon, at the wrist
]) + '\n'


# ---------------------------------------------------------------- quad00 skin.py, ported
def _ring(c, v, w, a, b, n=2.6, k=48):
    t = np.linspace(0, 2 * np.pi, k, endpoint=False)
    ct, st = np.cos(t), np.sin(t)
    x = a * np.sign(ct) * np.abs(ct) ** (2 / n)
    y = b * np.sign(st) * np.abs(st) ** (2 / n)
    return c[None, :] + x[:, None] * v[None, :] + y[:, None] * w[None, :]


def loft(P, u, n=2.6, slices=12, margin=MARGIN, dome=(True, True), dome_k=0.28):
    """A smooth shell over points P (mm) along axis u: per-slice extent + margin, superellipse n."""
    u = np.asarray(u, float) / np.linalg.norm(u)
    y = np.array([0, 1.0, 0]) if abs(u[1]) < 0.9 else np.array([0, 0, 1.0])
    v = y - u * (y @ u)
    v /= np.linalg.norm(v)
    w = np.cross(u, v)
    pu, pv, pw = P @ u, P @ v, P @ w
    u0, u1 = pu.min(), pu.max()
    edges = np.linspace(u0, u1, slices + 1)
    rings = []
    sec = []
    for i in range(slices):
        lo, hi = edges[i], edges[i + 1]
        m = (pu >= lo - 1e-6) & (pu <= hi + 1e-6)
        if m.sum() < 3:
            continue
        vc, wc = (pv[m].min() + pv[m].max()) / 2, (pw[m].min() + pw[m].max()) / 2
        a, b = (pv[m].max() - pv[m].min()) / 2 + margin, (pw[m].max() - pw[m].min()) / 2 + margin
        sec.append((lo, hi, vc, wc, a, b))
        for uu in (lo, hi):
            rings.append(_ring(uu * u + vc * v + wc * w, v, w, a, b, n))
    for end, sgn, on in ((0, -1, dome[0]), (-1, 1, dome[1])):
        lo, hi, vc, wc, a, b = sec[end]
        uu = lo if sgn < 0 else hi
        dz = max(margin, dome_k * min(a, b)) if on else margin
        for f, d in (((0.96, 0.3), (0.86, 0.6), (0.66, 0.85), (0.38, 1.0)) if on else ((0.94, 1.0),)):
            rings.append(_ring((uu + sgn * dz * d) * u + vc * v + wc * w, v, w, a * f, b * f, n))
    H = Manifold.hull_points(np.concatenate(rings).astype(np.float32))
    g = H.to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=True)


# ---------------------------------------------------------------- the arm's segments
def link_points(link, mats=None, step=0.006):
    """Structure points of one link at the zero pose, in mm, relative to the link's ORIGIN."""
    o = np.array(F.ORIGIN[link]) * 1e3
    P = []
    for p in F.build():
        if p[0] != link or (mats and p[1] not in mats):
            continue
        P += Gm.prim_points(p[:7], step=step)
    return np.array(P) * 1e3 - o if P else np.zeros((0, 3))


def cyl(a, b, r, sections=96):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return trimesh.creation.cylinder(radius=r, segment=[a, b], sections=sections)


def _tm(M):
    g = M.to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts))


def halves(mesh, ax, pos):
    """Split a shell at the plane coordinate[ax] = pos: (below, above)."""
    M = Manifold(_mm(mesh))
    out = []
    for sgn in (-1, 1):
        ext = [2000.0] * 3
        ctr = [0.0, 0.0, 0.0]
        ctr[ax] = pos + sgn * 1000.0
        out.append(_tm(M ^ Manifold(_mm(trimesh.creation.box(extents=ext).apply_translation(ctr)))))
    return out


GAP = 1.0            # mm, parting gap between a sleeve and its joint's puck (constant: the seam is coaxial)
RC = 0.30            # corner radius / half-depth (cortex plan corner)


def rr_ring(c, v, w, a, b, rc, k=10):
    """Rounded rectangle, half sizes a (along v) and b (along w), corner radius rc."""
    rc = max(0.5, min(rc, a - 0.1, b - 0.1))
    pts = []
    for cx, cy, a0 in ((a - rc, b - rc, 0.0), (-(a - rc), b - rc, 90.0), (-(a - rc), -(b - rc), 180.0), ((a - rc), -(b - rc), 270.0)):
        for t in np.linspace(0, 90, k):
            r = math.radians(a0 + t)
            pts.append((cx + rc * math.cos(r), cy + rc * math.sin(r)))
    P = np.array(pts)
    return c[None, :] + P[:, :1] * v[None, :] + P[:, 1:] * w[None, :]


def sleeve(stations, u=(1, 0, 0), v=(0, 1, 0), w=(0, 0, 1), sub=6, rc_k=RC, origin=(0.0, 0.0, 0.0)):
    """A lofted (not hulled) sleeve: rounded-rectangle stations (s, a, b, cw) - a along v, b along w -
    smoothstep-interpolated, stitched ring to ring, capped flat. Watertight."""
    u, v, w = (np.asarray(x, float) for x in (u, v, w))
    org = np.asarray(origin, float)
    st = sorted(stations)
    rings = []
    for (s0, a0, b0, c0), (s1, a1, b1, c1) in zip(st, st[1:]):
        for k in range(sub):
            t = k / sub
            e = t * t * (3 - 2 * t)
            a, b, cw = a0 + (a1 - a0) * e, b0 + (b1 - b0) * e, c0 + (c1 - c0) * e
            rings.append(rr_ring(org + (s0 + (s1 - s0) * t) * u + cw * w, v, w, a, b, rc_k * min(a, b)))
    s1, a1, b1, c1 = st[-1]
    rings.append(rr_ring(org + s1 * u + c1 * w, v, w, a1, b1, rc_k * min(a1, b1)))
    n = len(rings[0])
    V = np.concatenate(rings + [rings[0].mean(0)[None, :], rings[-1].mean(0)[None, :]])
    F_ = []
    for r in range(len(rings) - 1):
        for k in range(n):
            a_, b_ = r * n + k, r * n + (k + 1) % n
            F_ += [(a_, b_, b_ + n), (a_, b_ + n, a_ + n)]
    c0, c1 = len(V) - 2, len(V) - 1
    last = (len(rings) - 1) * n
    for k in range(n):
        F_.append((c0, (k + 1) % n, k))
        F_.append((c1, last + k, last + (k + 1) % n))
    m = trimesh.Trimesh(V, np.array(F_), process=True)
    trimesh.repair.fix_normals(m)
    return m


def need(P, ax, s0, s1, nst, perp=(1, 2), seams=()):
    """Per-station half-extents (a: |perp0| max, b: perp1 half-span) of points P (mm), outside the seam
    cylinders (centre, axis index, radius) that the pucks own."""
    keep = np.ones(len(P), bool)
    for c, cax, r in seams:
        o_ = [i for i in range(3) if i != cax]
        keep &= np.hypot(P[:, o_[0]] - c[o_[0]], P[:, o_[1]] - c[o_[1]]) > r
    Q = P[keep]
    out = []
    h = (s1 - s0) / (nst - 1) * 0.6
    for s in np.linspace(s0, s1, nst):
        m = np.abs(Q[:, ax] - s) < h
        if m.sum() < 3:
            out.append((s, 0.0, 0.0, 0.0))
            continue
        out.append((s, np.abs(Q[m, perp[0]]).max() + MARGIN, np.abs(Q[m, perp[1]] - F.Z_SH * 1e3 * (perp[1] == 2)).max() + MARGIN, 0.0))
    return out


def taper(req, a0, b0, a1, b1):
    """Stations on a straight taper a0,b0 -> a1,b1, each raised only as far as the structure needs."""
    s0, s1 = req[0][0], req[-1][0]
    return [(s, max(a0 + (a1 - a0) * (s - s0) / (s1 - s0), a), max(b0 + (b1 - b0) * (s - s0) / (s1 - s0), b), 0.0)
            for s, a, b, _ in req]


def seam_cut(m, c, ax, r, L=600.0):
    a = np.zeros(3)
    a[ax] = L / 2
    return _tm(Manifold(_mm(m)) - Manifold(_mm(cyl(np.asarray(c) - a, np.asarray(c) + a, r, 128))))


def skin_parts():
    """v10.2 (Rose's language): limbs are graphite pucks + slim tapered sleeves, no pods.

    Each sleeve is a loft of rounded-rectangle stations (corner 0.30 x the half-depth) on a straight
    taper, raised station by station only where the structure needs it. At every pitch joint both
    sleeves are cut back by ONE cylinder coaxial with the joint, r = puck + 1 mm: the 1 mm parting gap
    is invariant under the rotation, so no sliver or pinch can appear anywhere in the range. The puck is
    as wide as the sleeves (flush) with a flush gunmetal hub cap. The upper arm is one loft split by a
    1.5 mm gap at J3 with a gunmetal ring in it (over J3's bolt circle). The cable is inside the sleeves.
    """
    o = lambda l: np.array(F.ORIGIN[l]) * 1e3
    Pw = lambda l: link_points(l) + o(l)
    zs = F.Z_SH * 1e3
    S = F.stations()
    X3, U, Wm, xt = F.X3_OUT * 1e3, F.UPPER * 1e3, F.W * 1e3, S['xt'] * 1e3
    zo = (F.T_FOOT + F.J1M['L']) * 1e3
    loc = lambda m, l: m.apply_translation(-o(l))
    cab = np.array(F.cable_points(F.fk({}))) * 1e3
    cab = np.concatenate([cab + [0, 0, d] for d in (-F.R_CABLE * 1e3, F.R_CABLE * 1e3)])
    for xj in (0.0, F.UPPER * 1e3, F.W * 1e3):     # the joint crossings live under the pucks
        cab = cab[np.hypot(cab[:, 0] - xj, cab[:, 2] - zs) > 85.0]
    Rp = {'J2': F.R_LOBE_SH * 1e3 + 2.5, 'J4': F.R_LOBE_EL * 1e3 + 2.5, 'J6': F.R_LOBE6W * 1e3 + 2.5}
    J = {'J2': np.array([0.0, 0, zs]), 'J4': np.array([U, 0, zs]), 'J6': np.array([Wm, 0, zs])}
    seam = lambda j: (J[j], 1, Rp[j] + GAP)
    out = []
    xr = lambda P, a, b: P[(P[:, 0] >= a) & (P[:, 0] <= b)]
    # ---- upper arm: widest at the deltoid (over J2 / J3), ~0.7 at the elbow
    P = xr(np.concatenate([Pw('upper'), Pw('upperd'), cab]), -80, U + 80)
    req = need(P, 0, 0.0, U, 6, seams=[seam('J2'), seam('J4')])
    a_max = max(r[1] for r in req)
    b_max = max(r[2] for r in req)
    ua = sleeve(taper(req, a_max, b_max, 0.7 * a_max, 0.7 * b_max), origin=(0, 0, zs))
    for j in ('J2', 'J4'):
        ua = seam_cut(ua, *seam(j))
    lo = halves(ua, 0, X3 - 0.75)[0]
    hi = halves(ua, 0, X3 + 0.75)[1]
    out += [('upper', 'upper', loc(lo, 'upper'), 'skin'), ('upperd', 'upperd', loc(hi, 'upperd'), 'skin')]
    out.append(('ring-J3', 'upperd', loc(band(ua, 0, X3, width=1.5, grow=-0.6), 'upperd'), 'trim'))
    # ---- forearm: slimmer than the upper arm, tapering to the wrist
    P = xr(np.concatenate([Pw('fore'), Pw('yoke'), cab]), U - 80, Wm + 60)
    req = need(P, 0, U, Wm, 5, seams=[seam('J4'), seam('J6')])
    a0, b0 = max(r[1] for r in req[:2]), max(r[2] for r in req[:2])
    fa = sleeve(taper(req, a0, b0, 0.75 * a0, 0.7 * b0), origin=(0, 0, zs))
    for j in ('J4', 'J6'):
        fa = seam_cut(fa, *seam(j))
    out.append(('fore', 'fore', loc(fa, 'fore'), 'skin'))
    # ---- wrist: J6 to the tool flange; the one neon ring at J7
    P = xr(np.concatenate([Pw('wrist'), cab]), Wm - 60, xt + 3.2)
    req = need(P, 0, Wm, xt + 3.0, 4, seams=[seam('J6')])
    wa = seam_cut(sleeve(taper(req, req[1][1], req[1][2], req[-1][1], req[-1][2]), origin=(0, 0, zs)), *seam('J6'))
    out.append(('wrist', 'wrist', loc(wa, 'wrist'), 'skin'))
    c = np.array([xt, 0, zs])
    out.append(('ring-J7', 'wrist', loc(band(wa, 0, xt - 3.0, width=2.0, grow=0.4), 'wrist'), 'accent'))   # flush in the sleeve's own section
    # ---- pucks, flush with the widest sleeve that meets them, flush gunmetal hub caps
    half_w = lambda *ms: max(np.abs(m.vertices[:, 1]).max() for m in ms)
    for j, owner, ms in (('J2', 'turret', (ua,)), ('J4', 'fore', (ua, fa)), ('J6', 'wrist', (fa, wa))):
        hy = half_w(*ms)
        c = J[j]
        out.append(('puck-' + j, owner, loc(cyl(c - [0, hy, 0], c + [0, hy, 0], Rp[j], 128), owner), 'puck'))
        for sg in (-1, 1):
            out.append(('cap-%s%+d' % (j, sg), owner, loc(cyl(c + [0, sg * hy, 0], c + [0, sg * (hy + 0.4), 0], Rp[j] * 0.55, 96), owner), 'trim'))
    # ---- turret / shoulder: J1 output up to the J2 axis, as wide as the J2 puck, its top inside the puck
    hy2 = half_w(ua)
    P = Pw('turret')
    P = P[P[:, 2] <= zs]
    r1 = max(np.hypot(P[:, 0], P[:, 1]).max(), F.J1M['d'] / 2 * 1e3 + 10) + MARGIN
    # the turret stops where the J2 seam circle starts: the upper sleeve can swing down past it
    ztop = zs - (Rp['J2'] + GAP) - 0.5
    st = [(zo, r1, hy2 + 4, 0.0), (ztop, r1 - 4, hy2 + 2, 0.0)]
    tu = sleeve(st, u=(0, 0, 1), v=(1, 0, 0), w=(0, 1, 0), rc_k=0.42)
    out.append(('turret', 'turret', loc(tu, 'turret'), 'skin'))
    # ---- J1: desk - a graphite drum growing out of one calm base puck; body - a flush disc on the flank
    if F.PEDESTAL:
        out.append(('drum-J1', 'base', loc(cyl([0, 0, F.T_FOOT * 1e3], [0, 0, zo - GAP], r1, 128), 'base'), 'puck'))
        ph, rb, top, rr = F.PED_H * 1e3, max(r1 + 8.0, math.hypot(F.PED_S / 2, F.PED_S / 2) * 1e3 + 3.0), F.T_FOOT * 1e3, 14.0
        prof = [(0.0, -ph), (rb, -ph)] + [(rb - rr + rr * math.cos(math.radians(t)), top - rr + rr * math.sin(math.radians(t)))
                                          for t in np.linspace(0, 90, 12)] + [(0.0, top)]
        base = trimesh.creation.revolve(np.array(prof), sections=128)
        out.append(('desk-puck', 'base', loc(base, 'base'), 'skin'))
    else:
        rj = F.J1M['d'] / 2 * 1e3 + 6
        out.append(('flank-J1', 'base', loc(cyl([0, 0, zo - 3.0], [0, 0, zo - GAP], rj, 128), 'base'), 'puck'))
        ring = Manifold(_mm(cyl([0, 0, zo - 3.0], [0, 0, zo - GAP], rj + 3.0, 128))) - Manifold(_mm(cyl([0, 0, zo - 4], [0, 0, zo], rj + 0.6, 128)))
        out.append(('flank-ring', 'base', loc(_tm(ring), 'base'), 'trim'))
    return out


def band(shell, ax, pos, width=5.0, grow=1.0):
    """A trim band: the shell's own section at `pos` along axis ax, 1 mm proud, `width` wide
    (quad00's accent-chestline construction)."""
    c = shell.vertices.mean(0)
    big = shell.copy().apply_translation(-c)
    big.apply_scale(1.0 + grow / max(shell.extents.min() / 2, 1)).apply_translation(c)
    ext = [600.0, 600.0, 600.0]
    ext[ax] = width
    ctr = c.copy()
    ctr[ax] = pos
    box = trimesh.creation.box(extents=ext).apply_translation(ctr)
    g = (Manifold.from_mesh(_mm(big)) ^ Manifold.from_mesh(_mm(box))).to_mesh() if hasattr(Manifold, 'from_mesh') else (Manifold(_mm(big)) ^ Manifold(_mm(box))).to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts))


def _mm(t):
    from manifold3d import Mesh
    return Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32))


_CACHE = {}


def skin_meshes():
    key = (F.SIZE, F.UPPER, F.FORE, F.Z_SH, F.PEDESTAL, F.PED_H)
    if key not in _CACHE:
        _CACHE[key] = skin_parts()
    return _CACHE[key]


def write_stls(outdir, tag):
    os.makedirs(outdir, exist_ok=True)
    files = []
    for nm, link, m, mat in skin_meshes():
        import hashlib
        h = hashlib.md5(np.round(m.vertices, 3).tobytes()).hexdigest()[:8]   # vcad caches meshes by path: name by content
        fn = '%s-%s-%s.stl' % (tag, nm, h)
        m.export(os.path.join(outdir, fn))
        files.append((nm, link, fn, mat))
    return files


def frag(q, at=(0.0, 0.0, 0.0), R0=Gm.I3, prefix='a', outdir='.', skin=True, skip=(), stl_tag=None):
    """The posed arm and (skin=True) its shells as loon, no header. STLs land in outdir."""
    sk = set(skip) | ({(l, 'cap') for l in ('turret', 'fore', 'wrist')} if skin else set())   # the skin's own hub caps replace these
    src = F.world_loon(q, at=at, prefix=prefix, header=False, R0=R0, skip=sk, cable='cable' not in skip)
    if not skin:
        return src
    tag = stl_tag or ('skin-%s' % F.SIZE)
    T0 = F.fk(q)
    T = {k: (Gm.mm_(R0, Rk), Gm.add(Gm.mv(R0, tk), at)) for k, (Rk, tk) in T0.items()}
    L, roots = [], []
    import hashlib
    os.makedirs(outdir, exist_ok=True)
    for nm, link, m, mat in skin_meshes():
        if link in skip or (link, 'skin') in skip or nm in skip:
            continue
        # posed here, in world mm: vcad composes [rotate] differently for imported meshes than for
        # sketches, so the shells are written already in place and imported untransformed
        R, t_ = T[link]
        V = m.vertices @ np.array(R).T + np.array(t_) * 1e3
        h = hashlib.md5(np.round(V, 3).tobytes()).hexdigest()[:10]      # vcad caches meshes by path
        fn = '%s-%s-%s.stl' % (tag, nm, h)
        if not os.path.exists(os.path.join(outdir, fn)):
            trimesh.Trimesh(V, m.faces, process=False).export(os.path.join(outdir, fn))
        key = '%ss_%s' % (prefix, nm.replace('-', '_').replace('+', 'p'))
        L.append('[let %s [import-mesh-scaled 0.001 0.001 0.001 "%s"]]' % (key, fn))
        roots.append('[root %s "%s"]' % (key, mat))
    return src + '\n'.join(L + [''] + roots) + '\n'


_WRITTEN = {}

if __name__ == '__main__':
    d = sys.argv[1] if len(sys.argv) > 1 else '.'
    for nm, link, m, mat in skin_meshes():
        print('%-10s %-7s %6d faces  %s  extents %s' % (nm, link, len(m.faces), mat, np.round(m.extents, 1)))
