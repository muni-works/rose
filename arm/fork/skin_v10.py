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
    '[material "sheet_dark" 0.02 0.02 0.022 0.8 0.35]',    # 5052 plate, black anodized (shows only inside the ring pucks)
    '[material "cable_dark" 0.02 0.02 0.02 0.0 0.7]',      # black silicone harness
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


def perforate(m, x0, x1, zc, d=1.3, pitch=2.3, depth=2.5, skip=(), field=False, front=None, count_only=False):
    """The dot-grid (Rose: 1.1-1.4 mm holes on a 2.2-2.4 mm hex, only where air must pass): holes on the
    flat faces of a rounded-rectangle sleeve between x0 and x1 (mm), kept off the corner rounds.
    field=True (v10.5, rose): each face's grid sits in a rounded-rectangle field (corner 0.3 x its half
    height), edges parallel to the sleeve's; on the +-z (side) faces the field stops short of the `front`
    face (15 deg of the corner round + 3 mm) so the 3/4 silhouette edge stays solid.
    The sleeve stands in for a 2 mm wall, so the holes are drawn 2.5 mm deep into a dark cavity."""
    V = m.vertices
    sel = V[(V[:, 0] > x0) & (V[:, 0] < x1)]
    a, b = np.abs(sel[:, 1]).max(), np.abs(sel[:, 2] - zc).max()
    rc = RC * min(a, b)
    cyls, n = [], 0
    for face, half, other in ((1, a, b), (2, b, a)):     # +-y faces span z, +-z faces span y
        lim = other - rc - 1.5
        for sg in (-1, 1):
            if (face, sg) in skip:      # a face left clean, e.g. (1, -1) = the -y face
                continue
            u0, u1 = -lim, lim
            if field and face == 2 and front is not None:     # side faces: pull back from the front edge
                pull = rc * math.sin(math.radians(15)) + 3.0
                if front[1] < 0:
                    u0 += pull
                else:
                    u1 -= pull
            fx0, fx1 = x0 + 2, x1 - 2
            hu, hx = (u1 - u0) / 2, (fx1 - fx0) / 2
            uc, xc = (u0 + u1) / 2, (fx0 + fx1) / 2
            rf = RC * min(hu, hx)
            rows = np.arange(u0, u1 + 1e-6, pitch * math.sqrt(3) / 2)
            for i, u in enumerate(rows):
                off = (pitch / 2) * (i % 2)
                for x in np.arange(fx0 + off, fx1 + 1e-6, pitch):
                    if field:   # inside the rounded-rectangle field, a full hole radius in from its edge
                        dx, du = abs(x - xc) - (hx - rf), abs(u - uc) - (hu - rf)
                        if dx > 0 and du > 0 and math.hypot(dx, du) > rf - d / 2:
                            continue
                        if abs(x - xc) > hx - d / 2 or abs(u - uc) > hu - d / 2:
                            continue
                    n += 1
                    if count_only:
                        continue
                    c = np.array([x, 0.0, zc])
                    if face == 1:
                        c[1], c[2] = sg * half, zc + u
                        a0, a1 = c - [0, sg * depth, 0], c + [0, sg * 1.0, 0]
                    else:
                        c[2], c[1] = zc + sg * half, u
                        a0, a1 = c - [0, 0, sg * depth], c + [0, 0, sg * 1.0]
                    cyls.append(Manifold.cylinder(float(np.linalg.norm(a1 - a0)), d / 2, d / 2, 8)
                                .transform(_frame_to(a0, a1)))
    if count_only:
        return n * math.pi * (d / 2) ** 2
    holes = Manifold.batch_boolean(cyls, __import__('manifold3d').OpType.Add)
    return _tm(Manifold(_mm(m)) - holes), n


def _frame_to(a0, a1):
    """4x3 affine taking the +z unit cylinder at the origin to the segment a0 -> a1."""
    z = (a1 - a0) / np.linalg.norm(a1 - a0)
    x = np.cross(z, [1.0, 0, 0]) if abs(z[0]) < 0.9 else np.cross(z, [0, 1.0, 0])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.column_stack([x, y, z, a0]).astype(np.float32)


J3_OPEN = {}


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
    Rp = {'J2': F.R_LOBE_SH * 1e3 + 2.5, 'J4': F.R_LOBE_EL * 1e3 + 1.0, 'J6': F.R_LOBE6W * 1e3 + 2.5}   # v10.4 taper: J2 137, J4 126, J6 96 OD
    BEZEL = {'J2': 3.0, 'J4': 2.0, 'J6': 1.5}
    J = {'J2': np.array([0.0, 0, zs]), 'J4': np.array([U, 0, zs]), 'J6': np.array([Wm, 0, zs])}
    seam = lambda j: (J[j], 1, Rp[j] + GAP)
    out = []
    xr = lambda P, a, b: P[(P[:, 0] >= a) & (P[:, 0] <= b)]
    # ---- upper arm: widest at the deltoid (over J2 / J3), ~0.7 at the elbow
    P = xr(np.concatenate([Pw('upper'), Pw('upperd'), cab]), -80, U + 80)
    # v10.5 (rose's biceps): section 1.15 wide (lateral = arm z) : 1 deep (fore-aft = arm y), corner 0.3;
    # fullest 27 % down from J2, straight to 0.75 at the J3 seam, holding 0.72 to the elbow ring
    req = need(P, 0, 0.0, U, 13, seams=[seam('J2'), seam('J4')])
    def prof(s):
        s1 = 0.27 * U
        if s <= s1:
            return 1.0
        if s <= X3:
            return 1.0 + (0.75 - 1.0) * (s - s1) / (X3 - s1)
        return 0.75 + (0.72 - 0.75) * (s - X3) / (U - X3)
    a_full = max(max(r[1] / prof(r[0]) for r in req if r[1] > 0), max(r[2] / prof(r[0]) for r in req if r[2] > 0) / 1.15)
    B_IN = 55.0   # the sleeve's inner (torso-side, arm -z) half-extent cap: below the J2 seam the turret shell lives there
    st = []
    for s, a, b, _ in req:
        aa, bb = max(a_full * prof(s), a), max(1.15 * a_full * prof(s), b)
        cw = max(0.0, (bb - B_IN) / 2)            # fuller to the outside, held at B_IN on the inside
        st.append((s, aa, bb - cw, cw))
    ua = sleeve(st, origin=(0, 0, zs))
    for j in ('J2', 'J4'):
        ua = seam_cut(ua, *seam(j))
    lo = halves(ua, 0, X3 - 0.75)[0]
    hi = halves(ua, 0, X3 + 0.75)[1]
    x3a, x3b = (F.X3_CP + F.T_PL) * 1e3 + 4.0, X3 - 4.0
    # v10.5: J3's dot-grid on every face but the front - inner, outer, back - at the denser 1.4 / 2.2 hex (37 % open):
    # ~0.95 of the old four-face 29 % grid's open area, so J3 stays at ~20 % derate (thermal_v10 J3_GRID 0.95).
    # Front = body +x: arm -y on the left arm, +y on the right (ROSE_ARM_SIDE).
    front = (1, -1) if os.environ.get('ROSE_ARM_SIDE', 'l') == 'l' else (1, 1)
    J3_OPEN['ref'] = perforate(lo, x3a, x3b, zs, count_only=True)          # v10.3: 4 faces, 1.3 / 2.3, no field
    J3_OPEN['v105'] = perforate(lo, x3a, x3b, zs, d=1.4, pitch=2.2, skip=(front,), field=True, front=front, count_only=True)
    lo, nh3 = perforate(lo, x3a, x3b, zs, d=1.4, pitch=2.2, skip=(front,), field=True, front=front)
    out += [('upper', 'upper', loc(lo, 'upper'), 'skin'), ('upperd', 'upperd', loc(hi, 'upperd'), 'skin')]
    # v10.5: J3 is a 1.5 mm dark parting line with a 0.8 mm gunmetal hairline inset 0.6 mm (catches light only grazing)
    out.append(('ring-J3', 'upperd', loc(band(ua, 0, X3, width=0.8, grow=-0.6), 'upperd'), 'trim'))
    # ---- forearm: slimmer than the upper arm, tapering to the wrist
    P = xr(np.concatenate([Pw('fore'), Pw('yoke'), cab]), U - 80, Wm + 60)
    req = need(P, 0, U, Wm, 5, seams=[seam('J4'), seam('J6')])
    a0, b0 = max(r[1] for r in req[:2]), max(r[2] for r in req[:2])
    fa = sleeve(taper(req, a0, b0, 0.75 * a0, 0.7 * b0), origin=(0, 0, zs))
    for j in ('J4', 'J6'):
        fa = seam_cut(fa, *seam(j))
    out.append(('fore', 'fore', loc(fa.copy(), 'fore'), 'skin'))
    # ---- wrist: J6 to the tool flange; the one neon ring at J7
    P = xr(np.concatenate([Pw('wrist'), cab]), Wm - 60, xt + 3.2)
    req = need(P, 0, Wm, xt + 3.0, 4, seams=[seam('J6')])
    wa = seam_cut(sleeve(taper(req, req[1][1], req[1][2], req[-1][1], req[-1][2]), origin=(0, 0, zs)), *seam('J6'))
    wv, nh7 = perforate(wa, (S['x_ep'] + F.T_PL) * 1e3 + 4.0, S['xt'] * 1e3 - 8.0, zs)   # dot-grid over the J7 can
    out.append(('wrist', 'wrist', loc(wv, 'wrist'), 'skin'))
    c = np.array([xt, 0, zs])
    out.append(('ring-J7', 'wrist', loc(band(wa, 0, xt - 3.0, width=2.0, grow=0.4), 'wrist'), 'accent'))   # flush in the sleeve's own section
    # ---- pucks, flush with the widest sleeve that meets them, flush gunmetal hub caps
    half_w = lambda *ms: max(np.abs(m.vertices[:, 1]).max() for m in ms)
    def half_w_at(x0, *ms):   # v10.5: the sleeves' width where they meet this joint (within 30 mm of the seam)
        best = 0.0
        for m in ms:
            V = m.vertices
            near = np.abs(np.hypot(V[:, 0] - x0, V[:, 2] - zs) - (Rp_near.get(x0, 70.0))) < 30.0
            if near.any():
                best = max(best, np.abs(V[near, 1]).max())
        return best
    Rp_near = {0.0: Rp['J2'] + GAP, U: Rp['J4'] + GAP, Wm: Rp['J6'] + GAP}
    for j, owner, ms in (('J2', 'turret', (ua,)), ('J4', 'fore', (ua, fa)), ('J6', 'wrist', (fa, wa))):
        hy = half_w_at(J[j][0], *ms)
        c = J[j]
        # v10.3 (thermal_v10.py): the puck is a graphite RING; the motor's own dark face is open to air behind a
        # thin gunmetal bezel - a vented (1.3 / 2.3 hex) cap here costs J2 0.2 kg of hold, an open face costs none
        m_ = {'J2': F.J2M, 'J4': F.J4M, 'J6': F.J6M}[j]
        ri = m_['d'] / 2 * 1e3 + 1.5
        ring = Manifold(_mm(cyl(c - [0, hy, 0], c + [0, hy, 0], Rp[j], 128))) - Manifold(_mm(cyl(c - [0, hy + 1, 0], c + [0, hy + 1, 0], ri, 128)))
        # dot-grid round the ring's circumference (thermal_v10 BARREL = 1: the can's barrel sees air)
        holes = []
        rows = np.arange(-hy + 4.0, hy - 4.0 + 1e-6, 2.3 * math.sqrt(3) / 2)
        n_ = int(2 * math.pi * Rp[j] / 2.3)
        for i, yy in enumerate(rows):
            for k in range(n_):
                th = 2 * math.pi * (k + 0.5 * (i % 2)) / n_
                d_ = np.array([math.cos(th), 0.0, math.sin(th)])
                a0 = c + d_ * (ri - 1.0) + [0, yy, 0]
                a1 = c + d_ * (Rp[j] + 1.0) + [0, yy, 0]
                holes.append(Manifold.cylinder(float(np.linalg.norm(a1 - a0)), 0.65, 0.65, 8).transform(_frame_to(a0, a1)))
        ring = ring - Manifold.batch_boolean(holes, __import__('manifold3d').OpType.Add)
        out.append(('puck-' + j, owner, loc(_tm(ring), owner), 'puck'))
        for sg in (-1, 1):
            bz = Manifold(_mm(cyl(c + [0, sg * (hy - 0.5), 0], c + [0, sg * (hy + 0.4), 0], ri + BEZEL[j], 128))) - \
                Manifold(_mm(cyl(c + [0, sg * (hy - 1.0), 0], c + [0, sg * (hy + 1.0), 0], ri, 128)))
            out.append(('cap-%s%+d' % (j, sg), owner, loc(_tm(bz), owner), 'trim'))
    # v10.4 deltoid hood: the turret's shell carries on over the top third of the J2 ring (arm -x: up on the
    # humanoid, the shoulder line), 2.5 mm proud of the ring, so the shoulder grows out of rose's crown and less
    # drum shows from the front. It covers a third of J2's dot-grid (thermal_v10: J2 barrel 1.0 -> 0.8).
    hyJ = half_w(ua) + 1.0
    cJ = J['J2']
    hood = (Manifold(_mm(cyl(cJ - [0, hyJ, 0], cJ + [0, hyJ, 0], Rp['J2'] + 3.0, 128)))
            - Manifold(_mm(cyl(cJ - [0, hyJ + 1, 0], cJ + [0, hyJ + 1, 0], Rp['J2'] + 0.5, 128))))
    hood = hood ^ Manifold(_mm(trimesh.creation.box(extents=[200.0, 400.0, 400.0]).apply_translation(
        [cJ[0] - 100.0 - Rp['J2'] * 0.5, 0.0, cJ[2]])))
    if not F.PEDESTAL:   # body only: on the desk the arm needs J2's full backswing (the hood stops it at -75)
        out.append(('hood-J2', 'turret', loc(_tm(hood), 'turret'), 'skin'))
    # ---- turret / shoulder: J1 output up to the J2 axis, as wide as the J2 puck, its top inside the puck
    hy2 = half_w(ua)
    P = Pw('turret')
    P = P[P[:, 2] <= zs]
    r1 = max(np.hypot(P[:, 0], P[:, 1]).max(), F.J1M['d'] / 2 * 1e3 + 10) + MARGIN
    # the turret stops where the J2 seam circle starts: the upper sleeve can swing down past it
    # v10.4: one loft, asymmetric about the arm axis. Its "top" (arm -x: up on the humanoid at rest) continues
    # rose's shoulder line - 511 at the flank, a ~35 deg drop for 15 mm, then long and flat into the J2 puck;
    # its "bottom" (+x, where the upper sleeve swings) narrows to the keyhole tower's neck under the seam circle.
    ztop = zs - (Rp['J2'] + GAP) - 0.5
    wn = F.TOWER_NECK * 1e3 + MARGIN
    prof = [  # arm z, x_top (neg), x_bottom (pos)
        (zo, -r1, r1), (ztop, -(r1 - 3.0), r1 - 4.0), (zo + 15.0, -(Rp['J2'] + 1.3), wn),
        (zo + 35.0, -(Rp['J2'] + 0.8), wn), (zs, -(Rp['J2'] + 0.5), wn)]
    st = [(z_, hy2 + (4.0 if k == 0 else 2.0), (xb - xt) / 2, (xb + xt) / 2) for k, (z_, xt, xb) in enumerate(prof)]
    tu = seam_cut(sleeve(st, u=(0, 0, 1), v=(0, 1, 0), w=(1, 0, 0), rc_k=0.42), *seam('J2'))
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
        # body: J1's face flush on the torso flank as an OPEN graphite ring (r 61.5..66.5, 3 mm deep), the motor's
        # own face open to air inside it; rose's torso owns the gunmetal bezel round it (trim-j1ring, |y| 186.6)
        rj = F.J1M['d'] / 2 * 1e3 + 6.5
        ring = Manifold(_mm(cyl([0, 0, zo - 3.0], [0, 0, zo - GAP], rj, 128))) - Manifold(_mm(cyl([0, 0, zo - 4], [0, 0, zo], F.J1M['d'] / 2 * 1e3 + 1.5, 128)))
        out.append(('flank-J1', 'base', loc(_tm(ring), 'base'), 'puck'))
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
    key = (F.SIZE, F.UPPER, F.FORE, F.Z_SH, F.PEDESTAL, F.PED_H, os.environ.get('ROSE_ARM_SIDE', 'l'))
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
    # v10.3: with the skin on, the plates show only inside the open ring pucks: black-anodized there, and
    # the harness (silicone, black) reads as part of the joint, not an orange accent
    src = src.replace('"sheet"]', '"sheet_dark"]').replace('"cable"]', '"cable_dark"]')
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

# v10.3 skin and panel masses, for fork_checks (written to skin_mass_v10.json)
RHO_SKIN, WALL = 1010.0, 0.0012         # PA12 (MJF), 1.2 mm wall (exterior session's skin rule; CMF's 2 mm is the moulded PC/ABS limit)
PANELS = [  # (link, x0, x1 in mm world-zero, panel h mm, qty): 1 mm 5052 flank panels over the holder plates
    ('upper', 52.0, 149.0, 72.0, 2), ('upperd', 151.0, 245.0, 72.0, 2)]
PANEL_T, PAD_T, RHO_PAD = 0.0006, 0.00025, 3000.0   # 0.6 mm 5052 panel on a 0.25 mm gap pad
PANEL_FILL = 0.6      # v10.4: each panel windowed over its holder plate's own windows (40 % out; the plate isn't there either)
WALL_THIN = {k: 0.0009 for k in ('upper', 'upperd', 'fore', 'wrist', 'puck-J4', 'puck-J6')}   # v10.4: arm sleeves + distal rings 0.9 mm (MJF PA12, ribbed)
# v10.4 fastening, no new plate holes (FASTENING in V10.4.md): per part (link, kg, where)
FASTEN = [
    # sleeve halves: 2 printed C-clips per half snap over the holder plates' straight top / bottom edges
    # (3.175 plate), + 1 N52 6x3 magnet pair on the half-to-half seam: clips 0.4 g each, magnets 0.64 g each
    ('upper', 2 * (4 * 0.0004 + 2 * 0.00064), 0.100), ('upperd', 2 * (4 * 0.0004 + 2 * 0.00064), 0.200),
    ('fore', 2 * (4 * 0.0004 + 2 * 0.00064), 0.400), ('wrist', 2 * (4 * 0.0004 + 2 * 0.00064), 0.575),
    ('turret', 2 * (4 * 0.0004 + 2 * 0.00064), 0.0),
    # ring pucks: 3 M5 x 8 Al hex standoffs per face on the straddler plate A output screws (and plate B's
    # sleeve-flange screws), the bezel + ring screwed to them with M3 x 6: 1.1 g standoff + 0.4 g screw
    ('turret', 6 * 0.0015, 0.0), ('fore', 6 * 0.0015, 0.300), ('wrist', 6 * 0.0012, 0.500),
    # J1 drum (desk): 4 clips on the foot's rim; desk puck: sits under the pedestal base plate, captured
    ('base', 4 * 0.0004, 0.0),
    # J3 / J7 trim rings: VHB in a 0.3 mm groove, ~0
]


def skin_mass():
    """[(link, kg, x_m)] for the skin (2 mm wall over each part's outer area) + Al panels + gap pad."""
    rows = []
    for nm, link, m, mat in skin_meshes():
        if nm.startswith('puck'):   # ring: outer barrel + two end annuli (7 mm deep), before its dot-grid
            R_, W_ = m.extents[0] / 2, m.extents[1]
            A = (2 * math.pi * R_ * W_ + 2 * math.pi * (R_ ** 2 - (R_ - 7.0) ** 2)) * 1e-6
        elif mat in ('skin', 'vent'):
            A = m.convex_hull.area * 0.85 * 1e-6   # outer area only (hull: the dot-grid's hole walls don't count)
        elif nm.startswith('ring-'):   # a trim strip: perimeter x width (the stand-in is the sleeve's whole section)
            sec = m.area * 0.5
            A = 4 * math.sqrt(sec) * min(m.extents) * 1e-6
        else:
            A = m.area * 1e-6 * 0.5
        c = m.vertices.mean(0) / 1e3 + np.array(F.ORIGIN[link])
        rows.append((link, RHO_SKIN * WALL_THIN.get(nm, WALL) * A, float(c[0]), nm))
    for link, x0, x1, h, q in PANELS:
        a = (x1 - x0) * h * 1e-6 * q
        rows.append((link, (2680 * PANEL_T + RHO_PAD * PAD_T) * a * PANEL_FILL, (x0 + x1) / 2e3, 'panels'))
    for link, kg, x in FASTEN:
        rows.append((link, kg, x, 'fastening'))
    return rows


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--mass':
        import json
        R = skin_mass()
        json.dump([r[:3] for r in R], open(os.path.join(HERE, 'skin_mass_v10.json'), 'w'), indent=1)
        print('skin + panels: %.2f kg' % sum(r[1] for r in R))
        for r in R:
            print('  %-8s %-10s %5.0f g at x %.0f mm' % (r[0], r[3], r[1] * 1e3, r[2] * 1e3))
        sys.exit()
    d = sys.argv[1] if len(sys.argv) > 1 else '.'
    for nm, link, m, mat in skin_meshes():
        print('%-10s %-7s %6d faces  %s  extents %s' % (nm, link, len(m.faces), mat, np.round(m.extents, 1)))
