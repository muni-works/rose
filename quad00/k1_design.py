"""MINI humanoid, our own look on the K1's joint layout: every joint a rounded coaxial housing around OUR
RobStride (graphite caps in both ends), every link a bone lofted between its joints' housings (slimmer
than the joints, a swell for thigh / calf / forearm), the hip pitch-roll-yaw cluster merged into one lobe,
our own chest + pelvis blocks with the accent waist, an egg helmet with a visor band, shoes, mittens.
K1 frame: x fore, y left, z up (mm). Joint positions + axes from the K1 URDF (arms lowered)."""
import math
import numpy as np
import k1_structure as KS, trimesh
from manifold3d import CrossSection, FillRule, Manifold, Mesh, OpType, JoinType
import k1 as K, k1_teardown as KT, quad as Q
from design import ring, tm, M_, cyl

PAD, FIL = 4.0, 7.0          # housing: motor + 4 mm wall, 7 mm edge round


def unit(v):
    v = np.asarray(v, float); return v / np.linalg.norm(v)


def perp(a):
    t = np.array([0, 0, 1.0]) if abs(a[2]) < 0.9 else np.array([1.0, 0, 0])
    v = unit(np.cross(a, t)); return v, np.cross(a, v)


def housing(p, ax, mot, pad=PAD, fil=FIL, extra_len=6.0):
    """a rounded drum around a motor: hull of a full-radius ring set back from each face + a smaller one at it."""
    m = Q.MOTORS[mot]
    a = unit(ax); v, w = perp(a)
    r, L = m['D'] / 2 + pad, m['L'] + extra_len
    pts = []
    for s in (-1, 1):
        pts.append(ring(np.asarray(p) + a * s * (L / 2 - fil), v, w, r, r, n=2.0, k=72))
        pts.append(ring(np.asarray(p) + a * s * (L / 2 - fil * 0.3), v, w, r - fil * 0.55, r - fil * 0.55, n=2.0, k=72))
        pts.append(ring(np.asarray(p) + a * s * (L / 2), v, w, r - fil, r - fil, n=2.0, k=72))
    body = Manifold.hull_points(np.concatenate(pts).astype(np.float32))
    caps = [cyl(np.asarray(p) + a * s * (L / 2 + 0.3), a, (r - fil) * 0.82, 1.6) for s in (-1, 1)]
    return tm(body), trimesh.util.concatenate(caps), r, L


def SHOULDER_FLANK():
    """half-width of the chest at the shoulder line: the shoulder-pitch disc sits flush in it (full-size H2/T2 language)."""
    return 124.0 * KS.R() * KS.V.get('ky', 1.0)


def _circle(c, u, v, r, n=48):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return c[None, :] + r * (np.cos(t)[:, None] * u[None, :] + np.sin(t)[:, None] * v[None, :])


def hull_rings(rings, **meta):
    """the shell = hull of its section rings; the rings ride along (metadata) so exporters can loft them natively."""
    m = tm(Manifold.hull_points(np.concatenate(rings).astype(np.float32)))
    m.metadata['loft'] = [np.asarray(r, float) for r in rings]
    m.metadata.update(meta)
    return m


JOINT_CAPS = False                     # the orange joint caps (pucks); off since 2026-10-20
GROOVE_D, GROOVE_W = 1.2, 1.6          # parting lines: depth into the skin, width


def inset_rings(rings, d):
    """each section ring pulled in by d on both of its half-axes (an inset copy of a ring-built shell)."""
    out = []
    for r in rings:
        r = np.asarray(r, float); c = r.mean(0)
        _, _, vt = np.linalg.svd(r - c)
        u, v = vt[0], vt[1]
        x, y = (r - c) @ u, (r - c) @ v
        hx, hy = max(np.abs(x).max(), d + 0.5), max(np.abs(y).max(), d + 0.5)
        out.append(c + np.outer(x * (hx - d) / hx, u) + np.outer(y * (hy - d) / hy, v))
    return out


def _axes(a):
    """(axis, u, v) world unit vectors for a band along axis index a."""
    E = np.eye(3)
    return E[a], E[(a + 1) % 3], E[(a + 2) % 3]


def band_manifold(spec):
    """the region a parting line runs through: ('frame', axis, centre, half_u, half_v, rc) = a rounded-rect outline
    GROOVE_W wide, as a prism right through the part along axis; ('slab', axis, at) = a GROOVE_W slab across it."""
    if spec[0] == 'slab':
        _, a, at = spec
        lo = np.full(3, -1e3); sz = np.full(3, 2e3); lo[a] = at - GROOVE_W / 2; sz[a] = GROOVE_W
        return Manifold.cube(list(sz)).translate(list(lo))
    _, a, c, hu, hv, rc = spec
    ax, u, v = _axes(a)
    def prism(hu_, hv_, rc_):
        pts = rrect(np.zeros(3), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), hu_, hv_, rc_)[:, :2]
        cs = __import__('manifold3d').CrossSection([pts.tolist()])
        m = Manifold.extrude(cs, 600.0).translate([0, 0, -300.0])
        R = np.column_stack([u, v, ax])                    # local (u, v, axis) -> world
        return m.transform(np.hstack([R, np.asarray(c, float).reshape(3, 1)]))
    return prism(hu, hv, rc) - prism(hu - GROOVE_W, hv - GROOVE_W, max(rc - GROOVE_W, 1.0))


def cut_shell(m, cuts):
    """pocket cuts into a ring-built shell (vent slots, a mounting hole): boxes and x-axis cylinders, kept as loon cuts."""
    out_m = M_(m)
    for c in cuts:
        if c[0] == 'box':
            out_m = out_m - Manifold.cube(list(c[2])).translate(list(c[1]))
        elif c[0] == 'ycyl':                                    # ('ycyl', centre, radius[, half length]): a bore along y
            p, r = c[1], c[2]; hl = c[3] if len(c) > 3 else r
            out_m = out_m - Manifold.cylinder(2 * hl, r, r, 72).rotate([-90, 0, 0]).translate([p[0], p[1] - hl, p[2]])
        else:                                                   # ('xcyl', centre on the face, radius, depth)
            _, p, r, dep = c
            out_m = out_m - Manifold.cylinder(dep * 2, r, r, 48).rotate([0, 90, 0]).translate([p[0] - dep, p[1], p[2]])
    o = tm(out_m)
    o.metadata = dict(m.metadata, cut=list(m.metadata.get('cut', [])) + list(cuts))
    return o


def grooved(m, specs):
    """cut parting lines into a ring-built shell: shell minus (bands ^ skin), skin = shell minus its inset copy."""
    rings = m.metadata['loft']
    L = M_(m)
    skin = L - Manifold.hull_points(np.concatenate(inset_rings(rings, GROOVE_D)).astype(np.float32))
    bands = Manifold.batch_boolean([band_manifold(s) for s in specs], OpType.Add)
    out = tm(L - (bands ^ skin))
    out.metadata = dict(m.metadata, grooves=list(specs))
    return out


def rrect(c, v, w, a, b, rc, k=10):
    """a rounded rectangle section: flat sides, corner radius rc."""
    rc = min(rc, a * 0.98, b * 0.98)
    pts = []
    for sx, sy, t0 in ((1, 1, 0.0), (-1, 1, 0.5 * np.pi), (-1, -1, np.pi), (1, -1, 1.5 * np.pi)):
        t = np.linspace(t0, t0 + 0.5 * np.pi, k)
        x = sx * (a - rc) + rc * np.cos(t); y = sy * (b - rc) + rc * np.sin(t)
        pts.append(c[None, :] + x[:, None] * v[None, :] + y[:, None] * w[None, :])
    return np.concatenate(pts)


def bone(p0, r0, p1, r1, swell=1.0, flat=1.0, lateral=(0, 1.0, 0), n=2.2, half_w=None, depth=None):
    """a link lofted from a section at p0 to one at p1 (radii r0, r1), with a swell at 40 %.
    depth=[(t, r), ...] sculpts it instead: the fore/aft depth follows that profile, the width stays at half_w."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    u = unit(p1 - p0)
    v = unit(np.asarray(lateral, float) - u * (np.asarray(lateral) @ u)); w = np.cross(u, v)
    rings = []
    if depth:
        L_ = np.linalg.norm(p1 - p0)
        for t, r in depth:
            a = half_w if half_w is not None else r * flat
            rings.append(rrect(p0 + u * t * L_, v, w, a, r, 0.4 * min(a, r)))
        return hull_rings(rings)
    for t, k in ((0.0, 1.0), (0.18, 1.0 + 0.6 * (swell - 1)), (0.42, swell), (0.7, 1.0 + 0.5 * (swell - 1)), (1.0, 1.0)):
        r = r0 + (r1 - r0) * t
        a = r * k * flat if half_w is None else min(r * k * flat, half_w)   # never wider than the joint drums' faces
        rings.append(rrect(p0 + u * t * np.linalg.norm(p1 - p0), v, w, a, r * k, 0.4 * min(a, r * k)))
    return hull_rings(rings)


def block(x0, x1, y, z0, z1, n=2.8, taper=(1.0, 1.0), dome=0.0, RC=22.0, taper_top=None, trap=None, flare=None):
    """a rounded torso block: superellipse sections (x fore, y side) stacked up z, tapered."""
    rings = []
    for t in np.linspace(0, 1, 9):
        z = z0 + (z1 - z0) * t
        tt = t if taper_top is None else min(1.0, (z - z0) / (taper_top - z0))     # taper_top: full width from here up
        k = taper[0] + (taper[1] - taper[0]) * tt
        if flare:                                               # broad shoulders: the flanks flare out to the shoulder line
            zf0, zf1, kf = flare
            ks = 1.0 + (kf - 1.0) * min(1.0, max(0.0, (z - zf0) / (zf1 - zf0)))
        else:
            ks = 1.0
        rings.append(rrect(np.array([(x0 + x1) / 2, 0, z]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), y * k * ks, (x1 - x0) / 2 * (0.94 + 0.06 * k), RC))
    if dome:
        for f, d in ((0.9, 0.4), (0.7, 0.75), (0.4, 1.0)):
            fy = f if trap is None else f * trap               # trapezius: the top slopes down to the shoulders
            rings.append(rrect(np.array([(x0 + x1) / 2, 0, z1 + dome * d]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), y * taper[1] * fy * (flare[2] if flare else 1.0), (x1 - x0) / 2 * f, RC * f))
    return hull_rings(rings)


def torso_shell(lo, hi, CH, z0, z_ch, zc1, zs, flank, Rk):
    """the sleek one-piece torso (head language): near-vertical sides that funnel out late to the shoulder line,
    big plan radii and a head-style crown fillet. Piecewise hulls, so the concave flanks survive (export lofts the rings)."""
    ss = lambda a, b, z: (lambda u: u * u * (3 - 2 * u))(min(1.0, max(0.0, (z - a) / (b - a))))
    zf = zs - 40.0 * Rk                                         # the armpit: the flare ends at the shoulder drum's bottom edge
    def sec(z):
        zh = min(z0 + 88.0 * Rk, z0 + 0.6 * (zf - z0))          # full width from here up (the side bays); short chests scale it
        if z < zh:                                              # a skinnier waist, swelling smoothly to the chest
            k = 0.80 + (0.97 - 0.80) * ss(z0, zh, z)
        else:                                                   # a concave funnel: near-vertical, flaring late out over the shoulder drums
            u = min(1.0, max(0.0, (z - zh) / (zf - zh)))       # full width by the drum's bottom: the side wraps the whole drum
            k = 0.97 + (flank / CH - 0.97) * u ** 3
        x0 = lo[0] + 10 - 6 * ss(z0, z_ch, z); x1 = hi[0] - 12 + 6 * ss(z0, z_ch, z)
        lean = KS.V.get('torso_lean', 0.0) * ss(zs - 30.0 * Rk, zc1 + 10.0 * Rk, z)   # front + back lean in toward the top
        xm, h = (x0 + x1) / 2, (x1 - x0) / 2
        return xm, CH * k, h * (1 - lean)
    rf = 16.0 * Rk                                              # crown fillet, like the head's
    zt = zc1 + 10.0 * Rk
    rings = []
    rbt = 14.0 * Rk                                             # bottom round-off, like the crown
    xc, a_, b_ = sec(z0 + rbt)
    for th in np.linspace(0, np.pi / 2, 7)[:-1]:
        d_ = rbt * (1 - math.sin(th))
        rings.append(rrect(np.array([xc, 0, z0 + rbt - rbt * math.cos(th)]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), a_ - d_, b_ - d_, max(24.0 * Rk - d_, 3.0), k=14))
    for z_ in np.linspace(z0 + rbt, zt - rf, 16):
        xc, a_, b_ = sec(z_)
        rings.append(rrect(np.array([xc, 0, z_]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), a_, b_, 24.0 * Rk, k=14))
    xc, a_, b_ = sec(zt - rf)
    for th in np.linspace(0, np.pi / 2, 7)[1:]:
        d_ = rf * (1 - math.cos(th))
        rings.append(rrect(np.array([xc, 0, zt - rf + rf * math.sin(th)]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), a_ - d_, b_ - d_, max(24.0 * Rk - d_, 3.0), k=14))
    # concave sides: a hull of each ring pair (a hull of all of them would fill the funnel back to a V)
    m = tm(Manifold.batch_boolean([Manifold.hull_points(np.concatenate([a, b]).astype(np.float32)) for a, b in zip(rings, rings[1:])], OpType.Add))
    m.metadata['loft'] = [np.asarray(r, float) for r in rings]
    return m


def hullof_(m_):
    """the convex hull of a Manifold: one smooth cut instead of a faceted sweep."""
    return Manifold.hull_points(np.asarray(m_.to_mesh().vert_properties[:, :3], np.float32))


def roll_arch(mesh, c, rng, shell, rmin=0.0):
    """a wheel arch about the hip-roll axis (x): the annular sector the drum sweeps over `rng` (deg), +3 mm each way,
    extruded through the shell's depth -- one smooth cylindrical cut instead of a faceted sweep."""
    v = np.asarray(mesh.vertices) - np.asarray(c)
    r = np.hypot(v[:, 1], v[:, 2]); th = np.degrees(np.arctan2(v[:, 2], v[:, 1]))
    th0, th1 = th.min() + rng[0] - 2.0, th.max() + rng[1] + 2.0
    r0, r1 = max(r.min() - 3.0, rmin, 1.0), r.max() + 3.0
    a = np.radians(np.linspace(th0, th1, 96))
    pts = [(r1 * math.cos(t), r1 * math.sin(t)) for t in a] + [(r0 * math.cos(t), r0 * math.sin(t)) for t in a[::-1]]
    bb = (shell if isinstance(shell, Manifold) else M_(shell)).bounding_box()
    xlo, xhi = bb[0] - 5.0, bb[3] + 5.0
    sec = CrossSection([pts], FillRule.EvenOdd)
    return sec.extrude(xhi - xlo).transform(np.array([[0, 0, 1.0, xlo], [1.0, 0, 0, 0], [0, 1.0, 0, 0]])).translate([0, c[1], c[2]])


def sweep_union(parts, center, axes_ranges, scale=1.0, step=5.0):
    """union of `parts` (Manifold) rotated about `center` over a grid of (axis, lo, hi) ranges, dilated by `scale` about the centre."""
    import itertools
    P = parts.translate(list(-np.asarray(center))).scale([scale] * 3).translate(list(center)) if scale != 1.0 else parts
    grids = [[(np.asarray(a, float), d) for d in np.arange(lo, hi + 1e-6, step)] for a, lo, hi in axes_ranges]
    out = []
    for combo in itertools.product(*grids):
        Tm = np.eye(4)
        for a, d_ in combo:
            Tm = trimesh.transformations.rotation_matrix(np.radians(d_), a, center) @ Tm
        out.append(P.transform(Tm[:3]))
    return Manifold.batch_boolean(out, OpType.Add)


def relief(shell, mover):
    """shell minus a neighbour's swept path: the clearance cut a moving joint needs."""
    return tm((M_(shell) - mover).simplify(0.4))                # sweeps leave thousands of facets; 0.4 mm is invisible


def design():
    J, M = KS.world()
    out = []
    add = lambda n, m: out.append((n, m))
    for side, s in (('Left', 1), ('Right', -1)):
        tag = side[0].lower()
        jp = lambda n: J[n]['p']; ja = lambda n: J[n]['axis']
        # ---- leg: hip lobe (pitch + roll + yaw), thigh, knee, calf (holds the 2 ankle motors), ankle, shoe
        hp, hr, hy, kn, an = (f'{side}_Hip_Pitch', f'{side}_Hip_Roll', f'{side}_Hip_Yaw', f'{side}_Knee_Pitch', f'{side}_Ankle_Pitch')
        lay = KS.layout(J, side)                       # drums sit on the plate structure's motors, not the K1's frames
        Hp, cp, rp, Lp = housing(lay['pitch'][0], ja(hp), KS.V['big'], extra_len=26.0)
        Hy, cy_, ry_, Ly = housing(lay['yaw'][0], ja(hy), KS.V['yaw'], pad=3.0)
        # the hip: one drum under the pelvis flank, the same language as the knee; the roll motor hides inside the pelvis
        pod = Hp
        if KS.V['name'] != 'mini':                             # tiny: the pitch drum sits right under the roll can -- clear its swing
            rc_ = lay['roll'][0]; Lr_ = KS.BIG()['L'] + 12.0
            pod = tm(M_(pod) - Manifold.cylinder(Lr_, KS.BIG()['D'] / 2 + 1.5, KS.BIG()['D'] / 2 + 1.5, 96).rotate([0, 90, 0]).translate([rc_[0] - Lr_ / 2, rc_[1], rc_[2]]))
        add(f'cover-hip.{tag}l', pod)
        add(f'puck-hip.{tag}l', cp)
        add(f'cover-hipyaw.{tag}l', Hy)
        z_pb, z_yt = pod.bounds[0][2], lay['yaw'][0][2] + Ly / 2
        add(f'cover-hipneck.{tag}l', cyl(np.array([lay['yaw'][0][0], lay['yaw'][0][1], z_yt - 4]), [0, 0, 1.0], ry_ - 1.0, z_pb - z_yt + 16))   # pod into the yaw collar
        yaw_bot = lay['yaw'][0] - unit(ja(hy)) * (Ly / 2) * np.sign(unit(ja(hy))[2])
        Lg = dict(KS.leg(J, side)); kcy = jp(kn)[1]
        reach = lambda names: max(abs(v - kcy) for n_ in names for v in Lg[f'{n_}.{tag}l'].bounds[:, 1])
        xl = 34.0 if KS.V['name'] == 'mini' else 2 * (reach(('thigh-inner', 'thigh-outer')) + 2.5 + FIL) - KS.BIG()['L']
        kshift = np.zeros(3)
        if KS.V['name'] != 'mini':                             # tiny: the plates reach further outboard -- shift the drum out, thin inboard
            r_in = max(abs(v - kcy) for n_ in ('thigh-inner', 'shin-inner') for v in Lg[f'{n_}.{tag}l'].bounds[:, 1] if (v - kcy) * s < 0)
            r_out = reach(('thigh-outer',))
            kshift = np.array([0, s * (r_out - r_in) / 2, 0]); xl = (r_in + r_out) + 2 * (2.5 + FIL) - KS.BIG()['L']
        Hk, ck, rk, Lk = housing(jp(kn) + kshift, ja(kn), KS.V['big'], extra_len=xl)    # encloses the thigh plate pair (the clevis cheeks)
        add(f'cover-knee.{tag}l', Hk); add(f'puck-knee.{tag}l', ck)
        add(f'trim-yawring.{tag}l', cyl(yaw_bot, [0, 0, 1.0], ry_ - 2, 4.0))
        thigh = bone(yaw_bot + np.array([0, 0, 12.0]), ry_ * 0.92, jp(kn), rk * 0.86, swell=1.0, flat=KS.V['flat'], n=3.0, half_w=Lk / 2)
        ank = jp(an)
        # the calf nests between the thigh plates (a clevis knee): narrower, centred on the shin plates
        if KS.V['name'] == 'mini':
            cyo, chw = 4.5, 34.5
        else:                                                  # centred on the shin plates, 3 mm over them
            ys_ = [v - kcy for n_ in ('shin-inner', 'shin-outer') for v in Lg[f'{n_}.{tag}l'].bounds[:, 1]]
            cyo, chw = s * (max(ys_) + min(ys_)) / 2, (max(ys_) - min(ys_)) / 2 + 3.0
        yo = np.array([0, s * cyo, 0])
        ck0 = 0.84 if KS.V['name'] == 'mini' else 0.98       # tiny: the first ankle can sits right under the knee
        r0c = rk * ck0
        # mini: a sculpted calf -- full just under the knee, slim at the ankle; never thinner than the ankle cans need
        cdep = [(0.0, r0c), (0.16, r0c * 1.14), (0.40, r0c * 0.98), (0.68, r0c * 0.82), (1.0, 26.0)] if KS.V['name'] == 'mini' else None
        calf = bone(jp(kn) + yo, r0c, ank + yo + np.array([0, 0, 14.0 * max(1.0, KS.R())]), max(30.0 * max(1.0, KS.R()), KS.ANK()['D'] / 2 + 2.0 if KS.R() > 1.5 else 0.0), flat=KS.V['flat'], n=3.0, half_w=chw, depth=cdep)
        cmid = (jp(kn) + ank) / 2 + yo
        add(f'cover-calf.{tag}l', grooved(calf, [('frame', 0, cmid + np.array([0, 0, 18.0]), chw - 9.0, 0.30 * np.linalg.norm(jp(kn) - ank), 10.0)]))   # a shin panel, front and back
        calf_fat = M_(bone(jp(kn) + yo, rk * ck0 + 3, ank + yo + np.array([0, 0, 14.0 * max(1.0, KS.R())]), max(33.0 * max(1.0, KS.R()), KS.ANK()['D'] / 2 + 5.0 if KS.R() > 1.5 else 0.0), flat=KS.V['flat'], n=3.0, half_w=chw + 3.0,
                           depth=[(t, r + 3.0) for t, r in cdep] if cdep else None))
        if KS.V['name'] in ('mini', 'full'):                 # long thighs: the full fold cut
            add(f'cover-thigh.{tag}l', relief(thigh, sweep_union(calf_fat, jp(kn), [(ja(kn), -2, 130)], step=4.0)))
        else:                                                  # tiny: a 35 mm thigh, relieved for the fold (no clevis room)
            add(f'cover-thigh.{tag}l', relief(thigh, sweep_union(M_(calf), jp(kn), [(ja(kn), -2, 102)], scale=1.02, step=4.0)))
        Ha, ca, ra, La = housing(ank, ja(an), KS.V['ankle'], pad=-8.0, fil=5.0, extra_len=26.0)     # the ankle U-joint: a small drum, no motor in it
        add(f'trim-ankle.{tag}l', Ha)
        F = M[f'{side.lower()}_foot_link']
        lo, hi = F.bounds
        z0 = lo[2] - KS.V['foot_drop']
        rings, sole = [], []
        for t in np.linspace(0, 1, 16):
            x = lo[0] - 2 + t * (hi[0] - lo[0] + 4)
            h = 22 + 22 * math.exp(-((t - 0.34) / 0.2) ** 2) - 6 * t
            a = (hi[1] - lo[1]) / 2 * (0.8 + 0.2 * math.sin(math.pi * min(1.0, t / 0.82)))
            if t in (0.0, 1.0):
                a *= 0.75; h *= 0.85
            yc = (lo[1] + hi[1]) / 2
            rings.append(ring(np.array([x, yc, z0 + 7 + h / 2]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]), a, h / 2, n=3.0))
            sole.append(ring(np.array([x, yc, z0 + 3.5]), np.array([0, 1.0, 0]), np.array([0, 0, 1.0]), a + 1.5, 3.5, n=5.0))
        shoe = tm(Manifold.hull_points(np.concatenate(rings).astype(np.float32)))
        lower = (M_(calf) + M_(Ha)) ^ Manifold.cube([400, 400, 200]).translate([ank[0] - 200, ank[1] - 200, ank[2] - 40])
        add(f'cover-shoe.{tag}l', relief(shoe, sweep_union(lower, ank, [(ja(an), -52, 52), ([1.0, 0, 0], -22, 22)], scale=1.07, step=6.0)))   # instep pocket for the ankle
        add(f'sole.{tag}l', hull_rings(sole))
        # ---- arm: shoulder roll drum, upper arm (through the upper-arm rotation drum), elbow, forearm, mitten
        sp, sr, ep, ey = (f'A{side}_Shoulder_Pitch', f'{side}_Shoulder_Roll', f'{side}_Elbow_Pitch', f'{side}_Elbow_Yaw')
        U = KS.upper_layout(J, side)                   # arm drums on the structure's motors
        Hsr, csr, rsr, Lsr = housing(U['sr'][0] + np.array([3.0, 0, 0]), U['sr'][1], KS.V['arm'], extra_len=22.0)   # long enough to hold the yoke + upper-arm plates
        add(f'cover-shoulder.{tag}a', Hsr); add(f'puck-shoulder.{tag}a', csr)
        Hue, cue, rue, Lue = housing(U['ep'][0] + np.array([2.0, 0, -6.0]), U['ep'][1], KS.V['arm'], pad=6.0, extra_len=20.0)
        Hel, cel, rel, Lel = housing(U['ey'][0], U['ey'][1], KS.V['arm'], extra_len=14.0)   # room for the elbow + forearm plates
        add(f'trim-upperarm.{tag}a', Hue)              # the upper-arm rotation: a gunmetal drum mid-arm
        add(f'cover-elbow.{tag}a', Hel); add(f'puck-elbow.{tag}a', cel)
        epc = U['ep'][0]
        add(f'cover-upperarm.{tag}a', bone(U['sr'][0] + np.array([0, 0, -rsr * 0.5]), rsr * 0.8, epc, rue * 0.9, lateral=(0, 1.0, 0), half_w=Lsr / 2 - 8))
        ey_ = U['ey'][0]
        tip = ey_ + KS.hang(J, M, side)
        wrist = ey_ + (tip - ey_) * 0.62
        ma_ = KS.ARM(); g_, t_ = KS.G, KS.T
        f_in, f_out = -(ma_['L'] / 2 + g_) + 3.5, ma_['L'] / 2 + g_ + t_ + 3.3      # clear of E2 inboard, over F1 outboard
        fo = np.array([0, s * (f_in + f_out) / 2, 0]); f_half = (f_out - f_in) / 2  # forearm rides outboard of the inner elbow plate
        r0f = rel * 0.8
        fdep = [(0.0, r0f), (0.2, r0f * 1.12), (0.55, r0f * 0.92), (1.0, 15.0)]            # full under the elbow, slim at the wrist
        Wr = KS.wrist_layout(J, side, M)
        if Wr is not None:                                     # 7-DoF arm: the forearm ends round over the wrist-roll can
            mw = KS.WRIST(); rcw, pcw, ycw = Wr['roll'][0], Wr['pitch'][0], Wr['yaw'][0]
            r_sl = Wr['r_sl']                                  # sleeve: holds R0/R1 + the pitch can's ends
            f_half = r_sl; fo = np.array([0, rcw[1] - ey_[1], 0])      # same inboard face, wider outboard
            wrist = np.array([ey_[0], ey_[1], rcw[2] - mw['L'] / 2])
            fdep = [(0.0, r0f), (0.2, r0f * 1.12), (0.6, max(r0f * 0.9, r_sl + 2.0)), (1.0, r_sl)]
        fore = bone(ey_ + fo, r0f, wrist + fo, 18.0 if Wr is None else r_sl, lateral=(0, 1.0, 0), half_w=f_half, depth=fdep)
        add(f'cover-forearm.{tag}a', fore)
        # lower upper arm: rotation drum down to the elbow, full width, with the forearm's fold cut out of it (a clevis, like the knee)
        low = bone(np.array([epc[0], epc[1], Hue.bounds[0][2] - 1.0]), rue * 0.9, ey_, rel * 0.85, flat=1.25, lateral=(0, 1.0, 0), half_w=ma_['L'] / 2 + g_ + t_ + 3.3)
        fat = M_(bone(ey_ + fo, rel * 0.8 + 3, wrist + fo, 21.0, lateral=(0, 1.0, 0), half_w=f_half + 1.5, depth=[(t, r + 3.0) for t, r in fdep]))
        add(f'cover-elbowfork.{tag}a', relief(low, sweep_union(fat, ey_, [(U['ey'][1], -142, 2)], step=4.0)))
        u = unit(tip - wrist); v = unit(np.cross(u, [0, 1.0, 0])) if abs(u[1]) < 0.9 else unit(np.cross(u, [1.0, 0, 0])); w = np.cross(u, v)
        if Wr is None:
            palm = [ring(wrist + u * t, w, v, 13 * f, 22 * f, n=2.4) for t, f in ((4, 0.9), (18, 1.0), (40, 1.0), (56, 0.85), (66, 0.55), (71, 0.25))]
            add(f'cover-hand.{tag}a', hull_rings(palm))
            add(f'trim-wrist.{tag}a', cyl(wrist, u, 15.0, 5.0))
        else:                                                  # round sleeve (rolls) -> gunmetal pitch drum -> a hand around the yaw can
            WS = dict(KS.wrist(J, side, M)); g = lambda k: M_(WS[f'{k}.{tag}a'])
            z_sl0, z_sl1 = rcw[2] - mw['L'] / 2 - 1.0, pcw[2] + mw['D'] / 2 + 1.0
            add(f'cover-wristsleeve.{tag}a', cyl(np.array([rcw[0], rcw[1], (z_sl0 + z_sl1) / 2]), [0, 0, 1.0], r_sl, z_sl0 - z_sl1 if z_sl0 > z_sl1 else z_sl1 - z_sl0))
            drum = M_(cyl(pcw, [0, 1.0, 0], mw['D'] / 2 + 3.0, mw['L'] + 2 * (KS.G + KS.T) + 2.0))
            add(f'trim-wristpitch.{tag}a', tm(drum - sweep_union(g('wrist-P0'), pcw, [((0, 1.0, 0), -75, 75)], scale=1.04, step=5.0)))
            rd = (pcw[2] - mw['D'] / 2 - 3.0) - ycw[2] - 2.0          # the hand's crown: a dome about the yaw axis, under the pitch drum
            cw = np.array([ycw[0] + 4.0, ycw[1], 0])
            crown = [(ycw[2] + rd * math.cos(th), max(rd * math.sin(th), 6.0)) for th in np.linspace(0.0, 1.35, 6)]
            zt = tip[2]
            body_ = [(ycw[2] - 0.25 * (ycw[2] - zt), 35.0), (ycw[2] - 0.55 * (ycw[2] - zt), 31.0), (ycw[2] - 0.82 * (ycw[2] - zt), 20.0), (zt, 8.0)]
            palm = [ring(np.array([cw[0], cw[1], z_]), w, v, a_, 40.0 * min(1.0, a_ / 30.0 + 0.3), n=2.4) for z_, a_ in crown + body_]
            hand = M_(hull_rings(palm)) - sweep_union(g('wrist-P1') + g('wrist-P0'), ycw, [((1.0, 0, 0), -45, 45)], scale=1.04, step=5.0)
            add(f'cover-hand.{tag}a', tm(hand))
                # shoulder yoke: the shoulder-pitch drum, out of the chest flank into the shoulder drum (the arm hangs from it)
        # shoulder, full-size humanoid language (H2/T2): a round pitch disc flush on the chest flank, a deltoid that
        # belongs to the arm (wraps the roll drum, flows into the upper arm), the chest top sloping down to meet it
        spc, src = U['sp'][0], U['sr'][0]
        # to the deltoid's inner face (the roll drum's RADIUS + 3 mm: the roll axis runs fore/aft), less a running gap that grows with size
        y_in = min(SHOULDER_FLANK() - 14.0, abs(spc[1]) + KS.ARM()['L'] / 2 + KS.G - 2.0)
        if True:                                              # the drum wraps the whole pitch can (the shell's crown fillet no longer hides it)
            y_in = abs(spc[1]) - KS.ARM()['L'] / 2 - 2.0   # from the pitch output face (covers the yoke plates)
        y_out = abs(src[1]) - (rsr + 3.0) - 2.0 * max(1.0, KS.R())
        r_disc = 42.0 * max(KS.R(), 0.8)                                     # covers the yoke plates (40 mm from the axis)
        dmid = np.array([spc[0], s * (y_in + y_out) / 2, spc[2]])
        disc, _, _, _ = housing(dmid, [0, s * 1.0, 0], KS.V['arm'], pad=r_disc - KS.ARM()['D'] / 2, fil=8.0, extra_len=(y_out - y_in) - KS.ARM()['L'])
        add(f'cover-shoulderyoke.{tag}a', disc)                              # coaxial with the pitch axis: turns in place
        ua_m = dict(out)[f'cover-upperarm.{tag}a']
        ax_ = np.asarray(U['sr'][1], float); c_ = U['sr'][0] + np.array([3.0, 0, 0])
        vv, ww = np.array([0, 1.0, 0]), np.array([0, 0, 1.0])
        rr_ = rsr + 3.0                                                       # the roll drum, 3 mm proud: coaxial, so it rolls clean
        shell_pts = np.concatenate([_circle(c_ + ax_ * t_, vv, ww, rr_ - fr_, 160) for t_, fr_ in
                                    [(s_ * (Lsr / 2 - 8.0 + 8.0 * np.sin(a_)), 8.0 * (1 - np.cos(a_))) for s_ in (-1, 1) for a_ in np.linspace(0, np.pi / 2, 7)]])
        top = np.concatenate(ua_m.metadata['loft'][:2])                      # the upper arm's top sections
        add(f'cover-deltoid.{tag}a', tm(Manifold.hull_points(np.concatenate([shell_pts, top]).astype(np.float32))))
    # ---- torso: chest, accent waist, pelvis (inside the K1's trunk envelope, our own blocks)
    T = M['Trunk']; lo, hi = T.bounds
    zs = J['ALeft_Shoulder_Pitch']['p'][2]
    Hl0 = KS.head_layout(J); z_d1 = Hl0['yaw'][0][2] - 25.5 - KS.G - KS.T
    zc1 = max(zs + 28 * KS.R(), z_d1 + KS.T + 3.0)                # the chest closes over the neck-yaw deck
    # torso in three: a chest that narrows to the waist (a V, not a box), the orange band at the bottom of the
    # rib cage, a slimmer abdomen down to a pelvis that flares out over the hips
    Rk = KS.R()
    z_ab, z_band, z_ch = 14.0 * Rk, 54.0 * Rk, 76.0 * Rk
    Wl = KS.waist_layout()
    if Wl is not None:                                          # 3-DoF waist: the chest starts at the pitch axis, a slim drum below
        z_ch = Wl['pitch'][0][2] - 0.45 * KS.WAIST()['D']
    if Wl is None:                                              # H2/T2: a chest shell that stops above a narrow dark waist
        z_w1 = z_ab - 2.0                                       # one piece again: from the pelvis line...
        zc1 = zs + 36.0 * Rk                                    # ...up over the shoulder-pitch drums (crown = zc1 + 10)
        y_cap = dict(out)['cover-shoulderyoke.la'].bounds[1][1] + 2.0 if KS.V['name'] == 'mini' else 1.10 * KS.CHEST()   # top corners just over the shoulder drums (tiny: its arms swing too close)
        chest = torso_shell(lo, hi, KS.CHEST(), z_w1, max(z_ch, z_w1 + 20.0 * Rk), zc1, zs, y_cap, Rk)
        z_ch = z_w1                                             # (the panel groove spans the chest)
    else:                                                       # full: the same sleek shell, from the waist-pitch hub up
        y_cap = dict(out)['cover-shoulderyoke.la'].bounds[1][1] + 2.0   # the mini's shaping: corners just over the shoulder drums
        chest = torso_shell(lo, hi, KS.CHEST(), z_ch, z_ch + 20.0 * Rk, zc1, zs, y_cap, Rk)
    zc_mid = z_ch + (zc1 - z_ch) * 0.52
    chest_m = chest if Wl is not None else grooved(chest, [('frame', 0, np.array([0, 0, zc_mid]), KS.CHEST() * 0.56, (zc1 - z_ch) * 0.34, 14.0 * KS.R())])   # (full: plain -- its pocket cut sends it out as a mesh)   # chest panel, front + back
    E = KS.ELEC
    if KS.V['name'] == 'mini':
        xb = chest_m.bounds[0][0]
        Mc = M_(chest_m)                                        # exhaust slots through the (leaning) back wall at each slot's height
        cuts = [('box', [Mc.slice(zc).bounds()[0] - 2.0, -33.0, zc - 1.75], [7.0, 66.0, 3.5]) for zc in np.linspace(E['vent_out']['z'][0] + 3, E['vent_out']['z'][1] - 3, 5)]
        if Wl is None:                                          # intake grille, low front
            vy0, vy1 = E['vent_in']['y']; vz0, vz1 = E['vent_in']['z']
            for sy in (1, -1):                                   # one per side bay: the pack blocks the middle
                for zc in np.arange(vz0 + 2.0, vz1 - 1.0, 7.0):  # 3.5 mm slots on a 7 mm pitch: 50 % open
                    cuts.append(('box', [Mc.slice(zc).bounds()[2] - 12.0, vy0 if sy > 0 else -vy1, zc - 1.75], [14.0, vy1 - vy0, 3.5]))
        if E['estop']:
            cuts.append(('xcyl', [xb, 0.0, E['estop']['z'] * KS.R()], 8.1, 6.0))      # e-stop: 16.2 mm hole in the back
        chest_m = cut_shell(chest_m, cuts)
    if KS.waist_layout() is not None:                           # the waist-pitch stator plate turns in a thin internal pocket
        Wp_ = KS.waist_layout()['pitch'][0]
        WS_ = dict(KS.waist()); mw_ = KS.WAIST(); rc__ = KS.waist_layout()['roll'][0]
        barrel = M_(cyl(rc__, [1.0, 0, 0], mw_['D'] / 2 + 9.0, 2 * (mw_['L'] / 2 + KS.G + KS.T + 4.0)))   # the roll stage the chest pitches over
        roll_stage = M_(WS_['waist-W3']) + M_(WS_['waist-W2']) + barrel
        chest_m = relief(chest_m, hullof_(sweep_union(roll_stage, Wp_, [((0, 1.0, 0), -65, 35)], scale=1.04, step=5.0)))   # one clean arch under the chest
    if KS.V['name'] == 'mini':                                  # the back pack's opening (140 x 155): the pack face closes it
        plo, phi = KS.pack_box()                                  # the pack hatch: its back face fills it, flush low, 6 mm in high
        OPEN = [('box', [plo[0] - 100.0, plo[1] - 0.5, plo[2] - 0.5], [100.5, phi[1] - plo[1] + 1.0, phi[2] - plo[2] + 1.0])]
        chest_m = cut_shell(chest_m, OPEN)
    add('cover-chest', chest_m)
    if Wl is None:
        pass                                                    # one-piece torso: no separate waist band
    else:                                                       # H2-style waist: three coaxial pieces, no slots -- each shell turns
        mw = KS.WAIST(); pc_, rc_ = Wl['pitch'][0], Wl['roll'][0]   # about its own joint axis, so nothing needs cutting away
        WS = dict(KS.waist())
        r_d = math.hypot(mw['L'] / 2 + KS.G + KS.T + 2.0, mw['D'] / 2) + 4.0
        z_d = KS.Z_D0() + KS.T + 1.0                              # on the pelvis top, clear of the deck
        add('cover-waist', cyl(np.array([0, 0, (z_d + rc_[2]) / 2]), [0, 0, 1.0], r_d, rc_[2] - z_d))          # yaw drum (yaw stage), up to the roll axis
        add('cover-waistroll', cyl(rc_, [1.0, 0, 0], mw['D'] / 2 + 9.0, 2 * (mw['L'] / 2 + KS.G + KS.T + 4.0)))   # roll barrel (roll stage)
        hub = M_(cyl(pc_, [0, 1.0, 0], mw['D'] / 2 + 10.0, 0.62 * 2 * KS.CHEST() * 0.90)) - \
            sweep_union(M_(WS['waist-W3']), pc_, [((0, 1.0, 0), -65, 65)], scale=1.03, step=5.0)   # W3's pocket, inside the hub
        add('trim-waisthub', tm(hub))                         # gunmetal pitch hub (chest): carries the chest mount
    if Wl is None:
        if KS.V['name'] == 'mini':                              # intake grille, low front (the abdomen is the torso shell now)
            plo, phi = KS.pack_box()
            rings = [rrect(np.array([(plo[0] + phi[0]) / 2, 0, z_]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), phi[1] - d_, (phi[0] - plo[0]) / 2 - d_, 6.0)
                     for z_, d_ in ((plo[2], 1.5), (plo[2] + 1.5, 0.0), (phi[2] - 1.5, 0.0), (phi[2], 1.5))]
            add('cover-pack', hull_rings(rings))                  # the pack case (PA12): slides into the bay, its back is the hatch
    if KS.V['name'] == 'mini' and KS.ELEC['estop']:             # the e-stop: red 40 mm mushroom on the upper back
        es = KS.ELEC['estop']; xb = chest_m.bounds[0][0]
        head = cyl(np.array([xb - es['proud'] + 6.0, 0, es['z'] * KS.R()]), [1.0, 0, 0], es['head_d'] / 2, 12.0)
        stem = cyl(np.array([xb - es['proud'] / 2 + 1.0, 0, es['z'] * KS.R()]), [1.0, 0, 0], 11.0, es['proud'] - 10.0)
        add('estop-head', head); add('estop-collar', stem)
    # pelvis, K1-style: a block that wraps the two hidden hip-roll cans and follows their round bellies in,
    # then a narrow keel dropping between the hip drums
    rl = KS.layout(J, 'Left')['roll'][0]; m06 = KS.BIG(); Rk = KS.R()
    x0, x1 = lo[0] + 10, hi[0] - 12; xm, xh = (x0 + x1) / 2, (x1 - x0) / 2
    xh = max(xh, (max(abs(rl[0] - m06['L'] / 2 - xm), abs(rl[0] + m06['L'] / 2 - xm)) + 5.0) / 0.88)   # deep enough for the roll cans
    z_pt = 14.0 * Rk if KS.waist_layout() is None else KS.Z_D0() + KS.T + 0.5   # waist: the pelvis closes over the deck
    secs = [rrect(np.array([xm, 0, z_pt]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), KS.CHEST() * 0.84, xh, 24.0 * Rk)]   # meets the torso; flares over the hip cans below
    for th in np.linspace(0, np.pi / 2, 7):                     # the cans' lower quadrant, 5 mm proud
        r_ = m06['D'] / 2 + 5.0
        y_, z_ = rl[1] + r_ * np.cos(th), rl[2] - r_ * np.sin(th)
        secs.append(rrect(np.array([xm, 0, z_]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), max(y_, 30.0 * Rk), xh, 24.0 * Rk))   # constant depth: a box, like the torso
    upper = Manifold.hull_points(np.concatenate(secs).astype(np.float32))
    zk = rl[2] - m06['D'] / 2 - 5.0
    rk_ = 14.0 * Rk                                             # keel: one rounded box between the hip drums, filleted bottom
    kb = zk - 34.0 * Rk
    keel = [rrect(np.array([xm, 0, zk + 20 * Rk]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), 40.0 * Rk, xh * 0.8, 16.0 * Rk)]
    keel += [rrect(np.array([xm, 0, kb + rk_ * (1 - math.sin(th))]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]),
                   40.0 * Rk - rk_ * (1 - math.cos(th)), xh * 0.8 - rk_ * (1 - math.cos(th)), max(16.0 * Rk - rk_ * (1 - math.cos(th)), 3.0))
             for th in np.linspace(0, np.pi / 2, 6)]
    pel = upper + Manifold.hull_points(np.concatenate(keel).astype(np.float32))
    if KS.V['name'] in ('mini', 'full'):                        # a plain rounded box, like the head: just the safety cover over the
        bx0, bx1 = rl[0] - m06['L'] / 2 - KS.G - KS.T - 6.0, rl[0] + m06['L'] / 2 + KS.G + KS.T + 6.0      # two hip-roll cans, tucked under the torso
        by = rl[1] + m06['D'] / 2 + 4.0
        if KS.waist_layout() is not None:                       # full: it also carries the waist-yaw motor's deck
            wd = KS.WAIST()['D'] / 2 + 14.0
            bx0, bx1 = min(bx0, -wd), max(bx1, wd)
        bz0, bz1 = rl[2] - m06['D'] / 2 - 4.0, (z_ab - 2.0) if KS.waist_layout() is None else z_pt
        rcp, rfb = 0.32 * min(by, (bx1 - bx0) / 2), m06['D'] / 2 + 4.0     # bottom radius = the cans': concentric, clear of the drums
        bring = lambda z_, d_: rrect(np.array([(bx0 + bx1) / 2, 0, z_]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), by - d_, (bx1 - bx0) / 2 - min(d_, 6.0), max(rcp - min(d_, 6.0), 3.0), k=14)
        prs = [bring(bz0 + rfb * (1 - math.sin(th)), rfb * (1 - math.cos(th))) for th in np.linspace(np.pi / 2, 0, 11)] + [bring(bz1, 0.0)]
        pel = Manifold.hull_points(np.concatenate(prs).astype(np.float32))
    got = dict(out)
    for side in ('Left', 'Right'):
        t = side[0].lower(); lay = KS.layout(J, side)
        collar = M_(got[f'cover-hipneck.{t}l']) + M_(got[f'cover-hipyaw.{t}l'])
        hullof = lambda m_: Manifold.hull_points(np.asarray(m_.to_mesh().vert_properties[:, :3], np.float32))   # one smooth convex cut, not a faceted scoop
        pel = pel - hullof(sweep_union(collar, lay['pitch'][0], [((0, 1.0, 0), -110, 50)], scale=1.04))          # the thigh swings under the pelvis
        hr_m = dict(KS.leg(J, side))[f'chassis-hipR.{t}l']
        # the hip-roll bracket crosses the gap under the pelvis: sheathe it in a gunmetal guard so no bare aluminium shows
        upper = tm(M_(hr_m) ^ Manifold.cube([400, 400, 400]).translate([-200, -200, lay['pitch'][0][2] + (m06['D'] * 0.5 if KS.V.get('leg_in') else -20.0)]))
        gv = np.concatenate([upper.vertices + np.array([dx, 0, 0]) for dx in (-2.0, 2.0)])
        guard = tm(Manifold.hull_points(gv.astype(np.float32)))
        add(f'cover-hipguard.{t}l', guard)
        gc_ = guard.bounds.mean(0)
        brk = M_(guard).translate(list(-gc_)).scale([1.2, 1.0, 1.0]).translate(list(gc_)) + M_(hr_m)   # the bracket: an internal slot
        pel = pel - sweep_union(brk, lay['roll'][0], [((1.0, 0, 0), -24, 48)], scale=1.03, step=2.0)
        if KS.V['name'] == 'tiny':                              # the tiny's drum rolls in too close for an arch: keep the swept cut
            pel = pel - sweep_union(M_(got[f'cover-hip.{t}l']), lay['roll'][0], [((1.0, 0, 0), -24, 48)], scale=1.03, step=2.0)
        else:
            pel = pel - roll_arch(got[f'cover-hip.{t}l'], lay['roll'][0], (-24, 48), pel, rmin=m06['D'] / 2 + 7.0)            # the drum: one clean round arch

    if KS.V['name'] not in ('mini', 'full'):                    # mini + full: no pelvis cover -- the hip-roll cans are the pelvis (Cam)
        add('cover-pelvis', tm(pel))
    # ---- neck + head: a graphite neck drum, an egg helmet with a visor band
    Hl = KS.head_layout(J)
    if KS.V['name'] == 'mini':
        nz0, nw = Hl['yaw'][0][2] + 20.0, (30.0, 34.0)
    else:                                                       # yaw motor up in the neck: a wider neck from just above the chest dome
        nz0, nw = zc1 + 14.0 * KS.R() + 1.0, (max(33.0, KS.HEAD()['D'] / 2 + 3.0), max(36.0, KS.HEAD()['D'] / 2 + 3.0))
    nz1 = Hl['pitch'][0][2] - max(33.0, KS.HEAD()['D'] / 2 + 4.5) + 2.0   # 2 mm into the pitch drum (round about the pitch axis: no clash)
    if Wl is None:                                              # one round satin column, sunk into the chest crown (round: it yaws clean)
        nz0 = min(nz0, zc1 + 6.0 * Rk - 8.0)
    add('cover-neck', cyl(np.array([Hl['yaw'][0][0], 0, (nz0 + nz1) / 2]), [0, 0, 1.0], max(nw), nz1 - nz0))
    if KS.V['name'] != 'mini':                                  # round neck base, coaxial with the yaw: chest dome up into the neck
        yz0 = zc1 + 2.0
        add('trim-neckbase', cyl(np.array([Hl['yaw'][0][0], 0, (yz0 + nz0 + 4) / 2]), [0, 0, 1.0], max(31.5, KS.HEAD()['D'] / 2 + 1.5), nz0 + 4 - yz0))
    # the cortex head (Jetson + its own battery, buttons + ports on the back): a module with a flat bottom that sits on the
    # neck's dock plate and can live detached, on a desk. Box language: flat face for the screen, wedged back, crown fillet.
    Hh = M['Head_2']; hlo, hhi = Hh.bounds
    hp = KS.head_layout(J)['pitch'][0]; Rk = KS.R()
    hh, hw = (hhi[2] - hlo[2]) * 0.62, (hhi[1] - hlo[1]) / 2 * 0.96
    x_face, x_back = hp[0] + 46.0 * Rk, hp[0] - 58.0 * Rk        # the back reaches past the neck: no craning
    hd = (x_face - x_back) / 2
    zb = KS.DOCK_Z(J) + KS.T + 0.5                              # flat bottom, on the dock
    hc = np.array([x_face - hd, 0.0, zb + hh / 2])
    rc_p = 0.46 * min(hw, hd)                                   # plan corner radius: soft, pebble-like
    rf, rb_ = 0.42 * min(hw, hd), 0.24 * min(hw, hd)            # generous crown + bottom rounds (it still sits flat)
    hr_ = [(zb + rb_ * (1 - math.sin(th)), rb_ * (1 - math.cos(th))) for th in np.linspace(0, np.pi / 2, 5)]
    hr_ += [(hc[2] + hh / 2 - rf * (1 - math.sin(th)), rf * (1 - math.cos(th))) for th in np.linspace(np.pi / 2, 0, 7)]
    hr_ = sorted(hr_)
    wedge = lambda z_: 0.0                                      # no helmet wedge: one soft rounded box
    hr_ = [rrect(np.array([x_face - (hd - wedge(z_)), 0.0, z_]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), hw - d_, hd - d_ - wedge(z_), max(rc_p - d_, 2.0), k=14)
           for z_, d_ in hr_]
    hm = hull_rings(hr_)
    zm = zb + hh * 0.42                                          # vents: side slots near the back (in left, out right)
    vents = [('box', [x_back + rc_p + 4.0 + 7.0 * i, (hw - 4.0) if s_ > 0 else -(hw + 2.0), zm - 12.0 * Rk], [3.5, 6.0, 24.0 * Rk])
             for s_ in (1, -1) for i in range(5)]
    add('cover-helmet', cut_shell(hm, vents))
    m_h = KS.HEAD()
    r_hub, yo_ = max(33.0, m_h['D'] / 2 + 4.5), m_h['L'] / 2 + KS.G + KS.T + 1.0
    y0_, y1_ = -yo_, m_h['L'] / 2 + 0.5                         # the head-pitch drum in two coaxial halves: neck side (over the
    add('cover-headhub', cyl(np.array([hp[0], (y0_ + y1_) / 2, hp[2]]), [0, 1.0, 0], r_hub, y1_ - y0_))   # stator plate) and head side
    add('cover-headcheek', cyl(np.array([hp[0], (y1_ + 0.5 + yo_ + 0.5) / 2, hp[2]]), [0, 1.0, 0], r_hub, yo_ - y1_))   # (over the dock bracket)
    yd = m_h['L'] / 2 + KS.G + KS.T
    dock = [rrect(np.array([hp[0], 0, KS.DOCK_Z(J) + dz_]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), yd + 3.0, 37.0 * Rk, 8.0, k=10)
            for dz_ in (-1.0, KS.T + 0.4)]
    add('trim-headdock', hull_rings(dock))                       # the dock: a gunmetal plate the head seats on
    # the face: a flat screen, gloss black glass, a hair proud
    sw, sh = hw - rc_p - 6.0, 0.25 * hh                          # a calmer screen, well inside the face
    scr = [rrect(np.array([x_, 0.0, hc[2] + 0.06 * hh]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), sh - d_, sw - d_, 0.55 * min(sw, sh), k=12)
           for x_, d_ in ((x_face - 3.0, 0.0), (x_face + 0.4, 0.0), (x_face + 0.8, 0.4))]
    add('visor-face', hull_rings(scr))
    # the back panel (the cortex's own): power button + status LED, USB-C, Ethernet, on the flat back of the head
    zpm, pw, ph = zb + hh * 0.38, hw * 0.62, hh * 0.16
    prings = [rrect(np.array([x_back + dx, 0.0, zpm]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), ph - d_, pw - d_, 5.0, k=10)
              for dx, d_ in ((1.0, 0.0), (-0.8, 0.0), (-1.4, 0.5))]
    add('trim-headpanel', hull_rings(prings))
    xo = x_back - 1.4
    add('cover-headbutton', cyl(np.array([xo - 1.0, 0.68 * pw, zpm]), [1.0, 0, 0], 5.0, 2.4))                 # power
    add('visor-headled', cyl(np.array([xo - 0.4, 0.40 * pw, zpm]), [1.0, 0, 0], 1.6, 1.2))                  # status
    add('visor-headport', hull_rings([rrect(np.array([xo + dx, -0.10 * pw, zpm]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), 2.0, 5.0, 1.9, k=8) for dx in (0.4, -0.2)]))   # USB-C
    add('visor-headeth', hull_rings([rrect(np.array([xo + dx, -0.58 * pw, zpm]), np.array([0, 0, 1.0]), np.array([0, 1.0, 0]), 6.0, 8.0, 1.0, k=6) for dx in (0.4, -0.2)]))     # Ethernet
    # ear caps: the head-pitch axis, orange, both sides
    hp_ = J['Head_pitch']['p']
    for s in (1, -1):
        add(f'puck-ear.{"l" if s > 0 else "r"}', cyl(np.array([hp_[0], s * (hw - 1.0), hc[2] - hh * 0.02]), [0, 1.0, 0], 17.0, 4.0))
    return out if JOINT_CAPS else [(n, m) for n, m in out if not n.startswith('puck-')]   # caps off: plain satin drums, like the K1


PERM = {0: np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0.]]), 1: np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0.]]), 2: np.eye(3)}


def clip(plate, shell, wall=2.0):
    """router-cut profile = plate profile inside the shell's section, inset by the wall (plates are planar)."""
    P = Manifold(Mesh(vert_properties=np.asarray(plate.vertices, np.float32), tri_verts=np.asarray(plate.faces, np.uint32)))
    lo, hi = plate.bounds
    ax = int(np.argmin(hi - lo))
    R = PERM[ax]
    Rt = lambda R_: np.hstack([R_, np.zeros((3, 1))])
    Sr = shell.transform(Rt(R))
    sec = (Sr.slice(lo[ax] + 0.1) ^ Sr.slice(hi[ax] - 0.1)).offset(-wall, JoinType.Round)
    cut = Manifold.extrude(sec, hi[ax] - lo[ax] + 2).translate([0, 0, lo[ax] - 1]).transform(Rt(R.T))
    parts = sorted((P ^ cut).decompose(), key=lambda q: -q.volume())
    if not parts:                                               # no shell over it (open hips): the plate stays whole
        return plate
    return tm(parts[0])                                        # router-cut part = the main body (drops slivers)


def structure(out=None):
    """the plate frame clipped inside its shells; motors pass through untouched."""
    out = out or design()
    S = Manifold.batch_boolean([Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
                                for n, m in out if n.split('-')[0] in ('cover', 'puck', 'trim', 'accent')], OpType.Add)
    return [(n, m if (n.startswith(('rs', 'elec', 'node', 'cover-baffle')) or 'hipR' in n or 'elbowE2' in n or 'yokeP2' in n or n.startswith(('waist-', 'wrist-', 'torso-rail', 'torso-latch', 'pelvis-deck'))) else clip(m, S)) for n, m in KS.structure()]   # these span a joint's parting gap


def model():
    out = design()
    out = out + structure(out)
    out = [(n, K.simplify(m, grid=1.0) if len(m.faces) > 20000 else m) for n, m in out]   # swept relief cuts: decimate for the viewer only
    return dict(hip_x=0.0, plane=0.0, legs=[], body=None, walls=[], extra=out,
                note='mini humanoid: our coaxial housings + bone links on the K1 joint layout, RobStride inside every drum',
                posed={}, only=True, ground=True)
