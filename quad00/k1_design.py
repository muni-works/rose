"""MINI humanoid, our own look on the K1's joint layout: every joint a rounded coaxial housing around OUR
RobStride (graphite caps in both ends), every link a bone lofted between its joints' housings (slimmer
than the joints, a swell for thigh / calf / forearm), the hip pitch-roll-yaw cluster merged into one lobe,
our own chest + pelvis blocks with the accent waist, an egg helmet with a visor band, shoes, mittens.
K1 frame: x fore, y left, z up (mm). Joint positions + axes from the K1 URDF (arms lowered)."""
import math
import numpy as np
import k1_structure as KS, trimesh
from manifold3d import Manifold, Mesh, OpType, JoinType
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


def hull_rings(rings, **meta):
    """the shell = hull of its section rings; the rings ride along (metadata) so exporters can loft them natively."""
    m = tm(Manifold.hull_points(np.concatenate(rings).astype(np.float32)))
    m.metadata['loft'] = [np.asarray(r, float) for r in rings]
    m.metadata.update(meta)
    return m


def rrect(c, v, w, a, b, rc, k=10):
    """a rounded rectangle section: flat sides, corner radius rc."""
    rc = min(rc, a * 0.98, b * 0.98)
    pts = []
    for sx, sy, t0 in ((1, 1, 0.0), (-1, 1, 0.5 * np.pi), (-1, -1, np.pi), (1, -1, 1.5 * np.pi)):
        t = np.linspace(t0, t0 + 0.5 * np.pi, k)
        x = sx * (a - rc) + rc * np.cos(t); y = sy * (b - rc) + rc * np.sin(t)
        pts.append(c[None, :] + x[:, None] * v[None, :] + y[:, None] * w[None, :])
    return np.concatenate(pts)


def bone(p0, r0, p1, r1, swell=1.0, flat=1.0, lateral=(0, 1.0, 0), n=2.2, half_w=None):
    """a link lofted from a section at p0 to one at p1 (radii r0, r1), with a swell at 40 %."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    u = unit(p1 - p0)
    v = unit(np.asarray(lateral, float) - u * (np.asarray(lateral) @ u)); w = np.cross(u, v)
    rings = []
    for t, k in ((0.0, 1.0), (0.18, 1.0 + 0.6 * (swell - 1)), (0.42, swell), (0.7, 1.0 + 0.5 * (swell - 1)), (1.0, 1.0)):
        r = r0 + (r1 - r0) * t
        a = r * k * flat if half_w is None else min(r * k * flat, half_w)   # never wider than the joint drums' faces
        rings.append(rrect(p0 + u * t * np.linalg.norm(p1 - p0), v, w, a, r * k, 0.4 * min(a, r * k)))
    return hull_rings(rings)


def block(x0, x1, y, z0, z1, n=2.8, taper=(1.0, 1.0), dome=0.0, RC=22.0):
    """a rounded torso block: superellipse sections (x fore, y side) stacked up z, tapered."""
    rings = []
    for t in np.linspace(0, 1, 9):
        z = z0 + (z1 - z0) * t
        k = taper[0] + (taper[1] - taper[0]) * t
        rings.append(rrect(np.array([(x0 + x1) / 2, 0, z]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), y * k, (x1 - x0) / 2 * (0.94 + 0.06 * k), RC))
    if dome:
        for f, d in ((0.9, 0.4), (0.7, 0.75), (0.4, 1.0)):
            rings.append(rrect(np.array([(x0 + x1) / 2, 0, z1 + dome * d]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), y * taper[1] * f, (x1 - x0) / 2 * f, RC * f))
    return hull_rings(rings)


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
        add(f'trim-hipyaw.{tag}l', Hy)
        z_pb, z_yt = pod.bounds[0][2], lay['yaw'][0][2] + Ly / 2
        add(f'trim-hipneck.{tag}l', cyl(np.array([lay['yaw'][0][0], lay['yaw'][0][1], z_yt - 4]), [0, 0, 1.0], ry_ - 1.0, z_pb - z_yt + 16))   # pod into the yaw collar
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
        calf = bone(jp(kn) + yo, rk * ck0, ank + yo + np.array([0, 0, 14.0]), 30.0, flat=KS.V['flat'], n=3.0, half_w=chw)
        add(f'cover-calf.{tag}l', calf)
        calf_fat = M_(bone(jp(kn) + yo, rk * ck0 + 3, ank + yo + np.array([0, 0, 14.0]), 33.0, flat=KS.V['flat'], n=3.0, half_w=chw + 3.0))
        if KS.V['name'] == 'mini':
            add(f'cover-thigh.{tag}l', relief(thigh, sweep_union(calf_fat, jp(kn), [(ja(kn), -2, 130)], step=4.0)))
        else:                                                  # tiny: a 35 mm thigh, relieved for the fold (no clevis room)
            add(f'cover-thigh.{tag}l', relief(thigh, sweep_union(M_(calf), jp(kn), [(ja(kn), -2, 102)], scale=1.02, step=4.0)))
        Ha, ca, ra, La = housing(ank, ja(an), 'rs00', pad=-8.0, fil=5.0, extra_len=26.0)     # the ankle U-joint: a small drum, no motor in it
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
        Hsr, csr, rsr, Lsr = housing(U['sr'][0] + np.array([3.0, 0, 0]), U['sr'][1], 'rs00', extra_len=22.0)   # long enough to hold the yoke + upper-arm plates
        add(f'cover-shoulder.{tag}a', Hsr); add(f'puck-shoulder.{tag}a', csr)
        Hue, cue, rue, Lue = housing(U['ep'][0] + np.array([2.0, 0, -6.0]), U['ep'][1], 'rs00', pad=6.0, extra_len=20.0)
        Hel, cel, rel, Lel = housing(U['ey'][0], U['ey'][1], 'rs00', extra_len=14.0)   # room for the elbow + forearm plates
        add(f'trim-upperarm.{tag}a', Hue)              # the upper-arm rotation: a gunmetal drum mid-arm
        add(f'cover-elbow.{tag}a', Hel); add(f'puck-elbow.{tag}a', cel)
        epc = U['ep'][0]
        add(f'cover-upperarm.{tag}a', bone(U['sr'][0] + np.array([0, 0, -rsr * 0.5]), rsr * 0.8, epc, rue * 0.9, lateral=(0, 1.0, 0), half_w=Lsr / 2 - 8))
        ey_ = U['ey'][0]
        tip = ey_ + KS.hang(J, M, side)
        wrist = ey_ + (tip - ey_) * 0.62
        fo = np.array([0, s * 5.0, 0])                      # forearm rides outboard of the inner elbow plate
        fore = bone(ey_ + fo, rel * 0.8, wrist + fo, 18.0, lateral=(0, 1.0, 0), half_w=28.0)
        add(f'cover-forearm.{tag}a', fore)
        # lower upper arm: rotation drum down to the elbow, full width, with the forearm's fold cut out of it (a clevis, like the knee)
        low = bone(np.array([epc[0], epc[1], Hue.bounds[0][2] - 1.0]), rue * 0.9, ey_, rel * 0.85, flat=1.25, lateral=(0, 1.0, 0), half_w=33.0)
        fat = M_(bone(ey_ + fo, rel * 0.8 + 3, wrist + fo, 21.0, lateral=(0, 1.0, 0), half_w=29.5))
        add(f'cover-elbowfork.{tag}a', relief(low, sweep_union(fat, ey_, [(U['ey'][1], -142, 2)], step=4.0)))
        u = unit(tip - wrist); v = unit(np.cross(u, [0, 1.0, 0])) if abs(u[1]) < 0.9 else unit(np.cross(u, [1.0, 0, 0])); w = np.cross(u, v)
        palm = [ring(wrist + u * t, w, v, 13 * f, 22 * f, n=2.4) for t, f in ((4, 0.9), (18, 1.0), (40, 1.0), (56, 0.85), (66, 0.55), (71, 0.25))]
        add(f'cover-hand.{tag}a', hull_rings(palm))
        add(f'trim-wrist.{tag}a', cyl(wrist, u, 15.0, 5.0))
                # shoulder yoke: the shoulder-pitch drum, out of the chest flank into the shoulder drum (the arm hangs from it)
        spc, src = U['sp'][0], U['sr'][0]
        yc_ = np.array([(spc[0] + src[0]) / 2, 0, (spc[2] + src[2]) / 2])
        secs = [rrect(np.array([yc_[0], s * y_, yc_[2]]), np.array([1.0, 0, 0]), np.array([0, 0, 1.0]), 33.0, 32.0, 13.0) for y_ in (KS.CHEST() - 13.0, abs(src[1]) - Lsr / 2 + 8.0)]   # stops at the shoulder drum's inner face: the upper arm swings clear
        yoke = tm(Manifold.hull_points(np.concatenate(secs).astype(np.float32)))
        ua = M_(dict(out)[f'cover-upperarm.{tag}a'])
        add(f'cover-shoulderyoke.{tag}a', relief(yoke, sweep_union(ua, U['sr'][0], [(U['sr'][1], -25, 25)], scale=1.03, step=3.0)))   # upper arm swings in clear
    # ---- torso: chest, accent waist, pelvis (inside the K1's trunk envelope, our own blocks)
    T = M['Trunk']; lo, hi = T.bounds
    zs = J['ALeft_Shoulder_Pitch']['p'][2]
    Hl0 = KS.head_layout(J); z_d1 = Hl0['yaw'][0][2] - 25.5 - KS.G - KS.T
    zc1 = max(zs + 28 * KS.R(), z_d1 + KS.T + 3.0)                # the chest closes over the neck-yaw deck
    add('cover-chest', block(lo[0] + 4, hi[0] - 6, KS.CHEST(), 40.0 * KS.R(), zc1, taper=(1.0, 1.0), dome=14.0 * KS.R()))
    add('accent-waist', block(lo[0] + 8, hi[0] - 10, KS.CHEST() - 3.0, 14.0 * KS.R(), 40.0 * KS.R(), RC=18.0 * KS.R()))
    # pelvis, K1-style: a block that wraps the two hidden hip-roll cans and follows their round bellies in,
    # then a narrow keel dropping between the hip drums
    rl = KS.layout(J, 'Left')['roll'][0]; m06 = KS.BIG(); Rk = KS.R()
    x0, x1 = lo[0] + 10, hi[0] - 12; xm, xh = (x0 + x1) / 2, (x1 - x0) / 2
    xh = max(xh, (max(abs(rl[0] - m06['L'] / 2 - xm), abs(rl[0] + m06['L'] / 2 - xm)) + 5.0) / 0.88)   # deep enough for the roll cans
    secs = [rrect(np.array([xm, 0, 14.0 * Rk]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), KS.CHEST(), xh, 22.0 * Rk)]
    for th in np.linspace(0, np.pi / 2, 7):                     # the cans' lower quadrant, 5 mm proud
        r_ = m06['D'] / 2 + 5.0
        y_, z_ = rl[1] + r_ * np.cos(th), rl[2] - r_ * np.sin(th)
        secs.append(rrect(np.array([xm, 0, z_]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), max(y_, 30.0 * Rk), xh * (1 - 0.12 * np.sin(th)), 22.0 * Rk))
    upper = Manifold.hull_points(np.concatenate(secs).astype(np.float32))
    zk = rl[2] - m06['D'] / 2 - 5.0
    keel = [rrect(np.array([xm, 0, z_]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), w_ * Rk, xh * f_, 16.0 * Rk)
            for z_, w_, f_ in ((zk + 20 * Rk, 46.0, 0.86), (zk - 26 * Rk, 38.0, 0.7), (zk - 34 * Rk, 30.0, 0.55))]
    pel = upper + Manifold.hull_points(np.concatenate(keel).astype(np.float32))
    got = dict(out)
    for side in ('Left', 'Right'):
        t = side[0].lower(); lay = KS.layout(J, side)
        collar = M_(got[f'trim-hipneck.{t}l']) + M_(got[f'trim-hipyaw.{t}l'])
        pel = pel - sweep_union(collar, lay['pitch'][0], [((0, 1.0, 0), -110, 50)], scale=1.04)          # the thigh swings under the pelvis
        hr_m = dict(KS.leg(J, side))[f'chassis-hipR.{t}l']
        # the hip-roll bracket crosses the gap under the pelvis: sheathe it in a gunmetal guard so no bare aluminium shows
        upper = tm(M_(hr_m) ^ Manifold.cube([400, 400, 400]).translate([-200, -200, lay['pitch'][0][2] - 20.0]))
        gv = np.concatenate([upper.vertices + np.array([dx, 0, 0]) for dx in (-2.0, 2.0)])
        guard = tm(Manifold.hull_points(gv.astype(np.float32)))
        add(f'trim-hipguard.{t}l', guard)
        gc_ = guard.bounds.mean(0)
        hipr = M_(guard).translate(list(-gc_)).scale([1.2, 1.0, 1.0]).translate(list(gc_)) + M_(hr_m) + M_(got[f'cover-hip.{t}l'])   # slot with running clearance
        pel = pel - sweep_union(hipr, lay['roll'][0], [((1.0, 0, 0), -24, 48)], scale=1.03, step=2.0)             # and rolls out from it
    add('cover-pelvis', tm(pel))
    # ---- neck + head: a graphite neck drum, an egg helmet with a visor band
    Hl = KS.head_layout(J)
    if KS.V['name'] == 'mini':
        nz0, nw = Hl['yaw'][0][2] + 20.0, (30.0, 34.0)
    else:                                                       # yaw motor up in the neck: a wider neck from just above the chest dome
        nz0, nw = zc1 + 14.0 * KS.R() + 1.0, (33.0, 36.0)
    nz1 = Hl['pitch'][0][2] - 10.0
    neck = [rrect(np.array([Hl['yaw'][0][0], 0, z_]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), nw[0], nw[1], 14.0) for z_ in (nz0, nz1)]
    add('trim-neck', hull_rings(neck))
    if KS.V['name'] != 'mini':                                  # round neck base, coaxial with the yaw: chest dome up into the neck
        yz0 = zc1 + 2.0
        add('trim-neckbase', cyl(np.array([Hl['yaw'][0][0], 0, (yz0 + nz0 + 4) / 2]), [0, 0, 1.0], 31.5, nz0 + 4 - yz0))
    Hh = M['Head_2']; hlo, hhi = Hh.bounds
    hc = (hlo + hhi) / 2
    hh, hw, hd = (hhi[2] - hlo[2]) * 0.86, (hhi[1] - hlo[1]) / 2 * 0.84, (hhi[0] - hlo[0]) / 2 * 0.98
    hc = hc + np.array([0, 0, -6.0])
    hpz = KS.head_layout(J)['pitch'][0][2]
    if KS.V['name'] != 'mini' and hpz > hc[2] + 0.05 * hh:       # head motors stacked above the shoulders: lift the helmet onto them
        dz = hpz - (hc[2] + 0.05 * hh)
        hc = hc + np.array([0, 0, dz]); hlo = hlo + np.array([0, 0, dz]); hhi = hhi + np.array([0, 0, dz])
    hr_ = []
    for t in np.linspace(-1, 1, 25):                           # hybrid: flat side panels, domed crown + jaw
        k = math.sqrt(max(0.0, 1 - t * t)) * (0.94 if t < -0.4 else 1.0)
        a_, b_ = hw * max(k, 0.06) ** 0.6, hd * max(k, 0.06)       # sides stay flat further up (k**0.6): flat cheeks, domed crown
        hr_.append(rrect(np.array([hc[0], hc[1], hc[2] + t * hh / 2]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), a_, b_, 0.6 * min(a_, b_), k=14))
    helmet = Manifold.hull_points(np.concatenate(hr_).astype(np.float32))
    hpc = KS.head_layout(J)['pitch'][0]
    pr, px = KS.V['pocket']
    pocket = Manifold.cylinder(2 * pr, pr, pr, 72).rotate([-90, 0, 0]).translate([hpc[0], -pr, hpc[2]]) + \
        Manifold.cube([2 * px, 2 * pr, hpc[2] - hlo[2] + 20]).translate([hpc[0] - px, -pr, hlo[2] - 20])
    helmet = helmet - pocket
    add('trim-headhub', cyl(hpc, [0, 1.0, 0], 33.0, 56.0))           # the head-pitch hub, coaxial: lives in the helmet's opening
    hm = tm(helmet)
    hm.metadata.update(loft=[np.asarray(r, float) for r in hr_],      # native: loft minus (y-cylinder r pr about the pitch axis + box)
                       cut=[('ycyl', hpc, pr), ('box', [hpc[0] - px, -pr, hlo[2] - 20], [2 * px, 2 * pr, hpc[2] - hlo[2] + 20])])
    add('cover-helmet', hm)
    # the face: an oval of gloss black glass, the front of the helmet a hair proud
    oval = trimesh.creation.cylinder(radius=1.0, height=1.0, sections=96).apply_transform(
        trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])).apply_scale([hd * 1.2, hw * 0.74, hh * 0.3]).apply_translation(
        [hc[0] + hd * 0.6, hc[1], hc[2] + hh * 0.04])
    big = M_(tm(helmet).copy().apply_translation(-hc).apply_scale(1.01).apply_translation(hc))
    vm = tm(big ^ M_(oval))
    vm.metadata.update(visor=dict(helmet=hm.metadata, hc=hc, s=1.01, oval_c=[hc[0] + hd * 0.6, hc[1], hc[2] + hh * 0.04], oval_s=[hd * 1.2, hw * 0.74, hh * 0.3]))
    add('visor-face', vm)
    # ear caps: the head-pitch axis, orange, both sides
    hp_ = J['Head_pitch']['p']
    for s in (1, -1):
        add(f'puck-ear.{"l" if s > 0 else "r"}', cyl(np.array([hp_[0], s * (hw - 1.0), hc[2] - hh * 0.02]), [0, 1.0, 0], 17.0, 4.0))
    return out


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
    return tm(parts[0])                                        # router-cut part = the main body (drops slivers)


def structure(out=None):
    """the plate frame clipped inside its shells; motors pass through untouched."""
    out = out or design()
    S = Manifold.batch_boolean([Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
                                for n, m in out if n.split('-')[0] in ('cover', 'puck', 'trim', 'accent')], OpType.Add)
    return [(n, m if (n.startswith('rs') or 'hipR' in n or 'elbowE2' in n) else clip(m, S)) for n, m in KS.structure()]   # these span a joint's parting gap


def model():
    out = design()
    out = out + structure(out)
    out = [(n, K.simplify(m, grid=1.0) if len(m.faces) > 20000 else m) for n, m in out]   # swept relief cuts: decimate for the viewer only
    return dict(hip_x=0.0, plane=0.0, legs=[], body=None, walls=[], extra=out,
                note='mini humanoid: our coaxial housings + bone links on the K1 joint layout, RobStride inside every drum',
                posed={}, only=True, ground=True)
