"""MINI humanoid, our own look on the K1's joint layout: every joint a rounded coaxial housing around OUR
RobStride (graphite caps in both ends), every link a bone lofted between its joints' housings (slimmer
than the joints, a swell for thigh / calf / forearm), the hip pitch-roll-yaw cluster merged into one lobe,
our own chest + pelvis blocks with the accent waist, an egg helmet with a visor band, shoes, mittens.
K1 frame: x fore, y left, z up (mm). Joint positions + axes from the K1 URDF (arms lowered)."""
import math
import numpy as np, trimesh
from manifold3d import Manifold
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


def bone(p0, r0, p1, r1, swell=1.0, flat=1.0, lateral=(0, 1.0, 0), n=2.2):
    """a link lofted from a section at p0 to one at p1 (radii r0, r1), with a swell at 40 %."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    u = unit(p1 - p0)
    v = unit(np.asarray(lateral, float) - u * (np.asarray(lateral) @ u)); w = np.cross(u, v)
    rings = []
    for t, k in ((0.0, 1.0), (0.18, 1.0 + 0.6 * (swell - 1)), (0.42, swell), (0.7, 1.0 + 0.5 * (swell - 1)), (1.0, 1.0)):
        r = r0 + (r1 - r0) * t
        rings.append(ring(p0 + u * t * np.linalg.norm(p1 - p0), v, w, r * k * flat, r * k, n=n))
    return tm(Manifold.hull_points(np.concatenate(rings).astype(np.float32)))


def block(x0, x1, y, z0, z1, n=2.8, taper=(1.0, 1.0), dome=0.0):
    """a rounded torso block: superellipse sections (x fore, y side) stacked up z, tapered."""
    rings = []
    for t in np.linspace(0, 1, 9):
        z = z0 + (z1 - z0) * t
        k = taper[0] + (taper[1] - taper[0]) * t
        rings.append(ring(np.array([(x0 + x1) / 2, 0, z]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), y * k, (x1 - x0) / 2 * (0.94 + 0.06 * k), n=n))
    if dome:
        for f, d in ((0.9, 0.4), (0.7, 0.75), (0.4, 1.0)):
            rings.append(ring(np.array([(x0 + x1) / 2, 0, z1 + dome * d]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), y * taper[1] * f, (x1 - x0) / 2 * f, n=n))
    return tm(Manifold.hull_points(np.concatenate(rings).astype(np.float32)))


def design():
    J, M = KT.world(K.arms_down())
    out = []
    add = lambda n, m: out.append((n, m))
    for side, s in (('Left', 1), ('Right', -1)):
        tag = side[0].lower()
        jp = lambda n: J[n]['p']; ja = lambda n: J[n]['axis']
        # ---- leg: hip lobe (pitch + roll + yaw), thigh, knee, calf (holds the 2 ankle motors), ankle, shoe
        hp, hr, hy, kn, an = (f'{side}_Hip_Pitch', f'{side}_Hip_Roll', f'{side}_Hip_Yaw', f'{side}_Knee_Pitch', f'{side}_Ankle_Pitch')
        Hp, cp, rp, Lp = housing(jp(hp), ja(hp), 'rs06')
        Hr, cr, rr, Lr = housing(jp(hr), ja(hr), 'rs06')
        Hy, cy_, ry_, Ly = housing(jp(hy), ja(hy), 'rs02', pad=3.0)
        lobe = tm(Manifold.hull_points(np.concatenate([Hp.vertices, Hr.vertices, Hy.vertices]).astype(np.float32)))
        add(f'cover-hiplobe.{tag}l', lobe)
        add(f'puck-hipcaps.{tag}l', trimesh.util.concatenate([cp, cr]))
        yaw_bot = jp(hy) - unit(ja(hy)) * (Ly / 2) * np.sign(unit(ja(hy))[2])
        Hk, ck, rk, Lk = housing(jp(kn), ja(kn), 'rs06')
        add(f'cover-knee.{tag}l', Hk); add(f'puck-knee.{tag}l', ck)
        add(f'puck-yawring.{tag}l', cyl(yaw_bot, [0, 0, 1.0], ry_ - 2, 4.0))
        add(f'cover-thigh.{tag}l', bone(yaw_bot - np.array([0, 0, 2.0]), ry_ * 0.92, jp(kn), rk * 0.86, swell=1.08, flat=0.92))
        ank = jp(an)
        add(f'cover-calf.{tag}l', bone(jp(kn), rk * 0.84, ank + np.array([0, 0, 22.0]), 24.0, swell=1.12, flat=0.9))
        Ha, ca, ra, La = housing(ank, ja(an), 'rs00', pad=-8.0, fil=5.0)     # the ankle U-joint: a small drum, no motor in it
        add(f'puck-ankle.{tag}l', Ha)
        F = M[f'{side.lower()}_foot_link']
        lo, hi = F.bounds
        z0 = lo[2]
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
        add(f'cover-shoe.{tag}l', tm(Manifold.hull_points(np.concatenate(rings).astype(np.float32))))
        add(f'sole.{tag}l', tm(Manifold.hull_points(np.concatenate(sole).astype(np.float32))))
        # ---- arm: shoulder roll drum, upper arm (through the upper-arm rotation drum), elbow, forearm, mitten
        sp, sr, ep, ey = (f'A{side}_Shoulder_Pitch', f'{side}_Shoulder_Roll', f'{side}_Elbow_Pitch', f'{side}_Elbow_Yaw')
        Hsr, csr, rsr, Lsr = housing(jp(sr), ja(sr), 'rs00')
        add(f'cover-shoulder.{tag}a', Hsr); add(f'puck-shoulder.{tag}a', csr)
        Hue, cue, rue, Lue = housing(jp(ep), ja(ep), 'rs00', pad=2.0)
        Hel, cel, rel, Lel = housing(jp(ey), ja(ey), 'rs00')
        add(f'cover-elbow.{tag}a', Hel); add(f'puck-elbow.{tag}a', cel)
        add(f'cover-upperarm.{tag}a', bone(jp(sr) + np.array([0, 0, -rsr * 0.5]), rsr * 0.8, jp(ey), rel * 0.8, swell=1.06, lateral=(1.0, 0, 0)))
        add(f'puck-upperarmring.{tag}a', cyl(jp(ep) - unit(ja(ep)) * 6, ja(ep), rue - 1.0, 4.0))
        Hd = M[f'{side.lower()}_hand_link']
        tip = Hd.vertices[np.argmin(Hd.vertices[:, 2])]
        wrist = jp(ey) + (tip - jp(ey)) * 0.62
        add(f'cover-forearm.{tag}a', bone(jp(ey), rel * 0.8, wrist, 18.0, swell=1.1, lateral=(1.0, 0, 0)))
        u = unit(tip - wrist); v = unit(np.cross(u, [0, 1.0, 0])) if abs(u[1]) < 0.9 else unit(np.cross(u, [1.0, 0, 0])); w = np.cross(u, v)
        palm = [ring(wrist + u * t, w, v, 13 * f, 22 * f, n=2.4) for t, f in ((4, 0.9), (18, 1.0), (40, 1.0), (56, 0.85), (66, 0.55), (71, 0.25))]
        add(f'cover-hand.{tag}a', tm(Manifold.hull_points(np.concatenate(palm).astype(np.float32))))
        add(f'puck-wrist.{tag}a', cyl(wrist, u, 15.0, 5.0))
        # torso flank caps: hip pitch + shoulder pitch show on the body
        add(f'puck-shoulderpitch.{tag}a', cyl(jp(sp) + np.array([0, s * 26.0, 0]), [0, 1.0, 0], 24.0, 4.0))
    # ---- torso: chest, accent waist, pelvis (inside the K1's trunk envelope, our own blocks)
    T = M['Trunk']; lo, hi = T.bounds
    zs = J['ALeft_Shoulder_Pitch']['p'][2]
    add('cover-chest', block(lo[0] + 4, hi[0] - 6, 96.0, 40.0, zs + 28, n=2.9, taper=(0.86, 1.0), dome=18.0))
    add('accent-waist', block(lo[0] + 12, hi[0] - 14, 80.0, 14.0, 40.0, n=2.9))
    add('cover-pelvis', block(lo[0] + 8, hi[0] - 10, 86.0, lo[2] + 10, 14.0, n=2.8, taper=(0.9, 0.95)))
    # ---- neck + head: a graphite neck drum, an egg helmet with a visor band
    Hn, cn, rn, Ln = housing(J['AAHead_yaw']['p'] + np.array([0, 0, 6.0]), J['AAHead_yaw']['axis'], 'rs00', pad=-6.0)
    add('puck-neck', Hn)
    Hh = M['Head_2']; hlo, hhi = Hh.bounds
    hc = (hlo + hhi) / 2
    hh, hw, hd = (hhi[2] - hlo[2]) * 0.92, (hhi[1] - hlo[1]) / 2 * 0.86, (hhi[0] - hlo[0]) / 2 * 0.95
    hr_ = []
    for t in np.linspace(-1, 1, 17):
        k = math.sqrt(max(0.0, 1 - abs(t) ** 2.4))
        hr_.append(ring(np.array([hc[0], hc[1], hc[2] + t * hh / 2]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), hw * max(k, 0.1), hd * max(k, 0.1), n=2.2))
    helmet = Manifold.hull_points(np.concatenate(hr_).astype(np.float32))
    add('cover-helmet', tm(helmet))
    vis = trimesh.creation.box(extents=[hd * 1.2, hw * 2.2, hh * 0.24]).apply_translation([hc[0] + hd * 0.75, hc[1], hc[2] + hh * 0.06])
    big = M_(tm(helmet).copy().apply_translation(-hc).apply_scale(1.012).apply_translation(hc))
    add('visor-face', tm(big ^ M_(vis)))
    return out


def model():
    out = design()
    return dict(hip_x=0.0, plane=0.0, legs=[], body=None, walls=[], extra=out,
                note='mini humanoid: our coaxial housings + bone links on the K1 joint layout, RobStride inside every drum',
                posed={}, only=True, ground=True)
