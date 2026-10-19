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
    return tm(Manifold.hull_points(np.concatenate(rings).astype(np.float32)))


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
    return tm(Manifold.hull_points(np.concatenate(rings).astype(np.float32)))


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
        Hp, cp, rp, Lp = housing(lay['pitch'][0], ja(hp), 'rs06', extra_len=26.0)
        Hy, cy_, ry_, Ly = housing(lay['yaw'][0], ja(hy), 'rs02', pad=3.0)
        # the hip: one drum under the pelvis flank, the same language as the knee; the roll motor hides inside the pelvis
        pod = Hp
        add(f'cover-hip.{tag}l', pod)
        add(f'puck-hip.{tag}l', cp)
        add(f'trim-hipyaw.{tag}l', Hy)
        z_pb, z_yt = pod.bounds[0][2], lay['yaw'][0][2] + Ly / 2
        add(f'trim-hipneck.{tag}l', cyl(np.array([lay['yaw'][0][0], lay['yaw'][0][1], z_yt - 4]), [0, 0, 1.0], ry_ - 1.0, z_pb - z_yt + 16))   # pod into the yaw collar
        yaw_bot = lay['yaw'][0] - unit(ja(hy)) * (Ly / 2) * np.sign(unit(ja(hy))[2])
        Hk, ck, rk, Lk = housing(jp(kn), ja(kn), 'rs06', extra_len=34.0)    # encloses the thigh plate pair (the clevis cheeks)
        add(f'cover-knee.{tag}l', Hk); add(f'puck-knee.{tag}l', ck)
        add(f'trim-yawring.{tag}l', cyl(yaw_bot, [0, 0, 1.0], ry_ - 2, 4.0))
        thigh = bone(yaw_bot + np.array([0, 0, 12.0]), ry_ * 0.92, jp(kn), rk * 0.86, swell=1.0, flat=1.3, n=3.0, half_w=Lk / 2)
        ank = jp(an)
        # the calf nests between the thigh plates (a clevis knee): narrower, centred on the shin plates
        yo = np.array([0, s * 4.5, 0])
        calf = bone(jp(kn) + yo, rk * 0.84, ank + yo + np.array([0, 0, 14.0]), 30.0, flat=1.3, n=3.0, half_w=34.5)
        add(f'cover-calf.{tag}l', calf)
        calf_fat = M_(bone(jp(kn) + yo, rk * 0.84 + 3, ank + yo + np.array([0, 0, 14.0]), 33.0, flat=1.3, n=3.0, half_w=37.5))
        add(f'cover-thigh.{tag}l', relief(thigh, sweep_union(calf_fat, jp(kn), [(ja(kn), -2, 130)], step=4.0)))
        Ha, ca, ra, La = housing(ank, ja(an), 'rs00', pad=-8.0, fil=5.0, extra_len=26.0)     # the ankle U-joint: a small drum, no motor in it
        add(f'trim-ankle.{tag}l', Ha)
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
        shoe = tm(Manifold.hull_points(np.concatenate(rings).astype(np.float32)))
        lower = (M_(calf) + M_(Ha)) ^ Manifold.cube([400, 400, 200]).translate([ank[0] - 200, ank[1] - 200, ank[2] - 40])
        add(f'cover-shoe.{tag}l', relief(shoe, sweep_union(lower, ank, [(ja(an), -52, 52), ([1.0, 0, 0], -22, 22)], scale=1.07, step=6.0)))   # instep pocket for the ankle
        add(f'sole.{tag}l', tm(Manifold.hull_points(np.concatenate(sole).astype(np.float32))))
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
        add(f'cover-upperarm.{tag}a', bone(jp(sr) + np.array([0, 0, -rsr * 0.5]), rsr * 0.8, epc, rue * 0.9, lateral=(0, 1.0, 0), half_w=Lsr / 2 - 8))
        ey_ = U['ey'][0]; y_e2 = abs(ey_[1]) - 25.5 - KS.G - KS.T          # inboard face of the elbow plate E2
        cheek = [rrect(np.array([ey_[0], s * (y_e2 - 1.5), z_]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), 30.0, 5.5, 5.0) for z_ in (epc[2] - 30, ey_[2])]
        add(f'cover-elbowfork.{tag}a', tm(Manifold.hull_points(np.concatenate(cheek).astype(np.float32))))
        Hd = M[f'{side.lower()}_hand_link']
        tip = Hd.vertices[np.argmin(Hd.vertices[:, 2])]
        wrist = jp(ey) + (tip - jp(ey)) * 0.62
        fo = np.array([0, s * 3.5, 0])                      # outboard of the fork cheek
        add(f'cover-forearm.{tag}a', bone(jp(ey) + fo, rel * 0.8, wrist + fo, 18.0, lateral=(0, 1.0, 0), half_w=28.3))
        u = unit(tip - wrist); v = unit(np.cross(u, [0, 1.0, 0])) if abs(u[1]) < 0.9 else unit(np.cross(u, [1.0, 0, 0])); w = np.cross(u, v)
        palm = [ring(wrist + u * t, w, v, 13 * f, 22 * f, n=2.4) for t, f in ((4, 0.9), (18, 1.0), (40, 1.0), (56, 0.85), (66, 0.55), (71, 0.25))]
        add(f'cover-hand.{tag}a', tm(Manifold.hull_points(np.concatenate(palm).astype(np.float32))))
        add(f'trim-wrist.{tag}a', cyl(wrist, u, 15.0, 5.0))
                # shoulder yoke: the shoulder-pitch drum, out of the chest flank into the shoulder drum (the arm hangs from it)
        spc, src = U['sp'][0], U['sr'][0]
        yc_ = np.array([(spc[0] + src[0]) / 2, 0, (spc[2] + src[2]) / 2])
        secs = [rrect(np.array([yc_[0], s * y_, yc_[2]]), np.array([1.0, 0, 0]), np.array([0, 0, 1.0]), 33.0, 32.0, 13.0) for y_ in (88.0, abs(src[1]) - Lsr / 2 + 8.0)]   # stops at the shoulder drum's inner face: the upper arm swings clear
        add(f'cover-shoulderyoke.{tag}a', tm(Manifold.hull_points(np.concatenate(secs).astype(np.float32))))
    # ---- torso: chest, accent waist, pelvis (inside the K1's trunk envelope, our own blocks)
    T = M['Trunk']; lo, hi = T.bounds
    zs = J['ALeft_Shoulder_Pitch']['p'][2]
    add('cover-chest', block(lo[0] + 4, hi[0] - 6, 101.0, 40.0, zs + 28, taper=(1.0, 1.0), dome=14.0))
    add('accent-waist', block(lo[0] + 8, hi[0] - 10, 98.0, 14.0, 40.0, RC=18.0))
    pel = M_(block(lo[0] + 10, hi[0] - 12, 101.0, lo[2] + 22, 14.0, n=2.8, taper=(0.92, 1.0)))   # holds both hip-roll motors
    got = dict(out)
    for side in ('Left', 'Right'):
        t = side[0].lower(); lay = KS.layout(J, side)
        collar = M_(got[f'trim-hipneck.{t}l']) + M_(got[f'trim-hipyaw.{t}l'])
        pel = pel - sweep_union(collar, lay['pitch'][0], [((0, 1.0, 0), -110, 50)], scale=1.04)          # the thigh swings under the pelvis
        hipr = M_(dict(KS.leg(J, side))[f'chassis-hipR.{t}l']) + M_(got[f'cover-hip.{t}l'])
        pel = pel - sweep_union(hipr, lay['roll'][0], [((1.0, 0, 0), -22, 47)], scale=1.03)             # and rolls out from it
    add('cover-pelvis', tm(pel.simplify(0.4)))
    # ---- neck + head: a graphite neck drum, an egg helmet with a visor band
    Hl = KS.head_layout(J)
    nz0, nz1 = Hl['yaw'][0][2] + 20.0, Hl['pitch'][0][2] - 10.0
    neck = [rrect(np.array([Hl['yaw'][0][0], 0, z_]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), 30.0, 34.0, 14.0) for z_ in (nz0, nz1)]
    add('trim-neck', tm(Manifold.hull_points(np.concatenate(neck).astype(np.float32))))
    Hh = M['Head_2']; hlo, hhi = Hh.bounds
    hc = (hlo + hhi) / 2
    hh, hw, hd = (hhi[2] - hlo[2]) * 0.86, (hhi[1] - hlo[1]) / 2 * 0.84, (hhi[0] - hlo[0]) / 2 * 0.98
    hc = hc + np.array([0, 0, -6.0])
    hr_ = []
    for t in np.linspace(-1, 1, 25):                           # hybrid: flat side panels, domed crown + jaw
        k = math.sqrt(max(0.0, 1 - t * t)) * (0.94 if t < -0.4 else 1.0)
        a_, b_ = hw * max(k, 0.06) ** 0.6, hd * max(k, 0.06)       # sides stay flat further up (k**0.6): flat cheeks, domed crown
        hr_.append(rrect(np.array([hc[0], hc[1], hc[2] + t * hh / 2]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), a_, b_, 0.6 * min(a_, b_), k=14))
    helmet = Manifold.hull_points(np.concatenate(hr_).astype(np.float32))
    hpc = KS.head_layout(J)['pitch'][0]
    pocket = Manifold.cylinder(80, 40, 40, 72).rotate([-90, 0, 0]).translate([hpc[0], -40, hpc[2]]) + \
        Manifold.cube([92, 80, hpc[2] - hlo[2] + 20]).translate([hpc[0] - 46, -40, hlo[2] - 20])
    helmet = helmet - pocket
    add('trim-headhub', cyl(hpc, [0, 1.0, 0], 33.0, 56.0))           # the head-pitch hub, coaxial: lives in the helmet's opening
    add('cover-helmet', tm(helmet))
    # the face: an oval of gloss black glass, the front of the helmet a hair proud
    oval = trimesh.creation.cylinder(radius=1.0, height=1.0, sections=96).apply_transform(
        trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])).apply_scale([hd * 1.2, hw * 0.74, hh * 0.3]).apply_translation(
        [hc[0] + hd * 0.6, hc[1], hc[2] + hh * 0.04])
    big = M_(tm(helmet).copy().apply_translation(-hc).apply_scale(1.01).apply_translation(hc))
    add('visor-face', tm(big ^ M_(oval)))
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
    return [(n, m if (n.startswith('rs') or 'hipR' in n) else clip(m, S)) for n, m in KS.structure()]   # hip R spans the roll gap


def model():
    out = design()
    out = out + structure(out)
    out = [(n, K.simplify(m, grid=1.0) if len(m.faces) > 20000 else m) for n, m in out]   # swept relief cuts: decimate for the viewer only
    return dict(hip_x=0.0, plane=0.0, legs=[], body=None, walls=[], extra=out,
                note='mini humanoid: our coaxial housings + bone links on the K1 joint layout, RobStride inside every drum',
                posed={}, only=True, ground=True)
