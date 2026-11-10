"""Industrial design over the mini humanoid's packaging.

Rules:
 1. nothing structural shows -- every shell encloses all the parts of its segment;
 2. every joint is a concentric graphite hub; the shells either side end in arcs around it, 1.5 mm gap;
 3. one surface family -- shells lofted through designed sections (one superellipse exponent), widths
    from a smooth CONCAVE envelope of the packaging, not per-slice bumps;
 4. parting lines on one coronal plane through the body;
 5. primary forms: helmet + full-face visor, chest / accent band / pelvis, hip and shoulder lobes,
    moulded outsole.
"""
import math, re
import numpy as np
import trimesh
from manifold3d import Manifold, Mesh

GAP = 1.5            # shell-to-hub gap
SKIN = 2.5           # shell standoff over the packaging
N = 2.25             # the one superellipse exponent: soft, not boxy


def mm(t):
    return Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32))


def tm(M):
    g = M.to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=False)


def M_(t):
    return Manifold(mm(t))


def ring(c, v, w, a, b, n=N, k=48):
    t = np.linspace(0, 2 * np.pi, k, endpoint=False)
    ct, st = np.cos(t), np.sin(t)
    x = a * np.sign(ct) * np.abs(ct) ** (2 / n)
    y = b * np.sign(st) * np.abs(st) ** (2 / n)
    return c[None, :] + x[:, None] * v[None, :] + y[:, None] * w[None, :]


def concave_upper(xs, ys):
    """the smallest concave function over the points (xs, ys): a smooth bulge, never a dent."""
    pts = sorted(zip(xs, ys))
    hull = []
    for p in pts:
        while len(hull) >= 2:
            (x1, y1), (x2, y2) = hull[-2], hull[-1]
            if (x2 - x1) * (p[1] - y1) - (y2 - y1) * (p[0] - x1) >= 0:   # p is above the chord: drop the middle
                hull.pop()
            else:
                break
        hull.append(p)
    hx, hy = zip(*hull)
    return lambda x: float(np.interp(x, hx, hy))


def frame(a, b, lateral=(0, 1.0, 0)):
    u = np.asarray(b, float) - np.asarray(a, float)
    u /= np.linalg.norm(u)
    v = np.asarray(lateral, float) - u * (np.asarray(lateral) @ u)
    v /= np.linalg.norm(v)
    return u, v, np.cross(u, v)


def designed_shell(P, a_pt, b_pt, ext=(0.0, 0.0), stations=10, taper=(1.0, 1.0), n=N, skin=SKIN, dome=None):
    """a shell along a -> b (extended ext[0] before a, ext[1] past b) over packaging points P: sections are
    the concave envelope of the packaging + skin, scaled by a linear taper (top, bottom) but never below
    the packaging; a dome closes the end that is not cut by a joint (dome = 'a' / 'b')."""
    a_pt, b_pt = np.asarray(a_pt, float), np.asarray(b_pt, float)
    u, v, w = frame(a_pt, b_pt)
    L = np.linalg.norm(b_pt - a_pt)
    Q = P - a_pt
    pu, pv, pw = Q @ u, Q @ v, Q @ w
    u0, u1 = min(pu.min(), 0.0) - ext[0], max(pu.max(), L) + ext[1]
    us = np.linspace(u0, u1, stations * 3)
    env = []
    for i in range(len(us) - 1):
        m = (pu >= us[i]) & (pu <= us[i + 1])
        if m.sum() < 2:
            continue
        env.append(((us[i] + us[i + 1]) / 2, pv[m].min(), pv[m].max(), pw[m].min(), pw[m].max()))
    E = np.array(env)
    fa = concave_upper(E[:, 0], (E[:, 2] - E[:, 1]) / 2 + skin)
    fb = concave_upper(E[:, 0], (E[:, 4] - E[:, 3]) / 2 + skin)
    cvs = np.polyfit(E[:, 0], (E[:, 1] + E[:, 2]) / 2, 1)
    cws = np.polyfit(E[:, 0], (E[:, 3] + E[:, 4]) / 2, 1)
    rings = []
    span = (u1 - u0) / max(stations - 1, 1)
    for x in np.linspace(u0, u1, stations):
        t = (x - u0) / max(u1 - u0, 1)
        k = taper[0] + (taper[1] - taper[0]) * t
        aa, bb = fa(x) * k, fb(x) * k
        cv, cw = np.polyval(cvs, x), np.polyval(cws, x)
        # fit in the superellipse metric: grow the section until every packaging point near it is inside (+ skin)
        m = np.abs(pu - x) <= span
        if m.any():
            dv = np.abs(pv[m] - cv) + skin * 0.7
            dw = np.abs(pw[m] - cw) + skin * 0.7
            g = float(((dv / aa) ** n + (dw / bb) ** n).max() ** (1.0 / n))
            if g > 1.0:
                aa, bb = aa * g, bb * g
        rings.append(ring(a_pt + x * u + cv * v + cw * w, v, w, aa, bb, n))
    for end in (dome or ''):
        x = u0 if end == 'a' else u1
        sg = -1 if end == 'a' else 1
        aa, bb = fa(x), fb(x)
        for f, d in ((0.96, 0.25), (0.85, 0.5), (0.62, 0.78), (0.3, 0.95)):
            rings.append(ring(a_pt + (x + sg * d * 0.55 * min(aa, bb)) * u + np.polyval(cvs, x) * v + np.polyval(cws, x) * w, v, w, aa * f, bb * f, n))
    return Manifold.hull_points(np.concatenate(rings).astype(np.float32))


def cyl(c, ax, r, h, sections=96):
    ax = np.asarray(ax, float) / np.linalg.norm(ax)
    return trimesh.creation.cylinder(radius=r, segment=[np.asarray(c) - ax * h / 2, np.asarray(c) + ax * h / 2], sections=sections)


def halfspace_cut(S, point, normal):
    """remove the side of `point` the normal points to."""
    nrm = np.asarray(normal, float) / np.linalg.norm(normal)
    return S.trim_by_plane(tuple(-nrm), float(-(nrm @ np.asarray(point))))


def axis_of(m):
    P = m.vertices - m.vertices.mean(0)
    val, vec = np.linalg.eigh(np.cov(P.T))
    ext = [np.ptp(P @ vec[:, i]) for i in range(3)]
    i = min(range(3), key=lambda i: abs(ext[(i + 1) % 3] - ext[(i + 2) % 3]))
    return m.vertices.mean(0), vec[:, i], ext[i], max(ext[(i + 1) % 3], ext[(i + 2) % 3]) / 2


def width_along(parts, c, ax):
    P = np.concatenate([p.vertices for p in parts])
    d = (P - c) @ (np.asarray(ax) / np.linalg.norm(ax))
    return d.min(), d.max()


def seam(S, c, normal=(0, 0, 1.0), width=0.8, proud=0.3):
    """a parting line on the coronal plane: the shell's section, a hair proud."""
    sh = tm(S)
    k = 1.0 + proud / max(min(sh.extents) / 2, 1)
    big = M_(sh.copy().apply_translation(-sh.vertices.mean(0)).apply_scale(k).apply_translation(sh.vertices.mean(0)))
    nrm = np.asarray(normal, float)
    slab = trimesh.creation.box(extents=[3000, 3000, width]) if abs(nrm[2]) > 0.9 else trimesh.creation.box(extents=[width, 3000, 3000])
    slab.apply_translation(c)
    return tm(big ^ M_(slab))


def design(extra):
    """the skin for the K1-style mini (body frame: x up, y out, z fore). Returns [(name, mesh)]."""
    P = {}
    for n, m in extra:
        P[n] = m
    out = []
    def have(n): return n in P
    def parts(names): return [P[n] for n in names if n in P]
    def pts(names): return np.concatenate([P[n].vertices for n in names if n in P])
    # ---------------- torso: chest / accent band / pelvis, flank hubs for the hip + shoulder pitch -----------
    tnames = [n for n in P if re.match(r'^(chassis-(side|bar|floor|front|back)(\.[lr])?|int-battery)$', n)]
    T = pts(tnames)
    lo, hi = T.min(0), T.max(0)
    xw = lo[0] + 0.40 * (hi[0] - lo[0])
    box = lambda x0, x1: np.array([[x, y, z] for x in np.linspace(x0, x1, 6) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    chest = designed_shell(box(xw + 14, hi[0]), [xw + 14, 0, 0], [hi[0], 0, 0], stations=8, taper=(0.98, 1.04), n=2.7, skin=4.0, dome='b')
    pelvis = designed_shell(box(lo[0], xw - 14), [lo[0], 0, 0], [xw - 14, 0, 0], stations=6, taper=(0.94, 1.0), n=2.6, skin=4.0, dome='a')
    band = designed_shell(box(xw - 22, xw + 22), [xw - 22, 0, 0], [xw + 22, 0, 0], stations=3, n=3.2, skin=2.0)
    # flank hubs: hip pitch + shoulder pitch, a drum through the torso side at each pitch axis
    flank = []
    for n in [n for n in P if re.match(r'^rs0\d-(lpitch|apitch)\.', n)]:
        c, ax, L, r = axis_of(P[n])
        s = 1 if c[1] > 0 else -1
        rr = r + 3.0
        for S_name in ('chest', 'pelvis'):
            pass
        flank.append((n, np.array([c[0], s * (hi[1] + 4.0 + 3.0), c[2]]), r))
    out += [('cover-chest', tm(chest)), ('cover-pelvis', tm(pelvis)), ('accent-waist', tm(band))]
    out += [('puck-seam-chest', seam(chest, [0, 0, 0])), ('puck-seam-pelvis', seam(pelvis, [0, 0, 0]))]
    # ---------------- limbs -------------------------------------------------------------------------------
    for tag in ('ll', 'lr', 'al', 'ar'):
        leg = tag[0] == 'l'
        mo = 'rs06' if leg else 'rs00'
        mid = f'{mo}-knee.{tag}'
        if not have(mid):
            continue
        # joint centres + axes
        kc, kax, kL, kr = axis_of(P[mid])
        roll = f'{mo}-{tag[0]}roll.{tag}'
        rc, rax, rL, rr = axis_of(P[roll])
        yaw = f'{"rs02" if leg else "rs00"}-{tag[0]}yaw.{tag}'
        yc, yax, yL, yr = axis_of(P[yaw])
        cap = P[f'chassis-cap.{tag}']
        cap_c = cap.vertices.mean(0)
        if (kc - cap_c) @ yax < 0:
            yax = -yax                                          # yaw axis pointing down the limb
        thigh_parts = [f'thigh-inner.{tag}', f'thigh-outer.{tag}', mid, f'chassis-cap.{tag}']
        W = (lambda lo_, hi_: (lo_, hi_))(*width_along(parts(thigh_parts), kc, kax))
        hub_r_knee = kr + 4.5                                   # covers the box plates' round ends (W/2 = r + 3.5)
        # yaw collar plane: the cap's far face
        cap_lo = (cap.vertices - cap_c) @ yax
        collar_pt = cap_c + yax * cap_lo.min()
        # --- lobe (hip / shoulder): roll + yaw stack + brackets, a rounded block; roll hub fore and aft
        lobe_names = [n for n in P if n.endswith('.' + tag) and re.match(r'^(chassis-(pitchplate|rollmount|rollplate|yawmount)|rs0\d-.(roll|yaw)(-out)?|rs0\d-.pitch-out)\.', n)]
        LP = pts(lobe_names)
        lobe = designed_shell(LP, collar_pt - yax * (np.ptp(LP @ yax) * 0.95), collar_pt, stations=6, n=2.4, skin=3.0, dome='a')
        lobe = halfspace_cut(lobe, collar_pt - yax * 2.0, yax)  # stops 2 mm above the collar
        out.append((f'cover-lobe.{tag}', tm(lobe)))
        out.append((f'puck-collar.{tag}', cyl(collar_pt + yax * 0.5, yax, yr + 6.0, 5.0)))
        # roll hub: a drum on the lobe, fore + aft faces
        lw = width_along([tm(lobe)], rc, rax)
        out.append((f'puck-roll.{tag}', cyl(rc + rax * (lw[0] + lw[1]) / 2, rax, rr * 0.82, (lw[1] - lw[0]) + 1.2)))
        # --- thigh / upper arm: collar -> knee hub
        TP = pts(thigh_parts)
        th = designed_shell(TP, collar_pt + yax * 3.0, kc, ext=(0.0, hub_r_knee), stations=9, taper=(1.0, 0.92), n=N)
        th = halfspace_cut(th, collar_pt + yax * 3.0, -yax)
        knee_drum = cyl(kc + kax * (W[0] + W[1]) / 2, kax, hub_r_knee, (W[1] - W[0]) * 0.92)
        th = th - M_(cyl(kc + kax * (W[0] + W[1]) / 2, kax, hub_r_knee + GAP, 2000))
        nm = 'thigh' if leg else 'upperarm'
        out.append((f'cover-{nm}.{tag}', tm(th)))
        out.append((f'puck-{nm}-seam.{tag}', seam(th, [0, 0, kc[2]])))
        out.append((f'puck-knee.{tag}', knee_drum))
        # --- shin / forearm: knee hub -> ankle hub (leg) or the hand (arm)
        if leg:
            ac, aax, aL, ar = axis_of(P[f'pivot-ankle.{tag}'])
            shin_parts = [n for n in P if n.endswith('.' + tag) and re.match(r'^(shin|rs00-ankle(-b)?|crank-ankle(-b)?|link-ankle(-b)?)\.', n)]
            SP = pts(shin_parts)
            Wa = width_along(parts(shin_parts), ac, aax)
            hub_r_ank = 24.0
            sh = designed_shell(SP, kc, ac, ext=(hub_r_knee, hub_r_ank), stations=10, taper=(1.0, 0.82), n=N)
            sh = sh - M_(cyl(kc + kax * (W[0] + W[1]) / 2, kax, hub_r_knee + GAP, 2000)) - M_(cyl(ac, aax, hub_r_ank + GAP, 2000))
            out.append((f'cover-shin.{tag}', tm(sh)))
            out.append((f'puck-shin-seam.{tag}', seam(sh, [0, 0, kc[2]])))
            out.append((f'puck-ankle.{tag}', cyl(ac + aax * (Wa[0] + Wa[1]) / 2, aax, hub_r_ank, (Wa[1] - Wa[0]) + 2 * SKIN)))
            # outsole + upper: a moulded shoe under the ankle hub
            F = P[f'foot.{tag}'].vertices
            FL = F[F[:, 0] <= F[:, 0].min() + 26.0]
            x0, z0, z1 = FL[:, 0].min(), FL[:, 2].min(), FL[:, 2].max()
            yc_, hw = (FL[:, 1].min() + FL[:, 1].max()) / 2, (FL[:, 1].max() - FL[:, 1].min()) / 2 + 3
            rings, sole_r = [], []
            for t in np.linspace(0, 1, 16):
                z = z0 - 4 + t * (z1 - z0 + 10)
                h = 18 + 24 * math.exp(-((t - 0.34) / 0.2) ** 2) - 5 * t
                a = hw * (0.8 + 0.2 * math.sin(math.pi * min(1.0, t / 0.82)))
                if t in (0.0, 1.0):
                    a *= 0.75; h *= 0.85
                rings.append(ring(np.array([x0 + 7 + h / 2, yc_, z]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), a, h / 2, n=3.2))
                sole_r.append(ring(np.array([x0 + 3.5, yc_, z]), np.array([0, 1.0, 0]), np.array([1.0, 0, 0]), a + 1.5, 3.5, n=5.0))
            upper = Manifold.hull_points(np.concatenate(rings).astype(np.float32)) - M_(cyl(ac, aax, hub_r_ank + GAP, 2000))
            out.append((f'cover-shoe.{tag}', tm(upper)))
            out.append((f'sole.{tag}', tm(Manifold.hull_points(np.concatenate(sole_r).astype(np.float32)))))
        else:
            fore = [f'shin.{tag}']
            FP = pts(fore)
            paw = P.get(f'paw.{tag}')
            pc = paw.centroid if paw is not None else FP.mean(0)
            sh = designed_shell(FP, kc, pc, ext=(hub_r_knee, 0.0), stations=8, taper=(1.0, 0.8), n=N)
            sh = sh - M_(cyl(kc + kax * (W[0] + W[1]) / 2, kax, hub_r_knee + GAP, 2000))
            sh = halfspace_cut(sh, pc, (pc - kc))
            out.append((f'cover-forearm.{tag}', tm(sh)))
            out.append((f'puck-forearm-seam.{tag}', seam(sh, [0, 0, kc[2]])))
            u = (pc - kc) / np.linalg.norm(pc - kc)
            wv = np.array([0, 0, 1.0]) - u * u[2]; wv /= np.linalg.norm(wv)
            vv = np.cross(wv, u)
            palm = [ring(pc + u * t, vv, wv, 14 * f, 25 * f, n=2.4) for t, f in ((2, 0.9), (20, 1.0), (45, 1.0), (62, 0.85), (72, 0.55), (77, 0.25))]
            thumb = [ring(pc + u * 10 + wv * 24 + (u * 0.8 + wv * 0.5) * t, vv, np.cross(u * 0.8 + wv * 0.5, vv) / np.linalg.norm(np.cross(u * 0.8 + wv * 0.5, vv)), 7, 7.5, n=2.2) for t in (0, 12, 20)]
            hand = Manifold.hull_points(np.concatenate(palm).astype(np.float32)) + Manifold.hull_points(np.concatenate(thumb).astype(np.float32))
            out.append((f'cover-hand.{tag}', tm(hand)))
            out.append((f'puck-wrist.{tag}', cyl(pc - u * 3, u, 15.0, 6.0)))
    # flank hubs (drawn after the lobes so their size can match)
    for n, c, r in flank:
        out.append((f'puck-flank.{n.split(".")[1]}', cyl(c, [0, 1.0, 0], r + 4.0, 7.0)))
    # ---------------- head: helmet + full-face visor, neck collar ----------------------------------------
    hn = [n for n in P if re.match(r'^(head-.*|rs00-neckpitch|chassis-neck-plate|int-camera)$', n)]
    HP = pts(hn)
    hlo, hhi = HP.min(0), HP.max(0)
    hc = (hlo + hhi) / 2
    hh = (hhi[0] - hlo[0])
    hw = (hhi[1] - hlo[1]) / 2 + 5
    hd = (hhi[2] - hlo[2]) / 2 + 5
    # a helmet: fitted over the head box in the superellipse metric, full domes top and bottom
    helmet = designed_shell(HP, [hlo[0], hc[1], hc[2]], [hhi[0], hc[1], hc[2]], stations=7, n=2.3, skin=5.0, dome='ab')
    out.append(('cover-helmet', tm(helmet)))
    vbox = trimesh.creation.capsule(height=hw * 1.3, radius=hh * 0.17, count=[32, 32]).apply_transform(
        trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])).apply_scale([1.0, 1.0, 2.2]).apply_translation([hc[0] + hh * 0.12, hc[1], hc[2] + hd * 0.75])   # a goggle band across the face
    hs = tm(helmet)
    big = M_(hs.copy().apply_translation(-hc).apply_scale(1.012).apply_translation(hc))
    out.append(('visor-face', tm(big ^ M_(vbox))))
    out.append(('puck-helmet-seam', seam(helmet, [0, 0, hc[2] - hd * 0.1])))
    nk = [n for n in P if n in ('rs00-neckyaw', 'chassis-neck')]
    if nk:
        NP = pts(nk)
        out.append(('puck-neck', cyl([(NP[:, 0].min() + NP[:, 0].max()) / 2, 0, 0], [1, 0, 0], 26.0, NP[:, 0].max() - NP[:, 0].min() + 6)))
    return out


def enclosure(extra, skin_parts, structural=r'^(chassis-|thigh-|shin|yoke|crank-|link-|pivot-|rs0\d-|foot|int-|head-)'):
    """structure left outside the skin (cover + puck + sole + visor + accent): mm^3 per part."""
    S = None
    for n, m in skin_parts:
        if n.startswith(('cover-', 'puck-', 'sole', 'visor-', 'accent-')):
            try:
                S = M_(m) if S is None else S + M_(m)
            except Exception:
                pass
    bad = []
    for n, m in extra:
        if not re.match(structural, n):
            continue
        try:
            v = (M_(m) - S).volume()
        except Exception:
            continue
        if v > 50:
            bad.append((n, round(v)))
    return sorted(bad, key=lambda t: -t[1])
