"""Print-ready lymb skin (v10.7): true shells with the attachment features, watertight, one file per part.

    uv run -q --with numpy --with manifold3d --with trimesh --with scipy --with lxml --with networkx \
        python arm/skin_print.py            -> arm/skin-stl/print/ + PRINT.md

Every part is built from the same lofts the render skin uses (skin_v10.LOFTS), so it is the frozen v10.6 skin:
  sleeves (upper / upperd / fore / wrist / turret)  outer loft - inner loft (each station inset by the wall,
        the inner loft run 3 mm past both ends so the ends are OPEN where the plates pass into the joints),
        cut back by the coaxial joint seams, split front / back on the arm's y = 0 parting plane.
        Features: 2 C-clips per half over each holder plate's straight top / bottom edge (a 3.375 mm slot,
        5 mm deep, on the plate's own y), and 2 magnet bosses per half on the parting plane (Ø6.2 x 3.2 pocket
        for an N52 Ø6 x 3 disc). J3 / J7 dot-grids are through-holes.
  ring pucks (J2 / J4 / J6)  a tube, OD = ring, wall 1.2 / 0.9 mm, open both faces; the circumferential dot-grid
        (Ø1.3 on a 2.3 mm hex) meshed through the wall; on each face a 3-spoke spider to 3 of the motor's own
        screws (output pattern on the A face, back pattern on the B face) where the Al standoffs land.
  bezels (6)  0.9 mm gunmetal annuli; bonded to the ring face (VHB) - no holes.
  J3 hairline / J7 accent  a 0.8 / 2 mm wide strip of the sleeve's section, 1.2 mm deep.
  turret, J1 drum, desk puck  shells, open toward the structure.
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import numpy as np
import trimesh
from manifold3d import Manifold, OpType
import rose_fork as F
import skin_v10 as S

OUT = os.path.join(HERE, 'skin-stl', 'print')
_mm, _tm, cyl = S._mm, S._tm, S.cyl
S._EPS = {}


def M(t):
    return Manifold(_mm(t))


def box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return M(trimesh.creation.box(extents=hi - lo).apply_translation((lo + hi) / 2))


def inset(st, w, ext=3.0):
    s = [(s_, a - w, b - w, c) for s_, a, b, c in st]
    s.sort()
    s.insert(0, (s[0][0] - ext,) + s[0][1:])
    s.append((s[-1][0] + ext,) + s[-1][1:])
    return s


def shell(name, wall):
    L = S.LOFTS[name]
    outer = M(S.sleeve(L['st'], **L['kw']))
    inner = M(S.sleeve(inset(L['st'], wall), **L['kw']))
    sh = outer - inner
    S._EPS[name] = M(S.sleeve(inset(L['st'], 0.05, ext=0.0), **L['kw']))   # features stay 0.05 mm inside the skin face: no coplanar unions
    for c, ax, r in L['seams']:
        a = np.zeros(3)
        a[ax] = 300.0
        sh = sh - M(cyl(np.asarray(c) - a, np.asarray(c) + a, r, 128))
    return sh, outer


EPS_FOR = [None]


def plates(link):
    """(y centre, x0, x1, z_top, z_bot) of each holder plate (sheet extruded along y) of a link, world-zero mm."""
    out = []
    for p in F.build():
        if p[0] != link or p[1] != 'sheet' or abs(p[6][1]) < 1e-6:
            continue
        pts = np.array(F.Gm.prim_points(p[:7], step=0.004)) * 1e3
        out.append((pts[:, 1].mean(), pts[:, 0].min(), pts[:, 0].max(), pts))
    return out


def clips(sh, outer, link, x0, x1, side):
    """C-clips over the plates' straight top / bottom edges, on this half (side -1 front / +1 back)."""
    zs = F.Z_SH * 1e3
    feats, slots = [], []
    for yc, px0, px1, pts in plates(link):
        if np.sign(yc) != side:
            continue
        a, b = max(px0, x0) + 8.0, min(px1, x1) - 8.0
        if b - a < 10:
            continue
        for x in (a + 0.25 * (b - a), a + 0.75 * (b - a)):
            near = pts[np.abs(pts[:, 0] - x) < 3.0]
            if len(near) < 4:
                continue
            for edge, sg in ((near[:, 2].max(), 1), (near[:, 2].min(), -1)):
                if abs(edge - zs) < 15 or abs(edge - zs) > 60:   # not a straight neck edge (window, or the lobe) - skip
                    continue
                lo = [x - 3.0, yc - 3.2, min(edge - sg * 5.0, edge + sg * 60.0)]
                hi = [x + 3.0, yc + 3.2, max(edge - sg * 5.0, edge + sg * 60.0)]
                feats.append(box(lo, hi) ^ EPS_FOR[0])
                slots.append(box([x - 4.0, yc - 1.69, min(edge - sg * 6.0, edge + sg * 0.3)],
                                 [x + 4.0, yc + 1.69, max(edge - sg * 6.0, edge + sg * 0.3)]))
    if feats:
        sh = sh + Manifold.batch_boolean(feats, OpType.Add)
        sh = sh - Manifold.batch_boolean(slots, OpType.Add)
    return sh, len(feats)


def magnets(sh, outer, x0, x1, side, zc):
    """2 magnet bosses on the parting plane, top and bottom, Ø6.2 x 3.2 pockets."""
    V = np.asarray(outer.to_mesh().vert_properties)[:, :3]
    xm = (x0 + x1) / 2
    sel = V[np.abs(V[:, 0] - xm) < 6]
    n = 0
    for sg in (1, -1):
        zt = (sel[:, 2].max() if sg > 0 else sel[:, 2].min()) - sg * 6.0
        c = np.array([xm, 0.0, zt])
        boss = M(cyl(c, c + [0, side * 4.0, 0], 5.0, 32)) ^ EPS_FOR[0]
        sh = sh + boss
        sh = sh - M(cyl(c - [0, side * 0.1, 0], c + [0, side * 3.2, 0], 3.1, 32))
        n += 1
    return sh, n


def halves(sh):
    return {'front': sh ^ box([-2000, -2000, -2000], [2000, 0.0, 2000]), 'back': sh ^ box([-2000, 0.0, -2000], [2000, 2000, 2000])}


def grid(sh, outer_tm, x0, x1, zc, **kw):
    """Dot-grid through-holes: reuse skin_v10.perforate's field logic on the outer surface."""
    cnt = S.perforate(outer_tm, x0, x1, zc, count_only=True, **kw)
    holes, n = S.perforate(outer_tm, x0, x1, zc, holes_only=True, **kw)   # 2.5 deep from 1 mm proud: through a 0.9 / 1.2 wall
    return sh - holes, n


def ring(j, owner, wall):
    misc = S.LOFTS['_misc']
    Rp, c = misc['Rp'][j], np.asarray(misc['J'][j], float)
    m = {'J2': F.J2M, 'J4': F.J4M, 'J6': F.J6M}[j]
    hy = S.LOFTS['_rings'][j]
    if S.V108:
        hy = hy - 1.5          # v10.8: the barrel stops inside the 1.5 mm gunmetal rings
    tube = M(cyl(c - [0, hy, 0], c + [0, hy, 0], Rp, 160)) - M(cyl(c - [0, hy + 1, 0], c + [0, hy + 1, 0], Rp - wall, 160))
    holes = []
    rows = np.arange(-hy + 4.0, hy - 4.0 + 1e-6, 2.3 * math.sqrt(3) / 2)
    n_ = int(2 * math.pi * Rp / 2.3)
    for i, yy in enumerate(rows):
        for k in range(n_):
            th = 2 * math.pi * (k + 0.5 * (i % 2) + 0.137) / n_     # off the tube's facet lines (no coincident edges)
            d_ = np.array([math.cos(th), 0.0, math.sin(th)])
            a0, a1 = c + d_ * (Rp - wall - 1.0) + [0, yy, 0], c + d_ * (Rp + 1.0) + [0, yy, 0]
            holes.append(Manifold.cylinder(float(np.linalg.norm(a1 - a0)), 0.65, 0.65, 8).transform(S._frame_to(a0, a1)))
    # spiders: A face (+y, the motor's output side) to its output pattern, B face (-y) to the back pattern
    spokes = []
    for face, sgn in (('out', 1), ('back', -1)):
        n, bc, dh = m[face]
        r_s = bc / 2 * 1e3
        yf = sgn * (hy - 1.5)
        for k in range(3):
            kk = round(k * n / 3)
            th = math.pi / n + 2 * math.pi * kk / n          # rose_fork.pattern's phase
            d_ = np.array([math.cos(th), 0.0, math.sin(th)])
            tdir = np.array([-d_[2], 0.0, d_[0]])
            p0 = c + d_ * (r_s - 5.0) + [0, yf, 0]
            p1 = c + d_ * (Rp - wall + 0.5) + [0, yf, 0]
            L = np.linalg.norm(p1 - p0)
            sp = M(trimesh.creation.box(extents=[L, 2.5, 9.0], transform=_basis(d_, tdir, (p0 + p1) / 2)))
            hole = M(cyl(c + d_ * r_s + [0, yf - 3, 0], c + d_ * r_s + [0, yf + 3, 0], dh / 2 * 1e3, 24))
            pad = M(cyl(c + d_ * r_s + [0, yf - 1.25, 0], c + d_ * r_s + [0, yf + 1.25, 0], dh / 2 * 1e3 + 3.0, 32))
            spokes.append((sp + pad) - hole)
    tube = tube + Manifold.batch_boolean(spokes, OpType.Add)
    if S.V108:
        return tube, 0         # v10.8: sealed, no dot-grid
    tube = tube - Manifold.batch_boolean(holes, OpType.Add)     # the grid last, so no hole edge lands on a spoke face
    return tube, len(holes)


def _basis(x, z_, origin):
    x = x / np.linalg.norm(x)
    y = np.array([0.0, 1.0, 0.0])
    z = np.cross(x, y)
    T = np.eye(4)
    T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = x, y, z, origin
    return T


def strip(name, xpos, width, depth=1.2, grow=-0.6):
    L = S.LOFTS[name]
    g = [(s, a + grow, b + grow, c) for s, a, b, c in L['st']]
    outer = M(S.sleeve(g, **L['kw']))
    inner = M(S.sleeve([(s, a - depth, b - depth, c) for s, a, b, c in g], **L['kw']))
    return (outer - inner) ^ box([xpos - width / 2, -500, -500], [xpos + width / 2, 500, 500])


def export(part, link, man, rows, wall, finish, process, fmt='stl'):
    o = np.array(F.ORIGIN[link]) * 1e3
    t = _tm(man).apply_translation(-o)
    t.merge_vertices()
    fn = '%s.%s' % (part, fmt)
    t.export(os.path.join(OUT, fn))
    rows.append(dict(file=fn, part=part, link=link, finish=finish, process=process, wall=wall,
                     volume_cm3=round(man.volume() / 1000.0, 2), watertight=bool(t.is_watertight),
                     bbox=[round(v) for v in t.extents], faces=len(t.faces)))


def build(tag='', only=None):
    os.makedirs(OUT, exist_ok=True)
    S.skin_meshes()
    misc = S.LOFTS['_misc']
    zs, X3 = misc['zs'], misc['X3']
    rows = []
    PA, PT, PU, GA = ('MJF PA12, dyed black #121213 (or white #ECECE9 on the white arm), vapour-smoothed satin', 'MJF PA12, dyed graphite',
                      'turned 6061, gunmetal anodize', 'SLA, Make green #39ff14')
    if S.V108:
        PT = 'MJF PA12, dyed black #121213 (joints are black on both colourways), vapour-smoothed satin'
    PD = 'turned 6061, black anodize satin, on a 0.25 mm gap pad to the motor (NOT printed)'
    want = lambda p: only is None or p in only
    # upper arm: one shell, J3 grid, split at J3 (1.5 mm), halves
    if want('upper'):
        sh, outer = shell('upper', 0.9)
        EPS_FOR[0] = S._EPS['upper']
        L = S.LOFTS['upper']
        outer_tm = _tm(outer)
        if not S.V108:   # v10.8: no J3 grid (the 6061 band under the sleeve cools J3)
            sh, nh = grid(sh, outer_tm, misc['x3a'], misc['x3b'], zs, d=1.4, pitch=2.2, skip=(misc['front'],), field=True, front=misc['front'])
        pieces = {'upper': sh ^ box([-2000, -2000, -2000], [X3 - 0.75, 2000, 2000]), 'upperd': sh ^ box([X3 + 0.75, -2000, -2000], [2000, 2000, 2000])}
        for nm, piece in pieces.items():
            xs = np.asarray(piece.to_mesh().vert_properties)[:, 0]
            for side_nm, side in (('front', -1), ('back', 1)):
                h = halves(piece)[side_nm]
                h, nc = clips(h, outer, nm, xs.min(), xs.max(), side)
                h, nmg = magnets(h, outer, xs.min(), xs.max(), side, zs)
                export('%s-%s%s' % (nm, side_nm, tag if nm == 'upper' else ''), nm, h, rows, 0.9, 'skin', PA)
        export('ring-J3', 'upperd', strip('upper', X3, 0.8), rows, 1.2, 'trim', PU)
    for nm, wall, gridspec in (('fore', 0.9, None), ('wrist', 0.9, 'J7'), ('turret', 1.2, None)):
        if not want(nm):
            continue
        sh, outer = shell(nm, wall)
        EPS_FOR[0] = S._EPS[nm]
        if gridspec and not S.V108:
            g0, g1 = S.LOFTS['wrist']['grid']
            sh, _ = grid(sh, _tm(outer), g0, g1, zs)
        V = np.asarray(sh.to_mesh().vert_properties)[:, :3]
        for side_nm, side in (('front', -1), ('back', 1)):
            h = halves(sh)[side_nm]
            if nm != 'turret':
                h, _ = clips(h, outer, 'yoke' if nm == 'fore' else nm, V[:, 0].min(), V[:, 0].max(), side)
                if nm == 'fore':
                    h, _ = clips(h, outer, 'fore', V[:, 0].min(), V[:, 0].max(), side)
            h, _ = magnets(h, outer, V[:, 0].min(), V[:, 0].max(), side, zs)
            export('%s-%s%s' % (nm, side_nm, tag if nm == 'turret' else ''), nm, h, rows, wall, 'skin', PA)
        if nm == 'wrist':
            export('ring-J7', 'wrist', strip('wrist', misc['xt'] - 3.0, 2.0), rows, 1.2, 'accent', GA)
    # rings + bezels
    if want('rings'):
        for j, owner, wall in (('J2', 'turret', 1.2), ('J4', 'fore', 0.9), ('J6', 'wrist', 0.9)):
            if S.V108 and j not in S.CAPS_ONLY:
                continue        # v10.8: the J2 drum is turned 6061 (BOM), not printed
            r, nh = ring(j, owner, wall)
            export('puck-%s' % j, owner, r, rows, wall, 'puck', PT, fmt='3mf')
            rows[-1]['holes'] = nh
        for nm, link, m, mat in S.skin_meshes():
            if nm.startswith('disc-'):
                export(nm, link, M(m).translate(tuple(np.array(F.ORIGIN[link]) * 1e3)), rows, 1.2, 'drum', PD)
            if nm.startswith('puck-') and S.V108 and nm[5:] not in S.CAPS_ONLY:   # the 6061 drum barrel: 1.0 mm wall + the spiders
                export(nm, link, ring(nm[5:], link, 1.0)[0], rows, 1.0, 'drum', PD)
            if nm.startswith('cap-'):
                export(nm, link, M(m).translate(tuple(np.array(F.ORIGIN[link]) * 1e3)), rows, 0.9, 'trim', PU)
    # J1 drum / desk puck / body face ring
    if want('base'):
        for nm, link, m, mat in S.skin_meshes():
            if nm == 'drum-J1':
                Rr = m.extents[0] / 2
                z0, z1 = m.bounds[0][2], m.bounds[1][2]
                tube = M(cyl([0, 0, z0], [0, 0, z1], Rr, 160)) - M(cyl([0, 0, z0 - 1], [0, 0, z1 + 1], Rr - 1.2, 160))
                export('drum-J1', 'base', tube.translate(tuple(np.array(F.ORIGIN['base']) * 1e3)), rows, 1.2, 'puck', PT)
            elif nm == 'desk-puck':
                o = M(m)
                Rr = m.extents[0] / 2
                sh = o - (M(trimesh.Trimesh(m.vertices * [(Rr - 1.2) / Rr, (Rr - 1.2) / Rr, 1.0], m.faces)).translate((0, 0, -1.2)))
                export('desk-puck', 'base', sh.translate(tuple(np.array(F.ORIGIN['base']) * 1e3)), rows, 1.2, 'skin', PA)
            elif nm == 'flank-J1':
                export('flank-J1' + tag, 'base', M(m).translate(tuple(np.array(F.ORIGIN['base']) * 1e3)), rows, 2.0, 'puck', PT)
    return rows


if __name__ == '__main__':
    # one arm-config per process (rose_fork reads its env at import): the caller runs desk-L, desk-R (upper only), body
    tag = sys.argv[1] if len(sys.argv) > 1 else ''
    only = sys.argv[2].split(',') if len(sys.argv) > 2 else None
    print(json.dumps(build(tag, only)))
