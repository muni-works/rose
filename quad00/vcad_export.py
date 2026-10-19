"""export the mini (or tiny) humanoid to vcad as a loon assembly: one rigid body per link, revolute joints at the
K1 limits. Plates are native sketch + extrude (outlines taken from the clipped, router-cut plates), motors and drums
are native cylinders, the bones / blocks / helmet / hands native lofts of their section rings; only the shells carved
by swept clearance cuts (relief) come in as meshes (import-mesh) in each body's frame.

    python vcad_export.py [mini|tiny]   ->  vcad/<variant>/<variant>.loon + vcad/<variant>/stl/*.stl
"""
import os, sys
import numpy as np, trimesh
from manifold3d import Manifold, Mesh
import k1_structure as KS

REC = {}                                     # id(mesh) -> native description
KEEP = []                                    # recorded meshes stay alive, so ids are never reused


def _patch():
    import k1_design as KD
    m0 = KS.motor

    def motor(c, ax, mot):
        m = m0(c, ax, mot)
        REC[id(m)] = ('cyl', np.asarray(c, float), np.asarray(ax, float), KS.Q.MOTORS[mot]['D'] / 2, KS.Q.MOTORS[mot]['L'], 0.0); KEEP.append(m)
        return m
    KS.motor = motor
    c0 = KD.cyl

    def cyl(c, ax, r, h, sections=96):
        m = c0(c, ax, r, h, sections)
        REC[id(m)] = ('cyl', np.asarray(c, float), np.asarray(ax, float), r, h, 0.0); KEEP.append(m)
        return m
    KD.cyl = cyl
    h0 = KD.housing

    def housing(p, ax, mot, pad=KD.PAD, fil=KD.FIL, extra_len=6.0):
        body, caps, r, L = h0(p, ax, mot, pad, fil, extra_len)
        REC[id(body)] = ('cyl', np.asarray(p, float), np.asarray(ax, float), r, L, fil)
        a = np.asarray(ax, float) / np.linalg.norm(ax)
        REC[id(caps)] = ('caps', np.asarray(p, float), a, (r - fil) * 0.82, L, 1.6); KEEP.extend([body, caps])
        return body, caps, r, L
    KD.housing = housing
    return KD


MATS = {'cover': ('satin_black', 0.090, 0.094, 0.106, 0.1, 0.55), 'puck': ('municipal_orange', 1.0, 0.353, 0.039, 0.0, 0.45),
        'accent': ('municipal_orange', 1.0, 0.353, 0.039, 0.0, 0.45), 'trim': ('gunmetal', 0.30, 0.32, 0.34, 0.8, 0.35),
        'visor': ('visor_glass', 0.02, 0.02, 0.025, 0.2, 0.05), 'sole': ('sole_rubber', 0.17, 0.176, 0.19, 0.0, 0.9),
        'plate': ('aluminium', 0.75, 0.76, 0.78, 0.9, 0.3), 'motor': ('motor_dark', 0.08, 0.085, 0.09, 0.6, 0.4)}


def mat_of(n):
    k = n.split('-')[0].split('.')[0]
    if k in MATS:
        return k
    if n.startswith('rs'):
        return 'motor'
    return 'plate'


def f(v):
    return f'{v:.3f}'


def _basis(a):
    a = np.asarray(a, float) / np.linalg.norm(a)
    u = np.cross(a, [1.0, 0, 0] if abs(a[0]) < 0.9 else [0, 1.0, 0]); u /= np.linalg.norm(u)
    return a, u, np.cross(a, u)


def _circle(c, u, v, r, n=48):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return c[None, :] + r * (np.cos(t)[:, None] * u[None, :] + np.sin(t)[:, None] * v[None, :])


def cyl_src(c, ax, r, h, fil=0.0):
    """a cylinder (or a drum with rounded ends) from explicit axis vectors -- no Euler angles, no fillet op:
    plain = an extruded circle; filleted = a loft through rings tracing the end radii."""
    a, u, v = _basis(ax)
    c = np.asarray(c, float)
    if not fil:
        p0 = c - a * h / 2
        pts = [(r * np.cos(t), r * np.sin(t)) for t in np.linspace(0, 2 * np.pi, 48, endpoint=False)]
        segs = ' '.join(f'[line {f(p[0])} {f(p[1])} {f(q[0])} {f(q[1])}]' for p, q in zip(pts, pts[1:] + pts[:1]))
        e = a * h
        return (f'[extrude {f(e[0])} {f(e[1])} {f(e[2])} [sketch {f(p0[0])} {f(p0[1])} {f(p0[2])} '
                f'{f(u[0])} {f(u[1])} {f(u[2])} {f(v[0])} {f(v[1])} {f(v[2])} #[{segs}]]]')
    rings = []
    for end in (-1, 1):
        ts = np.linspace(0, np.pi / 2, 6)
        ts = ts if end < 0 else ts[::-1]
        for t in ts:                              # the end round: radius r - fil + fil*sin(t), axial h/2 - fil + fil*cos(t)
            rr = r - fil + fil * np.sin(t)
            z = end * (h / 2 - fil + fil * np.cos(t))
            rings.append(_circle(c + a * z, u, v, max(rr, 0.5)))
    return loft_src(rings, np.zeros(3))


def plate_src(m, origin):
    """a plate mesh -> [difference holes [extrude outline]] in the body frame."""
    lo, hi = m.bounds
    ax = int(np.argmin(hi - lo)); t = hi[ax] - lo[ax]
    M = Manifold(Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
    u, v = [i for i in range(3) if i != ax]
    if ax == 0:
        u, v = 1, 2
    if ax == 1:
        u, v = 0, 2
    if ax == 2:
        u, v = 0, 1
    perm = np.eye(3)[[u, v, ax]]
    sec = M.transform(np.hstack([perm, np.zeros((3, 1))])).slice((lo[ax] + hi[ax]) / 2).simplify(0.08)
    polys = [np.asarray(p) for p in sec.to_polygons() if len(p) >= 3]
    area = lambda p: 0.5 * np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])
    outer = [p for p in polys if area(p) > 0]
    holes = [p for p in polys if area(p) <= 0]
    o3 = np.zeros(3); o3[ax] = lo[ax] - origin[ax]
    du, dv = origin[u], origin[v]
    xa = np.zeros(3); xa[u] = 1; ya = np.zeros(3); ya[v] = 1; ext = np.zeros(3); ext[ax] = t

    def sk(p, proud=0.0):
        p = p - np.array([du, dv])
        o = o3.copy(); o[ax] -= proud; e = ext.copy(); e[ax] += 2 * proud
        segs = ' '.join(f'[line {f(p[i][0])} {f(p[i][1])} {f(p[(i + 1) % len(p)][0])} {f(p[(i + 1) % len(p)][1])}]' for i in range(len(p)))
        return (f'[extrude {f(e[0])} {f(e[1])} {f(e[2])} [sketch {f(o[0])} {f(o[1])} {f(o[2])} '
                f'{f(xa[0])} {f(xa[1])} {f(xa[2])} {f(ya[0])} {f(ya[1])} {f(ya[2])} #[{segs}]]]')
    if not outer:
        return None
    s = sk(outer[0]) if len(outer) == 1 else '[union ' + ' '.join(sk(p) for p in outer[:2]) + ']'
    for h in holes:                          # holes: extrude a hair longer so the cut is clean
        hs = sk(h, proud=1.0)
        s = f'[difference {hs} {s}]'
    return s


def loft_src(rings, o):
    """section rings (3D point loops) -> [loft #[sketch ...]]: each ring sketched in its own plane, points kept."""
    sks = []
    for r in rings:
        r = np.asarray(r, float) - o
        c = r.mean(0)
        _, _, vt = np.linalg.svd(r - c)
        u, n = vt[0], vt[2]
        v = np.cross(n, u)
        p = np.c_[(r - c) @ u, (r - c) @ v]
        segs = ' '.join(f'[line {f(p[i][0])} {f(p[i][1])} {f(p[(i + 1) % len(p)][0])} {f(p[(i + 1) % len(p)][1])}]' for i in range(len(p)))
        sks.append(f'[sketch {f(c[0])} {f(c[1])} {f(c[2])} {f(u[0])} {f(u[1])} {f(u[2])} {f(v[0])} {f(v[1])} {f(v[2])} #[{segs}]]')
    return '[loft #[' + ' '.join(sks) + ']]'


def cuts_src(src, cuts, o):
    for kind, *a in cuts:
        if kind == 'ycyl':
            c, r = np.asarray(a[0], float) - o, a[1]
            tool = f'[translate {f(c[0])} {f(-r)} {f(c[2])} [rotate -90 0 0 [cylinder-n {f(r)} {f(2 * r)} 72]]]'
        else:
            p, sz = np.asarray(a[0], float) - o, a[1]
            tool = f'[translate {f(p[0])} {f(p[1])} {f(p[2])} [cube {f(sz[0])} {f(sz[1])} {f(sz[2])}]]'
        src = f'[difference {tool} {src}]'
    return src


def shell_src(m, o):
    """native source for a ring-built shell (loft, optional cuts) or the visor; None -> import the mesh."""
    md = m.metadata
    if 'visor' in md:
        V = md['visor']; hc = np.asarray(V['hc'], float) - o; oc = np.asarray(V['oval_c'], float) - o; sx, sy, sz = V['oval_s']
        helmet = cuts_src(loft_src(V['helmet']['loft'], o), V['helmet'].get('cut', []), o)
        big = f'[translate {f(hc[0])} {f(hc[1])} {f(hc[2])} [scale {V["s"]} {V["s"]} {V["s"]} [translate {f(-hc[0])} {f(-hc[1])} {f(-hc[2])} {helmet}]]]'
        oval = f'[translate {f(oc[0])} {f(oc[1])} {f(oc[2])} [scale {f(sx)} {f(sy)} {f(sz)} [rotate 0 90 0 [translate 0 0 -0.5 [cylinder-n 1 1 96]]]]]'
        return f'[intersection {oval} {big}]'
    if 'loft' in md:
        return cuts_src(loft_src(md['loft'], o), md.get('cut', []), o)
    return None


def pivots(J):
    P = {'torso': np.zeros(3)}
    H = KS.head_layout(J)
    P['hyaw'], P['hpitch'] = H['yaw'][0], H['pitch'][0]
    for side, t in (('Left', 'l'), ('Right', 'r')):
        Lg, U = KS.layout(J, side), KS.upper_layout(J, side)
        for k in ('roll', 'pitch', 'yaw', 'knee'):
            P[f'{k}.{t}'] = Lg[k][0]
        P[f'anklep.{t}'] = P[f'ankle.{t}'] = Lg['ankle']
        for k in ('sp', 'sr', 'ep', 'ey'):
            P[f'{k}.{t}'] = U[k][0]
    return P


def joints(J):
    """(name, parent, child, axis, lo, hi) -- axes signed as the K1's, limits from its URDF."""
    out = []
    lim = lambda n: tuple(np.degrees(J[n]['lim']))
    sgn = lambda ours, n: np.asarray(ours, float) * (np.sign(np.dot(ours, J[n]['axis'])) or 1.0)
    for side, t in (('Left', 'l'), ('Right', 'r')):
        s = 1 if t == 'l' else -1
        for jn, par, ch, a in (('Hip_Roll', 'torso', f'roll.{t}', [1, 0, 0]), ('Hip_Pitch', f'roll.{t}', f'pitch.{t}', [0, 1, 0]),
                               ('Hip_Yaw', f'pitch.{t}', f'yaw.{t}', [0, 0, 1]), ('Knee_Pitch', f'yaw.{t}', f'knee.{t}', [0, 1, 0]),
                               ('Ankle_Pitch', f'knee.{t}', f'anklep.{t}', [0, 1, 0]), ('Ankle_Roll', f'anklep.{t}', f'ankle.{t}', [1, 0, 0])):
            n = f'{side}_{jn}'
            out.append((f'{t}_{jn.lower()}', par, ch, sgn(a, n), *lim(n)))
        for jn, par, ch, a, n in (('shoulder_pitch', 'torso', f'sp.{t}', [0, 1, 0], f'A{side}_Shoulder_Pitch'),
                                  ('shoulder_roll', f'sp.{t}', f'sr.{t}', [1, 0, 0], f'{side}_Shoulder_Roll'),
                                  ('upperarm_rot', f'sr.{t}', f'ep.{t}', [0, 0, -1], f'{side}_Elbow_Pitch'),
                                  ('elbow', f'ep.{t}', f'ey.{t}', [0, s, 0], f'{side}_Elbow_Yaw')):
            out.append((f'{t}_{jn}', par, ch, sgn(a, n), *lim(n)))
    out.append(('head_yaw', 'torso', 'hyaw', sgn([0, 0, 1], 'AAHead_yaw'), *lim('AAHead_yaw')))
    out.append(('head_pitch', 'hyaw', 'hpitch', sgn([0, 1, 0], 'Head_pitch'), *lim('Head_pitch')))
    return out


def body_of(n):
    import rom_mini as R
    b = R.body_of(n)
    return b


def main(variant='mini'):
    KS.use(variant)
    KD = _patch()
    import k1 as K
    J, M = KS.world()
    D = KD.design()
    C = KD.structure(D)
    P = pivots(J)
    outdir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vcad', variant)
    os.makedirs(os.path.join(outdir, 'stl'), exist_ok=True)
    parts, inst, fixed = [], [], []
    anchor = {}
    used_mats = set()
    n_native = n_mesh = 0
    for n, m in D + C:
        b = body_of(n)
        if b is None:
            print('  (skipped, no body)', n)
            continue
        o = P[b]
        rec = REC.get(id(m))
        if rec and rec[0] == 'cyl':
            _, c, ax, r, h, fil = rec
            src = cyl_src(c - o, ax, r, h, fil)
        elif rec and rec[0] == 'caps':
            _, c, a, r, L, th = rec
            src = '[union ' + ' '.join(cyl_src(c - o + a * s * (L / 2 + 0.3), a, r, th) for s in (-1, 1)) + ']'
        elif mat_of(n) == 'plate':
            src = plate_src(m, o)
        else:
            src = shell_src(m, o)
        if src is None:                       # lofted / hull / relief-cut shell: mesh in the body frame
            mm = K.simplify(m, grid=0.8) if len(m.faces) > 20000 else m
            fn = n.replace('.', '_') + '.stl'
            mm.copy().apply_translation(-o).export(os.path.join(outdir, 'stl', fn))
            src = f'[import-mesh-scaled 0.001 0.001 0.001 "stl/{fn}"]'    # import-mesh reads STL as metres
            n_mesh += 1
        else:
            n_native += 1
        mk = mat_of(n); used_mats.add(mk)
        pn = n.replace('.', '_')
        parts.append(f'[part "{pn}" {src} "{MATS[mk][0]}"]')
        inst.append(f'[instance "{pn}_i" "{pn}" 0.0 0.0 0.0]')
        if b in anchor:
            fixed.append(f'[fixed-joint "fix_{pn}" "{anchor[b]}" 0.0 0.0 0.0 "{pn}_i" 0.0 0.0 0.0]')
        else:
            anchor[b] = f'{pn}_i'
    for t in ('l', 'r'):                      # the ankle U-joint cross: a body of its own between pitch and roll
        pn = f'ankle_cross_{t}'
        parts.append(f'[part "{pn}" [translate -5 -5 -5 [cube 10 10 10]] "aluminium"]')
        inst.append(f'[instance "{pn}_i" "{pn}" 0.0 0.0 0.0]')
        anchor[f'anklep.{t}'] = f'{pn}_i'
        used_mats.add('plate')
    jts = []
    for jn, par, ch, a, lo, hi in joints(J):
        pa = P[ch] - P[par]
        jts.append(f'[revolute-joint "{jn}" {f(a[0])} {f(a[1])} {f(a[2])} {f(lo)} {f(hi)} "{anchor[par]}" {f(pa[0])} {f(pa[1])} {f(pa[2])} "{anchor[ch]}" 0.0 0.0 0.0]')
    mats = {MATS[k][0]: MATS[k] for k in used_mats}
    head = [f';; {variant} humanoid -- generated by quad00/vcad_export.py from k1_design + k1_structure; do not hand-edit.',
            f';; {len(parts)} parts ({n_native} native loon, {n_mesh} imported shell meshes), {len(jts)} revolute joints (K1 limits).',
            ';; frame: x fore, y left, z up, mm; each body is authored about its joint pivot.']
    head += [f'[material "{v[0]}" {v[1]} {v[2]} {v[3]} {v[4]} {v[5]}]' for v in mats.values()]
    src = '\n'.join(head) + '\n[assembly\n #[' + '\n   '.join(parts) + ']\n #[' + '\n   '.join(inst) + ']\n #[' + \
        '\n   '.join(fixed + jts) + ']\n "' + anchor['torso'] + '"]\n'
    path = os.path.join(outdir, f'{variant}.loon')
    open(path, 'w').write(src)
    KS.use('mini')
    print(f'{path}: {len(parts)} parts ({n_native} native, {n_mesh} meshes), {len(jts)} joints, {len(src) / 1e3:.0f} kB')


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'mini')
