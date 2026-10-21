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
    '[material "skin" 0.090 0.094 0.106 0.1 0.55]',        # satin black (quad00 cover)
    '[material "puck" 0.08 0.085 0.09 0.6 0.4]',           # graphite (quad00 motor_dark)
    '[material "trim" 0.30 0.32 0.34 0.8 0.35]',           # gunmetal (quad00 trim)
    '[material "accent" 1.0 0.353 0.039 0.0 0.45]',        # municipal orange (quad00 accent)
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


def segments():
    """[(name, link, mm points in that link's frame, axis, n, dome)] for v10."""
    S = F.stations()
    o = lambda l: np.array(F.ORIGIN[l]) * 1e3
    def clip(P, link, x0, x1, ax=0):
        g = P[:, ax] + o(link)[ax]
        return P[(g >= x0 * 1e3) & (g <= x1 * 1e3)]
    out = []
    base = link_points('base')
    g = base + o('base')
    if F.PEDESTAL:
        ped = base[g[:, 2] < -0.5]
        out.append(('pedestal', 'base', ped, (0, 0, 1), 3.6, (False, False)))
    drum = base[g[:, 2] > F.T_FOOT * 1e3 + 0.5]
    out.append(('base', 'base', drum, (0, 0, 1), 2.0, (False, False)))
    out.append(('turret', 'turret', link_points('turret'), (0, 0, 1), 3.0, (False, True)))
    up = link_points('upper')
    out.append(('upper', 'upper', clip(up, 'upper', 0.045, F.X3_OUT), (1, 0, 0), 2.6, (True, False)))
    ud = link_points('upperd')
    out.append(('upperd', 'upperd', clip(ud, 'upperd', F.X3_OUT, F.UPPER), (1, 0, 0), 2.6, (False, True)))
    fo = np.concatenate([link_points('fore'), link_points('yoke') + o('yoke') - o('fore')])
    out.append(('fore', 'fore', clip(fo, 'fore', F.UPPER, F.W), (1, 0, 0), 2.6, (True, True)))
    wr = link_points('wrist')
    out.append(('wrist', 'wrist', clip(wr, 'wrist', F.W, S['xt'] + F.T_PL), (1, 0, 0), 2.6, (True, False)))
    return out


def cyl(a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    return trimesh.creation.cylinder(radius=r, segment=[a, b], sections=72)


def trim_parts(shells):
    """Pucks on the pitch axes, rings at the roll planes. [(name, link, mesh in link frame mm, mat)]"""
    o = lambda l: np.array(F.ORIGIN[l]) * 1e3
    z = F.Z_SH * 1e3
    def half_y(names, link, x):
        """the widest |y| of these shells near x (mm, link frame) - where the puck must stand flush."""
        best = 0.0
        for nm in names:
            m, ml = shells[nm]
            V = m.vertices + o(ml) - o(link)
            near = np.abs(V[:, 0] + o(link)[0] - x) < 30
            if near.any():
                best = max(best, np.abs(V[near, 1]).max())
        return best
    def r_at(nm, link, x, ax=0):
        m, ml = shells[nm]
        V = m.vertices + o(ml)
        near = np.abs(V[:, ax] - x) < 15
        c = np.array([x, 0, z]) if ax == 0 else np.array([0, 0, x])
        d = V[near] - c
        d[:, ax] = 0
        return np.linalg.norm(d, axis=1).max()
    out = []
    for nm, link, x, m, parts in (('J2', 'turret', 0.0, F.J2M, ('turret', 'upper')),
                                   ('J4', 'fore', F.UPPER, F.J4M, ('upperd', 'fore')),
                                   ('J6', 'wrist', F.W, F.J6M, ('fore', 'wrist'))):
        xm = x * 1e3
        hy = half_y(parts, link, xm) + 0.6
        c = np.array([xm, 0, z]) - o(link)
        out.append(('puck-' + nm, link, cyl(c - [0, hy, 0], c + [0, hy, 0], m['d'] / 2 * 1e3 * 0.78), 'puck'))
    # roll planes: J3 (upper | upperd), J7 (wrist | tool, the accent), J1 (base | turret)
    x3 = F.X3_OUT * 1e3
    out.append(('ring-J3', 'upperd', band(shells['upperd'][0], 0, x3 - o('upperd')[0] + 3.0), 'trim'))
    xt = F.stations()['xt'] * 1e3
    c = np.array([xt, 0, z]) - o('tool')
    out.append(('ring-J7', 'tool', cyl(c - [0.0, 0, 0], c + [4.0, 0, 0], F.J7M['d'] / 2 * 1e3 + 3.0), 'accent'))
    zo = (F.T_FOOT + F.J1M['L']) * 1e3
    b = band(shells['base'][0], 2, zo - o('base')[2] - 3.0)
    out.append(('ring-J1', 'turret', b.apply_translation(o('base') - o('turret')), 'trim'))
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
        shells = {}
        for name, link, P, ax, n, dome in segments():
            shells[name] = (loft(P, ax, n=n, slices=10, dome=dome), link)
        parts = [(nm, link, m, 'skin') for nm, (m, link) in shells.items()] + trim_parts(shells)
        _CACHE[key] = parts
    return _CACHE[key]


def write_stls(outdir, tag):
    os.makedirs(outdir, exist_ok=True)
    files = []
    for nm, link, m, mat in skin_meshes():
        fn = '%s-%s.stl' % (tag, nm)
        m.export(os.path.join(outdir, fn))
        files.append((nm, link, fn, mat))
    return files


def frag(q, at=(0.0, 0.0, 0.0), R0=Gm.I3, prefix='a', outdir='.', skin=True, skip=(), stl_tag=None):
    """The posed arm and (skin=True) its shells as loon, no header. STLs land in outdir."""
    src = F.world_loon(q, at=at, prefix=prefix, header=False, R0=R0, skip=skip)
    if not skin:
        return src
    tag = stl_tag or ('skin-%s' % F.SIZE)
    files = write_stls(outdir, tag) if not os.path.exists(os.path.join(outdir, '%s-upper.stl' % tag)) or tag not in _WRITTEN else _WRITTEN[tag]
    _WRITTEN[tag] = files
    T0 = F.fk(q)
    T = {k: (Gm.mm_(R0, Rk), Gm.add(Gm.mv(R0, tk), at)) for k, (Rk, tk) in T0.items()}
    L, roots = [], []
    for i, (nm, link, fn, mat) in enumerate(files):
        if link in skip or (link, 'skin') in skip or nm in skip:
            continue
        R, t = T[link]
        L.append('[let %ss%d %s]' % (prefix, i, F.place_loon(R, t, '[import-mesh-scaled 0.001 0.001 0.001 "%s"]' % fn)))
        roots.append('[root %ss%d "%s"]' % (prefix, i, mat))
    return src + '\n'.join(L + [''] + roots) + '\n'


_WRITTEN = {}

if __name__ == '__main__':
    d = sys.argv[1] if len(sys.argv) > 1 else '.'
    for nm, link, m, mat in skin_meshes():
        print('%-10s %-7s %6d faces  %s  extents %s' % (nm, link, len(m.faces), mat, np.round(m.extents, 1)))
