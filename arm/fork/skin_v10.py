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


def skin_parts():
    """v10.1 skin, [(name, link, mesh in the link frame (mm), material)].

    Vocabulary (quad00): every pitch joint is a graphite DRUM, coaxial, r = the joint lobe + 3.5 mm, as
    wide as the straddler - it swallows both links' lobes, so no plate edge can show at a joint. The
    shells run drum to drum, clipped at the joint axes (their ends live inside the drums). The upper
    arm is ONE loft over upper + upperd, split at the J3 plane, so the parting line is continuous and the
    two halves share their section there; a gunmetal band marks it. J1 is the same: base drum and
    pedestal share a round section (the desk mount reads as the humanoid's shoulder drum), a gunmetal
    band at the J1 output plane. The J7 accent ring is the one orange note.
    """
    o = lambda l: np.array(F.ORIGIN[l]) * 1e3
    Pw = lambda l, mats=None: link_points(l, mats) + o(l)
    zs = F.Z_SH * 1e3
    S = F.stations()
    X3, U, Wm, xt = F.X3_OUT * 1e3, F.UPPER * 1e3, F.W * 1e3, S['xt'] * 1e3
    zo = (F.T_FOOT + F.J1M['L']) * 1e3
    out = []
    loc = lambda m, l: m.apply_translation(-o(l))
    def xr(P, a, b):
        return P[(P[:, 0] >= a) & (P[:, 0] <= b)]
    # drums
    drums = (('J2', 0.0, F.R_LOBE_SH, ('turret', 'upper'), 'turret', F.J2M),
             ('J4', U, F.R_LOBE_EL, ('upperd', 'fore'), 'fore', F.J4M),
             ('J6', Wm, F.R_LOBE6W, ('yoke', 'wrist'), 'wrist', F.J6M))
    for nm, x, R, links, owner, m in drums:
        r = R * 1e3 + 3.5
        P = np.concatenate([Pw(l) for l in links])
        near = P[np.hypot(P[:, 0] - x, P[:, 2] - zs) < r + 1]
        hy = np.abs(near[:, 1]).max() + MARGIN + 1.0
        c = np.array([x, 0.0, zs])
        out.append(('drum-' + nm, owner, loc(cyl(c - [0, hy, 0], c + [0, hy, 0], r), owner), 'puck'))
        rc = m['d'] / 2 * 1e3 * 0.62                      # flush hub caps, gunmetal, both faces
        for sg in (-1, 1):
            out.append(('cap-%s%+d' % (nm, sg), owner, loc(cyl(c + [0, sg * hy, 0], c + [0, sg * (hy + 0.8), 0], rc), owner), 'trim'))
    cab = np.array(F.cable_points(F.fk({}))) * 1e3          # the harness rides inside the shells
    cab = np.concatenate([cab + [0, 0, d] for d in (-F.R_CABLE * 1e3, F.R_CABLE * 1e3)])
    # upper arm: one loft, split at J3
    P = xr(np.concatenate([Pw('upper'), Pw('upperd'), cab]), 0.0, U)
    arm_u = loft(P - [0, 0, 0], (1, 0, 0), n=2.6, slices=14, dome=(False, False))
    a, b = halves(arm_u, 0, X3)
    out += [('upper', 'upper', loc(a, 'upper'), 'skin'), ('upperd', 'upperd', loc(b, 'upperd'), 'skin')]
    out.append(('ring-J3', 'upperd', loc(band(arm_u, 0, X3 + 2.5), 'upperd'), 'trim'))
    # forearm (+ the bolted yoke), drum to drum
    P = xr(np.concatenate([Pw('fore'), Pw('yoke'), cab]), U, Wm)
    out.append(('fore', 'fore', loc(loft(P, (1, 0, 0), n=2.6, slices=12, dome=(False, False)), 'fore'), 'skin'))
    # wrist: drum to the tool flange, J7 inside
    P = xr(np.concatenate([Pw('wrist'), cab]), Wm, xt + F.T_PL * 1e3)
    out.append(('wrist', 'wrist', loc(loft(P, (1, 0, 0), n=2.6, slices=10, dome=(False, False)), 'wrist'), 'skin'))
    c = np.array([xt, 0, zs])
    out.append(('ring-J7', 'tool', loc(cyl(c, c + [4.0, 0, 0], F.J7M['d'] / 2 * 1e3 + 3.0), 'tool'), 'accent'))
    # turret: J1 output to the J2 axis (its top lives inside the J2 drum)
    P = Pw('turret')
    P = P[(P[:, 2] <= zs)]
    out.append(('turret', 'turret', loc(loft(P, (0, 0, 1), n=3.0, slices=8, dome=(False, False)), 'turret'), 'skin'))
    # J1: a round drum over the motor and collar; the desk pedestal below it on the same section
    Pb = Pw('base')
    drum = Pb[(Pb[:, 2] >= F.T_FOOT * 1e3 + 0.5) & (Pb[:, 2] <= zo)]
    rr = np.hypot(drum[:, 0], drum[:, 1]).max() + MARGIN
    out.append(('base', 'base', loc(cyl([0, 0, -0.5], [0, 0, zo], rr), 'base'), 'skin'))
    out.append(('ring-J1', 'turret', loc(cyl([0, 0, zo - 1.0], [0, 0, zo + 3.0], rr + 1.0), 'turret'), 'trim'))
    if F.PEDESTAL:
        ph = F.PED_H * 1e3
        rp = max(math.hypot(F.PED_S / 2, F.PED_S / 2), F.R_FOOT) * 1e3 + MARGIN   # the tube's corners and the foot, inside
        rings = []                                                          # a flared skirt: the shoulder
        for t, r in ((0.0, rp + 10), (0.25, rp + 6), (0.6, rp + 1), (1.0, rp)):  # drum's section meeting the desk
            rings.append(_ring(np.array([0, 0, -ph + t * ph]), np.array([1.0, 0, 0]), np.array([0, 1.0, 0]), r, r, 2.0, 96))
        H = Manifold.hull_points(np.concatenate(rings).astype(np.float32))
        out.append(('pedestal', 'base', loc(_tm(H), 'base'), 'skin'))
        out.append(('ring-ped', 'base', loc(cyl([0, 0, -1.5], [0, 0, 1.5], rp + 1.0), 'base'), 'trim'))
        out.append(('drum-ped', 'base', loc(cyl([0, 0, 1.5], [0, 0, F.T_FOOT * 1e3 + 1.0], rp), 'base'), 'puck'))
    else:
        # body: the chest socket. J1's drum runs in through the chest shell to the torso side plate;
        # a gunmetal bezel sits flush on the shell (render_v10 cuts the recess in the humanoid's cover)
        ph = F.PED_H * 1e3
        out.append(('socket', 'base', loc(cyl([0, 0, -ph], [0, 0, 0.0], rr), 'base'), 'puck'))
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
