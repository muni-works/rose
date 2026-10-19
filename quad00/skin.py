"""Industrial-design skin for the mini humanoid: lofted shells instead of hulls.

Each segment's shell is lofted through rounded-rectangle (superellipse) sections taken along the segment's
own axis, sized to the structure inside plus a margin, with domed ends; the torso gets a designed chest
flare. Every joint motor whose axis crosses a shell becomes a flush graphite 'puck'. The head gets a dark
visor, the hands are mittens, the feet sit on a dark sole. Shells are stand-ins for printed (MJF PA12) /
moulded covers on the plate frame.
"""
import math, re
import numpy as np
import trimesh
from manifold3d import Manifold

MARGIN = 6.0                     # shell standoff over the structure


def _ring(c, u, v, w, a, b, n=2.6, k=40):
    t = np.linspace(0, 2 * np.pi, k, endpoint=False)
    ct, st = np.cos(t), np.sin(t)
    x = a * np.sign(ct) * np.abs(ct) ** (2 / n)
    y = b * np.sign(st) * np.abs(st) ** (2 / n)
    return c[None, :] + x[:, None] * v[None, :] + y[:, None] * w[None, :]


def _frame(P, up=None):
    """segment axis u (principal direction, or `up`), v ~ lateral (body y), w = u x v."""
    c = P.mean(0)
    if up is None:
        C = np.cov((P - c).T)
        val, vec = np.linalg.eigh(C)
        u = vec[:, np.argmax(val)]
    else:
        u = np.asarray(up, float)
    u = u / np.linalg.norm(u)
    y = np.array([0, 1.0, 0])
    v = y - u * (y @ u)
    if np.linalg.norm(v) < 1e-6:
        v = np.array([0, 0, 1.0]) - u * u[2]
    v /= np.linalg.norm(v)
    w = np.cross(u, v)
    return c, u, v, w


def loft(P, up=None, n=2.6, slices=12, margin=MARGIN, dome=True, design=None):
    """a smooth shell over points P: sections along the axis, each the slice's extent + margin."""
    c, u, v, w = _frame(P, up)
    Q = P - c
    pu, pv, pw = Q @ u, Q @ v, Q @ w
    u0, u1 = pu.min(), pu.max()
    edges = np.linspace(u0, u1, slices + 1)
    rings = []
    for i in range(slices):
        lo, hi = edges[i], edges[i + 1]
        m = (pu >= lo - 1e-6) & (pu <= hi + 1e-6)
        if m.sum() < 3:
            continue
        vc, wc = (pv[m].min() + pv[m].max()) / 2, (pw[m].min() + pw[m].max()) / 2
        a, b = (pv[m].max() - pv[m].min()) / 2 + margin, (pw[m].max() - pw[m].min()) / 2 + margin
        if design is not None:
            a, b = design((lo + hi) / 2, u0, u1, a, b)
        for uu in (lo, hi):
            rings.append(_ring(c + uu * u + vc * v + wc * w, u, v, w, a, b, n))
    if dome:      # rounded ends: shrinking rings beyond each end
        for end, sgn in ((u0, -1), (u1, 1)):
            m = (pu <= u0 + (u1 - u0) / slices) if sgn < 0 else (pu >= u1 - (u1 - u0) / slices)
            vc, wc = (pv[m].min() + pv[m].max()) / 2, (pw[m].min() + pw[m].max()) / 2
            a, b = (pv[m].max() - pv[m].min()) / 2 + margin, (pw[m].max() - pw[m].min()) / 2 + margin
            dz = max(margin, 0.28 * min(a, b))                   # dome height scales with the section
            for f, d in ((0.96, 0.3), (0.86, 0.6), (0.66, 0.85), (0.38, 1.0)):
                rings.append(_ring(c + (end + sgn * dz * d) * u + vc * v + wc * w, u, v, w, a * f, b * f, n))
    H = Manifold.hull_points(np.concatenate(rings).astype(np.float32))
    g = H.to_mesh()
    return trimesh.Trimesh(np.asarray(g.vert_properties)[:, :3], np.asarray(g.tri_verts), process=True)


def _axis_of_cylinder(m):
    """a motor can's axis: of its three principal extents, the odd one out (the other two are its diameter;
    the can may carry its output ring, so 'shortest' is not reliable)."""
    P = m.vertices - m.vertices.mean(0)
    val, vec = np.linalg.eigh(np.cov(P.T))
    ext = [np.ptp(P @ vec[:, i]) for i in range(3)]
    i_ax = min(range(3), key=lambda i: abs(ext[(i + 1) % 3] - ext[(i + 2) % 3]))
    r = max(ext[(i_ax + 1) % 3], ext[(i_ax + 2) % 3]) / 2
    return m.vertices.mean(0), vec[:, i_ax], ext[i_ax], r


def puck(m, shells, extra=1.5):
    """a flush cap over a joint motor: as long as the shells it crosses are wide along its axis (+ extra)."""
    c, ax, L, r = _axis_of_cylinder(m)
    half = L / 2 + MARGIN
    for s in shells:
        V = s.vertices - c
        along = V @ ax
        radial = np.linalg.norm(V - np.outer(along, ax), axis=1)
        near = radial < r
        if near.any():
            half = max(half, np.abs(along[near]).max())
    h = 2 * half + 2 * extra
    return trimesh.creation.cylinder(radius=r * 0.98, segment=[c - ax * h / 2, c + ax * h / 2], sections=72)


SEGMENTS = [
    # name, regex on base names, axis ('up' = body x, None = principal), superellipse n, per-limb (tagged)
    ('hip', r'^(chassis-(pitchplate|rollmount|rollplate|yawmount)|rs06-l(roll|roll-out|pitch-out)|rs02-lyaw(-out)?)$', None, 2.4, 'l'),
    ('thigh', r'^(chassis-cap|thigh-(inner|outer)|rs06-knee)$', None, 2.8, 'l'),
    ('shin', r'^(shin|rs00-ankle(-b)?|crank-ankle(-b)?|link-ankle(-b)?)$', None, 2.6, 'l'),
    ('foot', r'^(foot|pivot-ankle)$', None, 3.2, 'l'),
    ('shoulder', r'^(chassis-(pitchplate|rollmount|rollplate|yawmount)|rs00-a(roll|roll-out|pitch-out|yaw|yaw-out))$', None, 2.2, 'a'),
    ('upperarm', r'^(chassis-cap|thigh-(inner|outer)|rs00-knee)$', None, 2.6, 'a'),
    ('forearm', r'^(shin)$', None, 2.4, 'a'),
]
PUCK_MOTORS = r'^rs0\d-(knee|lroll|aroll|ankle|ankle-b|neckpitch)$'


def chest_flare(um, u0, u1, a, b):
    t = (um - u0) / max(u1 - u0, 1)                            # 0 at the waist, 1 at the shoulders
    return a * (1.0 + 0.14 * t ** 0.8), b * (1.0 + 0.12 * math.sin(math.pi * min(1.0, t * 1.2)))   # wider + deeper chest


def skin(extra):
    """build the skin for the K1-style mini from its parts list (body frame: x up, y out, z fore)."""
    by_tag = {}
    torso, head, neck, flank = [], [], [], []
    for n, m in extra:
        base, _, tag = n.partition('.')
        if base.startswith('head-') or base in ('rs00-neckpitch', 'chassis-neck-plate', 'int-camera'):
            head.append(m.vertices); continue
        if base in ('rs00-neckyaw', 'chassis-neck'):
            neck.append(m.vertices); continue
        if re.match(r'^(chassis-(side|bar|floor|front|back)|int-battery)$', base) or re.match(r'^rs0\d-(lpitch|apitch)$', base):
            torso.append(m.vertices)
            if tag:
                flank.append((base, tag, m))
            continue
        by_tag.setdefault(tag, []).append((base, m))
    out = []
    # torso: designed chest flare over the structure
    T = np.concatenate(torso)
    lo, hi = T.min(0), T.max(0)                                 # the torso is a box: span its bounds densely
    g = [np.linspace(lo[i], hi[i], 9) for i in range(3)]
    T = np.array([[x, y, z] for x in g[0] for y in (lo[1], hi[1]) for z in (lo[2], hi[2])] +
                 [[x, y, z] for x in g[0] for y in g[1] for z in (lo[2], hi[2])])
    x0, x1 = T[:, 0].min(), T[:, 0].max()
    xw = x0 + 0.42 * (x1 - x0)                                  # waist line
    chest = T[T[:, 0] >= xw + 6]
    pelvis = T[T[:, 0] <= xw - 6]
    out.append(('cover-chest', loft(chest, up=[1, 0, 0], n=3.2, slices=8, margin=9.0, design=chest_flare)))
    out.append(('cover-pelvis', loft(pelvis, up=[1, 0, 0], n=2.8, slices=5, margin=7.0)))
    y0_, y1_, z0_, z1_ = T[:, 1].min(), T[:, 1].max(), T[:, 2].min(), T[:, 2].max()
    band = np.array([[x, y, z] for x in (xw - 10, xw + 10) for y in (y0_, y1_) for z in (z0_, z1_)])   # the plates are sparse: span them
    out.append(('puck-waist', loft(band, up=[1, 0, 0], n=3.0, slices=1, margin=4.0, dome=False)))
    # hip + shoulder pitch: a puck on the torso flank over each pitch motor's output
    ymax = T[:, 1].max()
    for base, tag, m in flank:
        cc, ax, L, r = _axis_of_cylinder(m)
        s = 1 if cc[1] > 0 else -1
        y0, y1 = s * (ymax - 4.0), s * (ymax + 8.0 + 2.0)
        out.append((f'puck-{base}.{tag}', trimesh.creation.cylinder(radius=r * 0.98, segment=[[cc[0], y0, cc[2]], [cc[0], y1, cc[2]]], sections=72)))
    H = np.concatenate(head)
    hc = H.mean(0)
    H = np.concatenate([H, hc + (H - hc) * np.array([1.0, 1.18, 1.0])])     # a fuller head: wider than the mechanism
    hs = loft(H, up=[1, 0, 0], n=2.05, slices=8, margin=12.0)
    out.append(('cover-head', hs))
    # visor: a rounded patch on the face, a hair proud, dark gloss
    c = hs.vertices.mean(0)
    ext = hs.extents
    visor_box = trimesh.creation.capsule(height=ext[1] * 0.42, radius=ext[0] * 0.17, count=[24, 24]) \
        .apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0])) \
        .apply_translation([c[0] + ext[0] * 0.06, c[1], c[2] + ext[2] * 0.5])
    V = Manifold(_mm(hs.copy().apply_scale(1.0))).__xor__(Manifold(_mm(visor_box)))
    vm = V.to_mesh()
    vis = trimesh.Trimesh(np.asarray(vm.vert_properties)[:, :3], np.asarray(vm.tri_verts))
    vis.apply_translation([0, 0, 1.2])
    out.append(('visor-face', vis))
    out.append(('puck-neck', loft(np.concatenate(neck), up=[1, 0, 0], n=2.0, slices=3, margin=1.0, dome=False)))
    for tag, items in by_tag.items():
        if tag not in ('ll', 'lr', 'al', 'ar'):
            continue
        limb = tag[0]
        for seg, pat, axis, n_, which in SEGMENTS:
            if which != limb:
                continue
            P = [m.vertices for base, m in items if re.match(pat, base)]
            if not P:
                continue
            sh = loft(np.concatenate(P), up=axis, n=n_, slices=10, margin=MARGIN if seg != 'foot' else 4.0)
            out.append((f'cover-{seg}.{tag}', sh))
            if seg == 'foot':      # dark rubber sole under the foot shell
                lo = sh.vertices[:, 0].min()
                sm = Manifold(_mm(sh)).trim_by_plane((-1.0, 0.0, 0.0), -(lo + 9.0)).to_mesh()   # keep x <= lo + 9
                sole = trimesh.Trimesh(np.asarray(sm.vert_properties)[:, :3], np.asarray(sm.tri_verts))
                out.append((f'sole.{tag}', sole.apply_translation([-0.6, 0, 0])))
        # hands: mittens -- a loft from the forearm end along the forearm, flattened
        paw = [m for base, m in items if base == 'paw']
        fore = [m.vertices for base, m in items if base == 'shin']
        if paw and fore:
            pc = paw[0].centroid
            F = np.concatenate(fore)
            c_, u_, v_, w_ = _frame(F)
            if (pc - c_) @ u_ < 0:
                u_ = -u_
            pts = [pc + u_ * t + v_ * s * 12 + w_ * r * 22 for t in np.linspace(-20, 55, 6) for s in (-1, 1) for r in (-1, 1)]
            out.append((f'cover-hand.{tag}', loft(np.array(pts), n=2.2, slices=5, margin=4.0)))
        # pucks: joint motors whose axis crosses the shells
        shells = [m for n_, m in out if n_.endswith(f'.{tag}') and n_.startswith('cover-')]
        for base, m in items:
            if re.match(PUCK_MOTORS, base):
                out.append((f'puck-{base}.{tag}', puck(m, shells)))
    return out


def _mm(t):
    from manifold3d import Mesh
    return Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32))
