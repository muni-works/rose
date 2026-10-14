"""Pose-aware geometry for the rose arm: every solid as a sketch + extrude with an explicit frame,
grouped by link, posed by forward kinematics. One model feeds the posed renders and the cable
checks, so what is checked is what is drawn.

A primitive is (link, material, origin, u, v, outline, vec): `outline` is a closed 2D path in the
(u, v) sketch plane as ('line', p, q) / ('arc', p, q, c, ccw) items, in metres; `vec` is the extrude.
Coordinates are the ZERO-POSE world frame; FK maps a link's zero-pose world points to the pose.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rose_arm_layout as A
import rose_arm_export as X
import rose_arm_loon as LG

# ------------------------------------------------------------------ small linear algebra
def add(a, b): return (a[0] + b[0], a[1] + b[1], a[2] + b[2])
def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def mul(a, k): return (a[0] * k, a[1] * k, a[2] * k)
def dot(a, b): return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
def cross(a, b): return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])
def norm(a): return math.sqrt(dot(a, a))
def unit(a):
    n = norm(a)
    return mul(a, 1.0 / n) if n > 1e-12 else a


def rot(axis, q):
    """Rotation matrix about a unit axis."""
    x, y, z = unit(axis)
    c, s, C = math.cos(q), math.sin(q), 1 - math.cos(q)
    return ((c + x * x * C, x * y * C - z * s, x * z * C + y * s),
            (y * x * C + z * s, c + y * y * C, y * z * C - x * s),
            (z * x * C - y * s, z * y * C + x * s, c + z * z * C))


def mv(M, v): return tuple(M[i][0] * v[0] + M[i][1] * v[1] + M[i][2] * v[2] for i in range(3))
def mm_(A_, B_): return tuple(tuple(sum(A_[i][k] * B_[k][j] for k in range(3)) for j in range(3)) for i in range(3))
I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

# ------------------------------------------------------------------ kinematics
LINKS = [(l[0], l[1], l[2]) for l in X.LINKS]          # name, joint, parent
JOINT_NAMES = [l[1] for l in X.LINKS if l[1]]
AXIS = {j: X.JOINTS[j][2] for j in JOINT_NAMES}
ORIGIN = {name: X.joint_origin(joint) for name, joint, _ in LINKS}


def fk(q):
    """q: {joint: rad}. Returns {link: (R, t)} mapping a link's zero-pose world point p to R (p - o) + t."""
    T = {}
    for name, joint, parent in LINKS:
        o = ORIGIN[name]
        if parent is None:
            T[name] = (I3, o)
            continue
        Rp, tp = T[parent]
        op = ORIGIN[parent]
        t = add(mv(Rp, sub(o, op)), tp)
        R = mm_(Rp, rot(AXIS[joint], q.get(joint, 0.0)))
        T[name] = (R, t)
    return T


def xform(T, link, p):
    R, t = T[link]
    return add(mv(R, sub(p, ORIGIN[link])), t)


def xdir(T, link, d):
    return mv(T[link][0], d)


# which link carries each body
ACT_LINK = {}
for name, joint, parent, boxes, acts in X.LINKS:
    for a in acts:
        ACT_LINK[a] = name
BOX_LINK = {}
for name, joint, parent, boxes, acts in X.LINKS:
    for b in boxes:
        BOX_LINK[b] = name

# ------------------------------------------------------------------ primitive builders
def rect_outline(u0, v0, u1, v1):
    P = [(u0, v0), (u1, v0), (u1, v1), (u0, v1)]
    return [('line', P[i], P[(i + 1) % 4]) for i in range(4)]


def strut_outline(p, q, w):
    """A bar of width w from p to q in the sketch plane."""
    d = unit((q[0] - p[0], q[1] - p[1], 0.0))
    n = (-d[1] * w / 2, d[0] * w / 2)
    P = [(p[0] + n[0], p[1] + n[1]), (q[0] + n[0], q[1] + n[1]), (q[0] - n[0], q[1] - n[1]), (p[0] - n[0], p[1] - n[1])]
    return [('line', P[i], P[(i + 1) % 4]) for i in range(4)]


def circle_outline(r):
    return [('arc', (r, 0.0), (-r, 0.0), (0.0, 0.0), True), ('arc', (-r, 0.0), (r, 0.0), (0.0, 0.0), True)]


def perp_basis(d):
    d = unit(d)
    h = (1.0, 0.0, 0.0) if abs(d[0]) < 0.9 else (0.0, 0.0, 1.0)
    u = unit(cross(d, h))
    v = cross(d, u)
    return u, v


def cylinder(link, mat, a, b, r):
    u, v = perp_basis(sub(b, a))
    return (link, mat, a, u, v, circle_outline(r), sub(b, a))


def actuators():
    out = []
    for j in A.joints(True):
        label, a, b, r, model = A.seg(j)
        out.append(cylinder(ACT_LINK[label], 'actuator', a, b, r))
    return out


def profile_outline_2d(prof):
    """The link profile outline from rose_arm_loon, as 2D items in (x, z-about-axis), metres."""
    items = []
    txt = LG.profile_solid(prof)
    import re
    for m in re.finditer(r'\[(line|arc) ([^\]]*)\]', txt):
        nums = m.group(2).split()
        f = [float(n) / 1e3 for n in nums[:6]]
        if m.group(1) == 'line':
            items.append(('line', (f[0], f[1]), (f[2], f[3])))
        else:
            items.append(('arc', (f[0], f[1]), (f[2], f[3]), (f[4], f[5]), nums[6] == 'true'))
    return items


def knuckle_outline(e, x_far, r_cap):
    """A side plate round a pitch lobe: half-disc on the outside, straight back to x_far."""
    _, xc, r = e
    s = 1 if x_far < xc else -1          # +1: link runs to -x of the lobe (lobe at the right end)
    top, bot = (xc, r), (xc, -r)
    ft, fb = (x_far, r), (x_far, -r)
    return [('line', ft, top), ('arc', top, bot, (xc, 0.0), s < 0),
            ('line', bot, fb), ('line', fb, ft)] if s > 0 else \
           [('line', fb, bot), ('arc', bot, top, (xc, 0.0), False), ('line', top, ft), ('line', ft, fb)]


def bone(prof, link):
    """Knuckle plate pairs at lobes, end frames at drum ends, X-braced lattice cage between."""
    B = A.BONE
    label, (x0, x1), (y0, y1), ends, waist = prof
    z0 = A.BASE_H
    W = y1 - y0
    out = []
    H = 2 * max((e[2] if e[0] == 'lobe' else e[1]) for e in ends)
    ca, cb = A.cage_span(prof)
    short = cb - ca < 0.03
    for i, e in enumerate(ends):
        if e[0] == 'lobe':
            xc, r = e[1], e[2]
            if short:
                x_far = x1 if i == 0 else x0
            else:
                x_far = (xc + r + B['knuckle_len']) if i == 0 else (xc - r - B['knuckle_len'])
            ol = knuckle_outline(e, x_far, r)
            for yp, sgn in ((y1, -1), (y0 + B['knuckle_t'], -1)):
                out.append((link, 'sheet', (0.0, yp, z0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), ol, (0.0, -B['knuckle_t'], 0.0)))
        else:
            xe = x0 if i == 0 else x1 - A.T_PLATE
            h = e[1]
            # end frame: a square ring round the drum boss (four bars), plate in the y-z plane
            fw = 0.012
            for (uu0, vv0, uu1, vv1) in ((y0, -h, y1, -h + fw), (y0, h - fw, y1, h), (y0, -h, y0 + fw, h), (y1 - fw, -h, y1, h)):
                out.append((link, 'sheet', (xe, 0.0, z0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
                            rect_outline(uu0, vv0, uu1, vv1), (A.T_PLATE, 0.0, 0.0)))
    if short:
        # short segment: two plain side plates join knuckle to end frame
        for e in ends:
            if e[0] == 'flat':
                h = e[1]
                for yp in (y1, y0 + B['knuckle_t']):
                    xa, xb = (x0, x1)
                    out.append((link, 'sheet', (0.0, yp, z0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                                rect_outline(xa, -h, xb, h), (0.0, -B['knuckle_t'], 0.0)))
        return out
    if B['kind'] == '2020':
        # four 2020 bars inside the knuckle plates, square ring bulkheads every B['s']
        kt = B['knuckle_t']
        ys = (y0 + kt + 0.010, y1 - kt - 0.010)
        zs = (z0 - H / 2 + 0.010, z0 + H / 2 - 0.010)
        xa = ca - B['knuckle_len'] if ends[0][0] == 'lobe' else ca
        xb = cb + B['knuckle_len'] if ends[1][0] == 'lobe' else cb
        for yy in ys:
            for zz in zs:
                out.append((link, 'bar', (xa, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
                            rect_outline(yy - 0.010, zz - 0.010, yy + 0.010, zz + 0.010), (xb - xa, 0.0, 0.0)))
        n = max(2, round((cb - ca) / B['s']) + 1)
        fw = 0.024
        for k in range(n):
            xk = ca + (cb - ca) * k / (n - 1) - A.T_PLATE / 2
            yl, yh, zl, zh = ys[0] - 0.012, ys[1] + 0.012, zs[0] - 0.012, zs[1] + 0.012
            for (uu0, vv0, uu1, vv1) in ((yl, zl, yh, zl + fw), (yl, zh - fw, yh, zh), (yl, zl, yl + fw, zh), (yh - fw, zl, yh, zh)):
                out.append((link, 'sheet', (xk, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0),
                            rect_outline(uu0, vv0, uu1, vv1), (A.T_PLATE, 0.0, 0.0)))
        return out
    # lattice cage between ca and cb: four faces, rails + X bracing per bay
    tf, w = B['tf'], B['w']
    L = cb - ca
    hz = H / 2
    faces = [  # origin, u, v, face width (v extent), extrude
        ((0.0, 0.0, z0 + hz), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (y0, y1), (0.0, 0.0, -tf)),
        ((0.0, 0.0, z0 - hz + tf), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (y0, y1), (0.0, 0.0, -tf)),
        ((0.0, y1, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (z0 - hz, z0 + hz), (0.0, -tf, 0.0)),
        ((0.0, y0 + tf, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), (z0 - hz, z0 + hz), (0.0, -tf, 0.0)),
    ]
    for org, u, v, (va, vb), ext in faces:
        bw = vb - va
        nb = max(1, round(L / bw))
        s = L / nb
        segs = [((ca, va + w / 2), (cb, va + w / 2)), ((ca, vb - w / 2), (cb, vb - w / 2))]
        for k in range(nb):
            xa, xb = ca + k * s, ca + (k + 1) * s
            segs += [((xa, va + w / 2), (xb, vb - w / 2)), ((xa, vb - w / 2), (xb, va + w / 2))]
        for p, q in segs:
            out.append((link, 'sheet', org, u, v, strut_outline(p, q, w), ext))
    return out


def base_parts():
    out = []
    for b in ('base_plate', 'col_floor', 'column', 'grip_plate'):
        _, p0, p1, link = A.boxes(True)[[x[0] for x in A.boxes(True)].index(b)]
        r = {'base_plate': 0.030, 'col_floor': 0.012, 'column': 0.018, 'grip_plate': 0.006}[b]
        x0, y0, x1, y1 = p0[0], p0[1], p1[0], p1[1]
        ol = [('line', (x0 + r, y0), (x1 - r, y0)), ('arc', (x1 - r, y0), (x1, y0 + r), (x1 - r, y0 + r), True),
              ('line', (x1, y0 + r), (x1, y1 - r)), ('arc', (x1, y1 - r), (x1 - r, y1), (x1 - r, y1 - r), True),
              ('line', (x1 - r, y1), (x0 + r, y1)), ('arc', (x0 + r, y1), (x0, y1 - r), (x0 + r, y1 - r), True),
              ('line', (x0, y1 - r), (x0, y0 + r)), ('arc', (x0, y0 + r), (x0 + r, y0), (x0 + r, y0 + r), True)]
        out.append((BOX_LINK[b], 'base' if b != 'grip_plate' else 'sheet', (0.0, 0.0, p0[2]), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                    ol, (0.0, 0.0, p1[2] - p0[2])))
    # gripper fingers
    gx, gy, z0 = A.GRIP_X, A.HA_YC, A.BASE_H
    for dx in (-0.011, 0.011):
        out.append(('tool', 'sheet', (0.0, 0.0, z0 - A.R('EduLite') - 0.06), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
                    rect_outline(gx + dx - 0.003, gy - 0.004, gx + dx + 0.003, gy + 0.004), (0.0, 0.0, 0.054)))
    return out


def primitives():
    prims = actuators() + base_parts()
    link = {'ua_A': 'upper_a', 'ua_B': 'upper_b', 'fa_C': 'fore_c', 'fa_D': 'fore_d', 'hand': 'hand'}
    for prof in A.profiles(True):
        prims += bone(prof, link[prof[0]])
    return prims


# ------------------------------------------------------------------ posing + emission
def pose_prim(T, prim):
    link, mat, o, u, v, ol, vec = prim
    return (link, mat, xform(T, link, o), xdir(T, link, u), xdir(T, link, v), ol, xdir(T, link, vec))


def prim_loon(prim):
    _, _, o, u, v, ol, vec = prim
    f = lambda p: '%.3f %.3f' % (p[0] * 1e3, p[1] * 1e3)
    segs = []
    for it in ol:
        if it[0] == 'line':
            segs.append('[line %s %s]' % (f(it[1]), f(it[2])))
        else:
            segs.append('[arc %s %s %s %s]' % (f(it[1]), f(it[2]), f(it[3]), 'true' if it[4] else 'false'))
    return '[extrude %.3f %.3f %.3f [sketch %.3f %.3f %.3f  %.5f %.5f %.5f  %.5f %.5f %.5f  #[%s]]]' % (
        vec[0] * 1e3, vec[1] * 1e3, vec[2] * 1e3, o[0] * 1e3, o[1] * 1e3, o[2] * 1e3,
        u[0], u[1], u[2], v[0], v[1], v[2], ' '.join(segs))


MATERIALS = ('[material "actuator" 0.10 0.10 0.11 0.6 0.5]\n[material "sheet" 0.80 0.81 0.82 0.9 0.35]\n'
             '[material "base" 0.55 0.56 0.58 0.9 0.4]\n[material "bar" 0.70 0.72 0.75 0.95 0.45]\n[material "cable" 0.95 0.40 0.10 0.0 0.6]\n'
             '[material "cable_cam" 0.15 0.35 0.80 0.0 0.6]\n')


def world_loon(q, extra=(), title='rose arm'):
    T = fk(q)
    lines = ['; %s - posed world geometry. GENERATED by arm/rose_arm_geom.py' % title, MATERIALS]
    roots = []
    for i, p in enumerate(primitives()):
        pp = pose_prim(T, p)
        lines.append('[let p%d %s]' % (i, prim_loon(pp)))
        roots.append('[root p%d "%s"]' % (i, p[1]))
    for i, (src, mat) in enumerate(extra):
        lines.append('[let c%d %s]' % (i, src))
        roots.append('[root c%d "%s"]' % (i, mat))
    return '\n'.join(lines + [''] + roots) + '\n'


# ------------------------------------------------------------------ point clouds for distance checks
def prim_points(prim, step=0.008):
    """Sample a primitive's outline on both faces plus a few interior lines."""
    _, _, o, u, v, ol, vec = prim
    pts2 = []
    for it in ol:
        if it[0] == 'line':
            p, q = it[1], it[2]
            n = max(2, int(math.hypot(q[0] - p[0], q[1] - p[1]) / step) + 1)
            pts2 += [(p[0] + (q[0] - p[0]) * k / (n - 1), p[1] + (q[1] - p[1]) * k / (n - 1)) for k in range(n)]
        else:
            p, q, c, ccw = it[1], it[2], it[3], it[4]
            r = math.hypot(p[0] - c[0], p[1] - c[1])
            a0 = math.atan2(p[1] - c[1], p[0] - c[0])
            a1 = math.atan2(q[1] - c[1], q[0] - c[0])
            da = (a1 - a0) % (2 * math.pi) if ccw else -((a0 - a1) % (2 * math.pi))
            if abs(da) < 1e-9:
                da = 2 * math.pi if ccw else -2 * math.pi
            n = max(3, int(abs(da) * r / step) + 1)
            pts2 += [(c[0] + r * math.cos(a0 + da * k / (n - 1)), c[1] + r * math.sin(a0 + da * k / (n - 1))) for k in range(n)]
    L = norm(vec)
    nz = max(2, int(L / step) + 1)
    out = []
    for (a, b) in pts2:
        base = add(o, add(mul(u, a), mul(v, b)))
        for k in range(nz):
            out.append(add(base, mul(vec, k / (nz - 1))))
    return out


_CLOUD = None


def clouds():
    """Zero-pose point clouds per primitive, cached: [(link, mat, points, (lo, hi))]."""
    global _CLOUD
    if _CLOUD is None:
        _CLOUD = []
        for p in primitives():
            pts = prim_points(p)
            _CLOUD.append((p[0], p[1], pts))
    return _CLOUD


def posed_clouds(T):
    out = []
    for link, mat, pts in clouds():
        R, t = T[link]
        o = ORIGIN[link]
        P = [add(mv(R, sub(p, o)), t) for p in pts]
        lo = tuple(min(p[i] for p in P) for i in range(3))
        hi = tuple(max(p[i] for p in P) for i in range(3))
        out.append((link, mat, P, lo, hi))
    return out


if __name__ == '__main__':
    import json
    q = {}
    if len(sys.argv) > 1:
        q = {k: math.radians(v) for k, v in json.loads(sys.argv[1]).items()}
    sys.stdout.write(world_loon(q))
