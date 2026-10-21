"""Interference and pinch-point audit for the fork arm.

Every primitive is a sketch outline (lines / arcs, minus its windows) extruded along a vector, so
a point is INSIDE a primitive when it projects inside the outline (and not inside a window) and
lies between the two faces. That gives a true solid-in-solid test, not just a distance.

Poses: every frame of the motion video plus a full sweep of each pitch joint.

Findings, per pair of links that move relative to each other:
  INTERFERENCE  points of one body inside another (> 0.5 mm deep) - a geometry error
  CRUSH         their gap closes below 25 mm (ISO 13854 finger) from wider, at some pose
  SHEAR         a gap that stays between 4 and 25 mm while they move relative to each other
Gaps <= 4 mm that never open are not finger-reachable (ISO 13857) and are not flagged, nor are
parts of the same link (they do not move relative to each other), but same-link interference at
the zero pose is reported separately - two parts of one link occupying the same space is a model bug.

Run:  python3 arm/fork/audit.py
"""
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import rose_fork as F
import rose_arm_geom as Gm
from rose_arm_geom import add, sub, mul, dot, cross, norm, unit

FINGER, NOREACH, DEPTH = 0.025, 0.004, 0.0005


def poly(ol, step=0.003):
    pts = []
    for it in ol:
        if it[0] == 'line':
            pts.append(it[1])
        else:
            p, q, c, ccw = it[1], it[2], it[3], it[4]
            r = math.hypot(p[0] - c[0], p[1] - c[1])
            a0 = math.atan2(p[1] - c[1], p[0] - c[0])
            a1 = math.atan2(q[1] - c[1], q[0] - c[0])
            da = (a1 - a0) % (2 * math.pi) if ccw else -((a0 - a1) % (2 * math.pi))
            if abs(da) < 1e-9:
                da = 2 * math.pi if ccw else -2 * math.pi
            n = max(4, int(abs(da) * r / step))
            pts += [(c[0] + r * math.cos(a0 + da * k / n), c[1] + r * math.sin(a0 + da * k / n)) for k in range(n)]
    return pts


def in_poly(x, y, P):
    c = False
    n = len(P)
    for i in range(n):
        (x1, y1), (x2, y2) = P[i], P[(i + 1) % n]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            c = not c
    return c


def edge_dist(x, y, P):
    best = 1e9
    n = len(P)
    for i in range(n):
        (x1, y1), (x2, y2) = P[i], P[(i + 1) % n]
        dx, dy = x2 - x1, y2 - y1
        L2 = dx * dx + dy * dy or 1e-18
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / L2))
        best = min(best, math.hypot(x - x1 - t * dx, y - y1 - t * dy))
    return best


class Solid:
    def __init__(self, i, prim):
        link, mat, o, u, v, ol, vec = prim[:7]
        self.i, self.link, self.mat = i, link, mat
        self.o, self.u, self.v, self.vec = o, unit(u), unit(v), vec
        self.L = norm(vec)
        self.n = unit(vec)
        self.P = poly(ol)
        self.H = [poly(h) for h in (prim[7] if len(prim) > 7 else [])]
        self.surf = Gm.prim_points(prim[:7], step=0.006)
        lo = tuple(min(p[k] for p in self.surf) for k in range(3))
        hi = tuple(max(p[k] for p in self.surf) for k in range(3))
        self.lo, self.hi = lo, hi
        self.name = '%s %s #%d' % (link, mat, i)

    def depth(self, p):
        """How far p is inside this solid (m), or <= 0 if outside."""
        d = sub(p, self.o)
        a, b, c = dot(d, self.u), dot(d, self.v), dot(d, self.n)
        if c <= 0 or c >= self.L or not in_poly(a, b, self.P):
            return 0.0
        for h in self.H:
            if in_poly(a, b, h):
                return 0.0
        return min(c, self.L - c, edge_dist(a, b, self.P))


SOLIDS = [Solid(i, p) for i, p in enumerate(F.build())]
LINKS = [l[0] for l in F.LINKS]


def to_local(T, link, p):
    """World point -> the zero-pose frame of `link`."""
    R, t = T[link]
    d = sub(p, t)
    return add((R[0][0] * d[0] + R[1][0] * d[1] + R[2][0] * d[2],
                R[0][1] * d[0] + R[1][1] * d[1] + R[2][1] * d[2],
                R[0][2] * d[0] + R[1][2] * d[1] + R[2][2] * d[2]), F.ORIGIN[link])


def posed_surf(T, s):
    R, t = T[s.link]
    o = F.ORIGIN[s.link]
    return [add(Gm.mv(R, sub(p, o)), t) for p in s.surf]


def aabb(P):
    return tuple(min(p[k] for p in P) for k in range(3)), tuple(max(p[k] for p in P) for k in range(3))


def overlap(a, b, pad):
    return all(min(a[1][k], b[1][k]) - max(a[0][k], b[0][k]) > -pad for k in range(3))


def min_gap(PA, PB, cap):
    best = cap
    for x in PA[::2]:
        for y in PB[::2]:
            d = (x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2 + (x[2] - y[2]) ** 2
            if d < best * best:
                best = math.sqrt(d)
    return best


def poses():
    import importlib.util
    spec = importlib.util.spec_from_file_location('anim', '/tmp/claude-501/v6anim.py')
    # the motion keyframes, re-stated (the anim script renders on import)
    K = [
        (0.0, dict(J1_yaw=0, J2_sho_pitch=-80, J4_elbow=100, J5_fore_roll=0, J6_wrist_pitch=-60, J7_tool_roll=0)),
        (1.6, dict(J1_yaw=0, J2_sho_pitch=-25, J4_elbow=60, J5_fore_roll=0, J6_wrist_pitch=55, J7_tool_roll=0)),
        (3.0, dict(J1_yaw=-60, J2_sho_pitch=-10, J4_elbow=45, J5_fore_roll=0, J6_wrist_pitch=55, J7_tool_roll=90)),
        (4.2, dict(J1_yaw=-60, J2_sho_pitch=-10, J4_elbow=45, J5_fore_roll=80, J6_wrist_pitch=30, J7_tool_roll=-90)),
        (5.6, dict(J1_yaw=50, J2_sho_pitch=-40, J4_elbow=80, J5_fore_roll=-60, J6_wrist_pitch=40, J7_tool_roll=0)),
        (7.0, dict(J1_yaw=50, J2_sho_pitch=0, J4_elbow=0, J5_fore_roll=0, J6_wrist_pitch=0, J7_tool_roll=0)),
        (8.6, dict(J1_yaw=0, J2_sho_pitch=-80, J4_elbow=100, J5_fore_roll=0, J6_wrist_pitch=-60, J7_tool_roll=0)),
    ]
    out = []
    for i in range(0, int(K[-1][0] * 15) + 1, 2):
        t = i / 15
        for (t0, a), (t1, b) in zip(K, K[1:]):
            if t0 <= t <= t1:
                s = (t - t0) / (t1 - t0)
                s = s * s * (3 - 2 * s)
                q = {j: math.radians(a[j] + (b[j] - a[j]) * s) for j in a}
                if F.V10:      # v10: the roll the video exercises is the upper arm's
                    q['J3_upper_roll'] = q.pop('J5_fore_roll')
                out.append(('video %.1fs' % t, q))
                break
    # v10: the short turret stops J2 at +20 (down)
    for j, lo, hi in (('J2_sho_pitch', -180, 20 if F.V10 else 40), ('J4_elbow', -105, 105), ('J6_wrist_pitch', -120, 120),
                      ('J3_upper_roll' if F.V10 else 'J5_fore_roll', -180, 180), ('J7_tool_roll', -180, 180)):
        for d in range(lo, hi + 1, 10):
            out.append(('%s %d' % (j, d), {j: math.radians(d)}))
    return out


def main():
    print('%d solids, %d links' % (len(SOLIDS), len(LINKS)))
    # 1. same-link interference at the zero pose (model bugs)
    print('\n== same-link interference at the zero pose (two parts of one link in the same space) ==')
    found = False
    for a, b in itertools.combinations(SOLIDS, 2):
        if a.link != b.link or not overlap((a.lo, a.hi), (b.lo, b.hi), 0):
            continue
        da = max((b.depth(p) for p in a.surf), default=0)
        db = max((a.depth(p) for p in b.surf), default=0)
        d = max(da, db)
        if d > DEPTH:
            found = True
            print('  %-28s <-> %-28s %.1f mm deep' % (a.name, b.name, d * 1e3))
    if not found:
        print('  none')
    # 2. across poses
    P = poses()
    worst_int = {}
    gaps = {}
    for label, q in P:
        T = F.fk(q)
        posed = {s.i: posed_surf(T, s) for s in SOLIDS}
        boxes = {i: aabb(v) for i, v in posed.items()}
        for a, b in itertools.combinations(SOLIDS, 2):
            if a.link == b.link:
                continue
            key = (a.i, b.i)
            if not overlap(boxes[a.i], boxes[b.i], FINGER):
                g = FINGER
            else:
                # interference: a's posed surface points inside b, and vice versa
                if overlap(boxes[a.i], boxes[b.i], 0):
                    d1 = max((b.depth(to_local(T, b.link, p)) for p in posed[a.i][::2]), default=0)
                    d2 = max((a.depth(to_local(T, a.link, p)) for p in posed[b.i][::2]), default=0)
                    d = max(d1, d2)
                    if d > DEPTH and d > worst_int.get(key, (0, ''))[0]:
                        worst_int[key] = (d, label)
                g = min_gap(posed[a.i], posed[b.i], FINGER)
            mn, mx, at = gaps.get(key, (1e9, 0.0, ''))
            if g < mn:
                mn, at = g, label
            gaps[key] = (mn, max(mx, g), at)
    S = {s.i: s for s in SOLIDS}
    print('\n== INTERFERENCE across %d poses (video frames + joint sweeps) ==' % len(P))
    if not worst_int:
        print('  none')
    for (i, j), (d, label) in sorted(worst_int.items(), key=lambda kv: -kv[1][0]):
        print('  %-28s <-> %-28s %5.1f mm deep at %s' % (S[i].name, S[j].name, d * 1e3, label))
    print('\n== CRUSH: a gap that closes under 25 mm from wider ==')
    crush = [(k, v) for k, v in gaps.items() if v[0] < FINGER and v[1] >= FINGER and v[0] > 0 and k not in worst_int]
    if not crush:
        print('  none')
    for (i, j), (mn, mx, at) in sorted(crush, key=lambda kv: kv[1][0]):
        print('  %-28s <-> %-28s closes to %4.1f mm at %s' % (S[i].name, S[j].name, mn * 1e3, at))
    print('\n== SHEAR: stays 4-25 mm while moving relative ==')
    shear = [(k, v) for k, v in gaps.items() if NOREACH < v[0] < FINGER and v[1] < FINGER and k not in worst_int]
    if not shear:
        print('  none')
    for (i, j), (mn, mx, at) in sorted(shear, key=lambda kv: kv[1][0]):
        print('  %-28s <-> %-28s %4.1f .. %4.1f mm' % (S[i].name, S[j].name, mn * 1e3, mx * 1e3))


if __name__ == '__main__':
    main()


def interferes(q, gap_floor=0.0):
    """True if any two links interpenetrate at pose q (or, with gap_floor, come closer than it)."""
    T = F.fk(q)
    posed = {s.i: posed_surf(T, s) for s in SOLIDS}
    boxes = {i: aabb(v) for i, v in posed.items()}
    for a, b in itertools.combinations(SOLIDS, 2):
        if a.link == b.link or not overlap(boxes[a.i], boxes[b.i], gap_floor):
            continue
        if overlap(boxes[a.i], boxes[b.i], 0):
            if any(b.depth(to_local(T, b.link, p)) > DEPTH for p in posed[a.i][::2]):
                return (a.name, b.name)
            if any(a.depth(to_local(T, a.link, p)) > DEPTH for p in posed[b.i][::2]):
                return (a.name, b.name)
        if gap_floor and adjacent(a.link, b.link) is False and min_gap(posed[a.i], posed[b.i], gap_floor) < gap_floor:
            return (a.name, b.name)
    return None


PARENT = {l[0]: l[2] for l in F.LINKS}


def adjacent(a, b):
    return PARENT.get(a) == b or PARENT.get(b) == a


def joint_range(j, lo, hi, step=5, gap_floor=0.0):
    ok = {}
    a = 0
    while a - step >= lo and not interferes({j: math.radians(a - step)}, gap_floor):
        a -= step
    b = 0
    while b + step <= hi and not interferes({j: math.radians(b + step)}, gap_floor):
        b += step
    return a, b


def ranges():
    print('\n== true joint ranges, others at zero (no interference) ==')
    for j, lo, hi in (('J2_sho_pitch', -180, 180), ('J4_elbow', -180, 180), ('J6_wrist_pitch', -180, 180)):
        a, b = joint_range(j, lo, hi)
        hit_lo = interferes({j: math.radians(a - 5)}) if a - 5 >= lo else None
        hit_hi = interferes({j: math.radians(b + 5)}) if b + 5 <= hi else None
        print('  %-15s %5d .. %-5d  (next: %s / %s)' % (j, a, b, hit_lo, hit_hi))
