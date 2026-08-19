"""Swept and static interference for rose pro's structure.

Two checks the generator never had, and the reason the model that "stands at
+0.0%" could carry an 80 x 3 x 72 mm interference between the thigh's knee-stator
plate and the shank's knee-output plate without anyone noticing: the sim only
touches the feet, and vcad's union hides one solid inside another.

  static  - every plate against every plate of a DIFFERENT link, zero pose.
            Plates of the same link may touch (that is what a bend is).
  swept   - the child side of a joint (its plates AND the actuator bodies fixed
            to it) rotated through the joint's URDF range, against the parent
            side. Point cloud on the moving set, analytic distance to the static
            primitives, so it is cheap enough to sample the range finely.

Everything is trunk-frame millimetres, left side, same as rose_pro_links.

The joint ranges come from rose_pro_export (they are what the URDF says the
joint can do), so this is checked against the SAME range the controller is
allowed to command. If a range changes there, it changes here.
"""
import math

import rose_pro_links as L

MIN_GAP = 4.0          # the layout's own clearance floor
STEP_DEG = 2.0

# joint ranges, radians, from rose_pro_export.add_joint calls
RANGES = {
    'knee':      (0.0, 2.3),
    'ank_pitch': (-0.9, 0.5),
    'ank_roll':  (-0.4, 0.4),
    'hip_yaw':   (-1.0, 1.0),
}

# which link's plates ride on the child side of each joint (everything distal)
CHILD_LINKS = {
    'knee':      ('shank', 'ankle', 'foot'),
    'ank_pitch': ('ankle', 'foot'),
    'ank_roll':  ('foot',),
    'hip_yaw':   ('thigh', 'shank', 'ankle', 'foot'),
}
# actuator BODIES fixed to the child side (their stator is on a child link)
CHILD_BODIES = {
    'knee':      ('ank_pitch',),
    'ank_pitch': ('ank_roll',),
    'ank_roll':  (),
    'hip_yaw':   ('knee', 'ank_pitch'),
}
# parent-side plates and bodies the child must clear
PARENT_LINKS = {
    'knee':      ('thigh', 'hipB'),
    'ank_pitch': ('shank', 'thigh'),
    'ank_roll':  ('ankle', 'shank'),
    'hip_yaw':   ('hipB', 'hipA', 'trunkS', 'trunkC'),
}
PARENT_BODIES = {
    'knee':      ('hip_yaw', 'knee'),
    'ank_pitch': ('knee', 'ank_pitch'),
    'ank_roll':  ('ank_pitch', 'ank_roll'),
    'hip_yaw':   ('hip_yaw', 'hip_roll', 'hip_pitch'),
}


def rot(axis, th):
    c, s = math.cos(th), math.sin(th)
    if axis == 'x':
        return [[1, 0, 0], [0, c, -s], [0, s, c]]
    if axis == 'y':
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def apply(R, c, p):
    d = [p[i] - c[i] for i in range(3)]
    return [c[i] + sum(R[i][k] * d[k] for k in range(3)) for i in range(3)]


def plate_points(pl, pitch=6.0):
    """Sample a plate as the ROUNDED rectangle it actually is.

    The loon gives every plate up to 25 mm corner radii (rose_pro_loon.corner_radius);
    a plain box over-reports the corners, and a plate corner is exactly what sweeps
    furthest. So sample the rounded outline in-plane, on both faces and the edge.
    """
    i = L.AXI[pl.normal]
    ui, vi = [k for k in range(3) if k != i]
    r = min(25.0, (pl.u1 - pl.u0) / 2.5, (pl.v1 - pl.v0) / 2.5)
    # back off where a hole would break through the rounded corner, like the loon does
    from rose_pro_loon import corner_radius
    r = corner_radius(pl.u0, pl.u1, pl.v0, pl.v1, pl.holes)
    def outline():
        # perimeter of a rounded rect, as points
        out = []
        cx0, cx1, cy0, cy1 = pl.u0 + r, pl.u1 - r, pl.v0 + r, pl.v1 - r
        n_arc = max(3, int(round(r * 1.5708 / pitch)))
        segs = [((cx1, pl.v0), (cx0, pl.v0)), ((pl.u0, cy0), (pl.u0, cy1)),
                ((cx0, pl.v1), (cx1, pl.v1)), ((pl.u1, cy1), (pl.u1, cy0))]
        corners = [((cx1, cy0), -90.0), ((cx0, cy0), 180.0), ((cx0, cy1), 90.0), ((cx1, cy1), 0.0)]
        for (a, b_) in segs:
            n = max(1, int(round(math.hypot(b_[0] - a[0], b_[1] - a[1]) / pitch)))
            out += [(a[0] + (b_[0] - a[0]) * t / n, a[1] + (b_[1] - a[1]) * t / n) for t in range(n + 1)]
        for ((cx, cy), a0) in corners:
            for k in range(n_arc + 1):
                th = math.radians(a0 + 90.0 * k / n_arc)
                out.append((cx + r * math.cos(th), cy + r * math.sin(th)))
        return out
    per = outline()
    # interior grid on both faces (the edge points already cover the rim)
    inner = []
    nu = max(1, int(round((pl.u1 - pl.u0) / pitch)))
    nv = max(1, int(round((pl.v1 - pl.v0) / pitch)))
    for a in range(nu + 1):
        for b_ in range(nv + 1):
            u = pl.u0 + (pl.u1 - pl.u0) * a / nu
            v = pl.v0 + (pl.v1 - pl.v0) * b_ / nv
            if rrect_ok(u, v, pl, r):
                inner.append((u, v))
    pts = []
    for n in (pl.n0, pl.n1, (pl.n0 + pl.n1) / 2.0):
        for (u, v) in per + (inner if n != (pl.n0 + pl.n1) / 2.0 else []):
            p = [0.0] * 3
            p[i] = n
            p[ui] = u
            p[vi] = v
            pts.append(tuple(p))
    return pts


def rrect_ok(u, v, pl, r):
    ax, ay = (pl.u1 - pl.u0) / 2.0, (pl.v1 - pl.v0) / 2.0
    cx, cy = (pl.u0 + pl.u1) / 2.0, (pl.v0 + pl.v1) / 2.0
    qx, qy = abs(u - cx) - ax + r, abs(v - cy) - ay + r
    return (min(max(qx, qy), 0.0) + math.hypot(max(qx, 0.0), max(qy, 0.0)) - r) <= 0.0


def box_points(b, pitch=6.0):
    """Sample the six faces of an axis-aligned box."""
    pts = []
    (x0, x1), (y0, y1), (z0, z1) = b
    def lin(a, b_):
        n = max(1, int(round((b_ - a) / pitch)))
        return [a + (b_ - a) * i / n for i in range(n + 1)]
    xs, ys, zs = lin(x0, x1), lin(y0, y1), lin(z0, z1)
    for x in xs:
        for y in ys:
            pts += [(x, y, z0), (x, y, z1)]
    for x in xs:
        for z in zs:
            pts += [(x, y0, z), (x, y1, z)]
    for y in ys:
        for z in zs:
            pts += [(x0, y, z), (x1, y, z)]
    return pts


def cyl_points(ax, cu, cv, r, lo, hi, n_ax=8, n_ang=24):
    """Sample a cylinder's curved surface plus both rims (with a cap centre)."""
    i = L.AXI[ax]
    ui, vi = [k for k in range(3) if k != i]
    pts = []
    for a in range(n_ax + 1):
        t = lo + (hi - lo) * a / n_ax
        for j in range(n_ang):
            th = 2 * math.pi * j / n_ang
            p = [0.0] * 3
            p[i] = t
            p[ui] = cu + r * math.cos(th)
            p[vi] = cv + r * math.sin(th)
            pts.append(tuple(p))
    for t in (lo, hi):
        for rr in (r * 0.5, 0.0):
            for j in range(n_ang if rr else 1):
                th = 2 * math.pi * j / n_ang
                p = [0.0] * 3
                p[i] = t
                p[ui] = cu + rr * math.cos(th)
                p[vi] = cv + rr * math.sin(th)
                pts.append(tuple(p))
    return pts


def d_box(p, b):
    """Signed-ish distance from a point to a box: >0 outside, <=0 inside."""
    dx = max(b[0][0] - p[0], 0.0, p[0] - b[0][1])
    dy = max(b[1][0] - p[1], 0.0, p[1] - b[1][1])
    dz = max(b[2][0] - p[2], 0.0, p[2] - b[2][1])
    d = math.sqrt(dx * dx + dy * dy + dz * dz)
    if d > 0:
        return d
    # inside: negative penetration depth
    return -min(p[0] - b[0][0], b[0][1] - p[0], p[1] - b[1][0], b[1][1] - p[1],
                p[2] - b[2][0], b[2][1] - p[2])


def d_plate(p, pl, r=None):
    """Distance from a point to a plate modelled as a ROUNDED-rect slab.

    In-plane rounded-rect SDF combined with the thickness: the corners are what
    the box model gets wrong, and static plates deserve the same honesty as
    moving ones.
    """
    if r is None:
        from rose_pro_loon import corner_radius
        r = corner_radius(pl.u0, pl.u1, pl.v0, pl.v1, pl.holes)
    i = L.AXI[pl.normal]
    ui, vi = [k for k in range(3) if k != i]
    ax, ay = (pl.u1 - pl.u0) / 2.0, (pl.v1 - pl.v0) / 2.0
    cx, cy = (pl.u0 + pl.u1) / 2.0, (pl.v0 + pl.v1) / 2.0
    qx, qy = abs(p[ui] - cx) - ax + r, abs(p[vi] - cy) - ay + r
    dxy = min(max(qx, qy), 0.0) + math.hypot(max(qx, 0.0), max(qy, 0.0)) - r
    dn = max(pl.n0 - p[i], 0.0, p[i] - pl.n1)
    if dxy > 0 or dn > 0:
        return math.hypot(max(dxy, 0.0), dn)
    return max(dxy, -min(p[i] - pl.n0, pl.n1 - p[i]))


def d_cyl(p, ax, cu, cv, r, lo, hi):
    i = L.AXI[ax]
    ui, vi = [k for k in range(3) if k != i]
    dr = math.hypot(p[ui] - cu, p[vi] - cv) - r
    da = max(lo - p[i], 0.0, p[i] - hi)
    if dr > 0 or da > 0:
        return math.hypot(max(dr, 0.0), da)
    return max(dr, -min(p[i] - lo, hi - p[i]))    # inside: negative


def bodies_by_name():
    out = {}
    for (n, ax, (cu, cv), r, (lo, hi)) in L.obstacles():
        out[n] = (ax, cu, cv, r, lo, hi)
    return out


def static_check(plates):
    """Plate-vs-plate interference across links, zero pose.

    Box overlap first (cheap), then the real question for any box hit: does the
    ROUNDED outline of one plate actually reach into the other? A 25 mm corner
    radius removes exactly the corner nicks a box test reports.
    Returns (depth_mm, a, b, box_overlap_dims).
    """
    import itertools
    from rose_pro_loon import corner_radius
    hits = []
    for p, q in itertools.combinations(plates, 2):
        if p.link == q.link:
            continue
        a, b = p.box(), q.box()
        d = [min(a[k][1], b[k][1]) - max(a[k][0], b[k][0]) for k in range(3)]
        if not all(v > 0.25 for v in d):
            continue
        rq = corner_radius(q.u0, q.u1, q.v0, q.v1, q.holes)
        rp = corner_radius(p.u0, p.u1, p.v0, p.v1, p.holes)
        depth = 0.0
        for pt in plate_points(p, pitch=3.0):
            depth = min(depth, d_plate(pt, q, rq))
        for pt in plate_points(q, pitch=3.0):
            depth = min(depth, d_plate(pt, p, rp))
        if depth < -0.25:
            hits.append((-depth, '%s/%s' % (p.link, p.tag), '%s/%s' % (q.link, q.tag), d))
    return hits


def sweep(joint, plates, verbose=True):
    """Rotate the child side through the joint range; report worst gap."""
    J = L.JOINTS[joint]
    ax = J['ax']
    c = J['c']
    B = bodies_by_name()

    moving = []      # (label, points)
    for p in plates:
        if p.link in CHILD_LINKS[joint]:
            moving.append(('%s/%s' % (p.link, p.tag), plate_points(p)))
    for bn in CHILD_BODIES[joint]:
        moving.append(('body:%s' % bn, cyl_points(*B[bn])))

    from rose_pro_loon import corner_radius
    static_plates = [(p, corner_radius(p.u0, p.u1, p.v0, p.v1, p.holes))
                     for p in plates if p.link in PARENT_LINKS[joint]]
    static_bodies = [(bn, B[bn]) for bn in PARENT_BODIES[joint]]

    lo, hi = RANGES[joint]
    n = int(math.ceil(math.degrees(hi - lo) / STEP_DEG))
    worst = (1e9, None, None, None)
    per_pair = {}
    for k in range(n + 1):
        th = lo + (hi - lo) * k / n
        R = rot(ax, th)
        for (ml, pts) in moving:
            for p0 in pts:
                p = apply(R, c, p0)
                for sp, sr in static_plates:
                    d = d_plate(p, sp, sr)
                    key = (ml, '%s/%s' % (sp.link, sp.tag))
                    if d < per_pair.get(key, (1e9,))[0]:
                        per_pair[key] = (d, math.degrees(th))
                    if d < worst[0]:
                        worst = (d, ml, key[1], math.degrees(th))
                for (bn, cyl) in static_bodies:
                    d = d_cyl(p, *cyl)
                    key = (ml, 'body:%s' % bn)
                    if d < per_pair.get(key, (1e9,))[0]:
                        per_pair[key] = (d, math.degrees(th))
                    if d < worst[0]:
                        worst = (d, ml, key[1], math.degrees(th))
    if verbose:
        print('\n--- sweep %s: %.0f..%.0f deg, step %.0f ---'
              % (joint, math.degrees(lo), math.degrees(hi), STEP_DEG))
        rows = sorted(per_pair.items(), key=lambda kv: kv[1][0])
        for (ml, sl), (d, th) in rows[:8]:
            # d == 0 is a pair that RUNS stacked face-to-face by design (stator
            # plate / output plate on one pancake face) - a bearing surface, not
            # a clash. Below zero is material inside material.
            flag = '  CLASH' if d < -0.05 else ('  TIGHT' if d < MIN_GAP else '')
            print('  %-24s vs %-24s %+7.1f mm at %5.0f deg%s' % (ml, sl, d, th, flag))
    return worst


def main():
    for p in L.PLATES:
        p.fit()
    print('=' * 72)
    print('static plate/plate interference across links (zero pose)')
    print('=' * 72)
    hits = static_check(L.PLATES)
    # depth is the deepest a point of one plate sits inside the other. Two
    # coplanar 3 mm plates fully overlapping give 1.5 (half a thickness), so
    # 1.2 is the line between "material inside material" and a fold-corner nick.
    real = [h for h in hits if h[0] >= 1.2]
    for depth, a, b, d in sorted(hits, key=lambda h: -h[0]):
        print('  %-22s x %-22s  box %5.1f x %5.1f x %5.1f  depth %.1f mm%s'
              % (a, b, d[0], d[1], d[2], depth, '   <-- REAL' if depth >= 1.2 else '   (fold nick)'))
    print('  %d overlaps, %d real' % (len(hits), len(real)))

    print('\n' + '=' * 72)
    print('swept interference through the URDF joint ranges')
    print('=' * 72)
    ok = True
    for j in ('knee', 'ank_pitch', 'ank_roll', 'hip_yaw'):
        d, ml, sl, th = sweep(j, L.PLATES)
        if d < -0.05:
            ok = False
    print('\n  VERDICT: %s' % ('PASS' if ok and not real else 'FAIL'))
    return ok and not real


if __name__ == '__main__':
    import sys
    sys.exit(0 if main() else 1)
