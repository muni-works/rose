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
    # Hip pitch: the leg against the pelvis, -115..92 deg. UNITREE.md listed this
    # under "Honest residue" as never swept at all - and the pelvis is now a
    # faceted pod whose lower fore/aft panels lean INTO exactly the volume the
    # thigh folds through, so it is the one that had to be added.
    'hip_pitch': (-2.0, 1.6),
    # Arms, added when the arms got structure. The URDF used to give shoulder
    # roll -1.7..1.6 rad on the left, i.e. it permitted adduction THROUGH the
    # torso, and the sweep dodged it by starting at 0. The H3 pass CAPPED it
    # instead: rose_pro_export now says -0.21..1.7 (left; mirrored right), and
    # -0.21 rad (-12 deg) is swept here like any other limit. -12 deg is where
    # the FOREARM's side walls close on the chest side plate (they hang 362 mm
    # under the roll axis, so every degree of adduction moves them 6.3 mm); the
    # second limiter, 1 deg behind it, is the upper arm's web against the roll
    # stator plate. The arm can lie against the body and no further.
    # 1.7 stands. It looked for a while like the skin cost 9 deg of abduction
    # (the upper-arm shell met the chest shell at 97 deg), but the collider was
    # a band of shoB's rear web that the shoulder-roll seam should have taken
    # and did not: r 56 instead of 54 (skin.py SEAM) removes it and the pair
    # clears by +3.0 mm at the full 1.7. rose_pro_export says the same.
    'sho_roll':  (-0.21, 1.7),
    'sho_pitch': (-3.0, 1.5),      # the whole arm about the shoulder, never swept before
    'wrist_yaw': (-2.4, 2.4),
    # -2.3 (132 deg) stands, and the BONES are what hold it there, not the skin:
    # 6 deg further the forearm's front wall reaches the shoulder-roll can
    # (+1.2 mm at 138 deg, through the layout's 4 mm floor at 136, touching at
    # 139). The skin has 3.5 mm to spare over the whole of that range once the
    # forearm form block is stepped to follow the bones (skin.py FORM), so the
    # skin costs the elbow nothing - the elbow is a bone-layout limit.
    'elb_pitch': (-2.3, 0.3),      # forward flexion only (export has the same): folded back it meets the shoulder roll pod
    'waist_yaw': (-1.0, 1.0),
    # the head. Added with the H3 helmet: the helmet's rim is 29 mm off the chest
    # deck and it PITCHES, so the underside profile is a swept result, not a shape.
    'head_yaw':   (-1.0, 1.0),
    'head_pitch': (-0.35, 0.85),
}

# which link's plates ride on the child side of each joint (everything distal)
CHILD_LINKS = {
    'knee':      ('shank', 'crankA', 'crankB', 'ankle', 'foot'),
    'ank_pitch': ('ankle', 'foot'),
    'ank_roll':  ('foot',),
    'hip_yaw':   ('thigh', 'shank', 'crankA', 'crankB', 'ankle', 'foot'),
    'hip_pitch': ('hipA', 'hipB', 'thigh', 'shank', 'crankA', 'crankB', 'ankle', 'foot'),
    'sho_roll':  ('shoB', 'elbA', 'hand'),
    'elb_pitch': ('elbA', 'hand'),
    'sho_pitch': ('shoA', 'shoB', 'elbA', 'hand'),
    'wrist_yaw': ('hand',),
    'waist_yaw': ('trunkS', 'trunkC', 'neck', 'head', 'shoA', 'shoC'),
    'head_yaw':   ('neck', 'head'),
    'head_pitch': ('head',),
}
# actuator BODIES fixed to the child side (their stator is on a child link)
CHILD_BODIES = {
    'knee':      ('ank_mA', 'ank_mB', 'ank_pitch', 'ank_roll'),
    'ank_pitch': ('ank_roll',),
    'ank_roll':  (),
    'hip_yaw':   ('knee', 'ank_mA', 'ank_mB', 'ank_pitch', 'ank_roll'),
    'hip_pitch': ('hip_roll', 'hip_yaw', 'knee', 'ank_mA', 'ank_mB', 'ank_pitch', 'ank_roll'),
    'sho_roll':  ('elb_pitch', 'wrist_yaw'),
    'elb_pitch': ('wrist_yaw',),
    'sho_pitch': ('sho_roll', 'elb_pitch', 'wrist_yaw'),
    'wrist_yaw': (),
    'waist_yaw': ('sho_pitch', 'sho_roll', 'head_yaw', 'head_pitch'),
    'head_yaw':   ('head_pitch',),
    'head_pitch': (),
}
# parent-side plates and bodies the child must clear
PARENT_LINKS = {
    'knee':      ('thigh', 'hipB'),
    'ank_pitch': ('shank', 'crankA', 'crankB', 'thigh'),
    'ank_roll':  ('ankle', 'shank'),
    'hip_yaw':   ('hipB', 'hipA', 'pelvisS', 'pelvisC', 'trunkS', 'trunkC'),
    'hip_pitch': ('pelvisS', 'pelvisC', 'trunkS', 'trunkC'),
    'sho_roll':  ('shoA', 'shoC', 'trunkS', 'trunkC'),
    # the whole arm about the shoulder pitch, against the trunk and the new cap
    'sho_pitch': ('shoC', 'trunkS', 'trunkC', 'neck', 'head'),
    'wrist_yaw': ('elbA',),
    # the torso over the pelvis. Arms EXCLUDED: hanging straight down they are
    # beside the hip-roll pods and a 25 deg waist yaw swings them into the pods -
    # true of a person too, and a pose the controller lifts the arms out of.
    'waist_yaw': ('pelvisS', 'pelvisC', 'hipA'),
    'elb_pitch': ('shoB', 'shoA'),
    'head_yaw':   ('trunkC', 'trunkS'),
    'head_pitch': ('neck', 'trunkC', 'trunkS'),
}
PARENT_BODIES = {
    'knee':      ('hip_yaw', 'knee'),
    'ank_pitch': ('knee', 'ank_mA', 'ank_mB', 'ank_pitch'),
    'ank_roll':  ('ank_pitch', 'ank_roll'),
    'hip_yaw':   ('hip_yaw', 'hip_roll', 'hip_pitch'),
    'hip_pitch': ('hip_pitch', 'waist_yaw'),
    'sho_roll':  ('sho_roll', 'sho_pitch'),
    'elb_pitch': ('elb_pitch', 'sho_roll'),
    'sho_pitch': ('sho_pitch',),
    'wrist_yaw': ('wrist_yaw',),
    'waist_yaw': ('waist_yaw', 'hip_pitch', 'hip_roll'),
    'head_yaw':   ('head_yaw', 'sho_pitch'),
    'head_pitch': ('head_yaw', 'head_pitch'),
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


def plate_r(pl):
    """The corner radius the loon gives this plate (backs off where a hole says no)."""
    from rose_pro_loon import corner_radius
    return corner_radius(pl.u0, pl.u1, pl.v0, pl.v1, pl.holes, want=pl.r_want, poly=pl.poly)


def plate_points(pl, pitch=6.0):
    """Sample a plate as the ROUNDED outline it actually is, in WORLD coordinates.

    The loon gives every plate rounded corners (rose_pro_loon.corner_radius); a
    plain box over-reports the corners, and a plate corner is exactly what sweeps
    furthest. So sample the rounded outline in-plane (rect or polygon), on both
    faces and the edge, then map through the plate's own frame - which applies
    its tilt, if it has one.
    """
    r = plate_r(pl)
    per = []
    if pl.poly:
        poly = pl.outline()
        n = len(poly)
        ins = L.poly_inset(poly, r)                     # arc centres sit at the inset vertices
        for i in range(n):
            a, b_ = poly[i], poly[(i + 1) % n]
            ex, ey = b_[0] - a[0], b_[1] - a[1]
            ln = math.hypot(ex, ey) or 1e-9
            nx, ny = ey / ln, -ex / ln                  # outward normal (CCW polygon)
            c0, c1 = ins[i], ins[(i + 1) % n]
            p0 = (c0[0] + nx * r, c0[1] + ny * r)       # straight part between tangent points
            p1 = (c1[0] + nx * r, c1[1] + ny * r)
            m = max(1, int(round(math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / pitch)))
            per += [(p0[0] + (p1[0] - p0[0]) * t / m, p0[1] + (p1[1] - p0[1]) * t / m) for t in range(m + 1)]
            nxt = poly[(i + 2) % n]                     # arc at vertex i+1, this normal -> next normal
            fx, fy = nxt[0] - b_[0], nxt[1] - b_[1]
            fl = math.hypot(fx, fy) or 1e-9
            mx, my = fy / fl, -fx / fl
            a0, a1 = math.atan2(ny, nx), math.atan2(my, mx)
            while a1 < a0:
                a1 += 2 * math.pi
            k = max(2, int(round(r * (a1 - a0) / pitch)))
            per += [(c1[0] + r * math.cos(a0 + (a1 - a0) * t / k), c1[1] + r * math.sin(a0 + (a1 - a0) * t / k))
                    for t in range(k + 1)]
    else:
        cx0, cx1, cy0, cy1 = pl.u0 + r, pl.u1 - r, pl.v0 + r, pl.v1 - r
        n_arc = max(3, int(round(r * 1.5708 / pitch)))
        segs = [((cx1, pl.v0), (cx0, pl.v0)), ((pl.u0, cy0), (pl.u0, cy1)),
                ((cx0, pl.v1), (cx1, pl.v1)), ((pl.u1, cy1), (pl.u1, cy0))]
        corners = [((cx1, cy0), -90.0), ((cx0, cy0), 180.0), ((cx0, cy1), 90.0), ((cx1, cy1), 0.0)]
        for (a, b_) in segs:
            m = max(1, int(round(math.hypot(b_[0] - a[0], b_[1] - a[1]) / pitch)))
            per += [(a[0] + (b_[0] - a[0]) * t / m, a[1] + (b_[1] - a[1]) * t / m) for t in range(m + 1)]
        for ((cx, cy), a0) in corners:
            for k in range(n_arc + 1):
                th = math.radians(a0 + 90.0 * k / n_arc)
                per.append((cx + r * math.cos(th), cy + r * math.sin(th)))
    # interior grid on both faces (the edge points already cover the rim)
    inner = []
    nu = max(1, int(round((pl.u1 - pl.u0) / pitch)))
    nv = max(1, int(round((pl.v1 - pl.v0) / pitch)))
    for a in range(nu + 1):
        for b_ in range(nv + 1):
            u = pl.u0 + (pl.u1 - pl.u0) * a / nu
            v = pl.v0 + (pl.v1 - pl.v0) * b_ / nv
            if pl.sdf_uv(u, v, r) <= 0.0:
                inner.append((u, v))
    pts = []
    for nn in (pl.n0, pl.n1, (pl.n0 + pl.n1) / 2.0):
        for (u, v) in per + (inner if nn != (pl.n0 + pl.n1) / 2.0 else []):
            pts.append(pl.local_to_world(nn, u, v))
    return pts


def rrect_ok(u, v, pl, r):
    return pl.sdf_uv(u, v, r) <= 0.0


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
    """Distance from a WORLD point to a plate: its rounded outline (rect or
    polygon) combined with its thickness, in the plate's own (possibly tilted)
    frame. The corners are what the box model gets wrong, and static plates
    deserve the same honesty as moving ones.
    """
    if r is None:
        r = plate_r(pl)
    n, u, v = pl.world_to_local(p)
    dxy = pl.sdf_uv(u, v, r)
    dn = max(pl.n0 - n, 0.0, n - pl.n1)
    if dxy > 0 or dn > 0:
        return math.hypot(max(dxy, 0.0), dn)
    return max(dxy, -min(n - pl.n0, pl.n1 - n))


def d_cyl(p, ax, cu, cv, r, lo, hi):
    i = L.AXI[ax]
    ui, vi = [k for k in range(3) if k != i]
    dr = math.hypot(p[ui] - cu, p[vi] - cv) - r
    da = max(lo - p[i], 0.0, p[i] - hi)
    if dr > 0 or da > 0:
        return math.hypot(max(dr, 0.0), da)
    return max(dr, -min(p[i] - lo, hi - p[i]))    # inside: negative


def passes_through(body, pl):
    """A body whose axis runs through a clearance hole in the plate (a pin through
    a fork, an actuator through a web) is not a clash with that plate."""
    ax, cu, cv, r, lo, hi = body
    if pl.normal != ax:
        return False
    return any(math.hypot(hu - cu, hv - cv) < 1.0 and hd / 2.0 >= r for (hu, hv, hd) in pl.holes)


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
    hits = []
    for p, q in itertools.combinations(plates, 2):
        if p.link == q.link:
            continue
        a, b = p.box(), q.box()
        d = [min(a[k][1], b[k][1]) - max(a[k][0], b[k][0]) for k in range(3)]
        if not all(v > 0.25 for v in d):
            continue
        rq = plate_r(q)
        rp = plate_r(p)
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
            moving.append(('%s/%s' % (p.link, p.tag), plate_points(p), p))
    for bn in CHILD_BODIES[joint]:
        moving.append(('body:%s' % bn, cyl_points(*B[bn]), B[bn]))

    static_plates = [(p, plate_r(p))
                     for p in plates if p.link in PARENT_LINKS[joint]]
    static_bodies = [(bn, B[bn]) for bn in PARENT_BODIES[joint]]

    lo, hi = RANGES[joint]
    n = int(math.ceil(math.degrees(hi - lo) / STEP_DEG))
    worst = (1e9, None, None, None)
    per_pair = {}
    for k in range(n + 1):
        th = lo + (hi - lo) * k / n
        R = rot(ax, th)
        for (ml, pts, mobj) in moving:
            for p0 in pts:
                p = apply(R, c, p0)
                for sp, sr in static_plates:
                    if not isinstance(mobj, L.Plate) and passes_through(mobj, sp):
                        continue
                    d = d_plate(p, sp, sr)
                    key = (ml, '%s/%s' % (sp.link, sp.tag))
                    if d < per_pair.get(key, (1e9,))[0]:
                        per_pair[key] = (d, math.degrees(th))
                    if d < worst[0]:
                        worst = (d, ml, key[1], math.degrees(th))
                for (bn, cyl) in static_bodies:
                    if isinstance(mobj, L.Plate) and passes_through(cyl, mobj):
                        continue
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
    for j in ('knee', 'ank_pitch', 'ank_roll', 'hip_pitch', 'hip_yaw', 'sho_pitch', 'sho_roll',
              'elb_pitch', 'wrist_yaw', 'waist_yaw', 'head_yaw', 'head_pitch'):
        d, ml, sl, th = sweep(j, L.PLATES)
        if d < -0.05:
            ok = False
    print('\n  VERDICT: %s' % ('PASS' if ok and not real else 'FAIL'))
    return ok and not real


if __name__ == '__main__':
    import sys
    sys.exit(0 if main() else 1)
