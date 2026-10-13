"""rose arm, fork concept (v2, 2026-10-12: windowed 3 mm plates, two bars, collar cassettes, hollow base).

6 DOF: base yaw, shoulder pitch, elbow pitch, then a spherical wrist (roll, pitch, roll).
Every pitch joint is a FORK: the child link is two flat side plates that straddle its parent's
end. One plate bolts to the motor's output flange; the other rides a plain bearing on a hollow
sleeve on the joint axis. The parent holds the motor's stator ring between its own plates.
So parent and child share one centreline - no offsets, no zig-zag.

Links taper in plan: wide where they straddle the joint behind them, narrow where they hold the
motor ahead. Each link is two side plates + 2020 bars along their inner edges + flat recessed
covers top and bottom (a closed box for torsion, but the side plates are what you see).

Cables:
  pitch joints  through the hollow sleeve, ON the axis: the cable only twists. No loop.
  roll / yaw    a clock-spring cassette coaxial with the joint, inside the base drum or the
                wrist yoke. Hidden.

Run:  python3 arm/fork/rose_fork.py [pose.json] > out.loon
"""
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
import rose_arm_geom as Gm
from rose_arm_geom import add, sub, mul, dot, cross, norm, unit, rot, mv, mm_, I3, rect_outline, prim_loon, circle_outline

# ---------------------------------------------------------------- parts (m)
RS06 = dict(d=0.088, L=0.064, kg=0.621)
RS00 = dict(d=0.061, L=0.066, kg=0.310)
EDU = dict(d=0.046, L=0.044, kg=0.242)
T_PL = 0.003          # fork side plate (v2: 4 -> 3 mm, windowed)
T_CV = 0.0015         # cover
GAP = 0.002           # plate to motor / plate to plate running clearance
BAR = 0.020

Z_SH = 0.210          # shoulder axis height
UPPER = 0.300         # v3: 330 -> 300
FORE = 0.260          # elbow to wrist centre, v3: 300 -> 260 (flange ~760 mm, reBot class)
W = UPPER + FORE      # wrist centre x
R_LOBE6 = RS06['d'] / 2 + 0.006      # lobe radius round an RS06 joint
R_LOBE0 = RS00['d'] / 2 + 0.006

# plate centre-plane half-widths (y), from the stack-up: motor | gap | parent plate | gap | child plate
def stack(motor_L, layers):
    """half-width of the centre plane of each nested plate layer around a motor of length motor_L."""
    out, y = [], motor_L / 2 + GAP
    for t in layers:
        out.append(y + t / 2)
        y += t + GAP
    return out


Y_TURRET, Y_UPPER_PROX = stack(RS06['L'], [T_PL, T_PL])          # shoulder: turret holds J2, upper straddles
Y_UPPER_DIST, Y_FORE_PROX = stack(RS06['L'], [T_PL, T_PL])        # elbow: upper holds J4, forearm straddles
Y_YOKE, Y_WRIST = stack(RS00['L'], [T_PL, T_PL])                  # wrist pitch: yoke holds J6, wrist straddles
Y_FORE_DIST = RS00['d'] / 2 + GAP + T_PL / 2 + 0.004              # forearm end holds the J5 roll motor (inline)

LINKS = [  # name, joint, parent, axis, origin
    ('base', None, None, None, (0.0, 0.0, 0.0)),
    ('turret', 'J1_yaw', 'base', (0, 0, 1), (0.0, 0.0, 0.120)),
    ('upper', 'J2_sho_pitch', 'turret', (0, 1, 0), (0.0, 0.0, Z_SH)),
    ('fore', 'J4_elbow', 'upper', (0, 1, 0), (UPPER, 0.0, Z_SH)),
    ('yoke', 'J5_fore_roll', 'fore', (1, 0, 0), (W - 0.070, 0.0, Z_SH)),
    ('wrist', 'J6_wrist_pitch', 'yoke', (0, 1, 0), (W, 0.0, Z_SH)),
    ('tool', 'J7_tool_roll', 'wrist', (1, 0, 0), (W + 0.040, 0.0, Z_SH)),
]
ORIGIN = {l[0]: l[4] for l in LINKS}


def fk(q):
    T = {}
    for name, joint, parent, axis, o in LINKS:
        if parent is None:
            T[name] = (I3, o)
            continue
        Rp, tp = T[parent]
        t = add(mv(Rp, sub(o, ORIGIN[parent])), tp)
        T[name] = (mm_(Rp, rot(axis, q.get(joint, 0.0))), t)
    return T


def xf(T, link, p):
    R, t = T[link]
    return add(mv(R, sub(p, ORIGIN[link])), t)


def xd(T, link, d):
    return mv(T[link][0], d)


# ---------------------------------------------------------------- shape builders
def capsule(L, r1, r2):
    """Tapered capsule outline in (s, z): circle r1 at s=0, r2 at s=L, outer tangents. Clockwise."""
    c = (r1 - r2) / L
    th = math.acos(max(-1.0, min(1.0, c)))
    p1t = (r1 * math.cos(th), r1 * math.sin(th))
    p2t = (L + r2 * math.cos(th), r2 * math.sin(th))
    p2b, p1b = (p2t[0], -p2t[1]), (p1t[0], -p1t[1])
    return [('line', p1t, p2t), ('arc', p2t, p2b, (L, 0.0), False),
            ('line', p2b, p1b), ('arc', p1b, p1t, (0.0, 0.0), False)]


def tower(r, foot_w, h):
    """Turret plate in (x, z) about the shoulder axis: a lobe on top, tangent flanks down to a foot."""
    import rose_arm_loon as LG
    fl, fr = (-foot_w / 2, -h), (foot_w / 2, -h)
    tl = LG._tangent(fl, (0.0, 0.0), r, upper=True)
    tr = LG._tangent(fr, (0.0, 0.0), r, upper=True)
    return [('line', fl, fr), ('line', fr, tr), ('arc', tr, tl, (0.0, 0.0), True), ('line', tl, fl)]


def plate(link, mat, p0, p1, z, yo0, yo1, r0, r1, side, t=T_PL):
    """A fork side plate from joint p0 to p1 (x), centre plane at y = side*yo0 .. side*yo1, lobes r0, r1."""
    a = (p0, side * yo0, z)
    b = (p1, side * yo1, z)
    d = unit(sub(b, a))
    L = norm(sub(b, a))
    n = unit(cross((0, 0, 1), d))           # horizontal normal, +y-ish
    o = add(a, mul(n, t / 2 * side))
    return (link, mat, o, d, (0.0, 0.0, 1.0), capsule(L, r0, r1), mul(n, -t * side), windows(L, r0, r1))


FLANGE = 0.020        # solid plate kept above and below each window (the bars and covers fix to it)
RIB = 0.018           # web left between two windows


def stadium(s0, s1, h):
    """Horizontal stadium window from s0 to s1, half-height h, in (s, z). Clockwise."""
    a, b = s0 + h, s1 - h
    if b <= a:
        return None
    return [('line', (a, h), (b, h)), ('arc', (b, h), (b, -h), (b, 0.0), False),
            ('line', (b, -h), (a, -h)), ('arc', (a, -h), (a, h), (a, 0.0), False)]


def windows(L, r0, r1):
    """One or two lightening windows between the lobes; the lobes stay solid round the bearings."""
    s0, s1 = r0 + 0.008, L - r1 - 0.008
    h = min(r0, r1) - FLANGE
    if s1 - s0 < 2 * h + 0.01 or h < 0.008:
        return []
    if s1 - s0 > 4 * h + RIB + 0.02:
        mid = (s0 + s1) / 2
        return [w for w in (stadium(s0, mid - RIB / 2, h), stadium(mid + RIB / 2, s1, h)) if w]
    return [w for w in (stadium(s0, s1, h),) if w]


def bars_and_covers(link, p0, p1, z, yo0, yo1, h, s_in=0.05):
    """2020 bars along the inner top/bottom edge of each plate, plus recessed covers."""
    out = []
    xa, xb = p0 + s_in, p1 - s_in
    f = lambda x: yo0 + (yo1 - yo0) * (x - p0) / (p1 - p0)
    for side in (1, -1):
        for zs in (-1,):           # v2: bottom corners only - the cable runs in their inner slot
            ya = side * (f(xa) - T_PL / 2 - GAP - BAR / 2)
            yb = side * (f(xb) - T_PL / 2 - GAP - BAR / 2)
            zc = z + zs * (h - BAR / 2)
            d = unit((xb - xa, yb - ya, 0.0))
            out.append((link, 'bar', (xa, ya, zc), (0.0, 0.0, 1.0), cross((0.0, 0.0, 1.0), d) if False else unit(cross(d, (0.0, 0.0, 1.0))),
                        [('line', (-BAR / 2, -BAR / 2), (BAR / 2, -BAR / 2)), ('line', (BAR / 2, -BAR / 2), (BAR / 2, BAR / 2)),
                         ('line', (BAR / 2, BAR / 2), (-BAR / 2, BAR / 2)), ('line', (-BAR / 2, BAR / 2), (-BAR / 2, -BAR / 2))],
                        (xb - xa, yb - ya, 0.0)))
    for zs in (1, -1):
        zc = z + zs * (h - 0.002) if zs > 0 else z + zs * (h - BAR - 0.001)
        ya, yb = f(xa) - T_PL / 2 - GAP - 0.004, f(xb) - T_PL / 2 - GAP - 0.004
        ol = [('line', (xa, -ya), (xb, -yb)), ('line', (xb, -yb), (xb, yb)), ('line', (xb, yb), (xa, ya)), ('line', (xa, ya), (xa, -ya))]
        out.append((link, 'cover', (0.0, 0.0, zc - (T_CV if zs > 0 else 0.0)), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), ol, (0.0, 0.0, T_CV)))
    return out


def cyl(link, mat, a, b, r):
    return Gm.cylinder(link, mat, a, b, r)


def shell(link, mat, z0, z1, r, t=0.002):
    """A thin-walled drum: an outer cylinder with the inner one taken out (sheet rolled or spun)."""
    outer = Gm.cylinder(link, mat, (0, 0, z0), (0, 0, z1), r)
    inner = circle_outline(r - t)
    return outer + ([inner],)


def collar(link, mat, xa, xb, z, r_in, r_out, t=0.002):
    """An annular housing round a roll motor: a 2 mm shell; the clock-spring cassette lives inside it."""
    outer = Gm.cylinder(link, mat, (xa, 0, z), (xb, 0, z), r_out)
    return outer + ([circle_outline(r_out - t)],)


def build():
    P = []
    z = Z_SH
    # base: a spun shell round J1 on a foot ring; the J1 cassette sits in the annulus round the motor
    P.append(shell('base', 'base', 0.0, 0.010, 0.105, 0.003))
    P.append(cyl('base', 'base', (0, 0, 0.0), (0, 0, 0.004), 0.105))
    P.append(shell('base', 'base', 0.010, 0.118, 0.070))
    P.append(cyl('base', 'actuator', (0, 0, 0.040), (0, 0, 0.104), RS06['d'] / 2))
    # turret: two tower plates holding J2 between them, on a disc on the J1 output
    P.append(cyl('turret', 'base', (0, 0, 0.120), (0, 0, 0.126), 0.072))
    for side in (1, -1):
        y = side * Y_TURRET
        win = [stadium(-0.034, 0.034, 0.018)]
        win = [[(it[0],) + tuple((pt[0], pt[1] - 0.085) if isinstance(pt, tuple) else pt for pt in it[1:]) for it in w] for w in win if w]
        P.append(('turret', 'sheet', (0.0, y + side * T_PL / 2, z), (1, 0, 0), (0, 0, 1), tower(R_LOBE6, 0.130, z - 0.126),
                  (0, -side * T_PL, 0), win))
    P.append(cyl('turret', 'actuator', (0, -RS06['L'] / 2, z), (0, RS06['L'] / 2, z), RS06['d'] / 2))
    P.append(cyl('turret', 'sleeve', (0, -Y_UPPER_PROX - 0.008, z), (0, -Y_TURRET + T_PL / 2, z), 0.011))
    # upper arm
    for side in (1, -1):
        P.append(plate('upper', 'sheet', 0.0, UPPER, z, Y_UPPER_PROX, Y_UPPER_DIST, R_LOBE6, R_LOBE6, side))
    P += bars_and_covers('upper', 0.0, UPPER, z, Y_UPPER_PROX, Y_UPPER_DIST, R_LOBE6 - 0.010)
    P.append(cyl('upper', 'actuator', (UPPER, -RS06['L'] / 2, z), (UPPER, RS06['L'] / 2, z), RS06['d'] / 2))
    P.append(cyl('upper', 'sleeve', (UPPER, -Y_FORE_PROX - 0.008, z), (UPPER, -Y_UPPER_DIST + T_PL / 2, z), 0.011))
    # forearm: straddles the upper arm, narrows to the J5 end plate
    x5 = W - 0.070
    xe = x5 - RS00['L'] - 0.004
    for side in (1, -1):
        P.append(plate('fore', 'sheet', UPPER, xe, z, Y_FORE_PROX, Y_FORE_DIST, R_LOBE6, R_LOBE0 + 0.008, side))
    P += bars_and_covers('fore', UPPER, xe, z, Y_FORE_PROX, Y_FORE_DIST, R_LOBE0)
    P.append(cyl('fore', 'sheet', (xe - 0.003, 0, z), (xe, 0, z), R_LOBE0 + 0.010))
    P.append(cyl('fore', 'actuator', (x5 - RS00['L'], 0, z), (x5, 0, z), RS00['d'] / 2))
    # J5 collar: fixed half on the forearm, turning half on the yoke, cassette inside
    rc_in, rc_out = RS00['d'] / 2 + 0.002, RS00['d'] / 2 + 0.014
    P.append(collar('fore', 'collar', xe, x5 - 0.026, z, rc_in, rc_out))
    P.append(collar('yoke', 'collar_turn', x5 - 0.025, x5 + 0.004, z, rc_in, rc_out))
    # yoke: two plates from the J5 output to J6
    for side in (1, -1):
        P.append(plate('yoke', 'sheet', x5 + 0.004, W, z, Y_YOKE, Y_YOKE, rc_out - 0.004, R_LOBE0, side))
    P.append(cyl('yoke', 'sheet', (x5 + 0.004, 0, z), (x5 + 0.007, 0, z), rc_out))
    P.append(cyl('yoke', 'actuator', (W, -RS00['L'] / 2, z), (W, RS00['L'] / 2, z), RS00['d'] / 2))
    P.append(cyl('yoke', 'sleeve', (W, -Y_WRIST - 0.007, z), (W, -Y_YOKE + T_PL / 2, z), 0.009))
    # wrist: straddles the yoke, carries J7 on the tool axis inside its own collar
    x7 = W + 0.040
    for side in (1, -1):
        P.append(plate('wrist', 'sheet', W, x7 + 0.004, z, Y_WRIST, Y_WRIST, R_LOBE0, rc_out, side))
    P.append(cyl('wrist', 'sheet', (x7 + 0.004, 0, z), (x7 + 0.007, 0, z), rc_out))
    P.append(cyl('wrist', 'actuator', (x7 + 0.007, 0, z), (x7 + 0.007 + RS00['L'], 0, z), RS00['d'] / 2))
    P.append(collar('wrist', 'collar', x7 + 0.007, x7 + 0.007 + RS00['L'] - 0.022, z, rc_in, rc_out))
    xt = x7 + 0.007 + RS00['L']
    P.append(collar('tool', 'collar_turn', xt - 0.021, xt, z, rc_in, rc_out))
    # tool: flange, gripper motor across the tool axis, two tapered jaws
    P.append(cyl('tool', 'sheet', (xt, 0, z), (xt + 0.004, 0, z), rc_out))
    P.append(cyl('tool', 'actuator', (xt + 0.004 + EDU['d'] / 2, -EDU['L'] / 2, z), (xt + 0.004 + EDU['d'] / 2, EDU['L'] / 2, z), EDU['d'] / 2))
    for dy in (-0.010, 0.010):
        jaw = [('line', (0.0, -0.016), (0.075, -0.006)), ('line', (0.075, -0.006), (0.075, 0.006)),
               ('line', (0.075, 0.006), (0.0, 0.016)), ('line', (0.0, 0.016), (0.0, -0.016))]
        P.append(('tool', 'sheet', (xt + 0.004 + EDU['d'], dy - 0.003, z), (1, 0, 0), (0, 0, 1), jaw, (0, 0.006, 0)))
    return P


# ---------------------------------------------------------------- cable (bus + camera, one bundle)
R_CABLE = 0.004


def cable_points(T):
    """Centreline of the harness at pose T, base to gripper. Pitch joints are crossed ON the axis
    (through the sleeve); roll/yaw joints through their cassettes."""
    z = Z_SH
    pts = []
    def P(link, p):
        pts.append(xf(T, link, p))
    # base: in at the side, up the J1 axis into the base cassette, up through the turret disc
    P('base', (-0.110, 0.0, 0.020))
    P('base', (-0.050, 0.0, 0.020))
    P('base', (0.0, 0.0, 0.020))
    P('turret', (0.0, 0.0, 0.130))
    # up inside the turret (between the tower plates, behind the motor) to the J2 axis, out along -y through the sleeve
    P('turret', (-0.050, -Y_TURRET + 0.010, 0.150))
    P('turret', (0.0, -Y_TURRET + 0.010, z))
    P('turret', (0.0, -Y_UPPER_PROX - 0.010, z))
    # J2 crossed on axis. Into the upper arm, along the lower bar, to the elbow axis
    P('upper', (0.0, -Y_UPPER_PROX - 0.010, z))
    P('upper', (0.040, -Y_UPPER_PROX + 0.016, z - 0.020))
    P('upper', (UPPER - 0.040, -Y_UPPER_DIST + 0.016, z - 0.020))
    P('upper', (UPPER, -Y_UPPER_DIST + 0.010, z))
    P('upper', (UPPER, -Y_FORE_PROX - 0.010, z))
    # J4 on axis. Forearm, to the J5 end plate
    P('fore', (UPPER, -Y_FORE_PROX - 0.010, z))
    P('fore', (UPPER + 0.040, -Y_FORE_PROX + 0.016, z - 0.020))
    x5 = W - 0.070
    xe = x5 - RS00['L'] - 0.004
    rc = RS00['d'] / 2 + 0.008              # inside the collar annulus
    P('fore', (xe - 0.030, -Y_FORE_DIST + 0.014, z - 0.012))
    P('fore', (xe - 0.008, 0.0, z - rc))
    P('fore', (x5 - 0.026, 0.0, z - rc))
    # J5: the clock spring in the collar takes the roll. Into the yoke, up to the J6 axis on -y
    P('yoke', (x5 - 0.024, 0.0, z - rc))
    P('yoke', (x5 + 0.010, 0.0, z - rc + 0.004))
    P('yoke', (W - 0.034, -Y_YOKE + 0.008, z - 0.014))
    P('yoke', (W, -Y_YOKE + 0.008, z))
    P('yoke', (W, -Y_WRIST - 0.007, z))
    # J6 on axis. Wrist plates, into the J7 collar
    P('wrist', (W, -Y_WRIST - 0.007, z))
    P('wrist', (W + 0.024, -Y_WRIST + 0.008, z - 0.016))
    x7 = W + 0.040
    P('wrist', (x7 + 0.010, 0.0, z - rc))
    xt = x7 + 0.007 + RS00['L']
    P('wrist', (xt - 0.022, 0.0, z - rc))
    P('tool', (xt - 0.020, 0.0, z - rc))
    P('tool', (xt + 0.004 + EDU['d'] / 2, -EDU['L'] / 2 - 0.004, z - 0.010))
    return pts


def cable_prims(T):
    pts = cable_points(T)
    out = []
    for a, b in zip(pts, pts[1:]):
        if norm(sub(b, a)) > 1e-4:
            out.append(Gm.cylinder('x', 'cable', a, b, R_CABLE))
    return out, pts


MATERIALS = ('[material "actuator" 0.09 0.09 0.10 0.6 0.5]\n[material "sheet" 0.84 0.85 0.86 0.9 0.32]\n'
             '[material "bar" 0.62 0.64 0.67 0.95 0.45]\n[material "cover" 0.30 0.31 0.33 0.8 0.5]\n'
             '[material "base" 0.20 0.21 0.22 0.85 0.45]\n[material "sleeve" 0.90 0.55 0.15 0.9 0.3]\n'
             '[material "cassette" 0.90 0.55 0.15 0.9 0.3]\n[material "collar" 0.20 0.21 0.22 0.85 0.45]\n[material "collar_turn" 0.84 0.85 0.86 0.9 0.32]\n[material "cable" 0.95 0.40 0.10 0.0 0.6]\n')


def world_loon(q):
    T = fk(q)
    L = ['; rose arm - fork concept v1. GENERATED by arm/fork/rose_fork.py', MATERIALS]
    roots = []
    for i, p in enumerate(build()):
        link, mat, o, u, v, ol, vec = p[:7]
        holes = p[7] if len(p) > 7 else []
        O, U, V, VEC = xf(T, link, o), xd(T, link, u), xd(T, link, v), xd(T, link, vec)
        body = prim_loon((link, mat, O, U, V, ol, VEC))
        for h in holes:
            # the cutter: same sketch frame, started 1 mm behind the plate and 2 mm longer than it
            n = unit(VEC)
            cut = prim_loon((link, mat, sub(O, mul(n, 0.001)), U, V, h, add(VEC, mul(n, 0.002))))
            body = '[difference %s %s]' % (cut, body)
        L.append('[let p%d %s]' % (i, body))
        roots.append('[root p%d "%s"]' % (i, mat))
    cp, pts = cable_prims(T)
    for i, p in enumerate(cp):
        L.append('[let c%d %s]' % (i, prim_loon(p)))
        roots.append('[root c%d "cable"]' % i)
    for i, p in enumerate(pts):
        L.append('[let k%d [translate %.3f %.3f %.3f [sphere %.3f]]]' % (i, p[0] * 1e3, p[1] * 1e3, p[2] * 1e3, R_CABLE * 1e3))
        roots.append('[root k%d "cable"]' % i)
    return '\n'.join(L + [''] + roots) + '\n'


# ---------------------------------------------------------------- numbers
def area2d(ol, step=0.002):
    """True area of a closed 2D outline (lines and arcs), by sampling it."""
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
    a = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        a += x1 * y2 - x2 * y1
    return abs(a) / 2


def prim_mass(p):
    """Mass of one primitive (windows subtracted when the 8th element lists them). Actuators: None."""
    link, mat, o, u, v, ol, vec = p[:7]
    if mat == 'actuator':
        return None
    if mat == 'bar':
        return 0.44 * norm(vec)
    rho = {'sheet': 2680, 'cover': 2680, 'base': 2680, 'sleeve': 7800, 'cassette': 1200,
           'collar': 2680, 'collar_turn': 2680}.get(mat, 0.0)
    area = area2d(ol) - sum(area2d(h) for h in (p[7] if len(p) > 7 else []))
    m = rho * area * norm(vec)
    if mat == 'cover':
        m *= 0.6           # covers get their own windows when cut
    return m


def summary():
    by = {}
    for p in build():
        m = prim_mass(p)
        if m is None:
            continue
        by[p[0]] = by.get(p[0], 0.0) + m
    act = 3 * RS06['kg'] + 3 * RS00['kg'] + EDU['kg']
    struct = sum(by.values())
    print('fork concept v1, 6 DOF')
    print('  reach shoulder -> flange  %.0f mm' % ((W + 0.040 + 0.008 + RS00['L'] + 0.004 + EDU['L'] + 0.060) * 1e3))
    print('  widths: turret %.0f, upper %.0f -> %.0f, forearm %.0f -> %.0f, wrist %.0f mm (outer)' % (
        2e3 * (Y_TURRET + T_PL / 2), 2e3 * (Y_UPPER_PROX + T_PL / 2), 2e3 * (Y_UPPER_DIST + T_PL / 2),
        2e3 * (Y_FORE_PROX + T_PL / 2), 2e3 * (Y_FORE_DIST + T_PL / 2), 2e3 * (Y_WRIST + T_PL / 2)))
    print('  structure by link: ' + ', '.join('%s %.2f' % (k, v) for k, v in by.items()))
    print('  actuators %.2f kg (RS06 x3, RS00 x3, EduLite)   structure %.2f kg   total ~%.2f kg + harness' % (act, struct, act + struct))


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--summary':
        summary()
        sys.exit()
    q = {}
    if len(sys.argv) > 1:
        q = {k: math.radians(v) for k, v in json.loads(open(sys.argv[1]).read() if os.path.exists(sys.argv[1]) else sys.argv[1]).items()}
    sys.stdout.write(world_loon(q))
