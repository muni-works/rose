"""rose arm, fork concept (v6, 2026-10-13: shoulder motor flipped onto the arm, constant-width upper arm,
roll motors inside the links, flat sheet only, hub caps over the pitch-joint cable crossings).

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
RS03 = dict(d=0.106, L=0.054, kg=0.880)      # manual: 54.1 long, 8x M4 on 98
RS04 = dict(d=0.120, L=0.052, kg=1.420)      # manual: 39 + 13.2 long, 10x M4 on 106
# v5 (2026-10-13): RS03 is not available. Two motor models only: RS04 at base yaw, shoulder and
# elbow; RS00 for the three wrist joints. Shoulder and elbow become the same joint.
J1M, J2M, J4M = RS04, RS04, RS04
RS00 = dict(d=0.061, L=0.066, kg=0.310)
EDU = dict(d=0.046, L=0.044, kg=0.242)
T_PL = 0.003          # fork side plate (v2: 4 -> 3 mm, windowed)
T_CV = 0.0015         # cover
GAP = 0.002           # plate to motor / plate to plate running clearance
BAR = 0.020

Z_SH = 0.200          # shoulder axis height (v6: the yaw motor stands on a flat foot)
UPPER = 0.260         # v5: 280 -> 260
FORE = 0.220          # elbow to wrist centre, v5: 240 -> 220 (flange ~703 mm)
W = UPPER + FORE      # wrist centre x
R_LOBE6 = RS06['d'] / 2 + 0.006      # lobe radius round an RS06 joint
R_LOBE_SH = J2M['d'] / 2 + 0.006     # shoulder lobe
R_LOBE_EL = J4M['d'] / 2 + 0.006     # elbow lobe
R_LOBE0 = RS00['d'] / 2 + 0.006

# plate centre-plane half-widths (y), from the stack-up: motor | gap | parent plate | gap | child plate
def stack(motor_L, layers):
    """half-width of the centre plane of each nested plate layer around a motor of length motor_L."""
    out, y = [], motor_L / 2 + GAP
    for t in layers:
        out.append(y + t / 2)
        y += t + GAP
    return out


# v6 stack-up, per side of a pitch joint (y), motor of length L between the parent's plates:
#   +y (output side):  motor | GAP | parent A | spacer gap | child A
#   -y (back side):    motor | GAP | parent B | CABLE_GAP  | child B | hub cap
# The cable crosses in CABLE_GAP: out of the parent through plate B, radially to the sleeve,
# along the axis inside the sleeve (the only twisting length), out under the hub cap, back in.
SPACER_GAP = 0.003     # v6.1: <= 4 mm everywhere a plate passes a plate (ISO 13857: no finger reach)
CABLE_GAP = 0.003      # v6.1: flat high-flex cable (~3 mm) lies in routed grooves in the B plates
T_PB = 0.004           # B-side plates are 4 mm, with a 2.5 mm deep routed groove for the flat cable


def joint_y(L, min_pa=0.0):
    """(parent A, child A, parent B, child B) centre-plane |y| for a pitch joint with a motor of length L.
    min_pa pushes the holder's plates out (with spacers on the motor) when something must pass inside."""
    pa = max(L / 2 + GAP + T_PL / 2, min_pa)
    ca = pa + T_PL / 2 + SPACER_GAP + T_PL / 2
    pb = max(L / 2 + GAP + T_PB / 2, min_pa)
    cb = pb + T_PB / 2 + CABLE_GAP + T_PB / 2
    return pa, ca, pb, cb


SH_PA, SH_CA, SH_PB, SH_CB = joint_y(J2M['L'])      # shoulder: upper holds J2 (motor flipped onto the arm)
EL_PA, EL_CA, EL_PB, EL_CB = joint_y(J4M['L'])      # elbow: upper holds J4, forearm straddles
Y_FORE_DIST = RS00['d'] / 2 + GAP + T_PL / 2 + 0.0005     # forearm tip: just clears the J5 roll motor inside it
WR_PA, WR_CA, WR_PB, WR_CB = joint_y(RS00['L'], Y_FORE_DIST + T_PL + GAP)   # yoke plates pass outside the forearm tip
Y_UPPER = SH_PA                                     # the upper arm: constant width, same joint at both ends
Y_UPPER_DIST = Y_UPPER
Y_YOKE, Y_WRIST = WR_PA, WR_CA
Y_FORE_PROX = EL_CA

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


MOTOR_CLEAR = 0.010   # bars and covers stop this far outside the motor at either end


def bars_and_covers(link, p0, p1, z, yo0, yo1, h, r0=0.04, r1=0.04):
    """2020 bars along the inner bottom edge of each plate, plus recessed covers. They start and stop
    MOTOR_CLEAR outside the motors at the two ends (radii r0, r1), so a fold never brings a motor
    into a cover."""
    out = []
    xa, xb = p0 + r0 + MOTOR_CLEAR, p1 - r1 - MOTOR_CLEAR
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


def covers(link, xa, xb, z, y_in0, y_in1, h):
    """Flat covers top and bottom between the plates' inner faces (tabbed into slots in the plates).
    No bars: the arm is flat sheet only."""
    out = []
    for zs in (1, -1):
        zc = z + zs * h - (T_CV if zs > 0 else 0.0)
        ol = [('line', (xa, -y_in0[1]), (xb, -y_in1[1])), ('line', (xb, -y_in1[1]), (xb, y_in1[0])),
              ('line', (xb, y_in1[0]), (xa, y_in0[0])), ('line', (xa, y_in0[0]), (xa, -y_in0[1]))]
        out.append((link, 'cover', (0.0, 0.0, zc), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), ol, (0.0, 0.0, T_CV)))
    return out


def cross_plate(link, x, z, ya, yb, h):
    """A motor mount across the link, in the y-z plane: the inline roll motors' stators bolt here."""
    ol = [('line', (-yb, -h), (ya, -h)), ('line', (ya, -h), (ya, h)), ('line', (ya, h), (-yb, h)), ('line', (-yb, h), (-yb, -h))]
    return (link, 'sheet', (x, 0.0, z), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), ol, (0.003, 0.0, 0.0))


def hub(link, x, y_out, z, r=0.020, h=0.012):
    """The hub cap over a pitch joint's sleeve: the cable turns inside it. The joint's one accent."""
    return Gm.cylinder(link, 'sleeve', (x, y_out, z), (x, y_out - h, z), r)


def build():
    P = []
    z = Z_SH
    # base: a flat foot, the yaw RS04 standing on it in a slim collar (the yaw cable cassette)
    P.append(cyl('base', 'base', (0, 0, 0.0), (0, 0, 0.006), 0.115))
    zy0 = 0.006
    P.append(cyl('base', 'actuator', (0, 0, zy0), (0, 0, zy0 + J1M['L']), J1M['d'] / 2))
    P.append(collar('base', 'collar', 0.0, 0.0, 0.0, 0.0, 0.0) if False else
             (lambda c: c)(Gm.cylinder('base', 'collar', (0, 0, zy0), (0, 0, zy0 + J1M['L'] - 0.003), J1M['d'] / 2 + 0.010) + ([circle_outline(J1M['d'] / 2 + 0.008)],)))
    zt = zy0 + J1M['L']
    # turret: a disc on the yaw output and two tower plates that straddle the upper arm at the shoulder
    P.append(cyl('turret', 'base', (0, 0, zt), (0, 0, zt + 0.006), J1M['d'] / 2 + 0.010))
    for side, yo in ((1, SH_CA), (-1, SH_CB)):
        tt = T_PL if side > 0 else T_PB
        win = [stadium(-0.030, 0.030, 0.016)]
        win = [[(it[0],) + tuple((pt[0], pt[1] - 0.095) if isinstance(pt, tuple) else pt for pt in it[1:]) for it in w] for w in win if w]
        P.append(('turret', 'sheet', (0.0, side * (yo + tt / 2), z), (1, 0, 0), (0, 0, 1),
                  tower(R_LOBE_SH, 0.170, z - zt - 0.006), (0, -side * tt, 0), win))
    P.append(hub('turret', 0.0, -SH_CB - T_PB / 2 - 0.0005, z))
    # upper arm: constant width, the same RS04 joint at both ends, both motors held between its plates
    P.append(plate('upper', 'sheet', 0.0, UPPER, z, SH_PA, EL_PA, R_LOBE_SH, R_LOBE_EL, 1))
    P.append(plate('upper', 'sheet', 0.0, UPPER, z, SH_PB, EL_PB, R_LOBE_SH, R_LOBE_EL, -1, t=T_PB))
    P.append(cyl('upper', 'actuator', (0, -J2M['L'] / 2, z), (0, J2M['L'] / 2, z), J2M['d'] / 2))
    P.append(cyl('upper', 'actuator', (UPPER, -J4M['L'] / 2, z), (UPPER, J4M['L'] / 2, z), J4M['d'] / 2))
    yi = Y_UPPER - T_PL / 2
    P += covers('upper', R_LOBE_SH + MOTOR_CLEAR, UPPER - R_LOBE_EL - MOTOR_CLEAR, z, (yi, yi), (yi, yi), R_LOBE_EL - 0.012)
    # forearm: straddles the elbow, runs straight to the J5 roll motor, which sits inside its far end
    x5 = W - 0.070
    r_end = R_LOBE0 + 0.008
    P.append(plate('fore', 'sheet', UPPER, x5 - r_end, z, EL_CA, Y_FORE_DIST, R_LOBE_EL, r_end, 1))
    P.append(plate('fore', 'sheet', UPPER, x5 - r_end, z, EL_CB, Y_FORE_DIST, R_LOBE_EL, r_end, -1, t=T_PB))
    P.append(hub('fore', UPPER, -EL_CB - T_PB / 2 - 0.0005, z))
    xs5 = x5 - RS00['L']
    P.append(cross_plate('fore', xs5 - 0.003, z, Y_FORE_DIST - T_PL / 2, Y_FORE_DIST - T_PL / 2, r_end - 0.006))
    P.append(cyl('fore', 'actuator', (xs5, 0, z), (x5, 0, z), RS00['d'] / 2))
    ya_in = lambda x: EL_CA + (Y_FORE_DIST - EL_CA) * (x - UPPER) / (x5 - r_end - UPPER) - T_PL / 2
    yb_in = lambda x: EL_CB + (Y_FORE_DIST - EL_CB) * (x - UPPER) / (x5 - r_end - UPPER) - T_PL / 2
    xa, xb = UPPER + R_LOBE_EL + MOTOR_CLEAR, xs5 - 0.010
    P += covers('fore', xa, xb, z, (ya_in(xa), yb_in(xa)), (ya_in(xb), yb_in(xb)), r_end - 0.012)
    # yoke: a hub disc on the J5 output and two plates holding J6
    YOKE_HUB = 0.008
    P.append(cyl('yoke', 'sheet', (x5 + 0.002, 0, z), (x5 + 0.002 + YOKE_HUB, 0, z), WR_PA + T_PL))
    r_y0 = 0.020                                       # yoke plates' rear lobe: starts behind the hub, no overhang
    xy0 = x5 + 0.002 + YOKE_HUB + r_y0
    P.append(plate('yoke', 'sheet', xy0, W, z, WR_PA, WR_PA, r_y0, R_LOBE0, 1))
    P.append(plate('yoke', 'sheet', xy0, W, z, WR_PB, WR_PB, r_y0, R_LOBE0, -1, t=T_PB))
    P.append(cyl('yoke', 'sheet', (x5 + 0.002 + YOKE_HUB, 0, z), (xy0, 0, z), 0.012))      # hub boss up to the plates' root
    P.append(cyl('yoke', 'actuator', (W, -RS00['L'] / 2, z), (W, RS00['L'] / 2, z), RS00['d'] / 2))
    # wrist: straddles J6 and runs on over the J7 roll motor, which sits inside it
    x7 = W + 0.040
    xt = x7 + 0.007 + RS00['L']
    P.append(plate('wrist', 'sheet', W, xt - R_LOBE0, z, WR_CA, WR_CA, R_LOBE0, R_LOBE0, 1))
    P.append(plate('wrist', 'sheet', W, xt - R_LOBE0, z, WR_CB, WR_CB, R_LOBE0, R_LOBE0, -1, t=T_PB))
    P.append(hub('wrist', W, -WR_CB - T_PB / 2 - 0.0005, z, r=0.016, h=0.010))
    P.append(cross_plate('wrist', x7 + 0.004, z, WR_CA - T_PL / 2, WR_CB - T_PL / 2, R_LOBE0 - 0.006))
    P.append(cyl('wrist', 'actuator', (x7 + 0.007, 0, z), (xt, 0, z), RS00['d'] / 2))
    # tool: flange on the J7 output, the gripper motor across it, two tapered jaws
    P.append(cyl('tool', 'sheet', (xt, 0, z), (xt + 0.004, 0, z), R_LOBE0 - 0.004))
    P.append(cyl('tool', 'actuator', (xt + 0.004 + EDU['d'] / 2, -EDU['L'] / 2, z), (xt + 0.004 + EDU['d'] / 2, EDU['L'] / 2, z), EDU['d'] / 2))
    for dy in (-0.010, 0.010):
        jaw = [('line', (0.0, -0.016), (0.075, -0.006)), ('line', (0.075, -0.006), (0.075, 0.006)),
               ('line', (0.075, 0.006), (0.0, 0.016)), ('line', (0.0, 0.016), (0.0, -0.016))]
        P.append(('tool', 'sheet', (xt + 0.004 + EDU['d'], dy - 0.003, z), (1, 0, 0), (0, 0, 1), jaw, (0, 0.006, 0)))
    return P


# ---------------------------------------------------------------- cable (bus + camera, one bundle)
R_CABLE = 0.0015       # v6.1: flat high-flex cable, ~3 mm thick


def cable_points(T):
    """Harness centreline, base to gripper. Inside links it runs along the centreline (seen through the
    windows). At each pitch joint: out through parent plate B, in the CABLE_GAP to the sleeve, along the
    axis (twist only), under the hub cap, back in through child plate B. Roll joints: inside the link
    ends, through their cassettes."""
    z = Z_SH
    pts = []
    def P(link, p):
        pts.append(xf(T, link, p))
    D = 2 * R_CABLE

    def crossing(holder, straddler, x, pb, cb, r_h, r_s, d_h, d_s, from_holder):
        """One pitch joint. The holder carries the motor between its plates (plate B at -pb); the
        straddler's plate B is at -cb, outside it. In the CABLE_GAP between them each leg lies flat
        on its own plate; they meet on the axis inside the sleeve and turn under the hub cap - the
        only length that twists. The holder's leg leaves through its plate just outside the motor
        (r_h), under the straddler's lobe, so it is covered; the straddler's leg enters its own link
        beyond the holder's lobe (r_s), on the inside of its own plate. d_h, d_s: (dx, dz) directions."""
        y_h = -(pb + T_PL / 2 + R_CABLE + 0.001)
        y_s = -(cb - T_PL / 2 - R_CABLE - 0.001)
        y_cap = -(cb + T_PL / 2 + 0.005)
        at = lambda r, d: (x + d[0] * r, z + d[1] * r)
        hx, hz = at(r_h, d_h)
        sx, sz = at(r_s, d_s)
        cx, cz = at(0.014, d_s)
        seq = [(holder, (hx, -pb + T_PL + R_CABLE, hz)), (holder, (hx, y_h, hz)), (holder, (x, y_h, z)),
               (straddler, (x, y_cap, z)), (straddler, (cx, y_cap, cz)), (straddler, (cx, y_s, cz)),
               (straddler, (sx, y_s, sz)), (straddler, (sx, -cb + T_PL + R_CABLE, sz))]
        if not from_holder:
            seq = seq[::-1]
        for link, pt in seq:
            P(link, pt)

    zt = 0.006 + J1M['L']
    # base: in through the foot, up the yaw collar (cassette), into the turret, up inside its back plate
    P('base', (-0.110, 0.0, 0.003))
    P('base', (-J1M['d'] / 2 - 0.004, 0.0, 0.003))
    P('base', (-J1M['d'] / 2 - 0.004, 0.0, zt - 0.010))
    P('turret', (-0.030, 0.0, zt + 0.012))
    P('turret', (0.0, -SH_CB + T_PL + R_CABLE, zt + 0.012))
    # shoulder: from the turret (straddler, leg coming up from below) into the upper arm (holder)
    crossing('upper', 'turret', 0.0, SH_PB, SH_CB, J2M['d'] / 2 + 0.004, R_LOBE_SH + 0.012, (1, 0), (0, -1), from_holder=False)
    P('upper', (0.090, 0.0, z))
    P('upper', (UPPER - 0.090, 0.0, z))                    # the design line: down the middle of the arm
    crossing('upper', 'fore', UPPER, EL_PB, EL_CB, J4M['d'] / 2 + 0.004, R_LOBE_EL + 0.012, (-1, 0), (1, 0), from_holder=True)
    x5 = W - 0.070
    xs5 = x5 - RS00['L']
    P('fore', (UPPER + 0.090, 0.0, z))
    P('fore', (xs5 - 0.020, 0.0, z))
    P('fore', (xs5 - 0.010, 0.0, z - RS00['d'] / 2 - 0.004))
    P('fore', (x5 - 0.004, 0.0, z - RS00['d'] / 2 - 0.004))
    P('yoke', (x5 + 0.010, 0.0, z - RS00['d'] / 2 - 0.004))
    crossing('yoke', 'wrist', W, WR_PB, WR_CB, RS00['d'] / 2 + 0.004, R_LOBE0 + 0.010, (-1, 0), (1, 0), from_holder=True)
    x7 = W + 0.040
    xt = x7 + 0.007 + RS00['L']
    P('wrist', (x7 + 0.010, 0.0, z - RS00['d'] / 2 - 0.004))
    P('wrist', (xt - 0.004, 0.0, z - RS00['d'] / 2 - 0.004))
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
