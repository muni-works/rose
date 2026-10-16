"""rose arm, fork concept (v8, 2026-10-13: DFM + efficiency - one 3 mm stock for every plate, thin
caps, flat yoke bridge, bigger windows. v7: hybrid wrist RS02 / RS02 / RS00. v6: shoulder motor flipped onto the arm, constant-width upper arm,
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
RS03 = dict(d=0.106, L=0.054, kg=0.880)      # not available - reference only


def motor(d, L, nose_d, nose_L, kg, back, out, front):
    """A RobStride motor as the fork sees it (m). The output face is at the end of a smaller 'nose'
    that stands nose_L proud of the front mounting face. Patterns are (count, bolt-circle D, clearance D).
    v9 (2026-10-13): every motor is BACK-mounted to its holder - the front patterns sit too close to
    the nose for a flat plate to keep a web (RS04: Ø106 bolt circle round a Ø98 nose)."""
    return dict(d=d, L=L, nose_d=nose_d, nose_L=nose_L, kg=kg, back=back, out=out, front=front)


# from the user manuals' drawings + STEP (INTERFACES.md)
RS04 = motor(0.120, 0.0522, 0.098, 0.0132, 1.420, back=(10, 0.106, 0.0045), out=(8, 0.036, 0.0055), front=(10, 0.106, 0.0045))
RS02 = motor(0.0785, 0.0415, 0.065, 0.028, 0.380, back=(4, 0.052, 0.0045), out=(6, 0.024, 0.0045), front=(9, 0.073, 0.0034))
RS00 = motor(0.057, 0.051, 0.035, 0.001, 0.310, back=(4, 0.038, 0.0034), out=(6, 0.027, 0.0034), front=(6, 0.050, 0.0034))
# v5: RS04 at base yaw, shoulder, elbow. v7: hybrid wrist RS02 / RS02 / RS00. Three motor models.
J1M, J2M, J4M = RS04, RS04, RS04
J5M, J6M, J7M = RS02, RS02, RS00
# (peak, stall-continuous) N.m from the manuals; RS02 / RS00 stall scaled from rated by RS06's 8/11 - ESTIMATES
RATING = {id(RS04): (120.0, 25.0), id(RS02): (17.0, 4.4), id(RS00): (14.0, 3.6)}
NAME = {id(RS04): 'RS04', id(RS02): 'RS02', id(RS00): 'RS00'}

# ROSE_ARM=mini: the K1-scale arm (2026-10-16). Same fork, one size down: RS02 where full has RS04,
# RS00 for the whole wrist. Upper / fore match rose mini's humanoid arm (180 / 170 mm).
SIZE = os.environ.get('ROSE_ARM', 'full')
if SIZE == 'mini':
    J1M, J2M, J4M = RS02, RS02, RS02
    J5M, J6M, J7M = RS00, RS00, RS00
if os.environ.get('ROSE_ARM_MOTORS'):      # e.g. RS02,RS02,RS02,RS00,RS00,RS00 for J1,J2,J4,J5,J6,J7
    J1M, J2M, J4M, J5M, J6M, J7M = (globals()[n] for n in os.environ['ROSE_ARM_MOTORS'].split(','))
EDU = dict(d=0.046, L=0.044, kg=0.242)
T_PL = 0.003175       # every plate, cover, disc and bridge: one 1/8" (3.175 mm) 5052 stock - SendCutSend and local supply
T_FOOT = 0.00635      # 1/4" foot
T_CV = T_PL
T_PB = T_PL
GAP = 0.002
BAR = 0.020

Z_SH = 0.200          # shoulder axis height
UPPER = 0.260
FORE = 0.220          # elbow to wrist centre
Z_TURRET = 0.120      # J1 output height
if SIZE == 'mini':
    Z_SH, UPPER, FORE, Z_TURRET = 0.130, 0.180, 0.170, 0.075
# ROSE_ARM_TWIN_SH=1: two J2 motors back to back on a centre web in the upper arm's shoulder lobe,
#   outputs facing out to both turret plates - the fork's bushing side becomes a second drive.
# ROSE_ARM_NO_J5=1: no forearm roll - the J6 holder plates tab straight into the forearm's end plate (5 DOF).
TWIN_SH = os.environ.get('ROSE_ARM_TWIN_SH') == '1'
NO_J5 = os.environ.get('ROSE_ARM_NO_J5') == '1'
if os.environ.get('ROSE_ARM_LEN'):         # upper,fore in mm
    UPPER, FORE = (float(v) / 1e3 for v in os.environ['ROSE_ARM_LEN'].split(','))
JAW = 0.075 if SIZE != 'mini' else 0.050   # jaw length past the gripper motor
W = UPPER + FORE      # wrist centre x
R_LOBE6 = RS06['d'] / 2 + 0.006
R_LOBE_SH = J2M['d'] / 2 + 0.006     # shoulder lobe
R_LOBE_EL = J4M['d'] / 2 + 0.006     # elbow lobe
R_LOBE0 = RS00['d'] / 2 + 0.006
R_LOBE5 = J5M['d'] / 2 + 0.006       # forearm tip round the J5 roll motor
R_LOBE6W = J6M['d'] / 2 + 0.006      # wrist-pitch lobe

# v9 stack-up of a pitch joint along its axis (y). The holder bolts the motor's BACK face to plate B.
# Plate A of the holder is a clearance plate round the nose, its outer face 1 mm short of the output.
# The straddler bolts plate A to the output through a 3 mm spacer disc; plate B rides a bushing on
# the sleeve. Both A-side and B-side plate-to-plate gaps are 4 mm (finger-safe, ISO 13857).
#   +y:  motor nose ... output face | spacer disc 3 | straddler A
#        holder A (around the nose, outer face at output - 1 mm)
#   -y:  back face | holder B | CABLE_GAP 4 | straddler B | hub cap
SPACER_T = T_PL
CABLE_GAP = 0.004
SPACER_GAP = 0.004    # A side: holder A outer face to straddler A inner face (1 + spacer 3)


def joint_y(m):
    """(holder A, straddler A, holder B, straddler B) centre-plane |y|, and (y_out, y_back), for motor m.
    The motor is placed so the holder's two plates sit symmetrically about y = 0."""
    a = (m['L'] + T_PL + 0.001) / 2                  # output face y
    y_back = a - m['L']
    pa = a - 0.001 - T_PL / 2
    ca = a + SPACER_T + T_PL / 2
    pb = -y_back + T_PB / 2
    cb = pb + T_PB / 2 + CABLE_GAP + T_PB / 2
    return pa, ca, pb, cb, a, y_back


SH_PA, SH_CA, SH_PB, SH_CB, SH_OUT, SH_BACK = joint_y(J2M)   # shoulder: upper holds J2, turret straddles
if TWIN_SH:   # centre web T thick on y = 0, each motor's back on one face; the stack is mirror-symmetric
    SH_BACK = T_PL / 2
    SH_OUT = SH_BACK + J2M['L']
    SH_PA = SH_PB = SH_OUT - 0.001 - T_PL / 2
    SH_CA = SH_CB = SH_OUT + SPACER_T + T_PL / 2
EL_PA, EL_CA, EL_PB, EL_CB, EL_OUT, EL_BACK = joint_y(J4M)   # elbow: upper holds J4, forearm straddles
WR_PA, WR_CA, WR_PB, WR_CB, WR_OUT, WR_BACK = joint_y(J6M)   # wrist pitch: yoke holds J6, wrist straddles
Y_FORE_DIST = J5M['d'] / 2 + GAP + T_PL / 2 + 0.0005         # forearm tip encloses the J5 roll motor
Y_UPPER = SH_PA
Y_UPPER_DIST = Y_UPPER
Y_YOKE, Y_WRIST = WR_PA, WR_CA
Y_FORE_PROX = EL_CA

LINKS = [  # name, joint, parent, axis, origin
    ('base', None, None, None, (0.0, 0.0, 0.0)),
    ('turret', 'J1_yaw', 'base', (0, 0, 1), (0.0, 0.0, Z_TURRET)),
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
        a = 0.0 if (NO_J5 and joint == 'J5_fore_roll') else q.get(joint, 0.0)   # J5 is a bolted joint then
        T[name] = (mm_(Rp, rot(axis, a)), t)
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


def plate(link, mat, p0, p1, z, yo0, yo1, r0, r1, side, t=T_PL, keep0=0.0, keep1=0.0, holes=(), hmax=1.0):
    """A fork side plate from joint p0 to p1 (x), centre plane at y = side*yo0 .. side*yo1, lobes r0, r1.
    keep0 / keep1: radius round each lobe centre that the lightening window must stay outside of.
    holes: extra cut-outs in the plate's own (s, z) coordinates - bolt patterns, bores, slots."""
    a = (p0, side * yo0, z)
    b = (p1, side * yo1, z)
    d = unit(sub(b, a))
    L = norm(sub(b, a))
    n = unit(cross((0, 0, 1), d))
    o = add(a, mul(n, t / 2 * side))
    return (link, mat, o, d, (0.0, 0.0, 1.0), capsule(L, r0, r1), mul(n, -t * side),
            windows(L, r0, r1, keep0, keep1, t, hmax) + list(holes))


def circ(cs, cz, r):
    """A round hole at (cs, cz) in a plate's own coordinates."""
    return [('arc', (cs + r, cz), (cs - r, cz), (cs, cz), True), ('arc', (cs - r, cz), (cs + r, cz), (cs, cz), True)]


def rect(cs, cz, ls, lz):
    """A rectangular slot centred at (cs, cz), ls x lz."""
    x0, x1, z0, z1 = cs - ls / 2, cs + ls / 2, cz - lz / 2, cz + lz / 2
    return [('line', (x0, z0), (x1, z0)), ('line', (x1, z0), (x1, z1)), ('line', (x1, z1), (x0, z1)), ('line', (x0, z1), (x0, z0))]


def pattern(cs, cz, pat, phase=None):
    """A motor bolt pattern (count, bolt-circle D, clearance D) centred at (cs, cz)."""
    n, bc, dh = pat
    ph = math.pi / n if phase is None else phase
    return [circ(cs + bc / 2 * math.cos(ph + 2 * math.pi * k / n), cz + bc / 2 * math.sin(ph + 2 * math.pi * k / n), dh / 2)
            for k in range(n)]


SLOT_L, TAB_L = 0.0104, 0.010     # tab-and-slot: 10 mm tabs, slots 0.4 mm long and 0.2 mm wide over


FLANGE = 0.014        # v8: solid plate above and below each window (stress < 10 MPa; >= 1.5 t web rule)
RIB = 0.012           # v8: web left between two windows


def stadium(s0, s1, h):
    """Horizontal stadium window from s0 to s1, half-height h, in (s, z). Clockwise."""
    a, b = s0 + h, s1 - h
    if b <= a:
        return None
    return [('line', (a, h), (b, h)), ('arc', (b, h), (b, -h), (b, 0.0), False),
            ('line', (b, -h), (a, -h)), ('arc', (a, -h), (a, h), (a, 0.0), False)]


def circle_hole(cx, r):
    return [('arc', (cx + r, 0.0), (cx - r, 0.0), (cx, 0.0), True), ('arc', (cx - r, 0.0), (cx + r, 0.0), (cx, 0.0), True)]


def windows(L, r0, r1, keep0=0.0, keep1=0.0, t=T_PL, hmax=1.0):
    """One stadium window between the lobes, outside the keep radius round each lobe centre (bolt
    patterns and bores live inside it). Every web >= max(1.5 t, 4 mm)."""
    web = max(1.5 * t, 0.004)
    s0 = max(r0 * 0.55, keep0) + web + 0.002
    s1 = L - max(r1 * 0.55, keep1) - web - 0.002
    h = min(min(r0, r1) - FLANGE, (s1 - s0) / 2, hmax)
    if h >= 0.008:
        w = stadium(s0, s1, h)
        if w:
            return [w]
    return []


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


def disc(link, mat, x_or_y_or_z, axis, r, t, holes=(), c=(0.0, 0.0)):
    """A flat round 3 mm part normal to `axis` ('x', 'y' or 'z') at that coordinate, centred on the
    arm axis line (x axis at Z_SH for x/y normals, the base axis for z)."""
    ol = [('arc', (c[0] + r, c[1]), (c[0] - r, c[1]), c, True), ('arc', (c[0] - r, c[1]), (c[0] + r, c[1]), c, True)]
    if axis == 'z':
        return (link, mat, (0.0, 0.0, x_or_y_or_z), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), ol, (0.0, 0.0, t), list(holes))
    if axis == 'y':
        return (link, mat, (c[0], x_or_y_or_z, Z_SH), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0),
                [('arc', (r, 0.0), (-r, 0.0), (0.0, 0.0), True), ('arc', (-r, 0.0), (r, 0.0), (0.0, 0.0), True)], (0.0, t, 0.0), list(holes))
    if axis == 'x':
        return (link, mat, (x_or_y_or_z, 0.0, Z_SH), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), ol, (t, 0.0, 0.0), list(holes))
    raise ValueError(axis)


def spacer(link, x, y0, pat):
    """The 3 mm spacer disc a straddler's plate A bolts to the motor output through (normal y)."""
    r = pat[1] / 2 + pat[2] + 0.005
    ol = [('arc', (r, 0.0), (-r, 0.0), (0.0, 0.0), True), ('arc', (-r, 0.0), (r, 0.0), (0.0, 0.0), True)]
    return (link, 'sheet', (x, y0, Z_SH), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), ol, (0.0, SPACER_T, 0.0), pattern(0.0, 0.0, pat))


def motor_prims(link, m, along, at, y_or_x_back, sign=1):
    """Motor body + nose. along: 'y' (pitch joints, output toward +y) or 'x' / 'z' (inline, output toward +)."""
    L, nL = m['L'], m['nose_L']
    b0 = y_or_x_back
    b1 = b0 + sign * (L - nL)
    o1 = b0 + sign * L
    if along == 'y':
        x = at
        return [cyl(link, 'actuator', (x, b0, Z_SH), (x, b1, Z_SH), m['d'] / 2), cyl(link, 'actuator', (x, b1, Z_SH), (x, o1, Z_SH), m['nose_d'] / 2)]
    if along == 'x':
        return [cyl(link, 'actuator', (b0, 0, Z_SH), (b1, 0, Z_SH), m['d'] / 2), cyl(link, 'actuator', (b1, 0, Z_SH), (o1, 0, Z_SH), m['nose_d'] / 2)]
    return [cyl(link, 'actuator', (0, 0, b0), (0, 0, b1), m['d'] / 2), cyl(link, 'actuator', (0, 0, b1), (0, 0, o1), m['nose_d'] / 2)]


def tabbed_rect(x0, x1, y0, y1, tabs_y0, tabs_y1, depth):
    """A rectangle with 10 mm tabs sticking out `depth` past its y0 and y1 edges at the given x's."""
    pts = [(x0, y0)]
    for tx in sorted(tabs_y0):
        pts += [(tx - TAB_L / 2, y0), (tx - TAB_L / 2, y0 - depth), (tx + TAB_L / 2, y0 - depth), (tx + TAB_L / 2, y0)]
    pts += [(x1, y0), (x1, y1)]
    for tx in sorted(tabs_y1, reverse=True):
        pts += [(tx + TAB_L / 2, y1), (tx + TAB_L / 2, y1 + depth), (tx - TAB_L / 2, y1 + depth), (tx - TAB_L / 2, y1)]
    pts += [(x0, y1)]
    return [('line', pts[i], pts[(i + 1) % len(pts)]) for i in range(len(pts))]


def cap(link, x, y_face, r=0.020, h=0.012, t=0.0012):
    """Hub cap over a pitch joint's sleeve: a thin closed cup - a tube and its end disc."""
    tube = Gm.cylinder(link, 'cap', (x, y_face, Z_SH), (x, y_face - h + t, Z_SH), r) + ([circle_outline(r - t)],)
    end = Gm.cylinder(link, 'cap', (x, y_face - h + t, Z_SH), (x, y_face - h, Z_SH), r)
    return [tube, end]


def stations():
    """x of the distal stations along the arm at the zero pose (m)."""
    x5 = W - 0.078                                  # J5 output face
    x_cp = x5 - J5M['L'] - T_PL                     # forearm end plate, J5 back-mounted on its far face
    if NO_J5:                                       # one end plate: forearm plates on one face, J6 holders on the other
        x5 = x_cp = W - R_LOBE6W - 0.020
    x_ep = W + R_LOBE6W + 0.028                     # wrist end plate, J7 back-mounted on its far face
    xt = x_ep + T_PL + J7M['L']                     # J7 output face = tool flange
    return dict(x5=x5, x_cp=x_cp, x_ep=x_ep, xt=xt)


def dplate(link, xc, x_end, z, yc, side, r0, h, tabs_z, holes=(), keep=0.0, t=T_PL):
    """A side plate with a lobe (radius r0) on the joint at x = xc and a flat end at x = x_end, half
    height h, with 10 mm tabs standing T proud of the flat end at tabs_z - they pass through the end
    plate. Centre plane y = side*yc. Holes in the plate's own (s, z), s measured from xc toward x_end."""
    sg = 1.0 if x_end > xc else -1.0
    L = abs(x_end - xc)
    d = math.hypot(L, h)
    ph = math.atan2(h, L) + math.acos(min(1.0, r0 / d))
    tt = (r0 * math.cos(ph), r0 * math.sin(ph))
    tb = (tt[0], -tt[1])
    pts = [(L, -h)]
    for tz in sorted(tabs_z):
        pts += [(L, tz - TAB_L / 2), (L + T_PL, tz - TAB_L / 2), (L + T_PL, tz + TAB_L / 2), (L, tz + TAB_L / 2)]
    pts += [(L, h), tt]
    ol = [('line', pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    ol += [('arc', tt, tb, (0.0, 0.0), True), ('line', tb, (L, -h))]
    o = (xc, side * (yc + t / 2), z)
    return (link, 'sheet', o, (sg, 0.0, 0.0), (0.0, 0.0, 1.0), ol, (0.0, -side * t, 0.0),
            windows(L, r0, h, keep, 0.0, t) + list(holes))


def end_plate(link, x, z, ya, yb, hz, tabs_z, holes, hy_extra=0.0):
    """A cross plate normal to x at x .. x+T, slotted for the side plates' tabs at y = +ya and -yb."""
    web = max(1.5 * T_PL, 0.004)
    ey = (T_PL + 0.0002) / 2 + web + 0.0003 + hy_extra
    y0, y1 = -yb - ey, ya + ey
    ol = [('line', (y0, -hz), (y1, -hz)), ('line', (y1, -hz), (y1, hz)), ('line', (y1, hz), (y0, hz)), ('line', (y0, hz), (y0, -hz))]
    slots = [rect(yy, tz, T_PL + 0.0002, SLOT_L) for yy in (ya, -yb) for tz in tabs_z]
    return (link, 'sheet', (x, 0.0, z), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0), ol, (T_PL, 0.0, 0.0), list(holes) + slots)


def build():
    P = []
    z = Z_SH
    T = T_PL
    # ---- base: 6 mm foot, the yaw RS04 back-mounted on it, printed cassette collar
    foot_holes = pattern(0.0, 0.0, J1M['back']) + [circ(0.0, 0.0, 0.040)] + \
        [circ(0.095 * math.cos(a), 0.095 * math.sin(a), 0.0033) for a in (math.pi / 4, 3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4)]
    P.append(('base', 'base', (0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0),
              [('arc', (0.115, 0.0), (-0.115, 0.0), (0.0, 0.0), True), ('arc', (-0.115, 0.0), (0.115, 0.0), (0.0, 0.0), True)],
              (0.0, 0.0, T_FOOT), foot_holes))
    zb = T_FOOT
    P += motor_prims('base', J1M, 'z', 0.0, zb)
    zo = zb + J1M['L']
    P.append(Gm.cylinder('base', 'printed', (0, 0, zb), (0, 0, zo - J1M['nose_L'] - 0.002), J1M['d'] / 2 + 0.010)
             + ([circle_outline(J1M['d'] / 2 + 0.008)],))
    # ---- turret: disc on the yaw output, two tower plates tabbed into it, straddling the upper arm
    rd = J1M['d'] / 2 + 0.010
    tower_tabs_x = (-0.040, 0.040)
    disc_holes = pattern(0.0, 0.0, J1M['out']) + [circ(0.0, 0.045, 0.012), circ(0.0, -0.045, 0.012)]
    for side, yc in ((1, SH_CA), (-1, SH_CB)):
        for tx in tower_tabs_x:
            disc_holes.append(rect(tx, side * yc, SLOT_L, T + 0.0002))
    P.append(disc('turret', 'base', zo, 'z', rd, T, disc_holes))
    zt = zo + T
    h_t = z - zt
    for side, yc, cb in ((1, SH_CA, False), (-1, SH_CB, True)):
        ol = tower(R_LOBE_SH, 2 * rd, h_t)
        # tabs down into the disc
        fl, fr = (-rd, -h_t), (rd, -h_t)
        tabs = []
        for tx in tower_tabs_x:
            tabs += [(tx - TAB_L / 2, -h_t), (tx - TAB_L / 2, -h_t - T), (tx + TAB_L / 2, -h_t - T), (tx + TAB_L / 2, -h_t)]
        pts = [fl] + tabs + [fr]
        ol = [('line', pts[i], pts[i + 1]) for i in range(len(pts) - 1)] + ol[1:]
        holes = [stadium(-0.030, 0.030, 0.016)]
        holes = [[(it[0],) + tuple((pt[0], pt[1] - 0.105) if isinstance(pt, tuple) else pt for pt in it[1:]) for it in w] for w in holes]
        holes += [circ(0.0, 0.0, 0.009)] if (cb and not TWIN_SH) else pattern(0.0, 0.0, J2M['out'])
        P.append(('turret', 'sheet', (0.0, side * (yc + T / 2), z), (1, 0, 0), (0, 0, 1), ol, (0, -side * T, 0), holes))
    P.append(spacer('turret', 0.0, SH_OUT, J2M['out']))
    if TWIN_SH:
        P.append(spacer('turret', 0.0, -SH_OUT - SPACER_T, J2M['out']))
    else:
        P += cap('turret', 0.0, -SH_CB - T / 2)
    # ---- upper arm: constant width; the same back-mounted RS04 joint at both ends
    h_cv = R_LOBE_EL - 0.012
    xa, xb = R_LOBE_SH + MOTOR_CLEAR, UPPER - R_LOBE_EL - MOTOR_CLEAR
    cover_tabs = (xa + 0.020, xb - 0.020)
    yi = SH_PA - T / 2
    slots = []
    for zc in (h_cv - T / 2, -h_cv + T / 2):
        for tx in cover_tabs:
            slots.append(rect(tx, zc, SLOT_L, T + 0.0002))
    nose_bore = J2M['nose_d'] / 2 + 0.002
    back_keep = J2M['back'][1] / 2 + J2M['back'][2]
    A_holes = [circ(0.0, 0.0, nose_bore), circ(UPPER, 0.0, nose_bore)] + slots
    B_holes = [circ(0.0, 0.0, 0.045), circ(UPPER, 0.0, 0.045)] + pattern(0.0, 0.0, J2M['back']) + pattern(UPPER, 0.0, J4M['back']) + slots
    P.append(plate('upper', 'sheet', 0.0, UPPER, z, SH_PA, EL_PA, R_LOBE_SH, R_LOBE_EL, 1, keep0=nose_bore, keep1=nose_bore, holes=A_holes, hmax=h_cv - T - 0.0062))
    P.append(plate('upper', 'sheet', 0.0, UPPER, z, SH_PB, EL_PB, R_LOBE_SH, R_LOBE_EL, -1, t=T_PB, keep0=back_keep, keep1=back_keep, holes=B_holes, hmax=h_cv - T - 0.0062))
    for zs in (1, -1):
        zc = z + zs * h_cv - (T if zs > 0 else 0.0)
        ol = tabbed_rect(xa, xb, -yi, yi, cover_tabs, cover_tabs, T)
        P.append(('upper', 'cover', (0.0, 0.0, zc), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0), ol, (0.0, 0.0, T)))
    P += motor_prims('upper', J2M, 'y', 0.0, SH_BACK)
    if TWIN_SH:   # second motor mirrored, and the centre web both backs bolt to (tabbed into the covers)
        P += motor_prims('upper', J2M, 'y', 0.0, -SH_BACK, sign=-1)
        P.append(disc('upper', 'sheet', -T / 2, 'y', R_LOBE_SH - 0.004, T,
                      pattern(0.0, 0.0, J2M['back']) + [circ(0.0, 0.0, 0.012)]))
    P += motor_prims('upper', J4M, 'y', UPPER, EL_BACK)
    # ---- forearm: parallel plates straddling the elbow, flat ends tabbed through an end plate;
    #      the J5 roll RS02 back-mounts on the end plate's far face, a cartridge like the tool roll
    S = stations()
    x5, x_cp, x_ep, xt = S['x5'], S['x_cp'], S['x_ep'], S['xt']
    h_f, tabs_f = 0.036, (-0.020, 0.020)
    for side, yc, bush in ((1, EL_CA, False), (-1, EL_CB, True)):
        holes = [circ(0.0, 0.0, 0.009)] if bush else pattern(0.0, 0.0, J4M['out'])
        P.append(dplate('fore', UPPER, x_cp, z, yc, side, R_LOBE_EL, h_f, tabs_f, holes, keep=0.030))
    if NO_J5:
        P.append(end_plate('fore', x_cp, z, EL_CA, EL_CB, h_f, tabs_f, [circ(0.0, 0.0, 0.012)]
                           + [rect(yy, tz, T_PL + 0.0002, SLOT_L) for yy in (WR_PA, -WR_PB) for tz in (-0.010, 0.010)]))
    else:
        P.append(end_plate('fore', x_cp, z, EL_CA, EL_CB, h_f, tabs_f, pattern(0.0, 0.0, J5M['back'], math.pi / 4) + [circ(0.0, 0.0, 0.018)]))
        P += motor_prims('fore', J5M, 'x', 0.0, x_cp + T)
    P.append(spacer('fore', UPPER, EL_OUT, J4M['out']))
    P += cap('fore', UPPER, -EL_CB - T / 2)
    # ---- yoke: a bridge on the J5 output; two holder plates for J6 tab through it
    h_y, tabs_y = 0.022, (-0.010, 0.010)
    nose6 = J6M['nose_d'] / 2 + 0.002
    keep6b = J6M['back'][1] / 2 + J6M['back'][2]
    if not NO_J5:
        P.append(end_plate('yoke', x5, z, WR_PA, WR_PB, 0.0205, tabs_y, pattern(0.0, 0.0, J5M['out'], math.pi / 6)))
    P.append(dplate('yoke', W, x5 + T, z, WR_PA, 1, R_LOBE6W, h_y, tabs_y, [circ(0.0, 0.0, nose6)], keep=nose6))
    P.append(dplate('yoke', W, x5 + T, z, WR_PB, -1, R_LOBE6W, h_y, tabs_y,
                    [circ(0.0, 0.0, 0.018)] + pattern(0.0, 0.0, J6M['back']), keep=keep6b))
    P += motor_prims('yoke', J6M, 'y', W, WR_BACK)
    # ---- wrist: straddles J6; flat ends tabbed through the end plate that carries the J7 cartridge
    h_w, tabs_w = 0.026, (-0.010, 0.010)
    for side, yc, bush in ((1, WR_CA, False), (-1, WR_CB, True)):
        holes = [circ(0.0, 0.0, 0.006)] if bush else pattern(0.0, 0.0, J6M['out'])
        P.append(dplate('wrist', W, x_ep, z, yc, side, R_LOBE6W, h_w, tabs_w, holes, keep=0.018))
    P.append(end_plate('wrist', x_ep, z, WR_CA, WR_CB, h_w, tabs_w, pattern(0.0, 0.0, J7M['back'], math.pi / 4) + [circ(0.0, 0.0, 0.010)]))
    P += motor_prims('wrist', J7M, 'x', 0.0, x_ep + T)
    P.append(spacer('wrist', W, WR_OUT, J6M['out']))
    P += cap('wrist', W, -WR_CB - T / 2, r=0.016, h=0.010)
    # ---- tool: flange on the J7 output, the gripper motor across it, two tapered jaws
    P.append(disc('tool', 'sheet', xt, 'x', J7M['d'] / 2 + 0.002, T, pattern(0.0, 0.0, J7M['out'], 0.0)))
    P.append(cyl('tool', 'actuator', (xt + T + EDU['d'] / 2, -EDU['L'] / 2, z), (xt + T + EDU['d'] / 2, EDU['L'] / 2, z), EDU['d'] / 2))
    for dy in (-0.010, 0.010):
        jaw = [('line', (0.0, -0.016), (JAW, -0.006)), ('line', (JAW, -0.006), (JAW, 0.006)),
               ('line', (JAW, 0.006), (0.0, 0.016)), ('line', (0.0, 0.016), (0.0, -0.016))]
        P.append(('tool', 'sheet', (xt + T + EDU['d'], dy - T / 2, z), (1, 0, 0), (0, 0, 1), jaw, (0, T, 0)))
    return P


def flange_x():
    """Shoulder axis to the jaw tips (m)."""
    return stations()['xt'] + T_PL + EDU['d'] + JAW


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
    S = stations()
    x5, x_cp, x_ep, xt = S['x5'], S['x_cp'], S['x_ep'], S['xt']
    P('fore', (UPPER + 0.080, 0.0, z))
    P('fore', (x_cp - 0.012, 0.0, z))
    P('fore', (x_cp - 0.006, 0.0, z - J5M['d'] / 2 - 0.006))          # out under the end plate
    P('fore', (x5 - 0.004, 0.0, z - J5M['d'] / 2 - 0.006))            # along under the J5 cartridge
    P('yoke', (x5 + 0.010, 0.0, z - J5M['d'] / 2 - 0.006))
    crossing('yoke', 'wrist', W, WR_PB, WR_CB, J6M['d'] / 2 + 0.004, R_LOBE6W + 0.010, (-1, 0), (1, 0), from_holder=True)
    P('wrist', (x_ep - 0.008, 0.0, z - J7M['d'] / 2 - 0.006))
    P('wrist', (xt - 0.004, 0.0, z - J7M['d'] / 2 - 0.006))           # along under the J7 cartridge
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
             '[material "base" 0.20 0.21 0.22 0.85 0.45]\n[material "sleeve" 0.90 0.55 0.15 0.9 0.3]\n[material "cap" 0.90 0.55 0.15 0.9 0.3]\n[material "printed" 0.20 0.21 0.22 0.0 0.6]\n'
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
    rho = {'sheet': 2680, 'cover': 2680, 'base': 2680, 'sleeve': 7800, 'cassette': 1200, 'cap': 2680,
           'printed': 1250, 'collar': 2680, 'collar_turn': 2680}.get(mat, 0.0)
    area = area2d(ol) - sum(area2d(h) for h in (p[7] if len(p) > 7 else []))
    m = rho * area * norm(vec)
    if mat == 'cover':
        pass               # v9: covers are plain 3 mm, tabbed - mass as drawn
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
