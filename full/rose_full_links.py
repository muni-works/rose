"""Structural link definitions for rose-full at 1800 mm, and their checks.

STRUCTURAL SYSTEM: cut-and-bent 3 mm 5052 sheet. No angle stock, no machined
blocks. (SIZING.md: at 68 kg the leg channels want 4 mm and the hip output and
knee want machined fittings; the table is still 3 mm until the wraps are re-swept
at the new loads. T is one constant.)

Every coordinate below is DERIVED from rose_full_layout (joint centres, actuator
envelopes) - the 1173 mm table was absolute millimetres tuned by hand around
X6-60 / X4-36 and could not move. The idioms are the same: face plates on
pancake faces (stator plate on the face, output plate outset one thickness),
wrapped channels around the actuator a link carries, lapped pairs where a bend
line would cross a bolt circle, and bend tabs recorded so the bend rule can be
checked.

What the bigger actuators changed (see SIZING.md for why they are bigger):
  * the hip-roll X12 sits FORWARD (body x -70..18) so its rear face is at the
    hip-pitch output plate's rear edge and hipA becomes ONE blank (pitch_face
    bends straight into roll_face: the X15 output ring R50 is 23 mm from the
    line). At 1173 the roll body was centred and the coupling needed a tab it
    never quite had.
  * hipB is ONE blank too: the roll output plate bends at its bottom into the
    yaw stator plate (nearest yaw ring hole 17 mm from the line).
  * the arm hangs at y 255: the hip-roll pods reach y 226 and a hand beside the
    hip is a hand, not a clash. The chest is +/-165 so shoulders stay 1.5 x
    chest and chest:waist lands at 1.70 (H2 1.6).
  * the deck IS the head-yaw stator plate: the yaw body hangs inside the chest
    top and bolts to the deck from below. No risers.

Frame: trunk origin, x fwd, y left, z up, millimetres. Left side only; right is
mirrored about y.
"""
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rose_full_layout as LAY

T = 3.0                     # plate thickness
ANGLE = (0.0, 0.0, 0.0)     # retired: no angle stock in the bent-sheet scheme
MIN_EDGE = lambda d: T + min(d / 2.0, 5.0)
SLOT_R = 8.0
HEM = 8.0

# actuator interfaces, from the vendor 2D drawings (~/Developer/myactuator_assets).
# od, axial length INCLUDING the proud output flange, stator BCD / hole / count,
# output BCD / hole / count, output pilot (protrudes into the output plate's
# bore), spigot (output flange diameter: the stator plate's bore). Hole dia is
# the bolt's clearance size minus the 1.1 face_plate adds.
IFACE = {
    'X15-450': dict(od=166.0, ln=72.0,  sb=157.0, sh=4.4, sn=12, ob=100.0, oh=4.4, on=12, pilot=115.0, spig=144.0),
    'X12-320': dict(od=124.0, ln=88.0,  sb=116.5, sh=3.3, sn=16, ob=72.0,  oh=4.4, on=12, pilot=85.0,  spig=109.0),
    'X10-200': dict(od=124.0, ln=65.5,  sb=116.5, sh=3.3, sn=16, ob=72.0,  oh=4.4, on=12, pilot=85.0,  spig=107.0),
    'X8-120':  dict(od=96.0,  ln=79.0,  sb=88.0,  sh=2.3, sn=10, ob=55.0,  oh=3.3, on=12, pilot=48.0,  spig=64.0),
    'X6-60':   dict(od=80.0,  ln=67.54, sb=72.5,  sh=3.4, sn=12, ob=44.0,  oh=4.5, on=12, pilot=32.0,  spig=53.0),
    'X4-36':   dict(od=55.0,  ln=61.0,  sb=48.0,  sh=3.4, sn=6,  ob=26.0,  oh=3.4, on=6,  pilot=35.0,  spig=35.0),
    'X4-10':   dict(od=55.0,  ln=55.5,  sb=48.0,  sh=3.4, sn=6,  ob=26.0,  oh=3.4, on=6,  pilot=35.0,  spig=35.0),
    # the pushrod ankle's pins: not actuators, but bodies the checks must see
    'PIN':     dict(od=12.0,  ln=50.5,  sb=0, sh=0, sn=0, ob=0, oh=0, on=0, pilot=0, spig=0),
    'PINX':    dict(od=12.0,  ln=135.0, sb=0, sh=0, sn=0, ob=0, oh=0, on=0, pilot=0, spig=0),
}

# ---- layout, in mm ----
mm = lambda v: round(v * 1000.0, 2)
HIP_Y, LEG_Y, SHO_Y = mm(LAY.HIP_Y), mm(LAY.LEG_Y), mm(LAY.SHO_Y)
ZHP, ZHR, ZHY, ZK = mm(LAY.Z_HIP_P), mm(LAY.Z_HIP_R), mm(LAY.Z_HIP_Y), mm(LAY.Z_KNEE)
ZAP, ZAR, ZS = mm(LAY.Z_ANK_P), mm(LAY.Z_ANK_R), mm(LAY.Z_SOLE)
ZMA, ZMB = mm(LAY.Z_ANK_MA), mm(LAY.Z_ANK_MB)
ZSP, ZSR, ZE, ZW = mm(LAY.Z_SHO_P), mm(LAY.Z_SHO_R), mm(LAY.Z_ELB), mm(LAY.Z_WRIST)
ZHEY, ZHEP, ZCROWN = mm(LAY.Z_HEAD_Y), mm(LAY.Z_HEAD_P), mm(LAY.Z_CROWN)
ZWAIST = mm(LAY.Z_WAIST)
KB = mm(LAY.KNEE_BACK)
ROLL_X, SHR_X = mm(LAY.ROLL_X), mm(LAY.SHR_X)

def rad(m): return IFACE[m]['od'] / 2.0
def ln(m):  return IFACE[m]['ln']

# joint: name, centre, axis, model, body direction, mount-face mode
#   'mid'  -> body straddles the joint, mount face at -half along the axis dir
#   'end'  -> body runs from the joint along bdir, mount face AT the joint
JOINTS = {
    'hip_pitch': dict(c=(0, HIP_Y, ZHP),       ax='y', m='X15-450', bd=-1, mode='end'),
    'hip_roll':  dict(c=(ROLL_X, LEG_Y, ZHR),  ax='x', m='X12-320', bd=-1, mode='mid'),
    'hip_yaw':   dict(c=(0, LEG_Y, ZHY),       ax='z', m='X10-200', bd=-1, mode='end'),
    'knee':      dict(c=(-KB, LEG_Y, ZK),      ax='y', m='X15-450', bd=-1, mode='end'),
    'ank_mA':    dict(c=(-KB, LEG_Y + T, ZMA), ax='y', m='X8-120',  bd=-1, mode='end'),
    'ank_mB':    dict(c=(-KB, LEG_Y + T, ZMB), ax='y', m='X8-120',  bd=-1, mode='end'),
    'ank_pitch': dict(c=(-KB, LEG_Y - 4.25, ZAP), ax='y', m='PIN',  bd=-1, mode='mid'),
    'ank_roll':  dict(c=(-KB + 2.5, LEG_Y, ZAR), ax='x', m='PINX', bd=-1, mode='mid'),
    'sho_pitch': dict(c=(0, SHO_Y, ZSP),       ax='y', m='X8-120',  bd=-1, mode='end'),
    'sho_roll':  dict(c=(SHR_X, SHO_Y, ZSR),   ax='x', m='X6-60',   bd=-1, mode='mid'),
    'elb_pitch': dict(c=(0, SHO_Y, ZE),        ax='y', m='X6-60',   bd=+1, mode='end'),
    'wrist_yaw': dict(c=(0, SHO_Y, ZW),        ax='z', m='X4-36',   bd=-1, mode='end'),
    'waist_yaw': dict(c=(0, 0.0, ZWAIST),      ax='z', m='X8-120',  bd=-1, mode='end'),
    'head_yaw':  dict(c=(0, 0.0, ZHEY),        ax='z', m='X4-36',   bd=-1, mode='end'),
    'head_pitch':dict(c=(0, 0.0, ZHEP),        ax='y', m='X4-36',   bd=-1, mode='mid'),
}
AXI = {'x': 0, 'y': 1, 'z': 2}


def body_span(j):
    """(lo, hi) of the actuator body along its own axis, and the mount-face coord."""
    i = AXI[j['ax']]
    c = j['c'][i]
    l = IFACE[j['m']]['ln']
    if j['mode'] == 'mid':
        lo, hi = c - l / 2, c + l / 2
        face = lo if j['bd'] < 0 else hi
    else:
        lo, hi = (c - l, c) if j['bd'] < 0 else (c, c + l)
        face = c
    return lo, hi, face


def obstacles():
    """Actuator bodies as (name, axis, (u,v) centre, radius, (lo,hi) along axis)."""
    out = []
    for n, j in JOINTS.items():
        i = AXI[j['ax']]
        lo, hi, _ = body_span(j)
        uv = tuple(j['c'][k] for k in range(3) if k != i)
        out.append((n, j['ax'], uv, IFACE[j['m']]['od'] / 2.0, (lo, hi)))
    return out


def hit(box, axis, uv, r, ext):
    i = AXI[axis]
    a0, a1 = max(box[i][0], ext[0]), min(box[i][1], ext[1])
    if a1 <= a0:
        return None
    j, k = [n for n in range(3) if n != i]
    d = math.hypot(max(box[j][0] - uv[0], 0, uv[0] - box[j][1]),
                   max(box[k][0] - uv[1], 0, uv[1] - box[k][1]))
    return None if d >= r else (a1 - a0, r - d)


def _rot_axis(axis, deg):
    """3x3 rotation about a world axis ('x'|'y'|'z'), degrees."""
    c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    if axis == 'x':
        return ((1, 0, 0), (0, c, -s_), (0, s_, c))
    if axis == 'y':
        return ((c, 0, s_), (0, 1, 0), (-s_, 0, c))
    return ((c, -s_, 0), (s_, c, 0), (0, 0, 1))


def _apply(R, c, p):
    d = [p[i] - c[i] for i in range(3)]
    return tuple(c[i] + sum(R[i][k] * d[k] for k in range(3)) for i in range(3))


def poly_sdf(pt, poly):
    """Signed distance from an in-plane point to a CONVEX polygon (CCW), <0 inside."""
    n = len(poly)
    inside = -1e9
    outside = 1e9
    for i in range(n):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        L2 = ex * ex + ey * ey or 1e-9
        # outward normal for CCW is (ey, -ex)
        nl = math.sqrt(L2)
        sdist = ((pt[0] - ax) * ey - (pt[1] - ay) * ex) / nl
        inside = max(inside, sdist)
        t = max(0.0, min(1.0, ((pt[0] - ax) * ex + (pt[1] - ay) * ey) / L2))
        outside = min(outside, math.hypot(pt[0] - ax - t * ex, pt[1] - ay - t * ey))
    return inside if inside <= 0 else outside


def poly_inset(poly, r):
    """Inset a CONVEX CCW polygon by r (edge-offset); the rounded outline is its
    Minkowski sum with a disc of r, so sdf_rounded = sdf(inset) - r."""
    n = len(poly)
    lines = []
    for i in range(n):
        (ax, ay), (bx, by) = poly[i], poly[(i + 1) % n]
        ex, ey = bx - ax, by - ay
        nl = math.hypot(ex, ey) or 1e-9
        nx, ny = ey / nl, -ex / nl          # outward
        lines.append((nx, ny, nx * ax + ny * ay - r))    # n.p = d - r
    out = []
    for i in range(n):
        (a1, b1, c1), (a2, b2, c2) = lines[i - 1], lines[i]
        det = a1 * b2 - a2 * b1
        if abs(det) < 1e-12:
            out.append(poly[i])
            continue
        out.append(((c1 * b2 - c2 * b1) / det, (a1 * c2 - a2 * c1) / det))
    return out


class Plate:
    """A flat plate: normal axis, thickness span, and an in-plane outline.

    The outline is a rectangle u0..u1 x v0..v1 with rounded corners, or - if
    `poly` is given - a CONVEX polygon (list of (u, v), CCW) with every vertex
    rounded by the same radius; u0..u1/v0..v1 are then its bounding box. A
    plate may be TILTED: `tilt = (axis, deg, pivot_xyz)` rotates the finished
    plate about the world axis through `pivot`. Every check that used to read
    the rectangle now asks `sdf_uv` / `to_world` / `to_local`, so a tilted
    trapezoid is checked exactly, not as a box.

    `holes` are round bores (u, v, dia). `slots` are rectangular APERTURES
    (u0, u1, v0, v1) with SLOT_R corners. A plate whose outline is a slot plus
    a narrow rim is a hem frame: the 180-degree fold around a window edge,
    modelled as the 3 mm doubled strip it becomes (see cobot.py rule 2).
    """

    def __init__(self, link, tag, normal, n0, n1, u0, u1, v0, v1, holes=(), slots=(),
                 r_want=20.0, poly=None, tilt=None):
        self.link, self.tag, self.normal = link, tag, normal
        self.r_want = r_want          # corner radius asked for (rose_full_loon.corner_radius backs off)
        self.n0, self.n1 = n0, n1
        self.u0, self.u1, self.v0, self.v1 = u0, u1, v0, v1
        self.holes = list(holes)      # (u, v, dia)
        self.slots = list(slots)      # (u0, u1, v0, v1)
        self.poly = [tuple(p) for p in poly] if poly else None
        self.tilt = tilt              # (axis, deg, (px, py, pz)) or None
        if self.poly:
            us = [p[0] for p in self.poly]
            vs = [p[1] for p in self.poly]
            self.u0, self.u1, self.v0, self.v1 = min(us), max(us), min(vs), max(vs)

    # ---- frames ----
    def axes(self):
        i = AXI[self.normal]
        ui, vi = [k for k in range(3) if k != i]
        return i, ui, vi

    def local_to_world(self, n, u, v):
        """(n, u, v) in the plate's own frame -> world xyz (applies the tilt)."""
        i, ui, vi = self.axes()
        p = [0.0, 0.0, 0.0]
        p[i], p[ui], p[vi] = n, u, v
        if self.tilt:
            ax, deg, piv = self.tilt
            return _apply(_rot_axis(ax, deg), piv, p)
        return tuple(p)

    def world_to_local(self, p):
        """world xyz -> (n, u, v) in the plate's own frame (undoes the tilt)."""
        if self.tilt:
            ax, deg, piv = self.tilt
            p = _apply(_rot_axis(ax, -deg), piv, p)
        i, ui, vi = self.axes()
        return p[i], p[ui], p[vi]

    def outline(self):
        """The in-plane outline as a CCW polygon (rect or poly), unrounded."""
        if self.poly:
            return list(self.poly)
        return [(self.u0, self.v0), (self.u1, self.v0), (self.u1, self.v1), (self.u0, self.v1)]

    def sdf_uv(self, u, v, r):
        """Signed distance to the ROUNDED outline in-plane (<0 inside), corner radius r."""
        if self.poly:
            return poly_sdf((u, v), poly_inset(self.poly, r)) - r
        ax, ay = (self.u1 - self.u0) / 2.0, (self.v1 - self.v0) / 2.0
        cx, cy = (self.u0 + self.u1) / 2.0, (self.v0 + self.v1) / 2.0
        qx, qy = abs(u - cx) - ax + r, abs(v - cy) - ay + r
        return min(max(qx, qy), 0.0) + math.hypot(max(qx, 0.0), max(qy, 0.0)) - r

    def box(self):
        """World AABB. Exact for an untilted plate; the rotated prism's AABB otherwise."""
        if not self.tilt:
            b = [None] * 3
            i, ui, vi = self.axes()
            b[i] = (self.n0, self.n1)
            b[ui] = (self.u0, self.u1)
            b[vi] = (self.v0, self.v1)
            return tuple(b)
        pts = [self.local_to_world(n, u, v) for n in (self.n0, self.n1)
               for (u, v) in self.outline()]
        return tuple((min(p[k] for p in pts), max(p[k] for p in pts)) for k in range(3))

    def fit(self):
        """Grow a RECT outline until every hole clears the edge by MIN_EDGE.

        Hand-picked plate extents were repeatedly too small for the bolt circle
        they had to carry, so the outline is derived from the holes instead. A
        polygon outline is not grown; its containment is checked instead."""
        if self.poly:
            return
        for (u, v, d) in self.holes:
            m = MIN_EDGE(d)
            self.u0 = min(self.u0, u - d / 2 - m)
            self.u1 = max(self.u1, u + d / 2 + m)
            self.v0 = min(self.v0, v - d / 2 - m)
            self.v1 = max(self.v1, v + d / 2 + m)

    def area(self):
        """Net area: outline minus windows minus bores (what the blank weighs)."""
        if self.poly:
            a = 0.0
            for i in range(len(self.poly)):
                (x0, y0), (x1, y1) = self.poly[i], self.poly[(i + 1) % len(self.poly)]
                a += x0 * y1 - x1 * y0
            a = abs(a) / 2.0
        else:
            a = (self.u1 - self.u0) * (self.v1 - self.v0)
        a -= sum((s1 - s0) * (t1 - t0) for (s0, s1, t0, t1) in self.slots)
        a -= sum(math.pi * (d / 2.0) ** 2 for (_, _, d) in self.holes)
        return a

    def mass(self):
        return self.area() * T * 2.68e-3 / 1000.0   # kg, 5052


def ring(bcd, cu, cv, dia, n, clock=0.0):
    return [(cu + (bcd / 2) * math.cos(math.radians(i * 360.0 / n + clock)),
             cv + (bcd / 2) * math.sin(math.radians(i * 360.0 / n + clock)), dia)
            for i in range(n)]


def face_plate(link, tag, joint, role, u0, u1, v0, v1, outset=0.0):
    """Plate bolted flat to a joint's stator or output face."""
    j = JOINTS[joint]
    i = AXI[j['ax']]
    _, _, face = body_span(j)
    ifc = IFACE[j['m']]
    # The plate sits on the side of the mount face AWAY from the actuator body.
    blo, bhi, _ = body_span(j)
    if face < (blo + bhi) / 2.0:          # body sits above the face -> plate below
        n0, n1 = face - T - outset, face - outset
    else:                                  # body below -> plate above
        n0, n1 = face + outset, face + T + outset
    ui, vi = [k for k in range(3) if k != i]
    cu, cv = j['c'][ui], j['c'][vi]
    bcd = ifc['sb'] if role == 'stator' else ifc['ob']
    dia = ifc['sh'] if role == 'stator' else ifc['oh']
    n = ifc['sn'] if role == 'stator' else ifc['on']
    holes = ring(bcd, cu, cv, dia + 1.1, n, clock=180.0 / n)
    bore = ifc['spig'] if role == 'stator' else ifc['pilot']
    holes.append((cu, cv, bore + 1.0))
    pl = Plate(link, tag, j['ax'], n0, n1, u0, u1, v0, v1, holes)
    pl.fit()
    return pl


PLATES = []


def L(link, tag, joint, role, u0, u1, v0, v1, outset=0.0):
    PLATES.append(face_plate(link, tag, joint, role, u0, u1, v0, v1, outset))
    return PLATES[-1]


def P(*a, **k):
    PLATES.append(Plate(*a, **k))
    return PLATES[-1]


def face_n(joint, role, outset=0.0):
    """Normal-axis span (n0, n1) a face plate gets - for placing siblings."""
    j = JOINTS[joint]
    blo, bhi, face = body_span(j)
    if face < (blo + bhi) / 2.0:
        return face - T - outset, face - outset
    return face + outset, face + T + outset


# ------------------------------------------------------------------ links ----
# ---- hipA: hip pitch OUTPUT -> hip roll STATOR. One blank, one bend. ----
# pitch_face is the X15 output plate (y 97..100); its rear edge at x = -73 is the
# roll body's rear face less one thickness, and bends 90 deg into roll_face.
RF_X0 = ROLL_X - ln('X12-320') / 2.0 - T          # roll stator plate x0 = -73
pf = L('hipA', 'pitch_face', 'hip_pitch', 'output', RF_X0, 70.0, ZHP - 70.0, ZHP + 70.0, outset=T)
rf = L('hipA', 'roll_face',  'hip_roll',  'stator', HIP_Y + 2 * T, LEG_Y + 66.0, ZHR - 66.0, ZHR + 66.0)

YO_OUT = 3 * T                        # the yaw OUTPUT plate's outset (6 mm spacer ring): see thigh
# ---- hipB: hip roll OUTPUT -> hip yaw STATOR. One blank, THREE bends: a Z-bracket. ----
# The yaw stator plate is sandwiched inside the thigh: the thigh's top plate above
# it, the thigh's flanges around it, the yaw body below. It cannot leave through the
# channel's open back - the flanges sweep +/-57 deg and their rear ends cross the
# back at r 104 (any tongue at x < -62 is hit by 20 deg). What the flanges do NOT
# sweep is the disc r < 81 above the stator plate, and the thigh's top plate leaves
# a strip x < -51 of that uncovered. So: a LEG bends up from the stator plate's rear
# edge at x -69 (r 67, 10.5 from the nearest ring hole), a BRIDGE bends back off it
# 5 over the thigh's channel-top plate and 4 under the roll body, and the roll
# output plate bends up off the bridge at x -114. Every number is a sweep.
YF_X0 = -71.0
yf = L('hipB', 'yaw_face', 'hip_yaw', 'stator', YF_X0, 66.0, LEG_Y - 66.0, LEG_Y + 66.0)
yf.r_want = 35.0                      # a 132 square with r20 corners reaches r 93; the thigh flanges sweep r 85.5
RO_X0 = ROLL_X - ln('X12-320') / 2.0 - 2 * T              # roll output plate x0 (-114)
BR_Z1 = yf.n1 + YO_OUT + 2 * T + 5.0                      # bridge bottom: 5 over the thigh's channel-top plate; its top must also pass 4 under roll_face's bottom edge
P('hipB', 'leg',    'x', YF_X0, YF_X0 + T, LEG_Y - 20.0, LEG_Y + 20.0, yf.n1, BR_Z1)
P('hipB', 'bridge', 'z', BR_Z1, BR_Z1 + T, RO_X0, YF_X0 + T, LEG_Y - 20.0, LEG_Y + 20.0)
ro = L('hipB', 'roll_out', 'hip_roll', 'output', LEG_Y - 52.0, LEG_Y + 52.0, BR_Z1 + T, ZHR + 52.0, outset=T)

# ---- thigh: hip yaw OUTPUT -> knee STATOR. Wrapped channel, jogged to the knee. ----
# Same architecture as the 1173 thigh (DESIGN.md "wrap, don't tab" + the jog), on
# a D124 yaw body and a D166 knee body.
YAW_R = rad('X10-200')
BOSS_R = YAW_R + 13.0                 # X10 connector boss: not on the drawing; X6's is OD+10.9. Assumed.
WRAP = BOSS_R + 9.0                   # 84: inner face of anything wrapping the yaw body. 4 more than
                                      # boss+5: the flanges sweep +/-57 deg and the X10 stator plate
                                      # below them (132 square, reach 80 with r35 corners) must clear.
TH_FRONT = WRAP
TH_Y0, TH_Y1 = LEG_Y - WRAP, LEG_Y + WRAP
# the yaw OUTPUT plate sits on a 6 mm spacer ring (3T outset, not the usual T):
# hipB's bridge has to pass between the roll body and the channel top, and the
# output plate + top plate stack needs that room under the roll body.
KNEE_R = rad('X15-450')
TH_FLG_BOT = ZK + KNEE_R + 8.0                         # flange bottom clears the knee body by 8 (shelf under it)
# The yaw OUTPUT plate is +/-51 (its pilot bore and ring); the channel's TOP is a
# separate plate stacked on it and bolted through the same ring, clipped at the rear
# to x -10: its rotating rear edge must miss hipB's leg (r 68, straight behind) at
# +/-57 deg, and a rear edge at x -30 still swept through it. The flanges hang from
# the top plate's side edges and reach back only to x -30 as lobes - their rear ends
# then sweep r 91 between +/-109 deg and never cross the back where the leg is. So
# the yaw body's rear half is EXPOSED, as the G1's hip actuators are.
yo = L('thigh', 'yaw_out', 'hip_yaw', 'output', -51.0, 51.0, LEG_Y - 51.0, LEG_Y + 51.0, outset=YO_OUT)
ct = P('thigh', 'top', 'z', yo.n1, yo.n1 + T, -10.0, TH_FRONT + T, TH_Y0 - T, TH_Y1 + T,
       holes=[h for h in yo.holes if h[2] < 10.0 and h[0] > -10.0 + 8.0])   # the front six of the ring
TH_TOP = ct.n1
TH_REAR = -30.0
# TAPER. The flanges are TILTED inward about the top plate's side edges so the
# channel narrows from 174 at the hip to 142 at the shelf - the H2's thigh goes
# 128 -> 74 over its length; ours can only lean its two walls, and it leans them
# 4.6 deg. Their rear edge is also cut on the slant (trapezoid) from x -30 at the
# top to x 0 at the bottom. The web between them is then a trapezoid too. The
# windows and hems ride the same tilt; the shelf spans the narrowed bottom.
TH_IN = 16.0                                            # how far each flange bottom leans in
TH_H = TH_TOP - TH_FLG_BOT
TH_TILT = math.degrees(math.atan2(TH_IN, TH_H))
TH_BOT_REAR = 0.0
TH_WIN = (TH_BOT_REAR + 12.0, TH_FRONT - 20.0, TH_FLG_BOT + 18.0, TH_TOP - 18.0)
TH_HEMR = (TH_WIN[0] - HEM, TH_WIN[1] + HEM, TH_WIN[2] - HEM, TH_WIN[3] + HEM)
FL_POLY = [(TH_BOT_REAR, TH_FLG_BOT), (TH_FRONT + T, TH_FLG_BOT), (TH_FRONT + T, TH_TOP), (TH_REAR, TH_TOP)]
PIV_IN = (0.0, TH_Y0 - T / 2.0, TH_TOP)
PIV_OUT = (0.0, TH_Y1 + T / 2.0, TH_TOP)
# a y-normal plate tilted about x: +deg rotates +z toward -y... we want the BOTTOM
# (which is at -z from the pivot) to move toward +y for the inner flange (y small)
# and toward -y for the outer. Rotation about x by a: y' = y cos a - z sin a; for a
# point below the pivot (dz < 0), y' - y = -dz sin a > 0 when a > 0. So inner +, outer -.
P('thigh', 'flange_in',  'y', TH_Y0 - T, TH_Y0, 0, 0, 0, 0, slots=[TH_WIN], poly=FL_POLY, tilt=('x', TH_TILT, PIV_IN))
P('thigh', 'flange_out', 'y', TH_Y1, TH_Y1 + T, 0, 0, 0, 0, slots=[TH_WIN], poly=FL_POLY, tilt=('x', -TH_TILT, PIV_OUT))
P('thigh', 'hem_in',     'y', TH_Y0 - 2 * T, TH_Y0 - T, *TH_HEMR, slots=[TH_WIN], tilt=('x', TH_TILT, PIV_IN))
P('thigh', 'hem_out',    'y', TH_Y1 + T, TH_Y1 + 2 * T, *TH_HEMR, slots=[TH_WIN], tilt=('x', -TH_TILT, PIV_OUT))
WEB_POLY = [(TH_Y0 - T + TH_IN, TH_FLG_BOT), (TH_Y1 + T - TH_IN, TH_FLG_BOT), (TH_Y1 + T, TH_TOP), (TH_Y0 - T, TH_TOP)]
P('thigh', 'web',        'x', TH_FRONT, TH_FRONT + T, 0, 0, 0, 0, poly=WEB_POLY)
# the jog: shelf under the yaw body back to the knee plate's front, lower web, knee plate
TH_LO_X = -KB + KNEE_R + 4.0                           # lower web inner face, 4 clear of the knee body front
TH_LO_Y0, TH_LO_Y1 = LEG_Y + T, LEG_Y + T + 40.0
P('thigh', 'shelf',  'z', TH_FLG_BOT - T, TH_FLG_BOT, TH_LO_X, TH_FRONT + T, TH_LO_Y0, TH_Y1 + T - TH_IN)
kf = L('thigh', 'knee_face', 'knee', 'stator', -KB - KNEE_R - 5.0, TH_LO_X + T, ZK - 70.0, ZK + 70.0)
# the knee stator plate - the biggest flat face on the leg - is an OCTAGON: 35 mm
# corner cuts, which leaves its front and rear edges straight where web_lo and
# the gusset bend off them
KC = 35.0
kf.poly = [(kf.u0 + KC, kf.v0), (kf.u1 - KC, kf.v0), (kf.u1, kf.v0 + KC), (kf.u1, kf.v1 - KC),
           (kf.u1 - KC, kf.v1), (kf.u0 + KC, kf.v1), (kf.u0, kf.v1 - KC), (kf.u0, kf.v0 + KC)]
P('thigh', 'knee_gus', 'x', -KB - KNEE_R - 5.0 - T, -KB - KNEE_R - 5.0, LEG_Y - 64.0, LEG_Y, kf.v0 + KC, kf.v1 - KC)
P('thigh', 'web_lo', 'x', TH_LO_X, TH_LO_X + T, TH_LO_Y0, TH_LO_Y1, kf.v0 + KC, TH_FLG_BOT - T)

# ---- shank: knee OUTPUT -> ankle motors -> pushrod ankle ----
# (d) of the H3 programme. No actuator at the ankle: both ankle motors are
# X8-120s on the shank's inner face at z -627 / -729 (bodies inboard like the
# knee's), their output CRANKS on the outer face, two vertical pushrods down to
# a sheet YOKE on a pitch pin held by two fork brackets, the foot on a roll pin
# through the yoke. ONE web plate, y 167..170: it is the knee output plate at
# the top, both motors' stator plate in the middle (their faces are at y 167),
# and it stops 85 over the pitch pin - the yoke's jaws sweep r 77 about that
# pin and nothing in their y-band may be inside it. What the sweeps placed:
# motors with their tops 114 under the knee axis (the knee stator plate's
# octagon reaches r 110 and the shank folds 132 deg past it).
MR = rad('X8-120')
WEB_Y0 = face_n('knee', 'output', T)[0]                  # 167
assert abs(face_n('ank_mA', 'stator')[0] - WEB_Y0) < 0.01, 'motor faces must lie in the web plane'
SW_POLY = [(-KB - 30.0, ZAP + 90.0), (-KB + 30.0, ZAP + 90.0), (-KB + 66.0, ZMB - 51.0),
           (-KB + 66.0, ZK + 66.0), (-KB - 66.0, ZK + 66.0), (-KB - 66.0, ZMB - 51.0)]
sw = P('shank', 'web', 'y', WEB_Y0, WEB_Y0 + T, 0, 0, 0, 0, poly=SW_POLY)
for j_, role in (('knee', 'output'), ('ank_mA', 'stator'), ('ank_mB', 'stator')):
    j = JOINTS[j_]
    ifc = IFACE[j['m']]
    bcd = ifc['sb'] if role == 'stator' else ifc['ob']
    dia = ifc['sh'] if role == 'stator' else ifc['oh']
    n_ = ifc['sn'] if role == 'stator' else ifc['on']
    sw.holes += ring(bcd, -KB, j['c'][2], dia + 1.1, n_, clock=180.0 / n_)
    sw.holes.append((-KB, j['c'][2], (ifc['spig'] if role == 'stator' else ifc['pilot']) + 1.0))
# front flange folds OUTBOARD off the web's straight front edge (x 6)
P('shank', 'flange_fr', 'x', -KB + 66.0, -KB + 66.0 + T, WEB_Y0 + T, WEB_Y0 + T + 70.0, ZMB - 51.0, ZK - KNEE_R - 5.0)
# fork brackets, inboard and outboard: each laps the web, bends under at z ZAP+82
# (4 over the jaws' swept top), bends down into a fork plate carrying the pin.
FK_Z = ZAP + 87.0                                       # 4 over the jaws' swept top (r 83 about the pitch pin)
FORK_IN_Y = WEB_Y0 - 29.5                              # inboard fork outer face (137.5): spine starts at 143.5
FORK_OUT_Y = WEB_Y0 + 15.0                             # outboard fork inner face (182): jaws end at 178
for tag, lap_y0, f_y0, br_y0, br_y1 in (('in', WEB_Y0 - T, FORK_IN_Y - T, FORK_IN_Y, WEB_Y0),
                                        ('out', WEB_Y0 + T, FORK_OUT_Y, WEB_Y0 + T, FORK_OUT_Y)):
    P('shank', 'fork_lap_' + tag, 'y', lap_y0, lap_y0 + T, -KB - 30.0, -KB + 30.0, FK_Z, FK_Z + 50.0,
      holes=[(-KB - 15.0, FK_Z + 30.0, 4.5), (-KB + 15.0, FK_Z + 30.0, 4.5)])
    P('shank', 'fork_bridge_' + tag, 'z', FK_Z - T, FK_Z, -KB - 30.0, -KB + 30.0, br_y0, br_y1)
    P('shank', 'fork_' + tag, 'y', f_y0, f_y0 + T, -KB - 30.0, -KB + 30.0, ZAP - 10.0, FK_Z - T,
      holes=[(-KB, ZAP, 13.0)])
sw.holes += [(-KB - 15.0, FK_Z + 30.0, 4.5), (-KB + 15.0, FK_Z + 30.0, 4.5)]
# cranks: the motors' output plates, as bars to the pushrod pins (A forward, B back)
PIN_A_X, PIN_B_X = -KB + 68.0, -KB - 60.0
ca = L('crankA', 'bar', 'ank_mA', 'output', -KB - 20.0, PIN_A_X + 12.0, ZMA - 20.0, ZMA + 20.0, outset=T)
cb = L('crankB', 'bar', 'ank_mB', 'output', PIN_B_X - 12.0, -KB + 20.0, ZMB - 20.0, ZMB + 20.0, outset=T)
ca.holes.append((PIN_A_X, ZMA, 6.5))
cb.holes.append((PIN_B_X, ZMB, 6.5))
ROD_Y = ca.n0 + T / 2.0
# pushrods (emitted by the loon as D10 steel cylinders; not plates, not swept -
# they are the one part of the ankle the sweeps do not see)
RODS = [('rodA', PIN_A_X, ROD_Y, ZAP + 15.0, ZMA), ('rodB', PIN_B_X, ROD_Y, ZAP + 15.0, ZMB)]

# ---- ankle: the YOKE on the pitch pin; carries the roll pin ----
YK_Y0 = FORK_IN_Y + 6.0                                 # spine inner face (143.5), 6 off the inboard fork: the foot
                                                        # column's top corners swing 3.7 inboard at 23 deg of roll
YK_Y1 = FORK_OUT_Y - 4.0                                # jaws' outer edge (178), 4 off the outboard fork
ys = P('ankle', 'spine', 'y', YK_Y0, YK_Y0 + T, PIN_B_X - 1.5, PIN_A_X + 1.5, ZAP - 10.0, ZAP + 25.0,
       holes=[(-KB, ZAP, 13.0)])
for tag, x_ in (('jaw_fr', PIN_A_X), ('jaw_rr', PIN_B_X)):
    P('ankle', tag, 'x', x_ - 1.5, x_ + 1.5, YK_Y0, YK_Y1, ZAP - 45.0, ZAP + 25.0,
      holes=[(ROD_Y, ZAP + 15.0, 6.5), (LEG_Y, ZAR, 13.0)])

# ---- foot: a column on the roll pin, a foot flange, the sole ----
# The column rides the roll pin between the jaws; its top stops 9.5 over the pin
# (the hole's margin) and the pitch pin is 28 above, so its corners (r 14) clear.
CY0, CY1 = LEG_Y - 9.5, YK_Y1                            # column y 154.5..178
P('foot', 'column', 'x', -KB - 1.5, -KB + 1.5, CY0, CY1, ZS + T, ZAR + 9.5,
  holes=[(LEG_Y, ZAR, 13.0), (LEG_Y - 4.0, ZS + 18.0, 4.5), (LEG_Y + 8.0, ZS + 18.0, 4.5)])
P('foot', 'col_foot', 'z', ZS + T, ZS + 2 * T, -KB + 1.5, -KB + 45.0, CY0, CY1,
  holes=[(-KB + 30.0, LEG_Y - 4.0, 4.5), (-KB + 30.0, LEG_Y + 8.0, 4.5)])
P('foot', 'sole', 'z', ZS, ZS + T, -KB - 60.0, -KB + 170.0, LEG_Y - 55.0, LEG_Y + 55.0,
  holes=[(-KB + 30.0, LEG_Y - 4.0, 4.5), (-KB + 30.0, LEG_Y + 8.0, 4.5)])

# ---- arm ----
# shoA: shoulder pitch OUTPUT -> shoulder roll STATOR. Lapped pair: the tab bends
# off shop_out's rear edge INBOARD (y 140..218 at the bend, lobed down beside the
# roll body), onto a stator plate extended inboard to carry it. The roll body sits
# forward (x -55..12.5) so the shoulder pitch body (R48) clears the tab plane.
SRF_X0 = SHR_X - ln('X6-60') / 2.0 - T                # -58.02: shoulder roll stator plate x0
spo = L('shoA', 'shop_out', 'sho_pitch', 'output', SRF_X0, 36.0, ZSP - 36.0, ZSP + 36.0, outset=T)
SA_Y0 = 172.0                         # inboard limit of the shoA lap: chest side is at y 165..168
srf = L('shoA', 'shor_face', 'sho_roll', 'stator', SA_Y0, SHO_Y + 44.0, ZSR - 44.0, ZSR + 44.0)
srf.holes += [(SA_Y0 + 12.0, ZSR - 15.0, 4.5), (SA_Y0 + 12.0, ZSR + 15.0, 4.5)]   # lap bolts
# shoB: shoulder roll OUTPUT -> elbow STATOR. Channel open inboard; web 15 below the
# roll axis; elbow body passes through the web; stator bridged over the body.
SB_Y = SHO_Y + 46.0                   # outboard of the roll stator plate's fitted edge (R43.75) by 2
SB_XF = 46.0                          # web front edge: elbow clearance hole R41 + 5
sro = L('shoB', 'shor_out', 'sho_roll', 'output', SHO_Y - 30.0, SB_Y, ZSR - 30.0, ZSR + 30.0, outset=T)
ELB_R = rad('X6-60')
web = P('shoB', 'web', 'y', SB_Y, SB_Y + T, sro.n0, SB_XF, ZE - ELB_R - 6.0, ZSR - 15.0,
        holes=[(0.0, ZE, 2 * ELB_R + 2.0), (-14.0, ZE + 60.0, 4.5), (14.0, ZE + 60.0, 4.5)])
P('shoB', 'flange_fr', 'x', SB_XF, SB_XF + T, SHO_Y + 4.0, SB_Y + T, web.v0, web.v1)
ef = L('shoB', 'elb_face', 'elb_pitch', 'stator', -32.0, 32.0, ZE - 32.0, ZE + ELB_R + 6.0)
P('shoB', 'elb_shelf', 'z', ef.v1, ef.v1 + T, ef.u0, ef.u1, ef.n0, SB_Y)
P('shoB', 'elb_lap', 'y', SB_Y - T, SB_Y, ef.u0, ef.u1, ef.v1 + T, ef.v1 + T + 30.0,
  holes=[(-14.0, ZE + 60.0, 4.5), (14.0, ZE + 60.0, 4.5)])
# elbA: elbow OUTPUT -> wrist STATOR. A fork bent inboard off the output plate,
# lapped at the wrist by tabs bent up from the wrist stator plate.
eo = L('elbA', 'elb_out', 'elb_pitch', 'output', -36.0, 36.0, ZE - 30.0, ZE + 30.0, outset=T)
FK_Y0 = SHO_Y - 28.0
P('elbA', 'flange_fr', 'x',  eo.u1, eo.u1 + T, FK_Y0, eo.n0, ZW + 3.0, eo.v1,
  holes=[(FK_Y0 + 12.0, ZW + 30.0, 4.5), (FK_Y0 + 12.0, ZW + 50.0, 4.5)])
P('elbA', 'flange_rr', 'x', eo.u0 - T, eo.u0, FK_Y0, eo.n0, ZW + 3.0, eo.v1,
  holes=[(FK_Y0 + 12.0, ZW + 30.0, 4.5), (FK_Y0 + 12.0, ZW + 50.0, 4.5)])
wf = L('elbA', 'wrist_face', 'wrist_yaw', 'stator', eo.u0, eo.u1, SHO_Y - 30.0, SHO_Y + 30.0)
P('elbA', 'wtab_fr', 'x', eo.u1 - T, eo.u1, wf.v0, wf.v1, wf.n1, wf.n1 + 60.0,
  holes=[(FK_Y0 + 12.0, ZW + 30.0, 4.5), (FK_Y0 + 12.0, ZW + 50.0, 4.5)])
P('elbA', 'wtab_rr', 'x', eo.u0, eo.u0 + T, wf.v0, wf.v1, wf.n1, wf.n1 + 60.0,
  holes=[(FK_Y0 + 12.0, ZW + 30.0, 4.5), (FK_Y0 + 12.0, ZW + 50.0, 4.5)])
WR_BOT = ZW - ln('X4-36')
P('hand', 'palm', 'z', WR_BOT - 2 * T, WR_BOT - T, -30.0, 30.0, SHO_Y - 30.0, SHO_Y + 30.0)

# ---- pelvis: a PART, on the waist yaw ----
# The two X15 hip-pitch bodies (y 22..94, x +/-83, z -143..23) live in it; its
# side walls ARE their stator plates; its top plate is the waist-yaw stator
# plate (the X8 stands face-up inside it, 4 over the hip bodies). Front and
# rear are hexagons - the bottom corners chamfered off, which lets the hip
# bodies' lower rims show through at the corners, the way the H2's pelvis pod
# shows its hip rings. (c) of the H3 programme.
WA_Y = face_n('hip_pitch', 'stator')[0]                # 94
WA_X0, WA_X1 = -rad('X15-450') - 10.0, rad('X15-450') + 5.0
PV_Z0 = ZHP - rad('X15-450') - 7.0
pt = L('pelvisC', 'top', 'waist_yaw', 'stator', WA_X0 - T, WA_X1 + T, -WA_Y - T, WA_Y + T)
PV_Z1 = pt.n0                                           # waist stator plate bottom = wall top
hm = L('pelvisS', 'hip_mount', 'hip_pitch', 'stator', WA_X0 - T, WA_X1 + T, PV_Z0, PV_Z1)
PV_C = 30.0
PV_HEX = [(-WA_Y + PV_C, PV_Z0), (WA_Y - PV_C, PV_Z0), (WA_Y, PV_Z0 + PV_C), (WA_Y, PV_Z1),
          (-WA_Y, PV_Z1), (-WA_Y, PV_Z0 + PV_C)]
P('pelvisC', 'rear_lo',  'x', WA_X0 - T, WA_X0, 0, 0, 0, 0, poly=PV_HEX)
P('pelvisC', 'front_lo', 'x', WA_X1, WA_X1 + T, 0, 0, 0, 0, poly=PV_HEX)

# ---- trunk: a chest with CHAMFERS, standing on the waist yaw OUTPUT ----
# The chest floor is the waist-yaw output plate (stacked on the pelvis top, the
# pancake). Front and rear are ONE convex octagon each (floor width at the seam,
# flaring to the chest over 48 mm, full width to z 420, chamfering in to the
# neck); the side is three plates - a waist chamfer panel, the flat side with
# its window, a shoulder chamfer. That is the H2's torso (widest at two-thirds
# height, narrowing to waist and neck) in four folds a side.
fl = L('trunkC', 'floor', 'waist_yaw', 'output', -WA_X0 - 0.0, 0.0, 0.0, 0.0, outset=T)
ZF = fl.n1                                              # chest floor top
FL_Y = 108.0                                            # floor half-width (the seam)
fl.u0, fl.u1, fl.v0, fl.v1 = -102.0, 97.0, -FL_Y, FL_Y
ZC0 = 160.0                                             # chest full width from here
ZC1 = 420.0                                             # ... to here
CH_Y = 165.0
CH_X0, CH_X1 = -102.0, 97.0
NK_Y = 115.0                                            # half-width at the deck
ZD = ZHEY - T                                           # deck top = head yaw face (the deck is its stator plate)
SIDE_WIN = (-60.0, 55.0, ZC0 + 40.0, ZC1 - 40.0)
P('trunkS', 'side', 'y', CH_Y, CH_Y + T, CH_X0, CH_X1, ZC0, ZC1, slots=[SIDE_WIN])
P('trunkS', 'side_hem', 'y', CH_Y - T, CH_Y, SIDE_WIN[0] - HEM, SIDE_WIN[1] + HEM,
  SIDE_WIN[2] - HEM, SIDE_WIN[3] + HEM, slots=[SIDE_WIN])
def chamfer(tag, y_edge, z_edge, y_far, z_far, n_in):
    """A side chamfer panel from the side plate's edge (y_edge, z_edge) to (y_far, z_far):
    defined flat, hanging from the pivot edge by the slant length, tilted about x."""
    dy, dz = y_far - y_edge, z_far - z_edge
    L_ = math.hypot(dy, dz)
    ang = math.degrees(math.atan2(dy, dz))
    for a_ in (-ang, ang):
        R_ = _rot_axis('x', a_)
        q = _apply(R_, (0.0, y_edge, z_edge), (0.0, y_edge, z_edge + L_))
        if abs(q[1] - y_far) < 0.5 and abs(q[2] - z_far) < 0.5:
            return P('trunkS', tag, 'y', y_edge + n_in, y_edge + n_in + T, CH_X0, CH_X1, z_edge, z_edge + L_,
                     tilt=('x', a_, (0.0, y_edge, z_edge)))
    raise AssertionError('chamfer %s: tilt sign' % tag)
chamfer('waist_chamfer', CH_Y + T / 2.0, ZC0, FL_Y + T / 2.0, ZF, -T / 2.0)
chamfer('sho_chamfer',   CH_Y + T / 2.0, ZC1, NK_Y + T / 2.0, ZD, -T / 2.0)
ss = L('trunkS', 'sho_stat', 'sho_pitch', 'stator', -52.0, 52.0, ZSP - 52.0, ZSP + 52.0)
P('trunkS', 'outrigger', 'z', ss.v0 - T, ss.v0, -30.0, 30.0, CH_Y, ss.n1)
FR_WIN = (-60.0, 60.0, ZC0 + 20.0, ZC1 - 20.0)
FR_HEMR = (FR_WIN[0] - HEM, FR_WIN[1] + HEM, FR_WIN[2] - HEM, FR_WIN[3] + HEM)
OCT = [(-FL_Y - T, ZF), (FL_Y + T, ZF), (CH_Y + T, ZC0), (CH_Y + T, ZC1),
       (NK_Y + T, ZD), (-NK_Y - T, ZD), (-CH_Y - T, ZC1), (-CH_Y - T, ZC0)]
P('trunkC', 'rear',      'x', CH_X0 - T, CH_X0, 0, 0, 0, 0, slots=[FR_WIN], poly=OCT)
P('trunkC', 'rear_hem',  'x', CH_X0, CH_X0 + T, *FR_HEMR, slots=[FR_WIN])
P('trunkC', 'front',     'x', CH_X1, CH_X1 + T, 0, 0, 0, 0, slots=[FR_WIN], poly=OCT)
P('trunkC', 'front_hem', 'x', CH_X1 - T, CH_X1, *FR_HEMR, slots=[FR_WIN])
L('trunkC', 'deck', 'head_yaw', 'stator', CH_X0, CH_X1, -NK_Y - T, NK_Y + T)

# ---- head: an open cage on the pitch output ----
HZ0, HZ1 = ZHEP - 8.0, ZCROWN - T
HW = 60.0
P('head', 'cage_l', 'y', HW - T, HW, -70.0, 55.0, HZ0, HZ1, holes=[(-8.0, ZHEP + 42.0, 60.0)])
P('head', 'cage_r', 'y', -HW, -HW + T, -70.0, 55.0, HZ0, HZ1, holes=[(-8.0, ZHEP + 42.0, 60.0)])
P('head', 'cage_top', 'z', HZ1, HZ1 + T, -70.0, 55.0, -HW, HW, holes=[(-8.0, 0.0, 50.0)])
P('head', 'cage_face', 'x', 55.0, 55.0 + T, -HW, HW, HZ0 + 10.0, HZ1 - 10.0,
  holes=[(-32.0, ZHEP + 52.0, 28.0), (32.0, ZHEP + 52.0, 28.0), (0.0, ZHEP + 18.0, 20.0)])
L('neck', 'hy_out', 'head_yaw', 'output', -28.0, 28.0, -28.0, 28.0, outset=T)
L('neck', 'hp_face', 'head_pitch', 'stator', -30.0, 30.0, ZHEP - 30.0, ZHEP + 30.0)


OWNER = {
    ('head', 'cage_l'): 'head_pitch', ('head', 'cage_r'): 'head_pitch',
    ('head', 'cage_face'): 'head_pitch', ('head', 'cage_top'): 'head_pitch',
    ('pelvisS', 'hip_mount'): 'hip_pitch', ('trunkS', 'sho_stat'): 'sho_pitch',
    ('trunkC', 'deck'): 'head_yaw', ('pelvisC', 'top'): 'waist_yaw', ('trunkC', 'floor'): 'waist_yaw',
    ('hipA', 'pitch_face'): 'hip_pitch', ('hipA', 'roll_face'): 'hip_roll',
    ('hipB', 'roll_out'): 'hip_roll',    ('hipB', 'yaw_face'): 'hip_yaw',
    ('thigh', 'yaw_out'): 'hip_yaw',     ('thigh', 'knee_face'): 'knee', ('thigh', 'top'): 'hip_yaw',
    ('shank', 'web'): 'knee',
    ('crankA', 'bar'): 'ank_mA',         ('crankB', 'bar'): 'ank_mB',
    ('shoA', 'shop_out'): 'sho_pitch',   ('shoA', 'shor_face'): 'sho_roll',
    ('shoB', 'shor_out'): 'sho_roll',    ('shoB', 'elb_face'): 'elb_pitch',
    ('elbA', 'elb_out'): 'elb_pitch',    ('elbA', 'wrist_face'): 'wrist_yaw',
    ('neck', 'hy_out'): 'head_yaw',      ('neck', 'hp_face'): 'head_pitch',
}
SECOND_OWNER = {
    ('head', 'cage_l'): 'head_pitch', ('head', 'cage_r'): 'head_pitch',
    ('head', 'cage_face'): 'head_pitch', ('head', 'cage_top'): 'head_pitch',
    ('pelvisS', 'hip_mount'): 'hip_pitch', ('trunkS', 'sho_stat'): 'sho_pitch',
    ('trunkC', 'deck'): 'head_yaw', ('pelvisC', 'top'): 'waist_yaw', ('trunkC', 'floor'): 'waist_yaw', ('shank', 'web'): 'ank_pitch', ('shank', 'web'): 'knee'}


ANGLES = []   # retired with the angle stock

# Bend tabs: each is an ordinary plate; what makes it a bend is that it shares an
# edge with its neighbour. A tab with targets bends from them; one with none laps.
BEND_TABS = [
    # link, tab normal, thickness span, u range, v range, bends_to (plate tags)
    # shoA: upper lobe bends off shop_out inboard; lower lobe laps shor_face's inboard extension
    ('shoA',  'x', (SRF_X0 - T, SRF_X0), (SA_Y0, spo.n0), (spo.v0, spo.v0 + 30.0), ('shop_out',)),
    ('shoA',  'x', (SRF_X0 - T, SRF_X0), (SA_Y0, sro.u0 - 4.0), (srf.v0, spo.v0), ()),
]
ANGLE_MASS = (ANGLE[0] + ANGLE[1] - ANGLE[2]) * ANGLE[2] * 2.70e-3 / 1000.0  # kg per mm

# Bend LINES for plates that exist in the table already. (link, axis the line is
# measured on, coordinate, plates governed).
BENDS = [
    ('hipA',  'x', RF_X0 + T / 2.0,              ('pitch_face', 'roll_face')),
    ('hipB',  'x', YF_X0 + T / 2.0,              ('yaw_face', 'leg')),
    ('hipB',  'z', BR_Z1 + T / 2.0,              ('leg', 'bridge')),
    ('hipB',  'x', RO_X0 + T / 2.0,              ('bridge', 'roll_out')),
    ('thigh', 'z', TH_TOP - T / 2.0,             ('top', 'flange_in', 'flange_out', 'web')),
    ('thigh', 'z', TH_FLG_BOT - T / 2.0,         ('shelf',)),
    ('thigh', 'x', TH_LO_X + T / 2.0,            ('web_lo', 'knee_face')),
    ('thigh', 'x', -KB - KNEE_R - 5.0 - T / 2.0, ('knee_gus',)),
    ('shank', 'x', -KB + 66.0 + T / 2.0,         ('web', 'flange_fr')),
    ('shank', 'z', FK_Z - T / 2.0,               ('fork_lap_in', 'fork_bridge_in', 'fork_lap_out', 'fork_bridge_out')),
    ('shank', 'y', FORK_IN_Y - T / 2.0,          ('fork_bridge_in', 'fork_in')),
    ('shank', 'y', FORK_OUT_Y + T / 2.0,         ('fork_bridge_out', 'fork_out')),
    ('ankle', 'x', PIN_A_X - 1.5 + T / 2.0,      ('spine', 'jaw_fr')),
    ('ankle', 'x', PIN_B_X - 1.5 + T / 2.0,      ('jaw_rr',)),
    ('foot',  'z', ZS + T + T / 2.0,             ('column', 'col_foot')),
    ('shoB',  'y', SB_Y + T / 2.0,               ('shor_out', 'web')),
    ('shoB',  'x', SB_XF + T / 2.0,              ('flange_fr',)),
    ('shoB',  'z', ef.v1 + T / 2.0,              ('elb_face', 'elb_shelf')),
    ('shoB',  'y', SB_Y - T / 2.0,               ('elb_shelf', 'elb_lap')),
    ('elbA',  'x', eo.u1 + T / 2.0,              ('elb_out', 'flange_fr')),
    ('elbA',  'x', eo.u0 - T / 2.0,              ('flange_rr',)),
    ('elbA',  'x', eo.u1 - T / 2.0,              ('wrist_face', 'wtab_fr')),
    ('elbA',  'x', eo.u0 + T / 2.0,              ('wtab_rr',)),
]


def dump_json(path):
    import json
    out = {'thickness': T, 'angle': ANGLE, 'plates': [], 'angles': []}
    for p in PLATES:
        out['plates'].append(dict(link=p.link, tag=p.tag, normal=p.normal,
                                  n0=p.n0, n1=p.n1, u0=p.u0, u1=p.u1,
                                  v0=p.v0, v1=p.v1, holes=p.holes, slots=p.slots, r_want=p.r_want,
                                  poly=p.poly, tilt=p.tilt))
    for (lk, ax, org, ext, flip) in ANGLES:
        out['angles'].append(dict(link=lk, axis=ax, org=org, ext=ext, flip=flip))
    json.dump(out, open(path, 'w'), indent=1)
    return out


for (lk, nrm, span, ur, vr, _tgt) in BEND_TABS:
    PLATES.append(Plate(lk, 'bendtab', nrm, span[0], span[1], ur[0], ur[1], vr[0], vr[1]))


def main():
    print('=== plates ===')
    tot = 0.0
    per = {}
    for p in PLATES:
        tot += p.mass()
        per[p.link] = per.get(p.link, 0.0) + p.mass()
    for link in sorted(per):
        n = sum(1 for p in PLATES if p.link == link)
        print('  %-8s %d plate(s)  %6.0f g' % (link, n, per[link] * 1000))
    print('  %-8s %28.0f g   (one side; angle stock not counted)'
          % ('TOTAL', tot * 1000))

    print('\n=== hole containment (hole fully inside its plate) ===')
    bad = 0
    for p in PLATES:
        i = AXI[p.normal]
        for (u, v, d) in p.holes:
            if p.poly:
                m = -poly_sdf((u, v), p.poly) - d / 2.0
            else:
                m = min(u - p.u0, p.u1 - u, v - p.v0, p.v1 - v) - d / 2.0
            if m < T:
                print('  %-8s %-11s hole d%.1f at (%.1f, %.1f): margin %.2f mm'
                      % (p.link, p.tag, d, u, v, m))
                bad += 1
        for (su0, su1, sv0, sv1) in p.slots:
            m = min(su0 - p.u0, p.u1 - su1, sv0 - p.v0, p.v1 - sv1)
            if m < T:
                print('  %-8s %-11s slot (%.0f..%.0f, %.0f..%.0f): margin %.2f mm'
                      % (p.link, p.tag, su0, su1, sv0, sv1, m))
                bad += 1
    print('  holes/slots with insufficient edge margin: %d' % bad)

    print('\n=== plate vs actuator body clearance ===')
    obs = obstacles()
    clash = 0
    for p in PLATES:
        for (n, ax, uv, r, ext) in obs:
            h = hit(p.box(), ax, uv, r, ext)
            # a plate with a clearance bore centred on the actuator axis, larger than
            # the actuator, lets the body pass THROUGH it - not a clash
            if h and p.normal == ax:
                for (hu, hv, hd) in p.holes:
                    if math.hypot(hu - uv[0], hv - uv[1]) < 1.0 and hd / 2.0 >= r:
                        h = None
                        break
            if h:
                # a plate bolted to a face legitimately touches that actuator
                own = OWNER.get((p.link, p.tag)) == n
                own = own or SECOND_OWNER.get((p.link, p.tag)) == n
                own = own or THIRD_OWNER.get((p.link, p.tag)) == n
                if h[1] > 0.6 and not own:
                    print('  %-8s %-11s vs %-10s  %.1f mm deep' % (p.link, p.tag, n, h[1]))
                    clash += 1
    print('  clashes: %d' % clash)

    print('\n=== bend rule (hole centre >= %.1f mm from bend line) ===' % (3 * T + 1.5))
    lim = 3 * T + 1.5
    worst, bad = 1e9, 0
    for p in PLATES:
        i = AXI[p.normal]
        for (tab, ax, span, ur, vr, tgt) in BEND_TABS:
            # only plates actually BENT to this tab are governed by the rule; a
            # lapped-and-bolted plate has no bend and no rule
            if tab != p.link or p.normal == ax or p.tag not in tgt:
                continue
            line = sum(span) / 2.0
            k = AXI[ax]
            if k == i:
                continue
            ui, vi = [q for q in range(3) if q != i]
            for (hu, hv, hd) in p.holes:
                pos = hu if ui == k else (hv if vi == k else None)
                if pos is None:
                    continue
                d = abs(pos - line)
                worst = min(worst, d)
                if d < lim:
                    print('  %-7s %-11s hole at %.1f is %.1f mm from the %s bend'
                          % (p.link, p.tag, pos, d, tab))
                    bad += 1
    for p in PLATES:
        i = AXI[p.normal]
        for (lk, ax, line, tgt) in BENDS:
            if lk != p.link or p.tag not in tgt:
                continue
            k = AXI[ax]
            if k == i:
                continue
            ui, vi = [q for q in range(3) if q != i]
            for (hu, hv, hd) in p.holes:
                pos = hu if ui == k else (hv if vi == k else None)
                if pos is None:
                    continue
                d = abs(pos - line)
                worst = min(worst, d)
                if d < lim:
                    print('  %-7s %-11s hole at %.1f is %.1f mm from the %s=%.1f bend'
                          % (p.link, p.tag, pos, d, ax, line))
                    bad += 1
    print('  bend-rule violations: %d   (tightest %.1f mm)' % (bad, worst))
    print('\n=== mass ===')
    print('  structure per side: %.0f g of 3 mm sheet' % (sum(p.mass() for p in PLATES) * 1000))

    import os
    d = dump_json(os.path.expanduser('~/Developer/pond_assets/rose_pro_plates.json'))
    print('\nwrote ~/Developer/pond_assets/rose_pro_plates.json  (%d plates, %d angles)'
          % (len(d['plates']), len(d['angles'])))


if __name__ == '__main__':
    main()
