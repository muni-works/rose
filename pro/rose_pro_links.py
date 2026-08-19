"""Structural link definitions for rose-pro at 1800 mm, and their checks.

STRUCTURAL SYSTEM: cut-and-bent 3 mm 5052 sheet. No angle stock, no machined
blocks. (SIZING.md: at 68 kg the leg channels want 4 mm and the hip output and
knee want machined fittings; the table is still 3 mm until the wraps are re-swept
at the new loads. T is one constant.)

Every coordinate below is DERIVED from rose_pro_layout (joint centres, actuator
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
import rose_pro_layout as LAY

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
        self.r_want = r_want          # corner radius asked for (rose_pro_loon.corner_radius backs off)
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
# channel narrows from 174 at the hip to 122 at the shelf - the H2's thigh goes
# 143 -> 72 over its length; ours can only lean its two walls, and it leans them
# 7.6 deg. Their rear edge is also cut on the slant (trapezoid) from x -30 at the
# top to x 0 at the bottom. The web between them is then a trapezoid too. The
# windows and hems ride the same tilt; the shelf spans the narrowed bottom.
# 26 mm of lean, not 16: the lean is pivoted at the TOP, so nothing it does can
# reach the yaw body or grow the hip_yaw sweep envelope (the flange's 0.6 mm
# tight against hipB's stator plate is at the top, and is unchanged); what limits
# it at the bottom is the knee body, and the flange bottoms stop 8 mm above it.
# 22 and not more: the plate-vs-body check uses the plate's AABB, and a TILTED
# plate's AABB is the rotated prism's box - it reports the flange as 3.8 mm into
# the yaw body at 26 mm of lean while the flange itself is 24 mm clear there
# (the lean is nothing at the top, where the body is). 22 is where the box stops
# lying. sweep.py, which is exact, has room for more.
TH_IN = 22.0                                            # how far each flange bottom leans in
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
P('thigh', 'web_lo', 'x', TH_LO_X, TH_LO_X + T, TH_LO_Y0, TH_LO_Y1, kf.v0 + KC, TH_FLG_BOT - T)
# KNEE SHROUD (DESIGN.md rule 3: cover what can pinch). The knee is an X15-450:
# a D166 body lying across the leg, and in profile it was the single heaviest
# read on the robot - a bare can where the thigh should end. It is also the one
# closure on the leg that can crush a hand at 68 kg. So the flat rear gusset that
# used to stiffen the knee plate's rear edge is now the LAST of four facets that
# start under the shelf, ride over the top of the body and wrap its back: one
# blank bent off the knee stator plate's straight rear edge, 40 deg per fold,
# tangent to r 88 (5 mm off the D166 body), corners at r 93.6. That is 187 mm
# across against the body's 166 and the stator octagon's 178 - it does not grow
# the knee, it turns it into the bottom of the thigh.
# It lives INBOARD (y 100..162): everything the shank carries near the knee is
# outboard of y 167 (the web, its front flange, both crank plates), so the shroud
# has 5 mm of y clearance through the whole 0..132 deg fold, and the nearest
# shank BODY in its own y band is a D96 ankle motor at r 114 about the knee axis.
KS_R = 88.0
KS_Y0, KS_Y1 = 88.0, 162.0                              # 4 mm past the body's inboard rim (y 92)
KS_RW = 10.0                                            # the ONE place rule 1's 25 mm is not asked for:
                                                        # these corners are the fold lines BETWEEN facets,
                                                        # and a 25 mm round on each leaves the band a row
                                                        # of pills. The band's outer edges get hems.
KS_ARC = (-40.0, 0.0, 40.0, 80.0, 120.0)                # facet edges, deg from +z, rearward
for _a0, _a1 in zip(KS_ARC[:-1], KS_ARC[1:]):
    _am = (_a0 + _a1) / 2.0
    _h = KS_R * math.tan(math.radians((_a1 - _a0) / 2.0)) + KS_RW    # +r so the facets overlap at the fold
    _p = P('thigh', 'knee_shroud%d' % int(_am), 'z', ZK + KS_R, ZK + KS_R + T,
           -KB - _h, -KB + _h, KS_Y0, KS_Y1, tilt=('y', -_am, (-KB, 0.0, ZK)))
    _p.r_want = KS_RW

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

# ---- the CALF SHELL: the channel that wraps the two ankle motors ----
# Both X8-120s hang INBOARD of the web (their bodies run y 88..167), so before
# this they were simply out in the air: two cans on a stick, which is what the
# side view said. The shell is one more blank on the same front bend line (x 6):
# two lap tabs on the web's inboard face -> a front closure across the inboard
# width -> the inboard wall at y 79..82 (6 mm off the motor bodies) -> a chamfer
# panel that pinches to the ankle.
# The wall's PROFILE is the calf: 131 mm deep at the knee end, swelling to 142 at
# z -650 (28 % of the way from knee to ankle - the H2's calf is 30 % down) and
# pinching to 106 at z -800. It is closed at the front and open at the back: the
# cranks (y 170..173), both pushrods (y 171.5) and the crank pins are outboard of
# the web and stay visible, because they are what explains the ankle.
# The wall cannot lean until it is BELOW the motors - a tilted wall at y > 88
# walks straight into a D96 body - so the whole pinch is one 37 deg chamfer panel
# under them, y 82 -> 131, which is also where the H2 pinches.
CF_Y = WEB_Y0 - 85.0                                    # 82: inboard face of the wall
CF_Z0, CF_Z1 = ZMB - 71.0, ZK - 95.0                    # -800 .. -560
CF_POLY = [(-KB + 66.0, CF_Z1), (-KB - 65.0, CF_Z1), (-KB - 76.0, ZMB + 79.0),
           (-KB - 40.0, CF_Z0), (-KB + 66.0, CF_Z0)]    # CCW: front-top, rear-top, calf, rear-bottom, front-bottom
P('shank', 'calf_in', 'y', CF_Y - T, CF_Y, 0, 0, 0, 0, poly=CF_POLY)
P('shank', 'calf_front', 'x', -KB + 66.0, -KB + 66.0 + T, CF_Y, WEB_Y0, CF_Z0, CF_Z1)
CF_TIN = 49.0                                           # how far the chamfer leans inboard
CF_TH = 82.0                                            # its slant length
CF_TILT = math.degrees(math.asin(CF_TIN / CF_TH))       # 37 deg
P('shank', 'calf_taper', 'y', CF_Y - T, CF_Y, -KB - 36.0, -KB + 40.0, CF_Z0 - CF_TH, CF_Z0,
  tilt=('x', CF_TILT, (0.0, CF_Y - T / 2.0, CF_Z0)))
# lap tabs: the two z-bands on the web's inboard face that are NOT covered by a
# motor (above A, below B). Their bolts are 22 mm off the x 6 bend line.
for _tag, _z0, _z1 in (('top', ZMA + 48.0 + 2.0, CF_Z1), ('bot', CF_Z0, ZMB - 48.0 - 4.0)):
    P('shank', 'calf_lap_' + _tag, 'y', WEB_Y0 - T, WEB_Y0, -KB + 30.0, -KB + 66.0, _z0, _z1,
      holes=[(-KB + 44.0, (_z0 + _z1) / 2.0, 4.5)])
    sw.holes.append((-KB + 44.0, (_z0 + _z1) / 2.0, 4.5))
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
SOLE_X0, SOLE_X1 = -KB - 60.0, -KB + 170.0
SOLE_Y0, SOLE_Y1 = LEG_Y - 55.0, LEG_Y + 55.0
P('foot', 'sole', 'z', ZS, ZS + T, SOLE_X0, SOLE_X1, SOLE_Y0, SOLE_Y1,
  holes=[(-KB + 30.0, LEG_Y - 4.0, 4.5), (-KB + 30.0, LEG_Y + 8.0, 4.5)])
# ---- the foot is a PAN, not a plate ----
# The sole plane and its footprint are frozen (the sim stands on them); everything
# here is added ABOVE it. Two side walls bent up off the sole's long edges, 25 mm
# tall at the ankle and cut down over the toe; a heel wall across the back; and a
# toe chamfer lapped on the walls' sloped top edges, so the front 55 mm of the
# foot is a wedge. Total 28 mm from the ground - a shoe rather than a flat plate.
# The heel wall is short (18) and set 3 mm inside the sole's rear edge: the rear
# ankle jaw hangs to z -996 at y 143..178 and 23 deg of roll lifts that corner of
# the pan 8 mm.
PAN_Z1 = ZS + 2 * T + 25.0                              # -1001: top of the side walls
PAN_TOE = -KB + 120.0                                   # where the toe chamfer starts
PAN_SIDE = [(SOLE_X0, ZS + T), (SOLE_X1 - 10.0, ZS + T), (PAN_TOE, PAN_Z1), (SOLE_X0, PAN_Z1)]
for _tag, _y0 in (('wall_in', SOLE_Y0), ('wall_out', SOLE_Y1 - T)):
    P('foot', _tag, 'y', _y0, _y0 + T, 0, 0, 0, 0, poly=PAN_SIDE)
P('foot', 'heel', 'x', SOLE_X0 + T, SOLE_X0 + 2 * T, SOLE_Y0, SOLE_Y1, ZS + T, ZS + T + 18.0)
PAN_TL = math.hypot(SOLE_X1 - 10.0 - PAN_TOE, PAN_Z1 - ZS - T)
PAN_TILT = math.degrees(math.asin((PAN_Z1 - ZS - T) / PAN_TL))
P('foot', 'toe', 'z', PAN_Z1 - T, PAN_Z1, PAN_TOE, PAN_TOE + PAN_TL, SOLE_Y0, SOLE_Y1,
  tilt=('y', PAN_TILT, (PAN_TOE, 0.0, PAN_Z1)))

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

# ---- profile panels: an x-normal plate tilted about y ----
# The chest was a rectangle in profile because `Plate` is axis-aligned and the
# front/rear panels are x-normal: their outline lives in (y, z), so a polygon
# there shapes the FRONT view and nothing shapes the SIDE view. A tilt about y
# is what shapes the side view - the panel leans, and the outline's v axis
# becomes slant length up the lean. Same trick `chamfer` plays about x, one
# axis over. (H3 (a): the chest is deep at the sternum and tapers to the waist.)
def xpanel(link, tag, x0, z0, x1, z1, wfun, brk, out, **kw):
    """A profile panel running from (x0, z0) to (x1, z1) in side view (z1 > z0),
    half-width wfun(z) with breakpoints `brk`, material on the +x side of the
    fold line if out > 0 else the -x side. Returns the Plate."""
    dx, dz = x1 - x0, z1 - z0
    assert dz > 0.0, tag
    Lp = math.hypot(dx, dz)
    ang = math.degrees(math.atan2(dx, dz))
    zs = [z0] + [z for z in brk if z0 < z < z1] + [z1]
    rim = [(wfun(z), z0 + (z - z0) / dz * Lp) for z in zs]
    poly = ([(-rim[0][0], rim[0][1])] + rim
            + [(-w, v) for (w, v) in reversed(rim[1:])])
    n0, n1 = (x0, x0 + T) if out > 0 else (x0 - T, x0)
    return P(link, tag, 'x', n0, n1, 0, 0, 0, 0, poly=poly,
             tilt=('y', ang, (x0, 0.0, z0)), **kw)


# ---- pelvis: a closed faceted pod, on the waist yaw ----
# The two X15 hip-pitch bodies (y 22..94, x +/-83, z -143..23) live in it; its
# side walls ARE their stator plates; its top plate is the waist-yaw stator
# plate (the X8 stands face-up inside it, 4 over the hip bodies). The walls are
# square down to z -110 and then the pod CHAMFERS IN, fore and aft, to a 136 mm
# crotch at z -150 - a tilted panel each way, matched by the same cut in the
# side wall's outline, so the pod tapers toward the thighs in profile and the
# hip rings still show on its sides. How far in is not taste: at z -110 the
# hip-pitch body still reaches x 66, so the fold line starts at x +/-68.
WA_Y = face_n('hip_pitch', 'stator')[0]                # 94
WA_X0, WA_X1 = -rad('X15-450') - 10.0, rad('X15-450') + 5.0
PV_Z0 = ZHP - rad('X15-450') - 7.0
pt = L('pelvisC', 'top', 'waist_yaw', 'stator', WA_X0 - T, WA_X1 + T, -WA_Y - T, WA_Y + T)
PV_Z1 = pt.n0                                           # waist stator plate bottom = wall top
PV_ZB = -110.0                                          # the pod goes square to here, then chamfers
PV_XF, PV_XR = 68.0, -68.0                              # crotch fold lines, front and rear
hm = L('pelvisS', 'hip_mount', 'hip_pitch', 'stator', WA_X0 - T, WA_X1 + T, PV_Z0, PV_Z1)
# The side wall's TOP-FRONT corner is cut on a hard diagonal and the front wall
# stops at z 25 - both are the new hip_pitch sweep talking. At full flexion
# (-107..-115 deg) the thigh's top plate, web and inboard flange fold up THROUGH
# the volume in front of the pod at z 33..67, and a square pelvis put 1.5 mm of
# aluminium in it. So the front wall covers the hip-pitch bodies (z -143..23)
# and stops where they stop - which leaves the waist-yaw body showing above it:
# the exposed waist reversal, arrived at by a clearance rather than by taste.
PV_ZTF = 18.0                                           # front wall top (5 under the hip bodies' rim)
PV_XT = 40.0                                            # side wall's top-front corner
hm.poly = [(WA_X0 - T, PV_ZB), (PV_XR, PV_Z0), (PV_XF, PV_Z0), (WA_X1 + T, PV_ZB),
           (WA_X1 + T, PV_ZTF), (PV_XT, PV_Z1), (WA_X0 - T, PV_Z1)]
# r 10 and not the house 25 on the four pod panels: at 25 the wall's bottom edge
# and the chamfer's top edge each round away from the fold they share and the
# chamfer reads as a loose lip. These are fold edges, not edges a person meets.
P('pelvisC', 'rear_lo',  'x', WA_X0 - T, WA_X0, -WA_Y, WA_Y, PV_ZB, PV_Z1, r_want=10.0)
P('pelvisC', 'front_lo', 'x', WA_X1, WA_X1 + T, -WA_Y, WA_Y, PV_ZB, PV_ZTF, r_want=10.0)
xpanel('pelvisC', 'rear_ch',  PV_XR, PV_Z0, WA_X0, PV_ZB, lambda z: WA_Y, (), -1, r_want=10.0)
xpanel('pelvisC', 'front_ch', PV_XF, PV_Z0, WA_X1, PV_ZB, lambda z: WA_Y, (), +1, r_want=10.0)

# ---- trunk: a chest that is deep at the sternum, on an exposed waist ----
# The chest floor is the waist-yaw output plate (stacked on the pelvis top, the
# pancake). Above it the shell does NOT start: a 38 mm WAIST COLLAR - two side
# plates 166 wide bent up off the floor, open front and back - carries the chest
# and leaves the waist-yaw stack showing between the pelvis (194 wide) and the
# chest (336). That is the H2's exposed waist link and the second of its four
# width reversals; the ratio chest:waist is 336:166 = 2.02.
# From z 150 the shell flares to full width at 230 and holds it to 470 - 8 mm
# above the shoulder-pitch axis, so the shoulders ARE the chest's top corners -
# then chamfers to the neck. In PROFILE the front is two tilted panels breaking
# at the sternum (x 73 -> 100 -> 72) and the rear two breaking at the lumbar
# (x -78 -> -105 -> -92): 151 mm deep at the waist, 205 at the sternum, 164 at
# the deck. Per face: one hemmed window, laid along the panel's own lean.
fl = L('trunkC', 'floor', 'waist_yaw', 'output', -WA_X0 - 0.0, 0.0, 0.0, 0.0, outset=T)
ZF = fl.n1                                              # chest floor top
WY = 80.0                                               # waist half-width (X8-120 is D96)
fl.u0, fl.u1, fl.v0, fl.v1 = -75.0, 70.0, -WY, WY
ZWT = 150.0                                             # waist collar top = chest shell base
ZC0 = 230.0                                             # chest full width from here
ZC1 = 470.0                                             # ... to here (sho pitch axis 462)
CH_Y = 165.0
NK_Y = 115.0                                            # half-width at the deck
ZD = ZHEY - T                                           # deck top = head yaw face (the deck is its stator plate)
# the profile: (x at the waist, at the break, at the deck) and the break height
XF_W, XF_S, XF_D, ZSTERN = 73.0, 100.0, 72.0, 360.0
XR_W, XR_L, XR_D, ZLUMBAR = -78.0, -105.0, -92.0, 330.0
lerp = lambda a, b, t: a + (b - a) * t
def chest_w(z):
    """Chest half-width at height z, at the FRONT/REAR panels' fold line."""
    if z <= ZWT:
        return WY + T
    if z < ZC0:
        return lerp(WY + T, CH_Y + T, (z - ZWT) / (ZC0 - ZWT))
    if z <= ZC1:
        return CH_Y + T
    return lerp(CH_Y + T, NK_Y + T, (z - ZC1) / (ZD - ZC1))
def chest_x(z):
    """(rear, front) fold-line x at height z - the chest's profile."""
    xf = lerp(XF_W, XF_S, (z - ZWT) / (ZSTERN - ZWT)) if z <= ZSTERN else \
         lerp(XF_S, XF_D, (z - ZSTERN) / (ZD - ZSTERN))
    xr = lerp(XR_W, XR_L, (z - ZWT) / (ZLUMBAR - ZWT)) if z <= ZLUMBAR else \
         lerp(XR_L, XR_D, (z - ZLUMBAR) / (ZD - ZLUMBAR))
    return xr, xf
P('trunkS', 'waist', 'y', WY, WY + T, -75.0, 70.0, ZF, ZWT)
# the side plate's outline IS the profile: its front and rear edges follow the
# tilted panels' fold lines (0.5 inside, so the shell laps instead of overlapping).
SIDE_WIN = (-45.0, 40.0, ZC0 + 60.0, ZC1 - 50.0)
SIDE_POLY = [(chest_x(ZC0)[0] + 0.5, ZC0), (chest_x(ZC0)[1] - 0.5, ZC0),
             (XF_S - 0.5, ZSTERN), (chest_x(ZC1)[1] - 0.5, ZC1),
             (chest_x(ZC1)[0] + 0.5, ZC1), (XR_L + 0.5, ZLUMBAR)]
P('trunkS', 'side', 'y', CH_Y, CH_Y + T, 0, 0, 0, 0, poly=SIDE_POLY, slots=[SIDE_WIN])
P('trunkS', 'side_hem', 'y', CH_Y - T, CH_Y, SIDE_WIN[0] - HEM, SIDE_WIN[1] + HEM,
  SIDE_WIN[2] - HEM, SIDE_WIN[3] + HEM, slots=[SIDE_WIN])
def chamfer(tag, y_edge, z_edge, y_far, z_far, n_in, x0, x1):
    """A side chamfer panel from the side plate's edge (y_edge, z_edge) to (y_far, z_far):
    defined flat, hanging from the pivot edge by the slant length, tilted about x."""
    dy, dz = y_far - y_edge, z_far - z_edge
    L_ = math.hypot(dy, dz)
    ang = math.degrees(math.atan2(dy, dz))
    for a_ in (-ang, ang):
        R_ = _rot_axis('x', a_)
        q = _apply(R_, (0.0, y_edge, z_edge), (0.0, y_edge, z_edge + L_))
        if abs(q[1] - y_far) < 0.5 and abs(q[2] - z_far) < 0.5:
            return P('trunkS', tag, 'y', y_edge + n_in, y_edge + n_in + T, x0, x1, z_edge, z_edge + L_,
                     tilt=('x', a_, (0.0, y_edge, z_edge)))
    raise AssertionError('chamfer %s: tilt sign' % tag)
chamfer('waist_chamfer', CH_Y + T / 2.0, ZC0, WY + T / 2.0, ZWT, -T / 2.0,
        chest_x(ZWT)[0] + 0.5, chest_x(ZWT)[1] - 0.5)
chamfer('sho_chamfer',   CH_Y + T / 2.0, ZC1, NK_Y + T / 2.0, ZD, -T / 2.0,
        chest_x(ZD)[0] + 0.5, chest_x(ZD)[1] - 0.5)
ss = L('trunkS', 'sho_stat', 'sho_pitch', 'stator', -52.0, 52.0, ZSP - 52.0, ZSP + 52.0)
P('trunkS', 'outrigger', 'z', ss.v0 - T, ss.v0, -30.0, 30.0, CH_Y, ss.n1)
BRK = (ZWT, ZC0, ZC1, ZD)
fr_lo = xpanel('trunkC', 'front_lo', XF_W, ZWT, XF_S, ZSTERN, chest_w, BRK, +1)
xpanel('trunkC', 'front_up', XF_S, ZSTERN, XF_D, ZD, chest_w, BRK, +1)
rr_lo = xpanel('trunkC', 'rear_lo', XR_W, ZWT, XR_L, ZLUMBAR, chest_w, BRK, -1)
xpanel('trunkC', 'rear_up', XR_L, ZLUMBAR, XR_D, ZD, chest_w, BRK, -1)
# the windows live in the panel's own frame: v is slant up the lean, not z.
def window(pl, tag, v0, v1, half, out):
    win = (-half, half, pl.v0 + v0, pl.v0 + v1)
    pl.slots.append(win)
    P(pl.link, tag, 'x', pl.n0 - T if out > 0 else pl.n1, pl.n0 if out > 0 else pl.n1 + T,
      win[0] - HEM, win[1] + HEM, win[2] - HEM, win[3] + HEM, slots=[win], tilt=pl.tilt)
window(fr_lo, 'front_hem', 62.0, 182.0, 58.0, +1)
window(rr_lo, 'rear_hem',  57.0, 167.0, 58.0, -1)
L('trunkC', 'deck', 'head_yaw', 'stator', XR_D, XF_D, -NK_Y - T, NK_Y + T)

# ---- head: a FACETED HELMET on the pitch output. (H3 programme, item e) ----
# The old head was a 198 mm open cage - four plates, a die on a visible neck
# stack, and the single biggest "H2 vs ours" difference in profile. The H2's is a
# small closed helmet sitting straight on the shoulders. This is that, in bent
# sheet: 156 tall x 170 wide x 167 deep, closed on every face a person can see.
#
# The sagittal profile is ONE convex polygon and every panel is an edge of it, so
# the side plates are that polygon and the front/top/rear plates are the folds
# between its vertices. Seven brake hits round the section:
#   jaw (42,553) -> face (66,588) -> crown_fr (79,681) -> top (38,706)
#   -> crown_bk (-55,706) -> rear (-88,671) -> underside (-88,559) -> jaw
# and the flanks themselves tilt inward 6.3 deg, so the crown is 136 not 170.
#
# EVERY number on the underside is the head_pitch sweep, not taste. The rim
# swings about the pitch axis (z 581.4, 57 over the deck) through the URDF's own
# -20..+49 deg, and the deck is 4 mm below the lowest point it reaches:
#   chin  (42, 553): at +49 deg it drops to z 531.1 - 3.7 over the yaw output plate
#   rear  (-88, 559): at -20 deg it drops to z 530.2 - 5.8 over the deck
# That is why the underside RISES to the rear and why the chin is where it is:
# a lower chin looks better standing still and shears on the deck looking down.
# Resting gap chin-to-deck is 28.6 mm (sweep.py measures it), against 49 for the
# cage - and the cage's
# 198 mm of box above it is now 156.
#
# The cost, recorded: the crown lands at z 709, not the Z_CROWN marker's 771, so
# the robot stands 1738 mm rather than 1800. The head shrank and nothing below it
# grew. Putting it back is a rose_pro_layout change (TORSO / leg proportions),
# not a head one, and a taller neck to reach 1800 would undo this whole pass.
HD_Y = 85.0                              # flank half-width (helmet 170 wide)
HD_YT = 68.0                             # ... narrowing to this at the crown (136 across)
HD_CHIN, HD_FACE0 = (42.0, 553.0), (66.0, 588.0)
HD_FACE1, HD_BROW = (79.0, 681.0), (38.0, 706.0)
HD_NAPE, HD_REAR1, HD_REAR0 = (-55.0, 706.0), (-88.0, 671.0), (-88.0, 559.0)
HD_TOP = HD_BROW[1]


def pan(tag, p0, p1, hw, n_in, **kw):
    """A head panel spanning two points of the sagittal profile: an x-normal
    plate defined flat at p0 and tilted about y until its far edge lands on p1.
    n_in picks which side of the fold line the 3 mm sits on. `hw` is a half-width,
    or a (near, far) pair - a TRAPEZOID blank, which is how the crown panels
    follow the flanks as those fold inward."""
    (xa, za), (xb, zb) = p0, p1
    dx, dz = xb - xa, zb - za
    Ln = math.hypot(dx, dz)
    if isinstance(hw, tuple):
        w0, w1 = hw
        kw['poly'] = [(-w0, za), (w0, za), (w1, za + Ln), (-w1, za + Ln)]
        hw = max(w0, w1)
    return P('head', tag, 'x', xa + n_in, xa + n_in + T, -hw, hw, za, za + Ln,
             tilt=('y', math.degrees(math.atan2(dx, dz)), (xa, 0.0, za)), **kw)


# The flanks are the profile polygon itself, TILTED inward about x so the helmet
# loses width going up - 170 across the jaw, 136 at the crown. One tilt instead
# of a separate crown-chamfer strip a side: a strip 28 mm across the fold is all
# corner at any useful radius, and it left the shell reading as stacked slabs.
HD_POLY = [HD_CHIN, HD_FACE0, HD_FACE1, HD_BROW, HD_NAPE, HD_REAR1, HD_REAR0]
HD_TA = math.degrees(math.atan2(HD_Y - HD_YT, HD_TOP - HD_CHIN[1]))
P('head', 'flank_l', 'y', HD_Y - T, HD_Y, 0, 0, 0, 0, poly=HD_POLY,
  tilt=('x', HD_TA, (0.0, HD_Y, HD_CHIN[1])))
P('head', 'flank_r', 'y', -HD_Y, -HD_Y + T, 0, 0, 0, 0, poly=HD_POLY,
  tilt=('x', -HD_TA, (0.0, -HD_Y, HD_CHIN[1])))
# every other panel is a trapezoid cut to meet the flanks where it lands
hw = lambda z: HD_Y - (HD_Y - HD_YT) * (z - HD_CHIN[1]) / (HD_TOP - HD_CHIN[1]) - 1.0
# the face, tilted 8 deg so the resting gaze is slightly DOWN (DESIGN.md rule 4).
# One hemmed visor slot across it - a raw 3 mm edge at eye height on a machine
# that leans in to look at you is exactly what rule 2 is about - with the stereo
# pair on a plate 20 mm behind it, 66 mm apart (a person's interpupillary
# distance), so the bores read as eyes set INTO the visor rather than as a face.
VIS = (-60.0, 60.0, 616.0, 640.0)
fc = pan('face', HD_FACE0, HD_FACE1, (hw(HD_FACE0[1]), hw(HD_FACE1[1])), -T, slots=[VIS])
FACE_TILT = fc.tilt
P('head', 'visor_hem', 'x', HD_FACE0[0] - 2 * T, HD_FACE0[0] - T, VIS[0] - HEM, VIS[1] + HEM,
  VIS[2] - HEM, VIS[3] + HEM, slots=[VIS], tilt=FACE_TILT)
P('head', 'eyes', 'x', HD_FACE0[0] - 20.0 - T, HD_FACE0[0] - 20.0, -54.0, 54.0, 606.0, 650.0,
  holes=[(-33.0, 628.0, 28.0), (33.0, 628.0, 28.0)], tilt=FACE_TILT)
pan('jaw',      HD_CHIN,  HD_FACE0, (hw(HD_CHIN[1]), hw(HD_FACE0[1])), -T)
pan('crown_fr', HD_FACE1, HD_BROW,  (hw(HD_FACE1[1]), hw(HD_TOP)), -T)
P('head', 'crown_top', 'z', HD_TOP, HD_TOP + T, HD_NAPE[0], HD_BROW[0], -hw(HD_TOP), hw(HD_TOP))
pan('crown_bk', HD_REAR1, HD_NAPE,  (hw(HD_REAR1[1]), hw(HD_TOP)), 0.0)
P('head', 'rear', 'x', HD_REAR0[0], HD_REAR0[0] + T, 0, 0, 0, 0,
  poly=[(-hw(HD_REAR0[1]), HD_REAR0[1]), (hw(HD_REAR0[1]), HD_REAR0[1]),
        (hw(HD_REAR1[1]), HD_REAR1[1]), (-hw(HD_REAR1[1]), HD_REAR1[1])])
# rule 1's 25 mm is for the corners that strike a person. A fold panel 40 mm
# across the fold is all corner at r 25 - it stops meeting its neighbours and the
# shell reads as stacked slabs. The SILHOUETTE keeps 25; the seams get 6.
for _p in PLATES:
    if _p.link == 'head' and _p.tag in ('jaw', 'crown_fr', 'crown_bk', 'visor_hem',
                                        'eyes', 'shelf'):
        _p.r_want = 6.0
# the head's own output plate, and the shelf that carries the shell off it. The
# shelf sits at x -78..-32: inboard of -32 it would foul the pitch body's r 27.5,
# and behind -78 it drops under the rim and into the deck at -20 deg.
# hp_out is trimmed to +/-42 x 56: any bigger and the output plate hangs BELOW the
# jaw line in profile, which is the gap this whole pass exists to close.
ho = L('head', 'hp_out', 'head_pitch', 'output', -42.0, 42.0, ZHEP - 28.0, ZHEP + 28.0, outset=T)
P('head', 'shelf', 'z', 559.0, 559.0 + T, -78.0, -34.0, -78.0, 78.0)

# ---- neck: the yaw output, and a bracket that stands the pitch actuator in it ----
# Both neck actuators are hidden: the X4-36 yaw hangs BELOW the deck inside the
# chest (it was always there), and the X4-36 pitch is inside the helmet - the
# flanks come down to z 555 at x 0 and the body's underside is 553.9. What is
# left showing is 29 mm of bracket, which is a neck.
L('neck', 'hy_out', 'head_yaw', 'output', -28.0, 28.0, -28.0, 28.0, outset=T)
hp = L('neck', 'hp_face', 'head_pitch', 'stator', -30.0, 30.0, ZHEP - 30.0, ZHEP + 30.0)
NK_FOOT = face_n('head_yaw', 'output', outset=T)[1]     # top of the yaw output plate
hp.v0 = NK_FOOT + T                                     # one blank: the stator plate runs down to the foot
# and its FRONT-bottom corner is chamfered off, because that corner is the one
# place the helmet's jaw comes back to. Everything closer to the pitch axis than
# the chin (r 50.7) is safe at any pitch; that corner square would sit at r 59.9,
# inside the annulus the jaw sweeps, and the swept check caught it at 19 deg.
NK_CH = (4.0, 25.6)
hp.poly = [(hp.u0, hp.v0), (NK_CH[0], hp.v0), (hp.u1, hp.v0 + NK_CH[1]), (hp.u1, hp.v1), (hp.u0, hp.v1)]
hp.u0, hp.u1 = min(q[0] for q in hp.poly), max(q[0] for q in hp.poly)
hp.v0, hp.v1 = min(q[1] for q in hp.poly), max(q[1] for q in hp.poly)
P('neck', 'foot', 'z', NK_FOOT, NK_FOOT + T, -30.0, NK_CH[0], face_n('head_pitch', 'stator')[0], -6.0)


HEAD_SHELL = ('flank_l', 'flank_r', 'face', 'visor_hem', 'eyes', 'jaw',
              'crown_fr', 'crown_top', 'crown_bk', 'rear', 'hp_out', 'shelf')
OWNER = {
    ('pelvisS', 'hip_mount'): 'hip_pitch', ('trunkS', 'sho_stat'): 'sho_pitch',
    ('trunkC', 'deck'): 'head_yaw', ('pelvisC', 'top'): 'waist_yaw', ('trunkC', 'floor'): 'waist_yaw',
    ('hipA', 'pitch_face'): 'hip_pitch', ('hipA', 'roll_face'): 'hip_roll',
    ('hipB', 'roll_out'): 'hip_roll',    ('hipB', 'yaw_face'): 'hip_yaw',
    ('thigh', 'yaw_out'): 'hip_yaw',     ('thigh', 'knee_face'): 'knee', ('thigh', 'top'): 'hip_yaw',
    # the knee shroud IS the knee's cover: every facet is tangent to r 88 about
    # the knee axis, 5 mm off the D166 body. Its tilted AABB reaches inside the
    # body (a rotated prism's box does); the plate never does.
    ('thigh', 'knee_shroud-20'): 'knee', ('thigh', 'knee_shroud20'): 'knee',
    ('thigh', 'knee_shroud60'): 'knee',  ('thigh', 'knee_shroud100'): 'knee',
    ('shank', 'web'): 'knee',
    ('crankA', 'bar'): 'ank_mA',         ('crankB', 'bar'): 'ank_mB',
    ('shoA', 'shop_out'): 'sho_pitch',   ('shoA', 'shor_face'): 'sho_roll',
    ('shoB', 'shor_out'): 'sho_roll',    ('shoB', 'elb_face'): 'elb_pitch',
    ('elbA', 'elb_out'): 'elb_pitch',    ('elbA', 'wrist_face'): 'wrist_yaw',
    ('neck', 'hy_out'): 'head_yaw',      ('neck', 'hp_face'): 'head_pitch',
    ('neck', 'foot'): 'head_yaw',
}
OWNER.update({('head', t): 'head_pitch' for t in HEAD_SHELL})
SECOND_OWNER = {
    ('pelvisS', 'hip_mount'): 'hip_pitch', ('trunkS', 'sho_stat'): 'sho_pitch',
    ('trunkC', 'deck'): 'head_yaw', ('pelvisC', 'top'): 'waist_yaw', ('trunkC', 'floor'): 'waist_yaw', ('shank', 'web'): 'ank_pitch', ('shank', 'web'): 'knee'}
SECOND_OWNER.update({('head', t): 'head_pitch' for t in HEAD_SHELL})
SECOND_OWNER[('neck', 'foot')] = 'head_pitch'


THIRD_OWNER = {}   # (referenced by the clash report; kept empty until something needs it)

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
    # the knee shroud: four folds off the knee stator plate's rear edge, 40 deg
    # apart. The fold lines are not axis-aligned (they are chords of r 88), so
    # they are named on the axis they are nearest; no facet carries a hole.
    ('thigh', 'x', -KB - KNEE_R - 5.0 - T / 2.0, ('knee_shroud100',)),
    ('shank', 'x', -KB + 66.0 + T / 2.0,         ('web', 'flange_fr',
                                                  'calf_lap_top', 'calf_lap_bot', 'calf_front')),
    ('shank', 'y', CF_Y - T / 2.0,               ('calf_front', 'calf_in')),
    ('shank', 'z', CF_Z0 - T / 2.0,              ('calf_in', 'calf_taper')),
    ('shank', 'z', FK_Z - T / 2.0,               ('fork_lap_in', 'fork_bridge_in', 'fork_lap_out', 'fork_bridge_out')),
    ('shank', 'y', FORK_IN_Y - T / 2.0,          ('fork_bridge_in', 'fork_in')),
    ('shank', 'y', FORK_OUT_Y + T / 2.0,         ('fork_bridge_out', 'fork_out')),
    ('ankle', 'x', PIN_A_X - 1.5 + T / 2.0,      ('spine', 'jaw_fr')),
    ('ankle', 'x', PIN_B_X - 1.5 + T / 2.0,      ('jaw_rr',)),
    ('foot',  'z', ZS + T + T / 2.0,             ('column', 'col_foot')),
    ('foot',  'y', SOLE_Y0 + T / 2.0,            ('sole', 'wall_in')),
    ('foot',  'y', SOLE_Y1 - T / 2.0,            ('sole', 'wall_out')),
    ('foot',  'x', SOLE_X0 + T + T / 2.0,        ('sole', 'heel')),
    ('shoB',  'y', SB_Y + T / 2.0,               ('shor_out', 'web')),
    ('shoB',  'x', SB_XF + T / 2.0,              ('flange_fr',)),
    ('shoB',  'z', ef.v1 + T / 2.0,              ('elb_face', 'elb_shelf')),
    ('shoB',  'y', SB_Y - T / 2.0,               ('elb_shelf', 'elb_lap')),
    ('elbA',  'x', eo.u1 + T / 2.0,              ('elb_out', 'flange_fr')),
    ('elbA',  'x', eo.u0 - T / 2.0,              ('flange_rr',)),
    ('elbA',  'x', eo.u1 - T / 2.0,              ('wrist_face', 'wtab_fr')),
    ('elbA',  'x', eo.u0 + T / 2.0,              ('wtab_rr',)),
    # the helmet: every fold of the sagittal profile, plus the flank's inward one.
    # The only holes in the shell are the eye bores, 50+ mm off the nearest line.
    ('head',  'z', HD_FACE0[1],                  ('jaw', 'face', 'visor_hem', 'eyes')),
    ('head',  'z', HD_FACE1[1],                  ('face', 'crown_fr')),
    ('head',  'z', HD_TOP,                       ('crown_fr', 'crown_top', 'crown_bk')),
    ('head',  'z', HD_REAR1[1],                  ('crown_bk', 'rear')),
    ('neck',  'z', NK_FOOT + T / 2.0,            ('foot', 'hp_face')),
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
