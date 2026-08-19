"""Structural link definitions for rose-pro, and their clearance / DFM check.

STRUCTURAL SYSTEM: cut-and-bent 3 mm sheet. No angle stock, no machined blocks.

An earlier version used flat plates joined by extruded angle. That was a workaround
for the bend rule (hole centre >= 3t + r = 10.5 mm) and it was too pessimistic: the
rule does not forbid a bend near a big bolt circle, it forces the flange to extend
PAST the circle before bending. pro/rose_pro_bent.py works out, per coupling,
where the two flange planes intersect relative to each bolt circle:

  elbA, neck          one bend, one blank
  hipA, hipB, thigh   two bends about perpendicular axes, one blank + corner relief
  ankle, shoA, shoB   bend line lands INSIDE a bolt circle (only 2 of 6 bolts clear
                      on one side) -> two bent pieces, lapped and bolted

Manufacturing: laser/waterjet blanks + press brake + bolts. Bend radius 1.5 (0.5t),
which 5052-H32 takes without cracking. Flat patterns still need bend allowance.

Frame: trunk origin, x fwd, y left, z up, millimetres. Left side only; right is
mirrored about y.
"""
import itertools
import math

T = 3.0                     # plate thickness
ANGLE = (0.0, 0.0, 0.0)     # retired: no angle stock in the bent-sheet scheme
# proportional rim for bolt holes, but CAP it: a D36 spigot bore was demanding a
# 21 mm rim and inflating plates until they fouled the next actuator.
MIN_EDGE = lambda d: T + min(d / 2.0, 5.0)

# actuator interfaces, from the vendor installation drawings (X-series V4 manual)
IFACE = {
    # od, length, stator BCD, stator hole, output BCD, output hole, pilot, spigot
    'X6-60': dict(od=80.0, ln=67.54, sb=72.5, sh=3.4, ob=44.0, oh=4.5, pilot=32.0, spig=53.0, n=12),
    'X4-36': dict(od=55.0, ln=61.0,  sb=48.0, sh=3.4, ob=26.0, oh=3.4, pilot=35.0, spig=35.0, n=6),
    'X4-10': dict(od=55.0, ln=55.5,  sb=48.0, sh=3.4, ob=26.0, oh=3.4, pilot=35.0, spig=35.0, n=6),
}

HIP_Y, SHO_Y = 75.0, 150.0
KB = 55.0                   # knee-back offset, mm; see KNEE_BACK in rose_pro_layout
SLOT_R = 8.0                # corner radius of rectangular apertures (concave: not a strike edge)
HEM = 8.0                   # width of the hem strip around a window edge

# joint: name, centre, axis, model, body direction, mount-face offset along axis
#   'mid'  -> body straddles the joint, mount face at -half along the axis dir
#   'end'  -> body runs from the joint along bdir, mount face AT the joint
JOINTS = {
    'hip_pitch': dict(c=(0, HIP_Y, -60.0),  ax='y', m='X6-60', bd=-1, mode='end'),
    'hip_roll':  dict(c=(0, HIP_Y, -145.0), ax='x', m='X6-60', bd=-1, mode='mid'),
    'hip_yaw':   dict(c=(0, HIP_Y, -196.0), ax='z', m='X6-60', bd=-1, mode='end'),
    'knee':      dict(c=(-KB, HIP_Y, -310.0), ax='y', m='X6-60', bd=-1, mode='end'),
    'ank_pitch': dict(c=(-KB, HIP_Y, -560.0), ax='y', m='X4-36', bd=-1, mode='end'),
    'ank_roll':  dict(c=(-KB, HIP_Y, -620.0), ax='x', m='X4-36', bd=-1, mode='mid'),
    'sho_pitch': dict(c=(0, SHO_Y, 230.0),  ax='y', m='X4-36', bd=-1, mode='end'),
    'sho_roll':  dict(c=(0, SHO_Y, 165.0),  ax='x', m='X4-36', bd=-1, mode='mid'),
    # body OUTBOARD: pointed inboard it ran into the y=100 torso wall, and moving
    # the arm out far enough to clear would have taken the shoulder span to 336 mm.
    'elb_pitch': dict(c=(0, SHO_Y, 40.0),   ax='y', m='X4-36', bd=+1, mode='end'),
    'wrist_yaw': dict(c=(0, SHO_Y, -115.0),  ax='z', m='X4-36', bd=-1, mode='end'),
    'head_yaw':  dict(c=(0, 0.0, 330.0),    ax='z', m='X4-10', bd=-1, mode='end'),
    'head_pitch':dict(c=(0, 0.0, 390.0),    ax='y', m='X4-10', bd=-1, mode='mid'),
}
AXI = {'x': 0, 'y': 1, 'z': 2}


def body_span(j):
    """(lo, hi) of the actuator body along its own axis, and the mount-face coord."""
    i = AXI[j['ax']]
    c = j['c'][i]
    ln = IFACE[j['m']]['ln']
    if j['mode'] == 'mid':
        lo, hi = c - ln / 2, c + ln / 2
        face = lo if j['bd'] < 0 else hi
    else:
        lo, hi = (c - ln, c) if j['bd'] < 0 else (c, c + ln)
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


class Plate:
    """A flat plate: normal axis, thickness span, and in-plane rectangle.

    `holes` are round bores (u, v, dia). `slots` are rectangular APERTURES
    (u0, u1, v0, v1) with SLOT_R corners - the windows that replaced the round
    lightening holes. A plate whose outline is a slot plus a narrow rim is a hem
    frame: the 180-degree fold around a window edge, modelled as the 3 mm doubled
    strip it becomes (see cobot.py rule 2).
    """

    def __init__(self, link, tag, normal, n0, n1, u0, u1, v0, v1, holes=(), slots=()):
        self.link, self.tag, self.normal = link, tag, normal
        self.n0, self.n1 = n0, n1
        self.u0, self.u1, self.v0, self.v1 = u0, u1, v0, v1
        self.holes = list(holes)      # (u, v, dia)
        self.slots = list(slots)      # (u0, u1, v0, v1)

    def box(self):
        b = [None] * 3
        i = AXI[self.normal]
        ui, vi = [k for k in range(3) if k != i]
        b[i] = (self.n0, self.n1)
        b[ui] = (self.u0, self.u1)
        b[vi] = (self.v0, self.v1)
        return tuple(b)

    def fit(self):
        """Grow the outline until every hole clears the edge by MIN_EDGE.

        Hand-picked plate extents were repeatedly too small for the bolt circle
        they had to carry, so the outline is derived from the holes instead."""
        for (u, v, d) in self.holes:
            m = MIN_EDGE(d)
            self.u0 = min(self.u0, u - d / 2 - m)
            self.u1 = max(self.u1, u + d / 2 + m)
            self.v0 = min(self.v0, v - d / 2 - m)
            self.v1 = max(self.v1, v + d / 2 + m)

    def area(self):
        """Net area: outline minus windows minus bores (what the blank weighs)."""
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
    # Derive that from the body midpoint rather than from bd: for mode='mid' the
    # face is an end of the body, for mode='end' it is the joint itself, and the
    # two give opposite answers for the same bd.
    blo, bhi, _ = body_span(j)
    if face < (blo + bhi) / 2.0:          # body sits above the face -> plate below
        n0, n1 = face - T - outset, face - outset
    else:                                  # body below -> plate above
        n0, n1 = face + outset, face + T + outset
    ui, vi = [k for k in range(3) if k != i]
    cu, cv = j['c'][ui], j['c'][vi]
    bcd = ifc['sb'] if role == 'stator' else ifc['ob']
    dia = ifc['sh'] if role == 'stator' else ifc['oh']
    holes = ring(bcd, cu, cv, dia + 1.1, ifc['n'], clock=180.0 / ifc['n'])
    bore = ifc['spig'] if role == 'stator' else ifc['pilot']
    holes.append((cu, cv, bore + 1.0))
    pl = Plate(link, tag, j['ax'], n0, n1, u0, u1, v0, v1, holes)
    pl.fit()
    return pl


PLATES = []


def L(link, tag, joint, role, u0, u1, v0, v1, outset=0.0):
    PLATES.append(face_plate(link, tag, joint, role, u0, u1, v0, v1, outset))


# ------------------------------------------------------------------ links ----
# hipA: hip pitch OUTPUT -> hip roll STATOR
L('hipA', 'pitch_face', 'hip_pitch', 'output', -62.0, 32.0, -95.0, -21.0, outset=T)
L('hipA', 'roll_face',  'hip_roll',  'stator', 33.0, 117.0, -188.0, -102.0)

# hipB: hip roll OUTPUT -> hip yaw STATOR. The roll output hub protrudes through the
# D54 bore in hipA's roll_face plate, so this plate stands one thickness further out.
L('hipB', 'roll_out', 'hip_roll', 'output', 45.0, 105.0, -184.0, -106.0, outset=T)
# yaw_face is 88 square: with 25 mm corners its corner sits at R51.9 from the yaw
# axis, inside the thigh flanges at 56 (which sweep around it when the hip yaws).
L('hipB', 'yaw_face', 'hip_yaw',  'stator', -44.0, 44.0, 31.0, 119.0)

# thigh: hip yaw OUTPUT -> knee STATOR.
#
# WRAPPED, not tabbed. The thigh is a channel that ENCLOSES the hip-yaw body the
# way a G1 link encloses its actuator: a top plate on the yaw output, two side
# flanges bent down from it, a web bent down from its FRONT edge, and the knee
# stator plate bent from that web. Load path is continuous sheet from the yaw
# output ring to the knee stator ring - no tab, no lap.
#
# Why the channel is open at the BACK and closed at the front, which is the
# opposite of the first instinct: the shank folds up BEHIND the thigh in deep
# knee flexion. sweep.py showed the old shank web punching 22 mm into the yaw
# body at 132 deg, and any thigh structure behind the knee axis is in that same
# path. The front is where nothing ever swings.
#
# Why the flanges sit 56 mm off the yaw axis: the yaw body ROTATES relative to
# the thigh (the thigh is on its output), and the X6 carries a radial connector
# boss out to R50.9. Everything that wraps it must clear that boss all the way
# round: 50.9 + 5 = 56. That makes the thigh 118 wide and 99 deep - the G1's
# thigh is 89 x 180, so we are in the same family.
YAW_R = IFACE['X6-60']['od'] / 2.0
BOSS_R = 50.9                                # X6 radial connector boss, vendor STEP
WRAP = BOSS_R + 5.0                          # inner face of anything wrapping the yaw body
TH_FRONT = WRAP                              # web inner face, x
TH_Y0, TH_Y1 = HIP_Y - WRAP, HIP_Y + WRAP    # flange inner faces, y = 19, 131
TH_TOP = -196.0 + T                          # yaw_out is outset T above the stator plate
TH_YAW_BOT = -196.0 - IFACE['X6-60']['ln']   # yaw body bottom, -263.5
TH_FLG_BOT = -310.0 + YAW_R + 5.0            # flange bottom clears the knee body top by 5

L('thigh', 'yaw_out', 'hip_yaw', 'output', -YAW_R, TH_FRONT + T, TH_Y0 - T, TH_Y1 + T, outset=T)
# The flanges carry a WINDOW each, over the yaw body: "expose what explains". The
# window is a rounded rectangle with a hemmed edge (the hem strip sits on the outer
# face: inward it would come within 2 mm of the connector boss sweep). H2's thigh
# shell has the same move - one long aperture in the side - and it is what makes a
# closed section read as a shell with something inside rather than a box.
TH_WIN = (-25.0, 40.0, -253.0, -205.0)                       # x0, x1, z0, z1
TH_HEMR = (TH_WIN[0] - HEM, TH_WIN[1] + HEM, TH_WIN[2] - HEM, TH_WIN[3] + HEM)
PLATES.append(Plate('thigh', 'flange_in',  'y', TH_Y0 - T, TH_Y0, -YAW_R, TH_FRONT + T, TH_FLG_BOT, TH_TOP, slots=[TH_WIN]))
PLATES.append(Plate('thigh', 'flange_out', 'y', TH_Y1, TH_Y1 + T, -YAW_R, TH_FRONT + T, TH_FLG_BOT, TH_TOP, slots=[TH_WIN]))
PLATES.append(Plate('thigh', 'hem_in',     'y', TH_Y0 - 2 * T, TH_Y0 - T, *TH_HEMR, slots=[TH_WIN]))
PLATES.append(Plate('thigh', 'hem_out',    'y', TH_Y1 + T, TH_Y1 + 2 * T, *TH_HEMR, slots=[TH_WIN]))
PLATES.append(Plate('thigh', 'web',        'x', TH_FRONT, TH_FRONT + T, TH_Y0 - T, TH_Y1 + T, TH_FLG_BOT, TH_TOP))
#
# THE JOG. Below the yaw body the thigh STEPS BACK to the knee instead of bridging
# to it with a 160 mm slab. The first cut carried the web straight down the front
# (x = 56) and bent the knee stator plate off it, so the stator plate had to reach
# from x = 59 back to the knee at x = -55 - 46: a 160 x 65 plate, and the leg's
# widest, deepest section (126 x 162 mm) sat AT THE KNEE. The H2's leg is the
# opposite: 143 x 128 under the hip tapering to 72 x 74 at the knee, with the knee a
# pinch between two cones. Ours cannot taper (axis-aligned sheet) but it can step:
#   web (front, x=56) -> bend at z=-265 -> shelf (z-normal, under the yaw body,
#   running BACK to x=-6) -> bend -> lower web (x=-6, 30 wide) -> bend -> knee
#   stator plate (x -101..-3) -> bend -> rear gusset.
# Four bends in series on one blank, all about y-lines or z-lines. The knee plate
# drops from 160 to 98 long; the section at the knee from 126 x 162 to 98 x 100 -
# that is the G1's S-through-the-hip and the knee-back offset, expressed as folds.
# The shelf top sits 1.5 below the yaw body (which rotates about z, so the gap is
# constant); its front edge is the web's bend line.
TH_LO_X = -6.0                               # lower web inner face (knee body front is -15)
TH_LO_Y0, TH_LO_Y1 = 78.0, 108.0             # lower web: outboard of the knee stator plate
PLATES.append(Plate('thigh', 'shelf',      'z', TH_FLG_BOT - T, TH_FLG_BOT, TH_LO_X, TH_FRONT + T, TH_LO_Y0, TH_Y1 + T))
PLATES.append(Plate('thigh', 'web_lo',     'x', TH_LO_X, TH_LO_X + T, TH_LO_Y0, TH_LO_Y1, -355.0, TH_FLG_BOT - T))
L('thigh', 'knee_face', 'knee', 'stator', -KB - 46.0, TH_LO_X + T, -350.0, -285.0)
# the stator plate gets a short flange bent down its REAR edge too - a gusset that
# turns a cantilever into a channel. It sits 5 mm clear of the knee body and stays
# out of the shank's sweep because it lives inboard of the shank web (y < 75).
PLATES.append(Plate('thigh', 'knee_gus', 'x', -KB - YAW_R - 5.0 - T, -KB - YAW_R - 5.0, 12.0, 78.0, -350.0, -285.0))

# shank: knee OUTPUT -> ankle pitch STATOR.
#
# The knee output plate sits OUTSET one thickness, exactly as hipB documents for
# the hip roll: the stator plate (thigh/knee_face) is on the face, the output hub
# protrudes through its spigot bore, and the output plate stacks outside it. The
# old model put both at 75..78 and they occupied the same 80 x 3 x 72 mm of
# aluminium - the sim never noticed because it only touches the feet, and vcad's
# union hid one inside the other. sweep.py's static check is what caught it.
#
# Because the web moved out one thickness it can no longer be the ankle stator
# plate too, so the ankle stator gets its own plate at 75..78 LAPPED under the
# web and bolted - the split-and-lap pattern the ankle already uses.
#
# One flange, at the FRONT. sweep.py again: a rear flange at 45 mm behind the
# knee axis enters the yaw body's zone above ~110 deg of flexion, no matter how
# far down it starts. The front never rises above the knee axis inside the range.
SH_FR = YAW_R + 5.0                          # front flange inner face, x = 45
sh = face_plate('shank', 'web', 'knee', 'output', -KB - 40.0, -KB + 40.0, -510.0, -295.0, outset=T)
PLATES.append(sh)
PLATES.append(Plate('shank', 'web_fr',    'y', sh.n0, sh.n1, -KB + 40.0, -KB + SH_FR + T, -510.0, -360.0))
PLATES.append(Plate('shank', 'flange_fr', 'x', -KB + SH_FR, -KB + SH_FR + T, 15.0, sh.n1, -515.0, -360.0))
lap = Plate('shank', 'ank_lap', 'y', 75.0, 78.0, -KB - 33.0, -KB + 33.0, -590.0, -480.0)
lap.holes += ring(IFACE['X4-36']['sb'], -KB, -560.0, IFACE['X4-36']['sh'] + 1.1, 6, 30.0)
lap.holes.append((-KB, -560.0, IFACE['X4-36']['spig'] + 1.0))
lap.holes += [(-KB - 20.0, -495.0, 4.5), (-KB + 20.0, -495.0, 4.5)]      # lap bolts, M4
lap.fit()
PLATES.append(lap)
sh.holes += [(-KB - 20.0, -495.0, 4.5), (-KB + 20.0, -495.0, 4.5)]

# ankle: ankle pitch OUTPUT -> ankle roll STATOR
# v0 held at -540: any lower and the plate's corner reaches inside the D55
# ankle roll body sitting 60 mm below it.
L('ankle', 'ankp_out', 'ank_pitch', 'output', -KB - 34.0, -KB + 34.0, -590.0, -534.0, outset=T)
L('ankle', 'ankr_face', 'ank_roll', 'stator', 44.0, 106.0, -651.0, -589.0)
# The ankle is a split pair: a tab bent off ankp_out laps onto ankr_face. The tab
# lands in the slab one thickness outboard of the stator plate - which is exactly
# where the FOOT's roll-output plate also lives (stator plate, then output plate,
# same stack as every pancake joint here). The old tab ran straight down through
# that plate: 3 x 25 x 32 mm of interference, found by sweep.py. So the tab is now
# an L: it crosses the output plate's height only ABOVE it, and drops to the lap
# outboard of it, onto a lap ear added to ankr_face. Two M4s in the lap.
PLATES.append(Plate('ankle', 'ankr_lapear', 'x',
                    JOINTS['ank_roll']['c'][0] - 30.5 - T, JOINTS['ank_roll']['c'][0] - 30.5,
                    104.0, 130.0, -640.0, -589.0,
                    holes=[(119.0, -600.0, 4.5), (119.0, -628.0, 4.5)]))

# foot: ankle roll OUTPUT -> sole. The sole keeps its position RELATIVE to the ankle
# (64 behind, 116 ahead), so with the ankle 55 mm back the heel is now 119 behind
# the trunk origin and the toe 61 ahead - the ankle at ~35% of foot length from the
# heel, roughly where a human's is.
L('foot', 'ankr_out', 'ank_roll', 'output', 54.0, 96.0, -641.0, -599.0, outset=T)
PLATES.append(Plate('foot', 'sole', 'z', -670.0, -667.0, -KB - 64.0, -KB + 116.0, 38.0, 112.0))

# ---- arm ----
# The first cut left the arms as four bare cylinders: the upper arm was a 3 mm strip
# that did not reach the shoulder it was meant to hang from, and the forearm had no
# structure at all between the elbow output and the wrist stator. Both links are
# now CHANNELS, the same move as the thigh. Every dimension below that looks odd
# is a swept radius: sweep.py now rotates the arm through shoulder abduction and
# the full elbow range, and the arm is shaped by what those sweeps allowed.
#
# shoA: shoulder pitch OUTPUT -> shoulder roll STATOR. Split pair, lapped. The lap
# tab used to sit in the slab x -36.5..-33.5 outboard of the roll bolt circle - the
# same slab the roll OUTPUT plate occupies, so the two were 1.5 mm inside each other
# (sweep.py static, "REAL"). It now laps INBOARD of the bolt circle, y 106..121, on
# a stator plate extended inboard to carry it: that is the one sector the rolling
# upper arm never sweeps (abduction swings it outboard and up; the output plate's
# inboard-top corner comes no closer than y 127). Two lobes, like the ankle tab:
# the lower rect is the lap, the upper rect (z 208..236) bends off shop_out.
L('shoA', 'shop_out', 'sho_pitch', 'output', -33.5, 26.0, 204.0, 256.0, outset=T)
L('shoA', 'shor_face', 'sho_roll', 'stator', 106.0, 178.0, 139.0, 191.0)
PLATES[-1].holes += [(113.0, 150.0, 4.5), (113.0, 180.0, 4.5)]        # lap bolts, M4
#
# shoB: shoulder roll OUTPUT -> elbow STATOR. A channel open INBOARD: rear = the roll
# output plate (x -36.5..-33.5); outboard web bent from its outer edge at y 180
# (2.5 clear of the roll body's end), running down past the elbow with a D58
# clearance hole the elbow body passes THROUGH (the elbow body points outboard,
# y 150..211); front flange bent from the web. Section 79.5 x 59 where there was
# a 3 mm strip. Three things here are the shape of a sweep:
#   * The web's top is 15 mm BELOW the roll axis. A corner at (dy=30, dz=-15)
#     swept through 0..97 deg abduction stays outboard of y=180 the whole way
#     (y' = 33.5 cos(th - 27 deg) >= 30); a top at the axis height landed in the
#     shoulder outrigger at 89 deg. The roll body's outboard end shows above it.
#   * The forearm is a bar offset 37 mm from the elbow axis, and through +/-132 deg
#     it sweeps EVERYTHING within r=156 of that axis in its own y-band. So the
#     upper arm's front flange (x 40) and the elbow stator's bridge both live
#     OUTBOARD of y 147, where the forearm is not: the flange at y 148..183, and
#     the stator plate bent at its top (z=75, r=45 from the elbow axis, clear of
#     the 36.6 mm radius the output plate sweeps) into a shelf over the elbow
#     body and a lap tab bolted to the inside of the web.
SB_Y, SB_XF = 180.0, 40.0
L('shoB', 'shor_out', 'sho_roll', 'output', 128.0, SB_Y, 141.0, 185.0, outset=T)
PLATES.append(Plate('shoB', 'web',       'y', SB_Y, SB_Y + T, -36.5, SB_XF, 8.0, 150.0,
                    holes=[(0.0, 40.0, 58.0), (-13.0, 97.0, 4.5), (13.0, 97.0, 4.5)]))
PLATES.append(Plate('shoB', 'flange_fr', 'x', SB_XF, SB_XF + T, 148.0, SB_Y + T, 8.0, 150.0))
L('shoB', 'elb_face', 'elb_pitch', 'stator', -28.5, 28.5, 8.0, 75.0)
PLATES.append(Plate('shoB', 'elb_shelf', 'z', 75.0, 78.0, -28.5, 28.5, 147.0, SB_Y))
PLATES.append(Plate('shoB', 'elb_lap',   'y', SB_Y - T, SB_Y, -28.5, 28.5, 78.0, 110.0,
                    holes=[(-13.0, 97.0, 4.5), (13.0, 97.0, 4.5)]))
#
# elbA: elbow OUTPUT -> wrist STATOR. The elbow output plate is at y 144..147 (the
# pancake stack: stator plate on the face, output plate outset inboard of it, body
# outboard). Its front and rear edges bend INBOARD into two flanges, x +/-36..39,
# y 122..144, hanging the full 176 mm to the wrist - a fork, like the H2's knee
# clevis. Inboard, because outboard of y 147 is where the upper arm keeps its
# flange and stator bridge, and the forearm sweeps its whole y-band; not further
# inboard than 122 because the hip-pitch coupling's tab reaches y 118 at z -96. The wrist
# stator plate (which needs y 118..182 for its ring, so it cannot be one blank
# with the fork) bends tabs UP from its x-edges that lap inside the fork: the one
# place on the arm where nothing sweeps. Section 78 x 22.
L('elbA', 'elb_out', 'elb_pitch', 'output', -36.0, 36.0, 14.0, 64.5, outset=T)
PLATES.append(Plate('elbA', 'flange_fr', 'x',  36.0,  39.0, 122.0, 144.0, -112.0, 64.5,
                    holes=[(133.0, -85.0, 4.5), (133.0, -100.0, 4.5)]))
PLATES.append(Plate('elbA', 'flange_rr', 'x', -39.0, -36.0, 122.0, 144.0, -112.0, 64.5,
                    holes=[(133.0, -85.0, 4.5), (133.0, -100.0, 4.5)]))
L('elbA', 'wrist_face', 'wrist_yaw', 'stator', -36.0, 36.0, 122.0, 178.0)
_wf = PLATES[-1]
PLATES.append(Plate('elbA', 'wtab_fr', 'x',  33.0,  36.0, _wf.v0, _wf.v1, -112.0, -70.0,
                    holes=[(133.0, -85.0, 4.5), (133.0, -100.0, 4.5)]))
PLATES.append(Plate('elbA', 'wtab_rr', 'x', -36.0, -33.0, _wf.v0, _wf.v1, -112.0, -70.0,
                    holes=[(133.0, -85.0, 4.5), (133.0, -100.0, 4.5)]))
PLATES.append(Plate('hand', 'palm', 'z', -179.0, -176.0, -22.0, 22.0, 128.0, 172.0))

# ---- trunk ----
# The torso shell IS the structure - no separate frame + skin, which would double
# the wall. Side plates sit at y +/-100 (200 mm wide) because the measured torso was
# 27% narrower than human proportion, which is what made it read as a tall slab.
# The hip-pitch stators mount inboard at y=75 on short brackets carried on standoffs.
# CHEST OVER WAIST, not one slab. The first cut's torso was 206 x 141 for 380 mm
# with the hip stack continuing the same box downward - half the robot's height was
# one unmodulated prism, and that is what read heavy, not the holes in it. The
# H2 torso is 255 wide at the chest and ~160 at the waist (1.6:1) over a separate
# 216-wide pelvis pod; it has a WAIST. Axis-aligned sheet cannot taper, but it
# can step:
#   waist  z -100..60   y +/-78   x -60..55    (156 x 115) - the pelvis. Its side
#          walls ARE the hip-pitch stator plates, grown to the full wall: no
#          standoffs, no bracket, and the hip output plates (y 78..81) now stand
#          on the OUTSIDE of the pelvis, exactly where the H2 shows its hip rings.
#          It encloses the hip-pitch bodies, which is rule 2 of UNITREE.md.
#   chest  z 60..270    y +/-103  x -73..68    (206 x 141) - as wide as before,
#          210 tall instead of 380, with the deck 40 above the shoulder axis
#          instead of 70 so the shoulders are the top corners, not a hole in the
#          side of a box that keeps going.
#   ledge  z 60..63     the chest floor, a frame whose opening is the waist.
# Chest : waist width 1.32 (H2 1.6); the step is 25 mm a side. A tilted chamfer
# panel would do it better and is the next Plate-class extension.
#
# The round lightening fields are gone. Each chest face carries ONE rounded-
# rectangle window with a hemmed edge (the hem strip on the inside face, so the
# skin stays flush and the window reveal reads as a 6 mm folded edge). The waist
# is closed: a pelvis is a pod.
ZW, ZD = 60.0, 270.0
WA_Y, CH_Y = 75.0, 100.0                     # inner faces of waist wall / chest side
WA_X0, WA_X1 = -57.0, 52.0                   # waist front/rear inner faces
CH_X0, CH_X1 = -70.0, 65.0                   # chest front/rear inner faces

# trunkS is mirrored: waist wall (= hip pitch stator), chest side, shoulder bracket
L('trunkS', 'hip_mount', 'hip_pitch', 'stator', WA_X0 - T, WA_X1 + T, -100.0, ZW)
SIDE_WIN = (-45.0, 40.0, 95.0, 185.0)
PLATES.append(Plate('trunkS', 'side', 'y', CH_Y, CH_Y + T, CH_X0, CH_X1, ZW, ZD,
                    holes=[(0.0, 230.0, 62.0)], slots=[SIDE_WIN]))
PLATES.append(Plate('trunkS', 'side_hem', 'y', CH_Y - T, CH_Y,
                    SIDE_WIN[0] - HEM, SIDE_WIN[1] + HEM, SIDE_WIN[2] - HEM, SIDE_WIN[3] + HEM,
                    slots=[SIDE_WIN]))
L('trunkS', 'sho_stat', 'sho_pitch', 'stator', -30.0, 30.0, 200.0, 260.0)
PLATES.append(Plate('trunkS', 'outrigger', 'z', 197.0, 200.0, -30.0, 30.0, CH_Y, 153.0))

# trunkC is centred and not mirrored: waist front/rear, chest front/rear, ledge,
# deck and neck riser.
PLATES.append(Plate('trunkC', 'rear_lo',  'x', WA_X0 - T, WA_X0, -WA_Y, WA_Y, -100.0, ZW))
PLATES.append(Plate('trunkC', 'front_lo', 'x', WA_X1, WA_X1 + T, -WA_Y, WA_Y, -100.0, ZW))
FR_WIN = (-45.0, 45.0, 110.0, 200.0)
FR_HEMR = (FR_WIN[0] - HEM, FR_WIN[1] + HEM, FR_WIN[2] - HEM, FR_WIN[3] + HEM)
PLATES.append(Plate('trunkC', 'rear',     'x', CH_X0 - T, CH_X0, -CH_Y - T, CH_Y + T, ZW, ZD, slots=[FR_WIN]))
PLATES.append(Plate('trunkC', 'rear_hem', 'x', CH_X0, CH_X0 + T, *FR_HEMR, slots=[FR_WIN]))
PLATES.append(Plate('trunkC', 'front',    'x', CH_X1, CH_X1 + T, -CH_Y - T, CH_Y + T, ZW, ZD, slots=[FR_WIN]))
PLATES.append(Plate('trunkC', 'front_hem','x', CH_X1 - T, CH_X1, *FR_HEMR, slots=[FR_WIN]))
PLATES.append(Plate('trunkC', 'ledge',    'z', ZW, ZW + T, CH_X0, CH_X1, -CH_Y, CH_Y,
                    slots=[(WA_X0, WA_X1, -WA_Y, WA_Y)]))
# the head-yaw body now stands clear above the deck (274.5 vs 273), so the deck
# no longer needs a bore under it
PLATES.append(Plate('trunkC', 'deck', 'z', ZD, ZD + T, CH_X0, CH_X1, -CH_Y - T, CH_Y + T))
PLATES.append(Plate('trunkC', 'riser_l', 'x', 31.0, 34.0, -26.0, 26.0, ZD, 330.0))
PLATES.append(Plate('trunkC', 'riser_r', 'x', -34.0, -31.0, -26.0, 26.0, ZD, 330.0))
L('trunkC', 'hy_stat', 'head_yaw', 'stator', -30.0, 30.0, -30.0, 30.0)

# head: a minimal open cage, not a shell - lightest option and consistent with the
# exposed-skeleton limbs.
HZ0, HZ1 = 392.0, 500.0        # 108 tall vs 78 wide: a cube read as a die
PLATES.append(Plate('head', 'cage_l', 'y', 36.0, 39.0, -48.0, 40.0, HZ0, HZ1,
                    holes=[(-4.0, 446.0, 58.0)]))
PLATES.append(Plate('head', 'cage_r', 'y', -39.0, -36.0, -48.0, 40.0, HZ0, HZ1,
                    holes=[(-4.0, 446.0, 58.0)]))
PLATES.append(Plate('head', 'cage_top', 'z', HZ1, HZ1 + 3.0, -48.0, 40.0, -39.0, 39.0,
                    holes=[(-4.0, 0.0, 44.0)]))
# the face carries the stereo pair - the one element that gives the machine a front
PLATES.append(Plate('head', 'cage_face', 'x', 40.0, 43.0, -39.0, 39.0, HZ0 + 8.0, HZ1 - 8.0,
                    holes=[(-24.0, 452.0, 24.0), (24.0, 452.0, 24.0), (0.0, 415.0, 18.0)]))

# head
L('neck', 'hy_out', 'head_yaw', 'output', -28.0, 28.0, -28.0, 28.0, outset=T)
L('neck', 'hp_face', 'head_pitch', 'stator', -28.0, 28.0, 362.0, 418.0)


OWNER = {
    ('head', 'cage_l'): 'head_pitch', ('head', 'cage_r'): 'head_pitch',
    ('head', 'cage_face'): 'head_pitch', ('head', 'cage_top'): 'head_pitch',
    ('trunkS', 'hip_mount'): 'hip_pitch', ('trunkS', 'sho_stat'): 'sho_pitch',
    ('trunkC', 'hy_stat'): 'head_yaw',
    ('hipA', 'pitch_face'): 'hip_pitch', ('hipA', 'roll_face'): 'hip_roll',
    ('hipB', 'roll_out'): 'hip_roll',    ('hipB', 'yaw_face'): 'hip_yaw',
    ('thigh', 'yaw_out'): 'hip_yaw',     ('thigh', 'knee_face'): 'knee',
    ('shank', 'web'): 'knee',
    ('ankle', 'ankp_out'): 'ank_pitch',  ('ankle', 'ankr_face'): 'ank_roll',
    ('foot', 'ankr_out'): 'ank_roll',
    ('shoA', 'shop_out'): 'sho_pitch',   ('shoA', 'shor_face'): 'sho_roll',
    ('shoB', 'shor_out'): 'sho_roll',    ('shoB', 'elb_face'): 'elb_pitch',
    ('elbA', 'elb_out'): 'elb_pitch',    ('elbA', 'wrist_face'): 'wrist_yaw',
    ('neck', 'hy_out'): 'head_yaw',      ('neck', 'hp_face'): 'head_pitch',
}
# the shank plate legitimately touches BOTH actuators it spans
SECOND_OWNER = {
    ('head', 'cage_l'): 'head_pitch', ('head', 'cage_r'): 'head_pitch',
    ('head', 'cage_face'): 'head_pitch', ('head', 'cage_top'): 'head_pitch',
    ('trunkS', 'hip_mount'): 'hip_pitch', ('trunkS', 'sho_stat'): 'sho_pitch',
    ('trunkC', 'hy_stat'): 'head_yaw',('shank', 'web'): 'ank_pitch'}


ANGLES = []   # retired with the angle stock

# Bend tabs: the intermediate flange of a two-bend part, or the lap flange of a
# split pair. Each is an ordinary plate; what makes it a bend is that it shares an
# edge with its neighbour. BENDS records the lines so the rule can be checked.
BEND_TABS = [
    # link, tab normal, thickness span, u range, v range, bends_to (plate tags)
    # A tab listed with two targets is the middle flange of a TWO-BEND blank.
    # A tab with one target bends from that flange and LAPS (bolts) to the other -
    # used where the direct bend line would land inside a bolt circle.
    # hipA's tab bends off pitch_face, so it starts at y=78 (the plate's inner
    # face), not 75: at 75 it sat inside the hip stator plate's slab.
    ('hipA',  'z', (-99.0, -96.0), (-62.0, 32.0), (78.0, 118.0), ('pitch_face', 'roll_face')),
    # hipB's tab bends off the BOTTOM of roll_out (z=-184); it was recorded at
    # z=-96, a hundred millimetres up, punching through the hip stator and output
    # plates - two of sweep.py's four REAL overlaps. Where it belongs it is 3 mm
    # above the thigh's yaw_out plate, which rotates about z and never closes that.
    ('hipB',  'z', (-187.0, -184.0), (-62.0, -39.8), (45.0, 105.0), ('roll_out', 'yaw_face')),
    # split pairs: X-normal lap tab, bent off the output flange about a Z line
    # L-shaped: the upper rect crosses over the foot's output plate, the lower
    # rect is the lap onto ankr_lapear. Both listed so the rule sees both.
    ('ankle', 'x', (-KB - 36.5, -KB - 33.5), (78.0, 130.0), (-588.0, -538.0), ('ankp_out',)),
    ('ankle', 'x', (-KB - 36.5, -KB - 33.5), (108.0, 130.0), (-640.0, -588.0), ()),
    # shoA: two lobes. Upper bends off shop_out's rear edge (z 208..236 sits above
    # everything the rolling upper arm sweeps); lower is the lap onto shor_face's
    # inboard extension, y 106..121, with two M4s.
    ('shoA',  'x', (-36.5, -33.5), (106.0, 156.0), (208.0, 236.0), ('shop_out',)),
    ('shoA',  'x', (-36.5, -33.5), (106.0, 121.0), (128.0, 208.0), ()),
]
ANGLE_MASS = (ANGLE[0] + ANGLE[1] - ANGLE[2]) * ANGLE[2] * 2.70e-3 / 1000.0  # kg per mm

# Bend LINES for the wrapped links. Unlike BEND_TABS these add no plate: the plate
# bent FROM already exists in the table. (link, axis the line runs along... no -
# axis the line is measured on, coordinate, plates governed).
#   thigh: side flanges and web bend down from the top plate -> lines at z=-191.5
#          knee_face bends from the web -> line at x = TH_FRONT + T/2
#   shank: front flange bends from web_fr -> line at x = SH_FR + T/2
BENDS = [
    ('thigh', 'z', -196.0 + T + T / 2.0, ('flange_in', 'flange_out', 'web')),
    ('thigh', 'z', TH_FLG_BOT - T / 2.0,  ('shelf',)),            # shelf off the web's bottom
    ('thigh', 'x', TH_LO_X + T / 2.0,     ('web_lo', 'knee_face')),  # web_lo off the shelf; knee_face off web_lo
    ('thigh', 'x', -KB - YAW_R - 5.0 - T / 2.0, ('knee_gus',)),
    ('shank', 'x', SH_FR + T / 2.0,      ('flange_fr',)),
    ('shoB',  'y', SB_Y + T / 2.0,       ('shor_out', 'web')),     # web off shor_out's outboard edge
    ('shoB',  'x', SB_XF + T / 2.0,      ('flange_fr',)),          # flange off the web's front edge
    ('shoB',  'z', 75.0 + T / 2.0,       ('elb_face', 'elb_shelf')),  # shelf off the stator plate's top
    ('shoB',  'y', SB_Y - T / 2.0,       ('elb_shelf', 'elb_lap')),
    ('elbA',  'x',  37.5,                ('elb_out', 'flange_fr')),  # fork off the output plate's edges
    ('elbA',  'x', -37.5,                ('flange_rr',)),
    ('elbA',  'x',  34.5,                ('wrist_face', 'wtab_fr')), # wrist tabs off the stator plate
    ('elbA',  'x', -34.5,                ('wtab_rr',)),
]


def dump_json(path):
    import json
    out = {'thickness': T, 'angle': ANGLE, 'plates': [], 'angles': []}
    for p in PLATES:
        out['plates'].append(dict(link=p.link, tag=p.tag, normal=p.normal,
                                  n0=p.n0, n1=p.n1, u0=p.u0, u1=p.u1,
                                  v0=p.v0, v1=p.v1, holes=p.holes, slots=p.slots))
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
