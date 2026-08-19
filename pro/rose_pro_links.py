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
    """A flat plate: normal axis, thickness span, and in-plane rectangle."""

    def __init__(self, link, tag, normal, n0, n1, u0, u1, v0, v1, holes=()):
        self.link, self.tag, self.normal = link, tag, normal
        self.n0, self.n1 = n0, n1
        self.u0, self.u1, self.v0, self.v1 = u0, u1, v0, v1
        self.holes = list(holes)      # (u, v, dia)

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
        return (self.u1 - self.u0) * (self.v1 - self.v0)

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
PLATES.append(Plate('thigh', 'flange_in',  'y', TH_Y0 - T, TH_Y0, -YAW_R, TH_FRONT + T, TH_FLG_BOT, TH_TOP))
PLATES.append(Plate('thigh', 'flange_out', 'y', TH_Y1, TH_Y1 + T, -YAW_R, TH_FRONT + T, TH_FLG_BOT, TH_TOP))
PLATES.append(Plate('thigh', 'web',        'x', TH_FRONT, TH_FRONT + T, TH_Y0 - T, TH_Y1 + T, TH_FLG_BOT, TH_TOP))
# below the yaw body the web narrows to what the knee stator plate needs
PLATES.append(Plate('thigh', 'web_lo',     'x', TH_FRONT, TH_FRONT + T, 60.0, TH_Y1 + T, -355.0, TH_FLG_BOT))
L('thigh', 'knee_face', 'knee', 'stator', -KB - 46.0, TH_FRONT + T, -350.0, -285.0)
# the knee now hangs 111 mm behind the web's bend line, so the stator plate gets a
# short flange bent down its REAR edge too - a gusset that turns a cantilever into a
# channel. It sits 5 mm clear of the knee body and stays out of the shank's sweep
# because it lives inboard of the shank web (y < 75).
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

# arm
L('shoA', 'shop_out', 'sho_pitch', 'output', -26.0, 26.0, 204.0, 256.0, outset=T)
L('shoA', 'shor_face', 'sho_roll', 'stator', 122.0, 178.0, 139.0, 191.0)
L('shoB', 'shor_out', 'sho_roll', 'output', 124.0, 176.0, 141.0, 189.0, outset=T)
L('shoB', 'elb_face', 'elb_pitch', 'stator', -28.0, 28.0, 14.0, 66.0)
L('elbA', 'elb_out', 'elb_pitch', 'output', -26.0, 26.0, 16.0, 64.0, outset=T)
L('elbA', 'wrist_face', 'wrist_yaw', 'stator', -28.0, 28.0, 122.0, 178.0)
PLATES.append(Plate('hand', 'palm', 'z', -179.0, -176.0, -22.0, 22.0, 128.0, 172.0))

# ---- trunk ----
# The torso shell IS the structure - no separate frame + skin, which would double
# the wall. Side plates sit at y +/-100 (200 mm wide) because the measured torso was
# 27% narrower than human proportion, which is what made it read as a tall slab.
# The hip-pitch stators mount inboard at y=75 on short brackets carried on standoffs.
def lighten(u0, u1, v0, v1, d, du, dv):
    """Staggered, graded lightening cutouts.

    A uniform square grid of identical holes reads as pegboard. Staggering alternate
    columns and shrinking the holes toward the plate edges reads as engineered, and
    is also closer to right: material wants to stay where the load paths run, i.e.
    near the mounting faces at the ends."""
    out = []
    nu = max(int((u1 - u0) // du), 1)
    span_u = nu * du
    ou = u0 + ((u1 - u0) - span_u) / 2.0
    for i in range(nu + 1):
        u = ou + i * du
        stagger = (dv / 2.0) if i % 2 else 0.0
        v = v0 + stagger
        while v <= v1:
            # taper the hole toward the top and bottom of the plate
            t = min(v - v0, v1 - v) / max((v1 - v0) / 2.0, 1.0)
            dd = d * (0.62 + 0.38 * min(t, 1.0))
            if dd > 16.0:
                out.append((u, v, dd))
            v += dv
    return out

L('trunkS', 'hip_mount', 'hip_pitch', 'stator', -50.0, 50.0, -100.0, -10.0)
PLATES.append(Plate('trunkS', 'side', 'y', 100.0, 103.0, -70.0, 65.0, -80.0, 300.0,
                    holes=lighten(-42.0, 37.0, -30.0, 185.0, 62.0, 62.0, 62.0)
                          + [(0.0, 230.0, 62.0)]))
L('trunkS', 'sho_stat', 'sho_pitch', 'stator', -30.0, 30.0, 200.0, 260.0)
PLATES.append(Plate('trunkS', 'outrigger', 'z', 194.0, 197.0, -30.0, 30.0, 100.0, 153.0))

# trunkC is centred and not mirrored: spine, front panel, deck and neck riser.
PLATES.append(Plate('trunkC', 'rear', 'x', -73.0, -70.0, -103.0, 103.0, -80.0, 300.0,
                    holes=lighten(-72.0, 72.0, -30.0, 250.0, 66.0, 62.0, 66.0)))
PLATES.append(Plate('trunkC', 'front', 'x', 65.0, 68.0, -103.0, 103.0, -80.0, 300.0,
                    holes=lighten(-72.0, 72.0, -30.0, 250.0, 66.0, 62.0, 66.0)))
PLATES.append(Plate('trunkC', 'deck', 'z', 300.0, 303.0, -70.0, 65.0, -103.0, 103.0,
                    holes=[(0.0, 0.0, 62.0)]))
PLATES.append(Plate('trunkC', 'riser_l', 'x', 31.0, 34.0, -26.0, 26.0, 300.0, 330.0))
PLATES.append(Plate('trunkC', 'riser_r', 'x', -34.0, -31.0, -26.0, 26.0, 300.0, 330.0))
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
                    holes=[(-27.0, 452.0, 24.0), (27.0, 452.0, 24.0), (0.0, 415.0, 18.0)]))

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
    ('hipA',  'z', (-99.0, -96.0), (-62.0, 32.0), (75.0, 118.0), ('pitch_face', 'roll_face')),
    ('hipB',  'z', (-96.0, -93.0), (-62.0, -37.0), (45.0, 117.0), ('roll_out', 'yaw_face')),
    # split pairs: X-normal lap tab, bent off the output flange about a Z line
    # L-shaped: the upper rect crosses over the foot's output plate, the lower
    # rect is the lap onto ankr_lapear. Both listed so the rule sees both.
    ('ankle', 'x', (-KB - 36.5, -KB - 33.5), (78.0, 130.0), (-588.0, -538.0), ('ankp_out',)),
    ('ankle', 'x', (-KB - 36.5, -KB - 33.5), (108.0, 130.0), (-640.0, -588.0), ()),
    ('shoA',  'x', (-36.5, -33.5), (151.5, 196.0), (128.0, 236.0), ('shop_out',)),
    ('shoB',  'y', (144.0, 147.0), (-34.0, 34.0), (12.0, 100.0), ('elb_face',)),
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
    ('thigh', 'x', WRAP + T / 2.0,       ('knee_face',)),
    ('shank', 'x', SH_FR + T / 2.0,      ('flange_fr',)),
]


def dump_json(path):
    import json
    out = {'thickness': T, 'angle': ANGLE, 'plates': [], 'angles': []}
    for p in PLATES:
        out['plates'].append(dict(link=p.link, tag=p.tag, normal=p.normal,
                                  n0=p.n0, n1=p.n1, u0=p.u0, u1=p.u1,
                                  v0=p.v0, v1=p.v1, holes=p.holes))
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
    print('  holes with insufficient edge margin: %d' % bad)

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
