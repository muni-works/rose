"""Machined actuator couplings for rose-pro.

Replaces the flat-plate + angle-stock scheme. That was a workaround for the sheet
bend rule, and it had a real defect: the plates located on bolt CLEARANCE, so every
joint's concentricity depended on how the bolts happened to pull up. These actuators
ship precision registers that were going unused:

  X6-60   D53 -0.010/-0.029 spigot, D5 dowel, D32 +0.025 output pilot
  X4-36   D35 -0.009/-0.025 spigot, D3 dowels, D26 output circle

A coupling is one machined 6061 part per joint pair. It presents two faces:
  OUTPUT face - bolts to the driving actuator's rotating hub, piloted on its boss
  STATOR face - bolts to the driven actuator's housing, piloted on its spigot
joined by a body that carries torque between them.

Design rules applied:
  * every face gets a counterbored register, so the part locates on metal, not bolts
  * socket heads are counterbored flush - a proud head is an interference waiting
    to happen in a joint cluster this tight
  * WALL is 8 mm at the flanges: these are loaded in bending off a 60 N.m joint, and
    3 mm plate in single bending was the weakest thing in the old scheme
  * the corner carries a gusset; an unribbed L is a hinge
  * lightening pockets go in the low-stress middle of each flange, never inside the
    bolt circle
"""
import itertools
import math

# Flange thickness is set by a STACK, not by feel: a register recess on the mounting
# face and a socket-head counterbore on the outer face eat into the same wall. Need
# REG_DEPTH + counterbore + 3 mm of web under the head. At 8 mm the X6 output flange
# left 1.1 mm, which is not a part, it is a washer with ambitions.
WALL_BY = {'X6-60': 12.0, 'X4-36': 10.0, 'X4-10': 10.0}
REG_DEPTH = 2.0       # register counterbore depth - enough to locate, no deeper
MIN_WEB = 3.0         # material remaining under a bolt head
GUSSET = 6.0          # corner rib thickness
CLEAR = 0.4           # register diametral clearance (register is h7, bore H8-ish)

# M-size -> (clearance dia, socket head dia, counterbore depth)
SHCS = {3: (3.4, 6.0, 3.4), 4: (4.5, 7.4, 4.4), 5: (5.5, 9.0, 5.4)}

IFACE = {
    'X6-60': dict(od=80.0, ln=67.54, sb=72.5, sM=3, ob=44.0, oM=4,
                  pilot=32.0, spig=53.0, n=12, dowel=5.0, dowel_bcd=38.5),
    # X4 output boss is D35 h7 with the 6 x M3 ON it at D26 - bolts INSIDE the
    # register. The stator side has no spigot: it locates on the D3 dowels and the
    # D48 +/-0.02 precision PCD, which is why that PCD is toleranced so tightly.
    'X4-36': dict(od=55.0, ln=61.0, sb=48.0, sM=3, ob=26.0, oM=3,
                  pilot=35.0, spig=0.0, n=6, dowel=3.0, dowel_bcd=30.0),
    'X4-10': dict(od=55.0, ln=55.5, sb=48.0, sM=3, ob=26.0, oM=3,
                  pilot=35.0, spig=0.0, n=6, dowel=3.0, dowel_bcd=30.0),
}

HIP_Y, SHO_Y = 75.0, 150.0
AXI = {'x': 0, 'y': 1, 'z': 2}

JOINTS = {
    'hip_pitch': dict(c=(0, HIP_Y, -60.0),  ax='y', m='X6-60', bd=-1, mode='end'),
    'hip_roll':  dict(c=(0, HIP_Y, -145.0), ax='x', m='X6-60', bd=-1, mode='mid'),
    'hip_yaw':   dict(c=(0, HIP_Y, -196.0), ax='z', m='X6-60', bd=-1, mode='end'),
    'knee':      dict(c=(0, HIP_Y, -310.0), ax='y', m='X6-60', bd=-1, mode='end'),
    'ank_pitch': dict(c=(0, HIP_Y, -560.0), ax='y', m='X4-36', bd=-1, mode='end'),
    'ank_roll':  dict(c=(0, HIP_Y, -620.0), ax='x', m='X4-36', bd=-1, mode='mid'),
    'sho_pitch': dict(c=(0, SHO_Y, 230.0),  ax='y', m='X4-36', bd=-1, mode='end'),
    'sho_roll':  dict(c=(0, SHO_Y, 165.0),  ax='x', m='X4-36', bd=-1, mode='mid'),
    'elb_pitch': dict(c=(0, SHO_Y, 40.0),   ax='y', m='X4-36', bd=+1, mode='end'),
    'wrist_yaw': dict(c=(0, SHO_Y, -115.0), ax='z', m='X4-36', bd=-1, mode='end'),
    'head_yaw':  dict(c=(0, 0.0, 330.0),    ax='z', m='X4-10', bd=-1, mode='end'),
    'head_pitch':dict(c=(0, 0.0, 390.0),    ax='y', m='X4-10', bd=-1, mode='mid'),
}

# coupling: name, driving joint (we bolt to its OUTPUT), driven joint (we carry its
# STATOR), and how far each flange reaches past its bolt circle toward the corner
COUPLINGS = [
    ('hipA',  'hip_pitch', 'hip_roll'),
    ('hipB',  'hip_roll',  'hip_yaw'),
    ('thigh', 'hip_yaw',   'knee'),
    ('ankle', 'ank_pitch', 'ank_roll'),
    ('shoA',  'sho_pitch', 'sho_roll'),
    ('shoB',  'sho_roll',  'elb_pitch'),
    ('elbA',  'elb_pitch', 'wrist_yaw'),
    ('neck',  'head_yaw',  'head_pitch'),
]


def body_span(j):
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


def outward(j):
    """+1 if the coupling sits at higher coordinate than the mount face."""
    lo, hi, face = body_span(j)
    return -1 if face < (lo + hi) / 2.0 else +1


class Flange:
    """One mounting face of a coupling."""

    def __init__(self, joint, role, reach):
        j = JOINTS[joint]
        ifc = IFACE[j['m']]
        self.joint, self.role = joint, role
        self.ax = j['ax']
        self.i = AXI[j['ax']]
        _, _, face = body_span(j)
        s = outward(j)
        self.s = s
        self.wall = WALL_BY[j['m']]
        self.n0, self.n1 = ((face, face + self.wall) if s > 0
                            else (face - self.wall, face))
        self.face = face
        ui, vi = [k for k in range(3) if k != self.i]
        self.cu, self.cv = j['c'][ui], j['c'][vi]
        if role == 'output':
            self.bcd, self.M, self.reg = ifc['ob'], ifc['oM'], ifc['pilot']
        else:
            self.bcd, self.M, self.reg = ifc['sb'], ifc['sM'], ifc['spig']
        self.n = ifc['n']
        self.dowel, self.dowel_bcd = ifc['dowel'], ifc['dowel_bcd']
        # outline: bolt circle + head clearance + wall, extended toward the corner
        hd = SHCS[self.M][1]
        self.rad = self.bcd / 2 + hd / 2 + 5.0
        self.reach = reach
        # through bore for cable routing; the coupling is an annulus, not a disc
        if role == 'output':
            self.bore = 33.0 if self.M == 4 else 18.0
        else:
            self.bore = 54.0 if ifc['spig'] > 0 else 36.0

    def bolts(self):
        cl, hd, cbd = SHCS[self.M]
        out = []
        for k in range(self.n):
            a = math.radians(k * 360.0 / self.n + 180.0 / self.n)
            out.append((self.cu + (self.bcd / 2) * math.cos(a),
                        self.cv + (self.bcd / 2) * math.sin(a), cl, hd + 0.6, cbd))
        return out

    def register(self):
        return (self.cu, self.cv, self.reg + CLEAR, REG_DEPTH)

    def volume(self):
        """Rough flange volume for mass accounting (disc + reach arm), mm^3."""
        # Full-thickness material only where it is needed: a local boss at each bolt
        # (the counterbore has to live somewhere) over a thinner continuous web.
        # A solid annulus at 12 mm was costing ~60% more for no structural gain.
        cl, hd, cbd = SHCS[self.M]
        web_t = max(cbd + MIN_WEB - REG_DEPTH, 5.0)
        ring = math.pi * (self.rad ** 2 - (self.bore / 2) ** 2) * web_t
        boss_r = hd / 2 + 3.0
        bosses = self.n * math.pi * boss_r ** 2 * (self.wall - web_t)
        arm = 2 * self.rad * self.reach * web_t * 0.6           # ribbed
        holes = self.n * math.pi * (cl / 2) ** 2 * self.wall
        return ring + bosses + arm - holes


def corner(fa, fb):
    """Where the two flange planes meet, and the gusset that ties them."""
    return dict(a_plane=(fa.n0, fa.n1), b_plane=(fb.n0, fb.n1))


def report():
    print('%-7s %-11s %-11s %7s %7s %8s' %
          ('coupl', 'output face', 'stator face', 'boltsA', 'boltsB', 'mass g'))
    total = 0.0
    rows = []
    for name, da, db in COUPLINGS:
        fa = Flange(da, 'output', 30.0)
        fb = Flange(db, 'stator', 30.0)
        # body: prism joining the two flange planes
        sep = abs(JOINTS[da]['c'][AXI[JOINTS[db]['ax']]] -
                  JOINTS[db]['c'][AXI[JOINTS[db]['ax']]]) if False else 0.0
        vol = fa.volume() + fb.volume()
        # connecting web between the flange planes
        pa = (fa.n0 + fa.n1) / 2
        pb = (fb.n0 + fb.n1) / 2
        span = math.dist(JOINTS[da]['c'], JOINTS[db]['c'])
        web = span * 55.0 * max(fa.wall, fb.wall) * 0.55   # webbed, not solid
        gus = 0.5 * 40.0 * 40.0 * GUSSET * 2
        vol += web + gus
        g = vol * 2.70e-3
        total += g
        rows.append((name, fa, fb, g))
        print('%-7s %-11s %-11s %4d x M%d %4d x M%d %8.0f' %
              (name, da, db, fa.n, fa.M, fb.n, fb.M, g))
    print('%-43s %8.0f g  per side' % ('', total))
    return rows


def checks(rows):
    print('\n=== interface checks ===')
    bad = 0
    for name, fa, fb, _ in rows:
        for f, tag in ((fa, 'output'), (fb, 'stator')):
            cl, hd, cbd = SHCS[f.M]
            web = f.wall - REG_DEPTH - cbd
            if web < MIN_WEB:
                print('  %s %s: only %.1f mm web under the M%d head (need %.1f)'
                      % (name, tag, web, f.M, MIN_WEB))
                bad += 1
            if f.reg <= 0:
                continue            # dowel-located face, no register to check
            if f.bcd / 2 < f.reg / 2:
                # bolts sit INSIDE the register recess (X4 output boss) - heads must fit
                if f.bcd / 2 + hd / 2 > f.reg / 2 - 1.0:
                    print('  %s %s: M%d heads on D%.1f will not fit inside the D%.1f register'
                          % (name, tag, f.M, f.bcd, f.reg))
                    bad += 1
            else:
                # bolts outside the register - need land between register wall and hole
                if f.bcd / 2 - cl / 2 < f.reg / 2 + 1.5:
                    print('  %s %s: no land between D%.1f register and D%.1f bolt circle'
                          % (name, tag, f.reg, f.bcd))
                    bad += 1
            # flange must be wide enough to hold the bolt heads
            if f.rad < f.bcd / 2 + hd / 2 + 3.0:
                print('  %s %s: flange r%.1f too small for M%d heads' % (name, tag, f.rad, f.M))
                bad += 1
    print('  interface faults: %d' % bad)
    return bad


if __name__ == '__main__':
    checks(report())
