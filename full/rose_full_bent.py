"""Can each rose-full coupling be ONE cut-and-bent sheet part?

Earlier I claimed bent sheet was infeasible here. That was too strong. The rule
(hole centre >= 3t + r from the bend line) does not forbid a bend near a large bolt
circle - it forces the flange to extend PAST the circle before bending. The real
question, per coupling, is where the two flange planes intersect relative to each
bolt circle:

  * bend line clear of both circles      -> one bend, one blank. Done.
  * bend line outside one circle but the
    other flange must reach across it    -> route via an intermediate tab: two bends
                                            about perpendicular axes, one blank, with
                                            a corner relief notch. Standard practice.
  * bend line lands INSIDE a bolt circle -> a single blank cannot present the whole
                                            circle. Either drop to the bolts on one
                                            side, or split into two bent parts lapped
                                            and bolted.

The hip roll pair is the hard case: its stator circle is centred at y=75, which is
exactly the plane the hip pitch output flange lives in, so the bend line passes
through the middle of the circle.
"""
import math

T, R_BEND = 3.0, 1.5
MIN_BEND = 3 * T + R_BEND          # 10.5 mm, hole centre to bend line
AXI = {'x': 0, 'y': 1, 'z': 2}

IFACE = {
    'X6-60': dict(od=80.0, ln=67.54, sb=72.5, ob=44.0, n=12, sM=3, oM=4),
    'X4-36': dict(od=55.0, ln=61.0, sb=48.0, ob=26.0, n=6, sM=3, oM=3),
    'X4-10': dict(od=55.0, ln=55.5, sb=48.0, ob=26.0, n=6, sM=3, oM=3),
}
HIP_Y, SHO_Y = 75.0, 150.0
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


def face_of(j):
    i = AXI[j['ax']]
    c = j['c'][i]
    ln = IFACE[j['m']]['ln']
    if j['mode'] == 'mid':
        lo, hi = c - ln / 2, c + ln / 2
        return lo if j['bd'] < 0 else hi, lo, hi
    lo, hi = (c - ln, c) if j['bd'] < 0 else (c, c + ln)
    return c, lo, hi


def analyse(name, da, db):
    ja, jb = JOINTS[da], JOINTS[db]
    ia, ib = AXI[ja['ax']], AXI[jb['ax']]
    fa, _, _ = face_of(ja)
    fb, _, _ = face_of(jb)
    ifa, ifb = IFACE[ja['m']], IFACE[jb['m']]

    # flange A lies in the plane {axis_a = fa}; flange B in {axis_b = fb}.
    # They intersect along the line where BOTH hold - the only possible bend line,
    # running along the third axis.
    third = [k for k in range(3) if k not in (ia, ib)][0]

    def clearance(j, ifc, role, plane_axis, plane_at):
        """Min distance from this flange's bolts to the bend line."""
        bcd = ifc['ob'] if role == 'output' else ifc['sb']
        n = ifc['n']
        i = AXI[j['ax']]
        ui, vi = [k for k in range(3) if k != i]
        cu, cv = j['c'][ui], j['c'][vi]
        # the bend line sits at plane_at along plane_axis, within this flange's plane
        pa = plane_axis
        best = 1e9
        signed = []
        for k in range(n):
            a = math.radians(k * 360.0 / n + 180.0 / n)
            p = {ui: cu + (bcd / 2) * math.cos(a), vi: cv + (bcd / 2) * math.sin(a)}
            d = p[pa] - plane_at
            signed.append(d)
            best = min(best, abs(d))
        usable = sum(1 for d in signed if abs(d) >= MIN_BEND)
        one_side = max(sum(1 for d in signed if d >= MIN_BEND),
                       sum(1 for d in signed if -d >= MIN_BEND))
        return best, usable, one_side, n

    # for flange A the bend line is at (axis_b = fb); for flange B it is at (axis_a = fa)
    ba, ua, sa, na = clearance(ja, ifa, 'output', ib, fb)
    bb, ub, sb_, nb = clearance(jb, ifb, 'stator', ia, fa)

    if ba >= MIN_BEND and bb >= MIN_BEND:
        verdict, detail = 'ONE BEND', 'bend line clear of both circles'
    elif ba >= MIN_BEND and sb_ >= 4:
        verdict, detail = 'TWO BENDS', 'route via a tab; corner relief needed'
    elif sa >= 4 and sb_ >= 4:
        verdict, detail = 'TWO BENDS', 'route via a tab; corner relief needed'
    else:
        verdict = 'SPLIT'
        detail = 'bend line crosses a bolt circle: %d/%d and %d/%d bolts on one side' \
                 % (sa, na, sb_, nb)
    return dict(name=name, ba=ba, bb=bb, sa=sa, na=na, sb=sb_, nb=nb,
                verdict=verdict, detail=detail, third='xyz'[third])


if __name__ == '__main__':
    print('%-7s %8s %8s  %-10s %s' %
          ('coupl', 'A gap', 'B gap', 'verdict', 'note'))
    for name, da, db in COUPLINGS:
        r = analyse(name, da, db)
        print('%-7s %7.1f  %7.1f  %-10s %s'
              % (r['name'], r['ba'], r['bb'], r['verdict'], r['detail']))
    print('\nrule: hole centre >= %.1f mm from the bend line (3t + r, t=%.0f r=%.1f)'
          % (MIN_BEND, T, R_BEND))
