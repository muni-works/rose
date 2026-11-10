"""J3 pose cap (v10, Cam 2026-10-20: keep the RS02 at J3, cap the poses in software).

J3 (upper-arm roll) carries gravity torque only when the forearm + hand leave the vertical plane
through the upper arm. j3_torque() takes the static torque about the J3 axis from the model (every
mass outboard of J3 at its link-frame centroid, plus the payload at the jaw tips); j3_ok() rejects
a pose above CAP = RS02 continuous (4.4 N.m, an estimate) x MARGIN.

    python3 arm/pose_cap.py      (ROSE_ARM=v10 set for you) -> the envelope table

Frames: q in radians (rose_fork convention). R0 = the arm-to-world rotation (identity on the desk;
render_v10.body_R0(s) on the humanoid), gravity is world -z.
"""
import math
import os
import sys

os.environ.setdefault('ROSE_ARM', 'v10')
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..'))
import rose_fork as F
import fork_checks as C
from rose_arm_geom import mv, sub, cross, dot, I3

G = 9.81
MARGIN = 0.78                      # v10.3: the skin derates J3 by ~20 % (thermal_v10.py): 3.43 N.m sits under the derated 3.50
CAP = F.RATING[id(F.J3M)][1] * MARGIN
OUTBOARD = ('upperd', 'fore', 'yoke', 'wrist', 'tool', 'harness')


def j3_torque(q, payload=0.0, R0=I3):
    """|static torque| about J3 (N.m) at pose q with `payload` kg at the jaw tips."""
    T = F.fk(q)
    w = lambda link, p: mv(R0, F.xf(T, link, p))
    o = w('upperd', F.ORIGIN['upperd'])
    ax = mv(R0, mv(T['upperd'][0], (1.0, 0.0, 0.0)))
    tau = 0.0
    pts = [(l if l != 'harness' else 'fore', m, x) for l, m, x in C.masses() if l in OUTBOARD]
    pts.append(('tool', payload, C.flange_x()))
    for link, m, x in pts:
        if link == 'fore' and x < F.UPPER:      # harness lump: only its outboard share counts
            continue
        r = sub(w(link, (x, 0.0, F.Z_SH)), o)
        tau += dot(cross(r, (0.0, 0.0, -m * G)), ax)
    return abs(tau)


def j3_ok(q, payload=0.0, R0=I3):
    return j3_torque(q, payload, R0) <= CAP


def envelope():
    """Fraction of a pose grid that passes, and the limiting poses, at 0 / 0.5 / 1.0 kg."""
    rad = math.radians
    print('J3 cap %.2f N.m (RS02 %.1f N.m continuous x %.0f %%)' % (CAP, F.RATING[id(F.J3M)][1], MARGIN * 100))
    # the worst case is the one that sizes everything: upper arm level, forearm level, swung sideways
    print('\nDESK, upper arm level, elbow at 90 (forearm hanging), J3 swings the forearm out of the plane: torque vs J3')
    for kg in (0.0, 0.5, 1.0):
        row = []
        for a in (0, 15, 30, 45, 60, 90):
            row.append('%3d deg %4.1f' % (a, j3_torque({'J3_upper_roll': rad(a), 'J4_elbow': rad(90), 'J2_sho_pitch': 0.0}, kg)))
        lim = max([a for a in range(0, 91) if j3_ok({'J3_upper_roll': rad(a), 'J4_elbow': rad(90)}, kg)] or [None])
        print('  %.1f kg: %s   ->  J3 out of plane up to %s deg' % (kg, ',  '.join(row), lim))
    print('\nHUMANOID (left arm), elbow at 90 (forearm forward), J3 swings the forearm inward/outward;')
    print('rows: J1 shoulder pitch raises the upper arm forward from hanging')
    import render_v10 as RV
    R0 = tuple(tuple(r) for r in RV.body_R0(1))
    for kg in (0.0, 0.5, 1.0):
        cells = []
        for j1 in (0, -45, -90):
            lim = max([a for a in range(0, 91) if j3_ok({'J1_yaw': rad(j1), 'J3_upper_roll': rad(90 - a), 'J4_elbow': rad(-90)}, kg, R0)
                       and j3_ok({'J1_yaw': rad(j1), 'J3_upper_roll': rad(90 + a), 'J4_elbow': rad(-90)}, kg, R0)] or [None])
            cells.append('arm raised %2d deg: forearm +/-%s deg off straight-ahead' % (-j1, lim))
        print('  %.1f kg: %s' % (kg, ';  '.join(cells)))
    print('\npose grid: J2 -150..0, J3 -90..90, J4 -120..120 (step 15), J6 0:')
    for kg in (0.0, 0.5, 1.0):
        n = ok = 0
        planar = 0
        for j2 in range(-150, 1, 15):
            for j3 in range(-90, 91, 15):
                for j4 in range(-120, 121, 15):
                    q = {'J2_sho_pitch': rad(j2), 'J3_upper_roll': rad(j3), 'J4_elbow': rad(j4)}
                    n += 1
                    if j3_ok(q, kg):
                        ok += 1
                        planar += j3 == 0
        print('  %.1f kg: %3.0f %% of poses pass' % (kg, 100.0 * ok / n))


if __name__ == '__main__':
    envelope()
