"""Kinematic layout for rose pro - the buildable humanoid on MyActuator X-series.

1800 mm cut (2026-08-19). The first rose pro was a 1173 mm machine on X6-60 /
X4-36; this is the same architecture re-laid at the height the line always
meant (pro 1800, max 2000) on the actuators that height demands - see SIZING.md:
hip pitch and knee on X15-450, hip roll X12-320, hip yaw X10-200, ankle and
shoulder pitch X8-120, the rest X6-60 / X4-36. Every position below is derived
from HEIGHT, the H2's proportions (UNITREE.md "H2, measured") and the actuator
envelopes; nothing is a bare number any more.

ARCHITECTURE, and the one thing that changed with the actuators: the hip is no
longer a coaxial column. At 1173 the three hip bodies (D80) stacked on one
y = 75 line. At 1800 the hip-pitch body is D166 and its twin sits end-to-end
with it at the centre, so the pitch axis lands at y = 95; and a D124 roll body
cannot nest under a D166 pitch body at the same y without 150 mm of z between
them. So the roll / yaw / knee / ankle column moves OUTBOARD of the pitch body
to y = LEG_Y = 165 - which is what the H2 does (pitch 80, roll 144, yaw 134,
knee 157) and what the G1's curved hip-roll shell sweeping "down and back" is
covering. The hip-pitch link becomes a real part: the pitch output at y 95
carrying the roll stator 70 mm outboard. Stance at the knee 330 (H2 314).

Verified actuator envelopes (vendor 2D drawings, ~/Developer/myactuator_assets):
  X15-450  D166 x 69 (+3 flange)  450 Nm peak  3.50 kg  20.25:1  72 V   J_m 31.6 kg.cm2
  X12-320  D124 x 85 (+3)         320          2.37     20:1     48 V   12.9
  X10-200  D124 x 63.5 (+2)       200          1.64     20:1     48 V   -
  X8-120   D96 x 76 (+3)          120          1.40     19.6:1   48 V   1.5
  X6-60    D80 x 67.5             60           0.82     19.6:1   48 V   0.66
  X4-36    D55 x 61               34           0.36     36:1     48 V   0.30
Each X has a radial connector boss past the body OD (X6 to R50.9); clocking
matters and wraps must clear the boss all the way round.
"""
import itertools
import math

HEIGHT = 1.800       # sole to crown

MODEL = {                       # diameter, axial length, mass, ratio, J_m (kg.m^2)
    'X15-450': (0.166, 0.069, 3.50, 20.25, 31.6e-4),
    'X12-320': (0.124, 0.085, 2.37, 20.0,  12.9e-4),
    'X10-200': (0.124, 0.0635, 1.64, 20.0, 6.0e-4),    # J_m not published; scaled from X12
    'X8-120':  (0.096, 0.076, 1.40, 19.61, 1.5e-4),
    'X6-60':   (0.080, 0.0675, 0.82, 19.612, 0.66e-4),
    'X4-36':   (0.055, 0.0610, 0.36, 36.0,   0.30e-4),
    'X4-10':   (0.055, 0.0555, 0.26, 12.6,   0.30e-4),
}
# peak torque and max speed (rad/s) for the URDF limits
PEAK = {'X15-450': (450.0, 11.3), 'X12-320': (320.0, 13.1), 'X10-200': (200.0, 8.4),
        'X8-120': (120.0, 16.5), 'X6-60': (60.0, 16.0), 'X4-36': (34.0, 11.6),
        'X4-10': (10.0, 33.2)}

JOINT_MODEL = {
    'hip_pitch': 'X15-450', 'hip_roll': 'X12-320', 'hip_yaw': 'X10-200',
    'knee': 'X15-450', 'ank_pitch': 'X8-120', 'ank_roll': 'X6-60',
    'sho_pitch': 'X8-120', 'sho_roll': 'X6-60', 'elb_pitch': 'X6-60',
    'wrist_yaw': 'X4-36', 'head_yaw': 'X4-36', 'head_pitch': 'X4-36',
    'waist_yaw': 'X8-120',
}


def R(model):
    return MODEL[model][0] / 2.0


def LN(model):
    return MODEL[model][1]


# ---- proportions (fractions of HEIGHT, from the H2 unless noted) ----
THIGH = 0.225 * HEIGHT          # hip pitch -> knee            405
SHANK = 0.270 * HEIGHT          # knee -> ankle pitch          486
TORSO = 0.290 * HEIGHT          # hip pitch -> shoulder pitch  522
UPPER_ARM = 0.126 * HEIGHT      # shoulder pitch -> elbow      227
FOREARM = 0.131 * HEIGHT        # elbow -> wrist               236
NECK = 0.033 * HEIGHT           # shoulder pitch -> head yaw    59 (the deck is the yaw stator; head 136 tall)
SOLE = 0.050                    # ankle roll axis -> ground
KNEE_BACK = 0.060               # knee column behind the hip yaw axis (G1 53 at 1.32 m;
                                # ours 55 at 1.17) - the deep-flexion clearance. To be
                                # re-swept once the plates exist.

# ---- lateral layout, from the envelopes ----
# hip pitch bodies end-to-end at the centre, 12 mm apart for the pelvis spine
HIP_Y = LN('X15-450') + 0.003 + 0.006 + 0.010 + 0.006   # 0.094: body + flange + gap/2 + boss/2 + margin
HIP_Y = round(HIP_Y, 3)
# the roll body (D124 along x) sits outboard of the pitch body with 8 mm between
LEG_Y = round(HIP_Y + 0.008 + R('X12-320'), 3)           # 0.164
# shoulders: H2 roll at 0.093 H; ours wider because the hip-roll pods reach
# LEG_Y + R(X12) = 0.226 and an arm hanging straight must not read as a clash
SHO_Y = 0.255                  # hip-roll pods reach 0.226; a hand beside the hip must clear them
SHO_X = 0.000
ROLL_X = -0.064                # hip-roll body centre: body x -108..-20. Its output plate stands at x -114..-111:
                               # the thigh flanges' top corners sweep r 104 about the yaw axis (+/-57 deg) and
                               # nothing of hipB may stand inside that at the height of the yaw output plate.
SHR_X = -0.02125               # shoulder-roll body centre: body x -55..12.5, rear face behind the D96 pitch body

# ---- vertical layout ----
Z_HIP_P = -0.060                                          # pitch axis below trunk origin
Z_HIP_R = Z_HIP_P - 0.040                                 # roll 40 below pitch (H2 19, G1 30)
# yaw face must clear the roll body: roll R + 5
Z_HIP_Y = Z_HIP_R - R('X12-320') - 0.031                # under the roll body: hipB's bridge (3), gaps (4+5), yaw output plate (3) on its 6 mm spacer, stator plate (3) - see rose_pro_links hipB
Z_KNEE = Z_HIP_P - THIGH
Z_ANK_P = Z_KNEE - SHANK
ANK_DZ = R('X8-120') + R('X6-60') + 0.028                 # serial ankle stack, 116: the roll stator plate's corners (r 61) swing under the
                                                          # pitch stator plate (R51) through ankle pitch. (d) deletes this stack.
Z_ANK_R = Z_ANK_P - ANK_DZ
Z_SOLE = Z_ANK_R - SOLE
Z_SHO_P = Z_HIP_P + TORSO
Z_SHO_R = Z_SHO_P - R('X8-120') - R('X6-60') - 0.012   # room for the shoulder stator plate's outrigger
Z_ELB = Z_SHO_P - UPPER_ARM
Z_WRIST = Z_ELB - FOREARM
Z_HEAD_Y = Z_SHO_P + NECK
Z_HEAD_P = Z_HEAD_Y + 0.060
Z_CROWN = Z_SOLE + HEIGHT
# waist yaw: an X8-120 standing face-UP between the hip-pitch bodies' tops and the
# chest floor. Its body hangs into the pelvis; its stator plate is the pelvis top,
# its output plate the chest floor. (c) of the H3 programme: a pelvis that is a part.
Z_WAIST = Z_HIP_P + R('X15-450') + 0.004 + LN('X8-120') + 0.003    # face: body bottom 4 over the hip bodies, +3 flange

# sanity: the yaw body hangs inside the thigh and must clear the knee body
assert Z_HIP_Y - LN('X10-200') > Z_KNEE + R('X15-450') + 0.005, 'yaw body into knee body'


# label, joint origin (m), joint axis, model, body direction, mode
# 'end' puts the body starting at the joint and running along bdir; 'mid'
# centres it. Every x-axis (roll) actuator is 'mid' so it straddles the
# sagittal plane.
def leg(side):
    s = 1 if side == 'L' else -1
    return [
        ('%s_hip_pitch' % side, (0, s * HIP_Y, Z_HIP_P), (0, 1, 0), 'X15-450', (0, -s, 0), 'end'),
        ('%s_hip_roll' % side,  (ROLL_X, s * LEG_Y, Z_HIP_R), (1, 0, 0), 'X12-320', (-1, 0, 0), 'mid'),
        ('%s_hip_yaw' % side,   (0, s * LEG_Y, Z_HIP_Y), (0, 0, 1), 'X10-200', (0, 0, -1), 'end'),
        ('%s_knee' % side,      (-KNEE_BACK, s * LEG_Y, Z_KNEE), (0, 1, 0), 'X15-450', (0, -s, 0), 'end'),
        ('%s_ank_pitch' % side, (-KNEE_BACK, s * LEG_Y, Z_ANK_P), (0, 1, 0), 'X8-120', (0, -s, 0), 'end'),
        ('%s_ank_roll' % side,  (-KNEE_BACK, s * LEG_Y, Z_ANK_R), (1, 0, 0), 'X6-60', (-1, 0, 0), 'mid'),
    ]


def arm(side):
    s = 1 if side == 'L' else -1
    y = s * SHO_Y
    return [
        ('%s_sho_pitch' % side, (SHO_X, y, Z_SHO_P), (0, 1, 0), 'X8-120', (0, -s, 0), 'end'),
        ('%s_sho_roll' % side,  (SHR_X, y, Z_SHO_R), (1, 0, 0), 'X6-60', (-1, 0, 0), 'mid'),
        ('%s_elb_pitch' % side, (SHO_X, y, Z_ELB), (0, 1, 0), 'X6-60', (0, s, 0), 'end'),
        ('%s_wrist_yaw' % side, (SHO_X, y, Z_WRIST), (0, 0, 1), 'X4-36', (0, 0, 1), 'end'),
    ]


HEAD = [
    ('waist_yaw',  (0, 0, Z_WAIST), (0, 0, 1), 'X8-120', (0, 0, -1), 'end'),
    ('head_yaw',   (0, 0, Z_HEAD_Y), (0, 0, 1), 'X4-36', (0, 0, -1), 'end'),
    ('head_pitch', (0, 0, Z_HEAD_P), (0, 1, 0), 'X4-36', (0, -1, 0), 'mid'),
]


def joints():
    return leg('L') + leg('R') + arm('L') + arm('R') + HEAD


JOINTS = joints()


def seg(j):
    label, org, axis, model, bdir, mode = j
    dia, ln, _, _, _ = MODEL[model]
    start = -ln / 2.0 if mode == 'mid' else 0.0
    a = tuple(org[i] + bdir[i] * start for i in range(3))
    b = tuple(org[i] + bdir[i] * (start + ln) for i in range(3))
    return label, a, b, dia / 2.0, model


def cloud(a, b, r, n_ax=20, n_ang=48):
    """Surface point cloud of a cylinder from a to b, radius r (ends capped)."""
    ax = tuple(b[i] - a[i] for i in range(3))
    ln = math.sqrt(sum(v * v for v in ax)) or 1e-9
    u = tuple(v / ln for v in ax)
    helper = (1, 0, 0) if abs(u[0]) < 0.9 else (0, 0, 1)
    e1 = (u[1] * helper[2] - u[2] * helper[1],
          u[2] * helper[0] - u[0] * helper[2],
          u[0] * helper[1] - u[1] * helper[0])
    m = math.sqrt(sum(v * v for v in e1)) or 1e-9
    e1 = tuple(v / m for v in e1)
    e2 = (u[1] * e1[2] - u[2] * e1[1],
          u[2] * e1[0] - u[0] * e1[2],
          u[0] * e1[1] - u[1] * e1[0])
    pts = []
    for i in range(n_ax + 1):
        t = ln * i / n_ax
        base = tuple(a[k] + u[k] * t for k in range(3))
        rings = [r] if 0 < i < n_ax else [r, r * 0.5, 0.0]
        for rr in rings:
            for j in range(n_ang if rr > 0 else 1):
                th = 2 * math.pi * j / n_ang
                pts.append(tuple(base[k] + rr * (math.cos(th) * e1[k] +
                                                 math.sin(th) * e2[k])
                                 for k in range(3)))
    return pts


def cyl_gap(a1, b1, r1, a2, b2, r2):
    p, q = cloud(a1, b1, r1), cloud(a2, b2, r2)
    best = float('inf')
    for x in p:
        for y in q:
            d = (x[0]-y[0])**2 + (x[1]-y[1])**2 + (x[2]-y[2])**2
            if d < best:
                best = d
    return math.sqrt(best)


def main():
    segs = [seg(j) for j in JOINTS]
    print('=== clearance check (4 mm target) ===')
    clash = []
    for (l1, a1, b1, r1, _), (l2, a2, b2, r2, _) in itertools.combinations(segs, 2):
        gap = cyl_gap(a1, b1, r1, a2, b2, r2) * 1000
        if gap < 20:
            clash.append((gap, l1, l2))
    for gap, l1, l2 in sorted(clash):
        print('  %-14s <-> %-14s %+7.1f mm%s' % (l1, l2, gap, '   CLASH' if gap < 4 else ''))
    if not clash:
        print('  all pairs clear by >20 mm')

    print('\n=== proportions (HEIGHT %.0f) ===' % (HEIGHT * 1000))
    rows = [('hip pitch y / leg column y', '%.0f / %.0f' % (HIP_Y * 1e3, LEG_Y * 1e3)),
            ('thigh / shank', '%.0f / %.0f  (%.2f)' % (THIGH * 1e3, SHANK * 1e3, SHANK / THIGH)),
            ('hip pitch above sole', '%.0f = %.0f%%' % ((Z_HIP_P - Z_SOLE) * 1e3, 100 * (Z_HIP_P - Z_SOLE) / HEIGHT)),
            ('shoulder above sole', '%.0f = %.0f%%' % ((Z_SHO_P - Z_SOLE) * 1e3, 100 * (Z_SHO_P - Z_SOLE) / HEIGHT)),
            ('torso (hip->shoulder)', '%.0f' % (TORSO * 1e3)),
            ('arm (shoulder->wrist)', '%.0f' % ((UPPER_ARM + FOREARM) * 1e3)),
            ('ankle stack / sole', '%.0f / %.0f' % (ANK_DZ * 1e3, SOLE * 1e3)),
            ('shoulder y', '%.0f' % (SHO_Y * 1e3)),
            ('waist yaw face z', '%.0f' % (Z_WAIST * 1e3)),
            ('trunk origin: sole / crown', '%.0f / %.0f' % (Z_SOLE * 1e3, Z_CROWN * 1e3))]
    for k, v in rows:
        print('  %-28s %s' % (k, v))

    print('\n=== actuators ===')
    total = 0.0
    per = {}
    for label, _, _, _, model in segs:
        per[model] = per.get(model, 0) + 1
        total += MODEL[model][2]
    for m in sorted(per, key=lambda k: -MODEL[k][2]):
        print('  %-8s x%2d  %5.2f kg' % (m, per[m], per[m] * MODEL[m][2]))
    print('  all actuators %.2f kg' % total)


if __name__ == '__main__':
    main()
