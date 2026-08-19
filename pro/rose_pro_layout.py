"""Kinematic layout for rose pro - the original humanoid on MyActuator X-series.

Second of three in the rose line: rose mini (K1-scale study), rose pro (this one,
the buildable X-series machine), rose max (full size, on our own rana actuators).

Not a K1 clone. Designed around the actuator envelopes, with a deliberately
non-concurrent ("stacked offset") hip. Concurrency was dropped on purpose:
it buys only Pieper's condition for closed-form IK, which learned policies
never use.

Joint order per leg is pitch -> roll -> yaw -> knee, matching the K1's order
rather than AGILOped's yaw-first. That choice is load-bearing: with pitch at
the top, the hip yaw actuator's body hangs DOWN inside the thigh tube instead
of adding stack height below the hip, which reclaims ~70 mm of leg length.

Verified actuator envelopes (from vendor STEP in ~/Developer/myactuator_assets):
  X6-60  D80.0 x 67.5 mm, 20 Nm rated / 60 peak, 0.82 kg, 19.612:1, J_m 0.66 kg.cm2
  X4-36  D55.0 x 61.0 mm, 10.5 / 34 Nm,          0.36 kg, 36:1,     J_m 0.30 kg.cm2
  X4-10  D55.0 x 55.5 mm, 4.0 / 10 Nm,           0.26 kg, 12.6:1
Each has an asymmetric radial connector boss past the body OD (X6 to R50.9,
X4 to R29.3) - clocking matters, so mounts must allow rotation.
"""
import itertools
import math

MODEL = {                       # diameter, axial length, mass, ratio, J_m (kg.m^2)
    'X6-60': (0.080, 0.0675, 0.82, 19.612, 0.66e-4),
    'X4-36': (0.055, 0.0610, 0.36, 36.0,   0.30e-4),
    'X4-10': (0.055, 0.0555, 0.26, 12.6,   None),
}

HIP_Y = 0.075        # hip axes half-spacing
SHO_Y = 0.150        # shoulder half-spacing. Driven by the D80 hip roll body, which
                     # reaches y = HIP_Y+40 = 115 laterally: a straight-down arm must
                     # clear that. An earlier version bought the clearance by setting the
                     # arms 40 mm forward, but that broke fore-aft symmetry, so we pay
                     # for it in shoulder width instead.
SHO_X = 0.000        # arms on the sagittal centreline -- keep this at zero
THIGH = 0.250        # hip pitch -> knee. Floor is 242.5: the knee body's top
                     # (knee_z + 40) must clear the hip yaw body's bottom (yaw_z - 67.5)
                     # by 5 mm. The two are near-tangent, so z separation buys clearance
                     # very slowly - do not tune this by small steps.
SHANK = 0.250        # knee -> ankle pitch. Was 200: thigh:shank came out 1.25:1
                     # against ~1.05:1 human, which is what made the legs read stumpy.
                     # Thigh is FORCED to 250 (the hip yaw body hangs inside it), so the
                     # only way to 1:1 is to lengthen the shank.
KNEE_BACK = 0.055    # knee (and everything below it) sits this far BEHIND the hip yaw
                     # axis. The G1 carries 53 mm here and it is not styling: in deep
                     # knee flexion the shank folds up behind the thigh, and with the
                     # knee under the yaw axis the shank web enters the yaw body above
                     # ~93 deg (pro/sweep.py, rounded plates). 55 mm is the smallest
                     # offset that clears the full 132 deg URDF range by >= 4 mm.
ANK_DZ = 0.060       # ankle pitch -> ankle roll
SOLE = 0.050         # ankle roll -> ground

# Body placement mode: 'end' puts the body starting at the joint and running along
# bdir; 'mid' centres it on the joint. Every X-axis (roll) actuator uses 'mid' so it
# straddles the sagittal plane -- otherwise all four roll bodies hang off the back and
# the robot is visibly lopsided front-to-back.

# label, joint origin (m), joint axis, model, body direction, mode
def leg(side):
    s = 1 if side == 'L' else -1
    y = s * HIP_Y
    return [
        ('%s_hip_pitch' % side, (0, y, -0.060), (0, 1, 0), 'X6-60', (0, -s, 0), 'end'),
        ('%s_hip_roll' % side,  (0, y, -0.145), (1, 0, 0), 'X6-60', (-1, 0, 0), 'mid'),
        ('%s_hip_yaw' % side,   (0, y, -0.196), (0, 0, 1), 'X6-60', (0, 0, -1), 'end'),
        ('%s_knee' % side,      (-KNEE_BACK, y, -0.060 - THIGH), (0, 1, 0), 'X6-60', (0, -s, 0), 'end'),
        ('%s_ank_pitch' % side, (-KNEE_BACK, y, -0.060 - THIGH - SHANK), (0, 1, 0), 'X4-36', (0, -s, 0), 'end'),
        ('%s_ank_roll' % side,  (-KNEE_BACK, y, -0.060 - THIGH - SHANK - ANK_DZ), (1, 0, 0), 'X4-36', (-1, 0, 0), 'mid'),
    ]


def arm(side):
    s = 1 if side == 'L' else -1
    y = s * SHO_Y
    return [
        ('%s_sho_pitch' % side, (SHO_X, y, 0.230), (0, 1, 0), 'X4-36', (0, -s, 0), 'end'),
        ('%s_sho_roll' % side,  (SHO_X, y, 0.165), (1, 0, 0), 'X4-36', (-1, 0, 0), 'mid'),
        ('%s_elb_pitch' % side, (SHO_X, y, 0.040), (0, 1, 0), 'X4-36', (0, s, 0), 'end'),
        ('%s_wrist_yaw' % side, (SHO_X, y, -0.115), (0, 0, 1), 'X4-36', (0, 0, 1), 'end'),
    ]


HEAD = [
    ('head_yaw',   (0, 0, 0.330), (0, 0, 1), 'X4-10', (0, 0, -1), 'end'),
    ('head_pitch', (0, 0, 0.390), (0, 1, 0), 'X4-10', (0, -1, 0), 'mid'),
]

def joints(scale=1.0, lat=1.0):
    """All 22 joints, optionally scaled to another variant of the line.

    Scaling the SKELETON but not the actuator ENVELOPES is the whole point: the
    bodies are fixed lumps of vendor hardware, so a variant's packing is a genuinely
    different problem and has to be re-checked, not assumed. `lat` widens the Y axis
    only, which is how max gets to be broader than a pure scale-up rather than just
    a taller copy - and widening Y moves the two hip-pitch bodies APART, so it can
    only help that pair while doing nothing for the in-plane ones. Do not trust monotonicity
    here - the hip-yaw/knee pair is near-tangent and its gap has moved the WRONG way
    under a length increase before.
    """
    def sc(j):
        label, org, axis, model, bdir, mode = j
        k = (scale, scale * lat, scale)      # lat widens Y only: broader, not taller
        return (label, tuple(c * k[i] for i, c in enumerate(org)), axis, model, bdir, mode)
    return [sc(j) for j in leg('L') + leg('R') + arm('L') + arm('R') + HEAD]


JOINTS = joints()


def seg(j):
    label, org, axis, model, bdir, mode = j
    dia, ln, _, _, _ = MODEL[model]
    start = -ln / 2.0 if mode == 'mid' else 0.0
    a = tuple(org[i] + bdir[i] * start for i in range(3))
    b = tuple(org[i] + bdir[i] * (start + ln) for i in range(3))
    return label, a, b, dia / 2.0, model


def cloud(a, b, r, n_ax=20, n_ang=48):
    """Surface point cloud of a cylinder from a to b, radius r.

    Sampled rather than analytic: cylinder-cylinder distance has no clean
    closed form, and an axis-segment test minus radii is WRONG for collinear
    cylinders (it reports interference between two units that merely sit end
    to end on a shared axis, e.g. left and right hip pitch).
    """
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
        rings = [r] if 0 < i < n_ax else [r, r * 0.5, 0.0]   # cap the ends
        for rr in rings:
            for j in range(n_ang if rr > 0 else 1):
                th = 2 * math.pi * j / n_ang
                pts.append(tuple(base[k] + rr * (math.cos(th) * e1[k] +
                                                 math.sin(th) * e2[k])
                                 for k in range(3)))
    return pts


def cyl_gap(a1, b1, r1, a2, b2, r2):
    """Approximate minimum surface-to-surface distance between two cylinders."""
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

    foot_z = -0.060 - THIGH - SHANK - ANK_DZ - SOLE
    head_top = 0.390 + 0.070
    print('\n=== proportions ===')
    print('  trunk origin to sole   %6.0f mm' % (-foot_z * 1000))
    print('  trunk origin to crown  %6.0f mm' % (head_top * 1000))
    print('  standing height        %6.0f mm   (K1: 1150)' % ((head_top - foot_z) * 1000))
    print('  hip pitch to sole      %6.0f mm  = %.0f%% of height'
          % ((-0.060 - foot_z) * 1000, 100 * (-0.060 - foot_z) / (head_top - foot_z)))
    print('  stance width           %6.0f mm   (K1: 192)' % (2 * HIP_Y * 1000))
    print('  thigh / shank          %6.0f / %.0f mm   (K1: 191 / 245)' % (THIGH * 1000, SHANK * 1000))

    print('\n=== mass and armature ===')
    per = {}
    for label, _, _, _, model in segs:
        grp = 'leg' if label[1:3] == '_h' or '_ank' in label or '_knee' in label else \
              ('arm' if '_sho' in label or '_elb' in label or '_wrist' in label else 'head')
        per.setdefault(grp, []).append(model)
    total = 0.0
    for grp, models in per.items():
        m = sum(MODEL[x][2] for x in models)
        total += m
        n = 2 if grp in ('leg', 'arm') else 1
        print('  %-5s %2d actuators  %5.2f kg total  (%.2f kg each side)' % (grp, len(models), m, m / n))
    print('  all actuators             %5.2f kg' % total)
    print('\n  armature N^2*J_m to set in the sim model:')
    for model in ('X6-60', 'X4-36'):
        _, _, _, n, jm = MODEL[model]
        print('    %-6s  %.4f kg.m2   (K1 hip pitch was 0.0478, knee 0.0956)' % (model, n * n * jm))

    cost = {'X6-60': 490, 'X4-36': 400, 'X4-10': 345}
    print('\n  actuator cost: $%s' % f"{sum(cost[m] for _, _, _, _, m in segs):,}")


if __name__ == '__main__':
    main()
