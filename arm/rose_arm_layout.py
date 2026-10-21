"""Kinematic layout for the rose arm - a 7-DOF table arm on RobStride integrated
actuators, structure in FLAT sheet cut on the Anolex 4040 (no press brake).

Why this exists: Seeed's reBot B601-RS (6 DOF + gripper, RS06 x3 / RS00 x4,
6.7 kg, 2.5 kg payload, 754 mm reach) proves the actuator choice. This lays the
same family out as a HUMANOID arm (shoulder pitch / upper-arm roll, elbow,
forearm roll, wrist pitch, tool roll, on a base yaw) with a base plate.

ARCHITECTURE (2026-10-12, second cut): links are closed boxes of flat plates,
tab-and-slot, and LINKS NEVER OVERLAP. Each pancake is the node - its stator
bolts to one link's wall, its output to the next link's wall - so the only
moving interface between two links is the actuator's own output circumference,
a gap nothing fits in. Pitch pancakes stand beside the boxes (offset elbow, as
the UR and G1 do); roll pancakes sit inline as drums between two box segments.
The first cut (open U channels, forearm web riding inside the upper-arm channel)
was a scissor through 164 deg of elbow travel and had no torsional stiffness;
the closed box fixes both. Every plate is under the 4040's bed because the roll
drums split each link into two boxes anyway.

Actuator envelopes measured from the vendor STEP (github.com/RobStride/
Product_Information, 2026-07-13), torque / mass / speed from the same spec table:
  RS06     D88 x 49 (+15 output boss, connector to R52)  36 Nm peak  11 rated  0.621 kg  9:1   480 rpm
  RS00     D61 x 51 (+15)                                14          5         0.310     10:1  315 rpm
  EduLite  D46 x 44                                      6           1.8       0.242     7.75  480 rpm

Run:  python3 arm/rose_arm_layout.py
"""
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from full.rose_full_layout import cloud   # noqa: E402  (main renamed pro -> full)

G = 9.81
PAYLOAD = 2.5          # kg at the flange, full reach - reBot's number, matched
SHEET_RHO = 2680.0     # 5052
T_SKIN = 0.0015         # every box face
T_PLATE = 0.003        # doublers under bolt circles, base, column floor
CUTOUT = 0.30          # lightening fraction on skins
BED = 0.390            # Anolex 4040 usable travel, conservative; no plate longer than this
GAP = 0.003            # between a drum and the box that follows it
SPACER = 0.005         # adapter plate between a pitch output and the next link's wall

MODEL = {   # dia, axial length (body + output boss), mass, ratio, peak Nm, rated Nm, no-load rpm, usd (amazon)
    'RS06':    (0.088, 0.064, 0.621, 9.0,  36.0, 11.0, 480, 260),
    'RS00':    (0.061, 0.066, 0.310, 10.0, 14.0, 5.0,  315, 165),
    'EduLite': (0.046, 0.044, 0.242, 7.75, 6.0,  1.8,  480, 110),
}


def R(m):
    return MODEL[m][0] / 2.0


def LN(m):
    return MODEL[m][1]


def KG(m):
    return MODEL[m][2]


# ---- links ----
BASE_H = 0.130           # base plate -> shoulder pitch axis
UPPER = 0.330            # shoulder pitch -> elbow pitch
FORE = 0.300             # elbow pitch -> wrist pitch

# box sections: outer size = drum diameter + 2 skins, so link and drum read as one body
UA_W = 2 * R('RS06') + 2 * T_SKIN      # 0.092 upper-arm box, square
FA_W = 2 * R('RS00') + 2 * T_SKIN      # 0.065 forearm and hand boxes
END = 0.045              # a box runs this far past a pitch axis to carry its bolt circle

# along the arm, x. Each roll drum sits GAP past the box segment that carries the pitch bolt circle.
J3_X = END + GAP + LN('RS06') / 2 + 0.025          # upper-arm roll drum, 25 mm clear of the shoulder: the J3 cable loop must clear the column (rose_arm_cables)
J5_X = UPPER + END + GAP + LN('RS00') / 2              # forearm roll drum centre
J7_X0 = UPPER + FORE + END + GAP                       # tool roll drum starts here
GRIP_X = J7_X0 + LN('RS00') + 0.004 + R('EduLite')     # gripper motor, face-down, past the tool roll output
REACH = GRIP_X + R('EduLite') + 0.004                  # flange
HAND = REACH - (UPPER + FORE)

# across the arm, y. Shoulder pitch body hangs on -y of the upper-arm box; elbow body on the same
# side; the forearm box hangs off the elbow output further -y; the wrist pitch body comes back +y
# off the forearm; the hand box sits just +y of the shoulder plane. Net hand offset ~ +40 mm.
UA_Y0 = 0.0                                   # upper-arm inner wall (3 mm, J2 and J4 stators bolt to its -y face)
UA_YC = UA_Y0 + UA_W / 2                      # upper-arm box centre line
J4_OUT_Y = UA_Y0 - LN('RS06')                 # elbow output face
FA_Y1 = J4_OUT_Y - SPACER                     # forearm inner wall outer face
FA_YC = FA_Y1 - FA_W / 2
J6_OUT_Y = FA_Y1 + LN('RS00')                 # wrist pitch output face (body grows +y off the forearm +y wall)
HA_Y0 = J6_OUT_Y + SPACER                     # hand inner wall
HA_YC = HA_Y0 + FA_W / 2

# column: a box beside the J2 body, its +y wall is J2's stator plate
COL_W, COL_D = 0.100, 0.060                   # x, y
COL_Y1 = UA_Y0 - LN('RS06') - T_PLATE         # column +y face (J2 stator bolts here)
COL_Z0 = 0.003 + LN('RS06') + T_PLATE         # on top of the J1 output + floor plate
COL_Z1 = BASE_H + UA_W / 2 + 0.005

JOINT_MODEL = {
    'J1_yaw': 'RS06',          # no gravity moment, but the whole arm's bending moment rides on this output bearing
    'J2_sho_pitch': 'RS06',
    'J3_upper_roll': 'RS06',   # elbow bent 90, it carries the forearm + payload moment - same as the elbow
    'J4_elbow': 'RS06',
    'J5_fore_roll': 'RS00',
    'J6_wrist_pitch': 'RS00',
    'J7_tool_roll': 'RS00',
    'gripper': 'EduLite',
}


# ---- geometry primitives: (label, kind, params) ----
def joints(dof7=True):
    """Actuators as cylinders: label, origin, axis, model, body direction, mode (pro's convention)."""
    z0 = BASE_H
    J = [
        ('J1_yaw',        (0, 0, 0.003),                    (0, 0, 1), 'RS06', (0, 0, 1), 'end'),
        ('J2_sho_pitch',  (0, UA_Y0, z0),                   (0, 1, 0), 'RS06', (0, -1, 0), 'end'),
    ]
    if dof7:
        J.append(('J3_upper_roll', (J3_X, UA_YC, z0),       (1, 0, 0), 'RS06', (1, 0, 0), 'mid'))
    J += [
        ('J4_elbow',      (UPPER, UA_Y0, z0),               (0, 1, 0), 'RS06', (0, -1, 0), 'end'),
        ('J5_fore_roll',  (J5_X, FA_YC, z0),                (1, 0, 0), 'RS00', (1, 0, 0), 'mid'),
        ('J6_wrist_pitch', (UPPER + FORE, FA_Y1, z0),       (0, 1, 0), 'RS00', (0, 1, 0), 'end'),
        ('J7_tool_roll',  (J7_X0, HA_YC, z0),               (1, 0, 0), 'RS00', (1, 0, 0), 'end'),
        ('gripper',       (GRIP_X, HA_YC, z0),              (0, 0, 1), 'EduLite', (0, 0, -1), 'mid'),
    ]
    return J


def boxes(dof7=True):
    """Closed sheet boxes as AABBs: label, (x0,y0,z0), (x1,y1,z1), link."""
    z0 = BASE_H
    h = UA_W / 2
    f = FA_W / 2
    B = [
        ('base_plate', (-0.080, -0.080, -T_PLATE), (0.080, 0.080, 0.0), 'base'),
        ('column', (-COL_W / 2, COL_Y1 - COL_D, COL_Z0), (COL_W / 2, COL_Y1, COL_Z1), 'base'),
        ('col_floor', (-COL_W / 2, COL_Y1 - COL_D, COL_Z0 - T_PLATE), (COL_W / 2, 0.040, COL_Z0), 'base'),
    ]
    link = {'ua_A': 'upper', 'ua_B': 'upper', 'ua': 'upper', 'fa_C': 'fore', 'fa_D': 'fore', 'hand': 'hand'}
    for label, (x0, x1), (y0, y1), ends, waist in profiles(dof7):
        pts = profile_poly(x0, x1, ends, waist)
        xs, zs = [p[0] for p in pts], [p[1] for p in pts]
        B.append((label, (min(xs), y0, z0 + min(zs)), (max(xs), y1, z0 + max(zs)), link[label]))
    B += [
        ('grip_plate', (J7_X0 + LN('RS00'), HA_YC - 0.020, z0 - R('EduLite') - 0.006), (REACH, HA_YC + 0.020, z0 - R('EduLite') - 0.003), 'hand'),
    ]
    return B


# ---- link profiles (the look) ----
# Each sheet segment is a side profile in x-z, extruded across y. An end that meets a
# PITCH motor is a round lobe concentric with that motor (the motor disc sits in a
# cradle of the same radius, plus LOBE_LIP); an end that meets a ROLL drum is flat at
# the drum's height, so box and drum read as one body. Long segments waist down to
# WAIST of their smaller end between joints - the bending moment is lowest mid-span.
LOBE_LIP = 0.002
# BONE (2026-10-12, from rose_arm_bone.py): open lattice cage - four X-braced lattice faces on
# corner rails, full depth (no waist), with solid 3 mm knuckle plates at each pitch lobe and a
# solid end frame where a link meets a roll drum. 55 % see-through, cables run in the core.
BONE = dict(kind='2020', s=0.080, tf=0.002, w=0.008, knuckle_t=0.003, knuckle_len=0.022)   # kind 'cage' or '2020'
WAIST = 1.0


def profiles(dof7=True):
    """label, (x0, x1), (y0, y1), [end0, end1] with end = ('flat', half_h) | ('lobe', xc, r), waist half_h or None."""
    rU = A6 = R('RS06') + LOBE_LIP
    r0 = R('RS00') + LOBE_LIP
    hu, hf = UA_W / 2, FA_W / 2
    P = []
    if dof7:
        P.append(('ua_A', (0.0, J3_X - LN('RS06') / 2 - GAP), (UA_Y0, UA_Y0 + UA_W), [('lobe', 0.0, rU), ('flat', hu)], None))
        P.append(('ua_B', (J3_X + LN('RS06') / 2 + GAP, UPPER), (UA_Y0, UA_Y0 + UA_W), [('flat', hu), ('lobe', UPPER, rU)], hu * WAIST))
    else:
        P.append(('ua', (0.0, UPPER), (UA_Y0, UA_Y0 + UA_W), [('lobe', 0.0, rU), ('lobe', UPPER, rU)], hu * WAIST))
    P += [
        ('fa_C', (UPPER, J5_X - LN('RS00') / 2 - GAP), (FA_Y1 - FA_W, FA_Y1), [('lobe', UPPER, rU), ('flat', hf)], None),
        ('fa_D', (J5_X + LN('RS00') / 2 + GAP, UPPER + FORE), (FA_Y1 - FA_W, FA_Y1), [('flat', hf), ('lobe', UPPER + FORE, r0)], hf * WAIST),
        ('hand', (UPPER + FORE, J7_X0 - GAP), (HA_Y0, HA_Y0 + FA_W), [('lobe', UPPER + FORE, r0), ('flat', hf)], None),
    ]
    return P


def profile_poly(x0, x1, ends, waist, n_arc=12):
    """Closed outline (x, z about the arm axis) for one profile: top edge left to right, round the right
    end, bottom edge right to left, round the left end. Lobes are sampled for area / extents."""
    def end_pts(e, x, right):
        if e[0] == 'flat':
            return [(x, e[1]), (x, -e[1])] if right else [(x, -e[1]), (x, e[1])]
        _, xc, r = e
        a0, a1 = (math.pi / 2, -math.pi / 2) if right else (-math.pi / 2, -3 * math.pi / 2)
        return [(xc + r * math.cos(a0 + (a1 - a0) * i / n_arc), r * math.sin(a0 + (a1 - a0) * i / n_arc)) for i in range(n_arc + 1)]
    L = end_pts(ends[0], x0, False)
    Rr = end_pts(ends[1], x1, True)
    top = [L[-1]]
    if waist:
        xm = (x0 + x1) / 2
        top.append((xm, waist))
    pts = top + Rr
    if waist:
        pts.append(((x0 + x1) / 2, -waist))
    pts += L[:-1]
    return pts


def poly_area_perim(pts):
    a = per = 0.0
    for i in range(len(pts)):
        (x1, z1), (x2, z2) = pts[i], pts[(i + 1) % len(pts)]
        a += x1 * z2 - x2 * z1
        per += math.hypot(x2 - x1, z2 - z1)
    return abs(a) / 2, per


LINK_OF = {'J1_yaw': 'base', 'J2_sho_pitch': 'base', 'J3_upper_roll': 'upper', 'J4_elbow': 'upper',
           'J5_fore_roll': 'fore', 'J6_wrist_pitch': 'fore', 'J7_tool_roll': 'hand', 'gripper': 'hand'}
FOLD_LINKS = ('fore', 'hand')        # what swings about the elbow


def seg(j):
    label, org, axis, model, bdir, mode = j
    ln = LN(model)
    start = -ln / 2.0 if mode == 'mid' else 0.0
    a = tuple(org[i] + bdir[i] * start for i in range(3))
    b = tuple(org[i] + bdir[i] * (start + ln) for i in range(3))
    return label, a, b, R(model), model


def box_cloud(p0, p1, n=6):
    pts = []
    for i in range(n + 1):
        for j in range(n + 1):
            u, v = i / n, j / n
            x = p0[0] + (p1[0] - p0[0]) * u
            y = p0[1] + (p1[1] - p0[1]) * u
            z = p0[2] + (p1[2] - p0[2]) * v
            pts += [(x, p0[1], z), (x, p1[1], z), (p0[0], y, z), (p1[0], y, z),
                    (x, p0[1] + (p1[1] - p0[1]) * v, p0[2]), (x, p0[1] + (p1[1] - p0[1]) * v, p1[2])]
    return pts


def aabb(p):
    return [min(v[i] for v in p) for i in range(3)], [max(v[i] for v in p) for i in range(3)]


def aabb_gap(a, b):
    d = 0.0
    for i in range(3):
        e = max(a[0][i] - b[1][i], b[0][i] - a[1][i], 0.0)
        d += e * e
    return math.sqrt(d)


def gap(p, q):
    best = float('inf')
    for x in p:
        for y in q:
            d = (x[0] - y[0]) ** 2 + (x[1] - y[1]) ** 2 + (x[2] - y[2]) ** 2
            if d < best:
                best = d
    return math.sqrt(best)


def rot_y(p, c, th):
    """Rotate p about a y-axis through c by th (positive folds the forearm down/under)."""
    x, z = p[0] - c[0], p[2] - c[2]
    return (c[0] + x * math.cos(th) + z * math.sin(th), p[1], c[2] - x * math.sin(th) + z * math.cos(th))


def bodies(dof7=True, fold_deg=0.0):
    """Everything as (label, link, point cloud), with the forearm + hand folded about the elbow."""
    th = math.radians(fold_deg)
    c = (UPPER, 0, BASE_H)
    out = []
    for j in joints(dof7):
        label, a, b, r, _ = seg(j)
        pts = cloud(a, b, r, n_ax=8, n_ang=24)
        link = LINK_OF[label]
        if link in FOLD_LINKS and th:
            pts = [rot_y(p, c, th) for p in pts]
        out.append((label, link, pts))
    for label, p0, p1, link in boxes(dof7):
        pts = box_cloud(p0, p1)
        if link in FOLD_LINKS and th:
            pts = [rot_y(p, c, th) for p in pts]
        out.append((label, link, pts))
    return out


# pairs that are bolted to each other on purpose (a stator or output on a wall)
BOLTED = {('J1_yaw', 'col_floor'), ('J1_yaw', 'base_plate'), ('J2_sho_pitch', 'column'), ('J2_sho_pitch', 'ua_A'),
          ('J2_sho_pitch', 'ua'), ('J3_upper_roll', 'ua_A'), ('J3_upper_roll', 'ua_B'), ('J4_elbow', 'ua_B'),
          ('J4_elbow', 'ua'), ('J4_elbow', 'fa_C'), ('J5_fore_roll', 'fa_C'), ('J5_fore_roll', 'fa_D'),
          ('J6_wrist_pitch', 'fa_D'), ('J6_wrist_pitch', 'hand'), ('J7_tool_roll', 'hand'), ('J7_tool_roll', 'grip_plate'),
          ('gripper', 'grip_plate'), ('column', 'col_floor'), ('col_floor', 'base_plate'), ('col_floor', 'ua_A'),
          ('hand', 'grip_plate'), ('J2_sho_pitch', 'col_floor'), ('J1_yaw', 'column')}


def clearance(dof7=True, fold_deg=0.0, report=20.0, folding_only=False):
    rows = []
    B = [(l, k, p, aabb(p)) for l, k, p in bodies(dof7, fold_deg)]
    for (l1, k1, p, b1), (l2, k2, q, b2) in itertools.combinations(B, 2):
        if k1 == k2 or (l1, l2) in BOLTED or (l2, l1) in BOLTED:
            continue
        if folding_only and not ({k1, k2} & set(FOLD_LINKS) and {k1, k2} & {'upper'}):
            continue      # the fold check is self-collision with the upper arm; the table is the shoulder's job
        if aabb_gap(b1, b2) * 1000 > report:
            continue
        g = gap(p, q) * 1000
        if g < report:
            rows.append((g, l1, l2))
    return sorted(rows)


def elbow_fold_limit(dof7=True, clear=4.0):
    worst = 0
    for deg in range(0, 181, 5):
        rows = clearance(dof7, deg, report=clear, folding_only=True)
        if rows:
            return worst, rows
        worst = deg
    return worst, []


# ---- mass ----
def box_mass(p0, p1, doublers=2):
    """A closed box, all 2 mm with windows, plus a 3 mm doubler (h x h) under each bolt circle.
    The doubler is a separate small plate riveted inside the wall: the bolt circle needs thread
    depth and bearing area, the 300 mm of wall between two joints does not."""
    L = p1[0] - p0[0]
    w = p1[1] - p0[1]
    h = p1[2] - p0[2]
    sides = SHEET_RHO * T_SKIN * (2 * L * h + 2 * L * w) * (1 - CUTOUT)
    ends = SHEET_RHO * T_SKIN * 2 * w * h
    dbl = SHEET_RHO * T_PLATE * h * h * doublers
    return sides + ends + dbl


M_GRIP_STRUCT = 0.25
M_HARNESS = 0.25


def masses(dof7=True):
    """(label, mass, x along the arm from the shoulder pitch axis), zero pose = arm horizontal."""
    rows = [('J2 RS06', KG('RS06'), 0.0)]
    prof = {p[0]: p for p in profiles(dof7)}
    for label, p0, p1, link in boxes(dof7):
        if link == 'base':
            continue
        if label in prof:
            m, xc = profile_mass(prof[label])
        else:
            m, xc = 0.02, (p0[0] + p1[0]) / 2
        rows.append((label, m, xc))
    for j in joints(dof7):
        label, org = j[0], j[1]
        if LINK_OF[label] != 'base':
            rows.append((label, KG(j[3]), org[0]))
    rows += [('gripper structure', M_GRIP_STRUCT, REACH - 0.020), ('harness', M_HARNESS, 0.35)]
    return rows


def cage_span(prof):
    """x-range of the lattice between the knuckles / end frames of a profile."""
    label, (x0, x1), _, ends, _ = prof
    a = x0 + (ends[0][2] + BONE['knuckle_len'] if ends[0][0] == 'lobe' else T_PLATE) if ends[0][0] == 'lobe' else x0 + T_PLATE
    if ends[0][0] == 'lobe':
        a = ends[0][1] + ends[0][2] + BONE['knuckle_len'] * 0.0
    b = ends[1][1] - ends[1][2] if ends[1][0] == 'lobe' else x1 - T_PLATE
    return a, b


def profile_mass(prof):
    """BONE: knuckle plate pairs at lobes, end frames at drums, lattice cage between. (mass, x of centroid)"""
    import rose_arm_bone as Bn
    label, (x0, x1), (y0, y1), ends, waist = prof
    W = y1 - y0
    m = 0.0
    xs = []
    for e, xe in ((ends[0], x0), (ends[1], x1)):
        if e[0] == 'lobe':
            r = e[2]
            area = math.pi * r * r / 2 + 2 * r * BONE['knuckle_len'] + 2 * r * r
            mk = 2 * SHEET_RHO * BONE['knuckle_t'] * area
            m += mk
            xs.append((mk, e[1]))
        else:
            h = 2 * e[1]
            mf = SHEET_RHO * T_PLATE * W * h * 0.55          # end frame, centre cut out round the drum boss
            m += mf
            xs.append((mf, xe))
    a, b = cage_span(prof)
    L = max(0.0, b - a)
    if L > 0.01:
        H = 2 * max((e[2] if e[0] == 'lobe' else e[1]) for e in ends)
        sec = Bn.bar2020('4x', W, H, L, s=BONE['s']) if BONE['kind'] == '2020' else \
            Bn.section('cage', W, H, L, tf=BONE['tf'], w=BONE['w'])
        m += sec['mass']
        xs.append((sec['mass'], (a + b) / 2))
    xc = sum(mm * x for mm, x in xs) / max(m, 1e-9)
    return m, xc


def profile_mass_box(prof):
    """Two windowed side profiles + one wrap strip round the perimeter + a doubler per lobe. (mass, x of centroid)"""
    label, (x0, x1), (y0, y1), ends, waist = prof
    pts = profile_poly(x0, x1, ends, waist)
    area, per = poly_area_perim(pts)
    w = y1 - y0
    m = SHEET_RHO * T_SKIN * (2 * area + per * w) * (1 - CUTOUT)
    m += sum(SHEET_RHO * T_PLATE * math.pi * e[2] ** 2 for e in ends if e[0] == 'lobe')
    xc = sum(p[0] for p in pts) / len(pts)
    return m, xc


def base_mass(dof7=True):
    m = KG('RS06')
    for label, p0, p1, link in boxes(dof7):
        if link == 'base':
            m += box_mass(p0, p1) if label == 'column' else SHEET_RHO * T_PLATE * (p1[0] - p0[0]) * (p1[1] - p0[1])
    return m


def arm_mass(dof7=True):
    return sum(m for _, m, _ in masses(dof7)) + base_mass(dof7)


def moment_past(x0, dof7=True, payload=PAYLOAD):
    """Gravity moment (Nm) about a horizontal axis at x0, arm horizontal, payload at the flange."""
    m = sum(mm * (x - x0) for _, mm, x in masses(dof7) if x > x0)
    return G * (m + payload * (REACH - x0))


def self_moment_past(x0, dof7=True):
    return moment_past(x0, dof7, payload=0.0)


R_SPRING = 0.060     # gas spring across J2, effective arm
E_AL, G_AL = 69e9, 26e9


def box_section(a, t):
    """Square thin-walled box, side a, wall t, windowed. Bending I = 2/3 t a^3; Bredt J = a^3 t.
    Windows knock bending down by (1 - CUTOUT) and torsion by (1 - CUTOUT)^2, which is
    conservative for windows kept off the corners."""
    I = 2.0 / 3.0 * t * a ** 3 * (1 - CUTOUT)
    J = a ** 3 * t * (1 - CUTOUT) ** 2
    return I, J


def stiffness(dof7=True):
    """Static deflection at the flange, arm horizontal, payload on: bending sag + roll twist."""
    def rect(w, h, t):
        I = (t * h ** 3 / 6 + w * t * h ** 2 / 2) * (1 - CUTOUT)
        J = 4 * (w * h) ** 2 * t / (2 * (w + h)) * (1 - CUTOUT) ** 2
        return I, J
    import rose_arm_bone as Bn
    if BONE['kind'] == '2020':
        su = Bn.bar2020('4x', UA_W, UA_W, UPPER, s=BONE['s'])
        sf = Bn.bar2020('4x', FA_W, FA_W, FORE, s=BONE['s'])
    else:
        su = Bn.section('cage', UA_W, UA_W * WAIST, UPPER, tf=BONE['tf'], w=BONE['w'])
        sf = Bn.section('cage', FA_W, FA_W * WAIST, FORE, tf=BONE['tf'], w=BONE['w'])
    Iu, Ju, If, Jf = su['I'], su['J'], sf['I'], sf['J']
    P_e = (moment_past(UPPER, dof7) - moment_past(UPPER + 1e-6, dof7)) / 1e-6 if False else None
    m_e = moment_past(UPPER, dof7)                     # N.m at the elbow
    F_e = m_e / (REACH - UPPER) * 1.0                  # equivalent tip-ish shear, conservative
    m_w = moment_past(UPPER + FORE, dof7)
    F_w = m_w / HAND
    th_u = (F_e * UPPER ** 2 / 2 + m_e * UPPER) / (E_AL * Iu)
    d_u = (F_e * UPPER ** 3 / 3 + m_e * UPPER ** 2 / 2) / (E_AL * Iu)
    th_f = (F_w * FORE ** 2 / 2 + m_w * FORE) / (E_AL * If)
    d_f = (F_w * FORE ** 3 / 3 + m_w * FORE ** 2 / 2) / (E_AL * If)
    sag = d_u + th_u * (FORE + HAND) + d_f + th_f * HAND
    # roll pose: elbow bent 90, the forearm moment arrives at the upper arm as torque
    twist = m_e * UPPER / (G_AL * Ju) * (REACH - UPPER)
    return sag, twist


def torque_table(dof7=True):
    sho = moment_past(0.0, dof7)
    sho_self = self_moment_past(0.0, dof7)
    elb = moment_past(UPPER, dof7)
    wri = moment_past(UPPER + FORE, dof7)
    rows = [
        ('J2 shoulder pitch', 'RS06', sho, sho - sho_self),
        ('J4 elbow', 'RS06', elb, elb),
        ('J6 wrist pitch', 'RS00', wri, wri),
        ('J5 forearm roll', 'RS00', wri, wri),
    ]
    if dof7:
        rows.insert(2, ('J3 upper-arm roll', 'RS06', elb, elb))
    return rows, sho_self


def plates(dof7=True):
    """Every flat plate with its blank size, for the 4040 check and the cut list."""
    out = []
    for label, p0, p1, link in boxes(dof7):
        L, w, h = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
        if label in ('base_plate', 'col_floor', 'grip_plate'):
            out.append((label, L, w, T_PLATE))
            continue
        out += [(label + ' wall x2', L, h, T_SKIN), (label + ' skin x2', L, w, T_SKIN), (label + ' end x2', w, h, T_SKIN),
                (label + ' doubler x2', h, h, T_PLATE)]
    return out


def main():
    for dof7 in (True, False):
        name = '7-DOF humanoid arm' if dof7 else '6-DOF (no upper-arm roll)'
        print('=' * 74)
        print('rose arm - %s - flat-pack sheet boxes' % name)
        print('=' * 74)

        print('\n--- links ---')
        print('  base -> shoulder %3.0f   upper %3.0f   fore %3.0f   hand %3.0f   reach from shoulder %3.0f mm   (reBot 754)'
              % (BASE_H * 1e3, UPPER * 1e3, FORE * 1e3, HAND * 1e3, REACH * 1e3))
        print('  boxes: upper arm %.0f sq, forearm / hand %.0f sq.  hand centre line y %+.0f from the shoulder plane'
              % (UA_W * 1e3, FA_W * 1e3, HA_YC * 1e3))

        print('\n--- mass ---')
        act = sum(KG(j[3]) for j in joints(dof7))
        tot = arm_mass(dof7)
        sheet = tot - act - M_HARNESS - M_GRIP_STRUCT
        print('  actuators %.2f   sheet %.2f   gripper %.2f   harness %.2f   TOTAL %.2f kg   (reBot RS: 6.7 kg)'
              % (act, sheet, M_GRIP_STRUCT, M_HARNESS, tot))
        print('  payload / mass  %.2f   (reBot 0.37)' % (PAYLOAD / tot))

        print('\n--- gravity torque, arm horizontal, %.1f kg at the flange ---' % PAYLOAD)
        rows, sho_self = torque_table(dof7)
        print('  %-20s %-5s %7s %7s %6s %6s' % ('joint', 'motor', 'need', 'peak', 'SF', 'rated'))
        for j, m, need, need_cb in rows:
            peak, rated = MODEL[m][4], MODEL[m][5]
            print('  %-20s %-5s %6.1f  %6.0f  %5.1f  %5.1fx' % (j, m, need, peak, peak / need, rated / need))
            if need_cb != need:
                print('  %-20s %-5s %6.1f  %6.0f  %5.1f  %5.1fx   <- gas spring across J2 (F = %.0f N at %.0f mm)'
                      % ('   + counterbalance', '', need_cb, peak, peak / need_cb, rated / need_cb,
                         sho_self / R_SPRING, R_SPRING * 1e3))
        sho_max = MODEL['RS06'][4] / 1.5
        print('  payload at full reach for SF 1.5 on the shoulder:  %.1f kg bare,  %.1f kg with the spring'
              % (max((sho_max - sho_self) / (G * REACH), 0), sho_max / (G * REACH)))

        sag, tw = stiffness(dof7)
        print('  flange deflection from structure: %.2f mm sag (arm out), %.2f mm from upper-arm twist (elbow at 90)'
              % (sag * 1e3, tw * 1e3))
        print('  (reBot quotes 0.2 mm repeatability; structure should stay well under the actuators\' own backlash)')

        print('\n--- clearance, zero pose (4 mm target; bolted pairs excluded) ---')
        rows = clearance(dof7)
        for g, l1, l2 in rows:
            print('  %-15s <-> %-15s %+7.1f mm%s' % (l1, l2, g, '   CLASH' if g < 4 else ''))
        if not rows:
            print('  all pairs clear by >20 mm')
        lim, hit = elbow_fold_limit(dof7)
        print('  elbow folds to %d deg%s' % (lim, '' if not hit else '  (next 5 deg: %s <-> %s at %.1f mm)' % (hit[0][1], hit[0][2], hit[0][0])))

        print('\n--- plates (Anolex 4040, bed %.0f mm) ---' % (BED * 1e3))
        over = []
        for label, a, b, t in plates(dof7):
            flag = '' if max(a, b) <= BED else '   TOO LONG'
            if flag:
                over.append(label)
            print('  %-18s %4.0f x %4.0f x %.1f%s' % (label, a * 1e3, b * 1e3, t * 1e3, flag))
        print('  %s' % ('every plate fits the bed' if not over else
                        'OVER THE BED: %s  -> split at mid-span with an internal 3 mm splice plate' % ', '.join(over)))

        print('\n--- BOM (motors, amazon retail) ---')
        per = {}
        for j in joints(dof7):
            per[j[3]] = per.get(j[3], 0) + 1
        cost = 0
        for m in per:
            print('  %-8s x%d  $%d' % (m, per[m], per[m] * MODEL[m][7]))
            cost += per[m] * MODEL[m][7]
        print('  motors $%d   + USB-CAN $25   (reBot RS full kit $1499 early bird)' % cost)
        print()


if __name__ == '__main__':
    main()
