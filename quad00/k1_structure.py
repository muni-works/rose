"""Structure for the K1-layout mini: 3.2 mm router-cut aluminium plates inside the bones, RobStrides bolted
to plate pairs, printed shells over. LEGS first. K1 frame: x fore, y left, z up (mm).

Packaging on the K1's joint axes with our motors:
  hip pitch  RS06, axis y, at the K1 hip-pitch point; stator back on the pelvis side plate (inboard),
             output outboard;
  hip roll   RS06, axis x, on the K1 roll AXIS but moved FORWARD along it, clear of the pitch can
             (the K1 nests a smaller roll 26 mm under its pitch; two RS06 cannot); output faces aft;
  hip yaw    RS02, axis z, at the K1 yaw point, stator up;
  knee       RS06, axis y, at the K1 knee point, stator on the thigh's inner plate, output outboard to
             the shin;
  ankle      2 x RS00 in the shin box (lateral axes), rods to the foot (as the K1 does from its shank).
"""
import math
import numpy as np, trimesh
from manifold3d import Manifold, CrossSection, FillRule
import leg as L
import quad as Q
import k1 as K, k1_teardown as KT

T = L.T
G = 1.0                      # assembly gap


def plate(profile, plane, w0, t=T):
    """extrude a shapely profile into a plate: plane 'xz' (normal y), 'yz' (normal x), 'xy' (normal z)."""
    cs = CrossSection(L.rings(profile), FillRule.EvenOdd)
    m = L.to_world(cs.extrude(t).translate((0, 0, w0)), plane)
    return L.tri(m)


def motor(c, ax, mot):
    m = Q.MOTORS[mot]
    a = np.asarray(ax, float) / np.linalg.norm(ax)
    return trimesh.creation.cylinder(radius=m['D'] / 2, segment=[np.asarray(c) - a * m['L'] / 2, np.asarray(c) + a * m['L'] / 2], sections=64)


def layout(J, side):
    """our joint placements (motor centre, axis, model) for one leg, on the K1's axes."""
    s = 1 if side == 'Left' else -1
    m06, m02, m00 = Q.MOTORS['rs06'], Q.MOTORS['rs02'], Q.MOTORS['rs00']
    hp = J[f'{side}_Hip_Pitch']['p'].copy()
    hr = J[f'{side}_Hip_Roll']['p'].copy()
    hy = J[f'{side}_Hip_Yaw']['p'].copy()
    kn = J[f'{side}_Knee_Pitch']['p'].copy()
    an = J[f'{side}_Ankle_Pitch']['p'].copy()
    # roll: inside the pelvis, axis fore/aft, inboard of the hip line (hidden); pitch: one drum under the pelvis flank;
    # yaw: straight under the pitch, then the thigh.  (the K1 hides its roll in the trunk the same way)
    roll_c = np.array([5.0, s * 48.0, -30.0])                # centred in the pelvis block
    hp = np.array([0.0, hp[1], roll_c[2] - m06['D'] / 2 - G - m06['D'] / 2])
    hy = np.array([0.0, hp[1], hp[2] - m06['D'] / 2 - G - T - G - m02['L'] / 2])
    return dict(s=s,
                pitch=(hp, np.array([0, s * 1.0, 0]), 'rs06'),     # output outboard (+s y)
                roll=(roll_c, np.array([1.0, 0, 0]), 'rs06'),      # output forward (+x)
                yaw=(hy, np.array([0, 0, -1.0]), 'rs02'),          # output down
                knee=(kn, np.array([0, s * 1.0, 0]), 'rs06'),      # output outboard
                ankle=an)


def leg(J, side):
    """the leg's plates and motors."""
    P = layout(J, side)
    s = P['s']
    m06, m02, m00 = Q.MOTORS['rs06'], Q.MOTORS['rs02'], Q.MOTORS['rs00']
    out = []
    hp, _, _ = P['pitch']; rc, _, _ = P['roll']; yc, _, _ = P['yaw']; kc, _, _ = P['knee']; ac = P['ankle']
    # -- motors
    for k_ in ('pitch', 'roll', 'yaw', 'knee'):
        c, a, mo = P[k_]
        out.append((f'{mo}-{k_}.{side[0].lower()}l', motor(c, a, mo)))
    tag = side[0].lower()
    span = lambda a_, b_: (min(a_, b_), max(a_, b_))
    face = lambda c_, d_: c_ if d_ > 0 else c_ - T              # plate w0 for a plate on the +/- side of a face at c_
    # -- roll mount (pelvis frame): yz plate on the roll stator back (aft)
    x_rb = rc[0] - m06['L'] / 2 - G
    rm = L.unary_union([L.circle(rc[1], rc[2], m06['back'][0] + 22), L.rounded(*span(0.0, rc[1])[:1], rc[2] - 30, span(0.0, rc[1])[1], rc[2] + 30, 8)])
    rm = rm.difference(L.holes_on(m06['back'][0], m06['back'][1], m06['hole'], phase=45, at=(rc[1], rc[2])))
    out.append((f'chassis-rollmount.{tag}l', plate(rm, 'yz', x_rb - T)))
    # -- hip R: yz plate on the roll output (fore), down + outboard to the pitch's inner plate
    x_ro = rc[0] + m06['L'] / 2 + G
    y_pf = hp[1] - s * (m06['L'] / 2 + G)                 # pitch back face (inboard)
    y_pi = face(y_pf, -s)                                  # inner hip plate w0
    y0, y1 = span(rc[1] - s * 20, y_pf - s * T)        # butts against the inner hip plate
    R1 = L.unary_union([L.circle(rc[1], rc[2], m06['out'][0] + 30), L.rounded(y0, hp[2] - 10, y1, rc[2], 8)])
    R1 = R1.difference(L.holes_on(m06['out'][0], m06['out'][1], m06['hole'], at=(rc[1], rc[2])))
    out.append((f'chassis-hipR.{tag}l', plate(R1, 'yz', x_ro)))
    # -- hip inner: xz plate on the pitch stator back, forward to hip R; stays below the roll can and above the yaw
    z_top = yc[2] + m02['L'] / 2 + G                       # yaw stator top
    z_rb = rc[2] - m06['D'] / 2 - G                        # roll can bottom
    HI = L.unary_union([L.circle(hp[0], hp[2], m06['back'][0] + 22), L.rounded(hp[0] - 30, hp[2] - 20, x_ro, z_rb, 8)])
    HI = HI.intersection(L.box(-1e3, z_top + T + 2, 1e3, z_rb))
    HI = HI.difference(L.holes_on(m06['back'][0], m06['back'][1], m06['hole'], phase=45, at=(hp[0], hp[2])))
    out.append((f'chassis-hipinner.{tag}l', plate(HI, 'xz', y_pi)))
    # -- hip A: xz plate on the pitch output (outboard), down to the yaw-top plate
    y_out = hp[1] + s * (m06['L'] / 2 + G)
    A1 = L.unary_union([L.circle(hp[0], hp[2], m06['out'][0] + 30), L.rounded(hp[0] - 40, z_top, hp[0] + 40, hp[2], 10)])
    A1 = A1.intersection(L.box(-1e3, z_top + T, 1e3, 1e3))
    A1 = A1.difference(L.holes_on(m06['out'][0], m06['out'][1], m06['hole'], at=(hp[0], hp[2])))
    out.append((f'chassis-hipA.{tag}l', plate(A1, 'xz', face(y_out, s))))
    # -- hip B2: xy plate on the yaw stator top, out to hip A
    yb0, yb1 = span(yc[1] - s * 40, y_out)
    B2 = L.unary_union([L.circle(yc[0], yc[1], m02['back'][0] + 22), L.rounded(yc[0] - 40, yb0, yc[0] + 40, yb1, 8)])
    B2 = B2.difference(L.holes_on(m02['back'][0], m02['back'][1], m02['hole'], phase=45, at=(yc[0], yc[1])))
    out.append((f'chassis-hipB2.{tag}l', plate(B2, 'xy', z_top)))
    # -- thigh: a top plate on the yaw output, an inner plate (knee stator) + outer plate down to the knee
    z_yo = yc[2] - m02['L'] / 2 - G                 # yaw output face (down)
    yk_in = kc[1] - s * (m06['L'] / 2 + G)          # knee back face (inboard)
    yk_out = kc[1] + s * (m06['L'] / 2 + G)         # knee output face (outboard)
    y_ti, y_to = (yk_in - T if s > 0 else yk_in), (yk_out + G + T + G if s > 0 else yk_out - G - T - G - T)   # outer thigh plate rides outside the shin plate
    top = L.rounded(min(y_ti, y_to) - 2, -40 + yc[0], max(y_ti, y_to) + T + 2, 40 + yc[0], 10)
    top = L.unary_union([L.rounded(yc[0] - 40, min(y_ti, y_to), yc[0] + 40, max(y_ti, y_to) + T, 10)])
    top = top.difference(L.holes_on(m02['out'][0], m02['out'][1], m02['hole'], at=(yc[0], yc[1])))
    out.append((f'chassis-thightop.{side[0].lower()}l', plate(top, 'xy', z_yo - T)))
    W = m06['D'] + 8
    side_p = L.stadium([(yc[0], z_yo - T - 40, 80), (kc[0], kc[2], W)])   # top edge meets the top plate's underside
    th_in = side_p.difference(L.holes_on(m06['back'][0], m06['back'][1], m06['hole'], phase=45, at=(kc[0], kc[2])))
    th_out = side_p.difference(L.circle(kc[0], kc[2], m06['out'][0] + 30))      # bearing seat around the shin hub
    out.append((f'thigh-inner.{side[0].lower()}l', plate(th_in, 'xz', y_ti)))
    out.append((f'thigh-outer.{side[0].lower()}l', plate(th_out, 'xz', y_to)))
    # -- shin: outer plate on the knee output (between the knee and the outer thigh plate), a shin box below
    y_so = yk_out if s > 0 else yk_out - T
    # ankle motors: two RS00 down the shin, lateral, between the outer shin plate and an inner shin plate
    y_si = (yk_out - G - m00['L'] - G - T) if s > 0 else (yk_out + G + m00['L'] + G)
    a_mot = []
    for f in (0.42, 0.66):
        c = kc + (ac - kc) * f
        cm = np.array([c[0], (y_so - G - m00['L'] / 2) if s > 0 else (y_so + T + G + m00['L'] / 2), c[2]])
        a_mot.append(cm)
        out.append((f'rs00-ankle{int(f * 100)}.{side[0].lower()}l', motor(cm, [0, s * 1.0, 0], 'rs00')))
    shin_o = L.stadium([(kc[0], kc[2], W - 4), (a_mot[0][0], a_mot[0][2], m00['D'] + 10), (a_mot[1][0], a_mot[1][2], m00['D'] + 10), (ac[0], ac[2], 40)])
    shin_o = shin_o.difference(L.holes_on(m06['out'][0], m06['out'][1], m06['hole'], at=(kc[0], kc[2])))
    out.append((f'shin-outer.{side[0].lower()}l', plate(shin_o, 'xz', y_so)))
    z_box = kc[2] - m06['D'] / 2 - 6               # the inner shin plate starts below the knee can
    shin_i = L.stadium([(a_mot[0][0], z_box - 34, m00['D'] + 22), (a_mot[1][0], a_mot[1][2], m00['D'] + 22), (ac[0], ac[2], 48)])
    out.append((f'shin-inner.{side[0].lower()}l', plate(shin_i, 'xz', y_si)))
    # bridge from the outer shin plate to the inner one under the knee can (a yz web), keeps the box closed
    web = L.rounded(min(y_si, y_so) + T, z_box - 16, max(y_si, y_so), z_box, 4)
    out.append((f'shin-web.{side[0].lower()}l', plate(web, 'yz', kc[0] - 20)))
    return out


SCALE = 100.0 / 94.0          # the K1 layout grown to a 100 cm robot (motors and plate stock don't scale)


def world():
    """the K1 joint layout (arms down), scaled to our height; link meshes scaled with it."""
    J, M = KT.world(K.arms_down())
    J = {n: dict(j, p=np.asarray(j['p']) * SCALE) for n, j in J.items()}
    M = {n: m.copy().apply_scale(SCALE) for n, m in M.items()}
    return J, M


def structure():
    J, M = world()
    out = []
    for side in ('Left', 'Right'):
        out += leg(J, side)
    return out
